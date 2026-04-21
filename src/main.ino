#include <Arduino.h>
#include <esp_dsp.h>
#include <time.h>
#include "freertos/timers.h"
#include "Communication/CommunicationMQTT.h"
#include "Communication/CommunicationLoRa.h"

#define ADC_PIN 35 // ADC1_CH7
#define SAMPLING_FREQUENCY 5000 // Sampling Rate = 1 / 200ms
#define LOOP_TIME 500 // Timer repeats every 1s, any lower rate cause overflow error due to low DRAM memory
#define SAMPLES 4096

int bufferIndex = 0, lastTime = 0;
float peakFreq = 0; // To be used to calculate final sampling rate

int* sampleBuffer = nullptr;
float* window = nullptr;
float* fftBuffer = nullptr;
float lastAvg = 0.0f;

volatile bool avgReady = false;

const char* topicSend = "dakshita/IoT/send";
const char* topicEcho = "dakshita/IoT/echo";

TimerHandle_t myTimer;

SemaphoreHandle_t bufferIndexMutex;
SemaphoreHandle_t peakFreqMutex;
SemaphoreHandle_t avgMutex;

TaskHandle_t fftTaskHandle;

void adcTask(void *pvParameters);
void mqttTask(void *pvParameters);

void setup() {
  Serial.begin(115200);
  delay (500);

  sampleBuffer = (int*) heap_caps_malloc(SAMPLES * sizeof(int),   MALLOC_CAP_8BIT);
  window    = (float*) heap_caps_malloc(SAMPLES * sizeof(float), MALLOC_CAP_8BIT);
  fftBuffer    = (float*) heap_caps_malloc(SAMPLES * 2 * sizeof(float), MALLOC_CAP_8BIT);
  if (!sampleBuffer || !window || !fftBuffer) return;

  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);
  pinMode(ADC_PIN, INPUT);

  // Set up Mutex to protect from racing conditions.
  bufferIndexMutex = xSemaphoreCreateMutex();
  peakFreqMutex    = xSemaphoreCreateMutex();
  avgMutex    = xSemaphoreCreateMutex();

  if (!bufferIndexMutex || !peakFreqMutex || !avgMutex) exit (1);

  // MQTTSetup ();
  topicSend = MQTTGetSendTopic ();
  topicEcho = MQTTGetEchoTopic ();
  // Setting up timer to run task handler every LOOP_TIME seconds, which computes the average over collected values.
  // In 1 seconds, at the initial sampling rate, we would have 5000 * 2 values.
  myTimer = xTimerCreate(
    "MyTimer",                   // name
    pdMS_TO_TICKS(LOOP_TIME),    // period: 1s
    pdTRUE,                      // repeating
    NULL,                        // timer ID (not needed)
    timerCallback                // callback function
  );
  xTimerStart(myTimer, 0);  // 0 = don't wait if scheduler is busy

  xTaskCreatePinnedToCore(adcTask, "ADC", 16384, NULL, 3, NULL, 1);
  // xTaskCreatePinnedToCore(mqttTask, "MQTT", 4096,  NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(loraTask, "LoRa", 8192, NULL, 1, NULL, 1);
}

// The Sampling task - reads the values from the ADC pin and runs FFT once the buffer is full.
void adcTask(void *pvParameters) {
    // Applying the Hann window to reduce spectral leakage
    // A pure sine wave would smears the neighboring bins and distort results.
    dsps_wind_hann_f32(window, SAMPLES);
    // Allocating memory for fft
    dsps_fft2r_init_fc32(NULL, SAMPLES);
    
    // DEBUG:
    // Serial.println ("starting fft loop-------------------------------------------------------------");
    while (true) {
        int raw = analogRead(ADC_PIN);
        // Outputting Telemetry Data for TelePlot
        Serial.printf("raw:%d\n", raw);


        xSemaphoreTake(bufferIndexMutex, portMAX_DELAY);
        // bounds check to not overrun the buffer
        if (bufferIndex < SAMPLES) sampleBuffer[bufferIndex++] = raw;
        else {
          computeFFT (sampleBuffer);
          bufferIndex = 0;
        }
        xSemaphoreGive(bufferIndexMutex);

        // Read peakFreq to compute delay — guard so computeFFT can't write mid-read
        xSemaphoreTake(peakFreqMutex, portMAX_DELAY);
        float freq = peakFreq;
        xSemaphoreGive(peakFreqMutex);

        // DEBUG:
        // Serial.printf ("%d. Sampling Rate: %f\n", bufferIndex + 1, 2 * freq);

        // If peak frequency has been calculated at least once, sample at 2 * peak frequency (Nyquist Rate).
        ets_delay_us(1000000UL / (freq == 0 ? SAMPLING_FREQUENCY : (uint32_t)(2 * freq)));
    }
}

void mqttTask(void *pvParameters) {
  while (true) {
    MQTTloop();
    float avg = 0;
    xSemaphoreTake(avgMutex, portMAX_DELAY);
    if (avgReady) {
      avg = lastAvg;
      avgReady = false;
    }
    xSemaphoreGive(avgMutex);

    char msg[32];
    snprintf(msg, sizeof(msg), "%.2f", avg);
    // DEBUG:
    // Serial.printf("Publishing avg over MQTT: %s\n", msg);
    MQTTPublish(topicSend, msg);

    vTaskDelay(pdMS_TO_TICKS(10));  // yield every 10ms — keeps connection alive without busy-waiting
  }
}


void loraTask(void *pvParameters) {
  Serial.println("[LoRa] Initialising radio...");

  int state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("[LoRa] Radio init failed: ");
    Serial.println(state);
    vTaskDelete(NULL); 
  }
  Serial.println("[LoRa] Radio OK");

  // OTAA join
  Serial.println("[LoRa] Joining TTN via OTAA...");
  node.beginOTAA(joinEUI, devEUI, keyNWKS, appKey);
  Serial.println("[LoRa] Joined TTN successfully!");

  for (;;) {
    float valueToSend = lastAvg;
    loRaSend(valueToSend);
    vTaskDelay(pdMS_TO_TICKS(15000));   // sending every 15 seconds
  }
}

void computeFFT(int* rawSamples) {
  for (int i = 0; i < SAMPLES; i++) {
    fftBuffer[i * 2 + 0] = rawSamples[i] * window[i];
    fftBuffer[i * 2 + 1] = 0.0f;
  }

  dsps_fft2r_fc32(fftBuffer, SAMPLES);
  dsps_bit_rev_fc32(fftBuffer, SAMPLES);

  // Find peak frequency
  float peakMagnitude = 0;
  int peakBin = 0;
  for (int i = 0; i < SAMPLES / 2; i++) {
    float real  = fftBuffer[i * 2 + 0], imag  = fftBuffer[i * 2 + 1];
    float mag = real * real + imag * imag;
    if (mag > peakMagnitude) {
      peakMagnitude = mag;
      peakBin = i;
    }
  }

  xSemaphoreTake(peakFreqMutex, portMAX_DELAY);
    float binResolution = SAMPLING_FREQUENCY / SAMPLES, newPeakFreq = peakBin * binResolution;
    peakFreq = 2 * newPeakFreq;
    float result = peakFreq;   // copy for printing after releasing lock
  xSemaphoreGive(peakFreqMutex);
  Serial.printf(">peak_freq:%.2f\n", peakFreq);
}

float computeAverage(int* buf, int len) {
    if (len <= 0) return 0.0f;
    float sum = 0.0f;
    for (int i = 0; i < len; i++)
        sum += buf[i];
    float avg = sum / len;
    Serial.printf(">average:%.4f\n", avg);
    return avg;
}

void timerCallback(TimerHandle_t xTimer) {
    xSemaphoreTake(bufferIndexMutex, portMAX_DELAY);
    float avg = computeAverage (sampleBuffer, bufferIndex + 1);
    xSemaphoreGive(bufferIndexMutex);

    xSemaphoreTake(avgMutex, portMAX_DELAY);
    lastAvg = avg;
    avgReady = true;
    xSemaphoreGive(avgMutex);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}