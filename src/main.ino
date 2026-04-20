#include <Arduino.h>
#include <esp_dsp.h>
#include "freertos/timers.h"
#include <time.h>

#define ADC_PIN 35 // ADC1_CH7
#define SAMPLING_FREQUENCY 5000 // Sampling Rate = 1 / 200ms
#define LOOP_TIME 000 // Timer repeats every 1s, any lower rate cause overflow error due to low DRAM memory
#define SAMPLES 4096

int bufferIndex = 0, lastTime = 0;
int sampleBuffer[SAMPLES];
float window[SAMPLES];
float samples[SAMPLES * 2];
float peakFreq = 0; // To be used to calculate final sampling rate

TimerHandle_t myTimer;
struct tm timeInfo, startTime;

SemaphoreHandle_t bufferIndexMutex;
SemaphoreHandle_t peakFreqMutex;

TaskHandle_t fftTaskHandle;

void adcTask(void *pvParameters);

void setup() {
  Serial.begin(115200);
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);
  pinMode(ADC_PIN, INPUT);

  // Set up Mutex to protect from racing conditions.
  bufferIndexMutex = xSemaphoreCreateMutex();
  peakFreqMutex    = xSemaphoreCreateMutex();

  if (!bufferIndexMutex || !peakFreqMutex) {
    Serial.println("ERROR: Failed to create mutexes");
    exit (1);
  }

  Serial.println ("starting timer----------------------------------------------------------------------");
  // Setting up timer to run task handler every LOOP_TIME seconds, which computes the average over collected values.
  // In 2 seconds, at the initial sampling rate, we would have 5000 * 2 values.
  myTimer = xTimerCreate(
    "MyTimer",                   // name
    pdMS_TO_TICKS(LOOP_TIME),    // period: 2s
    pdTRUE,                      // repeating
    NULL,                        // timer ID (not needed)
    timerCallback                // callback function
  );
  xTimerStart(myTimer, 0);  // 0 = don't wait if scheduler is busy
  xTaskCreatePinnedToCore(adcTask, "ADC", 16384, NULL, 3, NULL, 1);
}

// The Sampling task - reads the values from the ADC pin and runs FFT once the buffer is full.
void adcTask(void *pvParameters) {
    // Applying the Hann window to reduce spectral leakage
    // A pure sine wave would smears the neighboring bins and distort results.
    dsps_wind_hann_f32(window, SAMPLES);
    // Allocating memory for fft
    dsps_fft2r_init_fc32(NULL, SAMPLES);
    
    Serial.println ("starting fft loop-------------------------------------------------------------");

    while (true) {
        int raw = analogRead(ADC_PIN);
        Serial.printf("reading values: %d\n", raw);

        xSemaphoreTake(bufferIndexMutex, portMAX_DELAY);
        if (bufferIndex < SAMPLES) { // bounds check — don't overrun the buffer
            sampleBuffer[bufferIndex++] = raw;
        }
        xSemaphoreGive(bufferIndexMutex);

        if (bufferIndex >= SAMPLES) {
            computeFFT (sampleBuffer);
            bufferIndex = 0;
        }

        // Read peakFreq to compute delay — guard so computeFFT can't write mid-read
        xSemaphoreTake(peakFreqMutex, portMAX_DELAY);
        float freq = peakFreq;
        xSemaphoreGive(peakFreqMutex);

        Serial.printf ("Sampling Rate: %f\n", 2 * freq);

        // If peak frequency has been calculated at least once, sample at 2 * peak frequency (Nyquist Rate).
        ets_delay_us(1000000UL / (freq == 0 ? SAMPLING_FREQUENCY : (uint32_t)(2 * freq)));
    }
}

void computeFFT(int* rawSamples) {
  for (int i = 0; i < SAMPLES; i++) {
    samples[i * 2 + 0] = rawSamples[i] * window[i];
    samples[i * 2 + 1] = 0.0f;
  }

  dsps_fft2r_fc32(samples, SAMPLES);
  dsps_bit_rev_fc32(samples, SAMPLES);

  // Find peak frequency
  float peakMagnitude = 0;
  int   peakBin = 0;
  for (int i = 1; i < SAMPLES / 2; i++) {
    float real  = samples[i * 2 + 0],
          imag  = samples[i * 2 + 1];
    float mag = real * real + imag * imag;
    if (mag > peakMagnitude) {
      peakMagnitude = mag;
      peakBin = i;
    }
  }
  xSemaphoreTake(peakFreqMutex, portMAX_DELAY);
  uint32_t sampleRate = (peakFreq == 0) ? SAMPLING_FREQUENCY : (uint32_t)(2 * peakFreq);
  peakFreq            = peakBin * ((float)sampleRate / SAMPLES);
  float result        = peakFreq;   // copy for printing after releasing lock
  xSemaphoreGive(peakFreqMutex);
  Serial.printf("Peak: %.1f Hz (magnitude %.2f)--------------------------------------------------\n", result, peakMagnitude);
}

float computeAverage(int* buf, int len) {
    float sum = 0.0f;
    for (int i = 0; i < len; i++)
        sum += buf[i];
    float avg = sum / len;
    Serial.printf("Average: %f---------------------------\n", avg);
    return avg;
}

// Compute FFT and aggregate values every 5 seconds
void timerCallback(TimerHandle_t xTimer) {
    Serial.println("5 seconds passed---------------------------------------------------------------");
    xSemaphoreTake(bufferIndexMutex, portMAX_DELAY);
    bufferIndex = 0;
    xSemaphoreGive(bufferIndexMutex);
    computeAverage (sampleBuffer, bufferIndex + 1);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}