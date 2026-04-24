#include <Arduino.h>
#include <esp_dsp.h>
#include <time.h>
#include "freertos/timers.h"
#include "CommunicationMQTT.h"
#include "CommunicationLoRa.h"
#include "SignalGenerator.h"

// -----------------------------------------------------------------------------
// Initialising Data
// -----------------------------------------------------------------------------

// For FFT Analysis and Average Computation
#define ADC_PIN            35
#define SAMPLING_FREQUENCY 5000
#define LOOP_TIME          500
#define SAMPLES            4096
#define FFT_MAG_THRESHOLD_RATIO  0.05 // Fraction of the peak FFT magnitude used as threshold

int bufferIndex = 0, lastTime = 0;
float peakFreq    = 0.0f;

int* sampleBuffer = nullptr;
float* window = nullptr;
float* fftBuffer = nullptr;
float lastAvg = 0.0f;

volatile bool avgReady = false;

const char* topicSend;
const char* topicEcho;

TimerHandle_t myTimer;

// To prevent Racing Conditions
SemaphoreHandle_t bufferIndexMutex;
SemaphoreHandle_t peakFreqMutex;
SemaphoreHandle_t avgMutex;

// Task Handler
TaskHandle_t fftTaskHandle;

// For Transmission Tracking
static uint32_t bytesInitial = 0;        // bytes published at initial rate
static uint32_t bytesAdaptive = 0;       // bytes published at adaptive rate
static uint32_t packetsInitial = 0;
static uint32_t packetsAdaptive = 0;
static bool adaptiveActive = false;      // set true once FFT has run once
static uint64_t sessionStartUs = 0;      // time

// Forward Declarations
void adcTask(void *pvParameters);
void mqttTask(void *pvParameters);
void loraTask(void *pvParameters);
void computeFFT();
float computeAverage(int* buf, int len);
void trackTransmission(uint32_t payloadBytes);

void setup() {
  Serial.begin(115200);
  // Initialize signal generator: DAC pin 25, 500Hz sample rate
  signalGeneratorSetup(25, 500);
  // Set initial frequency (e.g., 2Hz)
  signalGeneratorSetFrequency(2.0f);
  // Start generating the signal
  signalGeneratorStart();

  sampleBuffer = (int*) heap_caps_malloc(SAMPLES * sizeof(int), MALLOC_CAP_SPIRAM);
  window = (float*) heap_caps_malloc(SAMPLES * sizeof(float), MALLOC_CAP_SPIRAM);
  fftBuffer = (float*) heap_caps_malloc(SAMPLES * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
  if (!sampleBuffer || !window || !fftBuffer) {
    Serial.println("FATAL: Memory allocation failed!");
    while(1) vTaskDelay(1);
  }

  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);
  pinMode(ADC_PIN, INPUT);

  bufferIndexMutex = xSemaphoreCreateMutex();
  peakFreqMutex = xSemaphoreCreateMutex();
  avgMutex = xSemaphoreCreateMutex();
  if (!bufferIndexMutex || !peakFreqMutex || !avgMutex) exit(1);

  MQTTSetup();
  topicSend = MQTTGetSendTopic();
  topicEcho = MQTTGetEchoTopic();

  sessionStartUs = esp_timer_get_time();   // start tracking from here

  // xTaskCreatePinnedToCore(loraTask, "LoRa", 8192,  NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(mqttTask, "MQTT", 4096,  NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(adcTask,  "ADC",  16384, NULL, 3, NULL, 1);
}

void adcTask(void *pvParameters) {
  Serial.println("Starting ADC Task");
  dsps_wind_hann_f32(window, SAMPLES); // Hanning window for spectral leakage reduction. Precompute once in setup, then reuse in every FFT.
  dsps_fft2r_init_fc32(NULL, SAMPLES); // Initialize FFT tables.
  
  uint64_t t0 = esp_timer_get_time();

  while (true) {
    int raw = analogRead(ADC_PIN);

    Serial.printf(">raw:%d\n", raw);
    Serial.printf(">volts:%.2f\n", raw * 3.3f / (SAMPLES - 1));

    xSemaphoreTake(bufferIndexMutex, portMAX_DELAY);
    bool bufferFull = (bufferIndex >= SAMPLES);
    if (!bufferFull && bufferIndex < SAMPLES) {
      sampleBuffer[bufferIndex++] = raw;
    }
    xSemaphoreGive(bufferIndexMutex);

    if (bufferFull) {
      float avg = computeAverage(sampleBuffer, SAMPLES);
      switchBuffer();
      computeFFT();
      
      uint64_t t1 = esp_timer_get_time();
      Serial.printf(">per_window:%llu\n", t1 - t0);
      t0 = t1;
      
      xSemaphoreTake(bufferIndexMutex, portMAX_DELAY);
      bufferIndex = 0;
      xSemaphoreGive(bufferIndexMutex);
    }

    xSemaphoreTake(peakFreqMutex, portMAX_DELAY);
    float freq = peakFreq;
    xSemaphoreGive(peakFreqMutex);

    Serial.printf(">peak_freq_adaptive:%.2f\n", freq);
    
    // Calculate delay but cap it to prevent watchdog timeout (max 100ms)
    uint32_t delayUs = 1000000UL / (freq == 0 ? SAMPLING_FREQUENCY : (uint32_t)(2 * freq));
    if (delayUs > 100000) delayUs = 100000;  // Cap at 100ms
    ets_delay_us(delayUs);
    vTaskDelay(1); // Yield to other tasks.
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

    if (topicSend) {
      char msg[32];
      snprintf(msg, sizeof(msg), "%.2f", avg);
      MQTTPublish(topicSend, msg);
      trackTransmission(strlen(msg));
    }

    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

void loraTask(void *pvParameters) {
  Serial.println("LoRa Initialising");

  SPI.begin(5, 19, 27, 18);   // SCK, MISO, MOSI, CS — required for TTGO
  int state = radio.begin(868.0);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("LoRa Radio init failed: %d\n", state);
    vTaskDelete(NULL);
  }
  Serial.println("LoRa Radio OK");
  radio.setSyncWord(0x34);

  node.beginABP(devAddr, fNwkSIntKey, sNwkSIntKey, nwkSEncKey, appSKey);
  state = node.activateABP();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("ABP failed: %d\n", state);
    vTaskDelete(NULL);
  }
  Serial.println("ABP session started");

  for (;;) {
    xSemaphoreTake(avgMutex, portMAX_DELAY);
    float valueToSend = lastAvg;
    xSemaphoreGive(avgMutex);
    loRaSend(valueToSend);
    trackTransmission(sizeof(float));
    vTaskDelay(pdMS_TO_TICKS(15000));
  }
}

void computeFFT() {
  if (adaptiveActive) return;
  
  // Apply window (alternate elements are 0 since we're using
  // a complex FFT buffer)
  for (int i = 0; i < SAMPLES; i++)
    fftBuffer[i * 2] = fftBuffer[i * 2] * window[i];

  dsps_fft2r_fc32(fftBuffer, SAMPLES);
  dsps_bit_rev_fc32(fftBuffer, SAMPLES);

  // Find peak magnitude across all bins
  double peakMag = 0.0;
  for (int i = 1; i < SAMPLES / 2; i++) {
    float real = fftBuffer[i * 2];
    float imag = fftBuffer[i * 2 + 1];
    float mag = sqrt(real * real + imag * imag);
    if (mag > peakMag) peakMag = mag;
  }

  double threshold = peakMag * FFT_MAG_THRESHOLD_RATIO;
  float freqRes = SAMPLING_FREQUENCY / (float)SAMPLES;
  
  xSemaphoreTake(peakFreqMutex, portMAX_DELAY);
  // Highest-frequency LOCAL PEAK above threshold
  for (int i = 2; i < SAMPLES / 2 - 1; i++) {
    float real = fftBuffer[i * 2];
    float imag = fftBuffer[i * 2 + 1];
    float mag = sqrt(real * real + imag * imag);
    
    float realPrev = fftBuffer[(i - 1) * 2];
    float imagPrev = fftBuffer[(i - 1) * 2 + 1];
    float magPrev = sqrt(realPrev * realPrev + imagPrev * imagPrev);
    
    float realNext = fftBuffer[(i + 1) * 2];
    float imagNext = fftBuffer[(i + 1) * 2 + 1];
    float magNext = sqrt(realNext * realNext + imagNext * imagNext);
    
    if (mag > threshold && mag > magPrev && mag > magNext) {
      float freq = i * freqRes;
      if (freq > peakFreq) peakFreq = freq;
    }
  }

  float result = peakFreq;
  xSemaphoreGive(peakFreqMutex);

  // Switch tracking to adaptive phase the First time FFT completes
  if (!adaptiveActive && peakFreq > 0) {
    adaptiveActive = true;
    Serial.printf("Adaptive rate active: %.1f Hz\n", peakFreq);
  }

  Serial.printf(">peak_freq:%.2f\n", result);
}

// Moves raw values from samplBuffer to fftBuffer.
void switchBuffer () {
  for (int i = 0; i < SAMPLES; i++) {
    fftBuffer[i * 2 + 0] = sampleBuffer[i];
    fftBuffer[i * 2 + 1] = 0.0f;
  }
  // bufferIndex reset by caller after releasing mutex
}

float computeAverage(int* buf, int len) {
  if (len <= 0) return 0.0f;
  float sum = 0.0f;
  for (int i = 0; i < len; i++) sum += buf[i];
  float avg = sum / len;
  Serial.printf(">average:%.4f\n", avg);
  
  // Update global state under mutex
  xSemaphoreTake(avgMutex, portMAX_DELAY);
      lastAvg = avg;
      avgReady = true;
  xSemaphoreGive(avgMutex);
  return avg;
}

// Called every time a payload is published (MQTT or LoRa)
// Before FFT completes peakFreq == 0 so we're at the initial rate.
// After FFT sets peakFreq we're at the adaptive rate.
void trackTransmission(uint32_t payloadBytes) {
  uint32_t totalBytes = payloadBytes + 13; // 13 bytes LoRaWAN MAC overhead
  if (!adaptiveActive) {
    bytesInitial += totalBytes;
    packetsInitial += 1;
  } else {
    bytesAdaptive += totalBytes;
    packetsAdaptive += 1;
  }
  float elapsedSec = (esp_timer_get_time() - sessionStartUs) / 1000000.0f;

  // Throughput for each phase
  float bpsInitial = elapsedSec > 0 && packetsInitial > 0 ? bytesInitial  / elapsedSec : 0.0f;
  float bpsAdaptive = elapsedSec > 0 && packetsAdaptive > 0 ? bytesAdaptive / elapsedSec : 0.0f;

  Serial.printf(">bytes_initial:%u\n",    bytesInitial);
  Serial.printf(">bytes_adaptive:%u\n",   bytesAdaptive);
  Serial.printf(">throughput_initial:%.3f\n",  bpsInitial);
  Serial.printf(">throughput_adaptive:%.3f\n", bpsAdaptive);

  if (adaptiveActive && packetsAdaptive > 0 && packetsAdaptive % 10 == 0) {
    float saving = bpsInitial > 0 ? 100.0f * (1.0f - bpsAdaptive / bpsInitial) : 0.0f;

    Serial.println("========== DATA VOLUME COMPARISON ==========");
    Serial.printf("Initial rate:   %u Hz  | %u packets | %u bytes | %.3f B/s\n",
                  SAMPLING_FREQUENCY, packetsInitial, bytesInitial, bpsInitial);
    Serial.printf("Adaptive rate:  %.1f Hz | %u packets | %u bytes | %.3f B/s\n",
                  peakFreq, packetsAdaptive, bytesAdaptive, bpsAdaptive);
    Serial.printf("Data reduction: %.1f%%\n", saving);
    Serial.println("============================================");
    Serial.printf(">data_saving_pct:%.1f\n", saving);
  }
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}