#include <Arduino.h>
#include <esp_dsp.h>
#include <time.h>
#include "freertos/timers.h"
#include "Communication/CommunicationMQTT.h"
#include "Communication/CommunicationLoRa.h"


// -----------------------------------------------------------------------------
// Initialising Data
// -----------------------------------------------------------------------------

// For FFT Analysis and Average Computation
#define ADC_PIN            35
#define SAMPLING_FREQUENCY 5000
#define LOOP_TIME          500
#define SAMPLES            4096

int   bufferIndex = 0, lastTime = 0;
float peakFreq    = 0.0f;

int*   sampleBuffer = nullptr;
float* window       = nullptr;
float* fftBuffer    = nullptr;
float  lastAvg      = 0.0f;

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
static uint32_t bytesInitial       = 0;   // bytes published at initial rate
static uint32_t bytesAdaptive      = 0;   // bytes published at adaptive rate
static uint32_t packetsInitial     = 0;
static uint32_t packetsAdaptive    = 0;
static bool     adaptiveActive     = false;  // set true once FFT has run once
static uint64_t sessionStartUs     = 0; // time

// Forward Declarations
void adcTask(void *pvParameters);
void mqttTask(void *pvParameters);
void loraTask(void *pvParameters);
void computeFFT(int* rawSamples);
float computeAverage(int* buf, int len);
void timerCallback(TimerHandle_t xTimer);
void trackTransmission(uint32_t payloadBytes);

void setup() {
  uint64_t windowStart = esp_timer_get_time();
  Serial.begin(115200);
  delay(500);

  sampleBuffer = (int*)   heap_caps_malloc(SAMPLES * sizeof(int),       MALLOC_CAP_8BIT);
  window       = (float*) heap_caps_malloc(SAMPLES * sizeof(float),     MALLOC_CAP_8BIT);
  fftBuffer    = (float*) heap_caps_malloc(SAMPLES * 2 * sizeof(float), MALLOC_CAP_8BIT);
  if (!sampleBuffer || !window || !fftBuffer) return;

  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);
  pinMode(ADC_PIN, INPUT);

  bufferIndexMutex = xSemaphoreCreateMutex();
  peakFreqMutex    = xSemaphoreCreateMutex();
  avgMutex         = xSemaphoreCreateMutex();
  if (!bufferIndexMutex || !peakFreqMutex || !avgMutex) exit(1);

  MQTTSetup();
  topicSend = MQTTGetSendTopic();
  topicEcho = MQTTGetEchoTopic();

  sessionStartUs = esp_timer_get_time();   // start tracking from here

  myTimer = xTimerCreate("MyTimer", pdMS_TO_TICKS(LOOP_TIME), pdTRUE, NULL, timerCallback);
  xTimerStart(myTimer, 0);


  // xTaskCreatePinnedToCore(loraTask, "LoRa", 8192,  NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(mqttTask, "MQTT", 4096,  NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(adcTask,  "ADC",  16384, NULL, 3, NULL, 0);

  uint64_t elapsed = esp_timer_get_time() - windowStart;
  Serial.printf(">setup_window_time_ms:%.2f\n", elapsed / 1000.0f);
}

void adcTask(void *pvParameters) {
  Serial.println("Starting ADC Task");
  dsps_wind_hann_f32(window, SAMPLES);
  dsps_fft2r_init_fc32(NULL, SAMPLES);

  while (true) {
    uint64_t t0 = esp_timer_get_time();
    int raw = analogRead(ADC_PIN);
    Serial.printf(">analog_read_time_ms:%.2f\n", (esp_timer_get_time() - t0) / 1000.0f);
    Serial.printf(">raw:%d\n", raw);

    xSemaphoreTake(bufferIndexMutex, portMAX_DELAY);
    if (bufferIndex < SAMPLES) {
      sampleBuffer[bufferIndex++] = raw;
    } else {
      computeFFT(sampleBuffer);
      bufferIndex = 0;
    }
    xSemaphoreGive(bufferIndexMutex);

    xSemaphoreTake(peakFreqMutex, portMAX_DELAY);
    float freq = peakFreq;
    xSemaphoreGive(peakFreqMutex);

    ets_delay_us(1000000UL / (freq == 0 ? SAMPLING_FREQUENCY : (uint32_t)(2 * freq)));
  }
}

void mqttTask(void *pvParameters) {
  MQTTloop();

  float avg = 0;
  xSemaphoreTake(avgMutex, portMAX_DELAY);
  if (avgReady) {
    avg      = lastAvg;
    avgReady = false;
  }
  xSemaphoreGive(avgMutex);

  char msg[32];
  snprintf(msg, sizeof(msg), "%.2f", avg);
  MQTTPublish(topicSend, msg);
  trackTransmission(strlen(msg));   // track bytes published

  vTaskDelay(pdMS_TO_TICKS(10000));
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

  // ABP credentials — from TTN console
  uint32_t devAddr  = 0x260B07DD;
  // uint8_t  nwkSKey[] = { 0xXX, 0xXX, ... };  // your NwkSEncKey from TTN
  uint8_t  appSKey[] = { 0xB3, 0x49, 0x0B, 0x9D, 0x2B, 0x5D, 0x96, 0x03,
                         0x8F, 0xD6, 0xC8, 0x27, 0xA1, 0x89, 0x3F, 0x96 };

  node.beginABP(devAddr, fNwkSIntKey, sNwkSIntKey, nwkSEncKey, appSKey);
  state = node.activateABP();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("ABP failed: %d\n", state);
    vTaskDelete(NULL);
  }
  Serial.println("ABP session started");

  for (;;) {
    float valueToSend = lastAvg;
    loRaSend(valueToSend);
    trackTransmission(sizeof(float));
    vTaskDelay(pdMS_TO_TICKS(15000));
  }
}

void computeFFT(int* rawSamples) {
  uint64_t t0 = esp_timer_get_time();
  for (int i = 0; i < SAMPLES; i++) {
    fftBuffer[i * 2 + 0] = rawSamples[i] * window[i];
    fftBuffer[i * 2 + 1] = 0.0f;
  }

  dsps_fft2r_fc32(fftBuffer, SAMPLES);
  dsps_bit_rev_fc32(fftBuffer, SAMPLES);

  float peakMagnitude = 0;
  int   peakBin       = 0;
  for (int i = 1; i < SAMPLES / 2; i++) {
    float re  = fftBuffer[i * 2 + 0];
    float im  = fftBuffer[i * 2 + 1];
    float mag = re * re + im * im;
    if (mag > peakMagnitude) { peakMagnitude = mag; peakBin = i; }
  }

  xSemaphoreTake(peakFreqMutex, portMAX_DELAY);
  float binResolution = (float)SAMPLING_FREQUENCY / SAMPLES;
  peakFreq            = peakBin * binResolution * 2.0f;
  float result        = peakFreq;
  xSemaphoreGive(peakFreqMutex);

  // Switch tracking to adaptive phase the First time FFT completes
  if (!adaptiveActive && peakFreq > 0) {
    adaptiveActive = true;
    Serial.printf("Adaptive rate active: %.1f Hz\n", peakFreq);
  }

  Serial.printf(">fft_computation_time_ms:%.2f\n", (esp_timer_get_time() - t0) / 1000.0f);
  Serial.printf(">peak_freq:%.2f\n", result);
}

float computeAverage(int* buf, int len) {
  if (len <= 0) return 0.0f;
  float sum = 0.0f;
  for (int i = 0; i < len; i++) sum += buf[i];
  float avg = sum / len;
  Serial.printf(">average:%.4f\n", avg);
  return avg;
}

void timerCallback(TimerHandle_t xTimer) {
  xSemaphoreTake(bufferIndexMutex, portMAX_DELAY);
  int count = bufferIndex;        // snapshot before reset
  bufferIndex = 0;
  xSemaphoreGive(bufferIndexMutex);

  float avg = computeAverage(sampleBuffer, count);

  xSemaphoreTake(avgMutex, portMAX_DELAY);
  lastAvg  = avg;
  avgReady = true;
  xSemaphoreGive(avgMutex);

  // Track every timer-triggered publish as one transmission event
  char msg[32];
  snprintf(msg, sizeof(msg), "%.2f", avg);
  trackTransmission(strlen(msg));
}

// Called every time a payload is published (MQTT or LoRa)
// Before FFT completes peakFreq == 0 so we're at the initial rate.
// After FFT sets peakFreq we're at the adaptive rate.
void trackTransmission(uint32_t payloadBytes) {
  uint32_t totalBytes = payloadBytes + 13; // 13 bytes LoRaWAN MAC overhead
  if (!adaptiveActive) {
    bytesInitial   += totalBytes;
    packetsInitial += 1;
  } else {
    bytesAdaptive   += totalBytes;
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