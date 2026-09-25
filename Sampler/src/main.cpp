#include <Arduino.h>
#include <heltec_unofficial.h>
#include "SignalGenerator.h"
#include "SignalAnalyzer.h"

#define GEN_PIN         6     // PWM output pin
#define SAMPLE_PIN      7     // ADC1 input pin

typedef struct {
    uint16_t buffer[FFT_SAMPLES];
} AudioFrame;

QueueHandle_t audioQueue;
SignalAnalyzer analyzer;
SignalGenerator sigGen(GEN_PIN);

// ====================================================================
// CORE 1: SIGNAL GENERATION TASK
// ====================================================================
void TaskSignalGenerator(void* pvParameters) {
    // Step interval for 5 Hz wave through 10uF RC filter: 1,000,000 / (5 * 32) = 6250 us
    const uint32_t stepIntervalUs = 1000000UL / LUT_SIZE;
    uint32_t nextStep = micros();

    for (;;) {
        while ((int32_t)(micros() - nextStep) < 0) {
            // Microsecond-accurate pacing
        }
        nextStep += stepIntervalUs;

        // <--- GENERATE SIGNAL CALLED HERE
        sigGen.generateSignal();

        // Feed watchdog to prevent task starvation lockup
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

// ====================================================================
// CORE 1: SAMPLER TASK
// ====================================================================
#define USE_SYNTHETIC_SAMPLES 1   // 1 = bypass ADC, feed sigGen math directly; 0 = real analogRead
void TaskSampler(void* pvParameters) {
    AudioFrame currentFrame;
    const uint32_t sampleIntervalUs = 1000000 / SAMPLING_FREQ;

#if USE_SYNTHETIC_SAMPLES
    float t = 0.0f;
    const float dt = 1.0f / (float)SAMPLING_FREQ;
#endif

    for (;;) {
        uint32_t nextTick = micros();

        for (int i = 0; i < FFT_SAMPLES; i++) {
            while ((int32_t)(micros() - nextTick) < 0) {
                taskYIELD();   // watchdog fix — needed in both modes
            }

#if USE_SYNTHETIC_SAMPLES
            float raw = sigGen.getSample(t);          // e.g. range ~[-6, +6]
            currentFrame.buffer[i] = (uint16_t)((raw + 10.0f) * 100.0f);  // shift+scale into a plausible ADC-like range
            t += dt;
#else
            currentFrame.buffer[i] = analogRead(SAMPLE_PIN);
#endif
            nextTick += sampleIntervalUs;
        }

        xQueueSend(audioQueue, &currentFrame, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ====================================================================
// CORE 0: ANALYTICS & TELEMETRY TASK
// ====================================================================
void TaskAnalyzer(void* pvParameters) {
    static AudioFrame frameToProcess;
    float dominantFreq = 0.0f;
    float peakMag = 0.0f;

    for (;;) {
        if (xQueueReceive(audioQueue, &frameToProcess, portMAX_DELAY) == pdTRUE) {
            analyzer.processSignal(frameToProcess.buffer, dominantFreq, peakMag);

            // Print continuous wave for Teleplot
            for (int i = 0; i < 64; i++) {
                Serial.printf(">raw_wave:%d\n", frameToProcess.buffer[i]);
            }
            Serial.printf(">peak_frequency:%.2f\n", dominantFreq);
            
            // Force USB CDC serial buffer flush
            Serial.flush();
        }
    }
}

void setup() {
    heltec_setup();

    // 1. Configure Hardware Pin Modes
    pinMode(GEN_PIN, OUTPUT);
    pinMode(SAMPLE_PIN, INPUT);

    // 2. Configure ADC1 Settings
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    const uint32_t pmwFreq = 20000;

    // 3. Setup Hardware PWM Carrier
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(GEN_PIN, pmwFreq, 8);
#else
    ledcSetup(0, pmwFreq, 8);
    ledcAttachPin(GEN_PIN, 0);
#endif

    // 4. <--- ADD COMPONENT CALLED HERE
    sigGen.addComponent(5.0f, 1.0f); // 5 Hz fundamental tone for 10uF RC filter

    // 5. Create Queue & FreeRTOS Tasks
    audioQueue = xQueueCreate(2, sizeof(AudioFrame));
    if (audioQueue == NULL) {
        while (1);
    }

    xTaskCreatePinnedToCore(TaskSignalGenerator, "SigGen",   2048,  NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(TaskSampler,         "Sampler",  4096,  NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(TaskAnalyzer,        "FFT_Proc", 16384, NULL, 2, NULL, 0);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}