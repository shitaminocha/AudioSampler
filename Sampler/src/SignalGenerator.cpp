#include "SignalGenerator.h"
#include <math.h>

int     signalDacPin        = 25;       // DAC pin (25 or 26)
float   signalSampleRate    = 10000.0f; // Sample rate in Hz
float   signalFrequency     = 1000.0f;  // Current frequency in Hz
float   signalPhase         = 0.0f;     // Current phase in radians
float   signalPhaseIncrement;           // Phase increment per sample
bool    signalRunning       = false;    // Flag to control the generation task
TaskHandle_t signalTaskHandle;          // Handle for the generation task

void signalGeneratorTask(void *pvParameters);

void signalGeneratorSetup(int dacPin, float sampleRate) {
    signalDacPin = dacPin;
    signalSampleRate = sampleRate;
    signalPhaseIncrement = 2.0f * PI * signalFrequency / signalSampleRate;
    Serial.println("Signal Generator Initialized");
}

void signalGeneratorSetFrequency(float freq) {
    signalFrequency = freq;
    signalPhaseIncrement = 2.0f * PI * signalFrequency / signalSampleRate;
    Serial.printf("Signal Frequency Set to: %.2f Hz\n", signalFrequency);
}

void signalGeneratorStart() {
    if (!signalRunning) {
        signalRunning = true;
        xTaskCreatePinnedToCore(signalGeneratorTask, "SignalGen", 2048, NULL, 1, &signalTaskHandle, 1);
        Serial.println("Signal Generator Started");
    }
}

void signalGeneratorStop() {
    if (signalRunning) {
        signalRunning = false;
        vTaskDelete(signalTaskHandle);
        Serial.println("Signal Generator Stopped");
    }
}

void signalGeneratorTask(void *pvParameters) {
    Serial.println("Starting Signal Generator Task");

    while (signalRunning) {
        // Calculate the sine sample: sin(phase) scaled to 0-255 for DAC
        float sample = sin(signalPhase) * 50.0f + 127.5f;
        // Output to DAC
        dacWrite(signalDacPin, (uint8_t)sample);
        Serial.printf(">signal:%d\n", (uint8_t)sample);

        // Update phase for next sample
        signalPhase += signalPhaseIncrement;
        // Wrap phase to keep it between 0 and 2*PI
        if (signalPhase >= 2.0f * PI) {
            signalPhase -= 2.0f * PI;
        }

        // Delay to maintain the sample rate (in microseconds)
        ets_delay_us((uint32_t)(1000000.0f / signalSampleRate));
    }

    vTaskDelete(NULL);
}