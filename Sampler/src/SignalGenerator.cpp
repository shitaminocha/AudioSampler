#include "SignalGenerator.h"

SignalGenerator::SignalGenerator(uint8_t pwmPin) : pin(pwmPin), lutIndex(0) {
    for (int i = 0; i < LUT_SIZE; i++) {
        lut[i] = 128;
    }
}

void SignalGenerator::addComponent(float frequency, float amplitude) {
    components.push_back({frequency, amplitude});
    rebuildLUT();
}

void SignalGenerator::rebuildLUT() {
    for (int i = 0; i < LUT_SIZE; i++) {
        float sample = 0.0f;
        float totalAmp = 0.0f;

        for (const auto& comp : components) {
            // FIX: Cast i to float (i / (float)LUT_SIZE) to prevent integer truncation to 0
            sample += comp.amplitude * sinf(2.0f * M_PI * comp.frequency * ((float)i / (float)LUT_SIZE));
            totalAmp += comp.amplitude;
        }

        if (totalAmp > 0.0f) {
            sample /= totalAmp;
        }

        // Offsets 0.0-1.0 float to 0-255 PWM duty cycle
        lut[i] = (uint8_t)(127.5f * (1.0f + sample));
    }
}

void SignalGenerator::generateSignal() {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(pin, lut[lutIndex]);
#else
    ledcWrite(0, lut[lutIndex]);
#endif

    lutIndex = (lutIndex + 1) % LUT_SIZE;
}

float SignalGenerator::getSample(float timeInSeconds) const {
    float sample = 0.0f;
    for (const auto& comp : components) {
        sample += comp.amplitude * sinf(2.0f * M_PI * comp.frequency * timeInSeconds);
    }
    return sample;
}