#ifndef SIGNAL_GENERATOR_H
#define SIGNAL_GENERATOR_H

#include <Arduino.h>
#include <vector>
#include <cmath>

#define LUT_SIZE 256

struct SignalComponent {
    float frequency;
    float amplitude;
};

class SignalGenerator {
private:
    uint8_t pin;
    uint8_t lut[LUT_SIZE];
    volatile uint8_t lutIndex;
    std::vector<SignalComponent> components;

    void rebuildLUT();

public:
    SignalGenerator(uint8_t pwmPin);

    // Explicit API methods
    void addComponent(float frequency, float amplitude);
    void generateSignal();

    // NEW: direct math evaluation, bypasses LUT/PWM/ADC entirely
    float getSample(float timeInSeconds) const;
};

#endif // SIGNAL_GENERATOR_H