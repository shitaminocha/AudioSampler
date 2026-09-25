#pragma once

#include <Arduino.h>
#include <arduinoFFT.h>

#define FFT_SAMPLES 512       // Must be power of 2
#define SAMPLING_FREQ 100   // 10 kHz sampling frequency

class SignalAnalyzer {
private:
    float vReal[FFT_SAMPLES];
    float vImag[FFT_SAMPLES];
    ArduinoFFT<float> FFT;

public:
    SignalAnalyzer();
    void processSignal(const uint16_t* adcData, float& peakFreq, float& peakValue);
    const float* getMagnitudes() const { return vReal; }
};