#include "SignalAnalyzer.h"

SignalAnalyzer::SignalAnalyzer() : FFT(vReal, vImag, FFT_SAMPLES, SAMPLING_FREQ) {
    FFT = ArduinoFFT<float>(vReal, vImag, FFT_SAMPLES, SAMPLING_FREQ);
}

void SignalAnalyzer::processSignal(const uint16_t* adcData, float& peakFreq, float& peakValue) {
    for (int i = 0; i < FFT_SAMPLES; i++) {
        vReal[i] = (float)adcData[i];
        vImag[i] = 0.0f;
    }

    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    peakFreq = FFT.majorPeak();
    peakValue = 0.0f;
    for (int i = 1; i < (FFT_SAMPLES / 2); i++) {
        if (vReal[i] > peakValue) {
            peakValue = vReal[i];
        }
    }
}