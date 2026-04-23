#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#ifdef BONUS
    // n(t) ~ N(0, sigma). 0.2 V → 248 LSB (0.2 / 3.3 * 4095)
    #define NOISE_SIGMA_LSB    248.0f

    // A(t): sparse spikes, sign random, magnitude ~ U(SPIKE_MIN, SPIKE_MAX),
    // injected with Bernoulli(p) per sample
    #define ANOMALY_PROB       0.1f     // test matrix: 0.01, 0.05, 0.10
    #define SPIKE_MIN_LSB      6200.0f   // 5 V in LSB
    #define SPIKE_MAX_LSB      18600.0f  // 15 V in LSB

    // Sliding window for both filters (centered on current sample).
    #define FILTER_WINDOW      40         // 5, 15, 40, 100
    
    // |x[i] - mean| / std > 3.0  →  anomaly
    #define ZSCORE_THRESHOLD   3.0f 
    
    // |x[i] - median| / (1.4826 × MAD) > 3.0  →  anomaly
    // MAD = median absolute deviation = median(|x[i] - median|)
    #define HAMPEL_THRESHOLD   3.0f      

    // Both filters are always evaluated at boot and their TPR/FPR/MER/exec
    // are printed to serial. ACTIVE_FILTER only selects which of the two
    // f_max estimates is handed to the adaptive sampler downstream.
    #define FILTER_ZSCORE      0
    #define FILTER_HAMPEL      1
    #define ACTIVE_FILTER      FILTER_HAMPEL
#endif

#define NUM_COMPONENTS 3
extern const float amplitudes[NUM_COMPONENTS];
extern const float frequencies[NUM_COMPONENTS];

// --- Sampling & FFT ---
#define FFT_SAMPLES        512    // Must be power of 2
#define MAX_SAMPLING_FREQ  500.0  // Hz, initial oversampling rate

// Fraction of the peak FFT magnitude used as threshold to separate real signal
// components from noise/leakage. only bins above the threshold are treated as real frequency components.
#define FFT_MAG_THRESHOLD_RATIO  0.05

// --- Aggregation ---
#define WINDOW_DURATION_SEC 30

// --- DAC output ---
#define DAC_OFFSET           2048  // Mid-scale = 1.65V
#define DAC_AMPLITUDE        1800  // Keep output within 0-4095
// Points per cycle of the highest frequency — controls waveform smoothness.
// 40 is empirical: enough for a smooth sine, low enough for I2C overhead.
// The actual update rate and LUT size are computed at runtime in buildSignalLUT()
// based on the signal frequencies, so changing the signal does not require
// changing any other constant here.
#define DAC_POINTS_PER_CYCLE 40
#define DAC_LUT_MAX          2048  // max LUT entries (compile-time array bound)


#endif
