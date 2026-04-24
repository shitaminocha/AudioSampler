#include "SignalGenerator.h"
#include "Globals.h"
#include <math.h>

// -----------------------------------------------------------------------------
// Signal definition: s(t) = SUM(a_k * sin(2*pi*f_k*t))
//
// Uses the ESP32 built-in 8-bit DAC on GPIO 25 (DAC1).
// GPIO 26 (DAC2) is NOT used — it is shared with LoRa DIO0.
//
// DAC output range: 0-255 → 0V-3.3V
// ADC input range with ADC_11db: 0-3.1V → keep DAC output below 242/255
//
// Safe DAC range: center=127, amplitude=100
//   min = 127-100 = 27  → 27/255*3.3 = 0.35V  ✓
//   max = 127+100 = 227 → 227/255*3.3 = 2.94V  ✓ (below 3.1V ceiling)
//
// Timing constraint: 1000 / (maxFreq * DAC_POINTS_PER_CYCLE) must be an
// integer number of ms for vTaskDelay to be accurate.
// With DAC_POINTS_PER_CYCLE=40:
//   maxFreq=5Hz  → updateRate=200Hz  → periodMs=5ms  ✓
//   maxFreq=25Hz → updateRate=1000Hz → periodMs=1ms  ✓
//   maxFreq=10Hz → updateRate=400Hz  → periodMs=2.5ms ✗
// -----------------------------------------------------------------------------

// Signal 1 — Low frequency (3, 4, 5 Hz)
// f_max=5Hz → adaptive rate=10Hz → 50x reduction from 500Hz
// Longest window fill time, biggest energy saving demonstration
// const float amplitudes[NUM_COMPONENTS]  = { 2.0f, 4.0f, 1.5f };
// const float frequencies[NUM_COMPONENTS] = { 3.0f, 4.0f, 5.0f };

// Signal 2 — Medium frequency (5, 10, 25 Hz) — ACTIVE
// f_max=25Hz → adaptive rate=50Hz → 10x reduction from 500Hz
// Good balance: fast window fill, clearly composite, meaningful saving
const float amplitudes[NUM_COMPONENTS]  = { 20.0f };
const float frequencies[NUM_COMPONENTS] = { 20.0f };
// const float amplitudes[NUM_COMPONENTS]  = { 2.0f, 4.0f, 1.5f };
// const float frequencies[NUM_COMPONENTS] = { 5.0f, 10.0f, 25.0f };

// Signal 3 — High frequency (25, 50, 100 Hz)
// f_max=100Hz → adaptive rate=200Hz → 2.5x reduction
// Smallest saving — useful as comparison where adaptive has minimal benefit
// Requires ets_delay_us (periodMs < 1ms)
// const float amplitudes[NUM_COMPONENTS]  = { 1.0f, 2.0f, 1.5f };
// const float frequencies[NUM_COMPONENTS] = { 25.0f, 50.0f, 100.0f };

// -----------------------------------------------------------------------------
// Lookup table — precomputes one full composite period as 8-bit DAC values.
// At runtime TaskDACGenerator just cycles through the array.
// -----------------------------------------------------------------------------
static uint8_t  signalLUT[DAC_LUT_MAX];
static int lutSize = 0;
static float dacUpdateRate = 0;
static uint32_t periodUs = 0;

static int gcd(int a, int b) {
    while (b) { int t = b; b = a % b; a = t; }
    return a;
}

void buildSignalLUT() {
    // Find max frequency
    float maxFreq = 0;
    for (int k = 0; k < NUM_COMPONENTS; k++)
        if (frequencies[k] > maxFreq) maxFreq = frequencies[k];

    dacUpdateRate = maxFreq * DAC_POINTS_PER_CYCLE;
    periodUs      = (uint32_t)(1000000.0f / dacUpdateRate);

    // Composite period = 1 / GCD(all frequencies)
    int gcdFreq = (int)(frequencies[0] * 1000);
    for (int k = 1; k < NUM_COMPONENTS; k++)
        gcdFreq = gcd(gcdFreq, (int)(frequencies[k] * 1000));
    float compositePeriod = 1000.0f / (float)gcdFreq;   // seconds

    lutSize = (int)(dacUpdateRate * compositePeriod);
    if (lutSize > DAC_LUT_MAX) lutSize = DAC_LUT_MAX;

    // Normalise by sum of absolute amplitudes
    float maxAmpSum = 0;
    for (int k = 0; k < NUM_COMPONENTS; k++)
        maxAmpSum += fabsf(amplitudes[k]);

    // Precompute 8-bit DAC values
    // Map [-maxAmpSum, +maxAmpSum] → [27, 227] (safe 8-bit range)
    const uint8_t DAC_CENTER    = 127;
    const uint8_t DAC_SWING     = 100;

    for (int i = 0; i < lutSize; i++) {
        float t   = (float)i / dacUpdateRate;
        float sig = 0.0f;
        for (int k = 0; k < NUM_COMPONENTS; k++)
            sig += amplitudes[k] * sinf(2.0f * PI * frequencies[k] * t);

        // Normalise and scale to 8-bit DAC range
        float normalised = sig / maxAmpSum;   // -1.0 to +1.0
        int   dacVal     = (int)(DAC_CENTER + normalised * DAC_SWING);

        // Clamp to 0-255
        if (dacVal < 0)   dacVal = 0;
        if (dacVal > 255) dacVal = 255;
        signalLUT[i] = (uint8_t)dacVal;
    }

    Serial.printf("[DAC] GPIO 25 | %d components | f_max=%.0f Hz\n",
                  NUM_COMPONENTS, maxFreq);
    Serial.printf("[DAC] LUT: %d entries | update %.0f Hz | period %.3f s\n",
                  lutSize, dacUpdateRate, compositePeriod);
    Serial.printf("[DAC] Inter-sample: %lu us | DAC range: %d-%d/255 (%.2fV-%.2fV)\n",
                  periodUs,
                  DAC_CENTER - DAC_SWING, DAC_CENTER + DAC_SWING,
                  (DAC_CENTER - DAC_SWING) / 255.0f * 3.3f,
                  (DAC_CENTER + DAC_SWING) / 255.0f * 3.3f);
}

// -----------------------------------------------------------------------------
// DAC task — runs on Core 0 at lowest priority so it never blocks adcTask.
// Cycles through signalLUT at dacUpdateRate.
// Uses vTaskDelay for periodMs >= 1ms, ets_delay_us for sub-millisecond periods.
// -----------------------------------------------------------------------------
void TaskDACGenerator(void *pvParameters) {
    uint32_t periodMs = periodUs / 1000;

    Serial.printf("[DAC] Task started | period %lu us\n", periodUs);

    int idx = 0;
    while (true) {
        dacWrite(25, signalLUT[idx]);   // GPIO 25 = DAC1, built-in ESP32 DAC

        // Throttled Teleplot output — every 20 samples to avoid Serial flood
        if (idx % 20 == 0) {
            Serial.printf(">dac_out:%u\n", signalLUT[idx]);
        }

        idx++;
        if (idx >= lutSize) idx = 0;

        if (periodMs >= 1) {
            vTaskDelay(pdMS_TO_TICKS(periodMs));
        } else {
            ets_delay_us(periodUs);   // busy-wait for sub-ms periods (Signal 3)
        }
    }
}