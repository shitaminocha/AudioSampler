# Adaptive IoT Sampler — ESP32 + FreeRTOS

An IoT system that samples a composite sinusoidal signal, computes its FFT to identify the dominant frequency, adapts the sampling rate to the Nyquist minimum, aggregates readings over a window, and transmits the aggregate to an edge server (MQTT over WiFi) and to the cloud (LoRaWAN via TTN).

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Hardware](#hardware)
3. [Project Structure](#project-structure)
4. [Setup and Installation](#setup-and-installation)
5. [Configuration](#configuration)
6. [How It Works](#how-it-works)
7. [Performance Evaluation](#performance-evaluation)
8. [LLM Usage](#llm-usage)
9. [What is Missing / Suggestions](#what-is-missing--suggestions)

---

## System Overview

```
┌─────────────────────────────────────────────────────┐
│                    TTGO LoRa32 V2.1                 │
│                                                     │
│  DAC (GPIO 25) ──► Signal Generator Task            │
│        │                                            │
│        ▼                                            │
│  ADC (GPIO 35) ──► ADC Task ──► FFT ──► Adapt Rate  │
│                        │                            │
│                        ▼                            │
│                   Compute Average                   │
│                   /            \                    │
│           MQTT Task          LoRa Task              │
│               │                  │                  │
└───────────────┼──────────────────┼──────────────────┘
                ▼                  ▼
         Edge Server (PC)    TTN Cloud
         Mosquitto MQTT       LoRaWAN
```

The signal can be generated internally via the ESP32 DAC and read back on the ADC, creating a closed-loop virtual sensor OR read by the ACD with the help of a PC and HeadPhone. The system starts at the maximum hardware sampling rate (5000 Hz), computes an FFT over a full buffer, identifies the highest-frequency component of the signal, and then drops the sampling rate to `2 × f_max` (Nyquist). All readings are averaged over the collection window and transmitted over both MQTT and LoRaWAN.

---

## Hardware

| Component | Details |
|---|---|
| Board | TTGO LoRa32 V2.1 (revision 1.6) |
| MCU | ESP32 (dual-core, 240 MHz) |
| LoRa chip | SX1276 |
| ADC pin | GPIO 35 (ADC1_CH7) |
| DAC pin | GPIO 25 (DAC1) |
| SPI (LoRa) | SCK=5, MISO=19, MOSI=27, CS=18 |
| LoRa DIO | DIO0=26, RST=14, DIO1=33 |

> **⚠️ Note:** GPIO 26 is shared between the SX1276 DIO0 and DAC2. The DAC output uses GPIO 25 (DAC1) to avoid this conflict.

---

## Project Structure

```
EnergyConsumption/
├── src/
│   ├── main.cpp                        # Main application — Measures energy consumption and sends it to serial printer
└── platformio.ini                      # Build configuration
Sampler/
├── src/
│   ├── main.ino                        # Main application — ADC, FFT, average, tasks
│   ├── SignalGenerator.cpp             # Virtual sensor: generates sine wave on DAC
│   └── Communication/
│       ├── CommunicationMQTT.cpp       # WiFi + MQTT connection, RTT measurement
│       └── CommunicationLoRa.cpp       # LoRaWAN ABP session, uplink transmission
├── include/
│   ├── Globals.h                       # Shared constants and compile-time config
│   ├── SignalGenerator.h
│   ├── CommunicationMQTT.h
│   ├── CommunicationLoRa.h
│   └── MQTTPasswords.h                 # WiFi credentials (not committed to git)
└── platformio.ini                      # Build configuration
```

---

## Setup and Installation

### Prerequisites

- [VS Code](https://code.visualstudio.com/) with [PlatformIO](https://platformio.org/install/ide?install=vscode) extension
- [Mosquitto MQTT broker](https://mosquitto.org/download/) installed on your PC
- A TTN account with a registered device (for LoRaWAN)
- Python 3 with `paho-mqtt` for the RTT echo server

### 1. Clone the repository

```bash
git clone <your-repo-url>
cd Sampler
```

### 2. Create `include/MQTTPasswords.h`

This file is not committed to git. Create it manually:

```cpp
#ifndef MQTT_PASSWORDS_H
#define MQTT_PASSWORDS_H

extern const char* WIFI_SSID;
extern const char* WIFI_PASS;

#endif
```

And create `src/Passwords.cpp`:

```cpp
#include "MQTTPasswords.h"

const char* WIFI_SSID = "your_wifi_name";
const char* WIFI_PASS = "your_wifi_password";
```

### 3. Update the MQTT broker IP

In `src/Communication/CommunicationMQTT.cpp`, set the broker IP to your PC's IP on the shared network:

```cpp
const char* mqtt_server = "192.168.x.x";  // your PC's IP
```

Find your IP with `ipconfig` (Windows) or `ip addr` (Linux/Mac).

### 4. Configure Mosquitto

Open `C:\Program Files\mosquitto\mosquitto.conf` (Windows) or `/etc/mosquitto/mosquitto.conf` (Linux) and add:

```
listener 1883 0.0.0.0
allow_anonymous true
```
Start the broker:
net stop mosquitto && net start mosquitto
```

### 5. Run the RTT echo server

```bash
pip install paho-mqtt
python echo_server.py
```

### 6. Flash the firmware

Open the project in VS Code with PlatformIO, then click **Build** and **Upload**, or run:

```bash
pio run -t upload
```

Open the Serial Monitor at 115200 baud to see live output. Open the Teleplot extension in VS Code to visualise the metrics.

---

## Configuration

All tunable parameters are in `include/Globals.h`:

| Parameter | Default | Description |
|---|---|---|
| `FFT_SAMPLES` | 512 | FFT window size — must be a power of 2 |
| `MAX_SAMPLING_FREQ` | 500 Hz | Initial oversampling rate |
| `FFT_MAG_THRESHOLD_RATIO` | 0.05 | Fraction of peak magnitude for frequency detection |
| `WINDOW_DURATION_SEC` | 30 | Aggregation window in seconds |
| `DAC_OFFSET` | 2048 | DAC midpoint (1.65V) |
| `DAC_AMPLITUDE` | 1800 | DAC swing (keeps output in 0–4095 range) |

Signal composition is set in the signal generator. The default is a 2-component sine wave: `2·sin(2π·3·t) + 4·sin(2π·5·t)`.

To change the signal, modify `amplitudes[]` and `frequencies[]` in `Globals.h`.

---

## How It Works

### Signal Generation

The `SignalGenerator` task runs on Core 1 at low priority. It computes a sine wave sample at each step and writes it to the DAC on GPIO 25. The signal is physically looped back to GPIO 35 (ADC) via a wire or the voltage divider circuit described below.

**Voltage divider circuit** (required to bias the AC signal into the ADC's 0–3.1V range):

<!-- 📷 INSERT IMAGE: voltage divider schematic — 1µF capacitor + two 10kΩ resistors biasing signal to 2.5V, feeding into GPIO 35 -->

### ADC Sampling and FFT

The `adcTask` runs on Core 1 at the highest priority. It:

1. Samples the ADC at the current sampling rate using `ets_delay_us`
2. Fills `sampleBuffer`
3. When the buffer is full, calls `switchBuffer()` to copy samples into the FFT buffer, then `computeFFT()`
4. `computeFFT()` applies a Hann window, runs `dsps_fft2r_fc32`, and finds the highest local peak above the magnitude threshold
5. Sets `peakFreq` to the dominant frequency, switches `adaptiveActive = true`
6. The delay between samples is recalculated each iteration: `1,000,000 / (2 × peakFreq)` µs

<!-- 📷 INSERT GRAPH: Teleplot screenshot showing >raw, >signal, and >peak_freq over time — showing rate dropping after FFT completes -->

### Adaptive Sampling Rate

Before FFT completes, the system samples at `MAX_SAMPLING_FREQ` (500 Hz). After the first FFT window, the rate drops to `2 × f_max`. For a signal with components at 3 Hz and 5 Hz, `f_max = 5 Hz` and the adaptive rate is 10 Hz — a 50× reduction.

<!-- 📷 INSERT GRAPH: Teleplot >peak_freq_adaptive over time — showing initial value of 0, then jump to 10Hz after FFT -->

### Window Average

Each time the sample buffer fills, `computeAverage()` is called over all `FFT_SAMPLES` values. The result is stored in `lastAvg` under mutex protection and flagged with `avgReady = true` for the MQTT and LoRa tasks to consume.

### MQTT Transmission (Edge Server)

The `mqttTask` runs on Core 0. It:
- Maintains the MQTT connection via `client.loop()` and `reconnect()`
- Publishes the latest average to `dakshita/IoT/send` every 10 seconds
- Measures RTT by timestamping each publish and receiving the echo from `dakshita/IoT/echo` via the callback
- Accumulates 100 RTT samples then prints the average adjusted RTT

<!-- 📷 INSERT SCREENSHOT: TTN Live Data tab showing uplink messages with decoded average value -->

### LoRaWAN Transmission (Cloud)

The `loraTask` (currently commented out in `setup()`) runs on Core 0. It:
- Initialises the SX1276 radio with explicit SPI pins
- Activates an ABP session using keys from TTN
- Encodes `lastAvg` as a 4-byte IEEE 754 float and calls `node.sendReceive()`
- Transmits every 15 seconds, respecting the TTN fair use policy

To decode the payload on TTN, add this formatter under **Payload Formatters → Uplink → Custom Javascript**:

```javascript
function decodeUplink(input) {
  var b = input.bytes;
  var buf = new ArrayBuffer(4);
  var v = new DataView(buf);
  b.forEach(function(x, i) { v.setUint8(i, x); });
  return { data: { average: v.getFloat32(0, true) } };
}
```

---

## Performance Evaluation

All metrics are streamed in real time to the Teleplot extension in VS Code using the `>name:value` format. To view them, open Teleplot, connect to the correct COM port at 115200 baud, and select the variables to plot.

### Maximum Sampling Frequency

The ESP32 ADC hardware ceiling is approximately **83,000 Hz**. In practice `ets_delay_us` at 0 µs gives around **5,000–10,000 Hz** reliable samples due to instruction overhead. The system starts at `MAX_SAMPLING_FREQ = 500 Hz` to avoid saturating the Serial output.

To measure the true hardware maximum, set `MAX_SAMPLING_FREQ` to `83000` and `SAMPLES` to `64`, then observe the `>per_window` trace.

<!-- 📷 INSERT GRAPH: Teleplot >per_window_time_ms showing window execution time at initial vs adaptive rate -->

### Per-Window Execution Time

Measured in `adcTask` using `esp_timer_get_time()` around the full fill-FFT-average cycle. Transmitted as `>per_window` in microseconds.

| Phase | Typical value |
|---|---|
| ADC read (single sample) | ~20–50 µs |
| FFT (4096-point) | ~50–80 ms |
| Window fill at 500 Hz (4096 samples) | ~8.2 seconds |
| Window fill at 10 Hz (4096 samples) | ~409 seconds |

<!-- 📷 INSERT TABLE or GRAPH: measured per-window times at initial vs adaptive rate, from Teleplot data -->

### Energy Savings

Energy is proportional to the number of ADC reads per second. At the adaptive rate the savings are:

```
Energy saving ≈ 1 - (adaptive_rate / initial_rate)
             = 1 - (10 / 500) = 98%
```

In practice, `ets_delay_us` is a busy-wait so the CPU does not sleep between samples. True energy savings require using the ESP32 light-sleep mode between samples with a timer wakeup. The current implementation demonstrates the sampling rate reduction but does not implement hardware sleep.

<!-- 📷 INSERT GRAPH: bar chart comparing estimated active time (proportional to samples/sec) at initial vs adaptive rate -->

### Data Volume

Tracked by `trackTransmission()` and printed every 10 adaptive packets. Teleplot traces:

- `>bytes_initial` — cumulative bytes sent at the initial rate
- `>bytes_adaptive` — cumulative bytes sent at the adaptive rate
- `>throughput_initial` and `>throughput_adaptive` — bytes per second
- `>data_saving_pct` — percentage reduction

<!-- 📷 INSERT GRAPH: Teleplot showing bytes_initial and bytes_adaptive growing over time, with data_saving_pct stabilising -->

### End-to-End Latency (RTT)

Measured by the MQTT callback in `CommunicationMQTT.cpp`:

1. ESP32 publishes its `millis()` timestamp to `dakshita/IoT/send`
2. The echo server on the PC replies with `timestamp,processing_ms` to `dakshita/IoT/echo`
3. The ESP32 callback computes `RTT = now - timestamp` and `adjusted_RTT = RTT - processing_ms`

After 100 samples, the average adjusted RTT is printed to Serial.

<!-- 📷 INSERT GRAPH: Teleplot >rtt over 100 samples, showing distribution and average -->

---

## LLM Usage

This project was developed iteratively using Claude (Anthropic) as the primary coding assistant. The workflow consisted of a series of prompts covering:

1. Converting Arduino `loop()` to FreeRTOS tasks
2. Adding a queue between the ADC and Serial tasks
3. Integrating FFT using the ESP-DSP library
4. Implementing binary search for adaptive rate (later replaced with direct Nyquist calculation)
5. Adding mutexes for shared variables
6. Integrating MQTT with WiFi
7. Adding LoRaWAN via RadioLib and TTN
8. Debugging memory layout, include guard issues, and linker errors
9. Adding Teleplot telemetry output
10. Data volume tracking

### Opportunities

- **Rapid scaffolding**: FreeRTOS task structure, mutex patterns, and queue wiring were generated correctly on the first or second attempt with minimal correction.
- **Debugging assistance**: The LLM correctly identified the root cause of the `app_main` linker error (missing include guards causing multiple definitions) after seeing the verbose build output.
- **API knowledge**: RadioLib's `beginOTAA` / `beginABP` argument changes between versions were handled correctly once the error message was provided.
- **Explanatory quality**: Each generated code block came with a clear explanation of design choices, making it easy to understand and modify.

### Limitations

- **Hallucinated APIs**: The LLM initially suggested `vTaskNotifyGiveFromISR` for the timer callback, which is correct, but also suggested using it for inter-task notification in contexts where `xTaskNotifyGive` was more appropriate.
- **Version sensitivity**: RadioLib's API changed between v5 and v6. The LLM initially generated v5-style `beginOTAA` calls with too many arguments.
- **Memory estimation errors**: The LLM underestimated PSRAM requirements and initially suggested stack sizes too small for DSP library initialisation.
- **No hardware access**: The LLM cannot verify that pin assignments, SPI initialisation, or hardware bring-up steps are correct without iterative feedback from actual serial output.
- **Accumulated context errors**: Over a long conversation, earlier design decisions (like using a queue vs a shared buffer) were sometimes contradicted in later suggestions, requiring the user to catch and correct inconsistencies.

---

## What is Missing / Suggestions

The following items from the assignment specification are partially or not yet implemented. Suggested code changes are noted for each.

### 1. True energy measurement

**Missing**: The code estimates energy savings from the sampling rate ratio, but does not implement actual low-power sleep between samples.

**Suggestion**: Replace `ets_delay_us` with `esp_sleep_enable_timer_wakeup` + `esp_light_sleep_start()`:

```cpp
// Replace this:
ets_delay_us(delayUs);

// With this (saves ~90% current during idle):
esp_sleep_enable_timer_wakeup(delayUs);
esp_light_sleep_start();
```

Measure current draw with a multimeter or INA219 sensor before and after to get real energy numbers.

### 2. LoRa task is commented out

**Missing**: `loraTask` is created but commented out in `setup()`. It needs to be re-enabled and tested with a gateway in range.

**Fix**:
```cpp
// Uncomment in setup():
xTaskCreatePinnedToCore(loraTask, "LoRa", 8192, NULL, 1, NULL, 0);
// Comment out mqttTask if running both causes memory issues
```

### 3. Signal is single-frequency, not composite

**Missing**: `SignalGenerator.cpp` generates a single sine wave. The assignment requires `SUM(a_k * sin(f_k))` — a sum of multiple frequency components.

**Suggestion**: Update `signalGeneratorTask` to sum multiple components:

```cpp
// In Globals.h
const float amplitudes[]  = {2.0f, 4.0f, 1.0f};
const float frequencies[] = {3.0f, 5.0f, 8.0f};
const int   NUM_COMPONENTS = 3;

// In signalGeneratorTask:
float sample = 0.0f;
for (int k = 0; k < NUM_COMPONENTS; k++) {
  sample += amplitudes[k] * sin(2.0f * PI * frequencies[k] * t);
}
t += 1.0f / signalSampleRate;
// Scale to DAC range: sample is in roughly [-7, 7], scale to [0, 255]
uint8_t dacVal = (uint8_t)((sample / 14.0f + 0.5f) * 255.0f);
dacWrite(signalDacPin, dacVal);
```

### 4. Multiple input signals not tested

**Missing**: The assignment bonus asks for at least 3 different input signals and performance comparison between them.

**Suggestion**: Add a `#define SIGNAL_TYPE` compile flag and switch between signal definitions:

```cpp
#if SIGNAL_TYPE == 1
  // 2sin(2π3t) + 4sin(2π5t)
#elif SIGNAL_TYPE == 2
  // Single high-frequency: 8sin(2π20t)
#elif SIGNAL_TYPE == 3
  // Closely spaced: sin(2π4t) + sin(2π5t)
#endif
```

Re-flash with each signal type and record the FFT output, adaptive rate, and data savings for comparison.

### 5. Bonus: Noisy signal with anomaly injection

**Present but not active**: `Globals.h` defines all the constants for Gaussian noise and anomaly injection (`NOISE_SIGMA_LSB`, `ANOMALY_PROB`, etc.) under the `#ifdef BONUS` guard, but the actual injection and filter code is not implemented.

**Suggestion**: Add to `signalGeneratorTask`:

```cpp
#ifdef BONUS
  // Gaussian noise (Box-Muller approximation)
  float noise = 0;
  for (int i = 0; i < 12; i++) noise += ((float)random(1000) / 1000.0f);
  noise = (noise - 6.0f) * NOISE_SIGMA_LSB;
  sample += noise;

  // Anomaly injection
  if ((float)random(1000) / 1000.0f < ANOMALY_PROB) {
    float spike = SPIKE_MIN_LSB + (float)random(1000) / 1000.0f * (SPIKE_MAX_LSB - SPIKE_MIN_LSB);
    sample += (random(2) == 0) ? spike : -spike;
  }
#endif
```

Z-score and Hampel filters would then run over each completed window before the FFT, removing anomalies and tracking TPR/FPR.

### 6. `timerCallback` not used

The code previously had a FreeRTOS timer to trigger the FFT every `LOOP_TIME` ms, but this was removed in favour of triggering FFT when the buffer fills. Consider re-adding the timer as an alternative trigger for fixed-interval aggregation windows independent of sampling rate.

---

<!-- 📷 FINAL IMAGE SUGGESTION: System architecture diagram showing all tasks, cores, queues, and external connections (MQTT broker, TTN) -->