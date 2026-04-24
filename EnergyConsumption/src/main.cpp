/**
 * ============================================================
 * Current Monitoring with INA219 – Arduino MEGA 2560
 * Framework : Arduino (PlatformIO)
 * Sensor    : INA219 (I2C)
 * ============================================================
 *
 * INA219 → MEGA 2560 Connection
 * ─────────────────────────────────
 * INA219 VCC  →  5V
 * INA219 GND  →  GND
 * INA219 SDA  →  Pin 20 (SDA)
 * INA219 SCL  →  Pin 21 (SCL)
 *
 * Connection with the other board (load to be monitored)
 * ─────────────────────────────────────────────────────
 * The VIN+ terminal of the INA219 goes to the positive supply
 * of the board to be monitored; VIN- to the positive pin of the load
 *
 * Available I2C addresses (A0/A1 pins on the INA219):
 * 0x40  →  A0=GND, A1=GND  (default)
 * 0x41  →  A0=VCC, A1=GND
 * 0x44  →  A0=GND, A1=VCC
 * 0x45  →  A0=VCC, A1=VCC
 * ============================================================
*/
 
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

#define INA219_ADDRESS  0x40      // Change if necessary
#define SAMPLE_INTERVAL 100       // ms between readings
#define OVERCURRENT_MA  500.0f    // Current alarm threshold (mA)
#define OVERVOLTAGE_V   5.5f      // Voltage alarm threshold (V)
#define LED_ALERT       13        // Integrated alarm LED

Adafruit_INA219 ina219(INA219_ADDRESS);

float peakCurrentMA  = 0.0f;
float peakVoltageV   = 0.0f;
float totalEnergyMWh = 0.0f;
uint32_t sampleCount = 0;
uint32_t lastSampleMs = 0;

void printHeader();
void printReadings(float busV, float shuntMV, float currentMA, float powerMW);
void printOnlyCurrentReading(float currentMA);
void checkAlerts(float currentMA, float busV);
void printStats();


void setup() {
    pinMode(LED_ALERT, OUTPUT);
    digitalWrite(LED_ALERT, LOW);
 
    Serial.begin(115200);
    while (!Serial) { ; }

    if (!ina219.begin()) {
        Serial.println("[ERROR] INA219 not found! Check the wiring.");
        Serial.print("         Expected I2C address: 0x");
        Serial.println(INA219_ADDRESS, HEX);
        while (true) {
            digitalWrite(LED_ALERT, !digitalRead(LED_ALERT));
            delay(200);
        }
    }
 
    // Calibration for 16V 400mA range
    ina219.setCalibration_16V_400mA();
    lastSampleMs = millis();
}
 

void loop() {
    uint32_t now = millis();
 
    if (now - lastSampleMs >= SAMPLE_INTERVAL) {
        uint32_t dtMs  = now - lastSampleMs;
        lastSampleMs   = now;
 

        float shuntMV  = ina219.getShuntVoltage_mV();
        float busV     = ina219.getBusVoltage_V();
        float currentMA= ina219.getCurrent_mA();
        float powerMW  = ina219.getPower_mW();
 
        // Ignore abnormal readings (overflow)
        if (currentMA > 400.0f || currentMA < -400.0f) {
            Serial.println("Sensor overflow - check the load");
            return;
        }

        sampleCount++;
        if (currentMA > peakCurrentMA) peakCurrentMA = currentMA;
        if (busV      > peakVoltageV)  peakVoltageV  = busV;
 
        printOnlyCurrentReading(currentMA);
        
        checkAlerts(currentMA, busV);
    }
}

void printOnlyCurrentReading(float currentMA) {
    Serial.print(">current:");
    Serial.print(currentMA, 2);
    Serial.println();
}


void checkAlerts(float currentMA, float busV) {
    bool alert = false;
 
    if (currentMA > OVERCURRENT_MA) {
        Serial.print("  [!] OVERCURRENT ALARM: ");
        Serial.print(currentMA, 2);
        Serial.print(" mA > threshold ");
        Serial.print(OVERCURRENT_MA, 0);
        Serial.println(" mA");
        alert = true;
    }
 
    if (busV > OVERVOLTAGE_V) {
        Serial.print(F("  [!] OVERVOLTAGE ALARM: "));
        Serial.print(busV, 3);
        Serial.print(F(" V > threshold "));
        Serial.print(OVERVOLTAGE_V, 2);
        Serial.println(F(" V"));
        alert = true;
    }

    digitalWrite(LED_ALERT, alert ? HIGH : LOW);
}