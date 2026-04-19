#include <Arduino.h>
int outPin = 4;

void setup () {
    pinMode (outPin, OUTPUT);
}

void loop () {
    digitalWrite (outPin, HIGH);
    delay (3000);
    // Serial.print ("World");
    digitalWrite (outPin, LOW);
    delay (5000);
}