#ifndef SIGNAL_GENERATOR_H
#define SIGNAL_GENERATOR_H

#include <Arduino.h>

// Initialize the signal generator with DAC pin and sample rate
void signalGeneratorSetup(int dacPin, float sampleRate);

// Set the frequency of the sine wave (in Hz)
void signalGeneratorSetFrequency(float freq);

// Start generating and outputting the signal
void signalGeneratorStart();

// Stop generating the signal
void signalGeneratorStop();

#endif