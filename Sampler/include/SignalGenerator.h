#ifndef SIGNAL_GENERATOR_H
#define SIGNAL_GENERATOR_H

#include <Arduino.h>

#define NUM_COMPONENTS 1
extern const float amplitudes[NUM_COMPONENTS];
extern const float frequencies[NUM_COMPONENTS];

void buildSignalLUT();
void TaskDACGenerator(void *pvParameters);

#endif