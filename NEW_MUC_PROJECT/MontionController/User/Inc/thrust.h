#ifndef __THRUST_H
#define __THRUST_H

#include "autocontrol.h"

// #define __A_1 0.178601329f
// #define __A_2 0.1760656194f
// #define __B 0.299777458f
// #define __C 0.275844479f
// #define __2b 0.216f

typedef struct {
    float A_1;
    float A_2;
    float B;
    float C;
    float _2b; 
} ThrustParams_t;



void ThrustAllocate(float *askedthrust, float *motorthrust);

#endif