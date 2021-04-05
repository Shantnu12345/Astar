//Author: Shantnu Kakkar

#include "helper.h"

float normalize(float angle, float ref)
{
    float temp = std::fmod(angle - ref, TwoPI);
    return temp >= 0.0f ? temp + ref: temp + ref + TwoPI;
}