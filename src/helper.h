#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
using namespace std;

const float PI    = 3.14f;
const float TwoPI = 2*PI;

float normalize(float angle, float ref = -PI)
{
    float temp = std::fmod(angle - ref, TwoPI);
    return temp >= 0.0f ? temp + ref: temp + ref + TwoPI;
}