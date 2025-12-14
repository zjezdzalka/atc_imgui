//
// Created by Natan on 12/14/2025.
//

#include "utils.h"
#include <cmath>

// Returns the shortest difference between two angles (-180..+180)
float angle_difference(float target, float current)
{
    float diff = fmodf(target - current + 540.0f, 360.0f) - 180.0f;
    return diff;
}

// Converts degrees to radians
float deg_to_rad(float d)
{
    return d * (IM_PI / 180.0f);
}

// Converts radians to degrees
float rad_to_deg(float r)
{
    return r * (180.0f / IM_PI);
}

// Generate a random 4-digit squawk code (1000-7999)
std::string generateSquawkCode()
{
    int code = 1000 + rand() % 7000;
    std::ostringstream ss;
    ss << std::setw(4) << std::setfill('0') << code;
    return ss.str();
}