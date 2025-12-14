//
// Created by Natan on 12/14/2025.
//

#ifndef IMGUI_RADAR_UTILS_H
#define IMGUI_RADAR_UTILS_H

#endif //IMGUI_RADAR_UTILS_H

#pragma once

#include <string>
#include <iomanip>

#ifndef IM_PI
#define IM_PI 3.14159265358979323846f
#endif

// Angle utilities
float angle_difference(float target, float current);
float deg_to_rad(float d);
float rad_to_deg(float r);

// Squawk code generator
std::string generateSquawkCode();

// Radar
static constexpr float radar_range_km = 80.0f;