//
// Created by Natan on 12/14/2025.
//

#ifndef IMGUI_RADAR_RUNWAY_H
#define IMGUI_RADAR_RUNWAY_H

#endif //IMGUI_RADAR_RUNWAY_H

#pragma once
#include <vector>
#include <string>

// Runway struct definition
struct Runway
{
    std::string name;
    float heading_deg;
    float x;
    float y;
};

// Function declaration
std::vector<Runway> createRunways();