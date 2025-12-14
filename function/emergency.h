//
// Created by Natan on 12/14/2025.
//

#ifndef IMGUI_RADAR_EMERGENCY_H
#define IMGUI_RADAR_EMERGENCY_H

#endif //IMGUI_RADAR_EMERGENCY_H

#pragma once

#include "aircraft.h"
#include <vector>

// Draws the emergency UI for a selected aircraft
void DrawEmergencyPanel(const Aircraft& sel);

// Random emergencies
void GenerateRandomEmergency(std::vector<Aircraft>& aircraft, float dt, float& timer, float interval = 20.0f);