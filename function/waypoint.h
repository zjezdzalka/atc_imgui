//
// Created by Natan on 12/14/2025.
//

#ifndef IMGUI_RADAR_WAYPOINT_H
#define IMGUI_RADAR_WAYPOINT_H

#endif //IMGUI_RADAR_WAYPOINT_H

#pragma once
#include <vector>
#include <string>

// Waypoint structure
struct Waypoint
{
    std::string name;
    float x, y; // Position of waypoint
};

// Function declaration
std::vector<Waypoint> createWaypoints();