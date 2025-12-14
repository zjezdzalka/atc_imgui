//
// Created by Natan on 12/14/2025.
//

#include "waypoint.h"

std::vector<Waypoint> createWaypoints()
{
    std::vector<Waypoint> waypoints;
    waypoints.push_back({"ALPHA", 20.0f, 30.0f});
    waypoints.push_back({"BRAVO", -25.0f, 35.0f});
    waypoints.push_back({"CHARLIE", 35.0f, -20.0f});
    waypoints.push_back({"DELTA", -30.0f, -25.0f});
    waypoints.push_back({"ECHO", 40.0f, 10.0f});
    waypoints.push_back({"FOXTROT", -15.0f, -40.0f});
    return waypoints;
}