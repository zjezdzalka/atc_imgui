//
// Created by Natan on 12/14/2025.
//

#ifndef IMGUI_RADAR_AIRCRAFT_H
#define IMGUI_RADAR_AIRCRAFT_H

#endif //IMGUI_RADAR_AIRCRAFT_H

#pragma once

#include "emergency_types.h"
#include <string>
#include <vector>

struct Aircraft
{
    std::string callsign;
    float x = 0.0f;
    float y = 0.0f;
    float altitude_ft = 10000.0f;
    float heading_deg = 0.0f;
    float speed_kts = 250.0f;
    bool selected = false;

    std::string squawk_code = "1200";
    bool is_overflight = false;

    // ILS approach
    bool ils_active = false;
    std::string ils_runway = "";

    // Emergency
    EmergencyType emergency = EMERGENCY_NONE;
    float emergency_timer = 0.0f;
    std::string emergency_message = "";

    // Crash effect
    bool is_crashed = false;
    float crash_timer = 0.0f;
    float crash_x = 0.0f;
    float crash_y = 0.0f;

    float target_altitude_ft = 10000.0f;
    float target_heading_deg = 0.0f;
    float target_speed_kts = 250.0f;

    float pending_altitude_ft = 10000.0f;
    float pending_heading_deg = 0.0f;
    float pending_speed_kts = 250.0f;

    float altitude_rate_fps = 0.0f;
    float speed_rate_kps = 0.0f;

    std::string last_response = "";
    float response_timer = 0.0f;

    float command_delay = 0.0f;
    bool has_pending_command = false;

    float distance2_to(const Aircraft& other) const;

    void initTargets();
    void setCommand(const std::string& response, float delay = 3.5f);
    void setImmediateResponse(const std::string& response, float duration = 3.0f);
};

void generateAircraft(std::vector<Aircraft>& aircraft,
                      Aircraft& a,
                      float radar_range_km,
                      int index);