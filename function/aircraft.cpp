//
// Created by rytui on 12/3/25.
//

#include "aircraft.h"
#include "utils.h"

#include <sstream>
#include <iomanip>
#include <cmath>
#include <cstdlib>

#ifndef IM_PI
#define IM_PI 3.14159265358979323846f
#endif

float Aircraft::distance2_to(const Aircraft& other) const
{
    // Calculate lateral distance to other aircraft
    float dx = x - other.x;
    float dy = y - other.y;
    return dx * dx + dy * dy;
}

void Aircraft::initTargets()
{
    // default aircraft values
    target_altitude_ft = altitude_ft;
    target_heading_deg = heading_deg;
    target_speed_kts = speed_kts;

    pending_altitude_ft = altitude_ft;
    pending_heading_deg = heading_deg;
    pending_speed_kts = speed_kts;

    altitude_rate_fps = 0.0f;
    speed_rate_kps = 0.0f;
}

void Aircraft::setCommand(const std::string& response, float delay)
{
    if (is_overflight) return;

    last_response = response;
    response_timer = 3.75f;
    command_delay = delay;
    has_pending_command = true;
}

void Aircraft::setImmediateResponse(const std::string& response, float duration)
{
    if (is_overflight) return;

    last_response = response;
    response_timer = duration;

    command_delay = 0.0f;
    has_pending_command = false;
}

void generateAircraft(std::vector<Aircraft>& aircraft,
                      Aircraft& a,
                      float radar_range_km,
                      int i)
{
    std::ostringstream ss;
    ss << "AC" << std::setw(2) << std::setfill('0') << (i + 1);
    a.callsign = ss.str();

    const float r = ((float)rand() / RAND_MAX) * radar_range_km;
    const float theta = ((float)rand() / RAND_MAX) * 2.0f * IM_PI;

    a.x = cosf(theta) * r;
    a.y = sinf(theta) * r;

    // 10% chance of overflight
    if (rand() % 100 < 10)
    {
        a.is_overflight = true;
        a.altitude_ft = 32000.0f + (rand() % 111) * 100;
        a.squawk_code = "----";
    }
    else
    {
        a.is_overflight = false;
        a.altitude_ft = 1600.0f + (rand() % 215) * 100;
        a.squawk_code = generateSquawkCode();

        if (rand() % 100 < 2)
        {
            int emergency_type = rand() % 4;
            a.emergency = (EmergencyType)(emergency_type + 1);

            float dist_to_airport = sqrtf(a.x * a.x + a.y * a.y);

            switch (a.emergency)
            {
            case EMERGENCY_LOW_FUEL:
                a.emergency_timer = 240.0f + dist_to_airport * 8.0f;
                a.emergency_message = "Low fuel - requesting priority landing";
                a.squawk_code = "7700";
                break;

            case EMERGENCY_MEDICAL:
                a.emergency_timer = 480.0f + dist_to_airport * 12.0f;
                a.emergency_message = "Medical emergency on board";
                a.squawk_code = "7700";
                break;

            case EMERGENCY_ENGINE_FAILURE:
                a.emergency_timer = 150.0f + dist_to_airport * 4.0f;
                a.emergency_message = "Engine failure - declaring emergency";
                a.squawk_code = "7700";
                break;

            case EMERGENCY_HYDRAULIC:
                a.emergency_timer = 320.0f + dist_to_airport * 10.0f;
                a.emergency_message = "Hydraulic system failure";
                a.squawk_code = "7700";
                break;

            default:
                break;
            }
        }
    }

    a.heading_deg = (float)(rand() % 72) * 5;
    a.speed_kts = 130.0f + (rand() % 180);
    a.selected = false;

    a.initTargets();
}

std::vector<Aircraft> generateInitialAircraft(int count, float radar_range_km)
{
    std::vector<Aircraft> aircraftList;

    for (int i = 0; i < count; ++i)
    {
        Aircraft a;
        generateAircraft(aircraftList, a, radar_range_km, i);
        aircraftList.push_back(a);
    }

    return aircraftList;
}