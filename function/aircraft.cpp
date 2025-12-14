//
// Created by rytui on 12/3/25.
//

#include<vector>
#include<string>
#include<iomanip>
#include<cmath>

#ifndef IM_PI
#define IM_PI 3.14159265358979323846f
#endif

struct Aircraft
{
    std::string callsign;
    // world position in kilometers relative to radar center (x east, y north)
    float x = 0.0f;
    float y = 0.0f;
    float altitude_ft = 10000.0f;
    float heading_deg = 0.0f; // 0 = east, +counterclockwise (so 90 = north)
    float speed_kts = 250.0f; // knots
    bool selected = false;

    // Target values for gradual transition
    float target_altitude_ft = 10000.0f;
    float target_heading_deg = 0.0f;
    float target_speed_kts = 250.0f;

    // Pending target values (applied after delay)
    float pending_altitude_ft = 10000.0f;
    float pending_heading_deg = 0.0f;
    float pending_speed_kts = 250.0f;

    // Velocity/rate values for smooth transitions
    float altitude_rate_fps = 0.0f; // feet per second
    float speed_rate_kps = 0.0f; // knots per second

    // Command acknowledgment system
    std::string last_response = "";
    float response_timer = 0.0f;

    // Delay before starting transition (reaction time)
    float command_delay = 0.0f;
    bool has_pending_command = false;

    // convenience
    float distance2_to(const Aircraft& other) const
    {
        float dx = x - other.x;
        float dy = y - other.y;
        return dx * dx + dy * dy;
    }

    // Initialize targets to current values
    void initTargets()
    {
        target_altitude_ft = altitude_ft;
        target_heading_deg = heading_deg;
        target_speed_kts = speed_kts;
        pending_altitude_ft = altitude_ft;
        pending_heading_deg = heading_deg;
        pending_speed_kts = speed_kts;
        altitude_rate_fps = 0.0f;
        speed_rate_kps = 0.0f;
    }

    // Set new command with acknowledgment
    void setCommand(const std::string& response, float delay = 3.5f)
    {
        last_response = response;
        response_timer = 5.0f; // Show response for 5 seconds
        command_delay = delay;
        has_pending_command = true;

        // DON'T overwrite pending values - they're set externally before calling setCommand
    }
};

void generateAircraft(std::vector<Aircraft>& aircrafts, Aircraft& a, const float radar_range_km, const int i) {
    std::ostringstream ss;
    ss << "AC" << std::setw(2) << std::setfill('0') << (i + 1);
    a.callsign = ss.str();

    // random position in circle
    const float r = ((float)rand() / RAND_MAX) * radar_range_km;
    const float theta = ((float)rand() / RAND_MAX) * 2.0f * IM_PI;
    a.x = cosf(theta) * r;
    a.y = sinf(theta) * r;

    a.altitude_ft = 1600.0f + (rand() % 430)*100;
    a.heading_deg = (float)(rand() % 72)*5;
    a.speed_kts = 130.0f + (rand() % 300); // 130..430 kts
    a.selected = false;

    // Initialize targets
    a.initTargets();
}
