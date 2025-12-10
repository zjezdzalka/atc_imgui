//
// Created by rytui on 12/3/25.
//

#include<vector>
#include<string>
#include<iomanip>
#include<cmath>
#include<sstream>

#ifndef IM_PI
#define IM_PI 3.14159265358979323846f
#endif

// Emergency types
enum EmergencyType
{
    EMERGENCY_NONE,
    EMERGENCY_LOW_FUEL,
    EMERGENCY_MEDICAL,
    EMERGENCY_ENGINE_FAILURE,
    EMERGENCY_HYDRAULIC
};

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
    bool is_overflight = false; // High altitude, non-contactable

    // ILS approach
    bool ils_active = false;
    std::string ils_runway = "";
    bool ils_established = false;
    float ils_localizer_deviation = 0.0f; // meters from centerline
    float ils_glideslope_deviation = 0.0f; // feet from glideslope
    bool cleared_to_land = false;
    bool go_around = false;
    float go_around_timer = 0.0f;

    // Hold pattern
    bool in_hold_pattern = false;
    float hold_heading = 0.0f;
    float hold_timer = 0.0f;
    float hold_duration = 0.0f; // 0 = indefinite
    bool hold_entry_complete = false;
    float hold_altitude = 0.0f;

    // Direct to point
    bool direct_to_active = false;
    float direct_to_x = 0.0f;
    float direct_to_y = 0.0f;
    float direct_tolerance_km = 1.0f;

    // Emergency
    EmergencyType emergency = EMERGENCY_NONE;
    float emergency_timer = 0.0f; // Time to land
    std::string emergency_message = "";

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

    // Command history
    std::vector<std::pair<std::string, std::string>> command_history; // ATC command, Pilot response

    float distance2_to(const Aircraft& other) const
    {
        float dx = x - other.x;
        float dy = y - other.y;
        return dx * dx + dy * dy;
    }

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

    void setCommand(const std::string& atc_command, const std::string& pilot_response, float delay = 3.5f)
    {
        if (is_overflight) return; // Cannot contact overflights

        command_history.push_back({atc_command, pilot_response});
        if (command_history.size() > 50) {
            command_history.erase(command_history.begin());
        }

        last_response = pilot_response;
        response_timer = 5.0f;
        command_delay = delay;
        has_pending_command = true;
    }

    void addCommandToHistory(const std::string& atc_command, const std::string& pilot_response) {
        command_history.push_back({atc_command, pilot_response});
        if (command_history.size() > 50) {
            command_history.erase(command_history.begin());
        }
    }
};

// Generate squawk code
std::string generateSquawkCode()
{
    int code = 1000 + rand() % 7000;
    std::ostringstream ss;
    ss << std::setw(4) << std::setfill('0') << code;
    return ss.str();
}

void generateAircraft(std::vector<Aircraft>& aircrafts, Aircraft& a, const float radar_range_km, const int i, bool is_random_spawn = false) {
    std::ostringstream ss;
    ss << "AC" << std::setw(2) << std::setfill('0') << (i + 1);
    a.callsign = ss.str();

    const float r = ((float)rand() / RAND_MAX) * radar_range_km;
    const float theta = ((float)rand() / RAND_MAX) * 2.0f * IM_PI;
    a.x = cosf(theta) * r;
    a.y = sinf(theta) * r;

    // 10% chance of overflight (only for random spawns, not initial)
    if (is_random_spawn && rand() % 100 < 10)
    {
        a.is_overflight = true;
        a.altitude_ft = 30000.0f + (rand() % 100) * 100; // 30000-40000 ft
        a.squawk_code = "----"; // No transponder contact
    }
    else
    {
        a.is_overflight = false;
        a.altitude_ft = 1600.0f + (rand() % 430) * 100;
        a.squawk_code = generateSquawkCode();

        // 0.5% chance of emergency (rarer)
        if (rand() % 200 < 1)
        {
            int emergency_type = rand() % 4;
            a.emergency = (EmergencyType)(emergency_type + 1);

            float dist_to_airport = sqrtf(a.x * a.x + a.y * a.y);

            switch(a.emergency)
            {
                case EMERGENCY_LOW_FUEL:
                    a.emergency_timer = 300.0f + dist_to_airport * 10.0f; // 5+ min
                    a.emergency_message = "Low fuel - requesting priority landing";
                    a.squawk_code = "7700";
                    break;
                case EMERGENCY_MEDICAL:
                    a.emergency_timer = 600.0f + dist_to_airport * 15.0f; // 10+ min
                    a.emergency_message = "Medical emergency on board";
                    a.squawk_code = "7700";
                    break;
                case EMERGENCY_ENGINE_FAILURE:
                    a.emergency_timer = 180.0f + dist_to_airport * 5.0f; // 3+ min
                    a.emergency_message = "Engine failure - declaring emergency";
                    a.squawk_code = "7700";
                    break;
                case EMERGENCY_HYDRAULIC:
                    a.emergency_timer = 400.0f + dist_to_airport * 12.0f; // 6+ min
                    a.emergency_message = "Hydraulic system failure";
                    a.squawk_code = "7700";
                    break;
                default:
                    break;
            }
        }
    }

    a.heading_deg = (float)(rand() % 72) * 5;
    a.speed_kts = 130.0f + (rand() % 300);
    a.selected = false;

    // Initialize all states
    a.ils_active = false;
    a.ils_runway = "";
    a.ils_established = false;
    a.cleared_to_land = false;
    a.go_around = false;
    a.in_hold_pattern = false;
    a.direct_to_active = false;

    a.initTargets();
}