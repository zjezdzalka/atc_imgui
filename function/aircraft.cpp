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
    // 0 should be north, clockwise -> 90 = east, 180 = south
    float speed_kts = 250.0f; // knots
    bool selected = false;

    // convenience
    float distance2_to(const Aircraft& other) const
    {
        float dx = x - other.x;
        float dy = y - other.y;
        return dx * dx + dy * dy;
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
}
