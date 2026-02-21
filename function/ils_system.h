// ils_system.h

#ifndef ILS_SYSTEM_H
#define ILS_SYSTEM_H

#pragma once
#include "aircraft.h"
#include "runway.h"
#include <string>
#include <cmath>

struct ILSInfo
{
    bool  is_intercepting           = false;
    float intercept_heading         = 0.0f;
    float last_intercept_heading    = -999.0f;
    int   turn_count                = 0;

    bool  established_localizer     = false;
    bool  established_glideslope    = false;
    bool  has_landed                = false;   // true = successful landing, not a crash

    float localizer_deviation       = 0.0f;    // deg: +LEFT of course, -RIGHT of course
    float glideslope_deviation      = 0.0f;    // ft:  +HIGH, -LOW
    float distance_to_runway        = 0.0f;    // km

    std::string ils_status          = "NOT ON ILS";
};

// Returns true if aircraft can intercept localizer (angle <= 30 deg, on approach side)
bool  canInterceptLocalizer(const Aircraft& a, const Runway& rwy);

// Returns the intercept angle in degrees for display
float getInterceptAngle(const Aircraft& a, const Runway& rwy);

// Localizer deviation: +LEFT of course, -RIGHT of course (degrees)
float calculateLocalizerDeviation(const Aircraft& a, const Runway& rwy);

// Glideslope deviation: +HIGH, -LOW (feet)
float calculateGlideslopeDeviation(const Aircraft& a, const Runway& rwy);

// Per-frame update. Writes directly to a.target_heading_deg / a.target_altitude_ft.
void  updateILSApproach(Aircraft& a, const Runway& rwy, ILSInfo& ils_info, float dt);

// Formatted status string for the HUD
std::string getILSStatusMessage(const ILSInfo& info);

#endif // ILS_SYSTEM_H