// ils_system.cpp

// ================================================================
// COORDINATE SYSTEM — read this before touching anything
// ================================================================
// heading_deg is in MATH convention:
//   0 deg  = East  (+X)    90 deg = North (+Y)
//  180 deg = West  (-X)   270 deg = South (-Y)
//  Angles increase counter-clockwise.
//
// Position update in main.cpp:
//   x += cosf(deg_to_rad(heading_deg)) * speed * dt;
//   y += sinf(deg_to_rad(heading_deg)) * speed * dt;
//
// Runway heading_deg = the direction the runway FACES (math convention).
//   RWY 27: heading=180 (faces West).  Aircraft FLY EAST (0 deg) to land.
//   RWY 09: heading=0   (faces East).  Aircraft FLY WEST (180 deg) to land.
//
// INBOUND COURSE = rwy.heading_deg + 180, wrapped to [0,360).
// That is the heading an aircraft must fly to approach and land.
//
// Runway positions from runway.cpp:
//   RWY 27: x=-1, y=-1    RWY 09: x=1, y=1
//
// Aircraft approaching RWY 27 (flying East) come from the WEST,
// i.e. their x is LESS than rwy.x (-1).  The inbound unit vector
// is East = (1,0), so (aircraft - runway) dotted with (1,0)
// = (a.x - rwy.x) which is negative for approach aircraft.
// Therefore "aircraft is on approach side" means along_track < 0,
// NOT > 0.  (The aircraft approaches from behind the threshold
// in the direction the runway faces, not in front of it.)
// ================================================================

#include "ils_system.h"
#include "utils.h"

#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>

#ifndef IM_PI
#define IM_PI 3.14159265358979323846f
#endif

// ----------------------------------------------------------------
// Static helpers
// ----------------------------------------------------------------

static float wrap360(float a)
{
    return fmodf(a + 360.0f, 360.0f);
}

// Math-convention bearing FROM (ax,ay) TOWARD (bx,by), [0,360)
static float bearing_toward(float ax, float ay, float bx, float by)
{
    return wrap360(rad_to_deg(atan2f(by - ay, bx - ax)));
}

// Inbound course: heading aircraft must fly to land on this runway
static float inbound_course(const Runway& rwy)
{
    return wrap360(rwy.heading_deg + 180.0f);
}

// Signed cross-track distance (km) from aircraft to runway centerline.
// The centerline extends in the INBOUND direction from the runway.
//
// Sign convention (from the pilot's seat flying inbound):
//   Positive = aircraft is to the LEFT  of centerline -> must turn RIGHT
//   Negative = aircraft is to the RIGHT of centerline -> must turn LEFT
//
// Derivation: inbound unit vector L = (cos(inbound), sin(inbound))
// Aircraft vector from runway: R = (a.x-rwy.x, a.y-rwy.y)
// Cross product (2D): R x L = Rx*Ly - Ry*Lx
// When flying East (inbound=0): L=(1,0), cross = Rx*0 - Ry*1 = -Ry
//   Aircraft north of centerline (Ry>0): cross=-Ry<0 => that would mean RIGHT.
//   But north IS left when flying East. So the sign must be FLIPPED.
// Therefore: cross_track = Ry*Lx - Rx*Ly  (opposite of the "standard" formula)
static float cross_track_km(const Aircraft& a, const Runway& rwy)
{
    float rad = deg_to_rad(inbound_course(rwy));
    float lx  = cosf(rad);
    float ly  = sinf(rad);
    float rx  = a.x - rwy.x;
    float ry  = a.y - rwy.y;
    // Corrected sign: positive = LEFT of inbound centerline
    return ry * lx - rx * ly;
}

// Along-track distance from runway threshold in the INBOUND direction.
// Aircraft on correct approach side are coming from BEHIND the threshold
// (opposite to inbound direction), so this returns NEGATIVE for them.
static float along_track_km(const Aircraft& a, const Runway& rwy)
{
    float rad = deg_to_rad(inbound_course(rwy));
    float lx  = cosf(rad);
    float ly  = sinf(rad);
    float rx  = a.x - rwy.x;
    float ry  = a.y - rwy.y;
    return rx * lx + ry * ly;
}

// ----------------------------------------------------------------
// Public API
// ----------------------------------------------------------------

bool canInterceptLocalizer(const Aircraft& a, const Runway& rwy)
{
    // Intercept angle = how far the aircraft heading differs from the inbound course.
    float course        = inbound_course(rwy);
    float intercept_ang = fabsf(angle_difference(course, a.heading_deg));

    // Aircraft must be on the APPROACH side: coming from behind the threshold.
    // along_track < 0 means aircraft is behind (upwind of) the runway -- correct.
    // along_track > 0 means aircraft has passed the runway -- wrong side.
    float along = along_track_km(a, rwy);

    return (intercept_ang <= 30.0f) && (along < 0.0f);
}

float getInterceptAngle(const Aircraft& a, const Runway& rwy)
{
    return fabsf(angle_difference(inbound_course(rwy), a.heading_deg));
}

float calculateLocalizerDeviation(const Aircraft& a, const Runway& rwy)
{
    float cross_km_val = cross_track_km(a, rwy);
    float dx           = rwy.x - a.x;
    float dy           = rwy.y - a.y;
    float dist_km      = sqrtf(dx*dx + dy*dy);

    if (dist_km < 0.05f) return 0.0f;

    // Angular deviation: asin(lateral_offset / slant_distance)
    float sin_val = std::clamp(cross_km_val / dist_km, -1.0f, 1.0f);
    return std::clamp(rad_to_deg(asinf(sin_val)), -20.0f, 20.0f);
}

float calculateGlideslopeDeviation(const Aircraft& a, const Runway& rwy)
{
    float dx       = rwy.x - a.x;
    float dy       = rwy.y - a.y;
    float dist_km  = sqrtf(dx*dx + dy*dy);
    float dist_nm  = dist_km * 0.539957f;

    // 3-degree glideslope: tan(3°)=0.05241, 1 NM=6076.12 ft
    float ideal_ft = dist_nm * 6076.12f * 0.05241f + 50.0f;
    return a.altitude_ft - ideal_ft; // positive=too high, negative=too low
}

// ----------------------------------------------------------------
// Main ILS update
// ----------------------------------------------------------------
void updateILSApproach(Aircraft& a, const Runway& rwy, ILSInfo& ils_info, float dt)
{
    // ---- Geometry ------------------------------------------------
    float dx       = rwy.x - a.x;
    float dy       = rwy.y - a.y;
    float dist_km  = sqrtf(dx*dx + dy*dy);
    float along    = along_track_km(a, rwy);
    float cross    = cross_track_km(a, rwy);
    float course   = inbound_course(rwy);

    ils_info.distance_to_runway   = dist_km;
    ils_info.localizer_deviation  = calculateLocalizerDeviation(a, rwy);
    ils_info.glideslope_deviation = calculateGlideslopeDeviation(a, rwy);

    // ---- Out-of-airspace explosion --------------------------------
    float dist_origin = sqrtf(a.x*a.x + a.y*a.y);
    if (dist_origin > 80.0f)
    {
        a.is_crashed  = true;
        a.crash_timer = 4.0f;
        a.crash_x     = a.x;
        a.crash_y     = a.y;
        a.setImmediateResponse("MAYDAY MAYDAY! Outside controlled airspace!", 4.0f);
        ils_info.ils_status = "LOST - OUTSIDE AIRSPACE";
        return;
    }

    // ---- Overshot runway (passed it) ------------------------------
    // along_track > 1.0 means aircraft is now in FRONT of the runway
    // (beyond the threshold), having come from behind.
    if (along > 1.0f)
    {
        a.is_crashed  = true;
        a.crash_timer = 4.0f;
        a.crash_x     = a.x;
        a.crash_y     = a.y;
        a.setImmediateResponse("MAYDAY! Runway overrun!", 4.0f);
        ils_info.ils_status = "OVERSHOT";
        return;
    }

    // ---- Successful landing ----------------------------------------
    if (dist_km < 0.4f && a.altitude_ft < 300.0f)
    {
        a.setImmediateResponse("Runway in sight, landing. Good day!", 4.0f);
        a.ils_active   = false;
        a.ils_runway   = "";
        a.is_crashed   = true;  // reuses removal — not counted as crash (has_landed=true)
        a.crash_timer  = 2.5f;
        a.crash_x      = a.x;
        a.crash_y      = a.y;
        ils_info.ils_status = "LANDED";
        ils_info.has_landed = true;
        return;
    }

    // ---- Localizer established check ------------------------------
    float hdg_error = fabsf(angle_difference(course, a.heading_deg));
    bool  aligned   = (hdg_error <= 8.0f);
    bool  centered  = (fabsf(ils_info.localizer_deviation) <= 1.5f);

    if (aligned && centered)
    {
        if (!ils_info.established_localizer)
        {
            ils_info.established_localizer = true;
            ils_info.turn_count            = 0;
            a.setImmediateResponse("Established localizer runway " + rwy.name + ".", 4.0f);
        }
    }
    else if (ils_info.established_localizer && fabsf(ils_info.localizer_deviation) > 3.0f)
    {
        ils_info.established_localizer  = false;
        ils_info.established_glideslope = false;
        a.setImmediateResponse("Localizer lost, going around!", 3.0f);
    }

    // ---- Glideslope established check -----------------------------
    if (ils_info.established_localizer && fabsf(ils_info.glideslope_deviation) <= 150.0f)
    {
        if (!ils_info.established_glideslope)
        {
            ils_info.established_glideslope = true;
            a.setImmediateResponse(
                "Established ILS runway " + rwy.name + ". Gear down, cleared to land.", 5.0f);
        }
    }
    else if (ils_info.established_glideslope && fabsf(ils_info.glideslope_deviation) > 500.0f)
    {
        ils_info.established_glideslope = false;
    }

    // ---- Heading guidance -----------------------------------------
    //
    // Three possible states:
    //
    // 1) FAR from centerline (|cross| > 2 km):
    //    Aim at a point on the extended centerline. One heading change.
    //
    // 2) CLOSE to centerline (|cross| <= 2 km), not yet established:
    //    Fly a 20-degree intercept cut. One heading change.
    //    cross > 0 means LEFT of course -> cut right (subtract from heading)
    //    cross < 0 means RIGHT of course -> cut left (add to heading)
    //
    // 3) ESTABLISHED:
    //    Proportional correction. No discrete turns.
    //
    // At most 3 heading targets total: far-aim -> cut -> roll-out.

    float desired_heading;

    if (!ils_info.established_localizer)
    {
        ils_info.is_intercepting = true;

        if (fabsf(cross) > 2.0f)
        {
            // State 1: aim at a point on the extended centerline behind the runway.
            // "Behind the runway" in inbound direction = subtract from runway position.
            float inbound_rad = deg_to_rad(course);
            float aim_dist    = std::min(dist_km * 0.65f, 10.0f);
            float aim_x       = rwy.x - cosf(inbound_rad) * aim_dist;
            float aim_y       = rwy.y - sinf(inbound_rad) * aim_dist;
            desired_heading   = bearing_toward(a.x, a.y, aim_x, aim_y);
            ils_info.ils_status = "INTERCEPTING - HEADING TO CENTERLINE";
        }
        else
        {
            // State 2: shallow 20-degree cut onto centerline.
            // cross > 0 = LEFT of course = must turn RIGHT = decrease heading (math CCW)
            float cut_dir   = (cross > 0.0f) ? -1.0f : 1.0f;
            desired_heading = wrap360(course + cut_dir * 20.0f);
            ils_info.ils_status = "INTERCEPTING - CUTTING ONTO LOCALIZER";
        }

        // Count as a new "turn" only if heading target changed meaningfully
        float change = fabsf(angle_difference(desired_heading, ils_info.last_intercept_heading));
        if (change > 8.0f)
        {
            ils_info.turn_count++;
            ils_info.last_intercept_heading = desired_heading;
        }

        ils_info.intercept_heading = desired_heading;
    }
    else
    {
        // State 3: established — proportional correction to stay centered.
        // cross > 0 = LEFT of course -> correct right -> subtract from heading
        // Gain: 4 deg per km, capped at ±15 deg
        float correction  = std::clamp(-cross * 4.0f, -15.0f, 15.0f);
        desired_heading   = wrap360(course + correction);
        ils_info.is_intercepting = false;

        if (ils_info.established_glideslope)
            ils_info.ils_status = "ON ILS - TRACKING";
        else
            ils_info.ils_status = "ON LOCALIZER - CAPTURING GLIDESLOPE";
    }

    // Write directly to target — bypasses command_delay so ILS
    // has immediate authority over the aircraft every frame.
    a.target_heading_deg  = desired_heading;
    a.pending_heading_deg = desired_heading;

    // ---- Altitude guidance ----------------------------------------
    if (ils_info.established_localizer)
    {
        // Follow 3-degree glideslope precisely
        float dist_nm  = dist_km * 0.539957f;
        float ideal_ft = dist_nm * 6076.12f * 0.05241f + 50.0f;
        float tgt_alt  = std::max(ideal_ft, 50.0f);

        a.target_altitude_ft  = tgt_alt;
        a.pending_altitude_ft = tgt_alt;
    }
    else if (dist_km < 25.0f)
    {
        // Not yet established but getting close: gentle descent toward capture altitude
        float dist_nm     = dist_km * 0.539957f;
        float capture_alt = dist_nm * 150.0f + 1200.0f;
        capture_alt       = std::clamp(capture_alt, 1200.0f, a.altitude_ft);

        a.target_altitude_ft  = capture_alt;
        a.pending_altitude_ft = capture_alt;
    }
}

// ----------------------------------------------------------------
// HUD status string
// ----------------------------------------------------------------
std::string getILSStatusMessage(const ILSInfo& info)
{
    std::ostringstream ss;
    ss << "[ILS] " << info.ils_status;

    ss << "  |  LOC: ";
    if (fabsf(info.localizer_deviation) < 0.5f)
        ss << "CENTERED";
    else if (info.localizer_deviation > 0.0f)
        ss << std::fixed << std::setprecision(1) << "L " << info.localizer_deviation << "deg";
    else
        ss << std::fixed << std::setprecision(1) << "R " << -info.localizer_deviation << "deg";

    ss << "  |  G/S: ";
    if (fabsf(info.glideslope_deviation) < 100.0f)
        ss << "ON PATH";
    else if (info.glideslope_deviation > 0.0f)
        ss << std::fixed << std::setprecision(0) << "HIGH +" << info.glideslope_deviation << "ft";
    else
        ss << std::fixed << std::setprecision(0) << "LOW " << info.glideslope_deviation << "ft";

    ss << "  |  DME: " << std::fixed << std::setprecision(1) << info.distance_to_runway << "km";

    if (info.is_intercepting)
        ss << "  |  Turn " << info.turn_count << "/3";

    return ss.str();
}