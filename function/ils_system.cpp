// ils_system.cpp

#include "ils_system.h"
#include "utils.h"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>

#ifndef IM_PI
#define IM_PI 3.14159265358979323846f
#endif

int planes_landed_count = 0;

static float wrap360(float a)
{
    return fmodf(a + 360.0f, 360.0f);
}

// Bearing FROM (ax,ay) TOWARD (bx,by)
static float bearing_toward(float ax, float ay, float bx, float by)
{
    float dx = bx - ax;
    float dy = by - ay;
    return wrap360(rad_to_deg(atan2f(dy, dx)));
}

// Inbound course: the heading the aircraft must fly to land.
// runway.h stores heading_deg as the direction the runway FACES (threshold to far end).
//   RWY 27: heading_deg=180 (faces West), aircraft land flying East  -> inbound = 0°  (math)
//   RWY 09: heading_deg=0   (faces East), aircraft land flying West  -> inbound = 180° (math)
// Aircraft approach from the OPPOSITE direction, so inbound = heading_deg + 180.
// Inbound course: the heading the aircraft must fly to land.
// A plane landing on RWY 09 flies East (0°). A plane landing on RWY 27 flies West (180°).
float inbound_course(const Runway& rwy)
{
    return wrap360(rwy.heading_deg);
}

// Distance from aircraft to runway
static float distance_to_runway(const Aircraft& a, const Runway& rwy)
{
    float dx = rwy.x - a.x;
    float dy = rwy.y - a.y;
    return sqrtf(dx*dx + dy*dy);
}

// Cross-track error: how far off the centerline (km)
static float cross_track_error(const Aircraft& a, const Runway& rwy)
{
    float course = inbound_course(rwy);
    float course_rad = deg_to_rad(course);

    // Direction vector of the approach path (from runway outward)
    float lx = cosf(course_rad);
    float ly = sinf(course_rad);

    // Vector from runway to aircraft
    float rx = rwy.x - a.x;
    float ry = rwy.y - a.y;

    // Perpendicular distance to the centerline
    return fabsf(rx * ly - ry * lx);
}

// Which side of the centerline (positive = left)
static float cross_track_signed(const Aircraft& a, const Runway& rwy)
{
    float course = inbound_course(rwy);
    float course_rad = deg_to_rad(course);

    float lx = cosf(course_rad);
    float ly = sinf(course_rad);

    float rx = a.x - rwy.x;
    float ry = a.y - rwy.y;

    return lx * ry - ly * rx;
}

// Check if aircraft is approaching from the correct side
static bool is_approaching_correctly(const Aircraft& a, const Runway& rwy)
{
    float course = inbound_course(rwy);
    float dir_to_rwy = bearing_toward(a.x, a.y, rwy.x, rwy.y);

    // The direction to the runway should be roughly opposite of where we want to go?
    // Actually, we want the aircraft to be generally pointing toward the runway area
    float heading_diff = fabsf(angle_difference(a.heading_deg, dir_to_rwy));

    // Aircraft should be pointing within 90° of the runway
    bool pointing_toward = (heading_diff <= 90.0f);

    // Also check if we're not past the runway
    float course_rad = deg_to_rad(course);
    float lx = cosf(course_rad);
    float ly = sinf(course_rad);
    float rx = a.x - rwy.x;
    float ry = a.y - rwy.y;
    float along = rx * lx + ry * ly;

    // along > 0: aircraft is displaced in the inbound direction from the runway
    // = aircraft is on the correct approach side (not yet past the threshold).
    bool in_front = (along > 0.0f);

    return pointing_toward && in_front;
}

// ----------------------------------------------------------------
// Public API
// ----------------------------------------------------------------

bool canInterceptLocalizer(const Aircraft& a, const Runway& rwy)
{
    float course = inbound_course(rwy);
    float intercept_ang = fabsf(angle_difference(course, a.heading_deg));

    // Must be strictly within 30° of the correct approach heading
    if (intercept_ang >= 30.0f)
        return false;

    // Must be within 50km
    float dist = distance_to_runway(a, rwy);
    if (dist > 50.0f)
        return false;

    // Project (aircraft - runway) onto the inbound course unit vector.
    // along > 0: aircraft is ahead of the runway in the inbound direction
    //           = on the correct approach side (hasn't passed the threshold).
    // along <= 0: aircraft is behind the threshold (wrong side) -> reject.
    float course_rad = deg_to_rad(course);

    float lx = cosf(course_rad);
    float ly = sinf(course_rad);
    float rx = rwy.x - a.x;
    float ry = rwy.y - a.y;
    float along = rx * lx + ry * ly;

    if (along <= 0.0f)
        return false;

    float heading_deg = rwy.heading_deg;
    double heading_rad = heading_deg * (M_PI / 180.0);

    double A = tan(heading_rad);
    double B = -1;
    double C = rwy.y - (A * rwy.x);

    double D = -1.0f/(A);
    double E = -1;
    double F = a.y - (D * a.x);

    double intersectH_ILS_W = A*E-B*D;
    double intersectH_ILS_x = (C*E-F*B)/intersectH_ILS_W;
    double intersectH_ILS_y = (A*F-D*C)/intersectH_ILS_W;

    double dx, dy;
    dx = fabs(intersectH_ILS_x - rwy.x);
    dy = fabs(intersectH_ILS_y - rwy.y);

    double distance_intersect_rwy = sqrt(dx*dx + dy*dy);
    double max_distance_aircraft_ILS = distance_intersect_rwy*sqrt(3)/3;
    double aircraft_distance_ILS = fabs(A*a.x + B*a.y + C)/sqrt(A*A + B*B);

    if (aircraft_distance_ILS > max_distance_aircraft_ILS) return false;

    /*
    mega lagorytm generalnie
    wyliczamy wzór ogólny runway i ILS
    Ax+By+C = 0
    A = tg(nachylenie%180)

    następnie wyliczyć takie B i C żeby przechodziło przez rwy.x i rwy.y
    czyli wsumie przyjąć B jako -1, a następnie obliczyć różnicę |y - Ax| jako wartość C

    wyliczamy linię prostopadłą do ILS
    Dx+Ey+F = 0;
    D = -ctg(nachylenie%180)
    I dobrać takie E i F żeby przechodziło przez pozycję Aircraft
    czyli wsumie przyjąć E jako -1, a następnie obliczyć różnicę |y - Dx| jako wartość F

    Następnie trójkąt 30,60,90. Dystans przecięcia linii prostopadłej z ILS od genezy runway.
    czyli Ax + By + C = Dx + Ey + F
    ax + c = dx + f

    i to jest a pierwiastków z 3.
    Zatem maksymalny dystans jaki samolot może być od ILS to a (zatem trzeba wyliczyć odległość * pierwiastek z 3 przez 3)
    No i sprawdzić dystans samolotu od ILS i zobaczyć czy przykracza maksymalną, jeśli tak return false, else true.
    czyli wzór d = |Ax0 + By0 + C|/sqrt(A^2+B^2), gdzie x0 to a.x, a y0 to b.y
    */
    return true;

}

float getInterceptAngle(const Aircraft& a, const Runway& rwy)
{
    return fabsf(angle_difference(inbound_course(rwy), a.heading_deg));
}

float calculateLocalizerDeviation(const Aircraft& a, const Runway& rwy)
{
    float cross = cross_track_signed(a, rwy);
    float dist = distance_to_runway(a, rwy);

    if (dist < 0.1f) return 0.0f;

    // Convert to degrees
    float deviation = rad_to_deg(atan2f(cross, dist));
    return std::clamp(deviation, -20.0f, 20.0f);
}

float calculateGlideslopeDeviation(const Aircraft& a, const Runway& rwy)
{
    float dist_km = distance_to_runway(a, rwy);
    float dist_nm = dist_km * 0.539957f;

    // 3-degree glideslope: tan(3°) * distance in ft
    float ideal_ft = dist_nm * 6076.12f * 0.05241f + 50.0f;
    return a.altitude_ft - ideal_ft;
}

// ----------------------------------------------------------------
// Main ILS update
// ----------------------------------------------------------------
// ----------------------------------------------------------------
// Main ILS update
// ----------------------------------------------------------------
// ----------------------------------------------------------------
// Main ILS update
// ----------------------------------------------------------------
void updateILSApproach(Aircraft& a, const Runway& rwy, ILSInfo& ils_info, float dt)
{
    float dist_km = distance_to_runway(a, rwy);
    float course = inbound_course(rwy);

    ils_info.distance_to_runway = dist_km;
    ils_info.localizer_deviation = calculateLocalizerDeviation(a, rwy);
    ils_info.glideslope_deviation = calculateGlideslopeDeviation(a, rwy);

    // ---- Out of airspace -----------------------------------------
    float dist_origin = sqrtf(a.x*a.x + a.y*a.y);
    if (dist_origin > 120.0f)
    {
        a.is_crashed = true;
        a.crash_timer = 4.0f;
        a.crash_x = a.x;
        a.crash_y = a.y;
        a.setImmediateResponse("MAYDAY! Outside controlled airspace!", 4.0f);
        ils_info.ils_status = "LOST";
        return;
    }

    // ---- Check if we're even on the correct side -----------------
    float dir_to_rwy = bearing_toward(a.x, a.y, rwy.x, rwy.y);
    float heading_diff = fabsf(angle_difference(course, a.heading_deg));

    if (heading_diff > 90.0f && dist_km > 10.0f)
    {
        a.target_heading_deg = course;
        ils_info.ils_status = "TURNING TO APPROACH HEADING";
        ils_info.is_intercepting = true;
        return;
    }

    // ---- Localizer established check -----------------------------
    bool aligned = (heading_diff <= 25.0f);
    bool centered = (fabsf(ils_info.localizer_deviation) <= 4.0f);

    if (aligned && centered && dist_km < 30.0f)
    {
        if (!ils_info.established_localizer)
        {
            ils_info.established_localizer = true;
            a.setImmediateResponse("Established localizer runway " + rwy.name, 4.0f);
        }
    }
    else if (ils_info.established_localizer && fabsf(ils_info.localizer_deviation) > 8.0f)
    {
        ils_info.established_localizer = false;
        ils_info.established_glideslope = false;
    }

    // ---- Glideslope established check ----------------------------
    if (ils_info.established_localizer && fabsf(ils_info.glideslope_deviation) <= 1000.0f && dist_km < 25.0f)
    {
        if (!ils_info.established_glideslope) {
            ils_info.established_glideslope = true;
            a.setImmediateResponse("Glideslope captured, following beam.", 4.0f);
        }
    }

    // ---- Heading guidance ----------------------------------------
    float desired_heading;

    if (!ils_info.established_localizer) {
        desired_heading = wrap360(course - ils_info.localizer_deviation * 4.0f);
        ils_info.ils_status = "INTERCEPTING - TARGET " + std::to_string((int)desired_heading) + "°";
    } else {
        float correction = std::clamp(-ils_info.localizer_deviation * 5.0f, -15.0f, 15.0f);
        desired_heading = wrap360(course + correction);

        if (ils_info.established_glideslope)
            ils_info.ils_status = "ON ILS - TRACKING";
        else
            ils_info.ils_status = "ON LOCALIZER - DESCENDING";
    }

    // Limit turn rate to realistic degrees per second
    float current_hdg = a.target_heading_deg;
    float hdg_change = angle_difference(desired_heading, current_hdg);
    float max_turn = 25.0f * dt;

    if (fabsf(hdg_change) > max_turn)
    {
        if (hdg_change > 0)
            desired_heading = wrap360(current_hdg + max_turn);
        else
            desired_heading = wrap360(current_hdg - max_turn);
    }

    a.target_heading_deg = desired_heading;

    // ---- Speed guidance (Auto-throttle for approach) -------------
    if (dist_km > 25.0f) {
        if (a.target_speed_kts > 220.0f) a.target_speed_kts = 220.0f;
    } else if (dist_km > 15.0f) {
        if (a.target_speed_kts > 180.0f) a.target_speed_kts = 180.0f;
    } else if (dist_km > 8.0f) {
        if (a.target_speed_kts > 160.0f) a.target_speed_kts = 160.0f;
    } else {
        if (a.target_speed_kts > 135.0f) a.target_speed_kts = 135.0f; // Final approach speed
    }

    // ---- Altitude guidance (Glideslope) --------------------------
    float dist_nm = dist_km * 0.539957f;
    float ideal_ft = dist_nm * 6076.12f * 0.05241f + 50.0f;

    // The plane immediately uses the glideslope calculation as its target.
    // We only apply it if the glideslope is LOWER than the plane's current
    // target altitude, so it catches the slope from below or rides it down,
    // rather than climbing if it's already low.
    if (ideal_ft < a.target_altitude_ft) {
        a.target_altitude_ft = std::max(ideal_ft, 0.0f);
    }

    // ---- Touchdown Logic -----------------------------------------
    if (dist_km < 1.2f && a.altitude_ft < 100.0f)
    {
        if (!a.is_on_ground) {
            a.setImmediateResponse("Touchdown! Welcome to the airport.", 3.0f);

            // Stop the plane and set landing state
            a.is_on_ground = true;
            a.landing_timer = 3.0f; // Stay on runway for 3 seconds

            // Disable flight systems
            a.ils_active = false;
            a.speed_kts = 0;
            a.target_speed_kts = 0;
            a.altitude_ft = 0;
            a.target_altitude_ft = 0;

            // Increment the landing counter
            planes_landed_count++;

            ils_info.ils_status = "LANDED";
            ils_info.has_landed = true;
        }
        return;
    }
}

// ----------------------------------------------------------------
// HUD status string
// ----------------------------------------------------------------
std::string getILSStatusMessage(const ILSInfo& info)
{
    std::ostringstream ss;
    ss << "[ILS] " << info.ils_status;

    ss << "  LOC: ";
    if (fabsf(info.localizer_deviation) < 0.5f)
        ss << "CENTERED";
    else if (info.localizer_deviation > 0.0f)
        ss << "L " << std::fixed << std::setprecision(1) << info.localizer_deviation << "°";
    else
        ss << "R " << std::fixed << std::setprecision(1) << -info.localizer_deviation << "°";

    ss << "  G/S: ";
    if (fabsf(info.glideslope_deviation) < 100.0f)
        ss << "ON PATH";
    else if (info.glideslope_deviation > 0.0f)
        ss << "HIGH +" << (int)info.glideslope_deviation << "ft";
    else
        ss << "LOW " << (int)info.glideslope_deviation << "ft";

    ss << "  DME: " << std::fixed << std::setprecision(1) << info.distance_to_runway << "km";

    return ss.str();
}