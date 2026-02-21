//
// Created by Natan on 12/14/2025.
//

#include "draw.h"
#include "runway.h"
#include "aircraft.h"
#include "utils.h"
#include <cmath>
#include <sstream>

void DrawRadarRings(ImDrawList* draw_list, float radar_range_km,
                    std::function<ImVec2(float,float)> world_to_screen,
                    ImU32 col_green, int rings)
{
    ImVec2 p0 = world_to_screen(0, 0);

    for (int i = 1; i <= rings; ++i)
    {
        float km_radius = (radar_range_km * i) / rings;
        ImVec2 p1 = world_to_screen(km_radius, 0);
        float screen_r = fabs(p1.x - p0.x);
        draw_list->AddCircle(p0, screen_r, col_green, 128, 1.0f);

        float range_nm = km_radius * 0.539957f;
        std::ostringstream ss;
        ss << (int)range_nm << " nm";
        draw_list->AddText(ImVec2(p0.x + screen_r - 30, p0.y - 12), col_green, ss.str().c_str());
    }
}

void DrawRunwaysAndILS(ImDrawList* draw_list, const std::vector<Runway>& runways,
                       std::function<ImVec2(float,float)> world_to_screen)
{
    ImU32 runway_color = IM_COL32(100, 150, 255, 255);
    ImU32 ils_color = IM_COL32(80, 120, 200, 200);
    ImU32 runway_line_color = IM_COL32(200, 200, 200, 255);

    for (const auto& rwy : runways)
    {
        // Calculate runway dimensions
        float runway_length_km = 6.0f;
        float runway_width_km = 0.09f;

        // Convert heading to radians
        float rwy_angle_rad = rwy.heading_deg * (3.14159265f / 180.0f);
        float rwy_perp_angle_rad = rwy_angle_rad + (3.14159265f / 2.0f);

        // Calculate half dimensions
        float half_length = runway_length_km / 2.0f;
        float half_width = runway_width_km / 2.0f;

        // Calculate the four corners of the runway
        ImVec2 corners[4];

        // Front left
        corners[0] = world_to_screen(
            rwy.x + cosf(rwy_angle_rad) * half_length + cosf(rwy_perp_angle_rad) * half_width,
            rwy.y + sinf(rwy_angle_rad) * half_length + sinf(rwy_perp_angle_rad) * half_width
        );

        // Front right
        corners[1] = world_to_screen(
            rwy.x + cosf(rwy_angle_rad) * half_length - cosf(rwy_perp_angle_rad) * half_width,
            rwy.y + sinf(rwy_angle_rad) * half_length - sinf(rwy_perp_angle_rad) * half_width
        );

        // Back right
        corners[2] = world_to_screen(
            rwy.x - cosf(rwy_angle_rad) * half_length - cosf(rwy_perp_angle_rad) * half_width,
            rwy.y - sinf(rwy_angle_rad) * half_length - sinf(rwy_perp_angle_rad) * half_width
        );

        // Back left
        corners[3] = world_to_screen(
            rwy.x - cosf(rwy_angle_rad) * half_length + cosf(rwy_perp_angle_rad) * half_width,
            rwy.y - sinf(rwy_angle_rad) * half_length + sinf(rwy_perp_angle_rad) * half_width
        );

        // Draw runway rectangle
        draw_list->AddQuadFilled(corners[0], corners[1], corners[2], corners[3], runway_color);
        draw_list->AddQuad(corners[0], corners[1], corners[2], corners[3], runway_line_color, 1.0f);

        // Draw runway name
        ImVec2 text_size = ImGui::CalcTextSize(rwy.name.c_str());
        ImVec2 text_pos = world_to_screen(rwy.x, rwy.y);
        draw_list->AddText(text_pos, IM_COL32(255, 255, 255, 255), rwy.name.c_str());

        // Draw ILS localizer - straight line, 30 km long
        float ils_angle = (rwy.heading_deg + 180.0f) * (3.14159265f / 180.0f); // Opposite direction
        float ils_len = 30.0f; // 30 km ILS

        // ILS starts at runway threshold
        ImVec2 ils_start = world_to_screen(
            rwy.x - cosf(rwy_angle_rad) * half_length,
            rwy.y - sinf(rwy_angle_rad) * half_length
        );

        // ILS end point
        ImVec2 ils_end = world_to_screen(
            rwy.x + cosf(ils_angle) * ils_len,
            rwy.y + sinf(ils_angle) * ils_len
        );

        // Draw the ILS line
        draw_list->AddLine(ils_start, ils_end, ils_color, 3.0f);
    }
}

void DrawWaypoints(ImDrawList* draw_list, const std::vector<Waypoint>& waypoints,
                   std::function<ImVec2(float,float)> world_to_screen)
{
    ImU32 waypoint_color = IM_COL32(255, 200, 100, 200);

    for (const auto& wp : waypoints)
    {
        ImVec2 wp_pos = world_to_screen(wp.x, wp.y);
        draw_list->AddCircle(wp_pos, 6.0f, waypoint_color, 12, 2.0f);
        draw_list->AddText(ImVec2(wp_pos.x + 10, wp_pos.y - 8), waypoint_color, wp.name.c_str());
    }
}

void DrawRadarSweep(ImDrawList* draw_list,
                    float radar_range_km,
                    float animation_speed,
                    std::function<ImVec2(float,float)> world_to_screen,
                    float &old_animation_speed)
{
    if (old_animation_speed == -1) old_animation_speed = animation_speed;
    float sweep_angle = fmodf(ImGui::GetTime() * 0.8f * animation_speed, 2.0f * IM_PI);
    float old_sweep_angle = fmodf(ImGui::GetTime() * 0.8f * animation_speed, 2.0f * IM_PI);
    float sweep_length_km = radar_range_km;

    float sx = cosf(sweep_angle) * sweep_length_km;
    float sy = sinf(sweep_angle) * sweep_length_km;

    ImVec2 sweep_center = world_to_screen(0.0f, 0.0f);
    ImVec2 sweep_tip = world_to_screen(sx, sy);

    ImU32 sweep_color = IM_COL32(120, 255, 120, 255);
    draw_list->AddLine(sweep_center, sweep_tip, sweep_color, 2.0f);
}

void DrawRadarAxes(ImDrawList* draw_list, float radar_range_km,
                   std::function<ImVec2(float,float)> world_to_screen, ImU32 col_green)
{
    float axis_len_km = radar_range_km;

    ImVec2 left  = world_to_screen(-axis_len_km, 0.0f);
    ImVec2 right = world_to_screen(+axis_len_km, 0.0f);
    draw_list->AddLine(left, right, col_green, 1.0f);

    ImVec2 down = world_to_screen(0.0f, -axis_len_km);
    ImVec2 up   = world_to_screen(0.0f, +axis_len_km);
    draw_list->AddLine(down, up, col_green, 1.0f);

    ImVec2 center = world_to_screen(0.0f, 0.0f);
    draw_list->AddCircleFilled(center, 3.0f, col_green);
}

void DrawCrashEffects(ImDrawList* draw_list,
                      const std::vector<Aircraft>& aircraft,
                      std::function<ImVec2(float, float)> world_to_screen)
{
    for (const auto& a : aircraft)
    {
        if (!a.is_crashed) continue;

        ImVec2 crash_pos = world_to_screen(a.crash_x, a.crash_y);

        float explosion_radius = 15.0f + sinf(ImGui::GetTime() * 10.0f) * 5.0f;

        // Explosion rings
        draw_list->AddCircle(crash_pos, explosion_radius, IM_COL32(255, 0, 0, 200), 32, 3.0f);
        draw_list->AddCircle(crash_pos, explosion_radius * 0.7f, IM_COL32(255, 100, 0, 180), 32, 2.0f);
        draw_list->AddCircle(crash_pos, explosion_radius * 0.4f, IM_COL32(255, 200, 0, 150), 32, 1.5f);

        // "CRASH!" text
        draw_list->AddText(ImVec2(crash_pos.x + 20, crash_pos.y - 20), IM_COL32(255, 50, 50, 255), "CRASH!");

        // Debris particles
        float time = ImGui::GetTime();
        for (int j = 0; j < 8; ++j)
        {
            float angle = time * 3.0f + j * (IM_PI / 4.0f);
            float dist = explosion_radius * 1.5f;
            ImVec2 particle_pos(
                crash_pos.x + cosf(angle) * dist,
                crash_pos.y + sinf(angle) * dist
            );
            draw_list->AddCircleFilled(particle_pos, 3.0f, IM_COL32(255, 150, 50, 200));
        }
    }
}

void DrawAircraft(ImDrawList* draw_list,
                  const std::vector<Aircraft>& aircraft,
                  const std::vector<std::pair<int,int>>& conflicts,
                  std::function<ImVec2(float,float)> world_to_screen,
                  int selected_index,
                  const ImVec2& win_pos,
                  const ImVec2& win_size,
                  float zoom_level)
{
    for (size_t i = 0; i < aircraft.size(); ++i)
    {
        const Aircraft& a = aircraft[i];
        if (a.is_crashed) continue;

        ImVec2 pos = world_to_screen(a.x, a.y);

        float margin = 50.0f;
        if (pos.x < win_pos.x - margin || pos.x > win_pos.x + win_size.x + margin ||
            pos.y < win_pos.y - margin || pos.y > win_pos.y + win_size.y + margin)
            continue;

        // Kolory
        ImU32 plane_color;
        if (a.emergency != EMERGENCY_NONE) plane_color = IM_COL32(255, 0, 0, 255);
        else if (a.is_overflight)          plane_color = IM_COL32(100, 100, 100, 255);
        else if (selected_index == (int)i) plane_color = IM_COL32(255, 255, 0, 255);
        else                               plane_color = IM_COL32(255, 255, 255, 255);

        float heading_rad = deg_to_rad(a.heading_deg);
        float cos_h = cosf(heading_rad);
        float sin_h = sinf(heading_rad);

        // Smooth autoscaling that adapts to zoom level
        float s = (selected_index == (int)i) ? 1.1f : 0.95f;

        if (zoom_level>1) s *= (1/zoom_level);

        // Ensure size stays within reasonable bounds using std::clamp
        s = std::clamp(s, 0.5f, 2.5f);

        auto transform = [&](float lx, float ly) -> ImVec2 {
            return ImVec2(
                pos.x + (lx * cos_h - ly * sin_h) * s,
                pos.y - (lx * sin_h + ly * cos_h) * s
            );
        };

        // ✔ W pełni symetryczna, wypukła sylwetka
        ImVec2 pts[] =
        {
            transform( 18,  0),   // Nose

            transform( 10,  3),   // Fuselage front right
            transform(  0,  3),   // Wing root right front
            transform(-10, 14),   // Wing tip right
            transform(-14, 14),
            transform(-6,   3),   // Wing root right back

            transform(-18,  3),   // Tail start right
            transform(-24,  6),   // Horizontal stabilizer right
            transform(-26,  0),   // Tail back center

            transform(-24, -6),   // Horizontal stabilizer left
            transform(-18, -3),   // Tail start left

            transform(-6,  -3),   // Wing root left back
            transform(-14,-14),
            transform(-10,-14),   // Wing tip left
            transform(  0,  -3),  // Wing root left front
            transform( 10,  -3)   // Fuselage front left
        };

        // Cienki outline dla kontrastu
        draw_list->AddPolyline(pts,
                               IM_ARRAYSIZE(pts),
                               plane_color,
                               ImDrawFlags_Closed,
                               1.0f);

        // Label
        std::string label = a.callsign + "\nFL" +
                            std::to_string((int)(a.altitude_ft / 100));

        draw_list->AddText(ImVec2(pos.x + 18, pos.y - 15),
                           IM_COL32_WHITE,
                           label.c_str());
    }
}

void DrawConflictLines(ImDrawList* draw_list,
                       const std::vector<Aircraft>& aircraft,
                       const std::vector<std::pair<int,int>>& conflicts,
                       std::function<ImVec2(float,float)> world_to_screen)
{
    for (const auto& cf : conflicts)
    {
        const Aircraft& a = aircraft[cf.first];
        const Aircraft& b = aircraft[cf.second];

        ImVec2 pa = world_to_screen(a.x, a.y);
        ImVec2 pb = world_to_screen(b.x, b.y);

        draw_list->AddLine(pa, pb, IM_COL32(255, 80, 80, 200), 2.0f);

        ImVec2 mid((pa.x + pb.x) * 0.5f, (pa.y + pb.y) * 0.5f);
        draw_list->AddText(ImVec2(mid.x + 4, mid.y + 4), IM_COL32(255, 120, 120, 220), "CONFLICT");
    }
}