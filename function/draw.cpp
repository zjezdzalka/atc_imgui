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
    ImU32 ils_color    = IM_COL32(80, 120, 200, 150);

    for (const auto& rwy : runways)
    {
        ImVec2 rwy_pos = world_to_screen(rwy.x, rwy.y);

        // Draw runway marker
        draw_list->AddText(ImVec2(rwy_pos.x + 12, rwy_pos.y - 8), runway_color, rwy.name.c_str());

        // Draw ILS localizer (6 km line in approach direction)
        float ils_angle = (rwy.heading_deg + 180.0f) * (3.14159265f / 180.0f); // Opposite direction
        float ils_len = 6.0f;
        ImVec2 ils_end = world_to_screen(
            rwy.x + cosf(ils_angle) * ils_len,
            rwy.y + sinf(ils_angle) * ils_len
        );
        draw_list->AddLine(ils_end, rwy_pos, ils_color, 2.0f);
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
                    std::function<ImVec2(float,float)> world_to_screen)
{
    float sweep_angle = fmodf(ImGui::GetTime() * 0.8f * animation_speed, 2.0f * IM_PI);
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
                  const ImVec2& win_size)
{
    for (size_t i = 0; i < aircraft.size(); ++i)
    {
        const Aircraft& a = aircraft[i];
        if (a.is_crashed) continue;

        ImVec2 pos = world_to_screen(a.x, a.y);

        float margin = 100.0f;
        bool on_screen = (pos.x >= win_pos.x - margin && pos.x <= win_pos.x + win_size.x + margin &&
                          pos.y >= win_pos.y - margin && pos.y <= win_pos.y + win_size.y + margin);
        if (!on_screen) continue;

        // Check conflict
        bool in_conflict = false;
        for (auto& cf : conflicts)
        {
            if ((int)i == cf.first || (int)i == cf.second)
            {
                in_conflict = true;
                break;
            }
        }

        // Determine blip color
        ImU32 blip_col;
        if (a.emergency != EMERGENCY_NONE)
            blip_col = IM_COL32(255, 0, 0, 255);
        else if (a.is_overflight)
            blip_col = IM_COL32(150, 150, 150, 180);
        else if (in_conflict)
            blip_col = IM_COL32(255, 64, 64, 255);
        else
            blip_col = IM_COL32(255, 255, 0, 255);

        float blip_radius = (selected_index == (int)i) ? 6.0f : 4.0f;
        draw_list->AddCircleFilled(pos, blip_radius, blip_col);

        // Heading line
        float head_rad = deg_to_rad(a.heading_deg);
        ImVec2 head_end(pos.x + cosf(head_rad) * 12.0f, pos.y - sinf(head_rad) * 12.0f);
        draw_list->AddLine(pos, head_end, IM_COL32(200, 200, 200, 180), 1.0f);

        // Text (callsign, FL, squawk)
        std::ostringstream ss;
        ss << a.callsign << "\n" << "FL" << (int)(a.altitude_ft / 100) << "\n" << a.squawk_code;
        draw_list->AddText(ImVec2(pos.x + 8.0f, pos.y - 10.0f),
                           a.is_overflight ? IM_COL32(150, 150, 150, 180) : IM_COL32(180, 240, 180, 220),
                           ss.str().c_str());
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