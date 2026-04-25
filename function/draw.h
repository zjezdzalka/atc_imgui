//
// Created by Natan on 12/14/2025.
//

#ifndef IMGUI_RADAR_DRAW_H
#define IMGUI_RADAR_DRAW_H

#endif //IMGUI_RADAR_DRAW_H

#pragma once
#include <imgui.h>
#include <functional>
#include <vector>
#include <utility>
#include "runway.h"
#include "waypoint.h"
#include "aircraft.h"

void DrawRadarRings(ImDrawList* draw_list, float radar_range_km,
                    std::function<ImVec2(float,float)> world_to_screen,
                    ImU32 col_green, int rings = 4);

void DrawRunwaysAndILS(ImDrawList* draw_list, const std::vector<Runway>& runways,
                       std::function<ImVec2(float, float)> world_to_screen);

void DrawWaypoints(ImDrawList* draw_list, const std::vector<Waypoint>& waypoints,
                   std::function<ImVec2(float,float)> world_to_screen);

void DrawRadarSweep(ImDrawList* draw_list,
                    float radar_range_km,
                    float animation_speed,
                    std::function<ImVec2(float,float)> world_to_screen,
                    float& old_animation_speed);

void DrawRadarAxes(ImDrawList* draw_list, float radar_range_km,
                   std::function<ImVec2(float,float)> world_to_screen, ImU32 col_green);

void DrawCrashEffects(ImDrawList* draw_list,
                      const std::vector<Aircraft>& aircraft,
                      std::function<ImVec2(float, float)> world_to_screen);

void DrawAircraft(ImDrawList* draw_list,
                  const std::vector<Aircraft>& aircraft,
                  const std::vector<std::pair<int,int>>& conflicts,
                  std::function<ImVec2(float,float)> world_to_screen,
                  int selected_index,
                  const ImVec2& win_pos,
                  const ImVec2& win_size,
                  float zoom_level);

void DrawConflictLines(ImDrawList* draw_list,
                       const std::vector<Aircraft>& aircraft,
                       const std::vector<std::pair<int,int>>& conflicts,
                       std::function<ImVec2(float,float)> world_to_screen);
