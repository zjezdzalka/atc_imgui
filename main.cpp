#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "function/glfw.cpp"
#include "function/aircraft.h"
#include "function/emergency.h"
#include "function/utils.h"
#include "function/runway.h"
#include "function/waypoint.h"
#include "function/draw.h"

#include <GLFW/glfw3.h>
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <sstream>
#include <algorithm>
#include <vector>
#include <string>
#include <iomanip>
#include <cfloat>

using namespace std;

int main(int, char**)
{
    // generation seed randomized
    srand((unsigned)time(nullptr));

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) return 1;

    #if __APPLE__
        const char* glsl_version = "#version 150";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    #else
        const char* glsl_version = "#version 130";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    #endif

    // glfw init

    GLFWwindow* window = glfwCreateWindow(1280, 768, "Air Traffic Controller", nullptr, nullptr);
    if (window == nullptr) return 1;

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // imgui init

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Data init
    std::vector<Runway> runways = createRunways();
    std::vector<Waypoint> waypoints = createWaypoints();

    // Aircraft generation
    const int initial_count = 12;
    std::vector<Aircraft> aircraft = generateInitialAircraft(initial_count, radar_range_km);
    int selected_index = -1;

    // Basic information
    const float ALT_MAX = 23000.0f;
    const float ALT_MIN = 1600.0f;
    const float SPD_MIN = 80.0f;
    const float SPD_MAX = 300.0f;
    float wind_heading = (float)(rand() % 360); // 0 - 360deg
    float wind_speed_kts = 5.0f + (rand() % 25); // 5-30 kts

    // Camera data
    float camera_x = 0.0f;
    float camera_y = 0.0f;
    float zoom_level = 1.0f;
    bool is_panning = false;
    ImVec2 last_mouse_pos;
    double last_time = glfwGetTime();
    float animation_speed = 1.0f; // Time multiplier

    const float right_panel_width = 450.0f;

    // Random emergency generation
    float random_emergency_timer = 0.0f;
    const float random_emergency_interval = 60.0f; // Emergency chance every 60 seconds
    int total_crash_count = 0;

    // Check if window not closed or game isn't finished.
    while (!glfwWindowShouldClose(window))
    {
        double now = glfwGetTime();
        float dt = (float)(now - last_time) * animation_speed;
        if (dt <= 0.0f) dt = 1.0f / 60.0f; // delta time
        last_time = now;

        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glfwPollEvents();

        // Random emergencies during game
        GenerateRandomEmergency(aircraft, dt, random_emergency_timer, random_emergency_interval);

        // Update every aircraft
        for (size_t i = 0; i < aircraft.size(); ++i)
        {
            auto& a = aircraft[i];

            const float max_alt_rate = 40.0f; // ft/s (~2000 fpm)
            const float alt_accel = 4.0f; // ft/s²
            const float max_speed_rate = 3.0f;
            const float speed_accel = 0.5f;

            // If plane is crashed
            if (a.is_crashed)
            {
                a.crash_timer -= dt;
                if (a.crash_timer <= 0.0f)
                {
                    // Delete plane from existance
                    aircraft.erase(aircraft.begin() + i);
                    i--;

                    if (selected_index == (int)i)
                    {
                        selected_index = -1;
                    }
                    else if (selected_index > (int)i)
                    {
                        selected_index--;
                    }
                }
                continue; // Do not iterate over crashed aircraft
            }

            if (a.response_timer > 0.0f)
                a.response_timer -= dt; // decrease wait until response

            // Update emergency timer
            if (a.emergency != EMERGENCY_NONE && !a.is_overflight)
            {
                a.emergency_timer -= dt;
                if (a.emergency_timer <= 0.0f)
                {
                    a.is_crashed = true;
                    a.crash_timer = 5.0f; // Crash time
                    a.crash_x = a.x;
                    a.crash_y = a.y;
                    total_crash_count++;
                    continue; // Do not update aircraft
                }
            }

            if (a.command_delay > 0.0f)
            {
                a.command_delay -= dt;
                if (a.command_delay <= 0.0f)
                {
                    if (a.has_pending_command)
                    {
                        // Reset values - no change in aircraft vals
                        a.target_altitude_ft = a.pending_altitude_ft;
                        a.target_heading_deg = a.pending_heading_deg;
                        a.target_speed_kts = a.pending_speed_kts;

                        a.has_pending_command = false;
                    }
                }
            }

            // check altitude diff
            float alt_error = a.target_altitude_ft - a.altitude_ft;
            if (fabs(alt_error) < 0.5f) // close enough to set
            {
                a.altitude_ft = a.target_altitude_ft;
                a.altitude_rate_fps = 0.0f;
            }
            else
            {
                // Desired vertical speed using braking distance logic
                float stopping_speed = sqrtf(2.0f * alt_accel * fabs(alt_error));
                float desired_rate =
                    std::clamp(stopping_speed, 0.0f, max_alt_rate) *
                    (alt_error > 0.0f ? 1.0f : -1.0f);

                // Accelerate vertical speed toward desired rate
                float rate_error = desired_rate - a.altitude_rate_fps;
                float max_delta = alt_accel * dt;

                a.altitude_rate_fps += std::clamp(rate_error, -max_delta, max_delta);
                float new_altitude = a.altitude_ft + a.altitude_rate_fps * dt;

                // Prevent overshoot if target moved this frame
                if ((alt_error > 0.0f && new_altitude > a.target_altitude_ft) ||
                    (alt_error < 0.0f && new_altitude < a.target_altitude_ft))
                {
                    a.altitude_ft = a.target_altitude_ft;
                    a.altitude_rate_fps = 0.0f;
                }
                else
                {
                    a.altitude_ft = new_altitude;
                }
            }

            // Heading transition
            float heading_diff = angle_difference(a.target_heading_deg, a.heading_deg);
            if (fabs(heading_diff) > 0.5f)
            {
                float turn_rate = 5.0f * dt;
                if (heading_diff > 0)
                {
                    a.heading_deg += std::min(turn_rate, heading_diff);
                }
                else
                {
                    a.heading_deg += std::max(-turn_rate, heading_diff);
                }

                a.heading_deg = fmodf(a.heading_deg + 360.0f, 360.0f);
            }
            else
            {
                a.heading_deg = a.target_heading_deg;
            }

            // Speed transition
            float speed_diff = a.target_speed_kts - a.speed_kts;
            if (fabs(speed_diff) > 0.5f)
            {
                float desired_rate = 0.0f;
                float distance_factor = sqrtf(2.0f * speed_accel * fabs(speed_diff));

                if (speed_diff > 0)
                {
                    desired_rate = std::min(max_speed_rate, distance_factor);
                }
                else
                {
                    desired_rate = -std::min(max_speed_rate, distance_factor);
                }

                float rate_diff = desired_rate - a.speed_rate_kps;
                if (fabs(rate_diff) > 0.05f)
                {
                    float accel_step = speed_accel * dt;
                    if (rate_diff > 0)
                    {
                        a.speed_rate_kps += std::min(accel_step, rate_diff);
                    }
                    else
                    {
                        a.speed_rate_kps += std::max(-accel_step, rate_diff);
                    }
                }
                else
                {
                    a.speed_rate_kps = desired_rate;
                }

                float new_speed = a.speed_kts + a.speed_rate_kps * dt;

                if ((speed_diff > 0 && new_speed >= a.target_speed_kts) ||
                    (speed_diff < 0 && new_speed <= a.target_speed_kts))
                {
                    a.speed_kts = a.target_speed_kts;
                    a.speed_rate_kps = 0.0f;
                }
                else
                {
                    a.speed_kts = new_speed;
                }
            }
            else
            {
                a.speed_kts = a.target_speed_kts;
                a.speed_rate_kps = 0.0f;
            }

            // Position update
            float speed_kmh = a.speed_kts * 1.852f;
            float speed_kms = speed_kmh / 3600.0f;
            float ang = deg_to_rad(a.heading_deg);
            a.x += cosf(ang) * speed_kms * dt;
            a.y += sinf(ang) * speed_kms * dt;
        }

        // Conflict detection
        const float conflict_horiz_km = 5.0f;
        const float conflict_horiz_km2 = conflict_horiz_km * conflict_horiz_km;
        const float conflict_vert_ft = 1000.0f;

        std::vector<std::pair<int, int>> conflicts;
        for (size_t i = 0; i < aircraft.size(); ++i)
        {
            for (size_t j = i + 1; j < aircraft.size(); ++j)
            {
                if (aircraft[i].is_crashed || aircraft[j].is_crashed)
                {
                    continue; // Crashed planes do not cause conflicts
                }

                float dx = aircraft[i].x - aircraft[j].x;
                float dy = aircraft[i].y - aircraft[j].y;
                float d2 = dx * dx + dy * dy;
                float dz = abs(aircraft[i].altitude_ft - aircraft[j].altitude_ft);
                if (d2 <= conflict_horiz_km2 && dz <= conflict_vert_ft)
                {
                    conflicts.emplace_back((int)i, (int)j);
                }
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        float radar_width = display_w - right_panel_width;
        float radar_height = display_h;

        ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings;

        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(radar_width, radar_height), ImGuiCond_Always);

        ImGui::Begin("Radar", nullptr, window_flags);

        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 win_pos = ImGui::GetWindowPos();
        ImVec2 win_size = ImGui::GetWindowSize();
        ImVec2 center(win_pos.x + win_size.x * 0.5f, win_pos.y + win_size.y * 0.5f);
        float radius_max = (win_size.x < win_size.y ? win_size.x : win_size.y) * 0.45f;

        ImVec2 mouse_pos = ImGui::GetMousePos();
        bool is_radar_hovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows);

        if (is_radar_hovered)
        {
            float wheel = io.MouseWheel;
            if (wheel != 0.0f)
            {
                float zoom_factor = 1.0f - wheel * 0.15f;
                float new_zoom = zoom_level * zoom_factor;
                new_zoom = std::max(0.2f, std::min(5.0f, new_zoom));

                ImVec2 mouse_rel(mouse_pos.x - center.x, mouse_pos.y - center.y);
                float scale_before = radius_max / (radar_range_km * zoom_level);
                float scale_after = radius_max / (radar_range_km * new_zoom);

                camera_x += mouse_rel.x / scale_before - mouse_rel.x / scale_after;
                camera_y -= mouse_rel.y / scale_before - mouse_rel.y / scale_after;

                zoom_level = new_zoom;
            }
        }

        bool shift_held = io.KeyShift;
        bool should_pan = is_radar_hovered && (
            (ImGui::IsMouseDown(0) && shift_held) ||
            ImGui::IsMouseDown(2) ||
            ImGui::IsMouseDown(1)
        );

        // Check if should be panned
        if (should_pan)
        {
            if (!is_panning)
            {
                is_panning = true;
                last_mouse_pos = mouse_pos;
            }
            else
            {
                ImVec2 delta(mouse_pos.x - last_mouse_pos.x, mouse_pos.y - last_mouse_pos.y);
                float scale = radius_max / (radar_range_km * zoom_level);
                camera_x -= delta.x / scale;
                camera_y += delta.y / scale;
                last_mouse_pos = mouse_pos;
            }
        }
        else
        {
            is_panning = false;
        }

        ImU32 col_green = IM_COL32(0, 255, 0, 100);
        ImU32 bg = IM_COL32(0, 15, 0, 200);
        draw_list->AddRectFilled(win_pos, ImVec2(win_pos.x + win_size.x, win_pos.y + win_size.y), bg, 8.0f);

        auto world_to_screen = [&](float wx, float wy)
        {
            float scale = radius_max / (radar_range_km * zoom_level);
            float vx = wx - camera_x;
            float vy = wy - camera_y;
            return ImVec2(center.x + vx * scale, center.y - vy * scale);
        };

        // Draw rings
        DrawRadarRings(draw_list, radar_range_km, world_to_screen, col_green);

        // Draw runways and ILS
        DrawRunwaysAndILS(draw_list, runways, world_to_screen);

        // Draw waypoints
        DrawWaypoints(draw_list, waypoints, world_to_screen);

        // Radar sweep
        DrawRadarSweep(draw_list, radar_range_km, animation_speed, world_to_screen);

        // Draw axes
        DrawRadarAxes(draw_list, radar_range_km, world_to_screen, col_green);

        // Draw crash effects
        DrawCrashEffects(draw_list, aircraft, world_to_screen);

        // Draw aircraft
        DrawAircraft(draw_list, aircraft, conflicts, world_to_screen, selected_index, win_pos, win_size);

        // Draw conflict lines
        DrawConflictLines(draw_list, aircraft, conflicts, world_to_screen);

        // Aircraft selection
        ImVec2 io_mouse = ImGui::GetMousePos();
        bool clicked = ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows) && ImGui::IsMouseClicked(0) && !
            shift_held;
        if (clicked)
        {
            float best_d = 999999.0f;
            int best_idx = -1;
            for (size_t i = 0; i < aircraft.size(); ++i)
            {
                if (aircraft[i].is_overflight || aircraft[i].is_crashed) continue;
                // Can't select overflights or crashed

                ImVec2 pos = world_to_screen(aircraft[i].x, aircraft[i].y);
                float dx = io_mouse.x - pos.x;
                float dy = io_mouse.y - pos.y;
                float d2 = dx * dx + dy * dy;
                if (d2 < best_d && d2 < (20.0f * 20.0f))
                {
                    best_d = d2;
                    best_idx = (int)i;
                }
            }
            if (best_idx >= 0)
            {
                if (selected_index >= 0) aircraft[selected_index].selected = false;
                selected_index = best_idx;
                aircraft[selected_index].selected = true;
            }
            else
            {
                if (selected_index >= 0)
                {
                    aircraft[selected_index].selected = false;
                    selected_index = -1;
                }
            }
        }

        ImGui::End();

        // RIGHT PANEL
        ImGuiWindowFlags panel_flags = ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse;

        ImGui::SetNextWindowPos(ImVec2(radar_width, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(right_panel_width, (float)display_h), ImGuiCond_Always);

        ImGui::Begin("RightPanel", nullptr, panel_flags);

        // SIMULATION CONTROL
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "SIMULATION CONTROL");
        ImGui::Separator();

        if (ImGui::Button("Spawn Random Aircraft", ImVec2(-1, 0)))
        {
            Aircraft a;
            generateAircraft(aircraft, a, radar_range_km, aircraft.size());
            aircraft.push_back(a);
        }

        ImGui::Text("Aircraft: %zu", aircraft.size());
        ImGui::Text("Conflicts: %zu", conflicts.size());

        // Count crashes
        ImGui::Text("Crashes: %d", total_crash_count);

        ImGui::Dummy(ImVec2(0, 5));
        ImGui::Text("Animation Speed:");
        ImGui::SliderFloat("##speed", &animation_speed, 0.1f, 5.0f, "%.1fx");

        ImGui::Dummy(ImVec2(0, 5));
        ImGui::Text("Wind: %03.0f° at %.0f kts", wind_heading, wind_speed_kts);

        ImGui::Dummy(ImVec2(0, 10));
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "CAMERA CONTROLS");
        ImGui::Separator();

        ImGui::BulletText("Two-finger scroll: Zoom");
        ImGui::BulletText("Shift + Drag: Pan");
        ImGui::BulletText("Right Click Drag: Pan");
        ImGui::Text("Zoom: %.1fx", 1.0f / zoom_level);
        ImGui::Text("Offset: (%.1f, %.1f) km", camera_x, camera_y);

        if (ImGui::Button("Reset View", ImVec2(-1, 0)))
        {
            camera_x = 0.0f;
            camera_y = 0.0f;
            zoom_level = 1.0f;
        }

        ImGui::Dummy(ImVec2(0, 20));

        // AIRCRAFT CONTROL
        if (selected_index >= 0 && selected_index < (int)aircraft.size())
        {
            Aircraft& sel = aircraft[selected_index];

            if (sel.is_crashed)
            {
                ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "PLANE CRASHED");
                ImGui::Text("This plane crashed.");
                ImGui::Text("Explosion will vanish in %.1f seconds", sel.crash_timer);
                ImGui::End();
                goto render_end;
            }

            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "AIRCRAFT CONTROL");
            ImGui::Separator();

            ImGui::SetWindowFontScale(2.0f);
            ImGui::Text("Callsign %s", sel.callsign.c_str());
            ImGui::SetWindowFontScale(1.0f);

            ImGui::Text("Squawk: %s", sel.squawk_code.c_str());

            // Emergency display
            if (sel.emergency != EMERGENCY_NONE)
            {
                ::DrawEmergencyPanel(sel);
            }

            // Show aircraft response
            if (sel.response_timer > 0.0f)
            {
                ImGui::Dummy(ImVec2(0, 5));
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.3f, 1.0f, 0.3f, 1.0f));
                ImGui::TextWrapped("[%s]: %s", sel.callsign.c_str(), sel.last_response.c_str());
                ImGui::PopStyleColor();

                if (sel.command_delay > 0.0f)
                {
                    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.3f, 1.0f),
                                       "(executing in %.1fs...)", sel.command_delay);
                }
            }

            ImGui::Dummy(ImVec2(0, 10));
            ImGui::Separator();

            // ILS Selection
            ImGui::Text("ILS Approach:");
            if (sel.ils_active)
            {
                ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f),
                                   "Active: RWY %s", sel.ils_runway.c_str());

                // Calculate glideslope deviation
                for (const auto& rwy : runways)
                {
                    if (rwy.name == sel.ils_runway)
                    {
                        float dist_to_rwy = sqrtf((sel.x - rwy.x) * (sel.x - rwy.x) +
                            (sel.y - rwy.y) * (sel.y - rwy.y));
                        float expected_alt_ft = dist_to_rwy * 1000.0f * tanf(deg_to_rad(3.0f)) * 3.28084f;
                        float deviation = sel.altitude_ft - expected_alt_ft;

                        if (deviation > 200.0f)
                        {
                            ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "TOO HIGH (%.0f ft)", deviation);
                        }
                        else if (deviation < -200.0f)
                        {
                            ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "TOO LOW (%.0f ft)", -deviation);
                        }
                        else
                        {
                            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "ON GLIDESLOPE");
                        }
                        break;
                    }
                }

                if (ImGui::Button("Disable ILS", ImVec2(-1, 0)))
                {
                    sel.ils_active = false;
                    sel.ils_runway = "";
                    sel.setCommand("ILS approach canceled");
                }
            }
            else
            {
                for (const auto& rwy : runways)
                {
                    std::string btn_label = "ILS RWY " + rwy.name;
                    if (ImGui::Button(btn_label.c_str(), ImVec2(-1, 0)))
                    {
                        sel.ils_active = true;
                        sel.ils_runway = rwy.name;
                        std::ostringstream r;
                        r << "Roger, cleared ILS approach runway " << rwy.name;
                        sel.setCommand(r.str());
                    }
                }
            }

            ImGui::Separator();

            // --- Altitude Controls ---
            ImGui::Text("Altitude: %.0f ft", sel.altitude_ft);

            if (fabs(sel.target_altitude_ft - sel.altitude_ft) > 5.0f)
            {
                ImGui::SameLine();
                ImGui::TextColored(
                    ImVec4(0.5f, 0.8f, 1.0f, 1.0f),
                    "-> %.0f ft",
                    sel.target_altitude_ft
                );
            }

            if (ImGui::Button("-1000##alt1", ImVec2(100, 0)))
            {
                float old_target = sel.target_altitude_ft;

                float requested = old_target - 1000.0f;
                float new_target = std::max(ALT_MIN, requested);

                float diff = old_target - new_target;

                if (diff <= 0.0f)
                {
                    sel.setImmediateResponse("Minimum altitude is 1600 ft", 5.0f);
                }
                else
                {
                    sel.pending_altitude_ft = new_target;
                    sel.has_pending_command = true;

                    std::ostringstream r;
                    if (requested < ALT_MIN)
                    {
                        r << "Descending " << (int)diff << " ft to minimum altitude of 1600 ft";
                    }
                    else
                    {
                        r << "Roger, descending " << (int)diff << " ft";
                    }
                    sel.setCommand(r.str(), 5.0f);
                    sel.command_delay = 5.0f;
                }
            }
            ImGui::SameLine();

            if (ImGui::Button("-100##alt2", ImVec2(100, 0)))
            {
                float old_target = sel.target_altitude_ft;

                float requested = old_target - 100.0f;
                float new_target = std::max(ALT_MIN, requested);

                float diff = old_target - new_target;

                if (diff <= 0.0f)
                {
                    sel.setImmediateResponse("Minimum altitude is 1600 ft", 5.0f);
                }
                else
                {
                    sel.pending_altitude_ft = new_target;
                    sel.has_pending_command = true;

                    std::ostringstream r;
                    if (requested < ALT_MIN)
                    {
                        r << "Descending " << (int)diff << " ft to minimum altitude of 1600 ft";
                    }
                    else
                    {
                        r << "Roger, descending " << (int)diff << " ft";
                    }
                    sel.setCommand(r.str(), 5.0f);
                    sel.command_delay = 5.0f;
                }
            }
            ImGui::SameLine();

            if (ImGui::Button("+100##alt3", ImVec2(100, 0)))
            {
                float old_target = sel.target_altitude_ft;

                float requested = old_target + 100.0f;
                float new_target = std::min(ALT_MAX, requested);

                float diff = new_target - old_target;

                if (diff <= 0.0f)
                {
                    sel.setImmediateResponse("Maximum altitude is 23000 ft", 5.0f);
                }
                else
                {
                    sel.pending_altitude_ft = new_target;
                    sel.has_pending_command = true;

                    std::ostringstream r;
                    if (requested > ALT_MAX)
                    {
                        r << "Climbing " << (int)diff << " ft to maximum altitude of 23000 ft";
                    }
                    else
                    {
                        r << "Roger, climbing " << (int)diff << " ft";
                    }
                    sel.setCommand(r.str(), 5.0f);
                    sel.command_delay = 5.0f;
                }
            }
            ImGui::SameLine();

            if (ImGui::Button("+1000##alt4", ImVec2(100, 0)))
            {
                float old_target = sel.target_altitude_ft;

                float requested = old_target + 1000.0f;
                float new_target = std::min(ALT_MAX, requested);

                float diff = new_target - old_target;

                if (diff <= 0.0f)
                {
                    sel.setImmediateResponse("Maximum altitude is 23000 ft", 5.0f);
                }
                else
                {
                    sel.pending_altitude_ft = new_target;
                    sel.has_pending_command = true;

                    std::ostringstream r;
                    if (requested > ALT_MAX)
                    {
                        r << "Climbing " << (int)diff << " ft to maximum altitude of 23000 ft";
                    }
                    else
                    {
                        r << "Roger, climbing " << (int)diff << " ft";
                    }
                    sel.setCommand(r.str(), 5.0f);
                    sel.command_delay = 5.0f;
                }
            }

            ImGui::Separator();
            // === HEADING ===
            ImGui::Text("Heading: %.0f°", fmodf(450.0f - sel.heading_deg, 360.0f));

            // Display target heading if different
            if (fabs(angle_difference(sel.target_heading_deg, sel.heading_deg)) > 1.0f)
            {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f),
                                   "-> %.0f°", fmodf(450.0f - sel.target_heading_deg, 360.0f));
            }

            // Heading buttons
            if (ImGui::Button("-5°##hdg1", ImVec2(210, 0)))
            {
                float old_target = sel.target_heading_deg;
                sel.pending_heading_deg = fmodf(sel.target_heading_deg + 5.0f, 360.0f);

                float diff = angle_difference(sel.pending_heading_deg, old_target);
                if (diff < 0) diff = -diff;

                std::ostringstream r;
                r << "Roger, turning left " << (int)diff << " degrees";
                sel.setCommand(r.str(), 3.5f); // stays for 3.5s
            }
            ImGui::SameLine();
            if (ImGui::Button("+5°##hdg2", ImVec2(210, 0)))
            {
                float old_target = sel.target_heading_deg;
                sel.pending_heading_deg = fmodf(sel.target_heading_deg - 5.0f + 360.0f, 360.0f);

                float diff = angle_difference(sel.pending_heading_deg, old_target);
                if (diff < 0) diff = -diff;

                std::ostringstream r;
                r << "Roger, turning right " << (int)diff << " degrees";
                sel.setCommand(r.str(), 3.5f);
            }

            // === SPEED ===
            ImGui::Text("Speed: %.0f kts", sel.speed_kts);

            // Display target speed if different
            if (fabs(sel.target_speed_kts - sel.speed_kts) > 1.0f)
            {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f),
                                   "-> %.0f kts", sel.target_speed_kts);
            }

            // Speed buttons
            if (ImGui::Button("-5 kts##spd1", ImVec2(210, 0)))
            {
                float old_target = sel.target_speed_kts;

                float requested = old_target - 5.0f;
                float new_target = std::max(SPD_MIN, requested);

                float diff = old_target - new_target;

                if (diff <= 0.0f)
                {
                    sel.setImmediateResponse("Minimum speed is 80 knots", 5.0f);
                }
                else
                {
                    sel.pending_speed_kts = new_target;
                    sel.has_pending_command = true;
                    sel.command_delay = 5.0f;

                    std::ostringstream r;
                    if (requested < SPD_MIN)
                    {
                        r << "Reducing speed " << (int)diff << " knots to minimum 80 knots";
                    }
                    else
                    {
                        r << "Roger, reducing speed " << (int)diff << " knots";
                    }
                    sel.setCommand(r.str(), 5.0f);
                }
            }
            ImGui::SameLine();

            if (ImGui::Button("+5 kts##spd2", ImVec2(210, 0)))
            {
                float old_target = sel.target_speed_kts;

                float requested = old_target + 5.0f;
                float new_target = std::min(SPD_MAX, requested);

                float diff = new_target - old_target;

                if (diff <= 0.0f)
                {
                    sel.setImmediateResponse("Maximum speed is 300 knots", 5.0f);
                }
                else
                {
                    sel.pending_speed_kts = new_target;
                    sel.has_pending_command = true;
                    sel.command_delay = 5.0f;

                    std::ostringstream r;
                    if (requested > SPD_MAX)
                    {
                        r << "Increasing speed " << (int)diff << " knots to maximum 300 knots";
                    }
                    else
                    {
                        r << "Roger, increasing speed " << (int)diff << " knots";
                    }
                    sel.setCommand(r.str(), 5.0f);
                }
            }

            // TEXT COMMAND INPUT
            static char command_buf[128] = "";
            static std::string command_feedback = "";
            static std::vector<std::string> history;
            static int history_index = -1;
            static float command_feedback_timer = 0.0f;

            ImGui::Separator();
            ImGui::Text("Command Input:");

            ImGuiInputTextFlags flags = ImGuiInputTextFlags_EnterReturnsTrue;

            if (ImGui::InputText("##cmd", command_buf, IM_ARRAYSIZE(command_buf), flags))
            {
                std::string cmd = command_buf;
                command_buf[0] = '\0';
                command_feedback = "";
                command_feedback_timer = 5.0f;

                history.push_back(cmd);
                history_index = -1;

                std::string u;
                for (char c : cmd) u += (char)toupper(c);

                auto extract_number = [&](const std::string& s) -> float
                {
                    for (int i = 0; i < (int)s.size(); i++)
                        if ((s[i] >= '0' && s[i] <= '9'))
                        {
                            return atof(s.c_str() + i);
                        }
                    return 0.0f;
                };

                // REMOVE AIRCRAFT command
                if (u.find("REMOVE") != std::string::npos &&
                    u.find("AIRCRAFT") != std::string::npos)
                {
                    aircraft.erase(aircraft.begin() + selected_index);
                    selected_index = -1;
                    command_feedback = "Aircraft removed.";
                }
                // ALT / FL command
                else if (u.rfind("FL", 0) == 0)
                {
                    float fl = extract_number(u);
                    float alt = fl * 100.0f;
                    alt = std::clamp(alt, ALT_MIN, ALT_MAX);

                    float old_pending = sel.pending_altitude_ft;
                    float diff = alt - old_pending;

                    sel.pending_altitude_ft = alt;
                    sel.has_pending_command = true;
                    sel.command_delay = 5.0f; // same delay as buttons

                    std::ostringstream response;
                    if (diff > 0)
                    {
                        response << "Climbing " << (int)diff << " ft to flight level " << (int)fl;
                    }
                    else
                    {
                        response << "Descending " << (int)(-diff) << " ft to flight level " << (int)fl;
                    }
                    sel.setCommand(response.str(), 5.0f);
                    command_feedback = "Flight level assigned.";
                }
                else if (u.rfind("ALT ", 0) == 0 || u.rfind("ALTITUDE ", 0) == 0 ||
                    u.rfind("CLIMB ", 0) == 0 || u.rfind("DESCEND ", 0) == 0)
                {
                    float alt = extract_number(u);
                    alt = std::clamp(alt, ALT_MIN, ALT_MAX);

                    float old_pending = sel.pending_altitude_ft;
                    float diff = alt - old_pending;

                    sel.pending_altitude_ft = alt;
                    sel.has_pending_command = true;
                    sel.command_delay = 5.0f;

                    std::ostringstream response;
                    if (diff > 0)
                    {
                        response << "Climbing " << (int)diff << " ft to " << (int)alt << " ft";
                    }
                    else
                    {
                        response << "Descending " << (int)(-diff) << " ft to " << (int)alt << " ft";
                    }
                    sel.setCommand(response.str(), 5.0f);
                    command_feedback = "Altitude assigned.";
                }
                // HDG / TURN commands
                else if (u.rfind("HDG ", 0) == 0 || u.rfind("HEADING ", 0) == 0)
                {
                    float hdg = extract_number(u);
                    float old_pending = sel.pending_heading_deg;

                    sel.pending_heading_deg = fmodf(450.0f - hdg, 360.0f);
                    sel.has_pending_command = true;
                    sel.command_delay = 3.5f; // same as heading buttons

                    float diff = angle_difference(sel.pending_heading_deg, old_pending);
                    if (diff < 0) diff = -diff;

                    std::ostringstream response;
                    response << "Turning to heading " << (int)hdg;
                    sel.setCommand(response.str(), 3.5f);
                    command_feedback = "Heading assigned.";
                }
                else if (u.rfind("TURN LEFT ", 0) == 0)
                {
                    float deg = extract_number(u);
                    float old_pending = sel.pending_heading_deg;

                    sel.pending_heading_deg = fmodf(sel.pending_heading_deg + deg, 360.0f);
                    sel.has_pending_command = true;
                    sel.command_delay = 3.5f;

                    float diff = angle_difference(sel.pending_heading_deg, old_pending);
                    if (diff < 0) diff = -diff;

                    std::ostringstream response;
                    response << "Turning left " << (int)diff << " degrees";
                    sel.setCommand(response.str(), 3.5f);
                    command_feedback = "Turning left.";
                }
                else if (u.rfind("TURN RIGHT ", 0) == 0)
                {
                    float deg = extract_number(u);
                    float old_pending = sel.pending_heading_deg;

                    sel.pending_heading_deg = fmodf(sel.pending_heading_deg - deg + 360.0f, 360.0f);
                    sel.has_pending_command = true;
                    sel.command_delay = 3.5f;

                    float diff = angle_difference(sel.pending_heading_deg, old_pending);
                    if (diff < 0) diff = -diff;

                    std::ostringstream response;
                    response << "Turning right " << (int)diff << " degrees";
                    sel.setCommand(response.str(), 3.5f);
                    command_feedback = "Turning right.";
                }
                // SPD / SPEED commands
                else if (u.rfind("SPD ", 0) == 0 || u.rfind("SPEED ", 0) == 0)
                {
                    float spd = extract_number(u);

                    float old_pending = sel.pending_speed_kts;
                    float requested = spd;
                    float new_speed = std::clamp(requested, SPD_MIN, SPD_MAX);

                    float diff = new_speed - old_pending;

                    if (fabs(diff) < 0.5f)
                    {
                        if (requested < SPD_MIN)
                        {
                            command_feedback = "Minimum speed is 80 knots.";
                        }
                        else if (requested > SPD_MAX)
                        {
                            command_feedback = "Maximum speed is 300 knots.";
                        }
                        else
                        {
                            command_feedback = "Speed unchanged.";
                        }
                    }
                    else
                    {
                        sel.pending_speed_kts = new_speed;
                        sel.has_pending_command = true;
                        sel.command_delay = 3.5f;

                        std::ostringstream response;
                        if (diff > 0)
                        {
                            if (requested > SPD_MAX)
                            {
                                response << "Increasing speed " << (int)diff
                                    << " knots to maximum 300 knots";
                            }
                            else
                            {
                                response << "Increasing speed " << (int)diff
                                    << " knots to " << (int)new_speed;
                            }
                        }
                        else
                        {
                            if (requested < SPD_MIN)
                            {
                                response << "Reducing speed " << (int)(-diff)
                                    << " knots to minimum 80 knots";
                            }
                            else
                            {
                                response << "Reducing speed " << (int)(-diff)
                                    << " knots to " << (int)new_speed;
                            }
                        }

                        sel.setCommand(response.str(), 3.5f);
                        command_feedback = "Speed assigned.";
                    }
                }
                else
                {
                    command_feedback = "Invalid or unknown command.";
                }
            }


            if (!command_feedback.empty() && command_feedback_timer > 0.0f)
            {
                command_feedback_timer -= dt;

                if (command_feedback_timer < 0.0f) command_feedback_timer = 0.0f;

                ImGui::TextColored(ImVec4(0.7f, 1.0f, 0.7f, 1.0f), "%s", command_feedback.c_str());
            }
        }
        else
        {
            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "AIRCRAFT CONTROL");
            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No aircraft selected.");
            ImGui::Text("Click on an aircraft");
            ImGui::Text("to control it.");
            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "(Overflights cannot be selected)");
        }

        ImGui::End();

    render_end:
        // Render
        ImGui::Render();
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
