#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "function/glfw.cpp"
#include "function/aircraft.cpp"

#include <cstdio>
#include <cmath>
#include <GLFW/glfw3.h>

#include <cstdlib>
#include <ctime>
#include <sstream>
#include <algorithm>

using namespace std;

static float deg_to_rad(float d) { return d * (IM_PI / 180.0f); }
static float rad_to_deg(float r) { return r * (180.0f / IM_PI); }

static constexpr float radar_range_km = 80.0f; // total radius in kilometers

// Helper function to normalize angle difference
static float angle_difference(float target, float current)
{
    float diff = fmodf(target - current + 540.0f, 360.0f) - 180.0f;
    return diff;
}

// Helper function to interpolate values smoothly
static float lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}

int main(int, char**)
{
    srand((unsigned)time(nullptr));
    fprintf(stdout, "Randomized seed.");

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        fprintf(stderr, "Error: GLFW initialization failed.");
        return 1;
    }

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

    GLFWwindow* window = glfwCreateWindow(1024, 768, "Air Traffic Controller", nullptr, nullptr);
    if (window == nullptr) {
        fprintf(stderr, "Error: Window failed to load.");
        return 1;
    }
    else fprintf(stdout, "Window created.");
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Setup Dear ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // --- Simulation state ---
    std::vector<Aircraft> aircraft;
    const int initial_count = 12;
    for (int i = 0; i < initial_count; ++i)
    {
        Aircraft a;
        generateAircraft(aircraft, a, radar_range_km, i);
        aircraft.push_back(a);
    }
    int selected_index = -1;

    // Camera/view state
    float camera_x = 0.0f; // Camera offset in km
    float camera_y = 0.0f;
    float zoom_level = 1.0f; // 1.0 = normal, >1.0 = zoomed in, <1.0 = zoomed out
    bool is_panning = false;
    ImVec2 last_mouse_pos;

    double last_time = glfwGetTime();

    while (!glfwWindowShouldClose(window))
    {
        // --- Time ---
        double now = glfwGetTime();
        float dt = (float)(now - last_time);
        if (dt <= 0.0f) dt = 1.0f / 60.0f;
        last_time = now;

        // Get display size (used for fullscreen radar and rendering)
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);

        // Poll events
        glfwPollEvents();

        // Update simulation
        for (auto& a : aircraft)
        {
            // Update timers
            if (a.response_timer > 0.0f)
                a.response_timer -= dt;

            // Handle command delay - apply pending targets when delay expires
            if (a.command_delay > 0.0f)
            {
                a.command_delay -= dt;
                if (a.command_delay <= 0.0f)
                {
                    // Apply the pending command
                    a.target_altitude_ft = a.pending_altitude_ft;
                    a.target_heading_deg = a.pending_heading_deg;
                    a.target_speed_kts = a.pending_speed_kts;
                    a.has_pending_command = false;
                }
            }

            // Gradual transitions with smooth acceleration/deceleration
            // More realistic rates for commercial aircraft
            const float max_alt_rate = 33.33f; // max 2000 ft/min = 33.33 ft/s
            const float alt_accel = 4.0f; // smoother acceleration ft/s²
            const float max_speed_rate = 3.0f; // max 3 knots/s (more realistic for jets)
            const float speed_accel = 0.5f; // slower acceleration knots/s²

            // Altitude transition with smooth ease-in-out
            float alt_diff = a.target_altitude_ft - a.altitude_ft;
            if (fabs(alt_diff) > 1.0f)
            {
                // Calculate desired rate based on distance (with smoother curve)
                float desired_rate = 0.0f;
                float distance_factor = sqrtf(2.0f * alt_accel * fabs(alt_diff));

                if (alt_diff > 0)
                    desired_rate = std::min(max_alt_rate, distance_factor);
                else
                    desired_rate = -std::min(max_alt_rate, distance_factor);

                // Smoothly adjust current rate toward desired rate
                float rate_diff = desired_rate - a.altitude_rate_fps;
                if (fabs(rate_diff) > 0.1f)
                {
                    float accel_step = alt_accel * dt;
                    if (rate_diff > 0)
                        a.altitude_rate_fps += std::min(accel_step, rate_diff);
                    else
                        a.altitude_rate_fps += std::max(-accel_step, rate_diff);
                }
                else
                {
                    a.altitude_rate_fps = desired_rate;
                }

                // Apply rate to altitude
                float new_altitude = a.altitude_ft + a.altitude_rate_fps * dt;

                // Check if we've reached or passed target
                if ((alt_diff > 0 && new_altitude >= a.target_altitude_ft) ||
                    (alt_diff < 0 && new_altitude <= a.target_altitude_ft))
                {
                    a.altitude_ft = a.target_altitude_ft;
                    a.altitude_rate_fps = 0.0f;
                }
                else
                {
                    a.altitude_ft = new_altitude;
                }
            }
            else
            {
                a.altitude_ft = a.target_altitude_ft;
                a.altitude_rate_fps = 0.0f;
            }

            // Heading transition (realistic turn rate: ~2-3 degrees per second for commercial jets)
            float heading_diff = angle_difference(a.target_heading_deg, a.heading_deg);
            if (fabs(heading_diff) > 0.5f)
            {
                float turn_rate = 2.5f * dt; // 2.5 degrees per second
                if (heading_diff > 0)
                    a.heading_deg += std::min(turn_rate, heading_diff);
                else
                    a.heading_deg += std::max(-turn_rate, heading_diff);

                a.heading_deg = fmodf(a.heading_deg + 360.0f, 360.0f);
            }
            else
            {
                a.heading_deg = a.target_heading_deg;
            }

            // Speed transition with smooth ease-in-out (realistic for jets)
            float speed_diff = a.target_speed_kts - a.speed_kts;
            if (fabs(speed_diff) > 0.5f)
            {
                // Calculate desired rate based on distance (smoother curve)
                float desired_rate = 0.0f;
                float distance_factor = sqrtf(2.0f * speed_accel * fabs(speed_diff));

                if (speed_diff > 0)
                    desired_rate = std::min(max_speed_rate, distance_factor);
                else
                    desired_rate = -std::min(max_speed_rate, distance_factor);

                // Smoothly adjust current rate toward desired rate
                float rate_diff = desired_rate - a.speed_rate_kps;
                if (fabs(rate_diff) > 0.05f)
                {
                    float accel_step = speed_accel * dt;
                    if (rate_diff > 0)
                        a.speed_rate_kps += std::min(accel_step, rate_diff);
                    else
                        a.speed_rate_kps += std::max(-accel_step, rate_diff);
                }
                else
                {
                    a.speed_rate_kps = desired_rate;
                }

                // Apply rate to speed
                float new_speed = a.speed_kts + a.speed_rate_kps * dt;

                // Check if we've reached or passed target
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

            // Position update (using current speed and heading)
            float speed_kmh = a.speed_kts * 1.852f;
            float speed_kms = speed_kmh / 3600.0f;
            float ang = deg_to_rad(a.heading_deg);
            a.x += cosf(ang) * speed_kms * dt;
            a.y += sinf(ang) * speed_kms * dt;
        }

        // Basic conflict detection
        const float conflict_horiz_km = 5.0f;
        const float conflict_horiz_km2 = conflict_horiz_km * conflict_horiz_km;
        const float conflict_vert_ft = 1000.0f;

        std::vector<std::pair<int, int>> conflicts;
        for (size_t i = 0; i < aircraft.size(); ++i)
        {
            for (size_t j = i + 1; j < aircraft.size(); ++j)
            {
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

        // Start new ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Radar window layout - FULLSCREEN
        ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_NoSavedSettings |
            ImGuiWindowFlags_NoBringToFrontOnFocus;

        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2((float)display_w, (float)display_h), ImGuiCond_Always);

        ImGui::Begin("Radar", nullptr, window_flags);

        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 win_pos = ImGui::GetWindowPos();
        ImVec2 win_size = ImGui::GetWindowSize();
        ImVec2 content_min = ImGui::GetWindowContentRegionMin();
        ImVec2 content_max = ImGui::GetWindowContentRegionMax();
        ImVec2 center(win_pos.x + win_size.x * 0.5f, win_pos.y + win_size.y * 0.5f);
        float radius_max = (win_size.x < win_size.y ? win_size.x : win_size.y) * 0.45f;

        // Handle mouse input for panning and zooming
        ImVec2 mouse_pos = ImGui::GetMousePos();
        bool is_radar_hovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows);

        // Zoom with mouse wheel (works with touchpad two-finger scroll)
        if (is_radar_hovered)
        {
            float wheel = io.MouseWheel;
            if (wheel != 0.0f)
            {
                float zoom_factor = 1.0f + wheel * 0.15f; // Increased sensitivity for touchpad
                float new_zoom = zoom_level * zoom_factor;
                // Limit zoom range
                new_zoom = std::max(0.2f, std::min(5.0f, new_zoom));

                // Zoom toward mouse position
                ImVec2 mouse_rel(mouse_pos.x - center.x, mouse_pos.y - center.y);
                float scale_before = radius_max / (radar_range_km * zoom_level);
                float scale_after = radius_max / (radar_range_km * new_zoom);

                camera_x += mouse_rel.x / scale_before - mouse_rel.x / scale_after;
                camera_y -= mouse_rel.y / scale_before - mouse_rel.y / scale_after;

                zoom_level = new_zoom;
            }
        }

        // Pan with left mouse drag (while holding Shift key for touchpad users)
        // OR middle/right mouse button
        bool shift_held = io.KeyShift;
        bool should_pan = is_radar_hovered && (
            (ImGui::IsMouseDown(0) && shift_held) ||  // Left click + Shift
            ImGui::IsMouseDown(2) ||                   // Middle mouse
            ImGui::IsMouseDown(1)                      // Right mouse
        );

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
                camera_y += delta.y / scale; // Inverted Y
                last_mouse_pos = mouse_pos;
            }
        }
        else
        {
            is_panning = false;
        }

        ImU32 col_green = IM_COL32(0, 255, 0, 100);
        ImU32 bg = IM_COL32(0, 20, 0, 200);
        draw_list->AddRectFilled(win_pos, ImVec2(win_pos.x + win_size.x, win_pos.y + win_size.y), bg, 8.0f);

        // Draw runways and ILS approach lines
        ImU32 runway_color = IM_COL32(100, 150, 255, 255);
        ImU32 ils_color = IM_COL32(80, 120, 200, 150);

        // Runway 09/27 (East-West)
        float runway_length = 60.0f; // pixels
        float runway_width = 8.0f;

        // Rotating sweep line
        float time = (float)ImGui::GetTime();
        float speed = 0.8f;

        ImU32 sweep_color = IM_COL32(120, 255, 120, 255);

        // Helper: world (km) -> screen pos
        auto world_to_screen = [&](float wx, float wy)
        {
            float scale = radius_max / (radar_range_km * zoom_level);

            float vx = wx - camera_x;
            float vy = wy - camera_y;

            return ImVec2(
                center.x + vx * scale,
                center.y - vy * scale
            );
        };

        // Draw rings and labels
        const int rings = 4;
        for (int i = 1; i <= rings; ++i)
        {
            float km_radius = (radar_range_km * i) / rings;

            // Convert TWO world points; use distance between them
            ImVec2 p0 = world_to_screen(0, 0);
            ImVec2 p1 = world_to_screen(km_radius, 0);

            float screen_r = fabs(p1.x - p0.x);

            draw_list->AddCircle(p0, screen_r, col_green, 128, 1.0f);

            // Label
            float range_nm = km_radius * 0.539957f;
            std::ostringstream ss;
            ss << (int)range_nm << " nm";

            draw_list->AddText(
                ImVec2(p0.x + screen_r - 30, p0.y - 12),
                col_green,
                ss.str().c_str()
            );
        }

        // Runway positions in world KM (centered)
        ImVec2 rw09 = world_to_screen(-1.0f, 0.0f);
        ImVec2 rw27 = world_to_screen(+1.0f, 0.0f);

        ImVec2 rw18 = world_to_screen(0.0f, -1.0f);
        ImVec2 rw36 = world_to_screen(0.0f, +1.0f);

        draw_list->AddLine(rw09, rw27, runway_color, 8.0f);

        ImVec2 ils09 = world_to_screen(-6.0f, 0.0f);
        ImVec2 ils27 = world_to_screen(+6.0f, 0.0f);

        draw_list->AddLine(ils09, rw09, ils_color, 2.0f);
        draw_list->AddLine(rw27, ils27, ils_color, 2.0f);

        draw_list->AddLine(rw18, rw36, runway_color, 8.0f);

        ImVec2 ils18 = world_to_screen(0.0f, -6.0f);
        ImVec2 ils36 = world_to_screen(0.0f, +6.0f);

        draw_list->AddLine(ils18, rw18, ils_color, 2.0f);
        draw_list->AddLine(rw36, ils36, ils_color, 2.0f);

        draw_list->AddText(rw27, runway_color, "09");
        draw_list->AddText(rw09, runway_color, "27");
        draw_list->AddText(rw36, runway_color, "36");
        draw_list->AddText(rw18, runway_color, "18");

        // Radar sweep (in world coordinates)
        float sweep_angle = fmodf(ImGui::GetTime() * 0.8f, 2.0f * IM_PI);

        // sweep length in world km
        float sweep_length_km = radar_range_km;

        // sweep endpoints in world space
        float sx = cosf(sweep_angle) * sweep_length_km;
        float sy = sinf(sweep_angle) * sweep_length_km;

        // convert to screen
        ImVec2 sweep_center = world_to_screen(0.0f, 0.0f);
        ImVec2 sweep_tip    = world_to_screen(sx, sy);

        draw_list->AddLine(sweep_center, sweep_tip, sweep_color, 2.0f);

        // world axes (in km)
        float axis_len_km = radar_range_km;

        // horizontal line (east-west)
        ImVec2 left  = world_to_screen(-axis_len_km, 0.0f);
        ImVec2 right = world_to_screen(+axis_len_km, 0.0f);
        draw_list->AddLine(left, right, col_green, 1.0f);

        // vertical line (north-south)
        ImVec2 down = world_to_screen(0.0f, -axis_len_km);
        ImVec2 up   = world_to_screen(0.0f, +axis_len_km);
        draw_list->AddLine(down, up, col_green, 1.0f);

        // center dot
        ImVec2 c = world_to_screen(0.0f, 0.0f);
        draw_list->AddCircleFilled(c, 3.0f, col_green);

        // trail
        const int trail_segments = 25;
        for (int i = 1; i < trail_segments; i++)
        {
            float fade = 1.0f - (float)i / trail_segments;
            float a = sweep_angle - (i * 0.03f);

            float tx = cosf(a) * sweep_length_km;
            float ty = sinf(a) * sweep_length_km;

            ImVec2 tpos = world_to_screen(tx, ty);
            ImU32 trail_color = IM_COL32(0, (int)(200 * fade), 0, (int)(120 * fade));

            draw_list->AddLine(sweep_center, tpos, trail_color, 1.0f);
        }


        // Draw aircraft blips
        for (size_t i = 0; i < aircraft.size(); ++i)
        {
            const Aircraft& a = aircraft[i];
            ImVec2 pos = world_to_screen(a.x, a.y);

            // Check if aircraft is visible on screen (with larger margin)
            float margin = 100.0f; // Allow aircraft to be off-radar but still on screen
            bool on_screen = (pos.x >= win_pos.x - margin && pos.x <= win_pos.x + win_size.x + margin &&
                            pos.y >= win_pos.y - margin && pos.y <= win_pos.y + win_size.y + margin);

            if (!on_screen)
                continue;

            bool in_conflict = false;
            for (auto& c : conflicts)
            {
                if ((int)i == c.first || (int)i == c.second)
                {
                    in_conflict = true;
                    break;
                }
            }

            ImU32 blip_col = in_conflict ? IM_COL32(255, 64, 64, 255) : IM_COL32(255, 255, 0, 255);
            float blip_radius = (selected_index == (int)i) ? 6.0f : 4.0f;
            draw_list->AddCircleFilled(pos, blip_radius, blip_col);

            float head_rad = deg_to_rad(a.heading_deg);
            ImVec2 head_end(pos.x + cosf(head_rad) * 12.0f, pos.y - sinf(head_rad) * 12.0f);
            draw_list->AddLine(pos, head_end, IM_COL32(200, 200, 200, 180), 1.0f);

            std::ostringstream ss;
            ss << a.callsign << " " << (int)(a.altitude_ft) << "ft";
            draw_list->AddText(ImVec2(pos.x + 8.0f, pos.y - 10.0f), IM_COL32(180, 240, 180, 220), ss.str().c_str());
        }

        // Draw conflict lines
        for (auto& c : conflicts)
        {
            const Aircraft& a = aircraft[c.first];
            const Aircraft& b = aircraft[c.second];
            ImVec2 pa = world_to_screen(a.x, a.y);
            ImVec2 pb = world_to_screen(b.x, b.y);
            draw_list->AddLine(pa, pb, IM_COL32(255, 80, 80, 200), 2.0f);
            ImVec2 mid((pa.x + pb.x) * 0.5f, (pa.y + pb.y) * 0.5f);
            draw_list->AddText(ImVec2(mid.x + 4, mid.y + 4), IM_COL32(255, 120, 120, 220), "CONFLICT");
        }

        // Selection
        ImVec2 io_mouse = ImGui::GetMousePos();
        bool clicked = ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows) && ImGui::IsMouseClicked(0);
        if (clicked)
        {
            float best_d = 999999.0f;
            int best_idx = -1;
            for (size_t i = 0; i < aircraft.size(); ++i)
            {
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

        ImGui::Dummy(ImVec2(0, 0));
        ImGui::End();

        // --- Aircraft control panel ---
        if (selected_index >= 0 && selected_index < (int)aircraft.size())
        {
            Aircraft& sel = aircraft[selected_index];
            std::string callSignLabel = "Callsign " + sel.callsign;

            ImGui::Begin("Aircraft Control");

            ImGui::SetWindowFontScale(2.0f);
            ImGui::Text(callSignLabel.c_str());
            ImGui::SetWindowFontScale(1.0f);

            // Show aircraft response if active
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

            // Display current and target values
            ImGui::Text("Altitude: %.0f ft", sel.altitude_ft);

            // Show pending target immediately, or active target if different from current
            float display_target = (sel.command_delay > 0.0f) ? sel.pending_altitude_ft : sel.target_altitude_ft;
            if (fabs(display_target - sel.altitude_ft) > 10.0f)
            {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f),
                    "-> %.0f ft", display_target);
            }

            if (ImGui::Button("-1000 ft"))
            {
                float old = sel.pending_altitude_ft;
                sel.pending_altitude_ft = std::max(0.0f, sel.pending_altitude_ft - 1000.0f);

                float diff = fabs(sel.pending_altitude_ft - old);

                std::ostringstream r;
                r << "Roger, descending " << (int)diff << " feet";
                sel.setCommand(r.str());
            }
            ImGui::SameLine();

            if (ImGui::Button("-100 ft"))
            {
                float old = sel.pending_altitude_ft;
                sel.pending_altitude_ft = std::max(0.0f, sel.pending_altitude_ft - 100.0f);

                float diff = fabs(sel.pending_altitude_ft - old);

                std::ostringstream r;
                r << "Roger, descending " << (int)diff << " feet";
                sel.setCommand(r.str());
            }
            ImGui::SameLine();

            if (ImGui::Button("+100 ft"))
            {
                float old = sel.pending_altitude_ft;
                sel.pending_altitude_ft += 100.0f;

                float diff = fabs(sel.pending_altitude_ft - old);

                std::ostringstream r;
                r << "Roger, climbing " << (int)diff << " feet";
                sel.setCommand(r.str());
            }
            ImGui::SameLine();

            if (ImGui::Button("+1000 ft"))
            {
                float old = sel.pending_altitude_ft;
                sel.pending_altitude_ft += 1000.0f;

                float diff = fabs(sel.pending_altitude_ft - old);

                std::ostringstream r;
                r << "Roger, climbing " << (int)diff << " feet";
                sel.setCommand(r.str());
            }

            ImGui::Separator();
            ImGui::Text("Heading: %.0f°", fmodf(450.0f - sel.heading_deg, 360.0f));

            // Show pending target immediately, or active target if different from current
            float display_heading = (sel.command_delay > 0.0f) ? sel.pending_heading_deg : sel.target_heading_deg;
            if (fabs(angle_difference(display_heading, sel.heading_deg)) > 1.0f)
            {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f),
                    "-> %.0f°", fmodf(450.0f - display_heading, 360.0f));
            }

            if (ImGui::Button("-5°"))
            {
                float old = sel.pending_heading_deg;
                sel.pending_heading_deg = fmodf(sel.pending_heading_deg + 5.0f, 360.0f);

                float diff = angle_difference(sel.pending_heading_deg, old);
                if (diff < 0) diff = -diff;     // always positive

                std::ostringstream r;
                r << "Roger, turning left " << (int)diff << " degrees";
                sel.setCommand(r.str());
            }
            ImGui::SameLine();
            if (ImGui::Button("+5°"))
            {
                float old = sel.pending_heading_deg;
                sel.pending_heading_deg = fmodf(sel.pending_heading_deg - 5.0f + 360.0f, 360.0f);

                float diff = angle_difference(old, sel.pending_heading_deg);
                if (diff < 0) diff = -diff;

                std::ostringstream r;
                r << "Roger, turning right " << (int)diff << " degrees";
                sel.setCommand(r.str());
            }

            ImGui::Separator();
            ImGui::Text("Speed: %.0f kts", sel.speed_kts);

            // Show pending target immediately, or active target if different from current
            float display_speed = (sel.command_delay > 0.0f) ? sel.pending_speed_kts : sel.target_speed_kts;
            if (fabs(display_speed - sel.speed_kts) > 1.0f)
            {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f),
                    "-> %.0f kts", display_speed);
            }

            if (ImGui::Button("-5 kts"))
            {
                float old = sel.pending_speed_kts;
                sel.pending_speed_kts = std::max(0.0f, sel.pending_speed_kts - 5.0f);

                float diff = fabs(sel.pending_speed_kts - old);

                std::ostringstream r;
                r << "Roger, reducing speed " << (int)diff << " knots";
                sel.setCommand(r.str());
            }
            ImGui::SameLine();

            if (ImGui::Button("+5 kts"))
            {
                float old = sel.pending_speed_kts;
                sel.pending_speed_kts += 5.0f;

                float diff = fabs(sel.pending_speed_kts - old);

                std::ostringstream r;
                r << "Roger, increasing speed " << (int)diff << " knots";
                sel.setCommand(r.str());
            }

            //---------------------------------------------------------------
            // TEXT COMMAND INPUT SYSTEM
            //---------------------------------------------------------------

            static char command_buf[128] = "";
            static std::string command_feedback = "";

            static std::vector<std::string> history;
            static int history_index = -1;

            static const char* known_cmds[] = {
                "HDG ", "HEADING ",
                "ALT ", "ALTITUDE ",
                "FL", "FL ",
                "SPD ", "SPEED ",
                "CLIMB ", "CLIMB AND MAINTAIN ",
                "DESCEND ", "DESCEND AND MAINTAIN ",
                "TURN LEFT ", "TURN LEFT HEADING ",
                "TURN RIGHT ", "TURN RIGHT HEADING "
            };

            static const int known_cmd_count = sizeof(known_cmds) / sizeof(const char*);

            ImGui::Separator();
            ImGui::Text("Command Input:");

            ImGuiInputTextFlags flags =
                ImGuiInputTextFlags_EnterReturnsTrue |
                ImGuiInputTextFlags_CallbackHistory |
                ImGuiInputTextFlags_CallbackCompletion;

            ImGuiInputTextCallback text_callback = [](ImGuiInputTextCallbackData* data) -> int
                {
                    auto* hist = reinterpret_cast<std::vector<std::string>*>(data->UserData);

                    if (data->EventFlag == ImGuiInputTextFlags_CallbackHistory)
                    {
                        if (hist->empty()) return 0;

                        if (data->EventKey == ImGuiKey_UpArrow)
                        {
                            if (history_index == -1)
                                history_index = (int)hist->size() - 1;
                            else if (history_index > 0)
                                history_index--;

                            data->DeleteChars(0, data->BufTextLen);
                            data->InsertChars(0, (*hist)[history_index].c_str());
                        }
                        else if (data->EventKey == ImGuiKey_DownArrow)
                        {
                            if (history_index >= 0)
                            {
                                history_index++;
                                if (history_index >= (int)hist->size())
                                    history_index = -1;

                                data->DeleteChars(0, data->BufTextLen);

                                if (history_index != -1)
                                    data->InsertChars(0, (*hist)[history_index].c_str());
                            }
                        }
                    }
                    else if (data->EventFlag == ImGuiInputTextFlags_CallbackCompletion)
                    {
                        const char* input = data->Buf;

                        for (int i = 0; i < known_cmd_count; i++)
                        {
                            if (strncmp(input, known_cmds[i], strlen(input)) == 0)
                            {
                                data->DeleteChars(0, data->BufTextLen);
                                data->InsertChars(0, known_cmds[i]);
                                break;
                            }
                        }
                    }

                    return 0;
                };

            if (ImGui::InputText("##cmd", command_buf, IM_ARRAYSIZE(command_buf),
                flags, text_callback, &history))
            {
                std::string cmd = command_buf;
                command_buf[0] = '\0';
                command_feedback = "";

                history.push_back(cmd);
                history_index = -1;

                std::string u;
                for (char c : cmd) u += (char)toupper(c);

                Aircraft& selected = aircraft[selected_index];

                auto extract_number = [&](const std::string& s) -> float
                    {
                        for (int i = 0; i < (int)s.size(); i++)
                            if ((s[i] >= '0' && s[i] <= '9'))
                                return atof(s.c_str() + i);
                        return 0.0f;
                    };

                // Command parser with responses
                if (u.rfind("FL", 0) == 0)
                {
                    float fl = extract_number(u);
                    selected.pending_altitude_ft = fl * 100.0f;
                    std::ostringstream response;
                    response << "Roger, climbing to flight level " << (int)fl;
                    selected.setCommand(response.str());
                    command_feedback = "Flight level assigned.";
                }
                else if (u.rfind("HDG ", 0) == 0 || u.rfind("HEADING ", 0) == 0)
                {
                    float hdg = extract_number(u);
                    selected.pending_heading_deg = fmodf(450.0f - hdg, 360.0f);
                    std::ostringstream response;
                    response << "Roger, turning to heading " << (int)hdg;
                    selected.setCommand(response.str());
                    command_feedback = "Heading assigned.";
                }
                else if (u.rfind("TURN LEFT ", 0) == 0)
                {
                    float deg = extract_number(u);
                    selected.pending_heading_deg = fmodf(selected.pending_heading_deg + deg, 360.0f);
                    std::ostringstream response;
                    response << "Roger, turning left " << (int)deg << " degrees";
                    selected.setCommand(response.str());
                    command_feedback = "Turning left.";
                }
                else if (u.rfind("TURN RIGHT ", 0) == 0)
                {
                    float deg = extract_number(u);
                    selected.pending_heading_deg = fmodf(selected.pending_heading_deg - deg + 360.0f, 360.0f);
                    std::ostringstream response;
                    response << "Roger, turning right " << (int)deg << " degrees";
                    selected.setCommand(response.str());
                    command_feedback = "Turning right.";
                }
                else if (u.rfind("TURN LEFT HEADING ", 0) == 0)
                {
                    float hdg = extract_number(u);
                    selected.pending_heading_deg = fmodf(450.0f - hdg, 360.0f);
                    std::ostringstream response;
                    response << "Roger, turning left to heading " << (int)hdg;
                    selected.setCommand(response.str());
                    command_feedback = "Turn left to heading.";
                }
                else if (u.rfind("TURN RIGHT HEADING ", 0) == 0)
                {
                    float hdg = extract_number(u);
                    selected.pending_heading_deg = fmodf(450.0f - hdg, 360.0f);
                    std::ostringstream response;
                    response << "Roger, turning right to heading " << (int)hdg;
                    selected.setCommand(response.str());
                    command_feedback = "Turn right to heading.";
                }
                else if (u.rfind("ALT ", 0) == 0 || u.rfind("ALTITUDE ", 0) == 0)
                {
                    float alt = extract_number(u);
                    selected.pending_altitude_ft = alt;
                    std::ostringstream response;
                    if (alt > selected.altitude_ft)
                        response << "Roger, climbing to " << (int)alt << " feet";
                    else
                        response << "Roger, descending to " << (int)alt << " feet";
                    selected.setCommand(response.str());
                    command_feedback = "Altitude assigned.";
                }
                else if (u.rfind("CLIMB AND MAINTAIN ", 0) == 0 ||
                    u.rfind("CLIMB ", 0) == 0)
                {
                    float alt = extract_number(u);
                    selected.pending_altitude_ft = alt;
                    std::ostringstream response;
                    response << "Roger, climb and maintain " << (int)alt;
                    selected.setCommand(response.str());
                    command_feedback = "Climbing.";
                }
                else if (u.rfind("DESCEND AND MAINTAIN ", 0) == 0 ||
                    u.rfind("DESCEND ", 0) == 0)
                {
                    float alt = extract_number(u);
                    selected.pending_altitude_ft = alt;
                    std::ostringstream response;
                    response << "Roger, descend and maintain " << (int)alt;
                    selected.setCommand(response.str());
                    command_feedback = "Descending.";
                }
                else if (u.rfind("SPD ", 0) == 0 || u.rfind("SPEED ", 0) == 0)
                {
                    float spd = extract_number(u);
                    selected.pending_speed_kts = spd;
                    std::ostringstream response;
                    response << "Roger, adjusting speed to " << (int)spd << " knots";
                    selected.setCommand(response.str());
                    command_feedback = "Speed assigned.";
                }
                else
                {
                    command_feedback = "Invalid or unknown command.";
                }
            }

            if (!command_feedback.empty())
            {
                ImGui::TextColored(ImVec4(0.7f, 1.0f, 0.7f, 1.0f),
                    "%s", command_feedback.c_str());
            }

            ImGui::Separator();
            ImGui::Text("Command Cheat Sheet:");
            ImGui::BeginChild("cheatsheet", ImVec2(0, 302), true);

            ImGui::TextColored(ImVec4(0.8f, 1.0f, 0.8f, 1.0f),
                "HEADING / TURNS");
            ImGui::BulletText("HDG 270");
            ImGui::BulletText("HEADING 090");
            ImGui::BulletText("TURN LEFT 20");
            ImGui::BulletText("TURN RIGHT 30");
            ImGui::BulletText("TURN LEFT HEADING 180");

            ImGui::TextColored(ImVec4(0.8f, 1.0f, 0.8f, 1.0f),
                "ALTITUDE");
            ImGui::BulletText("ALT 12000");
            ImGui::BulletText("ALTITUDE 8000");
            ImGui::BulletText("CLIMB 1000");
            ImGui::BulletText("DESCEND 500");
            ImGui::BulletText("CLIMB AND MAINTAIN 9000");
            ImGui::BulletText("DESCEND AND MAINTAIN 7000");
            ImGui::BulletText("FL180 (sets 18,000 ft)");

            ImGui::TextColored(ImVec4(0.8f, 1.0f, 0.8f, 1.0f),
                "SPEED");
            ImGui::BulletText("SPD 220");
            ImGui::BulletText("SPEED 280");

            ImGui::EndChild();

            Aircraft& selected = aircraft[selected_index];
            std::string removeButtonLabel = "Remove " + selected.callsign;

            if (ImGui::Button(removeButtonLabel.c_str()))
            {
                aircraft.erase(aircraft.begin() + selected_index);
                selected_index = -1;
            }

            ImGui::End();
        }

        // Control panel
        ImGui::Begin("Control");
        if (ImGui::Button("Spawn Random AC"))
        {
            Aircraft a;
            generateAircraft(aircraft, a, radar_range_km, aircraft.size());
            aircraft.push_back(a);
        }
        ImGui::Text("Aircraft: %zu", aircraft.size());
        ImGui::Text("Conflicts: %zu", conflicts.size());

        ImGui::Separator();
        ImGui::Text("Camera Controls:");
        ImGui::BulletText("Two-finger scroll: Zoom");
        ImGui::BulletText("Shift + Drag: Pan");
        ImGui::BulletText("Right Click Drag: Pan");
        ImGui::Text("Zoom: %.1fx", zoom_level);
        ImGui::Text("Offset: (%.1f, %.1f) km", camera_x, camera_y);

        if (ImGui::Button("Reset View"))
        {
            camera_x = 0.0f;
            camera_y = 0.0f;
            zoom_level = 1.0f;
        }

        ImGui::End();

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