#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "function/glfw.cpp"

#include <cstdio>
#include <cmath>
#include <GLFW/glfw3.h>

#include <cstdlib>
#include <ctime>
#include <sstream>
#include <algorithm>
#include <vector>
#include <string>
#include <iomanip>

using namespace std;

#ifndef IM_PI
#define IM_PI 3.14159265358979323846f
#endif

static float deg_to_rad(float d) { return d * (IM_PI / 180.0f); }
static float rad_to_deg(float r) { return r * (180.0f / IM_PI); }

static constexpr float radar_range_km = 80.0f;

// Runway data structure
struct Runway
{
    std::string name;
    float heading_deg; // true heading
    float x, y; // position in km
};

// Waypoint structure
struct Waypoint
{
    std::string name;
    float x, y; // position in km
};

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

    void setCommand(const std::string& response, float delay = 3.5f)
    {
        if (is_overflight) return; // Cannot contact overflights

        last_response = response;
        response_timer = 5.0f;
        command_delay = delay;
        has_pending_command = true;
    }
};

static float angle_difference(float target, float current)
{
    float diff = fmodf(target - current + 540.0f, 360.0f) - 180.0f;
    return diff;
}

// Generate squawk code
std::string generateSquawkCode()
{
    int code = 1000 + rand() % 7000;
    std::ostringstream ss;
    ss << std::setw(4) << std::setfill('0') << code;
    return ss.str();
}

void generateAircraft(std::vector<Aircraft>& aircrafts, Aircraft& a, const float radar_range_km, const int i) {
    std::ostringstream ss;
    ss << "AC" << std::setw(2) << std::setfill('0') << (i + 1);
    a.callsign = ss.str();

    const float r = ((float)rand() / RAND_MAX) * radar_range_km;
    const float theta = ((float)rand() / RAND_MAX) * 2.0f * IM_PI;
    a.x = cosf(theta) * r;
    a.y = sinf(theta) * r;

    // 10% chance of overflight
    if (rand() % 100 < 10)
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

        // 2% chance of emergency
        if (rand() % 100 < 2)
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

    a.initTargets();
}

int main(int, char**)
{
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

    GLFWwindow* window = glfwCreateWindow(1280, 768, "Air Traffic Controller", nullptr, nullptr);
    if (window == nullptr) return 1;

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Setup runways
    std::vector<Runway> runways;
    runways.push_back({"09", 90.0f, -1.0f, 0.0f});
    runways.push_back({"27", 270.0f, 1.0f, 0.0f});
    runways.push_back({"18", 180.0f, 0.0f, -1.0f});
    runways.push_back({"36", 0.0f, 0.0f, 1.0f});
    runways.push_back({"04", 40.0f, -0.8f, -0.6f});
    runways.push_back({"22", 220.0f, 0.8f, 0.6f});

    // Setup waypoints
    std::vector<Waypoint> waypoints;
    waypoints.push_back({"ALPHA", 20.0f, 30.0f});
    waypoints.push_back({"BRAVO", -25.0f, 35.0f});
    waypoints.push_back({"CHARLIE", 35.0f, -20.0f});
    waypoints.push_back({"DELTA", -30.0f, -25.0f});
    waypoints.push_back({"ECHO", 40.0f, 10.0f});
    waypoints.push_back({"FOXTROT", -15.0f, -40.0f});

    // Wind
    float wind_heading = (float)(rand() % 360);
    float wind_speed_kts = 5.0f + (rand() % 25); // 5-30 kts

    std::vector<Aircraft> aircraft;
    const int initial_count = 12;
    for (int i = 0; i < initial_count; ++i)
    {
        Aircraft a;
        generateAircraft(aircraft, a, radar_range_km, i);
        aircraft.push_back(a);
    }
    int selected_index = -1;

    float camera_x = 0.0f;
    float camera_y = 0.0f;
    float zoom_level = 1.0f;
    bool is_panning = false;
    ImVec2 last_mouse_pos;

    double last_time = glfwGetTime();
    float animation_speed = 1.0f; // Time multiplier

    const float right_panel_width = 450.0f;

    while (!glfwWindowShouldClose(window))
    {
        double now = glfwGetTime();
        float dt = (float)(now - last_time) * animation_speed;
        if (dt <= 0.0f) dt = 1.0f / 60.0f;
        last_time = now;

        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);

        glfwPollEvents();

        // Update aircraft
        for (auto& a : aircraft)
        {
            if (a.response_timer > 0.0f)
                a.response_timer -= dt;

            // Update emergency timer
            if (a.emergency != EMERGENCY_NONE && !a.is_overflight)
            {
                a.emergency_timer -= dt;
                if (a.emergency_timer <= 0.0f)
                {
                    // Emergency critical - aircraft would crash
                    // In real implementation, handle crash
                }
            }

            if (a.command_delay > 0.0f)
            {
                a.command_delay -= dt;
                if (a.command_delay <= 0.0f)
                {
                    a.target_altitude_ft = a.pending_altitude_ft;
                    a.target_heading_deg = a.pending_heading_deg;
                    a.target_speed_kts = a.pending_speed_kts;
                    a.has_pending_command = false;
                }
            }

            // ILS approach guidance
            if (a.ils_active && !a.ils_runway.empty())
            {
                // Find the runway
                for (const auto& rwy : runways)
                {
                    if (rwy.name == a.ils_runway)
                    {
                        // Calculate if aircraft is on correct glideslope
                        float dist_to_rwy = sqrtf((a.x - rwy.x) * (a.x - rwy.x) +
                                                   (a.y - rwy.y) * (a.y - rwy.y));

                        // Standard ILS glideslope is 3 degrees
                        float expected_alt_ft = dist_to_rwy * 1000.0f * tanf(deg_to_rad(3.0f)) * 3.28084f;

                        // Store deviation for display
                        float deviation = a.altitude_ft - expected_alt_ft;

                        // Could auto-adjust altitude here if desired
                        break;
                    }
                }
            }

            const float max_alt_rate = 33.33f;
            const float alt_accel = 4.0f;
            const float max_speed_rate = 3.0f;
            const float speed_accel = 0.5f;

            // Altitude transition
            float alt_diff = a.target_altitude_ft - a.altitude_ft;
            if (fabs(alt_diff) > 1.0f)
            {
                float desired_rate = 0.0f;
                float distance_factor = sqrtf(2.0f * alt_accel * fabs(alt_diff));

                if (alt_diff > 0)
                    desired_rate = std::min(max_alt_rate, distance_factor);
                else
                    desired_rate = -std::min(max_alt_rate, distance_factor);

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

                float new_altitude = a.altitude_ft + a.altitude_rate_fps * dt;

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

            // Heading transition
            float heading_diff = angle_difference(a.target_heading_deg, a.heading_deg);
            if (fabs(heading_diff) > 0.5f)
            {
                float turn_rate = 2.5f * dt;
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

            // Speed transition
            float speed_diff = a.target_speed_kts - a.speed_kts;
            if (fabs(speed_diff) > 0.5f)
            {
                float desired_rate = 0.0f;
                float distance_factor = sqrtf(2.0f * speed_accel * fabs(speed_diff));

                if (speed_diff > 0)
                    desired_rate = std::min(max_speed_rate, distance_factor);
                else
                    desired_rate = -std::min(max_speed_rate, distance_factor);

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
                float zoom_factor = 1.0f + wheel * 0.15f;
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
        ImU32 bg = IM_COL32(0, 20, 0, 200);
        draw_list->AddRectFilled(win_pos, ImVec2(win_pos.x + win_size.x, win_pos.y + win_size.y), bg, 8.0f);

        auto world_to_screen = [&](float wx, float wy)
        {
            float scale = radius_max / (radar_range_km * zoom_level);
            float vx = wx - camera_x;
            float vy = wy - camera_y;
            return ImVec2(center.x + vx * scale, center.y - vy * scale);
        };

        // Draw rings
        const int rings = 4;
        for (int i = 1; i <= rings; ++i)
        {
            float km_radius = (radar_range_km * i) / rings;
            ImVec2 p0 = world_to_screen(0, 0);
            ImVec2 p1 = world_to_screen(km_radius, 0);
            float screen_r = fabs(p1.x - p0.x);
            draw_list->AddCircle(p0, screen_r, col_green, 128, 1.0f);

            float range_nm = km_radius * 0.539957f;
            std::ostringstream ss;
            ss << (int)range_nm << " nm";
            draw_list->AddText(ImVec2(p0.x + screen_r - 30, p0.y - 12), col_green, ss.str().c_str());
        }

        // Draw runways and ILS
        ImU32 runway_color = IM_COL32(100, 150, 255, 255);
        ImU32 ils_color = IM_COL32(80, 120, 200, 150);

        for (const auto& rwy : runways)
        {
            ImVec2 rwy_pos = world_to_screen(rwy.x, rwy.y);

            // Draw runway marker
            draw_list->AddCircleFilled(rwy_pos, 8.0f, runway_color);
            draw_list->AddText(ImVec2(rwy_pos.x + 12, rwy_pos.y - 8), runway_color, rwy.name.c_str());

            // Draw ILS localizer (6km line in approach direction)
            float ils_angle = deg_to_rad(rwy.heading_deg + 180.0f); // Opposite direction
            float ils_len = 6.0f;
            ImVec2 ils_end = world_to_screen(
                rwy.x + cosf(ils_angle) * ils_len,
                rwy.y + sinf(ils_angle) * ils_len
            );
            draw_list->AddLine(ils_end, rwy_pos, ils_color, 2.0f);
        }

        // Draw waypoints
        ImU32 waypoint_color = IM_COL32(255, 200, 100, 200);
        for (const auto& wp : waypoints)
        {
            ImVec2 wp_pos = world_to_screen(wp.x, wp.y);
            draw_list->AddCircle(wp_pos, 6.0f, waypoint_color, 12, 2.0f);
            draw_list->AddText(ImVec2(wp_pos.x + 10, wp_pos.y - 8), waypoint_color, wp.name.c_str());
        }

        // Radar sweep
        float sweep_angle = fmodf(ImGui::GetTime() * 0.8f * animation_speed, 2.0f * IM_PI);
        float sweep_length_km = radar_range_km;
        float sx = cosf(sweep_angle) * sweep_length_km;
        float sy = sinf(sweep_angle) * sweep_length_km;
        ImVec2 sweep_center = world_to_screen(0.0f, 0.0f);
        ImVec2 sweep_tip = world_to_screen(sx, sy);
        ImU32 sweep_color = IM_COL32(120, 255, 120, 255);
        draw_list->AddLine(sweep_center, sweep_tip, sweep_color, 2.0f);

        // Draw axes
        float axis_len_km = radar_range_km;
        ImVec2 left = world_to_screen(-axis_len_km, 0.0f);
        ImVec2 right = world_to_screen(+axis_len_km, 0.0f);
        draw_list->AddLine(left, right, col_green, 1.0f);
        ImVec2 down = world_to_screen(0.0f, -axis_len_km);
        ImVec2 up = world_to_screen(0.0f, +axis_len_km);
        draw_list->AddLine(down, up, col_green, 1.0f);
        ImVec2 c = world_to_screen(0.0f, 0.0f);
        draw_list->AddCircleFilled(c, 3.0f, col_green);

        // Draw aircraft
        for (size_t i = 0; i < aircraft.size(); ++i)
        {
            const Aircraft& a = aircraft[i];
            ImVec2 pos = world_to_screen(a.x, a.y);

            float margin = 100.0f;
            bool on_screen = (pos.x >= win_pos.x - margin && pos.x <= win_pos.x + win_size.x + margin &&
                            pos.y >= win_pos.y - margin && pos.y <= win_pos.y + win_size.y + margin);

            if (!on_screen) continue;

            bool in_conflict = false;
            for (auto& cf : conflicts)
            {
                if ((int)i == cf.first || (int)i == cf.second)
                {
                    in_conflict = true;
                    break;
                }
            }

            ImU32 blip_col;
            if (a.emergency != EMERGENCY_NONE)
                blip_col = IM_COL32(255, 0, 0, 255); // Red for emergency
            else if (a.is_overflight)
                blip_col = IM_COL32(150, 150, 150, 180); // Gray for overflight
            else if (in_conflict)
                blip_col = IM_COL32(255, 64, 64, 255);
            else
                blip_col = IM_COL32(255, 255, 0, 255);

            float blip_radius = (selected_index == (int)i) ? 6.0f : 4.0f;
            draw_list->AddCircleFilled(pos, blip_radius, blip_col);

            float head_rad = deg_to_rad(a.heading_deg);
            ImVec2 head_end(pos.x + cosf(head_rad) * 12.0f, pos.y - sinf(head_rad) * 12.0f);
            draw_list->AddLine(pos, head_end, IM_COL32(200, 200, 200, 180), 1.0f);

            std::ostringstream ss;
            ss << a.callsign << " " << endl << "FL"<<(int)(a.altitude_ft / 100) << " " << endl << a.squawk_code; // to replace endl,
            draw_list->AddText(ImVec2(pos.x + 8.0f, pos.y - 10.0f),
                              a.is_overflight ? IM_COL32(150, 150, 150, 180) : IM_COL32(180, 240, 180, 220),
                              ss.str().c_str());
        }

        // Draw conflict lines
        for (auto& cf : conflicts)
        {
            const Aircraft& a = aircraft[cf.first];
            const Aircraft& b = aircraft[cf.second];
            ImVec2 pa = world_to_screen(a.x, a.y);
            ImVec2 pb = world_to_screen(b.x, b.y);
            draw_list->AddLine(pa, pb, IM_COL32(255, 80, 80, 200), 2.0f);
            ImVec2 mid((pa.x + pb.x) * 0.5f, (pa.y + pb.y) * 0.5f);
            draw_list->AddText(ImVec2(mid.x + 4, mid.y + 4), IM_COL32(255, 120, 120, 220), "CONFLICT");
        }

        // Selection
        ImVec2 io_mouse = ImGui::GetMousePos();
        bool clicked = ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows) && ImGui::IsMouseClicked(0) && !shift_held;
        if (clicked)
        {
            float best_d = 999999.0f;
            int best_idx = -1;
            for (size_t i = 0; i < aircraft.size(); ++i)
            {
                if (aircraft[i].is_overflight) continue; // Can't select overflights

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
        ImGui::Text("Zoom: %.1fx", zoom_level);
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

            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "AIRCRAFT CONTROL");
            ImGui::Separator();

            ImGui::SetWindowFontScale(2.0f);
            ImGui::Text("Callsign %s", sel.callsign.c_str());
            ImGui::SetWindowFontScale(1.0f);

            ImGui::Text("Squawk: %s", sel.squawk_code.c_str());

            // Emergency display
            if (sel.emergency != EMERGENCY_NONE)
            {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
                ImGui::Text("EMERGENCY: %s", sel.emergency_message.c_str());
                int mins = (int)(sel.emergency_timer / 60.0f);
                int secs = (int)(sel.emergency_timer) % 60;
                ImGui::Text("Time to land: %02d:%02d", mins, secs);
                ImGui::PopStyleColor();
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
                            ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "TOO HIGH (%.0f ft)", deviation);
                        else if (deviation < -200.0f)
                            ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "TOO LOW (%.0f ft)", -deviation);
                        else
                            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "ON GLIDESLOPE");
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

            // Altitude controls
            ImGui::Text("Altitude: %.0f ft", sel.altitude_ft);
            float display_target = (sel.command_delay > 0.0f) ? sel.pending_altitude_ft : sel.target_altitude_ft;
            if (fabs(display_target - sel.altitude_ft) > 10.0f)
            {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f), "-> %.0f ft", display_target);
            }

            if (ImGui::Button("-1000##alt1", ImVec2(100, 0)))
            {
                float old = sel.altitude_ft;
                sel.pending_altitude_ft = std::max(0.0f, sel.pending_altitude_ft - 1000.0f);
                float diff = fabs(sel.pending_altitude_ft - old);
                std::ostringstream r;
                r << "Roger, descending " << (int)diff << " feet";
                sel.setCommand(r.str());
            }
            ImGui::SameLine();
            if (ImGui::Button("-100##alt2", ImVec2(100, 0)))
            {
                float old = sel.altitude_ft;
                sel.pending_altitude_ft = std::max(0.0f, sel.pending_altitude_ft - 100.0f);
                float diff = fabs(sel.pending_altitude_ft - old);
                std::ostringstream r;
                r << "Roger, descending " << (int)diff << " feet";
                sel.setCommand(r.str());
            }
            ImGui::SameLine();
            if (ImGui::Button("+100##alt3", ImVec2(100, 0)))
            {
                float old = sel.altitude_ft;
                sel.pending_altitude_ft += 100.0f;
                float diff = fabs(sel.pending_altitude_ft - old);
                std::ostringstream r;
                r << "Roger, climbing " << (int)diff << " feet";
                sel.setCommand(r.str());
            }
            ImGui::SameLine();
            if (ImGui::Button("+1000##alt4", ImVec2(100, 0)))
            {
                float old = sel.altitude_ft;
                sel.pending_altitude_ft += 1000.0f;
                float diff = fabs(sel.pending_altitude_ft - old);
                std::ostringstream r;
                r << "Roger, climbing " << (int)diff << " feet";
                sel.setCommand(r.str());
            }

            ImGui::Separator();
            ImGui::Text("Heading: %.0f°", fmodf(450.0f - sel.heading_deg, 360.0f));
            float display_heading = (sel.command_delay > 0.0f) ? sel.pending_heading_deg : sel.target_heading_deg;
            if (fabs(angle_difference(display_heading, sel.heading_deg)) > 1.0f)
            {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f), "-> %.0f°", fmodf(450.0f - display_heading, 360.0f));
            }

            if (ImGui::Button("-5°##hdg1", ImVec2(210, 0)))
            {
                float old = sel.heading_deg;
                sel.pending_heading_deg = fmodf(sel.pending_heading_deg + 5.0f, 360.0f);
                float diff = angle_difference(sel.pending_heading_deg, old);
                if (diff < 0) diff = -diff;
                std::ostringstream r;
                r << "Roger, turning left " << (int)diff << " degrees";
                sel.setCommand(r.str());
            }
            ImGui::SameLine();
            if (ImGui::Button("+5°##hdg2", ImVec2(210, 0)))
            {
                float old = sel.heading_deg;
                sel.pending_heading_deg = fmodf(sel.pending_heading_deg - 5.0f + 360.0f, 360.0f);
                float diff = angle_difference(old, sel.pending_heading_deg);
                if (diff < 0) diff = -diff;
                std::ostringstream r;
                r << "Roger, turning right " << (int)diff << " degrees";
                sel.setCommand(r.str());
            }

            ImGui::Separator();
            ImGui::Text("Speed: %.0f kts", sel.speed_kts);
            float display_speed = (sel.command_delay > 0.0f) ? sel.pending_speed_kts : sel.target_speed_kts;
            if (fabs(display_speed - sel.speed_kts) > 1.0f)
            {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f), "-> %.0f kts", display_speed);
            }

            if (ImGui::Button("-5 kts##spd1", ImVec2(210, 0)))
            {
                float old = sel.speed_kts;
                sel.pending_speed_kts = std::max(0.0f, sel.pending_speed_kts - 5.0f);
                float diff = fabs(sel.pending_speed_kts - old);
                std::ostringstream r;
                r << "Roger, reducing speed " << (int)diff << " knots";
                sel.setCommand(r.str());
            }
            ImGui::SameLine();
            if (ImGui::Button("+5 kts##spd2", ImVec2(210, 0)))
            {
                float old = sel.speed_kts;
                sel.pending_speed_kts += 5.0f;
                float diff = fabs(sel.pending_speed_kts - old);
                std::ostringstream r;
                r << "Roger, increasing speed " << (int)diff << " knots";
                sel.setCommand(r.str());
            }

            // TEXT COMMAND INPUT
            static char command_buf[128] = "";
            static std::string command_feedback = "";
            static std::vector<std::string> history;
            static int history_index = -1;

            ImGui::Separator();
            ImGui::Text("Command Input:");

            ImGuiInputTextFlags flags = ImGuiInputTextFlags_EnterReturnsTrue;

            if (ImGui::InputText("##cmd", command_buf, IM_ARRAYSIZE(command_buf), flags))
            {
                std::string cmd = command_buf;
                command_buf[0] = '\0';
                command_feedback = "";

                history.push_back(cmd);
                history_index = -1;

                std::string u;
                for (char c : cmd) u += (char)toupper(c);

                auto extract_number = [&](const std::string& s) -> float
                {
                    for (int i = 0; i < (int)s.size(); i++)
                        if ((s[i] >= '0' && s[i] <= '9'))
                            return atof(s.c_str() + i);
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
                else if (u.rfind("FL", 0) == 0)
                {
                    float fl = extract_number(u);
                    sel.pending_altitude_ft = fl * 100.0f;
                    std::ostringstream response;
                    response << "Roger, climbing to flight level " << (int)fl;
                    sel.setCommand(response.str());
                    command_feedback = "Flight level assigned.";
                }
                else if (u.rfind("HDG ", 0) == 0 || u.rfind("HEADING ", 0) == 0)
                {
                    float hdg = extract_number(u);
                    sel.pending_heading_deg = fmodf(450.0f - hdg, 360.0f);
                    std::ostringstream response;
                    response << "Roger, turning to heading " << (int)hdg;
                    sel.setCommand(response.str());
                    command_feedback = "Heading assigned.";
                }
                else if (u.rfind("TURN LEFT ", 0) == 0)
                {
                    float deg = extract_number(u);
                    sel.pending_heading_deg = fmodf(sel.pending_heading_deg + deg, 360.0f);
                    std::ostringstream response;
                    response << "Roger, turning left " << (int)deg << " degrees";
                    sel.setCommand(response.str());
                    command_feedback = "Turning left.";
                }
                else if (u.rfind("TURN RIGHT ", 0) == 0)
                {
                    float deg = extract_number(u);
                    sel.pending_heading_deg = fmodf(sel.pending_heading_deg - deg + 360.0f, 360.0f);
                    std::ostringstream response;
                    response << "Roger, turning right " << (int)deg << " degrees";
                    sel.setCommand(response.str());
                    command_feedback = "Turning right.";
                }
                else if (u.rfind("ALT ", 0) == 0 || u.rfind("ALTITUDE ", 0) == 0)
                {
                    float alt = extract_number(u);
                    sel.pending_altitude_ft = alt;
                    std::ostringstream response;
                    if (alt > sel.altitude_ft)
                        response << "Roger, climbing to " << (int)alt << " feet";
                    else
                        response << "Roger, descending to " << (int)alt << " feet";
                    sel.setCommand(response.str());
                    command_feedback = "Altitude assigned.";
                }
                else if (u.rfind("CLIMB ", 0) == 0)
                {
                    float alt = extract_number(u);
                    sel.pending_altitude_ft = alt;
                    std::ostringstream response;
                    response << "Roger, climb and maintain " << (int)alt;
                    sel.setCommand(response.str());
                    command_feedback = "Climbing.";
                }
                else if (u.rfind("DESCEND ", 0) == 0)
                {
                    float alt = extract_number(u);
                    sel.pending_altitude_ft = alt;
                    std::ostringstream response;
                    response << "Roger, descend and maintain " << (int)alt;
                    sel.setCommand(response.str());
                    command_feedback = "Descending.";
                }
                else if (u.rfind("SPD ", 0) == 0 || u.rfind("SPEED ", 0) == 0)
                {
                    float spd = extract_number(u);
                    sel.pending_speed_kts = spd;
                    std::ostringstream response;
                    response << "Roger, adjusting speed to " << (int)spd << " knots";
                    sel.setCommand(response.str());
                    command_feedback = "Speed assigned.";
                }
                else
                {
                    command_feedback = "Invalid or unknown command.";
                }
            }

            if (!command_feedback.empty())
            {
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

        // Renderja nie ma
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