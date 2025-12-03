// main.cpp
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <stdio.h>
#include <math.h>
#include <GLFW/glfw3.h>

#include <vector>
#include <string>
#include <cstdlib>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <algorithm>

#ifndef IM_PI
#define IM_PI 3.14159265358979323846f
#endif

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

struct Aircraft
{
    std::string callsign;
    // world position in kilometers relative to radar center (x east, y north)
    float x = 0.0f;
    float y = 0.0f;
    float altitude_ft = 10000.0f;
    float heading_deg = 0.0f; // 0 = east, +counterclockwise (so 90 = north)
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

static float deg_to_rad(float d) { return d * (IM_PI / 180.0f); }
static float rad_to_deg(float r) { return r * (180.0f / IM_PI); }

int main(int, char**)
{
    srand((unsigned)time(nullptr));

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

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

    GLFWwindow* window = glfwCreateWindow(1024, 768, "Air Traffic Controller", NULL, NULL);
    if (window == NULL)
        return 1;
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
    const float radar_range_km = 80.0f; // total radius in kilometers
    for (int i = 0; i < initial_count; ++i)
    {
        Aircraft a;
        std::ostringstream ss;
        ss << "AC" << std::setw(2) << std::setfill('0') << (i + 1);
        a.callsign = ss.str();

        // random position in circle
        float r = ((float)rand() / RAND_MAX) * radar_range_km;
        float theta = ((float)rand() / RAND_MAX) * 2.0f * IM_PI;
        a.x = cosf(theta) * r;
        a.y = sinf(theta) * r;

        a.altitude_ft = 2000.0f + (rand() % 38000);
        a.heading_deg = (float)(rand() % 360);
        a.speed_kts = 130.0f + (rand() % 300); // 130..430 kts
        a.selected = false;
        aircraft.push_back(a);
    }
    int selected_index = -1;

    double last_time = glfwGetTime();

    while (!glfwWindowShouldClose(window))
    {
        // --- Time ---
        double now = glfwGetTime();
        float dt = (float)(now - last_time);
        if (dt <= 0.0f) dt = 1.0f / 60.0f;
        last_time = now;

        // Poll events
        glfwPollEvents();

        // Update simulation (simple kinematic: x,y in km, speed in knots -> km/s)
        for (auto& a : aircraft)
        {
            // knots to km/h : 1 kt = 1.852 km/h
            float speed_kmh = a.speed_kts * 1.852f;
            float speed_kms = speed_kmh / 3600.0f; // km per second
            float ang = deg_to_rad(a.heading_deg);
            a.x += cosf(ang) * speed_kms * dt;
            a.y += sinf(ang) * speed_kms * dt;
        }

        // Basic conflict detection: if two aircraft are within 5 km horizontally and 1000 ft vertically
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
                float dz = fabsf(aircraft[i].altitude_ft - aircraft[j].altitude_ft);
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

        // Radar window layout
        ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_NoSavedSettings;

        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
        ImVec2 winSize(700, 700);
        ImGui::SetNextWindowSize(winSize, ImGuiCond_Always);

        ImGui::Begin("Radar", nullptr, window_flags);

        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 win_pos = ImGui::GetWindowPos();
        ImVec2 win_size = ImGui::GetWindowSize();
        ImVec2 content_min = ImGui::GetWindowContentRegionMin();
        ImVec2 content_max = ImGui::GetWindowContentRegionMax();
        ImVec2 center(win_pos.x + win_size.x * 0.5f, win_pos.y + win_size.y * 0.5f);
        float radius_max = (win_size.x < win_size.y ? win_size.x : win_size.y) * 0.45f;

        ImU32 col_green = IM_COL32(0, 255, 0, 200);
        ImU32 bg = IM_COL32(0, 40, 0, 200);
        draw_list->AddRectFilled(win_pos, ImVec2(win_pos.x + win_size.x, win_pos.y + win_size.y), bg, 8.0f);

        // Draw rings (range markers) and labels
        const int rings = 4;
        for (int i = 1; i <= rings; ++i)
        {
            float r = (radius_max * i) / rings;
            draw_list->AddCircle(center, r, col_green, 64, 1.0f);
            // label radius in NM (convert km -> nm: 1 km = 0.539957 nm)
            float range_km = (radar_range_km * i) / rings;
            float range_nm = range_km * 0.539957f;
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(0) << range_nm << " nm";
            ImVec2 label_pos(center.x + r - 40, center.y - 12);
            draw_list->AddText(label_pos, col_green, ss.str().c_str());
        }

        // Crosshair
        draw_list->AddLine(ImVec2(center.x - radius_max, center.y), ImVec2(center.x + radius_max, center.y), col_green,
                           1.0f);
        draw_list->AddLine(ImVec2(center.x, center.y - radius_max), ImVec2(center.x, center.y + radius_max), col_green,
                           1.0f);
        draw_list->AddCircleFilled(center, 3.0f, col_green);

        // Rotating sweep line
        float time = (float)ImGui::GetTime();
        float speed = 0.8f; // radians per second
        float angle = fmodf(time * speed, 2.0f * IM_PI);
        float sweep_length = radius_max;
        ImVec2 sweep_end(center.x + cosf(angle) * sweep_length, center.y + sinf(angle) * sweep_length);
        ImU32 sweep_color = IM_COL32(120, 255, 120, 255);
        draw_list->AddLine(center, sweep_end, sweep_color, 2.0f);

        // Sweep trail glow
        const int trail_segments = 30;
        for (int i = 1; i < trail_segments; ++i)
        {
            float fade = 1.0f - (float)i / trail_segments;
            float trail_angle = angle - (i * 0.02f);
            ImVec2 tail(center.x + cosf(trail_angle) * sweep_length, center.y + sinf(trail_angle) * sweep_length);
            ImU32 trail_color = IM_COL32(0, (int)(200 * fade), 0, (int)(120 * fade));
            draw_list->AddLine(center, tail, trail_color, 1.0f);
        }

        // Helper: world (km) -> screen pos
        auto world_to_screen = [&](float wx, float wy) -> ImVec2
        {
            // scale: pixels per km
            float scale = radius_max / radar_range_km;
            return ImVec2(center.x + wx * scale, center.y - wy * scale); // y is inverted: world +y north -> screen -y
        };

        // Draw aircraft blips
        for (size_t i = 0; i < aircraft.size(); ++i)
        {
            const Aircraft& a = aircraft[i];
            ImVec2 pos = world_to_screen(a.x, a.y);

            // skip drawing if outside radar circle
            float dx = pos.x - center.x;
            float dy = pos.y - center.y;
            float dist = sqrtf(dx * dx + dy * dy);
            if (dist > radius_max + 8.0f)
                continue;

            // Check if in any conflict
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

            // draw heading vector small line
            float head_rad = deg_to_rad(a.heading_deg);
            ImVec2 head_end(pos.x + cosf(head_rad) * 12.0f, pos.y - sinf(head_rad) * 12.0f);
            draw_list->AddLine(pos, head_end, IM_COL32(200, 200, 200, 180), 1.0f);

            // draw callsign text offset
            std::ostringstream ss;
            ss << a.callsign << " " << (int)(a.altitude_ft) << "ft";
            draw_list->AddText(ImVec2(pos.x + 8.0f, pos.y - 10.0f), IM_COL32(180, 240, 180, 220), ss.str().c_str());
        }

        // Draw conflict lines between conflicting pairs
        for (auto& c : conflicts)
        {
            const Aircraft& a = aircraft[c.first];
            const Aircraft& b = aircraft[c.second];
            ImVec2 pa = world_to_screen(a.x, a.y);
            ImVec2 pb = world_to_screen(b.x, b.y);
            draw_list->AddLine(pa, pb, IM_COL32(255, 80, 80, 200), 2.0f);
            // midpoint label
            ImVec2 mid((pa.x + pb.x) * 0.5f, (pa.y + pb.y) * 0.5f);
            draw_list->AddText(ImVec2(mid.x + 4, mid.y + 4), IM_COL32(255, 120, 120, 220), "CONFLICT");
        }

        // Selection: when mouse clicked inside radar window, pick nearest blip
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
            // toggle selection
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
        ImGui::End(); // Radar window

        // --- Aircraft control panel (if selected) ---
        if (selected_index >= 0 && selected_index < (int)aircraft.size())
        {
            Aircraft& sel = aircraft[selected_index];
            std::string callSignLabel = "Callsign " + sel.callsign;
            //ustawianie sztywno, ale wtedy nie jest floating i draggable ts
            //ImGui::SetNextWindowPos(ImVec2(50, 50));
            ImGui::Begin("Aircraft Control");

            ImGui::SetWindowFontScale(2.0f);
            ImGui::Text(callSignLabel.c_str());
            ImGui::SetWindowFontScale(1.0f);

            ImGui::Dummy(ImVec2(0, 10));
            ImGui::Separator();

            // === ALTITUDE ===
            ImGui::Text("Altitude: %.0f ft", sel.altitude_ft);

            if (ImGui::Button("-100 ft"))
            {
                sel.altitude_ft -= 100.0f;
                if (sel.altitude_ft < 0) sel.altitude_ft = 0;
            }
            ImGui::SameLine();
            if (ImGui::Button("+100 ft"))
            {
                sel.altitude_ft += 100.0f;
            }


            // === HEADING ===
            ImGui::Separator();
            ImGui::Text("Heading: %.0f°", sel.heading_deg);

            if (ImGui::Button("-5°"))
            {
                sel.heading_deg -= 5.0f;
                if (sel.heading_deg < 0) sel.heading_deg += 360.0f;
            }
            ImGui::SameLine();
            if (ImGui::Button("+5°"))
            {
                sel.heading_deg += 5.0f;
                if (sel.heading_deg >= 360.0f) sel.heading_deg -= 360.0f;
            }


            // === SPEED ===
            ImGui::Separator();
            ImGui::Text("Speed: %.0f kts", sel.speed_kts);

            if (ImGui::Button("-5 kts"))
            {
                sel.speed_kts -= 5.0f;
                if (sel.speed_kts < 0) sel.speed_kts = 0;
            }
            ImGui::SameLine();
            if (ImGui::Button("+5 kts"))
            {
                sel.speed_kts += 5.0f;
            }

            //---------------------------------------------------------------
            // TEXT COMMAND INPUT SYSTEM (DROP-IN REPLACEMENT)
            //---------------------------------------------------------------

            static char command_buf[128] = "";
            static std::string command_feedback = "";

            // ---- Command history ----
            static std::vector<std::string> history;
            static int history_index = -1;

            // ---- Autocomplete suggestions ----
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

            // === INPUT TEXT BOX ===
            ImGui::Separator();
            ImGui::Text("Command Input:");

            ImGuiInputTextFlags flags =
                ImGuiInputTextFlags_EnterReturnsTrue |
                ImGuiInputTextFlags_CallbackHistory |
                ImGuiInputTextFlags_CallbackCompletion;

            // Callback for history + autocomplete
            ImGuiInputTextCallback text_callback = [](ImGuiInputTextCallbackData* data) -> int
            {
                auto* hist = reinterpret_cast<std::vector<std::string>*>(data->UserData);

                // HISTORY (Up/Down)
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

                // TAB COMPLETION
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

                // Convert to uppercase
                std::string u;
                for (char c : cmd) u += (char)toupper(c);

                Aircraft& selected = aircraft[selected_index];

                // -----------------------------------------------------
                // COMMAND PARSER (supports realistic ATC syntax)
                // -----------------------------------------------------

                auto extract_number = [&](const std::string& s) -> float
                {
                    for (int i = 0; i < (int)s.size(); i++)
                        if ((s[i] >= '0' && s[i] <= '9'))
                            return atof(s.c_str() + i);
                    return 0.0f;
                };

                // Flight levels: FL180 = 18,000 ft
                if (u.rfind("FL", 0) == 0)
                {
                    float fl = extract_number(u);
                    selected.altitude_ft = fl * 100.0f;
                    command_feedback = "Altitude set (flight level).";
                }
                else if (u.rfind("HDG ", 0) == 0 || u.rfind("HEADING ", 0) == 0)
                {
                    selected.heading_deg = fmodf(extract_number(u), 360.0f);
                    command_feedback = "Heading updated.";
                }
                else if (u.rfind("TURN LEFT ", 0) == 0)
                {
                    selected.heading_deg = fmodf(selected.heading_deg + extract_number(u), 360.0f);
                    command_feedback = "Turning left.";
                }
                else if (u.rfind("TURN RIGHT ", 0) == 0)
                {
                    selected.heading_deg = fmodf(selected.heading_deg - extract_number(u) + 360.0f, 360.0f);
                    command_feedback = "Turning right.";
                }
                else if (u.rfind("TURN LEFT HEADING ", 0) == 0)
                {
                    selected.heading_deg = fmodf(extract_number(u), 360.0f);
                    command_feedback = "Turn left to heading.";
                }
                else if (u.rfind("TURN RIGHT HEADING ", 0) == 0)
                {
                    selected.heading_deg = fmodf(extract_number(u), 360.0f);
                    command_feedback = "Turn right to heading.";
                }
                else if (u.rfind("ALT ", 0) == 0 || u.rfind("ALTITUDE ", 0) == 0)
                {
                    selected.altitude_ft = extract_number(u);
                    command_feedback = "Altitude updated.";
                }
                else if (u.rfind("CLIMB AND MAINTAIN ", 0) == 0 ||
                    u.rfind("CLIMB ", 0) == 0)
                {
                    selected.altitude_ft += extract_number(u);
                    command_feedback = "Climbing.";
                }
                else if (u.rfind("DESCEND AND MAINTAIN ", 0) == 0 ||
                    u.rfind("DESCEND ", 0) == 0)
                {
                    selected.altitude_ft = std::max(0.0f, selected.altitude_ft - extract_number(u));
                    command_feedback = "Descending.";
                }
                else if (u.rfind("SPD ", 0) == 0 || u.rfind("SPEED ", 0) == 0)
                {
                    selected.speed_kts = extract_number(u);
                    command_feedback = "Speed updated.";
                }
                else
                {
                    command_feedback = "Invalid or unknown command.";
                }
            }

            // -----------------------------------------------------
            // COMMAND FEEDBACK MESSAGE
            // -----------------------------------------------------
            if (!command_feedback.empty())
            {
                ImGui::TextColored(ImVec4(0.7f, 1.0f, 0.7f, 1.0f),
                                   "%s", command_feedback.c_str());
            }

            // -----------------------------------------------------
            // COMMAND CHEAT-SHEET
            // -----------------------------------------------------
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


        // small panel to add aircraft
        ImGui::Begin("Control");
        if (ImGui::Button("Spawn Random AC"))
        {
            Aircraft a;
            std::ostringstream ss;
            ss << "AC" << std::setw(2) << std::setfill('0') << (aircraft.size() + 1);
            a.callsign = ss.str();
            float r = ((float)rand() / RAND_MAX) * radar_range_km;
            float theta = ((float)rand() / RAND_MAX) * 2.0f * IM_PI;
            a.x = cosf(theta) * r;
            a.y = sinf(theta) * r;
            a.altitude_ft = 2000.0f + (rand() % 38000);
            a.heading_deg = (float)(rand() % 360);
            a.speed_kts = 130.0f + (rand() % 300);
            aircraft.push_back(a);
        }
        ImGui::Text("Aircraft: %zu", aircraft.size());
        ImGui::Text("Conflicts: %zu", conflicts.size());
        ImGui::End();


        // Render
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
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
