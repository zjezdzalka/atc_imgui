#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <stdio.h>
#include <math.h>
#include <GLFW/glfw3.h>

#ifndef IM_PI
#define IM_PI 3.14159265358979323846f
#endif

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

int main(int, char**)
{
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
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_NoSavedSettings;

        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
        ImVec2 winSize(500, 500);
        ImGui::SetNextWindowSize(winSize, ImGuiCond_Always);

        ImGui::Begin("Radar", nullptr, window_flags);

        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 win_pos = ImGui::GetWindowPos();
        ImVec2 win_size = ImGui::GetWindowSize();
        ImVec2 center(win_pos.x + win_size.x * 0.5f, win_pos.y + win_size.y * 0.5f);
        float radius_max = (win_size.x < win_size.y ? win_size.x : win_size.y) * 0.45f;

        ImU32 col_green = IM_COL32(0, 255, 0, 220);
        ImU32 bg = IM_COL32(0, 40, 0, 200);
        draw_list->AddRectFilled(win_pos, ImVec2(win_pos.x + win_size.x, win_pos.y + win_size.y), bg, 8.0f);

        // Draw rings
        const int rings = 4;
        for (int i = 1; i <= rings; ++i)
        {
            float r = (radius_max * i) / rings;
            draw_list->AddCircle(center, r, col_green, 64, 1.5f);
        }

        // Crosshair
        draw_list->AddLine(ImVec2(center.x - radius_max, center.y), ImVec2(center.x + radius_max, center.y), col_green, 1.0f);
        draw_list->AddLine(ImVec2(center.x, center.y - radius_max), ImVec2(center.x, center.y + radius_max), col_green, 1.0f);
        draw_list->AddCircleFilled(center, 4.0f, col_green);

        // === ROTATING SWEEP LINE ===
        float time = (float)ImGui::GetTime();      // current time in seconds
        float speed = 1.5f;                        // rotation speed (radians per second)
        float angle = fmodf(time * speed, 2.0f * IM_PI); // sweep angle
        float sweep_length = radius_max;

        // endpoint of the line
        ImVec2 sweep_end(center.x + cosf(angle) * sweep_length,
            center.y + sinf(angle) * sweep_length);

        // Draw the sweep line (bright head)
        ImU32 sweep_color = IM_COL32(0, 255, 0, 255);
        draw_list->AddLine(center, sweep_end, sweep_color, 2.0f);

        // Draw faded trail (like radar glow)
        const int trail_segments = 20;
        for (int i = 1; i < trail_segments; ++i)
        {
            float fade = 1.0f - (float)i / trail_segments;
            float trail_angle = angle - (i * 0.05f);
            ImVec2 tail(center.x + cosf(trail_angle) * sweep_length,
                center.y + sinf(trail_angle) * sweep_length);
            ImU32 trail_color = IM_COL32(0, (int)(200 * fade), 0, (int)(150 * fade));
            draw_list->AddLine(center, tail, trail_color, 1.0f);
        }

        ImGui::Dummy(ImVec2(0, 0));
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
