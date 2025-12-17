//
// Created by Natan on 12/14/2025.
//

#include "emergency.h"
#include "emergency_types.h"


#include <imgui.h>
#include <algorithm>
#include <cstdlib>
#include <cmath>

void DrawEmergencyPanel(const Aircraft& sel)
{
    // Does not show when no emergency detected
    if (sel.emergency == EMERGENCY_NONE)
        return;

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));

    ImGui::Text("EMERGENCY: %s", sel.emergency_message.c_str());

    int mins = (int)(sel.emergency_timer / 60.0f);
    int secs = (int)(sel.emergency_timer) % 60;
    ImGui::Text("Time to land: %02d:%02d", mins, secs);

    float max_time = 0.0f;
    switch (sel.emergency)
    {
    case EMERGENCY_LOW_FUEL:
        max_time = 600.0f;
        break;
    case EMERGENCY_MEDICAL:
        max_time = 1200.0f;
        break;
    case EMERGENCY_ENGINE_FAILURE:
        max_time = 300.0f;
        break;
    case EMERGENCY_HYDRAULIC:
        max_time = 480.0f;
        break;
    default:
        max_time = 600.0f;
        break;
    }

    float progress = 1.0f - (sel.emergency_timer / max_time);
    progress = std::clamp(progress, 0.0f, 1.0f);

    ImGui::ProgressBar(progress, ImVec2(-1, 20));

    if (sel.emergency_timer < 60.0f)
    {
        ImGui::TextColored(
            ImVec4(1.0f, 0.0f, 0.0f, 1.0f),
            "NO TIME LEFT! LAND IMMEDIATELY!"
        );
    }

    ImGui::PopStyleColor();
}

void GenerateRandomEmergency(std::vector<Aircraft>& aircraft, float dt, float& timer, float interval)
{
    // Update timer
    timer += dt;
    if (timer < interval) return;
    timer = 0.0f;

    // 5% chance to generate a random emergency
    if (rand() % 100 >= 5 || aircraft.empty()) return;

    // Find aircraft without current emergency, not overflight, and not crashed
    std::vector<int> candidates;
    for (int i = 0; i < (int)aircraft.size(); ++i)
    {
        if (aircraft[i].emergency == EMERGENCY_NONE &&
            !aircraft[i].is_overflight &&
            !aircraft[i].is_crashed)
        {
            candidates.push_back(i);
        }
    }

    if (candidates.empty()) return;

    // Pick a random aircraft
    int random_index = candidates[rand() % candidates.size()];
    Aircraft& random_ac = aircraft[random_index];

    // Assign a random emergency
    int emergency_type = rand() % 4;
    random_ac.emergency = (EmergencyType)(emergency_type + 1);

    float dist_to_airport = std::sqrt(random_ac.x * random_ac.x + random_ac.y * random_ac.y);

    switch (random_ac.emergency)
    {
        case EMERGENCY_LOW_FUEL:
            random_ac.emergency_timer = 480.0f + dist_to_airport * 16.0f;
            random_ac.emergency_message = "Low fuel - requesting priority landing";
            random_ac.squawk_code = "7700";
            break;
        case EMERGENCY_MEDICAL:
            random_ac.emergency_timer = 960.0f + dist_to_airport * 24.0f;
            random_ac.emergency_message = "Medical emergency on board";
            random_ac.squawk_code = "7700";
            break;
        case EMERGENCY_ENGINE_FAILURE:
            random_ac.emergency_timer = 300.0f + dist_to_airport * 8.0f;
            random_ac.emergency_message = "Engine failure - declaring emergency";
            random_ac.squawk_code = "7700";
            break;
        case EMERGENCY_HYDRAULIC:
            random_ac.emergency_timer = 640.0f + dist_to_airport * 20.0f;
            random_ac.emergency_message = "Hydraulic system failure";
            random_ac.squawk_code = "7700";
            break;
        default:
            break;
    }

    // Notify player
    random_ac.setCommand("MAYDAY MAYDAY MAYDAY! " + random_ac.emergency_message);
}