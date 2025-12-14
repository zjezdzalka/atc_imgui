//
// Created by Natan on 12/14/2025.
//

#include "runway.h"

std::vector<Runway> createRunways()
{
    std::vector<Runway> runways;
    runways.push_back({"09", 180.0f, -1.0f, -1.0f});
    runways.push_back({"27", 0.0f, 1.0f, 1.0f});
    return runways;
}