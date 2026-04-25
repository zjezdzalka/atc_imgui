//
// Created by Natan on 12/14/2025.
//

#include "runway.h"
std::vector<Runway> createRunways()
{
    std::vector<Runway> runways;

    // RWY 09: Faces East (0° math). Aircraft fly East (0°) to land.
    // Position: West side of airport at x = -25km
    runways.push_back({"09", 0.0f, -6.0f, -5.0f});


    runways.push_back({"21", 240.0f, -8.0f, 1.0f});

    // RWY 27: Faces West (180° math). Aircraft fly West (180°) to land.
    // Position: East side of airport at x = 25km
    runways.push_back({"27", 180.0f, -1.0f, -2.0f});

    return runways;
}