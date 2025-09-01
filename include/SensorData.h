
#pragma once
#include <vector>
#include <SFML/System/Vector2.hpp>

struct SensorData {
    std::vector<float> distances; // distance per ray
    float maxRange = 300.f;       // max sensing distance (pixels)
    int numRays = 32;             

    std::vector<sf::Vector2f> rayDirections;

};