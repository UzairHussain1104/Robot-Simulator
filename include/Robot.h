#pragma once
#include "SensorData.h"
#include <SFML/Graphics.hpp>

class Robot {
public:
    sf::CircleShape shape;
    float speed = 150.f;  // pixels per second
    sf::CircleShape goal;
    float heading = 0.f;
    SensorData sensorData;
    bool isMoving = true;

    Robot();

    void update(float dt);
    void setHeading(float dt);
    bool goalReached();

};