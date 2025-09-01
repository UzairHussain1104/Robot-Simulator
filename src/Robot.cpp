#include "Robot.h"
#include "SensorData.h"
#include <cmath>

Robot::Robot() {
    shape.setRadius(20.f);
    shape.setOrigin({20.f,20.f});
    shape.setFillColor(sf::Color::Green);

    goal.setRadius(20.f);
    goal.setFillColor(sf::Color::Red);
    goal.setOrigin({20.f, 20.f});

}


bool Robot::goalReached() {
    // Get centers
    sf::Vector2f posA = shape.getPosition();
    sf::Vector2f posB = goal.getPosition();

    // Compute distance squared between centers
    float dx = posA.x - posB.x;
    float dy = posA.y - posB.y;
    float distSq = dx * dx + dy * dy;

    // Compare with sum of radii
    float radiusSum = shape.getRadius() + goal.getRadius();
    return distSq < (radiusSum * radiusSum);
}

void Robot::update(float dt) {
    if (!isMoving) return;
    
    // Move forward
    float dx = cos(heading) * speed * dt;
    float dy = sin(heading) * speed * dt;
    shape.move({dx, dy});
    
    // stop moving if goal has been reached
    if(goalReached()) isMoving = false;
}

void Robot::setHeading(float dt){
    
    if (!isMoving) return;
    
    // Vector toward goal
    sf::Vector2f toGoal = goal.getPosition() - shape.getPosition();
    float desiredAngle = atan2(toGoal.y, toGoal.x);

    // Smallest angle difference (radians)
    float angleDiff = desiredAngle - heading;
    if (angleDiff > M_PI) angleDiff -= 2 * M_PI;
    if (angleDiff < -M_PI) angleDiff += 2 * M_PI;

    // Check front rays
    float turn = 0.f;
    float minDist = sensorData.maxRange;
    int minIndex = 0;

    for (int i = 0; i < sensorData.numRays; i++) {
        if (sensorData.distances[i] < minDist) {
            minDist = sensorData.distances[i];
            minIndex = i;
        }
    }

    if (minDist < 70.f) {
        // Obstacle avoidance: turn away from closest ray
        float rayCenter = (sensorData.numRays - 1) / 2.f;
        if (minIndex < rayCenter) turn = 1.f;   // obstacle on left, turn right
        else turn = -1.f;                       // obstacle on right, turn left
        heading += turn * 6.f * dt;            
    } else {
        // Turn toward goal
        heading += angleDiff * 2.f * dt;
    }
}