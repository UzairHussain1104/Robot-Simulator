#include "World.h"
#include <cmath>
#include <stdlib.h>
#include <cstdlib>
#include <ctime>


World::World(int numStationary, int numMoving, int numRobots) {
    std::srand(static_cast<unsigned>(std::time(nullptr)));
    
    // Stationary objects
    for (int i = 0; i < numStationary; i++) {
        sf::RectangleShape obs({250.f, 150.f});
        obs.setFillColor(sf::Color::Blue);

        // Try until no overlap
        bool placed = false;
        while (!placed) {
            obs.setPosition({
                static_cast<float>(rand() % 1500),
                static_cast<float>(rand() % 1100)
            });
            if (!overlapsAny(obs, obstacles)) {
                placed = true;
            }
        }

        obstacles.push_back(obs);
    }

    // Moving obstacles
    for (int i = 0; i < numMoving; i++) {
        sf::RectangleShape mob({50.f, 50.f});
        mob.setFillColor(sf::Color::Red);

        bool placed = false;
        while (!placed) {
            mob.setPosition({
                static_cast<float>(rand() % 1500),
                static_cast<float>(rand() % 1100)
            });
            if (!overlapsAny(mob, obstacles)) {
                placed = true;
            }
        }

        obstacles.push_back(mob);
        movingObstacles.push_back({obstacles.size()-1, {100.f, 100.f}});
    }

    // Robots
    for (int i = 0; i < numRobots; i++) {
        Robot robot;
        bool placed = false;
        while (!placed) {
            robot.shape.setPosition({
                static_cast<float>(rand() % 1500),
                static_cast<float>(rand() % 1100)
            });
            if (!overlapsAny(robot.shape, obstacles)) {
                placed = true;
            }
        }
        // Repeat for end goal
        placed = false;
        while (!placed) {
            robot.goal.setPosition({
                static_cast<float>(rand() % 1500),
                static_cast<float>(rand() % 1100)
            });
            if (!overlapsAny(robot.goal, obstacles)) {
                placed = true;
            }
        }
        robots.push_back(robot);
    }
}


//Return 0 if no collision, 1 if collision on x axis, 2 if collision on y axis
int World::checkCollision(sf::RectangleShape& a, sf::RectangleShape& b) {
    sf::FloatRect ra = a.getGlobalBounds();
    sf::FloatRect rb = b.getGlobalBounds();

    bool overlapX = ra.position.x < rb.position.x + rb.size.x &&
                    ra.position.x + ra.size.x > rb.position.x;

    bool overlapY = ra.position.y < rb.position.y + rb.size.y &&
                    ra.position.y + ra.size.y > rb.position.y;

    if (overlapX && overlapY){

        // Compute overlap depth on both axes
        float overlapLeft   = (ra.position.x + ra.size.x) - rb.position.x;
        float overlapRight  = (rb.position.x + rb.size.x) - ra.position.x;
        float overlapTop    = (ra.position.y + ra.size.y) - rb.position.y;
        float overlapBottom = (rb.position.y + rb.size.y) - ra.position.y;

        // Pick the smallest overlap axis
        float xOverlap = std::min(overlapLeft, overlapRight);
        float yOverlap = std::min(overlapTop, overlapBottom);

        if (xOverlap < yOverlap)
            return 1; // collision along X axis
        else
            return 2; // collision along Y axis

    }

    else return 0;
}


//check robot obstacle collisions
bool World::checkCollision(sf::CircleShape& a, sf::RectangleShape& b) {
    
    // Circle center
    sf::Vector2f circleCenter = a.getPosition();

    // Rectangle bounds
    sf::FloatRect rectBounds = b.getGlobalBounds();

    // Find closest point on rectangle to circle center
    float closestX = std::max(rectBounds.position.x, 
                        std::min(circleCenter.x, rectBounds.position.x + rectBounds.size.x));
    float closestY = std::max(rectBounds.position.y,  
                        std::min(circleCenter.y, rectBounds.position.y + rectBounds.size.y));

    // Compute distance squared
    float dx = circleCenter.x - closestX;
    float dy = circleCenter.y - closestY;
    float distSq = dx * dx + dy * dy;

    // Collision if distance < radius
    return distSq < (a.getRadius() * a.getRadius());
}

bool World::overlapsAny(sf::RectangleShape& newObs, std::vector<sf::RectangleShape>& existing) {
    for (const auto& obs : existing) {
        if (checkCollision(const_cast<sf::RectangleShape&>(newObs), const_cast<sf::RectangleShape&>(obs)) > 0) {
            return true;
        }
    }
    return false;
}

bool World::overlapsAny(sf::CircleShape& newObs, std::vector<sf::RectangleShape>& existing) {
    for (const auto& obs : existing) {
        if (checkCollision(const_cast<sf::CircleShape&>(newObs), const_cast<sf::RectangleShape&>(obs)) > 0) {
            return true;
        }
    }
    return false;
}

void World::update(float dt) {
    
    if(!isMoving) return;
    
    for (auto& robot : robots) {
        robot.update(dt);
    }

    // Move moving obstacles with collision checks
    for (size_t i = 0; i < movingObstacles.size(); i++) {
        auto& mob = movingObstacles[i];
        auto& obs = obstacles[mob.index];

        // Predict next position
        sf::Vector2f nextPos = obs.getPosition() + mob.velocity * dt;
        sf::RectangleShape temp = obs;
        temp.setPosition(nextPos);

        // Check collision against stationary obstacles
        for (size_t j = 0; j < obstacles.size(); j++) {
            if (j == mob.index) continue; // skip self
            if (checkCollision(temp, obstacles[j]) == 1) {mob.velocity.x = - mob.velocity.x; break;}
            else if (checkCollision(temp, obstacles[j]) == 2) {mob.velocity.y = - mob.velocity.y; break;}
        }


        // Bounce off window edges
        if (obs.getPosition().x < 0 || obs.getPosition().x + obs.getSize().x > 1600)
            mob.velocity.x = -mob.velocity.x;
        if (obs.getPosition().y < 0 || obs.getPosition().y + obs.getSize().y > 1200)
            mob.velocity.y = -mob.velocity.y;

        // Move if no collision
        obs.move(mob.velocity * dt);
    }
}

void World::draw(sf::RenderWindow& window){
    for (auto& robot : robots) {
        window.draw(robot.shape);
        window.draw(robot.goal);
    }
    for(auto& obs : obstacles){
        window.draw(obs);
    }
}

void World::generateSensorSnapshot(Robot& robot) const {
    
    SensorData data;
    data.distances.resize(data.numRays, data.maxRange);
    data.rayDirections.resize(data.numRays);

    float angleStep = M_PI / (data.numRays - 1); // pi radians cone
    
    sf::Vector2f currentPos =  robot.shape.getPosition();
    float baseAngle = robot.heading;  // robot’s current heading

    for (int i = 0; i < data.numRays; i++) {

        float angle = baseAngle - M_PI/2 + i * angleStep;
        
        sf::Vector2f dir(std::cos(angle), std::sin(angle));
        data.rayDirections[i] = dir;

        // ray marching to check collision with obstacles
        for (float r = 0; r < data.maxRange; r += 5.f) {
            sf::Vector2f point = currentPos + dir * r;

            if (point.x < 0.f || point.x >= 1600.f ||
                point.y < 0.f || point.y >= 1200.f) {
                data.distances[i] = r;
                break; // stop marching further
            }

            for (auto& obs : obstacles) {
                if (obs.getGlobalBounds().contains(point)) {
                    data.distances[i] = r;
                    r = data.maxRange; // break outer loop
                    break;
                }
            }
            for (auto& other : robots) {
                if (&other == &robot) continue; // skip self
                if (other.shape.getGlobalBounds().contains(point) && r < data.distances[i]) {
                    data.distances[i] = r;
                    r = data.maxRange; // break outer loop
                    break;
                }
            }

        }
    }
    robot.sensorData = data;
}

void World::setHeadings(float dt){
    for (auto& robot : robots) {
        robot.setHeading(dt);
    }
}

void World::updateSensorData() {
    for (auto& robot : robots) {
        generateSensorSnapshot(robot);
    }
}

void World::toggleMovement(){
    isMoving = !isMoving;
}

void World::updateSpeed(float speed){
    for (auto& robot : robots) {
        robot.speed += speed;
        if (robot.speed < 0.f) robot.speed = 0.f;
    }
}
