#pragma once
#include <SFML/Graphics.hpp>
#include "Robot.h"
#include "SensorData.h"

struct MovingObstacle {
    size_t index;  // index to obstacle in main vector
    sf::Vector2f velocity;      // current velocity
};

class World {
public:
    World(int numStationary, int numMoving, int numRobots); 
    std::vector<Robot> robots;
    std::vector<sf::RectangleShape> obstacles;
    std::vector<MovingObstacle> movingObstacles;
    bool isMoving = false;

    void update(float dt);
    void draw(sf::RenderWindow& window);
    
    void generateSensorSnapshot(Robot& robot) const;
    
    int checkCollision(sf::RectangleShape& a, sf::RectangleShape& b);
    bool checkCollision(sf::CircleShape& a, sf::RectangleShape& b);
    
    bool overlapsAny(sf::RectangleShape& newObs, std::vector<sf::RectangleShape>& existing);
    bool overlapsAny(sf::CircleShape& newObs, std::vector<sf::RectangleShape>& existing);
    
    void updateSensorData();
    void setHeadings(float dt);
    void toggleMovement();
    void updateSpeed(float speed);

};