#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <sstream>
#include "World.h"
#include "Robot.h"



std::mutex worldMutex;


std::atomic<bool> running{true};
std::atomic<bool> paused{true};

int numStationary = 7;
int numMoving = 7;
int numRobots = 3;

World world(numStationary, numMoving, numRobots);

// Physics thread
void physicsLoop() {
    sf::Clock clock;
    while (running) {
        float dt = clock.restart().asSeconds();
        // Move all objects (update world)
        if (!paused) {
            std::lock_guard<std::mutex> lock(worldMutex);
            world.update(dt);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
}

// Sensor thread
void sensorLoop() {
    while (running) {
        // update all sensor data for robots
        {
            std::lock_guard<std::mutex> lock(worldMutex);
            world.updateSensorData();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // e.g., 20 Hz
    }
}

// Path finding thread
void pathFindingLoop() {
    sf::Clock clock;
    while (running) {
        // update robot heading (direction)
        if (!paused) {
            float dt = clock.restart().asSeconds();
            std::lock_guard<std::mutex> lock(worldMutex);
            world.setHeadings(dt);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20Hz
    }
}

int main() {
    // Create window
    sf::RenderWindow* window = new sf::RenderWindow(sf::VideoMode({1600,1200}), "Simulator");

    sf::Font font;
    if (!font.openFromFile("arial.ttf")) {
        return -1; 
    }

    sf::Text uiText(font);
    uiText.setCharacterSize(20);
    uiText.setFillColor(sf::Color::White);
    uiText.setPosition({10.f, 10.f});

    sf::Clock clock;

    // Launch physics thread
    std::thread physicsThread(physicsLoop);

    //Launch sensor thread
    std::thread sensorsThread(sensorLoop);

    //Launch sensor thread
    std::thread pathThread(pathFindingLoop);

    while (window->isOpen()) {
        while (const std::optional event = window -> pollEvent()) {
            if (event->is<sf::Event::Closed>())
                window->close();
                //event->is<sf::Event::KeyPressed>()
            else if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {
                std::lock_guard<std::mutex> lock(worldMutex);
                switch (keyPressed ->scancode) {
                    case sf::Keyboard::Scancode::Space: // Pause
                        paused = !paused; 
                        world.toggleMovement();
                        break;
                    case sf::Keyboard::Scancode::Up: //Increase speed
                        world.updateSpeed(0.5f);
                        break;
                    case sf::Keyboard::Scancode::Down: //decrease speed
                        world.updateSpeed(-0.5f);
                        break;
                    case sf::Keyboard::Scancode::R:   // restart
                        world = World(7, 7, 3);
                        paused = true; 
                        break;
                    default: break;
                }
            }
        }

        //Render world
        {
            std::lock_guard<std::mutex> lock(worldMutex);

            uiText.setString(
                "Robot State:" +
                std::string(world.isMoving ? "Moving" : "Stopped")
            );

            window->clear();
            world.draw(*window);
            window->draw(uiText);
        }

        window->display();

    }

    physicsThread.join();
    sensorsThread.join();
    pathThread.join();
    return 0;
}