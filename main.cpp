#include <SFML/Audio.hpp>
#include <SFML/Graphics.hpp>
#include "ResourcePath.hpp"
#include "entities.h"
#include <vector>
#include "2d_bst.h"

// Quick and dirty int-to-string,
// works for our purposes
inline std::string i2s(int i)
{
    char c;
    std::string ret { };
    do {
        c = i % 10 + 48;
        ret = c + ret;
        i /= 10;
    } while ( i > 0 );
    return ret;
}

int main()
{
    // Init stuff

    int ScreenX { 640 };
    int ScreenY { 480 };
    sf::RenderWindow window(sf::VideoMode(ScreenX, ScreenY), "600 boids and a shark");
    sf::Font font;
    if (!font.loadFromFile(resourcePath() + "sansation.ttf")) {
        return EXIT_FAILURE;
    }
    window.setVerticalSyncEnabled(false);
    srand(time(NULL));
    
    // Set up vertexarray, container of boids, "global" variables

    const int       gBoidCount { 600 };
    const int       gMaxSpeed { 300 };
    float           gViewDistance { 250 };
    
    double          gTooClose { 10 };
    double          gCohesion { 0.01 };
    double          gAvoidance { 5 };
    double          gAlignment { 0.015 };
    
    sf::Vertex      vertexArray[gBoidCount * 3];
    std::vector<std::unique_ptr<cBoid>> boids;

    // Boids start from random positions
    
    for (auto i = 0; i < gBoidCount; ++i)
    {
        sf::Vector2f    pos ( rand() % (ScreenX - 100) + 50, rand() % (ScreenY - 100) + 50 );
        sf::Vector2f    vel ( rand() % gMaxSpeed - gMaxSpeed / 2,
                              rand() % gMaxSpeed - gMaxSpeed / 2);
        std::unique_ptr<cBoid> p { new cBoid(pos, vel, ScreenX, ScreenY, 10,
                                             gTooClose, gCohesion, gAvoidance,
                                             gAlignment) };
        p->mRepelFactor = 5;
        boids.push_back(std::move(p));
    }
    
    // Add monster to centre of screen
    double          monsterFactor { 10 };
    std::unique_ptr<cEnemy> p { new cEnemy(sf::Vector2f(ScreenX / 2, ScreenY / 2), monsterFactor) };
    p->mRepelArea = 100;
    

    // This is what the inheritance hierarchy looks like:
    // cEntity -> cBoid -> cEnemy (see in entities.h)
    
    cEnemy* monsterPtr = p.get();
    boids.push_back(std::move(p));
    
    // Set up kd-tree
    
    kdbst::c2dbst<float, cBoid*>    bst(nullptr);
    
    // Set up timers, clocks
    
    sf::Time        timeSinceLastUpdate { sf::Time::Zero };
    sf::Time        timeSinceLastRender { sf::Time::Zero };
    const sf::Time  oneSecond { sf::seconds(1.0) };
    sf::Clock       fpsClock;
    int             fps { 0 };
    sf::Clock       clock;
    sf::Text        tFPS;
    
    tFPS.setFont(font);
    tFPS.setCharacterSize(16);
    tFPS.setPosition(20, 20);
    tFPS.setColor(sf::Color::Green);

    // Jetzt geht's los
    
    sf::RectangleShape  monsta;
    monsta.setFillColor(sf::Color::Red);
    monsta.setSize(sf::Vector2f(10, 10));
    monsta.setOrigin(5.0, 5.0);
    
    
    // Main loop
    
    while (window.isOpen())
    {

        sf::Event event;
        while (window.pollEvent(event))
        {

            if (event.type == sf::Event::Closed) {
                window.close();
            }


            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape) {
                window.close();
            }
        }
        
        // Update monster
        
        sf::Vector2i pos = sf::Mouse::getPosition(window);
        monsterPtr->mPos = sf::Vector2f(pos.x, pos.y);
        monsta.setPosition(monsterPtr->mPos);

        // Reset bst, then add each boid
        
        bst.reset();
        
        for (const auto& i : boids)
            bst.put(i->mPos.x, i->mPos.y, i.get());
        
        // Steer.
        
        for (auto& i : boids)
        {
            auto n = bst.neigh(i->mPos.x, i->mPos.y, gViewDistance);
            i->steer(&bst.mAdjacent[0], n);
        }
        
        timeSinceLastUpdate = clock.restart();
        
        int ptr = 0;
        for(auto& i : boids)
        {
            i->update(timeSinceLastUpdate, true);
            i->render(&vertexArray[ptr]);
            ptr += 3;
        }
        
        window.clear();
        window.draw(&vertexArray[0], gBoidCount * 3, sf::Triangles);
        window.draw(monsta);
        
        // Calculate FPS
        
        ++fps;
        timeSinceLastRender += fpsClock.restart();
        if ( timeSinceLastRender >= oneSecond )
        {
            tFPS.setString(i2s(fps));
            fps = 0;
            timeSinceLastRender -= oneSecond;
        }
        
        window.draw(tFPS);
        
        // Draw everything

        window.display();
    }
    return 0;
}
