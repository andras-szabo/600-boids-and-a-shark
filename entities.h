#ifndef __testing_view_cone__entities__
#define __testing_view_cone__entities__

#include <SFML/Graphics.hpp>

class cEntity {
public:
    cEntity();
    cEntity(sf::Vector2f, sf::Vector2f, int, int);
    
    sf::Vector2f        update(sf::Time, bool);
    virtual void        render(sf::Vertex*) = 0;
    
    double              canSee(sf::Vector2f) const;
    
public:
    sf::Vector2f        mPos;
    sf::Vector2f        mVel;
    sf::Vector2f        mHeadingUnit;
    float               mHeading;
    float               mFOV;       // field of view

public:
    void                setHeading();
    void                setFOV(float);
    void                adjustFOV(float);

private:
    int                 mScreenSizeX;
    int                 mScreenSizeY;
    float               mRightFOV;
    float               mLeftFOV;
    
};

class cBoid : public cEntity {
public:
    cBoid();
    cBoid(sf::Vector2f, sf::Vector2f, int, int, int, double&, double&, double&, double&);
    
    virtual void        render(sf::Vertex*);
    virtual void        steer(cBoid*[], int);

public:
    double              mRepelFactor;
    double              mRepelArea;

private:
    int                 mSize;
    sf::Vector2f        mVertices[3];
    sf::Color           mColor;

    double&             mTooClose;
    double&             mCohesion;
    double&             mAvoidance;
    double&             mAlignment;
};

class cEnemy : public cBoid {
public:
    cEnemy();
    cEnemy(sf::Vector2f, double&);

    virtual void        render(sf::Vertex* v) { return; }
    virtual void        steer(cBoid*[], int) { return; }
};


#endif /* defined(__testing_view_cone__entities__) */
