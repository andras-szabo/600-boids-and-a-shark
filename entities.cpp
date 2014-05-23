#include "entities.h"
#include <cmath>
#include <iostream>

auto dot = [](sf::Vector2f a, sf::Vector2f b) { return (a.x * b.x) + (a.y * b.y); };

cEntity::cEntity(sf::Vector2f p, sf::Vector2f v, int sx, int sy):
mPos { p },
mVel { v },
mFOV { 360 },
mScreenSizeX { sx },
mScreenSizeY { sy },
mRightFOV { mFOV / 2 },
mLeftFOV { 360 - (mFOV/2) }
{
    
}

sf::Vector2f cEntity::update(sf::Time dt, bool wrap)
{
    if ( wrap == true )
    {
        if ( mPos.x < 5 )
        {
            mPos.x = mScreenSizeX - 5;
        }
        else if ( mPos.x > mScreenSizeX - 5 )
        {
            mPos.x = 5;
        }
        
        if ( mPos.y < 5 )
        {
            mPos.y = mScreenSizeY - 5;
        }
        else if ( mPos.y > mScreenSizeY - 5 )
        {
            mPos.y = 5;
        }
    }
    else    // if not wrap, bounce
    {
        bool reset_heading { false };
        if ( mPos.x < 5 || mPos.x > mScreenSizeX - 5 )
        {
            mVel.x *= -1;
            if ( mPos.x < 5 ) mPos.x = 5;
            if ( mPos.x > mScreenSizeX - 5 ) mPos.x = mScreenSizeX - 5;
            reset_heading = true;
        }
        
        if ( mPos.y < 5 || mPos.y > mScreenSizeY - 5 )
        {
            mVel.y *= -1;
            if ( mPos.y < 5 ) mPos.y = 5;
            if ( mPos.y > mScreenSizeY - 5 ) mPos.y = mScreenSizeY - 5;
            reset_heading = true;
        }
        
        if ( reset_heading ) setHeading();
        
    }

    mPos += mVel * dt.asSeconds();

    return mPos;
}

void cEntity::setHeading()
{
    float l = sqrt(pow(mVel.x, 2) + pow(mVel.y, 2));
    mHeadingUnit = sf::Vector2f ( mVel.x / l, mVel.y / l );
    mHeading = acos(dot(sf::Vector2f(1.0, 0.0), mHeadingUnit)) / 3.1456 * 180;
}

void cEntity::setFOV(float f)
{
    mFOV = f;
    mRightFOV = mFOV / 2;
    mLeftFOV = 360 - ( mFOV / 2 );
}

void cEntity::adjustFOV(float f)
{
    setFOV(mFOV + f);
}


double cEntity::canSee(sf::Vector2f target) const
{
    // Can this entity see the given coordinates? It can, if it's in its
    // FOV.
    // If it can, it returns the distance to the object.
    // If it can't, it returns -1;
    sf::Vector2f direction(target.x - mPos.x, target.y - mPos.y);
    double distance { sqrt(pow(direction.x, 2) + pow(direction.y, 2)) };
    
    if ( mFOV == 360 ) return distance;
    
    float angle = acos(dot(direction, mHeadingUnit) / distance ) / 3.1456 * 180;
    if ( angle <= mRightFOV || angle >= mLeftFOV )
        return distance;
    else return -1.0;
}

cBoid::cBoid(sf::Vector2f p, sf::Vector2f v, int sx, int sy, int sz,
             double& close, double& cohesion, double& avoidance, double& alignment):
cEntity { p, v, sx, sy },
mSize { sz },
mTooClose { close },
mCohesion { cohesion },
mAvoidance { avoidance },
mAlignment { alignment },
mRepelFactor { 2.0 },
mRepelArea { 0 }
{
    mVertices[0].x = mSize;
    mVertices[0].y = 0;
    
    mVertices[1].x = -mSize;
    mVertices[1].y = mSize/2;
    
    mVertices[2].x = -mSize;
    mVertices[2].y = -mSize/2;
    
    setHeading();
}

void cBoid::steer(cBoid* neighbours[], int count)
{
    if ( count == 0 ) return;
    int discount = 0;           // discounting those outside FOV
    
    sf::Vector2f    centreOfMass { 0, 0 };
    sf::Vector2f    avoidVector { 0, 0 };
    sf::Vector2f    dirVector { 0, 0 };
    sf::Vector2f    perceivedVel { 0, 0 };
    
    double          distance { 0 };
    
    for (auto i = 0; i < count; ++i)
    {
        distance = canSee(neighbours[i]->mPos);
        if (distance != -1.0)
        {
            // We should move toward this centre of mass
            centreOfMass += neighbours[i]->mPos;
         
            // If too close to others, move away
            dirVector = neighbours[i]->mPos - mPos;
            if ( distance < mTooClose || distance < neighbours[i]->mRepelArea )
            {
                if ( neighbours[i]->mRepelFactor == 10 )
                {
                    mColor = sf::Color::Red;
                }
                double repFact = ( 100 / distance ) * neighbours[i]->mRepelFactor;
                dirVector.x *= repFact;
                dirVector.y *= repFact;
                avoidVector -= dirVector;
            }
            
            // Try to align velocity with that of others nearby
            perceivedVel += neighbours[i]->mVel;
        }
        else // can't actually see the thing
        {
            ++discount;
        }
    }
    
    float actualCount = count - discount;
    
    if ( actualCount > 0 )
    {
            // To calc. movement to centre of mass, we need to average it
            centreOfMass.x /= actualCount;
            centreOfMass.y /= actualCount;
            
            // So we need to move in that direction - but let's not take the
            // full vector, only a fraction.
            // mCohesion: 0.01 (1%), 0.25 (25%) etc.
            centreOfMass -= mPos;
            centreOfMass.x *= mCohesion;
            centreOfMass.y *= mCohesion;
            
            // Let's try also averaging and weighing the avoidance vector
            // mAvoidance: 0.01 (1%), 0.25 (25%) etc.
            avoidVector.x /= (actualCount / mAvoidance);
            avoidVector.y /= (actualCount / mAvoidance);
            
            // Also average and weigh perceived velocity of nearby boids
            // as usual, weighing with mAlignment, a percentage value
            perceivedVel.x /= actualCount;
            perceivedVel.y /= actualCount;
            perceivedVel -= mVel;
            perceivedVel.x *= mAlignment;
            perceivedVel.y *= mAlignment;
        
            // Recalculating velocity means having to recalculate
            // heading as well.
        
            mVel += centreOfMass + avoidVector + perceivedVel;
        
            if ( mVel.x < -300 ) mVel.x = -300;
            if ( mVel.x > 300 ) mVel.x = 300;
        
            if ( mVel.y < -300 ) mVel.y = -300;
            if ( mVel.y > 300 ) mVel.y = 300;
        
            setHeading();
    }
}

void cBoid::render(sf::Vertex* va)
{
    // The cross product between actual heading and stationary position
    // tells us which way to rotate the object. Since the vector that
    // we compare the actual heading to is a unit vector (1;0), the cross
    // product will always equal the y coordinate of the actual heading
    // (i.e. velocity).
    
    sf::Transform tmp;
    if ( mVel.y > 0 ) tmp.rotate(mHeading);
    else tmp.rotate(-mHeading);

    // Then simply adjust points in the vertexarray
    
    for ( auto j = 0; j < 3; ++j )
    {
        *(va+j) = tmp.transformPoint(mVertices[j]);
        (*(va+j)).position += mPos;
        (*(va+j)).color = mColor;
    }
    
    mColor = sf::Color::White;
}

cEnemy::cEnemy(sf::Vector2f pos, double& rf):
cBoid { pos, sf::Vector2f(0,0), 0, 0, 0, rf, rf, rf, rf }
{
    mRepelFactor = rf;
}
