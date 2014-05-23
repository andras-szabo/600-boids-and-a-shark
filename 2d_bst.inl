using namespace kdbst;

template <class cKey, class cVal>
cNode<cKey, cVal>::cNode(cKey k1,
                         cKey k2,
                         cVal v,
                         int i):
mKey1 { k1 },
mKey2 { k2 },
mValue { v },
mStatus { i },
pLeft { nullptr },
pRight { nullptr }
{
    
}

template <class cKey, class cVal>
c2dbst<cKey, cVal>::c2dbst(cVal nv):
nullValue { nv },
mRoot { nullptr },
mPtr { 0 }
{
    
}

template <class cKey, class cVal>
c2dbst<cKey, cVal>::~c2dbst()
{
    free_mem(mRoot);
}

template <class cKey, class cVal>
void c2dbst<cKey, cVal>::free_mem(nodePtr p)
{
    if ( p == nullptr ) return;
    free_mem(p->pLeft);
    free_mem(p->pRight);
    delete p;
}

template <class cKey, class cVal>
inline bool c2dbst<cKey, cVal>::isActive(const nodePtr& p) const
{
    if ( p == nullptr ) return false;
    return p->mStatus == mStatus;
}

template <class cKey, class cVal>
cVal c2dbst<cKey, cVal>::get(cKey k1,
                             cKey k2)
{
    mEven = true;
    nodePtr p = mRoot;
    
    while ( isActive(p) && (p->mKey1 != k1 ||
            p->mKey2 != k2) )
    {
        if ( mEven == true )    // if at even levels, look at horizontal coord (k1)
        {
            // Go down appropriate branch,
            // then flip mEven
            
            if ( k1 < p->mKey1 )
                p = p->pLeft;
            else
                p = p->pRight;
            
            mEven = false;
            
        }
        else                  // odd level, look at k2
        {
            if ( k2 < p->mKey2 )
                p = p->pLeft;
            else
                p = p->pRight;
            
            mEven = true;
        }
    }
    
    return isActive(p) ? p->mValue : nullValue;
}

template <class cKey, class cVal>
void c2dbst<cKey, cVal>::put(cKey k1,
                             cKey k2,
                             cVal v)
{
    mEven = true;
    mRoot = put(mRoot, k1, k2, v);
}


//
//
// Put, internal operation, follows!
//
//

template <class cKey, class cVal>
cNode<cKey, cVal>* c2dbst<cKey, cVal>::put(nodePtr p,
                                           cKey k1,
                                           cKey k2,
                                           cVal v)
{
    if ( isActive(p) )
    {
        if ( p->mKey1 == k1 && p->mKey2 == k2 )
        {
            p->mValue = v;
            p->mStatus = mStatus;
        }
        else // No matching key.
        {
            if ( mEven == true )
            {
                mEven = false; // flip it
                if ( k1 < p->mKey1 )
                    p->pLeft = put(p->pLeft, k1, k2, v);
                else
                    p->pRight = put(p->pRight, k1, k2, v);
            }
            else // we're at an odd level
            {
                mEven = true; // flip it
                if ( k2 < p->mKey2 )
                    p->pLeft = put(p->pLeft, k1, k2, v);
                else
                    p->pRight = put(p->pRight, k1, k2, v);
            }
        }
        return p;
    }
    else    // p not active: nullptr or simply inactive
    {
        if ( p == nullptr )
        {
            return new cNode<cKey, cVal>(k1, k2, v, mStatus);
        }
        else    // we simply activate it
        {
            p->mStatus = mStatus;
            p->mKey1 = k1;
            p->mKey2 = k2;
            p->mValue = v;
            
            return p;
        }
    }
}

template <class cKey, class cVal>
void c2dbst<cKey, cVal>::reset()
{
    // Resetting amounts to flipping the status boolean
    
    ++mStatus;
    
    //if ( mRoot == nullptr ) return;
    //reset(mRoot);
}

/*
 
 Usage:
 
 neigh(key1, key2, dist) sets mAdjacent[MAX_NEIGH] up,
 filling it up with neighbours of the query position
 (key1, key2); where neighbours are inside the AABB
 that's described by the query position and dist
 (each side of the AABB equals dist*2, making the
 query position its centre.
 
 neigh(...) then returns a pointer that points to the
 one-after-the-last valid pointer in mAdjacent.
 
 Then one can simply iterate through mAdjacent to
 find all those who are in range, i.e.:
 
 auto n = neigh(x, y, dist);
 for (auto i = 0; i < n; ++i)
 {
    // ... this iterates through all relevant neighbours
    // which themselves are stored in mAdjacent
 }
 
 Important. Neigh doesn't count an object at the
 exact query position. (I.e. if we're supplying it
 an entity's coordinates, it won't count the entity
 itself as its own neighbour.)
 
*/

template <class cKey, class cVal>
int c2dbst<cKey, cVal>::neigh(cKey cx,
                               cKey cy,
                               float dist)
{
    mEven = true;
    mPtr = 0;
    neighbours(mRoot, cx, cy, dist);
    return mPtr;
}

template <class cKey, class cVal>
void c2dbst<cKey, cVal>::neighbours(nodePtr p,
                                    cKey cx,
                                    cKey cy,
                                    float dist)
{
    if ( !isActive(p) ) return;
    
    // Check if current node is within
    // rectangle.
    
    if ( cx - dist <= p->mKey1 &&
         cx + dist >= p->mKey1 &&
         cy - dist <= p->mKey2 &&
         cy + dist >= p->mKey2 )
    {
        // Yay, point found!
        // Push it onto the solution
        // array, and let everyone know
        // about it by moving forward
        // the mPtr pointer
        
        if ( p->mKey1 != cx || p->mKey2 != cy ) // let's not add the query pos itself
        {
            if ( mPtr < MAX_NEIGH )
                mAdjacent[mPtr++] = p->mValue;
        }
    }
    
    if ( mEven == true )
    {
        // We're at a point which divides
        // the screen vertically. Can points
        // be in the right subtree?
        // Yes, if current point is to the
        // left of the right edge of AABB.
        
        if ( cx + dist >= p->mKey1 )
        {
            mEven = false;
            neighbours(p->pRight, cx, cy, dist);
        }
        
        // Can there still be points in the
        // left subtree? Maybe.
        
        if ( cx - dist < p->mKey1 )
        {
            mEven = false;
            neighbours(p->pLeft, cx, cy, dist);
        }
    }
    else // mEven == false
    {
        if ( cy + dist >= p->mKey2 )
        {
            mEven = true;
            neighbours(p->pRight, cx, cy, dist);
        }
        
        if ( cy - dist < p->mKey2 )
        {
            mEven = true;
            neighbours(p->pLeft, cx, cy, dist);
        }
    }
}