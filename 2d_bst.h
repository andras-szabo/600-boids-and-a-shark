//
//  2d_bst.h
//  2D red-black BST
//
//  Created by Andras Szabo on 15/03/14.
//  Copyright (c) 2014 Andras Szabo. All rights reserved.
//

#ifndef __simple_bst___d_bst__
#define __simple_bst___d_bst__

namespace kdbst {
    
    const int MAX_NEIGH { 600 };
    
    template <class cKey, class cVal>
    struct cNode {
        typedef cNode<cKey, cVal>* nodePtr;
        
        cNode();
        cNode(cKey, cKey, cVal, int);
        
        cKey        mKey1;
        cKey        mKey2;
        cVal        mValue;
        int         mStatus;
        
        nodePtr     pLeft;
        nodePtr     pRight;
        
    };
    
    template <class cKey, class cVal>
    class c2dbst {
        typedef cNode<cKey, cVal>* nodePtr;

    public:
        c2dbst();
        c2dbst(cVal);
        ~c2dbst();
        
        cVal        get(cKey, cKey);
        void        put(cKey, cKey, cVal);
        void        reset();
        int         neigh(cKey, cKey, float);
        
    private:
        inline bool isActive(const nodePtr&) const;
        
        nodePtr     put(nodePtr, cKey, cKey, cVal);
        
        void        free_mem(nodePtr);
        void        neighbours(nodePtr, cKey, cKey, float);
    
    public:
        cVal            mAdjacent[MAX_NEIGH];
    
    private:
        const cVal      nullValue;
        
        nodePtr         mRoot;
        
        bool            mEven;
        int             mPtr;
        
        int             mStatus;
    };
    
    #include "2d_bst.inl"
}

#endif /* defined(__simple_bst___d_bst__) */
