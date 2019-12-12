/*
MIT License

Copyright (c) 2019 Advanced Micro Devices, Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

//---------------------------------------------------------------------------------------
// Bounding volume hierarchy type, derived from Radeon Rays
//---------------------------------------------------------------------------------------

// Copyright from Radeon Rays code:
/**********************************************************************
Copyright (c) 2016 Advanced Micro Devices, Inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
********************************************************************/

#pragma once

/*************************************************************************
DEFINES
**************************************************************************/
#include "FEMFXCommon.h"
#include "FEMFXAabb.h"

namespace AMD
{

    struct FmBvhNode
    {
        FmAabb box;
        //int parent;
        int left;
        int right;
        //int next;

        FmBvhNode()
        {
            left = 0;
            right = 0;
        }
    };

    // Bounding volume hierarchy built as binary tree of axis-aligned boxes.
    // Boxes include velocity bounds for culling CCD tests, using assumption of 
    // constant velocity for each vertex during the timestep.
    // BVH is rebuilt from scratch each step, using code from RadeonRays library.
    struct FmBvh
    {
        FmBvhNode* nodes;
        FmAabb*    primBoxes;
        int*       mortonCodesSorted;
        int*       primIndicesSorted;
        uint       numPrims;

        FmBvh()
        {
            nodes = NULL;
            primBoxes = NULL;
            mortonCodesSorted = NULL;
            primIndicesSorted = NULL;
            numPrims = 0;
        }

        FmBvhNode& GetLeafNode(uint primIdx)
        {
            uint nodeIdx = (numPrims - 1) + primIdx;
            return nodes[nodeIdx];
        }

        const FmBvhNode& GetLeafNode(uint primIdx) const
        {
            uint nodeIdx = (numPrims - 1) + primIdx;
            return nodes[nodeIdx];
        }
    };

    static inline void FmInitBvh(FmBvh* bvh)
    {
        bvh->nodes = NULL;
        bvh->primBoxes = NULL;
        bvh->mortonCodesSorted = NULL;
        bvh->primIndicesSorted = NULL;
        bvh->numPrims = 0;
    }

    static FM_FORCE_INLINE uint FmNumBvhNodes(uint numLeaves)
    {
        return numLeaves == 0 ? 0 : numLeaves * 2 - 1;
    }
}
