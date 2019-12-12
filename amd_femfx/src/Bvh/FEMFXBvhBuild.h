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
// Bounding volume hierarchy building, derived from Radeon Rays
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

#include "FEMFXCommonInternal.h"
#include "FEMFXBvh.h"
#include "FEMFXVectorMath.h"

namespace AMD
{
    typedef struct int2
    {
        int x;
        int y;
    } int2;

    void FmCalcMortonCode(
        // Bounds used to map box centers into 0 to 1 range.
        FmVector3 spacemax,
        FmVector3 spacemin,
        // Centers of primitive bounding boxes
        const FmAabb* bounds,
        // Number of primitives
        int numpositions,
        // Morton codes
        int* mortoncodes
    );

    // Set parent-child relationship
    void FmBuildHierarchy(
        // Sorted Morton codes of the primitives
        int* mortoncodes,
        // Bounds
        FmAabb* bounds,
        // Primitive indices
        int* indices,
        // Number of primitives
        int numprims,
        // Nodes
        FmBvhNode* nodes,
        // Leaf bounds
        FmAabb* boundssorted
    );

    void FmRefitBounds(
        int numprims,
        FmBvhNode* nodes
    );

    int2 FmFindSpan(int* mortoncodes, int numprims, int idx);

    // Build BVH from leaf primitives; assumes resultBvh numPrims is set and primBoxes have been computed
    void FmBuildBvhOnLeaves(FmBvh* resultBvh, const FmVector3& minPosition, const FmVector3& maxPosition);
}
