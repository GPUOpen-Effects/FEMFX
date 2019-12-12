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

#include "FEMFXCommonInternal.h"
#include "FEMFXBvhBuild.h"
#include "FEMFXSort.h"

using namespace std;

namespace AMD
{

    static inline int FmSign(float val)
    {
        return (val > 0.0f) - (val < 0.0f);
    }

    // Space out the lower 10 bits of integer at a stride of 3 bits.
    // Using this method adapted for different spacing:
    // http://www.graphics.stanford.edu/~seander/bithacks.html#InterleaveBMN
    static FM_FORCE_INLINE uint FmSpace10BitsStride3(uint value)
    {
        uint result = value;

        // Repeated shift, or, and mask to space bits at stride of 3
        result = (result | (result << 10)) & 0x000F801F;  // groups of 5 and 5
        result = (result | (result << 6)) & 0x3038607;    // groups of 2 3 2 3
        result = (result | (result << 2)) & 0x30C8619;    // groups of 2 2 1 2 2 1
        result = (result | (result << 2)) & 0x9249249;

        return result;
    }

    // Compute Morton code for a 3D position, assuming normalizedPos components are each >= 0.0f and <= 1.0f.
    // Convert elements to 10-bit integer and interleave into 32-bit code
    static FM_FORCE_INLINE uint FmComputeMortonCode(const FmVector3& normalizedPos)
    {
        float x = normalizedPos.x;
        float y = normalizedPos.y;
        float z = normalizedPos.z;

        // Scale to 10 bit integer
        uint xi = (uint)(x * 1024.0f);
        uint yi = (uint)(y * 1024.0f);
        uint zi = (uint)(z * 1024.0f);

        // Clamp - included for safety but may not be needed in practice
        xi = FmMinUint(xi, 1023);
        yi = FmMinUint(yi, 1023);
        zi = FmMinUint(zi, 1023);

        return (FmSpace10BitsStride3(xi) << 2) | (FmSpace10BitsStride3(yi) << 1) | FmSpace10BitsStride3(zi);
    }

    // Assign Morton codes to each of positions
    void FmCalcMortonCode(
        // Bounds used to map box centers into 0 to 1 range.
        FmVector3 spacemax,
        FmVector3 spacemin,
        // Centers of primitive bounding boxes
        FmAabb const* bounds,
        // Number of primitives
        int numpositions,
        // Morton codes
        int* mortoncodes
    )
    {
        for (int globalid = 0; globalid < numpositions; globalid++)
        {
            FmAabb bound = bounds[globalid];
            FmVector3 center = 0.5f * (bound.pmax + bound.pmin);
            center = (center - spacemin) / (spacemax - spacemin);
            mortoncodes[globalid] = (int)FmComputeMortonCode(center);
        }
    }

    FmAabb FmBboxUnion(FmAabb b1, FmAabb b2)
    {
        FmAabb res;
        res.pmin = min(b1.pmin, b2.pmin);
        res.pmax = max(b1.pmax, b2.pmax);
        res.vmin = min(b1.vmin, b2.vmin);
        res.vmax = max(b1.vmax, b2.vmax);
        return res;
    }

#define FM_LEAFIDX(i) ((numprims-1) + i)
#define FM_NODEIDX(i) (i)

    // Calculates longest common prefix length of bit representations
    // if  representations are equal we consider sucessive indices
    int FmDelta(int* mortoncodes, int numprims, int i1, int i2)
    {
        // Select left end
        int left = FmMinInt(i1, i2);
        // Select right end
        int right = FmMaxInt(i1, i2);
        // This is to ensure the node breaks if the index is out of bounds
        if (left < 0 || right >= numprims)
        {
            return -1;
        }
        // Fetch Morton codes for both ends
        int leftcode = mortoncodes[left];
        int rightcode = mortoncodes[right];

        // Special handling of duplicated codes: use their indices as a fallback
        return leftcode != rightcode ? (int)FmCountLeadingZeros((uint)(leftcode ^ rightcode)) : (int)(32 + FmCountLeadingZeros((uint)(left ^ right)));
    }

    // Shortcut for delta evaluation
#define FM_DELTA(i,j) FmDelta(mortoncodes,numprims,i,j)

    // Find span occupied by internal node with index idx
    int2 FmFindSpan(int* mortoncodes, int numprims, int idx)
    {
        // Find the direction of the range
        int d = FmSign((float)(FM_DELTA(idx, idx + 1) - FM_DELTA(idx, idx - 1)));

        // Find minimum number of bits for the break on the other side
        int deltamin = FM_DELTA(idx, idx - d);

        // Search conservative far end
        int lmax = 2;
        while (FM_DELTA(idx, idx + lmax * d) > deltamin)
            lmax *= 2;

        // Search back to find exact bound
        // with binary search
        int l = 0;
        int t = lmax;
        do
        {
            t /= 2;
            if (FM_DELTA(idx, idx + (l + t)*d) > deltamin)
            {
                l = l + t;
            }
        } while (t > 1);

        // Pack span 
        int2 span;
        span.x = FmMinInt(idx, idx + l*d);
        span.y = FmMaxInt(idx, idx + l*d);
        return span;
    }

    // Find split idx within the span
    int FmFindSplit(int* mortoncodes, int numprims, int2 span)
    {
        // Fetch codes for both ends
        int left = span.x;
        int right = span.y;

        // Calculate the number of identical bits from higher end
        int numidentical = FM_DELTA(left, right);

        do
        {
            // Proposed split
            int newsplit = (right + left) / 2;

            // If it has more equal leading bits than left and right accept it
            if (FM_DELTA(left, newsplit) > numidentical)
            {
                left = newsplit;
            }
            else
            {
                right = newsplit;
            }
        } while (right > left + 1);

        return left;
    }

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
        FmBvhNode* nodes
    )
    {
        // Set child
        for (int globalid = 0; globalid < numprims; globalid++)
        {
            nodes[FM_LEAFIDX(globalid)].left = nodes[FM_LEAFIDX(globalid)].right = indices[globalid];
            nodes[FM_LEAFIDX(globalid)].box = bounds[indices[globalid]];
        }

        // Set internal nodes
        for (int globalid = 0; globalid < numprims - 1; globalid++)
        {
            // Find span occupied by the current node
            int2 range = FmFindSpan(mortoncodes, numprims, globalid);

            // Find split position inside the range
            int  split = FmFindSplit(mortoncodes, numprims, range);

            // Create child nodes if needed
            int c1idx = (split == range.x) ? FM_LEAFIDX(split) : FM_NODEIDX(split);
            int c2idx = (split + 1 == range.y) ? FM_LEAFIDX(split + 1) : FM_NODEIDX(split + 1);

            nodes[FM_NODEIDX(globalid)].left = c1idx;
            nodes[FM_NODEIDX(globalid)].right = c2idx;
            //nodes[FM_NODEIDX(globalid)].next = (range.y + 1 < numprims) ? range.y + 1 : -1;
            //nodes[c1idx].parent = FM_NODEIDX(globalid);
            //nodes[c1idx].next = c2idx;
            //nodes[c2idx].parent = FM_NODEIDX(globalid);
            //nodes[c2idx].next = nodes[FM_NODEIDX(globalid)].next;
        }
    }

    // Compute bounds with recursion
    FmAabb FmRefitBboxRecursive(
        int numprims,
        FmBvhNode* nodes,
        int idx
    )
    {
        // Fetch kids
        int lc = nodes[idx].left;
        int rc = nodes[idx].right;

        if (lc == rc)
        {
            return nodes[idx].box;
        }

        // Calculate bounds
        FmAabb bl = FmRefitBboxRecursive(numprims, nodes, lc);
        FmAabb br = FmRefitBboxRecursive(numprims, nodes, rc);
        FmAabb b = FmBboxUnion(bl, br);

        // Write bounds
        nodes[idx].box = b;

        return b;
    }

    void FmRefitBounds(
        int numprims,
        FmBvhNode* nodes
    )
    {
        FmRefitBboxRecursive(numprims, nodes, 0);
    }

    static int FmCompareInts(const void* a, const void* b)
    {
        int ia = *(int *)a;
        int ib = *(int *)b;

        return FM_QSORT_INCREASING_RETVAL(ia, ib);
    }

    void FmBuildBvhOnLeaves(FmBvh* resultBvh, const FmVector3& minPosition, const FmVector3& maxPosition)
    {
        FmBvh& bvh = *resultBvh;

        uint numPrims = bvh.numPrims;
        if (numPrims == 0)
        {
            return;
        }

        FmCalcMortonCode(minPosition, maxPosition, bvh.primBoxes, (int)numPrims, bvh.mortonCodesSorted);

        for (uint i = 0; i < numPrims; i++)
        {
            bvh.primIndicesSorted[i] = (int)i;
        }
        qsort_permutation(bvh.mortonCodesSorted, bvh.primIndicesSorted, (size_t)bvh.numPrims, sizeof(int), FmCompareInts);

        FmBuildHierarchy(bvh.mortonCodesSorted, bvh.primBoxes, bvh.primIndicesSorted, (int)bvh.numPrims, bvh.nodes);

        FmRefitBounds((int)bvh.numPrims, bvh.nodes);
    }

    FmBvh* FmCreateBvh(uint numPrims)
    {
        FmBvh* bvh = FmMalloc<FmBvh>();
        uint numNodes = FmNumBvhNodes(numPrims);
        bvh->nodes = FmMallocArray<FmBvhNode>(numNodes);
        bvh->primBoxes = FmMallocArray<FmAabb>(numPrims);
        bvh->mortonCodesSorted = FmMallocArray<int>(numPrims);
        bvh->primIndicesSorted = FmMallocArray<int>(numPrims);
        bvh->numPrims = numPrims;
        return bvh;
    }

    void FmDestroyBvh(FmBvh* bvh)
    {
        if (!bvh)
        {
            return;
        }

        FmAlignedFree(bvh->nodes);
        FmAlignedFree(bvh->primBoxes);
        FmAlignedFree(bvh->mortonCodesSorted);
        FmAlignedFree(bvh->primIndicesSorted);
        FmAlignedFree(bvh);
    }

    void FmGetBoundingBox(FmVector3* minPosition, FmVector3* maxPosition, const FmBvh& bvh)
    {
        *minPosition = bvh.nodes[0].box.pmin;
        *maxPosition = bvh.nodes[0].box.pmax;
    }
}