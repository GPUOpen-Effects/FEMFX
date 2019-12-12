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

#pragma once

#include "FEMFXTypes.h"

namespace AMD
{
    // Moving AABB defined by corners of starting box and min/max velocity of geometry inside the box.
    struct FmAabb
    {
        FmVector3 pmin;
        FmVector3 pmax;
        FmVector3 vmin;
        FmVector3 vmax;

        // Extend bounds to cover another box
        inline void operator += (const FmAabb& other)
        {
            pmin = min(pmin, other.pmin);
            pmax = max(pmax, other.pmax);
            vmin = min(vmin, other.vmin);
            vmax = max(vmax, other.vmax);
        }
    };

    static FM_FORCE_INLINE bool FmAabbOverlap(const FmVector3& minCornerA, const FmVector3& maxCornerA, const FmVector3& minCornerB, const FmVector3& maxCornerB)
    {
        return (
            maxCornerA.x >= minCornerB.x && maxCornerA.y >= minCornerB.y && maxCornerA.z >= minCornerB.z &&
            maxCornerB.x >= minCornerA.x && maxCornerB.y >= minCornerA.y && maxCornerB.z >= minCornerA.z);
    }

    bool FmAabbCcd(float& impactTime, const FmAabb& aabb0, const FmAabb& aabb1, float deltaTime);
}
