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
// Continuous collision detection of AABBs with bounds on min and max velocity
//---------------------------------------------------------------------------------------

#include "FEMFXTriCcd.h"

namespace AMD
{

    static FM_FORCE_INLINE bool FmAabbCcdOverlapYZ(const FmAabb& aabb0, const FmAabb& aabb1, float t)
    {
        float proj01Y = (aabb0.pmin.y + aabb0.vmin.y * t) - (aabb1.pmax.y + aabb1.vmax.y * t);
        float proj01Z = (aabb0.pmin.z + aabb0.vmin.z * t) - (aabb1.pmax.z + aabb1.vmax.z * t);
        float proj10Y = (aabb1.pmin.y + aabb1.vmin.y * t) - (aabb0.pmax.y + aabb0.vmax.y * t);
        float proj10Z = (aabb1.pmin.z + aabb1.vmin.z * t) - (aabb0.pmax.z + aabb0.vmax.z * t);

        return (proj01Y <= 0.0f && proj01Z <= 0.0f && proj10Y <= 0.0f && proj10Z <= 0.0f);
    }

    static FM_FORCE_INLINE bool FmAabbCcdOverlapXZ(const FmAabb& aabb0, const FmAabb& aabb1, float t)
    {
        float proj01X = (aabb0.pmin.x + aabb0.vmin.x * t) - (aabb1.pmax.x + aabb1.vmax.x * t);
        float proj01Z = (aabb0.pmin.z + aabb0.vmin.z * t) - (aabb1.pmax.z + aabb1.vmax.z * t);
        float proj10X = (aabb1.pmin.x + aabb1.vmin.x * t) - (aabb0.pmax.x + aabb0.vmax.x * t);
        float proj10Z = (aabb1.pmin.z + aabb1.vmin.z * t) - (aabb0.pmax.z + aabb0.vmax.z * t);

        return (proj01X <= 0.0f && proj01Z <= 0.0f && proj10X <= 0.0f && proj10Z <= 0.0f);
    }

    static FM_FORCE_INLINE bool FmAabbCcdOverlapXY(const FmAabb& aabb0, const FmAabb& aabb1, float t)
    {
        float proj01X = (aabb0.pmin.x + aabb0.vmin.x * t) - (aabb1.pmax.x + aabb1.vmax.x * t);
        float proj01Y = (aabb0.pmin.y + aabb0.vmin.y * t) - (aabb1.pmax.y + aabb1.vmax.y * t);
        float proj10X = (aabb1.pmin.x + aabb1.vmin.x * t) - (aabb0.pmax.x + aabb0.vmax.x * t);
        float proj10Y = (aabb1.pmin.y + aabb1.vmin.y * t) - (aabb0.pmax.y + aabb0.vmax.y * t);

        return (proj01X <= 0.0f && proj01Y <= 0.0f && proj10X <= 0.0f && proj10Y <= 0.0f);
    }

    static FM_FORCE_INLINE bool FmAabbCcdOverlap(const FmAabb& aabb0, const FmAabb& aabb1, float t)
    {
        FmVector3 min0CornerPos, max0CornerPos, min1CornerPos, max1CornerPos;

        min0CornerPos = aabb0.pmin + aabb0.vmin * t;
        max0CornerPos = aabb0.pmax + aabb0.vmax * t;
        min1CornerPos = aabb1.pmin + aabb1.vmin * t;
        max1CornerPos = aabb1.pmax + aabb1.vmax * t;

        float proj01X = min0CornerPos.x - max1CornerPos.x;
        float proj01Y = min0CornerPos.y - max1CornerPos.y;
        float proj01Z = min0CornerPos.z - max1CornerPos.z;
        float proj10X = min1CornerPos.x - max0CornerPos.x;
        float proj10Y = min1CornerPos.y - max0CornerPos.y;
        float proj10Z = min1CornerPos.z - max0CornerPos.z;

        return (proj01X <= 0.0f && proj01Y <= 0.0f && proj01Z <= 0.0f && proj10X <= 0.0f && proj10Y <= 0.0f && proj10Z <= 0.0f);
    }

    // Find the impact time of two AABBs that move/deform during the time interval.
    // The FmAabb motion is defined by constant velocities of the min and max corners
    // at the start of the step towards min and max corners respectively at the end 
    // of the step.  
    // 
    // Vertices contained within the FmAabb at the start and end times also moving at 
    // constant velocities will be bounded for the entire interval.

    bool FmAabbCcd(float& impactTime, const FmAabb& aabb0, const FmAabb& aabb1, float deltaTime)
    {
#if 0
        float proj01X = aabb0.pmin.x - aabb1.pmax.x;
        float proj01Y = aabb0.pmin.y - aabb1.pmax.y;
        float proj01Z = aabb0.pmin.z - aabb1.pmax.z;
        float proj10X = aabb1.pmin.x - aabb0.pmax.x;
        float proj10Y = aabb1.pmin.y - aabb0.pmax.y;
        float proj10Z = aabb1.pmin.z - aabb0.pmax.z;

        if (proj01X <= 0.0f && proj01Y <= 0.0f && proj01Z <= 0.0f && proj10X <= 0.0f && proj10Y <= 0.0f && proj10Z <= 0.0f)
        {
            impactTime = 0.0f;
            return true;
        }

        float dVel01X = aabb1.vmax.x - aabb0.vmin.x;
        float dVel01Y = aabb1.vmax.y - aabb0.vmin.y;
        float dVel01Z = aabb1.vmax.z - aabb0.vmin.z;
        float dVel10X = aabb0.vmax.x - aabb1.vmin.x;
        float dVel10Y = aabb0.vmax.y - aabb1.vmin.y;
        float dVel10Z = aabb0.vmax.z - aabb1.vmin.z;

        // Make any negative velocities 0, causing inf below
        dVel01X = FmMaxFloat(dVel01X, 0.0f);
        dVel01Y = FmMaxFloat(dVel01Y, 0.0f);
        dVel01Z = FmMaxFloat(dVel01Z, 0.0f);
        dVel10X = FmMaxFloat(dVel10X, 0.0f);
        dVel10Y = FmMaxFloat(dVel10Y, 0.0f);
        dVel10Z = FmMaxFloat(dVel10Z, 0.0f);

        // Compute time to cross projected distances, and take min
        float dt01X = (proj01X <= 0.0f) ? FLT_MAX : proj01X / dVel01X;
        float dt01Y = (proj01Y <= 0.0f) ? FLT_MAX : proj01Y / dVel01Y;
        float dt01Z = (proj01Z <= 0.0f) ? FLT_MAX : proj01Z / dVel01Z;
        float dt10X = (proj10X <= 0.0f) ? FLT_MAX : proj10X / dVel10X;
        float dt10Y = (proj10Y <= 0.0f) ? FLT_MAX : proj10Y / dVel10Y;
        float dt10Z = (proj10Z <= 0.0f) ? FLT_MAX : proj10Z / dVel10Z;

        float dtX = FmMinFloat(dt01X, dt10X);
        float dtY = FmMinFloat(dt01Y, dt10Y);
        float dtZ = FmMinFloat(dt01Z, dt10Z);

        if (dtX < deltaTime && FmAabbCcdOverlapYZ(aabb0, aabb1, dtX))
        {
            impactTime = dtX;
            return true;
        }

        if (dtY < deltaTime && FmAabbCcdOverlapXZ(aabb0, aabb1, dtY))
        {
            impactTime = dtY;
            return true;
        }

        if (dtZ < deltaTime && FmAabbCcdOverlapXY(aabb0, aabb1, dtZ))
        {
            impactTime = dtZ;
            return true;
        }

        impactTime = deltaTime;
        return false;
#else
        FmSimdVector3 pmin0 = FmInitSimdVector3(aabb0.pmin);
        FmSimdVector3 pmax0 = FmInitSimdVector3(aabb0.pmax);
        FmSimdVector3 pmin1 = FmInitSimdVector3(aabb1.pmin);
        FmSimdVector3 pmax1 = FmInitSimdVector3(aabb1.pmax);
        FmSimdVector3 vmin0 = FmInitSimdVector3(aabb0.vmin);
        FmSimdVector3 vmax0 = FmInitSimdVector3(aabb0.vmax);
        FmSimdVector3 vmin1 = FmInitSimdVector3(aabb1.vmin);
        FmSimdVector3 vmax1 = FmInitSimdVector3(aabb1.vmax);

        FmSoa4Float proj01((pmin0 - pmax1).get128());
        FmSoa4Float proj10((pmin1 - pmax0).get128());

        if (all(proj01 <= 0.0f) && all(proj10 <= 0.0f))
        {
            impactTime = 0.0f;
            return true;
        }

        FmSoa4Float dVel01((vmax1 - vmin0).get128());
        FmSoa4Float dVel10((vmax0 - vmin1).get128());

        // Make any negative velocities 0, causing inf below
        FmSoa4Float dVel01Clamped = max(dVel01, 0.0f);
        FmSoa4Float dVel10Clamped = max(dVel10, 0.0f);

        // Compute time to cross projected distances, and take min
        FmSoa4Float dt01 = select(proj01 / dVel01Clamped, FLT_MAX, proj01 <= 0.0f);
        FmSoa4Float dt10 = select(proj10 / dVel10Clamped, FLT_MAX, proj10 <= 0.0f);
       
        FmSoa4Float projYZ(simd_shuffle_yzyz(proj01.get128(), proj10.get128()));
        FmSoa4Float projZX(simd_shuffle_zxzx(proj01.get128(), proj10.get128()));
        FmSoa4Float projXY(simd_shuffle_xyxy(proj01.get128(), proj10.get128()));

        FmSoa4Float dVelYZ(simd_shuffle_yzyz(dVel01.get128(), dVel10.get128()));
        FmSoa4Float dVelZX(simd_shuffle_zxzx(dVel01.get128(), dVel10.get128()));
        FmSoa4Float dVelXY(simd_shuffle_xyxy(dVel01.get128(), dVel10.get128()));

        FmSoa4Float dt = min(dt01, dt10);
        float dtX = simd_getx_ps(dt.get128());
        float dtY = simd_gety_ps(dt.get128());
        float dtZ = simd_getz_ps(dt.get128());

        projYZ = projYZ - dVelYZ * dtX;
        projZX = projZX - dVelZX * dtY;
        projXY = projXY - dVelXY * dtZ;

        if (dtX < deltaTime && all(projYZ <= 0.0f))
        {
            impactTime = dtX;
            return true;
        }

        if (dtY < deltaTime && all(projZX <= 0.0f))
        {
            impactTime = dtY;
            return true;
        }

        if (dtZ < deltaTime && all(projXY <= 0.0f))
        {
            impactTime = dtZ;
            return true;
        }

        impactTime = deltaTime;
        return false;
#endif
    }
}