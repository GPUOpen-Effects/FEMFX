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
// Statistics used for sleeping test
//---------------------------------------------------------------------------------------
#pragma once

#include "FEMFXCommonInternal.h"

namespace AMD
{
    // Statistics for sleeping test
    struct FmVelStats
    {
        float maxSpeed;    // Maximum speed of vertex in mesh 
        float avgSpeed;    // Average speed of all vertices in mesh
        uint numSamples;   // Number of samples collected
        uint stableCount;  // Number of steps under max speed threshold
    };

    static FM_FORCE_INLINE void FmInitVelStats(FmVelStats* velStats)
    {
        velStats->maxSpeed = 0.0f;
        velStats->avgSpeed = 0.0f;
        velStats->numSamples = 0;
        velStats->stableCount = 0;
    }

    // Update stats using a velocity sample.
    // Speeds are averaged with an incremental calculation.
    static FM_FORCE_INLINE void FmUpdateVelStats(FmVelStats* velStats, float speed, uint maxNumSamples)
    {
        FmVelStats& stats = *velStats;

        if (stats.numSamples == 0)
        {
            stats.maxSpeed = speed;
            stats.avgSpeed = speed;
        }
        else
        {
            // Update max
            if (speed > stats.maxSpeed)
            {
                stats.maxSpeed = speed;
            }

            // Update mean and variance
            float mean = stats.avgSpeed;

            mean = mean + (1.0f / (float)stats.numSamples) * (speed - mean);

            // Update stats
            stats.avgSpeed = mean;
        }

        if (stats.numSamples < maxNumSamples)
        {
            stats.numSamples++;
        }
    }

    // Check if stats fall within specified limits for minStableCount, or reset
    static FM_FORCE_INLINE bool FmCheckStable(FmVelStats* velStats, float maxSpeedThreshold, float avgSpeedThreshold, uint minStableCount)
    {
        FmVelStats& stats = *velStats;

        if (stats.maxSpeed <= maxSpeedThreshold
            && stats.avgSpeed <= avgSpeedThreshold)
        {
            stats.stableCount++;
            if (stats.stableCount > minStableCount)
            {
                return true;
            }
        }
        else
        {
            // If fails stable test, reset count and stats
            stats.numSamples = 0;
            stats.stableCount = 0;
        }

        return false;
    }

}