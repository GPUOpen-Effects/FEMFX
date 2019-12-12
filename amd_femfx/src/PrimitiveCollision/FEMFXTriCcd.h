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
// Continuous collision detection for points, line segments, and triangles whose vertices 
// move linearly within the timestep
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXAabb.h"
#include "FEMFXFeaturePairDistance.h"

#define FM_DEFAULT_MAX_CCD_ITERATIONS 4

namespace AMD
{
    // CCD operations between triangle features whose vertices move linearly within the timestep.
    // Reference: Brian Mirtich, "Impulse Based Dynamic Simulation of Rigid Body Systems"  (PhD thesis)

    // Triangle defined by three vertex starting positions and three velocities
    struct FmTri
    {
        FmVector3 pos0;
        FmVector3 pos1;
        FmVector3 pos2;
        FmVector3 vel0;
        FmVector3 vel1;
        FmVector3 vel2;
        uint id;

        FmTri() : id(0x3f) {}

        const FmVector3& Pos(uint i) const { return *(&pos0 + i); }
        const FmVector3& Vel(uint i) const { return *(&vel0 + i); }
        const FmVector3 Pos(uint i, float deltaTime) const { return Pos(i) + Vel(i)*deltaTime; }

        inline uint GetTriId() const
        {
            return id >> 6;
        }

        // Use id bits to assign ownership of features.
        // Reference: Curtis et al., "Fast Collision Detection for Deformable Models using Representative-Triangles"
        inline bool OwnsVert(uint i) const
        {
            return (id & (1 << i)) != 0;
        }
        inline bool OwnsEdge(uint i) const
        {
            return (id & (1 << (i + 3))) != 0;
        }
        inline bool OwnsAll() const
        {
            return (id & 0x3f) == 0x3f;
        }
        inline bool OwnsAny() const
        {
            return (id & 0x3f) != 0;
        }
    };

    enum FmCcdReturn
    {
        FM_CCD_RET_NO_IMPACT = 0,
        FM_CCD_RET_IMPACT,
        FM_CCD_RET_INITIALLY_CONTACTING,  // Initial distance <= contact gap
        FM_CCD_RET_MIN_TIME_STEP,
        FM_CCD_RET_MAX_ITERATIONS
    };

    struct FmCcdResult
    {
        FmVector3     direction;   // Direction pointing from i to j
        FmVector3     posi;
        FmVector3     posj;
        float            distance;
        float            time;
        uint             numIterations;
        FmFeaturePair featurePair;
        FmCcdReturn   retVal;
    };

    struct FmCcdTerminationConditions
    {
        float impactGap;          // Distance between tris at time of impact.  impactGap may be the sum of triangle thicknesses.
        float contactGap;         // Distance defining contact condition at beginning of timestep.  Assumed to be greater than impactGap.
        float impactGapAbsError;  // Search terminates if distance within this absolute error of impact gap.
        float impactGapRelError;  // Search terminates if distance within this relative error of impact gap.
        float minTimeStep;        // Search terminates if time step of next iteration is less than this minimum.
        uint  maxIterations;      // Search terminates if maximum iterations reached.

        FmCcdTerminationConditions() : impactGap(0.0f), contactGap(0.0f), impactGapAbsError(FLT_EPSILON), impactGapRelError(FLT_EPSILON), minTimeStep(FLT_EPSILON), maxIterations(FM_DEFAULT_MAX_CCD_ITERATIONS) {}
    };

    struct FmEdgeEdgePairData
    {
        FmVector3 eApos[2];
        FmVector3 eAvel[2];
        FmVector3 eBpos[2];
        FmVector3 eBvel[2];
        uint i;                // Triangle corner indices of edge start verts
        uint j;
    };

    struct FmVertexFacePairData
    {
        FmVector3 vApos;
        FmVector3 vAvel;
        FmVector3 fBpos[3];
        FmVector3 fBvel[3];
        uint i;                // Triangle corner index of vA
    };

    struct FmFaceVertexPairData
    {
        FmVector3 fApos[3];
        FmVector3 fAvel[3];
        FmVector3 vBpos;
        FmVector3 vBvel;
        uint j;                // Triangle corner index of vB
    };

    // CCD test of two edges whose vertices move linearly during deltaTime
    void FmEdgeEdgeCcd(
        FmCcdResult* contact,
        const FmEdgeEdgePairData& eePair,
        float deltaTime,
        const FmCcdTerminationConditions& conditions);

    // CCD test of a vertex vs. triangle face whose vertices move linearly during deltaTime
    void FmVertexFaceCcd(
        FmCcdResult* contact,
        const FmVertexFacePairData& vfPair,
        float deltaTime,
        const FmCcdTerminationConditions& conditions);

    // CCD test of a triangle face vs. vertex whose vertices move linearly during deltaTime
    void FmFaceVertexCcd(
        FmCcdResult* contact,
        const FmFaceVertexPairData& fvPair,
        float deltaTime,
        const FmCcdTerminationConditions& conditions);

    static FM_FORCE_INLINE FmFeaturePair FmReverseFeaturePair(const FmFeaturePair& featurePair)
    {
        FmFeaturePair result;
        result.itype = featurePair.jtype;
        result.jtype = featurePair.itype;
        result.i0 = featurePair.j0;
        result.i1 = featurePair.j1;
        result.i2 = featurePair.j2;
        result.j0 = featurePair.i0;
        result.j1 = featurePair.i1;
        result.j2 = featurePair.i2;
        result.t = featurePair.u;
        result.u = featurePair.t;
        return result;
    }

    // Compute an AABB with velocity that bounds a triangle in motion
    static FM_FORCE_INLINE FmAabb FmComputeTriAabb(const FmTri& tri, float deltaTime, float padding)
    {
        FmVector3 vpadding = FmInitVector3(padding);

        FmAabb aabb;
        aabb.pmin = min(tri.pos0, min(tri.pos1, tri.pos2)) - vpadding;
        aabb.pmax = max(tri.pos0, max(tri.pos1, tri.pos2)) + vpadding;

        FmVector3 triPos0End = tri.pos0 + tri.vel0 * deltaTime;
        FmVector3 triPos1End = tri.pos1 + tri.vel1 * deltaTime;
        FmVector3 triPos2End = tri.pos2 + tri.vel2 * deltaTime;

        FmVector3 pminEnd = min(triPos0End, min(triPos1End, triPos2End)) - vpadding;
        FmVector3 pmaxEnd = max(triPos0End, max(triPos1End, triPos2End)) + vpadding;
        aabb.vmin = (pminEnd - aabb.pmin) / deltaTime;
        aabb.vmax = (pmaxEnd - aabb.pmax) / deltaTime;

        return aabb;
    }
}
