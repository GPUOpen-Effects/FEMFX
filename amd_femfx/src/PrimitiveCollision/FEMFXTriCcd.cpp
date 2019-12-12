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

#include "FEMFXTriCcd.h"
#include "FEMFXFeaturePairDistance.h"

namespace AMD
{

    // Earliest time that vertex and triangle may reach gap distance, assuming vertices have constant velocities.
    static inline float FmVertexTriangleEarliestImpact(const FmVector3& normal, const FmVertexFacePairData& vfPair, float gap)
    {
        float proj0 = dot(normal, vfPair.fBpos[0] - vfPair.vApos);
        float proj1 = dot(normal, vfPair.fBpos[1] - vfPair.vApos);
        float proj2 = dot(normal, vfPair.fBpos[2] - vfPair.vApos);

        // Relative velocities between pairs of points projected on normal
        float vel0 = dot(normal, vfPair.vAvel);
        float vB0Proj = dot(normal, vfPair.fBvel[0]);
        float vB1Proj = dot(normal, vfPair.fBvel[1]);
        float vB2Proj = dot(normal, vfPair.fBvel[2]);

        float dVel0 = vel0 - vB0Proj;
        float dVel1 = vel0 - vB1Proj;
        float dVel2 = vel0 - vB2Proj;

        // Make any negative velocities 0, causing inf below
        dVel0 = FmMaxFloat(dVel0, 0.0f);
        dVel1 = FmMaxFloat(dVel1, 0.0f);
        dVel2 = FmMaxFloat(dVel2, 0.0f);

        // Projected distance between pairs of points
        float dist0 = proj0 - gap;
        float dist1 = proj1 - gap;
        float dist2 = proj2 - gap;

        // Compute time to cross projected distances, and take min
        float dt0 = (dist0 <= 0.0f) ? 0.0f : dist0 / dVel0;
        float dt1 = (dist1 <= 0.0f) ? 0.0f : dist1 / dVel1;
        float dt2 = (dist2 <= 0.0f) ? 0.0f : dist2 / dVel2;

        float dt = FmMinFloat(dt0, FmMinFloat(dt1, dt2));

        return dt;
    }

    // Earliest time that edge pair may reach gap distance, assuming vertices have constant velocities.
    static inline float FmEdgePairEarliestImpact(const FmVector3& normal, const FmEdgeEdgePairData& eePair, float gap)
    {
        float proj00 = dot(normal, eePair.eBpos[0] - eePair.eApos[0]);
        float proj01 = dot(normal, eePair.eBpos[1] - eePair.eApos[0]);
        float proj10 = dot(normal, eePair.eBpos[0] - eePair.eApos[1]);
        float proj11 = dot(normal, eePair.eBpos[1] - eePair.eApos[1]);

        // Relative velocities between pairs of points projected on normal
        float vA0Proj = dot(normal, eePair.eAvel[0]);
        float vA1Proj = dot(normal, eePair.eAvel[1]);
        float vB0Proj = dot(normal, eePair.eBvel[0]);
        float vB1Proj = dot(normal, eePair.eBvel[1]);

        float dVel00 = vA0Proj - vB0Proj;
        float dVel01 = vA0Proj - vB1Proj;
        float dVel10 = vA1Proj - vB0Proj;
        float dVel11 = vA1Proj - vB1Proj;

        // Make any negative velocities 0, causing inf below
        dVel00 = FmMaxFloat(dVel00, 0.0f);
        dVel01 = FmMaxFloat(dVel01, 0.0f);
        dVel10 = FmMaxFloat(dVel10, 0.0f);
        dVel11 = FmMaxFloat(dVel11, 0.0f);

        // Projected distance between pairs of points
        float dist00 = proj00 - gap;
        float dist01 = proj01 - gap;
        float dist10 = proj10 - gap;
        float dist11 = proj11 - gap;

        // Compute time to cross projected distances, and take min
        float dt00 = (dist00 <= 0.0f) ? 0.0f : dist00 / dVel00;
        float dt01 = (dist01 <= 0.0f) ? 0.0f : dist01 / dVel01;
        float dt10 = (dist10 <= 0.0f) ? 0.0f : dist10 / dVel10;
        float dt11 = (dist11 <= 0.0f) ? 0.0f : dist11 / dVel11;

        float dt = FmMinFloat(dt00, FmMinFloat(dt01, FmMinFloat(dt10, dt11)));

        return dt;
    }

    void FmEdgeEdgeCcd(
        FmCcdResult* contact,
        const FmEdgeEdgePairData& eePair,
        float deltaTime,
        const FmCcdTerminationConditions& conditions)
    {
        float impactGapRelScale = conditions.impactGapRelError + 1.0f;

        // Compute distance of edge pair
        FmDistanceResult distResult;

        uint i = eePair.i;
        uint j = eePair.j;
        uint i1 = (i + 1) % 3;
        uint j1 = (j + 1) % 3;

        FmSegmentPairDistance(&distResult, eePair.eApos[0], eePair.eApos[1], eePair.eBpos[0], eePair.eBpos[1], i, i1, j, j1);

        // Initialize contact with closest points
        contact->direction = distResult.direction;
        contact->posi = distResult.posi;
        contact->posj = distResult.posj;
        contact->distance = distResult.distance;
        contact->time = 0.0f;
        contact->numIterations = 0;
        contact->featurePair = distResult.featurePair;

        // Return if distance is less than contact gap
        if (distResult.distance <= conditions.contactGap)
        {
            contact->retVal = FM_CCD_RET_INITIALLY_CONTACTING;
            return;
        }

        // Initialize time of impact bound
        float dt = FmEdgePairEarliestImpact(distResult.direction, eePair, conditions.impactGap);

        if (dt > deltaTime)
        {
            contact->time = deltaTime;
            contact->retVal = FM_CCD_RET_NO_IMPACT;
            return;
        }
        else if (dt <= conditions.minTimeStep)
        {
            contact->retVal = FM_CCD_RET_MIN_TIME_STEP;
            return;
        }

        float t = dt;

        // Iteratively advance time, compute distance and next impact time bound, until it reaches impact distance or time bound is later than deltaTime
        uint iter = 0;
        while (iter++ < conditions.maxIterations)
        {
            contact->numIterations = iter;

            FmEdgeEdgePairData eePairAdv;
            eePairAdv.eApos[0] = eePair.eAvel[0] * t + eePair.eApos[0];
            eePairAdv.eApos[1] = eePair.eAvel[1] * t + eePair.eApos[1];
            eePairAdv.eAvel[0] = eePair.eAvel[0];
            eePairAdv.eAvel[1] = eePair.eAvel[1];
            eePairAdv.eBpos[0] = eePair.eBvel[0] * t + eePair.eBpos[0];
            eePairAdv.eBpos[1] = eePair.eBvel[1] * t + eePair.eBpos[1];
            eePairAdv.eBvel[0] = eePair.eBvel[0];
            eePairAdv.eBvel[1] = eePair.eBvel[1];
            eePairAdv.i = i;
            eePairAdv.j = j;

            FmSegmentPairDistance(&distResult, eePairAdv.eApos[0], eePairAdv.eApos[1], eePairAdv.eBpos[0], eePairAdv.eBpos[1], i, i1, j, j1);

            contact->direction = distResult.direction;
            contact->posi = distResult.posi;
            contact->posj = distResult.posj;
            contact->distance = distResult.distance;
            contact->time = t;
            contact->featurePair = distResult.featurePair;

            if (distResult.distance <= FmMaxFloat(conditions.impactGap + conditions.impactGapAbsError, conditions.impactGap * impactGapRelScale))
            {
                contact->retVal = FM_CCD_RET_IMPACT;
                return;
            }

            dt = FmEdgePairEarliestImpact(distResult.direction, eePairAdv, conditions.impactGap);

            float tnext = t + dt;

            if (tnext > deltaTime)
            {
                contact->time = deltaTime;
                contact->retVal = FM_CCD_RET_NO_IMPACT;
                return;
            }
            else if (tnext - t <= conditions.minTimeStep)
            {
                contact->retVal = FM_CCD_RET_MIN_TIME_STEP;
                return;
            }

            t = tnext;
        }

        contact->retVal = FM_CCD_RET_MAX_ITERATIONS;
    }

    void FmVertexFaceCcd(
        FmCcdResult* contact,
        const FmVertexFacePairData& vfPair,
        float deltaTime,
        const FmCcdTerminationConditions& conditions)
    {
        float impactGapRelScale = conditions.impactGapRelError + 1.0f;

        // Compute distance of vertex/triangle pair
        FmDistanceResult distResult;

        uint i = vfPair.i;

        FmPointTriangleDistance(&distResult, vfPair.vApos, vfPair.fBpos[0], vfPair.fBpos[1], vfPair.fBpos[2], i, 0, 1, 2);

        // Initialize contact with closest points
        contact->direction = distResult.direction;
        contact->posi = vfPair.vApos;
        contact->posj = distResult.posj;
        contact->distance = distResult.distance;
        contact->time = 0.0f;
        contact->numIterations = 0;
        contact->featurePair = distResult.featurePair;

        // Return if distance is less than contact gap
        if (distResult.distance <= conditions.contactGap)
        {
            contact->retVal = FM_CCD_RET_INITIALLY_CONTACTING;
            return;
        }

        // Initialize time of impact bound
        float dt = FmVertexTriangleEarliestImpact(distResult.direction, vfPair, conditions.impactGap);

        if (dt > deltaTime)
        {
            contact->time = deltaTime;
            contact->retVal = FM_CCD_RET_NO_IMPACT;
            return;
        }
        else if (dt <= conditions.minTimeStep)
        {
            contact->retVal = FM_CCD_RET_MIN_TIME_STEP;
            return;
        }

        float t = dt;

        // Iteratively advance time, compute distance and next impact time bound, until it reaches impact distance or time bound is later than deltaTime
        uint iter = 0;
        while (iter++ < conditions.maxIterations)
        {
            contact->numIterations = iter;

            FmVertexFacePairData vfPairAdv;
            vfPairAdv.vApos = vfPair.vAvel * t + vfPair.vApos;
            vfPairAdv.vAvel = vfPair.vAvel;
            vfPairAdv.fBpos[0] = vfPair.fBvel[0] * t + vfPair.fBpos[0];
            vfPairAdv.fBpos[1] = vfPair.fBvel[1] * t + vfPair.fBpos[1];
            vfPairAdv.fBpos[2] = vfPair.fBvel[2] * t + vfPair.fBpos[2];
            vfPairAdv.fBvel[0] = vfPair.fBvel[0];
            vfPairAdv.fBvel[1] = vfPair.fBvel[1];
            vfPairAdv.fBvel[2] = vfPair.fBvel[2];
            vfPairAdv.i = i;

            FmPointTriangleDistance(&distResult, vfPairAdv.vApos, vfPairAdv.fBpos[0], vfPairAdv.fBpos[1], vfPairAdv.fBpos[2], i, 0, 1, 2);

            contact->direction = distResult.direction;
            contact->posi = distResult.posi;
            contact->posj = distResult.posj;
            contact->distance = distResult.distance;
            contact->time = t;
            contact->featurePair = distResult.featurePair;

            if (distResult.distance <= FmMaxFloat(conditions.impactGap + conditions.impactGapAbsError, conditions.impactGap * impactGapRelScale))
            {
                contact->retVal = FM_CCD_RET_IMPACT;
                return;
            }

            dt = FmVertexTriangleEarliestImpact(distResult.direction, vfPairAdv, conditions.impactGap);

            float tnext = t + dt;

            if (tnext > deltaTime)
            {
                contact->time = deltaTime;
                contact->retVal = FM_CCD_RET_NO_IMPACT;
                return;
            }
            else if (tnext - t <= conditions.minTimeStep)
            {
                contact->retVal = FM_CCD_RET_MIN_TIME_STEP;
                return;
            }

            t = tnext;
        }

        contact->retVal = FM_CCD_RET_MAX_ITERATIONS;
    }

    void FmFaceVertexCcd(
        FmCcdResult* contact,
        const FmFaceVertexPairData& fvPair,
        float deltaTime,
        const FmCcdTerminationConditions& conditions)
    {
        FmCcdResult tmpContact;
        FmVertexFacePairData tmpVFPairData;
        tmpVFPairData.fBpos[0] = fvPair.fApos[0];
        tmpVFPairData.fBpos[1] = fvPair.fApos[1];
        tmpVFPairData.fBpos[2] = fvPair.fApos[2];
        tmpVFPairData.fBvel[0] = fvPair.fAvel[0];
        tmpVFPairData.fBvel[1] = fvPair.fAvel[1];
        tmpVFPairData.fBvel[2] = fvPair.fAvel[2];
        tmpVFPairData.i = fvPair.j;
        tmpVFPairData.vApos = fvPair.vBpos;
        tmpVFPairData.vAvel = fvPair.vBvel;

        FmVertexFaceCcd(&tmpContact, tmpVFPairData, deltaTime, conditions);

        contact->direction = -tmpContact.direction;
        contact->posi = tmpContact.posj;
        contact->posj = tmpContact.posi;
        contact->distance = tmpContact.distance;
        contact->time = tmpContact.time;
        contact->numIterations = tmpContact.numIterations;
        contact->featurePair = FmReverseFeaturePair(tmpContact.featurePair);
        contact->retVal = tmpContact.retVal;
    }

}
