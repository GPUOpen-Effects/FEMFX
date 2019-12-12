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
// Structure-of-arrays SIMD implementation of continuous collision detection for points,
// line segments, and triangles whose vertices move linearly within the timestep
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXTypes.h"
#include "FEMFXTriCcd.h"
#include "FEMFXSoaFeaturePairDistance.h"

namespace AMD
{

    template<class T>
    struct FEMSoaFXTri
    {
        typename T::SoaVector3 pos0;
        typename T::SoaVector3 pos1;
        typename T::SoaVector3 pos2;
        typename T::SoaVector3 vel0;
        typename T::SoaVector3 vel1;
        typename T::SoaVector3 vel2;
        typename T::SoaUint id;

        FEMSoaFXTri() : id(0x3f) {}

        const typename T::SoaVector3& Pos(uint i) const { return *(&pos0 + i); }
        const typename T::SoaVector3& Vel(uint i) const { return *(&vel0 + i); }
        const typename T::SoaVector3 Pos(uint i, typename T::SoaFloat deltaTime) const { return Pos(i) + Vel(i)*deltaTime; }

        inline typename T::SoaUint GetTriId() const
        {
            return id >> 6;
        }
        inline bool OwnsVert(typename T::SoaUint i) const
        {
            return (id & (1 << i)) != 0;
        }
        inline bool OwnsEdge(typename T::SoaUint i) const
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

    template<class T>
    struct FmSoaCcdResult
    {
        typename T::SoaVector3 direction;    // direction pointing from i to j
        typename T::SoaVector3 posi;
        typename T::SoaVector3 posj;
        typename T::SoaFloat   distance;
        typename T::SoaFloat   time;
        typename T::SoaUint    numIterations;
        FmSoaFeaturePair<T> featurePair;
        typename T::SoaUint    retVal;
    };

    template<class T>
    struct FmSoaCcdTerminationConditions
    {
        typename T::SoaFloat impactGap;          // Distance between tris at time of impact.  impactGap may be the sum of triangle thicknesses.
        typename T::SoaFloat contactGap;         // Distance defining contact condition at beginning of timestep.  Assumed to be greater than impactGap.
        typename T::SoaFloat impactGapAbsError;  // Search terminates if distance within this absolute error of impact gap.
        typename T::SoaFloat impactGapRelError;  // Search terminates if distance within this relative error of impact gap.
        typename T::SoaFloat minTimeStep;        // Search terminates if time step of next iteration is less than this minimum.
        typename T::SoaUint  maxIterations;      // Search terminates if maximum iterations reached.

        FmSoaCcdTerminationConditions() : impactGap(0.0f), contactGap(0.0f), impactGapAbsError(FLT_EPSILON), impactGapRelError(FLT_EPSILON), minTimeStep(FLT_EPSILON), maxIterations(FM_DEFAULT_MAX_CCD_ITERATIONS) {}
    };

    template<class T>
    struct FmSoaEEPairData
    {
        typename T::SoaVector3 eApos[2];
        typename T::SoaVector3 eAvel[2];
        typename T::SoaVector3 eBpos[2];
        typename T::SoaVector3 eBvel[2];
        typename T::SoaUint    i;           // triangle corner indices of edge start verts
        typename T::SoaUint    j;
    };

    template<class T>
    struct FmSoaVFPairData
    {
        typename T::SoaVector3 vApos;
        typename T::SoaVector3 vAvel;
        typename T::SoaVector3 fBpos[3];
        typename T::SoaVector3 fBvel[3];
        typename T::SoaUint    i;            // triangle corner index of vA
    };

    template<class T>
    struct FmSoaFVPairData
    {
        typename T::SoaVector3 fApos[3];
        typename T::SoaVector3 fAvel[3];
        typename T::SoaVector3 vBpos;
        typename T::SoaVector3 vBvel;
        typename T::SoaUint    j;            // triangle corner index of vB
    };

    template<class T>
    static FM_FORCE_INLINE void FmGetSlice(
        FmCcdResult* result, 
        const FmSoaCcdResult<T>& soaResult, 
        uint idx)
    {
        result->direction = FmGetSlice<T>(soaResult.direction, idx);
        result->posi = FmGetSlice<T>(soaResult.posi, idx);
        result->posj = FmGetSlice<T>(soaResult.posj, idx);
        result->distance = soaResult.distance.getSlice(idx);
        result->time = soaResult.time.getSlice(idx);
        result->numIterations = soaResult.numIterations.getSlice(idx);
        result->featurePair.itype = (FM_FEATURE_TYPE)soaResult.featurePair.itype.getSlice(idx);
        result->featurePair.i0 = soaResult.featurePair.i0.getSlice(idx);
        result->featurePair.i1 = soaResult.featurePair.i1.getSlice(idx);
        result->featurePair.i2 = soaResult.featurePair.i2.getSlice(idx);
        result->featurePair.jtype = (FM_FEATURE_TYPE)soaResult.featurePair.jtype.getSlice(idx);
        result->featurePair.j0 = soaResult.featurePair.j0.getSlice(idx);
        result->featurePair.j1 = soaResult.featurePair.j1.getSlice(idx);
        result->featurePair.j2 = soaResult.featurePair.j2.getSlice(idx);
        result->retVal = (FmCcdReturn)soaResult.retVal.getSlice(idx);
    }

    template<class T>
    static FM_FORCE_INLINE void FmSetSlice(FmSoaVFPairData<T>* soaPairData, uint idx, const FmVertexFacePairData& pairData)
    {
        FmSetSlice<T>(&soaPairData->vApos, idx, pairData.vApos);
        FmSetSlice<T>(&soaPairData->vAvel, idx, pairData.vAvel);
        FmSetSlice<T>(&soaPairData->fBpos[0], idx, pairData.fBpos[0]);
        FmSetSlice<T>(&soaPairData->fBvel[0], idx, pairData.fBvel[0]);
        FmSetSlice<T>(&soaPairData->fBpos[1], idx, pairData.fBpos[1]);
        FmSetSlice<T>(&soaPairData->fBvel[1], idx, pairData.fBvel[1]);
        FmSetSlice<T>(&soaPairData->fBpos[2], idx, pairData.fBpos[2]);
        FmSetSlice<T>(&soaPairData->fBvel[2], idx, pairData.fBvel[2]);
        soaPairData->i.setSlice(idx, pairData.i);
    }

    template<class T>
    static FM_FORCE_INLINE void FmSetSlice(FmSoaFVPairData<T>* soaPairData, uint idx, const FmFaceVertexPairData& pairData)
    {
        FmSetSlice<T>(&soaPairData->vBpos, idx, pairData.vBpos);
        FmSetSlice<T>(&soaPairData->vBvel, idx, pairData.vBvel);
        FmSetSlice<T>(&soaPairData->fApos[0], idx, pairData.fApos[0]);
        FmSetSlice<T>(&soaPairData->fAvel[0], idx, pairData.fAvel[0]);
        FmSetSlice<T>(&soaPairData->fApos[1], idx, pairData.fApos[1]);
        FmSetSlice<T>(&soaPairData->fAvel[1], idx, pairData.fAvel[1]);
        FmSetSlice<T>(&soaPairData->fApos[2], idx, pairData.fApos[2]);
        FmSetSlice<T>(&soaPairData->fAvel[2], idx, pairData.fAvel[2]);
        soaPairData->j.setSlice(idx, pairData.j);
    }

    template<class T>
    static FM_FORCE_INLINE void FmSetSlice(FmSoaEEPairData<T>* soaPairData, uint idx, const FmEdgeEdgePairData& pairData)
    {
        FmSetSlice<T>(&soaPairData->eApos[0], idx, pairData.eApos[0]);
        FmSetSlice<T>(&soaPairData->eAvel[0], idx, pairData.eAvel[0]);
        FmSetSlice<T>(&soaPairData->eApos[1], idx, pairData.eApos[1]);
        FmSetSlice<T>(&soaPairData->eAvel[1], idx, pairData.eAvel[1]);
        FmSetSlice<T>(&soaPairData->eBpos[0], idx, pairData.eBpos[0]);
        FmSetSlice<T>(&soaPairData->eBvel[0], idx, pairData.eBvel[0]);
        FmSetSlice<T>(&soaPairData->eBpos[1], idx, pairData.eBpos[1]);
        FmSetSlice<T>(&soaPairData->eBvel[1], idx, pairData.eBvel[1]);
        soaPairData->i.setSlice(idx, pairData.i);
        soaPairData->j.setSlice(idx, pairData.j);
    }

    template<class T>
    static FM_FORCE_INLINE FmSoaFeaturePair<T> FmReverseFeaturePair(const FmSoaFeaturePair<T>& featurePair)
    {
        FmSoaFeaturePair<T> result;
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

    // Earliest time that vertex and triangle may reach gap distance, assuming vertices have constant velocities.
    template<class T>
    static inline typename T::SoaFloat FmVertexTriangleEarliestImpact(const typename T::SoaVector3& normal, const FmSoaVFPairData<T>& vfPair, typename T::SoaFloat gap)
    {
        typedef typename T::SoaFloat SoaFloat;

        SoaFloat proj0 = dot(normal, vfPair.fBpos[0] - vfPair.vApos);
        SoaFloat proj1 = dot(normal, vfPair.fBpos[1] - vfPair.vApos);
        SoaFloat proj2 = dot(normal, vfPair.fBpos[2] - vfPair.vApos);

        // Relative velocities between pairs of points projected on normal
        SoaFloat vel0 = dot(normal, vfPair.vAvel);
        SoaFloat vB0Proj = dot(normal, vfPair.fBvel[0]);
        SoaFloat vB1Proj = dot(normal, vfPair.fBvel[1]);
        SoaFloat vB2Proj = dot(normal, vfPair.fBvel[2]);

        SoaFloat dVel0 = vel0 - vB0Proj;
        SoaFloat dVel1 = vel0 - vB1Proj;
        SoaFloat dVel2 = vel0 - vB2Proj;

        // Make any negative velocities 0, causing inf below
        dVel0 = max(dVel0, SoaFloat(0.0f));
        dVel1 = max(dVel1, SoaFloat(0.0f));
        dVel2 = max(dVel2, SoaFloat(0.0f));

        // Projected distance between pairs of points
        SoaFloat dist0 = proj0 - gap;
        SoaFloat dist1 = proj1 - gap;
        SoaFloat dist2 = proj2 - gap;

        // Compute time to cross projected distances, and take min
        SoaFloat dt0 = select(dist0 / dVel0, SoaFloat(0.0f), dist0 <= SoaFloat(0.0f));
        SoaFloat dt1 = select(dist1 / dVel1, SoaFloat(0.0f), dist1 <= SoaFloat(0.0f));
        SoaFloat dt2 = select(dist2 / dVel2, SoaFloat(0.0f), dist2 <= SoaFloat(0.0f));

        SoaFloat dt = min(dt0, min(dt1, dt2));

        return dt;
    }

    // Earliest time that edge pair may reach gap distance, assuming vertices have constant velocities.
    template<class T>
    static inline typename T::SoaFloat FmEdgePairEarliestImpact(const typename T::SoaVector3& normal, const FmSoaEEPairData<T>& eePair, typename T::SoaFloat gap)
    {
        typedef typename T::SoaFloat SoaFloat;

        SoaFloat proj00 = dot(normal, eePair.eBpos[0] - eePair.eApos[0]);
        SoaFloat proj01 = dot(normal, eePair.eBpos[1] - eePair.eApos[0]);
        SoaFloat proj10 = dot(normal, eePair.eBpos[0] - eePair.eApos[1]);
        SoaFloat proj11 = dot(normal, eePair.eBpos[1] - eePair.eApos[1]);

        // Relative velocities between pairs of points projected on normal
        SoaFloat vA0Proj = dot(normal, eePair.eAvel[0]);
        SoaFloat vA1Proj = dot(normal, eePair.eAvel[1]);
        SoaFloat vB0Proj = dot(normal, eePair.eBvel[0]);
        SoaFloat vB1Proj = dot(normal, eePair.eBvel[1]);

        SoaFloat dVel00 = vA0Proj - vB0Proj;
        SoaFloat dVel01 = vA0Proj - vB1Proj;
        SoaFloat dVel10 = vA1Proj - vB0Proj;
        SoaFloat dVel11 = vA1Proj - vB1Proj;

        // Make any negative velocities 0, causing inf below
        dVel00 = max(dVel00, SoaFloat(0.0f));
        dVel01 = max(dVel01, SoaFloat(0.0f));
        dVel10 = max(dVel10, SoaFloat(0.0f));
        dVel11 = max(dVel11, SoaFloat(0.0f));

        // Projected distance between pairs of points
        SoaFloat dist00 = proj00 - gap;
        SoaFloat dist01 = proj01 - gap;
        SoaFloat dist10 = proj10 - gap;
        SoaFloat dist11 = proj11 - gap;

        // Compute time to cross projected distances, and take min
        SoaFloat dt00 = select(dist00 / dVel00, SoaFloat(0.0f), dist00 <= SoaFloat(0.0f));
        SoaFloat dt01 = select(dist01 / dVel01, SoaFloat(0.0f), dist01 <= SoaFloat(0.0f));
        SoaFloat dt10 = select(dist10 / dVel10, SoaFloat(0.0f), dist10 <= SoaFloat(0.0f));
        SoaFloat dt11 = select(dist11 / dVel11, SoaFloat(0.0f), dist11 <= SoaFloat(0.0f));

        SoaFloat dt = min(dt00, min(dt01, min(dt10, dt11)));

        return dt;
    }

    template<class T>
    void FmEdgeEdgeCcd(
        FmSoaCcdResult<T>* contact,
        const FmSoaEEPairData<T>& eePair,
        typename T::SoaFloat deltaTime,
        const FmSoaCcdTerminationConditions<T>& conditions)
    {
        typedef typename T::SoaFloat SoaFloat;
        typedef typename T::SoaUint SoaUint;
        typedef typename T::SoaBool SoaBool;

        SoaFloat impactGapRelScale = conditions.impactGapRelError + SoaFloat(1.0f);

        // Compute distance of edge pair
        FmSoaDistanceResult<T> distResult;

        SoaUint i = eePair.i;
        SoaUint j = eePair.j;
        SoaUint i1 = i + SoaUint(1);
        i1 = select(i1, SoaUint(0), i1 >= SoaUint(3));
        SoaUint j1 = j + SoaUint(1);
        j1 = select(j1, SoaUint(0), j1 >= SoaUint(3));

        FmSegmentPairDistance(&distResult, eePair.eApos[0], eePair.eApos[1], eePair.eBpos[0], eePair.eBpos[1], i, i1, j, j1);

        // Initialize contact with closest points
        contact->direction = distResult.direction;
        contact->posi = distResult.posi;
        contact->posj = distResult.posj;
        contact->distance = distResult.distance;
        contact->time = 0.0f;
        contact->numIterations = 0;
        contact->featurePair = distResult.featurePair;
        contact->retVal = FM_CCD_RET_INITIALLY_CONTACTING;

        // Return if distance is less than contact gap
        SoaBool initiallyContacting = (distResult.distance <= conditions.contactGap);
        SoaBool finished = initiallyContacting;

        if (all(finished))
        {
            return;
        }

        // Initialize time of impact bound
        SoaFloat dt = FmEdgePairEarliestImpact(distResult.direction, eePair, conditions.impactGap);

        SoaBool noImpact = (dt > deltaTime);
        SoaBool minTimeStep = (dt <= conditions.minTimeStep);

        contact->time = select(SoaFloat(0.0f), deltaTime, (!finished) & noImpact);
        contact->retVal = select(contact->retVal, SoaUint(FM_CCD_RET_NO_IMPACT), (!finished) & noImpact);

        contact->retVal = select(contact->retVal, SoaUint(FM_CCD_RET_MIN_TIME_STEP), (!finished) & minTimeStep);

        finished = finished | noImpact;
        finished = finished | minTimeStep;

        if (all(finished))
        {
            return;
        }

        SoaFloat t = dt;

        // Iteratively advance time, compute distance and next impact time bound, until it reaches impact distance or time bound is later than deltaTime
        SoaUint iter = 0;

        do
        {
            iter++;
            contact->numIterations = select(iter, contact->numIterations, finished);

            FmSoaEEPairData<T> eePairAdv;
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

            contact->direction = select(distResult.direction, contact->direction, finished);
            contact->posi = select(distResult.posi, contact->posi, finished);
            contact->posj = select(distResult.posj, contact->posj, finished);
            contact->distance = select(distResult.distance, contact->distance, finished);
            contact->time = select(t, contact->time, finished);
            contact->featurePair.i0 = select(distResult.featurePair.i0, contact->featurePair.i0, finished);
            contact->featurePair.i1 = select(distResult.featurePair.i1, contact->featurePair.i1, finished);
            contact->featurePair.i2 = select(distResult.featurePair.i2, contact->featurePair.i2, finished);
            contact->featurePair.j0 = select(distResult.featurePair.j0, contact->featurePair.j0, finished);
            contact->featurePair.j1 = select(distResult.featurePair.j1, contact->featurePair.j1, finished);
            contact->featurePair.j2 = select(distResult.featurePair.j2, contact->featurePair.j2, finished);
            contact->featurePair.itype = select(distResult.featurePair.itype, contact->featurePair.itype, finished);
            contact->featurePair.jtype = select(distResult.featurePair.jtype, contact->featurePair.jtype, finished);
            contact->featurePair.t = select(distResult.featurePair.t, contact->featurePair.t, finished);

            SoaBool impacting = (distResult.distance <= max(conditions.impactGap + conditions.impactGapAbsError, conditions.impactGap * impactGapRelScale));

            contact->retVal = select(contact->retVal, SoaUint(FM_CCD_RET_IMPACT), (!finished) & impacting);

            finished = finished | impacting;

            if (all(finished))
            {
                return;
            }

            dt = FmEdgePairEarliestImpact(distResult.direction, eePairAdv, conditions.impactGap);

            SoaFloat tnext = t + dt;

            noImpact = (tnext > deltaTime);

            contact->time = select(contact->time, deltaTime, (!finished) & noImpact);
            contact->retVal = select(contact->retVal, SoaUint(FM_CCD_RET_NO_IMPACT), (!finished) & noImpact);
            finished = finished | noImpact;

            minTimeStep = (tnext - t <= conditions.minTimeStep);

            contact->retVal = select(contact->retVal, SoaUint(FM_CCD_RET_MIN_TIME_STEP), (!finished) & minTimeStep);
            finished = finished | minTimeStep;

            t = tnext;

            SoaBool maxIterations = (iter >= conditions.maxIterations);

            contact->retVal = select(contact->retVal, SoaUint(FM_CCD_RET_MAX_ITERATIONS), (!finished) & maxIterations);

            finished = finished | maxIterations;

        } while (!all(finished));
    }

    template<class T>
    void FmVertexFaceCcd(
        FmSoaCcdResult<T>* contact,
        const FmSoaVFPairData<T>& vfPair,
        typename T::SoaFloat deltaTime,
        const FmSoaCcdTerminationConditions<T>& conditions)
    {
        typedef typename T::SoaFloat SoaFloat;
        typedef typename T::SoaUint SoaUint;
        typedef typename T::SoaBool SoaBool;

        SoaFloat impactGapRelScale = conditions.impactGapRelError + SoaFloat(1.0f);

        // Compute distance of vertex/triangle pair
        FmSoaDistanceResult<T> distResult;

        SoaUint i = vfPair.i;

        FmPointTriangleDistance(&distResult, vfPair.vApos, vfPair.fBpos[0], vfPair.fBpos[1], vfPair.fBpos[2], i, 0, 1, 2);

        // Initialize contact with closest points
        contact->direction = distResult.direction;
        contact->posi = vfPair.vApos;
        contact->posj = distResult.posj;
        contact->distance = distResult.distance;
        contact->time = 0.0f;
        contact->numIterations = 0;
        contact->featurePair = distResult.featurePair;
        contact->retVal = FM_CCD_RET_INITIALLY_CONTACTING;

        // Return if distance is less than contact gap
        SoaBool initiallyContacting = (distResult.distance <= conditions.contactGap);
        SoaBool finished = initiallyContacting;

        if (all(finished))
        {
            return;
        }

        // Initialize time of impact bound
        SoaFloat dt = FmVertexTriangleEarliestImpact(distResult.direction, vfPair, conditions.impactGap);

        SoaBool noImpact = (dt > deltaTime);
        SoaBool minTimeStep = (dt <= conditions.minTimeStep);

        contact->time = select(SoaFloat(0.0f), deltaTime, (!finished) & noImpact);
        contact->retVal = select(contact->retVal, SoaUint(FM_CCD_RET_NO_IMPACT), (!finished) & noImpact);

        contact->retVal = select(contact->retVal, SoaUint(FM_CCD_RET_MIN_TIME_STEP), (!finished) & minTimeStep);

        finished = finished | noImpact;
        finished = finished | minTimeStep;

        if (all(finished))
        {
            return;
        }

        SoaFloat t = dt;

        // Iteratively advance time, compute distance and next impact time bound, until it reaches impact distance or time bound is later than deltaTime
        SoaUint iter = 0;     

        do
        {
            iter++;
            contact->numIterations = select(iter, contact->numIterations, finished);

            FmSoaVFPairData<T> vfPairAdv;
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

            contact->direction = select(distResult.direction, contact->direction, finished);
            contact->posi = select(distResult.posi, contact->posi, finished);
            contact->posj = select(distResult.posj, contact->posj, finished);
            contact->distance = select(distResult.distance, contact->distance, finished);
            contact->time = select(t, contact->time, finished);
            contact->featurePair.i0 = select(distResult.featurePair.i0, contact->featurePair.i0, finished);
            contact->featurePair.i1 = select(distResult.featurePair.i1, contact->featurePair.i1, finished);
            contact->featurePair.i2 = select(distResult.featurePair.i2, contact->featurePair.i2, finished);
            contact->featurePair.j0 = select(distResult.featurePair.j0, contact->featurePair.j0, finished);
            contact->featurePair.j1 = select(distResult.featurePair.j1, contact->featurePair.j1, finished);
            contact->featurePair.j2 = select(distResult.featurePair.j2, contact->featurePair.j2, finished);
            contact->featurePair.itype = select(distResult.featurePair.itype, contact->featurePair.itype, finished);
            contact->featurePair.jtype = select(distResult.featurePair.jtype, contact->featurePair.jtype, finished);
            contact->featurePair.t = select(distResult.featurePair.t, contact->featurePair.t, finished);

            SoaBool impacting = (distResult.distance <= max(conditions.impactGap + conditions.impactGapAbsError, conditions.impactGap * impactGapRelScale));

            contact->retVal = select(contact->retVal, SoaUint(FM_CCD_RET_IMPACT), (!finished) & impacting);
            finished = finished | impacting;

            if (all(finished))
            {
                return;
            }

            dt = FmVertexTriangleEarliestImpact(distResult.direction, vfPairAdv, conditions.impactGap);

            SoaFloat tnext = t + dt;

            noImpact = (tnext > deltaTime);

            contact->time = select(contact->time, deltaTime, (!finished) & noImpact);
            contact->retVal = select(contact->retVal, SoaUint(FM_CCD_RET_NO_IMPACT), (!finished) & noImpact);
            finished = finished | noImpact;

            minTimeStep = (tnext - t <= conditions.minTimeStep);

            contact->retVal = select(contact->retVal, SoaUint(FM_CCD_RET_MIN_TIME_STEP), (!finished) & minTimeStep);
            finished = finished | minTimeStep;

            t = tnext;

            SoaBool maxIterations = (iter >= conditions.maxIterations);

            contact->retVal = select(contact->retVal, SoaUint(FM_CCD_RET_MAX_ITERATIONS), (!finished) & maxIterations);

            finished = finished | maxIterations;

        } while (!all(finished));
    }

    template<class T>
    void FmFaceVertexCcd(
        FmSoaCcdResult<T>* contact,
        const FmSoaFVPairData<T>& fvPair,
        typename T::SoaFloat deltaTime,
        const FmSoaCcdTerminationConditions<T>& conditions)
    {
        FmSoaCcdResult<T> tmpContact;
        FmSoaVFPairData<T> tmpVFPairData;
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
