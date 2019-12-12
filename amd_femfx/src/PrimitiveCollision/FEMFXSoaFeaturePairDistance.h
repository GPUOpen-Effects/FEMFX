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
// Structure-of-arrays SIMD implementation of point, line segment, and triangle distance 
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXTypes.h"

namespace AMD
{
    template<class T>
    struct FmSoaFeaturePair
    {
        typename T::SoaUint  itype;
        typename T::SoaUint  jtype;
        typename T::SoaUint  i0, i1, i2;
        typename T::SoaUint  j0, j1, j2;
        typename T::SoaFloat t, u;

        FmSoaFeaturePair() : itype(0), jtype(0), i0(0), i1(0), i2(0), j0(0), j1(0), j2(0), t(0.0f), u(0.0f) {}
    };

    template<class T>
    struct FmSoaDistanceResult
    {
        typename T::SoaVector3 direction;  // direction pointing from i to j
        typename T::SoaVector3 posi;
        typename T::SoaVector3 posj;
        typename T::SoaFloat   distance;
        FmSoaFeaturePair<T>    featurePair;
    };

    template< class T >
    static inline typename T::SoaVector3 normalize(typename T::SoaFloat* lenSqr, const typename T::SoaVector3& a)
    {
        typename T::SoaFloat lSqr = dot(a, a);
        *lenSqr = lSqr;
        return a * 1.0f / sqrtf(lSqr);
    }

    // Distance and nearest points of line segment pair.
    // Reference: Lumelsky, "On Fast Computation of Distance between Line Segments"
    template< class T >
    static inline void FmSegmentPairDistance(
        FmSoaDistanceResult<T>* result,
        const typename T::SoaVector3& pA0, const typename T::SoaVector3& pA1, const typename T::SoaVector3& pB0, const typename T::SoaVector3& pB1,
        typename T::SoaUint i0, typename T::SoaUint i1, typename T::SoaUint j0, typename T::SoaUint j1)
    {
        typedef typename T::SoaVector3 SoaVector3;
        typedef typename T::SoaFloat SoaFloat;
        typedef typename T::SoaUint SoaUint;

        SoaVector3 vecAB = pB0 - pA0;
        SoaVector3 vecA = pA1 - pA0;
        SoaVector3 vecB = pB1 - pB0;
        SoaFloat vecAdotAB = dot(vecA, vecAB);
        SoaFloat vecBdotAB = dot(vecB, vecAB);
        SoaFloat vecAdotA = dot(vecA, vecA);
        SoaFloat vecBdotB = dot(vecB, vecB);
        SoaFloat vecAdotB = dot(vecA, vecB);

        // Line A parameterized by t: pA0 + vecA * t
        // Line B parameterized by u: pB0 + vecB * u

        // Compute t value of point on segment A nearest to line B
        SoaFloat tden = vecAdotA*vecBdotB - vecAdotB*vecAdotB;

        SoaFloat t = select((vecAdotAB*vecBdotB - vecBdotAB*vecAdotB) / tden, SoaFloat(1.0f), tden == SoaFloat(0.0f));
        t = select(t, SoaFloat(0.0f), t < SoaFloat(0.0f));
        t = select(t, SoaFloat(1.0f), t > SoaFloat(1.0f));

        // Compute u of corresponding point on line B 
        SoaFloat u = select((t*vecAdotB - vecBdotAB) / vecBdotB, SoaFloat(1.0f), vecBdotB == SoaFloat(0.0f));

        const SoaFloat lenSqrTol = FM_NORMALIZE_MAG_SQR_TOL;

        // Clamp u, recompute t if clamped, and compute closest points and normal
        SoaVector3 normal, pAClosest, pBClosest;
        FmSoaFeaturePair<T> featurePair;

        SoaFloat t2 = select(vecAdotAB / vecAdotA, SoaFloat(1.0f), vecAdotA == SoaFloat(0.0f));
        SoaFloat t3 = select((vecAdotB + vecAdotAB) / vecAdotA, SoaFloat(1.0f), vecAdotA == SoaFloat(0.0f));

        t = select(t, t2, u <= SoaFloat(0.0f));
        t = select(t, t3, u >= SoaFloat(1.0f));

        pAClosest = select(pA0 + vecA * t, pA0, t <= SoaFloat(0.0f));
        pAClosest = select(pAClosest, pA1, t >= SoaFloat(1.0f));

        pBClosest = select(pB0 + vecB * u, pB0, u <= SoaFloat(0.0f));
        pBClosest = select(pBClosest, pB1, u >= SoaFloat(1.0f));

        u = select(u, SoaFloat(0.0f), u < SoaFloat(0.0f));
        u = select(u, SoaFloat(1.0f), u > SoaFloat(1.0f));
        t = select(t, SoaFloat(0.0f), t < SoaFloat(0.0f));
        t = select(t, SoaFloat(1.0f), t > SoaFloat(1.0f));

        featurePair.itype = select(SoaUint(FM_FEATURE_TYPE_EDGE), SoaUint(FM_FEATURE_TYPE_VERTEX), (t == SoaFloat(0.0f)) | (t == SoaFloat(1.0f)));
        featurePair.jtype = select(SoaUint(FM_FEATURE_TYPE_EDGE), SoaUint(FM_FEATURE_TYPE_VERTEX), (u == SoaFloat(0.0f)) | (u == SoaFloat(1.0f)));

        featurePair.i0 = select(i0, i1, t == SoaFloat(1.0f));
        featurePair.i1 = i1;

        featurePair.j0 = select(j0, j1, u == SoaFloat(1.0f));
        featurePair.j1 = j1;

        featurePair.t = t;
        featurePair.u = u;

        SoaVector3 vecABClosest = pBClosest - pAClosest;
        SoaFloat lengthAB = length(vecABClosest);

        SoaFloat distance = lengthAB;
        SoaFloat lenSqr;
        normal = normalize<T>(&lenSqr, vecABClosest);

        normal = select(normal, SoaVector3(0.0f), lenSqr < lenSqrTol);
        distance = select(distance, SoaFloat(0.0f), lenSqr < lenSqrTol);

        result->direction = normal;
        result->distance = distance;
        result->posi = pAClosest;
        result->posj = pBClosest;
        result->featurePair = featurePair;
    }

    // Distance and nearest points of point and line segment.
    // Reference: Lumelsky, "On Fast Computation of Distance between Line Segments"
    template<class T>
    static inline void FmPointSegmentDistance(
        FmSoaDistanceResult<T>* result,
        const typename T::SoaVector3& pA, const typename T::SoaVector3& pB0, const typename T::SoaVector3& pB1,
        typename T::SoaUint i, typename T::SoaUint j0, typename T::SoaUint j1)
    {
        typedef typename T::SoaVector3 SoaVector3;
        typedef typename T::SoaFloat SoaFloat;
        typedef typename T::SoaBool SoaBool;
        typedef typename T::SoaUint SoaUint;

        SoaVector3 vecB = pB1 - pB0;
        SoaVector3 vecAB = pB0 - pA;
        SoaFloat vecBdotB = dot(vecB, vecB);
        SoaFloat vecBdotAB = dot(vecB, vecAB);

        // Line B parameterized by u: pB0 + vecB * u

        // Find u on line B closest to pA
        SoaFloat u = select(-vecBdotAB / vecBdotB, SoaFloat(1.0f), vecBdotB == SoaFloat(0.0f));

        const SoaFloat lenSqrTol = FM_NORMALIZE_MAG_SQR_TOL;

        // Clamp u and compute point and normal
        SoaVector3 normal;
        SoaVector3 pBClosest;
        FmSoaFeaturePair<T> featurePair;

        pBClosest = select(pB0 + vecB * u, pB0, u <= SoaFloat(0.0f));
        pBClosest = select(pBClosest, pB1, u >= SoaFloat(1.0f));

        u = select(u, SoaFloat(0.0f), u < SoaFloat(0.0f));
        u = select(u, SoaFloat(1.0f), u > SoaFloat(1.0f));

        featurePair.jtype = select(SoaUint(FM_FEATURE_TYPE_EDGE), SoaUint(FM_FEATURE_TYPE_VERTEX), (u == SoaFloat(0.0f)) | (u == SoaFloat(1.0f)));
        featurePair.j0 = select(j0, j1, u == SoaFloat(1.0f));
        featurePair.j1 = j1;

        featurePair.itype = FM_FEATURE_TYPE_VERTEX;
        featurePair.i0 = i;
        featurePair.u = u;

        SoaVector3 vecABClosest = pBClosest - pA;
        SoaFloat lengthAB = length(vecABClosest);

        SoaFloat distance = lengthAB;
        SoaFloat lenSqr;
        normal = normalize<T>(&lenSqr, vecABClosest);

        normal = select(normal, SoaVector3(0.0f), lenSqr < lenSqrTol);
        distance = select(distance, SoaFloat(0.0f), lenSqr < lenSqrTol);

        result->direction = normal;
        result->distance = distance;
        result->posi = pA;
        result->posj = pBClosest;
        result->featurePair = featurePair;
    }

    // Distance and nearest points of point and triangle.
    template<class T>
    static inline void FmPointTriangleDistance(
        FmSoaDistanceResult<T>* result,
        const typename T::SoaVector3& pA, const typename T::SoaVector3& pB0, const typename T::SoaVector3& pB1, const typename T::SoaVector3& pB2,
        typename T::SoaUint i, typename T::SoaUint j0, typename T::SoaUint j1, typename T::SoaUint j2)
    {
        typedef typename T::SoaVector3 SoaVector3;
        typedef typename T::SoaFloat SoaFloat;
        typedef typename T::SoaBool SoaBool;

        SoaVector3 vecEdgeB0 = pB1 - pB0;
        SoaVector3 vecEdgeB1 = pB2 - pB1;
        SoaVector3 vecEdgeB2 = pB0 - pB2;

        SoaVector3 vecB0ToA = pA - pB0;
        SoaVector3 vecB1ToA = pA - pB1;
        SoaVector3 vecB2ToA = pA - pB2;

        SoaVector3 triCross = cross(vecEdgeB0, vecEdgeB1);

        SoaFloat vertProj0 = dot(vecB0ToA, cross(vecEdgeB0, triCross));
        SoaFloat vertProj1 = dot(vecB1ToA, cross(vecEdgeB1, triCross));
        SoaFloat vertProj2 = dot(vecB2ToA, cross(vecEdgeB2, triCross));

        SoaFloat lenSqr;
        SoaVector3 triNormal = normalize<T>(&lenSqr, triCross);

        SoaBool triIsDegenerate = lenSqr < SoaFloat(FM_NORMALIZE_MAG_SQR_TOL);

        FmSoaDistanceResult<T> pntResult;
        SoaBool replaceResult;

        SoaFloat distance = FLT_MAX;
        SoaVector3 direction, closestPos;

        FmSoaFeaturePair<T> featurePair;

        FmPointSegmentDistance(&pntResult, pA, pB0, pB1, i, j0, j1);

        // vertProj0 > 0.0f || triIsDegenerate
        distance = pntResult.distance;
        direction = pntResult.direction;
        closestPos = pntResult.posj;

        featurePair.jtype = pntResult.featurePair.jtype;
        featurePair.j0 = pntResult.featurePair.j0;
        featurePair.j1 = pntResult.featurePair.j1;
        featurePair.u = pntResult.featurePair.u;

        // vertProj1 > 0.0f || triIsDegenerate
        FmPointSegmentDistance(&pntResult, pA, pB1, pB2, i, j1, j2);
        
        replaceResult = ((vertProj1 > SoaFloat(0.0f)) | triIsDegenerate) & (pntResult.distance < distance);

        distance = select(distance, pntResult.distance, replaceResult);
        direction = select(direction, pntResult.direction, replaceResult);
        closestPos = select(closestPos, pntResult.posj, replaceResult);
        featurePair.jtype = select(featurePair.jtype, pntResult.featurePair.jtype, replaceResult);
        featurePair.j0 = select(featurePair.j0, pntResult.featurePair.j0, replaceResult);
        featurePair.j1 = select(featurePair.j1, pntResult.featurePair.j1, replaceResult);
        featurePair.u = select(featurePair.u, pntResult.featurePair.u, replaceResult);

        // vertProj2 > 0.0f || triIsDegenerate
        FmPointSegmentDistance(&pntResult, pA, pB2, pB0, i, j2, j0);

        replaceResult = ((vertProj2 > SoaFloat(0.0f)) | triIsDegenerate) & (pntResult.distance < distance);

        distance = select(distance, pntResult.distance, replaceResult);
        direction = select(direction, pntResult.direction, replaceResult);
        closestPos = select(closestPos, pntResult.posj, replaceResult);
        featurePair.jtype = select(featurePair.jtype, pntResult.featurePair.jtype, replaceResult);
        featurePair.j0 = select(featurePair.j0, pntResult.featurePair.j0, replaceResult);
        featurePair.j1 = select(featurePair.j1, pntResult.featurePair.j1, replaceResult);
        featurePair.u = select(featurePair.u, pntResult.featurePair.u, replaceResult);

        // vertProj0 <= 0.0f && vertProj1 <= 0.0f && vertProj2 <= 0.0f && !triIsDegenerate
        SoaFloat proj = dot(triNormal, vecB0ToA);
        pntResult.direction = select(triNormal, -triNormal, proj > SoaFloat(0.0f));
        pntResult.posj = pA - triNormal * proj;
        pntResult.distance = fabsf(proj);
        pntResult.featurePair.jtype = FM_FEATURE_TYPE_FACE;
        pntResult.featurePair.j0 = j0;
        pntResult.featurePair.j1 = j1;
        pntResult.featurePair.j2 = j2;
        pntResult.featurePair.u = 0.0f;

        replaceResult = 
            (vertProj0 <= SoaFloat(0.0f)) & 
            (vertProj1 <= SoaFloat(0.0f)) & 
            (vertProj2 <= SoaFloat(0.0f)) & (!triIsDegenerate);

        distance = select(distance, pntResult.distance, replaceResult);
        direction = select(direction, pntResult.direction, replaceResult);
        closestPos = select(closestPos, pntResult.posj, replaceResult);
        featurePair.jtype = select(featurePair.jtype, pntResult.featurePair.jtype, replaceResult);
        featurePair.j0 = select(featurePair.j0, pntResult.featurePair.j0, replaceResult);
        featurePair.j1 = select(featurePair.j1, pntResult.featurePair.j1, replaceResult);
        featurePair.j2 = select(featurePair.j2, pntResult.featurePair.j2, replaceResult);
        featurePair.u = select(featurePair.u, pntResult.featurePair.u, replaceResult);

        featurePair.itype = FM_FEATURE_TYPE_VERTEX;
        featurePair.i0 = i;

        result->distance = distance;
        result->direction = direction;
        result->posi = pA;
        result->posj = closestPos;
        result->featurePair = featurePair;
    }

}
