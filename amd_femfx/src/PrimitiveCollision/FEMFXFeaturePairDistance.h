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
// API for finding closest distance of points, line segments, and triangles
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXTypes.h"

namespace AMD
{
    enum FM_FEATURE_TYPE
    {
        FM_FEATURE_TYPE_VERTEX,
        FM_FEATURE_TYPE_EDGE,
        FM_FEATURE_TYPE_FACE
    };

    struct FmFeaturePair
    {
        FM_FEATURE_TYPE itype;
        FM_FEATURE_TYPE jtype;
        uint i0, i1, i2;
        uint j0, j1, j2;
        float t, u;

        FmFeaturePair() : itype(FM_FEATURE_TYPE_VERTEX), jtype(FM_FEATURE_TYPE_VERTEX), i0(0), i1(0), i2(0), j0(0), j1(0), j2(0), t(0.0f), u(0.0f) {}
    };

    struct FmDistanceResult
    {
        FmVector3     direction;  // direction pointing from i to j
        FmVector3     posi;
        FmVector3     posj;
        float         distance;
        FmFeaturePair featurePair;
    };

    // Distance and nearest points of line segment pair.
    // Reference: Lumelsky, "On Fast Computation of Distance between Line Segments"
    static inline void FmSegmentPairDistance(
        FmDistanceResult* result,
        const FmVector3& pA0, const FmVector3& pA1, const FmVector3& pB0, const FmVector3& pB1,
        uint i0, uint i1, uint j0, uint j1)
    {
        FmVector3 vecAB = pB0 - pA0;
        FmVector3 vecA = pA1 - pA0;
        FmVector3 vecB = pB1 - pB0;
        float vecAdotAB = dot(vecA, vecAB);
        float vecBdotAB = dot(vecB, vecAB);
        float vecAdotA = dot(vecA, vecA);
        float vecBdotB = dot(vecB, vecB);
        float vecAdotB = dot(vecA, vecB);

        // Line A parameterized by t: pA0 + vecA * t
        // Line B parameterized by u: pB0 + vecB * u

        // Compute t value of point on segment A nearest to line B
        float tden = vecAdotA*vecBdotB - vecAdotB*vecAdotB;

        float t = (tden == 0.0f) ? 1.0f : (vecAdotAB*vecBdotB - vecBdotAB*vecAdotB) / tden;
        t = (t < 0.0f) ? 0.0f : t;
        t = (t > 1.0f) ? 1.0f : t;

        // Compute u of corresponding point on line B 
        float u = (vecBdotB == 0.0f) ? 1.0f : (t*vecAdotB - vecBdotAB) / vecBdotB;

        const float lenSqrTol = FM_NORMALIZE_MAG_SQR_TOL;

        // Clamp u, recompute t if clamped, and compute closest points and normal
        FmVector3 normal, pAClosest, pBClosest;
        FmFeaturePair featurePair;

        float t2 = (vecAdotA == 0.0f) ? 1.0f : vecAdotAB / vecAdotA;
        float t3 = (vecAdotA == 0.0f) ? 1.0f : (vecAdotB + vecAdotAB) / vecAdotA;

        t = (u <= 0.0f) ? t2 : t;
        t = (u >= 1.0f) ? t3 : t;

        pAClosest = (t <= 0.0f) ? pA0 : pA0 + vecA * t;
        pAClosest = (t >= 1.0f) ? pA1 : pAClosest;

        pBClosest = (u <= 0.0f) ? pB0 : pB0 + vecB * u;
        pBClosest = (u >= 1.0f) ? pB1 : pBClosest;

        u = (u < 0.0f) ? 0.0f : u;
        u = (u > 1.0f) ? 1.0f : u;
        t = (t < 0.0f) ? 0.0f : t;
        t = (t > 1.0f) ? 1.0f : t;

        featurePair.itype = (t == 0.0f || t == 1.0f) ? FM_FEATURE_TYPE_VERTEX : FM_FEATURE_TYPE_EDGE;
        featurePair.jtype = (u == 0.0f || u == 1.0f) ? FM_FEATURE_TYPE_VERTEX : FM_FEATURE_TYPE_EDGE;

        featurePair.i0 = (t == 1.0f) ? i1 : i0;
        featurePair.i1 = i1;

        featurePair.j0 = (u == 1.0f) ? j1 : j0;
        featurePair.j1 = j1;

        featurePair.t = t;
        featurePair.u = u;

        FmVector3 vecABClosest = pBClosest - pAClosest;
        float lengthAB = length(vecABClosest);

        float distance = lengthAB;
        float lenSqr;
        normal = FmNormalize(&lenSqr, vecABClosest);
        if (lenSqr < lenSqrTol)
        {
            normal = FmInitVector3(0.0f);
            distance = 0.0f;
        }

        result->direction = normal;
        result->distance = distance;
        result->posi = pAClosest;
        result->posj = pBClosest;
        result->featurePair = featurePair;
    }

    // Distance and nearest points of point and line segment.
    // Reference: Lumelsky, "On Fast Computation of Distance between Line Segments"
    static inline void FmPointSegmentDistance(
        FmDistanceResult* result,
        const FmVector3& pA, const FmVector3& pB0, const FmVector3& pB1,
        uint i, uint j0, uint j1)
    {
        FmVector3 vecB = pB1 - pB0;
        FmVector3 vecAB = pB0 - pA;
        float vecBdotB = dot(vecB, vecB);
        float vecBdotAB = dot(vecB, vecAB);

        // Line B parameterized by u: pB0 + vecB * u

        // Find u on line B closest to pA
        float u = (vecBdotB == 0.0f) ? 1.0f : -vecBdotAB / vecBdotB;

        const float lenSqrTol = FM_NORMALIZE_MAG_SQR_TOL;

        // Clamp u and compute point and normal
        FmVector3 normal;
        FmVector3 pBClosest;
        FmFeaturePair featurePair;

        pBClosest = (u <= 0.0f) ? pB0 : pB0 + vecB * u;
        pBClosest = (u >= 1.0f) ? pB1 : pBClosest;

        u = (u < 0.0f) ? 0.0f : u;
        u = (u > 1.0f) ? 1.0f : u;

        featurePair.jtype = (u == 0.0f || u == 1.0f) ? FM_FEATURE_TYPE_VERTEX : FM_FEATURE_TYPE_EDGE;
        featurePair.j0 = (u == 1.0f) ? j1 : j0;
        featurePair.j1 = j1;

        featurePair.itype = FM_FEATURE_TYPE_VERTEX;
        featurePair.i0 = i;
        featurePair.u = u;

        FmVector3 vecABClosest = pBClosest - pA;
        float lengthAB = length(vecABClosest);

        float distance = lengthAB;
        float lenSqr;
        normal = FmNormalize(&lenSqr, vecABClosest);
        if (lenSqr < lenSqrTol)
        {
            normal = FmInitVector3(0.0f);
            distance = 0.0f;
        }

        result->direction = normal;
        result->distance = distance;
        result->posi = pA;
        result->posj = pBClosest;
        result->featurePair = featurePair;
    }

    // Distance and nearest points of point and triangle.
    static inline void FmPointTriangleDistance(
        FmDistanceResult* result,
        const FmVector3& pA, const FmVector3& pB0, const FmVector3& pB1, const FmVector3& pB2,
        uint i, uint j0, uint j1, uint j2)
    {
        FmVector3 vecEdgeB0 = pB1 - pB0;
        FmVector3 vecEdgeB1 = pB2 - pB1;
        FmVector3 vecEdgeB2 = pB0 - pB2;

        FmVector3 vecB0ToA = pA - pB0;
        FmVector3 vecB1ToA = pA - pB1;
        FmVector3 vecB2ToA = pA - pB2;

        FmVector3 triCross = cross(vecEdgeB0, vecEdgeB1);

        float vertProj0 = dot(vecB0ToA, cross(vecEdgeB0, triCross));
        float vertProj1 = dot(vecB1ToA, cross(vecEdgeB1, triCross));
        float vertProj2 = dot(vecB2ToA, cross(vecEdgeB2, triCross));

        float lenSqr;
        FmVector3 triNormal = FmNormalize(&lenSqr, triCross);

        bool triIsDegenerate = lenSqr < FM_NORMALIZE_MAG_SQR_TOL;

        FmDistanceResult pntResult;

        float distance = FLT_MAX;
        FmVector3 direction, closestPos;

        FmFeaturePair featurePair;
        if (vertProj0 > 0.0f || triIsDegenerate)
        {
            FmPointSegmentDistance(&pntResult, pA, pB0, pB1, i, j0, j1);

            distance = pntResult.distance;
            direction = pntResult.direction;
            closestPos = pntResult.posj;

            featurePair.jtype = pntResult.featurePair.jtype;
            featurePair.j0 = pntResult.featurePair.j0;
            featurePair.j1 = pntResult.featurePair.j1;
            featurePair.u = pntResult.featurePair.u;
        }

        if (vertProj1 > 0.0f || triIsDegenerate)
        {
            FmPointSegmentDistance(&pntResult, pA, pB1, pB2, i, j1, j2);

            if (pntResult.distance < distance)
            {
                distance = pntResult.distance;
                direction = pntResult.direction;
                closestPos = pntResult.posj;

                featurePair.jtype = pntResult.featurePair.jtype;
                featurePair.j0 = pntResult.featurePair.j0;
                featurePair.j1 = pntResult.featurePair.j1;
                featurePair.u = pntResult.featurePair.u;
            }
        }

        if (vertProj2 > 0.0f || triIsDegenerate)
        {
            FmPointSegmentDistance(&pntResult, pA, pB2, pB0, i, j2, j0);

            if (pntResult.distance < distance)
            {
                distance = pntResult.distance;
                direction = pntResult.direction;
                closestPos = pntResult.posj;

                featurePair.jtype = pntResult.featurePair.jtype;
                featurePair.j0 = pntResult.featurePair.j0;
                featurePair.j1 = pntResult.featurePair.j1;
                featurePair.u = pntResult.featurePair.u;
            }
        }

        if (vertProj0 <= 0.0f && vertProj1 <= 0.0f && vertProj2 <= 0.0f && !triIsDegenerate)
        {
            float proj = dot(triNormal, vecB0ToA);
            direction = (proj > 0.0f) ? -triNormal : triNormal;
            closestPos = pA - triNormal * proj;
            distance = fabsf(proj);

            featurePair.jtype = FM_FEATURE_TYPE_FACE;
            featurePair.j0 = j0;
            featurePair.j1 = j1;
            featurePair.j2 = j2;
        }

        featurePair.itype = FM_FEATURE_TYPE_VERTEX;
        featurePair.i0 = i;

        result->distance = distance;
        result->direction = direction;
        result->posi = pA;
        result->posj = closestPos;
        result->featurePair = featurePair;
    }

}
