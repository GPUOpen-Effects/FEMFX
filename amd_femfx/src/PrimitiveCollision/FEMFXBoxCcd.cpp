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

#include "FEMFXTypes.h"
#include "FEMFXBoxCcd.h"
#include "FEMFXFeaturePairDistance.h"

namespace AMD
{
    struct FmTriFeature
    {
        uint id;       // point or edge feature
        uint type;     // 0: point 1: edge 2: face
    };

    // Algorithm:
    // Find separating axis with maximal separation.  Nearest features border this slab.
    // Due to rounding error or nongeneric cases where ties occur, may need to add larger 
    // incident feature for closest point test.
    // May include neighbor edge or face most aligned with separating axis.
    // Thus must support closest points of box faces, or box face and triangle.
    // If intersecting, separate along maximal axis to find closest features.

    // Find separation distance between box and tri along box face axis.
    // Assumes tri corner positions transformed into box coordinate system.
    static inline float FmBoxFaceTriSeparationBoxSpace(
        bool& posDirectionToTri, uint boxFaceDim, float boxHalfWidth,
        const FmVector3 triPosBoxSpace[3])
    {
        float triPos0Proj = triPosBoxSpace[0].getElem((int)boxFaceDim);
        float triPos1Proj = triPosBoxSpace[1].getElem((int)boxFaceDim);
        float triPos2Proj = triPosBoxSpace[2].getElem((int)boxFaceDim);

        // Find min and max projections of triangle corners relative to box center
        float triMinProj, triMaxProj;
        triMinProj = triMaxProj = triPos0Proj;
        triMinProj = FmMinFloat(triPos1Proj, triMinProj);
        triMaxProj = FmMaxFloat(triPos1Proj, triMaxProj);
        triMinProj = FmMinFloat(triPos2Proj, triMinProj);
        triMaxProj = FmMaxFloat(triPos2Proj, triMaxProj);

        // Compute maximum separation.  If box and tri overlap on this axis, result will be negative.
        float negSideSep = -boxHalfWidth - triMaxProj;
        float posSideSep = triMinProj - boxHalfWidth;

        bool posSide = (posSideSep > negSideSep);
        posDirectionToTri = posSide;
        return posSide ? posSideSep : negSideSep;
    }

    // Find separation distance between box and tri along tri normal axis.
    // Assumes tri corner positions transformed into box coordinate system.
    static inline float FmTriNormalBoxSeparationBoxSpace(
        bool& posDirectionToTri,
        const float boxHalfWidths[3],
        const FmVector3& triPos0BoxSpace, const FmVector3& triNormalBoxSpace)
    {
        // Find extents of box relative to triangle on tri normal axis.
        float boxPosProj = dot(-triPos0BoxSpace, triNormalBoxSpace);

        float absBoxAxisProj0 = boxHalfWidths[0] * fabsf(triNormalBoxSpace.x);
        float absBoxAxisProj1 = boxHalfWidths[1] * fabsf(triNormalBoxSpace.y);
        float absBoxAxisProj2 = boxHalfWidths[2] * fabsf(triNormalBoxSpace.z);

        float boxMinProj = boxPosProj - absBoxAxisProj0 - absBoxAxisProj1 - absBoxAxisProj2;
        float boxMaxProj = boxPosProj + absBoxAxisProj0 + absBoxAxisProj1 + absBoxAxisProj2;

        float negSideSep = boxMinProj;
        float posSideSep = -boxMaxProj;

        bool posSide = (posSideSep > negSideSep);
        posDirectionToTri = posSide;
        return posSide ? posSideSep : negSideSep;
    }

    // Find separation distance between box and tri along edge cross product direction.
    // Assumes tri corner positions transformed into box coordinate system.
    static inline float FmBoxTriEdgeCrossSeparationBoxSpace(
        bool& posDirectionToTri,
        const FmVector3& edgeCrossDir,
        uint boxEdgeDim1, float boxHalfWidth1, uint boxEdgeDim2, float boxHalfWidth2,
        const FmVector3& triPosBoxSpace, const FmVector3& triPos1BoxSpace, const FmVector3& triPos2BoxSpace)
    {
        float boxEdgeDir1Component = edgeCrossDir.getElem((int)boxEdgeDim1);
        float boxEdgeDir2Component = edgeCrossDir.getElem((int)boxEdgeDim2);

        float boxPosProj = dot(-triPosBoxSpace, edgeCrossDir);
        float absBoxAxisProj1 = boxHalfWidth1 * fabsf(boxEdgeDir1Component);
        float absBoxAxisProj2 = boxHalfWidth2 * fabsf(boxEdgeDir2Component);

        float boxMinProj = boxPosProj - absBoxAxisProj1 - absBoxAxisProj2;
        float boxMaxProj = boxPosProj + absBoxAxisProj1 + absBoxAxisProj2;

        float triProj1 = dot(triPos1BoxSpace - triPosBoxSpace, edgeCrossDir);
        float triProj2 = dot(triPos2BoxSpace - triPosBoxSpace, edgeCrossDir);
        float triMinProj = FmMinFloat(triProj1, triProj2);
        float triMaxProj = FmMaxFloat(triProj1, triProj2);

        float negSideSep = boxMinProj - triMaxProj;
        float posSideSep = triMinProj - boxMaxProj;

        bool posSide = (posSideSep > negSideSep);
        posDirectionToTri = posSide;
        return posSide ? posSideSep : negSideSep;
    }

    // Given direction in box space, find extremal feature in that direction.
    // Return face or edge feature if nearly perpendicular to direction.
    static inline void FmFindExtremalBoxFeature(FmBoxFeature& boxFeature, const FmVector3& direction)
    {
        bool signBoxAxisProj0 = (direction.x > 0.0f);
        bool signBoxAxisProj1 = (direction.y > 0.0f);
        bool signBoxAxisProj2 = (direction.z > 0.0f);
        float absDirectionX = signBoxAxisProj0 ? direction.x : -direction.x;
        float absDirectionY = signBoxAxisProj1 ? direction.y : -direction.y;
        float absDirectionZ = signBoxAxisProj2 ? direction.z : -direction.z;

        uint maxBoxCornerId = (((uint)signBoxAxisProj2) << 2) | (((uint)signBoxAxisProj1) << 1) | ((uint)signBoxAxisProj0);
        uint axisMask = 0x7;
        const float threshold = 0.001f;
        axisMask = (absDirectionX < threshold) ? axisMask & ~0x1 : axisMask;
        axisMask = (absDirectionY < threshold) ? axisMask & ~0x2 : axisMask;
        axisMask = (absDirectionZ < threshold) ? axisMask & ~0x4 : axisMask;

        boxFeature.signBits = maxBoxCornerId;
        boxFeature.axisMask = axisMask;
        boxFeature.SetDim();
    }

    // Find extremal feature of triangle in given direction.
    // Return entire face or edge feature if nearly perpendicular to direction.
    static inline void FmFindExtremalTriFeature(FmTriFeature& triFeature,
        const FmVector3 triPos[3],
        //const FmVector3& triNormal, 
        const FmVector3& direction)
    {
        //if (FmIsCloseToOrthogonal(triNormal, direction))
        //{
        //    triFeature.id = 0;
        //    triFeature.type = 2;
        //    return;
        //}

        float triPos1Proj = dot(triPos[1] - triPos[0], direction);
        float triPos2Proj = dot(triPos[2] - triPos[0], direction);

        float maxTriPosProj = 0.0f;
        uint maxTriPosId = 0;

        bool greater = (triPos1Proj > maxTriPosProj);
        maxTriPosProj = greater ? triPos1Proj : maxTriPosProj;
        maxTriPosId = greater ? 1 : maxTriPosId;

        greater = (triPos2Proj > maxTriPosProj);
        maxTriPosProj = greater ? triPos2Proj : maxTriPosProj;
        maxTriPosId = greater ? 2 : maxTriPosId;

        triFeature.id = maxTriPosId;
        triFeature.type = 0;

        FmVector3 triEdgeVec0 = triPos[(maxTriPosId + 1) % 3] - triPos[maxTriPosId];
        FmVector3 triEdgeVec1 = triPos[(maxTriPosId + 2) % 3] - triPos[maxTriPosId];

        float edgeLenSqr0 = lengthSqr(triEdgeVec0);
        float edgeLenSqr1 = lengthSqr(triEdgeVec1);

        if (edgeLenSqr0 < FM_NORMALIZE_MAG_SQR_TOL || FmIsCloseToOrthogonal(triEdgeVec0 / sqrtf(edgeLenSqr0), direction))
        {
            triFeature.type++;
        }

        if (edgeLenSqr1 < FM_NORMALIZE_MAG_SQR_TOL || FmIsCloseToOrthogonal(triEdgeVec1 / sqrtf(edgeLenSqr1), direction))
        {
            triFeature.id = (maxTriPosId + 2) % 3;
            triFeature.type++;
        }
    }

    // Compute closest points and return true if box point is within interior of its face.
    // Otherwise return false (assumes one of the tests with edges will find closest point).
    static inline bool FmBoxFaceInteriorPointDistance(
        FmDistanceResult* distResult,
        const FmVector3& boxFaceCenter, const FmVector3& boxFaceNormal,
        const FmVector3& boxEdgeDirA, float boxHalfWidthA,
        const FmVector3& boxEdgeDirB, float boxHalfWidthB,
        const FmVector3& point, uint j)
    {
        float normalProj = dot(point - boxFaceCenter, boxFaceNormal);
        float projA = dot(point - boxFaceCenter, boxEdgeDirA);
        float projB = dot(point - boxFaceCenter, boxEdgeDirB);
        float absProjA = fabsf(projA);
        float absProjB = fabsf(projB);

        if (absProjA < boxHalfWidthA && absProjB < boxHalfWidthB)
        {
            distResult->direction = boxFaceNormal;
            distResult->distance = normalProj;
            distResult->posi = boxFaceCenter + projA * boxEdgeDirA + projB * boxEdgeDirB;
            distResult->posj = point;
            distResult->featurePair.itype = FM_FEATURE_TYPE_FACE;
            distResult->featurePair.jtype = FM_FEATURE_TYPE_VERTEX;
            distResult->featurePair.i0 = 0;
            distResult->featurePair.j0 = j;
            return true;
        }

        return false;
    }

    // Find closest points between box face and triangle.
    // Finding the minimum distance over all edge/edge and point/face feature pairs.
    // TODO: Separate point/point test and more limited edge/edge test to avoid redundant work.
    // TODO: Early out for distance within tolerance of lower bound.
    static inline void FmBoxFaceTriClosestPoints(
        FmDistanceResult* distResult,
        const FmVector3& boxFaceCenter, const FmVector3& boxFaceNormal,
        const FmVector3& boxEdgeDirA, float boxHalfWidthA,
        const FmVector3& boxEdgeDirB, float boxHalfWidthB,
        const FmVector3 triPos[3])
    {
        FmDistanceResult lDistResult, minResult;

        FmVector3 boxCorner0 = boxFaceCenter - boxEdgeDirA * boxHalfWidthA - boxEdgeDirB * boxHalfWidthB;
        FmVector3 boxCorner1 = boxFaceCenter + boxEdgeDirA * boxHalfWidthA - boxEdgeDirB * boxHalfWidthB;
        FmVector3 boxCorner2 = boxFaceCenter + boxEdgeDirA * boxHalfWidthA + boxEdgeDirB * boxHalfWidthB;
        FmVector3 boxCorner3 = boxFaceCenter - boxEdgeDirA * boxHalfWidthA + boxEdgeDirB * boxHalfWidthB;

        // Test edge pairs
        FmSegmentPairDistance(&minResult, boxCorner0, boxCorner1, triPos[0], triPos[1], 0, 1, 0, 1);
        FmSegmentPairDistance(&lDistResult, boxCorner0, boxCorner1, triPos[1], triPos[2], 0, 1, 1, 2);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        FmSegmentPairDistance(&lDistResult, boxCorner0, boxCorner1, triPos[2], triPos[0], 0, 1, 2, 0);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        FmSegmentPairDistance(&lDistResult, boxCorner1, boxCorner2, triPos[0], triPos[1], 1, 2, 0, 1);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        FmSegmentPairDistance(&lDistResult, boxCorner1, boxCorner2, triPos[1], triPos[2], 1, 2, 1, 2);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        FmSegmentPairDistance(&lDistResult, boxCorner1, boxCorner2, triPos[2], triPos[0], 1, 2, 2, 0);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        FmSegmentPairDistance(&lDistResult, boxCorner2, boxCorner3, triPos[0], triPos[1], 2, 3, 0, 1);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        FmSegmentPairDistance(&lDistResult, boxCorner2, boxCorner3, triPos[1], triPos[2], 2, 3, 1, 2);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        FmSegmentPairDistance(&lDistResult, boxCorner2, boxCorner3, triPos[2], triPos[0], 2, 3, 2, 0);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        FmSegmentPairDistance(&lDistResult, boxCorner3, boxCorner0, triPos[0], triPos[1], 3, 0, 0, 1);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        FmSegmentPairDistance(&lDistResult, boxCorner3, boxCorner0, triPos[1], triPos[2], 3, 0, 1, 2);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        FmSegmentPairDistance(&lDistResult, boxCorner3, boxCorner0, triPos[2], triPos[0], 3, 0, 2, 0);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }

        // Test corners of box with tri face
        FmPointTriangleDistance(&lDistResult, boxCorner0, triPos[0], triPos[1], triPos[2], 0, 0, 1, 2);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        FmPointTriangleDistance(&lDistResult, boxCorner1, triPos[0], triPos[1], triPos[2], 1, 0, 1, 2);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        FmPointTriangleDistance(&lDistResult, boxCorner2, triPos[0], triPos[1], triPos[2], 2, 0, 1, 2);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        FmPointTriangleDistance(&lDistResult, boxCorner3, triPos[0], triPos[1], triPos[2], 3, 0, 1, 2);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }

        // Test corners of tri with box face
        bool valid = FmBoxFaceInteriorPointDistance(&lDistResult, boxFaceCenter, boxFaceNormal, boxEdgeDirA, boxHalfWidthA, boxEdgeDirB, boxHalfWidthB, triPos[0], 0);
        if (valid && lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        valid = FmBoxFaceInteriorPointDistance(&lDistResult, boxFaceCenter, boxFaceNormal, boxEdgeDirA, boxHalfWidthA, boxEdgeDirB, boxHalfWidthB, triPos[1], 1);
        if (valid && lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        valid = FmBoxFaceInteriorPointDistance(&lDistResult, boxFaceCenter, boxFaceNormal, boxEdgeDirA, boxHalfWidthA, boxEdgeDirB, boxHalfWidthB, triPos[2], 2);
        if (valid && lDistResult.distance < minResult.distance) { minResult = lDistResult; }

        *distResult = minResult;
    }

    void FmBoxFacePlusXTriClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const float boxHalfWidths[3],
        const FmVector3 triPosBoxSpace[3])
    {
        FmVector3 boxFaceCenter = FmInitVector3(boxHalfWidths[0], 0.0f, 0.0f);
        FmVector3 boxFaceNormal = FmInitVector3(1.0f, 0.0f, 0.0f);
        FmVector3 boxEdgeDirA = FmInitVector3(0.0f, 1.0f, 0.0f);
        float boxHalfWidthA = boxHalfWidths[1];
        FmVector3 boxEdgeDirB = FmInitVector3(0.0f, 0.0f, 1.0f);
        float boxHalfWidthB = boxHalfWidths[2];

        FmBoxFaceTriClosestPoints(distResult, boxFaceCenter, boxFaceNormal, boxEdgeDirA, boxHalfWidthA, boxEdgeDirB, boxHalfWidthB, triPosBoxSpace);
    }

    void FmBoxFaceMinusXTriClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const float boxHalfWidths[3],
        const FmVector3 triPosBoxSpace[3])
    {
        FmVector3 boxFaceCenter = FmInitVector3(-boxHalfWidths[0], 0.0f, 0.0f);
        FmVector3 boxFaceNormal = FmInitVector3(-1.0f, 0.0f, 0.0f);
        FmVector3 boxEdgeDirA = FmInitVector3(0.0f, 1.0f, 0.0f);
        float boxHalfWidthA = boxHalfWidths[1];
        FmVector3 boxEdgeDirB = FmInitVector3(0.0f, 0.0f, 1.0f);
        float boxHalfWidthB = boxHalfWidths[2];

        FmBoxFaceTriClosestPoints(distResult, boxFaceCenter, boxFaceNormal, boxEdgeDirA, boxHalfWidthA, boxEdgeDirB, boxHalfWidthB, triPosBoxSpace);
    }

    void FmBoxFacePlusYTriClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const float boxHalfWidths[3],
        const FmVector3 triPosBoxSpace[3])
    {
        FmVector3 boxFaceCenter = FmInitVector3(0.0f, boxHalfWidths[1], 0.0f);
        FmVector3 boxFaceNormal = FmInitVector3(0.0f, 1.0f, 0.0f);
        FmVector3 boxEdgeDirA = FmInitVector3(1.0f, 0.0f, 0.0f);
        float boxHalfWidthA = boxHalfWidths[0];
        FmVector3 boxEdgeDirB = FmInitVector3(0.0f, 0.0f, 1.0f);
        float boxHalfWidthB = boxHalfWidths[2];

        FmBoxFaceTriClosestPoints(distResult, boxFaceCenter, boxFaceNormal, boxEdgeDirA, boxHalfWidthA, boxEdgeDirB, boxHalfWidthB, triPosBoxSpace);
    }

    void FmBoxFaceMinusYTriClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const float boxHalfWidths[3],
        const FmVector3 triPosBoxSpace[3])
    {
        FmVector3 boxFaceCenter = FmInitVector3(0.0f, -boxHalfWidths[1], 0.0f);
        FmVector3 boxFaceNormal = FmInitVector3(0.0f, -1.0f, 0.0f);
        FmVector3 boxEdgeDirA = FmInitVector3(1.0f, 0.0f, 0.0f);
        float boxHalfWidthA = boxHalfWidths[0];
        FmVector3 boxEdgeDirB = FmInitVector3(0.0f, 0.0f, 1.0f);
        float boxHalfWidthB = boxHalfWidths[2];

        FmBoxFaceTriClosestPoints(distResult, boxFaceCenter, boxFaceNormal, boxEdgeDirA, boxHalfWidthA, boxEdgeDirB, boxHalfWidthB, triPosBoxSpace);
    }

    void FmBoxFacePlusZTriClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const float boxHalfWidths[3],
        const FmVector3 triPosBoxSpace[3])
    {
        FmVector3 boxFaceCenter = FmInitVector3(0.0f, 0.0f, boxHalfWidths[2]);
        FmVector3 boxFaceNormal = FmInitVector3(0.0f, 0.0f, 1.0f);
        FmVector3 boxEdgeDirA = FmInitVector3(1.0f, 0.0f, 0.0f);
        float boxHalfWidthA = boxHalfWidths[0];
        FmVector3 boxEdgeDirB = FmInitVector3(0.0f, 1.0f, 0.0f);
        float boxHalfWidthB = boxHalfWidths[1];

        FmBoxFaceTriClosestPoints(distResult, boxFaceCenter, boxFaceNormal, boxEdgeDirA, boxHalfWidthA, boxEdgeDirB, boxHalfWidthB, triPosBoxSpace);
    }

    void FmBoxFaceMinusZTriClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const float boxHalfWidths[3],
        const FmVector3 triPosBoxSpace[3])
    {
        FmVector3 boxFaceCenter = FmInitVector3(0.0f, 0.0f, -boxHalfWidths[2]);
        FmVector3 boxFaceNormal = FmInitVector3(0.0f, 0.0f, -1.0f);
        FmVector3 boxEdgeDirA = FmInitVector3(1.0f, 0.0f, 0.0f);
        float boxHalfWidthA = boxHalfWidths[0];
        FmVector3 boxEdgeDirB = FmInitVector3(0.0f, 1.0f, 0.0f);
        float boxHalfWidthB = boxHalfWidths[1];

        FmBoxFaceTriClosestPoints(distResult, boxFaceCenter, boxFaceNormal, boxEdgeDirA, boxHalfWidthA, boxEdgeDirB, boxHalfWidthB, triPosBoxSpace);
    }

    static inline void FmBoxFaceTriClosestPointsBoxSpace(FmDistanceResult* contact,
        const float boxHalfWidths[3], uint boxFaceDim, uint boxFaceSign,
        const FmVector3 triPosBoxSpace[3])
    {
        if (boxFaceDim == 0)
        {
            if (boxFaceSign == 0)
            {
                FmBoxFaceMinusXTriClosestPointsBoxSpace(contact, boxHalfWidths, triPosBoxSpace);
            }
            else
            {
                FmBoxFacePlusXTriClosestPointsBoxSpace(contact, boxHalfWidths, triPosBoxSpace);
            }
        }
        else if (boxFaceDim == 1)
        {
            if (boxFaceSign == 0)
            {
                FmBoxFaceMinusYTriClosestPointsBoxSpace(contact, boxHalfWidths, triPosBoxSpace);
            }
            else
            {
                FmBoxFacePlusYTriClosestPointsBoxSpace(contact, boxHalfWidths, triPosBoxSpace);
            }
        }
        else
        {
            if (boxFaceSign == 0)
            {
                FmBoxFaceMinusZTriClosestPointsBoxSpace(contact, boxHalfWidths, triPosBoxSpace);
            }
            else
            {
                FmBoxFacePlusZTriClosestPointsBoxSpace(contact, boxHalfWidths, triPosBoxSpace);
            }
        }
    }

    // Find closest points between box face and edge.
    // Finding the minimum distance over all edge/edge and point/face feature pairs.
    // TODO: Separate point/point test and more limited edge/edge test to avoid redundant work.
    // TODO: Early out for distance within tolerance of lower bound.
    static inline void FmBoxFaceEdgeClosestPoints(
        FmDistanceResult* distResult,
        const FmVector3& boxFaceCenter, const FmVector3& boxFaceNormal,
        const FmVector3& boxEdgeDirA, float boxHalfWidthA,
        const FmVector3& boxEdgeDirB, float boxHalfWidthB,
        const FmVector3& edgePos0, const FmVector3& edgePos1, uint j0, uint j1)
    {
        // Check distances of edge pairs
        FmDistanceResult lDistResult, minResult;

        FmVector3 boxCorner0 = boxFaceCenter - boxEdgeDirA * boxHalfWidthA - boxEdgeDirB * boxHalfWidthB;
        FmVector3 boxCorner1 = boxFaceCenter + boxEdgeDirA * boxHalfWidthA - boxEdgeDirB * boxHalfWidthB;
        FmVector3 boxCorner2 = boxFaceCenter + boxEdgeDirA * boxHalfWidthA + boxEdgeDirB * boxHalfWidthB;
        FmVector3 boxCorner3 = boxFaceCenter - boxEdgeDirA * boxHalfWidthA + boxEdgeDirB * boxHalfWidthB;

        // Find closest feature of edge pairs
        FmSegmentPairDistance(&minResult, boxCorner0, boxCorner1, edgePos0, edgePos1, 0, 1, j0, j1);
        FmSegmentPairDistance(&lDistResult, boxCorner1, boxCorner2, edgePos0, edgePos1, 1, 2, j0, j1);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        FmSegmentPairDistance(&lDistResult, boxCorner2, boxCorner3, edgePos0, edgePos1, 2, 3, j0, j1);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        FmSegmentPairDistance(&lDistResult, boxCorner3, boxCorner0, edgePos0, edgePos1, 3, 0, j0, j1);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }

        // Test edge points with box face
        bool valid = FmBoxFaceInteriorPointDistance(&lDistResult, boxFaceCenter, boxFaceNormal, boxEdgeDirA, boxHalfWidthA, boxEdgeDirB, boxHalfWidthB, edgePos0, j0);
        if (valid && lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        valid = FmBoxFaceInteriorPointDistance(&lDistResult, boxFaceCenter, boxFaceNormal, boxEdgeDirA, boxHalfWidthA, boxEdgeDirB, boxHalfWidthB, edgePos1, j1);
        if (valid && lDistResult.distance < minResult.distance) { minResult = lDistResult; }

        *distResult = minResult;
    }

    void FmBoxFacePlusXEdgeClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const float boxHalfWidths[3],
        const FmVector3& edgePos0BoxSpace, const FmVector3& edgePos1BoxSpace, uint j0, uint j1)
    {
        FmVector3 boxFaceCenter = FmInitVector3(boxHalfWidths[0], 0.0f, 0.0f);
        FmVector3 boxFaceNormal = FmInitVector3(1.0f, 0.0f, 0.0f);
        FmVector3 boxEdgeDirA = FmInitVector3(0.0f, 1.0f, 0.0f);
        float boxHalfWidthA = boxHalfWidths[1];
        FmVector3 boxEdgeDirB = FmInitVector3(0.0f, 0.0f, 1.0f);
        float boxHalfWidthB = boxHalfWidths[2];

        FmBoxFaceEdgeClosestPoints(distResult,
            boxFaceCenter, boxFaceNormal, boxEdgeDirA, boxHalfWidthA, boxEdgeDirB, boxHalfWidthB,
            edgePos0BoxSpace, edgePos1BoxSpace, j0, j1);
    }

    void FmBoxFaceMinusXEdgeClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const float boxHalfWidths[3],
        const FmVector3& edgePos0BoxSpace, const FmVector3& edgePos1BoxSpace, uint j0, uint j1)
    {
        FmVector3 boxFaceCenter = FmInitVector3(-boxHalfWidths[0], 0.0f, 0.0f);
        FmVector3 boxFaceNormal = FmInitVector3(-1.0f, 0.0f, 0.0f);
        FmVector3 boxEdgeDirA = FmInitVector3(0.0f, 1.0f, 0.0f);
        float boxHalfWidthA = boxHalfWidths[1];
        FmVector3 boxEdgeDirB = FmInitVector3(0.0f, 0.0f, 1.0f);
        float boxHalfWidthB = boxHalfWidths[2];

        FmBoxFaceEdgeClosestPoints(distResult,
            boxFaceCenter, boxFaceNormal, boxEdgeDirA, boxHalfWidthA, boxEdgeDirB, boxHalfWidthB,
            edgePos0BoxSpace, edgePos1BoxSpace, j0, j1);
    }

    void FmBoxFacePlusYEdgeClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const float boxHalfWidths[3],
        const FmVector3& edgePos0BoxSpace, const FmVector3& edgePos1BoxSpace, uint j0, uint j1)
    {
        FmVector3 boxFaceCenter = FmInitVector3(0.0f, boxHalfWidths[1], 0.0f);
        FmVector3 boxFaceNormal = FmInitVector3(0.0f, 1.0f, 0.0f);
        FmVector3 boxEdgeDirA = FmInitVector3(1.0f, 0.0f, 0.0f);
        float boxHalfWidthA = boxHalfWidths[0];
        FmVector3 boxEdgeDirB = FmInitVector3(0.0f, 0.0f, 1.0f);
        float boxHalfWidthB = boxHalfWidths[2];

        FmBoxFaceEdgeClosestPoints(distResult,
            boxFaceCenter, boxFaceNormal, boxEdgeDirA, boxHalfWidthA, boxEdgeDirB, boxHalfWidthB,
            edgePos0BoxSpace, edgePos1BoxSpace, j0, j1);
    }

    void FmBoxFaceMinusYEdgeClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const float boxHalfWidths[3],
        const FmVector3& edgePos0BoxSpace, const FmVector3& edgePos1BoxSpace, uint j0, uint j1)
    {
        FmVector3 boxFaceCenter = FmInitVector3(0.0f, -boxHalfWidths[1], 0.0f);
        FmVector3 boxFaceNormal = FmInitVector3(0.0f, -1.0f, 0.0f);
        FmVector3 boxEdgeDirA = FmInitVector3(1.0f, 0.0f, 0.0f);
        float boxHalfWidthA = boxHalfWidths[0];
        FmVector3 boxEdgeDirB = FmInitVector3(0.0f, 0.0f, 1.0f);
        float boxHalfWidthB = boxHalfWidths[2];

        FmBoxFaceEdgeClosestPoints(distResult,
            boxFaceCenter, boxFaceNormal, boxEdgeDirA, boxHalfWidthA, boxEdgeDirB, boxHalfWidthB,
            edgePos0BoxSpace, edgePos1BoxSpace, j0, j1);
    }

    void FmBoxFacePlusZEdgeClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const float boxHalfWidths[3],
        const FmVector3& edgePos0BoxSpace, const FmVector3& edgePos1BoxSpace, uint j0, uint j1)
    {
        FmVector3 boxFaceCenter = FmInitVector3(0.0f, 0.0f, boxHalfWidths[2]);
        FmVector3 boxFaceNormal = FmInitVector3(0.0f, 0.0f, 1.0f);
        FmVector3 boxEdgeDirA = FmInitVector3(1.0f, 0.0f, 0.0f);
        float boxHalfWidthA = boxHalfWidths[0];
        FmVector3 boxEdgeDirB = FmInitVector3(0.0f, 1.0f, 0.0f);
        float boxHalfWidthB = boxHalfWidths[1];

        FmBoxFaceEdgeClosestPoints(distResult,
            boxFaceCenter, boxFaceNormal, boxEdgeDirA, boxHalfWidthA, boxEdgeDirB, boxHalfWidthB,
            edgePos0BoxSpace, edgePos1BoxSpace, j0, j1);
    }

    void FmBoxFaceMinusZEdgeClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const float boxHalfWidths[3],
        const FmVector3& edgePos0BoxSpace, const FmVector3& edgePos1BoxSpace, uint j0, uint j1)
    {
        FmVector3 boxFaceCenter = FmInitVector3(0.0f, 0.0f, -boxHalfWidths[2]);
        FmVector3 boxFaceNormal = FmInitVector3(0.0f, 0.0f, -1.0f);
        FmVector3 boxEdgeDirA = FmInitVector3(1.0f, 0.0f, 0.0f);
        float boxHalfWidthA = boxHalfWidths[0];
        FmVector3 boxEdgeDirB = FmInitVector3(0.0f, 1.0f, 0.0f);
        float boxHalfWidthB = boxHalfWidths[1];

        FmBoxFaceEdgeClosestPoints(distResult,
            boxFaceCenter, boxFaceNormal, boxEdgeDirA, boxHalfWidthA, boxEdgeDirB, boxHalfWidthB,
            edgePos0BoxSpace, edgePos1BoxSpace, j0, j1);
    }

    static inline void FmBoxFaceEdgeClosestPointsBoxSpace(FmDistanceResult* contact,
        const float boxHalfWidths[3], uint boxFaceDim, uint boxFaceSign,
        const FmVector3& edgePos0BoxSpace, const FmVector3& edgePos1BoxSpace, uint j0, uint j1)
    {
        FmVector3 boxFaceCenter, boxFaceDir, boxEdgeDirA, boxEdgeDirB;
        if (boxFaceDim == 0)
        {
            if (boxFaceSign == 0)
            {
                FmBoxFaceMinusXEdgeClosestPointsBoxSpace(contact, boxHalfWidths, edgePos0BoxSpace, edgePos1BoxSpace, j0, j1);
            }
            else
            {
                FmBoxFacePlusXEdgeClosestPointsBoxSpace(contact, boxHalfWidths, edgePos0BoxSpace, edgePos1BoxSpace, j0, j1);
            }
        }
        else if (boxFaceDim == 1)
        {
            if (boxFaceSign == 0)
            {
                FmBoxFaceMinusYEdgeClosestPointsBoxSpace(contact, boxHalfWidths, edgePos0BoxSpace, edgePos1BoxSpace, j0, j1);
            }
            else
            {
                FmBoxFacePlusYEdgeClosestPointsBoxSpace(contact, boxHalfWidths, edgePos0BoxSpace, edgePos1BoxSpace, j0, j1);
            }
        }
        else
        {
            if (boxFaceSign == 0)
            {
                FmBoxFaceMinusZEdgeClosestPointsBoxSpace(contact, boxHalfWidths, edgePos0BoxSpace, edgePos1BoxSpace, j0, j1);
            }
            else
            {
                FmBoxFacePlusZEdgeClosestPointsBoxSpace(contact, boxHalfWidths, edgePos0BoxSpace, edgePos1BoxSpace, j0, j1);
            }
        }
    }

    // Find closest points between box face and point.
    static inline void FmBoxFacePointClosestPoints(
        FmDistanceResult* distResult,
        const FmVector3& boxFaceCenter, const FmVector3& boxFaceNormal,
        const FmVector3& boxEdgeDirA, float boxHalfWidthA,
        const FmVector3& boxEdgeDirB, float boxHalfWidthB,
        const FmVector3& point, uint j)
    {
        float projA = dot(point - boxFaceCenter, boxEdgeDirA);
        float projB = dot(point - boxFaceCenter, boxEdgeDirB);
        float absProjA = fabsf(projA);
        float absProjB = fabsf(projB);

        projA = FmMinFloat(projA, boxHalfWidthA);
        projA = FmMaxFloat(projA, -boxHalfWidthA);
        projB = FmMinFloat(projB, boxHalfWidthB);
        projB = FmMaxFloat(projB, -boxHalfWidthB);

        bool clampedA = absProjA > boxHalfWidthA;
        bool clampedB = absProjB > boxHalfWidthB;
        bool posA = projA > 0.0f;
        bool posB = projB > 0.0f;

        FmVector3 boxPoint = boxFaceCenter + projA * boxEdgeDirA + projB * boxEdgeDirB;
        distResult->direction = FmSafeNormalize(point - boxPoint, boxFaceNormal);
        distResult->distance = length(point - boxPoint);
        distResult->posi = boxFaceCenter + projA * boxEdgeDirA + projB * boxEdgeDirB;
        distResult->posj = point;
        distResult->featurePair.itype =
            (clampedA && clampedB) ? FM_FEATURE_TYPE_VERTEX :
            (clampedA ? FM_FEATURE_TYPE_EDGE :
            (clampedB ? FM_FEATURE_TYPE_EDGE : FM_FEATURE_TYPE_FACE));
        distResult->featurePair.jtype = FM_FEATURE_TYPE_VERTEX;
        distResult->featurePair.i0 = ((uint)posA) | (((uint)posB) << 1);         // signBits
        distResult->featurePair.i1 = ((uint)clampedA) | (((uint)clampedB) << 1); // axisMask
        distResult->featurePair.j0 = j;
    }

    void FmBoxFacePlusXPointClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const float boxHalfWidths[3],
        const FmVector3& pointBoxSpace, uint j)
    {
        FmVector3 boxFaceCenter = FmInitVector3(boxHalfWidths[0], 0.0f, 0.0f);
        FmVector3 boxFaceNormal = FmInitVector3(1.0f, 0.0f, 0.0f);
        FmVector3 boxPointDirA = FmInitVector3(0.0f, 1.0f, 0.0f);
        float boxHalfWidthA = boxHalfWidths[1];
        FmVector3 boxPointDirB = FmInitVector3(0.0f, 0.0f, 1.0f);
        float boxHalfWidthB = boxHalfWidths[2];

        FmBoxFacePointClosestPoints(distResult,
            boxFaceCenter, boxFaceNormal, boxPointDirA, boxHalfWidthA, boxPointDirB, boxHalfWidthB,
            pointBoxSpace, j);
    }

    void FmBoxFaceMinusXPointClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const float boxHalfWidths[3],
        const FmVector3& pointBoxSpace, uint j)
    {
        FmVector3 boxFaceCenter = FmInitVector3(-boxHalfWidths[0], 0.0f, 0.0f);
        FmVector3 boxFaceNormal = FmInitVector3(-1.0f, 0.0f, 0.0f);
        FmVector3 boxPointDirA = FmInitVector3(0.0f, 1.0f, 0.0f);
        float boxHalfWidthA = boxHalfWidths[1];
        FmVector3 boxPointDirB = FmInitVector3(0.0f, 0.0f, 1.0f);
        float boxHalfWidthB = boxHalfWidths[2];

        FmBoxFacePointClosestPoints(distResult,
            boxFaceCenter, boxFaceNormal, boxPointDirA, boxHalfWidthA, boxPointDirB, boxHalfWidthB,
            pointBoxSpace, j);
    }

    void FmBoxFacePlusYPointClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const float boxHalfWidths[3],
        const FmVector3& pointBoxSpace, uint j)
    {
        FmVector3 boxFaceCenter = FmInitVector3(0.0f, boxHalfWidths[1], 0.0f);
        FmVector3 boxFaceNormal = FmInitVector3(0.0f, 1.0f, 0.0f);
        FmVector3 boxPointDirA = FmInitVector3(1.0f, 0.0f, 0.0f);
        float boxHalfWidthA = boxHalfWidths[0];
        FmVector3 boxPointDirB = FmInitVector3(0.0f, 0.0f, 1.0f);
        float boxHalfWidthB = boxHalfWidths[2];

        FmBoxFacePointClosestPoints(distResult,
            boxFaceCenter, boxFaceNormal, boxPointDirA, boxHalfWidthA, boxPointDirB, boxHalfWidthB,
            pointBoxSpace, j);
    }

    void FmBoxFaceMinusYPointClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const float boxHalfWidths[3],
        const FmVector3& pointBoxSpace, uint j)
    {
        FmVector3 boxFaceCenter = FmInitVector3(0.0f, -boxHalfWidths[1], 0.0f);
        FmVector3 boxFaceNormal = FmInitVector3(0.0f, -1.0f, 0.0f);
        FmVector3 boxPointDirA = FmInitVector3(1.0f, 0.0f, 0.0f);
        float boxHalfWidthA = boxHalfWidths[0];
        FmVector3 boxPointDirB = FmInitVector3(0.0f, 0.0f, 1.0f);
        float boxHalfWidthB = boxHalfWidths[2];

        FmBoxFacePointClosestPoints(distResult,
            boxFaceCenter, boxFaceNormal, boxPointDirA, boxHalfWidthA, boxPointDirB, boxHalfWidthB,
            pointBoxSpace, j);
    }

    void FmBoxFacePlusZPointClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const float boxHalfWidths[3],
        const FmVector3& pointBoxSpace, uint j)
    {
        FmVector3 boxFaceCenter = FmInitVector3(0.0f, 0.0f, boxHalfWidths[2]);
        FmVector3 boxFaceNormal = FmInitVector3(0.0f, 0.0f, 1.0f);
        FmVector3 boxPointDirA = FmInitVector3(1.0f, 0.0f, 0.0f);
        float boxHalfWidthA = boxHalfWidths[0];
        FmVector3 boxPointDirB = FmInitVector3(0.0f, 1.0f, 0.0f);
        float boxHalfWidthB = boxHalfWidths[1];

        FmBoxFacePointClosestPoints(distResult,
            boxFaceCenter, boxFaceNormal, boxPointDirA, boxHalfWidthA, boxPointDirB, boxHalfWidthB,
            pointBoxSpace, j);
    }

    void FmBoxFaceMinusZPointClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const float boxHalfWidths[3],
        const FmVector3& pointBoxSpace, uint j)
    {
        FmVector3 boxFaceCenter = FmInitVector3(0.0f, 0.0f, -boxHalfWidths[2]);
        FmVector3 boxFaceNormal = FmInitVector3(0.0f, 0.0f, -1.0f);
        FmVector3 boxPointDirA = FmInitVector3(1.0f, 0.0f, 0.0f);
        float boxHalfWidthA = boxHalfWidths[0];
        FmVector3 boxPointDirB = FmInitVector3(0.0f, 1.0f, 0.0f);
        float boxHalfWidthB = boxHalfWidths[1];

        FmBoxFacePointClosestPoints(distResult,
            boxFaceCenter, boxFaceNormal, boxPointDirA, boxHalfWidthA, boxPointDirB, boxHalfWidthB,
            pointBoxSpace, j);
    }

    static inline void FmBoxFacePointClosestPointsBoxSpace(FmDistanceResult* contact,
        const float boxHalfWidths[3], uint boxFaceDim, uint boxFaceSign,
        const FmVector3& pointBoxSpace, uint j)
    {
        if (boxFaceDim == 0)
        {
            if (boxFaceSign == 0)
            {
                FmBoxFaceMinusXPointClosestPointsBoxSpace(contact, boxHalfWidths, pointBoxSpace, j);
            }
            else
            {
                FmBoxFacePlusXPointClosestPointsBoxSpace(contact, boxHalfWidths, pointBoxSpace, j);
            }
        }
        else if (boxFaceDim == 1)
        {
            if (boxFaceSign == 0)
            {
                FmBoxFaceMinusYPointClosestPointsBoxSpace(contact, boxHalfWidths, pointBoxSpace, j);
            }
            else
            {
                FmBoxFacePlusYPointClosestPointsBoxSpace(contact, boxHalfWidths, pointBoxSpace, j);
            }
        }
        else
        {
            if (boxFaceSign == 0)
            {
                FmBoxFaceMinusZPointClosestPointsBoxSpace(contact, boxHalfWidths, pointBoxSpace, j);
            }
            else
            {
                FmBoxFacePlusZPointClosestPointsBoxSpace(contact, boxHalfWidths, pointBoxSpace, j);
            }
        }
    }

    // Find closest points between edge and triangle.
    // Finding the minimum distance over all edge/edge and point/face feature pairs.
    // TODO: Separate point/point test and more limited edge/edge test to avoid redundant work.
    // TODO: Early out for distance within tolerance of lower bound.
    static inline void FmEdgeTriClosestPoints(
        FmDistanceResult* distResult,
        const FmVector3& edgePos0, const FmVector3& edgePos1,
        const FmVector3 triPos[3])
    {
        FmDistanceResult lDistResult, minResult;

        // Test edge pairs
        FmSegmentPairDistance(&minResult, edgePos0, edgePos1, triPos[0], triPos[1], 0, 1, 0, 1);
        FmSegmentPairDistance(&lDistResult, edgePos0, edgePos1, triPos[1], triPos[2], 0, 1, 1, 2);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        FmSegmentPairDistance(&lDistResult, edgePos0, edgePos1, triPos[2], triPos[0], 0, 1, 2, 0);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }

        // Test edge endpoints with tri face
        FmPointTriangleDistance(&lDistResult, edgePos0, triPos[0], triPos[1], triPos[2], 0, 0, 1, 2);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }
        FmPointTriangleDistance(&lDistResult, edgePos1, triPos[0], triPos[1], triPos[2], 1, 0, 1, 2);
        if (lDistResult.distance < minResult.distance) { minResult = lDistResult; }

        *distResult = minResult;
    }

    void FmEdgeXTriClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const FmVector3& edgeCenter, float edgeHalfWidth,
        const FmVector3 triPosBoxSpace[3])
    {
        FmVector3 edgePos0 = edgeCenter + FmInitVector3(-edgeHalfWidth, 0.0f, 0.0f);
        FmVector3 edgePos1 = edgeCenter + FmInitVector3(edgeHalfWidth, 0.0f, 0.0f);
        FmEdgeTriClosestPoints(distResult, edgePos0, edgePos1, triPosBoxSpace);
    }

    void FmEdgeYTriClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const FmVector3& edgeCenter, float edgeHalfWidth,
        const FmVector3 triPosBoxSpace[3])
    {
        FmVector3 edgePos0 = edgeCenter + FmInitVector3(0.0f, -edgeHalfWidth, 0.0f);
        FmVector3 edgePos1 = edgeCenter + FmInitVector3(0.0f, edgeHalfWidth, 0.0f);
        FmEdgeTriClosestPoints(distResult, edgePos0, edgePos1, triPosBoxSpace);
    }

    void FmEdgeZTriClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const FmVector3& edgeCenter, float edgeHalfWidth,
        const FmVector3 triPosBoxSpace[3])
    {
        FmVector3 edgePos0 = edgeCenter + FmInitVector3(0.0f, 0.0f, -edgeHalfWidth);
        FmVector3 edgePos1 = edgeCenter + FmInitVector3(0.0f, 0.0f, edgeHalfWidth);
        FmEdgeTriClosestPoints(distResult, edgePos0, edgePos1, triPosBoxSpace);
    }

    static inline void FmBoxEdgeTriClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const FmBoxFeature& boxFeature, const float boxHalfWidths[3],
        const FmVector3 triPosBoxSpace[3])
    {
        float boxSpaceX = (boxFeature.signBits & 0x1) ? boxHalfWidths[0] : -boxHalfWidths[0];
        float boxSpaceY = (boxFeature.signBits & 0x2) ? boxHalfWidths[1] : -boxHalfWidths[1];
        float boxSpaceZ = (boxFeature.signBits & 0x4) ? boxHalfWidths[2] : -boxHalfWidths[2];
        if ((boxFeature.axisMask & 0x1) == 0)
        {
            FmVector3 edgeCenter = FmInitVector3(0.0f, boxSpaceY, boxSpaceZ);
            FmEdgeXTriClosestPointsBoxSpace(distResult, edgeCenter, boxHalfWidths[0], triPosBoxSpace);
        }
        else if ((boxFeature.axisMask & 0x2) == 0)
        {
            FmVector3 edgeCenter = FmInitVector3(boxSpaceX, 0.0f, boxSpaceZ);
            FmEdgeYTriClosestPointsBoxSpace(distResult, edgeCenter, boxHalfWidths[1], triPosBoxSpace);
        }
        else
        {
            FmVector3 edgeCenter = FmInitVector3(boxSpaceX, boxSpaceY, 0.0f);
            FmEdgeZTriClosestPointsBoxSpace(distResult, edgeCenter, boxHalfWidths[2], triPosBoxSpace);
        }
    }

    void FmEdgeXSegmentClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const FmVector3& edgeCenter, float edgeHalfWidth,
        const FmVector3& segmentPos0, const FmVector3& segmentPos1)
    {
        FmVector3 edgePos0 = edgeCenter + FmInitVector3(-edgeHalfWidth, 0.0f, 0.0f);
        FmVector3 edgePos1 = edgeCenter + FmInitVector3(edgeHalfWidth, 0.0f, 0.0f);
        FmSegmentPairDistance(distResult, edgePos0, edgePos1, segmentPos0, segmentPos1, 0, 1, 0, 1);
    }

    void FmEdgeYSegmentClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const FmVector3& edgeCenter, float edgeHalfWidth,
        const FmVector3& segmentPos0, const FmVector3& segmentPos1)
    {
        FmVector3 edgePos0 = edgeCenter + FmInitVector3(0.0f, -edgeHalfWidth, 0.0f);
        FmVector3 edgePos1 = edgeCenter + FmInitVector3(0.0f, edgeHalfWidth, 0.0f);
        FmSegmentPairDistance(distResult, edgePos0, edgePos1, segmentPos0, segmentPos1, 0, 1, 0, 1);
    }

    void FmEdgeZSegmentClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const FmVector3& edgeCenter, float edgeHalfWidth,
        const FmVector3& segmentPos0, const FmVector3& segmentPos1)
    {
        FmVector3 edgePos0 = edgeCenter + FmInitVector3(0.0f, 0.0f, -edgeHalfWidth);
        FmVector3 edgePos1 = edgeCenter + FmInitVector3(0.0f, 0.0f, edgeHalfWidth);
        FmSegmentPairDistance(distResult, edgePos0, edgePos1, segmentPos0, segmentPos1, 0, 1, 0, 1);
    }

    static inline void FmBoxEdgeSegmentClosestPointsBoxSpace(
        FmDistanceResult* distResult,
        const FmBoxFeature& boxFeature, const float boxHalfWidths[3],
        const FmVector3& segmentPos0BoxSpace, const FmVector3& segmentPos1BoxSpace)
    {
        float boxSpaceX = (boxFeature.signBits & 0x1) ? boxHalfWidths[0] : -boxHalfWidths[0];
        float boxSpaceY = (boxFeature.signBits & 0x2) ? boxHalfWidths[1] : -boxHalfWidths[1];
        float boxSpaceZ = (boxFeature.signBits & 0x4) ? boxHalfWidths[2] : -boxHalfWidths[2];
        if ((boxFeature.axisMask & 0x1) == 0)
        {
            FmVector3 edgeCenter = FmInitVector3(0.0f, boxSpaceY, boxSpaceZ);
            FmEdgeXSegmentClosestPointsBoxSpace(distResult, edgeCenter, boxHalfWidths[0], segmentPos0BoxSpace, segmentPos1BoxSpace);
        }
        else if ((boxFeature.axisMask & 0x2) == 0)
        {
            FmVector3 edgeCenter = FmInitVector3(boxSpaceX, 0.0f, boxSpaceZ);
            FmEdgeYSegmentClosestPointsBoxSpace(distResult, edgeCenter, boxHalfWidths[1], segmentPos0BoxSpace, segmentPos1BoxSpace);
        }
        else
        {
            FmVector3 edgeCenter = FmInitVector3(boxSpaceX, boxSpaceY, 0.0f);
            FmEdgeZSegmentClosestPointsBoxSpace(distResult, edgeCenter, boxHalfWidths[2], segmentPos0BoxSpace, segmentPos1BoxSpace);
        }
    }

    bool FmBoxAndTriIntersect(const FmBox& box, const FmVector3 triPos[3])
    {
        FmVector3 boxPos = box.state.pos;
        FmMatrix3 invBoxRot = FmInitMatrix3(conj(box.state.quat));

        // Transform triangle corners into space of box to simplify math
        FmVector3 triPosBoxSpace[3];
        triPosBoxSpace[0] = mul(invBoxRot, triPos[0] - boxPos);
        triPosBoxSpace[1] = mul(invBoxRot, triPos[1] - boxPos);
        triPosBoxSpace[2] = mul(invBoxRot, triPos[2] - boxPos);

        // Find normal of triangle
        FmVector3 vecEdge0 = triPosBoxSpace[1] - triPosBoxSpace[0];
        FmVector3 vecEdge1 = triPosBoxSpace[2] - triPosBoxSpace[1];
        FmVector3 vecEdge2 = triPosBoxSpace[0] - triPosBoxSpace[2];

        FmVector3 triCross = cross(vecEdge0, vecEdge1);

        float lenSqr;
        FmVector3 triNormal = FmNormalize(&lenSqr, triCross);

        bool triIsDegenerate = lenSqr < FM_NORMALIZE_MAG_SQR_TOL;

        // Check all separating axes and pick one with largest separation distance.
        FmVector3 separatingAxisBoxSpace;
        float separation, maxSeparation;
        bool posDirToTri;

        // Box face axes
        separatingAxisBoxSpace = FmInitVector3(1.0f, 0.0f, 0.0f);
        maxSeparation = FmBoxFaceTriSeparationBoxSpace(posDirToTri, 0, box.halfWidths[0], triPosBoxSpace);

        separatingAxisBoxSpace = FmInitVector3(0.0f, 1.0f, 0.0f);
        separation = FmBoxFaceTriSeparationBoxSpace(posDirToTri, 1, box.halfWidths[1], triPosBoxSpace);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }

        separatingAxisBoxSpace = FmInitVector3(0.0f, 0.0f, 1.0f);
        separation = FmBoxFaceTriSeparationBoxSpace(posDirToTri, 2, box.halfWidths[2], triPosBoxSpace);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }

        // Cross products of box and triangle edge directions
        separatingAxisBoxSpace = FmSafeNormalizeCrossX(vecEdge0);
        separation = FmBoxTriEdgeCrossSeparationBoxSpace(posDirToTri, separatingAxisBoxSpace, 1, box.halfWidths[1], 2, box.halfWidths[2],
            triPosBoxSpace[0], triPosBoxSpace[1], triPosBoxSpace[2]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossX(vecEdge1);
        separation = FmBoxTriEdgeCrossSeparationBoxSpace(posDirToTri, separatingAxisBoxSpace, 1, box.halfWidths[1], 2, box.halfWidths[2],
            triPosBoxSpace[1], triPosBoxSpace[2], triPosBoxSpace[0]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossX(vecEdge2);
        separation = FmBoxTriEdgeCrossSeparationBoxSpace(posDirToTri, separatingAxisBoxSpace, 1, box.halfWidths[1], 2, box.halfWidths[2],
            triPosBoxSpace[2], triPosBoxSpace[0], triPosBoxSpace[1]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossY(vecEdge0);
        separation = FmBoxTriEdgeCrossSeparationBoxSpace(posDirToTri, separatingAxisBoxSpace, 2, box.halfWidths[2], 0, box.halfWidths[0],
            triPosBoxSpace[0], triPosBoxSpace[1], triPosBoxSpace[2]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossY(vecEdge1);
        separation = FmBoxTriEdgeCrossSeparationBoxSpace(posDirToTri, separatingAxisBoxSpace, 2, box.halfWidths[2], 0, box.halfWidths[0],
            triPosBoxSpace[1], triPosBoxSpace[2], triPosBoxSpace[0]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossY(vecEdge2);
        separation = FmBoxTriEdgeCrossSeparationBoxSpace(posDirToTri, separatingAxisBoxSpace, 2, box.halfWidths[2], 0, box.halfWidths[0],
            triPosBoxSpace[2], triPosBoxSpace[0], triPosBoxSpace[1]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossZ(vecEdge0);
        separation = FmBoxTriEdgeCrossSeparationBoxSpace(posDirToTri, separatingAxisBoxSpace, 0, box.halfWidths[0], 1, box.halfWidths[1],
            triPosBoxSpace[0], triPosBoxSpace[1], triPosBoxSpace[2]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossZ(vecEdge1);
        separation = FmBoxTriEdgeCrossSeparationBoxSpace(posDirToTri, separatingAxisBoxSpace, 0, box.halfWidths[0], 1, box.halfWidths[1],
            triPosBoxSpace[1], triPosBoxSpace[2], triPosBoxSpace[0]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossZ(vecEdge2);
        separation = FmBoxTriEdgeCrossSeparationBoxSpace(posDirToTri, separatingAxisBoxSpace, 0, box.halfWidths[0], 1, box.halfWidths[1],
            triPosBoxSpace[2], triPosBoxSpace[0], triPosBoxSpace[1]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }

        // Triangle normal direction
        if (!triIsDegenerate)
        {
            separatingAxisBoxSpace = triNormal;
            separation = FmTriNormalBoxSeparationBoxSpace(posDirToTri, box.halfWidths, triPosBoxSpace[0], triNormal);
            if (separation > maxSeparation)
            {
                maxSeparation = separation;
            }
        }

        return maxSeparation <= 0.0f;
    }

    void FmBoxTriClosestPoints(FmDistanceResult* contact, const FmBox& box, const FmVector3 triPos[3])
    {
        FmVector3 boxPos = box.state.pos;
        FmMatrix3 boxRot = FmInitMatrix3(box.state.quat);

        // Transform triangle corners into space of box to simplify math
        FmMatrix3 invBoxRot = transpose(boxRot);
        FmVector3 triPosBoxSpace[3];
        triPosBoxSpace[0] = mul(invBoxRot, triPos[0] - boxPos);
        triPosBoxSpace[1] = mul(invBoxRot, triPos[1] - boxPos);
        triPosBoxSpace[2] = mul(invBoxRot, triPos[2] - boxPos);

        // Find normal of triangle
        FmVector3 vecEdge0 = triPosBoxSpace[1] - triPosBoxSpace[0];
        FmVector3 vecEdge1 = triPosBoxSpace[2] - triPosBoxSpace[1];
        FmVector3 vecEdge2 = triPosBoxSpace[0] - triPosBoxSpace[2];

        FmVector3 triCross = cross(vecEdge0, vecEdge1);

        float lenSqr;
        FmVector3 triNormal = FmNormalize(&lenSqr, triCross);

        bool triIsDegenerate = lenSqr < FM_NORMALIZE_MAG_SQR_TOL;

        // Check all separating axes and pick one with largest separation distance.
        FmVector3 separatingAxisBoxSpace, maxSeparatingAxisBoxSpace;
        float separation, maxSeparation;
        bool posDirToTri;

        // Box face axes
        separatingAxisBoxSpace = FmInitVector3(1.0f, 0.0f, 0.0f);
        maxSeparation = FmBoxFaceTriSeparationBoxSpace(posDirToTri, 0, box.halfWidths[0], triPosBoxSpace);
        maxSeparatingAxisBoxSpace = posDirToTri ? separatingAxisBoxSpace : -separatingAxisBoxSpace;

        separatingAxisBoxSpace = FmInitVector3(0.0f, 1.0f, 0.0f);
        separation = FmBoxFaceTriSeparationBoxSpace(posDirToTri, 1, box.halfWidths[1], triPosBoxSpace);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
            maxSeparatingAxisBoxSpace = posDirToTri ? separatingAxisBoxSpace : -separatingAxisBoxSpace;
        }

        separatingAxisBoxSpace = FmInitVector3(0.0f, 0.0f, 1.0f);
        separation = FmBoxFaceTriSeparationBoxSpace(posDirToTri, 2, box.halfWidths[2], triPosBoxSpace);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
            maxSeparatingAxisBoxSpace = posDirToTri ? separatingAxisBoxSpace : -separatingAxisBoxSpace;
        }

        // Cross products of box and triangle edge directions
        separatingAxisBoxSpace = FmSafeNormalizeCrossX(vecEdge0);
        separation = FmBoxTriEdgeCrossSeparationBoxSpace(posDirToTri, separatingAxisBoxSpace, 1, box.halfWidths[1], 2, box.halfWidths[2],
            triPosBoxSpace[0], triPosBoxSpace[1], triPosBoxSpace[2]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
            maxSeparatingAxisBoxSpace = posDirToTri ? separatingAxisBoxSpace : -separatingAxisBoxSpace;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossX(vecEdge1);
        separation = FmBoxTriEdgeCrossSeparationBoxSpace(posDirToTri, separatingAxisBoxSpace, 1, box.halfWidths[1], 2, box.halfWidths[2],
            triPosBoxSpace[1], triPosBoxSpace[2], triPosBoxSpace[0]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
            maxSeparatingAxisBoxSpace = posDirToTri ? separatingAxisBoxSpace : -separatingAxisBoxSpace;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossX(vecEdge2);
        separation = FmBoxTriEdgeCrossSeparationBoxSpace(posDirToTri, separatingAxisBoxSpace, 1, box.halfWidths[1], 2, box.halfWidths[2],
            triPosBoxSpace[2], triPosBoxSpace[0], triPosBoxSpace[1]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
            maxSeparatingAxisBoxSpace = posDirToTri ? separatingAxisBoxSpace : -separatingAxisBoxSpace;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossY(vecEdge0);
        separation = FmBoxTriEdgeCrossSeparationBoxSpace(posDirToTri, separatingAxisBoxSpace, 2, box.halfWidths[2], 0, box.halfWidths[0],
            triPosBoxSpace[0], triPosBoxSpace[1], triPosBoxSpace[2]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
            maxSeparatingAxisBoxSpace = posDirToTri ? separatingAxisBoxSpace : -separatingAxisBoxSpace;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossY(vecEdge1);
        separation = FmBoxTriEdgeCrossSeparationBoxSpace(posDirToTri, separatingAxisBoxSpace, 2, box.halfWidths[2], 0, box.halfWidths[0],
            triPosBoxSpace[1], triPosBoxSpace[2], triPosBoxSpace[0]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
            maxSeparatingAxisBoxSpace = posDirToTri ? separatingAxisBoxSpace : -separatingAxisBoxSpace;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossY(vecEdge2);
        separation = FmBoxTriEdgeCrossSeparationBoxSpace(posDirToTri, separatingAxisBoxSpace, 2, box.halfWidths[2], 0, box.halfWidths[0],
            triPosBoxSpace[2], triPosBoxSpace[0], triPosBoxSpace[1]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
            maxSeparatingAxisBoxSpace = posDirToTri ? separatingAxisBoxSpace : -separatingAxisBoxSpace;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossZ(vecEdge0);
        separation = FmBoxTriEdgeCrossSeparationBoxSpace(posDirToTri, separatingAxisBoxSpace, 0, box.halfWidths[0], 1, box.halfWidths[1],
            triPosBoxSpace[0], triPosBoxSpace[1], triPosBoxSpace[2]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
            maxSeparatingAxisBoxSpace = posDirToTri ? separatingAxisBoxSpace : -separatingAxisBoxSpace;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossZ(vecEdge1);
        separation = FmBoxTriEdgeCrossSeparationBoxSpace(posDirToTri, separatingAxisBoxSpace, 0, box.halfWidths[0], 1, box.halfWidths[1],
            triPosBoxSpace[1], triPosBoxSpace[2], triPosBoxSpace[0]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
            maxSeparatingAxisBoxSpace = posDirToTri ? separatingAxisBoxSpace : -separatingAxisBoxSpace;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossZ(vecEdge2);
        separation = FmBoxTriEdgeCrossSeparationBoxSpace(posDirToTri, separatingAxisBoxSpace, 0, box.halfWidths[0], 1, box.halfWidths[1],
            triPosBoxSpace[2], triPosBoxSpace[0], triPosBoxSpace[1]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
            maxSeparatingAxisBoxSpace = posDirToTri ? separatingAxisBoxSpace : -separatingAxisBoxSpace;
        }

        // Triangle normal direction
        if (!triIsDegenerate)
        {
            separatingAxisBoxSpace = triNormal;
            separation = FmTriNormalBoxSeparationBoxSpace(posDirToTri, box.halfWidths, triPosBoxSpace[0], triNormal);
            if (separation > maxSeparation)
            {
                maxSeparation = separation;
                maxSeparatingAxisBoxSpace = posDirToTri ? separatingAxisBoxSpace : -separatingAxisBoxSpace;
            }
        }

        // Find extremal features of box and triangle and compute their distance.
        FmBoxFeature boxFeature;
        FmFindExtremalBoxFeature(boxFeature, maxSeparatingAxisBoxSpace);

        FmTriFeature triFeature;
        FmFindExtremalTriFeature(triFeature, triPosBoxSpace, -maxSeparatingAxisBoxSpace);

        // If triangle intersecting box, translate to nonintersecting position to compute closest points
        FmVector3 separationOffsetBoxSpace = FmInitVector3(0.0f);
        if (maxSeparation <= 0.0f)
        {
            float boxRad = sqrtf(
                box.halfWidths[0] * box.halfWidths[0] +
                box.halfWidths[1] * box.halfWidths[1] +
                box.halfWidths[2] * box.halfWidths[2]);
            float offsetDistance = FmMaxFloat(-maxSeparation, 0.01f*boxRad);
            separationOffsetBoxSpace = maxSeparatingAxisBoxSpace * offsetDistance;
            triPosBoxSpace[0] += separationOffsetBoxSpace;
            triPosBoxSpace[1] += separationOffsetBoxSpace;
            triPosBoxSpace[2] += separationOffsetBoxSpace;
        }

        FmVector3 maxSeparatingAxis = mul(boxRot, maxSeparatingAxisBoxSpace);

        float boxSpaceX = (boxFeature.signBits & 0x1) ? box.halfWidths[0] : -box.halfWidths[0];
        float boxSpaceY = (boxFeature.signBits & 0x2) ? box.halfWidths[1] : -box.halfWidths[1];
        float boxSpaceZ = (boxFeature.signBits & 0x4) ? box.halfWidths[2] : -box.halfWidths[2];

        if (boxFeature.IsPoint()) // box point
        {
            FmVector3 boxSpacePoint = FmInitVector3(boxSpaceX, boxSpaceY, boxSpaceZ);
            FmVector3 boxPoint = boxPos + mul(boxRot, boxSpacePoint);

            if (triFeature.type == 0 || triFeature.type == 1) // tri point or edge
            {
                triFeature.type = 2;
            }

            FmPointTriangleDistance(contact, boxSpacePoint, triPosBoxSpace[0], triPosBoxSpace[1], triPosBoxSpace[2], 0, 0, 1, 2);

            FmVector3 direction = mul(boxRot, contact->direction);
            FmVector3 triPoint = boxPos + mul(boxRot, contact->posj - separationOffsetBoxSpace);

            contact->direction = (maxSeparation <= 0.0f) ? maxSeparatingAxis : direction;
            contact->distance = (maxSeparation <= 0.0f) ? maxSeparation : contact->distance;
            contact->posi = boxPoint;
            contact->posj = triPoint;
        }
        else if (boxFeature.IsEdge())
        {
            FmVector3 boxSpacePos0, boxSpacePos1;
            if ((boxFeature.axisMask & 0x1) == 0)
            {
                boxSpacePos0 = FmInitVector3(-boxSpaceX, boxSpaceY, boxSpaceZ);
                boxSpacePos1 = FmInitVector3(boxSpaceX, boxSpaceY, boxSpaceZ);
            }
            else if ((boxFeature.axisMask & 0x2) == 0)
            {
                boxSpacePos0 = FmInitVector3(boxSpaceX, -boxSpaceY, boxSpaceZ);
                boxSpacePos1 = FmInitVector3(boxSpaceX, boxSpaceY, boxSpaceZ);
            }
            else
            {
                boxSpacePos0 = FmInitVector3(boxSpaceX, boxSpaceY, -boxSpaceZ);
                boxSpacePos1 = FmInitVector3(boxSpaceX, boxSpaceY, boxSpaceZ);
            }

            if (triFeature.type == 0) // tri point
            {
                FmVector3 triPointBoxSpace = triPosBoxSpace[triFeature.id];

                FmPointSegmentDistance(contact, triPointBoxSpace, boxSpacePos0, boxSpacePos1, triFeature.id, 0, 1);

                FmVector3 direction = mul(boxRot, -contact->direction);
                FmVector3 boxPoint = boxPos + mul(boxRot, contact->posj);
                FmVector3 triPoint = boxPos + mul(boxRot, contact->posi - separationOffsetBoxSpace);

                contact->direction = (maxSeparation <= 0.0f) ? maxSeparatingAxis : direction;
                contact->distance = (maxSeparation <= 0.0f) ? maxSeparation : contact->distance;
                contact->posi = boxPoint;
                contact->posj = triPoint;
            }
            else if (triFeature.type == 1) // tri edge
            {
                uint id0 = triFeature.id;
                uint id1 = (triFeature.id + 1) % 3;
                FmVector3 triPointBoxSpace0 = triPosBoxSpace[id0];
                FmVector3 triPointBoxSpace1 = triPosBoxSpace[id1];

                FmSegmentPairDistance(contact, boxSpacePos0, boxSpacePos1, triPointBoxSpace0, triPointBoxSpace1, 0, 1, id0, id1);

                FmVector3 direction = mul(boxRot, contact->direction);
                FmVector3 boxPoint = boxPos + mul(boxRot, contact->posi);
                FmVector3 triPoint = boxPos + mul(boxRot, contact->posj - separationOffsetBoxSpace);

                contact->direction = (maxSeparation <= 0.0f) ? maxSeparatingAxis : direction;
                contact->distance = (maxSeparation <= 0.0f) ? maxSeparation : contact->distance;
                contact->posi = boxPoint;
                contact->posj = triPoint;
            }
            else // tri face
            {
                FmBoxEdgeTriClosestPointsBoxSpace(contact, boxFeature, box.halfWidths, triPosBoxSpace);

                FmVector3 direction = mul(boxRot, contact->direction);
                FmVector3 boxPoint = boxPos + mul(boxRot, contact->posi);
                FmVector3 triPoint = boxPos + mul(boxRot, contact->posj - separationOffsetBoxSpace);

                contact->direction = (maxSeparation <= 0.0f) ? maxSeparatingAxis : direction;
                contact->distance = (maxSeparation <= 0.0f) ? maxSeparation : contact->distance;
                contact->posi = boxPoint;
                contact->posj = triPoint;
            }
        }
        else // box face
        {
            FM_ASSERT(boxFeature.IsFace());

            uint boxFaceDim = (uint)((boxFeature.axisMask & 0x1) ? 0 : 1);
            boxFaceDim = (boxFeature.axisMask & 0x4) ? 2 : boxFaceDim;

            uint boxFaceSign = ((boxFeature.signBits & boxFeature.axisMask) >> boxFaceDim);

            if (triFeature.type == 0) // tri point
            {
                uint id = triFeature.id;

                FmBoxFacePointClosestPointsBoxSpace(contact, box.halfWidths, boxFaceDim, boxFaceSign, triPosBoxSpace[id], id);
            }
            else if (triFeature.type == 1) // tri edge
            {
                uint id0 = triFeature.id;
                uint id1 = (triFeature.id + 1) % 3;
                FmVector3 triPos0BoxSpace = triPosBoxSpace[id0];
                FmVector3 triPos1BoxSpace = triPosBoxSpace[id1];

                FmBoxFaceEdgeClosestPointsBoxSpace(contact, box.halfWidths, boxFaceDim, boxFaceSign, triPos0BoxSpace, triPos1BoxSpace, id0, id1);
            }
            else // tri face
            {
                FmBoxFaceTriClosestPointsBoxSpace(contact, box.halfWidths, boxFaceDim, boxFaceSign, triPosBoxSpace);
            }

            FmVector3 direction = mul(boxRot, contact->direction);
            FmVector3 boxPoint = boxPos + mul(boxRot, contact->posi);
            FmVector3 triPoint = boxPos + mul(boxRot, contact->posj - separationOffsetBoxSpace);

            contact->direction = (maxSeparation <= 0.0f) ? maxSeparatingAxis : direction;
            contact->distance = (maxSeparation <= 0.0f) ? maxSeparation : contact->distance;
            contact->posi = boxPoint;
            contact->posj = triPoint;
        }
    }

    static inline float FmMaxLinearSpeedFromRotation(const FmBox& box)
    {
        float hx = box.halfWidths[0];
        float hy = box.halfWidths[1];
        float hz = box.halfWidths[2];
#if 0
        // Cheaper but more conservative bound, not taking box orientation into account
        float boxRadius = sqrtf(hx*hx + hy * hy + hz * hz);
        return boxRadius * length(box.state.angvel);
#else
        // Max linear speed from rotation will be at one of the box corners.
        // Find linear velocity of each corner by cross product with angular velocity.
        // Do in box space, and use distributive property of cross product.
        FmVector3 angVelBoxSpace = rotate(conj(box.state.quat), box.state.angVel);
        FmVector3 linVelX = hx * FmCrossX(angVelBoxSpace);
        FmVector3 linVelY = hy * FmCrossY(angVelBoxSpace);
        FmVector3 linVelZ = hz * FmCrossZ(angVelBoxSpace);

        // Check four corners, same speed as other four by symmetry.
        float linSpeedSqr0 = lengthSqr(linVelX + linVelY + linVelZ);
        float linSpeedSqr1 = lengthSqr(linVelX - linVelY + linVelZ);
        float linSpeedSqr2 = lengthSqr(linVelX + linVelY - linVelZ);
        float linSpeedSqr3 = lengthSqr(linVelX - linVelY - linVelZ);
        float linSpeedSqr = FmMaxFloat(linSpeedSqr0, FmMaxFloat(linSpeedSqr1, FmMaxFloat(linSpeedSqr2, linSpeedSqr3)));

        return sqrtf(linSpeedSqr);
#endif
    }

    // Earliest time that boxes may reach gap distance assuming constant linear and angular velocity.
    // Using conservative advancement for rigid body as described in Brian Mirtich's Ph.D. thesis.
    static inline float FmBoxBoxEarliestImpact(
        const FmVector3& normal, // from boxA to boxB
        float distance,
        const FmRigidBodyState& boxAState, float boxAMaxLinearSpeedFromRotation,
        const FmRigidBodyState& boxBState, float boxBMaxLinearSpeedFromRotation,
        float gap)
    {
        float boxAVelProj = dot(boxAState.vel, normal);
        float boxBVelProj = dot(boxBState.vel, normal);

        // Get relative normal velocity, with max possible from rotation
        float dVel = boxAVelProj - boxBVelProj + boxAMaxLinearSpeedFromRotation + boxBMaxLinearSpeedFromRotation;

        // Make negative velocity 0, causing inf below
        dVel = FmMaxFloat(dVel, 0.0f);

        // Compute time to cross projected distances
        float dist = distance - gap;
        float dt = (dist <= 0.0f) ? 0.0f : dist / dVel;
        return dt;
    }

    // Earliest time that box and triangle may reach gap distance, assuming constant linear and angular velocity of box, and constant linear velocity of triangle vertices.
    // Using conservative advancement for rigid body as described in Brian Mirtich's Ph.D. thesis.
    static inline float FmBoxTriEarliestImpact(
        const FmVector3& normal, // from box to tri
        float distance,
        const FmRigidBodyState& boxState, float boxMaxLinearSpeedFromRotation,
        const FmTri& tri,
        float gap)
    {
        float boxVelProj = dot(boxState.vel, normal) + boxMaxLinearSpeedFromRotation;

        // Relative velocities between pairs of points projected on normal
        float triVel0Proj = dot(normal, tri.vel0);
        float triVel1Proj = dot(normal, tri.vel1);
        float triVel2Proj = dot(normal, tri.vel2);

        float triVelProj = FmMinFloat(triVel0Proj, FmMinFloat(triVel1Proj, triVel2Proj));

        float dVel = boxVelProj - triVelProj;

        // Make negative velocity 0, causing inf below
        dVel = FmMaxFloat(dVel, 0.0f);

        // Compute time to cross projected distances
        float dist = distance - gap;
        float dt = (dist <= 0.0f) ? 0.0f : dist / dVel;
        return dt;
    }

    static inline float FmBoxSegmentEarliestImpact(
        const FmVector3& normal, // from box to tri
        float distance,
        const FmRigidBodyState& boxState, float boxMaxLinearSpeedFromRotation,
        const FmVector3& pos0, const FmVector3& vel0,
        const FmVector3& pos1, const FmVector3& vel1,
        float gap)
    {
        (void)pos0;
        (void)pos1;
        float boxVelProj = dot(boxState.vel, normal) + boxMaxLinearSpeedFromRotation;

        // Relative velocities between pairs of points projected on normal
        float segmentVel0Proj = dot(normal, vel0);
        float segmentVel1Proj = dot(normal, vel1);

        float segmentVelProj = FmMinFloat(segmentVel0Proj, segmentVel1Proj);

        float dVel = boxVelProj - segmentVelProj;

        // Make negative velocity 0, causing inf below
        dVel = FmMaxFloat(dVel, 0.0f);

        // Compute time to cross projected distances
        float dist = distance - gap;
        float dt = (dist <= 0.0f) ? 0.0f : dist / dVel;
        return dt;
    }

    static inline float FmBoxPointEarliestImpact(
        const FmVector3& normal, // from box to tri
        float distance,
        const FmRigidBodyState& boxState, float boxMaxLinearSpeedFromRotation,
        const FmVector3& pos, const FmVector3& vel,
        float gap)
    {
        (void)pos;
        float boxVelProj = dot(boxState.vel, normal) + boxMaxLinearSpeedFromRotation;

        // Relative velocities between pairs of points projected on normal
        float pointVelProj = dot(normal, vel);

        float dVel = boxVelProj - pointVelProj;

        // Make negative velocity 0, causing inf below
        dVel = FmMaxFloat(dVel, 0.0f);

        // Compute time to cross projected distances
        float dist = distance - gap;
        float dt = (dist <= 0.0f) ? 0.0f : dist / dVel;
        return dt;
    }

    void FmBoxFacePointCcd(FmCcdResult* contact,
        const FmBox& box,
        uint boxFaceDim, uint boxFaceSign,
        const FmVector3& pointPos, const FmVector3& pointVel,
        uint j,
        float deltaTime, const FmCcdTerminationConditions& conditions)
    {
        float impactGapRelScale = conditions.impactGapRelError + 1.0f;

        float boxFaceNormalScale = boxFaceSign ? 1.0f : -1.0f;
        uint boxEdgeDim1 = (boxFaceDim + 1) % 3;
        uint boxEdgeDim2 = (boxFaceDim + 2) % 3;
        float boxEdgeWidth1 = box.halfWidths[boxEdgeDim1];
        float boxEdgeWidth2 = box.halfWidths[boxEdgeDim2];
        uint signBit0 = boxFaceSign << boxFaceDim;
        uint axisBit0 = 1UL << boxFaceDim;

        // Compute distance of box face to point
        FmDistanceResult distResult;

        FmMatrix3 boxRot = FmInitMatrix3(box.state.quat);
        FmVector3 boxFaceNormal = boxRot.getCol((int)boxFaceDim);
        FmVector3 boxEdgeDir1 = boxRot.getCol((int)boxEdgeDim1);
        FmVector3 boxEdgeDir2 = boxRot.getCol((int)boxEdgeDim2);
        FmVector3 boxFaceCenter = box.state.pos + boxFaceNormal * boxFaceNormalScale * box.halfWidths[boxFaceDim];

        FmBoxFacePointClosestPoints(&distResult, boxFaceCenter, boxFaceNormal,
            boxEdgeDir1, boxEdgeWidth1, boxEdgeDir2, boxEdgeWidth2, pointPos, j);

        // Record box feature in i0 and i1, as sign bits and axis mask.  
        // First extract bits from the face/point result.
        uint signBit1 = (distResult.featurePair.i0 & 0x1) << boxEdgeDim1;
        uint axisBit1 = (distResult.featurePair.i1 & 0x1) << boxEdgeDim1;
        uint signBit2 = (distResult.featurePair.i0 >> 1) << boxEdgeDim2;
        uint axisBit2 = (distResult.featurePair.i1 >> 1) << boxEdgeDim2;
        distResult.featurePair.i0 = signBit0 | signBit1 | signBit2;
        distResult.featurePair.i1 = axisBit0 | axisBit1 | axisBit2;

        // Initialize contact with closest points
        contact->direction = distResult.direction;
        contact->posi = distResult.posi;
        contact->posj = distResult.posj;
        contact->distance = distResult.distance;
        contact->time = 0.0f;
        contact->featurePair = distResult.featurePair;

        // If closer than the contact gap, return the feature pair.
        if (distResult.distance <= conditions.contactGap)
        {
            contact->retVal = FM_CCD_RET_INITIALLY_CONTACTING;
            return;
        }

        float boxMaxLinearSpeedFromRotation = FmMaxLinearSpeedFromRotation(box);

        // Initialize time of impact bound
        float dt = FmBoxPointEarliestImpact(distResult.direction, distResult.distance,
            box.state, boxMaxLinearSpeedFromRotation,
            pointPos, pointVel, conditions.impactGap);

        if (dt > deltaTime)
        {
            contact->time = deltaTime;
            contact->retVal = FM_CCD_RET_NO_IMPACT;
            return;
        }
        else if (dt <= conditions.minTimeStep)
        {
            contact->time = 0.0f;
            contact->retVal = FM_CCD_RET_MIN_TIME_STEP;
            return;
        }

        float t = dt;

        // Iteratively advance time, compute distance and next impact time bound, until it reaches impact distance or time bound is later than deltaTime
        uint iter = 0;
        while (iter++ < conditions.maxIterations)
        {
            contact->numIterations = iter;

            FmRigidBodyState boxStateAdv;
            boxStateAdv.pos = box.state.pos + box.state.vel * t;
            boxStateAdv.quat = FmIntegrateQuat(box.state.quat, box.state.angVel, t);
            boxStateAdv.vel = box.state.vel;
            boxStateAdv.angVel = box.state.angVel;

            FmVector3 pointPosAdv = pointPos + pointVel * t;
            FmVector3 pointVelAdv = pointVel;

            FmMatrix3 boxRotAdv = FmInitMatrix3(boxStateAdv.quat);
            FmVector3 boxFaceNormalAdv = boxRotAdv.getCol((int)boxFaceDim);
            FmVector3 boxEdgeDir1Adv = boxRotAdv.getCol((int)boxEdgeDim1);
            FmVector3 boxEdgeDir2Adv = boxRotAdv.getCol((int)boxEdgeDim2);
            FmVector3 boxFaceCenterAdv = boxStateAdv.pos + boxFaceNormalAdv * boxFaceNormalScale * box.halfWidths[boxFaceDim];

            FmBoxFacePointClosestPoints(&distResult, boxFaceCenterAdv, boxFaceNormalAdv,
                boxEdgeDir1Adv, boxEdgeWidth1, boxEdgeDir2Adv, boxEdgeWidth2, pointPosAdv, j);

            // Record box feature in i0 and i1, as sign bits and axis mask.  
            signBit1 = (distResult.featurePair.i0 & 0x1) << boxEdgeDim1;
            axisBit1 = (distResult.featurePair.i1 & 0x1) << boxEdgeDim1;
            signBit2 = (distResult.featurePair.i0 >> 1) << boxEdgeDim2;
            axisBit2 = (distResult.featurePair.i1 >> 1) << boxEdgeDim2;
            distResult.featurePair.i0 = signBit0 | signBit1 | signBit2;
            distResult.featurePair.i1 = axisBit0 | axisBit1 | axisBit2;

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

            dt = FmBoxPointEarliestImpact(distResult.direction, distResult.distance,
                boxStateAdv, boxMaxLinearSpeedFromRotation,
                pointPosAdv, pointVelAdv, conditions.impactGap);

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

    void FmBoxEdgeSegmentCcd(FmCcdResult* contact,
        const FmBox& box,
        const FmBoxFeature& boxFeature,
        const FmVector3& boxEdgePos0BoxSpace, const FmVector3& boxEdgePos1BoxSpace,
        const FmVector3& segmentPos0, const FmVector3& segmentVel0,
        const FmVector3& segmentPos1, const FmVector3& segmentVel1,
        uint j0, uint j1,
        float deltaTime, const FmCcdTerminationConditions& conditions)
    {
        FM_ASSERT(boxFeature.IsEdge());

        float impactGapRelScale = conditions.impactGapRelError + 1.0f;

        uint boxEdgeDim = boxFeature.dim;

        // Compute distance of box edge to segment
        FmDistanceResult distResult;

        FmMatrix3 boxRot = FmInitMatrix3(box.state.quat);
        FmVector3 boxEdgePos0 = box.state.pos + mul(boxRot, boxEdgePos0BoxSpace);
        FmVector3 boxEdgePos1 = box.state.pos + mul(boxRot, boxEdgePos1BoxSpace);

        FmSegmentPairDistance(&distResult, boxEdgePos0, boxEdgePos1, segmentPos0, segmentPos1, 0, 1, j0, j1);

        // Record box feature in i0 and i1, as sign bits and axis mask.
        uint signBit = distResult.featurePair.i0 << boxEdgeDim;
        uint axisBit = ((uint)(distResult.featurePair.itype == FM_FEATURE_TYPE_VERTEX)) << boxEdgeDim;
        uint clearBit = 1UL << boxEdgeDim;
        distResult.featurePair.i0 = (uint)((boxFeature.signBits & ~clearBit) | signBit);
        distResult.featurePair.i1 = (uint)((boxFeature.axisMask & ~clearBit) | axisBit);

        // Initialize contact with closest points
        contact->direction = distResult.direction;
        contact->posi = distResult.posi;
        contact->posj = distResult.posj;
        contact->distance = distResult.distance;
        contact->time = 0.0f;
        contact->featurePair = distResult.featurePair;

        // Return if distance is less than contact gap
        if (distResult.distance <= conditions.contactGap)
        {
            contact->retVal = FM_CCD_RET_INITIALLY_CONTACTING;
            return;
        }

        float boxMaxLinearSpeedFromRotation = FmMaxLinearSpeedFromRotation(box);

        // Initialize time of impact bound
        float dt = FmBoxSegmentEarliestImpact(distResult.direction, distResult.distance,
            box.state, boxMaxLinearSpeedFromRotation,
            segmentPos0, segmentVel0, segmentPos1, segmentVel1, conditions.impactGap);

        if (dt > deltaTime)
        {
            contact->time = deltaTime;
            contact->retVal = FM_CCD_RET_NO_IMPACT;
            return;
        }
        else if (dt <= conditions.minTimeStep)
        {
            contact->time = 0.0f;
            contact->retVal = FM_CCD_RET_MIN_TIME_STEP;
            return;
        }

        float t = dt;

        // Iteratively advance time, compute distance and next impact time bound, until it reaches impact distance or time bound is later than deltaTime
        uint iter = 0;
        while (iter++ < conditions.maxIterations)
        {
            contact->numIterations = iter;

            FmRigidBodyState boxStateAdv;
            boxStateAdv.pos = box.state.pos + box.state.vel * t;
            boxStateAdv.quat = FmIntegrateQuat(box.state.quat, box.state.angVel, t);
            boxStateAdv.vel = box.state.vel;
            boxStateAdv.angVel = box.state.angVel;

            FmVector3 segmentPos0Adv = segmentPos0 + segmentVel0 * t;
            FmVector3 segmentVel0Adv = segmentVel0;
            FmVector3 segmentPos1Adv = segmentPos1 + segmentVel1 * t;
            FmVector3 segmentVel1Adv = segmentVel1;

            FmMatrix3 boxRotAdv = FmInitMatrix3(boxStateAdv.quat);
            FmVector3 boxEdgePos0Adv = boxStateAdv.pos + mul(boxRotAdv, boxEdgePos0BoxSpace);
            FmVector3 boxEdgePos1Adv = boxStateAdv.pos + mul(boxRotAdv, boxEdgePos1BoxSpace);

            FmSegmentPairDistance(&distResult, boxEdgePos0Adv, boxEdgePos1Adv, segmentPos0Adv, segmentPos1Adv, 0, 1, j0, j1);

            // Record box feature in i0 and i1, as sign bits and axis mask.
            signBit = distResult.featurePair.i0 << boxEdgeDim;
            axisBit = ((uint)(distResult.featurePair.itype == FM_FEATURE_TYPE_VERTEX)) << boxEdgeDim;
            clearBit = 1UL << boxEdgeDim;
            distResult.featurePair.i0 = (boxFeature.signBits & ~clearBit) | signBit;
            distResult.featurePair.i1 = (boxFeature.axisMask & ~clearBit) | axisBit;

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

            dt = FmBoxSegmentEarliestImpact(distResult.direction, distResult.distance, boxStateAdv, boxMaxLinearSpeedFromRotation,
                segmentPos0Adv, segmentVel0Adv, segmentPos1Adv, segmentVel1Adv, conditions.impactGap);

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

    void FmBoxVertexTriCcd(FmCcdResult* contact,
        const FmBox& box, uint boxVertexId, const FmVector3& boxVertexPosBoxSpace, const FmTri& tri,
        float deltaTime, const FmCcdTerminationConditions& conditions)
    {
        float impactGapRelScale = conditions.impactGapRelError + 1.0f;

        FmDistanceResult distResult;

        FmMatrix3 boxRot = FmInitMatrix3(box.state.quat);
        FmVector3 boxPoint = box.state.pos + mul(boxRot, boxVertexPosBoxSpace);

        FmPointTriangleDistance(&distResult, boxPoint, tri.pos0, tri.pos1, tri.pos2, boxVertexId, 0, 1, 2);

        distResult.featurePair.i1 = 0x7; // i0 already sign bits, i1 is axis mask

        // Initialize contact with closest points
        contact->direction = distResult.direction;
        contact->posi = distResult.posi;
        contact->posj = distResult.posj;
        contact->distance = distResult.distance;
        contact->time = 0.0f;
        contact->featurePair = distResult.featurePair;

        // Return if distance is less than contact gap
        if (distResult.distance <= conditions.contactGap)
        {
            contact->retVal = FM_CCD_RET_INITIALLY_CONTACTING;
            return;
        }

        float boxMaxLinearSpeedFromRotation = FmMaxLinearSpeedFromRotation(box);

        // Initialize time of impact bound
        float dt = FmBoxTriEarliestImpact(distResult.direction, distResult.distance,
            box.state, boxMaxLinearSpeedFromRotation,
            tri, conditions.impactGap);

        if (dt > deltaTime)
        {
            contact->time = deltaTime;
            contact->retVal = FM_CCD_RET_NO_IMPACT;
            return;
        }
        else if (dt <= conditions.minTimeStep)
        {
            contact->time = 0.0f;
            contact->retVal = FM_CCD_RET_MIN_TIME_STEP;
            return;
        }

        float t = dt;

        // Iteratively advance time, compute distance and next impact time bound, until it reaches impact distance or time bound is later than deltaTime
        uint iter = 0;
        while (iter++ < conditions.maxIterations)
        {
            contact->numIterations = iter;

            FmRigidBodyState boxStateAdv;
            boxStateAdv.pos = box.state.pos + box.state.vel * t;
            boxStateAdv.quat = FmIntegrateQuat(box.state.quat, box.state.angVel, t);
            boxStateAdv.vel = box.state.vel;
            boxStateAdv.angVel = box.state.angVel;

            FmTri triAdv;
            triAdv.pos0 = tri.pos0 + tri.vel0 * t;
            triAdv.pos1 = tri.pos1 + tri.vel1 * t;
            triAdv.pos2 = tri.pos2 + tri.vel2 * t;
            triAdv.vel0 = tri.vel0;
            triAdv.vel1 = tri.vel1;
            triAdv.vel2 = tri.vel2;

            FmMatrix3 boxRotAdv = FmInitMatrix3(boxStateAdv.quat);
            FmVector3 boxPointAdv = boxStateAdv.pos + mul(boxRotAdv, boxVertexPosBoxSpace);

            FmPointTriangleDistance(&distResult, boxPointAdv, triAdv.pos0, triAdv.pos1, triAdv.pos2, boxVertexId, 0, 1, 2);

            distResult.featurePair.i1 = 0x7; // i0 already sign bits, i1 is axis mask

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

            dt = FmBoxTriEarliestImpact(distResult.direction, distResult.distance,
                boxStateAdv, boxMaxLinearSpeedFromRotation,
                triAdv, conditions.impactGap);

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

    void FmBoxFaceBoxVertexCcd(FmCcdResult* contact,
        const FmBox& boxA, uint boxAFaceDim, uint boxAFaceSign,
        const FmBox & boxB, uint boxBVertexId, const FmVector3& boxBVertexPosBoxSpace,
        float deltaTime, const FmCcdTerminationConditions& conditions)
    {
        float impactGapRelScale = conditions.impactGapRelError + 1.0f;

        float boxAFaceNormalScale = boxAFaceSign ? 1.0f : -1.0f;
        uint boxAEdgeDim1 = (boxAFaceDim + 1) % 3;
        uint boxAEdgeDim2 = (boxAFaceDim + 2) % 3;
        float boxAEdgeWidth1 = boxA.halfWidths[boxAEdgeDim1];
        float boxAEdgeWidth2 = boxA.halfWidths[boxAEdgeDim2];
        uint boxASignBit0 = boxAFaceSign << boxAFaceDim;
        uint boxAAxisBit0 = 1UL << boxAFaceDim;

        // Compute distance of box face to point
        FmDistanceResult distResult;

        FmMatrix3 boxARot = FmInitMatrix3(boxA.state.quat);
        FmVector3 boxAFaceNormal = boxARot.getCol((int)boxAFaceDim);
        FmVector3 boxAEdgeDir1 = boxARot.getCol((int)boxAEdgeDim1);
        FmVector3 boxAEdgeDir2 = boxARot.getCol((int)boxAEdgeDim2);
        FmVector3 boxAFaceCenter = boxA.state.pos + boxAFaceNormal * boxAFaceNormalScale * boxA.halfWidths[boxAFaceDim];

        FmVector3 boxBVertexPos = boxB.state.pos + rotate(boxB.state.quat, boxBVertexPosBoxSpace);

        FmBoxFacePointClosestPoints(&distResult,
            boxAFaceCenter, boxAFaceNormal, boxAEdgeDir1, boxAEdgeWidth1, boxAEdgeDir2, boxAEdgeWidth2,
            boxBVertexPos, boxBVertexId);

        // Record box feature in i0 and i1, as sign bits and axis mask.  
        // First extract bits from the face/point result.
        uint boxASignBit1 = (distResult.featurePair.i0 & 0x1) << boxAEdgeDim1;
        uint boxAAxisBit1 = (distResult.featurePair.i1 & 0x1) << boxAEdgeDim1;
        uint boxASignBit2 = (distResult.featurePair.i0 >> 1) << boxAEdgeDim2;
        uint boxAAxisBit2 = (distResult.featurePair.i1 >> 1) << boxAEdgeDim2;
        distResult.featurePair.i0 = boxASignBit0 | boxASignBit1 | boxASignBit2;
        distResult.featurePair.i1 = boxAAxisBit0 | boxAAxisBit1 | boxAAxisBit2;
        distResult.featurePair.j1 = 0x7; // j0 already sign bits, j1 is axis mask

        // Initialize contact with closest points
        contact->direction = distResult.direction;
        contact->posi = distResult.posi;
        contact->posj = distResult.posj;
        contact->distance = distResult.distance;
        contact->time = 0.0f;
        contact->featurePair = distResult.featurePair;

        // If closer than the contact gap, return the feature pair.
        if (distResult.distance <= conditions.contactGap)
        {
            contact->retVal = FM_CCD_RET_INITIALLY_CONTACTING;
            return;
        }

        float boxAMaxLinearSpeedFromRotation = FmMaxLinearSpeedFromRotation(boxA);
        float boxBMaxLinearSpeedFromRotation = FmMaxLinearSpeedFromRotation(boxB);

        // Initialize time of impact bound
        float dt = FmBoxBoxEarliestImpact(distResult.direction, distResult.distance,
            boxA.state, boxAMaxLinearSpeedFromRotation,
            boxB.state, boxBMaxLinearSpeedFromRotation,
            conditions.impactGap);

        if (dt > deltaTime)
        {
            contact->time = deltaTime;
            contact->retVal = FM_CCD_RET_NO_IMPACT;
            return;
        }
        else if (dt <= conditions.minTimeStep)
        {
            contact->time = 0.0f;
            contact->retVal = FM_CCD_RET_MIN_TIME_STEP;
            return;
        }

        float t = dt;

        // Iteratively advance time, compute distance and next impact time bound, until it reaches impact distance or time bound is later than deltaTime
        uint iter = 0;
        while (iter++ < conditions.maxIterations)
        {
            contact->numIterations = iter;

            FmRigidBodyState boxAStateAdv;
            boxAStateAdv.pos = boxA.state.pos + boxA.state.vel * t;
            boxAStateAdv.quat = FmIntegrateQuat(boxA.state.quat, boxA.state.angVel, t);
            boxAStateAdv.vel = boxA.state.vel;
            boxAStateAdv.angVel = boxA.state.angVel;

            FmRigidBodyState boxBStateAdv;
            boxBStateAdv.pos = boxB.state.pos + boxB.state.vel * t;
            boxBStateAdv.quat = FmIntegrateQuat(boxB.state.quat, boxB.state.angVel, t);
            boxBStateAdv.vel = boxB.state.vel;
            boxBStateAdv.angVel = boxB.state.angVel;

            FmMatrix3 boxARotAdv = FmInitMatrix3(boxAStateAdv.quat);
            FmVector3 boxAFaceNormalAdv = boxARotAdv.getCol((int)boxAFaceDim);
            FmVector3 boxAEdgeDir1Adv = boxARotAdv.getCol((int)boxAEdgeDim1);
            FmVector3 boxAEdgeDir2Adv = boxARotAdv.getCol((int)boxAEdgeDim2);
            FmVector3 boxAFaceCenterAdv = boxAStateAdv.pos + boxAFaceNormalAdv * boxAFaceNormalScale * boxA.halfWidths[boxAFaceDim];

            FmVector3 boxBVertexPosAdv = boxBStateAdv.pos + rotate(boxBStateAdv.quat, boxBVertexPosBoxSpace);

            FmBoxFacePointClosestPoints(&distResult, boxAFaceCenterAdv, boxAFaceNormalAdv,
                boxAEdgeDir1Adv, boxAEdgeWidth1, boxAEdgeDir2Adv, boxAEdgeWidth2, boxBVertexPosAdv, boxBVertexId);

            // Record box feature in i0 and i1, as sign bits and axis mask.  
            boxASignBit1 = (distResult.featurePair.i0 & 0x1) << boxAEdgeDim1;
            boxAAxisBit1 = (distResult.featurePair.i1 & 0x1) << boxAEdgeDim1;
            boxASignBit2 = (distResult.featurePair.i0 >> 1) << boxAEdgeDim2;
            boxAAxisBit2 = (distResult.featurePair.i1 >> 1) << boxAEdgeDim2;
            distResult.featurePair.i0 = boxASignBit0 | boxASignBit1 | boxASignBit2;
            distResult.featurePair.i1 = boxAAxisBit0 | boxAAxisBit1 | boxAAxisBit2;
            distResult.featurePair.j1 = 0x7; // j0 already sign bits, j1 is axis mask

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

            dt = FmBoxBoxEarliestImpact(distResult.direction, distResult.distance,
                boxAStateAdv, boxAMaxLinearSpeedFromRotation,
                boxBStateAdv, boxBMaxLinearSpeedFromRotation,
                conditions.impactGap);

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

    void FmBoxVertexBoxFaceCcd(FmCcdResult* contact,
        const FmBox& boxA, uint boxAVertexId, const FmVector3& boxAVertexPosBoxSpace,
        const FmBox& boxB, uint boxBFaceDim, uint boxBFaceSign,
        float deltaTime, const FmCcdTerminationConditions& conditions)
    {
        FmCcdResult faceVertexResult;
        FmBoxFaceBoxVertexCcd(&faceVertexResult,
            boxB, boxBFaceDim, boxBFaceSign,
            boxA, boxAVertexId, boxAVertexPosBoxSpace, deltaTime, conditions);

        contact->direction = -faceVertexResult.direction;
        contact->posi = faceVertexResult.posj;
        contact->posj = faceVertexResult.posi;
        contact->distance = faceVertexResult.distance;
        contact->time = faceVertexResult.time;
        contact->numIterations = faceVertexResult.numIterations;
        contact->featurePair = FmReverseFeaturePair(faceVertexResult.featurePair);
        contact->retVal = faceVertexResult.retVal;
    }

    void FmBoxEdgeBoxEdgeCcd(FmCcdResult* contact,
        const FmBox& boxA, const FmBoxFeature& boxAFeature, const FmVector3& boxAEdgePos0BoxSpace, const FmVector3& boxAEdgePos1BoxSpace,
        const FmBox& boxB, const FmBoxFeature& boxBFeature, const FmVector3& boxBEdgePos0BoxSpace, const FmVector3& boxBEdgePos1BoxSpace,
        float deltaTime, const FmCcdTerminationConditions& conditions)
    {
        FM_ASSERT(boxAFeature.IsEdge());
        FM_ASSERT(boxBFeature.IsEdge());

        float impactGapRelScale = conditions.impactGapRelError + 1.0f;

        uint boxAEdgeDim = boxAFeature.dim;
        uint boxBEdgeDim = boxBFeature.dim;

        // Compute distance of box edge to segment
        FmDistanceResult distResult;

        FmMatrix3 boxARot = FmInitMatrix3(boxA.state.quat);
        FmVector3 boxAEdgePos0 = boxA.state.pos + mul(boxARot, boxAEdgePos0BoxSpace);
        FmVector3 boxAEdgePos1 = boxA.state.pos + mul(boxARot, boxAEdgePos1BoxSpace);

        FmMatrix3 boxBRot = FmInitMatrix3(boxB.state.quat);
        FmVector3 boxBEdgePos0 = boxB.state.pos + mul(boxBRot, boxBEdgePos0BoxSpace);
        FmVector3 boxBEdgePos1 = boxB.state.pos + mul(boxBRot, boxBEdgePos1BoxSpace);

        FmSegmentPairDistance(&distResult, boxAEdgePos0, boxAEdgePos1, boxBEdgePos0, boxBEdgePos1, 0, 1, 0, 1);

        // Record box feature in i0 and i1, as sign bits and axis mask.
        uint boxASignBit = distResult.featurePair.i0 << boxAEdgeDim;
        uint boxAAxisBit = ((uint)(distResult.featurePair.itype == FM_FEATURE_TYPE_VERTEX)) << boxAEdgeDim;
        uint boxAClearBit = 1UL << boxAEdgeDim;
        distResult.featurePair.i0 = (uint)((boxAFeature.signBits & ~boxAClearBit) | boxASignBit);
        distResult.featurePair.i1 = (uint)((boxAFeature.axisMask & ~boxAClearBit) | boxAAxisBit);

        uint boxBSignBit = distResult.featurePair.j0 << boxBEdgeDim;
        uint boxBAxisBit = ((uint)(distResult.featurePair.jtype == FM_FEATURE_TYPE_VERTEX)) << boxBEdgeDim;
        uint boxBClearBit = 1UL << boxBEdgeDim;
        distResult.featurePair.j0 = (boxBFeature.signBits & ~boxBClearBit) | boxBSignBit;
        distResult.featurePair.j1 = (boxBFeature.axisMask & ~boxBClearBit) | boxBAxisBit;

        // Initialize contact with closest points
        contact->direction = distResult.direction;
        contact->posi = distResult.posi;
        contact->posj = distResult.posj;
        contact->distance = distResult.distance;
        contact->time = 0.0f;
        contact->featurePair = distResult.featurePair;

        // Return if distance is less than contact gap
        if (distResult.distance <= conditions.contactGap)
        {
            contact->retVal = FM_CCD_RET_INITIALLY_CONTACTING;
            return;
        }

        float boxAMaxLinearSpeedFromRotation = FmMaxLinearSpeedFromRotation(boxA);
        float boxBMaxLinearSpeedFromRotation = FmMaxLinearSpeedFromRotation(boxB);

        // Initialize time of impact bound
        float dt = FmBoxBoxEarliestImpact(distResult.direction, distResult.distance,
            boxA.state, boxAMaxLinearSpeedFromRotation,
            boxB.state, boxBMaxLinearSpeedFromRotation,
            conditions.impactGap);

        if (dt > deltaTime)
        {
            contact->time = deltaTime;
            contact->retVal = FM_CCD_RET_NO_IMPACT;
            return;
        }
        else if (dt <= conditions.minTimeStep)
        {
            contact->time = 0.0f;
            contact->retVal = FM_CCD_RET_MIN_TIME_STEP;
            return;
        }

        float t = dt;

        // Iteratively advance time, compute distance and next impact time bound, until it reaches impact distance or time bound is later than deltaTime
        uint iter = 0;
        while (iter++ < conditions.maxIterations)
        {
            contact->numIterations = iter;

            FmRigidBodyState boxAStateAdv;
            boxAStateAdv.pos = boxA.state.pos + boxA.state.vel * t;
            boxAStateAdv.quat = FmIntegrateQuat(boxA.state.quat, boxA.state.angVel, t);
            boxAStateAdv.vel = boxA.state.vel;
            boxAStateAdv.angVel = boxA.state.angVel;

            FmRigidBodyState boxBStateAdv;
            boxBStateAdv.pos = boxB.state.pos + boxB.state.vel * t;
            boxBStateAdv.quat = FmIntegrateQuat(boxB.state.quat, boxB.state.angVel, t);
            boxBStateAdv.vel = boxB.state.vel;
            boxBStateAdv.angVel = boxB.state.angVel;

            FmMatrix3 boxARotAdv = FmInitMatrix3(boxAStateAdv.quat);
            FmVector3 boxAEdgePos0Adv = boxAStateAdv.pos + mul(boxARotAdv, boxAEdgePos0BoxSpace);
            FmVector3 boxAEdgePos1Adv = boxAStateAdv.pos + mul(boxARotAdv, boxAEdgePos1BoxSpace);

            FmMatrix3 boxBRotAdv = FmInitMatrix3(boxBStateAdv.quat);
            FmVector3 boxBEdgePos0Adv = boxBStateAdv.pos + mul(boxBRotAdv, boxBEdgePos0BoxSpace);
            FmVector3 boxBEdgePos1Adv = boxBStateAdv.pos + mul(boxBRotAdv, boxBEdgePos1BoxSpace);

            FmSegmentPairDistance(&distResult, boxAEdgePos0Adv, boxAEdgePos1Adv, boxBEdgePos0Adv, boxBEdgePos1Adv, 0, 1, 0, 1);

            // Record box feature in i0 and i1, as sign bits and axis mask.
            boxASignBit = distResult.featurePair.i0 << boxAEdgeDim;
            boxAAxisBit = ((uint)(distResult.featurePair.itype == FM_FEATURE_TYPE_VERTEX)) << boxAEdgeDim;
            boxAClearBit = 1UL << boxAEdgeDim;
            distResult.featurePair.i0 = (uint)((boxAFeature.signBits & ~boxAClearBit) | boxASignBit);
            distResult.featurePair.i1 = (uint)((boxAFeature.axisMask & ~boxAClearBit) | boxAAxisBit);

            boxBSignBit = distResult.featurePair.j0 << boxBEdgeDim;
            boxBAxisBit = ((uint)(distResult.featurePair.jtype == FM_FEATURE_TYPE_VERTEX)) << boxBEdgeDim;
            boxBClearBit = 1UL << boxBEdgeDim;
            distResult.featurePair.j0 = (uint)((boxBFeature.signBits & ~boxBClearBit) | boxBSignBit);
            distResult.featurePair.j1 = (uint)((boxBFeature.axisMask & ~boxBClearBit) | boxBAxisBit);

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

            dt = FmBoxBoxEarliestImpact(distResult.direction, distResult.distance,
                boxAStateAdv, boxAMaxLinearSpeedFromRotation,
                boxBStateAdv, boxBMaxLinearSpeedFromRotation,
                conditions.impactGap);

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

    FmAabb FmComputeBoxAabb(const FmBox& box, float deltaTime, float padding)
    {
        (void)deltaTime;

        // Compute FmAabb bounding the oriented box for the entire step.
        // For simplicity using radius of enclosing sphere. TODO: tighter fit.

        float radius = sqrtf(
            box.halfWidths[0] * box.halfWidths[0] +
            box.halfWidths[1] * box.halfWidths[1] +
            box.halfWidths[2] * box.halfWidths[2]) + padding;

        FmVector3 vradius = FmInitVector3(radius);

        FmAabb aabb;
        aabb.pmin = box.state.pos - vradius;
        aabb.pmax = box.state.pos + vradius;
        aabb.vmin = box.state.vel;
        aabb.vmax = box.state.vel;
        return aabb;
    }

    // Find separation distance between boxes along box A face axis.
    // Assumes box B transformed into box A coordinate system.
    static inline float FmBoxFaceBoxSeparationBoxSpace(
        bool& posDirectionToBoxB, uint boxAFaceDim, float boxAHalfWidth,
        const FmVector3& boxBPosBoxASpace,
        const FmVector3& boxBAxis0BoxASpace, float boxBHalfWidthX,
        const FmVector3& boxBAxis1BoxASpace, float boxBHalfWidthY,
        const FmVector3& boxBAxis2BoxASpace, float boxBHalfWidthZ)
    {
        float boxBProj = boxBPosBoxASpace.getElem((int)boxAFaceDim);
        float boxBAxis0AbsProj = boxBHalfWidthX * fabsf(boxBAxis0BoxASpace.getElem((int)boxAFaceDim));
        float boxBAxis1AbsProj = boxBHalfWidthY * fabsf(boxBAxis1BoxASpace.getElem((int)boxAFaceDim));
        float boxBAxis2AbsProj = boxBHalfWidthZ * fabsf(boxBAxis2BoxASpace.getElem((int)boxAFaceDim));

        // Find min and max projections of box B relative to box A center
        float boxBMinProj, boxBMaxProj;
        boxBMinProj = boxBProj - boxBAxis0AbsProj - boxBAxis1AbsProj - boxBAxis2AbsProj;
        boxBMaxProj = boxBProj + boxBAxis0AbsProj + boxBAxis1AbsProj + boxBAxis2AbsProj;

        // Compute maximum separation.  If box and tri overlap on this axis, result will be negative.
        float negSideSep = -boxAHalfWidth - boxBMaxProj;
        float posSideSep = boxBMinProj - boxAHalfWidth;

        bool posSide = (posSideSep > negSideSep);
        posDirectionToBoxB = posSide;
        return posSide ? posSideSep : negSideSep;
    }

    // Find separation distance between boxes along edge cross product direction.
    // Assumes box B transformed into box A coordinate system.
    static inline float FmBoxEdgeBoxEdgeCrossSeparationBoxSpace(
        bool& posDirectionToBoxB,
        const FmVector3& edgeCrossDir,
        uint boxAEdgeDim1, float boxAHalfWidth1, uint boxAEdgeDim2, float boxAHalfWidth2,
        const FmVector3& boxBPosBoxASpace,
        const FmVector3& boxBEdge1BoxASpace, float boxBHalfWidth1,
        const FmVector3& boxBEdge2BoxASpace, float boxBHalfWidth2)
    {
        float boxAEdge1Component = edgeCrossDir.getElem((int)boxAEdgeDim1);
        float boxAEdge2Component = edgeCrossDir.getElem((int)boxAEdgeDim2);

        float boxAPosProj = 0.0f;
        float boxAAxis1AbsProj = boxAHalfWidth1 * fabsf(boxAEdge1Component);
        float boxAAxis2AbsProj = boxAHalfWidth2 * fabsf(boxAEdge2Component);

        float boxBPosProj = dot(boxBPosBoxASpace, edgeCrossDir);
        float boxBAxis1AbsProj = boxBHalfWidth1 * fabsf(dot(boxBEdge1BoxASpace, edgeCrossDir));
        float boxBAxis2AbsProj = boxBHalfWidth2 * fabsf(dot(boxBEdge2BoxASpace, edgeCrossDir));

        float boxAMinProj = boxAPosProj - boxAAxis1AbsProj - boxAAxis2AbsProj;
        float boxAMaxProj = boxAPosProj + boxAAxis1AbsProj + boxAAxis2AbsProj;

        float boxBMinProj = boxBPosProj - boxBAxis1AbsProj - boxBAxis2AbsProj;
        float boxBMaxProj = boxBPosProj + boxBAxis1AbsProj + boxBAxis2AbsProj;

        float negSideSep = boxAMinProj - boxBMaxProj;
        float posSideSep = boxBMinProj - boxAMaxProj;

        bool posSide = (posSideSep > negSideSep);
        posDirectionToBoxB = posSide;
        return posSide ? posSideSep : negSideSep;
    }

    bool FmBoxesIntersect(const FmBox& boxA, const FmBox& boxB)
    {
        FmVector3 boxAPos = boxA.state.pos;
        FmVector3 boxBPos = boxB.state.pos;
        FmMatrix3 boxARot = FmInitMatrix3(boxA.state.quat);
        FmMatrix3 boxBRot = FmInitMatrix3(boxB.state.quat);
        FmMatrix3 invBoxARot = transpose(boxARot);
        FmMatrix3 invBoxBRot = transpose(boxBRot);

        // Get box poses in space of other box
        FmVector3 boxBPosInA = mul(invBoxARot, boxBPos - boxAPos);
        FmMatrix3 boxBRotInA = mul(invBoxARot, boxBRot);

        FmVector3 boxAPosInB = mul(invBoxBRot, boxAPos - boxBPos);
        FmMatrix3 boxARotInB = mul(invBoxBRot, boxARot);

        // Check all separating axes and pick one with largest separation distance.
        FmVector3 separatingAxisBoxSpace;
        float separation, maxSeparation;
        bool posDirToBoxB, posDirToBoxA;

        // Box A face axes
        separatingAxisBoxSpace = FmInitVector3(1.0f, 0.0f, 0.0f);
        maxSeparation = FmBoxFaceBoxSeparationBoxSpace(posDirToBoxB, 0, boxA.halfWidths[0], boxBPosInA, boxBRotInA.col0, boxB.halfWidths[0], boxBRotInA.col1, boxB.halfWidths[1], boxBRotInA.col2, boxB.halfWidths[2]);

        separatingAxisBoxSpace = FmInitVector3(0.0f, 1.0f, 0.0f);
        separation = FmBoxFaceBoxSeparationBoxSpace(posDirToBoxB, 1, boxA.halfWidths[1], boxBPosInA, boxBRotInA.col0, boxB.halfWidths[0], boxBRotInA.col1, boxB.halfWidths[1], boxBRotInA.col2, boxB.halfWidths[2]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }

        separatingAxisBoxSpace = FmInitVector3(0.0f, 0.0f, 1.0f);
        separation = FmBoxFaceBoxSeparationBoxSpace(posDirToBoxB, 2, boxA.halfWidths[2], boxBPosInA, boxBRotInA.col0, boxB.halfWidths[0], boxBRotInA.col1, boxB.halfWidths[1], boxBRotInA.col2, boxB.halfWidths[2]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }

        // Box B face axes
        separatingAxisBoxSpace = FmInitVector3(1.0f, 0.0f, 0.0f);
        separation = FmBoxFaceBoxSeparationBoxSpace(posDirToBoxA, 0, boxB.halfWidths[0], boxAPosInB, boxARotInB.col0, boxA.halfWidths[0], boxARotInB.col1, boxA.halfWidths[1], boxARotInB.col2, boxA.halfWidths[2]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }

        separatingAxisBoxSpace = FmInitVector3(0.0f, 1.0f, 0.0f);
        separation = FmBoxFaceBoxSeparationBoxSpace(posDirToBoxA, 1, boxB.halfWidths[1], boxAPosInB, boxARotInB.col0, boxA.halfWidths[0], boxARotInB.col1, boxA.halfWidths[1], boxARotInB.col2, boxA.halfWidths[2]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }

        separatingAxisBoxSpace = FmInitVector3(0.0f, 0.0f, 1.0f);
        separation = FmBoxFaceBoxSeparationBoxSpace(posDirToBoxA, 2, boxB.halfWidths[2], boxAPosInB, boxARotInB.col0, boxA.halfWidths[0], boxARotInB.col1, boxA.halfWidths[1], boxARotInB.col2, boxA.halfWidths[2]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }

        // Cross products of box edge directions
        separatingAxisBoxSpace = FmSafeNormalizeCrossX(boxBRotInA.col0);
        separation = FmBoxEdgeBoxEdgeCrossSeparationBoxSpace(posDirToBoxB, separatingAxisBoxSpace, 1, boxA.halfWidths[1], 2, boxA.halfWidths[2],
            boxBPosInA, boxBRotInA.col1, boxB.halfWidths[1], boxBRotInA.col2, boxB.halfWidths[2]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossX(boxBRotInA.col1);
        separation = FmBoxEdgeBoxEdgeCrossSeparationBoxSpace(posDirToBoxB, separatingAxisBoxSpace, 1, boxA.halfWidths[1], 2, boxA.halfWidths[2],
            boxBPosInA, boxBRotInA.col2, boxB.halfWidths[2], boxBRotInA.col0, boxB.halfWidths[0]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossX(boxBRotInA.col2);
        separation = FmBoxEdgeBoxEdgeCrossSeparationBoxSpace(posDirToBoxB, separatingAxisBoxSpace, 1, boxA.halfWidths[1], 2, boxA.halfWidths[2],
            boxBPosInA, boxBRotInA.col0, boxB.halfWidths[0], boxBRotInA.col1, boxB.halfWidths[1]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossY(boxBRotInA.col0);
        separation = FmBoxEdgeBoxEdgeCrossSeparationBoxSpace(posDirToBoxB, separatingAxisBoxSpace, 2, boxA.halfWidths[2], 0, boxA.halfWidths[0],
            boxBPosInA, boxBRotInA.col1, boxB.halfWidths[1], boxBRotInA.col2, boxB.halfWidths[2]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossY(boxBRotInA.col1);
        separation = FmBoxEdgeBoxEdgeCrossSeparationBoxSpace(posDirToBoxB, separatingAxisBoxSpace, 2, boxA.halfWidths[2], 0, boxA.halfWidths[0],
            boxBPosInA, boxBRotInA.col2, boxB.halfWidths[2], boxBRotInA.col0, boxB.halfWidths[0]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossY(boxBRotInA.col2);
        separation = FmBoxEdgeBoxEdgeCrossSeparationBoxSpace(posDirToBoxB, separatingAxisBoxSpace, 2, boxA.halfWidths[2], 0, boxA.halfWidths[0],
            boxBPosInA, boxBRotInA.col0, boxB.halfWidths[0], boxBRotInA.col1, boxB.halfWidths[1]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossZ(boxBRotInA.col0);
        separation = FmBoxEdgeBoxEdgeCrossSeparationBoxSpace(posDirToBoxB, separatingAxisBoxSpace, 0, boxA.halfWidths[0], 1, boxA.halfWidths[1],
            boxBPosInA, boxBRotInA.col1, boxB.halfWidths[1], boxBRotInA.col2, boxB.halfWidths[2]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossZ(boxBRotInA.col1);
        separation = FmBoxEdgeBoxEdgeCrossSeparationBoxSpace(posDirToBoxB, separatingAxisBoxSpace, 0, boxA.halfWidths[0], 1, boxA.halfWidths[1],
            boxBPosInA, boxBRotInA.col2, boxB.halfWidths[2], boxBRotInA.col0, boxB.halfWidths[0]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }
        separatingAxisBoxSpace = FmSafeNormalizeCrossZ(boxBRotInA.col2);
        separation = FmBoxEdgeBoxEdgeCrossSeparationBoxSpace(posDirToBoxB, separatingAxisBoxSpace, 0, boxA.halfWidths[0], 1, boxA.halfWidths[1],
            boxBPosInA, boxBRotInA.col0, boxB.halfWidths[0], boxBRotInA.col1, boxB.halfWidths[1]);
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
        }

        return maxSeparation <= 0.0f;
    }

}
