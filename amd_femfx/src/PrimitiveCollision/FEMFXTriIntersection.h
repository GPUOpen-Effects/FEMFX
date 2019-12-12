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
// Functions to compute the intersection of a pair of triangles
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXTypes.h"

namespace AMD
{

    // Intersection (X) and Shadow (S) functions:
    //
    // Xij(featureA, featureB) is intersection of dimension i feature with dimension j feature (dimension of vert = 0, edge = 1, facet = 2) 
    // in coordinates dimensions up to (i + j).
    // E.g., X11 gives intersection state of edges (projections) in XY plane.
    // Nonzero result is intersecting, with sign affected by order of edge vertices.
    // 
    // Sij(featureA, featureB) returns Xij result when feature B "shadows" feature A, else 0.
    // Shadowing means intersection according to Xij (i+j < 3) and that at intersection point, point on B >= point on A in dimension (i + j + 1).
    // E.g., S11 = X11 if at the edge intersection in XY plane the point on edge B has z >= z of point on edge A, else 0
    //
    // Feature-pair intersection test in 3D is broken down into intersection tests in lower dimensions.
    // Intersection points are computed by interpolation of two lower-dimensional intersection points, one that shadows and one that 
    // intersects but does not shadow other feature.
    //
    // Intersection status and points of Xij(featureA, featureB) are identical regardless of order of feature vertices, with
    // sign of Xij return value affected by the order.
    //
    // Intersection status/points may be different between Xij(featureA, featureB) and Xji(featureB, featureA)
    //
    // Reference: Smith and Dodgson, "A topologically robust algorithm for Boolean operations on polyhedral shapes using approximate arithmetic"

    // Shadow function is intersection value if the value is non-zero and coord dimension of b >= a, else 0
    static FM_FORCE_INLINE int FmShadowFunction(int intersectionVal, float a, float b)
    {
        //return (intersectionVal != 0 && b >= a) ? intersectionVal : 0;
        return (b >= a) ? intersectionVal : 0;
    }

    // Intersection function for vertex A and edge B intersecting on X axis
    static FM_FORCE_INLINE int FmIntersectionX01(FmVector3* intersectionPointB, const FmVector3& vA, const FmVector3& eB0, const FmVector3& eB1)
    {
        int S00B0onA = (int)(eB0.x >= vA.x);
        int S00B1onA = (int)(eB1.x >= vA.x);

        // Get point on B that intersects A on X axis.
        // Intersection point is same regardless of order of edge vertices, when return value nonzero. 
        FmVector3 xBp = (S00B0onA != 0) ? eB0 : eB1;
        FmVector3 xBm = (S00B0onA != 0) ? eB1 : eB0;

        float t = (xBp.x - vA.x) / (xBp.x - xBm.x);

        FmVector3 vB;
        vB.x = vA.x;
        vB.y = xBp.y - t * (xBp.y - xBm.y);
        vB.z = xBp.z - t * (xBp.z - xBm.z);
        *intersectionPointB = vB;

        return S00B1onA - S00B0onA;
    }

    // Intersection function for edge A and vertex B intersecting on X axis
    static FM_FORCE_INLINE int FmIntersectionX10(FmVector3* intersectionPointA, const FmVector3& eA0, const FmVector3& eA1, const FmVector3& vB)
    {
        int S00BonA0 = (int)(vB.x >= eA0.x);
        int S00BonA1 = (int)(vB.x >= eA1.x);

        // Get point on B that intersects A on X axis.
        // Intersection point is same regardless of order of edge vertices, when return value nonzero. 
        FmVector3 xAp = (S00BonA0 != 0) ? eA0 : eA1;
        FmVector3 xAm = (S00BonA0 != 0) ? eA1 : eA0;

        float t = (vB.x - xAp.x) / (xAm.x - xAp.x);

        FmVector3 vA;
        vA.x = vB.x;
        vA.y = xAp.y - t * (xAp.y - xAm.y);
        vA.z = xAp.z - t * (xAp.z - xAm.z);
        *intersectionPointA = vA;

        return S00BonA0 - S00BonA1;
    }

    // Intersection function for edge A and edge B intersecting in XY plane
    int FmIntersectionX11(
        FmVector3* intersectionPointA, FmVector3* intersectionPointB,
        const FmVector3& eA0, const FmVector3& eA1, const FmVector3& eB0, const FmVector3& eB1);

    // Intersection function for vertex A and triangle B intersecting in XY plane
    int FmIntersectionX02(FmVector3* intersectionPointB, const FmVector3& vA, const FmVector3& fB0, const FmVector3& fB1, const FmVector3& fB2);

    // Intersection function for triangle A and vertex B intersecting in XY plane
    int FmIntersectionX20(FmVector3* intersectionPointA, const FmVector3& fA0, const FmVector3& fA1, const FmVector3& fA2, const FmVector3& vB);

    // Intersection function for edge A and triangle B intersecting in 3D
    int FmIntersectionX12(
        FmVector3* intersectionPoint,
        const FmVector3& eA0, const FmVector3& eA1,
        const FmVector3& fB0, const FmVector3& fB1, const FmVector3& fB2);

    // Intersection function for triangle A and edge B intersecting in 3D
    int FmIntersectionX21(
        FmVector3* intersectionPoint,
        const FmVector3& fA0, const FmVector3& fA1, const FmVector3& fA2,
        const FmVector3& eB0, const FmVector3& eB1);

    // Intersection function for vertex A and tetrahedron B intersecting in 3D
    int FmIntersectionX03(
        const FmVector3& vA,
        const FmVector3& tB0, const FmVector3& tB1, const FmVector3& tB2, const FmVector3& tB3);

    // Intersection function for tetrahedron A and vertex B intersecting in 3D
    int FmIntersectionX30(
        const FmVector3& tA0, const FmVector3& tA1, const FmVector3& tA2, const FmVector3& tA3,
        const FmVector3& vB);

    // Returns nonzero value == X02 when face B shadows vertex A
    static FM_FORCE_INLINE int FmShadowS02(
        const FmVector3& vA,
        const FmVector3& fB0, const FmVector3& fB1, const FmVector3& fB2)
    {
        FmVector3 intersectionPointBwithA;
        int X02ValBwithA = FmIntersectionX02(&intersectionPointBwithA, vA, fB0, fB1, fB2);
        return FmShadowFunction(X02ValBwithA, vA.z, intersectionPointBwithA.z);
    }

    // Returns nonzero value == X20 when vertex B shadows face A
    static FM_FORCE_INLINE int FmShadowS20(
        const FmVector3& fA0, const FmVector3& fA1, const FmVector3& fA2,
        const FmVector3& vB)
    {
        FmVector3 intersectionPointAwithB;
        int X20ValAwithB = FmIntersectionX20(&intersectionPointAwithB, fA0, fA1, fA2, vB);
        return FmShadowFunction(X20ValAwithB, intersectionPointAwithB.z, vB.z);
    }

    // Trivial reject of triangle overlap by projections against triangle planes.
    // TODO: Check robustness against degenerate tris.
    static FM_FORCE_INLINE bool
        FmRejectTriOverlap(
            const FmVector3& fA0, const FmVector3& fA1, const FmVector3& fA2,
            const FmVector3& fB0, const FmVector3& fB1, const FmVector3& fB2, float minDistance = 0.001f)
    {
        FmVector3 crossA = cross(fA1 - fA0, fA2 - fA0);
        FmVector3 crossB = cross(fB1 - fB0, fB2 - fB0);
        float magSqrA = lengthSqr(crossA);
        float magSqrB = lengthSqr(crossB);

        // Vectors from A plane to B verts, or from A verts to B plane
        FmVector3 diff0 = fB0 - fA0;
        FmVector3 diff1 = fB1 - fA1;
        FmVector3 diff2 = fB2 - fA2;

        float tolSqr = minDistance*minDistance;

        // Gives projection on triangle A normal multiplied by cross length
        float projA0 = dot(crossA, diff0);
        float projA1 = dot(crossA, diff1);
        float projA2 = dot(crossA, diff2);

        // Reject if projections have same sign and squares are greater than tolerance
        if ((projA0 > 0.0f && projA1 > 0.0f && projA2 > 0.0f) ||
            (projA0 < 0.0f && projA1 < 0.0f && projA2 < 0.0f))
        {
            float thresh = tolSqr * magSqrA;
            if (projA0*projA0 > thresh &&
                projA1*projA1 > thresh &&
                projA2*projA2 > thresh)
            {
                return true;
            }
        }

        float projB0 = dot(crossB, diff0);
        float projB1 = dot(crossB, diff1);
        float projB2 = dot(crossB, diff2);

        if ((projB0 > 0.0f && projB1 > 0.0f && projB2 > 0.0f) ||
            (projB0 < 0.0f && projB1 < 0.0f && projB2 < 0.0f))
        {
            float thresh = tolSqr * magSqrB;
            if (projB0*projB0 > thresh &&
                projB1*projB1 > thresh &&
                projB2*projB2 > thresh)
            {
                return true;
            }
        }

        return false;
    }

    static FM_FORCE_INLINE void FmBarycentricCoords(float barycentricCoords[3], const FmVector3& pos, const FmVector3& triPos0, const FmVector3& triPos1, const FmVector3& triPos2)
    {
        // Requires pos on triangle for valid result.
        // Compute barycentric values using ratio of sub-triangle area to triangle area.
        // Check for degenerate input and constrain results.
        FmVector3 u = triPos1 - triPos0;
        FmVector3 v = triPos2 - triPos0;
        FmVector3 w = pos - triPos0;

        FmVector3 uxv = cross(u, v);
        FmVector3 uxw = cross(u, w);
        FmVector3 vxw = cross(v, w);

        float len_uxv = length(uxv);
        float len_uxw = length(uxw);
        float len_vxw = length(vxw);

        float bary_x, bary_y, bary_z;

        bary_y = (len_uxv == 0.0f) ? 1.0f : len_vxw / len_uxv;
        bary_z = (len_uxv == 0.0f) ? 1.0f : len_uxw / len_uxv;

        bary_y = (bary_y > 1.0f) ? 1.0f : bary_y;
        bary_z = (bary_z > 1.0f) ? 1.0f : bary_z;

        bary_y = (bary_y + bary_z > 1.0f) ? 1.0f - bary_z : bary_y;
        bary_x = 1.0f - bary_y - bary_z;

        barycentricCoords[0] = bary_x;
        barycentricCoords[1] = bary_y;
        barycentricCoords[2] = bary_z;
    }

    // Compute barycentric, but check if point is equal to a triangle corner. Return corner index or else -1
    static FM_FORCE_INLINE int FmBarycentricCoordsCheckIfCorner(float barycentricCoords[3], const FmVector3& pos, const FmVector3& triPos0, const FmVector3& triPos1, const FmVector3& triPos2)
    {
        // Requires pos on triangle for valid result.
        // Compute barycentric values using ratio of sub-triangle area to triangle area.
        // Check for degenerate input and constrain results.

        if (FmIsEqual(pos, triPos0))
        {
            barycentricCoords[0] = 1.0f;
            barycentricCoords[1] = 0.0f;
            barycentricCoords[2] = 0.0f;
            return 0;
        }
        else if (FmIsEqual(pos, triPos1))
        {
            barycentricCoords[0] = 0.0f;
            barycentricCoords[1] = 1.0f;
            barycentricCoords[2] = 0.0f;
            return 1;
        }
        else if (FmIsEqual(pos, triPos2))
        {
            barycentricCoords[0] = 0.0f;
            barycentricCoords[1] = 0.0f;
            barycentricCoords[2] = 1.0f;
            return 2;
        }

        FmBarycentricCoords(barycentricCoords, pos, triPos0, triPos1, triPos2);
        return -1;
    }

    // Compute intersection points between two triangles (edge/face and face/edge) and the X intersection value for each.
    // Link start and end points into segments.
    struct FmTriIntersectionPoints
    {
        FmVector3 points[6];  // 3 edge/face + 3 face/edge
        int XVals[6];
        uint numSegments;
        uint startPoints[3];
        uint endPoints[3];

        FmTriIntersectionPoints() : numSegments(0) {}
    };

    bool FmComputeTriIntersection(FmTriIntersectionPoints* triIntersectionPoints, const FmVector3 fA[3], const FmVector3 fB[3]);

    // Contributions from the intersection of triangle edge with another surface.
    // Involves taking an integral over a subtriangle formed from tri pos 0, tri pos 1, and the edge intersection point.
    // isRetainedEdgeStart: "retained edge" means a directed edge belonging to the intersection volume surface.
    // If the point is the start of a retained edge, the integral is subtracted, else added.
    // The net effect of contributions from both the start and end points is an integral of the triangle between
    // tri corner 0 and the retained edge.  Can be combined with the contributions from other retained edges so the 
    // net result is an integral only over the portion of the face which belongs to the intersection volume surface.
    float FmComputeEdgeFaceIntersectionVolumeAndGradientContribution(
        FmVector3 dVdp_contribution[3],
        const FmVector3& triPos0, const FmVector3& triPos1, const FmVector3& triPos2, const FmVector3& edge1Pos,
        bool isRetainedEdgeStart);

    // Perform integrations if edgePos1 is not a corner.
    // Output if full integration, and number (sign) of full area integration, to be completed later.
    float FmComputePartialEdgeFaceIntersectionVolumeAndGradientContribution(
        bool* isFullFaceIntegration,
        int* numFullFaceIntegrations,
        FmVector3 dVdp_contribution[3],
        const FmVector3& triPos0, const FmVector3& triPos1, const FmVector3& triPos2, const FmVector3& edge1Pos,
        bool isRetainedEdgeStart);

    // Specialization of FmComputeEdgeFaceIntersectionVolumeAndGradientContribution() for full triangle
    float FmComputeFaceIntersectionVolumeAndGradientContribution(
        FmVector3 dVdp_contribution[3],
        const FmVector3& triPos0, const FmVector3& triPos1, const FmVector3& triPos2,
        bool isRetainedEdgeStart);

    // Computes volume and volume gradient contributions resulting from the intersection of two triangles.
    // For each triangle computes a contribution by integrating over the subtriangle formed from tri corner 0 and 
    // endpoints of the intersection segment.  
    // Can be combined with the contributions from other retained edges so the net result is an integral only 
    // over the portion of the face which belongs to the intersection volume surface.
    // NOTE: The tri intersection algorithm can produce extra degenerate segments but this won't affect the integral sums.
    void FmComputeTriIntersectionVolumeAndGradientContribution(
        FmVector3 dVdp_contributionA[3],
        FmVector3 dVdp_contributionB[3],
        float* V_contributionA,
        float* V_contributionB,
        const FmTriIntersectionPoints& triIntersectionPoints,
        const FmVector3& triAPos0, const FmVector3& triAPos1, const FmVector3& triAPos2,
        const FmVector3& triBPos0, const FmVector3& triBPos1, const FmVector3& triBPos2);

    // Perform integrations for any segments that aren't across the entire triangle area.
    // Output number of full area integrations, to be completed later.
    // Also output whether only full area integrations were computed (dVdp and V results are all zero).
    void FmComputePartialTriIntersectionVolumeAndGradientContribution(
        bool* onlyFullIntegrationsA,
        bool* onlyFullIntegrationsB,
        int* numFullIntegrationsA,
        int* numFullIntegrationsB,
        FmVector3 dVdp_contributionA[3],
        FmVector3 dVdp_contributionB[3],
        float* V_contributionA,
        float* V_contributionB,
        const FmTriIntersectionPoints& triIntersectionPoints,
        const FmVector3& triAPos0, const FmVector3& triAPos1, const FmVector3& triAPos2,
        const FmVector3& triBPos0, const FmVector3& triBPos1, const FmVector3& triBPos2);
}
