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
// BVH traversal operations to find contacts between FEM surface-triangle meshes, and 
// operations to test for nearest or intersected tets of rest-state meshes
//---------------------------------------------------------------------------------------

#include "AMD_FEMFX.h"
#include "FEMFXBvhBuild.h"
#include "FEMFXTetMesh.h"
#include "FEMFXTriCcd.h"
#include "FEMFXTriIntersection.h"
#include "FEMFXCollisionPairData.h"
#include "FEMFXFindContacts.h"

namespace AMD
{
    struct FmClosestTetResult;
    struct FmConstraintsBuffer;

    void FmFitLeafAabbs(FmTetMesh* tetMesh, float timestep, float padding)
    {
        FmBvh& bvh = tetMesh->bvh;

        uint numExteriorFaces = tetMesh->numExteriorFaces;

        bvh.numPrims = numExteriorFaces;

        if (numExteriorFaces == 0)
        {
            return;
        }

        FmVector3 minPosition = tetMesh->vertsPos[0];
        FmVector3 maxPosition = tetMesh->vertsPos[0];
        for (uint exteriorFaceId = 0; exteriorFaceId < numExteriorFaces; exteriorFaceId++)
        {
            FmExteriorFace& exteriorFace = tetMesh->exteriorFaces[exteriorFaceId];

            FmTetVertIds tetVerts = tetMesh->tetsVertIds[exteriorFace.tetId];
            FmFaceVertIds faceVerts;
            FmGetFaceVertIds(&faceVerts, exteriorFace.faceId, tetVerts);

            FmVector3 pos0 = tetMesh->vertsPos[faceVerts.ids[0]];
            FmVector3 pos1 = tetMesh->vertsPos[faceVerts.ids[1]];
            FmVector3 pos2 = tetMesh->vertsPos[faceVerts.ids[2]];
            FmVector3 vel0 = tetMesh->vertsVel[faceVerts.ids[0]];
            FmVector3 vel1 = tetMesh->vertsVel[faceVerts.ids[1]];
            FmVector3 vel2 = tetMesh->vertsVel[faceVerts.ids[2]];

            minPosition = min(minPosition, pos0);
            maxPosition = max(maxPosition, pos0);
            minPosition = min(minPosition, pos1);
            maxPosition = max(maxPosition, pos1);
            minPosition = min(minPosition, pos2);
            maxPosition = max(maxPosition, pos2);

            FmTri tri;
            tri.pos0 = pos0;
            tri.pos1 = pos1;
            tri.pos2 = pos2;
            tri.vel0 = vel0;
            tri.vel1 = vel1;
            tri.vel2 = vel2;

            FmAabb aabb = FmComputeTriAabb(tri, timestep, padding);

            bvh.primBoxes[exteriorFaceId] = aabb;
        }

        tetMesh->minPosition = minPosition;
        tetMesh->maxPosition = maxPosition;
    }

    // For CCD, must build after integration computes new velocities.
    void FmBuildHierarchy(FmTetMesh* tetMesh, float timestep, float gap)
    {
        FM_TRACE_SCOPED_EVENT(REBUILD_BVH);

        FmFitLeafAabbs(tetMesh, timestep, gap);
        FmBuildBvhOnLeaves(&tetMesh->bvh, tetMesh->minPosition, tetMesh->maxPosition);
    }

    // Fit leaves on rest position tets of a constructed TetMesh
    void FmFitLeafAabbsOnRestTets(FmBvh* outBvh, FmVector3* tetMeshMinPos, FmVector3* tetMeshMaxPos, const FmTetMesh& tetMesh)
    {
        FmBvh& bvh = *outBvh;

        uint numTets = tetMesh.numTets;

        bvh.numPrims = numTets;

        if (numTets == 0)
        {
            *tetMeshMinPos = FmInitVector3(0.0);
            *tetMeshMaxPos = FmInitVector3(0.0);
            return;
        }

        FmVector3 meshMinPos = tetMesh.vertsRestPos[0];
        FmVector3 meshMaxPos = tetMesh.vertsRestPos[0];
        for (uint tetId = 0; tetId < numTets; tetId++)
        {
            FmTetVertIds tetVerts = tetMesh.tetsVertIds[tetId];

            FmVector3 pos0 = tetMesh.vertsRestPos[tetVerts.ids[0]];
            FmVector3 pos1 = tetMesh.vertsRestPos[tetVerts.ids[1]];
            FmVector3 pos2 = tetMesh.vertsRestPos[tetVerts.ids[2]];
            FmVector3 pos3 = tetMesh.vertsRestPos[tetVerts.ids[3]];

            FmVector3 minPos = pos0;
            FmVector3 maxPos = pos0;
            minPos = min(minPos, pos1);
            maxPos = max(maxPos, pos1);
            minPos = min(minPos, pos2);
            maxPos = max(maxPos, pos2);
            minPos = min(minPos, pos3);
            maxPos = max(maxPos, pos3);

            meshMinPos = min(meshMinPos, pos0);
            meshMaxPos = max(meshMaxPos, pos0);
            meshMinPos = min(meshMinPos, pos1);
            meshMaxPos = max(meshMaxPos, pos1);
            meshMinPos = min(meshMinPos, pos2);
            meshMaxPos = max(meshMaxPos, pos2);
            meshMinPos = min(meshMinPos, pos3);
            meshMaxPos = max(meshMaxPos, pos3);

            FmAabb aabb;
            aabb.pmin = minPos;
            aabb.pmax = maxPos;
            aabb.vmin = FmInitVector3(0.0f);
            aabb.vmax = FmInitVector3(0.0f);

            bvh.primBoxes[tetId] = aabb;
        }

        *tetMeshMinPos = meshMinPos;
        *tetMeshMaxPos = meshMaxPos;
    }

    // Fit leaves on rest position tets provided as array of positions and tet vertex ids.
    void FmFitLeafAabbsOnRestTets(
        FmBvh* outBvh, FmVector3* tetMeshMinPos, FmVector3* tetMeshMaxPos,
        const FmVector3* vertRestPositions, const FmTetVertIds* tetVertIds, uint numTets)
    {
        FmBvh& bvh = *outBvh;

        bvh.numPrims = numTets;

        FmVector3 meshMinPos = vertRestPositions[0];
        FmVector3 meshMaxPos = vertRestPositions[0];
        for (uint tetId = 0; tetId < numTets; tetId++)
        {
            FmTetVertIds tetVerts = tetVertIds[tetId];

            FmVector3 pos0 = vertRestPositions[tetVerts.ids[0]];
            FmVector3 pos1 = vertRestPositions[tetVerts.ids[1]];
            FmVector3 pos2 = vertRestPositions[tetVerts.ids[2]];
            FmVector3 pos3 = vertRestPositions[tetVerts.ids[3]];

            FmVector3 minPos = pos0;
            FmVector3 maxPos = pos0;
            minPos = min(minPos, pos1);
            maxPos = max(maxPos, pos1);
            minPos = min(minPos, pos2);
            maxPos = max(maxPos, pos2);
            minPos = min(minPos, pos3);
            maxPos = max(maxPos, pos3);

            meshMinPos = min(meshMinPos, pos0);
            meshMaxPos = max(meshMaxPos, pos0);
            meshMinPos = min(meshMinPos, pos1);
            meshMaxPos = max(meshMaxPos, pos1);
            meshMinPos = min(meshMinPos, pos2);
            meshMaxPos = max(meshMaxPos, pos2);
            meshMinPos = min(meshMinPos, pos3);
            meshMaxPos = max(meshMaxPos, pos3);

            FmAabb aabb;
            aabb.pmin = minPos;
            aabb.pmax = maxPos;
            aabb.vmin = FmInitVector3(0.0f);
            aabb.vmax = FmInitVector3(0.0f);

            bvh.primBoxes[tetId] = aabb;
        }

        *tetMeshMinPos = meshMinPos;
        *tetMeshMaxPos = meshMaxPos;
    }

    // Build BVH over rest mesh tetrahedra for nearest tetrahedron query.
    // bvh expected to be preallocated for tetMesh->numTets leaves.
    void FmBuildRestMeshTetBvh(FmBvh* bvh, const FmTetMesh& tetMesh)
    {
        FmVector3 meshMinPos, meshMaxPos;
        FmFitLeafAabbsOnRestTets(bvh, &meshMinPos, &meshMaxPos, tetMesh);
        FmBuildBvhOnLeaves(bvh, meshMinPos, meshMaxPos);
    }

    // Build BVH over rest mesh tetrahedra for nearest tetrahedron query.
    // bvh expected to be preallocated for tetMesh->numTets leaves.
    void FmBuildRestMeshTetBvh(FmBvh* bvh, const FmVector3* vertRestPositions, const FmTetVertIds* tetVertIds, uint numTets)
    {
        FmVector3 meshMinPos, meshMaxPos;
        FmFitLeafAabbsOnRestTets(bvh, &meshMinPos, &meshMaxPos, vertRestPositions, tetVertIds, numTets);
        FmBuildBvhOnLeaves(bvh, meshMinPos, meshMaxPos);
    }

    struct FmClosestTetQueryState
    {
        const FmTetMesh* tetMesh;            // if NULL uses vertRestPositions and tetVertIds
        const FmVector3* vertRestPositions;
        const FmTetVertIds* tetVertIds;
        const FmBvh* bvh;
        FmVector3 queryPoint;
        FmVector3 closestPoint;
        uint closestTetId;
        uint closestFaceId;
        float closestDistance;
        bool insideTet;
    };

    static FM_FORCE_INLINE float FmPointBoxDistance(const FmVector3& queryPoint, const FmAabb& box)
    {
        float maxGapX = FmMaxFloat(FmMaxFloat(queryPoint.x - box.pmax.x, box.pmin.x - queryPoint.x), 0.0f);
        float maxGapY = FmMaxFloat(FmMaxFloat(queryPoint.y - box.pmax.y, box.pmin.y - queryPoint.y), 0.0f);
        float maxGapZ = FmMaxFloat(FmMaxFloat(queryPoint.z - box.pmax.z, box.pmin.z - queryPoint.z), 0.0f);

        float distance = sqrtf(maxGapX*maxGapX + maxGapY*maxGapY + maxGapZ*maxGapZ);

        return distance;
    }

    void FmFindClosestTetRecursive(FmClosestTetQueryState& queryState, uint idx)
    {
        const FmTetMesh* tetMesh = queryState.tetMesh;
        const FmVector3* vertRestPositions = queryState.vertRestPositions;
        const FmTetVertIds* tetVertIds = queryState.tetVertIds;
        const FmBvh* hierarchy = queryState.bvh;
        FmVector3 queryPoint = queryState.queryPoint;

        FmBvhNode* nodes = hierarchy->nodes;

        int lc = nodes[idx].left;
        int rc = nodes[idx].right;

        if (lc == rc)
        {
            // Leaf, find nearest position of tet
            uint tetId = (uint)lc;
            FmVector3 tetPos[4];

            if (tetMesh)
            {
                FmTetVertIds tetVerts = tetMesh->tetsVertIds[tetId];
                tetPos[0] = tetMesh->vertsRestPos[tetVerts.ids[0]];
                tetPos[1] = tetMesh->vertsRestPos[tetVerts.ids[1]];
                tetPos[2] = tetMesh->vertsRestPos[tetVerts.ids[2]];
                tetPos[3] = tetMesh->vertsRestPos[tetVerts.ids[3]];
            }
            else
            {
                FmTetVertIds tetVerts = tetVertIds[tetId];
                tetPos[0] = vertRestPositions[tetVerts.ids[0]];
                tetPos[1] = vertRestPositions[tetVerts.ids[1]];
                tetPos[2] = vertRestPositions[tetVerts.ids[2]];
                tetPos[3] = vertRestPositions[tetVerts.ids[3]];
            }

            int intersectionVal = FmIntersectionX03(queryPoint, tetPos[0], tetPos[1], tetPos[2], tetPos[3]);

            if (intersectionVal)
            {
                // Query point inside tet, can return
                queryState.closestPoint = queryPoint;
                queryState.closestTetId = tetId;
                queryState.closestDistance = 0.0f;
                queryState.insideTet = true;
                return;
            }
            else
            {
                // Check tet faces for points nearer than current result
                for (uint faceId = 0; faceId < 4; faceId++)
                {
                    FmFaceVertIds tetCorners;
                    FmGetFaceTetCorners(&tetCorners, faceId);
                    FmVector3 triPos0 = tetPos[tetCorners.ids[0]];
                    FmVector3 triPos1 = tetPos[tetCorners.ids[1]];
                    FmVector3 triPos2 = tetPos[tetCorners.ids[2]];

                    FmDistanceResult distResult;
                    FmPointTriangleDistance(&distResult, queryPoint, triPos0, triPos1, triPos2, 0, 0, 1, 2);

                    if (distResult.distance < queryState.closestDistance)
                    {
                        queryState.closestPoint = distResult.posj;
                        queryState.closestTetId = tetId;
                        queryState.closestFaceId = faceId;
                        queryState.closestDistance = distResult.distance;
                    }
                }
            }
        }
        else
        {
            FmAabb& lbox = hierarchy->nodes[lc].box;
            FmAabb& rbox = hierarchy->nodes[rc].box;

            float ldistance = FmPointBoxDistance(queryPoint, lbox);
            float rdistance = FmPointBoxDistance(queryPoint, rbox);

            if (ldistance < rdistance)
            {
                if (ldistance < queryState.closestDistance)
                {
                    FmFindClosestTetRecursive(queryState, (uint)lc);
                }
                if (rdistance < queryState.closestDistance)
                {
                    FmFindClosestTetRecursive(queryState, (uint)rc);
                }
            }
            else
            {
                if (rdistance < queryState.closestDistance)
                {
                    FmFindClosestTetRecursive(queryState, (uint)rc);
                }
                if (ldistance < queryState.closestDistance)
                {
                    FmFindClosestTetRecursive(queryState, (uint)lc);
                }
            }
        }
    }

    // Find the nearest tetrahedron and nearest point and face to the query point, 
    // given tet mesh and BVH built with FmBuildRestMeshTetBvh.
    void FmFindClosestTet(FmClosestTetResult* closestTet, const FmTetMesh* tetMesh, const FmBvh* bvh, const FmVector3& queryPoint)
    {
        FmClosestTetQueryState queryState;
        queryState.tetMesh = tetMesh;
        queryState.vertRestPositions = NULL;
        queryState.tetVertIds = NULL;
        queryState.bvh = bvh;
        queryState.queryPoint = queryPoint;
        queryState.closestPoint = FmInitVector3(0.0f);
        queryState.closestDistance = FLT_MAX;
        queryState.closestTetId = FM_INVALID_ID;
        queryState.closestFaceId = FM_INVALID_ID;
        queryState.insideTet = false;

        FmFindClosestTetRecursive(queryState, 0);

        closestTet->position = queryState.closestPoint;
        closestTet->distance = queryState.closestDistance;
        closestTet->tetId = queryState.closestTetId;
        closestTet->faceId = queryState.closestFaceId;
        closestTet->insideTet = queryState.insideTet;

        if (queryState.closestTetId == FM_INVALID_ID)
        {
            FM_PRINT(("FindClosestTet: invalid mesh or hierarchy\n"));
        }
        else
        {
            FmVector4 barycentrics = mul(tetMesh->tetsShapeParams[queryState.closestTetId].baryMatrix, FmVector4(queryState.closestPoint, 1.0f));
            closestTet->posBary[0] = barycentrics.x;
            closestTet->posBary[1] = barycentrics.y;
            closestTet->posBary[2] = barycentrics.z;
            closestTet->posBary[3] = barycentrics.w;
        }
    }

    // Find the closest tetrahedron, point and face to the query point, 
    // given tet mesh and BVH built with FmBuildRestMeshTetBvh.
    void FmFindClosestTet(
        FmClosestTetResult* closestTet,
        const FmVector3* vertRestPositions, const FmTetVertIds* tetVertIds, const FmBvh* bvh,
        const FmVector3& queryPoint)
    {
        FmClosestTetQueryState queryState;
        queryState.tetMesh = NULL;
        queryState.vertRestPositions = vertRestPositions;
        queryState.tetVertIds = tetVertIds;
        queryState.bvh = bvh;
        queryState.queryPoint = queryPoint;
        queryState.closestPoint = FmInitVector3(0.0f);
        queryState.closestDistance = FLT_MAX;
        queryState.closestTetId = FM_INVALID_ID;
        queryState.closestFaceId = FM_INVALID_ID;
        queryState.insideTet = false;

        FmFindClosestTetRecursive(queryState, 0);

        closestTet->position = queryState.closestPoint;
        closestTet->distance = queryState.closestDistance;
        closestTet->tetId = queryState.closestTetId;
        closestTet->faceId = queryState.closestFaceId;
        closestTet->insideTet = queryState.insideTet;

        if (queryState.closestTetId == FM_INVALID_ID)
        {
            FM_PRINT(("FindClosestTet: invalid mesh or hierarchy\n"));
        }
        else
        {
            FmMatrix4 baryMatrix = FmComputeTetBarycentricMatrix(vertRestPositions, tetVertIds[closestTet->tetId]);

            FmVector4 barycentrics = mul(baryMatrix, FmVector4(queryState.closestPoint, 1.0f));
            closestTet->posBary[0] = barycentrics.x;
            closestTet->posBary[1] = barycentrics.y;
            closestTet->posBary[2] = barycentrics.z;
            closestTet->posBary[3] = barycentrics.w;
        }
    }

    void FmFindIntersectedTetRecursive(FmClosestTetQueryState& queryState, uint idx)
    {
        const FmTetMesh* tetMesh = queryState.tetMesh;
        const FmVector3* vertRestPositions = queryState.vertRestPositions;
        const FmTetVertIds* tetVertIds = queryState.tetVertIds;
        const FmBvh* hierarchy = queryState.bvh;
        FmVector3 queryPoint = queryState.queryPoint;

        FmBvhNode* nodes = hierarchy->nodes;

        FmAabb& box = hierarchy->nodes[idx].box;

        if (queryPoint.x < box.pmin.x || queryPoint.x > box.pmax.x
            || queryPoint.y < box.pmin.y || queryPoint.y > box.pmax.y
            || queryPoint.z < box.pmin.z || queryPoint.z > box.pmax.z)
        {
            return;
        }

        int lc = nodes[idx].left;
        int rc = nodes[idx].right;

        if (lc == rc)
        {
            // Leaf, test intersection
            uint tetId = (uint)lc;
            FmVector3 tetPos[4];

            if (tetMesh)
            {
                FmTetVertIds tetVerts = tetMesh->tetsVertIds[tetId];
                tetPos[0] = tetMesh->vertsRestPos[tetVerts.ids[0]];
                tetPos[1] = tetMesh->vertsRestPos[tetVerts.ids[1]];
                tetPos[2] = tetMesh->vertsRestPos[tetVerts.ids[2]];
                tetPos[3] = tetMesh->vertsRestPos[tetVerts.ids[3]];
            }
            else
            {
                FmTetVertIds tetVerts = tetVertIds[tetId];
                tetPos[0] = vertRestPositions[tetVerts.ids[0]];
                tetPos[1] = vertRestPositions[tetVerts.ids[1]];
                tetPos[2] = vertRestPositions[tetVerts.ids[2]];
                tetPos[3] = vertRestPositions[tetVerts.ids[3]];
            }

            int intersectionVal = FmIntersectionX03(queryPoint, tetPos[0], tetPos[1], tetPos[2], tetPos[3]);

            if (intersectionVal)
            {
                // Query point inside tet, can return
                queryState.closestPoint = queryPoint;
                queryState.closestTetId = tetId;
                queryState.closestDistance = 0.0f;
                queryState.insideTet = true;
                return;
            }
        }
        else
        {
            FmFindIntersectedTetRecursive(queryState, (uint)lc);
            FmFindIntersectedTetRecursive(queryState, (uint)rc);
        }
    }

    // Find the nearest tetrahedron and nearest point and face to the query point, 
    // given tet mesh and BVH built with FmBuildRestMeshTetBvh.
    void FmFindIntersectedTet(FmClosestTetResult* closestTet, const FmTetMesh* tetMesh, const FmBvh* bvh, const FmVector3& queryPoint)
    {
        FmClosestTetQueryState queryState;
        queryState.tetMesh = tetMesh;
        queryState.vertRestPositions = NULL;
        queryState.tetVertIds = NULL;
        queryState.bvh = bvh;
        queryState.queryPoint = queryPoint;
        queryState.closestPoint = FmInitVector3(0.0f);
        queryState.closestDistance = FLT_MAX;
        queryState.closestTetId = FM_INVALID_ID;
        queryState.closestFaceId = FM_INVALID_ID;
        queryState.insideTet = false;

        FmFindIntersectedTetRecursive(queryState, 0);

        closestTet->position = queryState.closestPoint;
        closestTet->distance = queryState.closestDistance;
        closestTet->tetId = queryState.closestTetId;
        closestTet->faceId = queryState.closestFaceId;
        closestTet->insideTet = queryState.insideTet;
        closestTet->posBary[0] = 0.0f;
        closestTet->posBary[1] = 0.0f;
        closestTet->posBary[2] = 0.0f;
        closestTet->posBary[3] = 0.0f;

        if (queryState.closestTetId != FM_INVALID_ID)
        {
            FmVector4 barycentrics = mul(tetMesh->tetsShapeParams[queryState.closestTetId].baryMatrix, FmVector4(queryState.closestPoint, 1.0f));
            closestTet->posBary[0] = barycentrics.x;
            closestTet->posBary[1] = barycentrics.y;
            closestTet->posBary[2] = barycentrics.z;
            closestTet->posBary[3] = barycentrics.w;
        }
    }

    // Find the closest tetrahedron, point and face to the query point, 
    // given tet mesh and BVH built with FmBuildRestMeshTetBvh.
    void FmFindIntersectedTet(
        FmClosestTetResult* closestTet,
        const FmVector3* vertRestPositions, const FmTetVertIds* tetVertIds, const FmBvh* bvh,
        const FmVector3& queryPoint)
    {
        FmClosestTetQueryState queryState;
        queryState.tetMesh = NULL;
        queryState.vertRestPositions = vertRestPositions;
        queryState.tetVertIds = tetVertIds;
        queryState.bvh = bvh;
        queryState.queryPoint = queryPoint;
        queryState.closestPoint = FmInitVector3(0.0f);
        queryState.closestDistance = FLT_MAX;
        queryState.closestTetId = FM_INVALID_ID;
        queryState.closestFaceId = FM_INVALID_ID;
        queryState.insideTet = false;

        FmFindIntersectedTetRecursive(queryState, 0);

        closestTet->position = queryState.closestPoint;
        closestTet->distance = queryState.closestDistance;
        closestTet->tetId = queryState.closestTetId;
        closestTet->faceId = queryState.closestFaceId;
        closestTet->insideTet = queryState.insideTet;
        closestTet->posBary[0] = 0.0f;
        closestTet->posBary[1] = 0.0f;
        closestTet->posBary[2] = 0.0f;
        closestTet->posBary[3] = 0.0f;

        if (queryState.closestTetId != FM_INVALID_ID)
        {
            FmMatrix4 baryMatrix = FmComputeTetBarycentricMatrix(vertRestPositions, tetVertIds[closestTet->tetId]);

            FmVector4 barycentrics = mul(baryMatrix, FmVector4(queryState.closestPoint, 1.0f));
            closestTet->posBary[0] = barycentrics.x;
            closestTet->posBary[1] = barycentrics.y;
            closestTet->posBary[2] = barycentrics.z;
            closestTet->posBary[3] = barycentrics.w;
        }
    }

    struct FmTetsIntersectingBoxQueryState
    {
        uint* tets;
        uint numTets;
        const FmTetMesh* tetMesh;           // If NULL, use vertRestPositions and tetVertIds
        const FmVector3* vertRestPositions;
        const FmTetVertIds* tetVertIds;
        const FmBvh* bvh;
        FmVector3 boxHalfDimensions;
        FmVector3 boxCenterPos;
        FmMatrix3 boxRotation;
        FmMatrix3 boxRotationInv;
        FmVector3 boxPoints[8];
        FmVector3 aabbMinPos;
        FmVector3 aabbMaxPos;
    };

    void FmFindTetsIntersectingBoxRecursive(FmTetsIntersectingBoxQueryState& queryState, uint idx)
    {
        const FmTetMesh* tetMesh = queryState.tetMesh;
        const FmVector3* vertRestPositions = queryState.vertRestPositions;
        const FmTetVertIds* tetVertIds = queryState.tetVertIds;
        const FmBvh* hierarchy = queryState.bvh;
        FmBvhNode* nodes = hierarchy->nodes;

        // Return if box's AABB does not intersect BVH node
        FmAabb& box = hierarchy->nodes[idx].box;

        if (queryState.aabbMaxPos.x < box.pmin.x || queryState.aabbMinPos.x > box.pmax.x
            || queryState.aabbMaxPos.y < box.pmin.y || queryState.aabbMinPos.y > box.pmax.y
            || queryState.aabbMaxPos.z < box.pmin.z || queryState.aabbMinPos.z > box.pmax.z)
        {
            return;
        }

        int lc = nodes[idx].left;
        int rc = nodes[idx].right;

        if (lc == rc)
        {
            // Leaf, check for intersection
            uint tetId = (uint)lc;

            FmVector3 tetPos[4];
            if (tetMesh)
            {
                FmTetVertIds tetVerts = tetMesh->tetsVertIds[tetId];
                tetPos[0] = tetMesh->vertsRestPos[tetVerts.ids[0]];
                tetPos[1] = tetMesh->vertsRestPos[tetVerts.ids[1]];
                tetPos[2] = tetMesh->vertsRestPos[tetVerts.ids[2]];
                tetPos[3] = tetMesh->vertsRestPos[tetVerts.ids[3]];
            }
            else
            {
                FmTetVertIds tetVerts = tetVertIds[tetId];
                tetPos[0] = vertRestPositions[tetVerts.ids[0]];
                tetPos[1] = vertRestPositions[tetVerts.ids[1]];
                tetPos[2] = vertRestPositions[tetVerts.ids[2]];
                tetPos[3] = vertRestPositions[tetVerts.ids[3]];
            }

            FmVector3 tetPosBoxSpace[4];
            tetPosBoxSpace[0] = mul(queryState.boxRotationInv, tetPos[0] - queryState.boxCenterPos);
            tetPosBoxSpace[1] = mul(queryState.boxRotationInv, tetPos[1] - queryState.boxCenterPos);
            tetPosBoxSpace[2] = mul(queryState.boxRotationInv, tetPos[2] - queryState.boxCenterPos);
            tetPosBoxSpace[3] = mul(queryState.boxRotationInv, tetPos[3] - queryState.boxCenterPos);

            bool intersecting = false;

            // Check for box points inside tet
            for (uint boxPntIdx = 0; boxPntIdx < 8; boxPntIdx++)
            {
                if (FmIntersectionX30(tetPosBoxSpace[0], tetPosBoxSpace[1], tetPosBoxSpace[2], tetPosBoxSpace[3], queryState.boxPoints[boxPntIdx]))
                {
                    intersecting = true;
                    break;
                }
            }

            // Check for tet points inside box
            if (!intersecting)
            {
                FmVector3 halfDims = queryState.boxHalfDimensions;
                for (uint tetPntIdx = 0; tetPntIdx < 4; tetPntIdx++)
                {
                    if (tetPosBoxSpace[tetPntIdx].x >= -halfDims.x && tetPosBoxSpace[tetPntIdx].x <= halfDims.x
                        && tetPosBoxSpace[tetPntIdx].y >= -halfDims.y && tetPosBoxSpace[tetPntIdx].y <= halfDims.y
                        && tetPosBoxSpace[tetPntIdx].z >= -halfDims.z && tetPosBoxSpace[tetPntIdx].z <= halfDims.z)
                    {
                        intersecting = true;
                    }
                }
            }

            // Test for edges intersecting faces
            if (!intersecting)
            {
                for (uint tvi = 0; tvi < 4; tvi++)
                {
                    FmVector3 intersectionPnt;

                    FmVector3 tetEdgePosi = tetPosBoxSpace[tvi];
                    for (uint tvj = tvi + 1; tvj < 4; tvj++)
                    {
                        FmVector3 tetEdgePosj = tetPosBoxSpace[tvj];

                        if (FmIntersectionX12(&intersectionPnt, tetEdgePosi, tetEdgePosj, queryState.boxPoints[0], queryState.boxPoints[2], queryState.boxPoints[1])) { intersecting = true; break; }
                        if (FmIntersectionX12(&intersectionPnt, tetEdgePosi, tetEdgePosj, queryState.boxPoints[1], queryState.boxPoints[2], queryState.boxPoints[3])) { intersecting = true; break; }
                        if (FmIntersectionX12(&intersectionPnt, tetEdgePosi, tetEdgePosj, queryState.boxPoints[0], queryState.boxPoints[1], queryState.boxPoints[4])) { intersecting = true; break; }
                        if (FmIntersectionX12(&intersectionPnt, tetEdgePosi, tetEdgePosj, queryState.boxPoints[4], queryState.boxPoints[1], queryState.boxPoints[5])) { intersecting = true; break; }
                        if (FmIntersectionX12(&intersectionPnt, tetEdgePosi, tetEdgePosj, queryState.boxPoints[1], queryState.boxPoints[3], queryState.boxPoints[5])) { intersecting = true; break; }
                        if (FmIntersectionX12(&intersectionPnt, tetEdgePosi, tetEdgePosj, queryState.boxPoints[5], queryState.boxPoints[3], queryState.boxPoints[7])) { intersecting = true; break; }
                        if (FmIntersectionX12(&intersectionPnt, tetEdgePosi, tetEdgePosj, queryState.boxPoints[0], queryState.boxPoints[4], queryState.boxPoints[2])) { intersecting = true; break; }
                        if (FmIntersectionX12(&intersectionPnt, tetEdgePosi, tetEdgePosj, queryState.boxPoints[2], queryState.boxPoints[4], queryState.boxPoints[6])) { intersecting = true; break; }
                        if (FmIntersectionX12(&intersectionPnt, tetEdgePosi, tetEdgePosj, queryState.boxPoints[4], queryState.boxPoints[5], queryState.boxPoints[6])) { intersecting = true; break; }
                        if (FmIntersectionX12(&intersectionPnt, tetEdgePosi, tetEdgePosj, queryState.boxPoints[6], queryState.boxPoints[5], queryState.boxPoints[7])) { intersecting = true; break; }
                        if (FmIntersectionX12(&intersectionPnt, tetEdgePosi, tetEdgePosj, queryState.boxPoints[2], queryState.boxPoints[6], queryState.boxPoints[3])) { intersecting = true; break; }
                        if (FmIntersectionX12(&intersectionPnt, tetEdgePosi, tetEdgePosj, queryState.boxPoints[3], queryState.boxPoints[6], queryState.boxPoints[7])) { intersecting = true; break; }
                    }

                    FmFaceVertIds tetCorners;
                    FmGetFaceTetCorners(&tetCorners, tvi);
                    FmVector3 triPos0 = tetPosBoxSpace[tetCorners.ids[0]];
                    FmVector3 triPos1 = tetPosBoxSpace[tetCorners.ids[1]];
                    FmVector3 triPos2 = tetPosBoxSpace[tetCorners.ids[2]];

                    if (FmIntersectionX21(&intersectionPnt, triPos0, triPos1, triPos2, queryState.boxPoints[0], queryState.boxPoints[1])) { intersecting = true; break; }
                    if (FmIntersectionX21(&intersectionPnt, triPos0, triPos1, triPos2, queryState.boxPoints[2], queryState.boxPoints[3])) { intersecting = true; break; }
                    if (FmIntersectionX21(&intersectionPnt, triPos0, triPos1, triPos2, queryState.boxPoints[4], queryState.boxPoints[5])) { intersecting = true; break; }
                    if (FmIntersectionX21(&intersectionPnt, triPos0, triPos1, triPos2, queryState.boxPoints[6], queryState.boxPoints[7])) { intersecting = true; break; }
                    if (FmIntersectionX21(&intersectionPnt, triPos0, triPos1, triPos2, queryState.boxPoints[0], queryState.boxPoints[2])) { intersecting = true; break; }
                    if (FmIntersectionX21(&intersectionPnt, triPos0, triPos1, triPos2, queryState.boxPoints[1], queryState.boxPoints[3])) { intersecting = true; break; }
                    if (FmIntersectionX21(&intersectionPnt, triPos0, triPos1, triPos2, queryState.boxPoints[4], queryState.boxPoints[6])) { intersecting = true; break; }
                    if (FmIntersectionX21(&intersectionPnt, triPos0, triPos1, triPos2, queryState.boxPoints[5], queryState.boxPoints[7])) { intersecting = true; break; }
                    if (FmIntersectionX21(&intersectionPnt, triPos0, triPos1, triPos2, queryState.boxPoints[0], queryState.boxPoints[4])) { intersecting = true; break; }
                    if (FmIntersectionX21(&intersectionPnt, triPos0, triPos1, triPos2, queryState.boxPoints[1], queryState.boxPoints[5])) { intersecting = true; break; }
                    if (FmIntersectionX21(&intersectionPnt, triPos0, triPos1, triPos2, queryState.boxPoints[2], queryState.boxPoints[6])) { intersecting = true; break; }
                    if (FmIntersectionX21(&intersectionPnt, triPos0, triPos1, triPos2, queryState.boxPoints[3], queryState.boxPoints[7])) { intersecting = true; break; }
                }
            }

            if (intersecting)
            {
                queryState.tets[queryState.numTets] = tetId;
                queryState.numTets++;
            }
        }
        else
        {
            FmFindTetsIntersectingBoxRecursive(queryState, (uint)lc);
            FmFindTetsIntersectingBoxRecursive(queryState, (uint)rc);
        }
    }

    // Find all the rest mesh tetrahedra that intersect the box.
    // Saves output to tetsIntersectingBox array, expected to be large enough for all tets of the mesh.
    // Returns the number of intersected tets.
    uint FmFindTetsIntersectingBox(
        uint* tetsIntersectingBox,
        const FmTetMesh* tetMesh, const FmBvh* bvh,
        const FmVector3& boxHalfDimensions,
        const FmVector3& boxCenterPos,
        const FmMatrix3& boxRotation)
    {
        FmTetsIntersectingBoxQueryState queryState;
        queryState.tetMesh = tetMesh;
        queryState.vertRestPositions = NULL;
        queryState.tetVertIds = NULL;
        queryState.bvh = bvh;
        queryState.boxHalfDimensions = boxHalfDimensions;
        queryState.boxCenterPos = boxCenterPos;
        queryState.boxRotation = boxRotation;
        queryState.boxRotationInv = transpose(boxRotation);
        queryState.boxPoints[0] = FmInitVector3(-boxHalfDimensions.x, -boxHalfDimensions.y, -boxHalfDimensions.z);
        queryState.boxPoints[1] = FmInitVector3(boxHalfDimensions.x, -boxHalfDimensions.y, -boxHalfDimensions.z);
        queryState.boxPoints[2] = FmInitVector3(-boxHalfDimensions.x, boxHalfDimensions.y, -boxHalfDimensions.z);
        queryState.boxPoints[3] = FmInitVector3(boxHalfDimensions.x, boxHalfDimensions.y, -boxHalfDimensions.z);
        queryState.boxPoints[4] = FmInitVector3(-boxHalfDimensions.x, -boxHalfDimensions.y, boxHalfDimensions.z);
        queryState.boxPoints[5] = FmInitVector3(boxHalfDimensions.x, -boxHalfDimensions.y, boxHalfDimensions.z);
        queryState.boxPoints[6] = FmInitVector3(-boxHalfDimensions.x, boxHalfDimensions.y, boxHalfDimensions.z);
        queryState.boxPoints[7] = FmInitVector3(boxHalfDimensions.x, boxHalfDimensions.y, boxHalfDimensions.z);
        FmVector3 absCol0 = abs(boxRotation.col0);
        FmVector3 absCol1 = abs(boxRotation.col1);
        FmVector3 absCol2 = abs(boxRotation.col2);
        queryState.aabbMinPos = boxCenterPos - absCol0 * boxHalfDimensions.x - absCol1 * boxHalfDimensions.y - absCol2 * boxHalfDimensions.z;
        queryState.aabbMaxPos = boxCenterPos + absCol0 * boxHalfDimensions.x + absCol1 * boxHalfDimensions.y + absCol2 * boxHalfDimensions.z;
        queryState.tets = tetsIntersectingBox;
        queryState.numTets = 0;

        FmFindTetsIntersectingBoxRecursive(queryState, 0);

        return queryState.numTets;
    }

    // Find all the rest mesh tetrahedra that intersect the box.
    // Saves output to tetsIntersectingBox array, expected to be large enough for all tets of the mesh.
    // Returns the number of intersected tets.
    uint FmFindTetsIntersectingBox(
        uint* tetsIntersectingBox,
        const FmVector3* vertRestPositions, const FmTetVertIds* tetVertIds, const FmBvh* bvh,
        const FmVector3& boxHalfDimensions,
        const FmVector3& boxCenterPos,
        const FmMatrix3& boxRotation)
    {
        FmTetsIntersectingBoxQueryState queryState;
        queryState.tetMesh = NULL;
        queryState.vertRestPositions = vertRestPositions;
        queryState.tetVertIds = tetVertIds;
        queryState.bvh = bvh;
        queryState.boxHalfDimensions = boxHalfDimensions;
        queryState.boxCenterPos = boxCenterPos;
        queryState.boxRotation = boxRotation;
        queryState.boxRotationInv = transpose(boxRotation);
        queryState.boxPoints[0] = FmInitVector3(-boxHalfDimensions.x, -boxHalfDimensions.y, -boxHalfDimensions.z);
        queryState.boxPoints[1] = FmInitVector3(boxHalfDimensions.x, -boxHalfDimensions.y, -boxHalfDimensions.z);
        queryState.boxPoints[2] = FmInitVector3(-boxHalfDimensions.x, boxHalfDimensions.y, -boxHalfDimensions.z);
        queryState.boxPoints[3] = FmInitVector3(boxHalfDimensions.x, boxHalfDimensions.y, -boxHalfDimensions.z);
        queryState.boxPoints[4] = FmInitVector3(-boxHalfDimensions.x, -boxHalfDimensions.y, boxHalfDimensions.z);
        queryState.boxPoints[5] = FmInitVector3(boxHalfDimensions.x, -boxHalfDimensions.y, boxHalfDimensions.z);
        queryState.boxPoints[6] = FmInitVector3(-boxHalfDimensions.x, boxHalfDimensions.y, boxHalfDimensions.z);
        queryState.boxPoints[7] = FmInitVector3(boxHalfDimensions.x, boxHalfDimensions.y, boxHalfDimensions.z);
        FmVector3 absCol0 = abs(boxRotation.col0);
        FmVector3 absCol1 = abs(boxRotation.col1);
        FmVector3 absCol2 = abs(boxRotation.col2);
        queryState.aabbMinPos = boxCenterPos - absCol0 * boxHalfDimensions.x - absCol1 * boxHalfDimensions.y - absCol2 * boxHalfDimensions.z;
        queryState.aabbMaxPos = boxCenterPos + absCol0 * boxHalfDimensions.x + absCol1 * boxHalfDimensions.y + absCol2 * boxHalfDimensions.z;
        queryState.tets = tetsIntersectingBox;
        queryState.numTets = 0;

        FmFindTetsIntersectingBoxRecursive(queryState, 0);

        return queryState.numTets;
    }

    struct FmSceneCollisionPlanesContactsQueryState
    {
        FmCollidedObjectPair* objectPair;
        FmSceneCollisionPlanes collisionPlanes;
    };

    void FmFindSceneCollisionPlanesContactsRecursive(FmSceneCollisionPlanesContactsQueryState& queryState, uint idx)
    {
        FmCollidedObjectPair* objectPair = queryState.objectPair;
        const FmBvh* hierarchy = objectPair->objectAHierarchy;
        FmSceneCollisionPlanes& collisionPlanes = queryState.collisionPlanes;
        FmBvhNode* nodes = hierarchy->nodes;
        float timestep = objectPair->timestep;
        float distContactThreshold = objectPair->distContactThreshold;

        // Return if AABB shows no potential contact

        FmAabb& box = hierarchy->nodes[idx].box;
        bool results[6];

        float threshold = distContactThreshold * 0.5f; // distContactThreshold - AABB padding
        if (!FmAabbSceneCollisionPlanesPotentialContact(results, box, collisionPlanes, timestep, threshold))
        {
            return;
        }

        int lc = nodes[idx].left;
        int rc = nodes[idx].right;

        if (lc == rc)
        {
            // Leaf, check for contact
            uint exteriorFaceId = (uint)lc;

            FmGenerateFaceCollisionPlaneContacts(objectPair, results, exteriorFaceId, collisionPlanes);
        }
        else
        {
            FmFindSceneCollisionPlanesContactsRecursive(queryState, (uint)lc);
            FmFindSceneCollisionPlanesContactsRecursive(queryState, (uint)rc);
        }
    }

    uint FmFindSceneCollisionPlanesContactsQuery(
        FmCollidedObjectPair* objectPair,
        const FmSceneCollisionPlanes& collisionPlanes)
    {
        uint numContactsStart = objectPair->numDistanceContacts;

        FmSceneCollisionPlanesContactsQueryState queryState;
        queryState.objectPair = objectPair;
        queryState.collisionPlanes = collisionPlanes;

        FmFindSceneCollisionPlanesContactsRecursive(queryState, 0);

        return objectPair->numDistanceContacts - numContactsStart;
    }

    void FmCollideRecursive(FmCollidedObjectPair* objectPair, int indexA, int indexB)
    {
        FmBvh* hierarchyA = objectPair->objectAHierarchy;
        FmBvh* hierarchyB = objectPair->objectBHierarchy;
        float timestep = objectPair->timestep;

        FmBvhNode* nodeA = &hierarchyA->nodes[indexA];
        FmBvhNode* nodeB = &hierarchyB->nodes[indexB];
        FmAabb* boxA = &hierarchyA->nodes[indexA].box;
        FmAabb* boxB = &hierarchyB->nodes[indexB].box;
        bool nodeAIsLeaf = nodeA->left == nodeA->right;
        bool nodeBIsLeaf = nodeB->left == nodeB->right;
        float impactTime;

        if (FmAabbCcd(impactTime, *boxA, *boxB, timestep))
        {
            if (nodeAIsLeaf && nodeBIsLeaf)
            {
#if FM_SOA_TRI_INTERSECTION
                FmMeshCollisionTriPair& pair = objectPair->temps.meshCollisionTriPairs[objectPair->temps.numMeshCollisionTriPairs];
                pair.exteriorFaceIdA = (uint)nodeA->left;
                pair.exteriorFaceIdB = (uint)nodeB->left;
                objectPair->temps.numMeshCollisionTriPairs++;
                if (objectPair->temps.numMeshCollisionTriPairs >= FM_MAX_MESH_COLLISION_TRI_PAIR)
                {
                    FmGenerateContactsFromMeshCollisionTriPairs(objectPair);
                    objectPair->temps.numMeshCollisionTriPairs = 0;
                }
#else
                FmGenerateContacts(objectPair, nodeA->left, nodeB->left);
#endif
            }
            else if (nodeAIsLeaf)
            {
                FmCollideRecursive(objectPair, indexA, nodeB->left);
                FmCollideRecursive(objectPair, indexA, nodeB->right);
            }
            else if (nodeBIsLeaf)
            {
                FmCollideRecursive(objectPair, nodeA->left, indexB);
                FmCollideRecursive(objectPair, nodeA->right, indexB);
            }
            else
            {
                float sizeA = lengthSqr(boxA->pmax - boxA->pmin);
                float sizeB = lengthSqr(boxB->pmax - boxB->pmin);

                // Split the larger node
                if (sizeA > sizeB)
                {
                    FmCollideRecursive(objectPair, nodeA->left, indexB);
                    FmCollideRecursive(objectPair, nodeA->right, indexB);
                }
                else
                {
                    FmCollideRecursive(objectPair, indexA, nodeB->left);
                    FmCollideRecursive(objectPair, indexA, nodeB->right);
                }
            }
        }
    }

    static FM_FORCE_INLINE bool FmAabbOverlapXY(const FmAabb& aabb0, const FmAabb& aabb1)
    {
        return
            aabb0.pmin.x <= aabb1.pmax.x &&
            aabb0.pmin.y <= aabb1.pmax.y &&
            aabb1.pmin.x <= aabb0.pmax.x &&
            aabb1.pmin.y <= aabb0.pmax.y;
    }

    void FmFindInsideVertsRecursive(FmCollidedObjectPair* objectPair, int indexA, int indexB, bool includeA, bool includeB)
    {
        FmBvh* hierarchyA = objectPair->objectAHierarchy;
        FmBvh* hierarchyB = objectPair->objectBHierarchy;

        FmBvhNode* nodeA = &hierarchyA->nodes[indexA];
        FmBvhNode* nodeB = &hierarchyB->nodes[indexB];
        FmAabb* boxA = &hierarchyA->nodes[indexA].box;
        FmAabb* boxB = &hierarchyB->nodes[indexB].box;
        bool nodeAIsLeaf = nodeA->left == nodeA->right;
        bool nodeBIsLeaf = nodeB->left == nodeB->right;

        // If node found to be outside of other mesh bounding box, do not need to test any of its vertices as inside.
        // Pass flag down to all children tests to prevent computing intersection values.
        includeA = includeA && FmAabbOverlap(boxA->pmin, boxA->pmax, objectPair->objectBMinPosition, objectPair->objectBMaxPosition);
        includeB = includeB && FmAabbOverlap(boxB->pmin, boxB->pmax, objectPair->objectAMinPosition, objectPair->objectAMaxPosition);

        if ((includeA || includeB) && FmAabbOverlapXY(*boxA, *boxB))
        {
            if (nodeAIsLeaf && nodeBIsLeaf)
            {
                FmUpdateVertIntersectionVals(objectPair, (uint)nodeA->left, (uint)nodeB->left, includeA, includeB);
            }
            else if (nodeAIsLeaf)
            {
                FmFindInsideVertsRecursive(objectPair, indexA, nodeB->left, includeA, includeB);
                FmFindInsideVertsRecursive(objectPair, indexA, nodeB->right, includeA, includeB);
            }
            else if (nodeBIsLeaf)
            {
                FmFindInsideVertsRecursive(objectPair, nodeA->left, indexB, includeA, includeB);
                FmFindInsideVertsRecursive(objectPair, nodeA->right, indexB, includeA, includeB);
            }
            else
            {
                float sizeA = lengthSqr(boxA->pmax - boxA->pmin);
                float sizeB = lengthSqr(boxB->pmax - boxB->pmin);

                // Split the larger node
                if (sizeA > sizeB)
                {
                    FmFindInsideVertsRecursive(objectPair, nodeA->left, indexB, includeA, includeB);
                    FmFindInsideVertsRecursive(objectPair, nodeA->right, indexB, includeA, includeB);
                }
                else
                {
                    FmFindInsideVertsRecursive(objectPair, indexA, nodeB->left, includeA, includeB);
                    FmFindInsideVertsRecursive(objectPair, indexA, nodeB->right, includeA, includeB);
                }
            }
        }
    }

    void FmFindInsideVerts(FmCollidedObjectPair* meshPair)
    {
        FmFindInsideVertsRecursive(meshPair, 0, 0, true, true);
    }

    void FmCollideHierarchies(FmCollidedObjectPair* meshPair)
    {
        FmCollideRecursive(meshPair, 0, 0);

#if FM_SOA_TRI_INTERSECTION
        FmGenerateContactsFromMeshCollisionTriPairs(meshPair);
        meshPair->temps.numMeshCollisionTriPairs = 0;
#endif
    }

}