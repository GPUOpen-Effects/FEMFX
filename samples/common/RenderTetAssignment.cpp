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
// Sample code to compute correspondence between render vertices and FEM mesh tetrahedra
//---------------------------------------------------------------------------------------

#include "RenderTetAssignment.h"
#include "AMD_FEMFX.h"

namespace AMD
{
    // Intersection function for edge A and triangle B intersecting in 3D
    int FmIntersectionX12(
        FmVector3* intersectionPoint,
        const FmVector3& eA0, const FmVector3& eA1,
        const FmVector3& fB0, const FmVector3& fB1, const FmVector3& fB2);

    NonFractureRegions::NonFractureRegions(int numTets)
    {
        for (int tetIdx = 0; tetIdx < numTets; tetIdx++)
        {
            regionIdOfTet.push_back(FM_INVALID_ID);
            nonFractureFlagsOfTet.push_back(0);
        }
    }

    NonFractureRegions::~NonFractureRegions()
    {
        int numRegions = (int)regions.size();
        for (int regionIdx = 0; regionIdx < numRegions; regionIdx++)
        {
            regions[regionIdx].Destroy();
        }
    }

    // Pass in all tet mesh vertex positions and tet vertex ids, and list of tet ids in the non-fracture region.
    // Create ids lists and BVH for the non-fracture region.
    void NonFractureRegionData::Create(FmVector3* meshVertRestPositions, FmTetVertIds* meshTetVertIds, const std::vector<uint>& inTetIds)
    {
        int numTetsInRegion = (int)inTetIds.size();

        if (numTetsInRegion == 0)
        {
            return;
        }

        if (tetIds)
        {
            Destroy();
        }

        tetIds = new uint[numTetsInRegion];
        tetVertIds = new FmTetVertIds[numTetsInRegion];

        for (int regionTetIdx = 0; regionTetIdx < numTetsInRegion; regionTetIdx++)
        {
            uint tetId = tetIds[regionTetIdx] = inTetIds[regionTetIdx];

            tetVertIds[regionTetIdx] = meshTetVertIds[tetId];
        }

        bvh = FmCreateBvh(numTetsInRegion);
        FmBuildRestMeshTetBvh(bvh, meshVertRestPositions, tetVertIds, numTetsInRegion);

        numTets = numTetsInRegion;
    }

    void NonFractureRegionData::Destroy()
    {
        delete[] tetIds;
        delete[] tetVertIds;
        FmDestroyBvh(bvh);
    }

    // Create new multi-tet region data from tet mesh vertex positions and tet vertex ids, and list of tet ids in the non-fracture region.
    void NonFractureRegions::AddRegion(FmVector3* meshVertRestPositions, FmTetVertIds* meshTetVertIds, const std::vector<uint>& inTetIds, const std::vector<uint>& inTetNoFractureFlags)
    {
        int numTetsInRegion = (int)inTetIds.size();

        if (numTetsInRegion == 0)
        {
            return;
        }

        int regionId = (int)regions.size();

        // Create multi-tet region for set of tets
        NonFractureRegionData newRegionData;
        newRegionData.Create(meshVertRestPositions, meshTetVertIds, inTetIds);
        regions.push_back(newRegionData);

        // Set the region ids for these tets
        for (int regionTetIdx = 0; regionTetIdx < numTetsInRegion; regionTetIdx++)
        {
            int tetId = inTetIds[regionTetIdx];
            int nonFractureFlags = inTetNoFractureFlags[regionTetIdx];
            regionIdOfTet[tetId] = regionId;
            nonFractureFlagsOfTet[tetId] = nonFractureFlags;
        }
    }

    // After creating all multi-tet regions, remaining single tets are assigned higher region ids
    void NonFractureRegions::AddRemainingRegions(FmVector3* meshVertRestPositions, FmTetVertIds* meshTetVertIds)
    {
        int numTets = (int)regionIdOfTet.size();
        for (int tetId = 0; tetId < numTets; tetId++)
        {
            if (regionIdOfTet[tetId] == FM_INVALID_ID)
            {
                regionIdOfTet[tetId] = (uint)regions.size();

                std::vector<uint> tetIds;
                tetIds.push_back(tetId);

                std::vector<uint> tetNoFractureFlags;
                tetNoFractureFlags.push_back(0);

                AddRegion(meshVertRestPositions, meshTetVertIds, tetIds, tetNoFractureFlags);
            }
        }
    }

    // Group shard vertices if they have the same array of tet assignments. 
    void CreateShardVertGroups(ShardVertGroups* resultShardVertGroups, const ShardVertTetAssignments* shardVertTetAssignments, uint numShardVerts)
    {
        resultShardVertGroups->clear();

        // Iterate over shard vertices, create key and find existing group or insert new one
        for (uint shardVertId = 0; shardVertId < numShardVerts; shardVertId++)
        {
            ShardVertGroupKey key;
            key.shardVertTetAssignments = &shardVertTetAssignments[shardVertId];

            ShardVertGroups::iterator it = resultShardVertGroups->find(key);
            if (it == resultShardVertGroups->end())
            {
                // Create new group with shardVertId as first member
                std::pair< ShardVertGroupKey, ShardVertGroup > element;
                element.first = key;
                element.second.shardVertIndices.push_back(shardVertId);

                resultShardVertGroups->insert(element);
            }
            else
            {
                // Add shardVertId to existing group
                it->second.shardVertIndices.push_back(shardVertId);
            }
        }
    }

    // Create render data from shard vertex groups
    void CreateShardRenderData(
        std::vector<uint>* outBaryPosOffsets,                          // Output array of baryPosOffsets matching the shard vertex groups
        std::vector<uint>* outBaryPosOffsetIds,                        // Output ids of baryPosOffsets for all render vertices
        TetFractureShardVerticesToUpdate* outTetFractureShardVertices, // Output map from tet fracture faces to groups/baryPosOffset; array assumed presized for all tets
        ShardVertGroups* shardVertGroups,        // Shard vertex groups created by CreateShardVertGroups()
        uint numRenderVertices,
        const uint* shardVertIds,                // Map from render vertex to shard vertex; will remap to baryPosOffset index
        const ShardVertTetAssignments* shardVertTetAssignments)
    {
        // Create a bary pos offset for each group
        outBaryPosOffsets->clear();

        uint idx = 0;
        for (ShardVertGroups::iterator it = shardVertGroups->begin(); it != shardVertGroups->end(); it++)
        {
            // Key points to an array of tet assignnments which gives the current offset
            const ShardVertTetAssignments& shardVert = *it->first.shardVertTetAssignments;
            outBaryPosOffsets->push_back(shardVert.numTets - 1);

            // Set index
            it->second.idx = idx++;
        }

        // Iterate over the render vertices and use the shard vertex to find the group / bary pos offset.
        outBaryPosOffsetIds->clear();

        for (uint renderVertexId = 0; renderVertexId < numRenderVertices; renderVertexId++)
        {
            uint shardVertId = shardVertIds[renderVertexId];

            ShardVertGroupKey key;
            key.shardVertTetAssignments = &shardVertTetAssignments[shardVertId];

            uint baryPosOffsetId = (uint)-1;

            ShardVertGroups::const_iterator it = shardVertGroups->find(key);
            if (it != shardVertGroups->end())
            {
                baryPosOffsetId = it->second.idx;
            }

            outBaryPosOffsetIds->push_back(baryPosOffsetId);
        }

        for (ShardVertGroups::const_iterator it = shardVertGroups->begin(); it != shardVertGroups->end(); it++)
        {
            // Key points to an array of tet assignnments which gives the current offset
            const ShardVertTetAssignments& shardTetAssignments = *it->first.shardVertTetAssignments;

            uint groupIdx = it->second.idx;

            // Create map from each tet face to the shard vertex that must be updated, and the offset for that tet assignment
            for (int tetIdx = 0; tetIdx < (int)shardTetAssignments.numTets; tetIdx++)
            {
                unsigned int tetId = shardTetAssignments.tetAssignments[tetIdx].tetId;
                unsigned int faceId = shardTetAssignments.tetAssignments[tetIdx].nearestFaceId;

                std::vector<ShardVertUpdateInfo>& faceShardVertices = outTetFractureShardVertices[tetId].GetShardVerticesOfFace(faceId);

                ShardVertUpdateInfo updateInfo;
                updateInfo.shardVertexId = groupIdx;
                updateInfo.meshSectionId = 0;
                updateInfo.baryPosBaseId = 0;
                updateInfo.baryPosOffset = tetIdx;

                faceShardVertices.push_back(updateInfo);
            }
        }
    }

    void ComputeRenderVertTetAssignment(RenderVertTetAssignment* tetAssignment, const FmTetMeshBuffer& tetMeshBuffer, const FmBvh& bvh, const FmVector3& renderPos)
    {
        // Use before initial fracture
        assert(FmGetNumTetMeshes(tetMeshBuffer) == 1);

        FmTetMesh& tetMesh = *FmGetTetMesh(tetMeshBuffer, 0);

        FmClosestTetResult closestTet;
        FmFindClosestTet(&closestTet, &tetMesh, &bvh, renderPos);
        uint minTetId = closestTet.tetId;
        uint minFaceId = closestTet.faceId;
        bool inside = closestTet.insideTet;

        FmVector4 barycentrics = mul(FmGetTetRestBaryMatrix(tetMesh, minTetId), FmVector4(renderPos, 1.0f));

        tetAssignment->tetId = minTetId;
        tetAssignment->nearestFaceId = minFaceId;
        tetAssignment->barycentricCoords[0] = barycentrics.x;
        tetAssignment->barycentricCoords[1] = barycentrics.y;
        tetAssignment->barycentricCoords[2] = barycentrics.z;
        tetAssignment->barycentricCoords[3] = barycentrics.w;
        tetAssignment->inside = inside;
    }

    void ComputeRenderVertTetAssignment(
        RenderVertTetAssignment* tetAssignment,
        const FmVector3* vertRestPositions,
        const FmTetVertIds* tetVertIds,
        const FmBvh& bvh,
        const FmVector3& renderPos)
    {
        FmClosestTetResult closestTet;
        FmFindClosestTet(&closestTet, vertRestPositions, tetVertIds, &bvh, renderPos);
        uint minTetId = closestTet.tetId;
        uint minFaceId = closestTet.faceId;
        bool inside = closestTet.insideTet;

        FmVector4 barycentrics = FmComputeBarycentricCoords(vertRestPositions, tetVertIds[minTetId], renderPos);

        tetAssignment->tetId = minTetId;
        tetAssignment->nearestFaceId = minFaceId;
        tetAssignment->barycentricCoords[0] = barycentrics.x;
        tetAssignment->barycentricCoords[1] = barycentrics.y;
        tetAssignment->barycentricCoords[2] = barycentrics.z;
        tetAssignment->barycentricCoords[3] = barycentrics.w;
        tetAssignment->inside = inside;
    }

    void ComputeShardVertTetAssignments(ShardVertTetAssignmentsFixedNum* tetAssignments, const FmTetMeshBuffer& tetMeshBuffer, const FmBvh& bvh, const FmVector3& renderPos, const FmVector3& rootPos)
    {
        // Use before initial fracture
        assert(FmGetNumTetMeshes(tetMeshBuffer) == 1);

        FmTetMesh& tetMesh = *FmGetTetMesh(tetMeshBuffer, 0);

        ShardVertTetAssignmentsFixedNum assignments;
        assignments.numTets = 0;

        FmClosestTetResult closestTet;
        FmFindIntersectedTet(&closestTet, &tetMesh, &bvh, rootPos);
        uint tetId = closestTet.tetId;
        bool inside = closestTet.insideTet;

        // If rootPos inside tet, search line from root pos to render pos to created ordered list of tet assignments.
        if (inside)
        {
            FmTetVertIds tetVertIds = FmGetTetVertIds(tetMesh, tetId);

            FmVector3 tetPos[4];
            tetPos[0] = FmGetVertRestPosition(tetMesh, tetVertIds.ids[0]);
            tetPos[1] = FmGetVertRestPosition(tetMesh, tetVertIds.ids[1]);
            tetPos[2] = FmGetVertRestPosition(tetMesh, tetVertIds.ids[2]);
            tetPos[3] = FmGetVertRestPosition(tetMesh, tetVertIds.ids[3]);

            uint searchTetId = tetId;
            bool exitedMesh = false;
            bool renderPosInsideTet = false;

            do
            {
                FmVector4 barycentrics = mul(FmGetTetRestBaryMatrix(tetMesh, searchTetId), FmVector4(renderPos, 1.0f));

                RenderVertTetAssignment& tetAssignment = assignments.tetAssignments[assignments.numTets];
                assignments.numTets++;

                tetAssignment.tetId = searchTetId;
                tetAssignment.barycentricCoords[0] = barycentrics.x;
                tetAssignment.barycentricCoords[1] = barycentrics.y;
                tetAssignment.barycentricCoords[2] = barycentrics.z;
                tetAssignment.barycentricCoords[3] = barycentrics.w;

                uint exitFaceId = 0;
                renderPosInsideTet = true;

                tetVertIds = FmGetTetVertIds(tetMesh, searchTetId);
                tetPos[0] = FmGetVertRestPosition(tetMesh, tetVertIds.ids[0]);
                tetPos[1] = FmGetVertRestPosition(tetMesh, tetVertIds.ids[1]);
                tetPos[2] = FmGetVertRestPosition(tetMesh, tetVertIds.ids[2]);
                tetPos[3] = FmGetVertRestPosition(tetMesh, tetVertIds.ids[3]);

                for (uint faceId = 0; faceId < 4; faceId++)
                {
                    FmFaceVertIds tetCorners;
                    FmGetFaceTetCorners(&tetCorners, faceId);
                    FmVector3 triPos0 = tetPos[tetCorners.ids[0]];
                    FmVector3 triPos1 = tetPos[tetCorners.ids[1]];
                    FmVector3 triPos2 = tetPos[tetCorners.ids[2]];

                    FmVector3 X12IntersectionPoint;
                    int X12IntersectionVal = FmIntersectionX12(&X12IntersectionPoint, rootPos, renderPos, triPos0, triPos1, triPos2);

                    if (X12IntersectionVal > 0)  // exiting tet
                    {
                        exitFaceId = faceId;
                        renderPosInsideTet = false;

                        FmTetFaceIncidentTetIds faceIncidentTetIds = FmGetTetFaceIncidentTetIds(tetMesh, searchTetId);

                        uint incidentTetId = faceIncidentTetIds.ids[faceId];
                        if (FmIsExteriorFaceId(incidentTetId))
                        {
                            exitedMesh = true;
                        }

                        // step search to incident tet
                        searchTetId = incidentTetId;

                        break;
                    }
                }

                // Finish tet assignment fields
                tetAssignment.nearestFaceId = exitFaceId;
                tetAssignment.inside = renderPosInsideTet;

            } while (!exitedMesh && !renderPosInsideTet && assignments.numTets < MAX_RENDER_VERT_TET_ASSIGNMENTS);
        }

        if (assignments.numTets == 0)
        {
            ComputeRenderVertTetAssignment(&assignments.tetAssignments[0], tetMeshBuffer, bvh, rootPos);
            assignments.numTets++;
        }

        // Removes assignments beyond first exterior face hit
        UpdateShardVertTetAssignments(&assignments, tetMeshBuffer);

        *tetAssignments = assignments;
    }

    void ComputeShardVertTetAssignments(
        ShardVertTetAssignmentsFixedNum* tetAssignments,
        const FmVector3* vertRestPositions,
        const FmTetVertIds* tetVertIds,
        const FmTetFaceIncidentTetIds* tetFaceIncidentTetIds,
        const FmBvh& bvh,
        const FmVector3& renderPos, const FmVector3& rootPos)
    {
        ShardVertTetAssignmentsFixedNum assignments;
        assignments.numTets = 0;

        FmClosestTetResult closestTet;
        FmFindIntersectedTet(&closestTet, vertRestPositions, tetVertIds, &bvh, rootPos);
        uint tetId = closestTet.tetId;
        bool inside = closestTet.insideTet;

        // If rootPos inside tet, search line from root pos to render pos to created ordered list of tet assignments.
        if (inside)
        {
            FmTetVertIds tetVerts = tetVertIds[tetId];

            FmVector3 tetPos[4];
            tetPos[0] = vertRestPositions[tetVerts.ids[0]];
            tetPos[1] = vertRestPositions[tetVerts.ids[1]];
            tetPos[2] = vertRestPositions[tetVerts.ids[2]];
            tetPos[3] = vertRestPositions[tetVerts.ids[3]];

            uint searchTetId = tetId;
            bool exitedMesh = false;
            bool renderPosInsideTet = false;

            do
            {
                FmVector4 barycentrics = FmComputeBarycentricCoords(vertRestPositions, tetVertIds[searchTetId], renderPos);

                RenderVertTetAssignment& tetAssignment = assignments.tetAssignments[assignments.numTets];
                assignments.numTets++;

                tetAssignment.tetId = searchTetId;
                tetAssignment.barycentricCoords[0] = barycentrics.x;
                tetAssignment.barycentricCoords[1] = barycentrics.y;
                tetAssignment.barycentricCoords[2] = barycentrics.z;
                tetAssignment.barycentricCoords[3] = barycentrics.w;

                uint exitFaceId = 0;
                renderPosInsideTet = true;

                tetVerts = tetVertIds[searchTetId];

                tetPos[0] = vertRestPositions[tetVerts.ids[0]];
                tetPos[1] = vertRestPositions[tetVerts.ids[1]];
                tetPos[2] = vertRestPositions[tetVerts.ids[2]];
                tetPos[3] = vertRestPositions[tetVerts.ids[3]];

                for (uint faceId = 0; faceId < 4; faceId++)
                {
                    FmFaceVertIds tetCorners;
                    FmGetFaceTetCorners(&tetCorners, faceId);
                    FmVector3 triPos0 = tetPos[tetCorners.ids[0]];
                    FmVector3 triPos1 = tetPos[tetCorners.ids[1]];
                    FmVector3 triPos2 = tetPos[tetCorners.ids[2]];

                    FmVector3 X12IntersectionPoint;
                    int X12IntersectionVal = FmIntersectionX12(&X12IntersectionPoint, rootPos, renderPos, triPos0, triPos1, triPos2);

                    if (X12IntersectionVal > 0)  // exiting tet
                    {
                        exitFaceId = faceId;
                        renderPosInsideTet = false;

                        const FmTetFaceIncidentTetIds& faceIncidentTetIds = tetFaceIncidentTetIds[searchTetId];
                        uint incidentTetId = faceIncidentTetIds.ids[faceId];
                        if (FmIsExteriorFaceId(incidentTetId))
                        {
                            exitedMesh = true;
                        }

                        // step search to incident tet
                        searchTetId = incidentTetId;

                        break;
                    }
                }

                // Finish tet assignment fields
                tetAssignment.nearestFaceId = exitFaceId;
                tetAssignment.inside = renderPosInsideTet;

            } while (!exitedMesh && !renderPosInsideTet && assignments.numTets < MAX_RENDER_VERT_TET_ASSIGNMENTS);
        }

        if (assignments.numTets == 0)
        {
            ComputeRenderVertTetAssignment(&assignments.tetAssignments[0], vertRestPositions, tetVertIds, bvh, rootPos);
            assignments.numTets++;
        }

        // Remove assignments beyond first exterior face hit
        int numAssignments = (int)assignments.numTets;

        for (int assignmentId = 0; assignmentId < numAssignments - 1; assignmentId++)
        {
            RenderVertTetAssignment& tetAssignment = assignments.tetAssignments[assignmentId];

            if (tetAssignment.inside)
            {
                break;
            }

            const FmTetFaceIncidentTetIds& tetFaceIncidentTets = tetFaceIncidentTetIds[tetAssignment.tetId];

            if (FmIsExteriorFaceId(tetFaceIncidentTets.ids[tetAssignment.nearestFaceId]))
            {
                assignments.numTets = assignmentId + 1;
                break;
            }
        }

        *tetAssignments = assignments;
    }

    bool UpdateShardVertTetAssignments(ShardVertTetAssignmentsFixedNum* tetAssignments, const FmTetMeshBuffer& tetMeshBuffer)
    {
        uint numAssignments = tetAssignments->numTets;

        if (numAssignments <= 1)
        {
            return false;
        }

        for (uint assignmentId = 0; assignmentId < numAssignments - 1; assignmentId++)
        {
            RenderVertTetAssignment& tetAssignment = tetAssignments->tetAssignments[assignmentId];

            if (tetAssignment.inside)
            {
                return false;
            }

            uint tetId, meshIdx;
            FmTetMesh* tetMesh = FmGetTetMeshContainingTet(&tetId, &meshIdx, tetMeshBuffer, tetAssignment.tetId);

            FmTetFaceIncidentTetIds tetFaceIncidentTetIds = FmGetTetFaceIncidentTetIds(*tetMesh, tetId);

            if (FmIsExteriorFaceId(tetFaceIncidentTetIds.ids[tetAssignment.nearestFaceId]))
            {
                tetAssignments->numTets = assignmentId + 1;
                return true;
            }
        }
        return false;
    }

    void GetMeshBufferRenderCounts(uint* numVerts, uint* numTets, const FmTetMeshBuffer& tetMeshBuffer)
    {
        uint numMeshes = FmGetNumTetMeshes(tetMeshBuffer);

        uint totalVerts = 0;
        for (uint meshIdx = 0; meshIdx < numMeshes; meshIdx++)
        {
            uint numMeshVerts = FmGetNumVerts(*FmGetTetMesh(tetMeshBuffer, meshIdx));
            totalVerts += numMeshVerts;
        }
        *numVerts = totalVerts;
        *numTets = FmGetNumTets(tetMeshBuffer);
    }

    void GetMeshBufferRenderData(FmVector3* vertices, FmTetVertIds* tetVertIds, FmMatrix3* tetRotations, const FmTetMeshBuffer& tetMeshBuffer)
    {
        uint numMeshes = FmGetNumTetMeshes(tetMeshBuffer);

        uint* tetMeshVertOffsets = new uint[numMeshes];

        uint vertOffset = 0;
        for (uint meshIdx = 0; meshIdx < numMeshes; meshIdx++)
        {
            FmTetMesh& tetMesh = *FmGetTetMesh(tetMeshBuffer, meshIdx);

            tetMeshVertOffsets[meshIdx] = vertOffset;

            uint numVerts = FmGetNumVerts(tetMesh);
            for (uint vId = 0; vId < numVerts; vId++)
            {
                vertices[vertOffset] = FmGetVertPosition(tetMesh, vId);
                vertOffset++;
            }
        }

        uint numTets = FmGetNumTets(tetMeshBuffer);

        for (uint bufferTetId = 0; bufferTetId < numTets; bufferTetId++)
        {
            uint tetId, meshIdx;
            FmTetMesh* tetMesh = FmGetTetMeshContainingTet(&tetId, &meshIdx, tetMeshBuffer, bufferTetId);

            uint meshVertOffset = tetMeshVertOffsets[meshIdx];

            tetVertIds[bufferTetId] = FmGetTetVertIds(*tetMesh, tetId);
            tetVertIds[bufferTetId].ids[0] += meshVertOffset;
            tetVertIds[bufferTetId].ids[1] += meshVertOffset;
            tetVertIds[bufferTetId].ids[2] += meshVertOffset;
            tetVertIds[bufferTetId].ids[3] += meshVertOffset;

            tetRotations[bufferTetId] = FmGetTetRotation(*tetMesh, tetId);
        }

        delete [] tetMeshVertOffsets;
    }

    // Associate this render vertex with the non-fracture region nearest to rootPos.
    // Find nearest tet to renderPos in this non-fracture region, then walk to renderPos, recording all tets/faces crossed which can fracture.
    // This data can be used to switch the tet assignment after these faces break, to keep the render vertex from stretching.
    void ComputeShardVertTetAssignments(
        std::vector<RenderVertTetAssignment>* outputTetAssignments,
        const NonFractureRegions& nonFractureRegions,
        const FmTetMeshBuffer& tetMeshBuffer,
        FmVector3* vertRestPositions,
        const FmBvh& bvh, const FmVector3& renderPos, const FmVector3& rootPos)
    {
        outputTetAssignments->clear();

        const FmTetMesh& tetMesh = *FmGetTetMesh(tetMeshBuffer, 0);

        // Find nearest non-fracture region to rootPos
        FmClosestTetResult rootPosTetResult;

        FmFindClosestTet(&rootPosTetResult, &tetMesh, &bvh, rootPos);

        int rootNonFractureRegionId = nonFractureRegions.regionIdOfTet[rootPosTetResult.tetId];

        // Initial tet assignment for render vertex is closest point of non-fracture region to renderPos
        FmClosestTetResult renderPosTetResult;
        FmFindClosestTet(&renderPosTetResult, vertRestPositions,
            nonFractureRegions.regions[rootNonFractureRegionId].tetVertIds,
            nonFractureRegions.regions[rootNonFractureRegionId].bvh,
            renderPos);

        // Map from non-fracture region tet id to mesh tet id
        renderPosTetResult.tetId = nonFractureRegions.regions[rootNonFractureRegionId].tetIds[renderPosTetResult.tetId];

        RenderVertTetAssignment rootTetAssignment;
        rootTetAssignment.tetId = renderPosTetResult.tetId;
        rootTetAssignment.nearestFaceId = renderPosTetResult.faceId;
        rootTetAssignment.barycentricCoords[0] = renderPosTetResult.posBary[0];
        rootTetAssignment.barycentricCoords[1] = renderPosTetResult.posBary[1];
        rootTetAssignment.barycentricCoords[2] = renderPosTetResult.posBary[2];
        rootTetAssignment.barycentricCoords[3] = renderPosTetResult.posBary[3];
        rootTetAssignment.inside = renderPosTetResult.insideTet;

        // If the renderPos was inside this tet, output the first tet assignment
        if (rootTetAssignment.inside)
        {
            outputTetAssignments->push_back(rootTetAssignment);
            return;
        }

        // Walk from the root tet to the renderPos and record all the fracturable face crossings.
        // Each will give a possible fall-back tet assignment after fracture occurs.

        FmTetVertIds tetVerts = FmGetTetVertIds(tetMesh, rootTetAssignment.tetId);

        FmVector3 tetPos[4];
        tetPos[0] = vertRestPositions[tetVerts.ids[0]];
        tetPos[1] = vertRestPositions[tetVerts.ids[1]];
        tetPos[2] = vertRestPositions[tetVerts.ids[2]];
        tetPos[3] = vertRestPositions[tetVerts.ids[3]];

        FmVector3 startPos = (tetPos[0] + tetPos[1] + tetPos[2] + tetPos[3]) * 0.25f;

        RenderVertTetAssignment currentTetAssignment = rootTetAssignment;
        int currentTetId = rootTetAssignment.tetId;
        bool done = false;

        do
        {
            // Check current tet to see which face is crossed moving towards the renderPos
            uint exitFaceId = 0;
            uint incidentTetId = FM_INVALID_ID;
            bool isFracturableFace = false;
            bool exitedMesh = false;
            bool exitedTet = false;

            tetVerts = FmGetTetVertIds(tetMesh, currentTetId);

            tetPos[0] = vertRestPositions[tetVerts.ids[0]];
            tetPos[1] = vertRestPositions[tetVerts.ids[1]];
            tetPos[2] = vertRestPositions[tetVerts.ids[2]];
            tetPos[3] = vertRestPositions[tetVerts.ids[3]];

            for (uint faceId = 0; faceId < 4; faceId++)
            {
                FmFaceVertIds tetCorners;
                FmGetFaceTetCorners(&tetCorners, faceId);
                FmVector3 triPos0 = tetPos[tetCorners.ids[0]];
                FmVector3 triPos1 = tetPos[tetCorners.ids[1]];
                FmVector3 triPos2 = tetPos[tetCorners.ids[2]];

                FmVector3 X12IntersectionPoint;
                int X12IntersectionVal = FmIntersectionX12(&X12IntersectionPoint, startPos, renderPos, triPos0, triPos1, triPos2);

                if (X12IntersectionVal > 0)  // exiting tet
                {
                    exitFaceId = faceId;
                    exitedTet = true;

                    isFracturableFace = (nonFractureRegions.nonFractureFlagsOfTet[currentTetId] & (1 << exitFaceId)) == 0;

                    FmTetFaceIncidentTetIds faceIncidentTetIds = FmGetTetFaceIncidentTetIds(tetMesh, currentTetId);
                    incidentTetId = faceIncidentTetIds.ids[faceId];
                    if (FmIsExteriorFaceId(incidentTetId))
                    {
                        exitedMesh = true;
                    }

                    break;
                }
            }

            if (!exitedTet || exitedMesh)
            {
                done = true;
            }

            // Create tet assignment and add if search finished, or if crossed a fracturable face
            if (done || (exitedTet && isFracturableFace))
            {
                currentTetAssignment.tetId = currentTetId;
                currentTetAssignment.nearestFaceId = exitFaceId;
                currentTetAssignment.inside = !exitedTet;

                // If not the root tet, compute barycentric values for the renderPos
                FmVector4 barycentrics = FmComputeBarycentricCoords(vertRestPositions, tetVerts, renderPos);

                currentTetAssignment.barycentricCoords[0] = barycentrics.x;
                currentTetAssignment.barycentricCoords[1] = barycentrics.y;
                currentTetAssignment.barycentricCoords[2] = barycentrics.z;
                currentTetAssignment.barycentricCoords[3] = barycentrics.w;

                outputTetAssignments->push_back(currentTetAssignment);
            }

            currentTetId = incidentTetId;

        } while (!done);
    }
}
