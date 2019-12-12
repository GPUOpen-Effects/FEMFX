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

#pragma once

#include "AMD_FEMFX.h"
#include <vector>
#include <unordered_map>

#define MAX_RENDER_VERT_TET_ASSIGNMENTS 2

namespace AMD
{

    // The tetrahedron, and barycentric coordinates within it, that a render vertex is assigned to
    struct RenderVertTetAssignment
    {
        uint  tetId;
        uint  nearestFaceId;
        float barycentricCoords[4];
        bool  inside;
    };

    // A shard refers to a piece of the render geometry that will represent a broken piece of the tet mesh.
    // Each vertex of a shard gets multiple tet assignments, and the assignment can change after fracture.
    struct ShardVertTetAssignmentsFixedNum
    {
        RenderVertTetAssignment tetAssignments[MAX_RENDER_VERT_TET_ASSIGNMENTS];
        uint                    numTets;
    };

    struct ShardVertTetAssignments
    {
        std::vector<RenderVertTetAssignment> tetAssignments;
        uint numTets;

        ShardVertTetAssignments() : numTets(0) {}

        // Set num tets to min of tet assignments and MAX_RENDER_VERT_TET_ASSIGNMENTS
        void SetNumTets()
        {
            uint num = (uint)tetAssignments.size();

            if (num > MAX_RENDER_VERT_TET_ASSIGNMENTS)
            {
                num = MAX_RENDER_VERT_TET_ASSIGNMENTS;
            }

            numTets = num;
        }
    };

    // A non-fracture region is a group of tetrahedra that cannot separate due to flags preventing fracture on their faces.
    // To find the tet assignments for a shard vertex, it's necessary to determine the non-fracture regions and find which one 
    // the shard is attached to.
    // This struct contains the tet ids belonging to a non-fracture region, and a BVH computed over them, to accelerate render 
    // vertex preprocessing
    struct NonFractureRegionData
    {
        uint* tetIds;
        FmTetVertIds* tetVertIds;
        FmBvh* bvh;
        uint numTets;

        NonFractureRegionData()
        {
            tetIds = NULL;
            tetVertIds = NULL;
            bvh = NULL;
            numTets = 0;
        }

        // Pass in all tet mesh vertex positions and tet vertex ids, and list of tet ids in the non-fracture region.
        // Create ids lists and BVH for the non-fracture region.
        // Allocates memory, call Destroy() to free
        void Create(FmVector3* meshVertRestPositions, FmTetVertIds* meshTetVertIds, const std::vector<uint>& inTetIds);

        void Destroy();
    };

    // All non-fracture region data created for a tet mesh buffer.
    // Region ids for all tets, and data for each region
    struct NonFractureRegions
    {
        std::vector<NonFractureRegionData> regions;
        std::vector<uint> regionIdOfTet;
        std::vector<uint> nonFractureFlagsOfTet;

        NonFractureRegions(int numTets);
        ~NonFractureRegions();

        // Create new region data from tet mesh vertex positions and tet vertex ids, and list of tet ids in the non-fracture region.
        void AddRegion(FmVector3* meshVertRestPositions, FmTetVertIds* meshTetVertIds, const std::vector<uint>& tetIds, const std::vector<uint>& tetNoFractureFlags);

        // After adding regions, create regions for any unassigned single tets
        void AddRemainingRegions(FmVector3* meshVertRestPositions, FmTetVertIds* meshTetVertIds);
    };

    // Info to update the assignment to BaryPos
    struct ShardVertUpdateInfo
    {
        int meshSectionId;
        int shardVertexId;
        int baryPosBaseId; // Base id in BaryPos array
        int baryPosOffset; // Maximum offset after this tet face fractures

        ShardVertUpdateInfo()
        {
            meshSectionId = 0;
            shardVertexId = 0;
            baryPosBaseId = 0;
            baryPosOffset = 0;
        }
    };

    // Indices of shard positions that will need a new tet assignment on fracture of each of the four tet faces
    struct TetFractureShardVerticesToUpdate
    {
        std::vector<ShardVertUpdateInfo> shardVertices0;
        std::vector<ShardVertUpdateInfo> shardVertices1;
        std::vector<ShardVertUpdateInfo> shardVertices2;
        std::vector<ShardVertUpdateInfo> shardVertices3;

        std::vector<ShardVertUpdateInfo>& GetShardVerticesOfFace(int idx)
        {
            if (idx == 0)
            {
                return shardVertices0;
            }
            else if (idx == 1)
            {
                return shardVertices1;
            }
            else if (idx == 2)
            {
                return shardVertices2;
            }
            else
            {
                return shardVertices3;
            }
        }
    };

    // unordered_map to group shard vertices with the same tet assignments

    // Key is array of tet assignments
    class ShardVertGroupKey
    {
    public:
        const ShardVertTetAssignments* shardVertTetAssignments;

        ShardVertGroupKey() : shardVertTetAssignments(NULL) { }
    };

    // Value is a set of shard vert indices
    class ShardVertGroup
    {
    public:
        std::vector<uint> shardVertIndices;
        uint idx; // idx in output array of bary pos offsets

        ShardVertGroup() : idx(0) { }
    };

    // Hash combines size and tet ids of tet assignments
    class ShardVertKeyHash
    {
    public:
        std::size_t operator()(const ShardVertGroupKey& key) const
        {
            if (key.shardVertTetAssignments == NULL)
            {
                return 0;
            }
            uint numAssignments = (uint)key.shardVertTetAssignments->numTets;
            std::hash<uint> uintHash;
            std::size_t hashVal = uintHash(numAssignments);
            for (uint i = 0; i < numAssignments; i++)
            {
                hashVal = hashVal ^ uintHash(key.shardVertTetAssignments->tetAssignments[i].tetId);
            }
            return hashVal;
        }
    };

    // Test array of tet assignments is equal
    class ShardVertKeyEqual
    {
    public:
        bool operator()(const ShardVertGroupKey& lhs, const ShardVertGroupKey& rhs) const
        {
            if (lhs.shardVertTetAssignments == NULL || rhs.shardVertTetAssignments == NULL)
            {
                return false;
            }

            uint numAssignments = (uint)lhs.shardVertTetAssignments->numTets;
            if (numAssignments != rhs.shardVertTetAssignments->numTets)
            {
                return false;
            }

            if (numAssignments == 1)
            {
                return true;
            }

            for (uint i = 0; i < numAssignments; i++)
            {
                if (lhs.shardVertTetAssignments->tetAssignments[i].tetId != rhs.shardVertTetAssignments->tetAssignments[i].tetId)
                {
                    return false;
                }
            }

            return true;
        }
    };

    // Shard vertices grouped if they have the same array of tet assignments. 
    // These shard vertices can share an offset to a tet/barycentric position.  Shard vertices will have different barycentric positions so will need a different base index.
    typedef std::unordered_map<ShardVertGroupKey, ShardVertGroup, ShardVertKeyHash, ShardVertKeyEqual> ShardVertGroups;

    // Group shard vertices if they have the same array of tet assignments. 
    void CreateShardVertGroups(ShardVertGroups* resultShardVertexGroups, const ShardVertTetAssignments* shardVertTetAssignments, uint numShardVerts);

    // Create render data from shard vertex groups
    void CreateShardRenderData(
        std::vector<uint>* outBaryPosOffsets,                          // Output array of baryPosOffsets matching the shard vertex groups
        std::vector<uint>* outBaryPosOffsetIds,                        // Output ids of baryPosOffsets for all render vertices
        TetFractureShardVerticesToUpdate* outTetFractureShardVertices, // Output map from tet fracture faces to groups/baryPosOffset; array assumed presized for all tets
        ShardVertGroups* shardVertGroups,        // Shard vertex groups created by CreateShardVertGroups()
        uint numRenderVertices,
        const uint* shardVertIds,                // Map from render vertex to shard vertex; will remap to baryPosOffset index
        const ShardVertTetAssignments* shardVertTetAssignments);

    // Find nearest tet and barycentric coords (in rest position) to input position.
    // Mesh geometry is provided as FmTetMeshBuffer.
    // Search is accelerated with bvh.
    void ComputeRenderVertTetAssignment(RenderVertTetAssignment* tetAssignment, const FmTetMeshBuffer& tetMeshBuffer, const FmBvh& bvh, const FmVector3& vertPos);

    // Find nearest tet and barycentric coords (in rest position) to input position.
    // Mesh geometry is provided as vertRestPositions and tetVertIds.
    // Search is accelerated with bvh.
    void ComputeRenderVertTetAssignment(
        RenderVertTetAssignment* tetAssignment,
        const FmVector3* vertRestPositions,
        const FmTetVertIds* tetVertIds,
        const FmBvh& bvh,
        const FmVector3& renderPos);

    // Find set of possible tet assignments in order from root position to render position.
    // Should use final assignment in list.
    // Reference: Parker and O’Brien, "Real-Time Deformation and Fracture in a Game Environment"    
    void ComputeShardVertTetAssignments(ShardVertTetAssignmentsFixedNum* tetAssignments, const FmTetMeshBuffer& tetMeshBuffer, const FmBvh& bvh, const FmVector3& vertPos, const FmVector3& rootPos);

    // Find set of possible tet assignments in order from root position to render position.
    // Should use final assignment in list.
    // Reference: Parker and O’Brien, "Real-Time Deformation and Fracture in a Game Environment"    
    void ComputeShardVertTetAssignments(
        ShardVertTetAssignmentsFixedNum* tetAssignments,
        const FmVector3* vertRestPositions,
        const FmTetVertIds* tetVertIds,
        const FmTetFaceIncidentTetIds* tetFaceIncidentTetIds,
        const FmBvh& bvh,
        const FmVector3& renderPos, const FmVector3& rootPos);

    // Step through list of tet assignments and cut the list short whenever an exterior face is reached.
    bool UpdateShardVertTetAssignments(ShardVertTetAssignmentsFixedNum* tetAssignments, const FmTetMeshBuffer& tetMeshBuffer);

    void GetMeshBufferRenderCounts(uint* numVerts, uint* numTets, const FmTetMeshBuffer& tetMeshBuffer);

    void GetMeshBufferRenderData(FmVector3* vertices, FmTetVertIds* tetVertIds, FmMatrix3* tetRotations, const FmTetMeshBuffer& tetMeshBuffer);

    // Associate this render vertex with the non-fracture region nearest to RootPos.
    // Find nearest tet to RenderPos in this region, then walk to RenderPos, recording all tets/faces crossed which can fracture.
    // This data can be used to switch the tet assignment after these faces break, to keep render vertex from stretching.
    void ComputeShardVertTetAssignments(
        std::vector<RenderVertTetAssignment>* outputTetAssignments,
        const NonFractureRegions& nonFractureRegions,
        const FmTetMeshBuffer& tetMeshBuffer,
        FmVector3* vertRestPositions,
        const FmBvh& bvh, const FmVector3& renderPos, const FmVector3& rootPos);
}