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
// Tet mesh and tet mesh buffer definitions
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXCommonInternal.h"
#include "FEMFXAtomicOps.h"
#include "FEMFXBvh.h"
#include "FEMFXTetMath.h"
#include "FEMFXTetMaterialParams.h"
#include "FEMFXTetMeshConnectivity.h"
#include "FEMFXGraphColoring.h"
#include "FEMFXVelStats.h"

namespace AMD
{
    struct FmScene;
    struct FmMpcgSolverData;
    struct FmTetMeshBufferSetupParams;
    class FmAsyncTasksProgress;

    // Values accumulated from incident tets, included for rendering uses
    struct FmVertTetValues
    {
        float              tetStrainMagAvg;    // Average value over incident tets, computed if FM_OBJECT_FLAG_COMPUTE_TET_STRAIN_MAG set on object
        float              tetStrainMagMax;    // Max tetStrainMagAvg value seen since initialization
        FmQuat             tetQuatSum;         // Sum of incident tet quaterion orientations, used to smooth render mesh normals; normalizing expected outside of library e.g. in shader
    };

    // Tet material params for stress matrix calculation
    struct FmTetStressMaterialParams
    {
        float youngsModulus;            // Greater values will increase stiffness of material
        float poissonsRatio;            // Value must be < 0.5.  Defines how much material bulges when compressed, with 0 causing none.  Values closer to 0.5 worsen conditioning and require more iterations.

        inline FmTetStressMaterialParams() : youngsModulus(1.0e5f), poissonsRatio(0.25f) {}
    };

    // Tet material params for plasticity
    struct FmTetPlasticityMaterialParams
    {
        float plasticYieldThreshold;    // Threshold for stress magnitude where plastic deformation starts.
        float plasticCreep;             // Value >= 0 and <=1.  Portion of elastic deformation converted to plastic is creep * (stress_mag - yield)/stress_mag
        float plasticMin;               // Value > 0 and <= 1.  Minimum scale of compression from plastic deformation.  Smaller values allow greater plastic deformation but may worsen conditioning.
        float plasticMax;               // Value >= 1.  Maximum scale of stretch from plastic deformation.   Larger values allow greater plastic deformation but may worsen conditioning.

        inline FmTetPlasticityMaterialParams() : plasticYieldThreshold(0.0f), plasticCreep(0.0f), plasticMin(0.5f), plasticMax(2.0f) {}
    };

    // Tet material params for fracture
    struct FmTetFractureMaterialParams
    {
        float fractureStressThreshold;          // Threshold for stress max eigenvalue where fracture occurs

        inline FmTetFractureMaterialParams() : fractureStressThreshold(5.0e3f) {}
    };

    // Tet material params for deformation constraints
    struct FmTetDeformationMaterialParams
    {
        float lowerDeformationLimit;    // Value > 0 and <= 1, or unlimited if = 0.  Constrains minimum scale of deformation.
        float upperDeformationLimit;    // Value >= 1, or unlimited if = 0.  Constrains maximum scale of deformation.

        inline FmTetDeformationMaterialParams() : lowerDeformationLimit(0.0f), upperDeformationLimit(0.0f) {}
    };

    // Tet stiffness terms
    struct FmTetStiffnessState
    {
        uint                        stiffnessMatRowOffsets[4][4];  // Row offsets giving destinations of submats of rotatedStiffnessMat
#if !FM_MATRIX_ASSEMBLY_BY_TETS
        FmTetRotatedStiffnessMatrix rotatedStiffnessMat;           // 3x3 stiffness values for each pair of vertices in tet
#endif
    };

    // Tet mesh exterior faces used in collision detection.
    // Includes ids of the faces incident at its edges.
    // For CCD will test the component vertex/face and edge/edge feature pairs.
    // Can eliminate redundant tests of features that are shared by triangles.
    // Each exterior triangle is assigned exclusive ownership of a subset of features via a greedy assignment.
    // Reference: Curtis et al., "Fast Collision Detection for Deformable Models using Representative-Triangles"
    struct FmExteriorFace
    {
        uint id;                     // id plus feature ownership bits
        uint tetId;                  // id of tet mesh tet
        uint faceId;                 // 0..3 face of tet
        uint opposingTetId;          // Tet incident to this face following fracture
        uint edgeIncidentFaceIds[3]; // Exterior faces incident at edges

        inline FmExteriorFace() : id(0), tetId(FM_INVALID_ID), faceId(FM_INVALID_ID), opposingTetId(FM_INVALID_ID)
        {
            edgeIncidentFaceIds[0] = FM_INVALID_ID;
            edgeIncidentFaceIds[1] = FM_INVALID_ID;
            edgeIncidentFaceIds[2] = FM_INVALID_ID;
        }

        inline void SetId(uint inId) { id = (inId << 6) | (id & 0x3f); }
        inline uint GetId() const { return id >> 6; }
        inline void AssignVert(uint i) { id = id | (1 << i); }
        inline void AssignEdge(uint i) { id = id | (1 << (i + 3)); }
        inline void AssignAll() { id = id | 0x3f; }
        inline void ClearVert(uint i) { id = id & ~(1 << i); }
        inline void ClearEdge(uint i) { id = id & ~(1 << (i + 3)); }
        inline void ClearAssignments() { id = id & ~0x3f; }
        inline bool OwnsVert(uint i) const { return (id & (1 << i)) != 0; }
        inline bool OwnsEdge(uint i) const { return (id & (1 << (i + 3))) != 0; }
        inline bool OwnsAll() const { return (id & 0x3f) == 0x3f; }
        inline bool OwnsAny() const { return (id & 0x3f) != 0; }
    };

    // State updated depending on current strain and plasticity parameters
    struct FmTetPlasticityState
    {
        FmTetShapeParams plasticShapeParams;  // Recomputed shape parameters describing a new virtual rest position after plastic deformation
        FmMatrix3 plasticDeformationMatrix;   // Elastic deformation gradient = total deformation gradient * inverse(plasticDeformationMatrix)
#if FM_COMPUTE_PLASTIC_REL_ROTATION
        FmMatrix3 plasticTetRelRotation;      // Store rotation computed at end of step based on plastic rest positions
#endif
    };

    // For a list of tets to process in fracture code
    struct FmTetToFracture
    {
        FmVector3 fractureDirection;  // Direction of fracture relative to rest configuration
        uint tetId;                      // Id of fracturing tet
    };

    // Indices used for reordering features after fracture
    struct FmReorderInfo
    {
        uint dstIdx;    // Destination index after partitioning, and visited flag
        uint remapIdx;  // Remapped index after partitioning
    };

    // Tetrahedral mesh data structure.
    // Stores mesh topology, tet and vert properties for FEM simulation, and dynamic state.
    struct FmTetMesh
    {
        FmAtomicUint numTetsToFracture;

        uint                             bufferId;                             // Id of FmTetMeshBuffer which contains this
        uint                             objectId;                             // Id for this object
        uint                             islandId;                             // Id of FmConstraintIsland which contains this; NOTE: different meaning depending on FM_OBJECT_FLAG_SLEEPING
        FmVector3                        minPosition;                          // Minimum position of bounding box around tet mesh 
        FmVector3                        maxPosition;                          // Maximum position of bounding box around tet mesh
        FmVector3                        centerOfMass;                         // Center of mass
        FmVector3                        gravityVector;                        // Gravity acceleration, 0 by default; added with FmSceneControlParams::gravityVector
        float                            mass;                                 // Total mass of tet mesh
        FmVertNeighbors*                 vertsNeighbors;                       // Incident tets of vertices
        float*                           vertsMass;                            // Mass of vertices
        uint16_t*                        vertsFlags;                           // Flags of vertices
        uint*                            vertsIndex0;                          // Initial index of each vertex in FmTetMeshBuffer (before fracture)
        FmVector3*                       vertsRestPos;                         // Rest positions of vertices
        FmVector3*                       vertsPos;                             // Positions of vertices
        FmVector3*                       vertsVel;                             // Velocities of vertices
        FmVector3*                       vertsExtForce;                        // External forces on vertices, reset to zero during update.  Gravity is set separately as a scene constant.
        FmVertTetValues*                 vertsTetValues;                       // Values accumulated at vertex from incident tets, used for rendering
        FmVertConnectivity               vertConnectivity;                     // Connectivity data which stores the tets that are incident to each vertex
        float*                           tetsMass;                             // Mass of tetrahedra
        float*                           tetsFrictionCoeff;                    // Friction coefficients of tetrahedra
        uint16_t*                        tetsFlags;                            // Flags of tetrahedra
        FmTetShapeParams*                tetsShapeParams;                      // Shape parameters of tetrahedra
        float*                           tetsRestDensity;                      // Density of tetrahedra at rest
        uint*                            tetsMaxUnconstrainedSolveIterations;  // Maximum number of CG iterations to use with this material
        FmTetStressMaterialParams*       tetsStressMaterialParams;             // Material parameters of tetrahedra for stress matrix
        FmTetDeformationMaterialParams*  tetsDeformationMaterialParams;        // Material parameters of tetrahedra for deformation
        FmTetFractureMaterialParams*     tetsFractureMaterialParams;           // Material parameters of tetrahedra for fracture
        FmTetPlasticityMaterialParams*   tetsPlasticityMaterialParams;         // Material parameters of tetrahedra for plasticity
        float*                           tetsStrainMag;                        // Strain magnitude of tetrahedra
        uint*                            tetsIndex0;                           // Initial index of each tetrahedron in FmTetMeshBuffer (before fracture)
        FmMatrix3*                       tetsRotation;                         // Tet rotations from rest to deformed shape
        FmTetFaceIncidentTetIds*         tetsFaceIncidentTetIds;               // Ids of face-incident tets, or exterior faces
        FmTetVertIds*                    tetsVertIds;                          // Vertex ids of tetrahedra
        FmTetStiffnessState*             tetsStiffness;                        // Stiffness matrix state recomputed each simulation step
        FmExteriorFace*                  exteriorFaces;                        // Exterior faces of mesh used in collision detection
        FmTetToFracture*                 tetsToFracture;                       // List of tets with sufficient stress to fracture, NULL for meshes without fracture enabled
        FmTetPlasticityState*            tetsPlasticity;                       // Plasticity related state, NULL for meshes without plasticity enabled
        FmMpcgSolverData*                solverData;                           // Pointer to solver data for this mesh
        FmBvh                            bvh;                                  // Bounding volume hierarchy used for collision detection
                                         
        uint                             numVerts;
        uint                             numTets;
        uint                             numExteriorFaces;
        uint                             maxVerts;
        uint                             maxExteriorFaces;
        uint                             numNewExteriorFaces;
        uint                             maxUnconstrainedSolveIterations;     // Maximum number of iterations for unconstrained (MPCG) solve, max over all tet materials
                                         
        float                            frictionCoeff;                       // Friction coefficient used for volume contacts
        float                            removeKinematicStressThreshold;      // Threshold to break kinematic flags, if FM_VERT_FLAG_KINEMATIC_REMOVABLE set
        float                            sleepMaxSpeedThreshold;              // Max speed must be under this threshold for sleeping 
        float                            sleepAvgSpeedThreshold;              // Average speed must be under this threshold for sleeping
        uint                             sleepStableCount;                    // Number of steps that speeds must fall under thresholds to initiate sleeping
        float                            extForceSpeedLimit;                  // For stability project external force to not increase vertex speed above limit
        float                            resetSpeedLimit;                     // Speed above this will trigger a reset
                                         
        FmVelStats                       velStats;                            // Statistics for sleeping test
                                         
        uint16_t                         flags;                               // Bitwise or of FM_OBJECT_FLAG_* values
        uint8_t                          collisionGroup;                      // Collision group index < 32
    };

    // Tracks where a vert is moved when fracture occurs.
    struct FmVertReference
    {
        uint meshIdx;
        uint vertId;
    };

    // Tracks where a tet is moved when fracture occurs.
    struct FmTetReference
    {
        uint meshIdx;
        uint tetId;
        uint fractureGroupId;
    };

    // Description of memory needed for the original tet mesh and tet meshes created from it by fracture.
    // Based on supplied maximum verts and meshes, can bound memory required for mesh geometry, state, and MPCG solver data,
    // and allocate a single contiguous buffer.
    // Also contains information about "fracture groups", connected-components of tets that may be created depending on 
    // use of FM_TET_FLAG_FACE*_FRACTURE_DISABLED flags.
    struct FmTetMeshBuffer
    {
        size_t            bufferNumBytes;
        uint              bufferId;

        FmTetMesh*        tetMeshes;          // Array of tet meshes created from original
        FmMpcgSolverData* solverData;         // Array of solver data structs, one per mesh

        FmFractureGroupCounts* fractureGroupCounts;    // Counts of features in each fracture group, used to resize buffers after fracture
        bool*                  visitedFractureGroup;   // Visited flags used in connected components

        uint*                  tetMeshVertOffsets;     // Keeps offset to first vertex of each tet mesh if all collected; used for rendering (currently this is only temporary data that could be removed)

        FmVertReference*  vertReferences;     // Array mapping from original vert ids to new locations in new meshes
        FmTetReference*   tetReferences;      // Array mapping from original tet ids to new locations in new meshes
        uint              numVerts;           // Initial number of verts before any fracture
        uint              numTets;            // Number of tets

        uint              numTetMeshes;
        uint              maxTetMeshes;       // Number of fracture groups
    };

    inline void FmInitTetMaterialParams(FmTetMaterialParams* materialParams, const FmTetMesh& tetMesh, uint tId)
    {
        materialParams->restDensity = tetMesh.tetsRestDensity[tId];
        materialParams->maxUnconstrainedSolveIterations = tetMesh.tetsMaxUnconstrainedSolveIterations[tId];
        materialParams->youngsModulus = tetMesh.tetsStressMaterialParams[tId].youngsModulus;
        materialParams->poissonsRatio = tetMesh.tetsStressMaterialParams[tId].poissonsRatio;
        materialParams->lowerDeformationLimit = tetMesh.tetsDeformationMaterialParams[tId].lowerDeformationLimit;
        materialParams->upperDeformationLimit = tetMesh.tetsDeformationMaterialParams[tId].upperDeformationLimit;

        if (tetMesh.tetsFractureMaterialParams)
        {
            materialParams->fractureStressThreshold = tetMesh.tetsFractureMaterialParams[tId].fractureStressThreshold;
        }
        else
        {
            materialParams->fractureStressThreshold = 0.0f;
        }

        if (tetMesh.tetsPlasticityMaterialParams)
        {
            materialParams->plasticYieldThreshold = tetMesh.tetsPlasticityMaterialParams[tId].plasticYieldThreshold;
            materialParams->plasticCreep = tetMesh.tetsPlasticityMaterialParams[tId].plasticCreep;
            materialParams->plasticMin = tetMesh.tetsPlasticityMaterialParams[tId].plasticMin;
            materialParams->plasticMax = tetMesh.tetsPlasticityMaterialParams[tId].plasticMax;
        }
        else
        {
            materialParams->plasticYieldThreshold = 0.0f;
            materialParams->plasticCreep = 0.0f;
            materialParams->plasticMin = 0.0f;
            materialParams->plasticMax = 0.0f;
        }
    }

    inline void FmInitTetMaterialParams(FmTetMesh* tetMesh, uint tId, const FmTetMaterialParams& materialParams)
    {
        tetMesh->tetsRestDensity[tId] = materialParams.restDensity;
        tetMesh->tetsMaxUnconstrainedSolveIterations[tId] = materialParams.maxUnconstrainedSolveIterations;
        tetMesh->tetsStressMaterialParams[tId].youngsModulus = materialParams.youngsModulus;
        tetMesh->tetsStressMaterialParams[tId].poissonsRatio = materialParams.poissonsRatio;
        tetMesh->tetsDeformationMaterialParams[tId].lowerDeformationLimit = materialParams.lowerDeformationLimit;
        tetMesh->tetsDeformationMaterialParams[tId].upperDeformationLimit = materialParams.upperDeformationLimit;

        if (tetMesh->tetsFractureMaterialParams)
        {
            tetMesh->tetsFractureMaterialParams[tId].fractureStressThreshold = materialParams.fractureStressThreshold;
        }

        if (tetMesh->tetsPlasticityMaterialParams)
        {
            tetMesh->tetsPlasticityMaterialParams[tId].plasticYieldThreshold = materialParams.plasticYieldThreshold;
            tetMesh->tetsPlasticityMaterialParams[tId].plasticCreep = materialParams.plasticCreep;
            tetMesh->tetsPlasticityMaterialParams[tId].plasticMin = materialParams.plasticMin;
            tetMesh->tetsPlasticityMaterialParams[tId].plasticMax = materialParams.plasticMax;
        }
    }

    // Initialize empty tet mesh
    static inline void FmInitTetMesh(FmTetMesh* mesh)
    {
        FmAtomicWrite(&mesh->numTetsToFracture.val, 0);
        mesh->bufferId = FM_INVALID_ID;
        mesh->objectId = FM_INVALID_ID;
        mesh->islandId = FM_INVALID_ID;
        mesh->minPosition = FmInitVector3(0.0f);
        mesh->maxPosition = FmInitVector3(0.0f);
        mesh->centerOfMass = FmInitVector3(0.0f);
        mesh->gravityVector = FmInitVector3(0.0f);
        mesh->mass = 1.0f;
        mesh->vertsNeighbors = NULL;
        mesh->vertsMass = NULL;
        mesh->vertsFlags = NULL;
        mesh->vertsIndex0 = NULL;
        mesh->vertsRestPos = NULL;
        mesh->vertsPos = NULL;
        mesh->vertsVel = NULL;
        mesh->vertsExtForce = NULL;
        mesh->vertsTetValues = NULL;
        mesh->vertConnectivity.incidentTets = NULL;
        mesh->vertConnectivity.numIncidentTets = 0;
        mesh->vertConnectivity.numIncidentTetsTotal = 0;
        mesh->vertConnectivity.numAdjacentVerts = 0;
        mesh->tetsMass = NULL;
        mesh->tetsFrictionCoeff = NULL;
        mesh->tetsFlags = NULL;
        mesh->tetsShapeParams = NULL;
        mesh->tetsRestDensity = NULL;
        mesh->tetsMaxUnconstrainedSolveIterations = NULL;
        mesh->tetsStressMaterialParams = NULL;
        mesh->tetsDeformationMaterialParams = NULL;
        mesh->tetsFractureMaterialParams = NULL;
        mesh->tetsPlasticityMaterialParams = NULL;
        mesh->tetsStrainMag = NULL;
        mesh->tetsIndex0 = NULL;
        mesh->tetsRotation = NULL;
        mesh->tetsFaceIncidentTetIds = NULL;
        mesh->tetsVertIds = NULL;
        mesh->tetsStiffness = NULL;
        mesh->exteriorFaces = NULL;
        mesh->tetsToFracture = NULL;
        mesh->tetsPlasticity = NULL;
        mesh->solverData = NULL;
        FmInitBvh(&mesh->bvh);
        mesh->numVerts = 0;
        mesh->numTets = 0;
        mesh->numExteriorFaces = 0;
        mesh->maxVerts = 0;
        mesh->maxExteriorFaces = 0;
        mesh->numNewExteriorFaces = 0;
        mesh->maxUnconstrainedSolveIterations = FM_DEFAULT_MAX_CG_ITERATIONS;
        mesh->frictionCoeff = FM_DEFAULT_FRICTION_COEFF;
        mesh->removeKinematicStressThreshold = FLT_MAX;
        mesh->sleepMaxSpeedThreshold = 2.0f;
        mesh->sleepAvgSpeedThreshold = 0.15f;
        mesh->sleepStableCount = 20;
        mesh->extForceSpeedLimit = 100.0f;
        mesh->resetSpeedLimit = 200.0f;
        FmInitVelStats(&mesh->velStats);
        mesh->flags = (FM_OBJECT_FLAG_NEEDS_CONNECTED_COMPONENTS | FM_OBJECT_FLAG_NEEDS_ADJACENT_VERT_OFFSETS);
        mesh->collisionGroup = 0;
    }

    // Initialize empty tet mesh buffer
    static inline void FmInitTetMeshBuffer(FmTetMeshBuffer* tetMeshBuffer)
    {
        tetMeshBuffer->bufferNumBytes = 0;
        tetMeshBuffer->bufferId = FM_INVALID_ID;
        tetMeshBuffer->tetMeshes = NULL;
        tetMeshBuffer->solverData = NULL;
        tetMeshBuffer->fractureGroupCounts = NULL;
        tetMeshBuffer->visitedFractureGroup = NULL;
        tetMeshBuffer->tetMeshVertOffsets = NULL;
        tetMeshBuffer->vertReferences = NULL;
        tetMeshBuffer->tetReferences = NULL;
        tetMeshBuffer->numVerts = 0;
        tetMeshBuffer->numTets = 0;
        tetMeshBuffer->numTetMeshes = 0;
        tetMeshBuffer->maxTetMeshes = 0;
    }

    static FM_FORCE_INLINE void FmDynamicAndMovingFlags(uint* dynamicFlags, uint* movingFlags, const FmTetMesh& tetMesh, const FmTetVertIds& tetVertIds, float barycentrics[4])
    {
        uint vId0 = tetVertIds.ids[0];
        uint vId1 = tetVertIds.ids[1];
        uint vId2 = tetVertIds.ids[2];
        uint vId3 = tetVertIds.ids[3];

        int nzb0Mask = -(int)(barycentrics[0] != 0.0f);
        int nzb1Mask = -(int)(barycentrics[1] != 0.0f);
        int nzb2Mask = -(int)(barycentrics[2] != 0.0f);
        int nzb3Mask = -(int)(barycentrics[3] != 0.0f);

        int dyn0Mask = -(int)FM_NOT_SET(tetMesh.vertsFlags[vId0], FM_VERT_FLAG_KINEMATIC);
        int dyn1Mask = -(int)FM_NOT_SET(tetMesh.vertsFlags[vId1], FM_VERT_FLAG_KINEMATIC);
        int dyn2Mask = -(int)FM_NOT_SET(tetMesh.vertsFlags[vId2], FM_VERT_FLAG_KINEMATIC);
        int dyn3Mask = -(int)FM_NOT_SET(tetMesh.vertsFlags[vId3], FM_VERT_FLAG_KINEMATIC);

        int mov0Mask = -(int)!FmIsZero(tetMesh.vertsVel[vId0]);
        int mov1Mask = -(int)!FmIsZero(tetMesh.vertsVel[vId1]);
        int mov2Mask = -(int)!FmIsZero(tetMesh.vertsVel[vId2]);
        int mov3Mask = -(int)!FmIsZero(tetMesh.vertsVel[vId3]);

        uint nonzeroBary = (nzb0Mask & 0x1) | (nzb1Mask & 0x2) | (nzb2Mask & 0x4) | (nzb3Mask & 0x8);
        uint dynamic = (dyn0Mask & 0x1) | (dyn1Mask & 0x2) | (dyn2Mask & 0x4) | (dyn3Mask & 0x8);
        uint moving = (mov0Mask & 0x1) | (mov1Mask & 0x2) | (mov2Mask & 0x4) | (mov3Mask & 0x8);

        *dynamicFlags = dynamic & nonzeroBary;
        *movingFlags = moving & nonzeroBary;
    }

    static FM_FORCE_INLINE void FmResetForceOnVert(FmTetMesh* tetMesh, uint vertId)
    {
        tetMesh->vertsExtForce[vertId] = FmInitVector3(0.0f);
        tetMesh->vertsFlags[vertId] &= ~FM_VERT_FLAG_EXT_FORCE_SET;
    }

    static FM_FORCE_INLINE bool FmIsTetInverted(const FmTetMesh& tetMesh, uint tetId)
    {
        FmTetVertIds tetVertIds = tetMesh.tetsVertIds[tetId];
        FmVector3 tetPositions[4];
        tetPositions[0] = tetMesh.vertsPos[tetVertIds.ids[0]];
        tetPositions[1] = tetMesh.vertsPos[tetVertIds.ids[1]];
        tetPositions[2] = tetMesh.vertsPos[tetVertIds.ids[2]];
        tetPositions[3] = tetMesh.vertsPos[tetVertIds.ids[3]];
        return FmIsTetInverted(tetPositions);
    }

    // Get total bytes needed for FmTetMesh data including FmTetMeshBuffer struct and all arrays.
    size_t FmGetTetMeshBufferSize(const FmTetMeshBufferSetupParams& params);

    // Suballocate memory from pBuffer to create FmTetMeshBuffer and arrays.
    // Applies initial values in TetMeshSetupParams, but application must still initialize most data including topology, state, matrices.
    // Also sets the pTetMeshPtr to the initial FmTetMesh (before fracture occurs).
    // NOTE: pBuffer must be 64-bit aligned memory
    FmTetMeshBuffer* FmSetupTetMeshBuffer(
        const FmTetMeshBufferSetupParams& params,
        const FmFractureGroupCounts* fractureGroupCounts,
        const uint* tetFractureGroupIds,
        uint8_t* pBuffer64ByteAligned, size_t bufferNumBytes, FmTetMesh** pTetMeshPtr);

    // Allocate tet mesh arrays based on specified sizes
    void FmAllocTetMeshData(FmTetMesh* tetMesh, uint numVerts, uint numTets, uint maxVerts, uint maxExteriorFaces, uint numVertIncidentTets, bool enablePlasticity = false, bool enableFracture = false);

    // Allocate tet mesh arrays with sufficient storage to create copy of srcMesh
    void FmAllocTetMeshData(FmTetMesh* dstMesh, const FmTetMesh& srcMesh);

    // Copy all data from srcMesh to dstMesh
    void FmCopyTetMesh(FmTetMesh* dstMesh, const FmTetMesh& srcMesh);

    // Copies subset of tets and vertices referenced by tetIds array and initializes connectivity.
    // Expects dstMesh allocated from srcMesh using FmAllocTetMeshData().
    // Expects that remapVertIds size is at least number of verts and all entries FM_INVALID_ID; function will restore all entries to FM_INVALID_ID before returning.
    void FmCopySubsetTetMesh(FmTetMesh* dstMesh, uint* remapVertIds, const FmTetMesh& srcMesh, const uint* tetIds, uint numTets);

    // Data write/read
    void FmWriteTetMesh(const char* filename, FmTetMesh* tetMesh);
    void FmReadTetMesh(const char* filename, FmTetMesh* tetMesh);

    // Free arrays in tetMesh
    void FmFreeTetMeshData(FmTetMesh* tetMesh);

    // Compute total number of adjacent vertices, and offset to each vert's neighbors array
    void FmUpdateAdjacentVertOffsets(FmTetMesh* tetMesh);

    // Assign verts and edges to exterior faces, which can accelerate collision detection or reduce number of contacts.
    // Also link exterior faces at their edges.
    // After mesh initialization, only processing the new exterior faces created by fracture.
    void FmAssignFeaturesToExteriorFaces(FmTetMesh* tetMesh, uint beginIndex, uint numFaces);

    // Build BVH acceleration structure.  Uses mesh vertex velocities to compute box velocites.
    void FmBuildHierarchy(FmTetMesh* tetMesh, float timestep, float gap);

    // Reset mesh in case of high velocities.
    // Failsafe for CG convergence issue seen mainly with small fractured pieces after large deformation/forces.
    // TODO: Better understand issue and use a more graceful recovery.
    void FmFailsafeReset(FmTetMesh* tetMesh, float resetSpeedLimit);

    // Step positions assuming velocity is set to end of step velocity.
    // Updates min and max position and center of mass.
    // If position is changed, set FM_OBJECT_FLAG_POS_CHANGED for next frame.
    void FmUpdateVertPositions(FmTetMesh* tetMesh, float timestep);

    // Update bounds and center of mass from vertex positions.
    void FmUpdateBoundsAndCenterOfMass(FmTetMesh* tetMesh);

    // Split mesh at vertices of fracturing tets, update topology and FmMpcgSolverData
    void FmFractureMesh(FmScene* scene, FmTetMesh* tetMesh);

    // Get array of new exterior faces from tet mesh
    const FmExteriorFace* FmGetNewExteriorFaces(const FmTetMesh& tetMesh);
}