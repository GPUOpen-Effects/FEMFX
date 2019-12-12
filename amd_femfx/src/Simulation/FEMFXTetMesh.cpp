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
// Initialization of tet mesh state and connectivity
//---------------------------------------------------------------------------------------

#include "AMD_FEMFX.h"
#include "FEMFXTetMath.h"
#include "FEMFXTetMesh.h"
#include "FEMFXMpcgSolver.h"
#include "FEMFXSleeping.h"
#include "FEMFXUpdateTetState.h"

namespace AMD
{
    // Initialize the vertex rest positions, positions (as a transformation of rest positions), mass, and velocity.
    // All vert members previously initialized in FmCreateTetMeshBuffer().
    // Mass will be replaced later if using FmSetMassesFromRestDensities().
    // Uses one velocity and external force value to initialize all vertices; can loop over vertices for more custom settings.
    void FmInitVertState(
        FmTetMesh* tetMesh,
        const FmVector3* vertRestPositions,
        const FmMatrix3& rotation, const FmVector3& translation,
        float mass,
        const FmVector3& velocity)
    {
        uint numVerts = tetMesh->numVerts;
        for (uint vId = 0; vId < numVerts; vId++)
        {
            tetMesh->vertsRestPos[vId] = vertRestPositions[vId];
            tetMesh->vertsPos[vId] = translation + mul(rotation, vertRestPositions[vId]);
            tetMesh->vertsVel[vId] = velocity;
            tetMesh->vertsMass[vId] = mass;
        }
    }

    // Initialize the tet vertex ids and material parameters.
    // All tet members previously initialized in FmCreateTetMeshBuffer().
    // Uses one material parameters value to initialize all tets; can loop over tets for more custom settings.
    // Sets tet mesh default removeKinematicStressThreshold to 4X of fractureStressThreshold
    void FmInitTetState(
        FmTetMesh* tetMesh,
        const FmTetVertIds* tetVertIds,
        const FmTetMaterialParams& materialParams,
        float frictionCoeff)
    {
        uint numTets = tetMesh->numTets;
        for (uint tId = 0; tId < numTets; tId++)
        {
            tetMesh->tetsVertIds[tId] = tetVertIds[tId];
            tetMesh->tetsFrictionCoeff[tId] = frictionCoeff;
            FmInitTetMaterialParams(tetMesh, tId, materialParams);
        }
        tetMesh->removeKinematicStressThreshold = materialParams.fractureStressThreshold * 4.0f;
    }

    size_t FmGetMpcgSolverSize(uint maxVerts, uint maxVertAdjacentVerts)
    {
        return FM_PAD_16(sizeof(*FmMpcgSolverData::kinematicFlags) * maxVerts)
            + FM_PAD_16(sizeof(*FmMpcgSolverData::PInvDiag) * maxVerts)
            + FM_PAD_16(sizeof(*FmMpcgSolverData::A.rowStarts) * (maxVerts * 2))  // extra rowStart per mesh to mark end of last row, 2x maxVerts worst case
            + FM_PAD_16(sizeof(*FmMpcgSolverData::A.submats) * (maxVerts + maxVertAdjacentVerts))
            + FM_PAD_16(sizeof(*FmMpcgSolverData::A.indices) * (maxVerts + maxVertAdjacentVerts))
            + FM_PAD_16(sizeof(*FmMpcgSolverData::b) * maxVerts)
            + FM_PAD_16(sizeof(*FmMpcgSolverData::mass) * maxVerts);
    }

    size_t FmGetTetMeshBufferSize(const FmTetMeshBufferSetupParams& params)
    {
        uint numTets = params.numTets;
        uint numVertIncidentTets = params.numVertIncidentTets;
        uint maxVerts = params.enableFracture ? FmMaxUint(params.maxVerts, params.numVerts) : params.numVerts;
        uint maxVertAdjacentVerts = params.maxVertAdjacentVerts;
        uint maxExteriorFaces = params.maxExteriorFaces;
        uint maxBvhNodes = FmNumBvhNodes(maxExteriorFaces);
        uint maxTetMeshes = params.enableFracture ? params.maxTetMeshes : 1;
        bool enablePlasticity = params.enablePlasticity;
        bool enableFracture = params.enableFracture;

        size_t numBytes =
            FM_PAD_64(sizeof(FmTetMeshBuffer))
            + FM_PAD_64(sizeof(*FmTetMeshBuffer::tetMeshes) * maxTetMeshes)
            + FM_PAD_16(sizeof(*FmTetMesh::vertsNeighbors) * maxVerts)
            + FM_PAD_16(sizeof(*FmTetMesh::vertsMass) * maxVerts)
            + FM_PAD_16(sizeof(*FmTetMesh::vertsFlags) * maxVerts)
            + FM_PAD_16(sizeof(*FmTetMesh::vertsIndex0) * maxVerts)
            + FM_PAD_16(sizeof(*FmTetMesh::vertsRestPos) * maxVerts)
            + FM_PAD_16(sizeof(*FmTetMesh::vertsPos) * maxVerts)
            + FM_PAD_16(sizeof(*FmTetMesh::vertsVel) * maxVerts)
            + FM_PAD_16(sizeof(*FmTetMesh::vertsExtForce) * maxVerts)
            + FM_PAD_16(sizeof(*FmTetMesh::vertsTetValues) * maxVerts)
            + FM_PAD_16(sizeof(*FmTetMesh::vertConnectivity.incidentTets) * numVertIncidentTets)
            + FM_PAD_16(sizeof(*FmTetMesh::tetsMass) * numTets)
            + FM_PAD_16(sizeof(*FmTetMesh::tetsFrictionCoeff) * numTets)
            + FM_PAD_16(sizeof(*FmTetMesh::tetsFlags) * numTets)
            + FM_PAD_16(sizeof(*FmTetMesh::tetsShapeParams) * numTets)
            + FM_PAD_16(sizeof(*FmTetMesh::tetsRestDensity) * numTets)
            + FM_PAD_16(sizeof(*FmTetMesh::tetsMaxUnconstrainedSolveIterations) * numTets)
            + FM_PAD_16(sizeof(*FmTetMesh::tetsStressMaterialParams) * numTets)
            + FM_PAD_16(sizeof(*FmTetMesh::tetsDeformationMaterialParams) * numTets)
            + (enableFracture ? FM_PAD_16(sizeof(*FmTetMesh::tetsFractureMaterialParams) * numTets) : 0)
            + (enablePlasticity ? FM_PAD_16(sizeof(*FmTetMesh::tetsPlasticityMaterialParams) * numTets) : 0)
            + FM_PAD_16(sizeof(*FmTetMesh::tetsStrainMag) * numTets)
            + FM_PAD_16(sizeof(*FmTetMesh::tetsIndex0) * numTets)
            + FM_PAD_16(sizeof(*FmTetMesh::tetsRotation) * numTets)
            + FM_PAD_16(sizeof(*FmTetMesh::tetsFaceIncidentTetIds) * numTets)
            + FM_PAD_16(sizeof(*FmTetMesh::tetsVertIds) * numTets)
            + FM_PAD_16(sizeof(*FmTetMesh::tetsStiffness) * numTets)
            + FM_PAD_16(sizeof(*FmTetMesh::exteriorFaces) * maxExteriorFaces)
            + (enableFracture ? FM_PAD_16(sizeof(*FmTetMesh::tetsToFracture) * numTets) : 0)
            + (enablePlasticity ? FM_PAD_16(sizeof(*FmTetMesh::tetsPlasticity) * numTets) : 0)
            + FM_PAD_16(sizeof(*FmTetMesh::bvh.nodes) * maxBvhNodes)
            + FM_PAD_16(sizeof(*FmTetMesh::bvh.primBoxes) * maxExteriorFaces)
            + FM_PAD_16(sizeof(*FmTetMesh::bvh.mortonCodesSorted) * maxExteriorFaces)
            + FM_PAD_16(sizeof(*FmTetMesh::bvh.primIndicesSorted) * maxExteriorFaces)
            + FmGetMpcgSolverSize(maxVerts, maxVertAdjacentVerts)
            + FM_PAD_16(sizeof(*FmTetMeshBuffer::solverData) * maxTetMeshes)
            + FM_PAD_16(sizeof(*FmTetMeshBuffer::fractureGroupCounts) * maxTetMeshes)
            + FM_PAD_16(sizeof(*FmTetMeshBuffer::visitedFractureGroup) * maxTetMeshes)
            + FM_PAD_16(sizeof(*FmTetMeshBuffer::tetMeshVertOffsets) * maxTetMeshes)
            + FM_PAD_16(sizeof(*FmTetMeshBuffer::vertReferences) * maxVerts)
            + FM_PAD_16(sizeof(*FmTetMeshBuffer::tetReferences) * numTets);


        return FM_PAD_64(numBytes);
    }

    void FmInitTetMeshVert(FmTetMesh* tetMesh, uint idx, uint16_t flags = 0)
    {
        FmVertNeighbors& vertNeighbors = tetMesh->vertsNeighbors[idx];
        vertNeighbors.incidentTetsStart = 0;
        vertNeighbors.numAdjacentVerts = 0;
        vertNeighbors.numIncidentTets = 0;
        tetMesh->vertsMass[idx] = 1.0f;
        tetMesh->vertsFlags[idx] = flags;

        tetMesh->vertsIndex0[idx] = idx;

        FmVertTetValues& vertTetValues = tetMesh->vertsTetValues[idx];
        vertTetValues.tetStrainMagAvg = 0.0f;
        vertTetValues.tetStrainMagMax = 0.0f;
        vertTetValues.tetQuatSum = FmInitQuat(0.0f, 0.0f, 0.0f, 0.0f);

        tetMesh->vertsRestPos[idx] = FmInitVector3(0.0f);
        tetMesh->vertsPos[idx] = FmInitVector3(0.0f);
        tetMesh->vertsVel[idx] = FmInitVector3(0.0f);
        tetMesh->vertsExtForce[idx] = FmInitVector3(0.0f);
    }

    void FmInitTetMeshTet(FmTetMesh* tetMesh, uint idx, uint16_t flags = 0)
    {
        tetMesh->tetsMass[idx] = 1.0f;
        tetMesh->tetsFrictionCoeff[idx] = FM_DEFAULT_FRICTION_COEFF;
        tetMesh->tetsFlags[idx] = flags;
        tetMesh->tetsRestDensity[idx] = 50.0f;
        tetMesh->tetsMaxUnconstrainedSolveIterations[idx] = FM_DEFAULT_MAX_CG_ITERATIONS;
        tetMesh->tetsStressMaterialParams[idx] = FmTetStressMaterialParams();
        tetMesh->tetsDeformationMaterialParams[idx] = FmTetDeformationMaterialParams();
        if (tetMesh->tetsFractureMaterialParams)
        {
            tetMesh->tetsFractureMaterialParams[idx] = FmTetFractureMaterialParams();
        }
        if (tetMesh->tetsPlasticityMaterialParams)
        {
            tetMesh->tetsPlasticityMaterialParams[idx] = FmTetPlasticityMaterialParams();
        }
        tetMesh->tetsIndex0[idx] = idx;

        tetMesh->tetsRotation[idx] = FmMatrix3::identity();

        FmTetFaceIncidentTetIds& tetFaceIncidentTetIds = tetMesh->tetsFaceIncidentTetIds[idx];
        tetFaceIncidentTetIds.ids[0] = FM_INVALID_ID;
        tetFaceIncidentTetIds.ids[1] = FM_INVALID_ID;
        tetFaceIncidentTetIds.ids[2] = FM_INVALID_ID;
        tetFaceIncidentTetIds.ids[3] = FM_INVALID_ID;

        FmTetVertIds& tetVertIds = tetMesh->tetsVertIds[idx];
        tetVertIds.ids[0] = FM_INVALID_ID;
        tetVertIds.ids[1] = FM_INVALID_ID;
        tetVertIds.ids[2] = FM_INVALID_ID;
        tetVertIds.ids[3] = FM_INVALID_ID;
    }

    void FmAllocTetMeshDataFromBuffer(
        FmTetMesh* tetMesh, uint maxVerts, uint numTets, uint maxExteriorFaces, uint numVertIncidentTets,
        bool enablePlasticity, bool enableFracture,
        uint8_t*& pBuffer, uint8_t* pBufferEnd)
    {
        uint maxBvhNodes = FmNumBvhNodes(maxExteriorFaces);

        tetMesh->vertsNeighbors = FmAllocFromBuffer<FmVertNeighbors>(&pBuffer, maxVerts, pBufferEnd);
        tetMesh->vertsMass = FmAllocFromBuffer<float>(&pBuffer, maxVerts, pBufferEnd);
        tetMesh->vertsFlags = FmAllocFromBuffer<uint16_t>(&pBuffer, maxVerts, pBufferEnd);
        tetMesh->vertsIndex0 = FmAllocFromBuffer<uint>(&pBuffer, maxVerts, pBufferEnd);
        tetMesh->vertsRestPos = FmAllocFromBuffer<FmVector3>(&pBuffer, maxVerts, pBufferEnd);
        tetMesh->vertsPos = FmAllocFromBuffer<FmVector3>(&pBuffer, maxVerts, pBufferEnd);
        tetMesh->vertsVel = FmAllocFromBuffer<FmVector3>(&pBuffer, maxVerts, pBufferEnd);
        tetMesh->vertsExtForce = FmAllocFromBuffer<FmVector3>(&pBuffer, maxVerts, pBufferEnd);
        tetMesh->vertsTetValues = FmAllocFromBuffer<FmVertTetValues>(&pBuffer, maxVerts, pBufferEnd);
        tetMesh->vertConnectivity.incidentTets = FmAllocFromBuffer<uint>(&pBuffer, numVertIncidentTets, pBufferEnd);
        tetMesh->vertConnectivity.numIncidentTets = numVertIncidentTets;
        tetMesh->vertConnectivity.numIncidentTetsTotal = numVertIncidentTets;
        tetMesh->vertConnectivity.numAdjacentVerts = 0; // calculated in FmUpdateAdjacentVertOffsets

        tetMesh->tetsMass = FmAllocFromBuffer<float>(&pBuffer, numTets, pBufferEnd);
        tetMesh->tetsFrictionCoeff = FmAllocFromBuffer<float>(&pBuffer, numTets, pBufferEnd);
        tetMesh->tetsFlags = FmAllocFromBuffer<uint16_t>(&pBuffer, numTets, pBufferEnd);
        tetMesh->tetsShapeParams = FmAllocFromBuffer<FmTetShapeParams>(&pBuffer, numTets, pBufferEnd);
        tetMesh->tetsRestDensity = FmAllocFromBuffer<float>(&pBuffer, numTets, pBufferEnd);
        tetMesh->tetsMaxUnconstrainedSolveIterations = FmAllocFromBuffer<uint>(&pBuffer, numTets, pBufferEnd);
        tetMesh->tetsStressMaterialParams = FmAllocFromBuffer<FmTetStressMaterialParams>(&pBuffer, numTets, pBufferEnd);
        tetMesh->tetsDeformationMaterialParams = FmAllocFromBuffer<FmTetDeformationMaterialParams>(&pBuffer, numTets, pBufferEnd);
        tetMesh->tetsFractureMaterialParams = enableFracture ? FmAllocFromBuffer<FmTetFractureMaterialParams>(&pBuffer, numTets, pBufferEnd) : NULL;
        tetMesh->tetsPlasticityMaterialParams = enablePlasticity ? FmAllocFromBuffer<FmTetPlasticityMaterialParams>(&pBuffer, numTets, pBufferEnd) : NULL;
        tetMesh->tetsStrainMag = FmAllocFromBuffer<float>(&pBuffer, numTets, pBufferEnd);
        tetMesh->tetsIndex0 = FmAllocFromBuffer<uint>(&pBuffer, numTets, pBufferEnd);
        tetMesh->tetsRotation = FmAllocFromBuffer<FmMatrix3>(&pBuffer, numTets, pBufferEnd);
        tetMesh->tetsFaceIncidentTetIds = FmAllocFromBuffer<FmTetFaceIncidentTetIds>(&pBuffer, numTets, pBufferEnd);
        tetMesh->tetsVertIds = FmAllocFromBuffer<FmTetVertIds>(&pBuffer, numTets, pBufferEnd);
        tetMesh->tetsStiffness = FmAllocFromBuffer<FmTetStiffnessState>(&pBuffer, numTets, pBufferEnd);
        tetMesh->tetsToFracture = enableFracture ? FmAllocFromBuffer<FmTetToFracture>(&pBuffer, numTets, pBufferEnd) : NULL;
        tetMesh->tetsPlasticity = enablePlasticity ? FmAllocFromBuffer<FmTetPlasticityState>(&pBuffer, numTets, pBufferEnd) : NULL;

        tetMesh->exteriorFaces = FmAllocFromBuffer<FmExteriorFace>(&pBuffer, maxExteriorFaces, pBufferEnd);
        tetMesh->bvh.nodes = FmAllocFromBuffer<FmBvhNode>(&pBuffer, maxBvhNodes, pBufferEnd);
        tetMesh->bvh.primBoxes = FmAllocFromBuffer<FmAabb>(&pBuffer, maxExteriorFaces, pBufferEnd);
        tetMesh->bvh.mortonCodesSorted = FmAllocFromBuffer<int>(&pBuffer, maxExteriorFaces, pBufferEnd);
        tetMesh->bvh.primIndicesSorted = FmAllocFromBuffer<int>(&pBuffer, maxExteriorFaces, pBufferEnd);
        tetMesh->bvh.numPrims = 0;

        tetMesh->numVerts = 0;
        tetMesh->numTets = numTets;
        tetMesh->numExteriorFaces = 0;
        tetMesh->maxVerts = maxVerts;
        tetMesh->maxExteriorFaces = maxExteriorFaces;
        tetMesh->numTetsToFracture = 0;
        tetMesh->numNewExteriorFaces = 0;

    }

    void FmAllocMpcgSolverDataFromBuffer(FmMpcgSolverData* solverData, uint maxVerts, uint maxVertAdjacentVerts, uint8_t*& pBuffer, uint8_t* pBufferEnd)
    {
        solverData->kinematicFlags = FmAllocFromBuffer<bool>(&pBuffer, maxVerts, pBufferEnd);
        solverData->PInvDiag = FmAllocFromBuffer<FmSMatrix3>(&pBuffer, maxVerts, pBufferEnd);
        solverData->A.rowStarts = FmAllocFromBuffer<uint>(&pBuffer, (maxVerts * 2), pBufferEnd); // extra rowStart per mesh to mark end of last row, 2x maxVerts worst case

        solverData->A.submats = FmAllocFromBuffer<FmSMatrix3>(&pBuffer, (maxVerts + maxVertAdjacentVerts), pBufferEnd);
        solverData->A.indices = FmAllocFromBuffer<uint>(&pBuffer, (maxVerts + maxVertAdjacentVerts), pBufferEnd);
        solverData->A.numRows = maxVerts;
        solverData->b = FmAllocFromBuffer<FmSVector3>(&pBuffer, maxVerts, pBufferEnd);
        solverData->mass = FmAllocFromBuffer<float>(&pBuffer, maxVerts, pBufferEnd);
        solverData->maxVertAdjacentVerts = maxVertAdjacentVerts;
        solverData->hasKinematicVerts = false;
    }

    FmTetMeshBuffer* FmSetupTetMeshBuffer(
        const FmTetMeshBufferSetupParams& params,
        const FmFractureGroupCounts* fractureGroupCounts,
        const uint* tetFractureGroupIds,
        uint8_t* pBuffer,
        size_t bufferNumBytes,
        FmTetMesh** pTetMeshPtr)
    {
        FM_ASSERT(((uintptr_t)pBuffer & 0x3f) == 0);

        uint numVerts = params.numVerts;
        uint numTets = params.numTets;
        uint numVertIncidentTets = params.numVertIncidentTets;
        uint maxVerts = params.enableFracture ? FmMaxUint(params.maxVerts, params.numVerts) : params.numVerts;
        uint maxVertAdjacentVerts = params.maxVertAdjacentVerts;
        uint maxExteriorFaces = params.maxExteriorFaces;
        uint maxTetMeshes = params.enableFracture ? params.maxTetMeshes : 1;
        uint collisionGroup = params.collisionGroup;
        bool enablePlasticity = params.enablePlasticity;
        bool enableFracture = params.enableFracture;
        bool isKinematic = params.isKinematic;

        FM_ASSERT(numVerts <= maxVerts);

        uint8_t* pBufferEnd = pBuffer + bufferNumBytes;

        FmTetMeshBuffer* pTetMeshBuffer = FmAllocFromBuffer64<FmTetMeshBuffer>(&pBuffer, 1, pBufferEnd);
        FmInitTetMeshBuffer(pTetMeshBuffer);

        pTetMeshBuffer->bufferNumBytes = bufferNumBytes;

        pTetMeshBuffer->tetMeshes = FmAllocFromBuffer64<FmTetMesh>(&pBuffer, maxTetMeshes, pBufferEnd);
        pTetMeshBuffer->solverData = FmAllocFromBuffer<FmMpcgSolverData>(&pBuffer, maxTetMeshes, pBufferEnd);

        FmTetMesh& tetMesh0 = pTetMeshBuffer->tetMeshes[0];
        FmMpcgSolverData& solverData0 = pTetMeshBuffer->solverData[0];

        FmInitTetMesh(&tetMesh0);
        FmInitMpcgSolverData(&solverData0);

        FmAllocTetMeshDataFromBuffer(&tetMesh0, maxVerts, numTets, maxExteriorFaces, numVertIncidentTets, enablePlasticity, enableFracture, pBuffer, pBufferEnd);

        for (uint i = 0; i < maxVerts; i++)
        {
            FmInitTetMeshVert(&tetMesh0, i, (uint16_t)(isKinematic ? FM_VERT_FLAG_KINEMATIC : 0));
        }

        for (uint i = 0; i < numTets; i++)
        {
            FmInitTetMeshTet(&tetMesh0, i);

            if (enablePlasticity)
            {
                tetMesh0.tetsPlasticity[i].plasticDeformationMatrix = FmMatrix3::identity();
#if FM_COMPUTE_PLASTIC_REL_ROTATION
                tetMesh0.tetsPlasticity[i].plasticTetRelRotation = FmMatrix3::identity();
#endif
            }
        }

        tetMesh0.numVerts = numVerts;
        tetMesh0.numTets = numTets;
        tetMesh0.numExteriorFaces = 0;
        tetMesh0.maxVerts = maxVerts;
        tetMesh0.maxExteriorFaces = maxExteriorFaces;
        tetMesh0.numTetsToFracture = 0;
        tetMesh0.numNewExteriorFaces = 0;
        tetMesh0.collisionGroup = (uint8_t)((collisionGroup < 32) ? (uint8_t)collisionGroup : 0);

        // Set initial flags to trigger connected components, adjacent vertex offset calculation, and BVH build for kinematic objects
        tetMesh0.flags = (FM_OBJECT_FLAG_NEEDS_CONNECTED_COMPONENTS | FM_OBJECT_FLAG_NEEDS_ADJACENT_VERT_OFFSETS | FM_OBJECT_FLAG_POS_CHANGED | FM_OBJECT_FLAG_VEL_CHANGED);

        if (isKinematic)
        {
            tetMesh0.flags |= FM_OBJECT_FLAG_KINEMATIC;
        }

        FmAllocMpcgSolverDataFromBuffer(&solverData0, maxVerts, maxVertAdjacentVerts, pBuffer, pBufferEnd);

        for (uint vId = 0; vId < maxVerts; vId++)
        {
            solverData0.kinematicFlags[vId] = true;
        }

        pTetMeshBuffer->fractureGroupCounts = FmAllocFromBuffer<FmFractureGroupCounts>(&pBuffer, maxTetMeshes, pBufferEnd);
        pTetMeshBuffer->visitedFractureGroup = FmAllocFromBuffer<bool>(&pBuffer, maxTetMeshes, pBufferEnd);
        pTetMeshBuffer->tetMeshVertOffsets = FmAllocFromBuffer<uint>(&pBuffer, maxTetMeshes, pBufferEnd);
        for (uint i = 0; i < maxTetMeshes; i++)
        {
            if (fractureGroupCounts)
            {
                pTetMeshBuffer->fractureGroupCounts[i] = fractureGroupCounts[i];
            }
            else
            {
                pTetMeshBuffer->fractureGroupCounts[i] = FmFractureGroupCounts();
            }

            pTetMeshBuffer->visitedFractureGroup[i] = false;
            pTetMeshBuffer->tetMeshVertOffsets[i] = 0;
        }

        pTetMeshBuffer->vertReferences = FmAllocFromBuffer<FmVertReference>(&pBuffer, maxVerts, pBufferEnd);
        pTetMeshBuffer->tetReferences = FmAllocFromBuffer<FmTetReference>(&pBuffer, numTets, pBufferEnd);

        for (uint i = 1; i < maxTetMeshes; i++)
        {
            FmInitTetMesh(&pTetMeshBuffer->tetMeshes[i]);
            FmInitMpcgSolverData(&pTetMeshBuffer->solverData[i]);
        }

        for (uint i = 0; i < maxTetMeshes; i++)
        {
            pTetMeshBuffer->tetMeshes[i].solverData = &pTetMeshBuffer->solverData[i];
        }

        pTetMeshBuffer->numVerts = numVerts;

        // Initialize the maps from the original vert and tet ids to new mesh ids
        for (uint i = 0; i < maxVerts; i++)
        {
            pTetMeshBuffer->vertReferences[i].meshIdx = 0;
            pTetMeshBuffer->vertReferences[i].vertId = i;
        }

        pTetMeshBuffer->numTets = numTets;
        for (uint i = 0; i < numTets; i++)
        {
            pTetMeshBuffer->tetReferences[i].meshIdx = 0;
            pTetMeshBuffer->tetReferences[i].tetId = i;
            pTetMeshBuffer->tetReferences[i].fractureGroupId = tetFractureGroupIds ? tetFractureGroupIds[i] : 0;
        }

        pTetMeshBuffer->numTetMeshes = 1;
        pTetMeshBuffer->maxTetMeshes = maxTetMeshes;

        if (pTetMeshPtr)
        {
            *pTetMeshPtr = &tetMesh0;
        }

        return pTetMeshBuffer;
    }

    FmTetMeshBuffer* FmCreateTetMeshBuffer(
        const FmTetMeshBufferSetupParams& params, 
        const FmFractureGroupCounts* fractureGroupCounts,
        const uint* tetFractureGroupIds,
        FmTetMesh** pTetMeshPtr)
    {
        size_t bufferNumBytes = FmGetTetMeshBufferSize(params);

        uint8_t* pBuffer = (uint8_t*)FmAlignedMalloc(bufferNumBytes, 64);
        if (pBuffer == NULL)
        {
            return NULL;
        }

        return FmSetupTetMeshBuffer(params, fractureGroupCounts, tetFractureGroupIds, pBuffer, bufferNumBytes, pTetMeshPtr);
    }

    void FmDestroyTetMeshBuffer(FmTetMeshBuffer* tetMeshBuffer)
    {
        FmAlignedFree(tetMeshBuffer);
    }

    size_t FmGetTetMeshBufferSize(const FmTetMeshBuffer& tetMeshBuffer)
    {
        return tetMeshBuffer.bufferNumBytes;
    }

    void FmAllocTetMeshData(FmTetMesh* tetMesh, uint numVerts, uint numTets, uint maxVerts, uint maxExteriorFaces, uint numVertIncidentTets, bool enablePlasticity, bool enableFracture)
    {
        tetMesh->maxVerts = maxVerts;
        tetMesh->numVerts = numVerts;
        tetMesh->numTets = numTets;
        tetMesh->numTetsToFracture = 0;
        tetMesh->numExteriorFaces = 0;
        tetMesh->numNewExteriorFaces = 0;
        tetMesh->maxExteriorFaces = maxExteriorFaces;

        uint maxBvhNodes = FmNumBvhNodes(maxExteriorFaces);

        tetMesh->vertsNeighbors = new FmVertNeighbors[maxVerts];
        tetMesh->vertsMass = new float[maxVerts];
        tetMesh->vertsFlags = new uint16_t[maxVerts];
        tetMesh->vertsIndex0 = new uint[maxVerts];
        tetMesh->vertsRestPos = new FmVector3[maxVerts];
        tetMesh->vertsPos = new FmVector3[maxVerts];
        tetMesh->vertsVel = new FmVector3[maxVerts];
        tetMesh->vertsExtForce = new FmVector3[maxVerts];
        tetMesh->vertsTetValues = new FmVertTetValues[maxVerts];
        tetMesh->tetsMass = new float[numTets];
        tetMesh->tetsFrictionCoeff = new float[numTets];
        tetMesh->tetsFlags = new uint16_t[numTets];
        tetMesh->tetsShapeParams = new FmTetShapeParams[numTets];
        tetMesh->tetsRestDensity = new float[numTets];
        tetMesh->tetsMaxUnconstrainedSolveIterations = new uint[numTets];
        tetMesh->tetsStressMaterialParams = new FmTetStressMaterialParams[numTets];
        tetMesh->tetsDeformationMaterialParams = new FmTetDeformationMaterialParams[numTets];
        tetMesh->tetsStrainMag = new float[numTets];
        tetMesh->tetsIndex0 = new uint[numTets];
        tetMesh->tetsRotation = new FmMatrix3[numTets];
        tetMesh->tetsFaceIncidentTetIds = new FmTetFaceIncidentTetIds[numTets];
        tetMesh->tetsVertIds = new FmTetVertIds[numTets];
        tetMesh->tetsStiffness = new FmTetStiffnessState[numTets];
        tetMesh->exteriorFaces = new FmExteriorFace[maxExteriorFaces];

        if (enablePlasticity)
        {
            tetMesh->tetsPlasticity = new FmTetPlasticityState[numTets];
            for (uint i = 0; i < numTets; i++)
            {
                tetMesh->tetsPlasticity[i].plasticDeformationMatrix = FmMatrix3::identity();
#if FM_COMPUTE_PLASTIC_REL_ROTATION
                tetMesh->tetsPlasticity[i].plasticTetRelRotation = FmMatrix3::identity();
#endif
            }
            tetMesh->tetsPlasticityMaterialParams = new FmTetPlasticityMaterialParams[numTets];
        }
        else
        {
            tetMesh->tetsPlasticity = NULL;
            tetMesh->tetsPlasticityMaterialParams = NULL;
        }

        if (enableFracture)
        {
            tetMesh->tetsToFracture = new FmTetToFracture[numTets];
            tetMesh->tetsFractureMaterialParams = new FmTetFractureMaterialParams[numTets];
        }
        else
        {
            tetMesh->tetsToFracture = NULL;
            tetMesh->tetsFractureMaterialParams = NULL;
        }

        tetMesh->bvh.nodes = new FmBvhNode[maxBvhNodes];
        tetMesh->bvh.primBoxes = new FmAabb[maxExteriorFaces];
        tetMesh->bvh.mortonCodesSorted = new int[maxExteriorFaces];
        tetMesh->bvh.primIndicesSorted = new int[maxExteriorFaces];

        for (uint i = 0; i < maxVerts; i++)
        {
            tetMesh->vertsFlags[i] = 0;
        }

        tetMesh->vertConnectivity.incidentTets = new uint[numVertIncidentTets];
        tetMesh->vertConnectivity.numIncidentTets = numVertIncidentTets;
        tetMesh->vertConnectivity.numIncidentTetsTotal = numVertIncidentTets;

        tetMesh->solverData = NULL;
    }

    void FmAllocTetMeshData(FmTetMesh* tetMesh, const FmTetMesh& srcMesh)
    {
        uint maxVerts = srcMesh.maxVerts;
        uint numVerts = srcMesh.numVerts;
        uint numTets = srcMesh.numTets;
        uint maxExteriorFaces = srcMesh.maxExteriorFaces;
        uint numVertIncidentTets = srcMesh.vertConnectivity.numIncidentTetsTotal;  // Set incident tets size to numIncidentTetsTotal so that src incidentTetsStart values are valid

        FmAllocTetMeshData(tetMesh, numVerts, numTets, maxVerts, maxExteriorFaces, numVertIncidentTets, (srcMesh.tetsPlasticity != NULL), (srcMesh.tetsToFracture != NULL));
    }

    void FmCopyTetMesh(FmTetMesh* dstMesh, const FmTetMesh& srcMesh)
    {
        dstMesh->objectId = srcMesh.objectId;

        for (uint i = 0; i < srcMesh.maxVerts; i++)
        {
            dstMesh->vertsNeighbors[i] = srcMesh.vertsNeighbors[i];
            dstMesh->vertsMass[i] = srcMesh.vertsMass[i];
            dstMesh->vertsFlags[i] = srcMesh.vertsFlags[i];
            dstMesh->vertsIndex0[i] = srcMesh.vertsIndex0[i];
            dstMesh->vertsRestPos[i] = srcMesh.vertsRestPos[i];
            dstMesh->vertsPos[i] = srcMesh.vertsPos[i];
            dstMesh->vertsVel[i] = srcMesh.vertsVel[i];
            dstMesh->vertsExtForce[i] = srcMesh.vertsExtForce[i];
            dstMesh->vertsTetValues[i] = srcMesh.vertsTetValues[i];
        }

        for (uint i = 0; i < srcMesh.numTets; i++)
        {
            dstMesh->tetsMass[i] = srcMesh.tetsMass[i];
            dstMesh->tetsFrictionCoeff[i] = srcMesh.tetsFrictionCoeff[i];
            dstMesh->tetsFlags[i] = srcMesh.tetsFlags[i];
            dstMesh->tetsShapeParams[i] = srcMesh.tetsShapeParams[i];
            dstMesh->tetsRestDensity[i] = srcMesh.tetsRestDensity[i];
            dstMesh->tetsMaxUnconstrainedSolveIterations[i] = srcMesh.tetsMaxUnconstrainedSolveIterations[i];
            dstMesh->tetsStressMaterialParams[i] = srcMesh.tetsStressMaterialParams[i];
            dstMesh->tetsDeformationMaterialParams[i] = srcMesh.tetsDeformationMaterialParams[i];
            dstMesh->tetsStrainMag[i] = srcMesh.tetsStrainMag[i];
            dstMesh->tetsIndex0[i] = srcMesh.tetsIndex0[i];
            dstMesh->tetsRotation[i] = srcMesh.tetsRotation[i];
            dstMesh->tetsFaceIncidentTetIds[i] = srcMesh.tetsFaceIncidentTetIds[i];
            dstMesh->tetsVertIds[i] = srcMesh.tetsVertIds[i];
            dstMesh->tetsStiffness[i] = srcMesh.tetsStiffness[i];
        }

        uint maxExteriorFaces = srcMesh.maxExteriorFaces;
        for (uint i = 0; i < maxExteriorFaces; i++)
        {
            dstMesh->exteriorFaces[i] = srcMesh.exteriorFaces[i];
            dstMesh->bvh.mortonCodesSorted[i] = srcMesh.bvh.mortonCodesSorted[i];
            dstMesh->bvh.primIndicesSorted[i] = srcMesh.bvh.primIndicesSorted[i];
            dstMesh->bvh.primBoxes[i] = srcMesh.bvh.primBoxes[i];
        }

        uint maxBvhNodes = FmNumBvhNodes(maxExteriorFaces);
        for (uint i = 0; i < maxBvhNodes; i++)
        {
            dstMesh->bvh.nodes[i] = srcMesh.bvh.nodes[i];
            dstMesh->bvh.nodes[i].box = srcMesh.bvh.nodes[i].box;
        }
        dstMesh->bvh.numPrims = srcMesh.bvh.numPrims;

        if (srcMesh.tetsToFracture)
        {
            for (uint i = 0; i < srcMesh.numTets; i++)
            {
                dstMesh->tetsToFracture[i] = srcMesh.tetsToFracture[i];
                dstMesh->tetsFractureMaterialParams[i] = srcMesh.tetsFractureMaterialParams[i];
            }
        }
        if (srcMesh.tetsPlasticity)
        {
            for (uint i = 0; i < srcMesh.numTets; i++)
            {
                dstMesh->tetsPlasticity[i] = srcMesh.tetsPlasticity[i];
                dstMesh->tetsPlasticityMaterialParams[i] = srcMesh.tetsPlasticityMaterialParams[i];
            }
        }

        dstMesh->vertConnectivity.numIncidentTets = srcMesh.vertConnectivity.numIncidentTets;
        dstMesh->vertConnectivity.numIncidentTetsTotal = srcMesh.vertConnectivity.numIncidentTetsTotal;
        dstMesh->vertConnectivity.numAdjacentVerts = srcMesh.vertConnectivity.numAdjacentVerts;

        for (uint i = 0; i < srcMesh.vertConnectivity.numIncidentTets; i++)
        {
            dstMesh->vertConnectivity.incidentTets[i] = srcMesh.vertConnectivity.incidentTets[i];
        }

        dstMesh->numVerts = srcMesh.numVerts;
        dstMesh->numTets = srcMesh.numTets;
        dstMesh->numExteriorFaces = srcMesh.numExteriorFaces;
        dstMesh->maxVerts = srcMesh.maxVerts;
        dstMesh->maxExteriorFaces = srcMesh.maxExteriorFaces;
        dstMesh->numTetsToFracture = srcMesh.numTetsToFracture;
        dstMesh->numNewExteriorFaces = srcMesh.numNewExteriorFaces;
        dstMesh->maxUnconstrainedSolveIterations = srcMesh.maxUnconstrainedSolveIterations;
        dstMesh->frictionCoeff = srcMesh.frictionCoeff;
        dstMesh->removeKinematicStressThreshold = srcMesh.removeKinematicStressThreshold;
        dstMesh->extForceSpeedLimit = srcMesh.extForceSpeedLimit;
        dstMesh->resetSpeedLimit = srcMesh.resetSpeedLimit;
        dstMesh->flags = srcMesh.flags;
        dstMesh->collisionGroup = srcMesh.collisionGroup;
    }

    void FmCopySubsetTetMesh(FmTetMesh* dstMesh, uint* remapVertIds, const FmTetMesh& srcMesh, const uint* tetIds, uint numTets)
    {
        // Iterate through tets to find all the vertices in the subset mesh, create map from old to new vert ids, copy vertex data, and create incident tet arrays
        dstMesh->vertConnectivity.numIncidentTets = 0;

        uint numVerts = 0;
        for (uint tIdx = 0; tIdx < numTets; tIdx++)
        {
            uint srcTetId = tetIds[tIdx];
            FmTetVertIds srcVertIds = srcMesh.tetsVertIds[srcTetId];

            for (uint cornerIdx = 0; cornerIdx < 4; cornerIdx++)
            {
                uint srcVertId = srcVertIds.ids[cornerIdx];
                if (remapVertIds[srcVertId] == FM_INVALID_ID)
                {
                    // New vertex, set map and copy vert data
                    uint dstVertId = numVerts;
                    remapVertIds[srcVertId] = dstVertId;

                    FmVertNeighbors& dstVertNeighbors = dstMesh->vertsNeighbors[dstVertId];
                    dstVertNeighbors = srcMesh.vertsNeighbors[srcVertId];

                    dstMesh->vertsMass[dstVertId] = srcMesh.vertsMass[srcVertId];
                    dstMesh->vertsFlags[dstVertId] = srcMesh.vertsFlags[srcVertId];
                    dstMesh->vertsIndex0[dstVertId] = srcMesh.vertsIndex0[srcVertId];
                    dstMesh->vertsRestPos[dstVertId] = srcMesh.vertsRestPos[srcVertId];
                    dstMesh->vertsPos[dstVertId] = srcMesh.vertsPos[srcVertId];
                    dstMesh->vertsVel[dstVertId] = srcMesh.vertsVel[srcVertId];
                    dstMesh->vertsTetValues[dstVertId] = srcMesh.vertsTetValues[srcVertId];
                    FmResetForceOnVert(dstMesh, dstVertId);

                    // Put incident tets at same start offset
                    dstMesh->vertConnectivity.incidentTets[dstVertNeighbors.incidentTetsStart] = tIdx;
                    dstVertNeighbors.numIncidentTets = 1;
                    dstVertNeighbors.numAdjacentVerts = 0;
                    numVerts++;
                }
                else
                {
                    // Previously seen vertex, update incident tet list
                    FmVertNeighbors& dstVertNeighbors = dstMesh->vertsNeighbors[remapVertIds[srcVertId]];

                    dstMesh->vertConnectivity.incidentTets[dstVertNeighbors.incidentTetsStart + dstVertNeighbors.numIncidentTets] = tIdx;
                    dstVertNeighbors.numIncidentTets++;
                }
                dstMesh->vertConnectivity.numIncidentTets++;
            }
        }

        // Copy tet data
        for (uint tIdx = 0; tIdx < numTets; tIdx++)
        {
            dstMesh->tetsMass[tIdx] = srcMesh.tetsMass[tetIds[tIdx]];
            dstMesh->tetsFrictionCoeff[tIdx] = srcMesh.tetsFrictionCoeff[tetIds[tIdx]];
            dstMesh->tetsFlags[tIdx] = srcMesh.tetsFlags[tetIds[tIdx]];
            dstMesh->tetsShapeParams[tIdx] = srcMesh.tetsShapeParams[tetIds[tIdx]];

            dstMesh->tetsRestDensity[tIdx] = srcMesh.tetsRestDensity[tetIds[tIdx]];
            dstMesh->tetsMaxUnconstrainedSolveIterations[tIdx] = srcMesh.tetsMaxUnconstrainedSolveIterations[tetIds[tIdx]];
            dstMesh->tetsStressMaterialParams[tIdx] = srcMesh.tetsStressMaterialParams[tetIds[tIdx]];
            dstMesh->tetsDeformationMaterialParams[tIdx] = srcMesh.tetsDeformationMaterialParams[tetIds[tIdx]];
            dstMesh->tetsStrainMag[tIdx] = srcMesh.tetsStrainMag[tetIds[tIdx]];
            dstMesh->tetsIndex0[tIdx] = srcMesh.tetsIndex0[tetIds[tIdx]];
            dstMesh->tetsRotation[tIdx] = srcMesh.tetsRotation[tetIds[tIdx]];
            dstMesh->tetsFaceIncidentTetIds[tIdx] = srcMesh.tetsFaceIncidentTetIds[tetIds[tIdx]];
            dstMesh->tetsVertIds[tIdx] = srcMesh.tetsVertIds[tetIds[tIdx]];
            dstMesh->tetsStiffness[tIdx] = srcMesh.tetsStiffness[tetIds[tIdx]];

            // remap vert ids in tets
            dstMesh->tetsVertIds[tIdx].ids[0] = remapVertIds[dstMesh->tetsVertIds[tIdx].ids[0]];
            dstMesh->tetsVertIds[tIdx].ids[1] = remapVertIds[dstMesh->tetsVertIds[tIdx].ids[1]];
            dstMesh->tetsVertIds[tIdx].ids[2] = remapVertIds[dstMesh->tetsVertIds[tIdx].ids[2]];
            dstMesh->tetsVertIds[tIdx].ids[3] = remapVertIds[dstMesh->tetsVertIds[tIdx].ids[3]];
        }

        if (dstMesh->tetsPlasticity && srcMesh.tetsPlasticity)
        {
            for (uint i = 0; i < numTets; i++)
            {
                dstMesh->tetsPlasticity[i] = srcMesh.tetsPlasticity[tetIds[i]];
                dstMesh->tetsPlasticityMaterialParams[i] = srcMesh.tetsPlasticityMaterialParams[tetIds[i]];
            }
        }

        if (dstMesh->tetsToFracture && srcMesh.tetsToFracture)
        {
            for (uint i = 0; i < numTets; i++)
            {
                dstMesh->tetsFractureMaterialParams[i] = srcMesh.tetsFractureMaterialParams[tetIds[i]];
            }
        }

        dstMesh->vertConnectivity.numAdjacentVerts = 0;

        dstMesh->numVerts = numVerts;
        dstMesh->numTets = numTets;
        dstMesh->numExteriorFaces = 0;
        dstMesh->maxVerts = srcMesh.maxVerts;
        dstMesh->maxExteriorFaces = srcMesh.maxExteriorFaces;
        dstMesh->numTetsToFracture = 0;
        dstMesh->numNewExteriorFaces = 0;
        dstMesh->maxUnconstrainedSolveIterations = srcMesh.maxUnconstrainedSolveIterations;
        dstMesh->frictionCoeff = srcMesh.frictionCoeff;
        dstMesh->removeKinematicStressThreshold = srcMesh.removeKinematicStressThreshold;
        dstMesh->extForceSpeedLimit = srcMesh.extForceSpeedLimit;
        dstMesh->resetSpeedLimit = srcMesh.resetSpeedLimit;
        dstMesh->flags = srcMesh.flags;
        dstMesh->collisionGroup = srcMesh.collisionGroup;

        // Finish connectivity initialization
        FmFinishConnectivityFromVertIncidentTets(dstMesh);

        // Reset the remapVertIds for another use
        for (uint idx = 0; idx < numTets; idx++)
        {
            uint tId = tetIds[idx];
            FmTetVertIds vertIds = srcMesh.tetsVertIds[tId];
            remapVertIds[vertIds.ids[0]] = FM_INVALID_ID;
            remapVertIds[vertIds.ids[1]] = FM_INVALID_ID;
            remapVertIds[vertIds.ids[2]] = FM_INVALID_ID;
            remapVertIds[vertIds.ids[3]] = FM_INVALID_ID;
        }
    }

    void FmWriteTetMesh(const char* filename, FmTetMesh* tetMesh)
    {
        FILE* fp;
        fopen_s(&fp, filename, "wb");

        if (!fp)
        {
            return;
        }
        fwrite(tetMesh, sizeof(*tetMesh), 1, fp);

        fwrite(tetMesh->vertsNeighbors, sizeof(*tetMesh->vertsNeighbors), tetMesh->maxVerts, fp);
        fwrite(tetMesh->vertsMass, sizeof(*tetMesh->vertsMass), tetMesh->maxVerts, fp);
        fwrite(tetMesh->vertsFlags, sizeof(*tetMesh->vertsFlags), tetMesh->maxVerts, fp);
        fwrite(tetMesh->vertsIndex0, sizeof(*tetMesh->vertsIndex0), tetMesh->maxVerts, fp);
        fwrite(tetMesh->vertsRestPos, sizeof(*tetMesh->vertsRestPos), tetMesh->maxVerts, fp);
        fwrite(tetMesh->vertsPos, sizeof(*tetMesh->vertsPos), tetMesh->maxVerts, fp);
        fwrite(tetMesh->vertsVel, sizeof(*tetMesh->vertsVel), tetMesh->maxVerts, fp);
        fwrite(tetMesh->vertsExtForce, sizeof(*tetMesh->vertsExtForce), tetMesh->maxVerts, fp);
        fwrite(tetMesh->vertsTetValues, sizeof(*tetMesh->vertsTetValues), tetMesh->maxVerts, fp);
        fwrite(tetMesh->vertConnectivity.incidentTets, sizeof(*tetMesh->vertConnectivity.incidentTets), tetMesh->vertConnectivity.numIncidentTets, fp);

        fwrite(tetMesh->tetsMass, sizeof(*tetMesh->tetsMass), tetMesh->numTets, fp);
        fwrite(tetMesh->tetsFrictionCoeff, sizeof(*tetMesh->tetsFrictionCoeff), tetMesh->numTets, fp);
        fwrite(tetMesh->tetsFlags, sizeof(*tetMesh->tetsFlags), tetMesh->numTets, fp);
        fwrite(tetMesh->tetsShapeParams, sizeof(*tetMesh->tetsShapeParams), tetMesh->numTets, fp);
        fwrite(tetMesh->tetsRestDensity, sizeof(*tetMesh->tetsRestDensity), tetMesh->numTets, fp);
        fwrite(tetMesh->tetsMaxUnconstrainedSolveIterations, sizeof(*tetMesh->tetsMaxUnconstrainedSolveIterations), tetMesh->numTets, fp);
        fwrite(tetMesh->tetsStressMaterialParams, sizeof(*tetMesh->tetsStressMaterialParams), tetMesh->numTets, fp);
        fwrite(tetMesh->tetsDeformationMaterialParams, sizeof(*tetMesh->tetsDeformationMaterialParams), tetMesh->numTets, fp);
        fwrite(tetMesh->tetsStrainMag, sizeof(*tetMesh->tetsStrainMag), tetMesh->numTets, fp);
        fwrite(tetMesh->tetsIndex0, sizeof(*tetMesh->tetsIndex0), tetMesh->numTets, fp);
        fwrite(tetMesh->tetsRotation, sizeof(*tetMesh->tetsRotation), tetMesh->numTets, fp);
        fwrite(tetMesh->tetsFaceIncidentTetIds, sizeof(*tetMesh->tetsFaceIncidentTetIds), tetMesh->numTets, fp);
        fwrite(tetMesh->tetsVertIds, sizeof(*tetMesh->tetsVertIds), tetMesh->numTets, fp);
        fwrite(tetMesh->tetsStiffness, sizeof(*tetMesh->tetsStiffness), tetMesh->numTets, fp);

        fwrite(tetMesh->exteriorFaces, sizeof(*tetMesh->exteriorFaces), tetMesh->maxExteriorFaces, fp);

        if (tetMesh->tetsToFracture)
        {
            fwrite(tetMesh->tetsToFracture, sizeof(*tetMesh->tetsToFracture), tetMesh->numTets, fp);
            fwrite(tetMesh->tetsFractureMaterialParams, sizeof(*tetMesh->tetsFractureMaterialParams), tetMesh->numTets, fp);
        }

        if (tetMesh->tetsPlasticity)
        {
            fwrite(tetMesh->tetsPlasticity, sizeof(*tetMesh->tetsPlasticity), tetMesh->numTets, fp);
            fwrite(tetMesh->tetsPlasticityMaterialParams, sizeof(*tetMesh->tetsPlasticityMaterialParams), tetMesh->numTets, fp);
        }

        uint maxBvhNodes = FmNumBvhNodes(tetMesh->maxExteriorFaces);
        fwrite(tetMesh->bvh.nodes, sizeof(*tetMesh->bvh.nodes), maxBvhNodes, fp);
        fwrite(tetMesh->bvh.primBoxes, sizeof(*tetMesh->bvh.primBoxes), tetMesh->maxExteriorFaces, fp);
        fwrite(tetMesh->bvh.mortonCodesSorted, sizeof(*tetMesh->bvh.mortonCodesSorted), tetMesh->maxExteriorFaces, fp);
        fwrite(tetMesh->bvh.primIndicesSorted, sizeof(*tetMesh->bvh.primIndicesSorted), tetMesh->maxExteriorFaces, fp);

        fclose(fp);
    }

    void FmReadTetMesh(const char* filename, FmTetMesh* tetMesh)
    {
        FILE* fp;
        fopen_s(&fp, filename, "rb");

        if (!fp)
        {
            return;
        }

        fread(tetMesh, sizeof(*tetMesh), 1, fp);
        tetMesh->solverData = NULL;

        tetMesh->vertsNeighbors = new FmVertNeighbors[tetMesh->maxVerts];
        tetMesh->vertsMass = new float[tetMesh->maxVerts];
        tetMesh->vertsFlags = new uint16_t[tetMesh->maxVerts];
        tetMesh->vertsIndex0 = new uint[tetMesh->maxVerts];
        tetMesh->vertsRestPos = new FmVector3[tetMesh->maxVerts];
        tetMesh->vertsPos = new FmVector3[tetMesh->maxVerts];
        tetMesh->vertsVel = new FmVector3[tetMesh->maxVerts];
        tetMesh->vertsExtForce = new FmVector3[tetMesh->maxVerts];
        tetMesh->vertsTetValues = new FmVertTetValues[tetMesh->maxVerts];
        tetMesh->vertConnectivity.incidentTets = new uint[tetMesh->vertConnectivity.numIncidentTets];

        tetMesh->tetsMass = new float[tetMesh->numTets];
        tetMesh->tetsFrictionCoeff = new float[tetMesh->numTets];
        tetMesh->tetsFlags = new uint16_t[tetMesh->numTets];
        tetMesh->tetsShapeParams = new FmTetShapeParams[tetMesh->numTets];
        tetMesh->tetsRestDensity = new float[tetMesh->numTets];
        tetMesh->tetsMaxUnconstrainedSolveIterations = new uint[tetMesh->numTets];
        tetMesh->tetsStressMaterialParams = new FmTetStressMaterialParams[tetMesh->numTets];
        tetMesh->tetsDeformationMaterialParams = new FmTetDeformationMaterialParams[tetMesh->numTets];
        tetMesh->tetsStrainMag = new float[tetMesh->numTets];
        tetMesh->tetsIndex0 = new uint[tetMesh->numTets];
        tetMesh->tetsRotation = new FmMatrix3[tetMesh->numTets];
        tetMesh->tetsFaceIncidentTetIds = new FmTetFaceIncidentTetIds[tetMesh->numTets];
        tetMesh->tetsVertIds = new FmTetVertIds[tetMesh->numTets];
        tetMesh->tetsStiffness = new FmTetStiffnessState[tetMesh->numTets];

        tetMesh->exteriorFaces = new FmExteriorFace[tetMesh->maxExteriorFaces];

        if (tetMesh->tetsToFracture)
        {
            tetMesh->tetsToFracture = new FmTetToFracture[tetMesh->numTets];
            tetMesh->tetsFractureMaterialParams = new FmTetFractureMaterialParams[tetMesh->numTets];
        }

        if (tetMesh->tetsPlasticity)
        {
            tetMesh->tetsPlasticity = new FmTetPlasticityState[tetMesh->numTets];
            tetMesh->tetsPlasticityMaterialParams = new FmTetPlasticityMaterialParams[tetMesh->numTets];
        }

        uint maxBvhNodes = FmNumBvhNodes(tetMesh->maxExteriorFaces);
        tetMesh->bvh.nodes = new FmBvhNode[maxBvhNodes];
        tetMesh->bvh.primBoxes = new FmAabb[tetMesh->maxExteriorFaces];
        tetMesh->bvh.mortonCodesSorted = new int[tetMesh->maxExteriorFaces];
        tetMesh->bvh.primIndicesSorted = new int[tetMesh->maxExteriorFaces];

        fread(tetMesh->vertsNeighbors, sizeof(*tetMesh->vertsNeighbors), tetMesh->maxVerts, fp);
        fread(tetMesh->vertsMass, sizeof(*tetMesh->vertsMass), tetMesh->maxVerts, fp);
        fread(tetMesh->vertsFlags, sizeof(*tetMesh->vertsFlags), tetMesh->maxVerts, fp);
        fread(tetMesh->vertsIndex0, sizeof(*tetMesh->vertsIndex0), tetMesh->maxVerts, fp);
        fread(tetMesh->vertsRestPos, sizeof(*tetMesh->vertsRestPos), tetMesh->maxVerts, fp);
        fread(tetMesh->vertsPos, sizeof(*tetMesh->vertsPos), tetMesh->maxVerts, fp);
        fread(tetMesh->vertsVel, sizeof(*tetMesh->vertsVel), tetMesh->maxVerts, fp);
        fread(tetMesh->vertsExtForce, sizeof(*tetMesh->vertsExtForce), tetMesh->maxVerts, fp);
        fread(tetMesh->vertsTetValues, sizeof(*tetMesh->vertsTetValues), tetMesh->maxVerts, fp);
        fread(tetMesh->vertConnectivity.incidentTets, sizeof(*tetMesh->vertConnectivity.incidentTets), tetMesh->vertConnectivity.numIncidentTets, fp);

        fread(tetMesh->tetsMass, sizeof(*tetMesh->tetsMass), tetMesh->numTets, fp);
        fread(tetMesh->tetsFrictionCoeff, sizeof(*tetMesh->tetsFrictionCoeff), tetMesh->numTets, fp);
        fread(tetMesh->tetsFlags, sizeof(*tetMesh->tetsFlags), tetMesh->numTets, fp);
        fread(tetMesh->tetsShapeParams, sizeof(*tetMesh->tetsShapeParams), tetMesh->numTets, fp);        
        fread(tetMesh->tetsRestDensity, sizeof(*tetMesh->tetsRestDensity), tetMesh->numTets, fp);
        fread(tetMesh->tetsMaxUnconstrainedSolveIterations, sizeof(*tetMesh->tetsMaxUnconstrainedSolveIterations), tetMesh->numTets, fp);
        fread(tetMesh->tetsStressMaterialParams, sizeof(*tetMesh->tetsStressMaterialParams), tetMesh->numTets, fp);
        fread(tetMesh->tetsDeformationMaterialParams, sizeof(*tetMesh->tetsDeformationMaterialParams), tetMesh->numTets, fp);
        fread(tetMesh->tetsStrainMag, sizeof(*tetMesh->tetsStrainMag), tetMesh->numTets, fp);
        fread(tetMesh->tetsIndex0, sizeof(*tetMesh->tetsIndex0), tetMesh->numTets, fp);
        fread(tetMesh->tetsRotation, sizeof(*tetMesh->tetsRotation), tetMesh->numTets, fp);
        fread(tetMesh->tetsFaceIncidentTetIds, sizeof(*tetMesh->tetsFaceIncidentTetIds), tetMesh->numTets, fp);
        fread(tetMesh->tetsVertIds, sizeof(*tetMesh->tetsVertIds), tetMesh->numTets, fp);
        fread(tetMesh->tetsStiffness, sizeof(*tetMesh->tetsStiffness), tetMesh->numTets, fp);

        fread(tetMesh->exteriorFaces, sizeof(*tetMesh->exteriorFaces), tetMesh->maxExteriorFaces, fp);

        if (tetMesh->tetsToFracture)
        {
            fread(tetMesh->tetsToFracture, sizeof(*tetMesh->tetsToFracture), tetMesh->numTets, fp);
            fread(tetMesh->tetsFractureMaterialParams, sizeof(*tetMesh->tetsFractureMaterialParams), tetMesh->numTets, fp);
        }

        if (tetMesh->tetsPlasticity)
        {
            fread(tetMesh->tetsPlasticity, sizeof(*tetMesh->tetsPlasticity), tetMesh->numTets, fp);
            fread(tetMesh->tetsPlasticityMaterialParams, sizeof(*tetMesh->tetsPlasticityMaterialParams), tetMesh->numTets, fp);
        }

        fread(tetMesh->bvh.nodes, sizeof(*tetMesh->bvh.nodes), maxBvhNodes, fp);
        fread(tetMesh->bvh.primBoxes, sizeof(*tetMesh->bvh.primBoxes), tetMesh->maxExteriorFaces, fp);
        fread(tetMesh->bvh.mortonCodesSorted, sizeof(*tetMesh->bvh.mortonCodesSorted), tetMesh->maxExteriorFaces, fp);
        fread(tetMesh->bvh.primIndicesSorted, sizeof(*tetMesh->bvh.primIndicesSorted), tetMesh->maxExteriorFaces, fp);

        fclose(fp);
    }

    void FmFreeTetMeshData(FmTetMesh* tetMesh)
    {
        FM_DELETE_ARRAY(tetMesh->vertsNeighbors);
        FM_DELETE_ARRAY(tetMesh->vertsMass);
        FM_DELETE_ARRAY(tetMesh->vertsFlags);
        FM_DELETE_ARRAY(tetMesh->vertsIndex0);
        FM_DELETE_ARRAY(tetMesh->vertsRestPos);
        FM_DELETE_ARRAY(tetMesh->vertsPos);
        FM_DELETE_ARRAY(tetMesh->vertsVel);
        FM_DELETE_ARRAY(tetMesh->vertsExtForce);
        FM_DELETE_ARRAY(tetMesh->vertsTetValues);
        FM_DELETE_ARRAY(tetMesh->tetsMass);
        FM_DELETE_ARRAY(tetMesh->tetsFrictionCoeff);
        FM_DELETE_ARRAY(tetMesh->tetsFlags);
        FM_DELETE_ARRAY(tetMesh->tetsShapeParams);
        FM_DELETE_ARRAY(tetMesh->tetsRestDensity);
        FM_DELETE_ARRAY(tetMesh->tetsMaxUnconstrainedSolveIterations);
        FM_DELETE_ARRAY(tetMesh->tetsStressMaterialParams);
        FM_DELETE_ARRAY(tetMesh->tetsDeformationMaterialParams);
        FM_DELETE_ARRAY(tetMesh->tetsFractureMaterialParams);
        FM_DELETE_ARRAY(tetMesh->tetsPlasticityMaterialParams);
        FM_DELETE_ARRAY(tetMesh->tetsStrainMag);
        FM_DELETE_ARRAY(tetMesh->tetsIndex0);
        FM_DELETE_ARRAY(tetMesh->tetsRotation);
        FM_DELETE_ARRAY(tetMesh->tetsFaceIncidentTetIds);
        FM_DELETE_ARRAY(tetMesh->tetsVertIds);
        FM_DELETE_ARRAY(tetMesh->tetsStiffness);
        FM_DELETE_ARRAY(tetMesh->exteriorFaces);

        FM_DELETE_ARRAY(tetMesh->tetsToFracture);

        FM_DELETE_ARRAY(tetMesh->tetsPlasticity);

        FM_DELETE_ARRAY(tetMesh->bvh.nodes);
        FM_DELETE_ARRAY(tetMesh->bvh.primBoxes);
        FM_DELETE_ARRAY(tetMesh->bvh.mortonCodesSorted);
        FM_DELETE_ARRAY(tetMesh->bvh.primIndicesSorted);

        FM_DELETE_ARRAY(tetMesh->vertConnectivity.incidentTets);

        tetMesh->numVerts = 0;
        tetMesh->numTets = 0;
        tetMesh->numExteriorFaces = 0;
        tetMesh->maxVerts = 0;
        tetMesh->maxExteriorFaces = 0;
        tetMesh->numTetsToFracture = 0;
        tetMesh->numNewExteriorFaces = 0;
        tetMesh->flags = 0;
        tetMesh->collisionGroup = 0;
        tetMesh->vertConnectivity.numAdjacentVerts = 0;
        tetMesh->vertConnectivity.numIncidentTets = 0;
        tetMesh->vertConnectivity.numIncidentTetsTotal = 0;

        tetMesh->solverData = NULL;
    }

    // Assign verts and edges to exterior faces, which can accelerate collision detection or reduce number of contacts.
    // Also link exterior faces at their edges.
    // After mesh initialization, only processing the new exterior faces created by fracture.
    void FmAssignFeaturesToExteriorFaces(FmTetMesh* tetMesh, uint beginIndex, uint numFaces)
    {
        uint* incidentTetsArray = tetMesh->vertConnectivity.incidentTets;

        for (uint exteriorFaceId = beginIndex; exteriorFaceId < beginIndex + numFaces; exteriorFaceId++)
        {
            FmExteriorFace& exteriorFace = tetMesh->exteriorFaces[exteriorFaceId];

            // Replace opposing tet id with buffer tet id, used to create fracture contact
            FM_ASSERT(exteriorFace.opposingTetId == FM_INVALID_ID || exteriorFace.opposingTetId < tetMesh->numTets);

            exteriorFace.opposingTetId = (exteriorFace.opposingTetId == FM_INVALID_ID) ?
                FM_INVALID_ID :
                tetMesh->tetsIndex0[exteriorFace.opposingTetId];  // buffer tet id

            uint faceId = exteriorFace.faceId;
            uint tetId = exteriorFace.tetId;
            FmTetVertIds tetVerts = tetMesh->tetsVertIds[tetId];

            FmFaceVertIds faceVerts;
            FmGetFaceVertIds(&faceVerts, faceId, tetVerts);

            // Assign any unassigned vertices or edges
            bool vertIsAssigned[3] = { false, false, false };
            bool edgeIsAssigned[3] = { false, false, false };

            // After fracture it's possible for multiple exterior surfaces to meet at an edge.
            // To prevent issues with future topology/assignment updates, will treat this as a hole 
            // and connect surfaces through, and also duplicate assignments in the different surfaces.
            // Do this by looping through face-incident tets until exterior face is found.
            FmEdgeIncidentFaces edgeIncidentFaces;
            FmGetTetFacesAtEdges(&edgeIncidentFaces, faceId);

            FmTetFaceIncidentTetIds faceIncidentTets = tetMesh->tetsFaceIncidentTetIds[tetId];

            for (uint edgeId = 0; edgeId < 3; edgeId++)
            {
                uint edgeVertId0 = faceVerts.ids[edgeId];
                uint edgeVertId1 = faceVerts.ids[(edgeId + 1) % 3];

                // Loop over tets that are connected at edge by face connections, until exterior face is found
                uint currentTetId = tetId;
                uint currentFaceId = faceId;
                uint currentEdgeId = edgeId;
                FmTetVertIds currentTetVerts = tetVerts;
                FmTetFaceIncidentTetIds currentFaceIncidentTets = faceIncidentTets;
                FmEdgeIncidentFaces currentEdgeIncidentFaces = edgeIncidentFaces;

                bool foundExteriorFace = false;
                do
                {
                    uint incidentFaceId = currentEdgeIncidentFaces.faceIds[currentEdgeId];
                    uint tetOrExtFaceId = currentFaceIncidentTets.ids[incidentFaceId];

                    if (FmIsExteriorFaceId(tetOrExtFaceId))
                    {
                        // Check exterior face for assigments of verts and edge, and make edge links
                        uint incidentExteriorFaceId = FmGetExteriorFaceId(tetOrExtFaceId);

                        FmExteriorFace& incidentExteriorFace = tetMesh->exteriorFaces[incidentExteriorFaceId];

                        FM_ASSERT(currentTetId == incidentExteriorFace.tetId);

                        FmFaceVertIds incidentFaceVerts;
                        FmGetFaceVertIds(&incidentFaceVerts, incidentFaceId, currentTetVerts);

                        uint incidentEdgeId = currentEdgeId; // incident face at edge has same edge number

                                                             // Check for ownership of edge.
                        if (incidentExteriorFace.OwnsEdge(incidentEdgeId))
                        {
                            edgeIsAssigned[edgeId] = true;
                        }

                        // Also check for ownership of verts, though may still need to check all incident exterior faces later.
                        // Edge verts on incident face are reversed.
                        if (incidentExteriorFace.OwnsVert(incidentEdgeId))
                        {
                            vertIsAssigned[(edgeId + 1) % 3] = true;
                        }

                        if (incidentExteriorFace.OwnsVert((incidentEdgeId + 1) % 3))
                        {
                            vertIsAssigned[edgeId] = true;
                        }

                        // Update edge-incident face ids.
                        exteriorFace.edgeIncidentFaceIds[edgeId] = incidentExteriorFaceId;
                        incidentExteriorFace.edgeIncidentFaceIds[incidentEdgeId] = exteriorFaceId;

                        foundExteriorFace = true;
                    }
                    else
                    {
                        // Jump to new tet incident at face
                        uint nextTetId = tetOrExtFaceId;
                        FmTetFaceIncidentTetIds nextFaceIncidentTets = tetMesh->tetsFaceIncidentTetIds[nextTetId];

                        uint nextFaceId = 0;
                        nextFaceId = nextFaceIncidentTets.ids[1] == currentTetId ? 1 : nextFaceId;
                        nextFaceId = nextFaceIncidentTets.ids[2] == currentTetId ? 2 : nextFaceId;
                        nextFaceId = nextFaceIncidentTets.ids[3] == currentTetId ? 3 : nextFaceId;

                        FM_ASSERT(nextFaceIncidentTets.ids[nextFaceId] == currentTetId);

                        FmTetVertIds nextTetVerts = tetMesh->tetsVertIds[nextTetId];
                        FmFaceVertIds nextFaceVerts;
                        FmGetFaceVertIds(&nextFaceVerts, nextFaceId, nextTetVerts);

                        // Find edge of next face corresponding to original edge.
                        // Matching simplified by traversal order and CCW face verts
                        uint nextEdgeId = 0;
                        nextEdgeId = (nextFaceVerts.ids[1] == edgeVertId0 && nextFaceVerts.ids[2] == edgeVertId1) ? 1 : nextEdgeId;
                        nextEdgeId = (nextFaceVerts.ids[2] == edgeVertId0 && nextFaceVerts.ids[0] == edgeVertId1) ? 2 : nextEdgeId;

                        FM_ASSERT(nextFaceVerts.ids[nextEdgeId] == edgeVertId0 && nextFaceVerts.ids[(nextEdgeId + 1) % 3] == edgeVertId1);

                        FmEdgeIncidentFaces nextEdgeIncidentFaces;
                        FmGetTetFacesAtEdges(&nextEdgeIncidentFaces, nextFaceId);

                        // Jump to next tet
                        currentTetId = nextTetId;
                        currentFaceId = nextFaceId;
                        currentEdgeId = nextEdgeId;
                        currentTetVerts = nextTetVerts;
                        currentFaceIncidentTets = nextFaceIncidentTets;
                        currentEdgeIncidentFaces = nextEdgeIncidentFaces;
                    }
                } while (!foundExteriorFace);
            }

            // Update vert assignments by searching all incident tets
            for (uint faceCornerId = 0; faceCornerId < 3; faceCornerId++)
            {
                uint vId = faceVerts.ids[faceCornerId];

                vertIsAssigned[faceCornerId] = (tetMesh->vertsFlags[vId] & FM_VERT_FLAG_ASSIGNMENT_UPDATED) || vertIsAssigned[faceCornerId];

                if (vertIsAssigned[faceCornerId])
                    continue;

                // Search for unassigned vertices among incident tets.
                FmVertNeighbors& vertNeighbors = tetMesh->vertsNeighbors[vId];
                uint* vertIncidentTets = &incidentTetsArray[vertNeighbors.incidentTetsStart];
                uint numIncidentTets = vertNeighbors.numIncidentTets;

                for (uint itId = 0; itId < numIncidentTets && !vertIsAssigned[faceCornerId]; itId++)
                {
                    uint incidentTetId = vertIncidentTets[itId];
                    FmTetVertIds incidentTetVerts = tetMesh->tetsVertIds[incidentTetId];
                    const FmTetFaceIncidentTetIds& incidentTetFaceIncidentTets = tetMesh->tetsFaceIncidentTetIds[incidentTetId];

                    // Check exterior faces of incident tet
                    for (uint tetFaceId = 0; tetFaceId < 4; tetFaceId++)
                    {
                        if (FmIsExteriorFaceId(incidentTetFaceIncidentTets.ids[tetFaceId]))
                        {
                            uint incidentTetExteriorFaceId = FmGetExteriorFaceId(incidentTetFaceIncidentTets.ids[tetFaceId]);
                            FM_ASSERT(incidentTetExteriorFaceId < tetMesh->numExteriorFaces + tetMesh->numNewExteriorFaces);

                            // don't check the same exterior face
                            if (exteriorFaceId == incidentTetExteriorFaceId)
                                continue;

                            FmExteriorFace& incidentTetFace = tetMesh->exteriorFaces[incidentTetExteriorFaceId];
                            FmFaceVertIds incidentTetFaceVerts;
                            FmGetFaceVertIds(&incidentTetFaceVerts, tetFaceId, incidentTetVerts);

                            int vertInTri = -1;
                            vertInTri = incidentTetFaceVerts.ids[0] == vId ? 0 : vertInTri;
                            vertInTri = incidentTetFaceVerts.ids[1] == vId ? 1 : vertInTri;
                            vertInTri = incidentTetFaceVerts.ids[2] == vId ? 2 : vertInTri;

                            if (vertInTri >= 0 && incidentTetFace.OwnsVert((uint)vertInTri))
                            {
                                vertIsAssigned[faceCornerId] = true;
                                break;
                            }
                        }
                    }
                }
            }

            FM_ASSERT(
                exteriorFace.edgeIncidentFaceIds[0] != FM_INVALID_ID &&
                exteriorFace.edgeIncidentFaceIds[1] != FM_INVALID_ID &&
                exteriorFace.edgeIncidentFaceIds[2] != FM_INVALID_ID);

            if (!vertIsAssigned[0]) exteriorFace.AssignVert(0);
            if (!vertIsAssigned[1]) exteriorFace.AssignVert(1);
            if (!vertIsAssigned[2]) exteriorFace.AssignVert(2);
            if (!edgeIsAssigned[0]) exteriorFace.AssignEdge(0);
            if (!edgeIsAssigned[1]) exteriorFace.AssignEdge(1);
            if (!edgeIsAssigned[2]) exteriorFace.AssignEdge(2);

            tetMesh->vertsFlags[faceVerts.ids[0]] |= FM_VERT_FLAG_ASSIGNMENT_UPDATED;
            tetMesh->vertsFlags[faceVerts.ids[1]] |= FM_VERT_FLAG_ASSIGNMENT_UPDATED;
            tetMesh->vertsFlags[faceVerts.ids[2]] |= FM_VERT_FLAG_ASSIGNMENT_UPDATED;
        }

        for (uint vId = 0; vId < tetMesh->numVerts; vId++)
        {
            tetMesh->vertsFlags[vId] &= ~FM_VERT_FLAG_ASSIGNMENT_UPDATED;
        }
    }

    bool FmHasMatchingFace(uint* matchingFaceA, uint* matchingFaceB, const FmTetVertIds& tetAVerts, const FmTetVertIds& tetBVerts)
    {
        FmFaceVertIds tetAFaceVerts[4], tetBFaceVerts[4];
        FmGetFaceVertIds(&tetAFaceVerts[0], 0, tetAVerts);
        FmGetFaceVertIds(&tetAFaceVerts[1], 1, tetAVerts);
        FmGetFaceVertIds(&tetAFaceVerts[2], 2, tetAVerts);
        FmGetFaceVertIds(&tetAFaceVerts[3], 3, tetAVerts);
        FmGetFaceVertIds(&tetBFaceVerts[0], 0, tetBVerts);
        FmGetFaceVertIds(&tetBFaceVerts[1], 1, tetBVerts);
        FmGetFaceVertIds(&tetBFaceVerts[2], 2, tetBVerts);
        FmGetFaceVertIds(&tetBFaceVerts[3], 3, tetBVerts);

        *matchingFaceA = FM_INVALID_ID;
        *matchingFaceB = FM_INVALID_ID;
        for (uint i = 0; i < 4; i++)
        {
            for (uint j = 0; j < 4; j++)
            {
                if (FmFacesAreMatching(tetAFaceVerts[i], tetBFaceVerts[j]))
                {
                    *matchingFaceA = i;
                    *matchingFaceB = j;
                    return true;
                }
            }
        }
        return false;
    }

    void FmFindTetIncidentTetsFromVerts(FmTetMesh* tetMesh)
    {
        uint* incidentTetsArray = tetMesh->vertConnectivity.incidentTets;

        // Link face incident tets by searching list of tets incident on each vertex.
        for (uint tetId = 0; tetId < tetMesh->numTets; tetId++)
        {
            FmTetVertIds tetVerts = tetMesh->tetsVertIds[tetId];
            FmTetFaceIncidentTetIds& tetFaceIncidentTets = tetMesh->tetsFaceIncidentTetIds[tetId];
            tetFaceIncidentTets.ids[0] = FmMakeExteriorFaceId(0);
            tetFaceIncidentTets.ids[1] = FmMakeExteriorFaceId(0);
            tetFaceIncidentTets.ids[2] = FmMakeExteriorFaceId(0);
            tetFaceIncidentTets.ids[3] = FmMakeExteriorFaceId(0);

            for (uint cornerId = 0; cornerId < 4; cornerId++)
            {
                uint vertId = tetVerts.ids[cornerId];

                FmVertNeighbors& vertNeighbors = tetMesh->vertsNeighbors[vertId];

                uint numIncidentTets = vertNeighbors.numIncidentTets;
                uint incidentTetsStart = vertNeighbors.incidentTetsStart;

                for (uint vtIdx = 0; vtIdx < numIncidentTets; vtIdx++)
                {
                    uint otherTetId = incidentTetsArray[incidentTetsStart + vtIdx];
                    FmTetVertIds otherTetVerts = tetMesh->tetsVertIds[otherTetId];

                    if (otherTetId == tetId)
                        continue;

                    uint faceA, faceB;
                    if (FmHasMatchingFace(&faceA, &faceB, tetVerts, otherTetVerts))
                    {
                        tetFaceIncidentTets.ids[faceA] = otherTetId;
                    }
                }
            }

            // Create exterior faces for collision detection, and keep indices in tetFaceIncidentTets
            for (uint faceId = 0; faceId < 4; faceId++)
            {
                if (FmIsExteriorFaceId(tetFaceIncidentTets.ids[faceId]))
                {
                    FmFaceVertIds faceVerts;
                    FmGetFaceVertIds(&faceVerts, faceId, tetVerts);

                    uint exteriorFaceId = tetMesh->numExteriorFaces;

                    FmExteriorFace& exteriorFace = tetMesh->exteriorFaces[exteriorFaceId];
                    exteriorFace = FmExteriorFace();
                    exteriorFace.SetId(exteriorFaceId);
                    exteriorFace.tetId = tetId;
                    exteriorFace.faceId = faceId;

                    tetFaceIncidentTets.ids[faceId] = FmMakeExteriorFaceId(exteriorFaceId);

                    tetMesh->numExteriorFaces++;
                }
            }
        }

        FmAssignFeaturesToExteriorFaces(tetMesh, 0, tetMesh->numExteriorFaces);
    }

    uint FmUpdateAdjacentVertOffsets(FmTetMesh* tetMesh, uint inputVertId, FmVertNeighbors& vertNeighbors, uint* incidentTetsArray)
    {
        uint incidentTetsStart = vertNeighbors.incidentTetsStart;
        uint numIncidentTets = vertNeighbors.numIncidentTets;

        FM_ASSERT(numIncidentTets > 0);

#if FM_SORT_MATRIX_ROW_VERTS
        // Assign offsets in stiffness matrix by sorted order of vert indices, 
        // except for initial offset which is reserved for diagonal entry.
        uint adjacentVerts[FM_MAX_VERT_INCIDENT_TETS * 4];
        uint numAdjacentVerts = 0;

        // Sort adjacent vert ids
        for (uint itIdx = incidentTetsStart; itIdx < incidentTetsStart + numIncidentTets; itIdx++)
        {
            uint tetId = incidentTetsArray[itIdx];
            FmTetVertIds tetVertIds = tetMesh->tetsVertIds[tetId];

            for (uint tvId = 0; tvId < 4; tvId++)
            {
                uint tetVertId = tetVertIds.ids[tvId];

                if (tetVertId == inputVertId)
                {
                    continue;
                }

                // Insert adjacent vert id in sorted order
                for (uint sId = 0; sId < numAdjacentVerts + 1; sId++)
                {
                    if (sId == numAdjacentVerts)
                    {
                        // Not found, insert at end
                        adjacentVerts[sId] = tetVertId;
                        numAdjacentVerts++;
                        break;
                    }
                    else
                    {
                        if (adjacentVerts[sId] == tetVertId)
                        {
                            // Already added
                            break;
                        }

                        if (adjacentVerts[sId] > tetVertId)
                        {
                            // Insert id and shift rest forward by one
                            for (uint dsId = numAdjacentVerts; dsId > sId; dsId--)
                            {
                                adjacentVerts[dsId] = adjacentVerts[dsId - 1];
                            }
                            adjacentVerts[sId] = tetVertId;
                            numAdjacentVerts++;
                            break;
                        }
                    }
                }
            }
        }

        // Assign offsets according to sorted order
        for (uint itIdx = incidentTetsStart; itIdx < incidentTetsStart + numIncidentTets; itIdx++)
        {
            uint tetId = incidentTetsArray[itIdx];
            FmTetVertIds tetVertIds = tetMesh->tetsVertIds[tetId];
            FmTetStiffnessState& tetStiffnessState = tetMesh->tetsStiffness[tetId];

            uint tetRowIdx = 0;
            uint stiffnessRowOffsets[4] = { 0 };

            for (uint tvId = 0; tvId < 4; tvId++)
            {
                uint tetVertId = tetVertIds.ids[tvId];

                if (tetVertId == inputVertId)
                {
                    stiffnessRowOffsets[tvId] = FM_INVALID_ID;
                    tetRowIdx = tvId;
                }
                else
                {
                    for (uint sId = 0; sId < numAdjacentVerts; sId++)
                    {
                        if (adjacentVerts[sId] == tetVertId)
                        {
                            stiffnessRowOffsets[tvId] = sId;
                            break;
                        }
                    }
                }
            }

            tetStiffnessState.stiffnessMatRowOffsets[tetRowIdx][0] = stiffnessRowOffsets[0];
            tetStiffnessState.stiffnessMatRowOffsets[tetRowIdx][1] = stiffnessRowOffsets[1];
            tetStiffnessState.stiffnessMatRowOffsets[tetRowIdx][2] = stiffnessRowOffsets[2];
            tetStiffnessState.stiffnessMatRowOffsets[tetRowIdx][3] = stiffnessRowOffsets[3];
        }
#else
        // Assign offsets in stiffness matrix according to order of vertices found in incident tets array.
        uint adjacentVertIds[FM_MAX_VERT_INCIDENT_TETS * 4];
        uint numAdjacentVerts = 0;
       
        for (uint tOffset = 0; tOffset < numIncidentTets; tOffset++)
        {
            uint itIdx = incidentTetsStart + tOffset;
            uint tetId = incidentTetsArray[itIdx];
            FmTetVertIds tetVertIds = tetMesh->tetsVertIds[tetId];

            uint tetVertexOffsets[4];
            uint tetRowIdx = 0;
            uint numNewAdjacentVerts = 0;
            for (uint i = 0; i < 4; i++)
            {
                tetVertexOffsets[i] = FM_INVALID_ID;

                uint adjVId = tetVertIds.ids[i];

                if (adjVId == inputVertId)
                {
                    tetRowIdx = i;
                }

                if (adjVId != inputVertId)
                {
                    // Check if vertex has already been seen
                    uint vertOffset = FM_INVALID_ID;
                    for (uint adjacentIdx = 0; adjacentIdx < numAdjacentVerts; adjacentIdx++)
                    {
                        if (adjacentVertIds[adjacentIdx] == adjVId)
                        {
                            vertOffset = adjacentIdx;
                            break;
                        }
                    }

                    if (vertOffset == FM_INVALID_ID)
                    {
                        // Not found, add new adjacent vert
                        adjacentVertIds[numAdjacentVerts + numNewAdjacentVerts] = adjVId;
                        tetVertexOffsets[i] = numAdjacentVerts + numNewAdjacentVerts;
                        numNewAdjacentVerts++;
                    }
                    else
                    {
                        tetVertexOffsets[i] = vertOffset;
                    }
                }
            }

            numAdjacentVerts += numNewAdjacentVerts;

            FmTetStiffnessState& tetStiffnessState = tetMesh->tetsStiffness[tetId];
            tetStiffnessState.stiffnessMatRowOffsets[tetRowIdx][0] = tetVertexOffsets[0];
            tetStiffnessState.stiffnessMatRowOffsets[tetRowIdx][1] = tetVertexOffsets[1];
            tetStiffnessState.stiffnessMatRowOffsets[tetRowIdx][2] = tetVertexOffsets[2];
            tetStiffnessState.stiffnessMatRowOffsets[tetRowIdx][3] = tetVertexOffsets[3];
        }
#endif

        vertNeighbors.numAdjacentVerts = numAdjacentVerts;

        return numAdjacentVerts;
    }

    // Compute total number of adjacent vertices, and offset to each vert's neighbors array
    void FmUpdateAdjacentVertOffsets(FmTetMesh* tetMesh)
    {
        uint numVerts = tetMesh->numVerts;
        uint* incidentTetsArray = tetMesh->vertConnectivity.incidentTets;
        uint totalVertAdjacentVerts = 0;
        for (uint vId = 0; vId < numVerts; vId++)
        {
            FmVertNeighbors& vertNeighbors = tetMesh->vertsNeighbors[vId];
            totalVertAdjacentVerts += FmUpdateAdjacentVertOffsets(tetMesh, vId, vertNeighbors, incidentTetsArray);
        }
        tetMesh->vertConnectivity.numAdjacentVerts = totalVertAdjacentVerts;
    }

    uint FmGetVertNumAdjacentVerts(const uint* vertexIncidentTets, uint numIncidentTets, const FmTetVertIds* tetVertIds)
    {
        if (numIncidentTets == 0)
        {
            return 0;
        }

        uint vertexNumAdjacentVerts = 3;

        for (uint itAIdx = 1; itAIdx < numIncidentTets; itAIdx++)
        {
            uint tetAId = vertexIncidentTets[itAIdx];
            FmTetVertIds tetAVertIds = tetVertIds[tetAId];

            bool tetAVertMatched[4] = { false, false, false, false };

            // check previous incident tets for the same verts, and use existing adjacent vert indices
            for (uint itBIdx = 0; itBIdx < itAIdx; itBIdx++)
            {
                uint tetBId = vertexIncidentTets[itBIdx];
                FmTetVertIds tetBVertIds = tetVertIds[tetBId];

                for (uint cornerIdx = 0; cornerIdx < 4; cornerIdx++)
                {
                    tetAVertMatched[cornerIdx] =
                        tetAVertMatched[cornerIdx]
                        || (tetAVertIds.ids[cornerIdx] == tetBVertIds.ids[0])
                        || (tetAVertIds.ids[cornerIdx] == tetBVertIds.ids[1])
                        || (tetAVertIds.ids[cornerIdx] == tetBVertIds.ids[2])
                        || (tetAVertIds.ids[cornerIdx] == tetBVertIds.ids[3]);
                }
            }

            // If there was no match to verts in previous tets, use the next index and increment count of adjacent verts
            for (uint cornerIdx = 0; cornerIdx < 4; cornerIdx++)
            {
                if (!tetAVertMatched[cornerIdx])
                {
                    vertexNumAdjacentVerts++;
                }
            }
        }

        return vertexNumAdjacentVerts;
    }

    uint FmFindTetFaceIncidentTets(
        FmTetFaceIncidentTetIds* tetFaceIncidentTetIds,
        FmArray<uint>* vertIncidentTets,
        const FmTetVertIds* tetVertIds,
        uint numTets)
    {
        uint numExterior = 0;

        // Link face incident tets by searching list of tets incident on each vertex.
        for (uint tetId = 0; tetId < numTets; tetId++)
        {
            FmTetVertIds tetVerts = tetVertIds[tetId];
            FmTetFaceIncidentTetIds& tetFaceIncidentTets = tetFaceIncidentTetIds[tetId];
            tetFaceIncidentTets.ids[0] = FmMakeExteriorFaceId(0);
            tetFaceIncidentTets.ids[1] = FmMakeExteriorFaceId(0);
            tetFaceIncidentTets.ids[2] = FmMakeExteriorFaceId(0);
            tetFaceIncidentTets.ids[3] = FmMakeExteriorFaceId(0);
            numExterior += 4;

            for (uint cornerId = 0; cornerId < 4; cornerId++)
            {
                uint vertId = tetVerts.ids[cornerId];

                uint numIncidentTets = vertIncidentTets[vertId].GetNumElems();

                for (uint vtIdx = 0; vtIdx < numIncidentTets; vtIdx++)
                {
                    uint otherTetId = vertIncidentTets[vertId][vtIdx];

                    FmTetVertIds otherTetVerts = tetVertIds[otherTetId];

                    if (otherTetId == tetId)
                        continue;

                    uint faceA, faceB;
                    if (FmHasMatchingFace(&faceA, &faceB, tetVerts, otherTetVerts))
                    {
                        if (FmIsExteriorFaceId(tetFaceIncidentTets.ids[faceA]))
                        {
                            numExterior--;
                        }
                        tetFaceIncidentTets.ids[faceA] = otherTetId;
                    }
                }
            }
        }

        return numExterior;
    }

    bool FmAddVertIncidentTets(FmTetMesh* tetMesh, uint* incidentTetsIndex, uint vId, const uint* tetsArray, uint numTets)
    {
        if (numTets > FM_MAX_VERT_INCIDENT_TETS)
        {
            return false;
        }

        uint* incidentTetsArray = tetMesh->vertConnectivity.incidentTets;

        uint tetsIndex = *incidentTetsIndex;

        FmVertNeighbors& vertNeighbors = tetMesh->vertsNeighbors[vId];
        vertNeighbors.numAdjacentVerts = 0;
        vertNeighbors.incidentTetsStart = tetsIndex;
        vertNeighbors.numIncidentTets = numTets;

        for (uint itId = 0; itId < numTets; itId++)
        {
            incidentTetsArray[tetsIndex] = tetsArray[itId];
            tetsIndex++;
        }

        *incidentTetsIndex = tetsIndex;

        return true;
    }

    void FmFinishConnectivityFromVertIncidentTets(FmTetMesh* tetMesh)
    {
        // Initialize extra FmVertNeighbors available for fracture
        uint numVerts = tetMesh->numVerts;
        uint maxVerts = tetMesh->maxVerts;
        for (uint vId = numVerts; vId < maxVerts; vId++)
        {
            FmVertNeighbors& vertNeighbors = tetMesh->vertsNeighbors[vId];
            vertNeighbors.numAdjacentVerts = 0;
            vertNeighbors.incidentTetsStart = 0;
            vertNeighbors.numIncidentTets = 0;
        }

        FmUpdateAdjacentVertOffsets(tetMesh);

        FmFindTetIncidentTetsFromVerts(tetMesh);
    }

    // Use lists of tets incident to verts to initialize tet mesh connectivity.
    // Returns whether completed; fails if any vertex has more than FM_MAX_VERT_INCIDENT_TETS
    bool FmInitConnectivity(FmTetMesh* tetMesh, const FmArray<uint>* vertIncidentTets)
    {
        uint numVerts = tetMesh->numVerts;
        uint incidentTetsIndex = 0;

        for (uint vId = 0; vId < numVerts; vId++)
        {
            uint numIncidentTets = vertIncidentTets[vId].GetNumElems();
            const uint* incidentTets = &vertIncidentTets[vId][0];

            if (numIncidentTets > FM_MAX_VERT_INCIDENT_TETS)
            {
                FM_PRINT(("Number of incident tets is greater than library limit of %u\n", FM_MAX_VERT_INCIDENT_TETS));
                return false;
            }
            FmAddVertIncidentTets(tetMesh, &incidentTetsIndex, vId, incidentTets, numIncidentTets);
        }

        FmFinishConnectivityFromVertIncidentTets(tetMesh);

        return true;
    }

    void FmSetMassesFromRestDensities(FmTetMesh* tetMesh, float restDensity)
    {
        for (uint vertId = 0; vertId < tetMesh->numVerts; vertId++)
        {
            tetMesh->vertsMass[vertId] = 0.0f;
        }

        for (uint tetId = 0; tetId < tetMesh->numTets; tetId++)
        {
            const FmTetShapeParams& shapeParams = tetMesh->tetsShapeParams[tetId];

            if (restDensity != 0.0f)
            {
                tetMesh->tetsRestDensity[tetId] = restDensity;
            }

            float mass = tetMesh->tetsRestDensity[tetId] * shapeParams.GetVolume();
            tetMesh->tetsMass[tetId] = mass;

            FmTetVertIds tetVerts = tetMesh->tetsVertIds[tetId];

            float vertMass = 0.25f * mass;
            tetMesh->vertsMass[tetVerts.ids[0]] += vertMass;
            tetMesh->vertsMass[tetVerts.ids[1]] += vertMass;
            tetMesh->vertsMass[tetVerts.ids[2]] += vertMass;
            tetMesh->vertsMass[tetVerts.ids[3]] += vertMass;
        }
    }

    void FmDistributeVertMassesToTets(FmTetMesh* tetMesh)
    {
        for (uint tetId = 0; tetId < tetMesh->numTets; tetId++)
        {
            tetMesh->tetsMass[tetId] = 0.0f;
        }

        uint numVerts = tetMesh->numVerts;
        uint* incidentTetsArray = tetMesh->vertConnectivity.incidentTets;
        for (uint vId = 0; vId < numVerts; vId++)
        {
            FmVertNeighbors& vertNeighbors = tetMesh->vertsNeighbors[vId];

            uint numIncidentTets = vertNeighbors.numIncidentTets;

            float massContribution = tetMesh->vertsMass[vId] / (float)numIncidentTets;

            for (uint itIdx = 0; itIdx < numIncidentTets; itIdx++)
            {
                uint incidentTetId = incidentTetsArray[vertNeighbors.incidentTetsStart + itIdx];

                tetMesh->tetsMass[incidentTetId] += massContribution;
            }
        }

        for (uint vertId = 0; vertId < tetMesh->numVerts; vertId++)
        {
            tetMesh->vertsMass[vertId] = 0.0f;
        }

        for (uint tetId = 0; tetId < tetMesh->numTets; tetId++)
        {
            float mass = tetMesh->tetsMass[tetId];

            FmTetVertIds tetVerts = tetMesh->tetsVertIds[tetId];

            float vertMass = 0.25f * mass;
            tetMesh->vertsMass[tetVerts.ids[0]] += vertMass;
            tetMesh->vertsMass[tetVerts.ids[1]] += vertMass;
            tetMesh->vertsMass[tetVerts.ids[2]] += vertMass;
            tetMesh->vertsMass[tetVerts.ids[3]] += vertMass;
        }
    }

    void FmSetTotalMass(FmTetMesh* tetMesh, float mass)
    {
        uint numVerts = tetMesh->numVerts;
        float currentMass = 0.0f;

        if (numVerts == 0)
        {
            return;
        }

        // Check current total mass
        for (uint vId = 0; vId < numVerts; vId++)
        {
            float vertMass = tetMesh->vertsMass[vId];
            currentMass += vertMass;
        }

        // Scale mass at all vertices for desired total
        float massScale = mass / currentMass;

        currentMass = 0.0f;
        for (uint vId = 0; vId < numVerts; vId++)
        {
            float newVertMass = tetMesh->vertsMass[vId] * massScale;
            tetMesh->vertsMass[vId] = newVertMass;
            currentMass += newVertMass;
        }

        tetMesh->mass = currentMass;
    }

    void FmEnableSurfaceCollisionCallback(FmTetMesh* tetMesh, bool isEnabled)
    {
        if (isEnabled)
        {
            tetMesh->flags &= ~FM_OBJECT_FLAG_SURFACE_COLLISION_CALLBACK_DISABLED;
        }
        else
        {
            tetMesh->flags |= FM_OBJECT_FLAG_SURFACE_COLLISION_CALLBACK_DISABLED;
        }
    }

    FmVector3 FmMoveRestPositionCenterOfMassToOrigin(FmTetMesh* tetMesh)
    {
        uint numVerts = tetMesh->numVerts;
        FmVector3 centerOfMass = FmInitVector3(0.0f);
        float totalMass = 0.0f;

        for (uint vId = 0; vId < numVerts; vId++)
        {
            FmVector3 position = tetMesh->vertsRestPos[vId];
            float vertMass = tetMesh->vertsMass[vId];

            totalMass += vertMass;
            centerOfMass += position * vertMass;
        }

        centerOfMass = centerOfMass / totalMass;

        for (uint vId = 0; vId < numVerts; vId++)
        {
            tetMesh->vertsRestPos[vId] -= centerOfMass;
        }

        return centerOfMass;
    }

    int FmCheckTetMeshAspectRatios(FmTetMesh* tetMesh)
    {
        uint numTets = tetMesh->numTets;
        for (uint tetId = 0; tetId < numTets; tetId++)
        {
            FmTetVertIds tetVertIds = tetMesh->tetsVertIds[tetId];

            FmVector3 tetRestPositions[4];
            tetRestPositions[0] = tetMesh->vertsRestPos[tetVertIds.ids[0]];
            tetRestPositions[1] = tetMesh->vertsRestPos[tetVertIds.ids[1]];
            tetRestPositions[2] = tetMesh->vertsRestPos[tetVertIds.ids[2]];
            tetRestPositions[3] = tetMesh->vertsRestPos[tetVertIds.ids[3]];

            if (FmIsTetInverted(tetRestPositions))
            {
                return -2;
            }

            float aspectRatio = FmComputeTetAspectRatio(tetRestPositions);

            if (aspectRatio > FM_MAX_TET_ASPECT_RATIO)
            {
                return -1;
            }
        }

        return 0;
    }

    int FmFinishTetMeshInit(FmTetMesh* tetMesh)
    {
        FmUpdateBoundsAndCenterOfMass(tetMesh);
        FmUpdateTetStateAndFracture(NULL, tetMesh, false, false);

        return FmCheckTetMeshAspectRatios(tetMesh);
    }

    // Return the current world position inside a tet that corresponds to the input barycentric values
    FmVector3 FmGetInterpolatedPosition(const float bary[4], const FmTetMesh& tetMesh, uint tetId)
    {
        FmTetVertIds tetVertIds = tetMesh.tetsVertIds[tetId];
        FmVector3 position0 = tetMesh.vertsPos[tetVertIds.ids[0]];
        FmVector3 position1 = tetMesh.vertsPos[tetVertIds.ids[1]];
        FmVector3 position2 = tetMesh.vertsPos[tetVertIds.ids[2]];
        FmVector3 position3 = tetMesh.vertsPos[tetVertIds.ids[3]];
        return FmInterpolate(bary, position0, position1, position2, position3);
    }

    // Return the current world velocity inside a tet that corresponds to the input barycentric values
    FmVector3 FmGetInterpolatedVelocity(const float bary[4], const FmTetMesh& tetMesh, uint tetId)
    {
        FmTetVertIds tetVertIds = tetMesh.tetsVertIds[tetId];
        FmVector3 velocity0 = tetMesh.vertsVel[tetVertIds.ids[0]];
        FmVector3 velocity1 = tetMesh.vertsVel[tetVertIds.ids[1]];
        FmVector3 velocity2 = tetMesh.vertsVel[tetVertIds.ids[2]];
        FmVector3 velocity3 = tetMesh.vertsVel[tetVertIds.ids[3]];
        return FmInterpolate(bary, velocity0, velocity1, velocity2, velocity3);
    }

    FmTetVertIds FmGetTetVertIds(const FmTetMesh& tetMesh, uint tetId)
    {
        return tetMesh.tetsVertIds[tetId];
    }

    FmTetFaceIncidentTetIds FmGetTetFaceIncidentTetIds(const FmTetMesh& tetMesh, uint tetId)
    {
        return tetMesh.tetsFaceIncidentTetIds[tetId];
    }

    // Get the current world positions of the four corners a tet
    void FmGetTetPositions(FmVector3 positions[4], const FmTetMesh& tetMesh, uint tetId)
    {
        FmTetVertIds tetVertIds = tetMesh.tetsVertIds[tetId];
        positions[0] = tetMesh.vertsPos[tetVertIds.ids[0]];
        positions[1] = tetMesh.vertsPos[tetVertIds.ids[1]];
        positions[2] = tetMesh.vertsPos[tetVertIds.ids[2]];
        positions[3] = tetMesh.vertsPos[tetVertIds.ids[3]];
    }

    // Get the current world positions of the three corners a tet face
    void FmGetTetFacePositions(FmVector3 positions[3], const FmTetMesh& tetMesh, uint tetId, uint faceId)
    {
        FmTetVertIds tetVertIds = tetMesh.tetsVertIds[tetId];
        FmFaceVertIds vertIds;
        FmGetFaceVertIds(&vertIds, faceId, tetVertIds);
        positions[0] = tetMesh.vertsPos[vertIds.ids[0]];
        positions[1] = tetMesh.vertsPos[vertIds.ids[1]];
        positions[2] = tetMesh.vertsPos[vertIds.ids[2]];
    }

    // Get average of the current world positions of the corners
    FmVector3 FmGetTetCenter(const FmTetMesh& tetMesh, uint tetId)
    {
        FmTetVertIds tetVertIds = tetMesh.tetsVertIds[tetId];
        FmVector3 position0 = tetMesh.vertsPos[tetVertIds.ids[0]];
        FmVector3 position1 = tetMesh.vertsPos[tetVertIds.ids[1]];
        FmVector3 position2 = tetMesh.vertsPos[tetVertIds.ids[2]];
        FmVector3 position3 = tetMesh.vertsPos[tetVertIds.ids[3]];
        return (position0 + position1 + position2 + position3) * 0.25f;
    }

    // Get the rotation of a tet
    FmMatrix3 FmGetTetRotation(const FmTetMesh& tetMesh, uint tetId)
    {
        return tetMesh.tetsRotation[tetId];
    }

    // Get the matrix derived from the tet rest positions that transforms a position in homogeneous coordinates (x,y,z,1) into barycentric coordinates.
    FmMatrix4 FmGetTetRestBaryMatrix(const FmTetMesh& tetMesh, uint tetId)
    {
        return tetMesh.tetsShapeParams[tetId].baryMatrix;
    }

    FmTetMaterialParams FmGetTetMaterialParams(const FmTetMesh& tetMesh, uint tetId)
    {
        FmTetMaterialParams result;
        FmInitTetMaterialParams(&result, tetMesh, tetId);
        return result;
    }

    uint FmGetTetMeshBufferId(const FmTetMesh& tetMesh)
    {
        return tetMesh.bufferId;
    }

    uint FmGetTetMeshBufferId(const FmTetMeshBuffer& tetMeshBuffer)
    {
        return tetMeshBuffer.bufferId;
    }

    // Get number of tetrahedra contained in tet mesh buffer.
    uint FmGetNumTets(const FmTetMeshBuffer& tetMeshBuffer)
    {
        return tetMeshBuffer.numTets;
    }

    void FmGetTetMeshBufferTetInfo(uint* fractureGroupId, uint16_t* tetFlags, const FmTetMeshBuffer& tetMeshBuffer, uint bufferTetId)
    {
        *fractureGroupId = FM_INVALID_ID;
        *tetFlags = 0;
        if (bufferTetId < tetMeshBuffer.numTets)
        {
            FmTetReference& tetReference = tetMeshBuffer.tetReferences[bufferTetId];
            *fractureGroupId = tetReference.fractureGroupId;
            *tetFlags = tetMeshBuffer.tetMeshes[tetReference.meshIdx].tetsFlags[tetReference.tetId];
        }
    }

    // Get number of tet meshes contained in FmTetMeshBuffer
    uint FmGetNumTetMeshes(const FmTetMeshBuffer& tetMeshBuffer)
    {
        return tetMeshBuffer.numTetMeshes;
    }

    // Get maximum number of tet meshes contained in tet mesh buffer after all fracture
    uint FmGetMaxTetMeshes(const FmTetMeshBuffer& tetMeshBuffer)
    {
        return tetMeshBuffer.maxTetMeshes;
    }

    // Get a FmTetMesh* by index with a FmTetMeshBuffer; returns NULL if not found.
    FmTetMesh* FmGetTetMesh(const FmTetMeshBuffer& tetMeshBuffer, uint meshIdx)
    {
        if (meshIdx >= tetMeshBuffer.numTetMeshes)
        {
            return NULL;
        }

        return &tetMeshBuffer.tetMeshes[meshIdx];
    }

    FmTetMesh* FmGetTetMeshContainingVert(uint* meshVertId, uint* bufferVertMeshIdx, const FmTetMeshBuffer& tetMeshBuffer, uint bufferVertId)
    {
        if (bufferVertId >= tetMeshBuffer.numVerts)
        {
            return NULL;
        }

        FmVertReference& vertRef = tetMeshBuffer.vertReferences[bufferVertId];

        uint meshIdx = vertRef.meshIdx;
        FmTetMesh* tetMesh = &tetMeshBuffer.tetMeshes[meshIdx];
        uint vertId = vertRef.vertId;

        FM_ASSERT(tetMesh->vertsIndex0[vertId] == bufferVertId);

        *bufferVertMeshIdx = meshIdx;
        *meshVertId = vertId;

        return tetMesh;
    }

    FmTetMesh* FmGetTetMeshContainingTet(uint* meshTetId, uint* bufferTetMeshIdx, const FmTetMeshBuffer& tetMeshBuffer, uint bufferTetId)
    {
        if (bufferTetId >= tetMeshBuffer.numTets)
        {
            return NULL;
        }

        FmTetReference& tetRef = tetMeshBuffer.tetReferences[bufferTetId];

        uint meshIdx = tetRef.meshIdx;
        FmTetMesh* tetMesh = &tetMeshBuffer.tetMeshes[meshIdx];
        uint tetId = tetRef.tetId;

        FM_ASSERT(tetMesh->tetsIndex0[tetId] == bufferTetId);

        *bufferTetMeshIdx = meshIdx;
        *meshTetId = tetId;

        return tetMesh;
    }

    void FmSetVertMass(FmTetMesh* tetMesh, uint vertId, float mass)
    {
        if (vertId >= tetMesh->numVerts)
        {
            return;
        }

        tetMesh->vertsMass[vertId] = mass;
    }

    uint16_t FmGetVertFlags(const FmTetMesh& tetMesh, uint vertId)
    {
        if (vertId >= tetMesh.numVerts)
        {
            return 0;
        }

        return tetMesh.vertsFlags[vertId];
    }

    void FmSetVertFlags(FmTetMesh* tetMesh, uint vertId, uint16_t flags)
    {
        if (vertId >= tetMesh->numVerts)
        {
            return;
        }

        tetMesh->vertsFlags[vertId] = flags;
    }

    void FmAddVertFlags(FmTetMesh* tetMesh, uint vertId, uint16_t flags)
    {
        if (vertId >= tetMesh->numVerts)
        {
            return;
        }

        tetMesh->vertsFlags[vertId] |= flags;
    }

    void FmRemoveVertFlags(FmTetMesh* tetMesh, uint vertId, uint16_t flags)
    {
        if (vertId >= tetMesh->numVerts)
        {
            return;
        }

        tetMesh->vertsFlags[vertId] &= ~flags;
    }

    void FmSetTetFlags(FmTetMesh* tetMesh, uint tetId, uint16_t flags)
    {
        if (tetId >= tetMesh->numTets)
        {
            return;
        }

        tetMesh->tetsFlags[tetId] = flags;
    }

    void FmAddTetFlags(FmTetMesh* tetMesh, uint tetId, uint16_t flags)
    {
        if (tetId >= tetMesh->numTets)
        {
            return;
        }

        tetMesh->tetsFlags[tetId] |= flags;
    }

    void FmRemoveTetFlags(FmTetMesh* tetMesh, uint tetId, uint16_t flags)
    {
        if (tetId >= tetMesh->numTets)
        {
            return;
        }

        tetMesh->tetsFlags[tetId] &= ~flags;
    }

    void FmAddForceToVert(FmScene* scene, FmTetMesh* tetMesh, uint vertId, const FmVector3& force)
    {
        if (vertId >= tetMesh->numVerts)
        {
            return;
        }

        tetMesh->vertsExtForce[vertId] += force;
        tetMesh->vertsFlags[vertId] |= FM_VERT_FLAG_EXT_FORCE_SET;

        // If part of sleeping island, mark for waking
        FmMarkIslandOfObjectForWaking(scene, tetMesh->objectId);
    }

    FmVector3 FmGetVertRestPosition(const FmTetMesh & tetMesh, uint vertId)
    {
        if (vertId >= tetMesh.numVerts)
        {
            return FmInitVector3(0.0f);
        }

        return tetMesh.vertsRestPos[vertId];
    }

    FmVector3 FmGetVertPosition(const FmTetMesh& tetMesh, uint vertId)
    {
        if (vertId >= tetMesh.numVerts)
        {
            return FmInitVector3(0.0f);
        }

        return tetMesh.vertsPos[vertId];
    }

    FmVector3 FmGetVertVelocity(const FmTetMesh& tetMesh, uint vertId)
    {
        if (vertId >= tetMesh.numVerts)
        {
            return FmInitVector3(0.0f);
        }

        return tetMesh.vertsVel[vertId];
    }

    FmVector3 FmGetVertExternalForce(const FmTetMesh& tetMesh, uint vertId)
    {
        if (vertId >= tetMesh.numVerts)
        {
            return FmInitVector3(0.0f);
        }

        return tetMesh.vertsExtForce[vertId];
    }

    float FmGetVertMass(const FmTetMesh& tetMesh, uint vertId)
    {
        if (vertId >= tetMesh.numVerts)
        {
            return 0.0f;
        }

        return tetMesh.vertsMass[vertId];
    }

    FmVertNeighbors FmGetVertNeighbors(const FmTetMesh& tetMesh, uint vertId)
    {
        if (vertId >= tetMesh.numVerts)
        {
            return FmVertNeighbors();
        }

        return tetMesh.vertsNeighbors[vertId];
    }

    uint FmGetVertIndex0(const FmTetMesh& tetMesh, uint vertId)
    {
        if (vertId >= tetMesh.numVerts)
        {
            return FM_INVALID_ID;
        }

        return tetMesh.vertsIndex0[vertId];
    }

    float FmGetVertTetStrainMagAvg(const FmTetMesh & tetMesh, uint vertId)
    {
        if (vertId >= tetMesh.numVerts)
        {
            return 0.0f;
        }

        return tetMesh.vertsTetValues[vertId].tetStrainMagAvg;
    }

    float FmGetVertTetStrainMagMax(const FmTetMesh & tetMesh, uint vertId)
    {
        if (vertId >= tetMesh.numVerts)
        {
            return 0.0f;
        }

        return tetMesh.vertsTetValues[vertId].tetStrainMagMax;
    }

    FmQuat FmGetVertTetQuatSum(const FmTetMesh & tetMesh, uint vertId)
    {
        if (vertId >= tetMesh.numVerts)
        {
            return FmInitQuat(0.0f, 0.0f, 0.0f, 1.0f);
        }

        return tetMesh.vertsTetValues[vertId].tetQuatSum;
    }

    void FmSetVertPosition(FmScene* scene, FmTetMesh* tetMesh, uint vertId, const FmVector3& position)
    {
        if (vertId >= tetMesh->numVerts)
        {
            return;
        }

        FmVector3& dstPos = tetMesh->vertsPos[vertId];

        if (!FmIsEqual(dstPos, position))
        {
            dstPos = position;
            tetMesh->flags |= FM_OBJECT_FLAG_POS_CHANGED;
        }

        // If part of sleeping island, mark for waking
        FmMarkIslandOfObjectForWaking(scene, tetMesh->objectId);
    }

    void FmSetVertVelocity(FmScene* scene, FmTetMesh* tetMesh, uint vertId, const FmVector3& velocity)
    {
        if (vertId >= tetMesh->numVerts)
        {
            return;
        }

        FmVector3& dstVel = tetMesh->vertsVel[vertId];
        
        if (!FmIsEqual(dstVel, velocity))
        {
            dstVel = velocity;
            tetMesh->flags |= FM_OBJECT_FLAG_VEL_CHANGED;
        }

        // If part of sleeping island, mark for waking
        FmMarkIslandOfObjectForWaking(scene, tetMesh->objectId);
    }

    void FmUpdateTetMaterialParams(FmScene* scene, FmTetMesh* tetMesh, uint meshTetId, const FmTetMaterialParams& newMaterialParams, float plasticDeformationAttenuation, bool volumePreservingPlasticity)
    {
        const FmTetShapeParams& tetShapeParams = tetMesh->tetsShapeParams[meshTetId];
        FmTetVertIds tetVertIds = tetMesh->tetsVertIds[meshTetId];

        FmTetMaterialParams prevTetMaterialParams;
        FmInitTetMaterialParams(&prevTetMaterialParams, *tetMesh, meshTetId);

        // If material is plastic then apply attentuation factor to plastic deformation matrix
        if (tetMesh->tetsPlasticity)
        {
            FmTetPlasticityState& plasticityState = tetMesh->tetsPlasticity[meshTetId];

            if (plasticDeformationAttenuation == 0.0f)
            {
                // Reset plastic deformation
                plasticityState.plasticDeformationMatrix = FmMatrix3::identity();
#if FM_COMPUTE_PLASTIC_REL_ROTATION
                plasticityState.plasticTetRelRotation = FmMatrix3::identity();
#endif
            }
            else if (plasticDeformationAttenuation >= 0.0f && plasticDeformationAttenuation < 1.0f)
            {
                FmMatrix3 plasticDeformationMatrix = plasticityState.plasticDeformationMatrix;

                // Limit total plastic deformation by limiting singular values
                FmVector3 totalPlasticDeformation;
                FmMatrix3 U, V;
                FmSvd3x3(&U, &totalPlasticDeformation, &V, plasticDeformationMatrix);

                totalPlasticDeformation.x = 1.0f + (totalPlasticDeformation.x - 1.0f) * plasticDeformationAttenuation;
                totalPlasticDeformation.y = 1.0f + (totalPlasticDeformation.y - 1.0f) * plasticDeformationAttenuation;
                totalPlasticDeformation.z = 1.0f + (totalPlasticDeformation.z - 1.0f) * plasticDeformationAttenuation;

                if (volumePreservingPlasticity)
                {
                    // Resize total to preserve volume
                    float plasticDet = totalPlasticDeformation.x * totalPlasticDeformation.y * totalPlasticDeformation.z;
                    totalPlasticDeformation = totalPlasticDeformation * powf(plasticDet, -(1.0f / 3.0f));
                }

                plasticDeformationMatrix = mul(mul(V, FmMatrix3::scale(totalPlasticDeformation)), transpose(V));

                plasticityState.plasticDeformationMatrix = plasticDeformationMatrix;
            }
        }

        // Tet stress, strain, stiffness mats must be updated if change to Young's modulus, Poisson's ratio, or plastic deformation
        if (prevTetMaterialParams.youngsModulus != newMaterialParams.youngsModulus
            || prevTetMaterialParams.poissonsRatio != newMaterialParams.poissonsRatio
            || plasticDeformationAttenuation < 1.0f)
        {
            FmTetShapeParams shapeParams;
            FmVector3 tetRestPosition0, tetRestPosition1, tetRestPosition2, tetRestPosition3;
            if (tetMesh->tetsPlasticity)
            {
                FmTetPlasticityState& plasticityState = tetMesh->tetsPlasticity[meshTetId];

                FmVector3 restPos0 = tetMesh->vertsRestPos[tetVertIds.ids[0]];
                FmVector3 restPos1 = tetMesh->vertsRestPos[tetVertIds.ids[1]];
                FmVector3 restPos2 = tetMesh->vertsRestPos[tetVertIds.ids[2]];
                FmVector3 restPos3 = tetMesh->vertsRestPos[tetVertIds.ids[3]];

                FmMatrix3 restVolumeMatrix = FmComputeTetVolumeMatrix(restPos0, restPos1, restPos2, restPos3);

                FmMatrix3 offsets = mul(plasticityState.plasticDeformationMatrix, restVolumeMatrix);
                tetRestPosition3 = FmVector3(0.0f);
                tetRestPosition0 = offsets.col0;
                tetRestPosition1 = offsets.col1;
                tetRestPosition2 = offsets.col2;

                FmComputeShapeParams(&shapeParams, tetRestPosition0, tetRestPosition1, tetRestPosition2, tetRestPosition3);

                // Update the plasticRestBaryMatrix to match new rest positions
                plasticityState.plasticShapeParams = shapeParams;
            }
        }

        // If density changes, then apply a change to masses of the tetrahedral vertices, and total mass.
        // Center of mass calculation out of date until end of next simulation step, but it isn't critical.
        if (prevTetMaterialParams.restDensity != newMaterialParams.restDensity)
        {
            float tetRestVolume = tetShapeParams.GetVolume();
            float tetPrevMass = prevTetMaterialParams.restDensity * tetRestVolume;
            float tetNewMass = newMaterialParams.restDensity * tetRestVolume;
            float tetMassChange = tetNewMass - tetPrevMass;
            float vertMassChange = 0.25f * tetMassChange;

            float vertMass0 = tetMesh->vertsMass[tetVertIds.ids[0]];
            float vertMass1 = tetMesh->vertsMass[tetVertIds.ids[1]];
            float vertMass2 = tetMesh->vertsMass[tetVertIds.ids[2]];
            float vertMass3 = tetMesh->vertsMass[tetVertIds.ids[3]];

            vertMass0 += vertMassChange;
            vertMass1 += vertMassChange;
            vertMass2 += vertMassChange;
            vertMass3 += vertMassChange;

            const float minMass = FLT_EPSILON;
            vertMass0 = FmMaxFloat(vertMass0, minMass);
            vertMass1 = FmMaxFloat(vertMass1, minMass);
            vertMass2 = FmMaxFloat(vertMass2, minMass);
            vertMass3 = FmMaxFloat(vertMass3, minMass);

            tetMesh->vertsMass[tetVertIds.ids[0]] = vertMass0;
            tetMesh->vertsMass[tetVertIds.ids[1]] = vertMass1;
            tetMesh->vertsMass[tetVertIds.ids[2]] = vertMass2;
            tetMesh->vertsMass[tetVertIds.ids[3]] = vertMass3;

            tetMesh->mass += tetMassChange;
        }

        FmInitTetMaterialParams(tetMesh, meshTetId, newMaterialParams);

        // If part of sleeping island, mark for waking
        FmMarkIslandOfObjectForWaking(scene, tetMesh->objectId);
    }

    void FmUpdateTetMaterialParams(FmScene* scene, FmTetMeshBuffer* tetMeshBuffer, uint bufferTetId, const FmTetMaterialParams& newMaterialParams, float plasticDeformationAttenuation, bool volumePreservingPlasticity)
    {
        FmTetReference& tetReference = tetMeshBuffer->tetReferences[bufferTetId];
        FmUpdateTetMaterialParams(scene, &tetMeshBuffer->tetMeshes[tetReference.meshIdx], tetReference.tetId, newMaterialParams, plasticDeformationAttenuation, volumePreservingPlasticity);
    }

    void FmUpdateAllTetMaterialParams(FmScene* scene, FmTetMesh* tetMesh, const FmTetMaterialParams& newMaterialParams, float plasticDeformationAttenuation, bool volumePreservingPlasticity)
    {
        uint numTets = tetMesh->numTets;
        for (uint meshTetId = 0; meshTetId < numTets; meshTetId++)
        {
            FmUpdateTetMaterialParams(scene, tetMesh, meshTetId, newMaterialParams, plasticDeformationAttenuation, volumePreservingPlasticity);
        }
    }

    void FmUpdateAllTetMaterialParams(FmScene* scene, FmTetMeshBuffer* tetMeshBuffer, const FmTetMaterialParams& newMaterialParams, float plasticDeformationAttenuation, bool volumePreservingPlasticity)
    {
        uint numTets = tetMeshBuffer->numTets;
        for (uint bufferTetId = 0; bufferTetId < numTets; bufferTetId++)
        {
            FmUpdateTetMaterialParams(scene, tetMeshBuffer, bufferTetId, newMaterialParams, plasticDeformationAttenuation, volumePreservingPlasticity);
        }
    }

    uint FmGetObjectId(const FmTetMesh& tetMesh)
    {
        return tetMesh.objectId;
    }

    uint FmGetNumVerts(const FmTetMesh& tetMesh)
    {
        return tetMesh.numVerts;
    }

    uint FmGetNumTets(const FmTetMesh& tetMesh)
    {
        return tetMesh.numTets;
    }

    uint FmGetNumExteriorFaces(const FmTetMesh& tetMesh)
    {
        return tetMesh.numExteriorFaces;
    }

    void FmGetExteriorFace(uint* tetId, uint* faceId, const FmTetMesh& tetMesh, uint extFaceIdx)
    {
        const FmExteriorFace& face = tetMesh.exteriorFaces[extFaceIdx];

        *tetId = face.tetId;
        *faceId = face.faceId;

    }

    uint FmGetNumNewExteriorFaces(const FmTetMesh& tetMesh)
    {
        return tetMesh.numNewExteriorFaces;
    }

    const FmExteriorFace* FmGetNewExteriorFaces(const FmTetMesh& tetMesh)
    {
        return tetMesh.exteriorFaces;
    }

    void FmGetNewExteriorFace(uint* tetId, uint* faceId, const FmTetMesh & tetMesh, uint newExtFaceIdx)
    {
        const FmExteriorFace* newFaces = tetMesh.exteriorFaces + tetMesh.numExteriorFaces - tetMesh.numNewExteriorFaces;
        const FmExteriorFace& newFace = newFaces[newExtFaceIdx];

        *tetId = newFace.tetId;
        *faceId = newFace.faceId;
    }

    void FmGetNewExteriorFaceInTetMeshBuffer(uint* bufferTetId, uint* bufferTetFaceId, const FmTetMesh& tetMesh, uint newExtFaceIdx)
    {
        if (newExtFaceIdx >= tetMesh.numNewExteriorFaces)
        {
            *bufferTetId = FM_INVALID_ID;
            *bufferTetFaceId = FM_INVALID_ID;
        }
        else
        {
            uint extFaceIdx = tetMesh.numExteriorFaces - tetMesh.numNewExteriorFaces + newExtFaceIdx;
            const FmExteriorFace& exteriorFace = tetMesh.exteriorFaces[extFaceIdx];
            *bufferTetId = tetMesh.tetsIndex0[exteriorFace.tetId];
            *bufferTetFaceId = exteriorFace.faceId;
        }
    }

    FmVector3 FmGetMinPosition(const FmTetMesh& tetMesh)
    {
        return tetMesh.minPosition;
    }

    FmVector3 FmGetMaxPosition(const FmTetMesh& tetMesh)
    {
        return tetMesh.maxPosition;
    }

    FmVector3 FmGetCenterOfMass(const FmTetMesh& tetMesh)
    {
        return tetMesh.centerOfMass;
    }

    void FmSetExternalForceSpeedLimit(FmTetMesh* tetMesh, float speedLimit)
    {
        tetMesh->extForceSpeedLimit = speedLimit;
    }

    void FmSetRemoveKinematicThreshold(FmTetMesh* tetMesh, float stressThreshold)
    {
        tetMesh->removeKinematicStressThreshold = stressThreshold;
    }

    void FmSetCollisionGroup(FmTetMesh* tetMesh, uint8_t collisionGroup)
    {
        tetMesh->collisionGroup = collisionGroup;
    }

    void FmSetSleepMaxSpeedThreshold(FmTetMesh* tetMesh, float threshold)
    {
        tetMesh->sleepMaxSpeedThreshold = threshold;
    }

    void FmSetSleepAvgSpeedThreshold(FmTetMesh* tetMesh, float threshold)
    {
        tetMesh->sleepAvgSpeedThreshold = threshold;
    }

    void FmSetSleepStableCount(FmTetMesh* tetMesh, uint count)
    {
        tetMesh->sleepStableCount = count;
    }

    void FmSetGravity(FmTetMesh* tetMesh, const FmVector3& gravityVector)
    {
        if (tetMesh == NULL)
        {
            return;
        }

        tetMesh->gravityVector = gravityVector;
    }

    float FmComputeTetMeshVolume(const FmVector3* vertRestPositions, const FmTetVertIds* tetVertIds, uint numTets)
    {
        float totalRestVolume = 0.0f;

        for (uint tetId = 0; tetId < numTets; tetId++)
        {
            FmVector3 tetRestPosition0 = vertRestPositions[tetVertIds[tetId].ids[0]];
            FmVector3 tetRestPosition1 = vertRestPositions[tetVertIds[tetId].ids[1]];
            FmVector3 tetRestPosition2 = vertRestPositions[tetVertIds[tetId].ids[2]];
            FmVector3 tetRestPosition3 = vertRestPositions[tetVertIds[tetId].ids[3]];

            FmTetShapeParams shapeParams;
            FmComputeShapeParams(&shapeParams, tetRestPosition0, tetRestPosition1, tetRestPosition2, tetRestPosition3);

            totalRestVolume += shapeParams.GetVolume();
        }

        return totalRestVolume;
    }

    float FmComputeMeshConstantMatrices(FmTetMesh* tetMesh)
    {
        float totalRestVolume = 0.0f;

        uint numTets = tetMesh->numTets;
        for (uint tetId = 0; tetId < numTets; tetId++)
        {
            FmTetShapeParams& tetShapeParams = tetMesh->tetsShapeParams[tetId];
            FmTetVertIds tetVertIds = tetMesh->tetsVertIds[tetId];

            FmVector3 tetRestPosition0 = tetMesh->vertsRestPos[tetVertIds.ids[0]];
            FmVector3 tetRestPosition1 = tetMesh->vertsRestPos[tetVertIds.ids[1]];
            FmVector3 tetRestPosition2 = tetMesh->vertsRestPos[tetVertIds.ids[2]];
            FmVector3 tetRestPosition3 = tetMesh->vertsRestPos[tetVertIds.ids[3]];

            FmTetShapeParams shapeParams;
            FmComputeShapeParams(&shapeParams, tetRestPosition0, tetRestPosition1, tetRestPosition2, tetRestPosition3);

            // save shape parameters
            tetShapeParams = shapeParams;

            totalRestVolume += shapeParams.GetVolume();

            if (tetMesh->tetsPlasticity)
            {
                FmTetPlasticityState& plasticityState = tetMesh->tetsPlasticity[tetId];
                plasticityState.plasticShapeParams = tetShapeParams;
            }
        }

        return totalRestVolume;
    }

    // Get the average rest position of a tet
    static FM_FORCE_INLINE FmVector3 FmGetRestTetCenter(const FmTetMesh& tetMesh, uint tetId)
    {
        FmTetVertIds tetVertIds = tetMesh.tetsVertIds[tetId];
        FmVector3 position0 = tetMesh.vertsRestPos[tetVertIds.ids[0]];
        FmVector3 position1 = tetMesh.vertsRestPos[tetVertIds.ids[1]];
        FmVector3 position2 = tetMesh.vertsRestPos[tetVertIds.ids[2]];
        FmVector3 position3 = tetMesh.vertsRestPos[tetVertIds.ids[3]];
        return (position0 + position1 + position2 + position3) * 0.25f;
    }

    // Use an average position and velocity to reset a tet mesh
    void FmResetMeshAtCurrentPosition(FmTetMesh* tetMesh, float speedLimit)
    {
        // Use tet near center of mass to define current position and orientation
        uint numTets = tetMesh->numTets;

        if (numTets == 0)
        {
            return;
        }

        FmVector3 centerOfMass = tetMesh->centerOfMass;
        FmVector3 centerTetPos = FmGetTetCenter(*tetMesh, 0);
        uint centerTetId = 0;
        float centerTetDist = length(centerTetPos - centerOfMass);

        for (uint tId = 1; tId < numTets; tId++)
        {
            FmVector3 tetPos = FmGetTetCenter(*tetMesh, tId);
            float tetDist = length(tetPos - centerOfMass);

            if (tetDist < centerTetDist)
            {
                centerTetPos = tetPos;
                centerTetId = tId;
                centerTetDist = tetDist;
            }
        }

        FmVector3 velocity = FmInitVector3(0.0f);
        uint numVerts = tetMesh->numVerts;
        for (uint vId = 0; vId < numVerts; vId++)
        {
            velocity += tetMesh->vertsVel[vId];
        }
        velocity /= (float)numVerts;
        float speed = length(velocity);
        if (speed > speedLimit)
        {
            velocity = velocity * speedLimit / speed;
        }

        FmMatrix3 rotation = FmGetTetRotation(*tetMesh, centerTetId);
        FmVector3 translation = centerTetPos + mul(rotation, -FmGetRestTetCenter(*tetMesh, centerTetId));

        for (uint vId = 0; vId < numVerts; vId++)
        {
            if (!FM_IS_SET(tetMesh->vertsFlags[vId], FM_VERT_FLAG_KINEMATIC))
            {
                tetMesh->vertsPos[vId] = translation + mul(rotation, tetMesh->vertsRestPos[vId]);
                tetMesh->vertsVel[vId] = velocity;
            }
        }
    }

    // Reset mesh in case of high velocities.
    // Failsafe for CG convergence issue seen mainly with small fractured pieces after large deformation/forces.
    // TODO: Better understand issue and use a more graceful recovery.
    void FmFailsafeReset(FmTetMesh* tetMesh, float resetSpeedLimit)
    {
        uint numVerts = tetMesh->numVerts;
        for (uint vId = 0; vId < numVerts; vId++)
        {
            if (length(tetMesh->vertsVel[vId]) > resetSpeedLimit)
            {
                FmResetMeshAtCurrentPosition(tetMesh, resetSpeedLimit * 0.5f);
                break;
            }
        }

    }

    // Step positions assuming velocity is set to end of step velocity.
    // Updates min and max position and center of mass.
    // If position is changed, set FM_OBJECT_FLAG_POS_CHANGED for next frame.
    void FmUpdateVertPositions(FmTetMesh* tetMesh, float timestep)
    {
        uint numVerts = tetMesh->numVerts;
        FmVector3 centerOfMass = FmInitVector3(0.0f);
        float totalMass = 0.0f;

        if (numVerts == 0)
        {
            return;
        }

        FmVector3 minPosition = tetMesh->vertsPos[0];
        FmVector3 maxPosition = tetMesh->vertsPos[0];

        bool posChanged = false;
        for (uint vId = 0; vId < numVerts; vId++)
        {
            FmVector3 endVelocity = tetMesh->vertsVel[vId];
            FmVector3 endPosition = tetMesh->vertsPos[vId] + timestep * endVelocity; // integrate position with end velocity

            if (!FmIsZero(endVelocity))
            {
                posChanged = true;
            }

            tetMesh->vertsPos[vId] = endPosition;

            float vertMass = tetMesh->vertsMass[vId];
            totalMass += vertMass;
            centerOfMass += endPosition * vertMass;
            minPosition = min(minPosition, endPosition);
            maxPosition = max(maxPosition, endPosition);
        }

        centerOfMass = centerOfMass / totalMass;
        tetMesh->centerOfMass = centerOfMass;
        tetMesh->mass = totalMass;
        tetMesh->minPosition = minPosition;
        tetMesh->maxPosition = maxPosition;
        if (posChanged)
        {
            tetMesh->flags |= FM_OBJECT_FLAG_POS_CHANGED;
        }
        else
        {
            tetMesh->flags &= ~FM_OBJECT_FLAG_POS_CHANGED;
        }
    }

    void FmUpdateBoundsAndCenterOfMass(FmTetMesh* tetMesh)
    {
        uint numVerts = tetMesh->numVerts;
        FmVector3 centerOfMass = FmInitVector3(0.0f);
        float totalMass = 0.0f;

        if (numVerts == 0)
        {
            return;
        }

        FmVector3 minPosition = tetMesh->vertsPos[0];
        FmVector3 maxPosition = tetMesh->vertsPos[0];

        for (uint vId = 0; vId < numVerts; vId++)
        {
            FmVector3 position = tetMesh->vertsPos[vId];
            float vertMass = tetMesh->vertsMass[vId];

            minPosition = min(minPosition, position);
            maxPosition = max(maxPosition, position);

            totalMass += vertMass;
            centerOfMass += position * vertMass;
        }

        centerOfMass = centerOfMass / totalMass;

        tetMesh->minPosition = minPosition;
        tetMesh->maxPosition = maxPosition;
        tetMesh->centerOfMass = centerOfMass;
        tetMesh->mass = totalMass;
    }

}