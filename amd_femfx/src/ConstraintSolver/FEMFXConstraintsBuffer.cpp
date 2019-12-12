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
// Initialization of contacts/constraints buffer and temporary collision workspaces
//---------------------------------------------------------------------------------------

#include "FEMFXCommonInternal.h"
#include "FEMFXBvh.h"
#include "FEMFXBroadPhase.h"
#include "FEMFXConstraintIslands.h"
#include "FEMFXConstraintsBuffer.h"
#include "FEMFXCollisionPairData.h"
#include "FEMFXScene.h"
#include "FEMFXHashSet.h"

namespace AMD
{
    size_t FmGetConstraintsBufferSize(const FmConstraintsBufferSetupParams& params)
    {
        uint maxTetMeshes = params.maxTetMeshes;
        uint maxRigidBodies = params.maxRigidBodies;
        uint maxConstraintIslands = maxTetMeshes + maxRigidBodies;
        uint maxDistanceContacts = params.maxDistanceContacts;
        uint maxVolumeContacts = params.maxVolumeContacts;
        uint maxVolumeContactVerts = params.maxVolumeContactVerts;
        uint maxDeformationConstraints = params.maxDeformationConstraints;
        uint maxGlueConstraints = params.maxGlueConstraints;
        uint maxPlaneConstraints = params.maxPlaneConstraints;
        uint maxRigidBodyAngleConstraints = params.maxRigidBodyAngleConstraints;
        uint maxConstraints = maxDistanceContacts + maxVolumeContacts + maxDeformationConstraints + maxGlueConstraints + maxPlaneConstraints + maxRigidBodyAngleConstraints;
        uint maxBroadPhasePairs = params.maxBroadPhasePairs;
        uint maxRigidBodyBroadPhasePairs = params.maxRigidBodyBroadPhasePairs;
        uint maxBvhNodes = FmNumBvhNodes(maxTetMeshes + maxRigidBodies);
        uint maxBvhLeaves = maxTetMeshes + maxRigidBodies;

        size_t numBytes =
            FM_PAD_64(sizeof(FmConstraintsBuffer)) +
            FM_PAD_64(sizeof(*FmConstraintsBuffer::constraintIslands) * maxConstraintIslands) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::distanceContactsPairInfo) * maxDistanceContacts) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::distanceContacts) * maxDistanceContacts) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::volumeContacts) * maxVolumeContacts) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::volumeContactVerts) * maxVolumeContactVerts) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::deformationConstraints) * maxDeformationConstraints) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::glueConstraints) * maxGlueConstraints) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::planeConstraints) * maxPlaneConstraints) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::rigidBodyAngleConstraints) * maxRigidBodyAngleConstraints) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::gluedObjectPairs.elements) * maxGlueConstraints) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::freeGlueConstraintIds.freeIdsArray) * maxGlueConstraints) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::freePlaneConstraintIds.freeIdsArray) * maxPlaneConstraints) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::freeRigidBodyAngleConstraintIds.freeIdsArray) * maxRigidBodyAngleConstraints) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::islandObjectNodes) * maxConstraintIslands) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::allIslandConstraints) * maxConstraints) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::allIslandTetMeshIds) * maxTetMeshes) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::allIslandRigidBodyIds) * maxRigidBodies) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::allUserRigidBodyIslandIndices) * maxRigidBodies) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::sleepingConstraintIslands) * maxConstraintIslands) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::allSleepingIslandTetMeshIds) * maxTetMeshes) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::allSleepingIslandRigidBodyIds) * maxRigidBodies) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::freeSleepingIslandIds.freeIdsArray) * maxConstraintIslands) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::sleepingIslandIdToArrayIdx) * maxConstraintIslands) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::broadPhaseHierarchy.nodes) * maxBvhNodes) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::broadPhaseHierarchy.primBoxes) * maxBvhLeaves) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::broadPhaseHierarchy.mortonCodesSorted) * maxBvhLeaves) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::broadPhaseHierarchy.primIndicesSorted) * maxBvhLeaves) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::broadPhasePairs) * maxBroadPhasePairs) +
            FM_PAD_16(sizeof(*FmConstraintsBuffer::rigidBodyBroadPhasePairs) * maxRigidBodyBroadPhasePairs);

        return FM_PAD_64(numBytes);
    }

    FmConstraintsBuffer* FmSetupConstraintsBuffer(const FmConstraintsBufferSetupParams& params, uint8_t*& pBuffer, size_t bufferNumBytes)
    {
        FM_ASSERT(((uintptr_t)pBuffer & 0x3f) == 0);
        uint maxTetMeshes = params.maxTetMeshes;
        uint maxRigidBodies = params.maxRigidBodies;
        uint maxConstraintIslands = maxTetMeshes + maxRigidBodies;
        uint maxDistanceContacts = params.maxDistanceContacts;
        uint maxVolumeContacts = params.maxVolumeContacts;
        uint maxVolumeContactVerts = params.maxVolumeContactVerts;
        uint maxDeformationConstraints = params.maxDeformationConstraints;
        uint maxGlueConstraints = params.maxGlueConstraints;
        uint maxPlaneConstraints = params.maxPlaneConstraints;
        uint maxRigidBodyAngleConstraints = params.maxRigidBodyAngleConstraints;
        uint maxConstraints = maxDistanceContacts + maxVolumeContacts + maxDeformationConstraints + maxGlueConstraints + maxPlaneConstraints + maxRigidBodyAngleConstraints;
        uint maxBroadPhasePairs = params.maxBroadPhasePairs;
        uint maxRigidBodyBroadPhasePairs = params.maxRigidBodyBroadPhasePairs;
        uint maxBvhNodes = FmNumBvhNodes(maxTetMeshes + maxRigidBodies);
        uint maxBvhLeaves = maxTetMeshes + maxRigidBodies;

        //uint8_t* pBufferStart = pBuffer;
        uint8_t* pBufferEnd = pBuffer + bufferNumBytes;

        FmConstraintsBuffer* pConstraintsBuffer = FmAllocFromBuffer64<FmConstraintsBuffer>(&pBuffer, 1, pBufferEnd);
        FmInitConstraintsBuffer(pConstraintsBuffer);

        pConstraintsBuffer->bufferNumBytes = bufferNumBytes;

        pConstraintsBuffer->constraintIslands = FmAllocFromBuffer64<FmConstraintIsland>(&pBuffer, maxConstraintIslands, pBufferEnd);

        pConstraintsBuffer->distanceContactsPairInfo = FmAllocFromBuffer<FmDistanceContactPairInfo>(&pBuffer, maxDistanceContacts, pBufferEnd);
        pConstraintsBuffer->distanceContacts = FmAllocFromBuffer<FmDistanceContact>(&pBuffer, maxDistanceContacts, pBufferEnd);
        pConstraintsBuffer->volumeContacts = FmAllocFromBuffer<FmVolumeContact>(&pBuffer, maxVolumeContacts, pBufferEnd);
        pConstraintsBuffer->volumeContactVerts = FmAllocFromBuffer<FmVolumeContactVert>(&pBuffer, maxVolumeContactVerts, pBufferEnd);
        pConstraintsBuffer->deformationConstraints = FmAllocFromBuffer<FmDeformationConstraint>(&pBuffer, maxDeformationConstraints, pBufferEnd);
        pConstraintsBuffer->glueConstraints = FmAllocFromBuffer<FmGlueConstraint>(&pBuffer, maxGlueConstraints, pBufferEnd);
        pConstraintsBuffer->planeConstraints = FmAllocFromBuffer<FmPlaneConstraint>(&pBuffer, maxPlaneConstraints, pBufferEnd);
        pConstraintsBuffer->rigidBodyAngleConstraints = FmAllocFromBuffer<FmRigidBodyAngleConstraint>(&pBuffer, maxRigidBodyAngleConstraints, pBufferEnd);
        FmGluedObjectPairElement* glueObjectPairElements = FmAllocFromBuffer<FmGluedObjectPairElement>(&pBuffer, maxGlueConstraints, pBufferEnd);
        uint* freeGlueConstraintIds = FmAllocFromBuffer<uint>(&pBuffer, maxGlueConstraints, pBufferEnd);
        uint* freePlaneConstraintIds = FmAllocFromBuffer<uint>(&pBuffer, maxPlaneConstraints, pBufferEnd);
        uint* freeRigidBodyAngleConstraintIds = FmAllocFromBuffer<uint>(&pBuffer, maxRigidBodyAngleConstraints, pBufferEnd);
        pConstraintsBuffer->islandObjectNodes = FmAllocFromBuffer<FmCCNode>(&pBuffer, maxConstraintIslands, pBufferEnd);
        pConstraintsBuffer->allIslandConstraints = FmAllocFromBuffer<FmConstraintReference>(&pBuffer, maxConstraints, pBufferEnd);
        pConstraintsBuffer->allIslandTetMeshIds = FmAllocFromBuffer<uint>(&pBuffer, maxTetMeshes, pBufferEnd);
        pConstraintsBuffer->allIslandRigidBodyIds = FmAllocFromBuffer<uint>(&pBuffer, maxRigidBodies, pBufferEnd);
        pConstraintsBuffer->allUserRigidBodyIslandIndices = FmAllocFromBuffer<uint>(&pBuffer, maxRigidBodies, pBufferEnd);
        pConstraintsBuffer->sleepingConstraintIslands = FmAllocFromBuffer<FmSleepingConstraintIsland>(&pBuffer, maxConstraintIslands, pBufferEnd);
        pConstraintsBuffer->allSleepingIslandTetMeshIds = FmAllocFromBuffer<uint>(&pBuffer, maxTetMeshes, pBufferEnd);
        pConstraintsBuffer->allSleepingIslandRigidBodyIds = FmAllocFromBuffer<uint>(&pBuffer, maxRigidBodies, pBufferEnd);
        uint* freeSleepingIslandIds = FmAllocFromBuffer<uint>(&pBuffer, maxConstraintIslands, pBufferEnd);
        pConstraintsBuffer->sleepingIslandIdToArrayIdx = FmAllocFromBuffer<uint>(&pBuffer, maxConstraintIslands, pBufferEnd);
        pConstraintsBuffer->broadPhaseHierarchy.nodes = FmAllocFromBuffer<FmBvhNode>(&pBuffer, maxBvhNodes, pBufferEnd);
        pConstraintsBuffer->broadPhaseHierarchy.primBoxes = FmAllocFromBuffer<FmAabb>(&pBuffer, maxBvhLeaves, pBufferEnd);
        pConstraintsBuffer->broadPhaseHierarchy.mortonCodesSorted = FmAllocFromBuffer<int>(&pBuffer, maxBvhLeaves, pBufferEnd);
        pConstraintsBuffer->broadPhaseHierarchy.primIndicesSorted = FmAllocFromBuffer<int>(&pBuffer, maxBvhLeaves, pBufferEnd);
        pConstraintsBuffer->broadPhasePairs = FmAllocFromBuffer<FmBroadPhasePair>(&pBuffer, maxBroadPhasePairs, pBufferEnd);
        pConstraintsBuffer->rigidBodyBroadPhasePairs = FmAllocFromBuffer<FmBroadPhasePair>(&pBuffer, maxRigidBodyBroadPhasePairs, pBufferEnd);

        FmInitHashSet(&pConstraintsBuffer->gluedObjectPairs, glueObjectPairElements, maxGlueConstraints);

        FmInitFreeIds(&pConstraintsBuffer->freeGlueConstraintIds, freeGlueConstraintIds, maxGlueConstraints);
        FmInitFreeIds(&pConstraintsBuffer->freePlaneConstraintIds, freePlaneConstraintIds, maxPlaneConstraints);
        FmInitFreeIds(&pConstraintsBuffer->freeRigidBodyAngleConstraintIds, freeRigidBodyAngleConstraintIds, maxRigidBodyAngleConstraints);
        FmInitFreeIds(&pConstraintsBuffer->freeSleepingIslandIds, freeSleepingIslandIds, maxConstraintIslands);

        for (uint i = 0; i < maxConstraintIslands; i++)
        {
            pConstraintsBuffer->sleepingIslandIdToArrayIdx[i] = FM_INVALID_ID;
        }

        pConstraintsBuffer->maxDistanceContacts = maxDistanceContacts;
        pConstraintsBuffer->maxVolumeContacts = maxVolumeContacts;
        pConstraintsBuffer->maxVolumeContactVerts = maxVolumeContactVerts;
        pConstraintsBuffer->maxDeformationConstraints = maxDeformationConstraints;
        pConstraintsBuffer->maxGlueConstraints = maxGlueConstraints;
        pConstraintsBuffer->maxPlaneConstraints = maxPlaneConstraints;
        pConstraintsBuffer->maxRigidBodyAngleConstraints = maxRigidBodyAngleConstraints;
        pConstraintsBuffer->maxConstraintIslands = maxConstraintIslands;
        pConstraintsBuffer->maxBroadPhasePairs = maxBroadPhasePairs;
        pConstraintsBuffer->maxRigidBodyBroadPhasePairs = maxRigidBodyBroadPhasePairs;

        pBuffer = (uint8_t*)FM_PAD_64((uintptr_t)pBuffer);

        FM_ASSERT(((uintptr_t)pBuffer & 0x3f) == 0);
        return pConstraintsBuffer;
    }

    FmVolumeContactWorkspace* FmAllocVolumeContactWorkspace(uint8_t*& pBuffer, size_t maxFreeBytes, uint numVertsA, uint numVertsB, uint numFacesA
#if FM_SURFACE_INTERSECTION_CONTACTS
        , uint maxFacePairs
#endif
        )
    {
        // Double storage due to max load factor
        uint maxVertElementsA, maxVertElementsB, maxVertSetsA, maxVertSetsB;

        FmGetExpandableHashSetSizes(&maxVertElementsA, &maxVertSetsA, numVertsA);
        FmGetExpandableHashSetSizes(&maxVertElementsB, &maxVertSetsB, numVertsB);

        uint maxVertElements = maxVertElementsA + maxVertElementsB;
        uint maxVertSets = maxVertSetsA + maxVertSetsB;

        uint maxFaces = numFacesA;

        size_t workspaceNumBytes =
            FM_PAD_16(sizeof(FmVolumeContactWorkspace))
            + FM_PAD_16(sizeof(FmVolumeContactVertSet) * maxVertSets)
            + FM_PAD_16(sizeof(FmVolumeContactVertSetElement) * maxVertElements)
            + FM_PAD_16(sizeof(FmVolumeContactFace) * maxFaces)
#if FM_SURFACE_INTERSECTION_CONTACTS
            + FM_PAD_16(sizeof(FmIntersectingFacePair) * maxFacePairs)
#endif
            ;

        if (workspaceNumBytes > maxFreeBytes)
        {
            return NULL;
        }

        uint8_t* workspaceStart = pBuffer;
        size_t workspaceOffset = 0;

        // Suballocate structures
        FmVolumeContactWorkspace* pWorkspace = (FmVolumeContactWorkspace*)(workspaceStart + workspaceOffset);
        workspaceOffset += FM_PAD_16(sizeof(FmVolumeContactWorkspace));

        FmVolumeContactVertSet* pSets = (FmVolumeContactVertSet*)(workspaceStart + workspaceOffset);
        workspaceOffset += FM_PAD_16(sizeof(FmVolumeContactVertSet) * maxVertSets);

        FmVolumeContactVertSetElement* pElements = (FmVolumeContactVertSetElement*)(workspaceStart + workspaceOffset);
        workspaceOffset += FM_PAD_16(sizeof(FmVolumeContactVertSetElement) * maxVertElements);

        FmVolumeContactFace* pFaceElements = (FmVolumeContactFace*)(workspaceStart + workspaceOffset);
        workspaceOffset += FM_PAD_16(sizeof(FmVolumeContactFace) * maxFaces);

#if FM_SURFACE_INTERSECTION_CONTACTS
        FmIntersectingFacePair *pFacePairs = (FmIntersectingFacePair*)(workspaceStart + workspaceOffset);
        workspaceOffset += FM_PAD_16(sizeof(FmIntersectingFacePair) * maxFacePairs);
#endif

        // Init workspace struct
        uint initialVertSetSizeA = FmMinUint((uint)32, maxVertElementsA);
        pWorkspace->meshAVertSets.elements = pElements;
        pWorkspace->meshAVertSets.sets = pSets;
        pWorkspace->meshAVertSets.numElementsUsed = initialVertSetSizeA;
        pWorkspace->meshAVertSets.maxElements = maxVertElementsA;
        pWorkspace->meshAVertSets.numSets = 1;
        pWorkspace->meshAVertSets.maxSets = maxVertSetsA;

        uint initialVertSetSizeB = FmMinUint((uint)32, maxVertElementsB);
        pWorkspace->meshBVertSets.elements = pElements + maxVertElementsA;
        pWorkspace->meshBVertSets.sets = pSets + maxVertSetsA;
        pWorkspace->meshBVertSets.numElementsUsed = initialVertSetSizeB;
        pWorkspace->meshBVertSets.maxElements = maxVertElementsB;
        pWorkspace->meshBVertSets.numSets = 1;
        pWorkspace->meshBVertSets.maxSets = maxVertSetsB;

        pWorkspace->meshAFaces = pFaceElements;

        // Init first sets
        FmInitHashSet(&pWorkspace->meshAVertSets.sets[0], pWorkspace->meshAVertSets.elements, initialVertSetSizeA);
        FmInitHashSet(&pWorkspace->meshBVertSets.sets[0], pWorkspace->meshBVertSets.elements, initialVertSetSizeB);

#if FM_SURFACE_INTERSECTION_CONTACTS
        pWorkspace->intersectingFacePairs = pFacePairs;
        pWorkspace->numIntersectingFacePairs = 0;
        pWorkspace->maxIntersectingFacePairs = maxFacePairs;
#endif

        pBuffer += workspaceNumBytes;

        return pWorkspace;
    }

#if FM_SOA_TRI_INTERSECTION
    FmMeshCollisionTriPair* FmAllocMeshCollisionTriPairs(uint8_t *& pBuffer, size_t maxFreeBytes, uint maxObjectPairTriPairs)
    {
        uint numBytes = FM_PAD_16(sizeof(FmMeshCollisionTriPair) * maxObjectPairTriPairs);

        if (numBytes > maxFreeBytes)
        {
            return NULL;
        }

        FmMeshCollisionTriPair* pMeshCollisionTriPairs = (FmMeshCollisionTriPair*)pBuffer;

        pBuffer += numBytes;

        return pMeshCollisionTriPairs;
    }
#endif

#if FM_CONTACT_REDUCTION
    FmContactReductionWorkspace* FmAllocContactReductionWorkspace(uint8_t*& pBuffer, size_t maxFreeBytes, uint numVertsA, uint numVertsB)
    {
        size_t workspaceNumBytes =
            FM_PAD_16(sizeof(FmContactReductionWorkspace)) +
            FM_PAD_16(sizeof(FmVertKeptContactSet)*numVertsA) +
            FM_PAD_16(sizeof(FmVertKeptContactSet)*numVertsB);

        if (workspaceNumBytes > maxFreeBytes)
        {
            return NULL;
        }

        uint8_t* workspaceStart = pBuffer;
        size_t workspaceOffset = 0;

        // Suballocate structures
        FmContactReductionWorkspace* pWorkspace = (FmContactReductionWorkspace*)(workspaceStart + workspaceOffset);
        workspaceOffset += FM_PAD_16(sizeof(FmContactReductionWorkspace));

        pWorkspace->vertsContactsA = (FmVertKeptContactSet*)(workspaceStart + workspaceOffset);
        workspaceOffset += FM_PAD_16(sizeof(FmVertKeptContactSet) * numVertsA);

        pWorkspace->vertsContactsB = (FmVertKeptContactSet*)(workspaceStart + workspaceOffset);
        workspaceOffset += FM_PAD_16(sizeof(FmVertKeptContactSet) * numVertsB);

        for (uint i = 0; i < numVertsA; i++)
        {
            pWorkspace->vertsContactsA[i].numContactsTimeWeighted = 0;
            pWorkspace->vertsContactsA[i].numContactsDistWeighted = 0;
        }
        for (uint i = 0; i < numVertsB; i++)
        {
            pWorkspace->vertsContactsB[i].numContactsTimeWeighted = 0;
            pWorkspace->vertsContactsB[i].numContactsDistWeighted = 0;
        }

        pWorkspace->numVertsA = numVertsA;
        pWorkspace->numVertsB = numVertsB;

        pBuffer += workspaceNumBytes;

        return pWorkspace;
    }
#endif

    bool FmAllocCollidedObjectPairTemps(FmCollidedObjectPairTemps* temps, uint8_t*& pBuffer, uint8_t* pBufferEnd, 
        uint maxObjVerts, uint numVertsA, uint numVertsB, uint numFacesA, uint numFacesB, bool allocContactReductionWorkspace
#if FM_SURFACE_INTERSECTION_CONTACTS
        , uint maxSurfaceIntersectionPairs
#endif
    )
    {
        (void)numFacesB;

        if (numVertsA > maxObjVerts || numVertsB > maxObjVerts)
        {
            return false;
        }

        temps->volContactWorkspace = FmAllocVolumeContactWorkspace(pBuffer, (size_t)(pBufferEnd - pBuffer), numVertsA, numVertsB, numFacesA
#if FM_SURFACE_INTERSECTION_CONTACTS
            , maxSurfaceIntersectionPairs
#endif
        );

#if FM_SOA_TRI_INTERSECTION
        temps->meshCollisionTriPairs = FmAllocMeshCollisionTriPairs(pBuffer, (size_t)(pBufferEnd - pBuffer), FM_MAX_MESH_COLLISION_TRI_PAIR);
        temps->numMeshCollisionTriPairs = 0;
#endif

#if FM_CONTACT_REDUCTION
        if (allocContactReductionWorkspace)
        {
            temps->contactReductionWorkspace = FmAllocContactReductionWorkspace(pBuffer, (size_t)(pBufferEnd - pBuffer), numVertsA, numVertsB);
        }
        else
        {
            temps->contactReductionWorkspace = NULL;
        }
#endif

        temps->distanceContactPairInfoBuffer = FmAllocFromBuffer<FmDistanceContactPairInfo>(&pBuffer, FM_MAX_TEMP_DISTANCE_CONTACTS, pBufferEnd);
        temps->distanceContactBuffer = FmAllocFromBuffer<FmDistanceContact>(&pBuffer, FM_MAX_TEMP_DISTANCE_CONTACTS, pBufferEnd);
        temps->distanceContactTetVertIds = FmAllocFromBuffer<FmDistanceContactTetVertIds>(&pBuffer, FM_MAX_TEMP_DISTANCE_CONTACTS, pBufferEnd);
        temps->numDistanceContacts = 0;
        temps->numDistanceContactsNonzeroRefCount = 0;

        if (temps->volContactWorkspace == NULL
#if FM_SOA_TRI_INTERSECTION
            || temps->meshCollisionTriPairs == NULL
#endif
#if FM_CONTACT_REDUCTION
            || (allocContactReductionWorkspace && temps->contactReductionWorkspace == NULL)
#endif
            || temps->distanceContactPairInfoBuffer == NULL
            || temps->distanceContactBuffer == NULL
            || temps->distanceContactTetVertIds == NULL
            )
        {
            return false;
        }

        return true;
    }

    // Get constraint pointer by id; returns NULL if not found.
    FmGlueConstraint* FmGetGlueConstraint(const FmScene& scene, uint glueConstraintId)
    {
        const FmConstraintsBuffer* constraintsBuffer = scene.constraintsBuffer;

        if (glueConstraintId >= constraintsBuffer->numGlueConstraints)
        {
            return NULL;
        }

        const FmGlueConstraint& glueConstraint = constraintsBuffer->glueConstraints[glueConstraintId];

        if (FM_IS_SET(glueConstraint.flags, FM_CONSTRAINT_FLAG_DELETED))
        {
            return NULL;
        }

        return &constraintsBuffer->glueConstraints[glueConstraintId];
    }

    FmPlaneConstraint* FmGetPlaneConstraint(const FmScene& scene, uint planeConstraintId)
    {
        const FmConstraintsBuffer* constraintsBuffer = scene.constraintsBuffer;

        if (planeConstraintId >= constraintsBuffer->numPlaneConstraints)
        {
            return NULL;
        }

        const FmPlaneConstraint& planeConstraint = constraintsBuffer->planeConstraints[planeConstraintId];

        if (FM_IS_SET(planeConstraint.flags, FM_CONSTRAINT_FLAG_DELETED))
        {
            return NULL;
        }

        return &constraintsBuffer->planeConstraints[planeConstraintId];
    }

    FmRigidBodyAngleConstraint* FmGetRigidBodyAngleConstraint(const FmScene& scene, uint rigidBodyAngleConstraintId)
    {
        const FmConstraintsBuffer* constraintsBuffer = scene.constraintsBuffer;

        if (rigidBodyAngleConstraintId >= constraintsBuffer->numRigidBodyAngleConstraints)
        {
            return NULL;
        }

        const FmRigidBodyAngleConstraint& rigidBodyAngleConstraint = constraintsBuffer->rigidBodyAngleConstraints[rigidBodyAngleConstraintId];

        if (FM_IS_SET(rigidBodyAngleConstraint.flags, FM_CONSTRAINT_FLAG_DELETED))
        {
            return NULL;
        }

        return &constraintsBuffer->rigidBodyAngleConstraints[rigidBodyAngleConstraintId];
    }

    uint FmAddGlueConstraintToScene(FmScene* scene, const FmGlueConstraint& inGlueConstraint)
    {
        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;

        uint constraintIdx = FmReserveId(&constraintsBuffer->freeGlueConstraintIds);

        if (constraintIdx == FM_INVALID_ID)
        {
            return FM_INVALID_ID;
        }

        if (constraintIdx >= constraintsBuffer->numGlueConstraints)
        {
            constraintsBuffer->numGlueConstraints = constraintIdx + 1;
        }

        FmGlueConstraint& glueConstraint = constraintsBuffer->glueConstraints[constraintIdx];

        glueConstraint = inGlueConstraint;

        return constraintIdx;
    }

    uint FmAddGlueConstraintToScene(FmScene* scene, const FmGlueConstraintSetupParams& inGlueConstraintSetupParams)
    {
        FmGlueConstraint glueConstraint;

        glueConstraint.bufferIdA = inGlueConstraintSetupParams.bufferIdA;
        glueConstraint.bufferIdB = inGlueConstraintSetupParams.bufferIdB;
        glueConstraint.bufferTetIdA = inGlueConstraintSetupParams.bufferTetIdA;
        glueConstraint.bufferTetIdB = inGlueConstraintSetupParams.bufferTetIdB;
        glueConstraint.posBaryA[0] = inGlueConstraintSetupParams.posBaryA[0];
        glueConstraint.posBaryA[1] = inGlueConstraintSetupParams.posBaryA[1];
        glueConstraint.posBaryA[2] = inGlueConstraintSetupParams.posBaryA[2];
        glueConstraint.posBaryA[3] = inGlueConstraintSetupParams.posBaryA[3];
        glueConstraint.posBaryB[0] = inGlueConstraintSetupParams.posBaryB[0];
        glueConstraint.posBaryB[1] = inGlueConstraintSetupParams.posBaryB[1];
        glueConstraint.posBaryB[2] = inGlueConstraintSetupParams.posBaryB[2];
        glueConstraint.posBaryB[3] = inGlueConstraintSetupParams.posBaryB[3];
        glueConstraint.breakThreshold = inGlueConstraintSetupParams.breakThreshold;
        glueConstraint.kVelCorrection = inGlueConstraintSetupParams.kVelCorrection;
        glueConstraint.kPosCorrection = inGlueConstraintSetupParams.kPosCorrection;
        glueConstraint.minGlueConstraints = inGlueConstraintSetupParams.minGlueConstraints;
        if (!inGlueConstraintSetupParams.enabled)
        {
            glueConstraint.flags |= FM_CONSTRAINT_FLAG_DISABLED;
        }

        return FmAddGlueConstraintToScene(scene, glueConstraint);
    }

    uint FmAddPlaneConstraintToScene(FmScene* scene, const FmPlaneConstraint& inPlaneConstraint)
    {
        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;

        uint constraintIdx = FmReserveId(&constraintsBuffer->freePlaneConstraintIds);

        if (constraintIdx == FM_INVALID_ID)
        {
            return FM_INVALID_ID;
        }

        if (constraintIdx >= constraintsBuffer->numPlaneConstraints)
        {
            constraintsBuffer->numPlaneConstraints = constraintIdx + 1;
        }

        FmPlaneConstraint& planeConstraint = constraintsBuffer->planeConstraints[constraintIdx];

        planeConstraint = inPlaneConstraint;

        return constraintIdx;
    }

    uint FmAddPlaneConstraintToScene(FmScene* scene, const FmPlaneConstraintSetupParams& inPlaneConstraintSetupParams)
    {
        FmPlaneConstraint planeConstraint;

        planeConstraint.bufferIdA = inPlaneConstraintSetupParams.bufferIdA;
        planeConstraint.bufferIdB = inPlaneConstraintSetupParams.bufferIdB;
        planeConstraint.bufferTetIdA = inPlaneConstraintSetupParams.bufferTetIdA;
        planeConstraint.bufferTetIdB = inPlaneConstraintSetupParams.bufferTetIdB;
        planeConstraint.posBaryA[0] = inPlaneConstraintSetupParams.posBaryA[0];
        planeConstraint.posBaryA[1] = inPlaneConstraintSetupParams.posBaryA[1];
        planeConstraint.posBaryA[2] = inPlaneConstraintSetupParams.posBaryA[2];
        planeConstraint.posBaryA[3] = inPlaneConstraintSetupParams.posBaryA[3];
        planeConstraint.posBaryB[0] = inPlaneConstraintSetupParams.posBaryB[0];
        planeConstraint.posBaryB[1] = inPlaneConstraintSetupParams.posBaryB[1];
        planeConstraint.posBaryB[2] = inPlaneConstraintSetupParams.posBaryB[2];
        planeConstraint.posBaryB[3] = inPlaneConstraintSetupParams.posBaryB[3];
        planeConstraint.planeNormal0 = inPlaneConstraintSetupParams.planeNormal0;
        planeConstraint.planeNormal1 = inPlaneConstraintSetupParams.planeNormal1;
        planeConstraint.planeNormal2 = inPlaneConstraintSetupParams.planeNormal2;
        planeConstraint.bias0 = inPlaneConstraintSetupParams.bias0;
        planeConstraint.bias1 = inPlaneConstraintSetupParams.bias1;
        planeConstraint.bias2 = inPlaneConstraintSetupParams.bias2;
        planeConstraint.kVelCorrection = inPlaneConstraintSetupParams.kVelCorrection;
        planeConstraint.kPosCorrection = inPlaneConstraintSetupParams.kPosCorrection;
        if (inPlaneConstraintSetupParams.numDimensions == 1)
        {
            planeConstraint.flags |= FM_CONSTRAINT_FLAG_1D;
        }
        else if (inPlaneConstraintSetupParams.numDimensions == 2)
        {
            planeConstraint.flags |= FM_CONSTRAINT_FLAG_2D;
        }
        if (inPlaneConstraintSetupParams.nonNeg0)
        {
            planeConstraint.flags |= FM_CONSTRAINT_FLAG_NONNEG0;
        }
        if (inPlaneConstraintSetupParams.nonNeg1)
        {
            planeConstraint.flags |= FM_CONSTRAINT_FLAG_NONNEG1;
        }
        if (inPlaneConstraintSetupParams.nonNeg2)
        {
            planeConstraint.flags |= FM_CONSTRAINT_FLAG_NONNEG2;
        }
        if (!inPlaneConstraintSetupParams.enabled)
        {
            planeConstraint.flags |= FM_CONSTRAINT_FLAG_DISABLED;
        }

        return FmAddPlaneConstraintToScene(scene, planeConstraint);
    }

    uint FmAddRigidBodyAngleConstraintToScene(FmScene* scene, const FmRigidBodyAngleConstraint& inRigidBodyAngleConstraint)
    {
        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;

        uint constraintIdx = FmReserveId(&constraintsBuffer->freeRigidBodyAngleConstraintIds);

        if (constraintIdx == FM_INVALID_ID)
        {
            return FM_INVALID_ID;
        }

        if (constraintIdx >= constraintsBuffer->numRigidBodyAngleConstraints)
        {
            constraintsBuffer->numRigidBodyAngleConstraints = constraintIdx + 1;
        }

        FmRigidBodyAngleConstraint& rigidBodyAngleConstraint = constraintsBuffer->rigidBodyAngleConstraints[constraintIdx];

        rigidBodyAngleConstraint = inRigidBodyAngleConstraint;

        return constraintIdx;
    }

    uint FmAddRigidBodyAngleConstraintToScene(FmScene* scene, const FmRigidBodyAngleConstraintSetupParams& inRigidBodyAngleConstraintSetupParams)
    {
        FmRigidBodyAngleConstraint angleConstraint;

        angleConstraint.objectIdA = inRigidBodyAngleConstraintSetupParams.objectIdA;
        angleConstraint.objectIdB = inRigidBodyAngleConstraintSetupParams.objectIdB;
        angleConstraint.axisBodySpaceA = inRigidBodyAngleConstraintSetupParams.axisBodySpaceA;
        angleConstraint.axisBodySpaceB = inRigidBodyAngleConstraintSetupParams.axisBodySpaceB;
        angleConstraint.frictionCoeff = inRigidBodyAngleConstraintSetupParams.frictionCoeff;
        angleConstraint.kVelCorrection = inRigidBodyAngleConstraintSetupParams.kVelCorrection;
        angleConstraint.kPosCorrection = inRigidBodyAngleConstraintSetupParams.kPosCorrection;
        angleConstraint.type = inRigidBodyAngleConstraintSetupParams.type;
        if (!inRigidBodyAngleConstraintSetupParams.enabled)
        {
            angleConstraint.flags |= FM_CONSTRAINT_FLAG_DISABLED;
        }

        return FmAddRigidBodyAngleConstraintToScene(scene, angleConstraint);
    }

    // Remove a constraint from the scene.
    void FmRemoveGlueConstraintFromScene(FmScene* scene, uint glueConstraintId)
    {
        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;

        if (glueConstraintId >= constraintsBuffer->numGlueConstraints)
        {
            return;
        }

        FmGlueConstraint& glueConstraint = constraintsBuffer->glueConstraints[glueConstraintId];

        if (FM_IS_SET(glueConstraint.flags, FM_CONSTRAINT_FLAG_DELETED))
        {
            return;
        }

        glueConstraint = FmGlueConstraint();
        glueConstraint.flags |= FM_CONSTRAINT_FLAG_DELETED;

        FmReleaseId(&constraintsBuffer->freeGlueConstraintIds, glueConstraintId);
    }

    void FmRemovePlaneConstraintFromScene(FmScene* scene, uint planeConstraintId)
    {
        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;

        if (planeConstraintId >= constraintsBuffer->numPlaneConstraints)
        {
            return;
        }

        FmPlaneConstraint& planeConstraint = constraintsBuffer->planeConstraints[planeConstraintId];

        if (FM_IS_SET(planeConstraint.flags, FM_CONSTRAINT_FLAG_DELETED))
        {
            return;
        }

        planeConstraint = FmPlaneConstraint();
        planeConstraint.flags |= FM_CONSTRAINT_FLAG_DELETED;

        FmReleaseId(&constraintsBuffer->freePlaneConstraintIds, planeConstraintId);
    }

    void FmRemoveRigidBodyAngleConstraintFromScene(FmScene* scene, uint rigidBodyAngleConstraintId)
    {
        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;

        if (rigidBodyAngleConstraintId >= constraintsBuffer->numRigidBodyAngleConstraints)
        {
            return;
        }

        FmRigidBodyAngleConstraint& rigidBodyAngleConstraint = constraintsBuffer->rigidBodyAngleConstraints[rigidBodyAngleConstraintId];

        if (FM_IS_SET(rigidBodyAngleConstraint.flags, FM_CONSTRAINT_FLAG_DELETED))
        {
            return;
        }

        rigidBodyAngleConstraint = FmRigidBodyAngleConstraint();
        rigidBodyAngleConstraint.flags |= FM_CONSTRAINT_FLAG_DELETED;

        FmReleaseId(&constraintsBuffer->freeRigidBodyAngleConstraintIds, rigidBodyAngleConstraintId);
    }

    FmGlueConstraintSetupParams FmGetGlueConstraintParams(const FmScene& scene, uint glueConstraintId)
    {
        FmGlueConstraintSetupParams params;
        FmGlueConstraint* glueConstraint = FmGetGlueConstraint(scene, glueConstraintId);

        if (glueConstraint)
        {
            params.bufferIdA = glueConstraint->bufferIdA;
            params.bufferIdB = glueConstraint->bufferIdB;
            params.bufferTetIdA = glueConstraint->bufferTetIdA;
            params.bufferTetIdB = glueConstraint->bufferTetIdB;
            params.posBaryA[0] = glueConstraint->posBaryA[0];
            params.posBaryA[1] = glueConstraint->posBaryA[1];
            params.posBaryA[2] = glueConstraint->posBaryA[2];
            params.posBaryA[3] = glueConstraint->posBaryA[3];
            params.posBaryB[0] = glueConstraint->posBaryB[0];
            params.posBaryB[1] = glueConstraint->posBaryB[1];
            params.posBaryB[2] = glueConstraint->posBaryB[2];
            params.posBaryB[3] = glueConstraint->posBaryB[3];
            params.breakThreshold = glueConstraint->breakThreshold;
            params.kVelCorrection = glueConstraint->kVelCorrection;
            params.kPosCorrection = glueConstraint->kPosCorrection;
            params.minGlueConstraints = glueConstraint->minGlueConstraints;
            params.enabled = !FM_IS_SET(glueConstraint->flags, FM_CONSTRAINT_FLAG_DISABLED);
        }

        return params;
    }

    FmPlaneConstraintSetupParams FmGetPlaneConstraintParams(const FmScene& scene, uint planeConstraintId)
    {
        FmPlaneConstraintSetupParams params;
        FmPlaneConstraint* planeConstraint = FmGetPlaneConstraint(scene, planeConstraintId);

        if (planeConstraint)
        {
            params.bufferIdA = planeConstraint->bufferIdA;
            params.bufferIdB = planeConstraint->bufferIdB;
            params.bufferTetIdA = planeConstraint->bufferTetIdA;
            params.bufferTetIdB = planeConstraint->bufferTetIdB;
            params.posBaryA[0] = planeConstraint->posBaryA[0];
            params.posBaryA[1] = planeConstraint->posBaryA[1];
            params.posBaryA[2] = planeConstraint->posBaryA[2];
            params.posBaryA[3] = planeConstraint->posBaryA[3];
            params.posBaryB[0] = planeConstraint->posBaryB[0];
            params.posBaryB[1] = planeConstraint->posBaryB[1];
            params.posBaryB[2] = planeConstraint->posBaryB[2];
            params.posBaryB[3] = planeConstraint->posBaryB[3];
            params.planeNormal0 = planeConstraint->planeNormal0;
            params.planeNormal1 = planeConstraint->planeNormal1;
            params.planeNormal2 = planeConstraint->planeNormal2;
            params.bias0 = planeConstraint->bias0;
            params.bias1 = planeConstraint->bias1;
            params.bias2 = planeConstraint->bias2;
            params.kVelCorrection = planeConstraint->kVelCorrection;
            params.kPosCorrection = planeConstraint->kPosCorrection;
            if (FM_IS_SET(planeConstraint->flags, FM_CONSTRAINT_FLAG_1D))
            {
                params.numDimensions = 1;
            }
            else if (FM_IS_SET(planeConstraint->flags, FM_CONSTRAINT_FLAG_2D))
            {
                params.numDimensions = 2;
            }
            else
            {
                params.numDimensions = 3;
            }
            params.nonNeg0 = FM_IS_SET(planeConstraint->flags, FM_CONSTRAINT_FLAG_NONNEG0);
            params.nonNeg1 = FM_IS_SET(planeConstraint->flags, FM_CONSTRAINT_FLAG_NONNEG1);
            params.nonNeg2 = FM_IS_SET(planeConstraint->flags, FM_CONSTRAINT_FLAG_NONNEG2);
            params.enabled = !FM_IS_SET(planeConstraint->flags, FM_CONSTRAINT_FLAG_DISABLED);
        }
        return params;
    }

    FmRigidBodyAngleConstraintSetupParams FmGetRigidBodyAngleConstraintParams(const FmScene& scene, uint rigidBodyAngleConstraintId)
    {
        FmRigidBodyAngleConstraintSetupParams params;
        FmRigidBodyAngleConstraint* rigidBodyAngleConstraint = FmGetRigidBodyAngleConstraint(scene, rigidBodyAngleConstraintId);

        if (rigidBodyAngleConstraint)
        {
            params.objectIdA = rigidBodyAngleConstraint->objectIdA;
            params.objectIdB = rigidBodyAngleConstraint->objectIdB;
            params.axisBodySpaceA = rigidBodyAngleConstraint->axisBodySpaceA;
            params.axisBodySpaceB = rigidBodyAngleConstraint->axisBodySpaceB;
            params.frictionCoeff = rigidBodyAngleConstraint->frictionCoeff;
            params.kVelCorrection = rigidBodyAngleConstraint->kVelCorrection;
            params.kPosCorrection = rigidBodyAngleConstraint->kPosCorrection;
            params.type = rigidBodyAngleConstraint->type;
            params.enabled = !FM_IS_SET(rigidBodyAngleConstraint->flags, FM_CONSTRAINT_FLAG_DISABLED);
        }

        return params;
    }

    FmVector3 FmGetGlueConstraintImpulse(const FmScene& scene, uint glueConstraintId)
    {
        FmGlueConstraint* glueConstraint = FmGetGlueConstraint(scene, glueConstraintId);

        if (glueConstraint)
        {
            return FmInitVector3(glueConstraint->lambdaX, glueConstraint->lambdaY, glueConstraint->lambdaZ);
        }

        return FmInitVector3(0.0f);
    }

    void FmEnableGlueConstraint(FmScene* scene, uint glueConstraintId, bool enabled)
    {
        FmGlueConstraint* glueConstraint = FmGetGlueConstraint(*scene, glueConstraintId);

        if (glueConstraint)
        {
            if (enabled)
            {
                glueConstraint->flags &= ~FM_CONSTRAINT_FLAG_DISABLED;
            }
            else
            {
                glueConstraint->flags |= FM_CONSTRAINT_FLAG_DISABLED;
            }
        }
    }

    void FmEnablePlaneConstraint(FmScene* scene, uint planeConstraintId, bool enabled)
    {
        FmPlaneConstraint* planeConstraint = FmGetPlaneConstraint(*scene, planeConstraintId);

        if (planeConstraint)
        {
            if (enabled)
            {
                planeConstraint->flags &= ~FM_CONSTRAINT_FLAG_DISABLED;
            }
            else
            {
                planeConstraint->flags |= FM_CONSTRAINT_FLAG_DISABLED;
            }
        }
    }

    void FmEnableRigidBodyAngleConstraint(FmScene* scene, uint rigidBodyAngleConstraintId, bool enabled)
    {
        FmRigidBodyAngleConstraint* rigidBodyAngleConstraint = FmGetRigidBodyAngleConstraint(*scene, rigidBodyAngleConstraintId);

        if (rigidBodyAngleConstraint)
        {
            if (enabled)
            {
                rigidBodyAngleConstraint->flags &= ~FM_CONSTRAINT_FLAG_DISABLED;
            }
            else
            {
                rigidBodyAngleConstraint->flags |= FM_CONSTRAINT_FLAG_DISABLED;
            }
        }
    }

    bool FmGetGlueConstraintEnabled(const FmScene& scene, uint glueConstraintId)
    {
        FmGlueConstraint* glueConstraint = FmGetGlueConstraint(scene, glueConstraintId);

        if (glueConstraint == NULL)
        {
            return false;
        }
        else
        {
            return !FM_IS_SET(glueConstraint->flags, FM_CONSTRAINT_FLAG_DISABLED);
        }
    }

    bool FmGetPlaneConstraintEnabled(const FmScene& scene, uint planeConstraintId)
    {
        FmPlaneConstraint* planeConstraint = FmGetPlaneConstraint(scene, planeConstraintId);

        if (planeConstraint == NULL)
        {
            return false;
        }
        else
        {
            return !FM_IS_SET(planeConstraint->flags, FM_CONSTRAINT_FLAG_DISABLED);
        }
    }

    bool FmGetRigidBodyAngleConstraintEnabled(const FmScene& scene, uint rigidBodyAngleConstraintId)
    {
        FmRigidBodyAngleConstraint* rigidBodyAngleConstraint = FmGetRigidBodyAngleConstraint(scene, rigidBodyAngleConstraintId);

        if (rigidBodyAngleConstraint == NULL)
        {
            return false;
        }
        else
        {
            return !FM_IS_SET(rigidBodyAngleConstraint->flags, FM_CONSTRAINT_FLAG_DISABLED);
        }
    }

    void FmSetPlaneConstraintNormals(FmScene* scene, uint planeConstraintId, 
        const FmVector3& planeNormal0, const FmVector3& planeNormal1, const FmVector3& planeNormal2)
    {
        FmPlaneConstraint* planeConstraint = FmGetPlaneConstraint(*scene, planeConstraintId);

        if (planeConstraint)
        {
            planeConstraint->planeNormal0 = planeNormal0;
            planeConstraint->planeNormal1 = planeNormal1;
            planeConstraint->planeNormal2 = planeNormal2;
        }
    }

    void FmSetPlaneConstraintBiases(FmScene* scene, uint planeConstraintId,
        float bias0, float bias1, float bias2)
    {
        FmPlaneConstraint* planeConstraint = FmGetPlaneConstraint(*scene, planeConstraintId);

        if (planeConstraint)
        {
            planeConstraint->bias0 = bias0;
            planeConstraint->bias1 = bias1;
            planeConstraint->bias2 = bias2;
        }
    }

    void FmSetRigidBodyAngleConstraintFrictionCoeff(FmScene* scene, uint rigidBodyAngleConstraintId, float frictionCoeff)
    {
        FmRigidBodyAngleConstraint* rigidBodyAngleConstraint = FmGetRigidBodyAngleConstraint(*scene, rigidBodyAngleConstraintId);

        if (rigidBodyAngleConstraint)
        {
            rigidBodyAngleConstraint->frictionCoeff = frictionCoeff;
        }
    }
}