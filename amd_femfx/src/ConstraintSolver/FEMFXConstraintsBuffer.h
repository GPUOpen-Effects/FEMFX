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

#pragma once

#include "AMD_FEMFX.h"
#include "FEMFXHashSet.h"
#include "FEMFXUserConstraints.h"
#include "FEMFXFreeIds.h"

namespace AMD
{
    struct FmDeformationConstraint;
    struct FmCCNode;
    struct FmConstraintReference;
    struct FmBroadPhasePair;
    struct FmConstraintIsland;
    struct FmSleepingConstraintIsland;
    struct FmCollidedObjectPairTemps;

    struct FmGluedObjectPairKey
    {
        uint objectIdA;
        uint objectIdB;
    };

    struct FmGluedObjectPairElement
    {
        FmGluedObjectPairKey key;
        uint glueCount;
    };

    struct FmGluedObjectPairSetFuncs
    {
        static bool IsValidKey(const FmGluedObjectPairKey& key)
        {
            return (key.objectIdA != FM_INVALID_ID);
        }

        static bool KeysEqual(const FmGluedObjectPairKey& keyA, const FmGluedObjectPairKey& keyB)
        {
            return (keyA.objectIdA == keyB.objectIdA && keyA.objectIdB == keyB.objectIdB)
                || (keyA.objectIdA == keyB.objectIdB && keyA.objectIdB == keyB.objectIdA);
        }

        static void SetInvalidKey(FmGluedObjectPairKey& key)
        {
            key.objectIdA = FM_INVALID_ID;
        }

        static uint HashFunc(const FmGluedObjectPairKey& key)
        {
            return FmComputeHash(key.objectIdA | key.objectIdB);
        }

        static void InitValue(FmGluedObjectPairElement& elem)
        {
            elem.glueCount = 0;
        }
    };

    typedef FmHashSet<FmGluedObjectPairElement, FmGluedObjectPairSetFuncs> FmGluedObjectPairSet;

    // Description of memory needed for contacts/constraints, broad phase, and constraint islands.
    struct FmConstraintsBuffer
    {
        FmAtomicUint foundContacts;
        FmAtomicUint numDistanceContacts;
        FmAtomicUint numVolumeContacts;
        FmAtomicUint numVolumeContactVerts;
        FmAtomicUint numDeformationConstraints;
        FmAtomicUint exceededVolumeContactVerts;
        FmAtomicUint numBroadPhasePairs;
        FmAtomicUint numRigidBodyBroadPhasePairs;

        size_t bufferNumBytes;

        FmDistanceContactPairInfo*   distanceContactsPairInfo;
        FmDistanceContact*           distanceContacts;
        FmVolumeContact*             volumeContacts;
        FmVolumeContactVert*         volumeContactVerts;
        FmDeformationConstraint*     deformationConstraints;
        FmGlueConstraint*            glueConstraints;
        FmPlaneConstraint*           planeConstraints;
        FmRigidBodyAngleConstraint*  rigidBodyAngleConstraints;

        FmGluedObjectPairSet         gluedObjectPairs;

        FmFreeIds                    freeGlueConstraintIds;
        FmFreeIds                    freePlaneConstraintIds;
        FmFreeIds                    freeRigidBodyAngleConstraintIds;

        FmCCNode*                    islandObjectNodes;

        FmConstraintReference*       allIslandConstraints;          // Buffer for the constraints for all islands
        uint*                        allIslandTetMeshIds;           // Buffer for the tet mesh ids for all islands
        uint*                        allIslandRigidBodyIds;         // Buffer for the rigid body ids for all islands
        uint*                        allUserRigidBodyIslandIndices; // Each FEM island will have an array of rigid body islands; arrays will share this buffer
        FmConstraintIsland*          constraintIslands;             // Arrays of constraints and objects for each island, regenerated each simulation step

        FmSleepingConstraintIsland*  sleepingConstraintIslands;     // Saves island that has been put to sleep; woken after collision or other change to object
        uint*                        allSleepingIslandTetMeshIds;   // Buffer for the tet mesh ids for all sleeping islands
        uint*                        allSleepingIslandRigidBodyIds; // Buffer for the rigid body ids for all sleeping islands
        FmFreeIds                    freeSleepingIslandIds;
        uint*                        sleepingIslandIdToArrayIdx;

        uint                         islandRandomSeed;              // Deterministic seed value for random numbers in constraint island solve

        FmBvh                        broadPhaseHierarchy;
        FmBvh                        sleepingBroadPhaseHierarchy;   // Can use portions of arrays in broadPhaseHierarchy

        FmBroadPhasePair*            broadPhasePairs;
        FmBroadPhasePair*            rigidBodyBroadPhasePairs;      // Pairs containing rigid bodies are output to this list when FmSceneControlParams::rigidBodiesExternal true

        FmVector3                    worldMinPosition;
        FmVector3                    worldMaxPosition;

        FmCollisionGroupPairFlags    collisionGroupPairs;

        FmCallbackConstraintSolveInnerIteration  innerIterationCallback;
        FmCallbackConstraintSolveIslandCompleted islandCompletedCallback;
        void* userData;

        uint numGlueConstraints;
        uint numPlaneConstraints;
        uint numRigidBodyAngleConstraints;
        uint numConstraintIslands;
        uint numUserRigidBodyIslands;

        uint numSleepingConstraintIslands;
        uint numSleepingIslandTetMeshes;
        uint numSleepingIslandRigidBodies;

        uint maxDistanceContacts;
        uint maxVolumeContacts;
        uint maxVolumeContactVerts;
        uint maxDeformationConstraints;
        uint maxGlueConstraints;
        uint maxPlaneConstraints;
        uint maxRigidBodyAngleConstraints;
        uint maxConstraintIslands;
        uint maxBroadPhasePairs;
        uint maxRigidBodyBroadPhasePairs;
    };

    // Parameters needed to allocate constraints data
    struct FmConstraintsBufferSetupParams
    {
        uint maxTetMeshes;             // Limit on total tet meshes
        uint maxRigidBodies;           // Limit on total rigid bodies
        uint maxDistanceContacts;
        uint maxFractureContacts;
        uint maxVolumeContacts;
        uint maxVolumeContactVerts;
        uint maxDeformationConstraints;
        uint maxGlueConstraints;
        uint maxPlaneConstraints;
        uint maxRigidBodyAngleConstraints;
        uint maxBroadPhasePairs;
        uint maxRigidBodyBroadPhasePairs;

        inline FmConstraintsBufferSetupParams()
        {
            maxTetMeshes = 0;
            maxRigidBodies = 0;
            maxDistanceContacts = 0;
            maxFractureContacts = 0;
            maxVolumeContacts = 0;
            maxVolumeContactVerts = 0;
            maxDeformationConstraints = 0;
            maxGlueConstraints = 0;
            maxPlaneConstraints = 0;
            maxRigidBodyAngleConstraints = 0;
            maxBroadPhasePairs = 0;
            maxRigidBodyBroadPhasePairs = 0;
        }
    };

    static inline void FmInitCollisionGroupPairFlags(FmCollisionGroupPairFlags* pairFlags)
    {
        for (uint i = 0; i < 32; i++)
        {
            pairFlags->flags[i] = (1UL << i);
        }
    }

    // Init empty constraints buffer
    static inline void FmInitConstraintsBuffer(FmConstraintsBuffer* constraintsBuffer)
    {
        constraintsBuffer->bufferNumBytes = 0;
        constraintsBuffer->distanceContactsPairInfo = NULL;
        constraintsBuffer->distanceContacts = NULL;
        constraintsBuffer->volumeContacts = NULL;
        constraintsBuffer->volumeContactVerts = NULL;
        constraintsBuffer->deformationConstraints = NULL;
        constraintsBuffer->glueConstraints = NULL;
        constraintsBuffer->planeConstraints = NULL;
        constraintsBuffer->rigidBodyAngleConstraints = NULL;
        constraintsBuffer->gluedObjectPairs.elements = NULL;
        constraintsBuffer->gluedObjectPairs.numElements = 0;
        constraintsBuffer->gluedObjectPairs.maxElements = 0;
        FmInitFreeIds(&constraintsBuffer->freeGlueConstraintIds);
        FmInitFreeIds(&constraintsBuffer->freePlaneConstraintIds);
        FmInitFreeIds(&constraintsBuffer->freeRigidBodyAngleConstraintIds);
        constraintsBuffer->islandObjectNodes = NULL;
        constraintsBuffer->allIslandConstraints = NULL;
        constraintsBuffer->allIslandTetMeshIds = NULL;
        constraintsBuffer->allIslandRigidBodyIds = NULL;
        constraintsBuffer->allUserRigidBodyIslandIndices = NULL;
        constraintsBuffer->sleepingConstraintIslands = NULL;
        constraintsBuffer->allSleepingIslandTetMeshIds = NULL;
        constraintsBuffer->allSleepingIslandRigidBodyIds = NULL;
        FmInitFreeIds(&constraintsBuffer->freeSleepingIslandIds);
        constraintsBuffer->sleepingIslandIdToArrayIdx = NULL;
        constraintsBuffer->constraintIslands = NULL;
        constraintsBuffer->islandRandomSeed = 0;
        FmInitBvh(&constraintsBuffer->broadPhaseHierarchy);
        FmInitBvh(&constraintsBuffer->sleepingBroadPhaseHierarchy);
        constraintsBuffer->broadPhasePairs = NULL;
        constraintsBuffer->rigidBodyBroadPhasePairs = NULL;
        constraintsBuffer->worldMinPosition = FmInitVector3(-1e6f);
        constraintsBuffer->worldMaxPosition = FmInitVector3(1e6f);
        FmInitCollisionGroupPairFlags(&constraintsBuffer->collisionGroupPairs);
        constraintsBuffer->innerIterationCallback = NULL;
        constraintsBuffer->islandCompletedCallback = NULL;
        constraintsBuffer->userData = NULL;
        FmAtomicWrite(&constraintsBuffer->foundContacts.val, 0);
        FmAtomicWrite(&constraintsBuffer->numDistanceContacts.val, 0);
        FmAtomicWrite(&constraintsBuffer->numVolumeContacts.val, 0);
        FmAtomicWrite(&constraintsBuffer->numVolumeContactVerts.val, 0);
        FmAtomicWrite(&constraintsBuffer->numDeformationConstraints.val, 0);
        constraintsBuffer->numGlueConstraints = 0;
        constraintsBuffer->numPlaneConstraints = 0;
        constraintsBuffer->numRigidBodyAngleConstraints = 0;
        constraintsBuffer->numConstraintIslands = 0;
        constraintsBuffer->numBroadPhasePairs = 0;
        constraintsBuffer->numRigidBodyBroadPhasePairs = 0;
        constraintsBuffer->numUserRigidBodyIslands = 0;
        constraintsBuffer->numSleepingConstraintIslands = 0;
        constraintsBuffer->numSleepingIslandTetMeshes = 0;
        constraintsBuffer->numSleepingIslandRigidBodies = 0;
        constraintsBuffer->maxDistanceContacts = 0;
        constraintsBuffer->maxVolumeContacts = 0;
        constraintsBuffer->maxVolumeContactVerts = 0;
        constraintsBuffer->maxDeformationConstraints = 0;
        constraintsBuffer->maxGlueConstraints = 0;
        constraintsBuffer->maxPlaneConstraints = 0;
        constraintsBuffer->maxRigidBodyAngleConstraints = 0;
        constraintsBuffer->maxConstraintIslands = 0;
        constraintsBuffer->maxBroadPhasePairs = 0;
        constraintsBuffer->maxRigidBodyBroadPhasePairs = 0;
    }

    // Get total bytes needed for contact data including FmConstraintsBuffer struct and all arrays.
    size_t FmGetConstraintsBufferSize(const FmConstraintsBufferSetupParams& params);

    // Suballocate memory from pBuffer to create FmConstraintsBuffer and arrays.
    // Assumes pBuffer is 64-byte aligned, and pads it up to 64-bytes after allocations.
    FmConstraintsBuffer* FmSetupConstraintsBuffer(const FmConstraintsBufferSetupParams& params, uint8_t*& pBuffer, size_t bufferNumBytes);

    bool FmAllocCollidedObjectPairTemps(FmCollidedObjectPairTemps* temps, uint8_t*& pBuffer, uint8_t* pBufferEnd,
        uint maxObjVerts, uint numVertsA, uint numVertsB, uint numFacesA, uint numFacesB, bool allocContactReductionWorkspace
#if FM_SURFACE_INTERSECTION_CONTACTS
        , uint maxSurfaceIntersectionPairs
#endif
    );



}