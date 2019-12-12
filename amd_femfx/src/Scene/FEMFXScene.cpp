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
// Initialization of scene, adding and removing objects
//---------------------------------------------------------------------------------------

#include "AMD_FEMFX.h"
#include "FEMFXScene.h"
#include "FEMFXConstraintSolver.h"
#include "FEMFXThreadTempMemory.h"
#include "FEMFXMpcgSolver.h"
#include "FEMFXSleeping.h"

namespace AMD
{

    size_t FmGetSceneSize(const FmSceneSetupParams& params)
    {
        uint maxTetMeshBuffers = params.maxTetMeshBuffers;
        uint maxTetMeshes = FmMaxUint(params.maxTetMeshes, params.maxTetMeshBuffers);  // Need least one tet mesh per buffer
        uint maxRigidBodies = params.maxRigidBodies;

        FmConstraintsBufferSetupParams constraintsSetupParams;
        constraintsSetupParams.maxTetMeshes = maxTetMeshes;
        constraintsSetupParams.maxRigidBodies = maxRigidBodies;
        constraintsSetupParams.maxDistanceContacts = params.maxDistanceContacts;
        constraintsSetupParams.maxVolumeContacts = params.maxVolumeContacts;
        constraintsSetupParams.maxVolumeContactVerts = params.maxVolumeContactVerts;
        constraintsSetupParams.maxDeformationConstraints = params.maxDeformationConstraints;
        constraintsSetupParams.maxGlueConstraints = params.maxGlueConstraints;
        constraintsSetupParams.maxPlaneConstraints = params.maxPlaneConstraints;
        constraintsSetupParams.maxRigidBodyAngleConstraints = params.maxRigidBodyAngleConstraints;
        constraintsSetupParams.maxBroadPhasePairs = params.maxBroadPhasePairs;
        constraintsSetupParams.maxRigidBodyBroadPhasePairs = params.maxRigidBodyBroadPhasePairs;

        FmConstraintSolverBufferSetupParams constraintSolverSetupParams;
        constraintSolverSetupParams.maxTetMeshes = maxTetMeshes;
        constraintSolverSetupParams.maxRigidBodies = maxRigidBodies;
        constraintSolverSetupParams.maxTetMeshVerts = params.maxSceneVerts;
        constraintSolverSetupParams.maxConstraints = params.maxDistanceContacts + params.maxVolumeContacts + params.maxGlueConstraints + params.maxPlaneConstraints + params.maxDeformationConstraints;
        constraintSolverSetupParams.maxConstraintSolverDataSize = params.maxConstraintSolverDataSize;

        FmThreadTempMemoryBufferSetupParams threadTempMemorySetupParams;
        threadTempMemorySetupParams.numWorkerThreads = params.numWorkerThreads;
        threadTempMemorySetupParams.numBytesPerThread = 0; // calculated from below
        threadTempMemorySetupParams.maxTetMeshBufferFeatures = params.maxTetMeshBufferFeatures;

        size_t numBytes =
            FM_PAD_64(sizeof(FmScene)) +
            + FM_PAD_16(sizeof(*FmScene::tetMeshBuffers) * maxTetMeshBuffers)
            + FM_PAD_16(sizeof(*FmScene::awakeTetMeshIds) * maxTetMeshes)
            + FM_PAD_16(sizeof(*FmScene::awakeRigidBodyIds) * maxRigidBodies)
            + FM_PAD_16(sizeof(*FmScene::tetMeshPtrFromId) * maxTetMeshes)
            + FM_PAD_16(sizeof(*FmScene::tetMeshIdxFromId) * maxTetMeshes)
            + FM_PAD_16(sizeof(*FmScene::rigidBodyIdxFromId) * maxRigidBodies)
            + FM_PAD_16(sizeof(*FmScene::freeTetMeshIds.freeIdsArray) * maxTetMeshes)
            + FM_PAD_16(sizeof(*FmScene::freeRigidBodyIds.freeIdsArray) * maxRigidBodies)
            + FM_PAD_16(sizeof(*FmScene::rigidBodies) * maxRigidBodies)
            + FmGetConstraintsBufferSize(constraintsSetupParams)
            + FmGetConstraintSolverBufferSize(constraintSolverSetupParams)
            + FmGetThreadTempMemoryBufferSize(threadTempMemorySetupParams);

        return FM_PAD_64(numBytes);
    }

    FmScene* FmSetupScene(const FmSceneSetupParams& params, uint8_t* pBuffer, size_t bufferNumBytes)
    {
        FM_ASSERT(((uintptr_t)pBuffer & 0x3f) == 0);

        uint maxTetMeshBuffers = params.maxTetMeshBuffers;
        uint maxTetMeshes = FmMaxUint(params.maxTetMeshes, params.maxTetMeshBuffers);  // Need least one tet mesh per buffer
        uint maxRigidBodies = params.maxRigidBodies;

        FmConstraintsBufferSetupParams constraintsSetupParams;
        constraintsSetupParams.maxTetMeshes = maxTetMeshes;
        constraintsSetupParams.maxRigidBodies = maxRigidBodies;
        constraintsSetupParams.maxDistanceContacts = params.maxDistanceContacts;
        constraintsSetupParams.maxVolumeContacts = params.maxVolumeContacts;
        constraintsSetupParams.maxVolumeContactVerts = params.maxVolumeContactVerts;
        constraintsSetupParams.maxDeformationConstraints = params.maxDeformationConstraints;
        constraintsSetupParams.maxGlueConstraints = params.maxGlueConstraints;
        constraintsSetupParams.maxPlaneConstraints = params.maxPlaneConstraints;
        constraintsSetupParams.maxRigidBodyAngleConstraints = params.maxRigidBodyAngleConstraints;
        constraintsSetupParams.maxBroadPhasePairs = params.maxBroadPhasePairs;
        constraintsSetupParams.maxRigidBodyBroadPhasePairs = params.maxRigidBodyBroadPhasePairs;

        FmConstraintSolverBufferSetupParams constraintSolverSetupParams;
        constraintSolverSetupParams.maxTetMeshes = maxTetMeshes;
        constraintSolverSetupParams.maxRigidBodies = maxRigidBodies;
        constraintSolverSetupParams.maxTetMeshVerts = params.maxSceneVerts;
        constraintSolverSetupParams.maxConstraints = params.maxDistanceContacts + params.maxVolumeContacts + params.maxGlueConstraints + params.maxPlaneConstraints + params.maxDeformationConstraints;
        constraintSolverSetupParams.maxConstraintSolverDataSize = params.maxConstraintSolverDataSize;

        FmThreadTempMemoryBufferSetupParams threadTempMemorySetupParams;
        threadTempMemorySetupParams.numWorkerThreads = params.numWorkerThreads;
        threadTempMemorySetupParams.numBytesPerThread = 0; // calculated from below
        threadTempMemorySetupParams.maxTetMeshBufferFeatures = params.maxTetMeshBufferFeatures;

        //uint8_t* pBufferStart = pBuffer;
        uint8_t* pBufferEnd = pBuffer + bufferNumBytes;

        FmScene* pScene = FmAllocFromBuffer64<FmScene>(&pBuffer, 1, pBufferEnd);
        FmInitScene(pScene);

        pScene->bufferNumBytes = bufferNumBytes;

        pScene->constraintsBuffer = FmSetupConstraintsBuffer(constraintsSetupParams, pBuffer, (size_t)(pBufferEnd - pBuffer));
        pScene->constraintSolverBuffer = FmSetupConstraintSolverBuffer(constraintSolverSetupParams, pBuffer, (size_t)(pBufferEnd - pBuffer));
        pScene->threadTempMemoryBuffer = FmSetupThreadTempMemoryBuffer(threadTempMemorySetupParams, pBuffer, (size_t)(pBufferEnd - pBuffer));

        pScene->tetMeshBuffers = FmAllocFromBuffer<FmTetMeshBuffer*>(&pBuffer, maxTetMeshBuffers, pBufferEnd);
        pScene->awakeTetMeshIds = FmAllocFromBuffer<uint>(&pBuffer, maxTetMeshes, pBufferEnd);
        pScene->sleepingTetMeshIds = pScene->awakeTetMeshIds + maxTetMeshes;
        pScene->awakeRigidBodyIds = FmAllocFromBuffer<uint>(&pBuffer, maxRigidBodies, pBufferEnd);
        pScene->sleepingRigidBodyIds = pScene->awakeRigidBodyIds + maxRigidBodies;
        pScene->tetMeshPtrFromId = FmAllocFromBuffer<FmTetMesh*>(&pBuffer, maxTetMeshes, pBufferEnd);
        pScene->tetMeshIdxFromId = FmAllocFromBuffer<uint>(&pBuffer, maxTetMeshes, pBufferEnd);
        pScene->rigidBodyIdxFromId = FmAllocFromBuffer<uint>(&pBuffer, maxRigidBodies, pBufferEnd);
        uint* freeTetMeshIdsArray = FmAllocFromBuffer<uint>(&pBuffer, maxTetMeshes, pBufferEnd);
        uint* freeRigidBodyIdsArray = FmAllocFromBuffer<uint>(&pBuffer, maxRigidBodies, pBufferEnd);
        pScene->rigidBodies = FmAllocFromBuffer<FmRigidBody*>(&pBuffer, maxRigidBodies, pBufferEnd);

        memset(pScene->tetMeshBuffers, 0, sizeof(*pScene->tetMeshBuffers) * maxTetMeshBuffers);
        memset(pScene->awakeTetMeshIds, 0, sizeof(*FmScene::awakeTetMeshIds) * maxTetMeshes);
        memset(pScene->awakeRigidBodyIds, 0, sizeof(*FmScene::awakeRigidBodyIds) * maxRigidBodies);
        memset(pScene->tetMeshPtrFromId, 0, sizeof(*FmScene::tetMeshPtrFromId) * maxTetMeshes);
        memset(pScene->tetMeshIdxFromId, 0, sizeof(*FmScene::tetMeshIdxFromId) * maxTetMeshes);
        memset(pScene->rigidBodyIdxFromId, 0, sizeof(*FmScene::rigidBodyIdxFromId) * maxRigidBodies);

        pScene->maxTetMeshBuffers = maxTetMeshBuffers;
        pScene->maxTetMeshes = maxTetMeshes;
        pScene->maxRigidBodies = maxRigidBodies;
        pScene->maxSceneVerts = params.maxSceneVerts;
        pScene->maxTetMeshBufferFeatures = params.maxTetMeshBufferFeatures;
        pScene->params.rigidBodiesExternal = params.rigidBodiesExternal;

        FmInitFreeIds(&pScene->freeTetMeshIds, freeTetMeshIdsArray, maxTetMeshes);
        FmInitFreeIds(&pScene->freeRigidBodyIds, freeRigidBodyIdsArray, maxRigidBodies, FM_RB_FLAG);

        return pScene;
    }

    FmScene* FmCreateScene(const FmSceneSetupParams& params)
    {
        size_t bufferNumBytes = FmGetSceneSize(params);

        uint8_t* pBuffer = (uint8_t*)FmAlignedMalloc(bufferNumBytes, 64);
        if (pBuffer == NULL)
        {
            return NULL;
        }

        return FmSetupScene(params, pBuffer, bufferNumBytes);
    }

    void FmDestroyScene(FmScene* scene)
    {
        FmAlignedFree(scene);
    }

    size_t FmGetSceneSize(const FmScene& scene)
    {
        return scene.bufferNumBytes;
    }

    void FmGetSceneConstraintSolverDataSize(size_t* numBytes, size_t* highWaterMark, const FmScene& scene)
    {
        *numBytes = scene.constraintSolverBuffer->islandSolverDataArraysNumBytes;
        *highWaterMark = scene.constraintSolverBuffer->islandSolverDataArraysHighWaterMark;
    }

    const FmSceneControlParams& FmGetSceneControlParams(const FmScene& scene)
    {
        return scene.params;
    }

    void FmSetSceneControlParams(FmScene* scene, const FmSceneControlParams& sceneParams)
    {
        scene->params = sceneParams;
    }

    void FmSetSceneTaskSystemCallbacks(FmScene* scene, const FmTaskSystemCallbacks& callbacks)
    {
        scene->taskSystemCallbacks = callbacks;
    }

    void FmSetPostSceneUpdateCallback(FmScene* scene, FmTaskFuncCallback postSceneUpdateCallback, void* postSceneUpdateData)
    {
        scene->postUpdateSceneCallback = postSceneUpdateCallback;
        scene->postUpdateSceneData = postSceneUpdateData;
    }

    void FmSetSurfaceCollisionCallback(FmScene* scene, FmCallbackSurfaceCollision surfaceCollisionCallback)
    {
        scene->surfaceCollisionCallback = surfaceCollisionCallback;
    }

    void FmSetExternalRigidBodiesCallbacks(FmScene* scene, FmCallbackNotifyIslandWaking islandWakingCallback, FmCallbackNotifyIslandSleeping islandSleepingCallback, void* rigidBodiesUserData)
    {
        scene->islandWakingCallback = islandWakingCallback;
        scene->islandSleepingCallback = islandSleepingCallback;
        scene->rigidBodiesUserData = rigidBodiesUserData;
    }

    FmCollisionReport& FmGetSceneCollisionReportRef(FmScene* scene)
    {
        return scene->collisionReport;
    }

    FmFractureReport& FmGetSceneFractureReportRef(FmScene* scene)
    {
        return scene->fractureReport;
    }

    FmWarningsReport& FmGetSceneWarningsReportRef(FmScene* scene)
    {
        return scene->warningsReport;
    }

    void FmRemoveIdFromSceneAwakeTetMeshIds(FmScene* scene, uint objectId)
    {
        // Replace id from the end
        uint dstIdx = FmGetTetMeshIdxById(*scene, objectId);

        if (dstIdx == FM_INVALID_ID) // not in ids list, e.g., simulation disabled
        {
            return;
        }

        FM_ASSERT(scene->awakeTetMeshIds[dstIdx] == objectId);
        uint srcIdx = scene->numAwakeTetMeshes - 1;
        if (dstIdx < srcIdx)
        {
            uint srcId = scene->awakeTetMeshIds[srcIdx];
            scene->awakeTetMeshIds[dstIdx] = srcId;
            FmSetTetMeshIdxById(scene, srcId, dstIdx);
        }
        scene->numAwakeTetMeshes--;
    }

    void FmRemoveIdFromSceneSleepingTetMeshIds(FmScene* scene, uint objectId)
    {
        // Replace id from the end
        uint dstIdx = FmGetTetMeshIdxById(*scene, objectId);

        if (dstIdx == FM_INVALID_ID) // // not in ids list, e.g., simulation disabled
        {
            return;
        }

        uint maxTetMeshes = scene->maxTetMeshes;
        FM_ASSERT(scene->awakeTetMeshIds[dstIdx] == objectId);
        uint srcIdx = maxTetMeshes - scene->numSleepingTetMeshes;
        if (dstIdx > srcIdx)
        {
            uint srcId = scene->awakeTetMeshIds[srcIdx];
            scene->awakeTetMeshIds[dstIdx] = srcId;
            FmSetTetMeshIdxById(scene, srcId, dstIdx);
        }
        scene->numSleepingTetMeshes--;
        scene->sleepingTetMeshIds = scene->awakeTetMeshIds + maxTetMeshes - scene->numSleepingTetMeshes;
    }

    void FmRemoveIdFromSceneAwakeRigidBodyIds(FmScene* scene, uint objectId)
    {
        // Replace id from the end
        uint dstIdx = FmGetRigidBodyIdxById(*scene, objectId);

        if (dstIdx == FM_INVALID_ID) // not in ids list, e.g., simulation disabled
        {
            return;
        }

        FM_ASSERT(scene->awakeRigidBodyIds[dstIdx] == objectId);
        uint srcIdx = scene->numAwakeRigidBodies - 1;
        if (dstIdx < srcIdx)
        {
            uint srcId = scene->awakeRigidBodyIds[srcIdx];
            scene->awakeRigidBodyIds[dstIdx] = srcId;
            FmSetRigidBodyIdxById(scene, srcId, dstIdx);
        }
        scene->numAwakeRigidBodies--;
    }

    void FmRemoveIdFromSceneSleepingRigidBodyIds(FmScene* scene, uint objectId)
    {
        // Replace id from the end
        uint dstIdx = FmGetRigidBodyIdxById(*scene, objectId);

        if (dstIdx == FM_INVALID_ID) // not in ids list, e.g., simulation disabled
        {
            return;
        }

        uint maxRigidBodies = scene->maxRigidBodies;
        FM_ASSERT(scene->awakeRigidBodyIds[dstIdx] == objectId);
        uint srcIdx = maxRigidBodies - scene->numSleepingRigidBodies;
        if (dstIdx > srcIdx)
        {
            uint srcId = scene->awakeRigidBodyIds[srcIdx];
            scene->awakeRigidBodyIds[dstIdx] = srcId;
            FmSetRigidBodyIdxById(scene, srcId, dstIdx);
        }
        scene->numSleepingRigidBodies--;
        scene->sleepingRigidBodyIds = scene->awakeRigidBodyIds + maxRigidBodies - scene->numSleepingRigidBodies;
    }

    void FmAddIdToSceneAwakeTetMeshIds(FmScene* scene, uint objectId)
    {
        FM_ASSERT(scene->numAwakeTetMeshes + scene->numSleepingTetMeshes < scene->maxTetMeshes);

        uint dstIdx = scene->numAwakeTetMeshes;
        scene->awakeTetMeshIds[dstIdx] = objectId;
        FmSetTetMeshIdxById(scene, objectId, dstIdx);

        scene->numAwakeTetMeshes++;
    }

    void FmAddIdToSceneSleepingTetMeshIds(FmScene* scene, uint objectId)
    {
        FM_ASSERT(scene->numAwakeTetMeshes + scene->numSleepingTetMeshes < scene->maxTetMeshes);

        uint maxTetMeshes = scene->maxTetMeshes;
        uint dstIdx = maxTetMeshes - 1 - scene->numSleepingTetMeshes;
        scene->awakeTetMeshIds[dstIdx] = objectId;
        FmSetTetMeshIdxById(scene, objectId, dstIdx);

        scene->numSleepingTetMeshes++;
        scene->sleepingTetMeshIds = scene->awakeTetMeshIds + maxTetMeshes - scene->numSleepingTetMeshes;
    }

    void FmAddIdToSceneAwakeRigidBodyIds(FmScene* scene, uint objectId)
    {
        FM_ASSERT(scene->numAwakeRigidBodies + scene->numSleepingRigidBodies < scene->maxRigidBodies);

        uint dstIdx = scene->numAwakeRigidBodies;
        scene->awakeRigidBodyIds[dstIdx] = objectId;
        FmSetRigidBodyIdxById(scene, objectId, dstIdx);

        scene->numAwakeRigidBodies++;
    }

    void FmAddIdToSceneSleepingRigidBodyIds(FmScene* scene, uint objectId)
    {
        FM_ASSERT(scene->numAwakeRigidBodies + scene->numSleepingRigidBodies < scene->maxRigidBodies);

        uint maxRigidBodies = scene->maxRigidBodies;
        uint dstIdx = maxRigidBodies - 1 - scene->numSleepingRigidBodies;
        scene->awakeRigidBodyIds[dstIdx] = objectId;
        FmSetRigidBodyIdxById(scene, objectId, dstIdx);

        scene->numSleepingRigidBodies++;
        scene->sleepingRigidBodyIds = scene->awakeRigidBodyIds + maxRigidBodies - scene->numSleepingRigidBodies;
    }

    void FmRemoveTetMeshIdFromIsland(FmScene* scene, uint objectId)
    {
        FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, objectId);

        if (tetMesh.islandId == FM_INVALID_ID)
        {
            return;
        }

        // Remove id from island
        if (FM_IS_SET(tetMesh.flags, FM_OBJECT_FLAG_SLEEPING))
        {
            bool foundInIsland = false;
            FmSleepingConstraintIsland& constraintIsland = *FmGetSleepingConstraintIslandById(*scene, tetMesh.islandId);
            for (uint islandMeshIdx = 0; islandMeshIdx < constraintIsland.numTetMeshes; islandMeshIdx++)
            {
                if (constraintIsland.tetMeshIds[islandMeshIdx] == objectId)
                {
                    // Replace id from the end
                    constraintIsland.tetMeshIds[islandMeshIdx] = constraintIsland.tetMeshIds[constraintIsland.numTetMeshes - 1];
                    foundInIsland = true;
                    break;
                }
            }
            if (foundInIsland)
            {
                constraintIsland.numTetMeshes--;
            }
            else
            {
                FM_PRINT(("ERROR in FmRemoveTetMeshIdFromIsland: tet mesh not found in its island\n"));
            }
        }
        else
        {
            bool foundInIsland = false;
            FmConstraintIsland& constraintIsland = *FmGetConstraintIslandById(*scene, tetMesh.islandId);
            for (uint islandMeshIdx = 0; islandMeshIdx < constraintIsland.numTetMeshes; islandMeshIdx++)
            {
                if (constraintIsland.tetMeshIds[islandMeshIdx] == objectId)
                {
                    // Replace id from the end
                    constraintIsland.tetMeshIds[islandMeshIdx] = constraintIsland.tetMeshIds[constraintIsland.numTetMeshes - 1];
                    foundInIsland = true;
                    break;
                }
            }
            if (foundInIsland)
            {
                constraintIsland.numTetMeshes--;
            }
            else
            {
                FM_PRINT(("ERROR in FmRemoveTetMeshIdFromIsland: tet mesh not found in its island\n"));
            }
        }
    }

    void FmRemoveRigidBodyIdFromIsland(FmScene* scene, uint rigidBodyId)
    {
        FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, rigidBodyId);

        if (rigidBody.femIslandId == FM_INVALID_ID)
        {
            return;
        }

        // Remove id from active or sleeping ids list
        if (FM_IS_SET(rigidBody.flags, FM_OBJECT_FLAG_SLEEPING))
        {
            bool foundInIsland = false;
            FmSleepingConstraintIsland& constraintIsland = *FmGetSleepingConstraintIslandById(*scene, rigidBody.femIslandId);
            for (uint islandRbIdx = 0; islandRbIdx < constraintIsland.numRigidBodiesConnected; islandRbIdx++)
            {
                if (constraintIsland.rigidBodyIds[islandRbIdx] == rigidBodyId)
                {
                    // Replace id from the end
                    constraintIsland.rigidBodyIds[islandRbIdx] = constraintIsland.rigidBodyIds[constraintIsland.numRigidBodiesConnected - 1];
                    foundInIsland = true;
                    break;
                }
            }
            if (foundInIsland)
            {
                constraintIsland.numRigidBodiesConnected--;
            }
            else
            {
                FM_PRINT(("ERROR in FmRemoveRigidBodyIdFromIsland: rigid body not found in its sleeping island\n"));
            }
        }
        else
        {
            bool foundInIsland = false;
            FmConstraintIsland& constraintIsland = *FmGetConstraintIslandById(*scene, rigidBody.femIslandId);
            for (uint islandRbIdx = 0; islandRbIdx < constraintIsland.numRigidBodiesConnected; islandRbIdx++)
            {
                if (constraintIsland.rigidBodyIds[islandRbIdx] == rigidBodyId)
                {
                    // Replace id from the end
                    constraintIsland.rigidBodyIds[islandRbIdx] = constraintIsland.rigidBodyIds[constraintIsland.numRigidBodiesConnected - 1];
                    foundInIsland = true;
                    break;
                }
            }
            if (foundInIsland)
            {
                constraintIsland.numRigidBodiesConnected--;
            }
            else
            {
                FM_PRINT(("ERROR in FmRemoveRigidBodyIdFromIsland: rigid body not found in its island\n"));
            }
        }
    }

    // Reset indices or flags that are invalid outside of a scene
    void FmResetTetMeshBufferSceneState(FmTetMeshBuffer* tetMeshBuffer)
    {
        tetMeshBuffer->bufferId = FM_INVALID_ID;

        uint numTetMeshes = tetMeshBuffer->numTetMeshes;
        for (uint meshIdx = 0; meshIdx < numTetMeshes; meshIdx++)
        {
            FmTetMesh& tetMesh = tetMeshBuffer->tetMeshes[meshIdx];

            tetMesh.bufferId = FM_INVALID_ID;
            tetMesh.objectId = FM_INVALID_ID;
            tetMesh.islandId = FM_INVALID_ID;

            // Remove sleeping and disabled flag, which require consistent scene state
            tetMesh.flags &= ~(FM_OBJECT_FLAG_SLEEPING | FM_OBJECT_FLAG_SIMULATION_DISABLED);

            tetMesh.solverData->solverStateOffset = FM_INVALID_ID;
        }
    }

    uint FmAddTetMeshBufferToScene(FmScene* scene, FmTetMeshBuffer* tetMeshBuffer)
    {
        uint maxMeshBuffers = scene->maxTetMeshBuffers;

        if (scene->numTetMeshBufferSlots >= scene->maxTetMeshBuffers
            || scene->numTetMeshesTotal >= scene->maxTetMeshes)
        {
            return FM_INVALID_ID;
        }

        for (uint meshBufferIdx = 0; meshBufferIdx < maxMeshBuffers; meshBufferIdx++)
        {
            if (scene->tetMeshBuffers[meshBufferIdx] == NULL)
            {
                // Save tet mesh buffer to empty slot
                scene->tetMeshBuffers[meshBufferIdx] = tetMeshBuffer;

                // Reset flags and scene indices
                FmResetTetMeshBufferSceneState(tetMeshBuffer);

                // Increase numTetMeshBufferSlots if exceeded
                if (meshBufferIdx >= scene->numTetMeshBufferSlots)
                {
                    scene->numTetMeshBufferSlots = meshBufferIdx + 1;
                }

                // Store buffer id
                tetMeshBuffer->bufferId = meshBufferIdx;
                uint maxTetMeshes = tetMeshBuffer->maxTetMeshes;
                for (uint i = 0; i < maxTetMeshes; i++)
                {
                    tetMeshBuffer->tetMeshes[i].bufferId = meshBufferIdx;
                }

                // Add tet meshes to total
                scene->numTetMeshesTotal += tetMeshBuffer->numTetMeshes;

                uint numTetMeshes = tetMeshBuffer->numTetMeshes;
                for (uint meshIdx = 0; meshIdx < numTetMeshes; meshIdx++)
                {
                    // Assign object id to tet mesh
                    FmTetMesh& tetMesh = tetMeshBuffer->tetMeshes[meshIdx];
                    uint objectId = FmReserveId(&scene->freeTetMeshIds);
                    tetMesh.objectId = objectId;

                    // Add sub-mesh pointer to map
                    FmSetTetMeshPtrById(scene, objectId, &tetMesh);

                    FmAddIdToSceneAwakeTetMeshIds(scene, objectId);

                    FM_ASSERT(tetMesh.islandId == FM_INVALID_ID);

#if FM_DEBUG_CHECKS
                    float maxAspectRatio = 0.0f;
                    float avgAspectRatio = 0.0f;
                    uint numTets = tetMesh.numTets;
                    for (uint tetId = 0; tetId < numTets; tetId++)
                    {
                        FmTetVertIds tetVertIds = tetMesh.tetsVertIds[tetId];

                        FmVector3 tetRestPositions[4];
                        tetRestPositions[0] = tetMesh.vertsRestPos[tetVertIds.ids[0]];
                        tetRestPositions[1] = tetMesh.vertsRestPos[tetVertIds.ids[1]];
                        tetRestPositions[2] = tetMesh.vertsRestPos[tetVertIds.ids[2]];
                        tetRestPositions[3] = tetMesh.vertsRestPos[tetVertIds.ids[3]];

                        float aspectRatio = FmComputeTetAspectRatio(tetRestPositions);

                        avgAspectRatio += aspectRatio;
                        if (aspectRatio > maxAspectRatio)
                        {
                            maxAspectRatio = aspectRatio;
                        }

                        if (FmIsTetInverted(tetRestPositions))
                        {
                            FM_PRINT(("tetmesh buffer id: %u has inverted tet %u\n", tetMesh.bufferId, tetId));
                        }
                    }
                    avgAspectRatio /= (float)numTets;

                    FM_PRINT(("tetmesh buffer %u mesh %u: objectId: %u num verts: %u num tets: %u max tet aspect ratio: %f avg: %f\n", 
                        tetMesh.bufferId, meshIdx, tetMesh.objectId,
                        tetMesh.numVerts, tetMesh.numTets, maxAspectRatio, avgAspectRatio));

                    FmValidateMesh(tetMesh);
#endif
                }

                return meshBufferIdx;
            }
        }

        return FM_INVALID_ID;
    }

    void FmRemoveTetMeshBufferFromScene(FmScene* scene, uint tetMeshBufferId)
    {
        if (tetMeshBufferId >= scene->numTetMeshBufferSlots)
        {
            return;
        }

        FmTetMeshBuffer* tetMeshBuffer = scene->tetMeshBuffers[tetMeshBufferId];

        if (!tetMeshBuffer)
        {
            return;
        }

        // Subtract tet meshes from total
        scene->numTetMeshesTotal -= tetMeshBuffer->numTetMeshes;

        // For all tet meshes, release object ids and wake its island if sleeping
        uint numTetMeshes = tetMeshBuffer->numTetMeshes;
        for (uint meshIdx = 0; meshIdx < numTetMeshes; meshIdx++)
        {
            FmTetMesh& tetMesh = tetMeshBuffer->tetMeshes[meshIdx];
            uint objectId = tetMesh.objectId;

            // If part of sleeping island, mark for waking
            FmMarkIslandOfObjectForWaking(scene, objectId);

            // Remove id from active or sleeping ids list
            if (FM_IS_SET(tetMesh.flags, FM_OBJECT_FLAG_SLEEPING))
            {
                FmRemoveIdFromSceneSleepingTetMeshIds(scene, objectId);
            }
            else
            {
                FmRemoveIdFromSceneAwakeTetMeshIds(scene, objectId);
            }

            // Remove id from object's island
            FmRemoveTetMeshIdFromIsland(scene, objectId);

            // Release id and remove tet mesh pointer
            FmSetTetMeshPtrById(scene, objectId, NULL);

            FmReleaseId(&scene->freeTetMeshIds, objectId);
        }

        // Reset flags and scene indices
        FmResetTetMeshBufferSceneState(tetMeshBuffer);

        // Set mesh buffer pointer to NULL to remove from scene
        scene->tetMeshBuffers[tetMeshBufferId] = NULL;

        // Scan back to first non-NULL pointer to reduce number of tet mesh buffers
        uint numTetMeshBufferSlots = scene->numTetMeshBufferSlots;
        for (int meshBufferIdx = (int)numTetMeshBufferSlots - 1; meshBufferIdx >= 0; meshBufferIdx--)
        {
            if (scene->tetMeshBuffers[meshBufferIdx])
            {
                break;
            }
            else
            {
                numTetMeshBufferSlots--;
            }
        }
        scene->numTetMeshBufferSlots = numTetMeshBufferSlots;
    }

    void FmDisableObjectSimulation(FmScene* scene, uint objectId)
    {
        // If part of sleeping island, mark for waking
        FmMarkIslandOfObjectForWaking(scene, objectId);

        if (objectId & FM_RB_FLAG)
        {
            FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, objectId);

            // Remove id from active or sleeping ids list
            if (FM_IS_SET(rigidBody.flags, FM_OBJECT_FLAG_SLEEPING))
            {
                FmRemoveIdFromSceneSleepingRigidBodyIds(scene, objectId);
            }
            else
            {
                FmRemoveIdFromSceneAwakeRigidBodyIds(scene, objectId);
            }

            // Remove id from object's island
            FmRemoveRigidBodyIdFromIsland(scene, objectId);

            // Invalidate index in ids list
            FmSetRigidBodyIdxById(scene, objectId, FM_INVALID_ID);

            // Invalidate island id
            rigidBody.femIslandId = FM_INVALID_ID;

            // Mark as sim disabled
            rigidBody.flags |= FM_OBJECT_FLAG_SIMULATION_DISABLED;

            // Unset sleeping flag
            rigidBody.flags &= ~FM_OBJECT_FLAG_SLEEPING;
        }
        else
        {
            FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, objectId);

            // Remove id from active or sleeping ids list
            if (FM_IS_SET(tetMesh.flags, FM_OBJECT_FLAG_SLEEPING))
            {
                FmRemoveIdFromSceneSleepingTetMeshIds(scene, objectId);
            }
            else
            {
                FmRemoveIdFromSceneAwakeTetMeshIds(scene, objectId);
            }

            // Remove id from object's island
            FmRemoveTetMeshIdFromIsland(scene, objectId);

            // Invalidate index in ids list
            FmSetTetMeshIdxById(scene, objectId, FM_INVALID_ID);

            // Invalidate island id
            tetMesh.islandId = FM_INVALID_ID;

            // Mark as sim disabled
            tetMesh.flags |= FM_OBJECT_FLAG_SIMULATION_DISABLED;

            // Unset sleeping flag
            tetMesh.flags &= ~FM_OBJECT_FLAG_SLEEPING;
        }
    }

    void FmEnableSimulation(FmScene* scene, FmTetMesh* tetMesh, bool isEnabled)
    {
        if (!tetMesh)
        {
            return;
        }

        if (isEnabled)
        {
            tetMesh->flags &= ~FM_OBJECT_FLAG_SIMULATION_DISABLED;
        }
        else
        {
            FmDisableObjectSimulation(scene, tetMesh->objectId);
        }
    }

    void FmEnableSimulation(FmScene* scene, FmRigidBody* rigidBody, bool isEnabled)
    {
        if (!rigidBody)
        {
            return;
        }

        if (isEnabled)
        {
            rigidBody->flags &= ~FM_OBJECT_FLAG_SIMULATION_DISABLED;
        }
        else
        {
            FmDisableObjectSimulation(scene, rigidBody->objectId);
        }
    }

    void FmEnableSleeping(FmScene* scene, FmTetMesh* tetMesh, bool isEnabled)
    {
        if (!tetMesh)
        {
            return;
        }

        if (isEnabled)
        {
            tetMesh->flags &= ~FM_OBJECT_FLAG_SLEEPING_DISABLED;
        }
        else
        {
            tetMesh->flags |= FM_OBJECT_FLAG_SLEEPING_DISABLED;

            if (scene)
            {
                // If part of sleeping island, mark for waking
                FmMarkIslandOfObjectForWaking(scene, tetMesh->objectId);
            }
        }
    }

    void FmEnableSleeping(FmScene* scene, FmRigidBody* rigidBody, bool isEnabled)
    {
        if (!rigidBody)
        {
            return;
        }

        if (isEnabled)
        {
            rigidBody->flags &= ~FM_OBJECT_FLAG_SLEEPING_DISABLED;
        }
        else
        {
            rigidBody->flags |= FM_OBJECT_FLAG_SLEEPING_DISABLED;

            if (scene)
            {
                // If part of sleeping island, mark for waking
                FmMarkIslandOfObjectForWaking(scene, rigidBody->objectId);
            }
        }
    }

    void FmEnableStrainMagComputation(FmTetMesh* tetMesh, bool isEnabled)
    {
        if (!tetMesh)
        {
            return;
        }

        if (isEnabled)
        {
            tetMesh->flags |= FM_OBJECT_FLAG_COMPUTE_TET_STRAIN_MAG;
        }
        else
        {
            tetMesh->flags &= ~FM_OBJECT_FLAG_COMPUTE_TET_STRAIN_MAG;
        }
    }

    void FmEnableSelfCollision(FmTetMesh* tetMesh, bool isEnabled)
    {
        if (!tetMesh)
        {
            return;
        }

        if (isEnabled)
        {
            tetMesh->flags |= FM_OBJECT_FLAG_ENABLE_SELF_COLLISION;
        }
        else
        {
            tetMesh->flags &= ~FM_OBJECT_FLAG_ENABLE_SELF_COLLISION;
        }
    }

    FmTetMeshBuffer* FmGetTetMeshBuffer(const FmScene& scene, uint tetMeshBufferId)
    {
        if (tetMeshBufferId >= scene.maxTetMeshBuffers)
        {
            return NULL;
        }
        return scene.tetMeshBuffers[tetMeshBufferId];
    }

    FmTetMesh* FmGetTetMesh(const FmScene& scene, uint objectId)
    {
        if (scene.tetMeshBuffers == NULL || objectId >= scene.maxTetMeshes)
        {
            return NULL;
        }

        return FmGetTetMeshPtrById(scene, objectId);
    }

    // Reset indices or flags that are invalid outside of a scene
    void FmResetRigidBodySceneState(FmRigidBody* rigidBody)
    {
        rigidBody->objectId = FM_INVALID_ID;
        rigidBody->rbIslandId = FM_INVALID_ID;
        rigidBody->femIslandId = FM_INVALID_ID;
        rigidBody->foundInConstraint = false;

        // Remove sleeping and disabled flag, which require consistent scene state
        rigidBody->flags &= ~(FM_OBJECT_FLAG_SLEEPING | FM_OBJECT_FLAG_SIMULATION_DISABLED);
    }

    uint FmAddRigidBodyToScene(FmScene* scene, FmRigidBody* inRigidBody)
    {
        if (scene->freeRigidBodyIds.numFreeIds == 0)
        {
            return FM_INVALID_ID;
        }

        uint rbId = FmReserveId(&scene->freeRigidBodyIds);
        uint rbIdx = rbId & ~FM_RB_FLAG;

        if (rbIdx >= scene->numRigidBodySlots)
        {
            scene->numRigidBodySlots = rbIdx + 1;
        }

        scene->rigidBodies[rbIdx] = inRigidBody;

        FmRigidBody& rigidBody = *inRigidBody;

        FmResetRigidBodySceneState(&rigidBody);

        FM_ASSERT(rigidBody.femIslandId == FM_INVALID_ID);
        FM_ASSERT(rigidBody.rbIslandId == FM_INVALID_ID);

        rigidBody.objectId = rbId;

        FM_ASSERT(rigidBody.collisionObj);
        rigidBody.collisionObj->tetMeshes[0].objectId = rbId;

        FmAddIdToSceneAwakeRigidBodyIds(scene, rbId);

        return rbId;
    }

    void FmRemoveRigidBodyFromScene(FmScene* scene, uint rigidBodyId)
    {
        uint rbIdx = rigidBodyId & ~FM_RB_FLAG;
        if (rbIdx >= scene->numRigidBodySlots)
        {
            return;
        }

        FmRigidBody* pRigidBody = FmGetRigidBodyPtrById(*scene, rigidBodyId);

        if (!pRigidBody)
        {
            return;
        }

        FmRigidBody& rigidBody = *pRigidBody;

        // If part of sleeping island, mark for waking
        FmMarkIslandOfObjectForWaking(scene, rigidBodyId);

        // Remove id from active or sleeping ids list
        if (FM_IS_SET(rigidBody.flags, FM_OBJECT_FLAG_SLEEPING))
        {
            FmRemoveIdFromSceneSleepingRigidBodyIds(scene, rigidBodyId);
        }
        else
        {
            FmRemoveIdFromSceneAwakeRigidBodyIds(scene, rigidBodyId);
        }

        // Remove id from object's island
        FmRemoveRigidBodyIdFromIsland(scene, rigidBodyId);
    
        // Release id
        FmReleaseId(&scene->freeRigidBodyIds, rigidBodyId);

        // Reset and set pointer NULL
        FmResetRigidBodySceneState(&rigidBody);

        // Set mesh buffer pointer to NULL to remove from scene
        scene->rigidBodies[rbIdx] = NULL;

        // Scan back to first non-NULL pointer to reduce number of rigid body slots
        uint numRigidBodySlots = scene->numRigidBodySlots;
        for (int rbsIdx = (int)numRigidBodySlots - 1; rbsIdx >= 0; rbsIdx--)
        {
            if (scene->rigidBodies[rbsIdx])
            {
                break;
            }
            else
            {
                numRigidBodySlots--;
            }
        }
        scene->numRigidBodySlots = numRigidBodySlots;
    }

    FmRigidBody* FmGetRigidBody(const FmScene& scene, uint rigidBodyId)
    {
        uint rigidBodyIdx = rigidBodyId & ~FM_RB_FLAG;

        if (rigidBodyIdx >= scene.numRigidBodySlots)
        {
            return NULL;
        }

        return scene.rigidBodies[rigidBodyIdx];
    }

    uint FmGetNumEnabledTetMeshes(const FmScene& scene)
    {
        return scene.numAwakeTetMeshes + scene.numSleepingTetMeshes;
    }

    uint FmGetEnabledTetMeshId(const FmScene& scene, uint enabledIdx)
    {
        if (enabledIdx < scene.numAwakeTetMeshes)
        {
            return scene.awakeTetMeshIds[enabledIdx];
        }
        else if (enabledIdx < scene.numAwakeTetMeshes + scene.numSleepingTetMeshes)
        {
            return scene.sleepingTetMeshIds[enabledIdx - scene.numAwakeTetMeshes];
        }
        else
        {
            return FM_INVALID_ID;
        }
    }

    uint FmGetNumEnabledRigidBodies(const FmScene& scene)
    {
        return scene.numAwakeRigidBodies + scene.numSleepingRigidBodies;
    }

    uint FmGetEnabledRigidBodyId(const FmScene& scene, uint enabledIdx)
    {
        if (enabledIdx < scene.numAwakeRigidBodies)
        {
            return scene.awakeRigidBodyIds[enabledIdx];
        }
        else if (enabledIdx < scene.numAwakeRigidBodies + scene.numSleepingRigidBodies)
        {
            return scene.sleepingRigidBodyIds[enabledIdx - scene.numAwakeRigidBodies];
        }
        else
        {
            return FM_INVALID_ID;
        }
    }

}