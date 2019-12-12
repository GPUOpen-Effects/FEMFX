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
// Operations to detect object sleeping and move objects and islands in and out of 
// sleeping state
//---------------------------------------------------------------------------------------

#include "FEMFXMpcgSolverSetup.h"
#include "FEMFXParallelFor.h"
#include "FEMFXScene.h"
#include "FEMFXUpdateTetState.h"

namespace AMD
{
    class FmTaskDataPrepIslandTetMeshesForSleep : public FmAsyncTaskData
    {
    public:
        FM_CLASS_NEW_DELETE(FmTaskDataPrepIslandTetMeshesForSleep)

        FmScene* scene;
        uint* tetMeshIds;
        uint sleepingIslandId;

        FmTaskDataPrepIslandTetMeshesForSleep(FmScene* inScene, uint* inTetMeshIds, uint inSleepingIslandId)
        {
            scene = inScene;
            tetMeshIds = inTetMeshIds;
            sleepingIslandId = inSleepingIslandId;
        }
    };

    FM_WRAPPED_TASK_FUNC(FmTaskFuncPrepIslandTetMeshesForSleep)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT(MESH_STEP_VELOCITY_REBUILD_BVH);

        FmTaskDataPrepIslandTetMeshesForSleep* taskData = (FmTaskDataPrepIslandTetMeshesForSleep*)inTaskData;
        FmScene* scene = taskData->scene;
        uint* tetMeshIds = taskData->tetMeshIds;
        uint sleepingIslandId = taskData->sleepingIslandId;

        float timestep = scene->params.timestep;
        float aabbPadding = scene->params.distContactThreshold * 0.5f;
        float rayleighMassDamping = scene->params.kRayleighMassDamping;
        float rayleighStiffnessDamping = scene->params.kRayleighStiffnessDamping;

        uint beginIdx = taskData->progress.GetNextIndex();
        uint endIdx = beginIdx + 1;

        for (uint i = beginIdx; i < endIdx; i++)
        {
            uint tetMeshId = tetMeshIds[i];
            FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, tetMeshId);
            tetMesh.islandId = sleepingIslandId;
            FM_ASSERT(FM_NOT_SET(tetMesh.flags, FM_OBJECT_FLAG_SLEEPING));
            tetMesh.flags |= FM_OBJECT_FLAG_SLEEPING;

            // Reset stats used for sleeping
            FmInitVelStats(&tetMesh.velStats);

            // Set velocities to zero
            uint numVerts = tetMesh.numVerts;
            for (uint vIdx = 0; vIdx < numVerts; vIdx++)
            {
                tetMesh.vertsVel[vIdx] = FmInitVector3(0.0f);
                FmResetForceOnVert(&tetMesh, vIdx);
            }

            FmSetupMpcgSolve(NULL, FmGetSolverDataById(*scene, tetMeshId), &tetMesh, FmInitVector3(0.0f), rayleighMassDamping, rayleighStiffnessDamping, 0.0f, timestep);

            FmBuildHierarchy(&tetMesh, timestep, aabbPadding);

            FmUpdateTetStateAndFracture(scene, &tetMesh, false, false);
        }

        if (taskData)
        {
            if (taskData->parentTaskData)
            {
                // Parent task data is tracking completion of tet meshes of all islands, and deleted in follow up task
                taskData->parentTaskData->progress.TaskIsFinished();
            }

            // Update progress of this island
            taskData->progress.TaskIsFinished(taskData);
        }
    }


    // Create sleeping constraint island with objects from given island, assumed to all be awake.
    // Copies object ids to a second set of arrays for sleeping objects.
    // Moves objects ids from awake to sleeping ids lists.
    // Assumed to be called just after objects have just been added (between simulation steps) or after constraint solving.
    bool FmPutConstraintIslandToSleep(FmScene* scene, const FmConstraintIsland& srcIsland, bool useCallback, FmAsyncTaskData* parentTaskData)
    {
        FmConstraintsBuffer& constraintsBuffer = *scene->constraintsBuffer;
        uint maxTetMeshes = scene->maxTetMeshes;
        uint maxRigidBodies = scene->maxRigidBodies;
        FM_ASSERT(constraintsBuffer.numSleepingConstraintIslands < constraintsBuffer.maxConstraintIslands);

        // Create island and copy objects
        uint numIslandTetMeshes = srcIsland.numTetMeshes;
        uint numIslandRigidBodies = srcIsland.numRigidBodiesConnected;

        if (constraintsBuffer.numSleepingIslandTetMeshes + numIslandTetMeshes > maxTetMeshes
            || constraintsBuffer.numSleepingIslandRigidBodies + numIslandRigidBodies > maxRigidBodies)
        {
            return false;
        }

        // Pick slot for sleeping island and id which will be used to reference it.
        uint sleepingIslandIdx = constraintsBuffer.numSleepingConstraintIslands;
        constraintsBuffer.numSleepingConstraintIslands++;
        uint sleepingIslandId = FmReserveId(&constraintsBuffer.freeSleepingIslandIds);
        constraintsBuffer.sleepingIslandIdToArrayIdx[sleepingIslandId] = sleepingIslandIdx;

        FmSleepingConstraintIsland& sleepingIsland = constraintsBuffer.sleepingConstraintIslands[sleepingIslandIdx];

        // Move object ids into allSleepingIslandTetMeshIds and allSleepingIslandRigidBodyIds
        uint* sleepingTetMeshIds = constraintsBuffer.allSleepingIslandTetMeshIds + constraintsBuffer.numSleepingIslandTetMeshes;
        uint* sleepingRigidBodyIds = constraintsBuffer.allSleepingIslandRigidBodyIds + constraintsBuffer.numSleepingIslandRigidBodies;

        constraintsBuffer.numSleepingIslandTetMeshes += numIslandTetMeshes;
        constraintsBuffer.numSleepingIslandRigidBodies += numIslandRigidBodies;

        for (uint idx = 0; idx < numIslandTetMeshes; idx++)
        {
            sleepingTetMeshIds[idx] = srcIsland.tetMeshIds[idx];
        }
        for (uint idx = 0; idx < numIslandRigidBodies; idx++)
        {
            sleepingRigidBodyIds[idx] = srcIsland.rigidBodyIds[idx];
        }

        // Initialize island with just id and object arrays
        sleepingIsland.islandId = sleepingIslandId;
        sleepingIsland.flags = srcIsland.flags;
        sleepingIsland.flags |= FM_ISLAND_FLAG_SLEEPING;
        sleepingIsland.flags &= ~FM_ISLAND_FLAG_MARKED_FOR_SLEEPING;
        sleepingIsland.tetMeshIds = sleepingTetMeshIds;
        sleepingIsland.numTetMeshes = srcIsland.numTetMeshes;
        sleepingIsland.rigidBodyIds = sleepingRigidBodyIds;
        sleepingIsland.numRigidBodiesConnected = srcIsland.numRigidBodiesConnected;

        // Move active ids to sleeping ids list
        for (uint idx = 0; idx < sleepingIsland.numTetMeshes; idx++)
        {
            uint tetMeshId = sleepingIsland.tetMeshIds[idx];

            // Remove id from tetMeshIds
            FmRemoveIdFromSceneAwakeTetMeshIds(scene, tetMeshId);

            // Add to sleeping ids list
            FmAddIdToSceneSleepingTetMeshIds(scene, tetMeshId);
        }

        // Multithread remaining steps for tet meshes
#if FM_ASYNC_THREADING
        if (parentTaskData)
        {
            if (numIslandTetMeshes > 0)
            {
                FmTaskDataPrepIslandTetMeshesForSleep* taskData = new FmTaskDataPrepIslandTetMeshesForSleep(scene, sleepingTetMeshIds, sleepingIslandId);

                taskData->progress.Init(numIslandTetMeshes, NULL, NULL); // No follow task after processing island; may call follow task for parent after all islands completed
                taskData->parentTaskData = parentTaskData;

                // Add tasks for this parallel for to the total
                parentTaskData->progress.TasksAreStarting(numIslandTetMeshes);

                // runLoop must be true since execution not returning to loop before other async calls
                FmParallelForAsync("PrepIslandTetMeshesForSleep", FM_TASK_AND_WRAPPED_TASK_ARGS(FmTaskFuncPrepIslandTetMeshesForSleep), NULL, taskData, numIslandTetMeshes, scene->taskSystemCallbacks.SubmitAsyncTask, scene->params.numThreads, true);
            }
        }
        else
        {
            // Using a loop to not require a blocking parallel-for implementation.  This may be reached from FmCreateSleepingIsland()
            FmTaskDataPrepIslandTetMeshesForSleep taskData(scene, sleepingTetMeshIds, sleepingIslandId);
            for (uint i = 0; i < numIslandTetMeshes; i++)
            {
                FmTaskFuncPrepIslandTetMeshesForSleep(&taskData, i, i + 1);
            }
        }
#else
        FmTaskDataPrepIslandTetMeshesForSleep taskData(scene, sleepingTetMeshIds, sleepingIslandId);
        scene->taskSystemCallbacks.ParallelFor("PrepIslandTetMeshesForSleep", FmTaskFuncPrepIslandTetMeshesForSleep, &taskData, numIslandTetMeshes);
#endif

        for (uint idx = 0; idx < numIslandRigidBodies; idx++)
        {
            uint rigidBodyId = sleepingRigidBodyIds[idx];

            // Remove id from rigidBodyIds
            FmRemoveIdFromSceneAwakeRigidBodyIds(scene, rigidBodyId);

            // Add to sleeping ids list
            FmAddIdToSceneSleepingRigidBodyIds(scene, rigidBodyId);

            FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, rigidBodyId);
            rigidBody.femIslandId = sleepingIslandId;
            FM_ASSERT(FM_NOT_SET(rigidBody.flags, FM_OBJECT_FLAG_SLEEPING));
            rigidBody.flags |= FM_OBJECT_FLAG_SLEEPING;

            // Reset stats used for sleeping
            FmInitVelStats(&rigidBody.velStats);

            // Set velocities to zero
            rigidBody.state.vel = FmInitVector3(0.0f);
            rigidBody.state.angVel = FmInitVector3(0.0f);
            rigidBody.aabb.vmin = FmInitVector3(0.0f);
            rigidBody.aabb.vmax = FmInitVector3(0.0f);
        }

        if (useCallback && scene->islandSleepingCallback && sleepingIsland.numRigidBodiesConnected > 0)
        {
            scene->islandSleepingCallback(scene, scene->rigidBodiesUserData, sleepingIsland.rigidBodyIds, sleepingIsland.numRigidBodiesConnected);
        }

        return true;
    }

    bool FmCreateSleepingIsland(FmScene* scene, uint* tetMeshIds, uint numTetMeshes, uint* rigidBodyIds, uint numRigidBodies)
    {
        FmConstraintsBuffer& constraintsBuffer = *scene->constraintsBuffer;
        uint numSleepingIslands = constraintsBuffer.numSleepingConstraintIslands;
        uint maxIslands = constraintsBuffer.maxConstraintIslands;

        FM_ASSERT(numSleepingIslands < maxIslands);
        if (numSleepingIslands >= maxIslands)
        {
            return false;
        }

        // Create temporary island to add to sleeping islands
        FmConstraintIsland island;
        FmInitConstraintIsland(&island);
        island.tetMeshIds = tetMeshIds;
        island.rigidBodyIds = rigidBodyIds;
        island.numTetMeshes = numTetMeshes;
        island.numRigidBodiesConnected = numRigidBodies;

        // Filter out invalid or awake objects
        uint numTetMeshesOutput = 0;
        uint numRigidBodiesOutput = 0;

        for (uint meshIdx = 0; meshIdx < numTetMeshes; meshIdx++)
        {
            FmTetMesh* tetMesh = FmGetTetMesh(*scene, island.tetMeshIds[meshIdx]);

            // Restrict island ids to valid tet meshes 
            if (tetMesh && !FM_IS_SET(tetMesh->flags, FM_OBJECT_FLAG_SLEEPING))
            {
                island.tetMeshIds[numTetMeshesOutput] = island.tetMeshIds[meshIdx];
                numTetMeshesOutput++;

                // Remove id from object's current island
                FmRemoveTetMeshIdFromIsland(scene, tetMesh->objectId);
            }
        }

        for (uint rbIdx = 0; rbIdx < numRigidBodies; rbIdx++)
        {
            FmRigidBody* rigidBody = FmGetRigidBody(*scene, island.rigidBodyIds[rbIdx]);

            // Restrict island ids to valid rigid bodies
            if (rigidBody && !FM_IS_SET(rigidBody->flags, FM_OBJECT_FLAG_SLEEPING))
            {
                island.rigidBodyIds[numRigidBodiesOutput] = island.rigidBodyIds[rbIdx];
                numRigidBodiesOutput++;

                // Remove id from object's current island
                FmRemoveRigidBodyIdFromIsland(scene, rigidBody->objectId);
            }
        }

        island.numTetMeshes = numTetMeshesOutput;
        island.numRigidBodiesConnected = numRigidBodiesOutput;

        if (island.numTetMeshes > 0 || island.numRigidBodiesConnected > 0)
        {
            FmPutConstraintIslandToSleep(scene, island, true, NULL);
        }

        return true;
    }

    bool FmSetAllSceneObjectsSleeping(FmScene* scene)
    {
        return FmCreateSleepingIsland(scene, scene->awakeTetMeshIds, scene->numAwakeTetMeshes, scene->awakeRigidBodyIds, scene->numAwakeRigidBodies);
    }

    bool FmNotifyRigidBodiesSleeping(FmScene* scene, uint* rigidBodyIds, uint numRigidBodies)
    {
        FmConstraintsBuffer& constraintsBuffer = *scene->constraintsBuffer;
        uint numSleepingIslands = constraintsBuffer.numSleepingConstraintIslands;
        uint maxIslands = constraintsBuffer.maxConstraintIslands;

        FM_ASSERT(numSleepingIslands < maxIslands);
        if (numSleepingIslands >= maxIslands)
        {
            return false;
        }

        // Create temporary island to add to sleeping islands
        FmConstraintIsland island;
        FmInitConstraintIsland(&island);
        island.tetMeshIds = NULL;
        island.rigidBodyIds = rigidBodyIds;
        island.numTetMeshes = 0;
        island.numRigidBodiesConnected = numRigidBodies;

        FmPutConstraintIslandToSleep(scene, island, false, NULL);

        return true;
    }

    void FmPutMarkedIslandsToSleep(FmScene* scene, FmAsyncTaskData* parentTaskData)
    {
        FmConstraintsBuffer& constraintsBuffer = *scene->constraintsBuffer;
        uint numIslands = constraintsBuffer.numConstraintIslands;

        // For islands marked for sleeping, create sleeping island and mark bodies
        for (uint idx = 0; idx < numIslands; idx++)
        {
            FmConstraintIsland& constraintIsland = constraintsBuffer.constraintIslands[idx];

            if (FM_IS_SET(constraintIsland.flags, FM_ISLAND_FLAG_MARKED_FOR_SLEEPING))
            {
                FmPutConstraintIslandToSleep(scene, constraintIsland, true, parentTaskData);

                FmInitConstraintIsland(&constraintIsland);  // Clear source island
            }
        }
    }

    void FmWakeConstraintIsland(FmScene* scene, uint sleepingIslandId)
    {
        FmConstraintsBuffer& constraintsBuffer = *scene->constraintsBuffer;

        // Get the index of sleeping island in the constraintIslands array
        uint sleepingIslandIdx = constraintsBuffer.sleepingIslandIdToArrayIdx[sleepingIslandId];
        FM_ASSERT(sleepingIslandIdx != FM_INVALID_ID);

        // Release sleeping island id
        FmReleaseId(&constraintsBuffer.freeSleepingIslandIds, sleepingIslandId);
        constraintsBuffer.sleepingIslandIdToArrayIdx[sleepingIslandId] = FM_INVALID_ID;

        FmSleepingConstraintIsland& sleepingIsland = constraintsBuffer.sleepingConstraintIslands[sleepingIslandIdx];
        FM_ASSERT(sleepingIsland.islandId == sleepingIslandId);
        FM_ASSERT(FM_IS_SET(sleepingIsland.flags, FM_ISLAND_FLAG_SLEEPING));

        // Mark island as deleted
        sleepingIsland.islandId = FM_INVALID_ID;
        sleepingIsland.flags &= ~(FM_ISLAND_FLAG_SLEEPING | FM_ISLAND_FLAG_MARKED_FOR_WAKING);

        // Mark all objects in island as awake
        for (uint idx = 0; idx < sleepingIsland.numTetMeshes; idx++)
        {
            uint tetMeshId = sleepingIsland.tetMeshIds[idx];

            // Remove id from sleepingTetMeshIds
            FmRemoveIdFromSceneSleepingTetMeshIds(scene, tetMeshId);

            // Add to active ids list
            FmAddIdToSceneAwakeTetMeshIds(scene, tetMeshId);

            FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, tetMeshId);
            tetMesh.islandId = FM_INVALID_ID;
            tetMesh.flags &= ~FM_OBJECT_FLAG_SLEEPING;

            scene->numAwakenedTetMeshes++;
        }
        for (uint idx = 0; idx < sleepingIsland.numRigidBodiesConnected; idx++)
        {
            uint rigidBodyId = sleepingIsland.rigidBodyIds[idx];

            // Remove id from sleepingRigidBodyIds
            FmRemoveIdFromSceneSleepingRigidBodyIds(scene, rigidBodyId);

            // Add to active ids list
            FmAddIdToSceneAwakeRigidBodyIds(scene, rigidBodyId);

            FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, rigidBodyId);
            rigidBody.femIslandId = FM_INVALID_ID;
            rigidBody.flags &= ~FM_OBJECT_FLAG_SLEEPING;

            scene->numAwakenedRigidBodies++;
        }

        if (scene->islandWakingCallback && sleepingIsland.numRigidBodiesConnected > 0)
        {
            scene->islandWakingCallback(scene, scene->rigidBodiesUserData, sleepingIsland.rigidBodyIds, sleepingIsland.numRigidBodiesConnected);
        }
    }

    void FmCompactSleepingIslands(FmScene* scene)
    {
        FmConstraintsBuffer& constraintsBuffer = *scene->constraintsBuffer;
        uint numSleepingIslands = constraintsBuffer.numSleepingConstraintIslands;

        uint newNumSleepingIslands = 0;
        uint newNumSleepingTetMeshes = 0;
        uint newNumSleepingRigidBodies = 0;

        for (uint srcIslandIdx = 0; srcIslandIdx < numSleepingIslands; srcIslandIdx++)
        {
            uint dstIslandIdx = newNumSleepingIslands;

            // Save copy of src
            FmSleepingConstraintIsland srcIsland = constraintsBuffer.sleepingConstraintIslands[srcIslandIdx];

            if (srcIsland.islandId != FM_INVALID_ID)
            {
                FmSleepingConstraintIsland& dstIsland = constraintsBuffer.sleepingConstraintIslands[dstIslandIdx];

                dstIsland = srcIsland;
                dstIsland.tetMeshIds = constraintsBuffer.allSleepingIslandTetMeshIds + newNumSleepingTetMeshes;
                dstIsland.rigidBodyIds = constraintsBuffer.allSleepingIslandRigidBodyIds + newNumSleepingRigidBodies;

                uint numIslandTetMeshes = srcIsland.numTetMeshes;
                uint numIslandRigidBodies = srcIsland.numRigidBodiesConnected;

                newNumSleepingTetMeshes += numIslandTetMeshes;
                newNumSleepingRigidBodies += numIslandRigidBodies;

                if (dstIsland.tetMeshIds != srcIsland.tetMeshIds)
                {
                    for (uint idx = 0; idx < numIslandTetMeshes; idx++)
                    {
                        dstIsland.tetMeshIds[idx] = srcIsland.tetMeshIds[idx];
                    }
                }

                if (dstIsland.rigidBodyIds != srcIsland.rigidBodyIds)
                {
                    for (uint idx = 0; idx < numIslandRigidBodies; idx++)
                    {
                        dstIsland.rigidBodyIds[idx] = srcIsland.rigidBodyIds[idx];
                    }
                }

                constraintsBuffer.sleepingIslandIdToArrayIdx[dstIsland.islandId] = dstIslandIdx;
                newNumSleepingIslands++;
            }
        }

        constraintsBuffer.numSleepingConstraintIslands = newNumSleepingIslands;
        constraintsBuffer.numSleepingIslandTetMeshes = newNumSleepingTetMeshes;
        constraintsBuffer.numSleepingIslandRigidBodies = newNumSleepingRigidBodies;
    }

    void FmWakeMarkedIslands(FmScene* scene)
    {
        FmConstraintsBuffer& constraintsBuffer = *scene->constraintsBuffer;
        uint numSleepingIslands = constraintsBuffer.numSleepingConstraintIslands;

        // For islands marked for waking, move objects to active ids list and mark bodies
        for (uint sleepingIslandIdx = 0; sleepingIslandIdx < numSleepingIslands; sleepingIslandIdx++)
        {
            FmSleepingConstraintIsland& sleepingIsland = constraintsBuffer.sleepingConstraintIslands[sleepingIslandIdx];

            FM_ASSERT(constraintsBuffer.sleepingIslandIdToArrayIdx[sleepingIsland.islandId] == sleepingIslandIdx);

            if (FM_IS_SET(sleepingIsland.flags, FM_ISLAND_FLAG_MARKED_FOR_WAKING))
            {
                FmWakeConstraintIsland(scene, sleepingIsland.islandId);
            }
        }

        FM_ASSERT(scene->numAwakeTetMeshes + scene->numSleepingTetMeshes <= scene->maxTetMeshes);
        FM_ASSERT(scene->numAwakeRigidBodies + scene->numSleepingRigidBodies <= scene->maxRigidBodies);

        FmCompactSleepingIslands(scene);
    }

    bool FmMarkIslandOfObjectForWaking(FmScene* scene, uint objectId)
    {
        // Return if called without a scene or the object id invalid (not yet added to scene)
        if (scene == NULL || objectId == FM_INVALID_ID)
        {
            return false;
        }

        uint islandId = FM_INVALID_ID;
        if (objectId & FM_RB_FLAG)
        {
            FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, objectId);

            if (FM_NOT_SET(rigidBody.flags, FM_OBJECT_FLAG_SLEEPING))
            {
                return false;
            }

            islandId = rigidBody.femIslandId;
        }
        else
        {
            FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, objectId);

            if (FM_NOT_SET(tetMesh.flags, FM_OBJECT_FLAG_SLEEPING))
            {
                return false;
            }

            islandId = tetMesh.islandId;
        }
        FmSleepingConstraintIsland& sleepingIsland = *FmGetSleepingConstraintIslandById(*scene, islandId);
        sleepingIsland.flags |= FM_ISLAND_FLAG_MARKED_FOR_WAKING;
        FM_ASSERT(FM_IS_SET(sleepingIsland.flags, FM_ISLAND_FLAG_SLEEPING));

        return true;
    }

    void FmMarkIslandForSleeping(FmScene* scene, uint islandId)
    {
        FmConstraintsBuffer& constraintsBuffer = *scene->constraintsBuffer;

        uint islandIdx = islandId;
        FM_ASSERT(islandIdx != FM_INVALID_ID);
        FmConstraintIsland& island = constraintsBuffer.constraintIslands[islandIdx];
        island.flags |= FM_ISLAND_FLAG_MARKED_FOR_SLEEPING;
        FM_ASSERT(!FM_IS_SET(island.flags, FM_ISLAND_FLAG_SLEEPING));
    }

    // Notify of object waking due to external change or collision event, in order to wake FEM island.
    // Not necessary if calling FmSetState.
    void FmNotifyObjectWaking(FmScene* scene, uint objectId)
    {
        FmMarkIslandOfObjectForWaking(scene, objectId);
    }
}