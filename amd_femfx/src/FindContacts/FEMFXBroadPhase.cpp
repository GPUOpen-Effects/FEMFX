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
// Broad-phase based on scene BVH build and traversal
//---------------------------------------------------------------------------------------

#include "FEMFXBroadPhase.h"
#include "FEMFXBvhBuild.h"
#include "FEMFXBvhCollision.h"
#include "FEMFXScene.h"
#include "FEMFXRigidBody.h"
#include "FEMFXParallelFor.h"
#include "FEMFXSort.h"

namespace AMD
{
    void FmFitBroadPhaseLeafAabbs(
        FmBvh* resultBvh, FmVector3* worldMinPosition, FmVector3* worldMaxPosition,
        FmScene* scene,
        const uint* tetMeshIds, uint numTetMeshes,
        const uint* rigidBodyIds, uint numRigidBodies)  // may be awake or sleeping objects  
    {
        FmBvh& bvh = *resultBvh;

        bvh.numPrims = numTetMeshes + numRigidBodies;

        FmVector3 minPosition(0.0f), maxPosition(0.0f);

        if (numTetMeshes > 0)
        {
            FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, tetMeshIds[0]);

            minPosition = tetMesh.bvh.nodes[0].box.pmin;
            maxPosition = tetMesh.bvh.nodes[0].box.pmax;
        }
        else if (numRigidBodies > 0)
        {
            FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, rigidBodyIds[0]);

            minPosition = rigidBody.aabb.pmin;
            maxPosition = rigidBody.aabb.pmax;
        }

        for (uint meshIdx = 0; meshIdx < numTetMeshes; meshIdx++)
        {
            FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, tetMeshIds[meshIdx]);

            minPosition = min(minPosition, tetMesh.bvh.nodes[0].box.pmin);
            maxPosition = max(maxPosition, tetMesh.bvh.nodes[0].box.pmax);

            bvh.primBoxes[meshIdx] = tetMesh.bvh.nodes[0].box;
        }

        for (uint rbIdx = 0; rbIdx < numRigidBodies; rbIdx++)
        {
            FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, rigidBodyIds[rbIdx]);

            minPosition = min(minPosition, rigidBody.aabb.pmin);
            maxPosition = max(maxPosition, rigidBody.aabb.pmax);

            bvh.primBoxes[numTetMeshes + rbIdx] = rigidBody.aabb;
        }

        *worldMinPosition = minPosition;
        *worldMaxPosition = maxPosition;
    }

    void FmBuildBroadPhaseBvh(
        FmBvh* resultBvh, FmVector3* worldMinPosition, FmVector3* worldMaxPosition,
        FmScene* scene, const uint* tetMeshIds, uint numTetMeshes, uint* rigidBodyIds, uint numRigidBodies)
    {
        FmFitBroadPhaseLeafAabbs(resultBvh, worldMinPosition, worldMaxPosition, scene, tetMeshIds, numTetMeshes, rigidBodyIds, numRigidBodies);
        FmBuildBvhOnLeaves(resultBvh, *worldMinPosition, *worldMaxPosition);

        // Convert indices to object ids
        uint numObjects = resultBvh->numPrims;

        for (uint i = 0; i < numObjects; i++)
        {
            uint objectIdx = resultBvh->primIndicesSorted[i];
            uint objectId;
            if (objectIdx < numTetMeshes)
            {
                objectId = tetMeshIds[objectIdx];
            }
            else
            {
                objectId = rigidBodyIds[objectIdx - numTetMeshes];
            }

            resultBvh->primIndicesSorted[i] = objectId;
            resultBvh->GetLeafNode(i).left = objectId;
            resultBvh->GetLeafNode(i).right = objectId;
        }
    }

    void FmSuballocSleepingBvh(FmBvh* sleepingBvh, uint numSleepingPrims, const FmBvh& awakeBvh)
    {
        uint numAwakePrims = awakeBvh.numPrims;
        uint numAwakeNodes = FmNumBvhNodes(numAwakePrims);
        sleepingBvh->nodes = awakeBvh.nodes + numAwakeNodes;
        sleepingBvh->primBoxes = awakeBvh.primBoxes + numAwakePrims;
        sleepingBvh->mortonCodesSorted = awakeBvh.mortonCodesSorted + numAwakePrims;
        sleepingBvh->primIndicesSorted = awakeBvh.primIndicesSorted + numAwakePrims;
        sleepingBvh->numPrims = numSleepingPrims;
    }

    // For CCD, must build after the objects' implicit step and Bvh rebuild to computes new velocities.
    void FmBuildBroadPhaseHierarchies(FmTaskDataFindContacts* findContactsData)
    {
        FmScene* scene = findContactsData->scene;
        bool includeRigidBodiesInBroadPhase = findContactsData->includeRigidBodiesInBroadPhase;
        bool testWithSleepingObjects = findContactsData->testWithSleepingObjects;
        uint numTetMeshes = findContactsData->numTetMeshes;
        uint numRigidBodies = findContactsData->numRigidBodies;
        uint* tetMeshIds = findContactsData->tetMeshIds;
        uint* rigidBodyIds = findContactsData->rigidBodyIds;

        FmConstraintsBuffer& constraintsBuffer = *scene->constraintsBuffer;

        constraintsBuffer.broadPhaseHierarchy.numPrims = 0;
        constraintsBuffer.sleepingBroadPhaseHierarchy.numPrims = 0;

        FmVector3 minPosition = FmInitVector3(0.0f);
        FmVector3 maxPosition = FmInitVector3(0.0f);

        uint numAwakeObjects = numTetMeshes;
        if (includeRigidBodiesInBroadPhase)
        {
            numAwakeObjects += numRigidBodies;
        }

        if (numAwakeObjects > 0)
        {
            FmBuildBroadPhaseBvh(&constraintsBuffer.broadPhaseHierarchy, &minPosition, &maxPosition, scene, tetMeshIds, numTetMeshes, rigidBodyIds,
                includeRigidBodiesInBroadPhase ? numRigidBodies : 0);

            constraintsBuffer.worldMinPosition = minPosition;
            constraintsBuffer.worldMaxPosition = maxPosition;
        }

        if (testWithSleepingObjects)
        {
            uint numSleepingObjects = scene->numSleepingTetMeshes;
            if (includeRigidBodiesInBroadPhase)
            {
                numSleepingObjects += scene->numSleepingRigidBodies;
            }

            if (numSleepingObjects > 0)
            {
                FmSuballocSleepingBvh(&constraintsBuffer.sleepingBroadPhaseHierarchy, numSleepingObjects, constraintsBuffer.broadPhaseHierarchy);
                FmBuildBroadPhaseBvh(&constraintsBuffer.sleepingBroadPhaseHierarchy, &minPosition, &maxPosition, scene, scene->sleepingTetMeshIds, scene->numSleepingTetMeshes, scene->sleepingRigidBodyIds,
                    includeRigidBodiesInBroadPhase ? scene->numSleepingRigidBodies : 0);

                constraintsBuffer.worldMinPosition = min(constraintsBuffer.worldMinPosition, minPosition);
                constraintsBuffer.worldMaxPosition = max(constraintsBuffer.worldMaxPosition, maxPosition);
            }
        }
    }

    static FM_FORCE_INLINE void FmAddBroadPhasePair(FmScene* inScene, uint objectIdA, uint objectIdB)
    {
        FmScene& scene = *inScene;

        FmConstraintsBuffer& constraintsBuffer = *scene.constraintsBuffer;

        bool isRbPair = false;
        uint collisionGroupA;
        uint weightA;
        if (FM_IS_SET(objectIdA, FM_RB_FLAG))
        {
            FmRigidBody& rigidBodyA = *FmGetRigidBodyPtrById(scene, objectIdA);
            collisionGroupA = rigidBodyA.collisionGroup;
            weightA = 0;
            isRbPair = true;
        }
        else
        {
            FmTetMesh& tetMeshA = *FmGetTetMeshPtrById(scene, objectIdA);
            collisionGroupA = tetMeshA.collisionGroup;
            weightA = tetMeshA.numExteriorFaces;
        }

        uint collisionGroupB;
        uint weightB;
        if (FM_IS_SET(objectIdB, FM_RB_FLAG))
        {
            FmRigidBody& rigidBodyB = *FmGetRigidBodyPtrById(scene, objectIdB);
            collisionGroupB = rigidBodyB.collisionGroup;
            weightB = 0;
            isRbPair = true;
        }
        else
        {
            FmTetMesh& tetMeshB = *FmGetTetMeshPtrById(scene, objectIdB);
            collisionGroupB = tetMeshB.collisionGroup;
            weightB = tetMeshB.numExteriorFaces;
        }

        bool externalBroadPhasePairs = isRbPair && scene.params.rigidBodiesExternal;

        FmBroadPhasePair* pairs = externalBroadPhasePairs ? constraintsBuffer.rigidBodyBroadPhasePairs : constraintsBuffer.broadPhasePairs;
        uint& numPairs = externalBroadPhasePairs ? constraintsBuffer.numRigidBodyBroadPhasePairs.val : constraintsBuffer.numBroadPhasePairs.val;
        uint maxPairs = externalBroadPhasePairs ? constraintsBuffer.maxRigidBodyBroadPhasePairs : constraintsBuffer.maxBroadPhasePairs;

        if (FmAtomicRead(&numPairs) < maxPairs && FmGroupsCanCollide(scene, collisionGroupA, collisionGroupB))
        {
            uint pairIdx = FmAtomicIncrement(&numPairs) - 1;

            if (pairIdx < maxPairs)
            {
                pairs[pairIdx].objectIdA = FmMinUint(objectIdA, objectIdB);
                pairs[pairIdx].objectIdB = FmMaxUint(objectIdA, objectIdB);
                pairs[pairIdx].weight = weightA + weightB;
            }
            else
            {
                FmAtomicWrite(&numPairs, maxPairs);
            }
        }
    }

    void FmCollideBroadPhaseBvhPairCallback(void* userData, uint objectIdA, uint objectIdB)
    {
        if (objectIdA == objectIdB)
        {
            return; // self collision not yet supported
        }

        FmScene* scene = (FmScene*)userData;
        FmAddBroadPhasePair(scene, objectIdA, objectIdB);
    }

#if FM_PARALLEL_BROAD_PHASE_TRAVERSAL
    void FmCollideBroadPhaseObjectBvhCallback(void* userData, uint objectIdA, uint objectIdB)
    {
        if (objectIdA >= objectIdB)
        {
            return; // self collision not yet supported; cull duplicates
        }

        FmScene* scene = (FmScene*)userData;
        FmAddBroadPhasePair(scene, objectIdA, objectIdB);
    }

    void FmCollideBroadPhaseObjectSleepingBvhCallback(void* userData, uint objectIdA, uint objectIdB)
    {
        FM_ASSERT(objectIdA != objectIdB);

        FmScene* scene = (FmScene*)userData;
        FmAddBroadPhasePair(scene, objectIdA, objectIdB);
    }

    class FmTaskDataBroadPhaseCollideObjects : public FmAsyncTaskData
    {
    public:
        FM_CLASS_NEW_DELETE(FmTaskDataBroadPhaseCollideObjects)

        uint numTasks;
        FmScene* scene;
        float timestep;

        const FmBvh* broadPhaseHierarchy;
        const FmBvh* sleepingBroadPhaseHierarchy;
        bool testWithSleepingObjects;

        FmTaskDataBroadPhaseCollideObjects(
            uint inNumTasks,
            FmScene* inScene,
            float inTimestep,
            const FmBvh* inBroadPhaseHierarchy,
            const FmBvh* inSleepingBroadPhaseHierarchy,
            bool inTestWithSleepingObjects)
        {
            numTasks = inNumTasks;
            scene = inScene;
            timestep = inTimestep;
            broadPhaseHierarchy = inBroadPhaseHierarchy;
            sleepingBroadPhaseHierarchy = inSleepingBroadPhaseHierarchy;
            testWithSleepingObjects = inTestWithSleepingObjects;
        }
    };
#endif

    void FmSetBroadPhaseWarnings(FmScene* scene)
    {
        FmConstraintsBuffer& constraintsBuffer = *scene->constraintsBuffer;

        uint numBroadPhasePairs = FmAtomicRead(&constraintsBuffer.numBroadPhasePairs.val);
        uint numRigidBodyBroadPhasePairs = FmAtomicRead(&constraintsBuffer.numRigidBodyBroadPhasePairs.val);

        // Report if num broad phase pairs at maximum values
        if (numBroadPhasePairs == constraintsBuffer.maxBroadPhasePairs)
        {
            FmAtomicOr(&scene->warningsReport.flags.val, FM_WARNING_FLAG_HIT_LIMIT_SCENE_BROAD_PHASE_PAIRS);
        }
        if (scene->params.rigidBodiesExternal && numRigidBodyBroadPhasePairs == constraintsBuffer.maxRigidBodyBroadPhasePairs)
        {
            FmAtomicOr(&scene->warningsReport.flags.val, FM_WARNING_FLAG_HIT_LIMIT_SCENE_RIGID_BODY_BROAD_PHASE_PAIRS);
        }
    }

    // Sort by decreasing weight
    class FmCompareBroadPhasePairSize
    {
    public:
        FmCompareBroadPhasePairSize(void* inUserData) { (void)inUserData; }

        inline bool operator ()(const FmBroadPhasePair& pairA, const FmBroadPhasePair& pairB)
        {
            return pairA.weight > pairB.weight;
        }

        inline int Compare(const FmBroadPhasePair& pairA, const FmBroadPhasePair& pairB)
        {
            return FM_QSORT_DECREASING_RETVAL(pairA.weight, pairB.weight);
        }
    };

#if FM_PARALLEL_BROAD_PHASE_TRAVERSAL
    FM_WRAPPED_TASK_FUNC(FmTaskFuncBroadPhaseCollideObjects)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT(BROAD_PHASE);

        FmTaskDataBroadPhaseCollideObjects* taskData = (FmTaskDataBroadPhaseCollideObjects *)inTaskData;

        FmScene* scene = taskData->scene;
        float timestep = taskData->timestep;

        const FmBvh& broadPhaseHierarchy = *taskData->broadPhaseHierarchy;
        const FmBvh& sleepingBroadPhaseHierarchy = *taskData->sleepingBroadPhaseHierarchy;

        bool testWithSleepingObjects = taskData->testWithSleepingObjects;

        uint startIdx, endIdx;
        FmGetIndexRangeEvenDistribution(&startIdx, &endIdx, (uint)inTaskBeginIndex, taskData->numTasks, broadPhaseHierarchy.numPrims);

        for (uint i = startIdx; i < endIdx; i++)
        {
            const FmBvhNode& objectNode = broadPhaseHierarchy.GetLeafNode(i);
            uint objectId = objectNode.left;
            FmObjectBvhCcd(FmCollideBroadPhaseObjectBvhCallback, scene, objectNode.box, objectId, broadPhaseHierarchy, timestep);

            if (testWithSleepingObjects)
            {
                FmObjectBvhCcd(FmCollideBroadPhaseObjectSleepingBvhCallback, scene, objectNode.box, objectId, sleepingBroadPhaseHierarchy, timestep);
            }
        }

        taskData->progress.TaskIsFinished();  // Task data not deleted until later stage
    }

    void FmTaskFuncFinishBroadPhase(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FmTaskDataBroadPhaseCollideObjects* taskData = (FmTaskDataBroadPhaseCollideObjects *)inTaskData;
        FmScene* scene = taskData->scene;

        // Finish broad phase
        FmSetBroadPhaseWarnings(scene);

        // Sort broad phase pairs by estimated cost
        uint numBroadPhasePairs = FmAtomicRead(&scene->constraintsBuffer->numBroadPhasePairs.val);
        FmSort<FmBroadPhasePair, FmCompareBroadPhasePairSize>(scene->constraintsBuffer->broadPhasePairs, numBroadPhasePairs, NULL);

        FM_ADD_BROAD_PHASE_TIME();

        FmSetNextTask(taskData->followTask.func, taskData->followTask.data, 0, 1);

        delete taskData;
    }
#endif

    void FmBroadPhase(FmTaskDataFindContacts* findContactsData, FmTaskFuncCallback followTaskFunc, void* followTaskData)
    {
        FM_TRACE_SCOPED_EVENT(BROAD_PHASE);

        FM_SET_START_TIME();

        FmScene* scene = findContactsData->scene;
        float timestep = scene->params.timestep;// findContactsData->timestep;
        bool testWithSleepingObjects = findContactsData->testWithSleepingObjects;

        // Build hierarchy over scene
        FmBuildBroadPhaseHierarchies(findContactsData);

        // Zero pairs
        FmConstraintsBuffer& constraintsBuffer = *scene->constraintsBuffer;
        FmAtomicWrite(&constraintsBuffer.numBroadPhasePairs.val, 0);
        FmAtomicWrite(&constraintsBuffer.numRigidBodyBroadPhasePairs.val, 0);

#if FM_PARALLEL_BROAD_PHASE_TRAVERSAL
        if (constraintsBuffer.broadPhaseHierarchy.numPrims > 0)
        {
            // Create task data and start hierarchy collisions, followed by sorting

            // Set num tasks to multiple of threads
            uint numTasks = FmNextPowerOf2(scene->params.numThreads) * 8;

            FmTaskDataBroadPhaseCollideObjects* taskData = new FmTaskDataBroadPhaseCollideObjects(numTasks, scene, timestep,
                &constraintsBuffer.broadPhaseHierarchy, &constraintsBuffer.sleepingBroadPhaseHierarchy, testWithSleepingObjects);

            taskData->progress.Init(numTasks, FmTaskFuncFinishBroadPhase, taskData);
            taskData->followTask.func = followTaskFunc;
            taskData->followTask.data = followTaskData;

            FmParallelForAsync("CollideBroadPhase", FM_TASK_AND_WRAPPED_TASK_ARGS(FmTaskFuncBroadPhaseCollideObjects), NULL, taskData, numTasks, scene->taskSystemCallbacks.SubmitAsyncTask, scene->params.numThreads);
        }
        else
        {
            FmSetNextTask(followTaskFunc, followTaskData, 0, 1);
        }
#else
        (void)followTaskFunc;
        (void)followTaskData;

        // Awake tet meshes vs. awake tet meshes
        FmBvhSelfCcd(FmCollideBroadPhaseBvhPairCallback, scene, constraintsBuffer.broadPhaseHierarchy, timestep);

        if (testWithSleepingObjects)
        {
            // Awake tet meshes vs. sleeping tet meshes
            FmBvhPairCcd(FmCollideBroadPhaseBvhPairCallback, scene, constraintsBuffer.broadPhaseHierarchy, constraintsBuffer.sleepingBroadPhaseHierarchy, timestep);
        }

        // Finish broad phase
        FmSetBroadPhaseWarnings(scene);

        // Sort broad phase pairs by estimated cost
        FmSort<FmBroadPhasePair, FmCompareBroadPhasePairSize>(scene->constraintsBuffer->broadPhasePairs, scene->constraintsBuffer->numBroadPhasePairs.val, NULL);

        FM_ADD_BROAD_PHASE_TIME();
#endif
    }
}