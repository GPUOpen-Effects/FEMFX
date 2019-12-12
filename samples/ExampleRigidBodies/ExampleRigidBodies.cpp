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

#include "ExampleRigidBodies.h"
#include "SampleTaskSystem.h"
#include <vector>

using namespace AMD;

namespace AMD
{
#if FM_DEBUG_MESHES
    extern FmTetMesh* gFEMFXPreConstraintSolveMeshes;
    extern FmRigidBody* gFEMFXPreConstraintSolveRigidBodies;
    extern uint gFEMFXNumPreConstraintSolveMeshes;
    extern uint gFEMFXNumPreConstraintSolveRigidBodies;
#endif

#define FEM_SOLVED_ISLAND FM_ISLAND_FLAG_USER

    ExampleRigidBodiesScene::ExampleRigidBodiesScene(const ExampleRigidBodiesSceneSetupParams& setupParams)
    {
        // Setup scene
        FmSceneSetupParams sceneParams;
        sceneParams.maxTetMeshBuffers = 0;
        sceneParams.maxTetMeshes = 0;
        sceneParams.maxRigidBodies = setupParams.maxRigidBodies;
        sceneParams.maxDistanceContacts = setupParams.rigidRigidConstraintsSetupParams.maxDistanceContacts;
        sceneParams.maxVolumeContacts = setupParams.rigidRigidConstraintsSetupParams.maxVolumeContacts;
        sceneParams.maxVolumeContactVerts = setupParams.rigidRigidConstraintsSetupParams.maxVolumeContactVerts;
        sceneParams.maxDeformationConstraints = 0;
        sceneParams.maxGlueConstraints = setupParams.rigidRigidConstraintsSetupParams.maxGlueConstraints;
        sceneParams.maxPlaneConstraints = setupParams.rigidRigidConstraintsSetupParams.maxPlaneConstraints;
        sceneParams.maxRigidBodyAngleConstraints = setupParams.rigidRigidConstraintsSetupParams.maxRigidBodyAngleConstraints;
        sceneParams.maxBroadPhasePairs = 0;
        sceneParams.maxRigidBodyBroadPhasePairs = 0;
        sceneParams.maxSceneVerts = 0;
        sceneParams.maxTetMeshBufferFeatures = 16384;
        sceneParams.numWorkerThreads = setupParams.numWorkerThreads;

        sceneParams.maxConstraintSolverDataSize = FmEstimateSceneConstraintSolverDataSize(sceneParams);

        scene = FmCreateScene(sceneParams);

        scene->taskSystemCallbacks.SetCallbacks(
            SampleGetTaskSystemNumThreads,
            SampleGetTaskSystemWorkerIndex,
            SampleAsyncTask,
            SampleCreateSyncEvent,
            SampleDestroySyncEvent,
            SampleWaitForSyncEvent,
            SampleTriggerSyncEvent
#if !FM_ASYNC_THREADING
            , SampleCreateTaskWaitCounter,
            SampleWaitForTaskWaitCounter,
            SampleDestroyTaskWaitCounter,
            SampleSubmitTask,
            SampleParallelFor
#endif
        );

        scene->params.rigidBodiesExternal = true;

        // Leaving these to be handled by scene with FEM objects
        scene->params.includeRigidBodiesInBroadPhase = false;

        // Create islands for rigid bodies with no constraints, to test for sleeping
        scene->params.createZeroConstraintRigidBodyIslands = true;

        uint maxRigidBodies = setupParams.maxRigidBodies;

        // Setup rigid/FEM constraints buffer
        FmConstraintsBufferSetupParams constraintsSetupParamsRF;
        constraintsSetupParamsRF.maxTetMeshes = 0;
        constraintsSetupParamsRF.maxRigidBodies = maxRigidBodies;
        constraintsSetupParamsRF.maxDistanceContacts = setupParams.rigidFEMConstraintsSetupParams.maxDistanceContacts;
        constraintsSetupParamsRF.maxFractureContacts = setupParams.rigidFEMConstraintsSetupParams.maxFractureContacts;
        constraintsSetupParamsRF.maxVolumeContacts = setupParams.rigidFEMConstraintsSetupParams.maxVolumeContacts;
        constraintsSetupParamsRF.maxVolumeContactVerts = setupParams.rigidFEMConstraintsSetupParams.maxVolumeContactVerts;
        constraintsSetupParamsRF.maxDeformationConstraints = 0;
        constraintsSetupParamsRF.maxGlueConstraints = setupParams.rigidFEMConstraintsSetupParams.maxGlueConstraints;
        constraintsSetupParamsRF.maxPlaneConstraints = setupParams.rigidFEMConstraintsSetupParams.maxPlaneConstraints;
        constraintsSetupParamsRF.maxRigidBodyAngleConstraints = setupParams.rigidFEMConstraintsSetupParams.maxRigidBodyAngleConstraints;
        constraintsSetupParamsRF.maxBroadPhasePairs = setupParams.rigidFEMConstraintsSetupParams.maxBroadPhasePairs;
        constraintsSetupParamsRF.maxRigidBodyBroadPhasePairs = 0;

        size_t constraintsBufferSizeRF = FmGetConstraintsBufferSize(constraintsSetupParamsRF);
        uint8_t* constraintsBufferMemRF = (uint8_t*)FmAlignedMalloc(constraintsBufferSizeRF, 64);

        rigidFEMConstraints = FmSetupConstraintsBuffer(constraintsSetupParamsRF, constraintsBufferMemRF, constraintsBufferSizeRF);

        scene->params.constraintSolveParams.passParams[FM_GS_PASS_IDX].maxOuterIterations = 1;
        scene->params.constraintSolveParams.passParams[FM_GS_PASS_IDX].maxInnerIterations = 15;
        scene->params.constraintSolveParams.passParams[FM_CG_PASS_IDX].maxOuterIterations = 0;
        scene->params.constraintSolveParams.passParams[FM_CG_PASS_IDX].maxInnerIterations = 0;
        scene->params.constraintSolveParams.maxCgIterationsCgPass = 0;

        scene->params.constraintStabilizationParams.passParams[FM_GS_PASS_IDX].maxOuterIterations = 1;
        scene->params.constraintStabilizationParams.passParams[FM_GS_PASS_IDX].maxInnerIterations = 10;
        scene->params.constraintStabilizationParams.passParams[FM_CG_PASS_IDX].maxOuterIterations = 0;
        scene->params.constraintStabilizationParams.passParams[FM_CG_PASS_IDX].maxInnerIterations = 0;
        scene->params.constraintStabilizationParams.maxCgIterationsCgPass = 0;
    }

    ExampleRigidBodiesScene::~ExampleRigidBodiesScene()
    {
        FmAlignedFree(rigidFEMConstraints);

        // Delete tet meshes used for collision detection
        for (uint i = 0; i < scene->numRigidBodySlots; i++)
        {
            if (scene->rigidBodies[i])
            {
                FmDestroyRigidBody(scene->rigidBodies[i]);
            }
        }

        FmAlignedFree(scene);
    }

    uint AddRigidBodyToScenes(ExampleRigidBodiesScene* rbScene, FmScene* femScene, FmRigidBody* inRigidBody)
    {
        uint rbId = FmAddRigidBodyToScene(femScene, inRigidBody);

        if (rbId == FM_INVALID_ID)
        {
            return FM_INVALID_ID;
        }

        FmRigidBodySetupParams setupParams;
        setupParams.collisionGroup = inRigidBody->collisionGroup;
        setupParams.halfDimX = inRigidBody->dims[0];
        setupParams.halfDimY = inRigidBody->dims[1];
        setupParams.halfDimZ = inRigidBody->dims[2];
        setupParams.mass = inRigidBody->mass;
        setupParams.state = inRigidBody->state;
        setupParams.isKinematic = FM_IS_SET(inRigidBody->flags, FM_OBJECT_FLAG_KINEMATIC);

        FmRigidBody* rigidBody = FmCreateRigidBody(setupParams);

        uint rbId2 = FmAddRigidBodyToScene(rbScene->scene, rigidBody);

        (void)rbId2;
        FM_ASSERT(rbId == rbId2);

        return rbId;
    }

    void RemoveRigidBodyFromScenes(ExampleRigidBodiesScene* rbScene, FmScene* femScene, uint rigidBodyId)
    {
        // Get rigid body to delete tet mesh buffer used for collision detection
        FmRigidBody* rigidBody = FmGetRigidBody(*rbScene->scene, rigidBodyId);

        if (rigidBody == NULL)
        {
            return;
        }

        FmRemoveRigidBodyFromScene(femScene, rigidBodyId);
        FmRemoveRigidBodyFromScene(rbScene->scene, rigidBodyId);
    }

    FmRigidBody* GetRigidBody(const ExampleRigidBodiesScene& rbScene, uint rigidBodyId)
    {
        return FmGetRigidBody(*rbScene.scene, rigidBodyId);
    }

    void UpdateRigidBodyState(ExampleRigidBodiesScene* rbScene, FmScene* femScene, uint rigidBodyId, const FmRigidBodyState& state)
    {
        FmSetState(femScene, FmGetRigidBody(*femScene, rigidBodyId), state);
        FmSetState(rbScene->scene, FmGetRigidBody(*rbScene->scene, rigidBodyId), state);
    }

    void SetRigidBodyGravityVector(ExampleRigidBodiesScene* rbScene, FmScene* femScene, uint rigidBodyId, const FmVector3& gravityVector)
    {
        FmRigidBody* rb = FmGetRigidBody(*femScene, rigidBodyId);
        if (rb)
        {
            rb->gravityVector = gravityVector;
        }

        rb = FmGetRigidBody(*rbScene->scene, rigidBodyId);
        if (rb)
        {
            rb->gravityVector = gravityVector;
        }
    }

    void EnableRigidBodySimulation(ExampleRigidBodiesScene* rbScene, FmScene* femScene, uint rigidBodyId, bool isEnabled)
    {
        FmRigidBody* rb = FmGetRigidBody(*femScene, rigidBodyId);
        if (rb)
        {
            if (isEnabled)
            {
                rb->flags &= ~FM_OBJECT_FLAG_SIMULATION_DISABLED;
            }
            else
            {
                FmDisableObjectSimulation(femScene, rb->objectId);
            }
        }

        rb = FmGetRigidBody(*rbScene->scene, rigidBodyId);
        if (rb)
        {
            if (isEnabled)
            {
                rb->flags &= ~FM_OBJECT_FLAG_SIMULATION_DISABLED;
            }
            else
            {
                FmDisableObjectSimulation(rbScene->scene, rb->objectId);
            }
        }
    }

    void InitFindContacts(ExampleRigidBodiesScene* rbScene)
    {
        FmScene* rbFemScene = rbScene->scene;

        // Find contacts for rigid bodies
        FmConstraintsBuffer* rigidFEMConstraintsBuffer = rbScene->rigidFEMConstraints;
        FmAtomicWrite(&rigidFEMConstraintsBuffer->numDistanceContacts.val, 0);
        FmAtomicWrite(&rigidFEMConstraintsBuffer->numVolumeContacts.val, 0);
        FmAtomicWrite(&rigidFEMConstraintsBuffer->numVolumeContactVerts.val, 0);
        FmAtomicWrite(&rigidFEMConstraintsBuffer->numDeformationConstraints.val, 0);
        rigidFEMConstraintsBuffer->numGlueConstraints = 0;
        rigidFEMConstraintsBuffer->numPlaneConstraints = 0;
        rigidFEMConstraintsBuffer->numRigidBodyAngleConstraints = 0;

        FmConstraintsBuffer* rigidRigidConstraintsBuffer = rbFemScene->constraintsBuffer;
        FmAtomicWrite(&rigidRigidConstraintsBuffer->numDistanceContacts.val, 0);
        FmAtomicWrite(&rigidRigidConstraintsBuffer->numVolumeContacts.val, 0);
        FmAtomicWrite(&rigidRigidConstraintsBuffer->numVolumeContactVerts.val, 0);
        FmAtomicWrite(&rigidRigidConstraintsBuffer->numDeformationConstraints.val, 0);
        rigidRigidConstraintsBuffer->numGlueConstraints = 0;
        rigidRigidConstraintsBuffer->numPlaneConstraints = 0;
        rigidRigidConstraintsBuffer->numRigidBodyAngleConstraints = 0;
    }

    uint FmFindSceneCollisionPlanesContacts(
        FmScene* rbFemScene,
        FmConstraintsBuffer* constraintsBuffer,
        const FmSceneCollisionPlanes& collisionPlanes,
        const uint* rigidBodyIds,
        uint numRigidBodies,
        float timestep,
        float distContactBias,
        float distContactThreshold)
    {
        FmThreadTempMemoryBuffer* threadTempMemBuffer = rbFemScene->threadTempMemoryBuffer;
        uint8_t* pTempMemBufferStart = threadTempMemBuffer->buffers[0];
        uint8_t* pTempMemBufferEnd = pTempMemBufferStart + threadTempMemBuffer->numBytesPerBuffer;

        uint numContacts = 0;
        for (uint rbIdx = 0; rbIdx < numRigidBodies; rbIdx++)
        {
            FmRigidBody* pRigidBody = FmGetRigidBodyPtrById(*rbFemScene, rigidBodyIds[rbIdx]);

            FmCollidedObjectPair objectPair;
            objectPair.objectA = pRigidBody;
            objectPair.objectB = NULL;
            objectPair.tetMeshA = NULL;
            objectPair.tetMeshB = NULL;
            objectPair.objectAId = pRigidBody->objectId;
            objectPair.objectBId = FM_INVALID_ID;
            objectPair.objectAHierarchy = NULL;
            objectPair.objectBHierarchy = NULL;
            objectPair.objectAMinPosition = FmInitVector3(0.0f);
            objectPair.objectAMaxPosition = FmInitVector3(0.0f);
            objectPair.objectBMinPosition = FmInitVector3(0.0f);
            objectPair.objectBMaxPosition = FmInitVector3(0.0f);
            objectPair.constraintsBuffer = constraintsBuffer;
            objectPair.timestep = timestep;
            objectPair.distContactBias = distContactBias;
            objectPair.distContactThreshold = distContactThreshold;
            objectPair.volContactBias = 0.0f;
            objectPair.volContactThreshold = 0.0f;
            objectPair.numDistanceContacts = 0;
            objectPair.numVolumeContacts = 0;
            objectPair.volContactCenter = FmInitVector3(0.0f);
            objectPair.volContactObjectACenterPos = FmInitVector3(0.0f);
            objectPair.volContactObjectBCenterPos = FmInitVector3(0.0f);
            objectPair.volContactNormal = FmInitVector3(0.0f);
            objectPair.volContactV = 0.0f;
            objectPair.collisionReport = &rbFemScene->collisionReport;
            objectPair.numDistanceContactReports = 0;
            objectPair.numVolumeContactReports = 0;

            uint8_t* pTempMemBuffer = pTempMemBufferStart;

            // Allocate temp memory for object pair
            if (FmAllocCollidedObjectPairTemps(&objectPair.temps, pTempMemBuffer, pTempMemBufferEnd, threadTempMemBuffer->maxTetMeshBufferFeatures,
                0, 0, 0, 0, false
#if FM_SURFACE_INTERSECTION_CONTACTS
                , 0
#endif
            ))
            {
                FmFindSceneCollisionPlanesContactsRb(&objectPair, *pRigidBody, collisionPlanes);

                numContacts += objectPair.numDistanceContacts;
            }
        }
        return numContacts;
    }

    bool FindContactsCollisionPlanesAndBroadPhasePairs(ExampleRigidBodiesScene* rbScene, FmScene* scene, uint* rigidBodyIds, uint numRigidBodies)
    {
        FmScene* rbFemScene = rbScene->scene;
        FmConstraintsBuffer* rigidFEMConstraintsBuffer = rbScene->rigidFEMConstraints;
        FmConstraintsBuffer* rigidRigidConstraintsBuffer = rbFemScene->constraintsBuffer;
        (void)rigidRigidConstraintsBuffer;

        FmSceneControlParams& sceneParams = scene->params;

        float timestep = sceneParams.timestep;

        // Find contacts for rigid bodies
        bool foundContacts = FmFindSceneCollisionPlanesContacts(
            rbFemScene,
#if EXAMPLE_RB_ALL_CONSTRAINTS_IN_FEM_LIB
            rigidFEMConstraintsBuffer,  // external constraints not yet supported with dependency graph
#else
            rigidRigidConstraintsBuffer,
#endif
            sceneParams.collisionPlanes, rigidBodyIds, numRigidBodies, timestep, rbFemScene->params.distContactBias, rbFemScene->params.distContactThreshold) 
        > 0;

        FmThreadTempMemoryBuffer* threadTempMemBuffer = rbFemScene->threadTempMemoryBuffer;
        uint8_t* pTempMemBufferStart = threadTempMemBuffer->buffers[0];
        uint8_t* pTempMemBufferEnd = pTempMemBufferStart + threadTempMemBuffer->numBytesPerBuffer;

        uint numRigidBodyBroadPhasePairs = FmAtomicRead(&scene->constraintsBuffer->numRigidBodyBroadPhasePairs.val);
        for (uint pairIdx = 0; pairIdx < numRigidBodyBroadPhasePairs; pairIdx++)
        {
            FmBroadPhasePair& pair = scene->constraintsBuffer->rigidBodyBroadPhasePairs[pairIdx];

            FmRigidBody* rigidBodyB = GetRigidBody(*rbScene, pair.objectIdB);

            FM_ASSERT(rigidBodyB != NULL);

            if (pair.objectIdA & FM_RB_FLAG)
            {
                FmRigidBody* rigidBodyA = GetRigidBody(*rbScene, pair.objectIdA);

                FM_ASSERT(rigidBodyA != NULL);

                uint collisionGroupA = rigidBodyA->collisionGroup;
                uint collisionGroupB = rigidBodyB->collisionGroup;

                // Exclude kinematic vs kinematic collisions
                if ((FM_IS_SET(rigidBodyA->flags, FM_OBJECT_FLAG_KINEMATIC)
                    && FM_IS_SET(rigidBodyB->flags, FM_OBJECT_FLAG_KINEMATIC))
                    || !FmGroupsCanCollide(*scene, collisionGroupA, collisionGroupB))
                {
                    continue;
                }

                FmTetMesh* meshA = FmGetTetMesh(*(FmTetMeshBuffer*)rigidBodyA->collisionObj, 0);
                FmTetMesh* meshB = FmGetTetMesh(*(FmTetMeshBuffer*)rigidBodyB->collisionObj, 0);

                FmCollidedObjectPair objectPair;
                objectPair.objectA = rigidBodyA;
                objectPair.objectB = rigidBodyB;
                objectPair.tetMeshA = meshA;
                objectPair.tetMeshB = meshB;
                objectPair.objectAId = meshA->objectId;
                objectPair.objectBId = meshB->objectId;
                objectPair.objectAHierarchy = &meshA->bvh;
                objectPair.objectBHierarchy = &meshB->bvh;
                objectPair.objectAMinPosition = meshA->minPosition;
                objectPair.objectAMaxPosition = meshA->maxPosition;
                objectPair.objectBMinPosition = meshB->minPosition;
                objectPair.objectBMaxPosition = meshB->maxPosition;
#if EXAMPLE_RB_ALL_CONSTRAINTS_IN_FEM_LIB
                objectPair.constraintsBuffer = rigidFEMConstraintsBuffer;  // external constraints not yet supported with dependency graph
#else
                objectPair.constraintsBuffer = rigidRigidConstraintsBuffer;
#endif
                objectPair.timestep = timestep;
                objectPair.distContactBias = sceneParams.distContactBias;
                objectPair.distContactThreshold = sceneParams.distContactThreshold;
                objectPair.volContactBias = sceneParams.volContactBias;
                objectPair.volContactThreshold = sceneParams.volContactThreshold;
                objectPair.numDistanceContacts = 0;
                objectPair.numVolumeContacts = 0;

                FmVector3 objectACenterPos = rigidBodyA->state.pos;
                FmVector3 objectBCenterPos = rigidBodyB->state.pos;
                objectPair.volContactCenter = (objectACenterPos + objectBCenterPos) * 0.5f;
                objectPair.volContactObjectACenterPos = objectACenterPos - objectPair.volContactCenter;
                objectPair.volContactObjectBCenterPos = objectBCenterPos - objectPair.volContactCenter;

                objectPair.volContactNormal = FmInitVector3(0.0f);
                objectPair.volContactV = 0.0f;
                objectPair.collisionReport = &scene->collisionReport;
                objectPair.numDistanceContactReports = 0;
                objectPair.numVolumeContactReports = 0;

                uint8_t* pTempMemBuffer = pTempMemBufferStart;

                if (!FmAllocCollidedObjectPairTemps(&objectPair.temps, pTempMemBuffer, pTempMemBufferEnd, threadTempMemBuffer->maxTetMeshBufferFeatures, 
                    meshA->numVerts, meshB->numVerts, 0, 0, false
#if FM_SURFACE_INTERSECTION_CONTACTS
                    , FmMinUint(meshA->numExteriorFaces + meshB->numExteriorFaces, FM_SURFACE_INTERSECTION_MAX_CONTACTS)
#endif
                ))
                {
                    return foundContacts;
                }

                FmBoxBoxCcdTemps* ccdTemps = FmAllocBoxCcdTemps(pTempMemBuffer, (size_t)(pTempMemBufferEnd - pTempMemBuffer));
                if (ccdTemps == NULL)
                {
                    return foundContacts;
                }

                uint numContacts = FmFindContactsBoxBox(&objectPair, ccdTemps);

                if (numContacts)
                {
                    FmMarkIslandOfObjectForWaking(rbFemScene, rigidBodyA->objectId);
                    FmMarkIslandOfObjectForWaking(rbFemScene, rigidBodyB->objectId);

                    // Notify FEM scene so it can wake rigid bodies in case either is sleeping
                    FmNotifyObjectWaking(scene, rigidBodyA->objectId);
                    FmNotifyObjectWaking(scene, rigidBodyB->objectId);
                }

                foundContacts = foundContacts || numContacts;
            }
            else
            {
                FmTetMesh* meshA = FmGetTetMesh(*scene, pair.objectIdA);
                FmTetMesh* meshB = FmGetTetMesh(*(FmTetMeshBuffer*)rigidBodyB->collisionObj, 0);

                uint collisionGroupA = meshA->collisionGroup;
                uint collisionGroupB = rigidBodyB->collisionGroup;

                if (!FmGroupsCanCollide(*scene, collisionGroupA, collisionGroupB))
                {
                    continue;
                }

                FmCollidedObjectPair objectPair;
                objectPair.objectA = meshA;
                objectPair.objectB = rigidBodyB;
                objectPair.tetMeshA = meshA;
                objectPair.tetMeshB = meshB;
                objectPair.objectAId = meshA->objectId;
                objectPair.objectBId = meshB->objectId;
                objectPair.objectAHierarchy = &meshA->bvh;
                objectPair.objectBHierarchy = &meshB->bvh;
                objectPair.objectAMinPosition = meshA->minPosition;
                objectPair.objectAMaxPosition = meshA->maxPosition;
                objectPair.objectBMinPosition = meshB->minPosition;
                objectPair.objectBMaxPosition = meshB->maxPosition;
                objectPair.constraintsBuffer = rigidFEMConstraintsBuffer;
                objectPair.timestep = timestep;
                objectPair.distContactBias = sceneParams.distContactBias;
                objectPair.distContactThreshold = sceneParams.distContactThreshold;
                objectPair.volContactBias = sceneParams.volContactBias;
                objectPair.volContactThreshold = sceneParams.volContactThreshold;
                objectPair.numDistanceContacts = 0;
                objectPair.numVolumeContacts = 0;

                FmVector3 objectACenterPos = meshA->centerOfMass;
                FmVector3 objectBCenterPos = rigidBodyB->state.pos;
                objectPair.volContactCenter = (objectACenterPos + objectBCenterPos) * 0.5f;
                objectPair.volContactObjectACenterPos = objectACenterPos - objectPair.volContactCenter;
                objectPair.volContactObjectBCenterPos = objectBCenterPos - objectPair.volContactCenter;

                objectPair.volContactNormal = FmInitVector3(0.0f);
                objectPair.volContactV = 0.0f;
                objectPair.collisionReport = &scene->collisionReport;
                objectPair.numDistanceContactReports = 0;
                objectPair.numVolumeContactReports = 0;

                uint8_t* pTempMemBuffer = pTempMemBufferStart;

                if (!FmAllocCollidedObjectPairTemps(&objectPair.temps, pTempMemBuffer, pTempMemBufferEnd, threadTempMemBuffer->maxTetMeshBufferFeatures, 
                    meshA->numVerts, meshB->numVerts, 0, 0, false
#if FM_SURFACE_INTERSECTION_CONTACTS
                    , FmMinUint(meshA->numExteriorFaces + meshB->numExteriorFaces, FM_SURFACE_INTERSECTION_MAX_CONTACTS)
#endif
                ))
                {
                    return foundContacts;
                }

                uint numContacts = FmFindContactsTetMeshBox(&objectPair);

                if (numContacts)
                {
                    FmMarkIslandOfObjectForWaking(rbFemScene, rigidBodyB->objectId);

                    // Notify FEM scene so it can wake objects in case either is sleeping
                    FmNotifyObjectWaking(scene, meshA->objectId);
                    FmNotifyObjectWaking(scene, rigidBodyB->objectId);
                }

                foundContacts = foundContacts || numContacts;
            }
        }

        return foundContacts;
    }

    bool FindContacts(ExampleRigidBodiesScene* rbScene, FmScene* scene)
    {
        FM_TRACE_SCOPED_EVENT(FIND_CONTACTS);

        FmScene& rbFemScene = *rbScene->scene;

        InitFindContacts(rbScene);

        return FindContactsCollisionPlanesAndBroadPhasePairs(rbScene, scene, rbFemScene.awakeRigidBodyIds, rbFemScene.numAwakeRigidBodies);
    }

    bool WakeCollidedIslandsAndFindContacts(ExampleRigidBodiesScene* rbScene, FmScene* scene)
    {
        FM_TRACE_SCOPED_EVENT(WAKE_COLLIDED_FIND_CONTACTS);

        FmScene& rbFemScene = *rbScene->scene;

        FmWakeMarkedIslands(&rbFemScene);

        uint numAwakenedRigidBodies = rbFemScene.numAwakenedRigidBodies;
        uint* awakenedRigidBodyIds = rbFemScene.awakeRigidBodyIds + rbFemScene.numAwakeRigidBodies - numAwakenedRigidBodies;
        rbFemScene.numAwakenedRigidBodies = 0;

        return FindContactsCollisionPlanesAndBroadPhasePairs(rbScene, scene, awakenedRigidBodyIds, numAwakenedRigidBodies);
    }

    uint FindConstraintIslands(ExampleRigidBodiesScene* rbScene, FmScene* scene)
    {
        FmScene* rbFemScene = rbScene->scene;

        // Reset rigid body island ids to 0..numRigidBodies, so FindConstraintIslands below will only create islands based on RB/RB constraints.
        uint numRigidBodies = rbFemScene->numAwakeRigidBodies;
        for (uint rbIdx = 0; rbIdx < numRigidBodies; rbIdx++)
        {
            FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*rbFemScene, rbFemScene->awakeRigidBodyIds[rbIdx]);
            rigidBody.rbIslandId = rbIdx;
        }

        FmUserConstraints userConstraints;
        userConstraints.numIslands = numRigidBodies;
        FmFindConstraintIslands(rbFemScene, &userConstraints);

        uint numConstraintIslands = rbFemScene->constraintsBuffer->numConstraintIslands;

        // Update rigid body island ids based on the above island finding, to make FEM islands connect through rigid body islands.
        // Note in FindConstraintIslands() above, rigid bodies without any constraints may not be assigned to any island.
        // But the FEM connected components code requires an island id, so assigning these to increasing values starting from num islands.

        // Initialize rigid body island ids to invalid.
        for (uint rbIdx = 0; rbIdx < numRigidBodies; rbIdx++)
        {
            FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*rbFemScene, rbFemScene->awakeRigidBodyIds[rbIdx]);
            rigidBody.rbIslandId = FM_INVALID_ID;
        }

        // Set island ids for rigid bodies found in constraint islands.
        for (uint islandIdx = 0; islandIdx < numConstraintIslands; islandIdx++)
        {
            FmConstraintIsland& constraintIsland = rbFemScene->constraintsBuffer->constraintIslands[islandIdx];

            uint islandNumRigidBodies = constraintIsland.numRigidBodiesConnected;

            for (uint islandRbIdx = 0; islandRbIdx < islandNumRigidBodies; islandRbIdx++)
            {
                FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*rbFemScene, constraintIsland.rigidBodyIds[islandRbIdx]);
                rigidBody.rbIslandId = islandIdx;
            }
        }

        // Replace any invalid island ids which will be rigid bodies with no constraints.
        // Copy rigid body island ids to FEM scene.
        for (uint rbIdx = 0; rbIdx < numRigidBodies; rbIdx++)
        {
            uint rbId = rbFemScene->awakeRigidBodyIds[rbIdx];
            FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*rbFemScene, rbId);
            if (rigidBody.rbIslandId == FM_INVALID_ID)
            {
                rigidBody.rbIslandId = numConstraintIslands;
                numConstraintIslands++;
            }

            FmSetRigidBodyIsland(FmGetRigidBody(*scene, rbId), rigidBody.rbIslandId);
        }

        return numConstraintIslands;
    }

    // TODO: set norms for convergence test
    void ExampleInnerSolveCB(FmScene* scene, FmSolverIterationNorms* norms, void* userData, uint* rigidBodyIds, uint numRigidBodiesInFEMSolve, uint* islandIndices, uint numIslands, bool isStabilization)
    {
        (void)norms;
        (void)isStabilization;

        ExampleRigidBodiesScene* rbScene = (ExampleRigidBodiesScene*)userData;
        FmScene* rbFemScene = rbScene->scene;

        // Copy current delta pos/vel values from FEM solve to ExampleRigidBodies solve
        for (uint rbIdx = 0; rbIdx < numRigidBodiesInFEMSolve; rbIdx++)
        {
            uint rbId = rigidBodyIds[rbIdx];

            FmRigidBody& femRigidBody = *FmGetRigidBody(*scene, rbId);
            FmRigidBody& rigidBody = *GetRigidBody(*rbScene, rbId);

            rigidBody.deltaVel = femRigidBody.deltaVel;
            rigidBody.deltaAngVel = femRigidBody.deltaAngVel;
            rigidBody.deltaPos = femRigidBody.deltaPos;
            rigidBody.deltaAngPos = femRigidBody.deltaAngPos;
        }

        // Solve the external constraints modifying the delta pos/vel values
        for (uint idx = 0; idx < numIslands; idx++)
        {
            uint islandIdx = islandIndices[idx];
            if (islandIdx >= rbFemScene->constraintsBuffer->numConstraintIslands)
            {
                continue;
            }

            FmConstraintIsland& constraintIsland = rbFemScene->constraintsBuffer->constraintIslands[islandIdx];
            FmConstraintSolverData& constraintSolverData = rbFemScene->constraintSolverBuffer->islandSolverData[islandIdx];

            if (!constraintSolverData.isAllocated)
            {
                continue;
            }

            if (constraintIsland.numConstraints == 0)
            {
                continue;
            }

#if FM_CONSTRAINT_STABILIZATION_SOLVE
            if (isStabilization)
            {
                if (!FM_IS_SET(constraintIsland.flags, FM_ISLAND_FLAG_STABILIZATION_IS_SETUP))
                {
                    FmSetupConstraintStabilization(rbFemScene, &constraintSolverData, &constraintIsland, rbFemScene->params.timestep);
                    constraintIsland.flags |= FM_ISLAND_FLAG_STABILIZATION_IS_SETUP;
                }
                
                // One iteration of PGS interleaved following FEM inner loop iteration
                constraintSolverData.stabilizationParams.passParams[FM_GS_PASS_IDX].maxOuterIterations = 1;
                constraintSolverData.stabilizationParams.passParams[FM_GS_PASS_IDX].maxInnerIterations = 1;
                FmRunConstraintSolve(rbFemScene, &constraintSolverData, constraintIsland);
            }
            else
#endif
            {
                if (!FM_IS_SET(constraintIsland.flags, FM_ISLAND_FLAG_SOLVE_IS_SETUP))
                {
                    FmSetupConstraintSolve(rbFemScene, &constraintSolverData, &constraintIsland, rbFemScene->params.timestep);
                    constraintIsland.flags |= FM_ISLAND_FLAG_SOLVE_IS_SETUP;
                }

                // One iteration of PGS interleaved following FEM inner loop iteration
                constraintSolverData.solveParams.passParams[FM_GS_PASS_IDX].maxOuterIterations = 1;
                constraintSolverData.solveParams.passParams[FM_GS_PASS_IDX].maxInnerIterations = 1;
                FmRunConstraintSolve(rbFemScene, &constraintSolverData, constraintIsland);
            }
        }

        // Copy new delta pos/vel values back to FEM solve
        for (uint rbIdx = 0; rbIdx < numRigidBodiesInFEMSolve; rbIdx++)
        {
            uint rbId = rigidBodyIds[rbIdx];

            FmRigidBody& femRigidBody = *FmGetRigidBody(*scene, rbId);
            FmRigidBody& rigidBody = *GetRigidBody(*rbScene, rbId);

            femRigidBody.deltaPos = rigidBody.deltaPos;
            femRigidBody.deltaAngPos = rigidBody.deltaAngPos;
            femRigidBody.deltaVel = rigidBody.deltaVel;
            femRigidBody.deltaAngVel = rigidBody.deltaAngVel;
        }
    }

    void ExampleIslandCompletedCB(FmScene* scene, void* userData, uint* rigidBodyIds, uint numRigidBodiesInFEMSolve, uint* islandIndices, uint numIslands)
    {
        ExampleRigidBodiesScene* rbScene = (ExampleRigidBodiesScene*)userData;
        FmScene* rbFemScene = rbScene->scene;

        // Copy final state of rigid bodies from FEM to ExampleRigidBodies
        for (uint rbIdx = 0; rbIdx < numRigidBodiesInFEMSolve; rbIdx++)
        {
            uint rbId = rigidBodyIds[rbIdx];

            FmRigidBody& femRigidBody = *FmGetRigidBody(*scene, rbId);
            FmRigidBody& rigidBody = *GetRigidBody(*rbScene, rbId);

            rigidBody.state = femRigidBody.state;
            rigidBody.deltaVel = FmInitVector3(0.0f);
            rigidBody.deltaAngVel = FmInitVector3(0.0f);
            rigidBody.deltaPos = FmInitVector3(0.0f);
            rigidBody.deltaAngPos = FmInitVector3(0.0f);
        }

        // Update all the rigid bodies contained in the connected rigid body islands
        for (uint idx = 0; idx < numIslands; idx++)
        {
            uint islandIdx = islandIndices[idx];
            if (islandIdx >= rbFemScene->constraintsBuffer->numConstraintIslands)
            {
                continue;
            }

            FmConstraintIsland& constraintIsland = rbFemScene->constraintsBuffer->constraintIslands[islandIdx];

            // Mark constraint island as solved by FEM lib
            constraintIsland.flags |= FEM_SOLVED_ISLAND;

            uint numIslandRigidBodies = constraintIsland.numRigidBodiesInFEMSolve;
            for (uint rbIdx = 0; rbIdx < numIslandRigidBodies; rbIdx++)
            {
                uint rbId = constraintIsland.rigidBodyIds[rbIdx];
                FmRigidBody& rigidBody = *GetRigidBody(*rbScene, rbId);

                FmVector3 deltaPos = rigidBody.deltaPos;
                FmVector3 deltaAngPos = rigidBody.deltaAngPos;

                rigidBody.state.pos = rigidBody.state.pos + deltaPos;
                rigidBody.state.quat = FmIntegrateQuat(rigidBody.state.quat, deltaAngPos, 1.0f);

                rigidBody.deltaPos = FmInitVector3(0.0f);
                rigidBody.deltaAngPos = FmInitVector3(0.0f);

                FmVector3 deltaVel = rigidBody.deltaVel;
                FmVector3 deltaAngVel = rigidBody.deltaAngVel;

                rigidBody.state.vel = rigidBody.state.vel + deltaVel;
                rigidBody.state.angVel = rigidBody.state.angVel + deltaAngVel;

                rigidBody.deltaVel = FmInitVector3(0.0f);
                rigidBody.deltaAngVel = FmInitVector3(0.0f);
            }
        }

        for (uint idx = 0; idx < numIslands; idx++)
        {
            uint islandIdx = islandIndices[idx];
            if (islandIdx >= rbFemScene->constraintsBuffer->numConstraintIslands)
            {
                continue;
            }

            FmConstraintSolverData& constraintSolverData = rbFemScene->constraintSolverBuffer->islandSolverData[islandIdx];

            FmShutdownConstraintSolve(&constraintSolverData);
        }
    }

    void ExampleNotifyIslandSleepingCB(FmScene* scene, void* userData, uint* rigidBodyIds, uint numRigidBodies)
    {
        (void)scene;
        ExampleRigidBodiesScene* rbScene = (ExampleRigidBodiesScene*)userData;
        FmScene* rbFemScene = rbScene->scene;

        FmCreateSleepingIsland(rbFemScene, NULL, 0, rigidBodyIds, numRigidBodies);
    }

    void ExampleNotifyIslandWakingCB(FmScene* scene, void* userData, uint* rigidBodyIds, uint numRigidBodies)
    {
        (void)scene;
        ExampleRigidBodiesScene* rbScene = (ExampleRigidBodiesScene*)userData;
        FmScene* rbFemScene = rbScene->scene;

        // Mark rigid body scene islands for waking at later stage
        for (uint rbIdx = 0; rbIdx < numRigidBodies; rbIdx++)
        {
            uint rbId = rigidBodyIds[rbIdx];
            FmMarkIslandOfObjectForWaking(rbFemScene, rbId);
        }
    }

    // Solve rigid body islands which were not part of FEM scene solve
    void SolveRemainingRbIslands(ExampleRigidBodiesScene* rbScene, FmScene* scene)
    {
        FmScene* rbFemScene = rbScene->scene;
        FmSceneControlParams& sceneParams = rbFemScene->params;

        uint numConstraintIslands = rbFemScene->constraintsBuffer->numConstraintIslands;
        for (uint islandIdx = 0; islandIdx < numConstraintIslands; islandIdx++)
        {
            FmConstraintIsland& constraintIsland = rbFemScene->constraintsBuffer->constraintIslands[islandIdx];
            FmConstraintSolverData& constraintSolverData = rbFemScene->constraintSolverBuffer->islandSolverData[islandIdx];

            if (!constraintSolverData.isAllocated)
            {
                continue;
            }

            // Only process island if not already solved by FEM system
            if (FM_IS_SET(constraintIsland.flags, FEM_SOLVED_ISLAND))
            {
                continue;
            }

            if (constraintIsland.numConstraints == 0)
            {
                FmTestSleepingAndUpdateStats(rbFemScene, constraintIsland);
                continue;
            }

            FmSetupConstraintSolve(rbFemScene, &constraintSolverData, &constraintIsland, rbFemScene->params.timestep);
            constraintIsland.flags |= FM_ISLAND_FLAG_SOLVE_IS_SETUP;

            // Full PGS solve
            constraintSolverData.solveParams.passParams[FM_GS_PASS_IDX].maxOuterIterations = 1;
            constraintSolverData.solveParams.passParams[FM_GS_PASS_IDX].maxInnerIterations = sceneParams.constraintSolveParams.passParams[FM_GS_PASS_IDX].maxInnerIterations;
            FmRunConstraintSolve(rbFemScene, &constraintSolverData, constraintIsland);

#if FM_CONSTRAINT_STABILIZATION_SOLVE
            FM_ASSERT(!FM_IS_SET(constraintIsland.flags, FM_ISLAND_FLAG_STABILIZATION_IS_SETUP));

            FmSetupConstraintStabilization(rbFemScene, &constraintSolverData, &constraintIsland, rbFemScene->params.timestep);
            constraintIsland.flags |= FM_ISLAND_FLAG_STABILIZATION_IS_SETUP;

            // Full PGS solve
            constraintSolverData.stabilizationParams.passParams[FM_GS_PASS_IDX].maxOuterIterations = 1;
            constraintSolverData.stabilizationParams.passParams[FM_GS_PASS_IDX].maxInnerIterations = sceneParams.constraintStabilizationParams.passParams[FM_GS_PASS_IDX].maxInnerIterations;
            FmRunConstraintSolve(rbFemScene, &constraintSolverData, constraintIsland);
#endif

            FmApplyConstraintSolveDeltasAndTestSleeping(rbFemScene, &constraintSolverData, constraintIsland);

            FmShutdownConstraintSolve(&constraintSolverData);
        }

        FmConstraintsBuffer& constraintsBuffer = *rbFemScene->constraintsBuffer;
        uint numIslands = constraintsBuffer.numConstraintIslands;

        // Create sleeping island in both rigid body and FEM system.
        for (uint idx = 0; idx < numIslands; idx++)
        {
            FmConstraintIsland& constraintIsland = constraintsBuffer.constraintIslands[idx];

            if (FM_IS_SET(constraintIsland.flags, FM_ISLAND_FLAG_MARKED_FOR_SLEEPING))
            {
                FmPutConstraintIslandToSleep(rbFemScene, constraintIsland, true, NULL);

                FmNotifyRigidBodiesSleeping(scene, constraintIsland.rigidBodyIds, constraintIsland.numRigidBodiesConnected);

                FmInitConstraintIsland(&constraintIsland);  // Clear source island
            }
        }
    }

    void UpdateSceneRb_Async(ExampleRigidBodiesScene* rbScene, FmScene* scene, float timestep);

    void UpdateSceneRb(ExampleRigidBodiesScene* rbScene, FmScene* scene, float timestep)
    {
#if FM_ASYNC_THREADING
        UpdateSceneRb_Async(rbScene, scene, timestep);
#else
        FM_TRACE_START_FRAME();
        FM_TRACE_START_EVENT(SCENE_UPDATE);

        double t0;
        FM_GET_TIME(t0);

        scene->islandSleepingCallback = ExampleNotifyIslandSleepingCB;
        scene->islandWakingCallback = ExampleNotifyIslandWakingCB;
        scene->rigidBodiesUserData = rbScene;

        FmScene* rbFemScene = rbScene->scene;
        FmSceneControlParams& sceneParams = scene->params;
        FmVector3 sceneGravityDeltaVel = sceneParams.gravityVector * timestep;
        float aabbPadding = sceneParams.distContactThreshold * 0.5f;

        // Wake FEM scene islands and calls ExampleNotifyIslandWakingCB on connected rigid bodies.
        // ExampleRigidBodiesScene will mark islands for the following call.
        // Assuming that changes to ExampleRigidBodiesScene objects were communicated to FEM lib already through UpdateRigidBodyState or NotifyRigidBodyWaking
        FmUpdateUnconstrained(scene, timestep, NULL, NULL);

        // Integrate rigid bodies and update collision structures
        FmUpdateUnconstrained(rbFemScene, timestep, NULL, NULL);

        uint numRigidBodies = rbFemScene->numAwakeRigidBodies;

        for (uint rbIdx = 0; rbIdx < numRigidBodies; rbIdx++)
        {
            FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*rbFemScene, rbFemScene->awakeRigidBodyIds[rbIdx]);

            bool isKinematic = FM_IS_SET(rigidBody.flags, FM_OBJECT_FLAG_KINEMATIC);

            if (!isKinematic)
            {
                // Integrate gravity force
                rigidBody.state.vel += sceneGravityDeltaVel + rigidBody.gravityVector * timestep;
            }

            // Update FmAabb

            FmBox box;
            box.halfWidths[0] = rigidBody.dims[0];
            box.halfWidths[1] = rigidBody.dims[1];
            box.halfWidths[2] = rigidBody.dims[2];
            box.state = rigidBody.state;

            rigidBody.aabb = FmComputeBoxAabb(box, timestep, aabbPadding);

            // Update tet mesh used for collision detection
            FmVector3 boxPos = rigidBody.state.pos;
            FmVector3 boxVel = rigidBody.state.vel;
            FmVector3 boxAngVel = rigidBody.state.angVel;
            FmMatrix3 boxRot = FmInitMatrix3(rigidBody.state.quat);
            float boxHalfX = rigidBody.dims[0];
            float boxHalfY = rigidBody.dims[1];
            float boxHalfZ = rigidBody.dims[2];
            FmVector3 boxCompX = boxRot.col0 * boxHalfX;
            FmVector3 boxCompY = boxRot.col1 * boxHalfY;
            FmVector3 boxCompZ = boxRot.col2 * boxHalfZ;

            FmTetMesh& rbTetMesh = *FmGetTetMesh(*(FmTetMeshBuffer*)rigidBody.collisionObj, 0);
            rbTetMesh.vertsPos[0] = boxPos - boxCompX - boxCompY - boxCompZ;
            rbTetMesh.vertsPos[1] = boxPos + boxCompX - boxCompY - boxCompZ;
            rbTetMesh.vertsPos[2] = boxPos - boxCompX + boxCompY - boxCompZ;
            rbTetMesh.vertsPos[3] = boxPos + boxCompX + boxCompY - boxCompZ;
            rbTetMesh.vertsPos[4] = boxPos - boxCompX - boxCompY + boxCompZ;
            rbTetMesh.vertsPos[5] = boxPos + boxCompX - boxCompY + boxCompZ;
            rbTetMesh.vertsPos[6] = boxPos - boxCompX + boxCompY + boxCompZ;
            rbTetMesh.vertsPos[7] = boxPos + boxCompX + boxCompY + boxCompZ;
            for (uint i = 0; i < 8; i++)
            {
                rbTetMesh.vertsVel[i] = boxVel + cross(boxAngVel, rbTetMesh.vertsPos[i]);

                if (isKinematic)
                {
                    rbTetMesh.vertsFlags[i] |= FM_VERT_FLAG_KINEMATIC;
                }
                else
                {
                    rbTetMesh.vertsFlags[i] &= ~FM_VERT_FLAG_KINEMATIC;
                }
            }

            // Update the tet mesh hierarchy
            FmBuildHierarchy(&rbTetMesh, timestep, aabbPadding);

            // Reset the delta state values which are used in the constraint solve
            rigidBody.deltaVel = FmInitVector3(0.0f);
            rigidBody.deltaAngVel = FmInitVector3(0.0f);
            rigidBody.deltaPos = FmInitVector3(0.0f);
            rigidBody.deltaAngPos = FmInitVector3(0.0f);

            FmSetAabb(FmGetRigidBody(*scene, rigidBody.objectId), rigidBody.aabb);
            FmSetState(scene, FmGetRigidBody(*scene, rigidBody.objectId), rigidBody.state);
        }

        scene->params.includeRigidBodiesInBroadPhase = true;

        // Find contacts between active objects, and between active and sleeping objects.

        // Find FEM only contacts. On contact with sleeping object, mark FEM island for waking.
        FmFindContacts(scene, NULL, NULL);

        // Find FEM/RB and RB/RB contacts. On contact RB scene will mark its islands for waking, and notify FEM scene to mark its islands.
        FindContacts(rbScene, scene);

        // Wake FEM islands and find new contacts.  Calls FmNotifyIslandWaking for RB scene to mark its islands connected to FEM islands.
        FmWakeCollidedIslandsAndFindContacts(scene, NULL, NULL);

        WakeCollidedIslandsAndFindContacts(rbScene, scene);

#if FM_DEBUG_MESHES
        for (uint meshIdx = 0; meshIdx < gFEMFXNumPreConstraintSolveMeshes; meshIdx++)
        {
            FmFreeTetMeshData(&gFEMFXPreConstraintSolveMeshes[meshIdx]);
        }
        delete[] gFEMFXPreConstraintSolveMeshes;
        gFEMFXPreConstraintSolveMeshes = NULL;
        delete[] gFEMFXPreConstraintSolveRigidBodies;
        gFEMFXPreConstraintSolveRigidBodies = NULL;

        uint numSceneTetMeshes = GetNumEnabledTetMeshes(*scene);
        uint numSceneRigidBodies = GetNumEnabledRigidBodies(*scene);
        gFEMFXPreConstraintSolveMeshes = new FmTetMesh[numSceneTetMeshes];
        gFEMFXPreConstraintSolveRigidBodies = new FmRigidBody[numSceneRigidBodies];
        gFEMFXNumPreConstraintSolveMeshes = numSceneTetMeshes;
        gFEMFXNumPreConstraintSolveRigidBodies = numSceneRigidBodies;

        for (uint meshIdx = 0; meshIdx < numSceneTetMeshes; meshIdx++)
        {
            FmTetMesh* tetMesh = FmGetTetMeshPtrById(*scene, GetEnabledTetMeshId(*scene, meshIdx));
            FmInitTetMesh(&gFEMFXPreConstraintSolveMeshes[meshIdx]);
            FmAllocTetMeshData(&gFEMFXPreConstraintSolveMeshes[meshIdx], *tetMesh);
            FmCopyTetMesh(&gFEMFXPreConstraintSolveMeshes[meshIdx], *tetMesh);
        }
        for (uint rigidBodyIdx = 0; rigidBodyIdx < numSceneRigidBodies; rigidBodyIdx++)
        {
            gFEMFXPreConstraintSolveRigidBodies[rigidBodyIdx] = *FmGetRigidBodyPtrById(*scene, GetEnabledRigidBodyId(*scene, rigidBodyIdx));
        }
#endif

        uint numRigidBodyConstraintIslands = FindConstraintIslands(rbScene, scene);

        FmUserConstraints userConstraints;
        userConstraints.distanceContactsPairInfo = rbScene->rigidFEMConstraints->distanceContactsPairInfo;
        userConstraints.distanceContacts = rbScene->rigidFEMConstraints->distanceContacts;
        userConstraints.volumeContacts = rbScene->rigidFEMConstraints->volumeContacts;
        userConstraints.volumeContactVerts = rbScene->rigidFEMConstraints->volumeContactVerts;
        userConstraints.glueConstraints = rbScene->rigidFEMConstraints->glueConstraints;
        userConstraints.planeConstraints = rbScene->rigidFEMConstraints->planeConstraints;
        userConstraints.rigidBodyAngleConstraints = rbScene->rigidFEMConstraints->rigidBodyAngleConstraints;
        userConstraints.numDistanceContacts = rbScene->rigidFEMConstraints->numDistanceContacts.val;
        userConstraints.numVolumeContacts = rbScene->rigidFEMConstraints->numVolumeContacts.val;
        userConstraints.numGlueConstraints = rbScene->rigidFEMConstraints->numGlueConstraints;
        userConstraints.numPlaneConstraints = rbScene->rigidFEMConstraints->numPlaneConstraints;
        userConstraints.numRigidBodyAngleConstraints = rbScene->rigidFEMConstraints->numRigidBodyAngleConstraints;
        userConstraints.numVolumeContactVerts = rbScene->rigidFEMConstraints->numVolumeContactVerts.val;
        userConstraints.numIslands = numRigidBodyConstraintIslands;
#if EXAMPLE_RB_ALL_CONSTRAINTS_IN_FEM_LIB
        userConstraints.innerIterationCallback = NULL;
#else
        userConstraints.innerIterationCallback = ExampleInnerSolveCB;
#endif
        userConstraints.islandCompletedCallback = ExampleIslandCompletedCB;
        userConstraints.userData = rbScene;

        FmFindConstraintIslands(scene, &userConstraints);

        FmSceneConstraintSolve(scene, NULL, NULL);

        SolveRemainingRbIslands(rbScene, scene);

        for (uint rbIdx = 0; rbIdx < numRigidBodies; rbIdx++)
        {
            FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*rbFemScene, rbFemScene->awakeRigidBodyIds[rbIdx]);

            if (FM_NOT_SET(rigidBody.flags, FM_OBJECT_FLAG_SLEEPING))
            {
                FmStepState(rigidBody.state, timestep);
                FmSetState(scene, FmGetRigidBody(*scene, rigidBody.objectId), rigidBody.state);
            }
        }

        FM_TRACE_STOP_EVENT(SCENE_UPDATE);
        FM_TRACE_STOP_FRAME();
        FM_DISABLE_TRACE();

        FM_SET_TOTAL_STEP_TIME(t0);

        scene->params.simTime += timestep;
#endif
    }

#if FM_ASYNC_THREADING
    void TaskFuncUpdateSceneRbPart2(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    void TaskFuncUpdateSceneRbStartAux(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        TaskDataUpdateSceneRb* taskData = (TaskDataUpdateSceneRb*)inTaskData;

        ExampleRigidBodiesScene* rbScene = taskData->rbScene;
        FmScene* scene = taskData->scene;
        float timestep = taskData->timestep;

        scene->islandSleepingCallback = ExampleNotifyIslandSleepingCB;
        scene->islandWakingCallback = ExampleNotifyIslandWakingCB;
        scene->rigidBodiesUserData = rbScene;

        FmSceneControlParams& sceneParams = scene->params;
        FmVector3 sceneGravityDeltaVel = sceneParams.gravityVector * timestep;

        // Wake FEM scene islands and calls ExampleNotifyIslandWakingCB on connected rigid bodies.
        // ExampleRigidBodiesScene will mark islands for the following call.
        // Assuming that changes to ExampleRigidBodiesScene objects were communicated to FEM lib already through UpdateRigidBodyState or NotifyRigidBodyWaking

        FmUpdateUnconstrained(scene, timestep, TaskFuncUpdateSceneRbPart2, taskData);
    }

    void TaskFuncUpdateSceneRbStart(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        FmExecuteTask(TaskFuncUpdateSceneRbStartAux, inTaskData, inTaskBeginIndex, inTaskEndIndex);
    }

    void TaskFuncUpdateSceneRbPart3(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    void TaskFuncUpdateSceneRbPart2(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        TaskDataUpdateSceneRb* taskData = (TaskDataUpdateSceneRb*)inTaskData;

        ExampleRigidBodiesScene* rbScene = taskData->rbScene;
        FmScene* scene = taskData->scene;
        float timestep = taskData->timestep;

        // Measure time for UpdateUnconstrained()
        FM_SET_IMPLICIT_STEP_TIME();

        FmScene* rbFemScene = rbScene->scene;
        FmSceneControlParams& sceneParams = scene->params;
        FmVector3 sceneGravityDeltaVel = sceneParams.gravityVector * timestep;
        float aabbPadding = sceneParams.distContactThreshold * 0.5f;

        // Integrate rigid bodies and update collision structures
        FmUpdateUnconstrained(rbFemScene, timestep, NULL, NULL);

        uint numRigidBodies = rbFemScene->numAwakeRigidBodies;

        for (uint rbIdx = 0; rbIdx < numRigidBodies; rbIdx++)
        {
            FmRigidBody* pRigidBody = FmGetRigidBodyPtrById(*rbFemScene, rbFemScene->awakeRigidBodyIds[rbIdx]);

            FM_ASSERT(pRigidBody != NULL);

            FmRigidBody& rigidBody = *pRigidBody;

            bool isKinematic = FM_IS_SET(rigidBody.flags, FM_OBJECT_FLAG_KINEMATIC);

            if (!isKinematic)
            {
                // Integrate gravity force
                rigidBody.state.vel += sceneGravityDeltaVel + rigidBody.gravityVector * timestep;
            }

            // Update FmAabb

            FmBox box;
            box.halfWidths[0] = rigidBody.dims[0];
            box.halfWidths[1] = rigidBody.dims[1];
            box.halfWidths[2] = rigidBody.dims[2];
            box.state = rigidBody.state;

            rigidBody.aabb = FmComputeBoxAabb(box, timestep, aabbPadding);

            // Update tet mesh used for collision detection
            FmVector3 boxPos = rigidBody.state.pos;
            FmVector3 boxVel = rigidBody.state.vel;
            FmVector3 boxAngVel = rigidBody.state.angVel;
            FmMatrix3 boxRot = FmInitMatrix3(rigidBody.state.quat);

            if (rigidBody.collisionObj)
            {
                FmTetMesh& rbTetMesh = *FmGetTetMesh(*(FmTetMeshBuffer*)rigidBody.collisionObj, 0);
                uint numVerts = rbTetMesh.numVerts;
                for (uint vId = 0; vId < numVerts; vId++)
                {
                    rbTetMesh.vertsPos[vId] = mul(boxRot, rbTetMesh.vertsRestPos[vId]) + boxPos;
                    rbTetMesh.vertsVel[vId] = boxVel + cross(boxAngVel, rbTetMesh.vertsPos[vId]);
                    if (isKinematic)
                    {
                        rbTetMesh.vertsFlags[vId] |= FM_VERT_FLAG_KINEMATIC;
                    }
                    else
                    {
                        rbTetMesh.vertsFlags[vId] &= ~FM_VERT_FLAG_KINEMATIC;
                    }
                }

                rbTetMesh.minPosition = rigidBody.aabb.pmin;
                rbTetMesh.maxPosition = rigidBody.aabb.pmax;

                // Update the tet mesh hierarchy
                FmBuildHierarchy(&rbTetMesh, timestep, aabbPadding);
            }

            // Reset the delta state values which are used in the constraint solve
            rigidBody.deltaVel = FmInitVector3(0.0f);
            rigidBody.deltaAngVel = FmInitVector3(0.0f);
            rigidBody.deltaPos = FmInitVector3(0.0f);
            rigidBody.deltaAngPos = FmInitVector3(0.0f);

            FmSetAabb(FmGetRigidBody(*scene, rigidBody.objectId), rigidBody.aabb);
            FmSetState(scene, FmGetRigidBody(*scene, rigidBody.objectId), rigidBody.state);
        }

        scene->params.includeRigidBodiesInBroadPhase = true;

        // Find contacts between active objects, and between active and sleeping objects.

        // Find FEM only contacts. On contact with sleeping object, mark FEM island for waking.
        FmFindContacts(scene, TaskFuncUpdateSceneRbPart3, taskData);
    }

    void TaskFuncUpdateSceneRbPart4(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    void TaskFuncUpdateSceneRbPart3(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        TaskDataUpdateSceneRb* taskData = (TaskDataUpdateSceneRb*)inTaskData;

        ExampleRigidBodiesScene* rbScene = taskData->rbScene;
        FmScene* scene = taskData->scene;

        // Measure time for FindContacts()
        FM_ADD_MESH_CONTACTS_TIME();

        // Find FEM/RB and RB/RB contacts. On contact RB scene will mark its islands for waking, and notify FEM scene to mark its islands.
        FindContacts(rbScene, scene);

        // Wake FEM islands and find new contacts.  Calls FmNotifyIslandWaking for RB scene to mark its islands connected to FEM islands.
        FmWakeCollidedIslandsAndFindContacts(scene, TaskFuncUpdateSceneRbPart4, taskData);
    }

    void TaskFuncUpdateSceneRbPart5(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    void TaskFuncUpdateSceneRbPart4(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        TaskDataUpdateSceneRb* taskData = (TaskDataUpdateSceneRb*)inTaskData;

        ExampleRigidBodiesScene* rbScene = taskData->rbScene;
        FmScene* scene = taskData->scene;

        WakeCollidedIslandsAndFindContacts(rbScene, scene);

#if FM_DEBUG_MESHES
        for (uint meshIdx = 0; meshIdx < gFEMFXNumPreConstraintSolveMeshes; meshIdx++)
        {
            FmFreeTetMeshData(&gFEMFXPreConstraintSolveMeshes[meshIdx]);
        }
        delete[] gFEMFXPreConstraintSolveMeshes;
        gFEMFXPreConstraintSolveMeshes = NULL;
        delete[] gFEMFXPreConstraintSolveRigidBodies;
        gFEMFXPreConstraintSolveRigidBodies = NULL;

        uint numSceneTetMeshes = GetNumEnabledTetMeshes(*scene);
        uint numSceneRigidBodies = GetNumEnabledRigidBodies(*scene);
        gFEMFXPreConstraintSolveMeshes = new FmTetMesh[numSceneTetMeshes];
        gFEMFXPreConstraintSolveRigidBodies = new FmRigidBody[numSceneRigidBodies];
        gFEMFXNumPreConstraintSolveMeshes = numSceneTetMeshes;
        gFEMFXNumPreConstraintSolveRigidBodies = numSceneRigidBodies;

        for (uint meshIdx = 0; meshIdx < numSceneTetMeshes; meshIdx++)
        {
            FmTetMesh* tetMesh = FmGetTetMeshPtrById(*scene, GetEnabledTetMeshId(*scene, meshIdx));
            FmInitTetMesh(&gFEMFXPreConstraintSolveMeshes[meshIdx]);
            FmAllocTetMeshData(&gFEMFXPreConstraintSolveMeshes[meshIdx], *tetMesh);
            FmCopyTetMesh(&gFEMFXPreConstraintSolveMeshes[meshIdx], *tetMesh);
        }
        for (uint rigidBodyIdx = 0; rigidBodyIdx < numSceneRigidBodies; rigidBodyIdx++)
        {
            gFEMFXPreConstraintSolveRigidBodies[rigidBodyIdx] = *FmGetRigidBodyPtrById(*scene, GetEnabledRigidBodyId(*scene, rigidBodyIdx));
        }
#endif

        uint numRigidBodyConstraintIslands = FindConstraintIslands(rbScene, scene);

        FmUserConstraints userConstraints;
        userConstraints.distanceContactsPairInfo = rbScene->rigidFEMConstraints->distanceContactsPairInfo;
        userConstraints.distanceContacts = rbScene->rigidFEMConstraints->distanceContacts;
        userConstraints.volumeContacts = rbScene->rigidFEMConstraints->volumeContacts;
        userConstraints.volumeContactVerts = rbScene->rigidFEMConstraints->volumeContactVerts;
        userConstraints.glueConstraints = rbScene->rigidFEMConstraints->glueConstraints;
        userConstraints.planeConstraints = rbScene->rigidFEMConstraints->planeConstraints;
        userConstraints.rigidBodyAngleConstraints = rbScene->rigidFEMConstraints->rigidBodyAngleConstraints;
        userConstraints.numDistanceContacts = FmAtomicRead(&rbScene->rigidFEMConstraints->numDistanceContacts.val);
        userConstraints.numVolumeContacts = FmAtomicRead(&rbScene->rigidFEMConstraints->numVolumeContacts.val);
        userConstraints.numGlueConstraints = rbScene->rigidFEMConstraints->numGlueConstraints;
        userConstraints.numPlaneConstraints = rbScene->rigidFEMConstraints->numPlaneConstraints;
        userConstraints.numRigidBodyAngleConstraints = rbScene->rigidFEMConstraints->numRigidBodyAngleConstraints;
        userConstraints.numVolumeContactVerts = FmAtomicRead(&rbScene->rigidFEMConstraints->numVolumeContactVerts.val);
        userConstraints.numIslands = numRigidBodyConstraintIslands;
#if EXAMPLE_RB_ALL_CONSTRAINTS_IN_FEM_LIB
        userConstraints.innerIterationCallback = NULL;
#else
        userConstraints.innerIterationCallback = ExampleInnerSolveCB;
#endif
        userConstraints.islandCompletedCallback = ExampleIslandCompletedCB;
        userConstraints.userData = rbScene;

        FmFindConstraintIslands(scene, &userConstraints);

        FmSceneConstraintSolve(scene, TaskFuncUpdateSceneRbPart5, taskData);
    }

    void TaskFuncUpdateSceneRbPart5(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        TaskDataUpdateSceneRb* taskData = (TaskDataUpdateSceneRb*)inTaskData;

        ExampleRigidBodiesScene* rbScene = taskData->rbScene;
        FmScene* scene = taskData->scene;
        float timestep = taskData->timestep;

        SolveRemainingRbIslands(rbScene, scene);

        FmScene* rbFemScene = rbScene->scene;
        uint numRigidBodies = rbFemScene->numAwakeRigidBodies;

        for (uint rbIdx = 0; rbIdx < numRigidBodies; rbIdx++)
        {
            FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*rbFemScene, rbFemScene->awakeRigidBodyIds[rbIdx]);

            if (FM_NOT_SET(rigidBody.flags, FM_OBJECT_FLAG_SLEEPING))
            {
                FmStepState(rigidBody.state, timestep);
                FmSetState(scene, FmGetRigidBody(*scene, rigidBody.objectId), rigidBody.state);
            }
        }

        delete taskData;

        if (scene->postUpdateSceneCallback)
        {
            scene->postUpdateSceneCallback(scene->postUpdateSceneData, 0, 1);
        }
    }

    struct UpdateSceneRbFinishedData
    {
        FmSyncEvent* taskEvent;
        FmScene* scene;

        UpdateSceneRbFinishedData(FmSyncEvent* inTaskEvent, FmScene* inScene)
        {
            taskEvent = inTaskEvent;
            scene = inScene;
        }
    };

    static void TaskFuncUpdateSceneRbFinished(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        UpdateSceneRbFinishedData* taskData = (UpdateSceneRbFinishedData*)inTaskData;
        taskData->scene->taskSystemCallbacks.TriggerSyncEvent(taskData->taskEvent);
    }

    void UpdateSceneRb_Async(ExampleRigidBodiesScene* rbScene, FmScene* scene, float timestep)
    {
        FmSyncEvent* updateSceneFinishedEvent = scene->taskSystemCallbacks.CreateSyncEvent();

        FM_TRACE_START_FRAME();
        FM_TRACE_START_EVENT(SCENE_UPDATE);

        double t0;
        FM_GET_TIME(t0);

        UpdateSceneRbFinishedData sceneUpdateFinishedData(updateSceneFinishedEvent, scene);
        scene->postUpdateSceneCallback = TaskFuncUpdateSceneRbFinished;
        scene->postUpdateSceneData = &sceneUpdateFinishedData;

        TaskDataUpdateSceneRb* taskData = new TaskDataUpdateSceneRb(rbScene, scene, timestep);

        scene->taskSystemCallbacks.SubmitAsyncTask("TaskFuncUpdateSceneRbStart", TaskFuncUpdateSceneRbStart, taskData, 0, 1);

        scene->taskSystemCallbacks.WaitForSyncEvent(updateSceneFinishedEvent);

        scene->taskSystemCallbacks.DestroySyncEvent(updateSceneFinishedEvent);

        FM_TRACE_STOP_EVENT(SCENE_UPDATE);
        FM_TRACE_STOP_FRAME();
        FM_DISABLE_TRACE();

        FM_SET_TOTAL_STEP_TIME(t0);

        scene->params.simTime += timestep;
    }
#endif
}