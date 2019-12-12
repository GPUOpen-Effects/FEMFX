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

#ifndef _EXAMPLE_RIGID_BODIES_H_
#define _EXAMPLE_RIGID_BODIES_H_

#include "FEMFXInternal.h"
#include "FEMFXBoxCcd.h"

// Example rigid body system to demonstrate the FEM library API for external objects.  This is a
// pretty minimal system supporting boxes only.  It leverages some of the components of the FEM
// library for collision detection and creation of volume contacts.  The intention is for this to be
// replaced with a preferred rigid body engine.  
//
// The concept for combining systems is:
// - FEM/rigid collision detection and contact creation is performed outside the FEMFX library.
// - FEM/rigid contacts/constraints are registered with the FEMFX library prior to finding 
//   constraint islands.
// - The user can interleave passes of constraint solving by the FEMFX library and external rigid
//   body system.
// - After each FEMFX solve pass, a callback allows the external system to read current rigid body 
//   state change values and iterate through its own constraints.
// - The FEM library calls a callback when the island solve is complete in order to read final
//   rigid body state.
//
// Supplying the constraints to the library is done by setting a struct with pointers to externally 
// created arrays:
//
//    struct FmUserConstraints
//    {
//        FmDistanceContactPairInfo*   distanceContactsPairInfo;
//        FmDistanceContact*           distanceContacts;
//        FmVolumeContact*             volumeContacts;
//        FmVolumeContactVert*         volumeContactVerts;
//        FmGlueConstraint*            glueConstraints;
//        FmPlaneConstraint*           planeConstraints;
//        FmRigidBodyAngleConstraint*  rigidBodyAngleConstraints;
//        uint                         numDistanceContacts;
//        uint                         numVolumeContacts;
//        uint                         numVolumeContactVerts;
//        uint                         numGlueConstraints;
//        uint                         numPlaneConstraints;
//        uint                         numRigidBodyAngleConstraints;
//        uint                         numIslands; // Number of islands found by rigid body system; the rigid body islandId values are expected to be in range 0..numIslands-1.
//    
//        FmCallbackConstraintSolveInnerIteration  innerIterationCallback;
//        FmCallbackConstraintSolveIslandCompleted islandCompletedCallback;
//        void* userData;
//    };
//
// This is passed into the FmFindConstraintIslands() call:
//
//    // Find islands of objects (FEM meshes and supplied rigid bodies) which are connected by
//    // constraints.  Each island must be solved as a unit during constraint solving.  In order to
//    // also take into account constraints which are external to the library, this function also
//    // reads the rigid body island ids set in FmRigidBody::islandId and will group objects
//    // that belong to or touch the same rigid body island. 
//    void FmFindConstraintIslands(FmScene* scene, 
//        FmUserConstraints* userConstraints);
//
// The innerIterationCallback is called on every inner loop of the FEMFX constraint solver:
//
//    // Callback from the constraint solver to allow interleaving of external rigid body 
//    // constraints.  If provided the solver will call inside an inner iteration of a constraint
//    // island.  "islandIds" are all the FmRigidBody::islandId values that are connected to
//    // the FEM constraint island by constraints.  The external solver should solve constraints 
//    // within all these islands.  "rigidBodyIds" are ids of the rigid bodies that directly share
//    // constraints with tet meshes.  The callback should also set FmSolverIterationNorms values
//    // computed for the external constraints.
//    typedef void(*FmCallbackConstraintSolveInnerIteration) (
//        FmSolverIterationNorms* solverNorms,
//        void* userData, uint* rigidBodyIds, uint numRigidBodies, uint* islandIds, uint numIslands, bool isStabilization);
//
// The constraint solver solution for the current iteration can be found in the array of
// FmRigidBody structs included with FmUserConstraints.  The library sets these
// struct members:
//
//     FmVector3 deltaVel;     // on velocity solving pass
//     FmVector3 deltaAngVel;
//     FmVector3 deltaPos;     // on stabilization pass
//     FmVector3 deltaAngPos;
//
// These solution variables are current change in rigid body state (response) resulting from the
// current solution of constraint multipliers (lambdas).  In the constraint LCP terms this is 
// equal to inverse(mass_matrix) * transpose(jacobian) * lambda for the current lambda estimate.
//
// In the callback this response can then be changed by the user as changes are made to external 
// constraint multipliers.  Following the callback, the updated response is read by the FEMFX 
// solve.
//
// The islandCompletedCallback is called after the FEM library completes the shared solve, and 
// has applied the final deltaVel, deltaAngVel, deltaPos, deltaAngPos to the state of the rigid 
// bodies in its island (those which touch an FEM mesh by a constraint).

#define EXAMPLE_RB_ALL_CONSTRAINTS_IN_FEM_LIB 1

namespace AMD
{

    struct ExampleRigidBodiesSceneSetupParams
    {
        struct ConstraintsSetupParams
        {
            uint maxDistanceContacts;
            uint maxFractureContacts;
            uint maxVolumeContacts;
            uint maxVolumeContactVerts;
            uint maxGlueConstraints;
            uint maxPlaneConstraints;
            uint maxRigidBodyAngleConstraints;
            uint maxBroadPhasePairs;

            ConstraintsSetupParams()
            {
                maxDistanceContacts = 0;
                maxFractureContacts = 0;
                maxVolumeContacts = 0;
                maxVolumeContactVerts = 0;
                maxGlueConstraints = 0;
                maxPlaneConstraints = 0;
                maxRigidBodyAngleConstraints = 0;
                maxBroadPhasePairs = 0;
            }
        };
        ConstraintsSetupParams rigidRigidConstraintsSetupParams;
        ConstraintsSetupParams rigidFEMConstraintsSetupParams;

        uint maxRigidBodies;
        uint maxObjectPairTriPairs;         // Max number of tri pairs stored in temporary memory during mesh collision; if max reached, mesh collision will create contacts and clear buffer
        uint maxObjectPairVolContactVerts;  // Allocate enough temporary memory to support this many verts per object pair
        uint maxJacobianSubmats;            // Limit on total Jacobian submatrices: max of 6 per distance contact
        uint numWorkerThreads;

        ExampleRigidBodiesSceneSetupParams()
        {
            maxRigidBodies = 0;
            maxObjectPairTriPairs = 0;
            maxObjectPairVolContactVerts = 0;
            maxJacobianSubmats = 0;
            numWorkerThreads = 1;
        }
    };

    // An example (minimal) rigid body system which is just to illustrate interfacing with the FEM library.
    // It is actually built mostly of parts of the FEM library, which already includes rigid body state and 
    // supports some contacts/constraints as well as broad phase pairs with rigid bodies.  However the FEM 
    // library does not include mid or narrow phase collision detection with rigid bodies (a potentially 
    // large set of representations and tests).  In any case, this is meant to test the concept of combining 
    // the constraint solve between two scenes.
    struct ExampleRigidBodiesScene
    {
        FmScene*             scene;                // A FEMFX scene containing just rigid bodies
        FmConstraintsBuffer* rigidFEMConstraints;  // Constraints passed to the FEM library for solving

        ExampleRigidBodiesScene(const ExampleRigidBodiesSceneSetupParams& setupParams);
        ~ExampleRigidBodiesScene();
    };

    // Add a rigid body described by inRigidBody to both the rigid-body and FEM scenes.
    // Initializes tet mesh used for collision detection.
    // Returns id of object in FEM scene
    uint AddRigidBodyToScenes(ExampleRigidBodiesScene* rbScene, FmScene* femScene, FmRigidBody* inRigidBody);

    // Remove a rigid body from both rigid-body and FEM scenes, using id of object in FEM scene
    void RemoveRigidBodyFromScenes(ExampleRigidBodiesScene* rbScene, FmScene* femScene, uint rigidBodyId);

    FmRigidBody* GetRigidBody(const ExampleRigidBodiesScene& rbScene, uint rigidBodyId);

    // Make a change to the state of a rigid body.
    // Will cause waking of the body in both scenes.
    void UpdateRigidBodyState(ExampleRigidBodiesScene* rbScene, FmScene* femScene, uint rigidBodyId, const FmRigidBodyState& state);

    // Make a change to gravity vector of a rigid body.
    // Will cause waking of the body in both scenes.
    void SetRigidBodyGravityVector(ExampleRigidBodiesScene* rbScene, FmScene* femScene, uint rigidBodyId, const FmVector3& gravityVector);

    // Choose whether to enable rigid body simulation
    void EnableRigidBodySimulation(ExampleRigidBodiesScene* rbScene, FmScene* femScene, uint rigidBodyId, bool enable);

    void UpdateSceneRb(ExampleRigidBodiesScene* rbScene, FmScene* scene, float timestep);

    // To manually control synchronization of scene update, see UpdateSceneRb() implementation.
    // Must register a post-scene callback, create a TaskDataUpdateSceneRB with new, then 
    // use the FmSubmitTaskCallback to submit TaskFuncUpdateSceneRbStart with this data.
    class TaskDataUpdateSceneRb
    {
    public:
        FM_CLASS_NEW_DELETE(TaskDataUpdateSceneRb)

        ExampleRigidBodiesScene* rbScene;
        FmScene* scene;
        float timestep;

        TaskDataUpdateSceneRb(ExampleRigidBodiesScene* inRbScene, FmScene* inScene, float inTimestep)
        {
            rbScene = inRbScene;
            scene = inScene;
            timestep = inTimestep;
        }
    };

    void TaskFuncUpdateSceneRbStart(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);
}

#endif
