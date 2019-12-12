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

#include "FEMFXCommonInternal.h"

namespace AMD
{
    struct FmScene;
    struct FmSolverIterationNorms;
    struct FmDistanceContactPairInfo;
    struct FmDistanceContact;
    struct FmVolumeContact;
    struct FmVolumeContactVert;
    struct FmGlueConstraint;
    struct FmPlaneConstraint;
    struct FmRigidBodyAngleConstraint;

    // Callback from the constraint solver to allow interleaving of external rigid body 
    // constraints.  If provided the solver will call inside an inner iteration of a constraint
    // island.  "islandIndices" are all the FmRigidBody::rbIslandId values that are connected to
    // the FEM constraint island by constraints.  The external solver should solve constraints 
    // within all these islands.  "rigidBodyIds" are the rigid bodies that directly share
    // constraints with tet meshes.  The callback should also set FmSolverIterationNorms values
    // computed for the external constraints.
    typedef void(*FmCallbackConstraintSolveInnerIteration) (
        FmScene* scene,
        FmSolverIterationNorms* solverNorms,
        void* userData, uint* rigidBodyIds, uint numRigidBodiesInFEMSolve, uint* islandIndices, uint numIslands, bool isStabilization);

    // Called after an island solve is completed, so that changes to rigid body state can be copied.
    typedef void(*FmCallbackConstraintSolveIslandCompleted) (
        FmScene* scene, void* userData, uint* rigidBodyIds, uint numRigidBodiesInFEMSolve, uint* islandIndices, uint numIslands);

    // Structure for passing user constraints and callbacks into FmFindConstraintIslands
    struct FmUserConstraints
    {
        FmDistanceContactPairInfo*    distanceContactsPairInfo;
        FmDistanceContact*            distanceContacts;
        FmVolumeContact*              volumeContacts;
        FmVolumeContactVert*          volumeContactVerts;
        FmGlueConstraint*             glueConstraints;
        FmPlaneConstraint*            planeConstraints;
        FmRigidBodyAngleConstraint*   rigidBodyAngleConstraints;
        uint                          numDistanceContacts;
        uint                          numVolumeContacts;
        uint                          numVolumeContactVerts;
        uint                          numGlueConstraints;
        uint                          numPlaneConstraints;
        uint                          numRigidBodyAngleConstraints;
        uint                          numIslands; // Number of islands found by rigid body system; the rigid body rbIslandId values are expected to be in range 0..numIslands-1.

        FmCallbackConstraintSolveInnerIteration  innerIterationCallback;
        FmCallbackConstraintSolveIslandCompleted islandCompletedCallback;
        void* userData;

        FmUserConstraints()
        {
            distanceContactsPairInfo = NULL;
            distanceContacts = NULL;
            volumeContacts = NULL;
            volumeContactVerts = NULL;
            glueConstraints = NULL;
            planeConstraints = NULL;
            rigidBodyAngleConstraints = NULL;
            numDistanceContacts = 0;
            numVolumeContacts = 0;
            numVolumeContactVerts = 0;
            numGlueConstraints = 0;
            numPlaneConstraints = 0;
            numRigidBodyAngleConstraints = 0;
            numIslands = 0;

            innerIterationCallback = NULL;
            islandCompletedCallback = NULL;
            userData = NULL;
        }
    };

}