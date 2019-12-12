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
#include "FEMFXAsyncThreading.h"

namespace AMD
{
    struct FmScene;
    struct FmConstraintSolverData;
    struct FmConstraintIsland;
    struct FmTetMesh;

    class FmTaskDataApplySolveDeltasAndTestSleeping : public FmAsyncTaskData
    {
    public:
        FM_CLASS_NEW_DELETE(FmTaskDataApplySolveDeltasAndTestSleeping)

        FmScene* scene;
        FmConstraintSolverData* constraintSolverData;
        FmConstraintIsland* constraintIsland;

        uint numTetMeshes;
        uint numRigidBodies;
        uint numRigidBodyTasks;

        float timestep;
        bool rigidBodiesExternal;

        FmTaskDataApplySolveDeltasAndTestSleeping(
            FmScene* inScene, FmConstraintSolverData* inConstraintSolverData, FmConstraintIsland* inConstraintIsland,
            uint inNumTetMeshes,
            uint inNumRigidBodies, uint inNumRigidBodyTasks, float inTimestep, bool inRigidBodiesExternal)
        {
            scene = inScene;
            constraintSolverData = inConstraintSolverData;
            constraintIsland = inConstraintIsland;

            numTetMeshes = inNumTetMeshes;
            numRigidBodies = inNumRigidBodies;
            numRigidBodyTasks = inNumRigidBodyTasks;

            timestep = inTimestep;
            rigidBodiesExternal = inRigidBodiesExternal;
        }
    };

    // Compute tet rotation and rotated stiffness matrix.
    // Compute plastic deformation state change.
    // Create list of tets with sufficient stress for fracture.
    // Set mesh's unconstrained solver iterations as max of tet settings.
    void FmUpdateTetStateAndFracture(FmScene* scene, FmTetMesh* tetMesh,
        bool testFracture = true, bool updatePlasticity = true,
        FmAsyncTaskData* parentTaskData = NULL);
}