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

#pragma once

#include "FEMFXBvh.h"
#include "FEMFXTaskSystemInterface.h"
#include "FEMFXFindContacts.h"

#define FM_BROAD_PHASE_COLLIDE_OBJECTS_BATCH_SIZE 64
#define FM_BROAD_PHASE_PAIRS_SORT_BATCH_SIZE      512

#define FM_PARALLEL_BROAD_PHASE_TRAVERSAL  (1 && FM_ASYNC_THREADING)

namespace AMD
{
    struct FmScene;

    // Object pair found in broad phase
    struct FmBroadPhasePair
    {
        uint objectIdA;
        uint objectIdB;
        uint weight;     // weigh by estimated cost for prioritizing
    };

    void FmBuildBroadPhaseBvh(
        FmBvh* resultBvh, FmVector3* worldMinPosition, FmVector3* worldMaxPosition,
        FmScene* scene, const uint* tetMeshIds, uint numTetMeshes, uint* rigidBodyIds, uint numRigidBodies);

    void FmCollideBroadPhaseBvhPairCallback(void* userData, uint primIdA, uint primIdB);

    void FmBroadPhase(FmTaskDataFindContacts* findContactsData, FmTaskFuncCallback followTaskFunc, void* followTaskData);
}