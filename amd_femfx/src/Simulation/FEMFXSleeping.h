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

#pragma once

#include "FEMFXCommonInternal.h"
#include "FEMFXAsyncThreading.h"

namespace AMD
{
    struct FmScene;
    class FmAsyncTasksProgress;
    struct FmConstraintIsland;

    // Sets flag in an object's sleeping island for a later pass waking objects; tests sleeping status
    bool FmMarkIslandOfObjectForWaking(FmScene* scene, uint objectId);

    // Sets flag in an object's active island for a later pass which will create the sleeping island
    void FmMarkIslandForSleeping(FmScene* scene, uint sleepingIslandId);

    // Complete waking of marked islands which wakes objects and calls a callback for an external engine to update its objects.
    // This does not create new active islands; these are regenerated after new contacts are found.
    void FmWakeMarkedIslands(FmScene* scene);

    // Create sleeping islands for all marked active islands.
    // Clears the original island but leaves its id arrays in place.  Active islands are regenerated each frame.
    // If FM_ASYNC_THREADING and parentTaskData is non-NULL, this will execute asynchronously and may return before
    // tasks are complete.
    void FmPutMarkedIslandsToSleep(FmScene* scene, FmAsyncTaskData* parentTaskData);

    bool FmPutConstraintIslandToSleep(FmScene* scene, const FmConstraintIsland& srcIsland, bool useCallback, FmAsyncTaskData* parentTaskData);

}