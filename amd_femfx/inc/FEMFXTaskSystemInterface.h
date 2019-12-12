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
// FEMFX library's interface to an external task scheduler
//---------------------------------------------------------------------------------------

#pragma once

#include <stdint.h>

// Async threading dispatches work that detects completion and submits follow-up tasks.
// Avoids possibility that waiting thread will stall execution, and simplifies task system interface.
//
// NOTE: This is the only supported option.  The previous blocking approach is still included in the source,
// being somewhat easier to read and understand, and because this was helpful for development.  But future 
// releases may remove the non-async path.

#define FM_ASYNC_THREADING              1

namespace AMD
{
    // Get current number of worker threads
    typedef int(*FmGetTaskSystemNumThreadsCallback)();

    // Return index of current worker thread, >= 0 and < task system numThreads, unique between workers running concurrently.
    typedef int(*FmGetTaskSystemWorkerIndex)();

    // Prototype for a FEMFX task function
    typedef void(*FmTaskFuncCallback)(void* taskData, int32_t taskBeginIndex, int32_t taskEndIndex);

    // Submit asynchronous task to task scheduler, which should run FmExecuteTask with TaskFunc, taskData and taskIndex arguments
    typedef void(*FmSubmitAsyncTaskCallback)(const char* taskName, FmTaskFuncCallback TaskFunc, void* taskData, int32_t taskBeginIndex, int32_t taskEndIndex);

    // Lib holds a void* to external object representing an event which can be waited on until triggered by a task
    typedef void FmSyncEvent;

    // Create task event
    typedef FmSyncEvent* (*FmCreateSyncEventCallback)();

    // Destroy task event
    typedef void(*FmDestroySyncEventCallback)(FmSyncEvent* taskEvent);

    // Wait for task event to be triggered
    typedef void(*FmWaitForSyncEventCallback)(FmSyncEvent* taskEvent);

    // Trigger task event
    typedef void(*FmTriggerSyncEventCallback)(FmSyncEvent* taskEvent);

#if !FM_ASYNC_THREADING
    // Lib holds a void* to external object representing a synchronization primitive,
    // which holds number of work items currently being waited for.
    typedef void FmTaskWaitCounter;

    // Create counter at default unblocked value
    typedef FmTaskWaitCounter* (*FmCreateTaskWaitCounterCallback)();

    // Wait on counter, and offer thread for task processing.
    // NOTE: The library assumes no more than FmSceneSetupParams::numWorkerThreads is processing tasks simultaneously, and only allocates thread temporary memory for that number of threads.
    typedef void(*FmWaitForTaskWaitCounterCallback)(FmTaskWaitCounter* counter);

    // Destroy counter
    typedef void(*FmDestroyTaskWaitCounterCallback)(FmTaskWaitCounter* counter);

    // Submit task to task scheduler, which should run TaskFunc with taskData and taskIndex arguments.
    // If waitCounter non-NULL, this call must increment counter, and task must decrement on completion.
    // NOTE: For async threading to work, the implementation must execute each task by passing the task to FmExecuteTask().
    typedef void(*FmSubmitTaskCallback)(const char* taskName, FmTaskFuncCallback TaskFunc, void* taskData, int32_t taskBeginIndex, int32_t taskEndIndex, FmTaskWaitCounter* waitCounter);

    // Run an array of tasks in parallel and wait for all to complete.
    // Makes taskCount calls to TaskFunc, passing it taskData and a unique taskIndex in range 0..taskCount-1
    // NOTE: For async threading to work, the implementation must execute each task by passing the task to FmExecuteTask().
    typedef void(*FmParallelForCallback)(const char* taskName, FmTaskFuncCallback TaskFunc, void* taskData, int32_t taskCount);
#endif

    // Set of callbacks that define the interface to external scheduler.
    struct FmTaskSystemCallbacks
    {
        FmGetTaskSystemNumThreadsCallback GetTaskSystemNumThreads;
        FmGetTaskSystemWorkerIndex GetTaskSystemWorkerIndex;
        FmSubmitAsyncTaskCallback SubmitAsyncTask;
        FmCreateSyncEventCallback CreateSyncEvent;
        FmDestroySyncEventCallback DestroySyncEvent;
        FmWaitForSyncEventCallback WaitForSyncEvent;
        FmTriggerSyncEventCallback TriggerSyncEvent;
#if !FM_ASYNC_THREADING
        FmCreateTaskWaitCounterCallback CreateTaskWaitCounter;
        FmWaitForTaskWaitCounterCallback WaitForTaskWaitCounter;
        FmDestroyTaskWaitCounterCallback DestroyTaskWaitCounter;
        FmSubmitTaskCallback SubmitTask;
        FmParallelForCallback ParallelFor;
#endif

        FmTaskSystemCallbacks()
        {
            GetTaskSystemNumThreads = NULL;
            GetTaskSystemWorkerIndex = NULL;
            SubmitAsyncTask = NULL;
            CreateSyncEvent = NULL;
            DestroySyncEvent = NULL;
            WaitForSyncEvent = NULL;
            TriggerSyncEvent = NULL;
#if !FM_ASYNC_THREADING
            GetTaskSystemNumThreads = NULL;
            CreateTaskWaitCounter = NULL;
            WaitForTaskWaitCounter = NULL;
            DestroyTaskWaitCounter = NULL;
            SubmitTask = NULL;
            ParallelFor = NULL;
#endif
        }

        void SetCallbacks(
            FmGetTaskSystemNumThreadsCallback InGetTaskSystemNumThreads,
            FmGetTaskSystemWorkerIndex InGetTaskSystemWorkerIndex,
            FmSubmitAsyncTaskCallback InSubmitAsyncTask,
            FmCreateSyncEventCallback InCreateSyncEvent,
            FmDestroySyncEventCallback InDestroySyncEvent,
            FmWaitForSyncEventCallback InWaitForSyncEvent,
            FmTriggerSyncEventCallback InTriggerSyncEvent
#if !FM_ASYNC_THREADING
            , FmCreateTaskWaitCounterCallback InCreateTaskWaitCounter,
            FmWaitForTaskWaitCounterCallback InWaitForTaskWaitCounter,
            FmDestroyTaskWaitCounterCallback InDestroyTaskWaitCounter,
            FmSubmitTaskCallback InSubmitTask,
            FmParallelForCallback InParallelFor
#endif
            )
        {
            GetTaskSystemNumThreads = InGetTaskSystemNumThreads;
            GetTaskSystemWorkerIndex = InGetTaskSystemWorkerIndex;
            SubmitAsyncTask = InSubmitAsyncTask;
            CreateSyncEvent = InCreateSyncEvent;
            DestroySyncEvent = InDestroySyncEvent;
            WaitForSyncEvent = InWaitForSyncEvent;
            TriggerSyncEvent = InTriggerSyncEvent;
#if !FM_ASYNC_THREADING
            CreateTaskWaitCounter = InCreateTaskWaitCounter;
            WaitForTaskWaitCounter = InWaitForTaskWaitCounter;
            DestroyTaskWaitCounter = InDestroyTaskWaitCounter;
            SubmitTask = InSubmitTask;
            ParallelFor = InParallelFor;
#endif
        }
    };
}
