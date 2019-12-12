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

#include "FEMFXTypes.h"
#include "FEMFXAtomicOps.h"
#include "FEMFXParallelFor.h"
#include "FEMFXAsyncThreading.h"

namespace AMD
{
    // To control stack usage when using async tasks (especially if simulating with a single thread),
    // instead of making tail calls to follow-up tasks, each thread will execute a loop on a global
    // task function until it's NULL, and each task will overwrite the global task function with a 
    // follow-up task rather than calling it directly.
    struct FmTaskFuncLoopData
    {
        FmTaskFuncCallback func;
        void* data;
        int32_t beginIndex;
        int32_t endIndex;

        FmTaskFuncLoopData() : func(NULL), data(NULL), beginIndex(0), endIndex(1) {}

        void SetNextTask(FmTaskFuncCallback inFunc, void* inData)
        {
            func = inFunc;
            data = inData;
            beginIndex = 0;
            endIndex = 1;
        }

        void SetNextTask(FmTaskFuncCallback inFunc, void* inData, int32_t inBeginIndex, int32_t inEndIndex)
        {
            FM_ASSERT(func == NULL); // should have run and cleared this
            func = inFunc;
            data = inData;
            beginIndex = inBeginIndex;
            endIndex = inEndIndex;
        }
    };

    FM_THREAD_LOCAL_STORAGE FmTaskFuncLoopData gFEMFXTaskFuncLoopData;

    // Loop while there is a non-NULL task function set in gFEMFXTaskFuncLoopData.
    // Reduces use of stack with Async code.
    // Instead of tail calls, task functions should call FmSetNextTask and return.
    void FmExecuteTask(FmTaskFuncCallback inTaskFunc, void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        gFEMFXTaskFuncLoopData.SetNextTask(inTaskFunc, inTaskData, inTaskBeginIndex, inTaskEndIndex);

        while (gFEMFXTaskFuncLoopData.func)
        {
            FmTaskFuncCallback func = gFEMFXTaskFuncLoopData.func;
            void* data = gFEMFXTaskFuncLoopData.data;
            int32_t beginIndex = gFEMFXTaskFuncLoopData.beginIndex;
            int32_t endIndex = gFEMFXTaskFuncLoopData.endIndex;

            // Clear func to exit loop, unless called function adds new task
            gFEMFXTaskFuncLoopData.func = NULL;

            func(data, beginIndex, endIndex);
        }
    }

    void FmSetNextTask(FmTaskFuncCallback inTaskFunc, void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        gFEMFXTaskFuncLoopData.SetNextTask(inTaskFunc, inTaskData, inTaskBeginIndex, inTaskEndIndex);
    }

    void FmSetNextTask(const FmTask& inTask)
    {
        gFEMFXTaskFuncLoopData.SetNextTask(inTask.func, inTask.data, inTask.beginIndex, inTask.endIndex);
    }

    // Data needed for a thread to dispatch a portion of parallel-for tasks
    class FmParallelForDispatcherData
    {
    public:
        FM_CLASS_NEW_DELETE(FmParallelForDispatcherData)

        FmAtomicUint dispatcherIndexAtomic;
        FmAtomicUint numDispatchersIncomplete;

        FmSubmitAsyncTaskCallback SubmitAsyncTask;
        const char* taskName;
        FmTaskFuncCallback TaskFunc;
        FmTaskFuncCallback TaskFuncWrapped;
        FmBatchingFuncCallback BatchingFunc;
        void* taskData;
        uint problemSize;
        uint numDispatchers;

        FmParallelForDispatcherData(
            FmSubmitAsyncTaskCallback inSubmitAsyncTask,
            const char* inTaskName,
            FmTaskFuncCallback inTaskFunc,
            FmTaskFuncCallback inTaskFuncWrapped,
            FmBatchingFuncCallback inBatchingFunc,
            void* inTaskData,
            uint inProblemSize,
            uint inNumDispatchers)
        {
            SubmitAsyncTask = inSubmitAsyncTask;
            taskName = inTaskName;
            TaskFunc = inTaskFunc;
            TaskFuncWrapped = inTaskFuncWrapped;
            BatchingFunc = inBatchingFunc;
            taskData = inTaskData;
            problemSize = inProblemSize;
            numDispatchers = inNumDispatchers;
            FmAtomicWrite(&dispatcherIndexAtomic.val, 0);
            FmAtomicWrite(&numDispatchersIncomplete.val, numDispatchers);
        }
    };

    // Dispatch a portion of parallel-for tasks.
    // TODO: Include batching of tasks based on weights, using the begin and end indices
    FM_WRAPPED_TASK_FUNC(FmTaskFuncParallelForDispatcher)
    {
        (void)inTaskEndIndex;

        FmParallelForDispatcherData* dispatcherData = (FmParallelForDispatcherData *)inTaskData;

        FmBatchingFuncCallback BatchingFunc = dispatcherData->BatchingFunc;

        uint dispatcherIndex = (uint)inTaskBeginIndex;
        dispatcherIndex = FmAtomicIncrement(&dispatcherData->dispatcherIndexAtomic.val) - 1;

        uint numDispatchers = dispatcherData->numDispatchers;
        uint problemSize = dispatcherData->problemSize;

        // Compute range of parallel-for indices this dispatcher covers
        uint beginIndex, endIndex;
        FmGetIndexRangeEvenDistribution(&beginIndex, &endIndex, dispatcherIndex, numDispatchers, problemSize);

        if (BatchingFunc)
        {
            // Batch indices based on batching function
            void* taskData = dispatcherData->taskData;

            // Save first batch to run on this thread
            uint firstNumItems = (uint)BatchingFunc(taskData, beginIndex, endIndex);
            uint firstBeginIndex = beginIndex;

            beginIndex += firstNumItems;

            while (beginIndex < endIndex)
            {
                uint numItems = (uint)BatchingFunc(taskData, beginIndex, endIndex);

                // Submit task
                dispatcherData->SubmitAsyncTask(dispatcherData->taskName, dispatcherData->TaskFuncWrapped, dispatcherData->taskData, beginIndex, beginIndex + numItems);

                beginIndex += numItems;
            }

            if (firstNumItems > 0)
            {
                FmSetNextTask(dispatcherData->TaskFunc, dispatcherData->taskData, firstBeginIndex, firstBeginIndex + firstNumItems);
            }
        }
        else
        {
            uint numTasks = endIndex - beginIndex;

#define FM_STRIDED 1
#if FM_STRIDED
            // Experiment to improve ordering of tasks, however depends on task system; also should have no effect if using an atomic counter to ensure order.
            beginIndex = dispatcherIndex;
            uint stride = numDispatchers;
#endif

            // Run one task in-line but submit rest
            for (uint i = 1; i < numTasks; i++)
            {
#if FM_STRIDED
                uint taskIndex = beginIndex + stride * i;
#else
                uint taskIndex = beginIndex + i;
#endif
                dispatcherData->SubmitAsyncTask(dispatcherData->taskName, dispatcherData->TaskFuncWrapped, dispatcherData->taskData, taskIndex, taskIndex + 1);
            }

            if (numTasks >= 1)
            {
                FmSetNextTask(dispatcherData->TaskFunc, dispatcherData->taskData, beginIndex, beginIndex + 1);
            }
        }

        uint numIncomplete = FmAtomicDecrement(&dispatcherData->numDispatchersIncomplete.val);

        if (numIncomplete == 0)
        {
            delete dispatcherData;
        }
    }

    // Dispatch parallel-for tasks.
    // NOTE: Calling code is responsible for continuing execution when taskCount == 0.
    // TaskFuncWrapped is a function that calls TaskFunc using FmExecuteTask().
    // Any SubmitAsyncTask calls within this will use TaskFuncWrapped to ensure a loop is running which will catch FmSetNextTask().
    // If runLoop true, any processing of TaskFunc on this thread will also use TaskFuncWrapped.  
    // This is necessary if FmParallelForAsync is not called from FmExecuteTask(), or other FmParallelForAsync() calls may take place before returning to FmExecuteTask().
    void FmParallelForAsync(const char* taskName, 
        FmTaskFuncCallback TaskFunc, FmTaskFuncCallback TaskFuncWrapped, FmBatchingFuncCallback BatchingFunc, void* taskData, int32_t taskCount, 
        FmSubmitAsyncTaskCallback SubmitAsyncTask, uint numThreads, bool runLoop)
    {
        (void)taskName;

        if (taskCount <= 0)
        {
            return;
        }

        // If taskCount under a threshold will just submit all on this thread.
        // Otherwise, splits up the submitting work and spawns other threads to dispatch.

        const int32_t numSubmitsPerThread = 16;

        // Get number of dispatchers needed
        int numDispatchers = FmGetNumTasks((uint)taskCount, numSubmitsPerThread);
        numDispatchers = FmMinUint(numThreads * 8, numDispatchers);

        FmParallelForDispatcherData* dispatcherData = new FmParallelForDispatcherData(SubmitAsyncTask, taskName, TaskFunc, TaskFuncWrapped, BatchingFunc, taskData, (uint)taskCount, numDispatchers);

        // Submit other dispatchers
        for (uint i = 1; i < dispatcherData->numDispatchers; i++)
        {
            dispatcherData->SubmitAsyncTask("FEMFXParallelForDispatcher", FmTaskFuncParallelForDispatcherWrapped, dispatcherData, i, i + 1);
        }

        // Dispatch some tasks on this thread
        if (runLoop)
        {
            FmTaskFuncParallelForDispatcherWrapped(dispatcherData, 0, 1);
        }
        else
        {
            FmSetNextTask(FmTaskFuncParallelForDispatcher, dispatcherData, 0, 1);
        }
    }
}
