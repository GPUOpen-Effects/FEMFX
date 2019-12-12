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
// Asynchronous parallel-for call which executes an array of work items using specified 
// task callback function, and optionally may collect work items into task using a 
// batching callback function.  Expects task callback to detect completion and
// submit follow-up task.
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXTypes.h"
#include "FEMFXTaskSystemInterface.h"
#include "FEMFXAsyncThreading.h"

namespace AMD
{
    struct FmScene;

    // Prototype for a callback function to batch work items - returns a count of work items starting at taskBeginIndex
    // If provided to FmParallelForAsync, workers will apply the batching function to adjust granularity of tasks submitted to the task system
    typedef int32_t(*FmBatchingFuncCallback)(void* taskData, int32_t taskBeginIndex, int32_t taskEndIndex);

    // Dispatch parallel-for tasks.
    // NOTE: Calling code is responsible for continuing execution when taskCount == 0.
    // TaskFuncWrapped is a function that calls TaskFunc using FmExecuteTask().
    // Any SubmitAsyncTask calls within this will use TaskFuncWrapped to ensure a loop is running which will catch FmSetNextTask().
    // If runLoop true, any processing of TaskFunc on this thread will also use TaskFuncWrapped.  
    // This is necessary if FmParallelForAsync is not called from FmExecuteTask(), or other FmParallelForAsync() calls may take place before returning to FmExecuteTask().
    void FmParallelForAsync(const char* taskName, 
        FmTaskFuncCallback TaskFunc, FmTaskFuncCallback TaskFuncWrapped, FmBatchingFuncCallback BatchingFunc, void* taskData, int32_t taskCount, 
        FmSubmitAsyncTaskCallback SubmitAsyncTask, uint numThreads, bool runLoop = false);

    // Get number of tasks assuming maximum batch size per task
    static FM_FORCE_INLINE uint FmGetNumTasks(uint problemSize, uint maxTaskBatchSize)
    {
        return (problemSize + maxTaskBatchSize - 1) / maxTaskBatchSize;
    }

    // Get number of tasks based on desired batch size per task, but limited to at most maxTasks
    static FM_FORCE_INLINE uint FmGetNumTasksLimited(uint problemSize, uint taskBatchSize, uint maxTasks)
    {
        uint numTasks = (problemSize + taskBatchSize - 1) / taskBatchSize;
        numTasks = FmMinUint(maxTasks, numTasks);
        return numTasks;
    }

    // Get problem index range for the specified task index, assuming FmGetNumTasks() tasks
    static FM_FORCE_INLINE void FmGetIndexRange(uint* beginIndex, uint* endIndex, uint taskIndex, uint maxTaskBatchSize, uint problemSize)
    {
        uint begin = taskIndex * maxTaskBatchSize;
        uint end = begin + maxTaskBatchSize;
        begin = FmMinUint(begin, problemSize);
        end = FmMinUint(end, problemSize);
        *beginIndex = begin;
        *endIndex = end;
    }

    // Get number of tasks for a minimum batch size per task, assuming remainder will be evenly distributed to all tasks.
    static FM_FORCE_INLINE uint FmGetNumTasksMinBatchSize(uint problemSize, uint minTaskBatchSize)
    {
        return FmMaxUint(problemSize / minTaskBatchSize, 1);
    }

    // Get problem index range for the specified task index, assuming problem is distributed to tasks as evenly as possible.
    // NOTE: output range may be zero-sized if problemSize < taskCount
    static FM_FORCE_INLINE void FmGetIndexRangeEvenDistribution(uint* beginIndex, uint* endIndex, uint taskIndex, uint taskCount, uint problemSize)
    {
        uint taskBatchSize = problemSize / taskCount;
        uint remainderBatchSize = problemSize % taskCount;

        uint taskExtra = remainderBatchSize / taskCount;
        uint remainder = remainderBatchSize % taskCount;

        taskBatchSize += taskExtra;

        uint begin, end;
        if (taskIndex < remainder)
        {
            taskBatchSize++;
            begin = taskIndex * taskBatchSize;
        }
        else
        {
            begin = remainder * (taskBatchSize + 1) + (taskIndex - remainder) * taskBatchSize;
        }

        end = begin + taskBatchSize;

        *beginIndex = begin;
        *endIndex = end;
    }
}
