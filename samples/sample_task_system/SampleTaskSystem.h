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

#define SAMPLE_ASYNC_THREADING 1

#include <stdint.h>

namespace AMD
{
    typedef void(*FmTaskFuncCallback)(void* taskData, int32_t taskBeginIndex, int32_t taskEndIndex);

    // Task system initialization
    void SampleInitTaskSystem(int numThreads);
    void SampleDestroyTaskSystem();

    int SampleGetTaskSystemNumThreads();
    int SampleGetTaskSystemDefaultNumThreads();

    void SampleWaitForAllThreadsToStart();

    // Return index of current worker thread, between 0 and numThreads-1, unique between workers running concurrently
    int SampleGetTaskSystemWorkerIndex();

    typedef void FmSyncEvent;

    // Submit asynchronous task to task scheduler, which should run TaskFunc, taskData and taskIndex arguments
    void SampleAsyncTask(const char* taskName, FmTaskFuncCallback TaskFunc, void *taskData, int32_t taskBeginIndex, int32_t taskEndIndex);

    // Create task event
    FmSyncEvent* SampleCreateSyncEvent();

    // Destroy task event
    void SampleDestroySyncEvent(FmSyncEvent* taskEvent);

    // Wait for task event to be triggered
    void SampleWaitForSyncEvent(FmSyncEvent* taskEvent);

    // Trigger task event
    void SampleTriggerSyncEvent(FmSyncEvent* taskEvent);

#if !SAMPLE_ASYNC_THREADING
    typedef void FmTaskWaitCounter;

    // Create counter with count indicating unblocked
    FmTaskWaitCounter* SampleCreateTaskWaitCounter();

    // Wait on counter
    void SampleWaitForTaskWaitCounter(FmTaskWaitCounter* counter);

    // Destroy counter
    void SampleDestroyTaskWaitCounter(FmTaskWaitCounter* counter);

    // Submit task to task scheduler which runs TaskFunc with taskData and taskIndex arguments.
    // If waitCounter non-NULL, this call will increment counter, and task will decrement on completion
    void SampleSubmitTask(const char* taskName, FmTaskFuncCallback TaskFunc, void* taskData, int32_t taskBeginIndex, int32_t taskEndIndex, FmTaskWaitCounter* waitCounter);

    void SampleParallelFor(const char* taskName, FmTaskFuncCallback TaskFunc, void* taskData, int32_t taskCount);
#endif
}
