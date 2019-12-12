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

#include "SampleTaskSystem.h"

#define USE_MULTITHREADING 1

#ifndef SAMPLE_TASK_SYS_ENV_SET
#define USE_TL  1
#endif

#if USE_MULTITHREADING
#if USE_TL
#include "TLTaskSystem.h"
#endif
#else
#include <stdio.h>
#endif

namespace AMD
{
    int gSampleTaskSystemNumThreads = 1;

    void SampleInitTaskSystem(int numThreads)
    {
        if (numThreads == 0)
        {
            numThreads = SampleGetTaskSystemDefaultNumThreads();
        }

        gSampleTaskSystemNumThreads = numThreads;

#if USE_MULTITHREADING
#if USE_TL
        TLTaskSystem::Create(numThreads);
        gSampleTaskSystemNumThreads = numThreads;
#endif
#else
        (void)numThreads;
#endif
    }

    void SampleDestroyTaskSystem()
    {
#if USE_MULTITHREADING
#if USE_TL
        TLTaskSystem::Destroy();
#endif
#endif
    }

    void SampleWaitForAllThreadsToStart()
    {
#if USE_MULTITHREADING
#if USE_TL
        TLTaskSystem::Get()->WaitForAllWorkersToStart();
#endif
#endif
    }

    int SampleGetTaskSystemNumThreads()
    {
        return gSampleTaskSystemNumThreads;
    }

    int SampleGetTaskSystemDefaultNumThreads()
    {
#if USE_MULTITHREADING
#if USE_TL
        return TLTaskSystem::GetDefaultNumWorkerThreads();
#endif
#else
        return 1;
#endif
    }

    // Return index of current worker thread, >= 0 and < numThreads specified to SampleInitTaskSystem(), unique between workers running concurrently.
    // Returns -1 if number of workers exceeds numThreads.
    int SampleGetTaskSystemWorkerIndex()
    {
#if USE_MULTITHREADING
#if USE_TL
        TLTaskSystem* taskSys = TLTaskSystem::Get();
        return taskSys->GetWorkerIndex();
#endif
#else
        return 0;
#endif
    }

    void SampleAsyncTask(const char* taskName, FmTaskFuncCallback TaskFunc, void* taskData, int32_t taskBeginIndex, int32_t taskEndIndex)
    {
        (void)taskName;
#if USE_MULTITHREADING
#if USE_TL
        TLTask task;
        task.func = TaskFunc;
        task.data = taskData;
        task.beginIndex = taskBeginIndex;
        task.endIndex = taskEndIndex;

        TLTaskSystem* taskSys = TLTaskSystem::Get();
        taskSys->SubmitTask(task);
#endif
#else
        TaskFunc(taskData, taskBeginIndex, taskEndIndex);
#endif
    }

    FmSyncEvent* SampleCreateSyncEvent()
    {
#if USE_MULTITHREADING
#if USE_TL
        TLCounter* taskCounter = new TLCounter();
        taskCounter->Increment();
        return taskCounter;
#endif
#else
        bool* eventFlag = new bool(false);
        return eventFlag;
#endif
    }

    void SampleDestroySyncEvent(FmSyncEvent* taskEvent)
    {
#if USE_MULTITHREADING
#if USE_TL
        TLCounter* taskCounter = (TLCounter*)taskEvent;
        delete taskCounter;
#endif
#else
        bool* eventFlag = (bool *)taskEvent;
        delete eventFlag;
#endif
    }

    void SampleWaitForSyncEvent(FmSyncEvent* taskEvent)
    {
#if USE_MULTITHREADING
#if USE_TL
        TLCounter* taskCounter = (TLCounter*)taskEvent;
        taskCounter->WaitUntilZero();
#endif
#else
        bool* eventFlag = (bool *)taskEvent;
        while (*eventFlag == false)
        {
            printf("Event flag not set\n");
        }
#endif
    }

    void SampleTriggerSyncEvent(FmSyncEvent* taskEvent)
    {
#if USE_MULTITHREADING
#if USE_TL
        TLCounter* taskCounter = (TLCounter*)taskEvent;
        taskCounter->Decrement();
#endif
#else
        bool* eventFlag = (bool *)taskEvent;
        *eventFlag = true;
#endif
    }
}

