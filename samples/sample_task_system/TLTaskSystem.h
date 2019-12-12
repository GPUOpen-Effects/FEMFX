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
// Task system with multiple task queues and a pool of worker threads
//---------------------------------------------------------------------------------------

#pragma once

#include "TLCommon.h"
#include "TLTaskQueue.h"

#define TL_WAIT_COUNTER    1  // Enable condition variable waiting (otherwise only uses spin-waits/yields)
#define TL_THREAD_SETS     1  // Group threads into sets which can have different backoff constants
#define TL_THREAD_SET_SIZE 4  // Size of initial set of threads

namespace AMD
{
    // Thread-local state initialized when worker thread runs
    extern TL_THREAD_LOCAL_STORAGE int32_t gTLWorkerThreadIndex;

    // Worker thread loop
    uint32_t TLWorkerThread(void *inData);

    // For detecting core count
    extern int32_t TLGetProcessorInfo(int32_t* pNumPhysicalCores, int32_t* pNumLogicalCores);

    // Per-thread information used during scheduling
    struct TLWorkerInfo
    {
        int32_t setIndex;

        TLWorkerInfo() : setIndex(0) {}
    };

    // Task system that creates a pool of worker threads that each own a task queue, but may also take tasks from other queues.
    // NOTE: The implementation assumes that only one other thread outside the thread pool submits tasks.
    //
    // When tasks run out, worker threads will start a back-off process that includes spin-waits, yields, and waiting on a condition variable.
    //
    // Best performance was seen by having workers remove tasks from the end of their queues, but steal from the beginning of other queues.
    class TLTaskSystem
    {
    private:
        TLAtomicInt numWorkersStarted; // Atomic count of started workers to give each a unique index
        TLAtomicInt quitSignal;        // Atomic flag workers check to quit
        TLAtomicInt numTasks;          // Total number of tasks in all queues, checked before waiting on condition var

        // Counter/condition var is created for each set of threads to allow sleeping when there are insufficient tasks.
        // A value <= 0 indicates tasks available, and >0 unavailable.
        // These only need to be modified when task counts reach certain thresholds, reducing number of critical section locks.
        // Note increment/decrement operations may occur out of order and so be out of sync with the availability of tasks.
        // However as threads complete counter values will settle.
            
#if TL_WAIT_COUNTER
#if TL_THREAD_SETS
        TLCounter* waitCounters;   // Condition var to put set of threads to sleep
#else
        TLCounter waitCounter;   // Condition var to put set of threads to sleep
#endif
#endif

        int32_t numWorkerThreads;
        int32_t numThreadSets;

        int32_t numPhysicalCores;
        int32_t numLogicalCores;

        TLWorkerInfo* workerInfo;

        TLThread* workerThreads;
        TLTaskQueueList* taskQueues;   // Each worker and main thread has a task queue (a list of them to support overflow); workers can steal from other queues

        static TLTaskSystem* pInstance;   // For singleton

        TLTaskSystem(int32_t inNumWorkerThreads, int32_t inNumPhysicalCores, int32_t inNumLogicalCores) 
        {
            pInstance = this;

            TLAtomicWrite(&numWorkersStarted.val, 0);
            TLAtomicWrite(&quitSignal.val, 0);
            TLAtomicWrite(&numTasks.val, 0);

            numPhysicalCores = inNumPhysicalCores;
            numLogicalCores = inNumLogicalCores;

            CheckSetupParams(&numWorkerThreads, inNumWorkerThreads, inNumLogicalCores);

            workerInfo = new TLWorkerInfo[numWorkerThreads + 1]; // Add one for main thread

            for (int32_t threadIdx = 0; threadIdx < numWorkerThreads; threadIdx++)
            {
#if TL_THREAD_SETS
                // Two sets, one which spins longer
                int32_t threadSetIndex = (threadIdx < TL_THREAD_SET_SIZE) ? 0 : 1;
#else
                int32_t threadSetIndex = 0;
#endif

                workerInfo[threadIdx].setIndex = threadSetIndex;
            }

#if TL_THREAD_SETS
            numThreadSets = 2;
#else
            numThreadSets = 1;
#endif

#if TL_WAIT_COUNTER
#if TL_THREAD_SETS
            waitCounters = new TLCounter[numThreadSets];

            // Will make workers sleep-wait until tasks added
            for (int32_t i = 0; i < numThreadSets; i++)
            {
                waitCounters[i].Increment();
            }
#else
            waitCounter.Increment();
#endif
#endif
            workerThreads = new TLThread[numWorkerThreads];
            taskQueues = new TLTaskQueueList[numWorkerThreads + 1];

            for (int i = 0; i < numWorkerThreads; i++)
            {
                TLCreateJoinableThread(&workerThreads[i], TLWorkerThread, this, 0, "TLTaskSystem Worker Thread");
            }
        };

        ~TLTaskSystem()
        {
#if TL_WAIT_COUNTER
            // Wake workers and prevent additional waiting
#if TL_THREAD_SETS
            for (int32_t i = 0; i < numThreadSets; i++)
            {
                waitCounters[i].Decrement();
            }
#else
            waitCounter.Decrement();
#endif
#endif

            // Set value threads are checking to quit
            TLAtomicWrite(&quitSignal.val, 1);

            for (int i = 0; i < numWorkerThreads; i++)
            {
                TLJoinThread(workerThreads[i]);
            }

            delete[] workerInfo;

            delete[] workerThreads;

            delete[] taskQueues;

#if TL_WAIT_COUNTER && TL_THREAD_SETS
            delete [] waitCounters;
#endif
        };

        TLTaskSystem(TLTaskSystem const&) {};
        TLTaskSystem& operator=(TLTaskSystem const&) {};

        TL_FORCE_INLINE int32_t GetNumTasks()
        {
            return TLAtomicRead(&numTasks.val);
        }

        // Decrement num tasks and trigger sleep if 0
        TL_FORCE_INLINE void DecrementNumTasks()
        {
            int32_t newNumTasks = TLAtomicDecrement(&numTasks.val);

#if TL_WAIT_COUNTER
            // If removing last task for set of threads, set wait condition for set
#if TL_THREAD_SETS
            if (newNumTasks == 0)
            {
                waitCounters[0].Increment();
            }
            else if (newNumTasks == TL_THREAD_SET_SIZE)
            {
                waitCounters[1].Increment();
            }
#else
            if (newNumTasks == 0)
            {
                waitCounter.Increment();
            }
#endif
#else
            (void)newNumTasks;
#endif
        }

        // Increment num tasks and trigger wake if 1
        TL_FORCE_INLINE void IncrementNumTasks()
        {
            int32_t newNumTasks = TLAtomicIncrement(&numTasks.val);

#if TL_WAIT_COUNTER
            // If adding first task for set, wake set
#if TL_THREAD_SETS
            if (newNumTasks == 1)
            {
                waitCounters[0].Decrement();
            }
            else if (newNumTasks == TL_THREAD_SET_SIZE + 1)
            {
                waitCounters[1].Decrement();
            }
#else
            if (newNumTasks == 1)
            {
                waitCounter.Decrement();
            }
#endif
#else
            (void)newNumTasks;
#endif
        }

        // Try to submit task.
        // Only safe for one thread to submit at a time.
        TL_FORCE_INLINE bool TrySubmitTaskAtEnd(const TLTask& task, int32_t queueIndex)
        {
            return taskQueues[queueIndex].TrySubmitTaskAtEnd(task);
        }

        // Try to find task to run.
        // Multiple threads can call this concurrently
        TL_FORCE_INLINE bool TryClaimTask(TLTask* task)
        {
            if (GetNumTasks() == 0)
            {
                return false;
            }

            // Check own queue first.  Claim from end to improve locality.
            int32_t workerIndex = gTLWorkerThreadIndex;
            if (taskQueues[workerIndex].TryClaimTaskFromEnd(task))
            {
                DecrementNumTasks();
                return true;
            }

            // Steal from other queues.
            int32_t qIdxBase = workerIndex;

            int32_t numQueues = numWorkerThreads + 1;
            for (int32_t offset = 1; offset < numQueues; offset++)
            {
                if (GetNumTasks() == 0)
                {
                    return false;
                }

                int32_t qIdx = qIdxBase + offset;
                qIdx = (qIdx >= numQueues)? qIdx - numQueues : qIdx;

                if (taskQueues[qIdx].TryClaimTaskFromBeginning(task))
                {
                    DecrementNumTasks();
                    return true;
                }
            }

            return false;
        }

        static TL_FORCE_INLINE void CheckSetupParams(
            int32_t* outNumWorkerThreads,
            int32_t inNumWorkerThreads,
            int32_t inNumLogicalCores)
        {
            int32_t maxWorkerThreads = inNumLogicalCores;
            int32_t numWorkerThreads = (inNumWorkerThreads <= 0 || inNumWorkerThreads > maxWorkerThreads) ? maxWorkerThreads : inNumWorkerThreads;

            *outNumWorkerThreads = numWorkerThreads;
        }

    public:
        TL_CLASS_NEW_DELETE(TLTaskSystem)

        // Create task system and worker threads.
        // If inNumWorkerThreads is 0, sets number of workers to GetDefaultNumWorkerThreads().
        static void Create(int32_t inNumWorkerThreads)
        {
            // Get processor info
            int32_t numPhysical, numLogical;
            TLGetProcessorInfo(&numPhysical, &numLogical);

            // Check if input parameters match the current instance
            int32_t numWorkers;
            CheckSetupParams(&numWorkers, inNumWorkerThreads, numLogical);

            if (pInstance == NULL 
                || pInstance->numWorkerThreads != numWorkers)
            {
                delete pInstance;
                pInstance = new TLTaskSystem(numWorkers, numPhysical, numLogical);
            }
        }

        // Shutdown worker threads and destroy resource
        static void Destroy()
        {
            delete pInstance;
            pInstance = NULL;
        }

        // Get singleton
        static TLTaskSystem* Get()
        {
            return pInstance;
        }        

        TL_FORCE_INLINE int32_t GetNumWorkerThreads()
        {
            return numWorkerThreads;
        }

        TL_FORCE_INLINE int32_t GetNumPhysicalCores()
        {
            return numPhysicalCores;
        }

        TL_FORCE_INLINE int32_t GetNumLogicalCores()
        {
            return numLogicalCores;
        }

        TL_FORCE_INLINE TLWorkerInfo GetWorkerInfo(int32_t workerIndex)
        {
            return workerInfo[workerIndex];
        }

        static TL_FORCE_INLINE int32_t GetDefaultNumWorkerThreads()
        {
            int32_t numPhysicalCores, numLogicalCores;
            TLGetProcessorInfo(&numPhysicalCores, &numLogicalCores);
            return numLogicalCores;
        }

        // Called by worker thread on creation to reserve a worker index
        TL_FORCE_INLINE int32_t ReserveWorkerIndex()
        {
            return TLAtomicIncrement(&numWorkersStarted.val) - 1;
        }

        TL_FORCE_INLINE void WaitForAllWorkersToStart()
        {
            while (TLAtomicRead(&numWorkersStarted.val) < numWorkerThreads);
            {
                TLPause();
            }
        }

        // Return current worker index.
        // -1 for thread on which task system initialized 
        // >= 0 and < numWorkerThreads for worker threads
        TL_FORCE_INLINE int32_t GetWorkerIndex()
        {
            return gTLWorkerThreadIndex;
        }

        // Only worker threads will process tasks
        TL_FORCE_INLINE bool IsWorkerThread()
        {
            return (gTLWorkerThreadIndex >= 0);
        }

        // Called by worker thread to check for signal to quit
        TL_FORCE_INLINE int32_t GetQuitSignal()
        {
            return TLAtomicRead(&quitSignal.val);
        }

        // Process a submitted task, or sleep until signal that tasks available.
        // After wake-up will return whether task can be claimed or not.
        // Returns if task was processed.
        // NOTE: Should only be called by worker thread.
        bool TryProcessTaskOrWait(bool* didSleep, int32_t threadSetIndex)
        {
            (void)threadSetIndex;
            *didSleep = false;

#if TL_WAIT_COUNTER
            if (GetNumTasks() == 0)
            {
                TL_ASSERT(threadSetIndex >= 0 && threadSetIndex < numThreadSets);

                // Sleep on condition var, but if woken, return to poll for new tasks, in case more about to be added.
#if TL_THREAD_SETS
                *didSleep = waitCounters[threadSetIndex].WaitOneWakeup();
#else
                *didSleep = waitCounter.WaitOneWakeup();
#endif
            }
#endif

            return TryProcessTask();
        }

        // Process a submitted task if available.
        // Return if task was processed.
        // NOTE: Should only be called by worker thread.
        TL_FORCE_INLINE bool TryProcessTask()
        {
            TLTask task;
            if (TryClaimTask(&task))
            {
                // Run claimed task
                task.func(task.data, task.beginIndex, task.endIndex);

                return true;
            }

            return false;
        }

        // Called by worker thread to submit task to task system.
        // May be called by one thread outside of task system (such as main thread).
        void SubmitTask(const TLTask& task)
        {
            int32_t queueIndex = gTLWorkerThreadIndex;
            if (queueIndex == -1)
            {
                queueIndex = numWorkerThreads;
            }
            while(!TrySubmitTaskAtEnd(task, queueIndex))
            {
            }

            IncrementNumTasks();
        }
    };
}