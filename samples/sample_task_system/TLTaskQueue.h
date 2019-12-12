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
// Single-producer multi-consumer queue for use with task system.
//---------------------------------------------------------------------------------------

#pragma once

#include "TLCommon.h"
#include "TLCounter.h"

#define TL_TASKQ_MAX_TASKS 32768

#define TL_TASKQ_UNINITIALIZED_INDEX 0x80000000

#define TL_TASKQ_GET_BOUNDS(bounds, lowIdx, numTasks) \
    lowIdx = (bounds) >> 32; \
    numTasks = (bounds) & 0xffffffff;

#define TL_TASKQ_SET_BOUNDS(bounds, lowIdx, numTasks) \
    bounds = (((uint64_t)lowIdx) << 32) | (numTasks);

// Use spin lock to modify queue bounds and read or write data.
// This path is more general allowing any claim or submit calls to be concurrent.
#define TL_TASKQ_USE_LOCK 0

// Allows adding linked list nodes to expand task queue
#define TL_TASKQ_ADD_NODES 1

namespace AMD
{
    typedef void(*TLTaskCallback)(void* taskData, int32_t taskBeginIndex, int32_t taskEndIndex);

    // A task is a function pointer, data, and index.
    struct TLTask
    {
        TLTaskCallback func;       // task function callback
        void*          data;       // pointer to input data
        int32_t        beginIndex; // task specific index, TL_TASKQ_UNINITIALIZED_INDEX reserved for uninitialized
        int32_t        endIndex;   // task specific index

        TLTask()
        {
            func = NULL;
            data = NULL;
            beginIndex = TL_TASKQ_UNINITIALIZED_INDEX;
            endIndex = 0;
        }
    };

    // Single-producer multi-consumer queue for threads to add and claim tasks.
    // Allows one worker thread to append tasks and claim from the 
    // end of queue, while other threads may take tasks from beginning.
    class TLTaskQueue
    {
#if TL_TASKQ_USE_LOCK
        TLAtomicInt lock;
#endif
        TLAtomicInt64 bounds;
        TLTask tasksBuffer[TL_TASKQ_MAX_TASKS];

        // Pointers for creating a list
        TLTaskQueue* prev;
        TLTaskQueue* next;

    public:
        TL_CLASS_NEW_DELETE(TLTaskQueue)

        inline TLTaskQueue()
        {
            Init();
        }

        inline ~TLTaskQueue()
        {
        }
    
        inline void Init()
        {
#if TL_TASKQ_USE_LOCK
            TLAtomicWrite(&lock.val, 0);
#endif
            TLAtomicWrite64(&bounds.val, 0);

            for (int32_t i = 0; i < TL_TASKQ_MAX_TASKS; i++)
            {
                tasksBuffer[i] = TLTask();
            }

            prev = NULL;
            next = NULL;
        }

        inline TLTaskQueue* GetNextQueue()
        {
            return reinterpret_cast<TLTaskQueue*>(TLAtomicRead64((intptr_t *)&next));
        }

        inline TLTaskQueue* GetPrevQueue()
        {
            return reinterpret_cast<TLTaskQueue*>(TLAtomicRead64((intptr_t *)&prev));
        }

        inline void SetNextQueue(TLTaskQueue* inNextQueue)
        {
            TLAtomicWrite64((intptr_t *)&next, reinterpret_cast<int64_t>(inNextQueue));
        }

        inline void SetPrevQueue(TLTaskQueue* inPrevQueue)
        {
            TLAtomicWrite64((intptr_t *)&prev, reinterpret_cast<int64_t>(inPrevQueue));
        }

        // Try to claim a task from the beginning of the task queue, and return whether successful.
        // Safe for multiple threads to call this, while one thread submits tasks or claims from the end.
        TL_FORCE_INLINE bool TryClaimTaskFromBeginning(TLTask* task)
        {
#if TL_TASKQ_USE_LOCK
            uint32_t lowIdx, numTasks;
            uint64_t initialBounds = TLAtomicRead64(&bounds.val);
            TL_TASKQ_GET_BOUNDS(initialBounds, lowIdx, numTasks);

            if (numTasks == 0)
            {
                return false;
            }

            TLLock(&lock.val);

            initialBounds = *(volatile int64_t *)&bounds.val;
            TL_TASKQ_GET_BOUNDS(initialBounds, lowIdx, numTasks);

            if (numTasks == 0)
            {
                TLUnlock(&lock.val);
                return false;
            }

            uint32_t newLowIdx = (lowIdx + 1) % TL_TASKQ_MAX_TASKS;
            uint32_t newNumTasks = numTasks - 1;
            uint64_t newBounds;
            TL_TASKQ_SET_BOUNDS(newBounds, newLowIdx, newNumTasks);

            bounds.val = newBounds;

            *task = tasksBuffer[lowIdx];

            TLUnlock(&lock.val);
#else
            // First step is atomically updating bounds which claims a task and excludes other consumers.
            // Task data is expected to be valid, because producer doesn't update bounds until a task is written.
            // Producer is prevented from overwriting the same task (wrapping around the queue) until this consumer 
            // resets the beginIndex to TL_TASKQ_UNINITIALIZED_INDEX.

            // Loop until can successfully update bounds, or return if no tasks
            uint32_t lowIdx, numTasks;
            uint64_t initialBounds = TLAtomicRead64(&bounds.val);
            TL_TASKQ_GET_BOUNDS(initialBounds, lowIdx, numTasks);

            while (true)
            {
                if (numTasks == 0)
                {
                    return false;
                }

                // Try to increment lowIdx to claim task, or retry if another thread is first
                uint32_t newLowIdx = (lowIdx + 1) % TL_TASKQ_MAX_TASKS;
                uint32_t newNumTasks = numTasks - 1;
                uint64_t newBounds;
                TL_TASKQ_SET_BOUNDS(newBounds, newLowIdx, newNumTasks);

                uint64_t compareBounds = TLAtomicCompareExchange64(&bounds.val, newBounds, initialBounds);

                // If same bounds as when read, claim is successful
                if (compareBounds == initialBounds)
                {
                    break;
                }

                // Pause between atomic ops
                TLPause();

                // Retry
                initialBounds = compareBounds;
                TL_TASKQ_GET_BOUNDS(initialBounds, lowIdx, numTasks);
            }

            // A task is claimed by this thread.  Read value and then reset beginIndex to TL_TASKQ_UNINITIALIZED_INDEX, allowing producer to overwrite.
            TLTask& claimedTask = tasksBuffer[lowIdx];

            TLTask retTask = claimedTask;

            // Reset task index to allow producer to overwrite
            TLAtomicWrite(&claimedTask.beginIndex, TL_TASKQ_UNINITIALIZED_INDEX);

            *task = retTask;
#endif

            return true;
        }

        // Try to claim a task from the end of the task queue, and return whether successful.
        // Only safe for producer thread to call this.  May be called while other threads are
        // calling TryClaimTaskFromBeginning()
        TL_FORCE_INLINE bool TryClaimTaskFromEnd(TLTask* task)
        {
#if TL_TASKQ_USE_LOCK
            uint32_t lowIdx, numTasks;
            uint64_t initialBounds = TLAtomicRead64(&bounds.val);
            TL_TASKQ_GET_BOUNDS(initialBounds, lowIdx, numTasks);

            if (numTasks == 0)
            {
                return false;
            }

            TLLock(&lock.val);

            initialBounds = *(volatile int64_t *)&bounds.val;
            TL_TASKQ_GET_BOUNDS(initialBounds, lowIdx, numTasks);

            if (numTasks == 0)
            {
                TLUnlock(&lock.val);
                return false;
            }

            // Decrement numTasks to claim task at end
            uint32_t newLowIdx = lowIdx;
            uint32_t newNumTasks = numTasks - 1;
            uint64_t newBounds;
            TL_TASKQ_SET_BOUNDS(newBounds, newLowIdx, newNumTasks);

            bounds.val = newBounds;

            *task = tasksBuffer[(lowIdx + numTasks - 1) % TL_TASKQ_MAX_TASKS];

            TLUnlock(&lock.val);
#else
            // First step is atomically updating bounds which claims a task and excludes other consumers.
            // Task data is expected to be valid, because producer doesn't update bounds until a task is written.
            // Producer is prevented from overwriting the same task (wrapping around the queue) until this consumer 
            // resets the beginIndex to TL_TASKQ_UNINITIALIZED_INDEX.

            // Loop until can successfully update bounds, or return if no tasks
            uint32_t lowIdx, numTasks;
            uint64_t initialBounds = TLAtomicRead64(&bounds.val);
            TL_TASKQ_GET_BOUNDS(initialBounds, lowIdx, numTasks);

            while (true)
            {
                if (numTasks == 0)
                {
                    return false;
                }

                // Try to decrement numTasks to claim task at end, or retry if another thread is first
                uint32_t newLowIdx = lowIdx;
                uint32_t newNumTasks = numTasks - 1;
                uint64_t newBounds;
                TL_TASKQ_SET_BOUNDS(newBounds, newLowIdx, newNumTasks);

                uint64_t compareBounds = TLAtomicCompareExchange64(&bounds.val, newBounds, initialBounds);

                // If same bounds as when read, claim is successful
                if (compareBounds == initialBounds)
                {
                    break;
                }

                // Pause between atomic ops
                TLPause();

                // Retry
                initialBounds = compareBounds;
                TL_TASKQ_GET_BOUNDS(initialBounds, lowIdx, numTasks);
            }

            // A task is claimed by this thread.  Read value and then reset beginIndex to TL_TASKQ_UNINITIALIZED_INDEX, allowing producer to overwrite.
            TLTask& claimedTask = tasksBuffer[(lowIdx + numTasks - 1) % TL_TASKQ_MAX_TASKS];

            TLTask retTask = claimedTask;

            // Reset task index to allow producer to overwrite
            TLAtomicWrite(&claimedTask.beginIndex, TL_TASKQ_UNINITIALIZED_INDEX);

            *task = retTask;
#endif

            return true;
        }

        // Try to submit task, and return whether successful.
        // Only safe for one thread to use this at a time.
        inline bool TrySubmitTaskAtEnd(const TLTask& task)
        {
#if TL_TASKQ_USE_LOCK
            uint32_t lowIdx, numTasks;
            uint64_t initialBounds = TLAtomicRead64(&bounds.val);
            TL_TASKQ_GET_BOUNDS(initialBounds, lowIdx, numTasks);

            if (numTasks >= TL_TASKQ_MAX_TASKS)
            {
                return false;
            }

            TLLock(&lock.val);

            initialBounds = *(volatile int64_t *)&bounds.val;
            TL_TASKQ_GET_BOUNDS(initialBounds, lowIdx, numTasks);

            if (numTasks >= TL_TASKQ_MAX_TASKS)
            {
                TLUnlock(&lock.val);
                return false;
            }

            uint32_t newLowIdx = lowIdx;
            uint32_t newNumTasks = numTasks + 1;
            uint64_t newBounds;
            TL_TASKQ_SET_BOUNDS(newBounds, newLowIdx, newNumTasks);

            bounds.val = newBounds;

            uint32_t taskIdx = (lowIdx + numTasks) % TL_TASKQ_MAX_TASKS;
            tasksBuffer[taskIdx] = task;

            TLUnlock(&lock.val);
#else
            uint32_t lowIdx, numTasks;
            uint64_t initialBounds = TLAtomicRead64(&bounds.val);
            TL_TASKQ_GET_BOUNDS(initialBounds, lowIdx, numTasks);

            if (numTasks >= TL_TASKQ_MAX_TASKS)
            {
                return false;
            }

            // Update slot data at end of queue, but first wait for a reading consumer
            // to reset the task index to TL_TASKQ_UNINITIALIZED_INDEX.
            uint32_t taskIdx = (lowIdx + numTasks) % TL_TASKQ_MAX_TASKS;
            while (true)
            {
                TLTask& submitTask = tasksBuffer[taskIdx];

                // If all null can overwrite and exit
                if (TLAtomicRead(&submitTask.beginIndex) == TL_TASKQ_UNINITIALIZED_INDEX)
                {
                    // Write task
                    submitTask.func = task.func;
                    submitTask.data = task.data;
                    submitTask.endIndex = task.endIndex;
                    TLAtomicWrite(&submitTask.beginIndex, task.beginIndex);

                    break;
                }

                // Pause between reads
                TLPause();
            }

            // Now update the bounds to allow a reader to claim this slot.
            while (true)
            {
                // Try to increment num tasks to claim a slot, or retry if another thread is first
                uint32_t newLowIdx = lowIdx;
                uint32_t newNumTasks = numTasks + 1;
                uint64_t newBounds;
                TL_TASKQ_SET_BOUNDS(newBounds, newLowIdx, newNumTasks);

                uint64_t compareBounds = TLAtomicCompareExchange64(&bounds.val, newBounds, initialBounds);

                // If same bounds as when read, claim is successful
                if (compareBounds == initialBounds)
                {
                    break;
                }

                // Pause between atomic ops
                TLPause();

                // Retry
                initialBounds = compareBounds;
                TL_TASKQ_GET_BOUNDS(initialBounds, lowIdx, numTasks);
            }
#endif

            return true;
        }
    };

    // List of task queues, with nodes added for additional storage for submitted tasks.
    // The purpose of this is safety/correctness, but the expectation is that the task queue size is set large enough to avoid additional allocations.
    class TLTaskQueueList
    {
        TLTaskQueue* first;
        TLTaskQueue* last;

    public:
        TLTaskQueueList()
        {
            // Allocate one queue initially
            first = new TLTaskQueue();
            last = first;
        }

        ~TLTaskQueueList()
        {
            // Free all queues
            TLTaskQueue* queue = first;
            while (queue != NULL)
            {
                TLTaskQueue* next = queue->GetNextQueue();

                delete queue;

                queue = next;
            }
        }

        TL_FORCE_INLINE bool TryClaimTaskFromBeginning(TLTask* task)
        {
#if TL_TASKQ_ADD_NODES
            // Return first successful TryClaimTaskFromBeginning traversing from first to last queue
            TLTaskQueue* queue = first;

            do
            {
                if (queue->TryClaimTaskFromBeginning(task))
                {
                    return true;
                }

                queue = queue->GetNextQueue();

            } while (queue != NULL);

            return false;
#else
            return first->TryClaimTaskFromBeginning(task);
#endif
        }

        TL_FORCE_INLINE bool TryClaimTaskFromEnd(TLTask* task)
        {
#if TL_TASKQ_ADD_NODES
            // Return first successful TryClaimTaskFromEnd traversing from last to first queue
            TLTaskQueue* queue = last;

            do
            {
                if (queue->TryClaimTaskFromEnd(task))
                {
                    return true;
                }

                queue = queue->GetPrevQueue();

            } while (queue != NULL);

            return false;
#else
            return first->TryClaimTaskFromEnd(task);
#endif
        }

        TL_FORCE_INLINE bool TrySubmitTaskAtEnd(const TLTask& task)
        {
#if TL_TASKQ_ADD_NODES
            // Return after first successful TrySubmitTaskAtEnd traversing from first to last queue.
            // If not successful on last queue, add another node.
            // (For data locality, it would be better to submit task at the end, but this won't fill unused space.
            // This is written with expectation that more nodes are typically not created.)

            TLTaskQueue* queue = first;

            do
            {
                if (queue->TrySubmitTaskAtEnd(task))
                {
                    return true;
                }

                queue = queue->GetNextQueue();

            } while (queue != NULL);

            // Couldn't submit even to last queue, so add queue
            TLTaskQueue* next = new TLTaskQueue();
            next->SetPrevQueue(last);
            next->TrySubmitTaskAtEnd(task);

            last->SetNextQueue(next);

            // Update last queue - this is currently only used by the same thread
            last = next;

            return true;
#else
            return first->TrySubmitTaskAtEnd(task);
#endif

        }
    };
}
