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
// A task graph that lets you specify a set of tasks and dependencies between them.
// Running task will message dependent nodes and nodes with all dependencies met are 
// submitted to scheduler.  Includes event type that allows tasks to dynamically 
// message other nodes.
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXCommon.h"
#include "FEMFXArray.h"
#include "FEMFXAtomicOps.h"
#include "FEMFXTaskSystemInterface.h"
#include "FEMFXAsyncThreading.h"

namespace AMD
{
    class FmTaskGraph;

    // Node in a task graph that contains a task function and links to successor nodes.
    // The nodeFunc task is called with a pointer to this node.
    // The nodeFunc task must call this node's TaskIsFinished() method on completion.
    // Other custom data can be accessed by nodeFunc through the 'graph' pointer, if the graph is a derived class from FmTaskGraph.
    class FmTaskGraphNode
    {
        FmAtomicUint numPredecessorsIncomplete;  // num predecessors still incomplete
        int32_t numPredecessors;                         // num predecessors that must be completed before executing this node

        const char*        nodeName;
        FmTaskFuncCallback nodeFunc;
        FmTaskFuncCallback nodeFuncWrapped;  // "Wrapped" is the task called within FmExecuteTask loop for tail call optimization
        int32_t            nodeIndex;
        uint               nodeWeight;       // Weight which can be used to decide which node to run directly vs submit as task

        int32_t predecessorMessage; // Value stored by last completed predecessor

        FmArray<FmTaskGraphNode*> successors;
        FmTaskGraph* graph;

        // Called to signal completion to successor node, which may be ready to run.
        // If ppNextNode non-NULL, and *ppNextNode initialized to NULL, one of the ready successors will be returned for this thread to run.
        // The remaining ready nodes will be submitted tasks.
        void SignalSuccessors(int32_t message, FmTaskGraphNode** ppNextNode = NULL)
        {
            int numSuccessors = (int)successors.GetNumElems();
            for (int i = 0; i < numSuccessors; i++)
            {
                successors[i]->PredecessorComplete(message, ppNextNode);
            }
        }

    public:
        FM_CLASS_NEW_DELETE(FmTaskGraphNode)

        FmTaskGraphNode() : numPredecessorsIncomplete(0), numPredecessors(0), nodeName(NULL), nodeFunc(NULL), nodeFuncWrapped(NULL), nodeIndex(0), nodeWeight((uint)-1), predecessorMessage(-1), graph(NULL)
        {
        }

        FmTaskGraphNode(const char* inName, FmTaskGraph* inGraph, FmTaskFuncCallback inNodeFunc, FmTaskFuncCallback inNodeFuncWrapped, int32_t inNodeIndex, uint inNodeWeight = (uint)-1)
        {
            Init(inName, inGraph, inNodeFunc, inNodeFuncWrapped, inNodeIndex, inNodeWeight);
        }

        void Init(const char* inName, FmTaskGraph* inGraph, FmTaskFuncCallback inNodeFunc, FmTaskFuncCallback inNodeFuncWrapped, int32_t inNodeIndex, uint inNodeWeight = (uint)-1)
        {
            FmAtomicWrite(&numPredecessorsIncomplete.val, 0);
            numPredecessors = 0;

            nodeName = inName;
            nodeFunc = inNodeFunc;
            nodeFuncWrapped = inNodeFuncWrapped;
            nodeIndex = inNodeIndex;
            nodeWeight = inNodeWeight;

            predecessorMessage = -1;

            graph = inGraph;
        }

        inline const char* GetName() { return nodeName; }
        inline FmTaskFuncCallback GetTaskFunc() { return nodeFunc; }
        inline FmTaskFuncCallback GetTaskFuncWrapped() { return nodeFuncWrapped; }
        inline int32_t GetIndex() { return nodeIndex; }
        inline int32_t GetWeight() { return nodeWeight; }
        FmTaskGraph* GetGraph() { return graph; }
        inline int32_t GetPredecessorMessage() const { return predecessorMessage; }

        void AddSuccessor(FmTaskGraphNode* node)
        {
            successors.Add(node);
            node->IncrementNumPredecessors();
        }

        void SetNumPredecessors(int32_t inNumPredecessors)
        {
            numPredecessors = inNumPredecessors;
            FmAtomicWrite(&numPredecessorsIncomplete.val, inNumPredecessors);
        }

        void IncrementNumPredecessors()
        {
            numPredecessors++;
            FmAtomicIncrement(&numPredecessorsIncomplete.val);
        }

        // Called to signal completion to successor node, which may be ready to run.
        // If ppNextNode non-NULL, and *ppNextNode initialized to NULL, one of the ready successors will be returned for this thread to run.
        // The remaining ready nodes will be submitted tasks.
        inline void PredecessorComplete(int32_t message, FmTaskGraphNode** ppNextNode = NULL);

        // Called from scheduled task
        inline void Run();

        // Signal successors and update graph progress
        inline void TaskIsFinished(int32_t message, FmTaskGraphNode** ppNextNode = NULL);
    };

    // An event is just a list of successor nodes that can be signaled.
    // These can be used to implement some dynamic branching in the task graph.
    class FmTaskGraphEvent
    {
        FmArray<FmTaskGraphNode*> successors;

    public:
        FM_CLASS_NEW_DELETE(FmTaskGraphEvent)

        void AddSuccessor(FmTaskGraphNode* node)
        {
            successors.Add(node);
            node->IncrementNumPredecessors();
        }

        // Called to signal completion to successor node, which may be ready to run.
        // If ppNextNode non-NULL, and *ppNextNode initialized to NULL, one of the ready successors will be returned for this thread to run.
        // The remaining ready nodes will be submitted tasks.
        void SignalSuccessors(int32_t message, FmTaskGraphNode** ppNextNode = NULL)
        {
            int numSuccessors = (int)successors.GetNumElems();
            for (int i = 0; i < numSuccessors; i++)
            {
                successors[i]->PredecessorComplete(message, ppNextNode);
            }
        }

        uint32_t GetNumSuccessors()
        {
            return (uint32_t)successors.GetNumElems();
        }

        FmArray<FmTaskGraphNode*>& GetSuccessors()
        {
            return successors;
        }
    };

    // A task graph that can be run either with a wait until completion, or asychronously with a follow-up task to run once completion is detected.
    class FmTaskGraph
    {
        FmAsyncTasksProgress progress;

        FmSubmitAsyncTaskCallback SubmitAsyncTask;
#if !FM_ASYNC_THREADING
        FmCreateTaskWaitCounterCallback CreateTaskWaitCounter;
        FmWaitForTaskWaitCounterCallback WaitForTaskWaitCounter;
        FmDestroyTaskWaitCounterCallback DestroyTaskWaitCounter;
        FmSubmitTaskCallback SubmitTask;

        FmTaskWaitCounter* waitCounter; // Count of running nodes, after Start() value of 0 will signify graph is complete
#endif

        FmTaskGraphEvent startEvent;

    public:
        FM_CLASS_NEW_DELETE(FmTaskGraph)

        FmTaskGraph()
        {
            SubmitAsyncTask = NULL;
#if !FM_ASYNC_THREADING
            CreateTaskWaitCounter = NULL;
            WaitForTaskWaitCounter = NULL;
            DestroyTaskWaitCounter = NULL;
            SubmitTask = NULL;
            waitCounter = NULL;
#endif
        }

        void Destroy()
        {
            startEvent.GetSuccessors().Clear();
        }

        void SetCallbacks(
            FmSubmitAsyncTaskCallback InSubmitAsyncTask
#if !FM_ASYNC_THREADING
            , FmCreateTaskWaitCounterCallback InCreateTaskWaitCounter,
            FmWaitForTaskWaitCounterCallback InWaitForTaskWaitCounter,
            FmDestroyTaskWaitCounterCallback InDestroyTaskWaitCounter,
            FmSubmitTaskCallback InSubmitTask
#endif
        )
        {
            SubmitAsyncTask = InSubmitAsyncTask;
#if !FM_ASYNC_THREADING
            CreateTaskWaitCounter = InCreateTaskWaitCounter;
            WaitForTaskWaitCounter = InWaitForTaskWaitCounter;
            DestroyTaskWaitCounter = InDestroyTaskWaitCounter;
            SubmitTask = InSubmitTask;
#endif
        }

        void SetFollowTask(FmTaskFuncCallback followFunc, void* followData)
        {
            progress.Init(0, followFunc, followData);
        }

        void AddToStart(FmTaskGraphNode* node)
        {
            startEvent.AddSuccessor(node);
        }

        void SubmitNodeTask(FmTaskGraphNode* node, FmTaskGraphNode** ppNextNode = NULL)
        {
#if FM_ASYNC_THREADING
            progress.TaskIsStarting();
            if (ppNextNode)
            {
                // Saving one successor task to run directly, instead of submitting as task
                if (*ppNextNode == NULL)
                {
                    // First successor reached
                    *ppNextNode = node;
                }
                else
                {
                    // Run the largest-weight successor directly, submit the others.
                    // This may help perf by reducing delays in the longest execution path.
                    FmTaskGraphNode* nodeToSubmit = node;
                    FmTaskGraphNode* nodeToRun = *ppNextNode;

                    if (node->GetWeight() > nodeToRun->GetWeight())
                    {
                        nodeToSubmit = *ppNextNode;
                        nodeToRun = node;
                    }

                    SubmitAsyncTask(nodeToSubmit->GetName(), nodeToSubmit->GetTaskFuncWrapped(), nodeToSubmit, nodeToSubmit->GetIndex(), nodeToSubmit->GetIndex() + 1);
                    *ppNextNode = nodeToRun;
                }
            }
            else
            {
                SubmitAsyncTask(node->GetName(), node->GetTaskFuncWrapped(), node, node->GetIndex(), node->GetIndex() + 1);
            }
#else
            (void)ppNextNode;
            SubmitTask(node->GetName(), node->GetTaskFunc(), node, node->GetIndex(), 0, waitCounter);
#endif
        }

#if !FM_ASYNC_THREADING
        // Signal start and wait for graph to complete
        void StartAndWait()
        {
            waitCounter = CreateTaskWaitCounter();

            // Signal starting nodes
            startEvent.SignalSuccessors(0);

            WaitForTaskWaitCounter(waitCounter);

            DestroyTaskWaitCounter(waitCounter);

            waitCounter = NULL;
        }
#endif

        // Start graph using asynchronous threading.  Assumes follow task already set with SetFollowTask().
        // May call FmSetNextTask().  Must be called within FmExecuteTask() loop.
        void StartAsync()
        {
            // The task count is always incremented by one before a node is submitted, and that node won't decrement until it has 
            // finished incrementing the count by one for each of the successor nodes that it submits.  This means that the initial
            // increment can't be canceled out until all successive work for that node has completed.
            // By the same logic we first increment by one before signalling the start nodes, and decrement after.

            // Starting node, first increment by one
            TaskIsStarting();

            // Signal starting nodes
            FmTaskGraphNode* nextNode = NULL;
            int numSuccessors = (int)startEvent.GetNumSuccessors();
            for (int i = 0; i < numSuccessors; i++)
            {
                startEvent.GetSuccessors()[i]->PredecessorComplete(0, &nextNode);
            }

            // If a successor node claimed by this thread, can set it as next task to be executed in loop assumed to be in the callstack.
            // If the task exists, the TaskIsFinished() below can not detect the graph is complete and set a different task.
            if (nextNode)
            {
                FmSetNextTask(nextNode->GetTaskFunc(), nextNode, nextNode->GetIndex(), nextNode->GetIndex() + 1);
            }

            TaskIsFinished();
        }

        // Start graph using asynchronous threading and provide a task to run when complete.
        // May call FmSetNextTask().  Must be called within FmExecuteTask() loop.
        void StartAsync(FmTaskFuncCallback followFunc, void* followData)
        {
            SetFollowTask(followFunc, followData);
            StartAsync();
        }

        // Must use to increment running task count before submitting a task
        void TaskIsStarting()
        {
            progress.TaskIsStarting();
        }

        // Indicates task is complete, and must call from a node's task function
        void TaskIsFinished()
        {
            progress.TaskIsFinished();
        }
    };

    inline void FmTaskGraphNode::PredecessorComplete(int32_t message, FmTaskGraphNode** ppNextNode)
    {
        int32_t newNumPredecessorsIncomplete = FmAtomicDecrement(&numPredecessorsIncomplete.val);

        FM_ASSERT(newNumPredecessorsIncomplete >= 0);

        // If this was the last predecessor being waited on, enqueue the task.
        // It will signal successors after it is run
        if (newNumPredecessorsIncomplete == 0)
        {
            predecessorMessage = message;

            // Reset num predecessors for subsequent iteration, before submitting
            FmAtomicWrite(&numPredecessorsIncomplete.val, numPredecessors);

            graph->SubmitNodeTask(this, ppNextNode);
        }
    }

    inline void FmTaskGraphNode::Run()
    {
        nodeFunc(this, nodeIndex, 0);
    }

    inline void FmTaskGraphNode::TaskIsFinished(int32_t message, FmTaskGraphNode** ppNextNode)
    {
        SignalSuccessors(message, ppNextNode);

#if FM_ASYNC_THREADING
        // Set next task, which requires this is the tail of task reached from FmExecuteTask.
        // NOTE: if there is ppNextNode, graph->TaskIsFinished() can't finish the graph and can't also set a task
        if (ppNextNode && *ppNextNode)
        {
            FmTaskGraphNode* pNextNode = *ppNextNode;
            FmSetNextTask(pNextNode->GetTaskFunc(), pNextNode, pNextNode->GetIndex(), pNextNode->GetIndex() + 1);
        }
#endif

        graph->TaskIsFinished();
    }

}
