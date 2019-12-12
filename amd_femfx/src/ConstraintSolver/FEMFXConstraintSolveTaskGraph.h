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

#include "AMD_FEMFX.h"
#include "FEMFXTaskGraph.h"

namespace AMD
{
    struct FmConstraintSolverData;
    struct FmConstraintIsland;

    struct FmTaskGraphSolveData
    {
        FmScene* scene;
        FmConstraintSolverData* constraintSolverData;
        const FmConstraintIsland* constraintIsland;

        FmTaskGraphSolveData() : scene(NULL), constraintSolverData(NULL), constraintIsland(NULL) {}
    };

#if FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH
    // State per each partition-pair node needed for repeating iteration or calling subsequent tasks
    class FmPartitionPairConstraintNodeState
    {
    public:
        FM_CLASS_NEW_DELETE(FmPartitionPairConstraintNodeState)

        uint passIdx;
        uint innerIteration;
        uint outerIteration;
        uint numPredecessorsSameIteration;
        uint numTimesReached;
        bool hasSameIterationPredecessorWithRigidBodies;

        FmTaskGraphNode firstIterationNode;  // Node for task in the first iteration; start node may link to this
        FmTaskGraphNode repeat0Node;         // Alternating between two copies of nodes to support PGS solve inner loop
        FmTaskGraphNode repeat1Node;
        FmTaskGraphNode nextOuterIterationNode;  // Node for task in the next outer iteration

        FmTaskGraphEvent nextIteration0Successors; // Successor partition pairs, which are messaged if continuing to iterate the PGS constraint solve
        FmTaskGraphEvent nextIteration1Successors; // Successor partition pairs, which are messaged if continuing to iterate the PGS constraint solve
        FmTaskGraphEvent partitionMpcgSuccessors;  // Successor partitions, which are messaged if moving to object GS or MPCG solving
        FmTaskGraphEvent partitionGsSuccessors;    // Successor partitions, which are messaged if moving to object GS or MPCG solving

        FmPartitionPairConstraintNodeState()
        {
            passIdx = 0;
            innerIteration = 0;
            outerIteration = 0;
            numPredecessorsSameIteration = 0;
            numTimesReached = 0;
            hasSameIterationPredecessorWithRigidBodies = false;
        }
    };

    // Graph nodes for partition object solving tasks.
    class FmPartitionObjectNodeState
    {
    public:
        FM_CLASS_NEW_DELETE(FmPartitionObjectNodeState)

        FmTaskGraphNode  gsIterationRbDeltaNode;
        FmTaskGraphNode  mpcgSolveNode;
        FmTaskGraphEvent partitionPairSuccessors;     // Successor partition pairs, when restarting outer iteration
    };

    // Contains the task graph and nodes needed for one outer iteration of constraint solve.
    class FmConstraintSolveTaskGraph : public FmTaskGraph
    {
    public:
        FM_CLASS_NEW_DELETE(FmConstraintSolveTaskGraph)

        FmTaskGraphSolveData                 solveData;
        FmPartitionPairConstraintNodeState*  partitionPairConstraintNodes;
        FmPartitionObjectNodeState*          partitionObjectNodes;
        uint                                 numPartitions;
        uint                                 numPartitionPairs;

        FmConstraintSolveTaskGraph()
        {
            partitionPairConstraintNodes = NULL;
            partitionObjectNodes = NULL;
            numPartitions = 0;
            numPartitionPairs = 0;
        }
    };

    // Create task graph
    void FmCreateConstraintSolveTaskGraph(FmScene* scene, FmConstraintSolverData* constraintSolverData, const FmConstraintIsland* constraintIsland);

    // Free task graph
    void FmDestroyConstraintSolveTaskGraph(FmConstraintSolveTaskGraph* taskGraph);

    // Run task graph to execute one outer iteration of constraint solve.
    void FmRunConstraintSolveTaskGraph(FmConstraintSolveTaskGraph* taskGraph);

    // Run task graph to execute both CG and GS passes.
    // This will skip over the Start, Middle, End tasks that used to implement the outer loops and transition between passes.  
    // Loop counters and other control parameters are accessible or updated by the graph. 
    // The graph will start with the GS pass if there are no CG pass iterations.
    void FmRunConstraintSolveTaskGraphAsync(FmConstraintSolveTaskGraph* taskGraph, FmTaskFuncCallback followTask, void* followTaskData);

    // Make edge from start node to partition pair node.
    void FmMakeStartDependency(FmConstraintSolveTaskGraph* graph, uint partitionPairIdx);

    // Make edge from partition pair node to different node in the same PGS iteration.
    void FmMakeSameIterationDependency(FmConstraintSolveTaskGraph* graph, uint partitionPairAIdx, uint partitionPairBIdx);

    // Register dependency between partition pair node and node in the next PGS iteration.
    void FmMakeNextIterationDependency(FmConstraintSolveTaskGraph* graph, uint partitionPairAIdx, uint partitionPairBIdx);

    // Register dependency between partition pair node and partition node in the next solving phase.
    void FmMakePartitionTaskDependency(FmConstraintSolveTaskGraph* graph, uint partitionPairIdx, uint partitionIdx);

    // Register dependency between partition node and partition pair node in the next outer solve iteration
    void FmMakeNextOuterIterationDependency(FmConstraintSolveTaskGraph* graph, uint partitionIdx, uint partitionPairIdx);

    class FmTaskGraphNode;

    // Send messages to partition pair nodes in next iteration.
    void FmNextIterationMessages(FmConstraintSolveTaskGraph* graph, uint partitionPairIdx, uint iteration, FmTaskGraphNode** ppNextNode);

    // Send messages to partition nodes to run MPCG.
    void FmPartitionMpcgTaskMessages(FmConstraintSolveTaskGraph* graph, uint partitionPairIdx, uint outerIteration, FmTaskGraphNode** ppNextNode);

    // Send messages to partition nodes to run GS iteration.
    void FmPartitionGsTaskMessages(FmConstraintSolveTaskGraph* graph, uint partitionPairIdx, uint outerIteration, FmTaskGraphNode** ppNextNode);

    // Send messages to partition pair nodes in next outer iteration.
    void FmNextOuterIterationMessages(FmConstraintSolveTaskGraph* graph, uint partitionIdx, FmTaskGraphNode** ppNextNode);
#endif
}