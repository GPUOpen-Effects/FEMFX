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
// Construction and execution of dependency graph using task scheduler
//---------------------------------------------------------------------------------------

#include "FEMFXAsyncThreading.h"
#include "FEMFXConstraintSolveTaskGraph.h"
#include "FEMFXConstraintSolver.h"
#include "FEMFXScene.h"

#if FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH

namespace AMD
{
    // Allocate task graph and nodes needed for one outer iteration of constraint solve.
    FmConstraintSolveTaskGraph* FmAllocConstraintSolveTaskGraph(
        FmScene* scene,
        FmConstraintSolverData* constraintSolverData,
        const FmConstraintIsland* constraintIsland,
        FmTaskFuncCallback partitionPairPgsTask, FmTaskFuncCallback partitionPairPgsTaskWrapped, // Wrapped task is run within FmExecuteTask loop for tail call optimization
        FmTaskFuncCallback partitionMpcgTask, FmTaskFuncCallback partitionMpcgTaskWrapped,
        FmTaskFuncCallback partitionGsTask, FmTaskFuncCallback partitionGsTaskWrapped,
        FmTaskFuncCallback externalConstraintsTask, FmTaskFuncCallback externalConstraintsTaskWrapped)
    {
        FmConstraintSolveTaskGraph* taskGraph = new FmConstraintSolveTaskGraph();

        taskGraph->solveData.scene = scene;
        taskGraph->solveData.constraintSolverData = constraintSolverData;
        taskGraph->solveData.constraintIsland = constraintIsland;

        uint numPartitionPairs = constraintSolverData->numPartitionPairs;
        uint numPartitions = constraintSolverData->numPartitions;

        taskGraph->SetCallbacks(
            scene->taskSystemCallbacks.SubmitAsyncTask
#if !FM_ASYNC_THREADING
            , scene->taskSystemCallbacks.CreateTaskWaitCounter,
            scene->taskSystemCallbacks.WaitForTaskWaitCounter,
            scene->taskSystemCallbacks.DestroyTaskWaitCounter,
            scene->taskSystemCallbacks.SubmitTask
#endif
        );

        taskGraph->partitionPairConstraintNodes = new FmPartitionPairConstraintNodeState[numPartitionPairs + 1];  // one more node added for processing of external constraints
        taskGraph->numPartitionPairs = numPartitionPairs;
        taskGraph->numPartitions = numPartitions;

        for (uint i = 0; i < numPartitionPairs + 1; i++)
        {
            uint weight = (uint)-1;
            if (i < numPartitionPairs)
            {
                FmPartitionPair& partitionPair = constraintSolverData->partitionPairs[i];
                weight = partitionPair.numConstraints;
            }
            FmTaskFuncCallback taskFunc;
            FmTaskFuncCallback taskFuncWrapped;
            const char* taskName;
            if (i == numPartitionPairs)
            {
                taskFunc = externalConstraintsTask;
                taskFuncWrapped = externalConstraintsTaskWrapped;
                taskName = "ExternalConstraints";
            }
            else
            {
                taskFunc = partitionPairPgsTask;
                taskFuncWrapped = partitionPairPgsTaskWrapped;
                taskName = "PartitionPairPgs";
            }

            taskGraph->partitionPairConstraintNodes[i].firstIterationNode.Init(taskName, taskGraph, taskFunc, taskFuncWrapped, (int)i, weight);

            taskGraph->partitionPairConstraintNodes[i].repeat0Node.Init(taskName, taskGraph, taskFunc, taskFuncWrapped, (int)i, weight);

            taskGraph->partitionPairConstraintNodes[i].repeat1Node.Init(taskName, taskGraph, taskFunc, taskFuncWrapped, (int)i, weight);

            taskGraph->partitionPairConstraintNodes[i].nextOuterIterationNode.Init(taskName, taskGraph, taskFunc, taskFuncWrapped, (int)i, weight);
        }

        taskGraph->partitionObjectNodes = new FmPartitionObjectNodeState[numPartitions];

        for (uint i = 0; i < numPartitions; i++)
        {
            taskGraph->partitionObjectNodes[i].mpcgSolveNode.Init("PartitionMpcg", taskGraph, partitionMpcgTask, partitionMpcgTaskWrapped, (int)i);

            taskGraph->partitionObjectNodes[i].gsIterationRbDeltaNode.Init("PartitionGs", taskGraph, partitionGsTask, partitionGsTaskWrapped, (int)i);
        }

        return taskGraph;
    }

    // Free task graph.
    void FmDestroyConstraintSolveTaskGraph(FmConstraintSolveTaskGraph* taskGraph)
    {
        if (taskGraph)
        {
            taskGraph->FmTaskGraph::Destroy();
            delete[] taskGraph->partitionPairConstraintNodes;
            delete[] taskGraph->partitionObjectNodes;

            delete taskGraph;
        }
    }

    FM_WRAPPED_TASK_DECLARATION(FmNodeTaskFuncProcessPartitionPairConstraints)
    FM_WRAPPED_TASK_DECLARATION(FmNodeTaskFuncProcessPartitionMpcg)
    FM_WRAPPED_TASK_DECLARATION(FmNodeTaskFuncProcessPartitionGsIterationOrRbResponse)
    FM_WRAPPED_TASK_DECLARATION(FmNodeTaskFuncExternalPgsIteration)
        
    void FmCreateConstraintSolveTaskGraph(FmScene* scene, FmConstraintSolverData* constraintSolverData, const FmConstraintIsland* constraintIsland)
    {
        FM_ASSERT(constraintSolverData->taskGraph == NULL);

        constraintSolverData->taskGraph = FmAllocConstraintSolveTaskGraph(
            scene,
            constraintSolverData,
            constraintIsland,
            FM_TASK_AND_WRAPPED_TASK_ARGS(FmNodeTaskFuncProcessPartitionPairConstraints),
            FM_TASK_AND_WRAPPED_TASK_ARGS(FmNodeTaskFuncProcessPartitionMpcg),
            FM_TASK_AND_WRAPPED_TASK_ARGS(FmNodeTaskFuncProcessPartitionGsIterationOrRbResponse),
            FM_TASK_AND_WRAPPED_TASK_ARGS(FmNodeTaskFuncExternalPgsIteration));

        uint numPartitionPairs = constraintSolverData->numPartitionPairs;
        uint numPartitions = constraintSolverData->numPartitions;
        uint numColors = constraintSolverData->numPartitionPairIndependentSets;

        // Link GS and MPCG nodes to preceding and subsequent constraint nodes.
        for (uint partitionIdx = 0; partitionIdx < numPartitions; partitionIdx++)
        {
            // Check for dependencies from last to first color.
            // Only need to link to nodes in the first preceding color found, which will be linked to any previous nodes with the same partition.
            for (int colorIdx = (int)numColors - 1; colorIdx >= 0; colorIdx--)
            {
                FmGraphColoringSet& color = constraintSolverData->partitionPairIndependentSets[colorIdx];
                uint numInColor = color.numElements;

                bool foundInColor = false;

                for (uint colorElemIdx = 0; colorElemIdx < numInColor; colorElemIdx++)
                {
                    uint pairIdx = color.pStart[colorElemIdx];
                    FmPartitionPair& partitionPair = constraintSolverData->partitionPairs[pairIdx];

                    if (partitionIdx == partitionPair.partitionIdA || partitionIdx == partitionPair.partitionIdB)
                    {
                        FmMakePartitionTaskDependency(constraintSolverData->taskGraph, pairIdx, partitionIdx);

                        foundInColor = true;
                    }
                }

                if (foundInColor)
                {
                    break;
                }
            }

            // Link to partition pair nodes in the next outer iteration
            for (uint colorIdx = 0; colorIdx < numColors; colorIdx++)
            {
                FmGraphColoringSet& color = constraintSolverData->partitionPairIndependentSets[colorIdx];
                uint numInColor = color.numElements;

                bool foundInColor = false;

                for (uint colorElemIdx = 0; colorElemIdx < numInColor; colorElemIdx++)
                {
                    uint pairIdx = color.pStart[colorElemIdx];
                    FmPartitionPair& partitionPair = constraintSolverData->partitionPairs[pairIdx];

                    if (partitionIdx == partitionPair.partitionIdA || partitionIdx == partitionPair.partitionIdB)
                    {
                        FmMakeNextOuterIterationDependency(constraintSolverData->taskGraph, partitionIdx, pairIdx);

                        foundInColor = true;
                    }
                }

                if (foundInColor)
                {
                    break;
                }
            }
        }

        FmPartitionPairConstraintNodeState* partitionPairConstraintNodes = constraintSolverData->taskGraph->partitionPairConstraintNodes;

        // Link constraint nodes with dependencies
        for (uint colorIdx = 0; colorIdx < numColors; colorIdx++)
        {
            FmGraphColoringSet& color = constraintSolverData->partitionPairIndependentSets[colorIdx];
            uint numInColor = color.numElements;

            for (uint colorElemIdx = 0; colorElemIdx < numInColor; colorElemIdx++)
            {
                uint pairIdx = color.pStart[colorElemIdx];
                FmPartitionPair& partitionPair = constraintSolverData->partitionPairs[pairIdx];

                bool nodeHasRigidBodies = partitionPair.numRigidBodies > 0;
                bool nodeHasSameIterationSuccessorWithRigidBodies = false;

                // Create edges from start node to any node without predecessor
                if (partitionPairConstraintNodes[pairIdx].numPredecessorsSameIteration == 0)
                {
                    FmMakeStartDependency(constraintSolverData->taskGraph, pairIdx);
                }

                bool foundA = false;
                bool foundB = false;

                // Make links to subsequent colors with the same partitions.
                // Only need to link to nodes in the first subsequent color found; links will be made from that color to any subsequent nodes with this partition.
                for (uint futureColorIdx = colorIdx + 1; futureColorIdx < numColors * 2; futureColorIdx++)
                {
                    bool nextIteration = futureColorIdx >= numColors;
                    uint nextColorIdx = futureColorIdx % numColors;

                    FmGraphColoringSet& nextColor = constraintSolverData->partitionPairIndependentSets[nextColorIdx];
                    uint numInNextColor = nextColor.numElements;

                    bool foundAInColor = false;
                    bool foundBInColor = false;

                    for (uint nextColorElemIdx = 0; nextColorElemIdx < numInNextColor; nextColorElemIdx++)
                    {
                        uint nextPairIdx = nextColor.pStart[nextColorElemIdx];
                        FmPartitionPair& nextPartitionPair = constraintSolverData->partitionPairs[nextPairIdx];

                        if (!foundA && (partitionPair.partitionIdA == nextPartitionPair.partitionIdA || partitionPair.partitionIdA == nextPartitionPair.partitionIdB))
                        {
                            if (nextIteration)
                            {
                                FmMakeNextIterationDependency(constraintSolverData->taskGraph, pairIdx, nextPairIdx);
                            }
                            else
                            {
                                FmMakeSameIterationDependency(constraintSolverData->taskGraph, pairIdx, nextPairIdx);

                                partitionPairConstraintNodes[nextPairIdx].numPredecessorsSameIteration++;
                                if (nodeHasRigidBodies)
                                {
                                    partitionPairConstraintNodes[nextPairIdx].hasSameIterationPredecessorWithRigidBodies = true;
                                }
                                if (nextPartitionPair.numRigidBodies > 0)
                                {
                                    nodeHasSameIterationSuccessorWithRigidBodies = true;
                                }
                            }
                            foundAInColor = true;

                            if (partitionPair.partitionIdA == partitionPair.partitionIdB)
                            {
                                foundBInColor = true;
                            }
                        }

                        if (partitionPair.partitionIdA != partitionPair.partitionIdB)
                        {
                            if (!foundB && (partitionPair.partitionIdB == nextPartitionPair.partitionIdA || partitionPair.partitionIdB == nextPartitionPair.partitionIdB))
                            {
                                if (nextIteration)
                                {
                                    FmMakeNextIterationDependency(constraintSolverData->taskGraph, pairIdx, nextPairIdx);
                                }
                                else
                                {
                                    FmMakeSameIterationDependency(constraintSolverData->taskGraph, pairIdx, nextPairIdx);

                                    partitionPairConstraintNodes[nextPairIdx].numPredecessorsSameIteration++;
                                    if (nodeHasRigidBodies)
                                    {
                                        partitionPairConstraintNodes[nextPairIdx].hasSameIterationPredecessorWithRigidBodies = true;
                                    }
                                    if (nextPartitionPair.numRigidBodies > 0)
                                    {
                                        nodeHasSameIterationSuccessorWithRigidBodies = true;
                                    }
                                }
                                foundBInColor = true;
                            }
                        }
                    }

                    if (foundAInColor)
                    {
                        foundA = true;
                    }
                    if (foundBInColor)
                    {
                        foundB = true;
                    }

                    if (foundA && foundB)
                    {
                        break;
                    }
                }

                // Add node for processing external constraints, if island has rigid bodies, and callback to external system exists
                if (constraintIsland->numUserRigidBodyIslands > 0 && constraintIsland->innerIterationCallback)
                {
                    if (nodeHasRigidBodies && !nodeHasSameIterationSuccessorWithRigidBodies)
                    {
                        FmMakeSameIterationDependency(constraintSolverData->taskGraph, pairIdx, numPartitionPairs);
                    }
                    if (nodeHasRigidBodies && !partitionPairConstraintNodes[pairIdx].hasSameIterationPredecessorWithRigidBodies)
                    {
                        FmMakeNextIterationDependency(constraintSolverData->taskGraph, numPartitionPairs, pairIdx);
                    }
                }
            }
        }
    }

    // Run task graph to execute one outer iteration of constraint solve, and call its follow task when finished
    void FmRunConstraintSolveTaskGraphAsync(FmConstraintSolveTaskGraph* taskGraph, FmTaskFuncCallback followTask, void* followTaskData)
    {
        taskGraph->StartAsync(followTask, followTaskData);
    }

#if !FM_ASYNC_THREADING
    // Run task graph to execute one outer iteration of constraint solve.
    void FmRunConstraintSolveTaskGraph(FmConstraintSolveTaskGraph* taskGraph)
    {
        taskGraph->StartAndWait();
    }
#endif

    // Make edge from start node to partition pair node.
    void FmMakeStartDependency(FmConstraintSolveTaskGraph* graph, uint partitionPairIdx)
    {
        graph->AddToStart(&graph->partitionPairConstraintNodes[partitionPairIdx].firstIterationNode);
    }

    // Make edge from partition pair node to different node in the same PGS iteration.
    void FmMakeSameIterationDependency(FmConstraintSolveTaskGraph* graph, uint partitionPairAIdx, uint partitionPairBIdx)
    {
        graph->partitionPairConstraintNodes[partitionPairAIdx].firstIterationNode.AddSuccessor(&graph->partitionPairConstraintNodes[partitionPairBIdx].firstIterationNode);
        graph->partitionPairConstraintNodes[partitionPairAIdx].repeat0Node.AddSuccessor(&graph->partitionPairConstraintNodes[partitionPairBIdx].repeat0Node);
        graph->partitionPairConstraintNodes[partitionPairAIdx].repeat1Node.AddSuccessor(&graph->partitionPairConstraintNodes[partitionPairBIdx].repeat1Node);
        graph->partitionPairConstraintNodes[partitionPairAIdx].nextOuterIterationNode.AddSuccessor(&graph->partitionPairConstraintNodes[partitionPairBIdx].nextOuterIterationNode);
    }

    // Register dependency between partition pair node and node in the next PGS iteration.
    void FmMakeNextIterationDependency(FmConstraintSolveTaskGraph* graph, uint partitionPairAIdx, uint partitionPairBIdx)
    {
        // Register a predecessor but not an edge, which allows the messages to not be passed if exiting iterations
        graph->partitionPairConstraintNodes[partitionPairAIdx].nextIteration0Successors.AddSuccessor(&graph->partitionPairConstraintNodes[partitionPairBIdx].repeat0Node);
        graph->partitionPairConstraintNodes[partitionPairAIdx].nextIteration1Successors.AddSuccessor(&graph->partitionPairConstraintNodes[partitionPairBIdx].repeat1Node);
    }

    // Register dependency between partition pair node and partition node in the next solving phase.
    void FmMakePartitionTaskDependency(FmConstraintSolveTaskGraph* graph, uint partitionPairIdx, uint partitionIdx)
    {
        graph->partitionPairConstraintNodes[partitionPairIdx].partitionGsSuccessors.AddSuccessor(&graph->partitionObjectNodes[partitionIdx].gsIterationRbDeltaNode);

        graph->partitionPairConstraintNodes[partitionPairIdx].partitionMpcgSuccessors.AddSuccessor(&graph->partitionObjectNodes[partitionIdx].mpcgSolveNode);
    }

    // Register dependency between partition node and partition pair node in the next outer solve iteration
    void FmMakeNextOuterIterationDependency(FmConstraintSolveTaskGraph* graph, uint partitionIdx, uint partitionPairIdx)
    {
        graph->partitionObjectNodes[partitionIdx].partitionPairSuccessors.AddSuccessor(&graph->partitionPairConstraintNodes[partitionPairIdx].nextOuterIterationNode);
    }

    // Send messages to partition pair nodes in next iteration.
    void FmNextIterationMessages(FmConstraintSolveTaskGraph* graph, uint partitionPairIdx, uint iteration, FmTaskGraphNode** ppNextNode)
    {
        FmPartitionPairConstraintNodeState& node = graph->partitionPairConstraintNodes[partitionPairIdx];

        if ((iteration % 2) == 0)
        {
            node.nextIteration0Successors.SignalSuccessors(0, ppNextNode);
        }
        else
        {
            node.nextIteration1Successors.SignalSuccessors(0, ppNextNode);
        }
    }

    // Send messages to partition nodes to run MPCG.
    void FmPartitionMpcgTaskMessages(FmConstraintSolveTaskGraph* graph, uint partitionPairIdx, uint outerIteration, FmTaskGraphNode** ppNextNode)
    {
        FmPartitionPairConstraintNodeState& node = graph->partitionPairConstraintNodes[partitionPairIdx];

        node.partitionMpcgSuccessors.SignalSuccessors((int32_t)outerIteration, ppNextNode);
    }

    // Send messages to partition nodes to run GS iteration.
    void FmPartitionGsTaskMessages(FmConstraintSolveTaskGraph* graph, uint partitionPairIdx, uint outerIteration, FmTaskGraphNode** ppNextNode)
    {
        FmPartitionPairConstraintNodeState& node = graph->partitionPairConstraintNodes[partitionPairIdx];

        node.partitionGsSuccessors.SignalSuccessors((int32_t)outerIteration, ppNextNode);
    }

    // Send messages to partition pair nodes in next outer iteration.
    void FmNextOuterIterationMessages(FmConstraintSolveTaskGraph* graph, uint partitionIdx, FmTaskGraphNode** ppNextNode)
    {
        graph->partitionObjectNodes[partitionIdx].partitionPairSuccessors.SignalSuccessors(0, ppNextNode);
    }
}

#endif