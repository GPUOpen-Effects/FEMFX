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
// Sort functions, modified versions of qsort
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXTaskGraph.h"
#include "FEMFXParallelFor.h"

// Comparison result for x and y to sort in increasing order
#define FM_QSORT_INCREASING_RETVAL(x, y) (((x) > (y)) - ((x) < (y)))

// Comparison result for x and y to sort in decreasing order
#define FM_QSORT_DECREASING_RETVAL(x, y) (((x) < (y)) - ((x) > (y)))

// Standard qsort with userdata pointer provided to callback
void
qsort_userdata(void *a, void *userdata, size_t n, size_t es,
    int(*cmp) (const void *, const void *, void*));

// Standard qsort with permutation vector computed along with sorted result
void
qsort_permutation(void *a, int *permutation, size_t n, size_t es,
    int(*cmp) (const void *, const void *));

namespace AMD
{
    template<class T>
    void FmSortTask(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    template<class T>
    void FmSortTaskWrapped(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    template<class T>
    void FmMergeTask(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    template<class T>
    void FmMergeTaskWrapped(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    class FmSortTaskGraphNode : public FmTaskGraphNode
    {
    public:
        FM_CLASS_NEW_DELETE(FmSortTaskGraphNode)

        uint beginIndex;
        uint middleIndex;
        uint endIndex;
        uint srcBufferIndex;

        FmSortTaskGraphNode() : beginIndex(0), middleIndex(0), endIndex(0), srcBufferIndex(0) {}
    };

    /* Class to define for comparisons
    class SortCompareClass
    {
    public:
        void* userData;

        SortCompareClass(void* inUserData)
        {
            userData = inUserData;
        }

        // Return if refA and refB in proper order
        inline bool operator ()(const T& refA, const T& refB) const
        {
        }

        // Return -1 for in order, 0 for equal, 1 for out of order
        inline int Compare(const T& refA, const T& refB)
        {
        }
    };
    */

    template<class T, class CompareClass>
    int FmQSortCompareFunc(const void* inA, const void* inB, void* userData)
    {
        const T& a = *(T*)inA;
        const T& b = *(T*)inB;

        CompareClass compare(userData);
        return compare.Compare(a, b);
    }

    template<class T, class CompareClass>
    void FmSort(T* pElems, uint numElems, void* userData)
    {
#if 0
        qsort_userdata(pElems, userData, numElems, sizeof(T), FmQSortCompareFunc<T, CompareClass>);
#else
        std::sort(pElems, pElems + numElems, CompareClass(userData));
#endif
    }

    // Graph structure for parallel sort of an array.
    // Divides array into fixed number of sub-arrays for sort, then merges.
    // Requires definition of templated class FmSortCompareClass taking void pointer in constructor.
    // TODO: look at incorporating parallel merge; or avoid use of secondary buffer.
    template<class T, class CompareClass>
    class FmSortTaskGraph : public FmTaskGraph
    {
    public:
        FM_CLASS_NEW_DELETE(FmSortTaskGraph)

        FmSortTaskGraphNode* nodes;
        uint                 numSortNodes;
        T*                   pElemsBuffer[2];
        uint                 numElements;
        void*                sortClassUserData;

        FmSortTaskGraph() : nodes(NULL), numSortNodes(0), numElements(0), sortClassUserData(NULL) 
        {
            pElemsBuffer[0] = NULL;
            pElemsBuffer[1] = NULL;
        }

        ~FmSortTaskGraph()
        {
            Destroy();
        }

        void Destroy()
        {
            if (nodes)
            {
                delete[] nodes;
            }
            nodes = NULL;
            numSortNodes = 0;
            pElemsBuffer[0] = NULL;
            pElemsBuffer[1] = NULL;
            numElements = 0;
            sortClassUserData = 0;
        }

        void CreateAndRunGraph(
            T** outElemsSorted, T* inElemsUnsorted, T* inElemsBuffer, uint inNumElements,
            void* inSortClassUserData, 
            uint inMaxSortNodes, uint inBatchSize, FmTaskFuncCallback followTask, void* followTaskData)
        {
            (void)inElemsBuffer;
            *outElemsSorted = inElemsUnsorted;

            if (inNumElements < 2)
            {
                FmSetNextTask(followTask, followTaskData, 0, 1);
                return;
            }

            // Use power of two nodes
            numSortNodes = FmNextPowerOf2(inMaxSortNodes);
            pElemsBuffer[0] = inElemsUnsorted;
            pElemsBuffer[1] = inElemsBuffer;
            numElements = inNumElements;
            sortClassUserData = inSortClassUserData;

            // Reduce number of nodes for sufficient batch size
            uint batchSize = inNumElements / numSortNodes;
            while (batchSize < inBatchSize && numSortNodes > 1)
            {
                numSortNodes /= 2;
                batchSize = inNumElements / numSortNodes;
            }

            if (numSortNodes == 1)
            {
                // Sort inline
                FmSort<T, CompareClass>(inElemsUnsorted, numElements, sortClassUserData);
                FmSetNextTask(followTask, followTaskData, 0, 1);
                return;
            }

            // Create nodes
            uint numMergeNodes = numSortNodes - 1;

            nodes = new FmSortTaskGraphNode[numSortNodes + numMergeNodes];

            for (uint i = 0; i < numSortNodes; i++)
            {
                nodes[i].Init("FmSortNode", this, FmSortTask<T, CompareClass>, FmSortTaskWrapped<T, CompareClass>, i);

                uint nodeBeginIndex, nodeEndIndex;
                FmGetIndexRangeEvenDistribution(&nodeBeginIndex, &nodeEndIndex, i, numSortNodes, numElements);

                nodes[i].beginIndex = nodeBeginIndex;
                nodes[i].middleIndex = nodeEndIndex;
                nodes[i].endIndex = nodeEndIndex;

                AddToStart(&nodes[i]);
            }

            for (uint i = numSortNodes; i < numSortNodes + numMergeNodes; i++)
            {
                nodes[i].Init("FmMergeNode", this, FmMergeTask<T, CompareClass>, FmMergeTaskWrapped<T, CompareClass>, i);
            }

            uint levelNumNodes = numSortNodes;
            uint levelBeginIndex = 0;
            uint srcBufferIndex = 0;  // First merge level will use src buffer 0

            while (levelNumNodes >= 2)
            {
                for (uint i = 0; i < levelNumNodes; i++)
                {
                    uint nodeIdx = levelBeginIndex + i;
                    uint successorIdx = levelBeginIndex + levelNumNodes + i / 2;

                    nodes[nodeIdx].AddSuccessor(&nodes[successorIdx]);

                    nodes[successorIdx].srcBufferIndex = srcBufferIndex;

                    if (i % 2 == 0)
                    {
                        nodes[successorIdx].beginIndex = nodes[nodeIdx].beginIndex;
                    }
                    else
                    {
                        nodes[successorIdx].middleIndex = nodes[nodeIdx].beginIndex;
                        nodes[successorIdx].endIndex = nodes[nodeIdx].endIndex;
                    }
                }

                srcBufferIndex = 1 - srcBufferIndex;
                levelBeginIndex += levelNumNodes;
                levelNumNodes /= 2;
            }

            //*outElemsSorted = pElemsBuffer[srcBufferIndex];
            *outElemsSorted = pElemsBuffer[0];

#if FM_ASYNC_THREADING
            StartAsync(followTask, followTaskData);
#else
            StartAndWait();
#endif
        }
    };

    template<class T, class CompareClass>
    void FmSortTask(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;

        FM_TRACE_SCOPED_EVENT(SORT_TASK);

        FmSortTaskGraphNode* node = (FmSortTaskGraphNode*)inTaskData;
        FmSortTaskGraph<T, CompareClass>* graph = (FmSortTaskGraph<T, CompareClass>*)node->GetGraph();

        T* pElems = graph->pElemsBuffer[0];
        FmSort<T, CompareClass>(pElems + node->beginIndex, node->endIndex - node->beginIndex, graph->sortClassUserData);

        node->TaskIsFinished(0);
    }

    template<class T, class CompareClass>
    void FmSortTaskWrapped(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        FmExecuteTask(FmSortTask<T, CompareClass>, inTaskData, inTaskBeginIndex, inTaskEndIndex);
    }

    template<class T, class CompareClass>
    void FmMergeTask(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;

        FM_TRACE_SCOPED_EVENT(MERGE_TASK);

        FmSortTaskGraphNode* node = (FmSortTaskGraphNode*)inTaskData;
        FmSortTaskGraph<T, CompareClass>* graph = (FmSortTaskGraph<T, CompareClass>*)node->GetGraph();

        T* pElems = graph->pElemsBuffer[0];
        std::inplace_merge(pElems + node->beginIndex, pElems + node->middleIndex, pElems + node->endIndex, CompareClass(graph->sortClassUserData));

        node->TaskIsFinished(0);
    }

    template<class T, class CompareClass>
    void FmMergeTaskWrapped(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        FmExecuteTask(FmMergeTask<T, CompareClass>, inTaskData, inTaskBeginIndex, inTaskEndIndex);
    }

}
