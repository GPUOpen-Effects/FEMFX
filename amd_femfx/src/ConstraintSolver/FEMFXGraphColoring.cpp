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
// Graph coloring of the object/constraint partitions for multi-threading during 
// constraint solve
//---------------------------------------------------------------------------------------

#include "FEMFXScene.h"
#include "FEMFXConstraintSolver.h"

// Code below includes option of greedy sequential graph coloring or "Luby/Jones-Plassman"
//
// Luby/Jones-Plassman algorithm assigns a random hash value to each node.  On each pass, for each node compares its hash value 
// with all neighbor node hashes, and assigns that pass' color if the node has the maximum hash, or a second color if the node 
// has the minimum hash.   A benefit of this is colors can be assigned in parallel, though the prototype here runs serially.
//
// When coloring contacts, can update max or min hash values in the adjacent objects/vertices, and then check if contact has the
// max or min value, which will compared against all contacts connected to those objects/vertices. 

namespace AMD
{

    // Return if index0 hash is greater, or if index0 greater when hashes equal.
    // Equal indices will give false.
    static inline bool FmCompareGreaterHash(uint index0, uint index1)
    {
        uint hash0 = FmComputeHash(index0);
        uint hash1 = FmComputeHash(index1);
        return (hash0 > hash1) || (hash0 == hash1 && index0 > index1);
    }

    static inline uint FmIndexWithMaxHash(uint index0, uint index1)
    {
        return FmCompareGreaterHash(index0, index1) ? index0 : index1;
    }

    static inline uint FmIndexWithMinHash(uint index0, uint index1)
    {
        return FmCompareGreaterHash(index0, index1) ? index1 : index0;
    }

#define FM_GREEDY_SEQUENTIAL_COLORING 1

#if !FM_GREEDY_SEQUENTIAL_COLORING
    // Iterate over the partition pairs to find min and max hashes for member objects
    void FmObjectsMinMaxHash(
        FmConstraintSolverData* constraintSolverData,
        const FmConstraintIsland& constraintIsland)
    {
        uint numIslandTetMeshes = constraintIsland.numTetMeshes;
        uint numIslandRigidBodies = constraintIsland.numRigidBodiesInFEMSolve;
        uint numIslandPartitionPairs = constraintSolverData->numPartitionPairs;

        for (uint islandMeshIdx = 0; islandMeshIdx < numIslandTetMeshes; islandMeshIdx++)
        {
            FmObjectPartitionData& objectPartitionData = FmGetObjectPartitionDataRef(constraintSolverData, constraintIsland.tetMeshIds[islandMeshIdx]);
            objectPartitionData.minMaxHashPartitionPairs.minIdx = FM_INVALID_ID;
            objectPartitionData.minMaxHashPartitionPairs.maxIdx = FM_INVALID_ID;
        }

        for (uint islandRbIdx = 0; islandRbIdx < numIslandRigidBodies; islandRbIdx++)
        {
            FmObjectPartitionData& objectPartitionData = FmGetObjectPartitionDataRef(constraintSolverData, constraintIsland.rigidBodyIds[islandRbIdx]);
            objectPartitionData.minMaxHashPartitionPairs.minIdx = FM_INVALID_ID;
            objectPartitionData.minMaxHashPartitionPairs.maxIdx = FM_INVALID_ID;
        }

        // For each partition pair, update the min and max hash for objects belonging to pair
        for (uint partitionPairIdx = 0; partitionPairIdx < numIslandPartitionPairs; partitionPairIdx++)
        {
            FmPartitionPair& partitionPair = constraintSolverData->partitionPairs[partitionPairIdx];

            // Skip if batch already assigned color on previous pass
            if (partitionPair.graphColor != FM_INVALID_ID)
                continue;

            uint numTetMeshes = partitionPair.numTetMeshes;
            for (uint pairMeshIdx = 0; pairMeshIdx < numTetMeshes; pairMeshIdx++)
            {
                FmObjectPartitionData& objectPartitionData = FmGetObjectPartitionDataRef(constraintSolverData, constraintIsland.tetMeshIds[pairMeshIdx]);
                uint minIdx = objectPartitionData.minMaxHashPartitionPairs.minIdx;
                uint maxIdx = objectPartitionData.minMaxHashPartitionPairs.maxIdx;
                objectPartitionData.minMaxHashPartitionPairs.minIdx = (minIdx == FM_INVALID_ID) ? partitionPairIdx : FmIndexWithMinHash(partitionPairIdx, minIdx);
                objectPartitionData.minMaxHashPartitionPairs.maxIdx = (maxIdx == FM_INVALID_ID) ? partitionPairIdx : FmIndexWithMaxHash(partitionPairIdx, maxIdx);
            }

            uint numRigidBodies = partitionPair.numRigidBodies;
            for (uint pairRbIdx = 0; pairRbIdx < numRigidBodies; pairRbIdx++)
            {
                FmObjectPartitionData& objectPartitionData = FmGetObjectPartitionDataRef(constraintSolverData, constraintIsland.rigidBodyIds[pairRbIdx]);
                uint minIdx = objectPartitionData.minMaxHashPartitionPairs.minIdx;
                uint maxIdx = objectPartitionData.minMaxHashPartitionPairs.maxIdx;
                objectPartitionData.minMaxHashPartitionPairs.minIdx = (minIdx == FM_INVALID_ID) ? partitionPairIdx : FmIndexWithMinHash(partitionPairIdx, minIdx);
                objectPartitionData.minMaxHashPartitionPairs.maxIdx = (maxIdx == FM_INVALID_ID) ? partitionPairIdx : FmIndexWithMaxHash(partitionPairIdx, maxIdx);
            }
        }
    }
#endif

    struct FmComparePartitionPairs
    {
        FmConstraintSolverData* constraintSolverData;

        FmComparePartitionPairs(void* inConstraintSolverData)
        {
            constraintSolverData = (FmConstraintSolverData*)inConstraintSolverData;
        }

        inline bool operator() (uint idxA, uint idxB)
        {
            FmPartitionPair& pairA = constraintSolverData->partitionPairs[idxA];
            FmPartitionPair& pairB = constraintSolverData->partitionPairs[idxB];

            return pairA.numConstraints > pairB.numConstraints;
        }

        inline int Compare(uint idxA, uint idxB)
        {
            FmPartitionPair& pairA = constraintSolverData->partitionPairs[idxA];
            FmPartitionPair& pairB = constraintSolverData->partitionPairs[idxB];

            return FM_QSORT_DECREASING_RETVAL(pairA.numConstraints, pairB.numConstraints);
        }
    };

    void FmGraphColorPartitionPairs(FmConstraintSolverData* constraintSolverData)
    {
        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE_CREATE_GRAPH);

        uint numPartitionPairs = constraintSolverData->numPartitionPairs;
        for (uint pairIdx = 0; pairIdx < numPartitionPairs; pairIdx++)
        {
            constraintSolverData->partitionPairs[pairIdx].graphColor = FM_INVALID_ID;
        }

        uint numPartitionPairSets = 0;
#if !FM_GREEDY_SEQUENTIAL_COLORING
        uint numIterations = 0;
        uint totalMinSetElements = 0;
        uint totalMaxSetElements = 0;
#endif

        // First set will be partitions
        uint numPartitionsWithConstraints = constraintSolverData->numPartitionsWithConstraints;
        constraintSolverData->partitionPairIndependentSets[0].pStart = constraintSolverData->partitionPairMinSetElements;
        constraintSolverData->partitionPairIndependentSets[0].numElements = numPartitionsWithConstraints;
#if !FM_GREEDY_SEQUENTIAL_COLORING
        totalMinSetElements += numPartitionsWithConstraints;
#endif

        for (uint partitionPairIdx = 0; partitionPairIdx < numPartitionsWithConstraints; partitionPairIdx++)
        {
            FmPartitionPair& partitionPair = constraintSolverData->partitionPairs[partitionPairIdx];
            partitionPair.graphColor = 0;

            constraintSolverData->partitionPairIndependentSets[0].pStart[partitionPairIdx] = partitionPairIdx;

#if FM_GREEDY_SEQUENTIAL_COLORING
            for (uint pairMeshIdx = 0; pairMeshIdx < partitionPair.numTetMeshes; pairMeshIdx++)
            {
                FmGetObjectPartitionDataRef(constraintSolverData, partitionPair.tetMeshIds[pairMeshIdx]).maxAdjacentColor = 0;
            }

            for (uint pairRbIdx = 0; pairRbIdx < partitionPair.numRigidBodies; pairRbIdx++)
            {
                FmGetObjectPartitionDataRef(constraintSolverData, partitionPair.rigidBodyIds[pairRbIdx]).maxAdjacentColor = 0;
            }
#endif
        }

        numPartitionPairSets = 1;

#if FM_GREEDY_SEQUENTIAL_COLORING
        // Find other colors / independent sets.
        for (uint partitionPairIdx = numPartitionsWithConstraints; partitionPairIdx < numPartitionPairs; partitionPairIdx++)
        {
            FmPartitionPair& partitionPair = constraintSolverData->partitionPairs[partitionPairIdx];

            int maxAdjacentColor = -1;
            for (uint pairMeshIdx = 0; pairMeshIdx < partitionPair.numTetMeshes; pairMeshIdx++)
            {
                FmObjectPartitionData& objectPartitionData = FmGetObjectPartitionDataRef(constraintSolverData, partitionPair.tetMeshIds[pairMeshIdx]);                
                maxAdjacentColor = FmMaxInt(maxAdjacentColor, objectPartitionData.maxAdjacentColor);
            }

            for (uint pairRbIdx = 0; pairRbIdx < partitionPair.numRigidBodies; pairRbIdx++)
            {
                FmObjectPartitionData& objectPartitionData = FmGetObjectPartitionDataRef(constraintSolverData, partitionPair.rigidBodyIds[pairRbIdx]);
                maxAdjacentColor = FmMaxInt(maxAdjacentColor, objectPartitionData.maxAdjacentColor);
            }

            int color = maxAdjacentColor + 1;

            if (color > (int)numPartitionPairSets - 1)
            {
                constraintSolverData->partitionPairIndependentSets[color].numElements = 1;
                numPartitionPairSets = color + 1;
            }
            else
            {
                constraintSolverData->partitionPairIndependentSets[color].numElements++;
            }

            partitionPair.graphColor = color;

            for (uint pairMeshIdx = 0; pairMeshIdx < partitionPair.numTetMeshes; pairMeshIdx++)
            {
                FmObjectPartitionData& objectPartitionData = FmGetObjectPartitionDataRef(constraintSolverData, partitionPair.tetMeshIds[pairMeshIdx]);
                objectPartitionData.maxAdjacentColor = color;
            }

            for (uint pairRbIdx = 0; pairRbIdx < partitionPair.numRigidBodies; pairRbIdx++)
            {
                FmObjectPartitionData& objectPartitionData = FmGetObjectPartitionDataRef(constraintSolverData, partitionPair.rigidBodyIds[pairRbIdx]);
                objectPartitionData.maxAdjacentColor = color;
            }
        }

        uint partitionPairElementsIdx = 0;
        for (uint i = 0; i < numPartitionPairSets; i++)
        {
            FM_ASSERT(partitionPairElementsIdx + constraintSolverData->partitionPairIndependentSets[i].numElements <= numPartitionPairs);
            constraintSolverData->partitionPairIndependentSets[i].pStart = &constraintSolverData->partitionPairMinSetElements[partitionPairElementsIdx];
            partitionPairElementsIdx += constraintSolverData->partitionPairIndependentSets[i].numElements;
            constraintSolverData->partitionPairIndependentSets[i].numElements = 0;
        }

        for (uint partitionPairIdx = 0; partitionPairIdx < numPartitionPairs; partitionPairIdx++)
        {
            FmPartitionPair& partitionPair = constraintSolverData->partitionPairs[partitionPairIdx];
            uint color = partitionPair.graphColor;
            FM_ASSERT(color < numPartitionPairSets);

            uint idx = constraintSolverData->partitionPairIndependentSets[color].numElements;

            if (partitionPairIdx < numPartitionsWithConstraints)
            {
                FM_ASSERT(constraintSolverData->partitionPairIndependentSets[color].pStart[idx] == partitionPairIdx);
            }
            constraintSolverData->partitionPairIndependentSets[color].pStart[idx] = partitionPairIdx;
            constraintSolverData->partitionPairIndependentSets[color].numElements++;
        }
#else

        bool unassignedPair;

        do
        {
            uint minHashGraphColor = numPartitionPairSets;
            uint maxHashGraphColor = numPartitionPairSets + 1;
            uint numMinSetElements = 0;
            uint numMaxSetElements = 0;
            numIterations++;

            const FmConstraintIsland& constraintIsland = *constraintSolverData->constraintIsland;
            FmObjectsMinMaxHash(constraintSolverData, constraintIsland);

            unassignedPair = false;

            for (uint pairIdx = numPartitionsWithConstraints; pairIdx < numPartitionPairs; pairIdx++)
            {
                FmPartitionPair& partitionPair = constraintSolverData->partitionPairs[pairIdx];

                if (partitionPair.graphColor != FM_INVALID_ID)
                    continue;

                bool minHash = true;
                bool maxHash = true;

                uint numTetMeshes = partitionPair.numTetMeshes;
                for (uint pairMeshIdx = 0; pairMeshIdx < numTetMeshes && (minHash || maxHash); pairMeshIdx++)
                {
                    FmObjectPartitionData& objectPartitionData = FmGetObjectPartitionDataRef(constraintSolverData, constraintIsland.tetMeshIds[pairMeshIdx]);

                    FM_ASSERT(objectPartitionData.minMaxHashPartitionPairs.minIdx != FM_INVALID_ID);
                    FM_ASSERT(objectPartitionData.minMaxHashPartitionPairs.maxIdx != FM_INVALID_ID);

                    minHash = minHash && (pairIdx == objectPartitionData.minMaxHashPartitionPairs.minIdx);
                    maxHash = maxHash && (pairIdx == objectPartitionData.minMaxHashPartitionPairs.maxIdx);
                }

                uint numRigidBodies = partitionPair.numRigidBodies;
                for (uint pairRbIdx = 0; pairRbIdx < numRigidBodies && (minHash || maxHash); pairRbIdx++)
                {
                    FmObjectPartitionData& objectPartitionData = FmGetObjectPartitionDataRef(constraintSolverData, constraintIsland.rigidBodyIds[pairRbIdx]);

                    FM_ASSERT(objectPartitionData.minMaxHashPartitionPairs.minIdx != FM_INVALID_ID);
                    FM_ASSERT(objectPartitionData.minMaxHashPartitionPairs.maxIdx != FM_INVALID_ID);

                    minHash = minHash && (pairIdx == objectPartitionData.minMaxHashPartitionPairs.minIdx);
                    maxHash = maxHash && (pairIdx == objectPartitionData.minMaxHashPartitionPairs.maxIdx);
                }

                if (minHash)
                {
                    // Assign min hash graph color
                    partitionPair.graphColor = minHashGraphColor;

                    // Append batchIdx to min set buffer
                    constraintSolverData->partitionPairMinSetElements[totalMinSetElements] = pairIdx;
                    totalMinSetElements++;
                    numMinSetElements++;
                }
                else if (maxHash)
                {
                    // Assign max hash graph color
                    partitionPair.graphColor = maxHashGraphColor;

                    // Append batchIdx to max set buffer
                    constraintSolverData->partitionPairMaxSetElements[totalMaxSetElements] = pairIdx;
                    totalMaxSetElements++;
                    numMaxSetElements++;
                }
                else
                {
                    unassignedPair = true;
                }
            }

            // Create set arrays for min and max colors

            if (numMinSetElements)
            {
                constraintSolverData->partitionPairIndependentSets[numPartitionPairSets].pStart = constraintSolverData->partitionPairMinSetElements + totalMinSetElements - numMinSetElements;
                constraintSolverData->partitionPairIndependentSets[numPartitionPairSets].numElements = numMinSetElements;
                numPartitionPairSets++;
            }

            if (numMaxSetElements)
            {
                constraintSolverData->partitionPairIndependentSets[numPartitionPairSets].pStart = constraintSolverData->partitionPairMaxSetElements + totalMaxSetElements - numMaxSetElements;
                constraintSolverData->partitionPairIndependentSets[numPartitionPairSets].numElements = numMaxSetElements;
                numPartitionPairSets++;
            }
        } while (unassignedPair);
#endif

        constraintSolverData->numPartitionPairIndependentSets = numPartitionPairSets;

        // Sort partition pairs within each set by decreasing size
        for (uint i = 0; i < numPartitionPairSets; i++)
        {
            FmGraphColoringSet& set = constraintSolverData->partitionPairIndependentSets[i];

            FmSort<uint, FmComparePartitionPairs>(set.pStart, set.numElements, constraintSolverData);
        }
    }

}