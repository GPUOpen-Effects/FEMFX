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
// Partitions used to divide constraints and objects for multithreading when solving a 
// constraint island
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXCommonInternal.h"
#include "FEMFXAtomicOps.h"
#include "FEMFXBvh.h"
#include "FEMFXHashSet.h"

namespace AMD
{
    struct FmScene;
    struct FmConstraintSolverData;
    struct FmConstraintIsland;
    class FmSetupConstraintSolveTaskData;

    // Solving on partitions of bodies and constraints to improve parallelism.
    // Must solve constraints within partitions, and between partitions.
    // Representing both cases with a partition pair (where both partitions may be the same).
    // Each pair contains a list of constraints and bodies that will be solved together.
    // Independent pairs can be processed in parallel.

    // List of objects solved together
    struct FmPartition
    {
        uint*  objectIds;          // objectIds of tet meshes and rigid bodies
        uint*  objectNumVertices;  // Number of vertices per object, for sizing batches of objects
        uint   numObjects;         // Number of objects in partition
        uint   numTetMeshes;       // Number of tet meshes, which will be first set of objects
        uint   nodeCount;          // Count used for sizing partition
        uint   totalNumVertices;
        FmAabb bounds;             // Bounding box around objects in partition

        FmPartition() : objectIds(NULL), objectNumVertices(NULL), numObjects(0), numTetMeshes(0), nodeCount(0), totalNumVertices(0) {}
    };

    // A set for finding all objects within a pair of partitions
    struct FmPartitionObjectSetElement
    {
        uint key;
    };

    struct FmPartitionObjectSetFuncs
    {
        static bool IsValidKey(const uint& key)
        {
            return (key != FM_INVALID_ID);
        }

        static bool KeysEqual(const uint& keyA, const uint& keyB)
        {
            return (keyA == keyB);
        }

        static void SetInvalidKey(uint& key)
        {
            key = FM_INVALID_ID;
        }

        static uint HashFunc(const uint& key)
        {
            return FmComputeHash(key);
        }

        static void InitValue(FmPartitionObjectSetElement& elem)
        {
            (void)elem;
        }
    };

    typedef FmHashSet<FmPartitionObjectSetElement, FmPartitionObjectSetFuncs> FmPartitionObjectSet;

    // A pair of partitions that have some constraints between their objects.
    // This also represents a single partition when the partition ids are the same.
    struct FmPartitionPair
    {
        uint   partitionIdA;
        uint   partitionIdB;

        uint*  constraintIndices;            // Indices of constraint within constraint island
        uint   numConstraints;               // Total number of constraints in partition pair 

        uint*  tetMeshIds;                   // Ids of tet meshes in constraint island
        uint   numTetMeshes;

        uint*  rigidBodyIds;                 // Ids of rigid bodies in constraint island
        uint   numRigidBodies;

        FmPartitionObjectSet objectSet;   // Set used to make lists of objects within the partition pair

        uint                   graphColor;  // Graph color assigned to partition pair

        FmRandomState          randomState; // For randomizing rows of partition pair solve

        FmSolverIterationNorms norms;
    };

    // A set of partition pairs found by going through the island constraints
    struct FmPartitionPairKey
    {
        uint partitionIdA;
        uint partitionIdB;
    };

    struct FmPartitionPairSetElement
    {
        FmPartitionPairKey key;
        uint numConstraints;
        uint pairsArrayIdx;
    };

    struct FmPartitionPairSetFuncs
    {
        static bool IsValidKey(const FmPartitionPairKey& key)
        {
            return (key.partitionIdA != FM_INVALID_ID);
        }

        static bool KeysEqual(const FmPartitionPairKey& keyA, const FmPartitionPairKey& keyB)
        {
            return (keyA.partitionIdA == keyB.partitionIdA && keyA.partitionIdB == keyB.partitionIdB);
        }

        static void SetInvalidKey(FmPartitionPairKey& key)
        {
            key.partitionIdA = FM_INVALID_ID;
            key.partitionIdB = FM_INVALID_ID;
        }

        static uint HashFunc(const FmPartitionPairKey& key)
        {
            uint x = key.partitionIdA;
            uint y = key.partitionIdB;
            return FmComputeHash((x + y)*(x + y + 1) / 2 + y);
        }

        static void InitValue(FmPartitionPairSetElement& elem)
        {
            elem.numConstraints = 0;
            elem.pairsArrayIdx = FM_INVALID_ID;
        }
    };

    typedef FmHashSet<FmPartitionPairSetElement, FmPartitionPairSetFuncs> FmPartitionPairSet;

    // Per-object partition data stored per constraint island
    struct FmObjectPartitionData
    {
        uint                 partitionId;                // Id of partition that this mesh is part of 
        uint                 numIncidentConstraints;     // Count of constraints used in sizing partitions
        int                  maxAdjacentColor;           // Used in graph coloring partition pairs
        FmGraphColoringStats minMaxHashPartitionPairs;   // Used in graph coloring partition pairs

        FmObjectPartitionData() : partitionId(FM_INVALID_ID), numIncidentConstraints(0), maxAdjacentColor(-1) {}
    };

    void FmGraphColorPartitionPairs(FmConstraintSolverData* constraintSolverData);

    // Build BVH hierarchy on objects of island, to use for partitioning objects/contacts
    void FmBuildPartitionsHierarchy(
        FmScene* scene,
        FmBvh* partitionsHierarchy,
        uint* nodeCounts,
        FmConstraintSolverData* constraintSolverData,
        const FmConstraintIsland& constraintIsland);

    // Create the partitions and the pairs of partitions that have contacts/constraints between them
    void FmCreatePartitions(
        FmScene* scene,
        FmConstraintSolverData* constraintSolverData, FmConstraintIsland* constraintIsland,
        FmSetupConstraintSolveTaskData* taskData);

    void FmCreatePartitionPairs(
        FmScene* scene,
        FmConstraintSolverData* constraintSolverData, FmConstraintIsland* constraintIsland);
}
