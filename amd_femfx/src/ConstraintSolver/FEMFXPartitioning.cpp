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
// Construction of constraint and object partitions for multithreading the solve of a 
// constraint island
//---------------------------------------------------------------------------------------

#include "FEMFXBvhBuild.h"
#include "FEMFXScene.h"
#include "FEMFXConstraintSolver.h"
#include "FEMFXSort.h"

// Partitioning is used to create parallelism within constraint islands.
// Break constraint islands into partitions of objects/constraints.
// With graph coloring constraints in each partition pair may be processed in parallel.
// Reference: Harada, "Two-Level Constraint Solver and Pipelined Local Batching for Rigid Body Simulation on GPUs"

// Count used to size the partitions is currently count of constraints.
// Operations on objects within partition are multithreaded.
#define FM_PARTITION_TARGET_COUNT             512

namespace AMD
{
#define FM_COMPARE_AND_RETURN(x, y) if ((x) != (y)) return ((x) < (y));
#define FM_QSORT_COMPARE_AND_RETURN(x, y) if ((x) != (y)) return FM_QSORT_INCREASING_RETVAL(x, y);

    class FmCompareConstraintRefs
    {
        FmConstraintsBuffer* constraintsBuffer;

    public:
        FmCompareConstraintRefs(void* inConstraintsBuffer)
        {
            constraintsBuffer = (FmConstraintsBuffer*)inConstraintsBuffer;
        }

        inline bool operator ()(const FmConstraintReference& constraintRefA, const FmConstraintReference& constraintRefB) const
        {
            FM_COMPARE_AND_RETURN(constraintRefA.partitionIdA, constraintRefB.partitionIdA);
            FM_COMPARE_AND_RETURN(constraintRefA.partitionIdB, constraintRefB.partitionIdB);
            FM_COMPARE_AND_RETURN(constraintRefA.type, constraintRefB.type);
            FM_COMPARE_AND_RETURN(constraintRefA.objectIdA, constraintRefB.objectIdA);
            FM_COMPARE_AND_RETURN(constraintRefA.objectIdB, constraintRefB.objectIdB);
            FM_COMPARE_AND_RETURN(constraintRefA.idInObjectPair, constraintRefB.idInObjectPair);
            FM_ASSERT(0);

            return false;
        }

        inline int Compare(const FmConstraintReference& constraintRefA, const FmConstraintReference& constraintRefB)
        {
            FM_QSORT_COMPARE_AND_RETURN(constraintRefA.partitionIdA, constraintRefB.partitionIdA);
            FM_QSORT_COMPARE_AND_RETURN(constraintRefA.partitionIdB, constraintRefB.partitionIdB);
            FM_QSORT_COMPARE_AND_RETURN(constraintRefA.type, constraintRefB.type);
            FM_QSORT_COMPARE_AND_RETURN(constraintRefA.objectIdA, constraintRefB.objectIdA);
            FM_QSORT_COMPARE_AND_RETURN(constraintRefA.objectIdB, constraintRefB.objectIdB);
            FM_QSORT_COMPARE_AND_RETURN(constraintRefA.idInObjectPair, constraintRefB.idInObjectPair);
            FM_ASSERT(0);

            return 0;
        }
    };

#if !FM_ASYNC_THREADING
    static void FmSortIslandConstraints(FmScene* scene, FmConstraintIsland* constraintIsland)
    {
        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE_SORT_CONSTRAINTS);

        FmConstraintIsland& island = *constraintIsland;

        FmSort<FmConstraintReference, FmCompareConstraintRefs>(island.constraintRefs, island.numConstraints, scene->constraintsBuffer);
    }
#endif

    class FmCompareIslandTetMeshes
    {
        FmConstraintSolverData* constraintSolverData;

    public:
        FmCompareIslandTetMeshes(void* inConstraintSolverData)
        {
            constraintSolverData = (FmConstraintSolverData*)inConstraintSolverData;
        }

        // Sort by partition id
        inline bool operator()(uint objectIdA, uint objectIdB)
        {
            uint partitionIdA = FmGetObjectPartitionDataRef(constraintSolverData, objectIdA).partitionId;
            uint partitionIdB = FmGetObjectPartitionDataRef(constraintSolverData, objectIdB).partitionId;

            return (partitionIdA < partitionIdB);
        }

        // Sort by partition id
        inline int Compare(uint objectIdA, uint objectIdB)
        {
            uint partitionIdA = FmGetObjectPartitionDataRef(constraintSolverData, objectIdA).partitionId;
            uint partitionIdB = FmGetObjectPartitionDataRef(constraintSolverData, objectIdB).partitionId;

            return FM_QSORT_INCREASING_RETVAL(partitionIdA, partitionIdB);
        }
    };

    static void FmSortIslandTetMeshes(FmConstraintSolverData* constraintSolverData, FmConstraintIsland* island)
    {
        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE_SORT_TET_MESHES);

        FmSort<uint, FmCompareIslandTetMeshes>(island->tetMeshIds, island->numTetMeshes, constraintSolverData);
    }


    void FmFitPartitionsLeafAabbs(
        FmBvh* resultBvh, FmVector3* worldMinPosition, FmVector3* worldMaxPosition,
        FmScene* scene,
        const FmConstraintIsland& constraintIsland)
    {
        uint numTetMeshes = constraintIsland.numTetMeshes;
        uint numRigidBodies = constraintIsland.numRigidBodiesInFEMSolve;

        FmBvh& bvh = *resultBvh;

        bvh.numPrims = numTetMeshes + numRigidBodies;

        FmVector3 minPosition(0.0f), maxPosition(0.0f);

        if (numTetMeshes > 0)
        {
            const FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, constraintIsland.tetMeshIds[0]);
            minPosition = tetMesh.bvh.nodes[0].box.pmin;
            maxPosition = tetMesh.bvh.nodes[0].box.pmax;
        }
        else if (numRigidBodies > 0)
        {
            const FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, constraintIsland.rigidBodyIds[0]);
            minPosition = rigidBody.aabb.pmin;
            maxPosition = rigidBody.aabb.pmax;
        }

        for (uint islandMeshIdx = 0; islandMeshIdx < numTetMeshes; islandMeshIdx++)
        {
            const FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, constraintIsland.tetMeshIds[islandMeshIdx]);

            minPosition = min(minPosition, tetMesh.bvh.nodes[0].box.pmin);
            maxPosition = max(maxPosition, tetMesh.bvh.nodes[0].box.pmax);

            bvh.primBoxes[islandMeshIdx] = tetMesh.bvh.nodes[0].box;
        }

        for (uint islandRbIdx = 0; islandRbIdx < numRigidBodies; islandRbIdx++)
        {
            const FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, constraintIsland.rigidBodyIds[islandRbIdx]);

            minPosition = min(minPosition, rigidBody.aabb.pmin);
            maxPosition = max(maxPosition, rigidBody.aabb.pmax);

            bvh.primBoxes[numTetMeshes + islandRbIdx] = rigidBody.aabb;
        }

        *worldMinPosition = minPosition;
        *worldMaxPosition = maxPosition;
    }

    // Compute counts with recursion
    uint FmComputeNodeCountsRecursive(
        uint* nodeCounts,
        FmScene* scene,
        const FmBvhNode* nodes,
        FmConstraintSolverData* constraintSolverData,
        const FmConstraintIsland& constraintIsland,
        uint idx)
    {
        uint numTetMeshes = constraintIsland.numTetMeshes;

        // Fetch kids
        int lc = nodes[idx].left;
        int rc = nodes[idx].right;

        if (lc == rc)
        {
            uint count = 0;
            if ((uint)lc < numTetMeshes)
            {
                count = FmGetObjectPartitionDataRef(constraintSolverData, constraintIsland.tetMeshIds[lc]).numIncidentConstraints;
                //count += tetMesh.numVerts;
            }
            else
            {
                count = FmGetObjectPartitionDataRef(constraintSolverData, constraintIsland.rigidBodyIds[lc - numTetMeshes]).numIncidentConstraints;
                //count += 2;
            }

            nodeCounts[idx] = count;
            return count;
        }

        // Calculate bounds
        uint numl = FmComputeNodeCountsRecursive(nodeCounts, scene, nodes, constraintSolverData, constraintIsland, lc);
        uint numr = FmComputeNodeCountsRecursive(nodeCounts, scene, nodes, constraintSolverData, constraintIsland, rc);

        uint sum = numl + numr;

        // Write bounds
        nodeCounts[idx] = sum;

        return sum;
    }

    // Compute counts for each node of hierarchy, used for sizing partitions.
    void FmComputeNodeCounts(
        uint* nodeCounts,
        FmScene* scene,
        const FmBvhNode* nodes,
        FmConstraintSolverData* constraintSolverData,
        const FmConstraintIsland& constraintIsland)
    {
        const FmConstraintsBuffer& constraintsBuffer = *scene->constraintsBuffer;

        // Find number of constraints affecting each object.  Assumes numIncidentConstraints already reset to 0

        uint numConstraints = constraintIsland.numConstraints;
        for (uint constraintIdx = 0; constraintIdx < numConstraints; constraintIdx++)
        {
            FmConstraintReference& constraintRef = constraintIsland.constraintRefs[constraintIdx];

            uint objectIdA = FM_INVALID_ID;
            uint objectIdB = FM_INVALID_ID;
            bool isAStatic = false;
            bool isBStatic = false;

            if (constraintRef.type == FM_CONSTRAINT_TYPE_DISTANCE_CONTACT)
            {
                FmDistanceContactPairInfo& contact = constraintsBuffer.distanceContactsPairInfo[constraintRef.idx];

                objectIdA = contact.objectIdA;
                objectIdB = contact.objectIdB;
                isAStatic = FM_IS_SET(contact.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED);
                isBStatic = FM_IS_SET(contact.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED);
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_VOLUME_CONTACT)
            {
                FmVolumeContact& contact = constraintsBuffer.volumeContacts[constraintRef.idx];

                objectIdA = contact.objectIdA;
                objectIdB = contact.objectIdB;
                isAStatic = FM_IS_SET(contact.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED);
                isBStatic = FM_IS_SET(contact.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED);
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_DEFORMATION)
            {
                FmDeformationConstraint& constraint = constraintsBuffer.deformationConstraints[constraintRef.idx];

                objectIdA = constraint.objectId;
                objectIdB = constraint.objectId;
                isAStatic = false;
                isBStatic = true;
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_GLUE)
            {
                FmGlueConstraint& constraint = constraintsBuffer.glueConstraints[constraintRef.idx];

                objectIdA = constraint.objectIdA;
                objectIdB = constraint.objectIdB;
                isAStatic = FM_IS_SET(constraint.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED);
                isBStatic = FM_IS_SET(constraint.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED);
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_PLANE)
            {
                FmPlaneConstraint& constraint = constraintsBuffer.planeConstraints[constraintRef.idx];

                objectIdA = constraint.objectIdA;
                objectIdB = constraint.objectIdB;
                isAStatic = FM_IS_SET(constraint.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED);
                isBStatic = FM_IS_SET(constraint.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED);
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_RIGID_BODY_ANGLE)
            {
                FmRigidBodyAngleConstraint& constraint = constraintsBuffer.rigidBodyAngleConstraints[constraintRef.idx];

                objectIdA = constraint.objectIdA;
                objectIdB = constraint.objectIdB;
                isAStatic = FM_IS_SET(constraint.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED);
                isBStatic = FM_IS_SET(constraint.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED);
            }
            else
            {
                FM_ASSERT(0);
            }

            if (!isAStatic)
            {
                FmGetObjectPartitionDataRef(constraintSolverData, objectIdA).numIncidentConstraints++;
            }

            if (!isBStatic && objectIdA != objectIdB)
            {
                FmGetObjectPartitionDataRef(constraintSolverData, objectIdB).numIncidentConstraints++;
            }
        }

        // Recursively sum up node weights.
        FmComputeNodeCountsRecursive(nodeCounts, scene, nodes, constraintSolverData, constraintIsland, 0);
    }

    // Build BVH hierarchy on objects of island, to use for partitioning objects/contacts
    void FmBuildPartitionsHierarchy(
        FmScene* scene,
        FmBvh* partitionsHierarchy,
        uint* nodeCounts,
        FmConstraintSolverData* constraintSolverData,
        const FmConstraintIsland& constraintIsland)
    {
        FmVector3 minPosition, maxPosition;
        FmFitPartitionsLeafAabbs(partitionsHierarchy, &minPosition, &maxPosition, scene, constraintIsland);

        FmBuildBvhOnLeaves(partitionsHierarchy, minPosition, maxPosition);

        // Convert indices to object ids.
        // NOTE: leaf nodes must keep indices
        uint numTetMeshes = constraintIsland.numTetMeshes;
        uint numObjects = partitionsHierarchy->numPrims;
        FM_ASSERT(numObjects == numTetMeshes + constraintIsland.numRigidBodiesInFEMSolve);

        // Initialize partitions data
        for (uint objectIdx = 0; objectIdx < numObjects; objectIdx++)
        {
            constraintSolverData->objectPartitionData[objectIdx] = FmObjectPartitionData();
        }

        for (uint i = 0; i < numObjects; i++)
        {
            uint objectIdx = partitionsHierarchy->primIndicesSorted[i];
            if (objectIdx < numTetMeshes)
            {
                partitionsHierarchy->primIndicesSorted[i] = constraintIsland.tetMeshIds[objectIdx];
            }
            else
            {
                partitionsHierarchy->primIndicesSorted[i] = constraintIsland.rigidBodyIds[objectIdx - numTetMeshes];
            }
        }

        FmComputeNodeCounts(nodeCounts, scene, partitionsHierarchy->nodes, constraintSolverData, constraintIsland);
    }

    void FmCreatePartitionsRecursive(
        FmConstraintSolverData* constraintSolverData,
        const FmConstraintIsland& constraintIsland,
        uint nodeIdx)
    {
        FmBvh& bvh = constraintSolverData->partitionsHierarchy;
        uint* nodeCounts = constraintSolverData->nodeCounts;
        FmPartition* partitions = constraintSolverData->partitions;
        uint* objectNumVertices = constraintSolverData->allPartitionObjectNumVertices;
        uint& numPartitions = constraintSolverData->numPartitions;
        uint numObjects = constraintIsland.numTetMeshes + constraintIsland.numRigidBodiesInFEMSolve;

        FmBvhNode& node = bvh.nodes[nodeIdx];

        if (node.left == node.right)
        {
            uint leafIdx = nodeIdx - (numObjects - 1);
            partitions[numPartitions].objectIds = (uint*)&bvh.primIndicesSorted[leafIdx];
            partitions[numPartitions].objectNumVertices = &objectNumVertices[leafIdx];
            partitions[numPartitions].numObjects = 1;
            partitions[numPartitions].bounds = bvh.nodes[nodeIdx].box;
            partitions[numPartitions].nodeCount = nodeCounts[nodeIdx];
            partitions[numPartitions].totalNumVertices = 0;
            numPartitions++;
            return;
        }

        const uint maxObjects = FM_PARTITION_TARGET_COUNT * 3 / 2;
        const uint minObjects = FM_PARTITION_TARGET_COUNT / 4;

        if (nodeCounts[nodeIdx] <= FM_PARTITION_TARGET_COUNT ||
            (nodeCounts[nodeIdx] <= maxObjects &&
            (nodeCounts[node.left] < minObjects || nodeCounts[node.right] < minObjects)))
        {
            int2 range = FmFindSpan(bvh.mortonCodesSorted, numObjects, nodeIdx);
            partitions[numPartitions].objectIds = (uint*)&bvh.primIndicesSorted[range.x];
            partitions[numPartitions].objectNumVertices = &objectNumVertices[range.x];
            partitions[numPartitions].numObjects = range.y - range.x + 1;
            partitions[numPartitions].bounds = bvh.nodes[nodeIdx].box;
            partitions[numPartitions].nodeCount = nodeCounts[nodeIdx];
            partitions[numPartitions].totalNumVertices = 0;
            numPartitions++;
            return;
        }

        FmCreatePartitionsRecursive(constraintSolverData, constraintIsland, node.left);
        FmCreatePartitionsRecursive(constraintSolverData, constraintIsland, node.right);
    }

    // Get the pair of partitions linked by a constraint, by checking the referenced objects.
    // Sorts the two partitions so they may be swapped from the constraint objects
    static inline void FmGetPartitionIds(
        uint* resultPartitionIdA, uint* resultPartitionIdB, uint* resultObjectIdA, uint* resultObjectIdB, bool* resultIsAStatic, bool* resultIsBStatic,
        const FmScene& scene,
        FmConstraintSolverData* constraintSolverData,
        const FmConstraintReference& constraintRef)
    {
        uint partitionIdA = FM_INVALID_ID;
        uint partitionIdB = FM_INVALID_ID;
        uint objectIdA = FM_INVALID_ID;
        uint objectIdB = FM_INVALID_ID;
        bool isAStatic = false;
        bool isBStatic = false;

        const FmConstraintsBuffer& constraintsBuffer = *scene.constraintsBuffer;

        // Get the object ids
        if (constraintRef.type == FM_CONSTRAINT_TYPE_DISTANCE_CONTACT)
        {
            FmDistanceContactPairInfo& contact = constraintsBuffer.distanceContactsPairInfo[constraintRef.idx];

            objectIdA = contact.objectIdA;
            objectIdB = contact.objectIdB;
            isAStatic = FM_IS_SET(contact.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED);
            isBStatic = FM_IS_SET(contact.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED);
        }
        else if (constraintRef.type == FM_CONSTRAINT_TYPE_VOLUME_CONTACT)
        {
            FmVolumeContact& contact = constraintsBuffer.volumeContacts[constraintRef.idx];

            objectIdA = contact.objectIdA;
            objectIdB = contact.objectIdB;
            isAStatic = FM_IS_SET(contact.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED);
            isBStatic = FM_IS_SET(contact.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED);
        }
        else if (constraintRef.type == FM_CONSTRAINT_TYPE_DEFORMATION)
        {
            FmDeformationConstraint& constraint = constraintsBuffer.deformationConstraints[constraintRef.idx];

            objectIdA = constraint.objectId;
            objectIdB = constraint.objectId;
            isAStatic = false;
            isBStatic = true;
        }
        else if (constraintRef.type == FM_CONSTRAINT_TYPE_GLUE)
        {
            FmGlueConstraint& constraint = constraintsBuffer.glueConstraints[constraintRef.idx];

            objectIdA = constraint.objectIdA;
            objectIdB = constraint.objectIdB;
            isAStatic = FM_IS_SET(constraint.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED);
            isBStatic = FM_IS_SET(constraint.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED);
        }
        else if (constraintRef.type == FM_CONSTRAINT_TYPE_PLANE)
        {
            FmPlaneConstraint& constraint = constraintsBuffer.planeConstraints[constraintRef.idx];

            objectIdA = constraint.objectIdA;
            objectIdB = constraint.objectIdB;
            isAStatic = FM_IS_SET(constraint.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED);
            isBStatic = FM_IS_SET(constraint.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED);
        }
        else if (constraintRef.type == FM_CONSTRAINT_TYPE_RIGID_BODY_ANGLE)
        {
            FmRigidBodyAngleConstraint& constraint = constraintsBuffer.rigidBodyAngleConstraints[constraintRef.idx];

            objectIdA = constraint.objectIdA;
            objectIdB = constraint.objectIdB;
            isAStatic = FM_IS_SET(constraint.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED);
            isBStatic = FM_IS_SET(constraint.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED);
        }
        else
        {
            FM_ASSERT(0);
        }

        // Get the partition ids from the objects.  
        // If one of the objects is marked as static in the constraint, then there's no
        // real connection with its partition (the object won't be affected by processing the
        // constraint).  In that case, can ignore its partition id and make it the same as the 
        // other object's partition.
        FM_ASSERT(!isAStatic || !isBStatic);

        if (!isAStatic)
        {
            partitionIdA = FmGetObjectPartitionDataRef(constraintSolverData, objectIdA).partitionId;
        }

        if (!isBStatic)
        {
            partitionIdB = FmGetObjectPartitionDataRef(constraintSolverData, objectIdB).partitionId;
        }

        if (isAStatic)
        {
            partitionIdA = partitionIdB;
        }
        else if (isBStatic)
        {
            partitionIdB = partitionIdA;
        }

        // Sorts the two partitions so they may be swapped from the constraint objects
        if (partitionIdA > partitionIdB)
        {
            uint tmp = partitionIdA;
            partitionIdA = partitionIdB;
            partitionIdB = tmp;
        }

        *resultPartitionIdA = partitionIdA;
        *resultPartitionIdB = partitionIdB;
        *resultObjectIdA = objectIdA;
        *resultObjectIdB = objectIdB;
        *resultIsAStatic = isAStatic;
        *resultIsBStatic = isBStatic;
    }

    class FmComparePartitionObjects
    {
        FmScene* scene;

    public:
        FmComparePartitionObjects(void* inScene)
        {
            scene = (FmScene*)inScene;
        }

        inline bool operator()(uint objectIdA, uint objectIdB)
        {
            bool objARigid = FM_IS_SET(objectIdA, FM_RB_FLAG);
            bool objBRigid = FM_IS_SET(objectIdB, FM_RB_FLAG);

            if (objARigid)
            {
                return false;
            }

            if (objBRigid)
            {
                return true;
            }

            FmTetMesh& tetMeshA = *FmGetTetMeshPtrById(*scene, objectIdA);
            FmTetMesh& tetMeshB = *FmGetTetMeshPtrById(*scene, objectIdB);

            return tetMeshA.vertConnectivity.numAdjacentVerts > tetMeshB.vertConnectivity.numAdjacentVerts;
        }

        inline int Compare(uint objectIdA, uint objectIdB)
        {
            bool objARigid = FM_IS_SET(objectIdA, FM_RB_FLAG);
            bool objBRigid = FM_IS_SET(objectIdB, FM_RB_FLAG);

            if (objARigid)
            {
                return 1;
            }

            if (objBRigid)
            {
                return -1;
            }

            FmTetMesh& tetMeshA = *FmGetTetMeshPtrById(*scene, objectIdA);
            FmTetMesh& tetMeshB = *FmGetTetMeshPtrById(*scene, objectIdB);

            return FM_QSORT_DECREASING_RETVAL(tetMeshA.vertConnectivity.numAdjacentVerts, tetMeshB.vertConnectivity.numAdjacentVerts);
        }
    };

#define FM_CREATE_ONE_PARTITION 0  // Debug option to make one partition for the entire island

    // Create the partitions and the pairs of partitions that have contacts/constraints between them
    void FmCreatePartitions(
        FmScene* scene,
        FmConstraintSolverData* constraintSolverData, FmConstraintIsland* constraintIsland,
        FmSetupConstraintSolveTaskData* taskData)
    {
        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE_CREATE_PARTITIONS);

#if FM_CREATE_ONE_PARTITION
        FmBvh& bvh = constraintSolverData->partitionsHierarchy;
        constraintSolverData->numPartitions = 1;
        constraintSolverData->partitions[0].objectIds = (uint*)&bvh.primIndicesSorted[0];
        constraintSolverData->partitions[0].objectNumVertices = constraintSolverData->allPartitionObjectNumVertices;
        constraintSolverData->partitions[0].numObjects = bvh.numPrims;
        constraintSolverData->partitions[0].bounds = bvh.nodes[0].box;
        constraintSolverData->partitions[0].nodeCount = 0;
        constraintSolverData->partitions[0].totalNumVertices = 0;
#else
        constraintSolverData->numPartitions = 0;
        FmCreatePartitionsRecursive(constraintSolverData, *constraintIsland, 0);
#endif

        // Set partition id in the objects
        uint numPartitions = constraintSolverData->numPartitions;
        for (uint partitionId = 0; partitionId < numPartitions; partitionId++)
        {
            FmPartition& partition = constraintSolverData->partitions[partitionId];

            // Sort the objects by decreasing size, for load balancing during GS/MPCG on partition objects
            FmSort<uint, FmComparePartitionObjects>(partition.objectIds, partition.numObjects, scene);

            // Store sizes of objects for batching
            uint numObjects = partition.numObjects;
            for (uint partitionObjIdx = 0; partitionObjIdx < numObjects; partitionObjIdx++)
            {
                uint objectId = partition.objectIds[partitionObjIdx];

                bool isObjectRigid = FM_IS_SET(objectId, FM_RB_FLAG);

                if (isObjectRigid)
                {
                    partition.objectNumVertices[partitionObjIdx] = 2;
                    partition.totalNumVertices += 2;
                }
                else
                {
                    FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, objectId);
                    uint numVerts = tetMesh.numVerts;
                    partition.objectNumVertices[partitionObjIdx] = numVerts;
                    partition.totalNumVertices += numVerts;
                }
            }

            partition.numTetMeshes = 0;

            for (uint partitionMeshIdx = 0; partitionMeshIdx < partition.numObjects; partitionMeshIdx++)
            {
                uint islandObjectId = partition.objectIds[partitionMeshIdx];

                FmGetObjectPartitionDataRef(constraintSolverData, islandObjectId).partitionId = partitionId;

                if (!FM_IS_SET(islandObjectId, FM_RB_FLAG))
                {
                    partition.numTetMeshes++;
                }
            }
        }

        // Sort island objects by partition which will group partition objects' data in solver arrays
        FmSortIslandTetMeshes(constraintSolverData, constraintIsland);

        // Loop through the island constraints, and sum up the number per partition pair, in order to allocate subarrays.
        uint numConstraints = constraintIsland->numConstraints;
        for (uint islandConstraintIdx = 0; islandConstraintIdx < numConstraints; islandConstraintIdx++)
        {
            FmConstraintReference& constraintRef = constraintIsland->constraintRefs[islandConstraintIdx];

            uint partitionIdA, partitionIdB, objectIdA, objectIdB;
            bool isAStatic, isBStatic;
            FmGetPartitionIds(&partitionIdA, &partitionIdB, &objectIdA, &objectIdB, &isAStatic, &isBStatic, *scene, constraintSolverData, constraintRef);

            // Insert/find partition pair in set and add constraint
            FmPartitionPairKey key;
            key.partitionIdA = partitionIdA;
            key.partitionIdB = partitionIdB;

            constraintRef.partitionIdA = partitionIdA;
            constraintRef.partitionIdB = partitionIdB;

            bool foundInSet;
            FmPartitionPairSetElement* element = FmInsertElement(&foundInSet, &constraintSolverData->partitionPairSet, key);
            FM_ASSERT(element);
            element->numConstraints++;
        }

#if FM_ASYNC_THREADING
        taskData->sortTaskGraph = new FmSortTaskGraph<FmConstraintReference, FmCompareConstraintRefs>();
        taskData->sortTaskGraph->SetCallbacks(scene->taskSystemCallbacks.SubmitAsyncTask);
        taskData->sortTaskGraph->CreateAndRunGraph(&constraintIsland->constraintRefs, constraintIsland->constraintRefs, NULL, constraintIsland->numConstraints, NULL, scene->params.numThreads * 4, 128, FmSetupConstraintSolvePostSort, taskData);
#else
        // Sort all island constraints by partition pair, which will group partition-pair constraint data in solver arrays
        FmSortIslandConstraints(scene, constraintIsland);
#endif
    }

    // Create pairs of partitions that have contacts/constraints between them
    void FmCreatePartitionPairs(
        FmScene* scene,
        FmConstraintSolverData* constraintSolverData, FmConstraintIsland* constraintIsland)
    {
        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE_CREATE_PARTITIONS);

        // Create partition pairs from set.

        // Allocate the contact lists per partition pair, and the FmPartitionObjectSet memory, which is bounded by number of contacts
        uint numPartitionPairs = 0;
        uint constraintIdx = 0;
        uint objectIdx = 0;
        uint objectSetIdx = 0;

        // Put partitions at start of array
        uint numPartitions = constraintSolverData->numPartitions;
        uint numConstraints = constraintIsland->numConstraints;
        for (uint partitionIdx = 0; partitionIdx < numPartitions; partitionIdx++)
        {
            FmPartitionPairKey key;
            key.partitionIdA = partitionIdx;
            key.partitionIdB = partitionIdx;

            bool foundInSet;
            FmPartitionPairSetElement* element = FmInsertElement(&foundInSet, &constraintSolverData->partitionPairSet, key);

            if (foundInSet)  // found if partition contains constraints
            {
                FM_ASSERT(element->numConstraints);
                element->pairsArrayIdx = numPartitionPairs;
                FmPartitionPair& partitionPair = constraintSolverData->partitionPairs[numPartitionPairs];

                // allPartitionObjectIds and allPartitionObjectSetElements are sized using numConstraints x 2 as bound on number of objects
                uint maxObjectsInPartition = element->numConstraints * 2;
                uint maxObjectsInSet = maxObjectsInPartition * 2;

                partitionPair.partitionIdA = element->key.partitionIdA;
                partitionPair.partitionIdB = element->key.partitionIdB;
                partitionPair.constraintIndices = &constraintSolverData->allPartitionConstraintIndices[constraintIdx];
                partitionPair.numConstraints = 0;
                partitionPair.tetMeshIds = &constraintSolverData->allPartitionObjectIds[objectIdx];
                partitionPair.numTetMeshes = 0;
                partitionPair.rigidBodyIds = NULL;  // set later to follow tet mesh ids, once number of tet meshes known
                partitionPair.numRigidBodies = 0;
                partitionPair.graphColor = FM_INVALID_ID;
                FmInitHashSet(&partitionPair.objectSet, &constraintSolverData->allPartitionObjectSetElements[objectSetIdx], maxObjectsInSet);

                numPartitionPairs++;
                constraintIdx += element->numConstraints;
                objectIdx += maxObjectsInPartition;
                objectSetIdx += maxObjectsInSet;

                FM_ASSERT(constraintIdx <= numConstraints);
                FM_ASSERT(objectIdx <= numConstraints * 2);
                FM_ASSERT(objectSetIdx <= numConstraints * 2 * 2);
            }
        }

        constraintSolverData->numPartitionsWithConstraints = numPartitionPairs;

        // Init partition pairs, and fill in constraints and object sets
        for (uint islandConstraintIdx = 0; islandConstraintIdx < numConstraints; islandConstraintIdx++)
        {
            FmConstraintReference& constraintRef = constraintIsland->constraintRefs[islandConstraintIdx];

            uint partitionIdA, partitionIdB, objectIdA, objectIdB;
            bool isAStatic, isBStatic;
            FmGetPartitionIds(&partitionIdA, &partitionIdB, &objectIdA, &objectIdB, &isAStatic, &isBStatic, *scene, constraintSolverData, constraintRef);

            // Insert/find partition pair in set and add constraint
            FmPartitionPairKey pairKey;
            pairKey.partitionIdA = partitionIdA;
            pairKey.partitionIdB = partitionIdB;

            bool foundInSet;
            FmPartitionPairSetElement* element = FmInsertElement(&foundInSet, &constraintSolverData->partitionPairSet, pairKey);
            FM_ASSERT(element && foundInSet);

            if (element->pairsArrayIdx == FM_INVALID_ID)
            {
                // Create new partition pair

                element->pairsArrayIdx = numPartitionPairs;
                FmPartitionPair& partitionPair = constraintSolverData->partitionPairs[numPartitionPairs];

                // allPartitionObjectIds and allPartitionObjectSetElements are sized using numConstraints x 2 as bound on number of objects
                uint maxObjectsInPartition = element->numConstraints * 2;
                uint maxObjectsInSet = maxObjectsInPartition * 2;

                partitionPair.partitionIdA = element->key.partitionIdA;
                partitionPair.partitionIdB = element->key.partitionIdB;
                partitionPair.constraintIndices = &constraintSolverData->allPartitionConstraintIndices[constraintIdx];
                partitionPair.numConstraints = 0;
                partitionPair.tetMeshIds = &constraintSolverData->allPartitionObjectIds[objectIdx];
                partitionPair.numTetMeshes = 0;
                partitionPair.rigidBodyIds = NULL;  // set later to follow tet mesh ids, once number of tet meshes known
                partitionPair.numRigidBodies = 0;
                partitionPair.graphColor = FM_INVALID_ID;
                FmInitHashSet(&partitionPair.objectSet, &constraintSolverData->allPartitionObjectSetElements[objectSetIdx], maxObjectsInSet);

                numPartitionPairs++;
                constraintIdx += element->numConstraints;
                objectIdx += maxObjectsInPartition;
                objectSetIdx += maxObjectsInSet;

                FM_ASSERT(constraintIdx <= numConstraints);
                FM_ASSERT(objectIdx <= numConstraints * 2);
                FM_ASSERT(objectSetIdx <= numConstraints * 2 * 2);
            }

            FmPartitionPair& partitionPair = constraintSolverData->partitionPairs[element->pairsArrayIdx];

            partitionPair.constraintIndices[partitionPair.numConstraints] = islandConstraintIdx;
            partitionPair.numConstraints++;

            // Add objects of constraint to the partition pair, using set.
            // Count the number of tet meshes vs. rigid bodies.
            // Only add objects to a partition if affected by at least one constraint.
            if (!isAStatic)
            {
                FmInsertElement(&foundInSet, &partitionPair.objectSet, objectIdA);
                if (!foundInSet)
                {
                    if (objectIdA & FM_RB_FLAG)
                    {
                        partitionPair.numRigidBodies++;
                    }
                    else
                    {
                        partitionPair.numTetMeshes++;
                    }
                }
            }

            if (!isBStatic)
            {
                FmInsertElement(&foundInSet, &partitionPair.objectSet, objectIdB);
                if (!foundInSet)
                {
                    if (objectIdB & FM_RB_FLAG)
                    {
                        partitionPair.numRigidBodies++;
                    }
                    else
                    {
                        partitionPair.numTetMeshes++;
                    }
                }
            }
        }

        constraintSolverData->numPartitionPairs = numPartitionPairs;

        // Loop over partition pairs to fill in the object indices and init random numbers
        for (uint pairIdx = 0; pairIdx < numPartitionPairs; pairIdx++)
        {
            FmPartitionPair& partitionPair = constraintSolverData->partitionPairs[pairIdx];

            partitionPair.rigidBodyIds = partitionPair.tetMeshIds + partitionPair.numTetMeshes;

            partitionPair.randomState.Init(constraintIsland->randomState.RandomUint() + pairIdx);

            partitionPair.norms.Zero();

            uint partitionRbIdx = 0;
            uint partitionMeshIdx = 0;
            uint maxObjectSetElements = partitionPair.objectSet.maxElements;
            for (uint elemIdx = 0; elemIdx < maxObjectSetElements; elemIdx++)
            {
                FmPartitionObjectSetElement& element = partitionPair.objectSet.elements[elemIdx];
                if (FmPartitionObjectSetFuncs::IsValidKey(element.key))
                {
                    uint objectId = element.key;

                    if (objectId & FM_RB_FLAG)
                    {
                        partitionPair.rigidBodyIds[partitionRbIdx] = objectId;
                        partitionRbIdx++;
                    }
                    else
                    {
                        partitionPair.tetMeshIds[partitionMeshIdx] = objectId;
                        partitionMeshIdx++;
                    }
                }
            }
            FM_ASSERT(partitionRbIdx == partitionPair.numRigidBodies);
            FM_ASSERT(partitionMeshIdx == partitionPair.numTetMeshes);
        }
    }
}
