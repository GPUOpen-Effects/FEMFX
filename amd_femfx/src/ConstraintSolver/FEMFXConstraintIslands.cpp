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
// Code to find all constraint islands from scene objects and constraints, using 
// Union-Find connected components method.
//---------------------------------------------------------------------------------------

#include "FEMFXConstraintIslands.h"
#include "FEMFXScene.h"
#include "FEMFXSort.h"

namespace AMD
{

    // Union-Find connected components algorithm to find groups of objects connected by constraints, i.e., constraint islands.
    // This iterates over the constraints in the FEM system to link bodies.  However the constraints internal to the FEM system don't 
    // include RB/RB constraints which are managed by an external rigid body engine.  To create a constraint island that takes these 
    // external constraints into account, nodes are created for "rigid body islands" instead of individual rigid bodies.  
    // The rigid body island ids are assumed to be assigned by the external rigid body engine based on RB/RB constraint connections.
    // This is in order to support a tightly interleaved FEM engine and rigid body engine solve of each constraint island.

    void FmCCMakeSets(FmCCNode* nodes, uint numTetMeshes, uint numRigidBodyIslands)
    {
        uint numNodes = numTetMeshes + numRigidBodyIslands;
        for (uint i = 0; i < numNodes; i++)
        {
            nodes[i].parentIdx = i;
            nodes[i].rank = 0;
            nodes[i].numTetMeshes = (uint)(i < numTetMeshes);
            nodes[i].numRigidBodies = 0;  // Initialize in pass over rigid bodies
            nodes[i].numRigidBodyIslands = (uint)(i >= numTetMeshes);
            nodes[i].numConstraints = 0;
            nodes[i].id = (uint)-1;
        }
    }

    uint FmCCFindRootIndex(FmCCNode* nodes, uint nodeIdx)
    {
        FmCCNode& node = nodes[nodeIdx];

        if (node.parentIdx != nodeIdx)
        {
            node.parentIdx = FmCCFindRootIndex(nodes, node.parentIdx);
        }

        return node.parentIdx;
    }

    void FmCCUnionSets(FmCCNode* nodes, uint nodeIdxA, uint nodeIdxB, uint numPairElements)
    {
        uint rootIdxA = FmCCFindRootIndex(nodes, nodeIdxA);
        uint rootIdxB = FmCCFindRootIndex(nodes, nodeIdxB);

        FmCCNode& rootA = nodes[rootIdxA];
        FmCCNode& rootB = nodes[rootIdxB];

        if (rootIdxA == rootIdxB)
        {
            // Already in same set.
            // Add contacts for this pair
            rootA.numConstraints += numPairElements;
            return;
        }

        if (rootA.rank < rootB.rank)
        {
            rootA.parentIdx = rootIdxB;
            rootB.numTetMeshes += rootA.numTetMeshes;
            rootB.numRigidBodies += rootA.numRigidBodies;
            rootB.numRigidBodyIslands += rootA.numRigidBodyIslands;
            rootB.numConstraints += rootA.numConstraints + numPairElements;
        }
        else if (rootA.rank > rootB.rank)
        {
            rootB.parentIdx = rootIdxA;
            rootA.numTetMeshes += rootB.numTetMeshes;
            rootA.numRigidBodies += rootB.numRigidBodies;
            rootA.numRigidBodyIslands += rootB.numRigidBodyIslands;
            rootA.numConstraints += rootB.numConstraints + numPairElements;
        }
        else
        {
            rootB.parentIdx = rootIdxA;
            rootA.rank = rootA.rank + 1;
            rootA.numTetMeshes += rootB.numTetMeshes;
            rootA.numRigidBodies += rootB.numRigidBodies;
            rootA.numRigidBodyIslands += rootB.numRigidBodyIslands;
            rootA.numConstraints += rootB.numConstraints + numPairElements;
        }
    }

    class FmCompareCCNodes
    {
    public:
        FmCompareCCNodes(void* inUserData) { (void)inUserData; }

        inline bool operator ()(const FmCCNode& nodeA, const FmCCNode& nodeB)
        {
            return nodeA.parentIdx < nodeB.parentIdx;
        }

        inline int Compare(const FmCCNode& nodeA, const FmCCNode& nodeB)
        {
            return FM_QSORT_INCREASING_RETVAL(nodeA.parentIdx, nodeB.parentIdx);
        }
    };

    void FmCCGroup(FmCCNode* nodes, uint numNodes)
    {
        FmSort<FmCCNode, FmCompareCCNodes>(nodes, numNodes, NULL);
    }

    class FmCompareConstraintIslands
    {
    public:
        FmCompareConstraintIslands(void* inUserData) { (void)inUserData; }

        inline bool operator ()(const FmConstraintIsland& constraintIslandA, const FmConstraintIsland& constraintIslandB)
        {
            return constraintIslandA.numJacobianSubmats > constraintIslandB.numJacobianSubmats;
        }

        inline int Compare(const FmConstraintIsland& constraintIslandA, const FmConstraintIsland& constraintIslandB)
        {
            return FM_QSORT_DECREASING_RETVAL(constraintIslandA.numJacobianSubmats, constraintIslandB.numJacobianSubmats);
        }
    };

    static inline uint FmGetNodeIdx(uint objId, uint numTetMeshes, const FmScene& scene)
    {
        return (objId & FM_RB_FLAG) ? numTetMeshes + FmGetRigidBodyPtrById(scene, objId)->rbIslandId : FmGetTetMeshIdxById(scene, objId);
    }

    void FmFindConstraintIslands(FmConstraintsBuffer* constraintsBuffer, FmScene* scene)
    {
        uint numTetMeshes = scene->numAwakeTetMeshes;
        uint numRigidBodies = scene->numAwakeRigidBodies;

        uint numDistanceContacts = FmAtomicRead(&constraintsBuffer->numDistanceContacts.val);
        uint numVolumeContacts = FmAtomicRead(&constraintsBuffer->numVolumeContacts.val);
        uint numDeformationConstraints = FmAtomicRead(&constraintsBuffer->numDeformationConstraints.val);
        uint numGlueConstraints = constraintsBuffer->numGlueConstraints;
        uint numPlaneConstraints = constraintsBuffer->numPlaneConstraints;
        uint numRigidBodyAngleConstraints = constraintsBuffer->numRigidBodyAngleConstraints;

        bool rigidBodiesExternal = scene->params.rigidBodiesExternal;
        bool createZeroConstraintRigidBodyIslands = scene->params.createZeroConstraintRigidBodyIslands;

        uint numRigidBodyIslands = rigidBodiesExternal ? constraintsBuffer->numUserRigidBodyIslands : numRigidBodies;

        FM_ASSERT(constraintsBuffer->freeSleepingIslandIds.numFreeIds == constraintsBuffer->maxConstraintIslands - constraintsBuffer->numSleepingConstraintIslands);

        // Reset flag for being found in constraint known to FEM system.
        // Only rigid bodies with this set will be added to an FEM island's list of rigid bodies
        for (uint rbIdx = 0; rbIdx < numRigidBodies; rbIdx++)
        {
            FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, scene->awakeRigidBodyIds[rbIdx]);
            rigidBody.foundInConstraint = false;
            rigidBody.femIslandId = FM_INVALID_ID;

            if (!rigidBodiesExternal)
            {
                // Set different rbIslandId values so FmFindConstraintIslands will detect islands (see comments on method above)
                rigidBody.rbIslandId = rbIdx;
            }
        }

        uint numNodes = numTetMeshes + numRigidBodyIslands;

        FmCCNode* nodes = constraintsBuffer->islandObjectNodes;
        FmCCMakeSets(nodes, numTetMeshes, numRigidBodyIslands);

        // Count rigid bodies per rigid body island
        for (uint rbIdx = 0; rbIdx < numRigidBodies; rbIdx++)
        {
            FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, scene->awakeRigidBodyIds[rbIdx]);
            uint nodeIdx = numTetMeshes + rigidBody.rbIslandId;
            FM_ASSERT(nodeIdx < numNodes);
            nodes[nodeIdx].numRigidBodies++;
        }

        // Step over all constraints and union contact objects

        for (uint contactIdx = 0; contactIdx < numDistanceContacts; contactIdx++)
        {
            FmDistanceContactPairInfo& contact = constraintsBuffer->distanceContactsPairInfo[contactIdx];

#if FM_CONTACT_REDUCTION
            if (contact.refCount == 0)
            {
                continue;
            }
#endif

            bool isADynamic = FM_NOT_SET(contact.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED);
            bool isBDynamic = FM_NOT_SET(contact.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED);

            FM_ASSERT(isADynamic || isBDynamic);

            if ((contact.objectIdA & FM_RB_FLAG) && isADynamic)
            {
                FmRigidBody* pRigidBody = FmGetRigidBodyPtrById(*scene, contact.objectIdA);
                FM_ASSERT(pRigidBody != NULL);
                pRigidBody->foundInConstraint = true;
            }
            if ((contact.objectIdB & FM_RB_FLAG) && isBDynamic)
            {
                FmRigidBody* pRigidBody = FmGetRigidBodyPtrById(*scene, contact.objectIdB);
                FM_ASSERT(pRigidBody != NULL);
                pRigidBody->foundInConstraint = true;
            }

            uint nodeIdxA = 0;
            uint nodeIdxB = 0;
            if (isADynamic)
            {
                nodeIdxA = FmGetNodeIdx(contact.objectIdA, numTetMeshes, *scene);
            }
            if (isBDynamic)
            {
                nodeIdxB = FmGetNodeIdx(contact.objectIdB, numTetMeshes, *scene);
            }

            if (isADynamic && isBDynamic)
            {
                FmCCUnionSets(nodes, nodeIdxA, nodeIdxB, 1);
            }
            else if (isADynamic)
            {
                uint rootIdxA = FmCCFindRootIndex(nodes, nodeIdxA);
                nodes[rootIdxA].numConstraints++;
            }
            else if (isBDynamic)
            {
                uint rootIdxB = FmCCFindRootIndex(nodes, nodeIdxB);
                nodes[rootIdxB].numConstraints++;
            }
        }
        for (uint contactIdx = 0; contactIdx < numVolumeContacts; contactIdx++)
        {
            FmVolumeContact& contact = constraintsBuffer->volumeContacts[contactIdx];

            bool isADynamic = FM_NOT_SET(contact.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED);
            bool isBDynamic = FM_NOT_SET(contact.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED);

            FM_ASSERT(isADynamic || isBDynamic);

            if ((contact.objectIdA & FM_RB_FLAG) && isADynamic)
            {
                FmRigidBody* pRigidBody = FmGetRigidBodyPtrById(*scene, contact.objectIdA);
                FM_ASSERT(pRigidBody != NULL);
                pRigidBody->foundInConstraint = true;
            }
            if ((contact.objectIdB & FM_RB_FLAG) && isBDynamic)
            {
                FmRigidBody* pRigidBody = FmGetRigidBodyPtrById(*scene, contact.objectIdB);
                FM_ASSERT(pRigidBody != NULL);
                pRigidBody->foundInConstraint = true;
            }

            uint nodeIdxA = 0;
            uint nodeIdxB = 0;
            if (isADynamic)
            {
                nodeIdxA = FmGetNodeIdx(contact.objectIdA, numTetMeshes, *scene);
            }
            if (isBDynamic)
            {
                nodeIdxB = FmGetNodeIdx(contact.objectIdB, numTetMeshes, *scene);
            }

            if (isADynamic && isBDynamic)
            {
                FmCCUnionSets(nodes, nodeIdxA, nodeIdxB, 1);
            }
            else if (isADynamic)
            {
                uint rootIdxA = FmCCFindRootIndex(nodes, nodeIdxA);
                nodes[rootIdxA].numConstraints++;
            }
            else if (isBDynamic)
            {
                uint rootIdxB = FmCCFindRootIndex(nodes, nodeIdxB);
                nodes[rootIdxB].numConstraints++;
            }
        }
        // For deformation constraints, nothing to union, but need to update constraint counts.
        for (uint constraintIdx = 0; constraintIdx < numDeformationConstraints; constraintIdx++)
        {
            FmDeformationConstraint& deformationConstraint = constraintsBuffer->deformationConstraints[constraintIdx];

            uint nodeIdx = FmGetNodeIdx(deformationConstraint.objectId, numTetMeshes, *scene);

            uint rootIdx = FmCCFindRootIndex(nodes, nodeIdx);
            nodes[rootIdx].numConstraints++;
        }
        for (uint constraintIdx = 0; constraintIdx < numGlueConstraints; constraintIdx++)
        {
            FmGlueConstraint& glueConstraint = constraintsBuffer->glueConstraints[constraintIdx];

            if (FM_ANY_SET(glueConstraint.flags, (FM_CONSTRAINT_FLAG_DISABLED | FM_CONSTRAINT_FLAG_DELETED)))
            {
                continue;
            }

            bool isADynamic = FM_NOT_SET(glueConstraint.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED);
            bool isBDynamic = FM_NOT_SET(glueConstraint.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED);

            if (!isADynamic && !isBDynamic)
            {
                continue;
            }

            if ((glueConstraint.objectIdA & FM_RB_FLAG) && isADynamic)
            {
                FmRigidBody* pRigidBody = FmGetRigidBodyPtrById(*scene, glueConstraint.objectIdA);
                FM_ASSERT(pRigidBody != NULL);
                pRigidBody->foundInConstraint = true;
            }
            if ((glueConstraint.objectIdB & FM_RB_FLAG) && isBDynamic)
            {
                FmRigidBody* pRigidBody = FmGetRigidBodyPtrById(*scene, glueConstraint.objectIdB);
                FM_ASSERT(pRigidBody != NULL);
                pRigidBody->foundInConstraint = true;
            }

            uint nodeIdxA = 0;
            uint nodeIdxB = 0;
            if (isADynamic)
            {
                nodeIdxA = FmGetNodeIdx(glueConstraint.objectIdA, numTetMeshes, *scene);
            }
            if (isBDynamic)
            {
                nodeIdxB = FmGetNodeIdx(glueConstraint.objectIdB, numTetMeshes, *scene);
            }

            FM_ASSERT(nodeIdxA != FM_INVALID_ID && nodeIdxB != FM_INVALID_ID);

            if (isADynamic && isBDynamic)
            {
                FmCCUnionSets(nodes, nodeIdxA, nodeIdxB, 1);
            }
            else if (isADynamic)
            {
                uint rootIdxA = FmCCFindRootIndex(nodes, nodeIdxA);
                nodes[rootIdxA].numConstraints++;
            }
            else if (isBDynamic)
            {
                uint rootIdxB = FmCCFindRootIndex(nodes, nodeIdxB);
                nodes[rootIdxB].numConstraints++;
            }
        }
        for (uint constraintIdx = 0; constraintIdx < numPlaneConstraints; constraintIdx++)
        {
            FmPlaneConstraint& planeConstraint = constraintsBuffer->planeConstraints[constraintIdx];

            if (FM_ANY_SET(planeConstraint.flags, (FM_CONSTRAINT_FLAG_DISABLED | FM_CONSTRAINT_FLAG_DELETED)))
            {
                continue;
            }

            bool isADynamic = FM_NOT_SET(planeConstraint.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED);
            bool isBDynamic = FM_NOT_SET(planeConstraint.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED);

            if (!isADynamic && !isBDynamic)
            {
                continue;
            }

            if ((planeConstraint.objectIdA & FM_RB_FLAG) && isADynamic)
            {
                FmRigidBody* pRigidBody = FmGetRigidBodyPtrById(*scene, planeConstraint.objectIdA);
                FM_ASSERT(pRigidBody != NULL);
                pRigidBody->foundInConstraint = true;
            }
            if ((planeConstraint.objectIdB & FM_RB_FLAG) && isBDynamic)
            {
                FmRigidBody* pRigidBody = FmGetRigidBodyPtrById(*scene, planeConstraint.objectIdB);
                FM_ASSERT(pRigidBody != NULL);
                pRigidBody->foundInConstraint = true;
            }

            uint nodeIdxA = 0;
            uint nodeIdxB = 0;
            if (isADynamic)
            {
                nodeIdxA = FmGetNodeIdx(planeConstraint.objectIdA, numTetMeshes, *scene);
            }
            if (isBDynamic)
            {
                nodeIdxB = FmGetNodeIdx(planeConstraint.objectIdB, numTetMeshes, *scene);
            }

            FM_ASSERT(nodeIdxA != FM_INVALID_ID && nodeIdxB != FM_INVALID_ID);

            if (isADynamic && isBDynamic)
            {
                FmCCUnionSets(nodes, nodeIdxA, nodeIdxB, 1);
            }
            else if (isADynamic)
            {
                uint rootIdxA = FmCCFindRootIndex(nodes, nodeIdxA);
                nodes[rootIdxA].numConstraints++;
            }
            else if (isBDynamic)
            {
                uint rootIdxB = FmCCFindRootIndex(nodes, nodeIdxB);
                nodes[rootIdxB].numConstraints++;
            }
        }
        for (uint constraintIdx = 0; constraintIdx < numRigidBodyAngleConstraints; constraintIdx++)
        {
            FmRigidBodyAngleConstraint& rigidBodyAngleConstraint = constraintsBuffer->rigidBodyAngleConstraints[constraintIdx];

            if (FM_ANY_SET(rigidBodyAngleConstraint.flags, (FM_CONSTRAINT_FLAG_DISABLED | FM_CONSTRAINT_FLAG_DELETED)))
            {
                continue;
            }

            bool isADynamic = FM_NOT_SET(rigidBodyAngleConstraint.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED);
            bool isBDynamic = FM_NOT_SET(rigidBodyAngleConstraint.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED);

            if (!isADynamic && !isBDynamic)
            {
                continue;
            }

            if (isADynamic)
            {
                FmRigidBody* pRigidBody = FmGetRigidBodyPtrById(*scene, rigidBodyAngleConstraint.objectIdA);
                FM_ASSERT(pRigidBody != NULL);
                pRigidBody->foundInConstraint = true;
            }
            if (isBDynamic)
            {
                FmRigidBody* pRigidBody = FmGetRigidBodyPtrById(*scene, rigidBodyAngleConstraint.objectIdB);
                FM_ASSERT(pRigidBody != NULL);
                pRigidBody->foundInConstraint = true;
            }

            uint nodeIdxA = 0;
            uint nodeIdxB = 0;
            if (isADynamic)
            {
                nodeIdxA = FmGetNodeIdx(rigidBodyAngleConstraint.objectIdA, numTetMeshes, *scene);
            }
            if (isBDynamic)
            {
                nodeIdxB = FmGetNodeIdx(rigidBodyAngleConstraint.objectIdB, numTetMeshes, *scene);
            }

            FM_ASSERT(nodeIdxA != FM_INVALID_ID && nodeIdxB != FM_INVALID_ID);

            if (isADynamic && isBDynamic)
            {
                FmCCUnionSets(nodes, nodeIdxA, nodeIdxB, 1);
            }
            else if (isADynamic)
            {
                uint rootIdxA = FmCCFindRootIndex(nodes, nodeIdxA);
                nodes[rootIdxA].numConstraints++;
            }
            else if (isBDynamic)
            {
                uint rootIdxB = FmCCFindRootIndex(nodes, nodeIdxB);
                nodes[rootIdxB].numConstraints++;
            }
        }

        // Create constraint islands for sets with contacts.
        // Add mesh pointers to constraint islands.

        uint numConstraintIslands = 0;
        uint islandsTetMeshIdx = 0;
        uint islandsRigidBodyIdx = 0;
        uint islandsConstraintIdx = 0;
        uint islandsUserRigidBodyIslandIdx = 0;

        for (uint nodeIdx = 0; nodeIdx < numNodes; nodeIdx++)
        {
            FmCCNode& node = nodes[nodeIdx];

            uint rootIdx = FmCCFindRootIndex(nodes, node.parentIdx);
            FmCCNode& rootNode = nodes[rootIdx];

            // If id is invalid, create an island and assign a valid id.
            // Create an island for a single object with no constraints; no further solving, but will be tested for sleeping
            if (rootNode.id == ((uint)-1) && (rootNode.numConstraints > 0 || rootNode.numTetMeshes > 0 || (rootNode.numRigidBodies > 0 && createZeroConstraintRigidBodyIslands)))
            {
                // Record island index in root and at node
                uint constraintIslandIdx = numConstraintIslands;
                node.id = constraintIslandIdx;
                rootNode.id = constraintIslandIdx;

                // Init constraint island arrays
                FmConstraintIsland& island = constraintsBuffer->constraintIslands[constraintIslandIdx];
                FmInitConstraintIsland(&island);
                island.islandId = constraintIslandIdx;
                island.tetMeshIds = constraintsBuffer->allIslandTetMeshIds + islandsTetMeshIdx;
                island.rigidBodyIds = constraintsBuffer->allIslandRigidBodyIds + islandsRigidBodyIdx;
                island.userRigidBodyIslandIndices = constraintsBuffer->allUserRigidBodyIslandIndices + islandsUserRigidBodyIslandIdx;
                island.constraintRefs = constraintsBuffer->allIslandConstraints + islandsConstraintIdx;

                islandsTetMeshIdx += rootNode.numTetMeshes;
                islandsRigidBodyIdx += rootNode.numRigidBodies;
                islandsConstraintIdx += rootNode.numConstraints;
                islandsUserRigidBodyIslandIdx += rootNode.numRigidBodyIslands;
                numConstraintIslands++;
            }

            // Add mesh or rigid body island id to connected component
            uint islandId = rootNode.id;
            if (islandId != ((uint)-1))
            {
                FmConstraintIsland& island = constraintsBuffer->constraintIslands[islandId];

                if (nodeIdx < numTetMeshes)
                {
                    FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, scene->awakeTetMeshIds[nodeIdx]);
                    island.tetMeshIds[island.numTetMeshes] = tetMesh.objectId;
                    island.numTetMeshes++;
                    uint numVerts = tetMesh.numVerts;
                    island.numTetMeshVerts += numVerts;
                    island.numStateVecs3 += numVerts;

                    // Record island assignment in tet mesh
                    tetMesh.islandId = islandId;
                }
                else
                {
                    island.userRigidBodyIslandIndices[island.numUserRigidBodyIslands] = nodeIdx - numTetMeshes;
                    island.numUserRigidBodyIslands++;
                }
            }
            else
            {
                if (nodeIdx < numTetMeshes)
                {
                    // Tet mesh does not have island (has no constraints)
                    FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, scene->awakeTetMeshIds[nodeIdx]);
                    tetMesh.islandId = FM_INVALID_ID;
                }
            }
        }

        // Fill rigid body arrays
        for (uint rbIdx = 0; rbIdx < numRigidBodies; rbIdx++)
        {
            FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, scene->awakeRigidBodyIds[rbIdx]);
            uint nodeIdx = numTetMeshes + rigidBody.rbIslandId;
            FM_ASSERT(nodeIdx < numNodes);
            FmCCNode& node = nodes[nodeIdx];
            uint rootIdx = FmCCFindRootIndex(nodes, node.parentIdx);
            FmCCNode& rootNode = nodes[rootIdx];

            uint islandId = rootNode.id;

            if (islandId != ((uint)-1))  // -1 means rigid body island doesn't connect to any FEM object
            {
                FmConstraintIsland& island = constraintsBuffer->constraintIslands[islandId];

                uint islandNumRigidBodies = rootNode.numRigidBodies;

                // Group rigid bodies with known constraints at the start of the list.
                // Other rigid bodies can be written from the end of the list backwards.

                if (rigidBody.foundInConstraint)
                {
                    island.rigidBodyIds[island.numRigidBodiesInFEMSolve] = rigidBody.objectId;
                    island.numRigidBodiesInFEMSolve++;
                    island.numStateVecs3 += 2;
                }
                else
                {
                    uint numNotInFEMSolve = island.numRigidBodiesConnected - island.numRigidBodiesInFEMSolve;
                    island.rigidBodyIds[islandNumRigidBodies - 1 - numNotInFEMSolve] = rigidBody.objectId;
                }
                island.numRigidBodiesConnected++;
                FM_ASSERT(island.numRigidBodiesConnected <= islandNumRigidBodies);

                // Record island assignment in rigid body
                rigidBody.femIslandId = islandId;
            }
        }

        // Add constraint references to the created constraint islands.

        for (uint contactIdx = 0; contactIdx < numDistanceContacts; contactIdx++)
        {
            FmDistanceContactPairInfo& contact = constraintsBuffer->distanceContactsPairInfo[contactIdx];

#if FM_CONTACT_REDUCTION
            if (contact.refCount == 0)
            {
                continue;
            }
#endif

            uint nodeIdx = 0;
            if (FM_NOT_SET(contact.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED))
            {
                nodeIdx = FmGetNodeIdx(contact.objectIdA, numTetMeshes, *scene);
            }
            else if (FM_NOT_SET(contact.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED))
            {
                nodeIdx = FmGetNodeIdx(contact.objectIdB, numTetMeshes, *scene);
            }
            else
            {
                continue;
            }
            uint rootIdx = FmCCFindRootIndex(nodes, nodeIdx);
            uint constraintIslandIdx = nodes[rootIdx].id;
            FM_ASSERT(constraintIslandIdx != (uint(-1)));
            FmConstraintIsland& constraintIsland = constraintsBuffer->constraintIslands[constraintIslandIdx];

            uint numSubmats = FmGetNumJacobianSubmats(contact);

            FmConstraintReference& constraintRef = constraintIsland.constraintRefs[constraintIsland.numConstraints];
            constraintRef.idx = contactIdx;
            constraintRef.type = FM_CONSTRAINT_TYPE_DISTANCE_CONTACT;
            constraintRef.objectIdA = contact.objectIdA;
            constraintRef.objectIdB = contact.objectIdB;
            constraintRef.idInObjectPair = contact.idInObjectPair;
            constraintRef.partitionIdA = FM_INVALID_ID;
            constraintRef.partitionIdB = FM_INVALID_ID;
            constraintIsland.numConstraints++;
            constraintIsland.numJacobianSubmats += numSubmats;
            FM_ASSERT(constraintIsland.numConstraints <= nodes[rootIdx].numConstraints);

            uint16_t flags = contact.flags;
            bool isAStatic = FM_ALL_SET(flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED | FM_CONSTRAINT_FLAG_OBJECTA_ZEROVEL);
            bool isBStatic = FM_ALL_SET(flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED | FM_CONSTRAINT_FLAG_OBJECTB_ZEROVEL);
            if (isAStatic || isBStatic)
            {
                constraintIsland.numFixedAttachments++;
            }
        }
        for (uint contactIdx = 0; contactIdx < numVolumeContacts; contactIdx++)
        {
            FmVolumeContact& contact = constraintsBuffer->volumeContacts[contactIdx];
            uint nodeIdx = 0;
            if (FM_NOT_SET(contact.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED))
            {
                nodeIdx = FmGetNodeIdx(contact.objectIdA, numTetMeshes, *scene);
            }
            else if (FM_NOT_SET(contact.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED))
            {
                nodeIdx = FmGetNodeIdx(contact.objectIdB, numTetMeshes, *scene);
            }
            else
            {
                continue;
            }
            uint rootIdx = FmCCFindRootIndex(nodes, nodeIdx);
            uint constraintIslandIdx = nodes[rootIdx].id;
            FM_ASSERT(constraintIslandIdx != (uint(-1)));
            FmConstraintIsland& constraintIsland = constraintsBuffer->constraintIslands[constraintIslandIdx];

            uint numSubmats = FmGetNumJacobianSubmats(contact);

            FmConstraintReference& constraintRef = constraintIsland.constraintRefs[constraintIsland.numConstraints];
            constraintRef.idx = contactIdx;
            constraintRef.type = FM_CONSTRAINT_TYPE_VOLUME_CONTACT;
            constraintRef.objectIdA = contact.objectIdA;
            constraintRef.objectIdB = contact.objectIdB;
            constraintRef.idInObjectPair = contact.idInObjectPair;
            constraintRef.partitionIdA = FM_INVALID_ID;
            constraintRef.partitionIdB = FM_INVALID_ID;
            constraintIsland.numConstraints++;
            constraintIsland.numJacobianSubmats += numSubmats;
            FM_ASSERT(constraintIsland.numConstraints <= nodes[rootIdx].numConstraints);

            uint16_t flags = contact.flags;
            bool isAStatic = FM_ALL_SET(flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED | FM_CONSTRAINT_FLAG_OBJECTA_ZEROVEL);
            bool isBStatic = FM_ALL_SET(flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED | FM_CONSTRAINT_FLAG_OBJECTB_ZEROVEL);
            if (isAStatic || isBStatic)
            {
                constraintIsland.numFixedAttachments++;
            }
        }
        for (uint constraintIdx = 0; constraintIdx < numDeformationConstraints; constraintIdx++)
        {
            FmDeformationConstraint& deformationConstraint = constraintsBuffer->deformationConstraints[constraintIdx];
            uint nodeIdx = FmGetNodeIdx(deformationConstraint.objectId, numTetMeshes, *scene);
            uint rootIdx = FmCCFindRootIndex(nodes, nodeIdx);
            uint constraintIslandIdx = nodes[rootIdx].id;
            FM_ASSERT(constraintIslandIdx != (uint(-1)));
            FmConstraintIsland& constraintIsland = constraintsBuffer->constraintIslands[constraintIslandIdx];

            uint numSubmats = FmGetNumJacobianSubmats(deformationConstraint);

            FmConstraintReference& constraintRef = constraintIsland.constraintRefs[constraintIsland.numConstraints];
            constraintRef.idx = constraintIdx;
            constraintRef.type = FM_CONSTRAINT_TYPE_DEFORMATION;
            constraintRef.objectIdA = deformationConstraint.objectId;
            constraintRef.objectIdB = deformationConstraint.objectId;
            constraintRef.idInObjectPair = deformationConstraint.idInObject;
            constraintRef.partitionIdA = FM_INVALID_ID;
            constraintRef.partitionIdB = FM_INVALID_ID;
            constraintIsland.numConstraints++;
            constraintIsland.numJacobianSubmats += numSubmats;
            FM_ASSERT(constraintIsland.numConstraints <= nodes[rootIdx].numConstraints);
        }
        for (uint constraintIdx = 0; constraintIdx < numGlueConstraints; constraintIdx++)
        {
            FmGlueConstraint& glueConstraint = constraintsBuffer->glueConstraints[constraintIdx];

            if (FM_ANY_SET(glueConstraint.flags, (FM_CONSTRAINT_FLAG_DISABLED | FM_CONSTRAINT_FLAG_DELETED)))
            {
                continue;
            }

            uint nodeIdx = 0;
            if (FM_NOT_SET(glueConstraint.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED))
            {
                nodeIdx = FmGetNodeIdx(glueConstraint.objectIdA, numTetMeshes, *scene);
            }
            else if (FM_NOT_SET(glueConstraint.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED))
            {
                nodeIdx = FmGetNodeIdx(glueConstraint.objectIdB, numTetMeshes, *scene);
            }
            else
            {
                continue;
            }
            uint rootIdx = FmCCFindRootIndex(nodes, nodeIdx);
            uint constraintIslandIdx = nodes[rootIdx].id;
            FM_ASSERT(constraintIslandIdx != (uint(-1)));
            FmConstraintIsland& constraintIsland = constraintsBuffer->constraintIslands[constraintIslandIdx];

            uint numSubmats = FmGetNumJacobianSubmats(glueConstraint);

            FmConstraintReference& constraintRef = constraintIsland.constraintRefs[constraintIsland.numConstraints];
            constraintRef.idx = constraintIdx;
            constraintRef.type = FM_CONSTRAINT_TYPE_GLUE;
            constraintRef.objectIdA = glueConstraint.objectIdA;
            constraintRef.objectIdB = glueConstraint.objectIdB;
            constraintRef.idInObjectPair = constraintIdx;
            constraintRef.partitionIdA = FM_INVALID_ID;
            constraintRef.partitionIdB = FM_INVALID_ID;
            constraintIsland.numConstraints++;
            constraintIsland.numJacobianSubmats += numSubmats;
            FM_ASSERT(constraintIsland.numConstraints <= nodes[rootIdx].numConstraints);

            uint16_t flags = glueConstraint.flags;
            bool isAStatic = FM_ALL_SET(flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED | FM_CONSTRAINT_FLAG_OBJECTA_ZEROVEL);
            bool isBStatic = FM_ALL_SET(flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED | FM_CONSTRAINT_FLAG_OBJECTB_ZEROVEL);
            if (isAStatic || isBStatic)
            {
                constraintIsland.numFixedAttachments++;
            }
        }
        for (uint constraintIdx = 0; constraintIdx < numPlaneConstraints; constraintIdx++)
        {
            FmPlaneConstraint& planeConstraint = constraintsBuffer->planeConstraints[constraintIdx];

            if (FM_ANY_SET(planeConstraint.flags, (FM_CONSTRAINT_FLAG_DISABLED | FM_CONSTRAINT_FLAG_DELETED)))
            {
                continue;
            }

            uint nodeIdx = 0;
            if (FM_NOT_SET(planeConstraint.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED))
            {
                nodeIdx = FmGetNodeIdx(planeConstraint.objectIdA, numTetMeshes, *scene);
            }
            else if (FM_NOT_SET(planeConstraint.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED))
            {
                nodeIdx = FmGetNodeIdx(planeConstraint.objectIdB, numTetMeshes, *scene);
            }
            else
            {
                continue;
            }
            uint rootIdx = FmCCFindRootIndex(nodes, nodeIdx);
            uint constraintIslandIdx = nodes[rootIdx].id;
            FM_ASSERT(constraintIslandIdx != (uint(-1)));
            FmConstraintIsland& constraintIsland = constraintsBuffer->constraintIslands[constraintIslandIdx];

            uint numSubmats = FmGetNumJacobianSubmats(planeConstraint);

            FmConstraintReference& constraintRef = constraintIsland.constraintRefs[constraintIsland.numConstraints];
            constraintRef.idx = constraintIdx;
            constraintRef.type = FM_CONSTRAINT_TYPE_PLANE;
            constraintRef.objectIdA = planeConstraint.objectIdA;
            constraintRef.objectIdB = planeConstraint.objectIdB;
            constraintRef.idInObjectPair = constraintIdx;
            constraintRef.partitionIdA = FM_INVALID_ID;
            constraintRef.partitionIdB = FM_INVALID_ID;
            constraintIsland.numConstraints++;
            constraintIsland.numJacobianSubmats += numSubmats;
            FM_ASSERT(constraintIsland.numConstraints <= nodes[rootIdx].numConstraints);

            uint16_t flags = planeConstraint.flags;
            bool isAStatic = FM_ALL_SET(flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED | FM_CONSTRAINT_FLAG_OBJECTA_ZEROVEL);
            bool isBStatic = FM_ALL_SET(flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED | FM_CONSTRAINT_FLAG_OBJECTB_ZEROVEL);
            if (isAStatic || isBStatic)
            {
                constraintIsland.numFixedAttachments++;
            }
        }
        for (uint constraintIdx = 0; constraintIdx < numRigidBodyAngleConstraints; constraintIdx++)
        {
            FmRigidBodyAngleConstraint& rigidBodyAngleConstraint = constraintsBuffer->rigidBodyAngleConstraints[constraintIdx];

            if (FM_ANY_SET(rigidBodyAngleConstraint.flags, (FM_CONSTRAINT_FLAG_DISABLED | FM_CONSTRAINT_FLAG_DELETED)))
            {
                continue;
            }

            uint nodeIdx = 0;
            if (FM_NOT_SET(rigidBodyAngleConstraint.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED))
            {
                nodeIdx = FmGetNodeIdx(rigidBodyAngleConstraint.objectIdA, numTetMeshes, *scene);
            }
            else if (FM_NOT_SET(rigidBodyAngleConstraint.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED))
            {
                nodeIdx = FmGetNodeIdx(rigidBodyAngleConstraint.objectIdB, numTetMeshes, *scene);
            }
            else
            {
                continue;
            }
            uint rootIdx = FmCCFindRootIndex(nodes, nodeIdx);
            uint constraintIslandIdx = nodes[rootIdx].id;
            FM_ASSERT(constraintIslandIdx != (uint(-1)));
            FmConstraintIsland& constraintIsland = constraintsBuffer->constraintIslands[constraintIslandIdx];

            uint numSubmats = FmGetNumJacobianSubmats(rigidBodyAngleConstraint);

            FmConstraintReference& constraintRef = constraintIsland.constraintRefs[constraintIsland.numConstraints];
            constraintRef.idx = constraintIdx;
            constraintRef.type = FM_CONSTRAINT_TYPE_RIGID_BODY_ANGLE;
            constraintRef.objectIdA = rigidBodyAngleConstraint.objectIdA;
            constraintRef.objectIdB = rigidBodyAngleConstraint.objectIdB;
            constraintRef.idInObjectPair = constraintIdx;
            constraintRef.partitionIdA = FM_INVALID_ID;
            constraintRef.partitionIdB = FM_INVALID_ID;
            constraintIsland.numConstraints++;
            constraintIsland.numJacobianSubmats += numSubmats;
            FM_ASSERT(constraintIsland.numConstraints <= nodes[rootIdx].numConstraints);

            uint16_t flags = rigidBodyAngleConstraint.flags;
            bool isAStatic = FM_ALL_SET(flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED | FM_CONSTRAINT_FLAG_OBJECTA_ZEROVEL);
            bool isBStatic = FM_ALL_SET(flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED | FM_CONSTRAINT_FLAG_OBJECTB_ZEROVEL);
            if (isAStatic || isBStatic)
            {
                constraintIsland.numFixedAttachments++;
            }
        }

        constraintsBuffer->numConstraintIslands = numConstraintIslands;

        // Sort constraint islands by decreasing size
        FmSort<FmConstraintIsland, FmCompareConstraintIslands>(constraintsBuffer->constraintIslands, constraintsBuffer->numConstraintIslands, NULL);

        // Reassign island ids after sorting
        for (uint islandIdx = 0; islandIdx < constraintsBuffer->numConstraintIslands; islandIdx++)
        {
            FmConstraintIsland& island = constraintsBuffer->constraintIslands[islandIdx];
            island.islandId = islandIdx;

            for (uint meshIdx = 0; meshIdx < island.numTetMeshes; meshIdx++)
            {
                FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, island.tetMeshIds[meshIdx]);
                tetMesh.islandId = islandIdx;
            }

            for (uint rbIdx = 0; rbIdx < island.numRigidBodiesConnected; rbIdx++)
            {
                FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, island.rigidBodyIds[rbIdx]);
                rigidBody.femIslandId = islandIdx;
            }
        }
    }

}
