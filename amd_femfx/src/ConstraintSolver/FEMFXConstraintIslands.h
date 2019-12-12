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
// Groupings of objects formed by constraints linking dynamic state.  Each island is
// an independent problem with a separate constraint solve.
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXCommonInternal.h"
#include "FEMFXAtomicOps.h"
#include "FEMFXConstraints.h"
#include "FEMFXUserConstraints.h"

namespace AMD
{
    struct FmConstraintsBuffer;

    // Reference to a constraint used in constraint island.
    // Includes constraint type and index in global constraint arrays.
    struct FmConstraintReference
    {
        FmConstraintType type;                // Type of constraint
        uint             idx;                 // Index into array of this type of constraint

        // Data used for sorting, for locality, and deterministic behavior
        uint             partitionIdA;        // Partition ids of objects
        uint             partitionIdB;
        uint             objectIdA;           // Object ids
        uint             objectIdB;
        uint             idInObjectPair;      // Index within object pair for a unique ordering
    };

    // Connected component of meshes linked by constraints, which must be solved together.
    // Contains list of constraints and list of objects.
    struct FmConstraintIsland
    {
        FmAtomicUint isIslandStable;         // Flag set to 0 if any body is found which is moving
        FmAtomicUint numFixedPoints;         // Count of vertices or rigid bodies which are kinematic and not moving

        uint                   islandId;          // id for active island is the index into FmConstraintsBuffer::constraintIslands array

        FmConstraintReference* constraintRefs;    // References to scene constraints

        uint                   numConstraints;    // Total number of constraints in constraint island

        uint*                  tetMeshIds;        // Ids of tet meshes in constraint island
        uint                   numTetMeshes;
        uint                   numTetMeshVerts;   // Total number of tet mesh vertices in constraint island

        uint*                  rigidBodyIds;                // Ids of rigid bodies in constraint island
        uint                   numRigidBodiesInFEMSolve;    // Number of rigid bodies with constraints solved by FEM library, first entries of rigidBodyIds array
        uint                   numRigidBodiesConnected;     // Number of rigid bodies connected to this island by known or external constraints (external connections based on RB islands)

        uint*                  userRigidBodyIslandIndices; // All RigidBody::rbIslandId values that are connected to this FEM constraint island by constraints
        uint                   numUserRigidBodyIslands;

        uint                   numStateVecs3;
        uint                   numJacobianSubmats;         // Total number of 3x3 Jacobian submats needed for all constraints

        uint                   numFixedAttachments;        // Count of constraints in island which have a kinematic and not moving half

        FmRandomState          randomState;                // For randomizing constraints in contact island

        FmCallbackConstraintSolveInnerIteration  innerIterationCallback;
        FmCallbackConstraintSolveIslandCompleted islandCompletedCallback;
        void*                                    userData;

        uint16_t                                 flags;
    };

    // Saves necessary data from a constraint island when it's put to sleep, 
    // in order to wake objects as a group on events like a collision or deletion.
    struct FmSleepingConstraintIsland
    {
        uint      islandId;                  // id in range 0 to FmConstraintsBuffer::maxConstraintIslands - 1; mapped to index in sleepingConstraintIslands by sleepingIslandIdToArrayIdx
        uint*     tetMeshIds;
        uint      numTetMeshes;
        uint*     rigidBodyIds;
        uint      numRigidBodiesConnected;   // all rigid bodies connected to the FEM island by known or external constraints (same as numRigidBodiesConnected for active island)
        uint16_t  flags;
    };

    // Connected components, using Union-Find algorithm
    struct FmCCNode
    {
        uint parentIdx;           // Index of parent node, or self if root
        uint rank;                // Depth of node
        uint numTetMeshes;        // If a root node, number of tet meshes in component
        uint numRigidBodies;      // If a root node, number of rigid bodies in component
        uint numRigidBodyIslands; // If a root node, number of rigid body islands in component
        uint numConstraints;      // Count of constraints associated with node; if a root node, total constraints in component.
        uint id;                  // Component id assigned after finding
    };

    // Initialize empty constraint island
    static FM_FORCE_INLINE void FmInitConstraintIsland(FmConstraintIsland* island)
    {
        FmAtomicWrite(&island->isIslandStable.val, 0);
        FmAtomicWrite(&island->numFixedPoints.val, 0);
        island->islandId = FM_INVALID_ID;
        island->constraintRefs = NULL;
        island->numConstraints = 0;
        island->tetMeshIds = NULL;
        island->numTetMeshes = 0;
        island->numTetMeshVerts = 0;
        island->rigidBodyIds = NULL;
        island->numRigidBodiesInFEMSolve = 0;
        island->numRigidBodiesConnected = 0;
        island->userRigidBodyIslandIndices = NULL;
        island->numUserRigidBodyIslands = 0;
        island->numStateVecs3 = 0;
        island->numJacobianSubmats = 0;
        island->numFixedAttachments = 0;
        island->randomState.Init(0);
        island->innerIterationCallback = NULL;
        island->islandCompletedCallback = NULL;
        island->userData = NULL;
        island->flags = 0;
    }

    void FmFindConstraintIslands(FmConstraintsBuffer* constraintsBuffer, FmScene* scene);
}