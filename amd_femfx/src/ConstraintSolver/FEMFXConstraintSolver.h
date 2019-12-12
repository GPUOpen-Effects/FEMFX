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
// Definition of solver for constraint impulses and reponse of FEM and rigid bodies
//---------------------------------------------------------------------------------------

#pragma once

#include "AMD_FEMFX.h"
#include "FEMFXSolverMath.h"
#include "FEMFXGraphColoring.h"
#include "FEMFXPartitioning.h"
#include "FEMFXAsyncThreading.h"
#include "FEMFXSort.h"

// Data for constraint solver.
// Solves for contact impulses that when added into the FEM implicit step result in nonpenetration of 
// contact points at the end of the step.
// See: Otaduy et al., "Implicit Contact Handling for Deformable Objects"
//
// Let
// J = Jacobian that projects mesh positions to contact distance in normal direction.
// H = Jacobian that projects mesh positions to contact distance in tangent directions.
//
// There are two tangents per contact used to approximate Coulomb friction model.
// First tangent is aligned with direction of relative velocity at contact.
//
// Nonpenetration impulses have the form:
// J^T * lambda
// for all contacts i, lambda_i >= 0
//
// Friction impulses have the form:
// H^T * gamma
// for all contacts i, |gamma_i1| <= friction_coeff_i * lambda_i
//                     |gamma_i2| <= friction_coeff_i * lambda_i
//
// The additional impulses change the result velocity of the implicit integration step according to:
// (1)
// A * delta_vel = J^T * lambda + H^T * gamma
// where A = matrix for Implicit Euler solve (M + delta_t * C + delta_t^2 * K')
// 
// Nonpenetration constraint requires contact distance not go negative during step:
// (2)
// J * delta_vel + J * vel_unconstrained + (1 / delta_t) * g0  >= 0
// with g0 == initial contact distance
//
// Friction opposes movement in tangent directions
// H * delta_vel + H * vel_unconstrained = 0
//
// Forms a Mixed Linear Complementarity Problem(MLCP).
// 
// In the following J and H are combined into a single J matrix, with
// groups of three rows corresponding to normal and friction constraints.
//
// delta_vel, lambda, gamma solved with nested iterative method:
//
// Inner iterations: lambda(i)
// Starts with a formula for delta_vel(i) taken from block Jacobi iteration of eq (1):
//
// DA = block diagonal of A
// LA = negative block lower triangular part of A
// UA = negative block upper triangular part of A
//
// DA * delta_vel(i) = (LA + UA) * delta_vel(i-1) + J^T * lambda(i) =>
// (3)
// delta_vel(i) = DAinv * (LA + UA) * delta_vel(i-1) + DAinv * J^T * lambda(i)
// 
// Can substitute (3) into the constraint inequality (2):
// J * delta_vel(i) >= -(g0 / delta_t) - J * vel_unconstrained =>
// J * (DAinv * (LA + UA) * delta_vel(i-1) + DAinv * J^T * lambda(i)) >= -(g0 / delta_t) - J * vel_unconstrained =>
// (4)
// J * DAinv * J^T * lambda(i) >= -(g0 / delta_t) - J * vel_unconstrained - J * (LA + UA) * delta_vel(i-1)
//
// Projected Gauss-Seidel is used to solve (4) for lambda(i).
//
// Outer iterations:
// With lambda(i) computed, can now compute delta_vel(i) by block Jacobi.
// However instead the paper applies this lambda(i) in a Gauss-Seidel iteration of eq (1).
// Despite the inconsistency over multiple iterations lambda and delta_vel converge.
//
// To support kinematic vertices, any non-zero kinematic velocities are included in the calculation of
// J * vel_unconstrained above, but for the constraint solve J only includes submatrices corresponding
// to dynamic state.
//
// A method is applied to address the problem that the matrix created for contact solve can be dense
// when many contacts affect the same simulation degree of freedom.   The paper is mainly concerned with deformable
// vs. rigid bodies given the likelihood of dense matrices, but this is an optimization for deformable bodies as well.
//
// This method leaves the full contact solve matrix factorized as J * DAinv * J^T, and
// maintains the partial product (DAinv * J^T * lambda) which can be updated in constant time for 
// each change of lambda value.  This can be multiplied by a row of J to compute a new lambda in constant time.
//
// Volume contacts can be fit into the same system, but create a Jacobian row with a variable number of 
// non-zero submatrices.  Also, unlike the preemptive CCD contacts that slow approaching bodies, using impulses
// to fix volume error retroactively can add energy spikes.
// 
// Results have been more stable using the impulse solve only to avoid increasing interpenetration, and fix 
// intersection volume error with post-stabilization, which modifies positions directly rather than velocity.
// 
// Also due to some popping artifacts have limited the amount of volume correction each step to a small percentage of 
// current volume.  (A factor may be that the volume gradients are a local approximation and don't predict large 
// volume change well.  However numerical error may also be involved and more investigation is needed.)
//
// Solving with rigid-bodies:
// As in "Implicit Contact Handling for Deformable Objects", can interface FEM objects with rigid bodies by adding the 
// rigid-body DOFs to system and constraint equations based on FEM and rigid body state.
//
// The constraint Jacobian may have rows referencing only FEM state, only rigid state, or both.
//
// J =  ( J_f        )   // FEM only 
//      ( J_fr  J_fr )   // FEM/rigid
//      (       J_r  )   // rigid only
//
// Relationship of impulses and velocity response:
//
// ( A_f  0   ) * ( delta_v_f )  = J^T * ( lambda_f  )
// ( 0    A_r )   ( delta_v_r )          ( lambda_fr )
//                                       ( lambda_r  )
//
// In inner solve, one of the terms of right hand side is:
// - ( J_f        ) * ( DAinv_f           ) * ( LA_f + UA_f  0 ) * ( delta_v_f ) =
//   ( J_fr  J_fr )   (           DAinv_r )   ( 0            0 )   ( delta_v_r )     // LA and UA zero for rigid bodies
//   (       J_r  )
//
// - ( J_f        ) * ( DAinv_f * (LA_f + UA_f) * delta_v_f )
//   ( J_fr  J_rf )   ( 0                                   )  
//   (       J_r  )
//
// Modifications:
//
// We have tried several tweaks to the above method to help performance or stability.  
//
// To start with we had some trouble implementing the convergence criteria from the paper in a way that would avoid 
// artifacts, and we settled on using a fixed number of iterations for the inner and outer solves.
//
// To help performance we tried to cut down the number of inner solve iterations.  Our heuristic was to gradually reduce 
// the max number of inner iterations until only doing one per outer iteration, expecting that fewer inner iterations
// should be needed as the solve progresses.
//
// However there were some instances where this reduced stability.  We found that in these cases though, stability was 
// restored by using under-relaxation in the inner solve.  Although this reduces speed of convergence, this was 
// preferred since it supported reducing iteration counts.
// 
// Note the under-relaxation and the reduction/fall-off of inner interation counts are controllable by parameters in the
// scene.  See FmSceneControlParams
//
//
// Modification using MPCG for body response:
//
// We found that the above algorithm would work well in many cases, but for some meshes and materials with higher condition 
// number the constraint solve would converge slowly and require many iterations.  One factor is the convergence rate of the
// GS method used to update the FEM body response to computed constraint forces.
//
// To improve this we tried a different approach where constraint forces are first computed as if the FEM mesh vertices
// are independent, and then the FEM body response is applied using the CG method as in the implicit integration step.
// Then, because the response will cause velocity changes that violate the constraints, the process is repeated to find 
// additional constraint forces.  The forces computed on each iteration are summed to produce the total result.  Note the 
// projection of constraint force lambda values is modified so that it takes into account the running total.
//
// For solving constraint forces on the independent vertices, we modify equation (4) so that the block diagonal values 
// for the mesh vertices are just inverse(mass) * identity, and on the right hand side we need the current estimated 
// total velocity:
//
// J * Minv * J^T * lambda(i) >= -(g0 / delta_t) - J * vel_unconstrained - J * delta_vel(i-1)
// 
// This appeared to give better results relative to cost on some of the problem content.  Performance is helped somewhat 
// by reducing the max CG iterations for more approximate solves on each iteration.  
//
// Note we did not switch to this method exclusively because we seemed to get the best behavior using the two methods together.  
// Our implementation uses the CG-based pass initially, then applies these results as a warm-start for the GS-based pass.
// Iteration counts for each of the passes can be set in the FmSceneControlParams 
// 
//
// Warm starting:
//
// As yet we have not integrated warm starting from the previous frame's constraint lambdas, though this is a common 
// method to improve convergence in rigid body simulation.  We are still investigating.
//
// References:
// - Cline and Pai, "Post-stabilization for Rigid Body Simulation with Contact and Constraints"
// - Miguel and Otaduy, "Efficient Simulation of Contact Between Rigid and Deformable Objects"
// - Otaduy et al., "Implicit Contact Handling for Deformable Objects"

// Flags used in FmConstraintParams
#define FM_SOLVER_CONSTRAINT_FLAG_NONNEG0 0x1  // Multiplier for dimension 0 of constraint must be non-negative (otherwise not limited)
#define FM_SOLVER_CONSTRAINT_FLAG_NONNEG1 0x2  // Multiplier for dimension 1 of constraint must be non-negative (otherwise not limited)
#define FM_SOLVER_CONSTRAINT_FLAG_NONNEG2 0x4  // Multiplier for dimension 2 of constraint must be non-negative (otherwise not limited)

namespace AMD
{
    struct FmConstraintIsland;
    struct FmConstraintSolverBuffer;
    struct FmConstraintsBuffer;
    struct FmConstraintReference;
    struct FmScene;
    class FmCompareConstraintRefs;
    class FmTaskDataApplySolveDeltasAndTestSleeping;
#if FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH
    class FmConstraintSolveTaskGraph;
#endif

    // Parameters that may change with each iteration
    struct FmConstraintSolverIterationParams
    {
        uint outerIteration;
        bool outerIsLastIteration;
        bool outerForwardDirection;
        bool innerForwardDirection;

        FmConstraintSolverIterationParams() : outerIteration(0), outerIsLastIteration(false), outerForwardDirection(true), innerForwardDirection(true) {}
    };

    struct FmConstraintSolverData
    {
        // Matrix and vector data used in solver algorithms

        FmSMatrix3*           DAinv;                   // Inverted block diagonal of A matrix, concatenated for all FEM meshes and rigid bodies in solver island
        FmSMatrix3*           W;                       // Inverted mass matrix, concatenated for all FEM meshes and rigid bodies in solver island
        FmSVector3*           JTlambda;                // J^T * lambda3.  Can update in constant time for each lambda_k update, then multiply by J_k in constant time
        FmConstraintJacobian  J;                       // sparse constraint Jacobian

#if FM_CONSTRAINT_STABILIZATION_SOLVE
        FmSVector3*           deltaPos;                // Position change to correct constraint error
#endif
        FmSVector3*           deltaVel;                // Correction to unconstrained velocities for the computed constraint impulses
        FmSVector3*           pgsDeltaVelTerm;         // For DAinv * (UA + LA) * deltaVel, used in computing PGS right-hand-side
        FmSVector3*           velTemp;                 // Temporary for computing pgsRhsConstant in stabilization, and for summing total rigid body JTlambda in first solver pass

        FmSVector3*           pgsRhsConstant;          // Constant term of PGS right-hand-side, (-g0 / delta_t, 0.0, 0.0)^T - J * vel_unconstrained
        FmSVector3*           pgsRhs;                  // PGS right-hand-side for iterative solution of lambda
        FmSVector3*           lambda3;                 // lambda values for 3D constraints
        FmSVector3*           lambda3Temp;             // Temporary for -g0 / delta_t; also for holding sum of lambda values in CG pass

        // Parameters controlling solver algorithms

        FmConstraintSolverControlParams   solveParams;          // Parameters for constraint solve (velocity-based)
        FmConstraintSolverControlParams   stabilizationParams;  // Parameters for constraint stabilization (position-based)
        FmConstraintSolverControlParams*  currentControlParams;
        uint                              currentPassIdx;
        FmConstraintSolverIterationParams iterationParams;      // Parameters that may change with each iteration

        // Partitioning

        FmObjectPartitionData*       objectPartitionData;             // Partition ids and other state for objects in solver
        uint*                        allPartitionConstraintIndices;   // Buffer for the constraints for all partitions
        uint*                        allPartitionObjectIds;           // Buffer for the object ids for all partitions
        uint*                        allPartitionObjectNumVertices;   // Buffer for the num vertices of all objects
        FmPartitionObjectSetElement* allPartitionObjectSetElements;   // Buffer for the object set elements for all partitions
        FmPartitionPairSet           partitionPairSet;                // Set of partition pairs found by going through all the island constraints
        FmPartitionPair*             partitionPairs;                  // Arrays of constraints and meshes for each island
        uint                         numPartitionPairs;               // Size of partitionPairs

        FmBvh                        partitionsHierarchy;             // Hierarchy used to partition bodies for parallelism
        uint*                        nodeCounts;                      // Count of constraints for sizing partitions
        FmPartition*                 partitions;                      // Partitions are taken from hierarchy nodes of sufficient size
        uint                         numPartitions;                   // Number of partitions
        uint                         numPartitionsWithConstraints;    // Number of partitions that have constraints (the subset that needs processing in constraint solve)
        uint*                        partitionPairMinSetElements;     // Memory to hold "min sets" arrays (independent sets with min hash value)
        uint*                        partitionPairMaxSetElements;     // Memory to hold "max sets" arrays (independent sets with max hash value)
        FmGraphColoringSet*          partitionPairIndependentSets;    // Independent sets of partition pairs found by graph coloring
        uint                         numPartitionPairIndependentSets; // Number of independent sets found by graph coloring

        // Task graph

#if FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH
        FmConstraintSolveTaskGraph*  taskGraph;
#endif

#if FM_CONSTRAINT_SOLVER_CONVERGENCE_TEST
        FmSolverIterationNorms externaPgsNorms;
#endif

        FmConstraintIsland* constraintIsland;
        FmConstraintSolverBuffer* constraintSolverBuffer;

        uint8_t* pDynamicAllocatedBuffer; // If non-NULL, solver memory was dynamically allocated and must be freed

        uint numTetMeshVerts;
        uint numRigidBodies;
        uint numStateVecs3;
        uint numConstraints;
        uint maxStateVecs3;
        uint maxConstraints;
        uint maxJacobianSubmats;
        bool isAllocated;

        bool IsInStabilization()
        {
            return (currentControlParams == &stabilizationParams);
        }

        FmSVector3* GetDeltaVec()
        {
#if FM_CONSTRAINT_STABILIZATION_SOLVE
            return IsInStabilization() ? deltaPos : deltaVel;
#else
            return deltaVel;
#endif
        }

        FmSMatrix3* GetBlockDiagInv(uint passIdx)
        {
            return (passIdx == 0) ? W : DAinv;
        }

        FmSVector3* GetRhsInput(uint passIdx)
        {
            return (passIdx == 0) ? GetDeltaVec() : pgsDeltaVelTerm;
        }
    };

    class FmSetupConstraintSolveTaskData
    {
    public:
        FM_CLASS_NEW_DELETE(FmSetupConstraintSolveTaskData)

        FmScene* scene;
        FmConstraintSolverData* constraintSolverData;
        FmConstraintIsland* constraintIsland;
        float delta_t;
        FmSortTaskGraph<FmConstraintReference, FmCompareConstraintRefs>* sortTaskGraph;
        FmTaskFuncCallback followTaskFunc;
        void* followTaskData;

        FmSetupConstraintSolveTaskData(
            FmScene* inScene, FmConstraintSolverData* inConstraintSolverData, FmConstraintIsland* inConstraintIsland, float inDeltaT,
            FmTaskFuncCallback inFollowTaskFunc, void* inFollowTaskData) : sortTaskGraph(NULL)
        {
            scene = inScene;
            constraintSolverData = inConstraintSolverData;
            constraintIsland = inConstraintIsland;
            delta_t = inDeltaT;
            followTaskFunc = inFollowTaskFunc;
            followTaskData = inFollowTaskData;
        }
    };

    void FmSetupConstraintSolvePostSort(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    // Description of memory needed for all constraint solver islands.
    struct FmConstraintSolverBuffer
    {
        size_t                    bufferNumBytes;

        uint*                     rigidBodySolverOffsets;  // Maps from rigid body idx to a state offset within constraint island solve
        uint*                     tetMeshIdxFromId;        // Maps from tet mesh object id to index within constraint solve data
        uint*                     rigidBodyIdxFromId;      // Maps from rigid body object id to index within constraint solve data

        FmConstraintSolverData*   islandSolverData;

        uint8_t*                  islandSolverDataArraysStart;
        size_t                    islandSolverDataArraysNumBytes;
        size_t                    islandSolverDataArraysMaxBytes;
        size_t                    islandSolverDataArraysHighWaterMark;
    };

    // Parameters needed to allocate constraint solver data
    struct FmConstraintSolverBufferSetupParams
    {
        uint maxTetMeshes;        // Limit on total tet meshes
        uint maxTetMeshVerts;     // Limit on total verts of all tet meshes
        uint maxRigidBodies;      // Limit on total rigid bodies
        uint maxConstraints;      // Limit on total 3D constraints
        size_t maxConstraintSolverDataSize;  // Limit on total constraint solver memory size

        inline FmConstraintSolverBufferSetupParams()
        {
            maxTetMeshes = 0;
            maxTetMeshVerts = 0;
            maxRigidBodies = 0;
            maxConstraints = 0;
            maxConstraintSolverDataSize = 0;
        }
    };

    // Initialize empty constraint solver data
    static inline void FmInitConstraintSolverData(
        FmConstraintSolverData* solverData,
        const FmConstraintSolverControlParams& solveParams,
        const FmConstraintSolverControlParams& stabilizationParams)
    {
        solverData->DAinv = NULL;
        solverData->W = NULL;
        solverData->JTlambda = NULL;
#if FM_CONSTRAINT_STABILIZATION_SOLVE
        solverData->deltaPos = NULL;
#endif
        solverData->deltaVel = NULL;
        solverData->pgsDeltaVelTerm = NULL;
        solverData->velTemp = NULL;
        solverData->pgsRhsConstant = NULL;
        solverData->pgsRhs = NULL;
        solverData->lambda3 = NULL;
        solverData->lambda3Temp = NULL;

        solverData->solveParams = solveParams;
        solverData->stabilizationParams = stabilizationParams;
        solverData->currentControlParams = &solverData->solveParams;
        solverData->currentPassIdx = 0;
        solverData->iterationParams = FmConstraintSolverIterationParams();

        solverData->objectPartitionData = NULL;
        solverData->allPartitionConstraintIndices = NULL;
        solverData->allPartitionObjectIds = NULL;
        solverData->allPartitionObjectSetElements = NULL;
        solverData->partitionPairSet.elements = NULL;
        solverData->partitionPairSet.maxElements = 0;
        solverData->partitionPairSet.numElements = 0;
        solverData->partitionPairs = NULL;
        solverData->numPartitionPairs = 0;
        FmInitBvh(&solverData->partitionsHierarchy);
        solverData->nodeCounts = 0;
        solverData->partitions = NULL;
        solverData->numPartitions = 0;
        solverData->numPartitionsWithConstraints = 0;
        solverData->partitionPairMinSetElements = NULL;
        solverData->partitionPairMaxSetElements = NULL;
        solverData->partitionPairIndependentSets = NULL;
        solverData->numPartitionPairIndependentSets = 0;
#if FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH
        solverData->taskGraph = NULL;
#endif
        solverData->constraintIsland = NULL;
        solverData->constraintSolverBuffer = NULL;
        solverData->pDynamicAllocatedBuffer = NULL;
        solverData->numTetMeshVerts = 0;
        solverData->numRigidBodies = 0;
        solverData->numStateVecs3 = 0;
        solverData->numConstraints = 0;
        solverData->maxStateVecs3 = 0;
        solverData->maxConstraints = 0;
        solverData->maxJacobianSubmats = 0;
        solverData->isAllocated = false;
    }

    // Initialize empty constraint solver buffer
    static inline void FmInitConstraintSolverBuffer(FmConstraintSolverBuffer* constraintSolverBuffer)
    {
        constraintSolverBuffer->bufferNumBytes = 0;
        constraintSolverBuffer->rigidBodySolverOffsets = NULL;
        constraintSolverBuffer->tetMeshIdxFromId = NULL;
        constraintSolverBuffer->rigidBodyIdxFromId = NULL;
        constraintSolverBuffer->islandSolverData = NULL;
        constraintSolverBuffer->islandSolverDataArraysStart = NULL;
        constraintSolverBuffer->islandSolverDataArraysNumBytes = 0;
        constraintSolverBuffer->islandSolverDataArraysMaxBytes = 0;
    }

    static FM_FORCE_INLINE uint FmGetRigidBodySolverOffsetById(const FmConstraintSolverBuffer& solverBuffer, uint objectId)
    {
        return solverBuffer.rigidBodySolverOffsets[objectId & ~FM_RB_FLAG];
    }

    static FM_FORCE_INLINE void FmSetRigidBodySolverOffsetById(FmConstraintSolverBuffer* solverBuffer, uint objectId, uint solverOffset)
    {
        solverBuffer->rigidBodySolverOffsets[objectId & ~FM_RB_FLAG] = solverOffset;
    }

    static FM_FORCE_INLINE FmObjectPartitionData& FmGetObjectPartitionDataRef(FmConstraintSolverData* constraintSolverData, uint objectId)
    {
        uint idx;
        if (objectId & FM_RB_FLAG)
        {
            idx = constraintSolverData->constraintSolverBuffer->rigidBodyIdxFromId[objectId & ~FM_RB_FLAG];
        }
        else
        {
            idx = constraintSolverData->constraintSolverBuffer->tetMeshIdxFromId[objectId];
        }

        return constraintSolverData->objectPartitionData[idx];
    }

    // Allocate buffers for constraint solve of all islands.
    void FmAllocConstraintIslandSolverData(FmScene* scene);

    // Get total bytes needed for FmConstraintSolverData including ConstraintSolverBufferDesc struct and all arrays.
    size_t FmGetConstraintSolverBufferSize(const FmConstraintSolverBufferSetupParams& params);

    // Suballocate memory from pBuffer to create ConstraintSolverBufferDesc and arrays.
    // Assumes pBuffer is 64-byte aligned, and pads it up to 64-bytes after allocations.
    FmConstraintSolverBuffer* FmSetupConstraintSolverBuffer(const FmConstraintSolverBufferSetupParams& params, uint8_t*& pBuffer, size_t bufferNumBytes);

    // For a test mesh within a constraint island, update positions from velocity, and compute new fractures or plastic deformation.
    void FmUpdatePositionsFracturePlasticity(FmScene* scene, FmTetMesh* tetMesh, float timestep, FmAsyncTaskData* parentTaskData);

    // NOTE: with non-NULL followTaskFunc, this function must be called from within FmExecuteTask().
    void FmSetupConstraintSolve(
        FmScene* scene,
        FmConstraintSolverData* constraintSolverData,
        FmConstraintIsland* constraintIsland,
        float delta_t,
        FmTaskFuncCallback followTaskFunc = NULL,
        void* followTaskData = NULL);

    void FmShutdownConstraintSolve(FmConstraintSolverData* constraintSolverData);

#if FM_CONSTRAINT_STABILIZATION_SOLVE
    // NOTE: with non-NULL followTaskFunc, this function must be called from within FmExecuteTask().
    void FmSetupConstraintStabilization(
        FmScene* scene,
        FmConstraintSolverData* constraintSolverData,
        FmConstraintIsland* constraintIsland,
        float delta_t,
        FmTaskFuncCallback followTaskFunc = NULL,
        void* followTaskData = NULL);
#endif

    // Solve for constraint impulses and response.
    // Requires prior setup with FmSetupConstraintSolve() or FmSetupConstraintStabilization() which configure a velocity-based solve or position-based stabilization respectively.
    // NOTE: with non-NULL followTaskFunc, this function must be called from within FmExecuteTask().
    void FmRunConstraintSolve(FmScene* scene, FmConstraintSolverData* constraintSolverData, FmConstraintIsland& constraintIsland,
        FmTaskFuncCallback followTaskFunc = NULL,
        void* followTaskData = NULL);

    // Save constraint solver lambda results in the persistent constraints (currently glue, plane, and rb angle) to communicate to application.
    void FmUpdateConstraintLambdas(FmConstraintSolverData* constraintSolverData, FmConstraintsBuffer* constraintsBuffer, const FmConstraintIsland& constraintIsland);

    // Apply island solve's delta vel and delta pos, and test objects for sleeping.
    // NOTE: with non-NULL followTaskFunc, this function must be called from within FmExecuteTask().
    void FmApplyConstraintSolveDeltasAndTestSleeping(FmScene* scene, FmConstraintSolverData* constraintSolverData, FmConstraintIsland& constraintIsland,
        FmTaskFuncCallback followTaskFunc = NULL, void* followTaskData = NULL);

    // Update sleeping stats and mark island for sleeping.
    void FmTestSleepingAndUpdateStats(FmScene* scene, const FmConstraintIsland& constraintIsland);
}
