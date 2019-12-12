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
// Modified Preconditioned Conjugate Gradient solver
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXCommonInternal.h"
#include "FEMFXSolverMath.h"

// Modified Preconditioned Conjugate Gradient (MPCG) Solver
//
// Used to solve a backward-Euler integration step for the linear FEM dynamics ODE, with structure:
//
//   (M + delta_t * C + delta_t^2 * K') * vel_end = M * vel_start + delta_t * (external_forces - f0' - K' * pos_start)
// =                                  A * vel_end = b
//
// M = mass matrix (diagonal)
// C = damping matrix (using Rayleigh damping, C = kRayleighMassDamping * M + kRayleighStiffnessDamping * K')
// K' = sparse matrix summing all element rotated stiffness terms
// f0' = sum of element force offsets from rotated stiffness
//
// The original MPCG can constrain vertex velocities in one, two, or three directions, for kinematically controlling verts or simulating 
// contact.  This implementation only supports unconstrained or constrained in all directions.
//
// References: 
// - Baraff and Witkin, "Large Steps in Cloth Simulation"
// - Boxerman, "Speeding Up Cloth Simulation"

namespace AMD
{
    struct FmTetMesh;

    // Temporary data needed during MPCG solve.
    struct FmMpcgSolverDataTemps
    {
        FmSVector3* solution;       // Solution
        FmSVector3* r;              // Residual
        FmSVector3* p;
        FmSVector3* s;
        FmSVector3* h;

        static size_t GetNumBytes(uint maxVerts)
        {
            return FM_PAD_16(sizeof(*solution) * maxVerts)
                + FM_PAD_16(sizeof(*r) * maxVerts)
                + FM_PAD_16(sizeof(*p) * maxVerts)
                + FM_PAD_16(sizeof(*s) * maxVerts)
                + FM_PAD_16(sizeof(*h) * maxVerts);
        }

        void Alloc(uint8_t** pBuffer, uint8_t* pBufferEnd, uint maxVerts)
        {
            solution = FmAllocFromBuffer<FmSVector3>(pBuffer, maxVerts, pBufferEnd);
            r = FmAllocFromBuffer<FmSVector3>(pBuffer, maxVerts, pBufferEnd);
            p = FmAllocFromBuffer<FmSVector3>(pBuffer, maxVerts, pBufferEnd);
            s = FmAllocFromBuffer<FmSVector3>(pBuffer, maxVerts, pBufferEnd);
            h = FmAllocFromBuffer<FmSVector3>(pBuffer, maxVerts, pBufferEnd);
        }
    };

    // Collected data for MPCG solution of A * x = b
    struct FmMpcgSolverData
    {
        bool*                  kinematicFlags; // Flag storing whether vertex is kinematically driven
        FmSMatrix3*            PInvDiag;       // Block 3x3 diagonal preconditioner inverse
        FmSparseMatrixSubmat3  A;              // 3 rows of A per vertex
        FmSVector3*            b;              // Right-hand-side

        // Only used in constraint solve
        float*                 mass;              // Mass of each vertex  
        FmSolverIterationNorms norms;             // Computed in GS to measure convergence
        uint                   solverStateOffset; // Starting index of this mesh's verts in FmConstraintSolverData arrays

        uint numVertAdjacentVerts;
        uint maxVertAdjacentVerts;   // Bound on total number of adjacent verts ("adjacent" here including self); bounds memory needed for stiffness matrix

        bool hasKinematicVerts;
    };

    // Init empty MPCG solver data
    static inline void FmInitMpcgSolverData(FmMpcgSolverData* solverData)
    {
        solverData->kinematicFlags = NULL;
        solverData->PInvDiag = NULL;
        solverData->A.submats = NULL;
        solverData->A.indices = NULL;
        solverData->A.rowStarts = NULL;
        solverData->A.numRows = 0;
        solverData->b = NULL;
        solverData->mass = NULL;
        solverData->norms.Zero();
        solverData->solverStateOffset = FM_INVALID_ID;

        solverData->numVertAdjacentVerts = 0;
        solverData->maxVertAdjacentVerts = 0;
        solverData->hasKinematicVerts = false;
    }

    void FmAllocMpcgSolverData(
        FmMpcgSolverData* data,
        uint maxVerts,
        uint maxVertAdjacentVerts);  // Bound on total number of adjacent ("adjacent" here including self) verts, dependent on fracture

    // Alloc MPCG Solver data large enough for tet mesh
    void FmAllocMpcgSolverData(FmMpcgSolverData* data, const FmTetMesh& tetMesh);

    void FmFreeMpcgSolverData(FmMpcgSolverData* data);

    // Run unconstrained MPCG solve of implicit Euler step.
    // Returns numbers of steps taken.
    uint FmRunMpcgSolve(FmSVector3* solution, FmMpcgSolverData* data, FmMpcgSolverDataTemps* temps, float relErrorTol, uint maxIterations);
}