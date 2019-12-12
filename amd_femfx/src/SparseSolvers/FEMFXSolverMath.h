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
// Types and operations for sparse matrices used in MPCG and constraint solver
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXCommonInternal.h"
#include "FEMFXTypes.h"

// Use SIMD 3D vector/matrix types to accelerate MPCG and constraint solves
#define FM_SIMD_SOLVERS              1

namespace AMD
{
    // Typedefs for vectors/matrices used in solvers
#if FM_SIMD_SOLVERS
    typedef FmSimdVector3 FmSVector3;
    typedef FmSimdMatrix3 FmSMatrix3;
    static inline FmSVector3 FmInitSVector3(float val) { return FmInitSimdVector3(val); }
    static inline FmSVector3 FmInitSVector3(float x, float y, float z) { return FmInitSimdVector3(x, y, z); }
    static inline FmSVector3 FmInitSVector3(const FmVector3& vec) { return FmInitSimdVector3(vec); }
    static inline FmSMatrix3 FmInitSMatrix3(float val) { FmSVector3 vec = FmInitSimdVector3(val); return FmInitSimdMatrix3(vec, vec, vec); }
    static inline FmSMatrix3 FmInitSMatrix3(const FmSVector3& col0, const FmSVector3& col1, const FmSVector3& col2) { return FmInitSimdMatrix3(col0, col1, col2); }
    static inline FmSMatrix3 FmInitSMatrix3(const FmMatrix3& mat) { return FmInitSimdMatrix3(mat); }
#else
    typedef FmVector3 FmSVector3;
    typedef FmMatrix3 FmSMatrix3;
    static inline FmSVector3 FmInitSVector3(float val) { return FmInitVector3(val); }
    static inline FmSVector3 FmInitSVector3(float x, float y, float z) { return FmInitVector3(x, y, z); }
    static inline FmSVector3 FmInitSVector3(const FmVector3& vec) { return vec; }
    static inline FmVector3  FmInitVector3(const FmSVector3& vec) { return vec; }
    static inline FmSMatrix3 FmInitSMatrix3(float val) { FmSVector3 vec = FmInitVector3(val); return FmInitMatrix3(vec, vec, vec); }
    static inline FmSMatrix3 FmInitSMatrix3(const FmSVector3& col0, const FmSVector3& col1, const FmSVector3& col2) { return FmInitMatrix3(col0, col1, col2); }
    static inline FmSMatrix3 FmInitSMatrix3(const FmMatrix3& mat) { return mat; }
    static inline FmMatrix3  FmInitMatrix3(const FmSMatrix3& mat) { return mat; }
#endif

    // Metrics used to measure solver convergence
    struct FmSolverIterationNorms
    {
        float maxSolutionAbsDiff;    // Max component of absolute difference of solution from previous value = Linf_norm(solution_iplus1 - solution_i)
        float solutionSqrMag;        // Square of solution vector magnitude = L2_norm(solution)^2

        void Zero()
        {
            maxSolutionAbsDiff = 0.0f;
            solutionSqrMag = 0.0f;
        }

        void Update(const FmSVector3& deltaSolution, const FmSVector3& solution)
        {
            maxSolutionAbsDiff = FmMaxFloat(maxSolutionAbsDiff, maxElem(abs(deltaSolution)));
            solutionSqrMag += dot(solution, solution);
        }

        void Update(const FmSolverIterationNorms& norms)
        {
            maxSolutionAbsDiff = FmMaxFloat(maxSolutionAbsDiff, norms.maxSolutionAbsDiff);
            solutionSqrMag += norms.solutionSqrMag;
        }
    };

    // Compressed sparse row format, used with MPCG solver
    struct FmSparseMatrixSubmat3
    {
        FmSMatrix3* submats;    // Array for all elements
        uint*       indices;
        uint*       rowStarts;  // Array for row start indices.  Extra row for end of elements
        uint        numRows;    // Rows of 3D matrices

        FmSparseMatrixSubmat3() : submats(NULL), indices(NULL), rowStarts(NULL), numRows(0)
        {}
    };

    // Constraint type which determines type of projection in Projected Gauss-Seidel.
    enum FmSolverConstraintType
    {
        FM_SOLVER_CONSTRAINT_TYPE_3D_NORMAL2DFRICTION,  // All contacts
        FM_SOLVER_CONSTRAINT_TYPE_3D_JOINT1DFRICTION,   // Portion of rigid-body hinge and sliding joints with friction
        FM_SOLVER_CONSTRAINT_TYPE_1D,                   // 1D equality or non-negative constraint
        FM_SOLVER_CONSTRAINT_TYPE_2D,                   // 2D equality or non-negative constraint
        FM_SOLVER_CONSTRAINT_TYPE_3D,                   // 3D equality or non-negative constraint
        FM_NUM_SOLVER_CONSTRAINT_TYPES
    };

    // State type which determines the size of data in a Jacobian row
    enum FmStateType
    {
        FM_STATE_TYPE_3D,
        FM_STATE_TYPE_6D,   
        FM_STATE_TYPE_N,    
        FM_NUM_STATE_TYPES
    };

    typedef uint uoffset;

    // Parameters of constraint and offsets to Jacobian data
    struct FmConstraintParams
    {
        FmSVector3             diagInverse[2];    // Inverse of diagonal of constraint matrix = J * W * J^T (0) or J * DAinv * J^T (1)
        float                  frictionCoeff;

        // Currently only 3D states are supported, but other state types (e.g., N-dimensional for reduced-coordinate linkages) 
        // can be added by duplicating these members for each state type
        uint                   jacobiansNumStates;     // number of states in row
        uoffset                jacobianSubmatsOffset;  // byte offset in submats array
        uoffset                jacobianIndicesOffset;  // byte offset in indices array

        uint8_t                type;
        uint8_t                flags;
    };

    // Compressed sparse row format for constraint solver, with parameters to allow more general data formats
    struct FmConstraintJacobian
    {
        FmConstraintParams* params;
        FmSMatrix3*         submats;    // Array for all elements
        uint*               indices;

        FmConstraintJacobian() : params(NULL), submats(NULL), indices(NULL) {}
    };

    static inline float FmVinfnorm(FmSVector3* x, uint numRows3)
    {
        float result = 0.0f;
        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            result = FmMaxFloat(result, (float)maxElem(abs(x[row3])));
        }
        return result;
    }

    static inline float FmVinfnorm(float* x, uint numVals)
    {
        float result = 0.0f;
        for (uint idx = 0; idx < numVals; idx++)
        {
            result = FmMaxFloat(result, fabsf(x[idx]));
        }
        return result;
    }

    static inline void FmVfill(FmSVector3* result, FmSVector3 v, uint numRows3)
    {
        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            result[row3] = v;
        }
    }

    static inline void FmVcV(FmSVector3* result, FmSVector3* v, uint numRows3)
    {
        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            result[row3] = v[row3];
        }
    }

    static inline void FmVpV(FmSVector3* result, FmSVector3* v, FmSVector3* w, uint numRows3)
    {
        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            result[row3] = v[row3] + w[row3];
        }
    }

    static inline void FmVmV(FmSVector3* result, FmSVector3* v, FmSVector3* w, uint numRows3)
    {
        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            result[row3] = v[row3] - w[row3];
        }
    }

    // result = vt * diag * v
    static inline float FmVtxDiagxV(const FmSMatrix3* diag, const FmSVector3* v, uint numRows3)
    {
        float result = 0.0f;
        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            FmSVector3 vRow = v[row3];
            FmSMatrix3 diagRow = diag[row3];
            result += dot(vRow, mul(diagRow, vRow));
        }
        return result;
    }

    // result = diag * v
    static inline void FmDiagxV(FmSVector3* result, const FmSMatrix3* diag, const FmSVector3* v, uint numRows3)
    {
        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            FmSVector3 vRow = v[row3];
            FmSMatrix3 diagRow = diag[row3];
            result[row3] = mul(diagRow, vRow);
        }
    }

    // result = diag0 * diag1 * v
    static inline void FmDiagxDiagxV(FmSVector3* result, const bool* kinematicFlags, const FmSMatrix3* diag1, const FmSVector3* v, uint numRows3)
    {
        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            if (kinematicFlags[row3])
            {
                result[row3] = FmInitSVector3(0.0f);
            }
            else
            {
                result[row3] = mul(diag1[row3], v[row3]);
            }
        }
    }

    // u - M * v
    static inline void FmVmMxV(FmSVector3* FM_RESTRICT result, const FmSVector3* u, const FmSparseMatrixSubmat3& M, const FmSVector3* FM_RESTRICT v, uint numRows3)
    {
        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            uint rowStart = M.rowStarts[row3];
            uint rowEnd = M.rowStarts[row3 + 1];

            FmSVector3 rowResult = u[row3];

            FM_ASSERT(rowEnd >= rowStart);

            rowResult -= mul(M.submats[row3], v[row3]);
            for (uint i = rowStart; i < rowEnd; i++)
            {
                FmSMatrix3 submat = M.submats[i];
                uint idx = M.indices[i];

                rowResult -= mul(submat, v[idx]);
            }

            result[row3] = rowResult;
        }
    }

    // result = diag * M * v
    static inline void FmDiagxMxV(FmSVector3* FM_RESTRICT result, const bool* kinematicFlags, const FmSparseMatrixSubmat3& M, const FmSVector3* FM_RESTRICT v, uint numRows3)
    {
        uint* FM_RESTRICT rowStarts = M.rowStarts;
        uint* FM_RESTRICT indices = M.indices;
        FmSMatrix3* FM_RESTRICT submats = M.submats;
        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            if (kinematicFlags[row3])
            {
                result[row3] = FmInitSVector3(0.0f);
            }
            else
            {
                uint rowStart = rowStarts[row3];
                uint rowEnd = rowStarts[row3 + 1];

                FM_ASSERT(rowEnd >= rowStart);

                FmSVector3 rowResult = mul(M.submats[row3], v[row3]);

                for (uint i = rowStart; i < rowEnd; i++)
                {
                    FmSMatrix3 submat = submats[i];
                    uint idx = indices[i];

                    rowResult += mul(submat, v[idx]);
                }

                result[row3] = rowResult;
            }
        }
    }

    // result = u + s * v
    static inline void FmVpSxV(FmSVector3* result, const FmSVector3* u, float s, const FmSVector3* v, uint numRows3)
    {
        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            FmSVector3 u_row = u[row3];
            FmSVector3 v_row = v[row3];
            result[row3] = u_row + s * v_row;
        }
    }

    // result = diag * (u + s * v)
    static inline void FmDiagx_VpSxV(FmSVector3* result, const bool* kinematicFlags, const FmSVector3* u, float s, const FmSVector3* v, uint numRows3)
    {
        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            bool kinematic = kinematicFlags[row3];

            if (kinematic)
            {
                result[row3] = FmInitSVector3(0.0f);
            }
            else
            {
                FmSVector3 u_row = u[row3];
                FmSVector3 v_row = v[row3];
                result[row3] = u_row + s * v_row;
            }
        }
    }

    // result = diag * (u + s * v)
    static inline void FmDiagx_VpSxV(FmSVector3* result, const FmSVector3* u, float s, const FmSVector3* v, uint numRows3)
    {
        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            FmSVector3 u_row = u[row3];
            FmSVector3 v_row = v[row3];
            result[row3] = u_row + s * v_row;
        }
    }

    // returns inner(u, v)
    static inline float FmVtxV(const FmSVector3* u, const FmSVector3* v, uint numRows3)
    {
        float result = 0.0f;
        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            FmSVector3 u_row = u[row3];
            FmSVector3 v_row = v[row3];
            result += dot(u_row, v_row);
        }
        return result;
    }

    // result = u - M * v
    static FM_FORCE_INLINE FmSVector3 FmVmMxV_row(FmSVector3* FM_RESTRICT result, const FmSVector3* FM_RESTRICT u, const FmConstraintJacobian& M, const FmSVector3* FM_RESTRICT v, uint constraintIdx)
    {
        FmConstraintParams& constraintParams = M.params[constraintIdx];

        FmSMatrix3* FM_RESTRICT jacobianSubmats = (FmSMatrix3*)((uint8_t*)M.submats + constraintParams.jacobianSubmatsOffset);
        uint* FM_RESTRICT jacobianIndices = (uint*)((uint8_t*)M.indices + constraintParams.jacobianIndicesOffset);
        uint rowSize = constraintParams.jacobiansNumStates;

        FmSVector3 rowResult = u[constraintIdx];

        for (uint i = 0; i < rowSize; i++)
        {
            FmSMatrix3 submat = jacobianSubmats[i];
            uint idx = jacobianIndices[i];
            FmSVector3 v_idx = v[idx];

            rowResult -= mul(submat, v_idx);
        }

        result[constraintIdx] = rowResult;
        return rowResult;
    }

    // result = u - M * v
    static inline void FmVmMxV(FmSVector3* result, const FmSVector3* u, const FmConstraintJacobian& M, const FmSVector3* v, uint numConstraints)
    {
        for (uint constraintIdx = 0; constraintIdx < numConstraints; constraintIdx++)
        {
            FmVmMxV_row(result, u, M, v, constraintIdx);
        }
    }

    // Diagonal block of M * diag * M^T row
    static inline FmSMatrix3 FmDiagOfMxDiagxMT(const FmConstraintJacobian& M, const FmSMatrix3* FM_RESTRICT diag, uint constraintIdx)
    {
        FmConstraintParams& constraintParams = M.params[constraintIdx];

        FmSMatrix3* FM_RESTRICT jacobianSubmats = (FmSMatrix3*)((uint8_t*)M.submats + constraintParams.jacobianSubmatsOffset);
        uint* FM_RESTRICT jacobianIndices = (uint*)((uint8_t*)M.indices + constraintParams.jacobianIndicesOffset);
        uint rowSize = constraintParams.jacobiansNumStates;

        FmSMatrix3 result(0.0f);
        for (uint submatIdx = 0; submatIdx < rowSize; submatIdx++)
        {
            FmSMatrix3 submat = jacobianSubmats[submatIdx];
            uint idx = jacobianIndices[submatIdx];
            FmSMatrix3 diag_idx = diag[idx];

            result += mul(submat, mul(diag_idx, transpose(submat)));
        }
        return result;
    }

}
