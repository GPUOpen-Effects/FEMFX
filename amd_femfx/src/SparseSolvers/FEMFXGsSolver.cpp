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

#include "FEMFXGsSolver.h"
#include "FEMFXRandom.h"

namespace AMD
{
    static FM_FORCE_INLINE void FmGsIterationRow(FmSolverIterationNorms& norms, FmSVector3* x, FmMpcgSolverData& meshData, FmSVector3* b, uint row3, bool isKinematic)
    {
        uint vertOffset = meshData.solverStateOffset;
        const FmSparseMatrixSubmat3& A = meshData.A;
        const FmSMatrix3* DAinv = meshData.PInvDiag;

        FmSVector3 rowResult = b[vertOffset + row3];
        FmSMatrix3 DAinv_row = DAinv[row3];
        FmSVector3 x_row = x[vertOffset + row3];

        if (!isKinematic)
        {
            uint rowStart = A.rowStarts[row3];
            uint rowEnd = A.rowStarts[row3 + 1];

            // multiply non-diagonal entries

            FM_ASSERT(rowEnd >= rowStart);

            for (uint i = rowStart; i < rowEnd; i++)
            {
                FmSMatrix3 rowMatrix = A.submats[i];
                uint idx = A.indices[i];
                FmSVector3 x_idx = x[vertOffset + idx];

                rowResult -= mul(rowMatrix, x_idx);
            }
        }

        FmSVector3 xnew = mul(DAinv_row, rowResult);

#if FM_CONSTRAINT_SOLVER_CONVERGENCE_TEST
        FmSVector3 xprev = x_row;
        norms.Update(xnew - xprev, xnew);
#else
        (void)norms;
#endif

        x[vertOffset + row3] = xnew;
    }

    void FmGsIteration(FmSVector3* x, FmMpcgSolverData& meshData, FmSVector3* b)
    {
        uint numRows3 = meshData.A.numRows;

        FmSolverIterationNorms norms;
        norms.Zero();

        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            FmGsIterationRow(norms, x, meshData, b, row3, meshData.kinematicFlags[row3]);
        }

        meshData.norms = norms;
    }

    void FmGsIterationReverse(FmSVector3* x, FmMpcgSolverData& meshData, FmSVector3* b)
    {
        uint numRows3 = meshData.A.numRows;

        FmSolverIterationNorms norms;
        norms.Zero();

        for (int row3 = numRows3 - 1; row3 >= 0; row3--)
        {
            FmGsIterationRow(norms, x, meshData, b, row3, meshData.kinematicFlags[row3]);
        }

        meshData.norms = norms;
    }

}
