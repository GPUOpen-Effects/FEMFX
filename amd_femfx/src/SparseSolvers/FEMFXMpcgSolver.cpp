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

#include "FEMFXMpcgSolver.h"
#include "FEMFXTetMesh.h"

namespace AMD
{

    void FmAllocMpcgSolverData(
        FmMpcgSolverData* data,
        uint maxVerts,
        uint maxVertAdjacentVerts)  // bound on total number of adjacent verts ("adjacent" here including self), dependent on fracture
    {
        data->kinematicFlags = new bool[maxVerts];
        data->PInvDiag = new FmSMatrix3[maxVerts];
        data->b = new FmSVector3[maxVerts];
        data->mass = new float[maxVerts];
        data->norms.Zero();
        data->solverStateOffset = 0;
        data->A.submats = new FmSMatrix3[maxVerts + maxVertAdjacentVerts];
        data->A.indices = new uint[maxVerts + maxVertAdjacentVerts];
        data->A.rowStarts = new uint[maxVerts + 1];       // last rowStart is end of matrix
        data->A.numRows = 0;
        data->maxVertAdjacentVerts = maxVertAdjacentVerts;
        data->hasKinematicVerts = false;
    }

    void FmAllocMpcgSolverData(FmMpcgSolverData* data, const FmTetMesh& tetMesh)
    {
        FmAllocMpcgSolverData(data, tetMesh.maxVerts, tetMesh.maxVerts + tetMesh.vertConnectivity.numIncidentTetsTotal * 4);
    }

    void FmFreeMpcgSolverData(FmMpcgSolverData* data)
    {
        FM_DELETE_ARRAY(data->kinematicFlags);
        FM_DELETE_ARRAY(data->PInvDiag);
        FM_DELETE_ARRAY(data->b);
        FM_DELETE_ARRAY(data->mass);
        FM_DELETE_ARRAY(data->A.submats);
        FM_DELETE_ARRAY(data->A.indices);
        FM_DELETE_ARRAY(data->A.rowStarts);

        data->A.numRows = 0;
        data->maxVertAdjacentVerts = 0;
        data->solverStateOffset = 0;
        data->hasKinematicVerts = false;
    }

    // Run unconstrained MPCG solve of implicit Euler step, assuming no kinematic vertices
    // Returns numbers of steps taken.
    // Terminates when mag of residual < epsilon * mag of right-hand-side or maxIterations reached.
    uint FmRunMpcgSolveNoKinematic(FmSVector3* solution, FmMpcgSolverData* solverData, FmMpcgSolverDataTemps* solverDataTemps, float epsilon, uint maxIterations)
    {
        FmMpcgSolverData& data = *solverData;
        FmMpcgSolverDataTemps& temps = *solverDataTemps;

        FmSparseMatrixSubmat3& FM_RESTRICT A = data.A;
        FmSMatrix3* FM_RESTRICT PInvDiag = data.PInvDiag;
        FmSVector3* FM_RESTRICT x = solution;
        uint* FM_RESTRICT rowStarts = A.rowStarts;
        uint* FM_RESTRICT indices = A.indices;
        FmSMatrix3* FM_RESTRICT submats = A.submats;
        FmSVector3* FM_RESTRICT r = temps.r;
        FmSVector3* FM_RESTRICT p = temps.p;
        FmSVector3* FM_RESTRICT s = temps.s;
        FmSVector3* FM_RESTRICT h = temps.h;
        FmSVector3* FM_RESTRICT b = solverData->b;

        // Setup
        // solution already set to initial estimate in InitMpcgSolve

        uint numRows3 = data.A.numRows;

        // z = solution constraints
        // z' = (1 - S) * z = solution constraints or 0 for unconstrained verts 
        // s = S * (b - A * z')
        // r = S * (b - A * x)
        // p = S * Pinv * r
        // bdelta = s^T * Pinv * s
        // delta = r^T * p
        float bdelta = 0.0f;
        float delta = 0.0f;
        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            uint rowStart = rowStarts[row3];
            uint rowEnd = rowStarts[row3 + 1];

            FmSVector3 sResult = b[row3];
            FmSVector3 rResult = sResult;

            FmSMatrix3 PInvDiag_row = PInvDiag[row3];

            FM_ASSERT(rowEnd >= rowStart);

            rResult -= mul(submats[row3], x[row3]);

            for (uint i = rowStart; i < rowEnd; i++)
            {
                FmSMatrix3 submat = submats[i];
                uint idx = indices[i];
                FmSVector3 x_idx = x[idx];

                FmSVector3 ax = mul(submat, x_idx);
                rResult -= ax;
            }

            FmSVector3 pResult = mul(PInvDiag_row, rResult);

            bdelta += dot(sResult, mul(PInvDiag_row, sResult));
            delta += dot(rResult, pResult);

            r[row3] = rResult;
            p[row3] = pResult;
        }

        const float bdeltatol = 0.0f;
        if (bdelta <= bdeltatol)
        {
            return 0;
        }

        uint numIterations = 0;
        float deltaold = delta;

        float tol = epsilon * epsilon * bdelta;

        while (delta > tol && numIterations < maxIterations)
        {
            if (numIterations > 0)
            {
                // p = S * (h + (delta/deltaold)*p)
                FmDiagx_VpSxV(p, h, delta / deltaold, p, numRows3);
            }

            // s = S * A * p
            float pdots = 0.0f;
            for (uint row3 = 0; row3 < numRows3; row3++)
            {
                uint rowStart = rowStarts[row3];
                uint rowEnd = rowStarts[row3 + 1];

#if 0
                FM_ASSERT(rowEnd >= rowStart);
                FmSVector3 p_row = p[row3];
                FmSVector3 rowResult0 = mul(submats[row3], p_row);
                FmSVector3 rowResult1 = FmInitSVector3(0.0f);
                FmSVector3 rowResult2 = FmInitSVector3(0.0f);
                FmSVector3 rowResult3 = FmInitSVector3(0.0f);

                const uint groupSize = 4;
                uint numGroups = rowSize / groupSize;

                FmSMatrix3 submat0, submat1, submat2, submat3;
                uint idx0, idx1, idx2, idx3;
                FmSVector3 p0, p1, p2, p3;

                if (numGroups > 0)
                {
                    submat0 = submats[rowStart];
                    submat1 = submats[rowStart + 1];
                    submat2 = submats[rowStart + 2];
                    submat3 = submats[rowStart + 3];
                    idx0 = indices[rowStart];
                    idx1 = indices[rowStart + 1];
                    idx2 = indices[rowStart + 2];
                    idx3 = indices[rowStart + 3];
                    p0 = p[idx0];
                    p1 = p[idx1];
                    p2 = p[idx2];
                    p3 = p[idx3];

                    uint groupStart = rowStart + 4;
                    for (uint i = 0; i < numGroups - 1; i++)
                    {
                        rowResult0 += mul(submat0, p0);
                        rowResult1 += mul(submat1, p1);
                        rowResult2 += mul(submat2, p2);
                        rowResult3 += mul(submat3, p3);

                        submat0 = submats[groupStart];
                        submat1 = submats[groupStart + 1];
                        submat2 = submats[groupStart + 2];
                        submat3 = submats[groupStart + 3];

                        idx0 = indices[groupStart];
                        idx1 = indices[groupStart + 1];
                        idx2 = indices[groupStart + 2];
                        idx3 = indices[groupStart + 3];

                        p0 = p[idx0];
                        p1 = p[idx1];
                        p2 = p[idx2];
                        p3 = p[idx3];

                        groupStart += 4;
                    }

                    rowResult0 += mul(submat0, p0);
                    rowResult1 += mul(submat1, p1);
                    rowResult2 += mul(submat2, p2);
                    rowResult3 += mul(submat3, p3);
                }

                uint startIdx = rowStart + numGroups * groupSize;
                uint rowEnd = rowEnd;

                if (startIdx < rowEnd)
                {
                    submat0 = submats[startIdx];
                    idx0 = indices[startIdx];
                    p0 = p[idx0];

                    for (uint i = startIdx; i < rowEnd - 1; i++)
                    {
                        rowResult0 += mul(submat0, p0);

                        submat0 = submats[i + 1];
                        idx0 = indices[i + 1];
                        p0 = p[idx0];
                    }

                    rowResult0 += mul(submat0, p0);
                }

                FmSVector3 rowResult = rowResult0 + rowResult1;
                rowResult += (rowResult2 + rowResult3);
#else

                FM_ASSERT(rowEnd >= rowStart);

                FmSMatrix3 submat = submats[row3];
                FmSVector3 p_row = p[row3];
                FmSVector3 rowResult = mul(submat, p_row);

                for (uint i = rowStart; i < rowEnd; i++)
                {
                    submat = submats[i];
                    uint idx = indices[i];
                    FmSVector3 p_idx = p[idx];

                    rowResult += mul(submat, p_idx);
                }
#endif

                pdots += dot(p_row, rowResult);
                s[row3] = rowResult;
            }

            // alpha = delta / (p^T * s)
            float alpha = delta / pdots;

            // x = x + alpha * p
            // r = r - alpha * s
            // h = Pinv * r
            // delta = r^T * h
            deltaold = delta;
            delta = 0.0f;
            for (uint row3 = 0; row3 < numRows3; row3++)
            {
                FmSVector3 x_row = x[row3];
                FmSVector3 p_row = p[row3];
                FmSVector3 r_row = r[row3];
                FmSVector3 s_row = s[row3];
                FmSMatrix3 diagRow = PInvDiag[row3];

                FmSVector3 r_new = r_row - alpha * s_row;
                FmSVector3 h_new = mul(diagRow, r_new);

                delta += dot(r_new, h_new);

                x[row3] = x_row + alpha * p_row;
                r[row3] = r_new;
                h[row3] = h_new;
            }

            numIterations++;
        }

        return numIterations;
    }

    // Run unconstrained MPCG solve of implicit Euler step.
    // Returns numbers of steps taken.
    // Terminates when mag of residual < epsilon * mag of right-hand-side or maxIterations reached.
    uint FmRunMpcgSolve(FmSVector3* solution, FmMpcgSolverData* solverData, FmMpcgSolverDataTemps* solverDataTemps, float epsilon, uint maxIterations)
    {
        FM_TRACE_SCOPED_EVENT(MPCG_SOLVE);

        if (!solverData->hasKinematicVerts)
        {
            return FmRunMpcgSolveNoKinematic(solution, solverData, solverDataTemps, epsilon, maxIterations);
        }
        FmMpcgSolverData& data = *solverData;
        FmMpcgSolverDataTemps& temps = *solverDataTemps;

        FmSparseMatrixSubmat3& FM_RESTRICT A = data.A;
        FmSMatrix3* FM_RESTRICT PInvDiag = data.PInvDiag;
        FmSVector3* FM_RESTRICT x = solution;
        uint* FM_RESTRICT rowStarts = A.rowStarts;
        uint* FM_RESTRICT indices = A.indices;
        FmSMatrix3* FM_RESTRICT submats = A.submats;
        FmSVector3* FM_RESTRICT r = temps.r;
        FmSVector3* FM_RESTRICT p = temps.p;
        FmSVector3* FM_RESTRICT s = temps.s;
        FmSVector3* FM_RESTRICT h = temps.h;
        FmSVector3* FM_RESTRICT b = solverData->b;
        bool* FM_RESTRICT kinematicFlags = data.kinematicFlags;

        // Setup
        // solution already set to initial estimate in InitMpcgSolve

        uint numRows3 = data.A.numRows;

        // z = solution constraints
        // z' = (1 - S) * z = solution constraints or 0 for unconstrained verts 
        // s = S * (b - A * z')
        // r = S * (b - A * x)
        // p = S * Pinv * r
        // bdelta = s^T * Pinv * s
        // delta = r^T * p
        float bdelta = 0.0f;
        float delta = 0.0f;
        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            FmSVector3 sResult = FmInitSVector3(0.0f);
            FmSVector3 rResult = FmInitSVector3(0.0f);
            FmSVector3 pResult = FmInitSVector3(0.0f);
            if (!kinematicFlags[row3])
            {
                uint rowStart = rowStarts[row3];
                uint rowEnd = rowStarts[row3 + 1];

                sResult = b[row3];
                rResult = sResult;

                FmSMatrix3 PInvDiag_row = PInvDiag[row3];

                FM_ASSERT(rowEnd >= rowStart);

                {
                    FmSMatrix3 submat = submats[row3];
                    uint idx = row3;
                    FmSVector3 x_idx = x[idx];
                    bool kinematic = kinematicFlags[idx];

                    FmSVector3 ax = mul(submat, x_idx);
                    if (kinematic)
                    {
                        sResult -= ax;
                    }
                    rResult -= ax;
                }

                for (uint i = rowStart; i < rowEnd; i++)
                {
                    FmSMatrix3 submat = submats[i];
                    uint idx = indices[i];
                    FmSVector3 x_idx = x[idx];
                    bool kinematic = kinematicFlags[idx];

                    FmSVector3 ax = mul(submat, x_idx);
                    if (kinematic)
                    {
                        sResult -= ax;
                    }
                    rResult -= ax;
                }

                pResult = mul(PInvDiag_row, rResult);

                bdelta += dot(sResult, mul(PInvDiag_row, sResult));
                delta += dot(rResult, pResult);
            }

            r[row3] = rResult;
            p[row3] = pResult;
        }

        const float bdeltatol = 0.0f;
        if (bdelta <= bdeltatol)
        {
            return 0;
        }

        uint numIterations = 0;
        float deltaold = delta;

        float tol = epsilon * epsilon * bdelta;

        while (delta > tol && numIterations < maxIterations)
        {
            if (numIterations > 0)
            {
                // p = S * (h + (delta/deltaold)*p)
                FmDiagx_VpSxV(p, kinematicFlags, h, delta / deltaold, p, numRows3);
            }

            // s = S * A * p
            float pdots = 0.0f;
            for (uint row3 = 0; row3 < numRows3; row3++)
            {
                bool kinematic = kinematicFlags[row3];

                if (kinematic)
                {
                    s[row3] = FmInitSVector3(0.0f);
                }
                else
                {
                    uint rowStart = rowStarts[row3];
                    uint rowEnd = rowStarts[row3 + 1];

                    FM_ASSERT(rowEnd >= rowStart);

                    FmSMatrix3 submat = submats[row3];
                    FmSVector3 p_row = p[row3];
                    FmSVector3 rowResult = mul(submat, p_row);

                    for (uint i = rowStart; i < rowEnd; i++)
                    {
                        submat = submats[i];
                        uint idx = indices[i];
                        FmSVector3 p_idx = p[idx];

                        rowResult += mul(submat, p_idx);
                    }

                    pdots += dot(p_row, rowResult);
                    s[row3] = rowResult;
                }
            }

            // alpha = delta / (p^T * s)
            float alpha = delta / pdots;

            // x = x + alpha * p
            // r = r - alpha * s
            // h = Pinv * r
            // delta = r^T * h
            deltaold = delta;
            delta = 0.0f;
            for (uint row3 = 0; row3 < numRows3; row3++)
            {
                FmSVector3 x_row = x[row3];
                FmSVector3 p_row = p[row3];
                FmSVector3 r_row = r[row3];
                FmSVector3 s_row = s[row3];
                FmSMatrix3 diagRow = PInvDiag[row3];

                FmSVector3 r_new = r_row - alpha * s_row;
                FmSVector3 h_new = mul(diagRow, r_new);

                delta += dot(r_new, h_new);

                x[row3] = x_row + alpha * p_row;
                r[row3] = r_new;
                h[row3] = h_new;
            }

            numIterations++;
        }

        return numIterations;
    }
}