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
// Estimate of conditioning of system matrix used in implicit Euler solve
//---------------------------------------------------------------------------------------

#include "AMD_FEMFX.h"
#include "FEMFXMpcgSolverSetup.h"
#include "FEMFXTetMesh.h"
#include "FEMFXSvd3x3.h"

namespace AMD
{
    static int FmSign(float val)
    {
        return (val > 0.0f) - (val < 0.0f);
    }

    static float FmL1NormV(FmSVector3* v, uint numRows3)
    {
        float norm = 0.0f;
        for (uint row = 0; row < numRows3; row++)
        {
            FmSVector3 vAbs = abs(v[row]);
            norm += vAbs.x + vAbs.y + vAbs.z;
        }
        return norm;
    }

    static void FmVsigns(FmSVector3* signs, FmSVector3* v, uint numRows3)
    {
        for (uint row = 0; row < numRows3; row++)
        {
            signs[row].x = (float)FmSign(v[row].x);
            signs[row].y = (float)FmSign(v[row].y);
            signs[row].z = (float)FmSign(v[row].z);
        }
    }

    // return max element of v, vector index and dimension 0..2 of max element
    static float FmMaxAbsElem(uint* resultVIdx, uint* resultDim, FmSVector3* resultMask, FmSVector3* v, uint numRows3)
    {
        float maxAbs = -1.0f;
        uint dim = 0;
        uint vIdx = 0;
        for (uint row = 0; row < numRows3; row++)
        {
            FmSVector3 vAbs = abs(v[row]);
            if (vAbs.x > maxAbs)
            {
                maxAbs = vAbs.x;
                vIdx = row;
                dim = 0;
            }
            if (vAbs.y > maxAbs)
            {
                maxAbs = vAbs.y;
                vIdx = row;
                dim = 1;
            }
            if (vAbs.z > maxAbs)
            {
                maxAbs = vAbs.z;
                vIdx = row;
                dim = 2;
            }
        }

        *resultVIdx = vIdx;
        *resultDim = dim;
        *resultMask = FmInitSVector3((float)(dim == 0), (float)(dim == 1), (float)(dim == 2));

        return maxAbs;
    }

    // result = SqrtDAinv * M * SqrtDAinv * v
    static inline void FmPreMxV(FmSVector3* result, const FmSMatrix3* SqrtDAinv, const FmSparseMatrixSubmat3& M, const FmSVector3* v, uint numRows3)
    {
        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            uint rowStart = M.rowStarts[row3];
            uint rowEnd = M.rowStarts[row3 + 1];

            FmSVector3 rowResult = FmInitSVector3(0.0f);

            FM_ASSERT(rowEnd >= rowStart);

            rowResult += mul(M.submats[row3], mul(SqrtDAinv[row3], v[row3]));

            for (uint i = rowStart; i < rowEnd; i++)
            {
                FmSMatrix3 submat = M.submats[i];
                uint idx = M.indices[i];

                rowResult += mul(submat, mul(SqrtDAinv[idx], v[idx]));
            }

            result[row3] = mul(SqrtDAinv[row3], rowResult);
        }
    }

    // result = u - SqrtDAinv * M * SqrtDAinv * v
    static inline void FmVmPreMxV(FmSVector3* result, FmSVector3* u, const FmSMatrix3* SqrtDAinv, const FmSparseMatrixSubmat3& M, const FmSVector3* v, uint numRows3)
    {
        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            uint rowStart = M.rowStarts[row3];
            uint rowEnd = M.rowStarts[row3 + 1];

            FmSVector3 rowResult = FmInitSVector3(0.0f);

            FM_ASSERT(rowEnd >= rowStart);

            rowResult += mul(M.submats[row3], mul(SqrtDAinv[row3], v[row3]));

            for (uint i = rowStart; i < rowEnd; i++)
            {
                FmSMatrix3 submat = M.submats[i];
                uint idx = M.indices[i];

                rowResult += mul(submat, mul(SqrtDAinv[idx], v[idx]));
            }

            result[row3] = u[row3] - mul(SqrtDAinv[row3], rowResult);
        }
    }

    // Solve system: (SqrtDiagA^-1 * A * SqrtDiagA^-1) * x = b
    static uint FmPcgInModifiedHager(FmSVector3* solution, FmSMatrix3* SqrtDAinv, FmMpcgSolverData* solverData, FmSVector3* b, float tol, uint maxIterations)
    {
        FmMpcgSolverData& data = *solverData;

        FmMpcgSolverDataTemps temps;
        size_t tempsNumBytes = FmMpcgSolverDataTemps::GetNumBytes(solverData->A.numRows);
        uint8_t* tempsBufferStart = (uint8_t*)FmAlignedMalloc(tempsNumBytes, 16);
        uint8_t* tempsBuffer = tempsBufferStart;
        temps.Alloc(&tempsBuffer, tempsBuffer + tempsNumBytes, solverData->A.numRows);

        uint numRows3 = data.A.numRows;

        FmVmPreMxV(temps.r, b, SqrtDAinv, data.A, solution, numRows3);
        FmVcV(temps.p, temps.r, numRows3);

        // delta = r^T * r
        float delta = FmVtxV(temps.r, temps.r, numRows3);
        float deltaold = delta;
        float bdelta = delta;

        uint numIterations = 0;
        while (delta > tol*tol*bdelta && numIterations < maxIterations)
        {
            if (numIterations > 0)
            {
                // p = r + (delta/deltaold)*p
                FmVpSxV(temps.p, temps.r, delta / deltaold, temps.p, numRows3);
            }

            // s = A * p
            FmPreMxV(temps.s, SqrtDAinv, data.A, temps.p, numRows3);
            // alpha = delta / (p^T * s)
            float alpha = delta / FmVtxV(temps.p, temps.s, numRows3);
            // x = x + alpha * p
            FmVpSxV(solution, solution, alpha, temps.p, numRows3);
            // r = r - alpha * s
            FmVpSxV(temps.r, temps.r, -alpha, temps.s, numRows3);
            // delta = r^T * r
            deltaold = delta;
            delta = FmVtxV(temps.r, temps.r, numRows3);

            numIterations++;
        }

        FmAlignedFree(tempsBufferStart);

        return numIterations;
    }

    float FmL1NormInversePreconditioned(FmSMatrix3* SqrtDAinv, FmMpcgSolverData* solverData)
    {
        float pInv = 0.0f;
        uint numRows3 = solverData->A.numRows;

        uint maxIterations = 2000;
        float tolerance = FLT_EPSILON;

        FmSVector3* b = new FmSVector3[numRows3];
        FmSVector3* x = new FmSVector3[numRows3];
        FmSVector3* y = new FmSVector3[numRows3];
        FmSVector3* z = new FmSVector3[numRows3];

        float bVal = 1.0f / (numRows3 * 3);
        FmVfill(b, FmInitSVector3(bVal), numRows3);

        while (true)
        {
            FmVfill(x, FmInitSVector3(0.0f), numRows3);
            FmPcgInModifiedHager(x, SqrtDAinv, solverData, b, tolerance, maxIterations);

            FmPreMxV(y, SqrtDAinv, solverData->A, x, numRows3);

            float xL1 = FmL1NormV(x, numRows3);

            if (xL1 <= pInv)
            {
                break;
            }
            else
            {
                pInv = xL1;
            }

            FmVsigns(y, x, numRows3);

            FmVfill(z, FmInitSVector3(0.0f), numRows3);
            FmPcgInModifiedHager(z, SqrtDAinv, solverData, y, tolerance, maxIterations);

            uint vIdx, dim;
            FmSVector3 mask;
            float zMaxAbs = FmMaxAbsElem(&vIdx, &dim, &mask, z, numRows3);

            if (zMaxAbs < FmVtxV(z, b, numRows3))
            {
                break;
            }
            else
            {
                FmVfill(b, FmInitSVector3(0.0f), numRows3);

                b[vIdx] = mask;
            }
        }

        delete[] b;
        delete[] x;
        delete[] y;
        delete[] z;

        return pInv;
    }

    float FmL1NormPreconditioned(FmSMatrix3* SqrtDAinv, FmMpcgSolverData* solverData)
    {
        float pForward = 0.0f;
        uint numRows3 = solverData->A.numRows;

        FmSVector3* b = new FmSVector3[numRows3];
        FmSVector3* x = new FmSVector3[numRows3];
        FmSVector3* y = new FmSVector3[numRows3];
        FmSVector3* z = new FmSVector3[numRows3];

        float bVal = 1.0f / (numRows3 * 3);
        FmVfill(b, FmInitSVector3(bVal), numRows3);

        while (true)
        {
            FmPreMxV(x, SqrtDAinv, solverData->A, b, numRows3);

            float xL1 = FmL1NormV(x, numRows3);

            if (xL1 <= pForward)
            {
                break;
            }
            else
            {
                pForward = xL1;
            }

            FmVsigns(y, x, numRows3);

            FmPreMxV(z, SqrtDAinv, solverData->A, y, numRows3);

            uint vIdx, dim;
            FmSVector3 mask;
            float zMaxAbs = FmMaxAbsElem(&vIdx, &dim, &mask, z, numRows3);

            if (zMaxAbs < FmVtxV(z, b, numRows3))
            {
                break;
            }
            else
            {
                FmVfill(b, FmInitSVector3(0.0f), numRows3);

                b[vIdx] = mask;
            }
        }

        delete[] b;
        delete[] x;
        delete[] y;
        delete[] z;

        return pForward;
    }

    // Estimate L1-based condition number of system matrix used for FEM Implicit Euler solve.
    // Reference: Kushida, "Condition Number Estimation of Preconditioned Matrices"
    float FmEstimateConditionNumber(FmMpcgSolverData* solverData)
    {
        uint numRows3 = solverData->A.numRows;

        // Given preconditioning matrix M, algorithm requires M1 and M2 with M1^T = M2 and M1*M2 = M.
        // In this case M is block diagonal of system matrix, M1 and M2 are matrix sqrt of M.
        // Can compute matrix sqrt of symmetric matrix from the eigen-decomposition

        FmSMatrix3* SqrtDAinv = new FmSMatrix3[numRows3];
        for (uint row = 0; row < numRows3; row++)
        {
            FmVector3 eigenvals;
            FmMatrix3 eigenvectors;
            FmMatrix3 symmMat = FmInitMatrix3(solverData->PInvDiag[row]);

            FmEigenSymm3x3CyclicJacobi(&eigenvals, &eigenvectors, symmMat);

            // Ensure positive eigenvalues
            if (eigenvals.x < 0.0f)
            {
                eigenvals.x = -eigenvals.x;
                eigenvectors.col0 = -eigenvectors.col0;
            }
            if (eigenvals.y < 0.0f)
            {
                eigenvals.y = -eigenvals.y;
                eigenvectors.col1 = -eigenvectors.col1;
            }
            if (eigenvals.z < 0.0f)
            {
                eigenvals.z = -eigenvals.z;
                eigenvectors.col2 = -eigenvectors.col2;
            }

            SqrtDAinv[row] = FmInitSMatrix3(mul(eigenvectors, mul(FmMatrix3::scale(sqrt(eigenvals)), transpose(eigenvectors))));
        }

        float pInv = FmL1NormInversePreconditioned(SqrtDAinv, solverData);
        float pForward = FmL1NormPreconditioned(SqrtDAinv, solverData);

        float conditionEst = pInv * pForward;

        delete[] SqrtDAinv;

        return conditionEst;
    }

    float FmCheckTetMeshCondition(FmTetMesh* tetMesh, const FmSceneControlParams& sceneControlParams)
    {
        FmSetupMpcgSolve(NULL, tetMesh->solverData, tetMesh,
            sceneControlParams.gravityVector,
            sceneControlParams.kRayleighMassDamping,
            sceneControlParams.kRayleighStiffnessDamping,
            tetMesh->extForceSpeedLimit,
            sceneControlParams.timestep); // Initial est is current velocity

        return FmEstimateConditionNumber(tetMesh->solverData);
    }

    float FmCheckMaxTetMeshCondition(FmTetMeshBuffer* tetMeshBuffer, const FmSceneControlParams& sceneControlParams)
    {
        FmTetMesh& tetMesh = tetMeshBuffer->tetMeshes[0];
        if (tetMesh.tetsToFracture == NULL)
        {
            return FmCheckTetMeshCondition(&tetMesh, sceneControlParams);
        }
        else
        {
            FmTetMesh subsetTetMesh;
            FmAllocTetMeshData(&subsetTetMesh, tetMesh);

            FmMpcgSolverData subsetSolverData;
            FmAllocMpcgSolverData(&subsetSolverData, subsetTetMesh);
            subsetTetMesh.solverData = &subsetSolverData;

            uint numVerts = tetMesh.numVerts;
            uint* remapVertIds = new uint[numVerts];
            for (uint vId = 0; vId < numVerts; vId++)
            {
                remapVertIds[vId] = FM_INVALID_ID;
            }

            // Check each fracture group
            uint numTets = tetMeshBuffer->numTets;

            uint numFractureGroups = tetMeshBuffer->maxTetMeshes;
            FmArray<uint>* fractureGroupTets = new FmArray<uint>[numFractureGroups];
            for (uint i = 0; i < numTets; i++)
            {
                uint fractureGroupId = tetMeshBuffer->tetReferences[i].fractureGroupId;
                fractureGroupTets[fractureGroupId].Add(i);
            }

            float maxCondition = 0.0f;
            for (uint groupId = 0; groupId < numFractureGroups; groupId++)
            {
                uint* subsetTetIds = fractureGroupTets[groupId].GetData();
                uint subsetTetMeshNumTets = fractureGroupTets[groupId].GetNumElems();

                FmCopySubsetTetMesh(&subsetTetMesh, remapVertIds, tetMesh, subsetTetIds, subsetTetMeshNumTets);

                FmSetupMpcgSolve(NULL, subsetTetMesh.solverData, &subsetTetMesh,
                    sceneControlParams.gravityVector,
                    sceneControlParams.kRayleighMassDamping,
                    sceneControlParams.kRayleighStiffnessDamping,
                    subsetTetMesh.extForceSpeedLimit,
                    sceneControlParams.timestep); // Initial est is current velocity

                float tetCondition = FmEstimateConditionNumber(subsetTetMesh.solverData);

                if (tetCondition > maxCondition)
                {
                    maxCondition = tetCondition;
                }
            }

            delete[] fractureGroupTets;
            delete[] remapVertIds;
            FmFreeTetMeshData(&subsetTetMesh);
            FmFreeMpcgSolverData(&subsetSolverData);

            return maxCondition;
        }
    }
}
