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
// Functions that setup MPCG Solve for Implicit Euler integration of an FEM mesh
//---------------------------------------------------------------------------------------

#include "AMD_FEMFX.h"
#include "FEMFXMpcgSolverSetup.h"
#include "FEMFXTetMesh.h"
#if FM_SOA_TET_MATH
#include "FEMFXSoaTetMath.h"
#endif

namespace AMD
{
    void FmResizeMpcgMatrix(FmMpcgSolverData* data, const FmTetMesh& tetMesh)
    {
        uint numVerts = tetMesh.numVerts;
        FM_ASSERT(data->maxVertAdjacentVerts >= tetMesh.vertConnectivity.numAdjacentVerts);
        uint submatIdx = numVerts;
        for (uint vId = 0; vId < numVerts; vId++)
        {
            uint numAdjacentVerts = (tetMesh.vertsFlags[vId] & FM_VERT_FLAG_KINEMATIC) ? 0 : tetMesh.vertsNeighbors[vId].numAdjacentVerts;

            data->A.rowStarts[vId] = submatIdx;

            submatIdx += numAdjacentVerts;
        }
        FM_ASSERT(submatIdx <= data->maxVertAdjacentVerts + numVerts);
        data->A.rowStarts[numVerts] = submatIdx;
        data->A.numRows = numVerts;
    }

#if FM_MATRIX_ASSEMBLY_BY_TETS
    void FmAddRowSubmatrices(
        FmSMatrix3* diagSubmats,
        uint rowIdx,
        FmSMatrix3* rowSubmats,
        uint* rowIndices,
        uint rowSize,
        const FmTetVertIds& tetVertIds,
        const uint stiffnessRowOffsets[4],
        const FmSMatrix3& mat0, const FmSMatrix3& mat1, const FmSMatrix3& mat2, const FmSMatrix3& mat3)
    {
        (void)rowSize;
        uint vId0 = tetVertIds.ids[0];
        uint vId1 = tetVertIds.ids[1];
        uint vId2 = tetVertIds.ids[2];
        uint vId3 = tetVertIds.ids[3];

        if (vId0 == rowIdx)
        {
            diagSubmats[vId0] += mat0;
        }
        else
        {
            uint offset0 = stiffnessRowOffsets[0];
            FM_ASSERT(offset0 < rowSize);
            FM_ASSERT(rowIndices[offset0] == FM_INVALID_ID || rowIndices[offset0] == vId0);
            if (rowIndices[offset0] == FM_INVALID_ID)
            {
                rowSubmats[offset0] = mat0;
                rowIndices[offset0] = vId0;
            }
            else
            {
                rowSubmats[offset0] += mat0;
            }
        }

        if (vId1 == rowIdx)
        {
            diagSubmats[vId1] += mat1;
        }
        else
        {
            uint offset1 = stiffnessRowOffsets[1];
            FM_ASSERT(offset1 < rowSize);
            FM_ASSERT(rowIndices[offset1] == FM_INVALID_ID || rowIndices[offset1] == vId1);
            if (rowIndices[offset1] == FM_INVALID_ID)
            {
                rowSubmats[offset1] = mat1;
                rowIndices[offset1] = vId1;
            }
            else
            {
                rowSubmats[offset1] += mat1;
            }
        }

        if (vId2 == rowIdx)
        {
            diagSubmats[vId2] += mat2;
        }
        else
        {
            uint offset2 = stiffnessRowOffsets[2];
            FM_ASSERT(offset2 < rowSize);
            FM_ASSERT(rowIndices[offset2] == FM_INVALID_ID || rowIndices[offset2] == vId2);
            if (rowIndices[offset2] == FM_INVALID_ID)
            {
                rowSubmats[offset2] = mat2;
                rowIndices[offset2] = vId2;
            }
            else
            {
                rowSubmats[offset2] += mat2;
            }
        }

        if (vId3 == rowIdx)
        {
            diagSubmats[vId3] += mat3;
        }
        else
        {
            uint offset3 = stiffnessRowOffsets[3];
            FM_ASSERT(offset3 < rowSize);
            FM_ASSERT(rowIndices[offset3] == FM_INVALID_ID || rowIndices[offset3] == vId3);
            if (rowIndices[offset3] == FM_INVALID_ID)
            {
                rowSubmats[offset3] = mat3;
                rowIndices[offset3] = vId3;
            }
            else
            {
                rowSubmats[offset3] += mat3;
            }
        }
    }

#if FM_SOA_TET_MATH
    template<class SoaTypes>
    struct FmComputeStiffnessBatch
    {
        uint baseTetIdx;
        uint numTets; // number of tets batched

        // Inputs
        FmSoaTetShapeParams<SoaTypes> stressShapeParams;   // Shape params based on plastic deformed rest positions, for computing elastic stress
        typename SoaTypes::SoaMatrix3 stressTetRotation;
        typename SoaTypes::SoaFloat youngsModulus;
        typename SoaTypes::SoaFloat poissonsRatio;
        typename SoaTypes::SoaBool meshSupportsPlasticity;

        // Output
        FmSoaTetRotatedStiffnessMatrix<SoaTypes> rotatedStiffnessMat;

        FM_ALIGN(16) FmVector4 aosStressShapeParamsBaryMatrixCol0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector4 aosStressShapeParamsBaryMatrixCol1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector4 aosStressShapeParamsBaryMatrixCol2[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector4 aosStressShapeParamsBaryMatrixCol3[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosStressTetRotationCol0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosStressTetRotationCol1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosStressTetRotationCol2[SoaTypes::width] FM_ALIGN_END(16);

        FmComputeStiffnessBatch()
        {
            baseTetIdx = 0;
            numTets = 0;

            for (uint sliceIdx = 0; sliceIdx < SoaTypes::width; sliceIdx++)
            {
                aosStressShapeParamsBaryMatrixCol0[sliceIdx] = FmInitVector4(0.0f);
                aosStressShapeParamsBaryMatrixCol1[sliceIdx] = FmInitVector4(0.0f);
                aosStressShapeParamsBaryMatrixCol2[sliceIdx] = FmInitVector4(0.0f);
                aosStressShapeParamsBaryMatrixCol3[sliceIdx] = FmInitVector4(0.0f);
                aosStressTetRotationCol0[sliceIdx] = FmInitVector3(0.0f);
                aosStressTetRotationCol1[sliceIdx] = FmInitVector3(0.0f);
                aosStressTetRotationCol2[sliceIdx] = FmInitVector3(0.0f);
            }

            stressShapeParams.det = SoaTypes::SoaFloat(0.0f);
            youngsModulus = SoaTypes::SoaFloat(0.0f);
            poissonsRatio = SoaTypes::SoaFloat(0.0f);
            meshSupportsPlasticity = SoaTypes::SoaBool(false);
        }

        void SetBatchSlice(
            uint idx,
            const FmTetShapeParams& inStressShapeParams,
            const FmMatrix3& inStressTetRotation,
            float inYoungsModulus,
            float inPoissonsRatio,
            bool inMeshSupportsPlasticity)
        {
            aosStressShapeParamsBaryMatrixCol0[idx] = inStressShapeParams.baryMatrix.col0;
            aosStressShapeParamsBaryMatrixCol1[idx] = inStressShapeParams.baryMatrix.col1;
            aosStressShapeParamsBaryMatrixCol2[idx] = inStressShapeParams.baryMatrix.col2;
            aosStressShapeParamsBaryMatrixCol3[idx] = inStressShapeParams.baryMatrix.col3;
            aosStressTetRotationCol0[idx] = inStressTetRotation.col0;
            aosStressTetRotationCol1[idx] = inStressTetRotation.col1;
            aosStressTetRotationCol2[idx] = inStressTetRotation.col2;

            stressShapeParams.det.setSlice(idx, inStressShapeParams.det);

            youngsModulus.setSlice(idx, inYoungsModulus);
            poissonsRatio.setSlice(idx, inPoissonsRatio);

            meshSupportsPlasticity.setSlice(idx, inMeshSupportsPlasticity);
        }

        void GetBatchSlice(
            FmTetRotatedStiffnessMatrix* outRotatedStiffnessMat,
            uint idx)
        {
            FmGetSlice<SoaTypes>(outRotatedStiffnessMat, rotatedStiffnessMat, idx);
        }

        void ConvertAosToSoa()
        {
            FmSetSoaFromAlignedAos(&stressShapeParams.baryMatrix.col0, aosStressShapeParamsBaryMatrixCol0);
            FmSetSoaFromAlignedAos(&stressShapeParams.baryMatrix.col1, aosStressShapeParamsBaryMatrixCol1);
            FmSetSoaFromAlignedAos(&stressShapeParams.baryMatrix.col2, aosStressShapeParamsBaryMatrixCol2);
            FmSetSoaFromAlignedAos(&stressShapeParams.baryMatrix.col3, aosStressShapeParamsBaryMatrixCol3);

            FmSetSoaFromAlignedAos(&stressTetRotation.col0, aosStressTetRotationCol0);
            FmSetSoaFromAlignedAos(&stressTetRotation.col1, aosStressTetRotationCol1);
            FmSetSoaFromAlignedAos(&stressTetRotation.col2, aosStressTetRotationCol2);
        }

        void Compute()
        {
            FmSoaTetStressMatrix<SoaTypes> stressMat;
            FmSoaTetStiffnessMatrix<SoaTypes> stiffnessMat;
            FmComputeTetStressAndStiffnessMatrix(&stressMat, &stiffnessMat,
                stressShapeParams.baryMatrix.col0,
                stressShapeParams.baryMatrix.col1,
                stressShapeParams.baryMatrix.col2,
                stressShapeParams.GetVolume(),
                youngsModulus, poissonsRatio);

            typename SoaTypes::SoaVector3 stressRestPosition0;
            typename SoaTypes::SoaVector3 stressRestPosition1;
            typename SoaTypes::SoaVector3 stressRestPosition2;
            typename SoaTypes::SoaVector3 stressRestPosition3;

            // Recompute rest positions from the shape params so they reflect plastic deformation.
            // Safe to recompute for non-plastic meshes also.
            stressShapeParams.ComputeRestPositions(&stressRestPosition0, &stressRestPosition1, &stressRestPosition2, &stressRestPosition3);

            FmComputeRotatedStiffnessMatrix<SoaTypes>(&rotatedStiffnessMat, stiffnessMat,
                stressRestPosition0,
                stressRestPosition1,
                stressRestPosition2,
                stressRestPosition3,
                stressTetRotation);
        }
    };
#endif

    void FmApplyTetContributions(
        FmMpcgSolverData* data,
        FmTetMesh* tetMesh,
        const FmTetRotatedStiffnessMatrix& stiffness,
        const uint stiffnessMatRowOffsets[4][4],
        const FmSVector3& centerOfMass,
        FmTetVertIds tetVertIds,
        float dt2rayleigh,
        float deltat)
    {
        uint vId0 = tetVertIds.ids[0];
        uint vId1 = tetVertIds.ids[1];
        uint vId2 = tetVertIds.ids[2];
        uint vId3 = tetVertIds.ids[3];

        // Recenter positions to center of mass for possible numerical benefit.
        FmSVector3 xtemp[4];
#if FM_SOLVE_DELTAV
        xtemp[0] = FmInitSVector3(tetMesh->vertsPos[vId0]) - centerOfMass + FmInitSVector3(tetMesh->vertsVel[vId0])*deltat;
        xtemp[1] = FmInitSVector3(tetMesh->vertsPos[vId1]) - centerOfMass + FmInitSVector3(tetMesh->vertsVel[vId1])*deltat;
        xtemp[2] = FmInitSVector3(tetMesh->vertsPos[vId2]) - centerOfMass + FmInitSVector3(tetMesh->vertsVel[vId2])*deltat;
        xtemp[3] = FmInitSVector3(tetMesh->vertsPos[vId3]) - centerOfMass + FmInitSVector3(tetMesh->vertsVel[vId3])*deltat;
#else
        xtemp[0] = FmInitSVector3(tetMesh->vertsPos[vId0]) - centerOfMass;
        xtemp[1] = FmInitSVector3(tetMesh->vertsPos[vId1]) - centerOfMass;
        xtemp[2] = FmInitSVector3(tetMesh->vertsPos[vId2]) - centerOfMass;
        xtemp[3] = FmInitSVector3(tetMesh->vertsPos[vId3]) - centerOfMass;
#endif

        bool isKinematic0 = tetMesh->vertsFlags[vId0] & FM_VERT_FLAG_KINEMATIC;
        bool isKinematic1 = tetMesh->vertsFlags[vId1] & FM_VERT_FLAG_KINEMATIC;
        bool isKinematic2 = tetMesh->vertsFlags[vId2] & FM_VERT_FLAG_KINEMATIC;
        bool isKinematic3 = tetMesh->vertsFlags[vId3] & FM_VERT_FLAG_KINEMATIC;

        // Solver indices (currently same)
        uint svId0 = vId0;
        uint svId1 = vId1;
        uint svId2 = vId2;
        uint svId3 = vId3;

        FmTetVertIds svTetVertIds;
        svTetVertIds.ids[0] = svId0;
        svTetVertIds.ids[1] = svId1;
        svTetVertIds.ids[2] = svId2;
        svTetVertIds.ids[3] = svId3;

        FmSVector3 bterm[4];
        bterm[0] = FmInitSVector3(0.0f);
        bterm[1] = FmInitSVector3(0.0f);
        bterm[2] = FmInitSVector3(0.0f);
        bterm[3] = FmInitSVector3(0.0f);

        FmSMatrix3 Ke0, Ke1, Ke2, Ke3;

        if (!isKinematic0)
        {
            Ke0 = FmInitSMatrix3(stiffness.Keprime_diag0);
            Ke1 = FmInitSMatrix3(transpose(stiffness.Keprime_lower0));
            Ke2 = FmInitSMatrix3(transpose(stiffness.Keprime_lower1));
            Ke3 = FmInitSMatrix3(transpose(stiffness.Keprime_lower2));

            bterm[0] = mul(Ke0, xtemp[0]) + mul(Ke1, xtemp[1]) + mul(Ke2, xtemp[2]) + mul(Ke3, xtemp[3]) + FmInitSVector3(stiffness.f0eprime0);

            uint rowStart = data->A.rowStarts[svId0];
            uint rowSize = data->A.rowStarts[svId0 + 1] - rowStart;

            FmAddRowSubmatrices(
                data->A.submats,
                svId0,
                &data->A.submats[rowStart], &data->A.indices[rowStart], rowSize, svTetVertIds, stiffnessMatRowOffsets[0], Ke0*dt2rayleigh, Ke1*dt2rayleigh, Ke2*dt2rayleigh, Ke3*dt2rayleigh);
        }

        if (!isKinematic1)
        {
            Ke0 = FmInitSMatrix3(stiffness.Keprime_lower0);
            Ke1 = FmInitSMatrix3(stiffness.Keprime_diag1);
            Ke2 = FmInitSMatrix3(transpose(stiffness.Keprime_lower3));
            Ke3 = FmInitSMatrix3(transpose(stiffness.Keprime_lower4));

            bterm[1] = mul(Ke0, xtemp[0]) + mul(Ke1, xtemp[1]) + mul(Ke2, xtemp[2]) + mul(Ke3, xtemp[3]) + FmInitSVector3(stiffness.f0eprime1);

            uint rowStart = data->A.rowStarts[svId1];
            uint rowSize = data->A.rowStarts[svId1 + 1] - rowStart;

            FmAddRowSubmatrices(
                data->A.submats,
                svId1,
                &data->A.submats[rowStart], &data->A.indices[rowStart], rowSize, svTetVertIds, stiffnessMatRowOffsets[1], Ke0*dt2rayleigh, Ke1*dt2rayleigh, Ke2*dt2rayleigh, Ke3*dt2rayleigh);
        }

        if (!isKinematic2)
        {
            Ke0 = FmInitSMatrix3(stiffness.Keprime_lower1);
            Ke1 = FmInitSMatrix3(stiffness.Keprime_lower3);
            Ke2 = FmInitSMatrix3(stiffness.Keprime_diag2);
            Ke3 = FmInitSMatrix3(transpose(stiffness.Keprime_lower5));

            bterm[2] = mul(Ke0, xtemp[0]) + mul(Ke1, xtemp[1]) + mul(Ke2, xtemp[2]) + mul(Ke3, xtemp[3]) + FmInitSVector3(stiffness.f0eprime2);

            uint rowStart = data->A.rowStarts[svId2];
            uint rowSize = data->A.rowStarts[svId2 + 1] - rowStart;

            FmAddRowSubmatrices(
                data->A.submats,
                svId2,
                &data->A.submats[rowStart], &data->A.indices[rowStart], rowSize, svTetVertIds, stiffnessMatRowOffsets[2], Ke0*dt2rayleigh, Ke1*dt2rayleigh, Ke2*dt2rayleigh, Ke3*dt2rayleigh);
        }

        if (!isKinematic3)
        {
            Ke0 = FmInitSMatrix3(stiffness.Keprime_lower2);
            Ke1 = FmInitSMatrix3(stiffness.Keprime_lower4);
            Ke2 = FmInitSMatrix3(stiffness.Keprime_lower5);
            Ke3 = FmInitSMatrix3(stiffness.Keprime_diag3);

            bterm[3] = mul(Ke0, xtemp[0]) + mul(Ke1, xtemp[1]) + mul(Ke2, xtemp[2]) + mul(Ke3, xtemp[3]) + FmInitSVector3(stiffness.f0eprime3);

            uint rowStart = data->A.rowStarts[svId3];
            uint rowSize = data->A.rowStarts[svId3 + 1] - rowStart;

            FmAddRowSubmatrices(
                data->A.submats,
                svId3,
                &data->A.submats[rowStart], &data->A.indices[rowStart], rowSize, svTetVertIds, stiffnessMatRowOffsets[3], Ke0*dt2rayleigh, Ke1*dt2rayleigh, Ke2*dt2rayleigh, Ke3*dt2rayleigh);
        }

        data->b[svId0] -= bterm[0] * deltat;
        data->b[svId1] -= bterm[1] * deltat;
        data->b[svId2] -= bterm[2] * deltat;
        data->b[svId3] -= bterm[3] * deltat;
    }
#endif

    void FmSetupMpcgSolve(
        FmSVector3* solution,         // initial solution estimate for solve
        FmMpcgSolverData* data,
        FmTetMesh* tetMesh,
        const FmVector3& sceneGravityVector,
        float rayleighMassCoeff,
        float rayleighStiffnessCoeff,
        float extForceSpeedLimit,
        float deltat)
    {
        FM_TRACE_SCOPED_EVENT(MPCG_SETUP);

        if (tetMesh->flags & FM_OBJECT_FLAG_NEEDS_ADJACENT_VERT_OFFSETS)
        {
            FmUpdateAdjacentVertOffsets(tetMesh);

            tetMesh->flags &= ~FM_OBJECT_FLAG_NEEDS_ADJACENT_VERT_OFFSETS;
        }

        const FmTetStiffnessState* tetsStiffness = tetMesh->tetsStiffness;
        FmSVector3 centerOfMass = FmInitSVector3(tetMesh->centerOfMass);
        uint numVerts = tetMesh->numVerts;

        FmVector3 gravityVector = sceneGravityVector + tetMesh->gravityVector;
        float dt2 = deltat * deltat;

        data->A.numRows = numVerts;
        data->hasKinematicVerts = false;

        // Resize the sparse matrix rows; handles any change in kinematic flags
        FmResizeMpcgMatrix(data, *tetMesh);

#if FM_MATRIX_ASSEMBLY_BY_TETS
        // add mass and damping into A
        // first submatrix of A rows will be the diagonal
        for (uint vId = 0; vId < numVerts; vId++)
        {
            uint svId = vId;

            bool kinematic = (tetMesh->vertsFlags[vId] & FM_VERT_FLAG_KINEMATIC);

            float mass = tetMesh->vertsMass[vId];
            FmVector3 vel = tetMesh->vertsVel[vId];

            FmVector3 extForce = FmInitVector3(0.0f);
            
            if (FM_IS_SET(tetMesh->vertsFlags[vId], FM_VERT_FLAG_EXT_FORCE_SET))
            {
                extForce = tetMesh->vertsExtForce[vId];

                float speed = length(vel);
                if (speed > extForceSpeedLimit)
                {
                    extForce = FmInitVector3(0.0f);
                }
                else
                {
                    // Project external force to avoid exceeding maximum speed
                    FmVector3 deltaVel = extForce * deltat / mass;
                    FmVector3 newVel = vel + deltaVel;
                    speed = length(newVel);
                    if (speed > extForceSpeedLimit)
                    {
                        newVel *= extForceSpeedLimit / speed;
                        extForce = (newVel - vel) * mass / deltat;
                    }
                }

                FmResetForceOnVert(tetMesh, vId);
            }

            if (solution)
            {
#if FM_SOLVE_DELTAV
                solution[svId] = FmInitSVector3(0.0f);
#else
                solution[svId] = FmInitSVector3(vel);
#endif
            }

            uint rowStart = data->A.rowStarts[svId];

            if (kinematic)
            {
                data->A.submats[svId] = FmSMatrix3::identity();
                data->hasKinematicVerts = true;
            }
            else
            {
                FmSVector3 C_alpha_diag = FmInitSVector3(mass * rayleighMassCoeff);
                data->A.submats[svId] = FmSMatrix3::scale(FmInitSVector3(mass) + C_alpha_diag * deltat);
            }
            data->A.indices[svId] = svId;

            uint rowSize = data->A.rowStarts[svId + 1] - rowStart;
            for (uint idx = 0; idx < rowSize; idx++)
            {
                data->A.indices[rowStart + idx] = FM_INVALID_ID;
            }

#if FM_SOLVE_DELTAV
            data->b[svId] = FmInitSVector3(extForce + gravityVector * mass - vel * (mass * rayleighMassCoeff + rayleighStiffnessCoeff)) * deltat;
#else
            data->b[svId] = FmInitSVector3(vel * mass + (extForce + gravityVector * mass) * deltat);
#endif
            data->kinematicFlags[svId] = kinematic;
            data->mass[svId] = mass;
        }

        uint numTets = tetMesh->numTets;
        float dt2rayleigh = dt2 + deltat * rayleighStiffnessCoeff;

        bool meshSupportsPlasticity = (tetMesh->tetsPlasticity != NULL);

#if FM_SOA_TET_MATH
        FmComputeStiffnessBatch<FmSoaTypes> computeStiffnessBatch;
#endif

        // add tet K terms into A
        for (uint tIdx = 0; tIdx < numTets; tIdx++)
        {
            const FmTetShapeParams& tetShapeParams = tetMesh->tetsShapeParams[tIdx];
            const FmTetStressMaterialParams& materialParams = tetMesh->tetsStressMaterialParams[tIdx];
            FmMatrix3 tetRotation = tetMesh->tetsRotation[tIdx];
            FmTetVertIds tetVertIds = tetMesh->tetsVertIds[tIdx];

            FmTetShapeParams stressShapeParams;
            FmMatrix3 stressRotation;

            if (meshSupportsPlasticity)
            {
                const FmTetPlasticityState& plasticityState = tetMesh->tetsPlasticity[tIdx];

                stressShapeParams = plasticityState.plasticShapeParams;

#if FM_COMPUTE_PLASTIC_REL_ROTATION
                stressRotation = mul(plasticityState.plasticTetRelRotation, tetRotation);
#else
                stressRotation = tetRotation;
#endif
            }
            else
            {
                stressShapeParams = tetShapeParams;

                stressRotation = tetRotation;
            }

#if FM_SOA_TET_MATH
            uint batchNumTets = computeStiffnessBatch.numTets;

            if (batchNumTets == FmSoaTypes::width)
            {
                computeStiffnessBatch.ConvertAosToSoa();
                computeStiffnessBatch.Compute();

                for (uint batchIdx = 0; batchIdx < batchNumTets; batchIdx++)
                {
                    uint tetId = computeStiffnessBatch.baseTetIdx + batchIdx;

                    FmTetRotatedStiffnessMatrix stiffness;
                    computeStiffnessBatch.GetBatchSlice(&stiffness, batchIdx);

                    FmApplyTetContributions(
                        data,
                        tetMesh,
                        stiffness,
                        tetsStiffness[tetId].stiffnessMatRowOffsets,
                        centerOfMass,
                        tetMesh->tetsVertIds[tetId],
                        dt2rayleigh,
                        deltat);
                }

                computeStiffnessBatch.baseTetIdx = tIdx;
                computeStiffnessBatch.numTets = 0;
            }

            computeStiffnessBatch.SetBatchSlice(computeStiffnessBatch.numTets,
                stressShapeParams, tetRotation, materialParams.youngsModulus, materialParams.poissonsRatio, meshSupportsPlasticity);
            computeStiffnessBatch.numTets++;
#else
            FmTetStressMatrix stressMat;
            FmTetStiffnessMatrix stiffnessMat;
            FmComputeTetStressAndStiffnessMatrix(&stressMat, &stiffnessMat,
                stressShapeParams.baryMatrix.col0,
                stressShapeParams.baryMatrix.col1,
                stressShapeParams.baryMatrix.col2,
                stressShapeParams.GetVolume(),
                materialParams.youngsModulus, materialParams.poissonsRatio);

            // Recompute rest positions from the shape params so they reflect plastic deformation.
            // Safe to recompute for non-plastic meshes also.
            FmVector3 stressRestPosition0, stressRestPosition1, stressRestPosition2, stressRestPosition3;
            stressShapeParams.ComputeRestPositions(&stressRestPosition0, &stressRestPosition1, &stressRestPosition2, &stressRestPosition3);

            FmTetRotatedStiffnessMatrix stiffness;
            FmComputeRotatedStiffnessMatrix(&stiffness, stiffnessMat,
                stressRestPosition0,
                stressRestPosition1,
                stressRestPosition2,
                stressRestPosition3,
                stressRotation);

            FmApplyTetContributions(
                data,
                tetMesh,
                stiffness,
                tetsStiffness[tIdx].stiffnessMatRowOffsets,
                centerOfMass,
                tetVertIds,
                dt2rayleigh,
                deltat);
#endif
        }

#if FM_SOA_TET_MATH
        uint batchNumTets = computeStiffnessBatch.numTets;

        if (batchNumTets > 0)
        {
            computeStiffnessBatch.ConvertAosToSoa();
            computeStiffnessBatch.Compute();

            for (uint batchIdx = 0; batchIdx < batchNumTets; batchIdx++)
            {
                uint tetId = computeStiffnessBatch.baseTetIdx + batchIdx;

                FmTetRotatedStiffnessMatrix stiffness;
                computeStiffnessBatch.GetBatchSlice(&stiffness, batchIdx);

                FmApplyTetContributions(
                    data,
                    tetMesh,
                    stiffness,
                    tetsStiffness[tetId].stiffnessMatRowOffsets,
                    centerOfMass,
                    tetMesh->tetsVertIds[tetId],
                    dt2rayleigh,
                    deltat);
            }
        }
#endif

        // Initialize preconditioner
        for (uint vId = 0; vId < numVerts; vId++)
        {
            data->PInvDiag[vId] = data->kinematicFlags[vId] ? FmSMatrix3::identity() : inverse(data->A.submats[vId]);
        }
#else
        // Loop over verts to add tet K terms into A.
        const FmVertConnectivity& vertConnectivity = tetMesh->vertConnectivity;
        uint* incidentTetsArray = vertConnectivity.incidentTets;
        for (uint vId = 0; vId < numVerts; vId++)
        {
            const FmVertNeighbors& vertNeighbors = tetMesh->vertsNeighbors[vId];
            float mass = tetMesh->vertsMass[vId];
            FmVector3 vel = tetMesh->vertsVel[vId];

            // Add mass and damping into A
            // First submatrix of A rows will be the diagonal
            bool kinematic = (tetMesh->vertsFlags[vId] & FM_VERT_FLAG_KINEMATIC);

            FmVector3 extForce = FmInitVector3(0.0f);

            if (FM_IS_SET(tetMesh->vertsFlags[vId], FM_VERT_FLAG_EXT_FORCE_SET))
            {
                extForce = tetMesh->vertsExtForce[vId];

                float speed = length(vel);
                if (speed > extForceSpeedLimit)
                {
                    extForce = FmInitVector3(0.0f);
                }
                else
                {
                    // Project external force to avoid exceeding maximum speed
                    FmVector3 deltaVel = extForce * deltat / mass;
                    FmVector3 newVel = vel + deltaVel;
                    speed = length(newVel);
                    if (speed > extForceSpeedLimit)
                    {
                        newVel *= extForceSpeedLimit / speed;
                        extForce = (newVel - vel) * mass / deltat;
                    }
                }

                FmResetForceOnVert(tetMesh, vId);
            }

            if (solution)
            {
#if FM_SOLVE_DELTAV
                solution[vId] = FmInitSVector3(0.0f);
#else
                solution[vId] = FmInitSVector3(vel);
#endif
            }

            uint rowStart = data->A.rowStarts[vId];

            if (kinematic)
            {
                data->A.submats[vId] = FmSMatrix3::identity();
                data->hasKinematicVerts = true;
            }
            else
            {
                FmSVector3 C_alpha_diag = FmInitSVector3(mass * rayleighMassCoeff);
                data->A.submats[vId] = FmSMatrix3::scale(FmInitSVector3(mass) + C_alpha_diag * deltat);
            }
            data->A.indices[vId] = vId;

#if FM_SOLVE_DELTAV
            data->b[vId] = FmInitSVector3(extForce + gravityVector * mass - vel * (mass * rayleighMassCoeff + rayleighStiffnessCoeff)) * deltat;
#else
            data->b[vId] = FmInitSVector3(vel * mass + (extForce + gravityVector * mass) * deltat);
#endif

            data->kinematicFlags[vId] = kinematic;
            data->mass[vId] = mass;

            // For kinematic vertex A and b fully set above
            if (kinematic)
            {
                continue;
            }

            // Assemble A and b by accumulating stiffness terms from incident tets.
            for (uint sIdx = 0; sIdx < vertNeighbors.numAdjacentVerts; sIdx++)
            {
                data->A.submats[rowStart + sIdx] = FmInitSMatrix3(0.0f);
                data->A.indices[rowStart + sIdx] = FM_INVALID_ID;
            }

            FmSVector3 btets = FmInitSVector3(0.0f);

            uint incidentTetsStart = vertNeighbors.incidentTetsStart;
            uint numIncidentTets = vertNeighbors.numIncidentTets;
            for (uint itIdx = incidentTetsStart; itIdx < incidentTetsStart + numIncidentTets; itIdx++)
            {
                uint tetId = incidentTetsArray[itIdx];
                const FmTetStiffnessState& tetStiffnessState = tetsStiffness[tetId];
                const FmTetRotatedStiffnessMatrix& stiffness = tetStiffnessState.rotatedStiffnessMat;
                FmTetVertIds tetVertIds = tetMesh->tetsVertIds[tetId];

                uint vId0 = tetVertIds.ids[0];
                uint vId1 = tetVertIds.ids[1];
                uint vId2 = tetVertIds.ids[2];
                uint vId3 = tetVertIds.ids[3];

                // Recenter positions to center of mass for possible numerical benefit.
                FmSVector3 xtemp[4];
#if FM_SOLVE_DELTAV
                xtemp[0] = FmInitSVector3(tetMesh->vertsPos[vId0]) - centerOfMass + FmInitSVector3(tetMesh->vertsVel[vId0])*deltat;
                xtemp[1] = FmInitSVector3(tetMesh->vertsPos[vId1]) - centerOfMass + FmInitSVector3(tetMesh->vertsVel[vId1])*deltat;
                xtemp[2] = FmInitSVector3(tetMesh->vertsPos[vId2]) - centerOfMass + FmInitSVector3(tetMesh->vertsVel[vId2])*deltat;
                xtemp[3] = FmInitSVector3(tetMesh->vertsPos[vId3]) - centerOfMass + FmInitSVector3(tetMesh->vertsVel[vId3])*deltat;
#else
                xtemp[0] = FmInitSVector3(tetMesh->vertsPos[vId0]) - centerOfMass;
                xtemp[1] = FmInitSVector3(tetMesh->vertsPos[vId1]) - centerOfMass;
                xtemp[2] = FmInitSVector3(tetMesh->vertsPos[vId2]) - centerOfMass;
                xtemp[3] = FmInitSVector3(tetMesh->vertsPos[vId3]) - centerOfMass;
#endif

                FmMatrix3 Ke0, Ke1, Ke2, Ke3;
                FmVector3 f0eprime;
                uint tetVertIdx = 0;

                if (tetVertIds.ids[0] == vId)
                {
                    Ke0 = stiffness.Keprime_diag0;
                    Ke1 = transpose(stiffness.Keprime_lower0);
                    Ke2 = transpose(stiffness.Keprime_lower1);
                    Ke3 = transpose(stiffness.Keprime_lower2);
                    f0eprime = stiffness.f0eprime0;
                    tetVertIdx = 0;
                }
                else if (tetVertIds.ids[1] == vId)
                {
                    Ke0 = stiffness.Keprime_lower0;
                    Ke1 = stiffness.Keprime_diag1;
                    Ke2 = transpose(stiffness.Keprime_lower3);
                    Ke3 = transpose(stiffness.Keprime_lower4);
                    f0eprime = stiffness.f0eprime1;
                    tetVertIdx = 1;
                }
                else if (tetVertIds.ids[2] == vId)
                {
                    Ke0 = stiffness.Keprime_lower1;
                    Ke1 = stiffness.Keprime_lower3;
                    Ke2 = stiffness.Keprime_diag2;
                    Ke3 = transpose(stiffness.Keprime_lower5);
                    f0eprime = stiffness.f0eprime2;
                    tetVertIdx = 2;
                }
                else //if (tetVertIds.ids[3] == vId)
                {
                    Ke0 = stiffness.Keprime_lower2;
                    Ke1 = stiffness.Keprime_lower4;
                    Ke2 = stiffness.Keprime_lower5;
                    Ke3 = stiffness.Keprime_diag3;
                    f0eprime = stiffness.f0eprime3;
                    tetVertIdx = 3;
                }

                uint rowSubmatIdx[4];
                rowSubmatIdx[0] = tetStiffnessState.stiffnessMatRowOffsets[tetVertIdx][0];
                rowSubmatIdx[1] = tetStiffnessState.stiffnessMatRowOffsets[tetVertIdx][1];
                rowSubmatIdx[2] = tetStiffnessState.stiffnessMatRowOffsets[tetVertIdx][2];
                rowSubmatIdx[3] = tetStiffnessState.stiffnessMatRowOffsets[tetVertIdx][3];

                FmSMatrix3 SKe0, SKe1, SKe2, SKe3;
                SKe0 = FmInitSMatrix3(Ke0);
                SKe1 = FmInitSMatrix3(Ke1);
                SKe2 = FmInitSMatrix3(Ke2);
                SKe3 = FmInitSMatrix3(Ke3);

                FmSVector3 btemp = mul(SKe0, xtemp[0]) + mul(SKe1, xtemp[1]) + mul(SKe2, xtemp[2]) + mul(SKe3, xtemp[3]) + FmInitSVector3(f0eprime);

                uint submatIdx0 = (tetVertIdx == 0) ? vId : rowStart + rowSubmatIdx[0];
                FmSMatrix3 term0 = SKe0 * (dt2 + deltat * rayleighStiffnessCoeff);
                FmSMatrix3& submat0 = data->A.submats[submatIdx0];
                uint& idx0 = data->A.indices[submatIdx0];
                FM_ASSERT(idx0 == FM_INVALID_ID || idx0 == tetVertIds.ids[0]);
                submat0 += term0;
                idx0 = tetVertIds.ids[0];

                uint submatIdx1 = (tetVertIdx == 1) ? vId : rowStart + rowSubmatIdx[1];
                FmSMatrix3 term1 = SKe1 * (dt2 + deltat * rayleighStiffnessCoeff);
                FmSMatrix3& submat1 = data->A.submats[submatIdx1];
                uint& idx1 = data->A.indices[submatIdx1];
                FM_ASSERT(idx1 == FM_INVALID_ID || idx1 == tetVertIds.ids[1]);
                submat1 += term1;
                idx1 = tetVertIds.ids[1];

                uint submatIdx2 = (tetVertIdx == 2) ? vId : rowStart + rowSubmatIdx[2];
                FmSMatrix3 term2 = SKe2 * (dt2 + deltat * rayleighStiffnessCoeff);
                FmSMatrix3& submat2 = data->A.submats[submatIdx2];
                uint& idx2 = data->A.indices[submatIdx2];
                FM_ASSERT(idx2 == FM_INVALID_ID || idx2 == tetVertIds.ids[2]);
                submat2 += term2;
                idx2 = tetVertIds.ids[2];

                uint submatIdx3 = (tetVertIdx == 3) ? vId : rowStart + rowSubmatIdx[3];
                FmSMatrix3 term3 = SKe3 * (dt2 + deltat * rayleighStiffnessCoeff);
                FmSMatrix3& submat3 = data->A.submats[submatIdx3];
                uint& idx3 = data->A.indices[submatIdx3];
                FM_ASSERT(idx3 == FM_INVALID_ID || idx3 == tetVertIds.ids[3]);
                submat3 += term3;
                idx3 = tetVertIds.ids[3];

                btets += btemp;
            }

            data->b[vId] -= btets * deltat;

            data->PInvDiag[vId] = kinematic ? FmSMatrix3::identity() : inverse(data->A.submats[vId]);
        }
#endif

        data->norms.Zero();
    }
}