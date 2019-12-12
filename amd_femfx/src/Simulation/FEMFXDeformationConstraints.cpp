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
// Constraints to prevent excessive deformation as described in: Perez et al. 
// "Strain Limiting for Soft Finger Contact Simulation".
//---------------------------------------------------------------------------------------

#include "FEMFXSvd3x3.h"
#include "FEMFXAtomicOps.h"
#include "FEMFXTetMath.h"
#include "FEMFXTetMesh.h"
#include "FEMFXConstraints.h"
#include "FEMFXConstraintsBuffer.h"

namespace AMD
{
    void FmSetDeformationConstraintJacobians(
        FmMatrix3& jacobian0,
        FmMatrix3& jacobian1,
        FmMatrix3& jacobian2,
        FmMatrix3& jacobian3,
        const FmVector3& X0InvRow0,
        const FmVector3& X0InvRow1,
        const FmVector3& X0InvRow2,
        const FmVector3& X0InvRow3,
        const FmMatrix3& U,
        const FmMatrix3& V,
        float signs[3])
    {
        jacobian0.col0 = (dot(X0InvRow0, V.col0) * U.col0) * signs[0];
        jacobian0.col1 = (dot(X0InvRow0, V.col1) * U.col1) * signs[1];
        jacobian0.col2 = (dot(X0InvRow0, V.col2) * U.col2) * signs[2];
        jacobian0 = transpose(jacobian0);

        jacobian1.col0 = (dot(X0InvRow1, V.col0) * U.col0) * signs[0];
        jacobian1.col1 = (dot(X0InvRow1, V.col1) * U.col1) * signs[1];
        jacobian1.col2 = (dot(X0InvRow1, V.col2) * U.col2) * signs[2];
        jacobian1 = transpose(jacobian1);

        jacobian2.col0 = (dot(X0InvRow2, V.col0) * U.col0) * signs[0];
        jacobian2.col1 = (dot(X0InvRow2, V.col1) * U.col1) * signs[1];
        jacobian2.col2 = (dot(X0InvRow2, V.col2) * U.col2) * signs[2];
        jacobian2 = transpose(jacobian2);

        jacobian3.col0 = (dot(X0InvRow3, V.col0) * U.col0) * signs[0];
        jacobian3.col1 = (dot(X0InvRow3, V.col1) * U.col1) * signs[1];
        jacobian3.col2 = (dot(X0InvRow3, V.col2) * U.col2) * signs[2];
        jacobian3 = transpose(jacobian3);
    }

    // Add constraints to prevent excessive deformation.
    // Expects that unconstrained end-of-step velocities have been updated.
    // There are up to three constraints per tetrahedron, and using the same 3D constraints and 3x3 Jacobian submatrices as for contacts and glue.
    // This allows some benefit from SIMD but wastes space when fewer dimensions are needed.
    // Reference: Perez et al., "Strain Limiting for Soft Finger Contact Simulation"
    void FmAddDeformationConstraints(FmTetMesh* tetMesh, FmConstraintsBuffer* constraintsBuffer, float timestep)
    {
        (void)timestep;
        uint numTets = tetMesh->numTets;

        uint idWithinObject = 0;

        for (uint tetId = 0; tetId < numTets; tetId++)
        {
            const FmTetDeformationMaterialParams& materialParams = tetMesh->tetsDeformationMaterialParams[tetId];

            float lowerDeformationLimit = materialParams.lowerDeformationLimit;
            float upperDeformationLimit = materialParams.upperDeformationLimit;
            FM_ASSERT(lowerDeformationLimit == 0.0f || lowerDeformationLimit < 1.0f);
            FM_ASSERT(upperDeformationLimit == 0.0f || upperDeformationLimit > 1.0f);

            // Check deformation if lower or upper limit set
            if (lowerDeformationLimit != 0.0f || upperDeformationLimit != 0.0f)
            {
                FmTetVertIds tetVertIds = tetMesh->tetsVertIds[tetId];
                uint vId0 = tetVertIds.ids[0];
                uint vId1 = tetVertIds.ids[1];
                uint vId2 = tetVertIds.ids[2];
                uint vId3 = tetVertIds.ids[3];

                FmVector3 tetRestPosition0 = tetMesh->vertsRestPos[vId0];
                FmVector3 tetRestPosition1 = tetMesh->vertsRestPos[vId1];
                FmVector3 tetRestPosition2 = tetMesh->vertsRestPos[vId2];
                FmVector3 tetRestPosition3 = tetMesh->vertsRestPos[vId3];

                FmVector3 position0 = tetMesh->vertsPos[vId0];
                FmVector3 position1 = tetMesh->vertsPos[vId1];
                FmVector3 position2 = tetMesh->vertsPos[vId2];
                FmVector3 position3 = tetMesh->vertsPos[vId3];

                // Set which vertices in constraint are dynamic
                uint8_t dynamicFlags = 0;
                dynamicFlags |= (tetMesh->vertsFlags[vId0] & FM_VERT_FLAG_KINEMATIC) ? 0 : 0x1;
                dynamicFlags |= (tetMesh->vertsFlags[vId1] & FM_VERT_FLAG_KINEMATIC) ? 0 : 0x2;
                dynamicFlags |= (tetMesh->vertsFlags[vId2] & FM_VERT_FLAG_KINEMATIC) ? 0 : 0x4;
                dynamicFlags |= (tetMesh->vertsFlags[vId3] & FM_VERT_FLAG_KINEMATIC) ? 0 : 0x8;

                if (dynamicFlags == 0)
                {
                    continue;
                }

                // Deformation is measured as singular values of deformation gradient.
                // Singular value = 1.0 => no deformation (and product of singular values = 1.0 => volume preserved).
                FmMatrix3 volumeMatrix = FmComputeTetVolumeMatrix(position0, position1, position2, position3);
                FmMatrix3 restVolumeMatrixInv = inverse(FmComputeTetVolumeMatrix(tetRestPosition0, tetRestPosition1, tetRestPosition2, tetRestPosition3));

                FmMatrix3 deformationGradient = mul(volumeMatrix, restVolumeMatrixInv);

                FmMatrix3 U, V;
                FmVector3 sigma;
                FmSvd3x3(&U, &sigma, &V, deformationGradient);

                // Avoid issues when near inverted
                if (sigma.x < 0.01f || sigma.y < 0.01f || sigma.z < 0.01f)
                {
                    continue;
                }

                float deformationVals[3];
                deformationVals[0] = sigma.x;
                deformationVals[1] = sigma.y;
                deformationVals[2] = sigma.z;

                float deformationLimits[3];
                deformationLimits[0] = 0.0f;
                deformationLimits[1] = 0.0f;
                deformationLimits[2] = 0.0f;

                float signs[3];
                signs[0] = 0.0f;
                signs[1] = 0.0f;
                signs[2] = 0.0f;

                // Check if deformation is outside lower or upper limits.
                // Set a sign value per dimension so the constraint solver can always assume a positive constraint multiplier.
                // sign = 0 means no constraint in that dimension.

                // Set thresholds to add constraint a little before the limits, to improve temporal stability
                float lowerDeformationThreshold = 1.0f - (1.0f - lowerDeformationLimit) * 0.8f;
                float upperDeformationThreshold = 1.0f + (upperDeformationLimit - 1.0f) * 0.8f;

                bool createConstraint = false;
                for (uint dim = 0; dim < 3; dim++)
                {
                    float deformation = deformationVals[dim];
                    if (lowerDeformationLimit != 0.0f && deformation < lowerDeformationThreshold)
                    {
                        createConstraint = true;
                        deformationLimits[dim] = lowerDeformationLimit;
                        signs[dim] = 1.0f;
                    }
                    else if (upperDeformationLimit != 0.0f && deformation > upperDeformationThreshold)
                    {
                        createConstraint = true;
                        deformationLimits[dim] = upperDeformationLimit;
                        signs[dim] = -1.0f;
                    }
                }

                if (createConstraint &&
                    FmAtomicRead(&constraintsBuffer->numDeformationConstraints.val) < constraintsBuffer->maxDeformationConstraints)
                {
                    // Claim contact atomically to support multithreading
                    uint constraintIdx = FmAtomicIncrement(&constraintsBuffer->numDeformationConstraints.val) - 1;

                    if (constraintIdx < constraintsBuffer->maxDeformationConstraints)
                    {
                        FmDeformationConstraint deformationConstraint;

                        deformationConstraint.idInObject = idWithinObject;
                        idWithinObject++;

                        uint8_t movingFlags = 0;
                        movingFlags |= FmIsZero(tetMesh->vertsVel[vId0]) ? 0 : 0x1;
                        movingFlags |= FmIsZero(tetMesh->vertsVel[vId1]) ? 0 : 0x2;
                        movingFlags |= FmIsZero(tetMesh->vertsVel[vId2]) ? 0 : 0x4;
                        movingFlags |= FmIsZero(tetMesh->vertsVel[vId3]) ? 0 : 0x8;

                        // Compute jacobian submatrices corresponding to each of the tet vertices
                        FmVector3 X0InvRow0 = restVolumeMatrixInv.getRow(0);
                        FmVector3 X0InvRow1 = restVolumeMatrixInv.getRow(1);
                        FmVector3 X0InvRow2 = restVolumeMatrixInv.getRow(2);
                        FmVector3 X0InvRow3 = -X0InvRow0 - X0InvRow1 - X0InvRow2;

                        FmMatrix3 jacobian0, jacobian1, jacobian2, jacobian3;
                        FmSetDeformationConstraintJacobians(jacobian0, jacobian1, jacobian2, jacobian3, X0InvRow0, X0InvRow1, X0InvRow2, X0InvRow3, U, V, signs);

                        deformationConstraint.objectId = tetMesh->objectId;
                        deformationConstraint.tetId = tetId;
                        deformationConstraint.deformationDeltas.x = (deformationVals[0] - deformationLimits[0]) * signs[0];
                        deformationConstraint.deformationDeltas.y = (deformationVals[1] - deformationLimits[1]) * signs[1];
                        deformationConstraint.deformationDeltas.z = (deformationVals[2] - deformationLimits[2]) * signs[2];

                        deformationConstraint.jacobian0 = jacobian0;
                        deformationConstraint.jacobian1 = jacobian1;
                        deformationConstraint.jacobian2 = jacobian2;
                        deformationConstraint.jacobian3 = jacobian3;
                        deformationConstraint.dynamicFlags = dynamicFlags;
                        deformationConstraint.movingFlags = movingFlags;

                        constraintsBuffer->deformationConstraints[constraintIdx] = deformationConstraint;
                    }
                    else
                    {
                        FmAtomicWrite(&constraintsBuffer->numDeformationConstraints.val, constraintsBuffer->maxDeformationConstraints);
                    }
                }
            }
        }
    }
}