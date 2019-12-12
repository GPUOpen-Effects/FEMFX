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
// Functions that setup the constraint solve for FEMFX objects and constraints
//---------------------------------------------------------------------------------------

#include "FEMFXParallelFor.h"
#include "FEMFXConstraintSolveTaskGraph.h"
#include "FEMFXConstraintSolver.h"
#include "FEMFXConstraintIslands.h"
#include "FEMFXScene.h"
#include "FEMFXMpcgSolver.h"

// In contact Jacobian, compute a different tangent per vertex of the volume contact.
// Trying this to improve friction response for volume contacts.
#define FM_VOLCONTACT_TANGENT_PER_VERTEX 1

#define FM_INIT_RIGID_BODY_ARRAYS_BATCH_SIZE 128
#define FM_INIT_CONSTRAINTS_BATCH_SIZE 128

namespace AMD
{
    // Size of jacobian submatrix data; does not include size of indices which can be computed from numstates.
    static FM_FORCE_INLINE uint FmGetJacobianRowSize(
        uint* outputJacobianNumStates,
        uoffset* outputJacobianSubmatOffsets,
        uoffset* outputJacobianIndicesOffsets,
        uoffset submatsOffset, uoffset indicesOffset,
        const FmDistanceContactPairInfo& contact)
    {
        uint numSubmats = FmGetNumJacobianSubmats(contact);
        *outputJacobianNumStates = numSubmats;
        *outputJacobianSubmatOffsets = submatsOffset;
        *outputJacobianIndicesOffsets = indicesOffset;
        return numSubmats * sizeof(FmSMatrix3);
    }

    // Size of jacobian submatrix data; does not include size of indices which can be computed from numstates.
    static FM_FORCE_INLINE uint FmGetJacobianRowSize(
        uint* outputJacobianNumStates,
        uoffset* outputJacobianSubmatOffsets,
        uoffset* outputJacobianIndicesOffsets,
        uoffset submatsOffset, uoffset indicesOffset,
        const FmVolumeContact& contact)
    {
        uint numSubmats = FmGetNumJacobianSubmats(contact);
        *outputJacobianNumStates = numSubmats;
        *outputJacobianSubmatOffsets = submatsOffset;
        *outputJacobianIndicesOffsets = indicesOffset;
        return numSubmats * sizeof(FmSMatrix3);
    }

    // Size of jacobian submatrix data; does not include size of indices which can be computed from numstates.
    static FM_FORCE_INLINE uint FmGetJacobianRowSize(
        uint* outputJacobianNumStates,
        uoffset* outputJacobianSubmatOffsets,
        uoffset* outputJacobianIndicesOffsets,
        uoffset submatsOffset, uoffset indicesOffset,
        const FmGlueConstraint& glueConstraint)
    {
        uint numSubmats = FmGetNumJacobianSubmats(glueConstraint);
        *outputJacobianNumStates = numSubmats;
        *outputJacobianSubmatOffsets = submatsOffset;
        *outputJacobianIndicesOffsets = indicesOffset;
        return numSubmats * sizeof(FmSMatrix3);
    }

    // Size of jacobian submatrix data; does not include size of indices which can be computed from numstates.
    static FM_FORCE_INLINE uint FmGetJacobianRowSize(
        uint* outputJacobianNumStates,
        uoffset* outputJacobianSubmatOffsets,
        uoffset* outputJacobianIndicesOffsets,
        uoffset submatsOffset, uoffset indicesOffset,
        const FmPlaneConstraint& planeConstraint)
    {
        uint numSubmats = FmGetNumJacobianSubmats(planeConstraint);
        *outputJacobianNumStates = numSubmats;
        *outputJacobianSubmatOffsets = submatsOffset;
        *outputJacobianIndicesOffsets = indicesOffset;
        return numSubmats * sizeof(FmSMatrix3);
    }

    // Size of jacobian submatrix data; does not include size of indices which can be computed from numstates.
    static FM_FORCE_INLINE uint FmGetJacobianRowSize(
        uint* outputJacobianNumStates,
        uoffset* outputJacobianSubmatOffsets,
        uoffset* outputJacobianIndicesOffsets,
        uoffset submatsOffset, uoffset indicesOffset,
        const FmDeformationConstraint& deformationConstraint)
    {
        uint numSubmats = FmGetNumJacobianSubmats(deformationConstraint);
        *outputJacobianNumStates = numSubmats;
        *outputJacobianSubmatOffsets = submatsOffset;
        *outputJacobianIndicesOffsets = indicesOffset;
        return numSubmats * sizeof(FmSMatrix3);
    }

    // Size of jacobian submatrix data; does not include size of indices which can be computed from numstates.
    static FM_FORCE_INLINE uint FmGetJacobianRowSize(
        uint* outputJacobianNumStates,
        uoffset* outputJacobianSubmatOffsets,
        uoffset* outputJacobianIndicesOffsets,
        uoffset submatsOffset, uoffset indicesOffset,
        const FmRigidBodyAngleConstraint& rigidBodyAngleConstraint)
    {
        uint numSubmats = FmGetNumJacobianSubmats(rigidBodyAngleConstraint);
        *outputJacobianNumStates = numSubmats;
        *outputJacobianSubmatOffsets = submatsOffset;
        *outputJacobianIndicesOffsets = indicesOffset;
        return numSubmats * sizeof(FmSMatrix3);
    }

    static FM_FORCE_INLINE uint FmGetJacobianIndicesSize(const uint inputJacobianNumStates)
    {
        return (inputJacobianNumStates) * sizeof(uint);
    }

    static inline void FmInitConstraintParams(FmConstraintParams* params)
    {
        params->type = FM_SOLVER_CONSTRAINT_TYPE_3D_NORMAL2DFRICTION;
        params->flags = 0;
        params->frictionCoeff = 0.0f;
        params->diagInverse[0] = FmInitSVector3(0.0f);
        params->diagInverse[1] = FmInitSVector3(0.0f);
        params->jacobiansNumStates = 0;
        params->jacobianSubmatsOffset = 0;
        params->jacobianIndicesOffset = 0;
    }

    void FmResizeConstraintJacobian(
        FmConstraintSolverData* data,
        FmConstraintsBuffer* constraintsBuffer,
        FmConstraintIsland& constraintIsland)
    {
        uint numConstraints = constraintIsland.numConstraints;
        if (numConstraints == 0)
            return;

        uoffset submatsOffset = 0;
        uoffset indicesOffset = 0;

        for (uint constraintIdx = 0; constraintIdx < numConstraints; constraintIdx++)
        {
            FmConstraintReference& constraintRef = constraintIsland.constraintRefs[constraintIdx];

            FmConstraintParams& constraintParams = data->J.params[constraintIdx];
            FmInitConstraintParams(&constraintParams);

            if (constraintRef.type == FM_CONSTRAINT_TYPE_DISTANCE_CONTACT)
            {
                FmDistanceContactPairInfo& contact = constraintsBuffer->distanceContactsPairInfo[constraintRef.idx];

                uint rowSize = FmGetJacobianRowSize(
                    &constraintParams.jacobiansNumStates,
                    &constraintParams.jacobianSubmatsOffset,
                    &constraintParams.jacobianIndicesOffset,
                    submatsOffset,
                    indicesOffset,
                    contact);

                submatsOffset += rowSize;
                indicesOffset += FmGetJacobianIndicesSize(constraintParams.jacobiansNumStates);
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_VOLUME_CONTACT)
            {
                FmVolumeContact& contact = constraintsBuffer->volumeContacts[constraintRef.idx];

                uint rowSize = FmGetJacobianRowSize(
                    &constraintParams.jacobiansNumStates,
                    &constraintParams.jacobianSubmatsOffset,
                    &constraintParams.jacobianIndicesOffset,
                    submatsOffset,
                    indicesOffset,
                    contact);

                submatsOffset += rowSize;
                indicesOffset += FmGetJacobianIndicesSize(constraintParams.jacobiansNumStates);
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_DEFORMATION)
            {
                FmDeformationConstraint& deformationConstraint = constraintsBuffer->deformationConstraints[constraintRef.idx];

                uint rowSize = FmGetJacobianRowSize(
                    &constraintParams.jacobiansNumStates,
                    &constraintParams.jacobianSubmatsOffset,
                    &constraintParams.jacobianIndicesOffset,
                    submatsOffset,
                    indicesOffset,
                    deformationConstraint);

                submatsOffset += rowSize;
                indicesOffset += FmGetJacobianIndicesSize(constraintParams.jacobiansNumStates);
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_GLUE)
            {
                FmGlueConstraint& glueConstraint = constraintsBuffer->glueConstraints[constraintRef.idx];

                uint rowSize = FmGetJacobianRowSize(
                    &constraintParams.jacobiansNumStates,
                    &constraintParams.jacobianSubmatsOffset,
                    &constraintParams.jacobianIndicesOffset,
                    submatsOffset,
                    indicesOffset,
                    glueConstraint);

                submatsOffset += rowSize;
                indicesOffset += FmGetJacobianIndicesSize(constraintParams.jacobiansNumStates);
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_PLANE)
            {
                FmPlaneConstraint& planeConstraint = constraintsBuffer->planeConstraints[constraintRef.idx];

                uint rowSize = FmGetJacobianRowSize(
                    &constraintParams.jacobiansNumStates,
                    &constraintParams.jacobianSubmatsOffset,
                    &constraintParams.jacobianIndicesOffset,
                    submatsOffset,
                    indicesOffset,
                    planeConstraint);

                submatsOffset += rowSize;
                indicesOffset += FmGetJacobianIndicesSize(constraintParams.jacobiansNumStates);
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_RIGID_BODY_ANGLE)
            {
                FmRigidBodyAngleConstraint& rigidBodyAngleConstraint = constraintsBuffer->rigidBodyAngleConstraints[constraintRef.idx];

                uint rowSize = FmGetJacobianRowSize(
                    &constraintParams.jacobiansNumStates,
                    &constraintParams.jacobianSubmatsOffset,
                    &constraintParams.jacobianIndicesOffset,
                    submatsOffset,
                    indicesOffset,
                    rigidBodyAngleConstraint);

                submatsOffset += rowSize;
                indicesOffset += FmGetJacobianIndicesSize(constraintParams.jacobiansNumStates);
            }
            else
            {
                FM_ASSERT(0);
            }
        }
    }

    uint FmAddJacobianRowSubmats_RigidBodyA(
        FmSVector3* Jvel_unconstrained,
        FmSMatrix3* submats,
        uint* indices,
        uint stateVecOffsetA,
        const FmRigidBody& rigidBody,
        const FmDistanceContactPairInfo& contactPairInfo,
        const FmDistanceContact& contact)
    {
        bool isDynamic = FM_IS_SET(contactPairInfo.dynamicFlags, 0x3);
        bool isMoving = FM_IS_SET(contact.movingFlags, 0x3);

        if (!isDynamic && !isMoving)
        {
            return 0;
        }

        uint submatOffset = 0;

        FmSVector3 normal = FmInitSVector3(contact.normal);
        FmSVector3 tangent1 = FmInitSVector3(contact.tangent1);
        FmSVector3 tangent2 = FmInitSVector3(cross(contact.normal, contact.tangent1));

        FmSMatrix3 submatrix;
        submatrix.setCol0(normal);
        submatrix.setCol1(tangent1);
        submatrix.setCol2(tangent2);
        submatrix = transpose(submatrix);
        uint idx = stateVecOffsetA;

        if (isDynamic)
        {
            submats[submatOffset] = submatrix;
            indices[submatOffset] = idx;
            submatOffset++;
        }

        FmSVector3 Jvel = mul(submatrix, FmInitSVector3(rigidBody.state.vel));

        FmSVector3 comToPosA = FmInitSVector3(contact.comToPosA[0], contact.comToPosA[1], contact.comToPosA[2]);
        submatrix.setCol0(cross(comToPosA, normal));
        submatrix.setCol1(cross(comToPosA, tangent1));
        submatrix.setCol2(cross(comToPosA, tangent2));
        submatrix = transpose(submatrix);
        idx++;

        if (isDynamic)
        {
            submats[submatOffset] = submatrix;
            indices[submatOffset] = idx;
            submatOffset++;
        }

        Jvel += mul(submatrix, FmInitSVector3(rigidBody.state.angVel));

        *Jvel_unconstrained += Jvel;

        return submatOffset;
    }

    uint FmAddJacobianRowSubmats_RigidBodyB(
        FmSVector3* Jvel_unconstrained,
        FmSMatrix3* submats,
        uint* indices,
        uint stateVecOffsetB,
        const FmRigidBody& rigidBody,
        const FmDistanceContactPairInfo& contactPairInfo,
        const FmDistanceContact& contact)
    {
        bool isDynamic = FM_IS_SET(contactPairInfo.dynamicFlags, 0x30);
        bool isMoving = FM_IS_SET(contact.movingFlags, 0x30);

        if (!isDynamic && !isMoving)
        {
            return 0;
        }

        uint submatOffset = 0;

        FmSVector3 normal = FmInitSVector3(contact.normal);
        FmSVector3 tangent1 = FmInitSVector3(contact.tangent1);
        FmSVector3 tangent2 = FmInitSVector3(cross(contact.normal, contact.tangent1));

        FmSMatrix3 submatrix;
        submatrix.setCol0(-normal);
        submatrix.setCol1(-tangent1);
        submatrix.setCol2(-tangent2);
        submatrix = transpose(submatrix);
        uint idx = stateVecOffsetB;

        if (isDynamic)
        {
            submats[submatOffset] = submatrix;
            indices[submatOffset] = idx;
            submatOffset++;
        }

        FmSVector3 Jvel = mul(submatrix, FmInitSVector3(rigidBody.state.vel));

        FmSVector3 comToPosB = FmInitSVector3(contact.comToPosB[0], contact.comToPosB[1], contact.comToPosB[2]);
        submatrix.setCol0(cross(comToPosB, -normal));
        submatrix.setCol1(cross(comToPosB, -tangent1));
        submatrix.setCol2(cross(comToPosB, -tangent2));
        submatrix = transpose(submatrix);
        idx++;

        if (isDynamic)
        {
            submats[submatOffset] = submatrix;
            indices[submatOffset] = idx;
            submatOffset++;
        }

        Jvel += mul(submatrix, FmInitSVector3(rigidBody.state.angVel));

        *Jvel_unconstrained += Jvel;

        return submatOffset;
    }

    uint FmAddJacobianRowSubmats_TetMeshA(
        FmSVector3* Jvel_unconstrained,
        FmSMatrix3* submats,
        uint* indices,
        const FmMpcgSolverData& meshASolverData,
        const FmTetMesh& tetMesh,
        const FmDistanceContactPairInfo& contactPairInfo,
        const FmDistanceContact& contact)
    {
        uint vertOffsetA = meshASolverData.solverStateOffset;

        FmTetVertIds tetVertIdsA = tetMesh.tetsVertIds[contact.tetIdA];

        uint vId0 = tetVertIdsA.ids[0];
        uint vId1 = tetVertIdsA.ids[1];
        uint vId2 = tetVertIdsA.ids[2];
        uint vId3 = tetVertIdsA.ids[3];

        uint svId0 = vId0;
        uint svId1 = vId1;
        uint svId2 = vId2;
        uint svId3 = vId3;

        float weights0 = contact.posBaryA[0];
        float weights1 = contact.posBaryA[1];
        float weights2 = contact.posBaryA[2];
        float weights3 = contact.posBaryA[3];

        FmSVector3 normal = FmInitSVector3(contact.normal);
        FmSVector3 tangent1 = FmInitSVector3(contact.tangent1);
        FmSVector3 tangent2 = FmInitSVector3(cross(contact.normal, contact.tangent1));

        FmSVector3 Jvel = FmInitSVector3(0.0f);

        uint submatFlags = contactPairInfo.dynamicFlags | contact.movingFlags;

        uint submatOffset = 0;
        FmSMatrix3 submatrix;
        uint idx;
        if (submatFlags & 0x1)
        {
            submatrix.setCol0(normal * weights0);
            submatrix.setCol1(tangent1 * weights0);
            submatrix.setCol2(tangent2 * weights0);
            submatrix = transpose(submatrix);
            idx = vertOffsetA + svId0;

            if (contactPairInfo.dynamicFlags & 0x1)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId0]));
        }
        if (submatFlags & 0x2)
        {
            submatrix.setCol0(normal * weights1);
            submatrix.setCol1(tangent1 * weights1);
            submatrix.setCol2(tangent2 * weights1);
            submatrix = transpose(submatrix);
            idx = vertOffsetA + svId1;

            if (contactPairInfo.dynamicFlags & 0x2)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId1]));
        }
        if (submatFlags & 0x4)
        {
            submatrix.setCol0(normal * weights2);
            submatrix.setCol1(tangent1 * weights2);
            submatrix.setCol2(tangent2 * weights2);
            submatrix = transpose(submatrix);
            idx = vertOffsetA + svId2;

            if (contactPairInfo.dynamicFlags & 0x4)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId2]));
        }
        if (submatFlags & 0x8)
        {
            submatrix.setCol0(normal * weights3);
            submatrix.setCol1(tangent1 * weights3);
            submatrix.setCol2(tangent2 * weights3);
            submatrix = transpose(submatrix);
            idx = vertOffsetA + svId3;

            if (contactPairInfo.dynamicFlags & 0x8)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId3]));
        }

        *Jvel_unconstrained += Jvel;

        return submatOffset;
    }

    uint FmAddJacobianRowSubmats_TetMeshB(
        FmSVector3* Jvel_unconstrained,
        FmSMatrix3* submats,
        uint* indices,
        const FmMpcgSolverData& meshBSolverData,
        const FmTetMesh& tetMesh,
        const FmDistanceContactPairInfo& contactPairInfo,
        const FmDistanceContact& contact)
    {
        uint vertOffsetB = meshBSolverData.solverStateOffset;

        FmTetVertIds tetVertIdsB = tetMesh.tetsVertIds[contact.tetIdB];

        uint vId4 = tetVertIdsB.ids[0];
        uint vId5 = tetVertIdsB.ids[1];
        uint vId6 = tetVertIdsB.ids[2];
        uint vId7 = tetVertIdsB.ids[3];

        uint svId4 = vId4;
        uint svId5 = vId5;
        uint svId6 = vId6;
        uint svId7 = vId7;

        float weights4 = -contact.posBaryB[0];
        float weights5 = -contact.posBaryB[1];
        float weights6 = -contact.posBaryB[2];
        float weights7 = -contact.posBaryB[3];

        FmSVector3 normal = FmInitSVector3(contact.normal);
        FmSVector3 tangent1 = FmInitSVector3(contact.tangent1);
        FmSVector3 tangent2 = FmInitSVector3(cross(contact.normal, contact.tangent1));

        FmSVector3 Jvel = FmInitSVector3(0.0f);

        uint submatFlags = contactPairInfo.dynamicFlags | contact.movingFlags;

        uint submatOffset = 0;
        FmSMatrix3 submatrix;
        uint idx;
        if (submatFlags & 0x10)
        {
            submatrix.setCol0(normal * weights4);
            submatrix.setCol1(tangent1 * weights4);
            submatrix.setCol2(tangent2 * weights4);
            submatrix = transpose(submatrix);
            idx = vertOffsetB + svId4;

            if (contactPairInfo.dynamicFlags & 0x10)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId4]));
        }
        if (submatFlags & 0x20)
        {
            submatrix.setCol0(normal * weights5);
            submatrix.setCol1(tangent1 * weights5);
            submatrix.setCol2(tangent2 * weights5);
            submatrix = transpose(submatrix);
            idx = vertOffsetB + svId5;

            if (contactPairInfo.dynamicFlags & 0x20)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId5]));
        }
        if (submatFlags & 0x40)
        {
            submatrix.setCol0(normal * weights6);
            submatrix.setCol1(tangent1 * weights6);
            submatrix.setCol2(tangent2 * weights6);
            submatrix = transpose(submatrix);
            idx = vertOffsetB + svId6;

            if (contactPairInfo.dynamicFlags & 0x40)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId6]));
        }
        if (submatFlags & 0x80)
        {
            submatrix.setCol0(normal * weights7);
            submatrix.setCol1(tangent1 * weights7);
            submatrix.setCol2(tangent2 * weights7);
            submatrix = transpose(submatrix);
            idx = vertOffsetB + svId7;

            if (contactPairInfo.dynamicFlags & 0x80)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId7]));
        }

        *Jvel_unconstrained += Jvel;

        return submatOffset;
    }

    uint FmAddJacobianRowSubmats_RigidBodyA(
        FmSVector3* Jvel_unconstrained,
        FmSMatrix3* submats,
        uint* indices,
        uint stateVecOffsetA,
        const FmRigidBody& rigidBody,
        const FmVolumeContact& contact,
        const FmVolumeContactVert* volContactVerts)
    {
        FmSVector3 Jvel = FmInitSVector3(0.0f);

        uint submatOffset = 0;
        FmSMatrix3 submatrix;
        uint idx;
        for (uint vId = 0; vId < contact.numVolVertsA; vId++)
        {
            const FmVolumeContactVert& vert = volContactVerts[contact.volVertsStartA + vId];
            FmSVector3 row0, row1, row2;
            FmSVector3 dVdp = FmInitSVector3(vert.dVdp);
            FmSVector3 centerToVert = FmInitSVector3(vert.centerToVert);

            FmSVector3 tangent1 = FmInitSVector3(vert.tangent);
            FmSVector3 tangent2 = FmSafeNormalize(cross(dVdp, tangent1), normalize(FmOrthogonalVector(tangent1)));
#if FM_VOLCONTACT_TANGENT_PER_VERTEX
            float Ak = length(dVdp);
#else
            float Ak = dot(contact.normal, vert.dVdp);
#endif
            row0 = dVdp;
            row1 = tangent1 * Ak;
            row2 = tangent2 * Ak;
            submatrix.setCol0(row0);
            submatrix.setCol1(row1);
            submatrix.setCol2(row2);
            submatrix = transpose(submatrix);
            idx = stateVecOffsetA;

            if (vert.dynamic)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(rigidBody.state.vel));

            submatrix.setCol0(cross(centerToVert, row0));
            submatrix.setCol1(cross(centerToVert, row1));
            submatrix.setCol2(cross(centerToVert, row2));
            submatrix = transpose(submatrix);
            idx++;

            if (vert.dynamic)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(rigidBody.state.angVel));
        }

        *Jvel_unconstrained += Jvel;

        return submatOffset;
    }

    uint FmAddJacobianRowSubmats_RigidBodyB(
        FmSVector3* Jvel_unconstrained,
        FmSMatrix3* submats,
        uint* indices,
        uint stateVecOffsetB,
        const FmRigidBody& rigidBody,
        const FmVolumeContact& contact,
        const FmVolumeContactVert* volContactVerts)
    {
        FmSVector3 Jvel = FmInitSVector3(0.0f);

        uint submatOffset = 0;
        FmSMatrix3 submatrix;
        uint idx;
        for (uint vId = 0; vId < contact.numVolVertsB; vId++)
        {
            const FmVolumeContactVert& vert = volContactVerts[contact.volVertsStartB + vId];
            FmSVector3 row0, row1, row2;
            FmSVector3 dVdp = FmInitSVector3(vert.dVdp);
            FmSVector3 centerToVert = FmInitSVector3(vert.centerToVert);

            FmSVector3 tangent1 = FmInitSVector3(vert.tangent);
            FmSVector3 tangent2 = FmSafeNormalize(cross(dVdp, tangent1), normalize(FmOrthogonalVector(tangent1)));
#if FM_VOLCONTACT_TANGENT_PER_VERTEX
            float Ak = length(vert.dVdp);
#else
            float Ak = dot(contact.normal, vert.dVdp);
#endif
            row0 = dVdp;
            row1 = tangent1 * Ak;
            row2 = tangent2 * Ak;
            submatrix.setCol0(row0);
            submatrix.setCol1(row1);
            submatrix.setCol2(row2);
            submatrix = transpose(submatrix);
            idx = stateVecOffsetB;

            if (vert.dynamic)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(rigidBody.state.vel));

            submatrix.setCol0(cross(centerToVert, row0));
            submatrix.setCol1(cross(centerToVert, row1));
            submatrix.setCol2(cross(centerToVert, row2));
            submatrix = transpose(submatrix);
            idx++;

            if (vert.dynamic)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(rigidBody.state.angVel));
        }

        *Jvel_unconstrained += Jvel;

        return submatOffset;
    }

    uint FmAddJacobianRowSubmats_TetMeshA(
        FmSVector3* Jvel_unconstrained,
        FmSMatrix3* submats,
        uint* indices,
        const FmMpcgSolverData& meshASolverData,
        const FmTetMesh& tetMesh,
        const FmVolumeContact& contact,
        const FmVolumeContactVert* volContactVerts)
    {
        uint vertOffsetA = meshASolverData.solverStateOffset;

        FmSVector3 Jvel = FmInitSVector3(0.0f);

        uint submatOffset = 0;
        FmSMatrix3 submatrix;
        uint idx;
        for (uint vId = 0; vId < contact.numVolVertsA; vId++)
        {
            const FmVolumeContactVert& vert = volContactVerts[contact.volVertsStartA + vId];
            uint vIdA = vert.vertId;
            uint svIdA = vIdA;

            FmSVector3 dVdp = FmInitSVector3(vert.dVdp);

            FmSVector3 tangent1 = FmInitSVector3(vert.tangent);
            FmSVector3 tangent2 = FmSafeNormalize(cross(dVdp, tangent1), normalize(FmOrthogonalVector(tangent1)));
#if FM_VOLCONTACT_TANGENT_PER_VERTEX
            float Ak = length(dVdp);
#else
            float Ak = dot(contact.normal, vert.dVdp);
#endif
            submatrix.setCol0(dVdp);
            submatrix.setCol1(tangent1 * Ak);
            submatrix.setCol2(tangent2 * Ak);
            submatrix = transpose(submatrix);
            idx = vertOffsetA + svIdA;

            if (vert.dynamic)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vIdA]));
        }

        *Jvel_unconstrained += Jvel;

        return submatOffset;
    }

    uint FmAddJacobianRowSubmats_TetMeshB(
        FmSVector3* Jvel_unconstrained,
        FmSMatrix3* submats,
        uint* indices,
        const FmMpcgSolverData& meshBSolverData,
        const FmTetMesh& tetMesh,
        const FmVolumeContact& contact,
        const FmVolumeContactVert* volContactVerts)
    {
        uint vertOffsetB = meshBSolverData.solverStateOffset;

        FmSVector3 Jvel = FmInitSVector3(0.0f);

        uint submatOffset = 0;
        FmSMatrix3 submatrix;
        uint idx;
        for (uint vId = 0; vId < contact.numVolVertsB; vId++)
        {
            const FmVolumeContactVert& vert = volContactVerts[contact.volVertsStartB + vId];
            uint vIdB = vert.vertId;
            uint svIdB = vIdB;

            FmSVector3 dVdp = FmInitSVector3(vert.dVdp);

            FmSVector3 tangent1 = FmInitSVector3(vert.tangent);
            FmSVector3 tangent2 = FmSafeNormalize(cross(dVdp, tangent1), normalize(FmOrthogonalVector(tangent1)));
#if FM_VOLCONTACT_TANGENT_PER_VERTEX
            float Ak = length(dVdp);
#else
            float Ak = dot(contact.normal, vert.dVdp);
#endif
            submatrix.setCol0(dVdp);
            submatrix.setCol1(tangent1 * Ak);
            submatrix.setCol2(tangent2 * Ak);
            submatrix = transpose(submatrix);
            idx = vertOffsetB + svIdB;

            if (vert.dynamic)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vIdB]));
        }

        *Jvel_unconstrained += Jvel;

        return submatOffset;
    }

    uint FmAddJacobianRowSubmats_RigidBodyA(
        FmSVector3* Jvel_unconstrained,
        FmSMatrix3* submats,
        uint* indices,
        uint stateVecOffsetA,
        const FmRigidBody& rigidBody,
        const FmGlueConstraint& glueConstraint)
    {
        bool isDynamic = FM_IS_SET(glueConstraint.dynamicFlags, 0x3);
        bool isMoving = FM_IS_SET(glueConstraint.movingFlags, 0x3);

        if (!isDynamic && !isMoving)
        {
            return 0;
        }

        uint submatOffset = 0;
        FmSMatrix3 submatrix;
        uint idx;
        submatrix.setCol0(FmInitSVector3(1.0f, 0.0f, 0.0f));
        submatrix.setCol1(FmInitSVector3(0.0f, 1.0f, 0.0f));
        submatrix.setCol2(FmInitSVector3(0.0f, 0.0f, 1.0f));
        submatrix = transpose(submatrix);
        idx = stateVecOffsetA;

        if (isDynamic)
        {
            submats[submatOffset] = submatrix;
            indices[submatOffset] = idx;
            submatOffset++;
        }

        FmSVector3 Jvel = mul(submatrix, FmInitSVector3(rigidBody.state.vel));

        FmSVector3 comToPosA = FmInitSVector3(glueConstraint.comToPosA);
        submatrix.setCol0(cross(comToPosA, FmInitSVector3(1.0f, 0.0f, 0.0f)));
        submatrix.setCol1(cross(comToPosA, FmInitSVector3(0.0f, 1.0f, 0.0f)));
        submatrix.setCol2(cross(comToPosA, FmInitSVector3(0.0f, 0.0f, 1.0f)));
        submatrix = transpose(submatrix);
        idx++;

        if (isDynamic)
        {
            submats[submatOffset] = submatrix;
            indices[submatOffset] = idx;
            submatOffset++;
        }

        Jvel += mul(submatrix, FmInitSVector3(rigidBody.state.angVel));

        *Jvel_unconstrained += Jvel;

        return submatOffset;
    }

    uint FmAddJacobianRowSubmats_RigidBodyB(
        FmSVector3* Jvel_unconstrained,
        FmSMatrix3* submats,
        uint* indices,
        uint stateVecOffsetB,
        const FmRigidBody& rigidBody,
        const FmGlueConstraint& glueConstraint)
    {
        bool isDynamic = FM_IS_SET(glueConstraint.dynamicFlags, 0x30);
        bool isMoving = FM_IS_SET(glueConstraint.movingFlags, 0x30);

        if (!isDynamic && !isMoving)
        {
            return 0;
        }

        uint submatOffset = 0;
        FmSMatrix3 submatrix;
        uint idx;
        submatrix.setCol0(FmInitSVector3(-1.0f, 0.0f, 0.0f));
        submatrix.setCol1(FmInitSVector3(0.0f, -1.0f, 0.0f));
        submatrix.setCol2(FmInitSVector3(0.0f, 0.0f, -1.0f));
        submatrix = transpose(submatrix);
        idx = stateVecOffsetB;

        if (isDynamic)
        {
            submats[submatOffset] = submatrix;
            indices[submatOffset] = idx;
            submatOffset++;
        }

        FmSVector3 Jvel = mul(submatrix, FmInitSVector3(rigidBody.state.vel));

        FmSVector3 comToPosB = FmInitSVector3(glueConstraint.comToPosB);
        submatrix.setCol0(cross(comToPosB, FmInitSVector3(-1.0f, 0.0f, 0.0f)));
        submatrix.setCol1(cross(comToPosB, FmInitSVector3(0.0f, -1.0f, 0.0f)));
        submatrix.setCol2(cross(comToPosB, FmInitSVector3(0.0f, 0.0f, -1.0f)));
        submatrix = transpose(submatrix);
        idx++;

        if (isDynamic)
        {
            submats[submatOffset] = submatrix;
            indices[submatOffset] = idx;
            submatOffset++;
        }

        Jvel += mul(submatrix, FmInitSVector3(rigidBody.state.angVel));

        *Jvel_unconstrained += Jvel;

        return submatOffset;
    }

    uint FmAddJacobianRowSubmats_TetMeshA(
        FmSVector3* Jvel_unconstrained,
        FmSMatrix3* submats,
        uint* indices,
        const FmMpcgSolverData& meshASolverData,
        const FmTetMesh& tetMesh,
        const FmGlueConstraint& glueConstraint)
    {
        uint vertOffsetA = meshASolverData.solverStateOffset;

        FmTetVertIds tetVertIdsA = tetMesh.tetsVertIds[glueConstraint.tetIdA];

        uint vId0 = tetVertIdsA.ids[0];
        uint vId1 = tetVertIdsA.ids[1];
        uint vId2 = tetVertIdsA.ids[2];
        uint vId3 = tetVertIdsA.ids[3];

        uint svId0 = vId0;
        uint svId1 = vId1;
        uint svId2 = vId2;
        uint svId3 = vId3;

        float weights0 = glueConstraint.posBaryA[0];
        float weights1 = glueConstraint.posBaryA[1];
        float weights2 = glueConstraint.posBaryA[2];
        float weights3 = glueConstraint.posBaryA[3];

        FmSVector3 Jvel = FmInitSVector3(0.0f);

        uint submatFlags = glueConstraint.dynamicFlags | glueConstraint.movingFlags;

        uint submatOffset = 0;
        FmSMatrix3 submatrix;
        uint idx;
        if (submatFlags & 0x1)
        {
            submatrix.setCol0(FmInitSVector3(weights0, 0.0f, 0.0f));
            submatrix.setCol1(FmInitSVector3(0.0f, weights0, 0.0f));
            submatrix.setCol2(FmInitSVector3(0.0f, 0.0f, weights0));
            submatrix = transpose(submatrix);
            idx = vertOffsetA + svId0;

            if (glueConstraint.dynamicFlags & 0x1)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId0]));
        }
        if (submatFlags & 0x2)
        {
            submatrix.setCol0(FmInitSVector3(weights1, 0.0f, 0.0f));
            submatrix.setCol1(FmInitSVector3(0.0f, weights1, 0.0f));
            submatrix.setCol2(FmInitSVector3(0.0f, 0.0f, weights1));
            submatrix = transpose(submatrix);
            idx = vertOffsetA + svId1;

            if (glueConstraint.dynamicFlags & 0x2)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId1]));
        }
        if (submatFlags & 0x4)
        {
            submatrix.setCol0(FmInitSVector3(weights2, 0.0f, 0.0f));
            submatrix.setCol1(FmInitSVector3(0.0f, weights2, 0.0f));
            submatrix.setCol2(FmInitSVector3(0.0f, 0.0f, weights2));
            submatrix = transpose(submatrix);
            idx = vertOffsetA + svId2;

            if (glueConstraint.dynamicFlags & 0x4)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId2]));
        }
        if (submatFlags & 0x8)
        {
            submatrix.setCol0(FmInitSVector3(weights3, 0.0f, 0.0f));
            submatrix.setCol1(FmInitSVector3(0.0f, weights3, 0.0f));
            submatrix.setCol2(FmInitSVector3(0.0f, 0.0f, weights3));
            submatrix = transpose(submatrix);
            idx = vertOffsetA + svId3;

            if (glueConstraint.dynamicFlags & 0x8)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId3]));
        }

        *Jvel_unconstrained += Jvel;

        return submatOffset;
    }

    uint FmAddJacobianRowSubmats_TetMeshB(
        FmSVector3* Jvel_unconstrained,
        FmSMatrix3* submats,
        uint* indices,
        const FmMpcgSolverData& meshBSolverData,
        const FmTetMesh& tetMesh,
        const FmGlueConstraint& glueConstraint)
    {
        uint vertOffsetB = meshBSolverData.solverStateOffset;

        FmTetVertIds tetVertIdsB = tetMesh.tetsVertIds[glueConstraint.tetIdB];

        uint vId4 = tetVertIdsB.ids[0];
        uint vId5 = tetVertIdsB.ids[1];
        uint vId6 = tetVertIdsB.ids[2];
        uint vId7 = tetVertIdsB.ids[3];

        uint svId4 = vId4;
        uint svId5 = vId5;
        uint svId6 = vId6;
        uint svId7 = vId7;

        float weights4 = -glueConstraint.posBaryB[0];
        float weights5 = -glueConstraint.posBaryB[1];
        float weights6 = -glueConstraint.posBaryB[2];
        float weights7 = -glueConstraint.posBaryB[3];

        FmSVector3 Jvel = FmInitSVector3(0.0f);

        uint submatFlags = glueConstraint.dynamicFlags | glueConstraint.movingFlags;

        uint submatOffset = 0;
        FmSMatrix3 submatrix;
        uint idx;
        if (submatFlags & 0x10)
        {
            submatrix.setCol0(FmInitSVector3(weights4, 0.0f, 0.0f));
            submatrix.setCol1(FmInitSVector3(0.0f, weights4, 0.0f));
            submatrix.setCol2(FmInitSVector3(0.0f, 0.0f, weights4));
            submatrix = transpose(submatrix);
            idx = vertOffsetB + svId4;

            if (glueConstraint.dynamicFlags & 0x10)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId4]));
        }
        if (submatFlags & 0x20)
        {
            submatrix.setCol0(FmInitSVector3(weights5, 0.0f, 0.0f));
            submatrix.setCol1(FmInitSVector3(0.0f, weights5, 0.0f));
            submatrix.setCol2(FmInitSVector3(0.0f, 0.0f, weights5));
            submatrix = transpose(submatrix);
            idx = vertOffsetB + svId5;

            if (glueConstraint.dynamicFlags & 0x20)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId5]));
        }
        if (submatFlags & 0x40)
        {
            submatrix.setCol0(FmInitSVector3(weights6, 0.0f, 0.0f));
            submatrix.setCol1(FmInitSVector3(0.0f, weights6, 0.0f));
            submatrix.setCol2(FmInitSVector3(0.0f, 0.0f, weights6));
            submatrix = transpose(submatrix);
            idx = vertOffsetB + svId6;

            if (glueConstraint.dynamicFlags & 0x40)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId6]));
        }
        if (submatFlags & 0x80)
        {
            submatrix.setCol0(FmInitSVector3(weights7, 0.0f, 0.0f));
            submatrix.setCol1(FmInitSVector3(0.0f, weights7, 0.0f));
            submatrix.setCol2(FmInitSVector3(0.0f, 0.0f, weights7));
            submatrix = transpose(submatrix);
            idx = vertOffsetB + svId7;

            if (glueConstraint.dynamicFlags & 0x80)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId7]));
        }

        *Jvel_unconstrained += Jvel;

        return submatOffset;
    }

    uint FmAddJacobianRowSubmats_RigidBodyA(
        FmSVector3* Jvel_unconstrained,
        FmSMatrix3* submats,
        uint* indices,
        uint stateVecOffsetA,
        const FmRigidBody& rigidBody,
        const FmPlaneConstraint& planeConstraint)
    {
        bool isDynamic = FM_IS_SET(planeConstraint.dynamicFlags, 0x3);
        bool isMoving = FM_IS_SET(planeConstraint.movingFlags, 0x3);

        if (!isDynamic && !isMoving)
        {
            return 0;
        }

        uint submatOffset = 0;
        FmSMatrix3 submatrix;
        uint idx;
        submatrix.setCol0(FmInitSVector3(planeConstraint.planeNormal0));
        submatrix.setCol1(FmInitSVector3(planeConstraint.planeNormal1));
        submatrix.setCol2(FmInitSVector3(planeConstraint.planeNormal2));
        submatrix = transpose(submatrix);
        idx = stateVecOffsetA;

        if (isDynamic)
        {
            submats[submatOffset] = submatrix;
            indices[submatOffset] = idx;
            submatOffset++;
        }

        FmSVector3 Jvel = mul(submatrix, FmInitSVector3(rigidBody.state.vel));

        FmSVector3 comToPosA = FmInitSVector3(planeConstraint.comToPosA);
        submatrix.setCol0(cross(comToPosA, FmInitSVector3(planeConstraint.planeNormal0)));
        submatrix.setCol1(cross(comToPosA, FmInitSVector3(planeConstraint.planeNormal1)));
        submatrix.setCol2(cross(comToPosA, FmInitSVector3(planeConstraint.planeNormal2)));
        submatrix = transpose(submatrix);
        idx++;

        if (isDynamic)
        {
            submats[submatOffset] = submatrix;
            indices[submatOffset] = idx;
            submatOffset++;
        }

        Jvel += mul(submatrix, FmInitSVector3(rigidBody.state.angVel));

        *Jvel_unconstrained += Jvel;

        return submatOffset;
    }

    uint FmAddJacobianRowSubmats_RigidBodyB(
        FmSVector3* Jvel_unconstrained,
        FmSMatrix3* submats,
        uint* indices,
        uint stateVecOffsetB,
        const FmRigidBody& rigidBody,
        const FmPlaneConstraint& planeConstraint)
    {
        bool isDynamic = FM_IS_SET(planeConstraint.dynamicFlags, 0x30);
        bool isMoving = FM_IS_SET(planeConstraint.movingFlags, 0x30);

        if (!isDynamic && !isMoving)
        {
            return 0;
        }

        uint submatOffset = 0;
        FmSMatrix3 submatrix;
        uint idx;
        submatrix.setCol0(-FmInitSVector3(planeConstraint.planeNormal0));
        submatrix.setCol1(-FmInitSVector3(planeConstraint.planeNormal1));
        submatrix.setCol2(-FmInitSVector3(planeConstraint.planeNormal2));
        submatrix = transpose(submatrix);
        idx = stateVecOffsetB;

        if (isDynamic)
        {
            submats[submatOffset] = submatrix;
            indices[submatOffset] = idx;
            submatOffset++;
        }

        FmSVector3 Jvel = mul(submatrix, FmInitSVector3(rigidBody.state.vel));

        FmSVector3 comToPosB = FmInitSVector3(planeConstraint.comToPosB);
        submatrix.setCol0(cross(comToPosB, -FmInitSVector3(planeConstraint.planeNormal0)));
        submatrix.setCol1(cross(comToPosB, -FmInitSVector3(planeConstraint.planeNormal1)));
        submatrix.setCol2(cross(comToPosB, -FmInitSVector3(planeConstraint.planeNormal2)));
        submatrix = transpose(submatrix);
        idx++;

        if (isDynamic)
        {
            submats[submatOffset] = submatrix;
            indices[submatOffset] = idx;
            submatOffset++;
        }

        Jvel += mul(submatrix, FmInitSVector3(rigidBody.state.angVel));

        *Jvel_unconstrained += Jvel;

        return submatOffset;
    }

    uint FmAddJacobianRowSubmats_TetMeshA(
        FmSVector3* Jvel_unconstrained,
        FmSMatrix3* submats,
        uint* indices,
        const FmMpcgSolverData& meshASolverData,
        const FmTetMesh& tetMesh,
        const FmPlaneConstraint& planeConstraint)
    {
        uint vertOffsetA = meshASolverData.solverStateOffset;

        FmTetVertIds tetVertIdsA = tetMesh.tetsVertIds[planeConstraint.tetIdA];

        uint vId0 = tetVertIdsA.ids[0];
        uint vId1 = tetVertIdsA.ids[1];
        uint vId2 = tetVertIdsA.ids[2];
        uint vId3 = tetVertIdsA.ids[3];

        uint svId0 = vId0;
        uint svId1 = vId1;
        uint svId2 = vId2;
        uint svId3 = vId3;

        float weights0 = planeConstraint.posBaryA[0];
        float weights1 = planeConstraint.posBaryA[1];
        float weights2 = planeConstraint.posBaryA[2];
        float weights3 = planeConstraint.posBaryA[3];

        FmSVector3 Jvel = FmInitSVector3(0.0f);

        uint submatFlags = planeConstraint.dynamicFlags | planeConstraint.movingFlags;

        uint submatOffset = 0;
        FmSMatrix3 submatrix;
        uint idx;
        if (submatFlags & 0x1)
        {
            submatrix.setCol0(FmInitSVector3(planeConstraint.planeNormal0)*weights0);
            submatrix.setCol1(FmInitSVector3(planeConstraint.planeNormal1)*weights0);
            submatrix.setCol2(FmInitSVector3(planeConstraint.planeNormal2)*weights0);
            submatrix = transpose(submatrix);
            idx = vertOffsetA + svId0;

            if (planeConstraint.dynamicFlags & 0x1)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId0]));
        }
        if (submatFlags & 0x2)
        {
            submatrix.setCol0(FmInitSVector3(planeConstraint.planeNormal0)*weights1);
            submatrix.setCol1(FmInitSVector3(planeConstraint.planeNormal1)*weights1);
            submatrix.setCol2(FmInitSVector3(planeConstraint.planeNormal2)*weights1);
            submatrix = transpose(submatrix);
            idx = vertOffsetA + svId1;

            if (planeConstraint.dynamicFlags & 0x2)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId1]));
        }
        if (submatFlags & 0x4)
        {
            submatrix.setCol0(FmInitSVector3(planeConstraint.planeNormal0)*weights2);
            submatrix.setCol1(FmInitSVector3(planeConstraint.planeNormal1)*weights2);
            submatrix.setCol2(FmInitSVector3(planeConstraint.planeNormal2)*weights2);
            submatrix = transpose(submatrix);
            idx = vertOffsetA + svId2;

            if (planeConstraint.dynamicFlags & 0x4)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId2]));
        }
        if (submatFlags & 0x8)
        {
            submatrix.setCol0(FmInitSVector3(planeConstraint.planeNormal0)*weights3);
            submatrix.setCol1(FmInitSVector3(planeConstraint.planeNormal1)*weights3);
            submatrix.setCol2(FmInitSVector3(planeConstraint.planeNormal2)*weights3);
            submatrix = transpose(submatrix);
            idx = vertOffsetA + svId3;

            if (planeConstraint.dynamicFlags & 0x8)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId3]));
        }

        *Jvel_unconstrained += Jvel;

        return submatOffset;
    }

    uint FmAddJacobianRowSubmats_TetMeshB(
        FmSVector3* Jvel_unconstrained,
        FmSMatrix3* submats,
        uint* indices,
        const FmMpcgSolverData& meshBSolverData,
        const FmTetMesh& tetMesh,
        const FmPlaneConstraint& planeConstraint)
    {
        uint vertOffsetB = meshBSolverData.solverStateOffset;

        FmTetVertIds tetVertIdsB = tetMesh.tetsVertIds[planeConstraint.tetIdB];

        uint vId4 = tetVertIdsB.ids[0];
        uint vId5 = tetVertIdsB.ids[1];
        uint vId6 = tetVertIdsB.ids[2];
        uint vId7 = tetVertIdsB.ids[3];

        uint svId4 = vId4;
        uint svId5 = vId5;
        uint svId6 = vId6;
        uint svId7 = vId7;

        float weights4 = -planeConstraint.posBaryB[0];
        float weights5 = -planeConstraint.posBaryB[1];
        float weights6 = -planeConstraint.posBaryB[2];
        float weights7 = -planeConstraint.posBaryB[3];

        FmSVector3 Jvel = FmInitSVector3(0.0f);

        uint submatFlags = planeConstraint.dynamicFlags | planeConstraint.movingFlags;

        uint submatOffset = 0;
        FmSMatrix3 submatrix;
        uint idx;
        if (submatFlags & 0x10)
        {
            submatrix.setCol0(FmInitSVector3(planeConstraint.planeNormal0)*weights4);
            submatrix.setCol1(FmInitSVector3(planeConstraint.planeNormal1)*weights4);
            submatrix.setCol2(FmInitSVector3(planeConstraint.planeNormal2)*weights4);
            submatrix = transpose(submatrix);
            idx = vertOffsetB + svId4;

            if (planeConstraint.dynamicFlags & 0x10)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId4]));
        }
        if (submatFlags & 0x20)
        {
            submatrix.setCol0(FmInitSVector3(planeConstraint.planeNormal0)*weights5);
            submatrix.setCol1(FmInitSVector3(planeConstraint.planeNormal1)*weights5);
            submatrix.setCol2(FmInitSVector3(planeConstraint.planeNormal2)*weights5);
            submatrix = transpose(submatrix);
            idx = vertOffsetB + svId5;

            if (planeConstraint.dynamicFlags & 0x20)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId5]));
        }
        if (submatFlags & 0x40)
        {
            submatrix.setCol0(FmInitSVector3(planeConstraint.planeNormal0)*weights6);
            submatrix.setCol1(FmInitSVector3(planeConstraint.planeNormal1)*weights6);
            submatrix.setCol2(FmInitSVector3(planeConstraint.planeNormal2)*weights6);
            submatrix = transpose(submatrix);
            idx = vertOffsetB + svId6;

            if (planeConstraint.dynamicFlags & 0x40)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId6]));
        }
        if (submatFlags & 0x80)
        {
            submatrix.setCol0(FmInitSVector3(planeConstraint.planeNormal0)*weights7);
            submatrix.setCol1(FmInitSVector3(planeConstraint.planeNormal1)*weights7);
            submatrix.setCol2(FmInitSVector3(planeConstraint.planeNormal2)*weights7);
            submatrix = transpose(submatrix);
            idx = vertOffsetB + svId7;

            if (planeConstraint.dynamicFlags & 0x80)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId7]));
        }

        *Jvel_unconstrained += Jvel;

        return submatOffset;
    }

    uint FmAddJacobianRowSubmats_TetMesh(
        FmSVector3* Jvel_unconstrained,
        FmSMatrix3* submats,
        uint* indices,
        const FmMpcgSolverData& meshASolverData,
        const FmTetMesh& tetMesh,
        const FmDeformationConstraint& deformationConstraint)
    {
        uint vertOffsetA = meshASolverData.solverStateOffset;

        FmTetVertIds tetVertIds = tetMesh.tetsVertIds[deformationConstraint.tetId];

        uint vId0 = tetVertIds.ids[0];
        uint vId1 = tetVertIds.ids[1];
        uint vId2 = tetVertIds.ids[2];
        uint vId3 = tetVertIds.ids[3];

        uint svId0 = vId0;
        uint svId1 = vId1;
        uint svId2 = vId2;
        uint svId3 = vId3;

        FmSVector3 Jvel = FmInitSVector3(0.0f);

        uint submatFlags = deformationConstraint.dynamicFlags | deformationConstraint.movingFlags;

        uint submatOffset = 0;
        FmSMatrix3 submatrix;
        uint idx;
        if (submatFlags & 0x1)
        {
            submatrix = FmInitSMatrix3(deformationConstraint.jacobian0);
            idx = vertOffsetA + svId0;

            if (deformationConstraint.dynamicFlags & 0x1)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId0]));
        }
        if (submatFlags & 0x2)
        {
            submatrix = FmInitSMatrix3(deformationConstraint.jacobian1);
            idx = vertOffsetA + svId1;

            if (deformationConstraint.dynamicFlags & 0x2)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId1]));
        }
        if (submatFlags & 0x4)
        {
            submatrix = FmInitSMatrix3(deformationConstraint.jacobian2);
            idx = vertOffsetA + svId2;

            if (deformationConstraint.dynamicFlags & 0x4)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId2]));
        }
        if (submatFlags & 0x8)
        {
            submatrix = FmInitSMatrix3(deformationConstraint.jacobian3);
            idx = vertOffsetA + svId3;

            if (deformationConstraint.dynamicFlags & 0x8)
            {
                submats[submatOffset] = submatrix;
                indices[submatOffset] = idx;
                submatOffset++;
            }

            Jvel += mul(submatrix, FmInitSVector3(tetMesh.vertsVel[vId3]));
        }

        *Jvel_unconstrained += Jvel;

        return submatOffset;
    }

    uint FmAddJacobianRowSubmats_RigidBodyA(
        FmSVector3* Jvel_unconstrained,
        FmSMatrix3* submats,
        uint* indices,
        uint stateVecOffsetA,
        const FmRigidBody& rigidBody,
        const FmRigidBodyAngleConstraint& rigidBodyAngleConstraint)
    {
        bool isDynamic = FM_IS_SET(rigidBodyAngleConstraint.dynamicFlags, 0x3);
        bool isMoving = FM_IS_SET(rigidBodyAngleConstraint.movingFlags, 0x3);

        if (!isDynamic && !isMoving)
        {
            return 0;
        }

        FmSMatrix3 submatrix;
        uint submatOffset = 0;
        
        submatrix = FmInitSMatrix3(rigidBodyAngleConstraint.jacobianA);

        if (isDynamic)
        {
            submats[submatOffset] = submatrix;
            indices[submatOffset] = stateVecOffsetA + 1;   // angular velocity portion of rigid body state
            submatOffset++;
        }

        FmSVector3 Jvel = mul(submatrix, FmInitSVector3(rigidBody.state.angVel));

        *Jvel_unconstrained += Jvel;

        return submatOffset;
    }

    uint FmAddJacobianRowSubmats_RigidBodyB(
        FmSVector3* Jvel_unconstrained,
        FmSMatrix3* submats,
        uint* indices,
        uint stateVecOffsetB,
        const FmRigidBody& rigidBody,
        const FmRigidBodyAngleConstraint& rigidBodyAngleConstraint)
    {
        bool isDynamic = FM_IS_SET(rigidBodyAngleConstraint.dynamicFlags, 0x30);
        bool isMoving = FM_IS_SET(rigidBodyAngleConstraint.movingFlags, 0x30);

        if (!isDynamic && !isMoving)
        {
            return 0;
        }

        FmSMatrix3 submatrix;
        uint submatOffset = 0;

        submatrix = FmInitSMatrix3(rigidBodyAngleConstraint.jacobianB);

        if (isDynamic)
        {
            submats[submatOffset] = submatrix;
            indices[submatOffset] = stateVecOffsetB + 1;   // angular velocity portion of rigid body state
            submatOffset++;
        }

        FmSVector3 Jvel = mul(submatrix, FmInitSVector3(rigidBody.state.angVel));

        *Jvel_unconstrained += Jvel;

        return submatOffset;
    }

    class FmTaskDataInitSolverData : public FmAsyncTaskData
    {
    public:
        FmScene* scene;
        FmConstraintSolverData* constraintSolverData;
        FmConstraintIsland* constraintIsland;
        float timestep;
        float timestepInv;

        uint numTetMeshes;
        uint numRigidBodies;
        uint numRigidBodyTasks;
        uint numConstraints;
        uint numConstraintTasks;

        FmTaskDataInitSolverData(
            FmScene* inScene, FmConstraintSolverData* inConstraintSolverData, FmConstraintIsland* inConstraintIsland,
            float inTimestep,
            uint inNumTetMeshes,
            uint inNumRigidBodies, uint inNumRigidBodyTasks,
            uint inNumConstraints, uint inNumConstraintTasks)
        {
            scene = inScene;
            constraintSolverData = inConstraintSolverData;
            constraintIsland = inConstraintIsland;
            timestep = inTimestep;
            timestepInv = 1.0f / timestep;

            numTetMeshes = inNumTetMeshes;

            numRigidBodies = inNumRigidBodies;
            numRigidBodyTasks = inNumRigidBodyTasks;

            numConstraints = inNumConstraints;
            numConstraintTasks = inNumConstraintTasks;
        }
    };

    void FmTaskFuncInitMeshSolverArrayData(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE_INIT_MESH_DATA);

        FmTaskDataInitSolverData* taskData = (FmTaskDataInitSolverData*)inTaskData;
        FmScene* scene = taskData->scene;
        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        FmConstraintIsland& constraintIsland = *taskData->constraintIsland;

        uint beginIdx = (uint)inTaskBeginIndex;
        uint endIdx = (uint)inTaskEndIndex;

        for (uint islandMeshIdx = beginIdx; islandMeshIdx < endIdx; islandMeshIdx++)
        {
            FmMpcgSolverData& meshMpcgData = *FmGetSolverDataById(*scene, constraintIsland.tetMeshIds[islandMeshIdx]);

            uint stateOffset = meshMpcgData.solverStateOffset;

            uint numMeshVerts = meshMpcgData.A.numRows;

            // Initialize portion of constraint solve array data corresponding to this tet mesh.
            // pgsDeltaVelterm is initialized after first outer iteration of solve
            for (uint vId = 0; vId < numMeshVerts; vId++)
            {
                constraintSolverData->DAinv[stateOffset + vId] = meshMpcgData.PInvDiag[vId];
                float invMass = 1.0f / meshMpcgData.mass[vId];

                meshMpcgData.b[vId] = FmInitSVector3(0.0f);

                constraintSolverData->W[stateOffset + vId] =
                    FmInitSMatrix3(
                        FmInitSVector3(invMass, 0.0f, 0.0f),
                        FmInitSVector3(0.0f, invMass, 0.0f),
                        FmInitSVector3(0.0f, 0.0f, invMass));

                constraintSolverData->deltaVel[stateOffset + vId] = FmInitSVector3(0.0f);
                constraintSolverData->JTlambda[stateOffset + vId] = FmInitSVector3(0.0f);
            }
        }

        taskData->progress.TasksAreFinished(endIdx - beginIdx);
    }

#if FM_CONSTRAINT_STABILIZATION_SOLVE
    void FmTaskFuncInitMeshSolverArrayDataForStabilization(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE_INIT_MESH_DATA);

        FmTaskDataInitSolverData* taskData = (FmTaskDataInitSolverData*)inTaskData;
        FmScene* scene = taskData->scene;
        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        FmConstraintIsland& constraintIsland = *taskData->constraintIsland;
        float delta_t = taskData->timestep;

        uint beginIdx = (uint)inTaskBeginIndex;
        uint endIdx = (uint)inTaskEndIndex;

        for (uint islandMeshIdx = beginIdx; islandMeshIdx < endIdx; islandMeshIdx++)
        {
            FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, constraintIsland.tetMeshIds[islandMeshIdx]);
            FmMpcgSolverData& meshMpcgData = *FmGetSolverDataById(*scene, constraintIsland.tetMeshIds[islandMeshIdx]);

            uint numMeshVerts = meshMpcgData.A.numRows;
            uint stateOffset = meshMpcgData.solverStateOffset;

            FM_ASSERT(stateOffset != FM_INVALID_ID);

            // Initialize portion of constraint stabilization array data corresponding to this tet mesh.
            // pgsDeltaVelterm is initialized after first outer iteration of solve
            for (uint vId = 0; vId < numMeshVerts; vId++)
            {
                uint svId = vId;

                // Solving for lambda and deltaPos in:
                // A * deltaPos = J^T * lambda
                // and J * (deltaPos + vel * deltat) >= -g0 (penetration correction)
                // vel is pre-contact vel + contact solve deltaVel
                constraintSolverData->velTemp[stateOffset + svId] =
                    (FmInitSVector3(tetMesh.vertsVel[vId]) +
                        constraintSolverData->deltaVel[stateOffset + svId]) * delta_t;

                constraintSolverData->deltaPos[stateOffset + svId] = FmInitSVector3(0.0f);
                constraintSolverData->JTlambda[stateOffset + svId] = FmInitSVector3(0.0f);

                meshMpcgData.b[svId] = FmInitSVector3(0.0f);
            }
        }

        taskData->progress.TasksAreFinished(endIdx - beginIdx);
    }
#endif

    void FmTaskFuncInitRbSolverArrayData(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE_INIT_RB_DATA);

        FmTaskDataInitSolverData* taskData = (FmTaskDataInitSolverData*)inTaskData;
        FmScene* scene = taskData->scene;
        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        FmConstraintIsland& constraintIsland = *taskData->constraintIsland;
        const FmConstraintSolverBuffer& constraintSolverBuffer = *scene->constraintSolverBuffer;

        uint beginIdx, endIdx;
        FmGetIndexRangeEvenDistribution(&beginIdx, &endIdx, (uint)inTaskBeginIndex, taskData->numRigidBodyTasks, taskData->numRigidBodies);

        // Init values corresponding to rigid body DOFs
        for (uint islandRbIdx = beginIdx; islandRbIdx < endIdx; islandRbIdx++)
        {
            uint rbId = constraintIsland.rigidBodyIds[islandRbIdx];

            FmRigidBody& rb = *FmGetRigidBodyPtrById(*scene, rbId);
            uint stateOffset = FmGetRigidBodySolverOffsetById(constraintSolverBuffer, rbId);

            rb.worldInertiaTensor = FmTransformBodyInertiaToWorld(rb.bodyInertiaTensor, FmInitMatrix3(rb.state.quat));

            float invMass = 1.0f / rb.mass;

            constraintSolverData->DAinv[stateOffset] =
                FmInitSMatrix3(
                    FmInitSVector3(invMass, 0.0f, 0.0f),
                    FmInitSVector3(0.0f, invMass, 0.0f),
                    FmInitSVector3(0.0f, 0.0f, invMass));
            constraintSolverData->DAinv[stateOffset + 1] = FmInitSMatrix3(inverse(rb.worldInertiaTensor));

            constraintSolverData->W[stateOffset] = constraintSolverData->DAinv[stateOffset];
            constraintSolverData->W[stateOffset + 1] = constraintSolverData->DAinv[stateOffset + 1];

            constraintSolverData->deltaVel[stateOffset] = FmInitSVector3(0.0f);
            constraintSolverData->deltaVel[stateOffset + 1] = FmInitSVector3(0.0f);

            // For rigid bodies this term is always zero
            constraintSolverData->pgsDeltaVelTerm[stateOffset] = FmInitSVector3(0.0f);
            constraintSolverData->pgsDeltaVelTerm[stateOffset + 1] = FmInitSVector3(0.0f);

            constraintSolverData->JTlambda[stateOffset] = FmInitSVector3(0.0f);
            constraintSolverData->JTlambda[stateOffset + 1] = FmInitSVector3(0.0f);
        }

        taskData->progress.TaskIsFinished();
    }

    void FmTaskFuncInitRbSolverArrayDataForStabilization(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE_INIT_RB_DATA);

        FmTaskDataInitSolverData* taskData = (FmTaskDataInitSolverData*)inTaskData;
        FmScene* scene = taskData->scene;
        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        FmConstraintIsland& constraintIsland = *taskData->constraintIsland;
        const FmConstraintSolverBuffer& constraintSolverBuffer = *scene->constraintSolverBuffer;
        float delta_t = taskData->timestep;

        uint beginIdx, endIdx;
        FmGetIndexRangeEvenDistribution(&beginIdx, &endIdx, (uint)inTaskBeginIndex, taskData->numRigidBodyTasks, taskData->numRigidBodies);

        // Init values corresponding to rigid body DOFs
        for (uint islandRbIdx = beginIdx; islandRbIdx < endIdx; islandRbIdx++)
        {
            uint rbId = constraintIsland.rigidBodyIds[islandRbIdx];

            FmRigidBody& rb = *FmGetRigidBodyPtrById(*scene, rbId);
            uint stateOffset = FmGetRigidBodySolverOffsetById(constraintSolverBuffer, rbId);

            // Read deltaVel values from rigid body in case updated externally
            FmSVector3 deltaVel = constraintSolverData->deltaVel[stateOffset];
            FmSVector3 deltaAngVel = constraintSolverData->deltaVel[stateOffset + 1];
            constraintSolverData->velTemp[stateOffset] = (FmInitSVector3(rb.state.vel) + deltaVel) * delta_t;
            constraintSolverData->velTemp[stateOffset + 1] = (FmInitSVector3(rb.state.angVel) + deltaAngVel) * delta_t;

#if FM_CONSTRAINT_STABILIZATION_SOLVE
            constraintSolverData->deltaPos[stateOffset] = FmInitSVector3(0.0f);
            constraintSolverData->deltaPos[stateOffset + 1] = FmInitSVector3(0.0f);
#endif

            // For rigid bodies this term is always zero
            constraintSolverData->pgsDeltaVelTerm[stateOffset] = FmInitSVector3(0.0f);
            constraintSolverData->pgsDeltaVelTerm[stateOffset + 1] = FmInitSVector3(0.0f);

            constraintSolverData->JTlambda[stateOffset] = FmInitSVector3(0.0f);
            constraintSolverData->JTlambda[stateOffset + 1] = FmInitSVector3(0.0f);
        }

        taskData->progress.TaskIsFinished();
    }

    void FmTaskFuncInitConstraintArrayData(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE_INIT_CONSTRAINT_DATA);

        FmTaskDataInitSolverData* taskData = (FmTaskDataInitSolverData*)inTaskData;

        FmScene* scene = taskData->scene;
        FmConstraintSolverBuffer* constraintSolverBuffer = scene->constraintSolverBuffer;
        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;

        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        FmConstraintIsland& constraintIsland = *taskData->constraintIsland;
        float delta_t_inv = taskData->timestepInv;

        FmConstraintSolverControlParams& controlParams = *constraintSolverData->currentControlParams;

        uint beginIdx, endIdx;
        FmGetIndexRangeEvenDistribution(&beginIdx, &endIdx, (uint)inTaskBeginIndex, taskData->numConstraintTasks, taskData->numConstraints);

        // Accumulate contact data into constraint Jacobians.
        for (uint constraintIdx = beginIdx; constraintIdx < endIdx; constraintIdx++)
        {
            FmConstraintReference& constraintRef = constraintIsland.constraintRefs[constraintIdx];
            FmConstraintJacobian& J = constraintSolverData->J;
            FmConstraintParams& constraintParams = J.params[constraintIdx];

            FmSVector3 correctionVec = FmInitSVector3(0.0f);
            FmSVector3 Jvel_unconstrained = FmInitSVector3(0.0f);

            FmSMatrix3* jacobianSubmats = (FmSMatrix3*)((uint8_t*)J.submats + constraintParams.jacobianSubmatsOffset);
            uint* jacobianIndices = (uint*)((uint8_t*)J.indices + constraintParams.jacobianIndicesOffset);
            uint submatOffset = 0;

            if (constraintRef.type == FM_CONSTRAINT_TYPE_DISTANCE_CONTACT)
            {
                FmDistanceContactPairInfo& contactPairInfo = constraintsBuffer->distanceContactsPairInfo[constraintRef.idx];
                FmDistanceContact& contact = constraintsBuffer->distanceContacts[constraintRef.idx];

                constraintParams.type = FM_SOLVER_CONSTRAINT_TYPE_3D_NORMAL2DFRICTION;
                constraintParams.frictionCoeff = contact.frictionCoeff;

                if (contactPairInfo.objectIdA & FM_RB_FLAG)
                {
                    FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, contactPairInfo.objectIdA);
                    uint stateOffset = FmGetRigidBodySolverOffsetById(*constraintSolverBuffer, contactPairInfo.objectIdA);

                    submatOffset += FmAddJacobianRowSubmats_RigidBodyA(&Jvel_unconstrained, &jacobianSubmats[submatOffset], &jacobianIndices[submatOffset], stateOffset, rigidBody, contactPairInfo, contact);
                }
                else
                {
                    FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, contactPairInfo.objectIdA);
                    FmMpcgSolverData& meshASolverData = *FmGetSolverDataById(*scene, contactPairInfo.objectIdA);

                    submatOffset += FmAddJacobianRowSubmats_TetMeshA(&Jvel_unconstrained, &jacobianSubmats[submatOffset], &jacobianIndices[submatOffset], meshASolverData, tetMesh, contactPairInfo, contact);
                }

                if (contactPairInfo.objectIdB != FM_INVALID_ID)  // FM_INVALID_ID means collision plane or fixed point in world
                {
                    if (contactPairInfo.objectIdB & FM_RB_FLAG)
                    {
                        FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, contactPairInfo.objectIdB);
                        uint stateOffset = FmGetRigidBodySolverOffsetById(*constraintSolverBuffer, contactPairInfo.objectIdB);

                        submatOffset += FmAddJacobianRowSubmats_RigidBodyB(&Jvel_unconstrained, &jacobianSubmats[submatOffset], &jacobianIndices[submatOffset], stateOffset, rigidBody, contactPairInfo, contact);
                    }
                    else
                    {
                        FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, contactPairInfo.objectIdB);
                        FmMpcgSolverData& meshBSolverData = *FmGetSolverDataById(*scene, contactPairInfo.objectIdB);

                        submatOffset += FmAddJacobianRowSubmats_TetMeshB(&Jvel_unconstrained, &jacobianSubmats[submatOffset], &jacobianIndices[submatOffset], meshBSolverData, tetMesh, contactPairInfo, contact);
                    }
                }

                FM_ASSERT(submatOffset == FmGetNumJacobianSubmats(contactPairInfo));

                constraintSolverData->pgsRhs[constraintIdx] = FmInitSVector3(0.0f);
                constraintSolverData->lambda3[constraintIdx] = FmInitSVector3(0.0f);

                float kCorrection = controlParams.kDistanceCorrection;

                float normalError = -contact.normalProjDistance;
                kCorrection = (normalError < 0.0f) ? 1.0f : kCorrection;
                float normalCorrection = normalError * kCorrection * delta_t_inv;

                correctionVec = FmInitSVector3(normalCorrection, 0.0f, 0.0f);

                constraintSolverData->lambda3Temp[constraintIdx] = correctionVec;
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_VOLUME_CONTACT)
            {
                FmVolumeContact& contact = constraintsBuffer->volumeContacts[constraintRef.idx];

                constraintParams.type = FM_SOLVER_CONSTRAINT_TYPE_3D_NORMAL2DFRICTION;
                constraintParams.frictionCoeff = contact.frictionCoeff;

                if (contact.objectIdA & FM_RB_FLAG)
                {
                    FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, contact.objectIdA);
                    uint stateOffset = FmGetRigidBodySolverOffsetById(*constraintSolverBuffer, contact.objectIdA);

                    submatOffset += FmAddJacobianRowSubmats_RigidBodyA(&Jvel_unconstrained, &jacobianSubmats[submatOffset], &jacobianIndices[submatOffset], stateOffset, rigidBody, contact, constraintsBuffer->volumeContactVerts);
                }
                else
                {
                    FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, contact.objectIdA);
                    FmMpcgSolverData& meshASolverData = *FmGetSolverDataById(*scene, contact.objectIdA);

                    submatOffset += FmAddJacobianRowSubmats_TetMeshA(&Jvel_unconstrained, &jacobianSubmats[submatOffset], &jacobianIndices[submatOffset], meshASolverData, tetMesh, contact, constraintsBuffer->volumeContactVerts);
                }

                if (contact.objectIdB != FM_INVALID_ID)  // FM_INVALID_ID means collision plane or fixed point in world
                {
                    if (contact.objectIdB & FM_RB_FLAG)
                    {
                        FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, contact.objectIdB);
                        uint stateOffset = FmGetRigidBodySolverOffsetById(*constraintSolverBuffer, contact.objectIdB);

                        submatOffset += FmAddJacobianRowSubmats_RigidBodyB(&Jvel_unconstrained, &jacobianSubmats[submatOffset], &jacobianIndices[submatOffset], stateOffset, rigidBody, contact, constraintsBuffer->volumeContactVerts);
                    }
                    else
                    {
                        FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, contact.objectIdB);
                        FmMpcgSolverData& meshBSolverData = *FmGetSolverDataById(*scene, contact.objectIdB);

                        submatOffset += FmAddJacobianRowSubmats_TetMeshB(&Jvel_unconstrained, &jacobianSubmats[submatOffset], &jacobianIndices[submatOffset], meshBSolverData, tetMesh, contact, constraintsBuffer->volumeContactVerts);
                    }
                }

                FM_ASSERT(submatOffset == FmGetNumJacobianSubmats(contact));

                constraintSolverData->pgsRhs[constraintIdx] = FmInitSVector3(0.0f);
                constraintSolverData->lambda3[constraintIdx] = FmInitSVector3(0.0f);

                float kCorrection = controlParams.kVolumeCorrection;

                float volumeError = -contact.V;
                kCorrection = (volumeError < 0.0f) ? 1.0f : kCorrection;
                float volumeCorrection = volumeError * kCorrection * delta_t_inv;
                correctionVec = FmInitSVector3(volumeCorrection, 0.0f, 0.0f);

                constraintSolverData->lambda3Temp[constraintIdx] = correctionVec;
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_DEFORMATION)
            {
                FmDeformationConstraint& deformationConstraint = constraintsBuffer->deformationConstraints[constraintRef.idx];

                constraintParams.type = FM_SOLVER_CONSTRAINT_TYPE_3D;
                constraintParams.flags = FM_SOLVER_CONSTRAINT_FLAG_NONNEG0 | FM_SOLVER_CONSTRAINT_FLAG_NONNEG1 | FM_SOLVER_CONSTRAINT_FLAG_NONNEG2;

                FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, deformationConstraint.objectId);
                FmMpcgSolverData& meshSolverData = *FmGetSolverDataById(*scene, deformationConstraint.objectId);
                FM_ASSERT(meshSolverData.solverStateOffset != FM_INVALID_ID);

                submatOffset += FmAddJacobianRowSubmats_TetMesh(&Jvel_unconstrained, &jacobianSubmats[submatOffset], &jacobianIndices[submatOffset], meshSolverData, tetMesh, deformationConstraint);

                FM_ASSERT(submatOffset == FmGetNumJacobianSubmats(deformationConstraint));

                constraintSolverData->pgsRhs[constraintIdx] = FmInitSVector3(0.0f);
                constraintSolverData->lambda3[constraintIdx] = FmInitSVector3(0.0f);

                float kCorrection = deformationConstraint.kVelCorrection;

                // Apply kCorrection to positive correction only
                float kCorrectionX = deformationConstraint.deformationDeltas.x > 0.0f ? 1.0f : kCorrection;
                float kCorrectionY = deformationConstraint.deformationDeltas.y > 0.0f ? 1.0f : kCorrection;
                float kCorrectionZ = deformationConstraint.deformationDeltas.z > 0.0f ? 1.0f : kCorrection;

                correctionVec = -FmInitSVector3(
                    deformationConstraint.deformationDeltas.x * kCorrectionX,
                    deformationConstraint.deformationDeltas.y * kCorrectionY,
                    deformationConstraint.deformationDeltas.z * kCorrectionZ) * delta_t_inv;

                constraintSolverData->lambda3Temp[constraintIdx] = correctionVec;
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_GLUE)
            {
                FmGlueConstraint& glueConstraint = constraintsBuffer->glueConstraints[constraintRef.idx];

                constraintParams.type = FM_SOLVER_CONSTRAINT_TYPE_3D;

                if (glueConstraint.objectIdA == FM_INVALID_ID)
                {
                    continue;
                }

                if (glueConstraint.objectIdA & FM_RB_FLAG)
                {
                    FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, glueConstraint.objectIdA);
                    uint stateOffset = FmGetRigidBodySolverOffsetById(*constraintSolverBuffer, glueConstraint.objectIdA);

                    submatOffset += FmAddJacobianRowSubmats_RigidBodyA(&Jvel_unconstrained, &jacobianSubmats[submatOffset], &jacobianIndices[submatOffset], stateOffset, rigidBody, glueConstraint);
                }
                else
                {
                    FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, glueConstraint.objectIdA);
                    FmMpcgSolverData& meshASolverData = *FmGetSolverDataById(*scene, glueConstraint.objectIdA);

                    submatOffset += FmAddJacobianRowSubmats_TetMeshA(&Jvel_unconstrained, &jacobianSubmats[submatOffset], &jacobianIndices[submatOffset], meshASolverData, tetMesh, glueConstraint);
                }

                if (glueConstraint.objectIdB != FM_INVALID_ID)  // FM_INVALID_ID means collision plane or fixed point in world
                {
                    if (glueConstraint.objectIdB & FM_RB_FLAG)
                    {
                        FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, glueConstraint.objectIdB);
                        uint stateOffset = FmGetRigidBodySolverOffsetById(*constraintSolverBuffer, glueConstraint.objectIdB);

                        submatOffset += FmAddJacobianRowSubmats_RigidBodyB(&Jvel_unconstrained, &jacobianSubmats[submatOffset], &jacobianIndices[submatOffset], stateOffset, rigidBody, glueConstraint);
                    }
                    else
                    {
                        FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, glueConstraint.objectIdB);
                        FmMpcgSolverData& meshBSolverData = *FmGetSolverDataById(*scene, glueConstraint.objectIdB);

                        submatOffset += FmAddJacobianRowSubmats_TetMeshB(&Jvel_unconstrained, &jacobianSubmats[submatOffset], &jacobianIndices[submatOffset], meshBSolverData, tetMesh, glueConstraint);
                    }
                }

                FM_ASSERT(submatOffset == FmGetNumJacobianSubmats(glueConstraint));

                constraintSolverData->pgsRhs[constraintIdx] = FmInitSVector3(0.0f);
                constraintSolverData->lambda3[constraintIdx] = FmInitSVector3(0.0f);

                float kCorrection = glueConstraint.kVelCorrection;

                correctionVec = -FmInitSVector3(glueConstraint.deltaPos) * kCorrection * delta_t_inv;

                constraintSolverData->lambda3Temp[constraintIdx] = correctionVec;
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_PLANE)
            {
                FmPlaneConstraint& planeConstraint = constraintsBuffer->planeConstraints[constraintRef.idx];

                if (planeConstraint.objectIdA == FM_INVALID_ID)
                {
                    continue;
                }

                // TODO: specialize for 1 or 2D
                if (planeConstraint.flags & FM_CONSTRAINT_FLAG_1D)
                {
                    planeConstraint.planeNormal1 = FmInitVector3(0.0f);
                    planeConstraint.planeNormal2 = FmInitVector3(0.0f);
                }
                else if (planeConstraint.flags & FM_CONSTRAINT_FLAG_2D)
                {
                    planeConstraint.planeNormal2 = FmInitVector3(0.0f);
                }

                constraintParams.type = FM_SOLVER_CONSTRAINT_TYPE_3D;

                bool nonNeg0 = false;
                bool nonNeg1 = false;
                bool nonNeg2 = false;
                if (planeConstraint.flags & FM_CONSTRAINT_FLAG_NONNEG0)
                {
                    nonNeg0 = true;
                    constraintParams.flags |= FM_SOLVER_CONSTRAINT_FLAG_NONNEG0;
                }
                if (planeConstraint.flags & FM_CONSTRAINT_FLAG_NONNEG1)
                {
                    nonNeg1 = true;
                    constraintParams.flags |= FM_SOLVER_CONSTRAINT_FLAG_NONNEG1;
                }
                if (planeConstraint.flags & FM_CONSTRAINT_FLAG_NONNEG2)
                {
                    nonNeg2 = true;
                    constraintParams.flags |= FM_SOLVER_CONSTRAINT_FLAG_NONNEG2;
                }

                if (planeConstraint.objectIdA & FM_RB_FLAG)
                {
                    FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, planeConstraint.objectIdA);
                    uint stateOffset = FmGetRigidBodySolverOffsetById(*constraintSolverBuffer, planeConstraint.objectIdA);

                    submatOffset += FmAddJacobianRowSubmats_RigidBodyA(&Jvel_unconstrained, &jacobianSubmats[submatOffset], &jacobianIndices[submatOffset], stateOffset, rigidBody, planeConstraint);
                }
                else
                {
                    FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, planeConstraint.objectIdA);
                    FmMpcgSolverData& meshASolverData = *FmGetSolverDataById(*scene, planeConstraint.objectIdA);

                    submatOffset += FmAddJacobianRowSubmats_TetMeshA(&Jvel_unconstrained, &jacobianSubmats[submatOffset], &jacobianIndices[submatOffset], meshASolverData, tetMesh, planeConstraint);
                }

                if (planeConstraint.objectIdB != FM_INVALID_ID)  // FM_INVALID_ID means collision plane or fixed point in world
                {
                    if (planeConstraint.objectIdB & FM_RB_FLAG)
                    {
                        FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, planeConstraint.objectIdB);
                        uint stateOffset = FmGetRigidBodySolverOffsetById(*constraintSolverBuffer, planeConstraint.objectIdB);

                        submatOffset += FmAddJacobianRowSubmats_RigidBodyB(&Jvel_unconstrained, &jacobianSubmats[submatOffset], &jacobianIndices[submatOffset], stateOffset, rigidBody, planeConstraint);
                    }
                    else
                    {
                        FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, planeConstraint.objectIdB);
                        FmMpcgSolverData& meshBSolverData = *FmGetSolverDataById(*scene, planeConstraint.objectIdB);

                        submatOffset += FmAddJacobianRowSubmats_TetMeshB(&Jvel_unconstrained, &jacobianSubmats[submatOffset], &jacobianIndices[submatOffset], meshBSolverData, tetMesh, planeConstraint);
                    }
                }

                FM_ASSERT(submatOffset == FmGetNumJacobianSubmats(planeConstraint));

                constraintSolverData->pgsRhs[constraintIdx] = FmInitSVector3(0.0f);
                constraintSolverData->lambda3[constraintIdx] = FmInitSVector3(0.0f);

                float kCorrection = planeConstraint.kVelCorrection;

                float correction0 = planeConstraint.bias0 - planeConstraint.projection0;
                float correction1 = planeConstraint.bias1 - planeConstraint.projection1;
                float correction2 = planeConstraint.bias2 - planeConstraint.projection2;

                // For one-sided constraint, apply kCorrection only to positive correction
                float kCorrection0 = (nonNeg0 && correction0 < 0.0f) ? 1.0f : kCorrection;
                float kCorrection1 = (nonNeg1 && correction1 < 0.0f) ? 1.0f : kCorrection;
                float kCorrection2 = (nonNeg2 && correction2 < 0.0f) ? 1.0f : kCorrection;

                correctionVec = FmInitSVector3(
                    correction0 * kCorrection0, 
                    correction1 * kCorrection1, 
                    correction2 * kCorrection2) * delta_t_inv;

                constraintSolverData->lambda3Temp[constraintIdx] = correctionVec;
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_RIGID_BODY_ANGLE)
            {
                FmRigidBodyAngleConstraint& rigidBodyAngleConstraint = constraintsBuffer->rigidBodyAngleConstraints[constraintRef.idx];

                if (rigidBodyAngleConstraint.objectIdA == FM_INVALID_ID)
                {
                    continue;
                }

                constraintParams.type = FM_SOLVER_CONSTRAINT_TYPE_3D_JOINT1DFRICTION;
                constraintParams.frictionCoeff = rigidBodyAngleConstraint.frictionCoeff;

                {
                    FM_ASSERT(rigidBodyAngleConstraint.objectIdA & FM_RB_FLAG);

                    FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, rigidBodyAngleConstraint.objectIdA);
                    uint stateOffset = FmGetRigidBodySolverOffsetById(*constraintSolverBuffer, rigidBodyAngleConstraint.objectIdA);

                    submatOffset += FmAddJacobianRowSubmats_RigidBodyA(&Jvel_unconstrained, &jacobianSubmats[submatOffset], &jacobianIndices[submatOffset], stateOffset, rigidBody, rigidBodyAngleConstraint);
                }

                if (rigidBodyAngleConstraint.objectIdB != FM_INVALID_ID)  // FM_INVALID_ID means collision plane or fixed point in world
                {
                    FM_ASSERT(rigidBodyAngleConstraint.objectIdB & FM_RB_FLAG);

                    FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, rigidBodyAngleConstraint.objectIdB);
                    uint stateOffset = FmGetRigidBodySolverOffsetById(*constraintSolverBuffer, rigidBodyAngleConstraint.objectIdB);

                    submatOffset += FmAddJacobianRowSubmats_RigidBodyB(&Jvel_unconstrained, &jacobianSubmats[submatOffset], &jacobianIndices[submatOffset], stateOffset, rigidBody, rigidBodyAngleConstraint);
                }

                FM_ASSERT(submatOffset == FmGetNumJacobianSubmats(rigidBodyAngleConstraint));

                constraintSolverData->pgsRhs[constraintIdx] = FmInitSVector3(0.0f);
                constraintSolverData->lambda3[constraintIdx] = FmInitSVector3(0.0f);

                float kCorrection = rigidBodyAngleConstraint.kVelCorrection;

                correctionVec = FmInitSVector3(
                    -rigidBodyAngleConstraint.error0,
                    -rigidBodyAngleConstraint.error1,
                    -rigidBodyAngleConstraint.error2) * kCorrection * delta_t_inv;

                constraintSolverData->lambda3Temp[constraintIdx] = correctionVec;
            }
            else
            {
                FM_ASSERT(0);
            }

            // Init constant term of RHS for PGS iterations
            FmSVector3 pgsRhsConstant = correctionVec - Jvel_unconstrained;
            constraintSolverData->pgsRhsConstant[constraintIdx] = pgsRhsConstant;
            constraintSolverData->pgsRhs[constraintIdx] = pgsRhsConstant;

        }

        taskData->progress.TaskIsFinished();
    }

    FM_WRAPPED_TASK_FUNC(FmTaskFuncInitConstraintJacobianDiagonal)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE_INIT_CONSTRAINT_DATA);

        FmTaskDataInitSolverData* taskData = (FmTaskDataInitSolverData*)inTaskData;

        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;

        uint beginIdx, endIdx;
        FmGetIndexRangeEvenDistribution(&beginIdx, &endIdx, (uint)inTaskBeginIndex, taskData->numConstraintTasks, taskData->numConstraints);

        for (uint constraintIdx = beginIdx; constraintIdx < endIdx; constraintIdx++)
        {
            FmSMatrix3 diagBlock = FmDiagOfMxDiagxMT(constraintSolverData->J, constraintSolverData->DAinv, constraintIdx);

            float diagX = diagBlock.col0.x;
            float diagY = diagBlock.col1.y;
            float diagZ = diagBlock.col2.z;

            // Degeneracy can occur for contact with constrained vert
            const float tol = FLT_EPSILON;
            float diagXInv = (fabsf(diagX) < tol) ? 1.0f : 1.0f / diagX;
            float diagYInv = (fabsf(diagY) < tol) ? 1.0f : 1.0f / diagY;
            float diagZInv = (fabsf(diagZ) < tol) ? 1.0f : 1.0f / diagZ;

            constraintSolverData->J.params[constraintIdx].diagInverse[1] = FmInitSVector3(diagXInv, diagYInv, diagZInv);

            diagBlock = FmDiagOfMxDiagxMT(constraintSolverData->J, constraintSolverData->W, constraintIdx);

            diagX = diagBlock.getCol0().x;
            diagY = diagBlock.getCol1().y;
            diagZ = diagBlock.getCol2().z;

            diagXInv = (fabsf(diagX) < tol) ? 1.0f : 1.0f / diagX;
            diagYInv = (fabsf(diagY) < tol) ? 1.0f : 1.0f / diagY;
            diagZInv = (fabsf(diagZ) < tol) ? 1.0f : 1.0f / diagZ;

            constraintSolverData->J.params[constraintIdx].diagInverse[0] = FmInitSVector3(diagXInv, diagYInv, diagZInv);

        }

        taskData->progress.TaskIsFinished(taskData);
    }

    void FmTaskFuncInitConstraintArrayDataForStabilization(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE_INIT_CONSTRAINT_DATA);

        FmTaskDataInitSolverData* taskData = (FmTaskDataInitSolverData*)inTaskData;

        FmScene* scene = taskData->scene;
        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;

        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        FmConstraintIsland& constraintIsland = *taskData->constraintIsland;

        FmConstraintSolverControlParams& controlParams = *constraintSolverData->currentControlParams;

        uint beginIdx, endIdx;
        FmGetIndexRangeEvenDistribution(&beginIdx, &endIdx, (uint)inTaskBeginIndex, taskData->numConstraintTasks, taskData->numConstraints);

        // Accumulate contact data into constraint Jacobians.
        for (uint constraintIdx = beginIdx; constraintIdx < endIdx; constraintIdx++)
        {
            FmConstraintReference& constraintRef = constraintIsland.constraintRefs[constraintIdx];
            FmConstraintParams& constraintParams = constraintSolverData->J.params[constraintIdx];

            if (constraintRef.type == FM_CONSTRAINT_TYPE_DISTANCE_CONTACT)
            {
                FmDistanceContact& contact = constraintsBuffer->distanceContacts[constraintRef.idx];

                constraintParams.frictionCoeff = 0.0f;

                constraintSolverData->pgsRhs[constraintIdx] = FmInitSVector3(0.0f);
                constraintSolverData->lambda3[constraintIdx] = FmInitSVector3(0.0f);

                float kCorrection = controlParams.kDistanceCorrection;

                float normalError = -contact.normalProjDistance;
                kCorrection = (normalError < 0.0f) ? 1.0f : kCorrection;

                float normalCorrection = normalError * kCorrection;

                constraintSolverData->lambda3Temp[constraintIdx] = FmInitSVector3(normalCorrection, 0.0f, 0.0f);
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_VOLUME_CONTACT)
            {
                FmVolumeContact& contact = constraintsBuffer->volumeContacts[constraintRef.idx];

                constraintParams.frictionCoeff = 0.0f;

                constraintSolverData->pgsRhs[constraintIdx] = FmInitSVector3(0.0f);
                constraintSolverData->lambda3[constraintIdx] = FmInitSVector3(0.0f);

                float kCorrection = controlParams.kVolumeCorrection;

                float volumeError = -contact.V;
                kCorrection = (volumeError < 0.0f) ? 1.0f : kCorrection;

                float volumeCorrection = volumeError * kCorrection;

                constraintSolverData->lambda3Temp[constraintIdx] = FmInitSVector3(volumeCorrection, 0.0f, 0.0f);
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_DEFORMATION)
            {
                FmDeformationConstraint& constraint = constraintsBuffer->deformationConstraints[constraintRef.idx];

                constraintSolverData->pgsRhs[constraintIdx] = FmInitSVector3(0.0f);
                constraintSolverData->lambda3[constraintIdx] = FmInitSVector3(0.0f);

                float kCorrection = constraint.kPosCorrection;

                // For one-sided constraint, apply kCorrection to positive correction
                float kCorrectionX = constraint.deformationDeltas.x > 0.0f ? 1.0f : kCorrection;
                float kCorrectionY = constraint.deformationDeltas.y > 0.0f ? 1.0f : kCorrection;
                float kCorrectionZ = constraint.deformationDeltas.z > 0.0f ? 1.0f : kCorrection;

                constraintSolverData->lambda3Temp[constraintIdx] = -FmInitSVector3(
                    constraint.deformationDeltas.x * kCorrectionX,
                    constraint.deformationDeltas.y * kCorrectionY,
                    constraint.deformationDeltas.z * kCorrectionZ);
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_GLUE)
            {
                FmGlueConstraint& constraint = constraintsBuffer->glueConstraints[constraintRef.idx];

                if (constraint.objectIdA == FM_INVALID_ID)
                {
                    continue;
                }

                constraintSolverData->pgsRhs[constraintIdx] = FmInitSVector3(0.0f);
                constraintSolverData->lambda3[constraintIdx] = FmInitSVector3(0.0f);

                float kCorrection = constraint.kPosCorrection;

                constraintSolverData->lambda3Temp[constraintIdx] = -FmInitSVector3(constraint.deltaPos) * kCorrection;
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_PLANE)
            {
                FmPlaneConstraint& constraint = constraintsBuffer->planeConstraints[constraintRef.idx];

                if (constraint.objectIdA == FM_INVALID_ID)
                {
                    continue;
                }

                constraintSolverData->pgsRhs[constraintIdx] = FmInitSVector3(0.0f);
                constraintSolverData->lambda3[constraintIdx] = FmInitSVector3(0.0f);

                float kCorrection = constraint.kPosCorrection;

                // For one-sided constraint, apply kCorrection only to positive correction
                bool nonNeg0 = (constraintParams.flags & FM_SOLVER_CONSTRAINT_FLAG_NONNEG0) != 0;
                bool nonNeg1 = (constraintParams.flags & FM_SOLVER_CONSTRAINT_FLAG_NONNEG1) != 0;
                bool nonNeg2 = (constraintParams.flags & FM_SOLVER_CONSTRAINT_FLAG_NONNEG2) != 0;

                float correction0 = constraint.bias0 - constraint.projection0;
                float correction1 = constraint.bias1 - constraint.projection1;
                float correction2 = constraint.bias2 - constraint.projection2;

                float kCorrection0 = (nonNeg0 && correction0 < 0.0f) ? 1.0f : kCorrection;
                float kCorrection1 = (nonNeg1 && correction1 < 0.0f) ? 1.0f : kCorrection;
                float kCorrection2 = (nonNeg2 && correction2 < 0.0f) ? 1.0f : kCorrection;

                constraintSolverData->lambda3Temp[constraintIdx] = FmInitSVector3(
                    correction0 * kCorrection0,
                    correction1 * kCorrection1,
                    correction2 * kCorrection2);
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_RIGID_BODY_ANGLE)
            {
                FmRigidBodyAngleConstraint& constraint = constraintsBuffer->rigidBodyAngleConstraints[constraintRef.idx];

                if (constraint.objectIdA == FM_INVALID_ID)
                {
                    continue;
                }

                constraintParams.frictionCoeff = 0.0f;

                constraintSolverData->pgsRhs[constraintIdx] = FmInitSVector3(0.0f);
                constraintSolverData->lambda3[constraintIdx] = FmInitSVector3(0.0f);

                float kCorrection = constraint.kPosCorrection;

                constraintSolverData->lambda3Temp[constraintIdx] =
                    FmInitSVector3(
                        -constraint.error0,
                        -constraint.error1,
                        -constraint.error2) * kCorrection;
            }
        }

        taskData->progress.TaskIsFinished();
    }

    FM_WRAPPED_TASK_FUNC(FmTaskFuncInitConstraintPgsConstantForStabilization)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE_INIT_CONSTRAINT_DATA);

        FmTaskDataInitSolverData* taskData = (FmTaskDataInitSolverData*)inTaskData;

        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;

        uint beginIdx, endIdx;
        FmGetIndexRangeEvenDistribution(&beginIdx, &endIdx, (uint)inTaskBeginIndex, taskData->numConstraintTasks, taskData->numConstraints);

        for (uint constraintIdx = beginIdx; constraintIdx < endIdx; constraintIdx++)
        {
            // Initialize constant term of PGS right-hand-side
            FmSVector3 pgsRhsConstant = FmVmMxV_row(constraintSolverData->pgsRhsConstant, constraintSolverData->lambda3Temp, constraintSolverData->J, constraintSolverData->velTemp, constraintIdx);

            // PGS right-hand-side initially is the constant part assuming deltaVel or deltaPos initially zero
            constraintSolverData->pgsRhs[constraintIdx] = pgsRhsConstant;
        }

        taskData->progress.TaskIsFinished(taskData);
    }

    FM_WRAPPED_TASK_FUNC(FmTaskFuncInitConstraintSolverData)
    {
        (void)inTaskEndIndex;
        FmTaskDataInitSolverData* taskData = (FmTaskDataInitSolverData*)inTaskData;

        int32_t taskBeginIndex = inTaskBeginIndex;
        int32_t taskEndIndex = inTaskEndIndex;

        if (taskBeginIndex < (int32_t)taskData->numTetMeshes)
        {
            FmTaskFuncInitMeshSolverArrayData(inTaskData, taskBeginIndex, taskEndIndex);
        }
        else if (taskBeginIndex < (int32_t)(taskData->numTetMeshes + taskData->numRigidBodyTasks))
        {
            taskBeginIndex -= (int32_t)taskData->numTetMeshes;
            FmTaskFuncInitRbSolverArrayData(inTaskData, taskBeginIndex, taskBeginIndex + 1);
        }
        else
        {
            taskBeginIndex -= (int32_t)(taskData->numTetMeshes + taskData->numRigidBodyTasks);
            FmTaskFuncInitConstraintArrayData(inTaskData, taskBeginIndex, taskBeginIndex + 1);
        }
    }

#if FM_CONSTRAINT_STABILIZATION_SOLVE
    FM_WRAPPED_TASK_FUNC(FmTaskFuncInitConstraintSolverDataForStabilization)
    {
        (void)inTaskEndIndex;
        FmTaskDataInitSolverData* taskData = (FmTaskDataInitSolverData*)inTaskData;

        int32_t taskBeginIndex = inTaskBeginIndex;
        int32_t taskEndIndex = inTaskEndIndex;

        if (taskBeginIndex < (int32_t)taskData->numTetMeshes)
        {
            FmTaskFuncInitMeshSolverArrayDataForStabilization(inTaskData, taskBeginIndex, taskEndIndex);
        }
        else if (taskBeginIndex < (int32_t)(taskData->numTetMeshes + taskData->numRigidBodyTasks))
        {
            taskBeginIndex -= (int32_t)taskData->numTetMeshes;
            FmTaskFuncInitRbSolverArrayDataForStabilization(inTaskData, taskBeginIndex, taskBeginIndex + 1);
        }
        else
        {
            taskBeginIndex -= (int32_t)(taskData->numTetMeshes + taskData->numRigidBodyTasks);
            FmTaskFuncInitConstraintArrayDataForStabilization(inTaskData, taskBeginIndex, taskBeginIndex + 1);
        }
    }
#endif

    void FmTaskFuncSetupConstraintSolveEnd(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    int32_t FmBatchingFuncInitConstraintSolverData(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        FmTaskDataInitSolverData* taskData = (FmTaskDataInitSolverData*)inTaskData;

        FmScene* scene = taskData->scene;
        FmConstraintIsland* constraintIsland = taskData->constraintIsland;
        uint numTetMeshes = taskData->numTetMeshes;

        const uint minVerts = 256;

        uint numVerts = 0;
        uint objectIndex = (uint)inTaskBeginIndex;
        uint taskBeginIndex = (uint)inTaskBeginIndex;
        uint taskEndIndex = (uint)inTaskEndIndex;

        if (objectIndex < numTetMeshes)
        {
            while (numVerts < minVerts
                && objectIndex < taskEndIndex
                && objectIndex < numTetMeshes)
            {
                uint islandObjectId = constraintIsland->tetMeshIds[objectIndex];

                FmMpcgSolverData& meshData = *FmGetSolverDataById(*scene, islandObjectId);
                numVerts += meshData.A.numRows;
                objectIndex++;
            }

            return objectIndex - taskBeginIndex;
        }
        else
        {
            return 1;
        }
    }

    void FmSetupConstraintSolve(
        FmScene* scene,
        FmConstraintSolverData* constraintSolverData,
        FmConstraintIsland* constraintIsland,
        float delta_t,
        FmTaskFuncCallback followTaskFunc,
        void* followTaskData)
    {
        FM_ASSERT(constraintSolverData->isAllocated);

        FmConstraintSolverBuffer* constraintSolverBuffer = scene->constraintSolverBuffer;

        constraintSolverData->constraintSolverBuffer = constraintSolverBuffer;
        constraintSolverData->constraintIsland = constraintIsland;

        uint numTetMeshes = constraintIsland->numTetMeshes;
        uint numRigidBodies = constraintIsland->numRigidBodiesInFEMSolve;

        // Initialize maps from object id to partition data indices
        uint solverObjectIdx = 0;
        for (uint islandMeshIdx = 0; islandMeshIdx < numTetMeshes; islandMeshIdx++)
        {
            constraintSolverBuffer->tetMeshIdxFromId[constraintIsland->tetMeshIds[islandMeshIdx]] = solverObjectIdx;
            solverObjectIdx++;
        }
        for (uint islandRigidBodyIdx = 0; islandRigidBodyIdx < numRigidBodies; islandRigidBodyIdx++)
        {
            constraintSolverBuffer->rigidBodyIdxFromId[constraintIsland->rigidBodyIds[islandRigidBodyIdx] & ~FM_RB_FLAG] = solverObjectIdx;
            solverObjectIdx++;
        }

        // Preprocess the constraint island by partitioning the objects/constraints and setting up dependencies:

        // Create partitions of object and constraints, and sort objects by partition
        FmBuildPartitionsHierarchy(scene, &constraintSolverData->partitionsHierarchy,
            constraintSolverData->nodeCounts,
            constraintSolverData,
            *constraintIsland);

#if FM_ASYNC_THREADING
        FmSetupConstraintSolveTaskData* setupTaskData = new FmSetupConstraintSolveTaskData(scene, constraintSolverData, constraintIsland, delta_t, followTaskFunc, followTaskData);

        FmCreatePartitions(scene, constraintSolverData, constraintIsland, setupTaskData);
#else
        uint numConstraints = constraintIsland->numConstraints;
        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;

        FmCreatePartitions(scene, constraintSolverData, constraintIsland, NULL);
        FmCreatePartitionPairs(scene, constraintSolverData, constraintIsland);

        FmGraphColorPartitionPairs(constraintSolverData);

#if FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH
        FmCreateConstraintSolveTaskGraph(scene, constraintSolverData, constraintIsland);
#endif

        // Configure settings for solve; default to original GS-based method
        constraintSolverData->currentPassIdx = 0;
        constraintSolverData->currentControlParams = &constraintSolverData->solveParams;
        constraintSolverData->currentControlParams->passParams[0].currentMaxInnerIterations = constraintSolverData->currentControlParams->passParams[0].maxInnerIterations;
        constraintSolverData->currentControlParams->passParams[1].currentMaxInnerIterations = constraintSolverData->currentControlParams->passParams[1].maxInnerIterations;

        constraintSolverData->iterationParams.outerIteration = 0;
        constraintSolverData->iterationParams.outerIsLastIteration = false;
        constraintSolverData->iterationParams.outerForwardDirection = true;
        constraintSolverData->iterationParams.innerForwardDirection = true;

        constraintSolverData->numTetMeshVerts = constraintIsland->numTetMeshVerts;
        constraintSolverData->numStateVecs3 = constraintIsland->numStateVecs3;
        constraintSolverData->numConstraints = constraintIsland->numConstraints;

        // Compute base offsets in solver state arrays for each object
        uint solverStateOffset = 0;
        for (uint islandMeshIdx = 0; islandMeshIdx < numTetMeshes; islandMeshIdx++)
        {
            FmMpcgSolverData& meshMpcgData = *FmGetSolverDataById(*scene, constraintIsland->tetMeshIds[islandMeshIdx]);
            meshMpcgData.solverStateOffset = solverStateOffset;
            uint numMeshVerts = meshMpcgData.A.numRows;
            solverStateOffset += numMeshVerts;
        }

        FM_ASSERT(solverStateOffset == constraintIsland->numTetMeshVerts);

        for (uint islandRbIdx = 0; islandRbIdx < numRigidBodies; islandRbIdx++)
        {
            uint rbId = constraintIsland->rigidBodyIds[islandRbIdx];

            FmSetRigidBodySolverOffsetById(constraintSolverBuffer, rbId, solverStateOffset);

            solverStateOffset += 2;
        }

        // Resize the jacobian and set constraint offsets
        FmResizeConstraintJacobian(constraintSolverData, constraintsBuffer, *constraintIsland);

        uint rigidBodyBatchSize = FM_INIT_RIGID_BODY_ARRAYS_BATCH_SIZE;
        uint numRigidBodyTasks = FmGetNumTasksMinBatchSize(numRigidBodies, rigidBodyBatchSize);

        uint constraintsBatchSize = FM_INIT_CONSTRAINTS_BATCH_SIZE;
        uint numConstraintTasks = FmGetNumTasksMinBatchSize(numConstraints, constraintsBatchSize);

        uint numTasks = numTetMeshes + numRigidBodyTasks + numConstraintTasks;

        FmTaskDataInitSolverData taskDataInitSolverData(scene, constraintSolverData, constraintIsland, delta_t,
            numTetMeshes,
            numRigidBodies, numRigidBodyTasks,
            numConstraints, numConstraintTasks);

        scene->taskSystemCallbacks.ParallelFor("InitConstraintSolverData", FmTaskFuncInitConstraintSolverData, &taskDataInitSolverData, (int32_t)numTasks);
        scene->taskSystemCallbacks.ParallelFor("InitConstraintArrayData", FmTaskFuncInitConstraintJacobianDiagonal, &taskDataInitSolverData, (int32_t)numConstraintTasks);
#endif
    }

#if FM_ASYNC_THREADING
    void FmSetupConstraintSolvePostSort(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;

        FmSetupConstraintSolveTaskData* taskData = (FmSetupConstraintSolveTaskData*)inTaskData;

        FmScene* scene = taskData->scene;
        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        FmConstraintIsland* constraintIsland = taskData->constraintIsland;
        float delta_t = taskData->delta_t;
        FmTaskFuncCallback followTaskFunc = taskData->followTaskFunc;
        void* followTaskData = taskData->followTaskData;

        // Can delete sort task graph and consumed task data
        delete taskData->sortTaskGraph;
        delete taskData;
        taskData = NULL;

        FmConstraintSolverBuffer* constraintSolverBuffer = scene->constraintSolverBuffer;
        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;

        uint numConstraints = constraintIsland->numConstraints;
        uint numTetMeshes = constraintIsland->numTetMeshes;
        uint numRigidBodies = constraintIsland->numRigidBodiesInFEMSolve;

        FmCreatePartitionPairs(scene, constraintSolverData, constraintIsland);

        FmGraphColorPartitionPairs(constraintSolverData);

#if FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH
        FmCreateConstraintSolveTaskGraph(scene, constraintSolverData, constraintIsland);
#endif

        // Configure settings for solve; default to original GS-based method
        constraintSolverData->currentPassIdx = 0;
        constraintSolverData->currentControlParams = &constraintSolverData->solveParams;
        constraintSolverData->currentControlParams->passParams[0].currentMaxInnerIterations = constraintSolverData->currentControlParams->passParams[0].maxInnerIterations;
        constraintSolverData->currentControlParams->passParams[1].currentMaxInnerIterations = constraintSolverData->currentControlParams->passParams[1].maxInnerIterations;

        constraintSolverData->iterationParams.outerIteration = 0;
        constraintSolverData->iterationParams.outerIsLastIteration = false;
        constraintSolverData->iterationParams.outerForwardDirection = true;
        constraintSolverData->iterationParams.innerForwardDirection = true;

        constraintSolverData->numTetMeshVerts = constraintIsland->numTetMeshVerts;
        constraintSolverData->numStateVecs3 = constraintIsland->numStateVecs3;
        constraintSolverData->numConstraints = constraintIsland->numConstraints;

        // Compute base offsets in solver state arrays for each object
        uint solverStateOffset = 0;
        for (uint islandMeshIdx = 0; islandMeshIdx < numTetMeshes; islandMeshIdx++)
        {
            FmMpcgSolverData& meshMpcgData = *FmGetSolverDataById(*scene, constraintIsland->tetMeshIds[islandMeshIdx]);
            meshMpcgData.solverStateOffset = solverStateOffset;
            uint numMeshVerts = meshMpcgData.A.numRows;
            solverStateOffset += numMeshVerts;
        }

        FM_ASSERT(solverStateOffset == constraintIsland->numTetMeshVerts);

        for (uint islandRbIdx = 0; islandRbIdx < numRigidBodies; islandRbIdx++)
        {
            uint rbId = constraintIsland->rigidBodyIds[islandRbIdx];

            FmSetRigidBodySolverOffsetById(constraintSolverBuffer, rbId, solverStateOffset);

            solverStateOffset += 2;
        }

        // Resize the jacobian and set constraint offsets
        FmResizeConstraintJacobian(constraintSolverData, constraintsBuffer, *constraintIsland);

        uint rigidBodyBatchSize = FM_INIT_RIGID_BODY_ARRAYS_BATCH_SIZE;
        uint numRigidBodyTasks = FmGetNumTasksMinBatchSize(numRigidBodies, rigidBodyBatchSize);

        uint constraintsBatchSize = FM_INIT_CONSTRAINTS_BATCH_SIZE;
        uint numConstraintTasks = FmGetNumTasksMinBatchSize(numConstraints, constraintsBatchSize);

        uint numTasks = numTetMeshes + numRigidBodyTasks + numConstraintTasks;

        if (followTaskFunc)
        {
            FmTaskDataInitSolverData* taskDataInitSolverData = new FmTaskDataInitSolverData(scene, constraintSolverData, constraintIsland, delta_t,
                numTetMeshes,
                numRigidBodies, numRigidBodyTasks,
                numConstraints, numConstraintTasks);

            taskDataInitSolverData->followTask.func = followTaskFunc;
            taskDataInitSolverData->followTask.data = followTaskData;
            taskDataInitSolverData->progress.Init(numTasks, FmTaskFuncSetupConstraintSolveEnd, taskDataInitSolverData);

            FmParallelForAsync("InitConstraintSolverData", FM_TASK_AND_WRAPPED_TASK_ARGS(FmTaskFuncInitConstraintSolverData), 
                FmBatchingFuncInitConstraintSolverData,
                taskDataInitSolverData, numTasks, scene->taskSystemCallbacks.SubmitAsyncTask, scene->params.numThreads);
        }
    }

    void FmTaskFuncSetupConstraintSolveEnd(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;

        FmTaskDataInitSolverData* taskData = (FmTaskDataInitSolverData*)inTaskData;
        FmScene* scene = taskData->scene;
        uint numConstraintTasks = taskData->numConstraintTasks;

        // Call last parallel-for to set up constraint solve, which will call next task in
        // island solve.  Next task will get the island id from the task index.
        int32_t taskIndex = (int32_t)taskData->constraintIsland->islandId;
        taskData->progress.Init(numConstraintTasks, taskData->followTask.func, taskData->followTask.data, taskIndex, taskIndex + 1);

        FmParallelForAsync("InitConstraintArrayData", FM_TASK_AND_WRAPPED_TASK_ARGS(FmTaskFuncInitConstraintJacobianDiagonal), NULL, taskData, (int32_t)numConstraintTasks, scene->taskSystemCallbacks.SubmitAsyncTask, scene->params.numThreads);
    }
#endif

#if FM_CONSTRAINT_STABILIZATION_SOLVE
    void FmTaskFuncSetupConstraintStabilizationEnd(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    // Set up a post-stabilization solve.
    // Reference: Cline and Pai, "Post-stabilization for Rigid Body Simulation with Contact and Constraints"
    void FmSetupConstraintStabilization(
        FmScene* scene,
        FmConstraintSolverData* constraintSolverData,
        FmConstraintIsland* constraintIsland,
        float delta_t,
        FmTaskFuncCallback followTaskFunc,
        void* followTaskData)
    {
        // Configure settings for solve; default to original GS-based method
        constraintSolverData->currentControlParams = &constraintSolverData->stabilizationParams;
        constraintSolverData->currentPassIdx = 0;
        constraintSolverData->currentControlParams->passParams[0].currentMaxInnerIterations = constraintSolverData->currentControlParams->passParams[0].maxInnerIterations;
        constraintSolverData->currentControlParams->passParams[1].currentMaxInnerIterations = constraintSolverData->currentControlParams->passParams[1].maxInnerIterations;

        constraintSolverData->iterationParams.outerIteration = 0;
        constraintSolverData->iterationParams.outerIsLastIteration = false;
        constraintSolverData->iterationParams.outerForwardDirection = true;
        constraintSolverData->iterationParams.innerForwardDirection = true;

        uint numConstraints = constraintIsland->numConstraints;
        uint numTetMeshes = constraintIsland->numTetMeshes;
        uint numRigidBodies = constraintIsland->numRigidBodiesInFEMSolve;

        constraintSolverData->numTetMeshVerts = constraintIsland->numTetMeshVerts;
        constraintSolverData->numStateVecs3 = constraintIsland->numStateVecs3;
        constraintSolverData->numConstraints = constraintIsland->numConstraints;

        uint rigidBodyBatchSize = FM_INIT_RIGID_BODY_ARRAYS_BATCH_SIZE;
        uint numRigidBodyTasks = FmGetNumTasksMinBatchSize(numRigidBodies, rigidBodyBatchSize);

        uint constraintsBatchSize = FM_INIT_CONSTRAINTS_BATCH_SIZE;
        uint numConstraintTasks = FmGetNumTasksMinBatchSize(numConstraints, constraintsBatchSize);

        uint numTasks = numTetMeshes + numRigidBodyTasks + numConstraintTasks;

#if FM_ASYNC_THREADING
        if (followTaskFunc)
        {
            FmTaskDataInitSolverData* taskDataInitSolverData = new FmTaskDataInitSolverData(scene, constraintSolverData, constraintIsland, delta_t,
                numTetMeshes,
                numRigidBodies, numRigidBodyTasks,
                numConstraints, numConstraintTasks);

            taskDataInitSolverData->followTask.func = followTaskFunc;
            taskDataInitSolverData->followTask.data = followTaskData;
            taskDataInitSolverData->progress.Init(numTasks, FmTaskFuncSetupConstraintStabilizationEnd, taskDataInitSolverData);

            FmParallelForAsync("InitConstraintSolverDataForStabilization", FM_TASK_AND_WRAPPED_TASK_ARGS(FmTaskFuncInitConstraintSolverDataForStabilization), 
                FmBatchingFuncInitConstraintSolverData,
                taskDataInitSolverData, (int32_t)numTasks, scene->taskSystemCallbacks.SubmitAsyncTask, scene->params.numThreads);
        }
#else
        FmTaskDataInitSolverData taskDataInitSolverData(scene, constraintSolverData, constraintIsland, delta_t,
            numTetMeshes,
            numRigidBodies, numRigidBodyTasks,
            numConstraints, numConstraintTasks);

        scene->taskSystemCallbacks.ParallelFor("InitConstraintSolverDataForStabilization", FmTaskFuncInitConstraintSolverDataForStabilization, &taskDataInitSolverData, (int32_t)numTasks);
        scene->taskSystemCallbacks.ParallelFor("InitConstraintPgsConstantForStabilization", FmTaskFuncInitConstraintPgsConstantForStabilization, &taskDataInitSolverData, (int32_t)numConstraintTasks);
#endif
    }

#if FM_ASYNC_THREADING
    void FmTaskFuncSetupConstraintStabilizationEnd(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;

        FmTaskDataInitSolverData* taskData = (FmTaskDataInitSolverData*)inTaskData;
        FmScene* scene = taskData->scene;
        uint numConstraintTasks = taskData->numConstraintTasks;

        // Call last parallel-for to set up constraint solve, which will call next task in
        // island solve.  Next task will get the island id from the task index.
        int32_t taskIndex = (int32_t)taskData->constraintIsland->islandId;
        taskData->progress.Init(numConstraintTasks, taskData->followTask.func, taskData->followTask.data, taskIndex, taskIndex + 1);

        FmParallelForAsync("InitConstraintPgsConstantForStabilization", FM_TASK_AND_WRAPPED_TASK_ARGS(FmTaskFuncInitConstraintPgsConstantForStabilization), NULL, taskData, (int32_t)numConstraintTasks, scene->taskSystemCallbacks.SubmitAsyncTask, scene->params.numThreads);
    }
#endif
#endif

    void FmShutdownConstraintSolve(FmConstraintSolverData* constraintSolverData)
    {
        if (constraintSolverData->pDynamicAllocatedBuffer)
        {
            FmAlignedFree(constraintSolverData->pDynamicAllocatedBuffer);
            constraintSolverData->pDynamicAllocatedBuffer = NULL;
        }
#if FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH
        if (constraintSolverData->taskGraph)
        {
            FmDestroyConstraintSolveTaskGraph(constraintSolverData->taskGraph);
            constraintSolverData->taskGraph = NULL;
        }
#else
        (void)constraintSolverData;
#endif
    }
}