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
// Tetrahedron-specific types and operations for the FEM model
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXCommonInternal.h"
#include "FEMFXVectorMath.h"
#include "FEMFXSvd3x3.h"

namespace AMD
{
    // Parameters that describe tetrahedron shape and mapping from position to barycentric values.
    // Strain matrix is easily computed from these parameters.
    struct FmTetShapeParams
    {
        FmMatrix4 baryMatrix;   // Matrix that transforms a point within the tet to its barycentric coords
        float det;                 // Determinant of matrix m = [[1, 1, 1, 1], [x0, x1, x2, x3], [y0, y1, y2, y3], [z0, z1, z2, z3]] = 6 * volume

        inline float GetA0() const { return baryMatrix.col0.x; }
        inline float GetA1() const { return baryMatrix.col0.y; }
        inline float GetA2() const { return baryMatrix.col0.z; }
        inline float GetA3() const { return baryMatrix.col0.w; }

        inline float GetB0() const { return baryMatrix.col1.x; }
        inline float GetB1() const { return baryMatrix.col1.y; }
        inline float GetB2() const { return baryMatrix.col1.z; }
        inline float GetB3() const { return baryMatrix.col1.w; }

        inline float GetC0() const { return baryMatrix.col2.x; }
        inline float GetC1() const { return baryMatrix.col2.y; }
        inline float GetC2() const { return baryMatrix.col2.z; }
        inline float GetC3() const { return baryMatrix.col2.w; }

        inline float GetArea0() const { return 0.5f*det*sqrtf(GetA0()*GetA0() + GetB0()*GetB0() + GetC0()*GetC0()); }
        inline float GetArea1() const { return 0.5f*det*sqrtf(GetA1()*GetA1() + GetB1()*GetB1() + GetC1()*GetC1()); }
        inline float GetArea2() const { return 0.5f*det*sqrtf(GetA2()*GetA2() + GetB2()*GetB2() + GetC2()*GetC2()); }
        inline float GetArea3() const { return 0.5f*det*sqrtf(GetA3()*GetA3() + GetB3()*GetB3() + GetC3()*GetC3()); }
        inline FmVector3 GetNormal0() const { return -normalize(FmInitVector3(GetA0(), GetB0(), GetC0())); }
        inline FmVector3 GetNormal1() const { return -normalize(FmInitVector3(GetA1(), GetB1(), GetC1())); }
        inline FmVector3 GetNormal2() const { return -normalize(FmInitVector3(GetA2(), GetB2(), GetC2())); }
        inline FmVector3 GetNormal3() const { return -normalize(FmInitVector3(GetA3(), GetB3(), GetC3())); }
        inline float GetVolume() const { return det * (1.0f / 6.0f); }

        inline void ComputeRestPositions(FmVector3* outRestPosition0, FmVector3* outRestPosition1, FmVector3* outRestPosition2, FmVector3* outRestPosition3) const
        {
            FmMatrix4 restPosMat = inverse(baryMatrix);

            FmVector3 restPosition0 = restPosMat.col0.getXYZ();
            FmVector3 restPosition1 = restPosMat.col1.getXYZ();
            FmVector3 restPosition2 = restPosMat.col2.getXYZ();
            FmVector3 restPosition3 = restPosMat.col3.getXYZ();

            *outRestPosition0 = restPosition0 - restPosition3;
            *outRestPosition1 = restPosition1 - restPosition3;
            *outRestPosition2 = restPosition2 - restPosition3;
            *outRestPosition3 = FmInitVector3(0.0f);
        }
    };

    // Strain matrix, relating position offsets of tet vertices to strain
    struct FmTetStrainMatrix
    {
        FmMatrix3 Be00;   // 6 x 12 strain matrix
        FmMatrix3 Be01;
        FmMatrix3 Be02;
        FmMatrix3 Be03;
        FmMatrix3 Be10;
        FmMatrix3 Be11;
        FmMatrix3 Be12;
        FmMatrix3 Be13;
    };

    // Stress matrix, relating position offsets of tet vertices to stress
    struct FmTetStressMatrix
    {
        FmMatrix3 EBe00;  // E . Be which is 6 x 12 stress matrix
        FmMatrix3 EBe01;
        FmMatrix3 EBe02;
        FmMatrix3 EBe03;
        FmMatrix3 EBe10;
        FmMatrix3 EBe11;
        FmMatrix3 EBe12;
        FmMatrix3 EBe13;
    };

    // Stiffness matrix, relating position offsets of tet vertices to internal force
    struct FmTetStiffnessMatrix
    {
        FmMatrix3 Ke_diag0;   // Symmetric 12 x 12 stiffness matrix, submatrices on diagonal
        FmMatrix3 Ke_diag1;   
        FmMatrix3 Ke_diag2;   
        FmMatrix3 Ke_diag3;   
        FmMatrix3 Ke_lower0;  // Symmetric 12 x 12 stiffness matrix, submatrices below diagonal
        FmMatrix3 Ke_lower1;
        FmMatrix3 Ke_lower2;
        FmMatrix3 Ke_lower3;
        FmMatrix3 Ke_lower4;
        FmMatrix3 Ke_lower5;
    };

    // Rotated stiffness matrix.
    // Matrix combines steps of un-rotating deformed element positions, applying stiffness, and rotating forces back.
    // Reference: Muller and Gross, "Interactive Virtual Materials"
    struct FmTetRotatedStiffnessMatrix
    {
        FmMatrix3 Keprime_diag0;    // Submatrices on diagonal
        FmMatrix3 Keprime_diag1;
        FmMatrix3 Keprime_diag2;
        FmMatrix3 Keprime_diag3;
        FmMatrix3 Keprime_lower0;   // Submatrices below diagonal
        FmMatrix3 Keprime_lower1;
        FmMatrix3 Keprime_lower2;
        FmMatrix3 Keprime_lower3;
        FmMatrix3 Keprime_lower4;
        FmMatrix3 Keprime_lower5;
        FmVector3 f0eprime0;        // Term of the internal force expression containing the rest positions
        FmVector3 f0eprime1; 
        FmVector3 f0eprime2; 
        FmVector3 f0eprime3; 
    };

    // Compute quantities associated with shape of tet in rest position.
    void FmComputeShapeParams(FmTetShapeParams* shapeParams, const FmVector3& p0, const FmVector3& p1, const FmVector3& p2, const FmVector3& p3);

    // Compute strain matrix from shape parameters
    void FmComputeTetStrainMatrix(FmTetStrainMatrix* strainMat, const FmTetShapeParams& shapeParams);

    // Compute strain from shape parameters and tet position offsets
    void FmComputeTetStrain(
        FmVector3 strain[2],
        const FmTetShapeParams& shapeParams,
        const FmVector3& unrotatedOffset0,
        const FmVector3& unrotatedOffset1,
        const FmVector3& unrotatedOffset2,
        const FmVector3& unrotatedOffset3);

    // Compute stress matrix from shape and material parameters
    void FmComputeTetStressMatrix(
        FmTetStressMatrix* stressMat,
        const FmVector4& baryMatrixCol0,
        const FmVector4& baryMatrixCol1,
        const FmVector4& baryMatrixCol2,
        float youngsModulus,
        float poissonsRatio);

    // Compute stress and stiffness matrix from shape and material parameters
    void FmComputeTetStressAndStiffnessMatrix(
        FmTetStressMatrix* stressMat,
        FmTetStiffnessMatrix* stiffnessMat,
        const FmVector4& baryMatrixCol0,
        const FmVector4& baryMatrixCol1,
        const FmVector4& baryMatrixCol2,
        float volume,
        float youngsModulus,
        float poissonsRatio);

    // Extract rotation from the transform of rest positions to deformed positions, using polar decomposition
    FmMatrix3 FmComputeTetRotationPolarDecomp(
        const FmVector3& tetDeformedPosition0,
        const FmVector3& tetDeformedPosition1,
        const FmVector3& tetDeformedPosition2,
        const FmVector3& tetDeformedPosition3,
        const FmMatrix4& tetRestBaryMatrix);

    // Extract rotation from the transform of rest positions to deformed positions, using polar decomposition computed by SVD.
    // Robust in cases of near singularity or inversion.
    FmMatrix3 FmComputeTetRotationPolarDecompSvd(
        const FmVector3& tetDeformedPosition0,
        const FmVector3& tetDeformedPosition1,
        const FmVector3& tetDeformedPosition2,
        const FmVector3& tetDeformedPosition3,
        const FmMatrix4& tetRestBaryMatrix);

    // Compute rotated stiffness used to compute forces at each timestep
    void FmComputeRotatedStiffnessMatrix(
        FmTetRotatedStiffnessMatrix* rotatedStiffnessMat,
        const FmTetStiffnessMatrix& stiffnessMat,
        const FmVector3 tetRestPosition0,
        const FmVector3 tetRestPosition1,
        const FmVector3 tetRestPosition2,
        const FmVector3 tetRestPosition3,
        const FmMatrix3& tetRotation);

    // Compute volume matrix for a tetrahedron, needed for deformation gradient
    static FM_FORCE_INLINE FmMatrix3 FmComputeTetVolumeMatrix(const FmVector3& position0, const FmVector3& position1, const FmVector3& position2, const FmVector3& position3)
    {
        FmMatrix3 volumeMatrix;
        volumeMatrix.col0 = position0 - position3;
        volumeMatrix.col1 = position1 - position3;
        volumeMatrix.col2 = position2 - position3;
        return volumeMatrix;
    }

    // Compute elastic deformation as the total deformation gradient multiplied by inverse of plastic deformation matrix.
    // Returns the SVD of this elastic deformation = U * Fhat * V^T, where each component of Fhat gives a stretch or compression factor.
    // The plastic deformation matrix can be updated using this decomposition.
    static FM_FORCE_INLINE void FmComputeTetElasticDeformationGradient(
        FmMatrix3* U,
        FmVector3* Fhat,
        FmMatrix3* V,
        const FmVector3& tetDeformedPosition0,
        const FmVector3& tetDeformedPosition1,
        const FmVector3& tetDeformedPosition2,
        const FmVector3& tetDeformedPosition3,
        const FmMatrix3& restVolumeMatrixInv,
        const FmMatrix3& plasticDeformationMatrixInv)
    {
        FmMatrix3 volumeMatrix = FmComputeTetVolumeMatrix(tetDeformedPosition0, tetDeformedPosition1, tetDeformedPosition2, tetDeformedPosition3);

        FmMatrix3 F = mul(mul(volumeMatrix, restVolumeMatrixInv), plasticDeformationMatrixInv);

        FmSvd3x3(U, Fhat, V, F);
    }

    static FM_FORCE_INLINE bool FmIsTetInverted(const FmVector3 positions[4])
    {
        return dot(positions[3] - positions[0], cross(positions[1] - positions[0], positions[2] - positions[0])) < 0.0f;
    }

    // Ratio of maximum edge length to minimum height of vertex from plane containing other vertices.
    // Measure of tetrahedron quality.
    float FmComputeTetAspectRatio(const FmVector3 positions[4]);
    float FmComputeTetAspectRatio(const FmVector3& position0, const FmVector3& position1, const FmVector3& position2, const FmVector3& position3);

    // If tetrahedron aspect ratio is greater than the specified maximum, modifies the positions to improve aspect ratio, 
    // maintaining the original volume and centroid.
    bool FmEquilateralize(FmVector3* position0, FmVector3* position1, FmVector3* position2, FmVector3* position3, float maxAspectRatio, float minVolume);
}