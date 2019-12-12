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
// Structure-of-arrays SIMD implementation of tetrahedron math functions
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXVectorMath.h"
#include "FEMFXTetMath.h"
#include "FEMFXSoaSvd3x3.h"

namespace AMD
{
    template<class T>
    struct FmSoaTetShapeParams
    {
        typename T::SoaMatrix4 baryMatrix;   // Matrix that transforms a point within the tet to its barycentric coords
        typename T::SoaFloat det;            // Determinant of matrix m = [[1, 1, 1, 1], [x0, x1, x2, x3], [y0, y1, y2, y3], [z0, z1, z2, z3]] = 6 * volume

        inline typename T::SoaFloat GetA0() const { return baryMatrix.col0.x; }
        inline typename T::SoaFloat GetA1() const { return baryMatrix.col0.y; }
        inline typename T::SoaFloat GetA2() const { return baryMatrix.col0.z; }
        inline typename T::SoaFloat GetA3() const { return baryMatrix.col0.w; }

        inline typename T::SoaFloat GetB0() const { return baryMatrix.col1.x; }
        inline typename T::SoaFloat GetB1() const { return baryMatrix.col1.y; }
        inline typename T::SoaFloat GetB2() const { return baryMatrix.col1.z; }
        inline typename T::SoaFloat GetB3() const { return baryMatrix.col1.w; }

        inline typename T::SoaFloat GetC0() const { return baryMatrix.col2.x; }
        inline typename T::SoaFloat GetC1() const { return baryMatrix.col2.y; }
        inline typename T::SoaFloat GetC2() const { return baryMatrix.col2.z; }
        inline typename T::SoaFloat GetC3() const { return baryMatrix.col2.w; }

        inline typename T::SoaFloat GetArea0() const { return 0.5f*det*sqrtf(GetA0()*GetA0() + GetB0()*GetB0() + GetC0()*GetC0()); }
        inline typename T::SoaFloat GetArea1() const { return 0.5f*det*sqrtf(GetA1()*GetA1() + GetB1()*GetB1() + GetC1()*GetC1()); }
        inline typename T::SoaFloat GetArea2() const { return 0.5f*det*sqrtf(GetA2()*GetA2() + GetB2()*GetB2() + GetC2()*GetC2()); }
        inline typename T::SoaFloat GetArea3() const { return 0.5f*det*sqrtf(GetA3()*GetA3() + GetB3()*GetB3() + GetC3()*GetC3()); }
        inline typename T::SoaVector3 GetNormal0() const { return -normalize(FmInitVector3(GetA0(), GetB0(), GetC0())); }
        inline typename T::SoaVector3 GetNormal1() const { return -normalize(FmInitVector3(GetA1(), GetB1(), GetC1())); }
        inline typename T::SoaVector3 GetNormal2() const { return -normalize(FmInitVector3(GetA2(), GetB2(), GetC2())); }
        inline typename T::SoaVector3 GetNormal3() const { return -normalize(FmInitVector3(GetA3(), GetB3(), GetC3())); }
        inline typename T::SoaFloat GetVolume() const { return det * (1.0f / 6.0f); }

        inline void ComputeRestPositions(typename T::SoaVector3* outRestPosition0, typename T::SoaVector3* outRestPosition1, typename T::SoaVector3* outRestPosition2, typename T::SoaVector3* outRestPosition3) const
        {
            typename T::SoaMatrix4 restPosMat = inverse(baryMatrix);

            typename T::SoaVector3 restPosition0 = restPosMat.col0.getXYZ();
            typename T::SoaVector3 restPosition1 = restPosMat.col1.getXYZ();
            typename T::SoaVector3 restPosition2 = restPosMat.col2.getXYZ();
            typename T::SoaVector3 restPosition3 = restPosMat.col3.getXYZ();

            *outRestPosition0 = restPosition0 - restPosition3;
            *outRestPosition1 = restPosition1 - restPosition3;
            *outRestPosition2 = restPosition2 - restPosition3;
            *outRestPosition3 = typename T::SoaVector3(0.0f);
        }
    };

    // Stress matrix, relating position offsets of tet vertices to stress
    template<class T>
    struct FmSoaTetStressMatrix
    {
        typename T::SoaMatrix3 EBe00;  // E . Be which is 6 x 12 stress matrix
        typename T::SoaMatrix3 EBe01;
        typename T::SoaMatrix3 EBe02;
        typename T::SoaMatrix3 EBe03;
        typename T::SoaMatrix3 EBe10;
        typename T::SoaMatrix3 EBe11;
        typename T::SoaMatrix3 EBe12;
        typename T::SoaMatrix3 EBe13;
    };

    // Stiffness matrix, relating position offsets of tet vertices to internal force
    template<class T>
    struct FmSoaTetStiffnessMatrix
    {
        typename T::SoaMatrix3 Ke_diag0;   // Symmetric 12 x 12 stiffness matrix, submatrices on diagonal
        typename T::SoaMatrix3 Ke_diag1;
        typename T::SoaMatrix3 Ke_diag2;
        typename T::SoaMatrix3 Ke_diag3;
        typename T::SoaMatrix3 Ke_lower0;  // Symmetric 12 x 12 stiffness matrix, submatrices below diagonal
        typename T::SoaMatrix3 Ke_lower1;
        typename T::SoaMatrix3 Ke_lower2;
        typename T::SoaMatrix3 Ke_lower3;
        typename T::SoaMatrix3 Ke_lower4;
        typename T::SoaMatrix3 Ke_lower5;
    };

    // Rotated stiffness matrix.
    // Matrix combines steps of un-rotating deformed element positions, applying stiffness, and rotating forces back.
    // Reference: Muller and Gross, "Interactive Virtual Materials"
    template<class T>
    struct FmSoaTetRotatedStiffnessMatrix
    {
        typename T::SoaMatrix3 Keprime_diag0;    // Submatrices on diagonal
        typename T::SoaMatrix3 Keprime_diag1;
        typename T::SoaMatrix3 Keprime_diag2;
        typename T::SoaMatrix3 Keprime_diag3;
        typename T::SoaMatrix3 Keprime_lower0;   // Submatrices below diagonal
        typename T::SoaMatrix3 Keprime_lower1;
        typename T::SoaMatrix3 Keprime_lower2;
        typename T::SoaMatrix3 Keprime_lower3;
        typename T::SoaMatrix3 Keprime_lower4;
        typename T::SoaMatrix3 Keprime_lower5;
        typename T::SoaVector3 f0eprime0;        // Term of the internal force expression containing the rest positions
        typename T::SoaVector3 f0eprime1;
        typename T::SoaVector3 f0eprime2;
        typename T::SoaVector3 f0eprime3;
    };

    template<class T>
    void FmGetSlice(
        FmTetShapeParams* result,
        const FmSoaTetShapeParams<T>& soaResult,
        uint idx)
    {
        result->baryMatrix = FmGetSlice<T>(soaResult.baryMatrix, idx);
        result->det = soaResult.det.getSlice(idx);
    }

    template<class T>
    void FmSetSlice(FmSoaTetShapeParams<T>* soaData, uint idx, const FmTetShapeParams& data)
    {
        FmSetSlice<T>(&soaData->baryMatrix, idx, data.baryMatrix);
        soaData->det.setSlice(idx, data.det);
    }

    template<class T>
    void FmGetSlice(
        FmTetStressMatrix* result,
        const FmSoaTetStressMatrix<T>& soaResult,
        uint idx)
    {
        result->EBe00 = FmGetSlice<T>(soaResult.EBe00, idx);
        result->EBe01 = FmGetSlice<T>(soaResult.EBe01, idx);
        result->EBe02 = FmGetSlice<T>(soaResult.EBe02, idx);
        result->EBe03 = FmGetSlice<T>(soaResult.EBe03, idx);
        result->EBe10 = FmGetSlice<T>(soaResult.EBe10, idx);
        result->EBe11 = FmGetSlice<T>(soaResult.EBe11, idx);
        result->EBe12 = FmGetSlice<T>(soaResult.EBe12, idx);
        result->EBe13 = FmGetSlice<T>(soaResult.EBe13, idx);
    }

    template<class T>
    void FmGetSlice(
        FmTetStiffnessMatrix* result,
        const FmSoaTetStiffnessMatrix<T>& soaResult,
        uint idx)
    {
        result->Ke_diag0 = FmGetSlice<T>(soaResult.Ke_diag0, idx);
        result->Ke_diag1 = FmGetSlice<T>(soaResult.Ke_diag1, idx);
        result->Ke_diag2 = FmGetSlice<T>(soaResult.Ke_diag2, idx);
        result->Ke_diag3 = FmGetSlice<T>(soaResult.Ke_diag3, idx);

        result->Ke_lower0 = FmGetSlice<T>(soaResult.Ke_lower0, idx);
        result->Ke_lower1 = FmGetSlice<T>(soaResult.Ke_lower1, idx);
        result->Ke_lower2 = FmGetSlice<T>(soaResult.Ke_lower2, idx);
        result->Ke_lower3 = FmGetSlice<T>(soaResult.Ke_lower3, idx);
        result->Ke_lower4 = FmGetSlice<T>(soaResult.Ke_lower4, idx);
        result->Ke_lower5 = FmGetSlice<T>(soaResult.Ke_lower5, idx);
    }

    template<class T>
    void FmGetSlice(
        FmTetRotatedStiffnessMatrix* result,
        const FmSoaTetRotatedStiffnessMatrix<T>& soaResult,
        uint idx)
    {
        result->Keprime_diag0 = FmGetSlice<T>(soaResult.Keprime_diag0, idx);
        result->Keprime_diag1 = FmGetSlice<T>(soaResult.Keprime_diag1, idx);
        result->Keprime_diag2 = FmGetSlice<T>(soaResult.Keprime_diag2, idx);
        result->Keprime_diag3 = FmGetSlice<T>(soaResult.Keprime_diag3, idx);

        result->Keprime_lower0 = FmGetSlice<T>(soaResult.Keprime_lower0, idx);
        result->Keprime_lower1 = FmGetSlice<T>(soaResult.Keprime_lower1, idx);
        result->Keprime_lower2 = FmGetSlice<T>(soaResult.Keprime_lower2, idx);
        result->Keprime_lower3 = FmGetSlice<T>(soaResult.Keprime_lower3, idx);
        result->Keprime_lower4 = FmGetSlice<T>(soaResult.Keprime_lower4, idx);
        result->Keprime_lower5 = FmGetSlice<T>(soaResult.Keprime_lower5, idx);

        result->f0eprime0 = FmGetSlice<T>(soaResult.f0eprime0, idx);
        result->f0eprime1 = FmGetSlice<T>(soaResult.f0eprime1, idx);
        result->f0eprime2 = FmGetSlice<T>(soaResult.f0eprime2, idx);
        result->f0eprime3 = FmGetSlice<T>(soaResult.f0eprime3, idx);
    }

    template<class T>
    void FmSetSlice(FmSoaTetStiffnessMatrix<T>* soaMat, uint idx, const FmTetStiffnessMatrix& mat)
    {
        FmSetSlice<T>(&soaMat->Ke_diag0, idx, mat.Ke_diag0);
        FmSetSlice<T>(&soaMat->Ke_diag1, idx, mat.Ke_diag1);
        FmSetSlice<T>(&soaMat->Ke_diag2, idx, mat.Ke_diag2);
        FmSetSlice<T>(&soaMat->Ke_diag3, idx, mat.Ke_diag3);

        FmSetSlice<T>(&soaMat->Ke_lower0, idx, mat.Ke_lower0);
        FmSetSlice<T>(&soaMat->Ke_lower1, idx, mat.Ke_lower1);
        FmSetSlice<T>(&soaMat->Ke_lower2, idx, mat.Ke_lower2);
        FmSetSlice<T>(&soaMat->Ke_lower3, idx, mat.Ke_lower3);
        FmSetSlice<T>(&soaMat->Ke_lower4, idx, mat.Ke_lower4);
        FmSetSlice<T>(&soaMat->Ke_lower5, idx, mat.Ke_lower5);
    }

    template<class T>
    void FmComputeShapeParams(
        FmSoaTetShapeParams<T>* resultShapeParams,
        const typename T::SoaVector3& p0,
        const typename T::SoaVector3& p1,
        const typename T::SoaVector3& p2,
        const typename T::SoaVector3& p3)
    {
        FmSoaTetShapeParams<T> shapeParams;

        typename T::SoaFloat x0 = p0.x;
        typename T::SoaFloat y0 = p0.y;
        typename T::SoaFloat z0 = p0.z;
        typename T::SoaFloat x1 = p1.x;
        typename T::SoaFloat y1 = p1.y;
        typename T::SoaFloat z1 = p1.z;
        typename T::SoaFloat x2 = p2.x;
        typename T::SoaFloat y2 = p2.y;
        typename T::SoaFloat z2 = p2.z;
        typename T::SoaFloat x3 = p3.x;
        typename T::SoaFloat y3 = p3.y;
        typename T::SoaFloat z3 = p3.z;

        typename T::SoaFloat x01 = x0 - x1;
        typename T::SoaFloat x02 = x0 - x2;
        typename T::SoaFloat x03 = x0 - x3;
        typename T::SoaFloat x10 = -x01;
        typename T::SoaFloat x12 = x1 - x2;
        typename T::SoaFloat x13 = x1 - x3;
        typename T::SoaFloat x20 = -x02;
        typename T::SoaFloat x21 = -x12;
        typename T::SoaFloat x23 = x2 - x3;
        typename T::SoaFloat x30 = -x03;
        typename T::SoaFloat x31 = -x13;
        typename T::SoaFloat x32 = -x23;

        typename T::SoaFloat y01 = y0 - y1;
        typename T::SoaFloat y02 = y0 - y2;
        typename T::SoaFloat y03 = y0 - y3;
        typename T::SoaFloat y10 = -y01;
        typename T::SoaFloat y12 = y1 - y2;
        typename T::SoaFloat y13 = y1 - y3;
        typename T::SoaFloat y20 = -y02;
        typename T::SoaFloat y21 = -y12;
        typename T::SoaFloat y23 = y2 - y3;
        typename T::SoaFloat y30 = -y03;
        typename T::SoaFloat y31 = -y13;
        typename T::SoaFloat y32 = -y23;

        // determinant of M = [  1  1  1  1 ] = 6.0 x signed volume
        //                    [ p0 p1 p2 p3 ] 

        typename T::SoaFloat det = (x10*y2 + x02 * y1 + x21 * y0)*z3 + (x01*y3 + x30 * y1 + x13 * y0)*z2 + (x20*y3 + x03 * y2 + x32 * y0)*z1 +
            (x12*y3 + x31 * y2 + x23 * y1)*z0;

        typename T::SoaFloat m00 = (x1*y2 - x2 * y1)*z3 + (x3*y1 - x1 * y3)*z2 + (x2*y3 - x3 * y2)*z1;
        typename T::SoaFloat m01 = y12 * z3 + y31 * z2 + y23 * z1;
        typename T::SoaFloat m02 = x21 * z3 + x13 * z2 + x32 * z1;
        typename T::SoaFloat m03 = x12 * y3 + x31 * y2 + x23 * y1;
        typename T::SoaFloat m10 = (x2*y0 - x0 * y2)*z3 + (x0*y3 - x3 * y0)*z2 + (x3*y2 - x2 * y3)*z0;
        typename T::SoaFloat m11 = y20 * z3 + y03 * z2 + y32 * z0;
        typename T::SoaFloat m12 = x02 * z3 + x30 * z2 + x23 * z0;
        typename T::SoaFloat m13 = x20 * y3 + x03 * y2 + x32 * y0;
        typename T::SoaFloat m20 = (x0*y1 - x1 * y0)*z3 + (x3*y0 - x0 * y3)*z1 + (x1*y3 - x3 * y1)*z0;
        typename T::SoaFloat m21 = y01 * z3 + y30 * z1 + y13 * z0;
        typename T::SoaFloat m22 = x10 * z3 + x03 * z1 + x31 * z0;
        typename T::SoaFloat m23 = x01 * y3 + x30 * y1 + x13 * y0;
        typename T::SoaFloat m30 = (x1*y0 - x0 * y1)*z2 + (x0*y2 - x2 * y0)*z1 + (x2*y1 - x1 * y2)*z0;
        typename T::SoaFloat m31 = y10 * z2 + y02 * z1 + y21 * z0;
        typename T::SoaFloat m32 = x01 * z2 + x20 * z1 + x12 * z0;
        typename T::SoaFloat m33 = x10 * y2 + x02 * y1 + x21 * y0;

        shapeParams.det = det;

        // baryMatrix transforms [x, y, z, 1]^T to barycentric coords

        typename T::SoaFloat invDet = 1.0f / det;

        shapeParams.baryMatrix = typename T::SoaMatrix4(
            typename T::SoaVector4(
                m01 * invDet,
                m11 * invDet,
                m21 * invDet,
                m31 * invDet),
            typename T::SoaVector4(
                m02 * invDet,
                m12 * invDet,
                m22 * invDet,
                m32 * invDet),
            typename T::SoaVector4(
                m03 * invDet,
                m13 * invDet,
                m23 * invDet,
                m33 * invDet),
            typename T::SoaVector4(
                m00 * invDet,
                m10 * invDet,
                m20 * invDet,
                m30 * invDet));

        *resultShapeParams = shapeParams;
    }

    template<class T>
    void FmComputeTetStrain(
        typename T::SoaVector3 strain[2],
        const FmSoaTetShapeParams<T>& shapeParams,
        const typename T::SoaVector3& unrotatedOffset0,
        const typename T::SoaVector3& unrotatedOffset1,
        const typename T::SoaVector3& unrotatedOffset2,
        const typename T::SoaVector3& unrotatedOffset3)
    {
        typename T::SoaFloat a0 = shapeParams.GetA0();
        typename T::SoaFloat a1 = shapeParams.GetA1();
        typename T::SoaFloat a2 = shapeParams.GetA2();
        typename T::SoaFloat a3 = shapeParams.GetA3();
        typename T::SoaFloat b0 = shapeParams.GetB0();
        typename T::SoaFloat b1 = shapeParams.GetB1();
        typename T::SoaFloat b2 = shapeParams.GetB2();
        typename T::SoaFloat b3 = shapeParams.GetB3();
        typename T::SoaFloat c0 = shapeParams.GetC0();
        typename T::SoaFloat c1 = shapeParams.GetC1();
        typename T::SoaFloat c2 = shapeParams.GetC2();
        typename T::SoaFloat c3 = shapeParams.GetC3();

        typename T::SoaFloat x0 = unrotatedOffset0.x;
        typename T::SoaFloat y0 = unrotatedOffset0.y;
        typename T::SoaFloat z0 = unrotatedOffset0.z;
        typename T::SoaFloat x1 = unrotatedOffset1.x;
        typename T::SoaFloat y1 = unrotatedOffset1.y;
        typename T::SoaFloat z1 = unrotatedOffset1.z;
        typename T::SoaFloat x2 = unrotatedOffset2.x;
        typename T::SoaFloat y2 = unrotatedOffset2.y;
        typename T::SoaFloat z2 = unrotatedOffset2.z;
        typename T::SoaFloat x3 = unrotatedOffset3.x;
        typename T::SoaFloat y3 = unrotatedOffset3.y;
        typename T::SoaFloat z3 = unrotatedOffset3.z;

        strain[0].x = a3 * x3 + a2 * x2 + a1 * x1 + a0 * x0;
        strain[0].y = b3 * y3 + b2 * y2 + b1 * y1 + b0 * y0;
        strain[0].z = c3 * z3 + c2 * z2 + c1 * z1 + c0 * z0;
        strain[1].x = a3 * y3 + a2 * y2 + a1 * y1 + a0 * y0 + b3 * x3 + b2 * x2 + b1 * x1 + b0 * x0;
        strain[1].y = b3 * z3 + b2 * z2 + b1 * z1 + b0 * z0 + c3 * y3 + c2 * y2 + c1 * y1 + c0 * y0;
        strain[1].z = a3 * z3 + a2 * z2 + a1 * z1 + a0 * z0 + c3 * x3 + c2 * x2 + c1 * x1 + c0 * x0;
    }

    // Compute stress matrix from shape and material parameters
    template<class T>
    void FmComputeTetStressMatrix(
        FmSoaTetStressMatrix<T>* stressMat,
        const typename T::SoaVector4& baryMatrixCol0,
        const typename T::SoaVector4& baryMatrixCol1,
        const typename T::SoaVector4& baryMatrixCol2,
        typename T::SoaFloat youngsModulus,
        typename T::SoaFloat poissonsRatio)
    {
        typename T::SoaFloat a0 = baryMatrixCol0.x;
        typename T::SoaFloat a1 = baryMatrixCol0.y;
        typename T::SoaFloat a2 = baryMatrixCol0.z;
        typename T::SoaFloat a3 = baryMatrixCol0.w;
        typename T::SoaFloat b0 = baryMatrixCol1.x;
        typename T::SoaFloat b1 = baryMatrixCol1.y;
        typename T::SoaFloat b2 = baryMatrixCol1.z;
        typename T::SoaFloat b3 = baryMatrixCol1.w;
        typename T::SoaFloat c0 = baryMatrixCol2.x;
        typename T::SoaFloat c1 = baryMatrixCol2.y;
        typename T::SoaFloat c2 = baryMatrixCol2.z;
        typename T::SoaFloat c3 = baryMatrixCol2.w;

        typename T::SoaFloat scale = (youngsModulus / ((1.0f + poissonsRatio)*(1.0f - 2.0f * poissonsRatio)));

        typename T::SoaFloat v0 = scale * (1.0f - poissonsRatio);
        typename T::SoaFloat v1 = scale * poissonsRatio;
        typename T::SoaFloat v2 = scale * (0.5f - poissonsRatio);

        typename T::SoaFloat a0_v0 = a0 * v0;
        typename T::SoaFloat a0_v1 = a0 * v1;
        typename T::SoaFloat a0_v2 = a0 * v2;
        typename T::SoaFloat a1_v0 = a1 * v0;
        typename T::SoaFloat a1_v1 = a1 * v1;
        typename T::SoaFloat a1_v2 = a1 * v2;
        typename T::SoaFloat a2_v0 = a2 * v0;
        typename T::SoaFloat a2_v1 = a2 * v1;
        typename T::SoaFloat a2_v2 = a2 * v2;
        typename T::SoaFloat a3_v0 = a3 * v0;
        typename T::SoaFloat a3_v1 = a3 * v1;
        typename T::SoaFloat a3_v2 = a3 * v2;
        typename T::SoaFloat b0_v0 = b0 * v0;
        typename T::SoaFloat b0_v1 = b0 * v1;
        typename T::SoaFloat b0_v2 = b0 * v2;
        typename T::SoaFloat b1_v0 = b1 * v0;
        typename T::SoaFloat b1_v1 = b1 * v1;
        typename T::SoaFloat b1_v2 = b1 * v2;
        typename T::SoaFloat b2_v0 = b2 * v0;
        typename T::SoaFloat b2_v1 = b2 * v1;
        typename T::SoaFloat b2_v2 = b2 * v2;
        typename T::SoaFloat b3_v0 = b3 * v0;
        typename T::SoaFloat b3_v1 = b3 * v1;
        typename T::SoaFloat b3_v2 = b3 * v2;
        typename T::SoaFloat c0_v0 = c0 * v0;
        typename T::SoaFloat c0_v1 = c0 * v1;
        typename T::SoaFloat c0_v2 = c0 * v2;
        typename T::SoaFloat c1_v0 = c1 * v0;
        typename T::SoaFloat c1_v1 = c1 * v1;
        typename T::SoaFloat c1_v2 = c1 * v2;
        typename T::SoaFloat c2_v0 = c2 * v0;
        typename T::SoaFloat c2_v1 = c2 * v1;
        typename T::SoaFloat c2_v2 = c2 * v2;
        typename T::SoaFloat c3_v0 = c3 * v0;
        typename T::SoaFloat c3_v1 = c3 * v1;
        typename T::SoaFloat c3_v2 = c3 * v2;

        stressMat->EBe00 = typename T::SoaMatrix3(typename T::SoaVector3(a0_v0, a0_v1, a0_v1), typename T::SoaVector3(b0_v1, b0_v0, b0_v1), typename T::SoaVector3(c0_v1, c0_v1, c0_v0));
        stressMat->EBe01 = typename T::SoaMatrix3(typename T::SoaVector3(a1_v0, a1_v1, a1_v1), typename T::SoaVector3(b1_v1, b1_v0, b1_v1), typename T::SoaVector3(c1_v1, c1_v1, c1_v0));
        stressMat->EBe02 = typename T::SoaMatrix3(typename T::SoaVector3(a2_v0, a2_v1, a2_v1), typename T::SoaVector3(b2_v1, b2_v0, b2_v1), typename T::SoaVector3(c2_v1, c2_v1, c2_v0));
        stressMat->EBe03 = typename T::SoaMatrix3(typename T::SoaVector3(a3_v0, a3_v1, a3_v1), typename T::SoaVector3(b3_v1, b3_v0, b3_v1), typename T::SoaVector3(c3_v1, c3_v1, c3_v0));
        stressMat->EBe10 = typename T::SoaMatrix3(typename T::SoaVector3(b0_v2, 0.0f, c0_v2), typename T::SoaVector3(a0_v2, c0_v2, 0.0f), typename T::SoaVector3(0.0f, b0_v2, a0_v2));
        stressMat->EBe11 = typename T::SoaMatrix3(typename T::SoaVector3(b1_v2, 0.0f, c1_v2), typename T::SoaVector3(a1_v2, c1_v2, 0.0f), typename T::SoaVector3(0.0f, b1_v2, a1_v2));
        stressMat->EBe12 = typename T::SoaMatrix3(typename T::SoaVector3(b2_v2, 0.0f, c2_v2), typename T::SoaVector3(a2_v2, c2_v2, 0.0f), typename T::SoaVector3(0.0f, b2_v2, a2_v2));
        stressMat->EBe13 = typename T::SoaMatrix3(typename T::SoaVector3(b3_v2, 0.0f, c3_v2), typename T::SoaVector3(a3_v2, c3_v2, 0.0f), typename T::SoaVector3(0.0f, b3_v2, a3_v2));
    }

    // Compute stress and stiffness matrix from shape and material parameters
    template<class T>
    void FmComputeTetStressAndStiffnessMatrix(
        FmSoaTetStressMatrix<T>* stressMat, FmSoaTetStiffnessMatrix<T>* stiffnessMat, 
        const typename T::SoaVector4& baryMatrixCol0,
        const typename T::SoaVector4& baryMatrixCol1,
        const typename T::SoaVector4& baryMatrixCol2,
        typename T::SoaFloat volume,
        typename T::SoaFloat youngsModulus,
        typename T::SoaFloat poissonsRatio)
    {
        typename T::SoaFloat a0 = baryMatrixCol0.x;
        typename T::SoaFloat a1 = baryMatrixCol0.y;
        typename T::SoaFloat a2 = baryMatrixCol0.z;
        typename T::SoaFloat a3 = baryMatrixCol0.w;
        typename T::SoaFloat b0 = baryMatrixCol1.x;
        typename T::SoaFloat b1 = baryMatrixCol1.y;
        typename T::SoaFloat b2 = baryMatrixCol1.z;
        typename T::SoaFloat b3 = baryMatrixCol1.w;
        typename T::SoaFloat c0 = baryMatrixCol2.x;
        typename T::SoaFloat c1 = baryMatrixCol2.y;
        typename T::SoaFloat c2 = baryMatrixCol2.z;
        typename T::SoaFloat c3 = baryMatrixCol2.w;

        typename T::SoaFloat scale = (youngsModulus / ((1.0f + poissonsRatio)*(1.0f - 2.0f * poissonsRatio)));

        typename T::SoaFloat v0 = scale * (1.0f - poissonsRatio);
        typename T::SoaFloat v1 = scale * poissonsRatio;
        typename T::SoaFloat v2 = scale * (0.5f - poissonsRatio);

        typename T::SoaFloat a0_v0 = a0*v0;
        typename T::SoaFloat a0_v1 = a0*v1;
        typename T::SoaFloat a0_v2 = a0*v2;
        typename T::SoaFloat a1_v0 = a1*v0;
        typename T::SoaFloat a1_v1 = a1*v1;
        typename T::SoaFloat a1_v2 = a1*v2;
        typename T::SoaFloat a2_v0 = a2*v0;
        typename T::SoaFloat a2_v1 = a2*v1;
        typename T::SoaFloat a2_v2 = a2*v2;
        typename T::SoaFloat a3_v0 = a3*v0;
        typename T::SoaFloat a3_v1 = a3*v1;
        typename T::SoaFloat a3_v2 = a3*v2;
        typename T::SoaFloat b0_v0 = b0*v0;
        typename T::SoaFloat b0_v1 = b0*v1;
        typename T::SoaFloat b0_v2 = b0*v2;
        typename T::SoaFloat b1_v0 = b1*v0;
        typename T::SoaFloat b1_v1 = b1*v1;
        typename T::SoaFloat b1_v2 = b1*v2;
        typename T::SoaFloat b2_v0 = b2*v0;
        typename T::SoaFloat b2_v1 = b2*v1;
        typename T::SoaFloat b2_v2 = b2*v2;
        typename T::SoaFloat b3_v0 = b3*v0;
        typename T::SoaFloat b3_v1 = b3*v1;
        typename T::SoaFloat b3_v2 = b3*v2;
        typename T::SoaFloat c0_v0 = c0*v0;
        typename T::SoaFloat c0_v1 = c0*v1;
        typename T::SoaFloat c0_v2 = c0*v2;
        typename T::SoaFloat c1_v0 = c1*v0;
        typename T::SoaFloat c1_v1 = c1*v1;
        typename T::SoaFloat c1_v2 = c1*v2;
        typename T::SoaFloat c2_v0 = c2*v0;
        typename T::SoaFloat c2_v1 = c2*v1;
        typename T::SoaFloat c2_v2 = c2*v2;
        typename T::SoaFloat c3_v0 = c3*v0;
        typename T::SoaFloat c3_v1 = c3*v1;
        typename T::SoaFloat c3_v2 = c3*v2;

        stressMat->EBe00 = typename T::SoaMatrix3(typename T::SoaVector3(a0_v0, a0_v1, a0_v1), typename T::SoaVector3(b0_v1, b0_v0, b0_v1), typename T::SoaVector3(c0_v1, c0_v1, c0_v0));
        stressMat->EBe01 = typename T::SoaMatrix3(typename T::SoaVector3(a1_v0, a1_v1, a1_v1), typename T::SoaVector3(b1_v1, b1_v0, b1_v1), typename T::SoaVector3(c1_v1, c1_v1, c1_v0));
        stressMat->EBe02 = typename T::SoaMatrix3(typename T::SoaVector3(a2_v0, a2_v1, a2_v1), typename T::SoaVector3(b2_v1, b2_v0, b2_v1), typename T::SoaVector3(c2_v1, c2_v1, c2_v0));
        stressMat->EBe03 = typename T::SoaMatrix3(typename T::SoaVector3(a3_v0, a3_v1, a3_v1), typename T::SoaVector3(b3_v1, b3_v0, b3_v1), typename T::SoaVector3(c3_v1, c3_v1, c3_v0));
        stressMat->EBe10 = typename T::SoaMatrix3(typename T::SoaVector3(b0_v2, 0.0f, c0_v2), typename T::SoaVector3(a0_v2, c0_v2, 0.0f), typename T::SoaVector3(0.0f, b0_v2, a0_v2));
        stressMat->EBe11 = typename T::SoaMatrix3(typename T::SoaVector3(b1_v2, 0.0f, c1_v2), typename T::SoaVector3(a1_v2, c1_v2, 0.0f), typename T::SoaVector3(0.0f, b1_v2, a1_v2));
        stressMat->EBe12 = typename T::SoaMatrix3(typename T::SoaVector3(b2_v2, 0.0f, c2_v2), typename T::SoaVector3(a2_v2, c2_v2, 0.0f), typename T::SoaVector3(0.0f, b2_v2, a2_v2));
        stressMat->EBe13 = typename T::SoaMatrix3(typename T::SoaVector3(b3_v2, 0.0f, c3_v2), typename T::SoaVector3(a3_v2, c3_v2, 0.0f), typename T::SoaVector3(0.0f, b3_v2, a3_v2));

        stiffnessMat->Ke_diag0 = volume * typename T::SoaMatrix3(typename T::SoaVector3(c0*c0_v2 + b0*b0_v2 + a0*a0_v0, a0*b0_v2 + a0*b0_v1, a0*c0_v2 + a0*c0_v1), typename T::SoaVector3(a0*b0_v2 + a0*b0_v1, c0*c0_v2 + a0*a0_v2 + b0*b0_v0, b0*c0_v2 + b0*c0_v1), typename T::SoaVector3(a0*c0_v2 + a0*c0_v1, b0*c0_v2 + b0*c0_v1, b0*b0_v2 + a0*a0_v2 + c0*c0_v0));
        stiffnessMat->Ke_diag1 = volume * typename T::SoaMatrix3(typename T::SoaVector3(c1*c1_v2 + b1*b1_v2 + a1*a1_v0, a1*b1_v2 + a1*b1_v1, a1*c1_v2 + a1*c1_v1), typename T::SoaVector3(a1*b1_v2 + a1*b1_v1, c1*c1_v2 + a1*a1_v2 + b1*b1_v0, b1*c1_v2 + b1*c1_v1), typename T::SoaVector3(a1*c1_v2 + a1*c1_v1, b1*c1_v2 + b1*c1_v1, b1*b1_v2 + a1*a1_v2 + c1*c1_v0));
        stiffnessMat->Ke_diag2 = volume * typename T::SoaMatrix3(typename T::SoaVector3(c2*c2_v2 + b2*b2_v2 + a2*a2_v0, a2*b2_v2 + a2*b2_v1, a2*c2_v2 + a2*c2_v1), typename T::SoaVector3(a2*b2_v2 + a2*b2_v1, c2*c2_v2 + a2*a2_v2 + b2*b2_v0, b2*c2_v2 + b2*c2_v1), typename T::SoaVector3(a2*c2_v2 + a2*c2_v1, b2*c2_v2 + b2*c2_v1, b2*b2_v2 + a2*a2_v2 + c2*c2_v0));
        stiffnessMat->Ke_diag3 = volume * typename T::SoaMatrix3(typename T::SoaVector3(c3*c3_v2 + b3*b3_v2 + a3*a3_v0, a3*b3_v2 + a3*b3_v1, a3*c3_v2 + a3*c3_v1), typename T::SoaVector3(a3*b3_v2 + a3*b3_v1, c3*c3_v2 + a3*a3_v2 + b3*b3_v0, b3*c3_v2 + b3*c3_v1), typename T::SoaVector3(a3*c3_v2 + a3*c3_v1, b3*c3_v2 + b3*c3_v1, b3*b3_v2 + a3*a3_v2 + c3*c3_v0));
        stiffnessMat->Ke_lower0 = volume * typename T::SoaMatrix3(typename T::SoaVector3(c0*c1_v2 + b0*b1_v2 + a0*a1_v0, a1*b0_v2 + a0*b1_v1, a1*c0_v2 + a0*c1_v1), typename T::SoaVector3(a0*b1_v2 + a1*b0_v1, c0*c1_v2 + a0*a1_v2 + b0*b1_v0, b1*c0_v2 + b0*c1_v1), typename T::SoaVector3(a0*c1_v2 + a1*c0_v1, b0*c1_v2 + b1*c0_v1, b0*b1_v2 + a0*a1_v2 + c0*c1_v0));
        stiffnessMat->Ke_lower1 = volume * typename T::SoaMatrix3(typename T::SoaVector3(c0*c2_v2 + b0*b2_v2 + a0*a2_v0, a2*b0_v2 + a0*b2_v1, a2*c0_v2 + a0*c2_v1), typename T::SoaVector3(a0*b2_v2 + a2*b0_v1, c0*c2_v2 + a0*a2_v2 + b0*b2_v0, b2*c0_v2 + b0*c2_v1), typename T::SoaVector3(a0*c2_v2 + a2*c0_v1, b0*c2_v2 + b2*c0_v1, b0*b2_v2 + a0*a2_v2 + c0*c2_v0));
        stiffnessMat->Ke_lower2 = volume * typename T::SoaMatrix3(typename T::SoaVector3(c0*c3_v2 + b0*b3_v2 + a0*a3_v0, a3*b0_v2 + a0*b3_v1, a3*c0_v2 + a0*c3_v1), typename T::SoaVector3(a0*b3_v2 + a3*b0_v1, c0*c3_v2 + a0*a3_v2 + b0*b3_v0, b3*c0_v2 + b0*c3_v1), typename T::SoaVector3(a0*c3_v2 + a3*c0_v1, b0*c3_v2 + b3*c0_v1, b0*b3_v2 + a0*a3_v2 + c0*c3_v0));
        stiffnessMat->Ke_lower3 = volume * typename T::SoaMatrix3(typename T::SoaVector3(c1*c2_v2 + b1*b2_v2 + a1*a2_v0, a2*b1_v2 + a1*b2_v1, a2*c1_v2 + a1*c2_v1), typename T::SoaVector3(a1*b2_v2 + a2*b1_v1, c1*c2_v2 + a1*a2_v2 + b1*b2_v0, b2*c1_v2 + b1*c2_v1), typename T::SoaVector3(a1*c2_v2 + a2*c1_v1, b1*c2_v2 + b2*c1_v1, b1*b2_v2 + a1*a2_v2 + c1*c2_v0));
        stiffnessMat->Ke_lower4 = volume * typename T::SoaMatrix3(typename T::SoaVector3(c1*c3_v2 + b1*b3_v2 + a1*a3_v0, a3*b1_v2 + a1*b3_v1, a3*c1_v2 + a1*c3_v1), typename T::SoaVector3(a1*b3_v2 + a3*b1_v1, c1*c3_v2 + a1*a3_v2 + b1*b3_v0, b3*c1_v2 + b1*c3_v1), typename T::SoaVector3(a1*c3_v2 + a3*c1_v1, b1*c3_v2 + b3*c1_v1, b1*b3_v2 + a1*a3_v2 + c1*c3_v0));
        stiffnessMat->Ke_lower5 = volume * typename T::SoaMatrix3(typename T::SoaVector3(c2*c3_v2 + b2*b3_v2 + a2*a3_v0, a3*b2_v2 + a2*b3_v1, a3*c2_v2 + a2*c3_v1), typename T::SoaVector3(a2*b3_v2 + a3*b2_v1, c2*c3_v2 + a2*a3_v2 + b2*b3_v0, b3*c2_v2 + b2*c3_v1), typename T::SoaVector3(a2*c3_v2 + a3*c2_v1, b2*c3_v2 + b3*c2_v1, b2*b3_v2 + a2*a3_v2 + c2*c3_v0));
    }

    template<class T>
    void FmComputeRotatedStiffnessMatrix(
        FmSoaTetRotatedStiffnessMatrix<T>* rotatedStiffnessMat,
        const FmSoaTetStiffnessMatrix<T>& stiffnessMat,
        const typename T::SoaVector3& tetRestPosition0,
        const typename T::SoaVector3& tetRestPosition1,
        const typename T::SoaVector3& tetRestPosition2,
        const typename T::SoaVector3& tetRestPosition3,
        const typename T::SoaMatrix3& tetRotation)
    {
        typename T::SoaMatrix3 ReKe[4][4];

        typename T::SoaMatrix3 tetRotationInv = transpose(tetRotation);

        ReKe[0][0] = mul(tetRotation, stiffnessMat.Ke_diag0);
        ReKe[0][1] = mul(tetRotation, transpose(stiffnessMat.Ke_lower0));
        ReKe[0][2] = mul(tetRotation, transpose(stiffnessMat.Ke_lower1));
        ReKe[0][3] = mul(tetRotation, transpose(stiffnessMat.Ke_lower2));
        ReKe[1][0] = mul(tetRotation, stiffnessMat.Ke_lower0);
        ReKe[1][1] = mul(tetRotation, stiffnessMat.Ke_diag1);
        ReKe[1][2] = mul(tetRotation, transpose(stiffnessMat.Ke_lower3));
        ReKe[1][3] = mul(tetRotation, transpose(stiffnessMat.Ke_lower4));
        ReKe[2][0] = mul(tetRotation, stiffnessMat.Ke_lower1);
        ReKe[2][1] = mul(tetRotation, stiffnessMat.Ke_lower3);
        ReKe[2][2] = mul(tetRotation, stiffnessMat.Ke_diag2);
        ReKe[2][3] = mul(tetRotation, transpose(stiffnessMat.Ke_lower5));
        ReKe[3][0] = mul(tetRotation, stiffnessMat.Ke_lower2);
        ReKe[3][1] = mul(tetRotation, stiffnessMat.Ke_lower4);
        ReKe[3][2] = mul(tetRotation, stiffnessMat.Ke_lower5);
        ReKe[3][3] = mul(tetRotation, stiffnessMat.Ke_diag3);

        typename T::SoaVector3 vec0 = -tetRestPosition0;
        typename T::SoaVector3 vec1 = -tetRestPosition1;
        typename T::SoaVector3 vec2 = -tetRestPosition2;
        typename T::SoaVector3 vec3 = -tetRestPosition3;

        rotatedStiffnessMat->f0eprime0 = mul(ReKe[0][0], vec0) + mul(ReKe[0][1], vec1) + mul(ReKe[0][2], vec2) + mul(ReKe[0][3], vec3);
        rotatedStiffnessMat->f0eprime1 = mul(ReKe[1][0], vec0) + mul(ReKe[1][1], vec1) + mul(ReKe[1][2], vec2) + mul(ReKe[1][3], vec3);
        rotatedStiffnessMat->f0eprime2 = mul(ReKe[2][0], vec0) + mul(ReKe[2][1], vec1) + mul(ReKe[2][2], vec2) + mul(ReKe[2][3], vec3);
        rotatedStiffnessMat->f0eprime3 = mul(ReKe[3][0], vec0) + mul(ReKe[3][1], vec1) + mul(ReKe[3][2], vec2) + mul(ReKe[3][3], vec3);

        rotatedStiffnessMat->Keprime_diag0 = mul(ReKe[0][0], tetRotationInv);
        rotatedStiffnessMat->Keprime_lower0 = mul(ReKe[1][0], tetRotationInv);
        rotatedStiffnessMat->Keprime_lower1 = mul(ReKe[2][0], tetRotationInv);
        rotatedStiffnessMat->Keprime_lower2 = mul(ReKe[3][0], tetRotationInv);
        rotatedStiffnessMat->Keprime_diag1 = mul(ReKe[1][1], tetRotationInv);
        rotatedStiffnessMat->Keprime_lower3 = mul(ReKe[2][1], tetRotationInv);
        rotatedStiffnessMat->Keprime_lower4 = mul(ReKe[3][1], tetRotationInv);

        rotatedStiffnessMat->Keprime_diag2 = mul(ReKe[2][2], tetRotationInv);
        rotatedStiffnessMat->Keprime_lower5 = mul(ReKe[3][2], tetRotationInv);

        rotatedStiffnessMat->Keprime_diag3 = mul(ReKe[3][3], tetRotationInv);
    }

    template<class T>
    typename T::SoaMatrix3 FmComputeTetRotationPolarDecompSvd(
        const typename T::SoaVector3& tetDeformedPosition0,
        const typename T::SoaVector3& tetDeformedPosition1,
        const typename T::SoaVector3& tetDeformedPosition2,
        const typename T::SoaVector3& tetDeformedPosition3,
        const typename T::SoaMatrix4& tetRestBaryMatrix)
    {
        // Deformation of any point in tetrahedron = Barycentric2DeformedPos * OriginalPos2Barycentric
        typename T::SoaMatrix4 deformationTransform = mul(typename T::SoaMatrix4(
            typename T::SoaVector4(tetDeformedPosition0, typename T::SoaFloat(1.0f)),
            typename T::SoaVector4(tetDeformedPosition1, typename T::SoaFloat(1.0f)),
            typename T::SoaVector4(tetDeformedPosition2, typename T::SoaFloat(1.0f)),
            typename T::SoaVector4(tetDeformedPosition3, typename T::SoaFloat(1.0f))), tetRestBaryMatrix);

        typename T::SoaMatrix3 U, V;
        typename T::SoaVector3 sigma;
        FmSvd3x3<T>(&U, &sigma, &V, deformationTransform.getUpper3x3());

        return mul(U, transpose(V));
    }

    // Compute volume matrix for a tetrahedron, needed for deformation gradient
    template<class T>
    static FM_FORCE_INLINE typename T::SoaMatrix3 FmComputeTetVolumeMatrix(
        const typename T::SoaVector3& position0, 
        const typename T::SoaVector3& position1, 
        const typename T::SoaVector3& position2, 
        const typename T::SoaVector3& position3)
    {
        typename T::SoaMatrix3 volumeMatrix;
        volumeMatrix.col0 = position0 - position3;
        volumeMatrix.col1 = position1 - position3;
        volumeMatrix.col2 = position2 - position3;
        return volumeMatrix;
    }

    // Compute elastic deformation as the total deformation gradient multiplied by inverse of plastic deformation matrix.
    // Returns the SVD of this elastic deformation = U * Fhat * V^T, where each component of Fhat gives a stretch or compression factor.
    // The plastic deformation matrix can be updated using this decomposition.
    template<class T>
    static FM_FORCE_INLINE void FmComputeTetElasticDeformationGradient(
        typename T::SoaMatrix3* U,
        typename T::SoaVector3* Fhat,
        typename T::SoaMatrix3* V,
        const typename T::SoaVector3& tetDeformedPosition0,
        const typename T::SoaVector3& tetDeformedPosition1,
        const typename T::SoaVector3& tetDeformedPosition2,
        const typename T::SoaVector3& tetDeformedPosition3,
        const typename T::SoaMatrix3& restVolumeMatrixInv,
        const typename T::SoaMatrix3& plasticDeformationMatrixInv)
    {
        typename T::SoaMatrix3 volumeMatrix = FmComputeTetVolumeMatrix<T>(tetDeformedPosition0, tetDeformedPosition1, tetDeformedPosition2, tetDeformedPosition3);

        typename T::SoaMatrix3 F = mul(mul(volumeMatrix, restVolumeMatrixInv), plasticDeformationMatrixInv);

        FmSvd3x3<T>(U, Fhat, V, F);
    }
}