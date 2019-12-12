/*
   Copyright (C) 2006-2010 Sony Computer Entertainment Inc.
   All rights reserved.

   Redistribution and use in source and binary forms,
   with or without modification, are permitted provided that the
   following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Sony Computer Entertainment Inc nor the names
      of its contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _VECTORMATH_MAT_AOS_CPP__SCALAR_H
#define _VECTORMATH_MAT_AOS_CPP__SCALAR_H

#include "vectormath_utils.h"

namespace FmVectormath {

//-----------------------------------------------------------------------------
// Constants

#define _VECTORMATH_PI_OVER_2 1.570796327f

//-----------------------------------------------------------------------------
// Definitions

        VECTORMATH_FORCE_INLINE Matrix3::Matrix3(const Matrix3 & mat)
        {
            col0 = mat.col0;
            col1 = mat.col1;
            col2 = mat.col2;
        }

        VECTORMATH_FORCE_INLINE Matrix3::Matrix3(float scalar)
        {
            col0 = Vector3(scalar);
            col1 = Vector3(scalar);
            col2 = Vector3(scalar);
        }

        VECTORMATH_FORCE_INLINE Matrix3::Matrix3(const Quat & unitQuat)
        {
            float qx, qy, qz, qw, qx2, qy2, qz2, qxqx2, qyqy2, qzqz2, qxqy2, qyqz2, qzqw2, qxqz2, qyqw2, qxqw2;
            qx = unitQuat.getX();
            qy = unitQuat.getY();
            qz = unitQuat.getZ();
            qw = unitQuat.getW();
            qx2 = (qx + qx);
            qy2 = (qy + qy);
            qz2 = (qz + qz);
            qxqx2 = (qx * qx2);
            qxqy2 = (qx * qy2);
            qxqz2 = (qx * qz2);
            qxqw2 = (qw * qx2);
            qyqy2 = (qy * qy2);
            qyqz2 = (qy * qz2);
            qyqw2 = (qw * qy2);
            qzqz2 = (qz * qz2);
            qzqw2 = (qw * qz2);
            col0 = Vector3(((1.0f - qyqy2) - qzqz2), (qxqy2 + qzqw2), (qxqz2 - qyqw2));
            col1 = Vector3((qxqy2 - qzqw2), ((1.0f - qxqx2) - qzqz2), (qyqz2 + qxqw2));
            col2 = Vector3((qxqz2 + qyqw2), (qyqz2 - qxqw2), ((1.0f - qxqx2) - qyqy2));
        }

        VECTORMATH_FORCE_INLINE Matrix3::Matrix3(const Vector3 & _col0, const Vector3 & _col1, const Vector3 & _col2)
        {
            col0 = _col0;
            col1 = _col1;
            col2 = _col2;
        }

        VECTORMATH_FORCE_INLINE Matrix3 & Matrix3::setCol0(const Vector3 & _col0)
        {
            col0 = _col0;
            return *this;
        }

        VECTORMATH_FORCE_INLINE Matrix3 & Matrix3::setCol1(const Vector3 & _col1)
        {
            col1 = _col1;
            return *this;
        }

        VECTORMATH_FORCE_INLINE Matrix3 & Matrix3::setCol2(const Vector3 & _col2)
        {
            col2 = _col2;
            return *this;
        }

        VECTORMATH_FORCE_INLINE Matrix3 & Matrix3::setCol(int col, const Vector3 & vec)
        {
            *(&col0 + col) = vec;
            return *this;
        }

        VECTORMATH_FORCE_INLINE Matrix3 & Matrix3::setRow(int row, const Vector3 & vec)
        {
            col0.setElem(row, vec.getElem(0));
            col1.setElem(row, vec.getElem(1));
            col2.setElem(row, vec.getElem(2));
            return *this;
        }

        VECTORMATH_FORCE_INLINE Matrix3 & Matrix3::setElem(int col, int row, float val)
        {
            Vector3 tmpV3_0;
            tmpV3_0 = this->getCol(col);
            tmpV3_0.setElem(row, val);
            this->setCol(col, tmpV3_0);
            return *this;
        }

        VECTORMATH_FORCE_INLINE float Matrix3::getElem(int col, int row) const
        {
            return this->getCol(col).getElem(row);
        }

        VECTORMATH_FORCE_INLINE const Vector3 Matrix3::getCol0() const
        {
            return col0;
        }

        VECTORMATH_FORCE_INLINE const Vector3 Matrix3::getCol1() const
        {
            return col1;
        }

        VECTORMATH_FORCE_INLINE const Vector3 Matrix3::getCol2() const
        {
            return col2;
        }

        VECTORMATH_FORCE_INLINE const Vector3 Matrix3::getCol(int col) const
        {
            return *(&col0 + col);
        }

        VECTORMATH_FORCE_INLINE const Vector3 Matrix3::getRow(int row) const
        {
            return Vector3(col0.getElem(row), col1.getElem(row), col2.getElem(row));
        }

        VECTORMATH_FORCE_INLINE Matrix3 & Matrix3::operator =(const Matrix3 & mat)
        {
            col0 = mat.col0;
            col1 = mat.col1;
            col2 = mat.col2;
            return *this;
        }

        VECTORMATH_FORCE_INLINE const Matrix3 transpose(const Matrix3 & mat)
        {
            return Matrix3(
                Vector3(mat.getCol0().getX(), mat.getCol1().getX(), mat.getCol2().getX()),
                Vector3(mat.getCol0().getY(), mat.getCol1().getY(), mat.getCol2().getY()),
                Vector3(mat.getCol0().getZ(), mat.getCol1().getZ(), mat.getCol2().getZ())
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix3 inverse(const Matrix3 & mat)
        {
            Vector3 tmp0, tmp1, tmp2;
            float detinv;
            tmp0 = cross(mat.getCol1(), mat.getCol2());
            tmp1 = cross(mat.getCol2(), mat.getCol0());
            tmp2 = cross(mat.getCol0(), mat.getCol1());
            detinv = (1.0f / dot(mat.getCol2(), tmp2));
            return Matrix3(
                Vector3((tmp0.getX() * detinv), (tmp1.getX() * detinv), (tmp2.getX() * detinv)),
                Vector3((tmp0.getY() * detinv), (tmp1.getY() * detinv), (tmp2.getY() * detinv)),
                Vector3((tmp0.getZ() * detinv), (tmp1.getZ() * detinv), (tmp2.getZ() * detinv))
                );
        }

        VECTORMATH_FORCE_INLINE float determinant(const Matrix3 & mat)
        {
            return dot(mat.getCol2(), cross(mat.getCol0(), mat.getCol1()));
        }

        VECTORMATH_FORCE_INLINE const Matrix3 Matrix3::operator +(const Matrix3 & mat) const
        {
            return Matrix3(
                (col0 + mat.col0),
                (col1 + mat.col1),
                (col2 + mat.col2)
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix3 Matrix3::operator -(const Matrix3 & mat) const
        {
            return Matrix3(
                (col0 - mat.col0),
                (col1 - mat.col1),
                (col2 - mat.col2)
                );
        }

        VECTORMATH_FORCE_INLINE Matrix3 & Matrix3::operator +=(const Matrix3 & mat)
        {
            *this = *this + mat;
            return *this;
        }

        VECTORMATH_FORCE_INLINE Matrix3 & Matrix3::operator -=(const Matrix3 & mat)
        {
            *this = *this - mat;
            return *this;
        }

        VECTORMATH_FORCE_INLINE const Matrix3 Matrix3::operator -() const
        {
            return Matrix3(
                (-col0),
                (-col1),
                (-col2)
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix3 abs(const Matrix3 & mat)
        {
            return Matrix3(
                abs(mat.getCol0()),
                abs(mat.getCol1()),
                abs(mat.getCol2())
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix3 Matrix3::operator *(float scalar) const
        {
            return Matrix3(
                (col0 * scalar),
                (col1 * scalar),
                (col2 * scalar)
                );
        }

        VECTORMATH_FORCE_INLINE Matrix3 & Matrix3::operator *=(float scalar)
        {
            *this = *this * scalar;
            return *this;
        }

        VECTORMATH_FORCE_INLINE const Matrix3 operator *(float scalar, const Matrix3 & mat)
        {
            return mat * scalar;
        }

        VECTORMATH_FORCE_INLINE const Vector3 mul(const Matrix3& mat, const Vector3 & vec)
        {
            return Vector3(
                (((mat.col0.getX() * vec.getX()) + (mat.col1.getX() * vec.getY())) + (mat.col2.getX() * vec.getZ())),
                (((mat.col0.getY() * vec.getX()) + (mat.col1.getY() * vec.getY())) + (mat.col2.getY() * vec.getZ())),
                (((mat.col0.getZ() * vec.getX()) + (mat.col1.getZ() * vec.getY())) + (mat.col2.getZ() * vec.getZ()))
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix3 mul(const Matrix3& mat0, const Matrix3 & mat1)
        {
            return Matrix3(
                mul(mat0, mat1.col0),
                mul(mat0, mat1.col1),
                mul(mat0, mat1.col2)
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix3 mulPerElem(const Matrix3 & mat0, const Matrix3 & mat1)
        {
            return Matrix3(
                mat0.getCol0() * mat1.getCol0(),
                mat0.getCol1() * mat1.getCol1(),
                mat0.getCol2() * mat1.getCol2()
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix3 Matrix3::identity()
        {
            return Matrix3(
                Vector3::xAxis(),
                Vector3::yAxis(),
                Vector3::zAxis()
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix3 Matrix3::rotationX(float radians)
        {
            float s, c;
            s = sinf(radians);
            c = cosf(radians);
            return Matrix3(
                Vector3::xAxis(),
                Vector3(0.0f, c, s),
                Vector3(0.0f, -s, c)
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix3 Matrix3::rotationY(float radians)
        {
            float s, c;
            s = sinf(radians);
            c = cosf(radians);
            return Matrix3(
                Vector3(c, 0.0f, -s),
                Vector3::yAxis(),
                Vector3(s, 0.0f, c)
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix3 Matrix3::rotationZ(float radians)
        {
            float s, c;
            s = sinf(radians);
            c = cosf(radians);
            return Matrix3(
                Vector3(c, s, 0.0f),
                Vector3(-s, c, 0.0f),
                Vector3::zAxis()
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix3 Matrix3::rotationZYX(const Vector3 & radiansXYZ)
        {
            float sX, cX, sY, cY, sZ, cZ, tmp0, tmp1;
            sX = sinf(radiansXYZ.getX());
            cX = cosf(radiansXYZ.getX());
            sY = sinf(radiansXYZ.getY());
            cY = cosf(radiansXYZ.getY());
            sZ = sinf(radiansXYZ.getZ());
            cZ = cosf(radiansXYZ.getZ());
            tmp0 = (cZ * sY);
            tmp1 = (sZ * sY);
            return Matrix3(
                Vector3((cZ * cY), (sZ * cY), -sY),
                Vector3(((tmp0 * sX) - (sZ * cX)), ((tmp1 * sX) + (cZ * cX)), (cY * sX)),
                Vector3(((tmp0 * cX) + (sZ * sX)), ((tmp1 * cX) - (cZ * sX)), (cY * cX))
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix3 Matrix3::rotation(float radians, const Vector3 & unitVec)
        {
            float x, y, z, s, c, oneMinusC, xy, yz, zx;
#if 1
            __m128 msin, mcos;
            simd_sincos_ps(_mm_set1_ps(radians), &msin, &mcos);
            s = simd_getx_ps(msin);
            c = simd_getx_ps(mcos);
#else
            s = sinf(radians);
            c = cosf(radians);
#endif
            x = unitVec.getX();
            y = unitVec.getY();
            z = unitVec.getZ();
            xy = (x * y);
            yz = (y * z);
            zx = (z * x);
            oneMinusC = (1.0f - c);
            return Matrix3(
                Vector3((((x * x) * oneMinusC) + c), ((xy * oneMinusC) + (z * s)), ((zx * oneMinusC) - (y * s))),
                Vector3(((xy * oneMinusC) - (z * s)), (((y * y) * oneMinusC) + c), ((yz * oneMinusC) + (x * s))),
                Vector3(((zx * oneMinusC) + (y * s)), ((yz * oneMinusC) - (x * s)), (((z * z) * oneMinusC) + c))
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix3 Matrix3::rotation(const Quat & unitQuat)
        {
            return Matrix3(unitQuat);
        }

        VECTORMATH_FORCE_INLINE const Matrix3 Matrix3::scale(const Vector3 & scaleVec)
        {
            return Matrix3(
                Vector3(scaleVec.getX(), 0.0f, 0.0f),
                Vector3(0.0f, scaleVec.getY(), 0.0f),
                Vector3(0.0f, 0.0f, scaleVec.getZ())
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix3 appendScale(const Matrix3 & mat, const Vector3 & scaleVec)
        {
            return Matrix3(
                (mat.getCol0() * scaleVec.getX()),
                (mat.getCol1() * scaleVec.getY()),
                (mat.getCol2() * scaleVec.getZ())
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix3 prependScale(const Vector3 & scaleVec, const Matrix3 & mat)
        {
            return Matrix3(
                mat.getCol0() * scaleVec,
                mat.getCol1() * scaleVec,
                mat.getCol2() * scaleVec
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix3 select(const Matrix3 & mat0, const Matrix3 & mat1, bool select1)
        {
            return Matrix3(
                select(mat0.getCol0(), mat1.getCol0(), select1),
                select(mat0.getCol1(), mat1.getCol1(), select1),
                select(mat0.getCol2(), mat1.getCol2(), select1)
                );
        }

#ifdef _VECTORMATH_DEBUG

        VECTORMATH_FORCE_INLINE void print(const Matrix3 & mat)
        {
            print(mat.getRow(0));
            print(mat.getRow(1));
            print(mat.getRow(2));
        }

        VECTORMATH_FORCE_INLINE void print(const Matrix3 & mat, const char * name)
        {
            printf("%s:\n", name);
            print(mat);
        }

#endif

        VECTORMATH_FORCE_INLINE Matrix4::Matrix4(const Matrix4 & mat)
        {
            col0 = mat.col0;
            col1 = mat.col1;
            col2 = mat.col2;
            col3 = mat.col3;
        }

        VECTORMATH_FORCE_INLINE Matrix4::Matrix4(float scalar)
        {
            col0 = Vector4(scalar);
            col1 = Vector4(scalar);
            col2 = Vector4(scalar);
            col3 = Vector4(scalar);
        }

        VECTORMATH_FORCE_INLINE Matrix4::Matrix4(const Transform3 & mat)
        {
            col0 = Vector4(mat.getCol0(), 0.0f);
            col1 = Vector4(mat.getCol1(), 0.0f);
            col2 = Vector4(mat.getCol2(), 0.0f);
            col3 = Vector4(mat.getCol3(), 1.0f);
        }

        VECTORMATH_FORCE_INLINE Matrix4::Matrix4(const Vector4 & _col0, const Vector4 & _col1, const Vector4 & _col2, const Vector4 & _col3)
        {
            col0 = _col0;
            col1 = _col1;
            col2 = _col2;
            col3 = _col3;
        }

        VECTORMATH_FORCE_INLINE Matrix4::Matrix4(const Matrix3 & mat, const Vector3 & translateVec)
        {
            col0 = Vector4(mat.getCol0(), 0.0f);
            col1 = Vector4(mat.getCol1(), 0.0f);
            col2 = Vector4(mat.getCol2(), 0.0f);
            col3 = Vector4(translateVec, 1.0f);
        }

        VECTORMATH_FORCE_INLINE Matrix4::Matrix4(const Quat & unitQuat, const Vector3 & translateVec)
        {
            Matrix3 mat;
            mat = Matrix3(unitQuat);
            col0 = Vector4(mat.getCol0(), 0.0f);
            col1 = Vector4(mat.getCol1(), 0.0f);
            col2 = Vector4(mat.getCol2(), 0.0f);
            col3 = Vector4(translateVec, 1.0f);
        }

        VECTORMATH_FORCE_INLINE Matrix4 & Matrix4::setCol0(const Vector4 & _col0)
        {
            col0 = _col0;
            return *this;
        }

        VECTORMATH_FORCE_INLINE Matrix4 & Matrix4::setCol1(const Vector4 & _col1)
        {
            col1 = _col1;
            return *this;
        }

        VECTORMATH_FORCE_INLINE Matrix4 & Matrix4::setCol2(const Vector4 & _col2)
        {
            col2 = _col2;
            return *this;
        }

        VECTORMATH_FORCE_INLINE Matrix4 & Matrix4::setCol3(const Vector4 & _col3)
        {
            col3 = _col3;
            return *this;
        }

        VECTORMATH_FORCE_INLINE Matrix4 & Matrix4::setCol(int col, const Vector4 & vec)
        {
            *(&col0 + col) = vec;
            return *this;
        }

        VECTORMATH_FORCE_INLINE Matrix4 & Matrix4::setRow(int row, const Vector4 & vec)
        {
            col0.setElem(row, vec.getElem(0));
            col1.setElem(row, vec.getElem(1));
            col2.setElem(row, vec.getElem(2));
            col3.setElem(row, vec.getElem(3));
            return *this;
        }

        VECTORMATH_FORCE_INLINE Matrix4 & Matrix4::setElem(int col, int row, float val)
        {
            Vector4 tmpV3_0;
            tmpV3_0 = this->getCol(col);
            tmpV3_0.setElem(row, val);
            this->setCol(col, tmpV3_0);
            return *this;
        }

        VECTORMATH_FORCE_INLINE float Matrix4::getElem(int col, int row) const
        {
            return this->getCol(col).getElem(row);
        }

        VECTORMATH_FORCE_INLINE const Vector4 Matrix4::getCol0() const
        {
            return col0;
        }

        VECTORMATH_FORCE_INLINE const Vector4 Matrix4::getCol1() const
        {
            return col1;
        }

        VECTORMATH_FORCE_INLINE const Vector4 Matrix4::getCol2() const
        {
            return col2;
        }

        VECTORMATH_FORCE_INLINE const Vector4 Matrix4::getCol3() const
        {
            return col3;
        }

        VECTORMATH_FORCE_INLINE const Vector4 Matrix4::getCol(int col) const
        {
            return *(&col0 + col);
        }

        VECTORMATH_FORCE_INLINE const Vector4 Matrix4::getRow(int row) const
        {
            return Vector4(col0.getElem(row), col1.getElem(row), col2.getElem(row), col3.getElem(row));
        }

        VECTORMATH_FORCE_INLINE Matrix4 & Matrix4::operator =(const Matrix4 & mat)
        {
            col0 = mat.col0;
            col1 = mat.col1;
            col2 = mat.col2;
            col3 = mat.col3;
            return *this;
        }

        VECTORMATH_FORCE_INLINE const Matrix4 transpose(const Matrix4 & mat)
        {
            return Matrix4(
                Vector4(mat.getCol0().getX(), mat.getCol1().getX(), mat.getCol2().getX(), mat.getCol3().getX()),
                Vector4(mat.getCol0().getY(), mat.getCol1().getY(), mat.getCol2().getY(), mat.getCol3().getY()),
                Vector4(mat.getCol0().getZ(), mat.getCol1().getZ(), mat.getCol2().getZ(), mat.getCol3().getZ()),
                Vector4(mat.getCol0().getW(), mat.getCol1().getW(), mat.getCol2().getW(), mat.getCol3().getW())
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 inverse(const Matrix4 & mat)
        {
            Vector4 res0, res1, res2, res3;
            float mA, mB, mC, mD, mE, mF, mG, mH, mI, mJ, mK, mL, mM, mN, mO, mP, tmp0, tmp1, tmp2, tmp3, tmp4, tmp5, detInv;
            mA = mat.getCol0().getX();
            mB = mat.getCol0().getY();
            mC = mat.getCol0().getZ();
            mD = mat.getCol0().getW();
            mE = mat.getCol1().getX();
            mF = mat.getCol1().getY();
            mG = mat.getCol1().getZ();
            mH = mat.getCol1().getW();
            mI = mat.getCol2().getX();
            mJ = mat.getCol2().getY();
            mK = mat.getCol2().getZ();
            mL = mat.getCol2().getW();
            mM = mat.getCol3().getX();
            mN = mat.getCol3().getY();
            mO = mat.getCol3().getZ();
            mP = mat.getCol3().getW();
            tmp0 = ((mK * mD) - (mC * mL));
            tmp1 = ((mO * mH) - (mG * mP));
            tmp2 = ((mB * mK) - (mJ * mC));
            tmp3 = ((mF * mO) - (mN * mG));
            tmp4 = ((mJ * mD) - (mB * mL));
            tmp5 = ((mN * mH) - (mF * mP));
            res0.setX((((mJ * tmp1) - (mL * tmp3)) - (mK * tmp5)));
            res0.setY((((mN * tmp0) - (mP * tmp2)) - (mO * tmp4)));
            res0.setZ((((mD * tmp3) + (mC * tmp5)) - (mB * tmp1)));
            res0.setW((((mH * tmp2) + (mG * tmp4)) - (mF * tmp0)));
            detInv = (1.0f / ((((mA * res0.getX()) + (mE * res0.getY())) + (mI * res0.getZ())) + (mM * res0.getW())));
            res1.setX((mI * tmp1));
            res1.setY((mM * tmp0));
            res1.setZ((mA * tmp1));
            res1.setW((mE * tmp0));
            res3.setX((mI * tmp3));
            res3.setY((mM * tmp2));
            res3.setZ((mA * tmp3));
            res3.setW((mE * tmp2));
            res2.setX((mI * tmp5));
            res2.setY((mM * tmp4));
            res2.setZ((mA * tmp5));
            res2.setW((mE * tmp4));
            tmp0 = ((mI * mB) - (mA * mJ));
            tmp1 = ((mM * mF) - (mE * mN));
            tmp2 = ((mI * mD) - (mA * mL));
            tmp3 = ((mM * mH) - (mE * mP));
            tmp4 = ((mI * mC) - (mA * mK));
            tmp5 = ((mM * mG) - (mE * mO));
            res2.setX((((mL * tmp1) - (mJ * tmp3)) + res2.getX()));
            res2.setY((((mP * tmp0) - (mN * tmp2)) + res2.getY()));
            res2.setZ((((mB * tmp3) - (mD * tmp1)) - res2.getZ()));
            res2.setW((((mF * tmp2) - (mH * tmp0)) - res2.getW()));
            res3.setX((((mJ * tmp5) - (mK * tmp1)) + res3.getX()));
            res3.setY((((mN * tmp4) - (mO * tmp0)) + res3.getY()));
            res3.setZ((((mC * tmp1) - (mB * tmp5)) - res3.getZ()));
            res3.setW((((mG * tmp0) - (mF * tmp4)) - res3.getW()));
            res1.setX((((mK * tmp3) - (mL * tmp5)) - res1.getX()));
            res1.setY((((mO * tmp2) - (mP * tmp4)) - res1.getY()));
            res1.setZ((((mD * tmp5) - (mC * tmp3)) + res1.getZ()));
            res1.setW((((mH * tmp4) - (mG * tmp2)) + res1.getW()));
            return Matrix4(
                (res0 * detInv),
                (res1 * detInv),
                (res2 * detInv),
                (res3 * detInv)
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 affineInverse(const Matrix4 & mat)
        {
            Transform3 affineMat;
            affineMat.setCol0(mat.getCol0().getXYZ());
            affineMat.setCol1(mat.getCol1().getXYZ());
            affineMat.setCol2(mat.getCol2().getXYZ());
            affineMat.setCol3(mat.getCol3().getXYZ());
            return Matrix4(inverse(affineMat));
        }

        VECTORMATH_FORCE_INLINE const Matrix4 orthoInverse(const Matrix4 & mat)
        {
            Transform3 affineMat;
            affineMat.setCol0(mat.getCol0().getXYZ());
            affineMat.setCol1(mat.getCol1().getXYZ());
            affineMat.setCol2(mat.getCol2().getXYZ());
            affineMat.setCol3(mat.getCol3().getXYZ());
            return Matrix4(orthoInverse(affineMat));
        }

        VECTORMATH_FORCE_INLINE float determinant(const Matrix4 & mat)
        {
            float dx, dy, dz, dw, mA, mB, mC, mD, mE, mF, mG, mH, mI, mJ, mK, mL, mM, mN, mO, mP, tmp0, tmp1, tmp2, tmp3, tmp4, tmp5;
            mA = mat.getCol0().getX();
            mB = mat.getCol0().getY();
            mC = mat.getCol0().getZ();
            mD = mat.getCol0().getW();
            mE = mat.getCol1().getX();
            mF = mat.getCol1().getY();
            mG = mat.getCol1().getZ();
            mH = mat.getCol1().getW();
            mI = mat.getCol2().getX();
            mJ = mat.getCol2().getY();
            mK = mat.getCol2().getZ();
            mL = mat.getCol2().getW();
            mM = mat.getCol3().getX();
            mN = mat.getCol3().getY();
            mO = mat.getCol3().getZ();
            mP = mat.getCol3().getW();
            tmp0 = ((mK * mD) - (mC * mL));
            tmp1 = ((mO * mH) - (mG * mP));
            tmp2 = ((mB * mK) - (mJ * mC));
            tmp3 = ((mF * mO) - (mN * mG));
            tmp4 = ((mJ * mD) - (mB * mL));
            tmp5 = ((mN * mH) - (mF * mP));
            dx = (((mJ * tmp1) - (mL * tmp3)) - (mK * tmp5));
            dy = (((mN * tmp0) - (mP * tmp2)) - (mO * tmp4));
            dz = (((mD * tmp3) + (mC * tmp5)) - (mB * tmp1));
            dw = (((mH * tmp2) + (mG * tmp4)) - (mF * tmp0));
            return ((((mA * dx) + (mE * dy)) + (mI * dz)) + (mM * dw));
        }

        VECTORMATH_FORCE_INLINE const Matrix4 Matrix4::operator +(const Matrix4 & mat) const
        {
            return Matrix4(
                (col0 + mat.col0),
                (col1 + mat.col1),
                (col2 + mat.col2),
                (col3 + mat.col3)
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 Matrix4::operator -(const Matrix4 & mat) const
        {
            return Matrix4(
                (col0 - mat.col0),
                (col1 - mat.col1),
                (col2 - mat.col2),
                (col3 - mat.col3)
                );
        }

        VECTORMATH_FORCE_INLINE Matrix4 & Matrix4::operator +=(const Matrix4 & mat)
        {
            *this = *this + mat;
            return *this;
        }

        VECTORMATH_FORCE_INLINE Matrix4 & Matrix4::operator -=(const Matrix4 & mat)
        {
            *this = *this - mat;
            return *this;
        }

        VECTORMATH_FORCE_INLINE const Matrix4 Matrix4::operator -() const
        {
            return Matrix4(
                (-col0),
                (-col1),
                (-col2),
                (-col3)
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 abs(const Matrix4 & mat)
        {
            return Matrix4(
                abs(mat.getCol0()),
                abs(mat.getCol1()),
                abs(mat.getCol2()),
                abs(mat.getCol3())
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 Matrix4::operator *(float scalar) const
        {
            return Matrix4(
                (col0 * scalar),
                (col1 * scalar),
                (col2 * scalar),
                (col3 * scalar)
                );
        }

        VECTORMATH_FORCE_INLINE Matrix4 & Matrix4::operator *=(float scalar)
        {
            *this = *this * scalar;
            return *this;
        }

        VECTORMATH_FORCE_INLINE const Matrix4 operator *(float scalar, const Matrix4 & mat)
        {
            return mat * scalar;
        }

        VECTORMATH_FORCE_INLINE const Vector4 mul(const Matrix4& mat, const Vector4 & vec)
        {
            return Vector4(
                ((((mat.col0.getX() * vec.getX()) + (mat.col1.getX() * vec.getY())) + (mat.col2.getX() * vec.getZ())) + (mat.col3.getX() * vec.getW())),
                ((((mat.col0.getY() * vec.getX()) + (mat.col1.getY() * vec.getY())) + (mat.col2.getY() * vec.getZ())) + (mat.col3.getY() * vec.getW())),
                ((((mat.col0.getZ() * vec.getX()) + (mat.col1.getZ() * vec.getY())) + (mat.col2.getZ() * vec.getZ())) + (mat.col3.getZ() * vec.getW())),
                ((((mat.col0.getW() * vec.getX()) + (mat.col1.getW() * vec.getY())) + (mat.col2.getW() * vec.getZ())) + (mat.col3.getW() * vec.getW()))
                );
        }

        VECTORMATH_FORCE_INLINE const Vector4 mul(const Matrix4& mat, const Vector3 & vec)
        {
            return Vector4(
                (((mat.col0.getX() * vec.getX()) + (mat.col1.getX() * vec.getY())) + (mat.col2.getX() * vec.getZ())),
                (((mat.col0.getY() * vec.getX()) + (mat.col1.getY() * vec.getY())) + (mat.col2.getY() * vec.getZ())),
                (((mat.col0.getZ() * vec.getX()) + (mat.col1.getZ() * vec.getY())) + (mat.col2.getZ() * vec.getZ())),
                (((mat.col0.getW() * vec.getX()) + (mat.col1.getW() * vec.getY())) + (mat.col2.getW() * vec.getZ()))
                );
        }

        VECTORMATH_FORCE_INLINE const Vector4 mul(const Matrix4& mat, const Point3 & pnt)
        {
            return Vector4(
                ((((mat.col0.getX() * pnt.getX()) + (mat.col1.getX() * pnt.getY())) + (mat.col2.getX() * pnt.getZ())) + mat.col3.getX()),
                ((((mat.col0.getY() * pnt.getX()) + (mat.col1.getY() * pnt.getY())) + (mat.col2.getY() * pnt.getZ())) + mat.col3.getY()),
                ((((mat.col0.getZ() * pnt.getX()) + (mat.col1.getZ() * pnt.getY())) + (mat.col2.getZ() * pnt.getZ())) + mat.col3.getZ()),
                ((((mat.col0.getW() * pnt.getX()) + (mat.col1.getW() * pnt.getY())) + (mat.col2.getW() * pnt.getZ())) + mat.col3.getW())
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 mul(const Matrix4& mat, const Transform3 & tfrm)
        {
            return Matrix4(
                mul(mat, tfrm.getCol0()),
                mul(mat, tfrm.getCol1()),
                mul(mat, tfrm.getCol2()),
                mul(mat, Point3(tfrm.getCol3()))
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 mul(const Matrix4& mat0, const Matrix4 & mat1)
        {
            return Matrix4(
                mul(mat0, mat1.col0),
                mul(mat0, mat1.col1),
                mul(mat0, mat1.col2),
                mul(mat0, mat1.col3)
                );
        }


        VECTORMATH_FORCE_INLINE const Matrix4 Matrix4::identity()
        {
            return Matrix4(
                Vector4::xAxis(),
                Vector4::yAxis(),
                Vector4::zAxis(),
                Vector4::wAxis()
                );
        }

        VECTORMATH_FORCE_INLINE Matrix4 & Matrix4::setUpper3x3(const Matrix3 & mat3)
        {
            col0.setXYZ(mat3.getCol0());
            col1.setXYZ(mat3.getCol1());
            col2.setXYZ(mat3.getCol2());
            return *this;
        }

        VECTORMATH_FORCE_INLINE const Matrix3 Matrix4::getUpper3x3() const
        {
            return Matrix3(
                col0.getXYZ(),
                col1.getXYZ(),
                col2.getXYZ()
                );
        }

        VECTORMATH_FORCE_INLINE Matrix4 & Matrix4::setTranslation(const Vector3 & translateVec)
        {
            col3.setXYZ(translateVec);
            return *this;
        }

        VECTORMATH_FORCE_INLINE const Vector3 Matrix4::getTranslation() const
        {
            return col3.getXYZ();
        }

        VECTORMATH_FORCE_INLINE const Matrix4 Matrix4::rotationX(float radians)
        {
            float s, c;
            s = sinf(radians);
            c = cosf(radians);
            return Matrix4(
                Vector4::xAxis(),
                Vector4(0.0f, c, s, 0.0f),
                Vector4(0.0f, -s, c, 0.0f),
                Vector4::wAxis()
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 Matrix4::rotationY(float radians)
        {
            float s, c;
            s = sinf(radians);
            c = cosf(radians);
            return Matrix4(
                Vector4(c, 0.0f, -s, 0.0f),
                Vector4::yAxis(),
                Vector4(s, 0.0f, c, 0.0f),
                Vector4::wAxis()
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 Matrix4::rotationZ(float radians)
        {
            float s, c;
            s = sinf(radians);
            c = cosf(radians);
            return Matrix4(
                Vector4(c, s, 0.0f, 0.0f),
                Vector4(-s, c, 0.0f, 0.0f),
                Vector4::zAxis(),
                Vector4::wAxis()
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 Matrix4::rotationZYX(const Vector3 & radiansXYZ)
        {
            float sX, cX, sY, cY, sZ, cZ, tmp0, tmp1;
            sX = sinf(radiansXYZ.getX());
            cX = cosf(radiansXYZ.getX());
            sY = sinf(radiansXYZ.getY());
            cY = cosf(radiansXYZ.getY());
            sZ = sinf(radiansXYZ.getZ());
            cZ = cosf(radiansXYZ.getZ());
            tmp0 = (cZ * sY);
            tmp1 = (sZ * sY);
            return Matrix4(
                Vector4((cZ * cY), (sZ * cY), -sY, 0.0f),
                Vector4(((tmp0 * sX) - (sZ * cX)), ((tmp1 * sX) + (cZ * cX)), (cY * sX), 0.0f),
                Vector4(((tmp0 * cX) + (sZ * sX)), ((tmp1 * cX) - (cZ * sX)), (cY * cX), 0.0f),
                Vector4::wAxis()
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 Matrix4::rotation(float radians, const Vector3 & unitVec)
        {
            float x, y, z, s, c, oneMinusC, xy, yz, zx;
            s = sinf(radians);
            c = cosf(radians);
            x = unitVec.getX();
            y = unitVec.getY();
            z = unitVec.getZ();
            xy = (x * y);
            yz = (y * z);
            zx = (z * x);
            oneMinusC = (1.0f - c);
            return Matrix4(
                Vector4((((x * x) * oneMinusC) + c), ((xy * oneMinusC) + (z * s)), ((zx * oneMinusC) - (y * s)), 0.0f),
                Vector4(((xy * oneMinusC) - (z * s)), (((y * y) * oneMinusC) + c), ((yz * oneMinusC) + (x * s)), 0.0f),
                Vector4(((zx * oneMinusC) + (y * s)), ((yz * oneMinusC) - (x * s)), (((z * z) * oneMinusC) + c), 0.0f),
                Vector4::wAxis()
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 Matrix4::rotation(const Quat & unitQuat)
        {
            return Matrix4(Transform3::rotation(unitQuat));
        }

        VECTORMATH_FORCE_INLINE const Matrix4 Matrix4::scale(const Vector3 & scaleVec)
        {
            return Matrix4(
                Vector4(scaleVec.getX(), 0.0f, 0.0f, 0.0f),
                Vector4(0.0f, scaleVec.getY(), 0.0f, 0.0f),
                Vector4(0.0f, 0.0f, scaleVec.getZ(), 0.0f),
                Vector4::wAxis()
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 appendScale(const Matrix4 & mat, const Vector3 & scaleVec)
        {
            return Matrix4(
                (mat.getCol0() * scaleVec.getX()),
                (mat.getCol1() * scaleVec.getY()),
                (mat.getCol2() * scaleVec.getZ()),
                mat.getCol3()
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 prependScale(const Vector3 & scaleVec, const Matrix4 & mat)
        {
            Vector4 scale4;
            scale4 = Vector4(scaleVec, 1.0f);
            return Matrix4(
                mat.getCol0() * scale4,
                mat.getCol1() * scale4,
                mat.getCol2() * scale4,
                mat.getCol3() * scale4
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 Matrix4::translation(const Vector3 & translateVec)
        {
            return Matrix4(
                Vector4::xAxis(),
                Vector4::yAxis(),
                Vector4::zAxis(),
                Vector4(translateVec, 1.0f)
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 Matrix4::lookAt(const Point3 & eyePos, const Point3 & lookAtPos, const Vector3 & upVec)
        {
            Matrix4 m4EyeFrame;
            Vector3 v3X, v3Y, v3Z;
            v3Y = normalize(upVec);
            v3Z = normalize((eyePos - lookAtPos));
            v3X = normalize(cross(v3Y, v3Z));
            v3Y = cross(v3Z, v3X);
            m4EyeFrame = Matrix4(Vector4(v3X), Vector4(v3Y), Vector4(v3Z), Vector4(eyePos));
            return orthoInverse(m4EyeFrame);
        }

        VECTORMATH_FORCE_INLINE const Matrix4 Matrix4::perspective(float fovyRadians, float aspect, float zNear, float zFar)
        {
            float f, rangeInv;
            f = tanf(((float)(_VECTORMATH_PI_OVER_2)-(0.5f * fovyRadians)));
            rangeInv = (1.0f / (zNear - zFar));
            return Matrix4(
                Vector4((f / aspect), 0.0f, 0.0f, 0.0f),
                Vector4(0.0f, f, 0.0f, 0.0f),
                Vector4(0.0f, 0.0f, ((zNear + zFar) * rangeInv), -1.0f),
                Vector4(0.0f, 0.0f, (((zNear * zFar) * rangeInv) * 2.0f), 0.0f)
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 Matrix4::frustum(float left, float right, float bottom, float top, float zNear, float zFar)
        {
            float sum_rl, sum_tb, sum_nf, inv_rl, inv_tb, inv_nf, n2;
            sum_rl = (right + left);
            sum_tb = (top + bottom);
            sum_nf = (zNear + zFar);
            inv_rl = (1.0f / (right - left));
            inv_tb = (1.0f / (top - bottom));
            inv_nf = (1.0f / (zNear - zFar));
            n2 = (zNear + zNear);
            return Matrix4(
                Vector4((n2 * inv_rl), 0.0f, 0.0f, 0.0f),
                Vector4(0.0f, (n2 * inv_tb), 0.0f, 0.0f),
                Vector4((sum_rl * inv_rl), (sum_tb * inv_tb), (sum_nf * inv_nf), -1.0f),
                Vector4(0.0f, 0.0f, ((n2 * inv_nf) * zFar), 0.0f)
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 Matrix4::orthographic(float left, float right, float bottom, float top, float zNear, float zFar)
        {
            float sum_rl, sum_tb, sum_nf, inv_rl, inv_tb, inv_nf;
            sum_rl = (right + left);
            sum_tb = (top + bottom);
            sum_nf = (zNear + zFar);
            inv_rl = (1.0f / (right - left));
            inv_tb = (1.0f / (top - bottom));
            inv_nf = (1.0f / (zNear - zFar));
            return Matrix4(
                Vector4((inv_rl + inv_rl), 0.0f, 0.0f, 0.0f),
                Vector4(0.0f, (inv_tb + inv_tb), 0.0f, 0.0f),
                Vector4(0.0f, 0.0f, (inv_nf + inv_nf), 0.0f),
                Vector4((-sum_rl * inv_rl), (-sum_tb * inv_tb), (sum_nf * inv_nf), 1.0f)
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 select(const Matrix4 & mat0, const Matrix4 & mat1, bool select1)
        {
            return Matrix4(
                select(mat0.getCol0(), mat1.getCol0(), select1),
                select(mat0.getCol1(), mat1.getCol1(), select1),
                select(mat0.getCol2(), mat1.getCol2(), select1),
                select(mat0.getCol3(), mat1.getCol3(), select1)
                );
        }

#ifdef _VECTORMATH_DEBUG

        VECTORMATH_FORCE_INLINE void print(const Matrix4 & mat)
        {
            print(mat.getRow(0));
            print(mat.getRow(1));
            print(mat.getRow(2));
            print(mat.getRow(3));
        }

        VECTORMATH_FORCE_INLINE void print(const Matrix4 & mat, const char * name)
        {
            printf("%s:\n", name);
            print(mat);
        }

#endif

        VECTORMATH_FORCE_INLINE Transform3::Transform3(const Transform3 & tfrm)
        {
            col0 = tfrm.col0;
            col1 = tfrm.col1;
            col2 = tfrm.col2;
            col3 = tfrm.col3;
        }

        VECTORMATH_FORCE_INLINE Transform3::Transform3(float scalar)
        {
            col0 = Vector3(scalar);
            col1 = Vector3(scalar);
            col2 = Vector3(scalar);
            col3 = Vector3(scalar);
        }

        VECTORMATH_FORCE_INLINE Transform3::Transform3(const Vector3 & _col0, const Vector3 & _col1, const Vector3 & _col2, const Vector3 & _col3)
        {
            col0 = _col0;
            col1 = _col1;
            col2 = _col2;
            col3 = _col3;
        }

        VECTORMATH_FORCE_INLINE Transform3::Transform3(const Matrix3 & tfrm, const Vector3 & translateVec)
        {
            this->setUpper3x3(tfrm);
            this->setTranslation(translateVec);
        }

        VECTORMATH_FORCE_INLINE Transform3::Transform3(const Quat & unitQuat, const Vector3 & translateVec)
        {
            this->setUpper3x3(Matrix3(unitQuat));
            this->setTranslation(translateVec);
        }

        VECTORMATH_FORCE_INLINE Transform3 & Transform3::setCol0(const Vector3 & _col0)
        {
            col0 = _col0;
            return *this;
        }

        VECTORMATH_FORCE_INLINE Transform3 & Transform3::setCol1(const Vector3 & _col1)
        {
            col1 = _col1;
            return *this;
        }

        VECTORMATH_FORCE_INLINE Transform3 & Transform3::setCol2(const Vector3 & _col2)
        {
            col2 = _col2;
            return *this;
        }

        VECTORMATH_FORCE_INLINE Transform3 & Transform3::setCol3(const Vector3 & _col3)
        {
            col3 = _col3;
            return *this;
        }

        VECTORMATH_FORCE_INLINE Transform3 & Transform3::setCol(int col, const Vector3 & vec)
        {
            *(&col0 + col) = vec;
            return *this;
        }

        VECTORMATH_FORCE_INLINE Transform3 & Transform3::setRow(int row, const Vector4 & vec)
        {
            col0.setElem(row, vec.getElem(0));
            col1.setElem(row, vec.getElem(1));
            col2.setElem(row, vec.getElem(2));
            col3.setElem(row, vec.getElem(3));
            return *this;
        }

        VECTORMATH_FORCE_INLINE Transform3 & Transform3::setElem(int col, int row, float val)
        {
            Vector3 tmpV3_0;
            tmpV3_0 = this->getCol(col);
            tmpV3_0.setElem(row, val);
            this->setCol(col, tmpV3_0);
            return *this;
        }

        VECTORMATH_FORCE_INLINE float Transform3::getElem(int col, int row) const
        {
            return this->getCol(col).getElem(row);
        }

        VECTORMATH_FORCE_INLINE const Vector3 Transform3::getCol0() const
        {
            return col0;
        }

        VECTORMATH_FORCE_INLINE const Vector3 Transform3::getCol1() const
        {
            return col1;
        }

        VECTORMATH_FORCE_INLINE const Vector3 Transform3::getCol2() const
        {
            return col2;
        }

        VECTORMATH_FORCE_INLINE const Vector3 Transform3::getCol3() const
        {
            return col3;
        }

        VECTORMATH_FORCE_INLINE const Vector3 Transform3::getCol(int col) const
        {
            return *(&col0 + col);
        }

        VECTORMATH_FORCE_INLINE const Vector4 Transform3::getRow(int row) const
        {
            return Vector4(col0.getElem(row), col1.getElem(row), col2.getElem(row), col3.getElem(row));
        }

        VECTORMATH_FORCE_INLINE Transform3 & Transform3::operator =(const Transform3 & tfrm)
        {
            col0 = tfrm.col0;
            col1 = tfrm.col1;
            col2 = tfrm.col2;
            col3 = tfrm.col3;
            return *this;
        }

        VECTORMATH_FORCE_INLINE const Transform3 inverse(const Transform3 & tfrm)
        {
            Vector3 tmp0, tmp1, tmp2, inv0, inv1, inv2;
            float detinv;
            tmp0 = cross(tfrm.getCol1(), tfrm.getCol2());
            tmp1 = cross(tfrm.getCol2(), tfrm.getCol0());
            tmp2 = cross(tfrm.getCol0(), tfrm.getCol1());
            detinv = (1.0f / dot(tfrm.getCol2(), tmp2));
            inv0 = Vector3((tmp0.getX() * detinv), (tmp1.getX() * detinv), (tmp2.getX() * detinv));
            inv1 = Vector3((tmp0.getY() * detinv), (tmp1.getY() * detinv), (tmp2.getY() * detinv));
            inv2 = Vector3((tmp0.getZ() * detinv), (tmp1.getZ() * detinv), (tmp2.getZ() * detinv));
            return Transform3(
                inv0,
                inv1,
                inv2,
                Vector3((-((inv0 * tfrm.getCol3().getX()) + ((inv1 * tfrm.getCol3().getY()) + (inv2 * tfrm.getCol3().getZ())))))
                );
        }

        VECTORMATH_FORCE_INLINE const Transform3 orthoInverse(const Transform3 & tfrm)
        {
            Vector3 inv0, inv1, inv2;
            inv0 = Vector3(tfrm.getCol0().getX(), tfrm.getCol1().getX(), tfrm.getCol2().getX());
            inv1 = Vector3(tfrm.getCol0().getY(), tfrm.getCol1().getY(), tfrm.getCol2().getY());
            inv2 = Vector3(tfrm.getCol0().getZ(), tfrm.getCol1().getZ(), tfrm.getCol2().getZ());
            return Transform3(
                inv0,
                inv1,
                inv2,
                Vector3((-((inv0 * tfrm.getCol3().getX()) + ((inv1 * tfrm.getCol3().getY()) + (inv2 * tfrm.getCol3().getZ())))))
                );
        }

        VECTORMATH_FORCE_INLINE const Transform3 abs(const Transform3 & tfrm)
        {
            return Transform3(
                abs(tfrm.getCol0()),
                abs(tfrm.getCol1()),
                abs(tfrm.getCol2()),
                abs(tfrm.getCol3())
                );
        }

        VECTORMATH_FORCE_INLINE const Vector3 mul(const Transform3& tfrm, const Vector3 & vec)
        {
            return Vector3(
                (((tfrm.col0.getX() * vec.getX()) + (tfrm.col1.getX() * vec.getY())) + (tfrm.col2.getX() * vec.getZ())),
                (((tfrm.col0.getY() * vec.getX()) + (tfrm.col1.getY() * vec.getY())) + (tfrm.col2.getY() * vec.getZ())),
                (((tfrm.col0.getZ() * vec.getX()) + (tfrm.col1.getZ() * vec.getY())) + (tfrm.col2.getZ() * vec.getZ()))
                );
        }

        VECTORMATH_FORCE_INLINE const Point3 mul(const Transform3& tfrm, const Point3 & pnt)
        {
            return Point3(
                ((((tfrm.col0.getX() * pnt.getX()) + (tfrm.col1.getX() * pnt.getY())) + (tfrm.col2.getX() * pnt.getZ())) + tfrm.col3.getX()),
                ((((tfrm.col0.getY() * pnt.getX()) + (tfrm.col1.getY() * pnt.getY())) + (tfrm.col2.getY() * pnt.getZ())) + tfrm.col3.getY()),
                ((((tfrm.col0.getZ() * pnt.getX()) + (tfrm.col1.getZ() * pnt.getY())) + (tfrm.col2.getZ() * pnt.getZ())) + tfrm.col3.getZ())
                );
        }

        VECTORMATH_FORCE_INLINE const Transform3 mul(const Transform3 & tfrm0, const Transform3 & tfrm1)
        {
            return Transform3(
                mul(tfrm0, tfrm1.col0),
                mul(tfrm0, tfrm1.col1),
                mul(tfrm0, tfrm1.col2),
                Vector3(mul(tfrm0, Point3(tfrm1.col3)))
                );
        }

        VECTORMATH_FORCE_INLINE const Transform3 mulPerElem(const Transform3 & tfrm0, const Transform3 & tfrm1)
        {
            return Transform3(
                tfrm0.getCol0() * tfrm1.getCol0(),
                tfrm0.getCol1() * tfrm1.getCol1(),
                tfrm0.getCol2() * tfrm1.getCol2(),
                tfrm0.getCol3() * tfrm1.getCol3()
                );
        }

        VECTORMATH_FORCE_INLINE const Transform3 Transform3::identity()
        {
            return Transform3(
                Vector3::xAxis(),
                Vector3::yAxis(),
                Vector3::zAxis(),
                Vector3(0.0f)
                );
        }

        VECTORMATH_FORCE_INLINE Transform3 & Transform3::setUpper3x3(const Matrix3 & tfrm)
        {
            col0 = tfrm.getCol0();
            col1 = tfrm.getCol1();
            col2 = tfrm.getCol2();
            return *this;
        }

        VECTORMATH_FORCE_INLINE const Matrix3 Transform3::getUpper3x3() const
        {
            return Matrix3(col0, col1, col2);
        }

        VECTORMATH_FORCE_INLINE Transform3 & Transform3::setTranslation(const Vector3 & translateVec)
        {
            col3 = translateVec;
            return *this;
        }

        VECTORMATH_FORCE_INLINE const Vector3 Transform3::getTranslation() const
        {
            return col3;
        }

        VECTORMATH_FORCE_INLINE const Transform3 Transform3::rotationX(float radians)
        {
            float s, c;
            s = sinf(radians);
            c = cosf(radians);
            return Transform3(
                Vector3::xAxis(),
                Vector3(0.0f, c, s),
                Vector3(0.0f, -s, c),
                Vector3(0.0f)
                );
        }

        VECTORMATH_FORCE_INLINE const Transform3 Transform3::rotationY(float radians)
        {
            float s, c;
            s = sinf(radians);
            c = cosf(radians);
            return Transform3(
                Vector3(c, 0.0f, -s),
                Vector3::yAxis(),
                Vector3(s, 0.0f, c),
                Vector3(0.0f)
                );
        }

        VECTORMATH_FORCE_INLINE const Transform3 Transform3::rotationZ(float radians)
        {
            float s, c;
            s = sinf(radians);
            c = cosf(radians);
            return Transform3(
                Vector3(c, s, 0.0f),
                Vector3(-s, c, 0.0f),
                Vector3::zAxis(),
                Vector3(0.0f)
                );
        }

        VECTORMATH_FORCE_INLINE const Transform3 Transform3::rotationZYX(const Vector3 & radiansXYZ)
        {
            float sX, cX, sY, cY, sZ, cZ, tmp0, tmp1;
            sX = sinf(radiansXYZ.getX());
            cX = cosf(radiansXYZ.getX());
            sY = sinf(radiansXYZ.getY());
            cY = cosf(radiansXYZ.getY());
            sZ = sinf(radiansXYZ.getZ());
            cZ = cosf(radiansXYZ.getZ());
            tmp0 = (cZ * sY);
            tmp1 = (sZ * sY);
            return Transform3(
                Vector3((cZ * cY), (sZ * cY), -sY),
                Vector3(((tmp0 * sX) - (sZ * cX)), ((tmp1 * sX) + (cZ * cX)), (cY * sX)),
                Vector3(((tmp0 * cX) + (sZ * sX)), ((tmp1 * cX) - (cZ * sX)), (cY * cX)),
                Vector3(0.0f)
                );
        }

        VECTORMATH_FORCE_INLINE const Transform3 Transform3::rotation(float radians, const Vector3 & unitVec)
        {
            return Transform3(Matrix3::rotation(radians, unitVec), Vector3(0.0f));
        }

        VECTORMATH_FORCE_INLINE const Transform3 Transform3::rotation(const Quat & unitQuat)
        {
            return Transform3(Matrix3(unitQuat), Vector3(0.0f));
        }

        VECTORMATH_FORCE_INLINE const Transform3 Transform3::scale(const Vector3 & scaleVec)
        {
            return Transform3(
                Vector3(scaleVec.getX(), 0.0f, 0.0f),
                Vector3(0.0f, scaleVec.getY(), 0.0f),
                Vector3(0.0f, 0.0f, scaleVec.getZ()),
                Vector3(0.0f)
                );
        }

        VECTORMATH_FORCE_INLINE const Transform3 appendScale(const Transform3 & tfrm, const Vector3 & scaleVec)
        {
            return Transform3(
                (tfrm.getCol0() * scaleVec.getX()),
                (tfrm.getCol1() * scaleVec.getY()),
                (tfrm.getCol2() * scaleVec.getZ()),
                tfrm.getCol3()
                );
        }

        VECTORMATH_FORCE_INLINE const Transform3 prependScale(const Vector3 & scaleVec, const Transform3 & tfrm)
        {
            return Transform3(
                tfrm.getCol0() * scaleVec,
                tfrm.getCol1() * scaleVec,
                tfrm.getCol2() * scaleVec,
                tfrm.getCol3() * scaleVec
                );
        }

        VECTORMATH_FORCE_INLINE const Transform3 Transform3::translation(const Vector3 & translateVec)
        {
            return Transform3(
                Vector3::xAxis(),
                Vector3::yAxis(),
                Vector3::zAxis(),
                translateVec
                );
        }

        VECTORMATH_FORCE_INLINE const Transform3 select(const Transform3 & tfrm0, const Transform3 & tfrm1, bool select1)
        {
            return Transform3(
                select(tfrm0.getCol0(), tfrm1.getCol0(), select1),
                select(tfrm0.getCol1(), tfrm1.getCol1(), select1),
                select(tfrm0.getCol2(), tfrm1.getCol2(), select1),
                select(tfrm0.getCol3(), tfrm1.getCol3(), select1)
                );
        }

#ifdef _VECTORMATH_DEBUG

        VECTORMATH_FORCE_INLINE void print(const Transform3 & tfrm)
        {
            print(tfrm.getRow(0));
            print(tfrm.getRow(1));
            print(tfrm.getRow(2));
        }

        VECTORMATH_FORCE_INLINE void print(const Transform3 & tfrm, const char * name)
        {
            printf("%s:\n", name);
            print(tfrm);
        }

#endif

        VECTORMATH_FORCE_INLINE Quat::Quat(const Matrix3 & tfrm)
        {
            float trace, radicand, scale, xx, yx, zx, xy, yy, zy, xz, yz, zz, tmpx, tmpy, tmpz, tmpw, qx, qy, qz, qw;
            int negTrace, ZgtX, ZgtY, YgtX;
            int largestXorY, largestYorZ, largestZorX;

            xx = tfrm.getCol0().getX();
            yx = tfrm.getCol0().getY();
            zx = tfrm.getCol0().getZ();
            xy = tfrm.getCol1().getX();
            yy = tfrm.getCol1().getY();
            zy = tfrm.getCol1().getZ();
            xz = tfrm.getCol2().getX();
            yz = tfrm.getCol2().getY();
            zz = tfrm.getCol2().getZ();

            trace = ((xx + yy) + zz);

            negTrace = (trace < 0.0f);
            ZgtX = zz > xx;
            ZgtY = zz > yy;
            YgtX = yy > xx;
            largestXorY = (!ZgtX || !ZgtY) && negTrace;
            largestYorZ = (YgtX || ZgtX) && negTrace;
            largestZorX = (ZgtY || !YgtX) && negTrace;

            if (largestXorY)
            {
                zz = -zz;
                xy = -xy;
            }
            if (largestYorZ)
            {
                xx = -xx;
                yz = -yz;
            }
            if (largestZorX)
            {
                yy = -yy;
                zx = -zx;
            }

            radicand = (((xx + yy) + zz) + 1.0f);
            scale = (0.5f * (1.0f / sqrtf(radicand)));

            tmpx = ((zy - yz) * scale);
            tmpy = ((xz - zx) * scale);
            tmpz = ((yx - xy) * scale);
            tmpw = (radicand * scale);
            qx = tmpx;
            qy = tmpy;
            qz = tmpz;
            qw = tmpw;

            if (largestXorY)
            {
                qx = tmpw;
                qy = tmpz;
                qz = tmpy;
                qw = tmpx;
            }
            if (largestYorZ)
            {
                tmpx = qx;
                tmpz = qz;
                qx = qy;
                qy = tmpx;
                qz = qw;
                qw = tmpz;
            }

            x = qx;
            y = qy;
            z = qz;
            w = qw;
        }

        VECTORMATH_FORCE_INLINE const Matrix3 outer(const Vector3 & tfrm0, const Vector3 & tfrm1)
        {
            return Matrix3(
                (tfrm0 * tfrm1.getX()),
                (tfrm0 * tfrm1.getY()),
                (tfrm0 * tfrm1.getZ())
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix4 outer(const Vector4 & tfrm0, const Vector4 & tfrm1)
        {
            return Matrix4(
                (tfrm0 * tfrm1.getX()),
                (tfrm0 * tfrm1.getY()),
                (tfrm0 * tfrm1.getZ()),
                (tfrm0 * tfrm1.getW())
                );
        }

        VECTORMATH_FORCE_INLINE const Vector3 mul(const Vector3 & vec, const Matrix3 & mat)
        {
            return Vector3(
                (((vec.getX() * mat.getCol0().getX()) + (vec.getY() * mat.getCol0().getY())) + (vec.getZ() * mat.getCol0().getZ())),
                (((vec.getX() * mat.getCol1().getX()) + (vec.getY() * mat.getCol1().getY())) + (vec.getZ() * mat.getCol1().getZ())),
                (((vec.getX() * mat.getCol2().getX()) + (vec.getY() * mat.getCol2().getY())) + (vec.getZ() * mat.getCol2().getZ()))
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix3 crossMatrix(const Vector3 & vec)
        {
            return Matrix3(
                Vector3(0.0f, vec.getZ(), -vec.getY()),
                Vector3(-vec.getZ(), 0.0f, vec.getX()),
                Vector3(vec.getY(), -vec.getX(), 0.0f)
                );
        }

        VECTORMATH_FORCE_INLINE const Matrix3 crossMatrixMul(const Vector3 & vec, const Matrix3 & mat)
        {
            return Matrix3(cross(vec, mat.getCol0()), cross(vec, mat.getCol1()), cross(vec, mat.getCol2()));
        }

} // namespace FmVectormath

#endif
