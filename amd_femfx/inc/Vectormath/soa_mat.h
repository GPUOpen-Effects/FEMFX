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

#ifndef _SOA_VECTORMATH_MAT_H
#define _SOA_VECTORMATH_MAT_H

namespace FmVectormath {

        //-----------------------------------------------------------------------------
        // Constants

#define _VECTORMATH_PI_OVER_2 1.570796327f

//-----------------------------------------------------------------------------
// Definitions

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix3<SoaNFloat>::SoaMatrix3(const SoaMatrix3<SoaNFloat> & mat)
        {
            col0 = mat.col0;
            col1 = mat.col1;
            col2 = mat.col2;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix3<SoaNFloat>::SoaMatrix3(SoaNFloat scalar)
        {
            col0 = SoaVector3<SoaNFloat>(scalar);
            col1 = SoaVector3<SoaNFloat>(scalar);
            col2 = SoaVector3<SoaNFloat>(scalar);
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix3<SoaNFloat>::SoaMatrix3(const SoaQuat<SoaNFloat> & unitQuat)
        {
            SoaNFloat qx, qy, qz, qw, qx2, qy2, qz2, qxqx2, qyqy2, qzqz2, qxqy2, qyqz2, qzqw2, qxqz2, qyqw2, qxqw2;
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
            col0 = SoaVector3<SoaNFloat>(((1.0f - qyqy2) - qzqz2), (qxqy2 + qzqw2), (qxqz2 - qyqw2));
            col1 = SoaVector3<SoaNFloat>((qxqy2 - qzqw2), ((1.0f - qxqx2) - qzqz2), (qyqz2 + qxqw2));
            col2 = SoaVector3<SoaNFloat>((qxqz2 + qyqw2), (qyqz2 - qxqw2), ((1.0f - qxqx2) - qyqy2));
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix3<SoaNFloat>::SoaMatrix3(const SoaVector3<SoaNFloat> & _col0, const SoaVector3<SoaNFloat> & _col1, const SoaVector3<SoaNFloat> & _col2)
        {
            col0 = _col0;
            col1 = _col1;
            col2 = _col2;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix3<SoaNFloat> & SoaMatrix3<SoaNFloat>::setCol0(const SoaVector3<SoaNFloat> & _col0)
        {
            col0 = _col0;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix3<SoaNFloat> & SoaMatrix3<SoaNFloat>::setCol1(const SoaVector3<SoaNFloat> & _col1)
        {
            col1 = _col1;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix3<SoaNFloat> & SoaMatrix3<SoaNFloat>::setCol2(const SoaVector3<SoaNFloat> & _col2)
        {
            col2 = _col2;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix3<SoaNFloat> & SoaMatrix3<SoaNFloat>::setCol(int col, const SoaVector3<SoaNFloat> & vec)
        {
            *(&col0 + col) = vec;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix3<SoaNFloat> & SoaMatrix3<SoaNFloat>::setRow(int row, const SoaVector3<SoaNFloat> & vec)
        {
            col0.setElem(row, vec.getElem(0));
            col1.setElem(row, vec.getElem(1));
            col2.setElem(row, vec.getElem(2));
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix3<SoaNFloat> & SoaMatrix3<SoaNFloat>::setElem(int col, int row, SoaNFloat val)
        {
            SoaVector3<SoaNFloat> tmpV3_0;
            tmpV3_0 = this->getCol(col);
            tmpV3_0.setElem(row, val);
            this->setCol(col, tmpV3_0);
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaMatrix3<SoaNFloat>::getElem(int col, int row) const
        {
            return this->getCol(col).getElem(row);
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaMatrix3<SoaNFloat>::getCol0() const
        {
            return col0;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaMatrix3<SoaNFloat>::getCol1() const
        {
            return col1;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaMatrix3<SoaNFloat>::getCol2() const
        {
            return col2;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaMatrix3<SoaNFloat>::getCol(int col) const
        {
            return *(&col0 + col);
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaMatrix3<SoaNFloat>::getRow(int row) const
        {
            return SoaVector3<SoaNFloat>(col0.getElem(row), col1.getElem(row), col2.getElem(row));
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix3<SoaNFloat> & SoaMatrix3<SoaNFloat>::operator =(const SoaMatrix3<SoaNFloat> & mat)
        {
            col0 = mat.col0;
            col1 = mat.col1;
            col2 = mat.col2;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> transpose(const SoaMatrix3<SoaNFloat> & mat)
        {
            return SoaMatrix3<SoaNFloat>(
                SoaVector3<SoaNFloat>(mat.getCol0().getX(), mat.getCol1().getX(), mat.getCol2().getX()),
                SoaVector3<SoaNFloat>(mat.getCol0().getY(), mat.getCol1().getY(), mat.getCol2().getY()),
                SoaVector3<SoaNFloat>(mat.getCol0().getZ(), mat.getCol1().getZ(), mat.getCol2().getZ())
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> inverse(const SoaMatrix3<SoaNFloat> & mat)
        {
            SoaVector3<SoaNFloat> tmp0, tmp1, tmp2;
            SoaNFloat detinv;
            tmp0 = cross(mat.getCol1(), mat.getCol2());
            tmp1 = cross(mat.getCol2(), mat.getCol0());
            tmp2 = cross(mat.getCol0(), mat.getCol1());
            detinv = (1.0f / dot(mat.getCol2(), tmp2));
            return SoaMatrix3<SoaNFloat>(
                SoaVector3<SoaNFloat>((tmp0.getX() * detinv), (tmp1.getX() * detinv), (tmp2.getX() * detinv)),
                SoaVector3<SoaNFloat>((tmp0.getY() * detinv), (tmp1.getY() * detinv), (tmp2.getY() * detinv)),
                SoaVector3<SoaNFloat>((tmp0.getZ() * detinv), (tmp1.getZ() * detinv), (tmp2.getZ() * detinv))
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat determinant(const SoaMatrix3<SoaNFloat> & mat)
        {
            return dot(mat.getCol2(), cross(mat.getCol0(), mat.getCol1()));
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> SoaMatrix3<SoaNFloat>::operator +(const SoaMatrix3<SoaNFloat> & mat) const
        {
            return SoaMatrix3<SoaNFloat>(
                (col0 + mat.col0),
                (col1 + mat.col1),
                (col2 + mat.col2)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> SoaMatrix3<SoaNFloat>::operator -(const SoaMatrix3<SoaNFloat> & mat) const
        {
            return SoaMatrix3<SoaNFloat>(
                (col0 - mat.col0),
                (col1 - mat.col1),
                (col2 - mat.col2)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix3<SoaNFloat> & SoaMatrix3<SoaNFloat>::operator +=(const SoaMatrix3<SoaNFloat> & mat)
        {
            *this = *this + mat;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix3<SoaNFloat> & SoaMatrix3<SoaNFloat>::operator -=(const SoaMatrix3<SoaNFloat> & mat)
        {
            *this = *this - mat;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> SoaMatrix3<SoaNFloat>::operator -() const
        {
            return SoaMatrix3<SoaNFloat>(
                (-col0),
                (-col1),
                (-col2)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> abs(const SoaMatrix3<SoaNFloat> & mat)
        {
            return SoaMatrix3<SoaNFloat>(
                abs(mat.getCol0()),
                abs(mat.getCol1()),
                abs(mat.getCol2())
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> SoaMatrix3<SoaNFloat>::operator *(SoaNFloat scalar) const
        {
            return SoaMatrix3<SoaNFloat>(
                (col0 * scalar),
                (col1 * scalar),
                (col2 * scalar)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix3<SoaNFloat> & SoaMatrix3<SoaNFloat>::operator *=(SoaNFloat scalar)
        {
            *this = *this * scalar;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> operator *(SoaNFloat scalar, const SoaMatrix3<SoaNFloat> & mat)
        {
            return mat * scalar;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> mul(const SoaMatrix3<SoaNFloat>& mat, const SoaVector3<SoaNFloat> & vec)
        {
            return SoaVector3<SoaNFloat>(
                (((mat.col0.getX() * vec.getX()) + (mat.col1.getX() * vec.getY())) + (mat.col2.getX() * vec.getZ())),
                (((mat.col0.getY() * vec.getX()) + (mat.col1.getY() * vec.getY())) + (mat.col2.getY() * vec.getZ())),
                (((mat.col0.getZ() * vec.getX()) + (mat.col1.getZ() * vec.getY())) + (mat.col2.getZ() * vec.getZ()))
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> mul(const SoaMatrix3<SoaNFloat>& mat0, const SoaMatrix3<SoaNFloat> & mat1)
        {
            return SoaMatrix3<SoaNFloat>(
                mul(mat0, mat1.col0),
                mul(mat0, mat1.col1),
                mul(mat0, mat1.col2)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> mulPerElem(const SoaMatrix3<SoaNFloat> & mat0, const SoaMatrix3<SoaNFloat> & mat1)
        {
            return SoaMatrix3<SoaNFloat>(
                mat0.getCol0() * mat1.getCol0(),
                mat0.getCol1() * mat1.getCol1(),
                mat0.getCol2() * mat1.getCol2()
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> SoaMatrix3<SoaNFloat>::identity()
        {
            return SoaMatrix3<SoaNFloat>(
                SoaVector3<SoaNFloat>::xAxis(),
                SoaVector3<SoaNFloat>::yAxis(),
                SoaVector3<SoaNFloat>::zAxis()
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> SoaMatrix3<SoaNFloat>::rotationX(SoaNFloat radians)
        {
            SoaNFloat s, c;
            sincosf(radians, &s, &c);
            return SoaMatrix3<SoaNFloat>(
                SoaVector3<SoaNFloat>::xAxis(),
                SoaVector3<SoaNFloat>(0.0f, c, s),
                SoaVector3<SoaNFloat>(0.0f, -s, c)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> SoaMatrix3<SoaNFloat>::rotationY(SoaNFloat radians)
        {
            SoaNFloat s, c;
            sincosf(radians, &s, &c);
            return SoaMatrix3<SoaNFloat>(
                SoaVector3<SoaNFloat>(c, 0.0f, -s),
                SoaVector3<SoaNFloat>::yAxis(),
                SoaVector3<SoaNFloat>(s, 0.0f, c)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> SoaMatrix3<SoaNFloat>::rotationZ(SoaNFloat radians)
        {
            SoaNFloat s, c;
            sincosf(radians, &s, &c);
            return SoaMatrix3<SoaNFloat>(
                SoaVector3<SoaNFloat>(c, s, 0.0f),
                SoaVector3<SoaNFloat>(-s, c, 0.0f),
                SoaVector3<SoaNFloat>::zAxis()
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> SoaMatrix3<SoaNFloat>::rotationZYX(const SoaVector3<SoaNFloat> & radiansXYZ)
        {
            SoaNFloat sX, cX, sY, cY, sZ, cZ, tmp0, tmp1;
            sincosf(radiansXYZ.getX(), &sX, &cX);
            sincosf(radiansXYZ.getY(), &sY, &cY);
            sincosf(radiansXYZ.getZ(), &sZ, &cZ);
            tmp0 = (cZ * sY);
            tmp1 = (sZ * sY);
            return SoaMatrix3<SoaNFloat>(
                SoaVector3<SoaNFloat>((cZ * cY), (sZ * cY), -sY),
                SoaVector3<SoaNFloat>(((tmp0 * sX) - (sZ * cX)), ((tmp1 * sX) + (cZ * cX)), (cY * sX)),
                SoaVector3<SoaNFloat>(((tmp0 * cX) + (sZ * sX)), ((tmp1 * cX) - (cZ * sX)), (cY * cX))
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> SoaMatrix3<SoaNFloat>::rotation(SoaNFloat radians, const SoaVector3<SoaNFloat> & unitVec)
        {
            SoaNFloat x, y, z, s, c, oneMinusC, xy, yz, zx;
            sincosf(radians, &s, &c);
            x = unitVec.getX();
            y = unitVec.getY();
            z = unitVec.getZ();
            xy = (x * y);
            yz = (y * z);
            zx = (z * x);
            oneMinusC = (1.0f - c);
            return SoaMatrix3<SoaNFloat>(
                SoaVector3<SoaNFloat>((((x * x) * oneMinusC) + c), ((xy * oneMinusC) + (z * s)), ((zx * oneMinusC) - (y * s))),
                SoaVector3<SoaNFloat>(((xy * oneMinusC) - (z * s)), (((y * y) * oneMinusC) + c), ((yz * oneMinusC) + (x * s))),
                SoaVector3<SoaNFloat>(((zx * oneMinusC) + (y * s)), ((yz * oneMinusC) - (x * s)), (((z * z) * oneMinusC) + c))
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> SoaMatrix3<SoaNFloat>::rotation(const SoaQuat<SoaNFloat> & unitQuat)
        {
            return SoaMatrix3<SoaNFloat>(unitQuat);
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> SoaMatrix3<SoaNFloat>::scale(const SoaVector3<SoaNFloat> & scaleVec)
        {
            return SoaMatrix3<SoaNFloat>(
                SoaVector3<SoaNFloat>(scaleVec.getX(), 0.0f, 0.0f),
                SoaVector3<SoaNFloat>(0.0f, scaleVec.getY(), 0.0f),
                SoaVector3<SoaNFloat>(0.0f, 0.0f, scaleVec.getZ())
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> appendScale(const SoaMatrix3<SoaNFloat> & mat, const SoaVector3<SoaNFloat> & scaleVec)
        {
            return SoaMatrix3<SoaNFloat>(
                (mat.getCol0() * scaleVec.getX()),
                (mat.getCol1() * scaleVec.getY()),
                (mat.getCol2() * scaleVec.getZ())
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> prependScale(const SoaVector3<SoaNFloat> & scaleVec, const SoaMatrix3<SoaNFloat> & mat)
        {
            return SoaMatrix3<SoaNFloat>(
                mat.getCol0() * scaleVec,
                mat.getCol1() * scaleVec,
                mat.getCol2() * scaleVec
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> select(const SoaMatrix3<SoaNFloat> & mat0, const SoaMatrix3<SoaNFloat> & mat1, typename SoaNFloat::SoaBool select1)
        {
            return SoaMatrix3<SoaNFloat>(
                select(mat0.getCol0(), mat1.getCol0(), select1),
                select(mat0.getCol1(), mat1.getCol1(), select1),
                select(mat0.getCol2(), mat1.getCol2(), select1)
                );
        }

#ifdef _VECTORMATH_DEBUG

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print(const SoaMatrix3<SoaNFloat> & mat)
        {
            print(mat.getRow(0));
            print(mat.getRow(1));
            print(mat.getRow(2));
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print(const SoaMatrix3<SoaNFloat> & mat, const char * name)
        {
            printf("%s:\n", name);
            print(mat);
        }

#endif

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix4<SoaNFloat>::SoaMatrix4(const SoaMatrix4<SoaNFloat> & mat)
        {
            col0 = mat.col0;
            col1 = mat.col1;
            col2 = mat.col2;
            col3 = mat.col3;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix4<SoaNFloat>::SoaMatrix4(SoaNFloat scalar)
        {
            col0 = SoaVector4<SoaNFloat>(scalar);
            col1 = SoaVector4<SoaNFloat>(scalar);
            col2 = SoaVector4<SoaNFloat>(scalar);
            col3 = SoaVector4<SoaNFloat>(scalar);
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix4<SoaNFloat>::SoaMatrix4(const SoaTransform3<SoaNFloat> & mat)
        {
            col0 = SoaVector4<SoaNFloat>(mat.getCol0(), 0.0f);
            col1 = SoaVector4<SoaNFloat>(mat.getCol1(), 0.0f);
            col2 = SoaVector4<SoaNFloat>(mat.getCol2(), 0.0f);
            col3 = SoaVector4<SoaNFloat>(mat.getCol3(), 1.0f);
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix4<SoaNFloat>::SoaMatrix4(const SoaVector4<SoaNFloat> & _col0, const SoaVector4<SoaNFloat> & _col1, const SoaVector4<SoaNFloat> & _col2, const SoaVector4<SoaNFloat> & _col3)
        {
            col0 = _col0;
            col1 = _col1;
            col2 = _col2;
            col3 = _col3;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix4<SoaNFloat>::SoaMatrix4(const SoaMatrix3<SoaNFloat> & mat, const SoaVector3<SoaNFloat> & translateVec)
        {
            col0 = SoaVector4<SoaNFloat>(mat.getCol0(), 0.0f);
            col1 = SoaVector4<SoaNFloat>(mat.getCol1(), 0.0f);
            col2 = SoaVector4<SoaNFloat>(mat.getCol2(), 0.0f);
            col3 = SoaVector4<SoaNFloat>(translateVec, 1.0f);
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix4<SoaNFloat>::SoaMatrix4(const SoaQuat<SoaNFloat> & unitQuat, const SoaVector3<SoaNFloat> & translateVec)
        {
            SoaMatrix3<SoaNFloat> mat;
            mat = SoaMatrix3<SoaNFloat>(unitQuat);
            col0 = SoaVector4<SoaNFloat>(mat.getCol0(), 0.0f);
            col1 = SoaVector4<SoaNFloat>(mat.getCol1(), 0.0f);
            col2 = SoaVector4<SoaNFloat>(mat.getCol2(), 0.0f);
            col3 = SoaVector4<SoaNFloat>(translateVec, 1.0f);
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix4<SoaNFloat> & SoaMatrix4<SoaNFloat>::setCol0(const SoaVector4<SoaNFloat> & _col0)
        {
            col0 = _col0;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix4<SoaNFloat> & SoaMatrix4<SoaNFloat>::setCol1(const SoaVector4<SoaNFloat> & _col1)
        {
            col1 = _col1;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix4<SoaNFloat> & SoaMatrix4<SoaNFloat>::setCol2(const SoaVector4<SoaNFloat> & _col2)
        {
            col2 = _col2;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix4<SoaNFloat> & SoaMatrix4<SoaNFloat>::setCol3(const SoaVector4<SoaNFloat> & _col3)
        {
            col3 = _col3;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix4<SoaNFloat> & SoaMatrix4<SoaNFloat>::setCol(int col, const SoaVector4<SoaNFloat> & vec)
        {
            *(&col0 + col) = vec;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix4<SoaNFloat> & SoaMatrix4<SoaNFloat>::setRow(int row, const SoaVector4<SoaNFloat> & vec)
        {
            col0.setElem(row, vec.getElem(0));
            col1.setElem(row, vec.getElem(1));
            col2.setElem(row, vec.getElem(2));
            col3.setElem(row, vec.getElem(3));
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix4<SoaNFloat> & SoaMatrix4<SoaNFloat>::setElem(int col, int row, SoaNFloat val)
        {
            SoaVector4<SoaNFloat> tmpV3_0;
            tmpV3_0 = this->getCol(col);
            tmpV3_0.setElem(row, val);
            this->setCol(col, tmpV3_0);
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaMatrix4<SoaNFloat>::getElem(int col, int row) const
        {
            return this->getCol(col).getElem(row);
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> SoaMatrix4<SoaNFloat>::getCol0() const
        {
            return col0;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> SoaMatrix4<SoaNFloat>::getCol1() const
        {
            return col1;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> SoaMatrix4<SoaNFloat>::getCol2() const
        {
            return col2;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> SoaMatrix4<SoaNFloat>::getCol3() const
        {
            return col3;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> SoaMatrix4<SoaNFloat>::getCol(int col) const
        {
            return *(&col0 + col);
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> SoaMatrix4<SoaNFloat>::getRow(int row) const
        {
            return SoaVector4<SoaNFloat>(col0.getElem(row), col1.getElem(row), col2.getElem(row), col3.getElem(row));
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix4<SoaNFloat> & SoaMatrix4<SoaNFloat>::operator =(const SoaMatrix4<SoaNFloat> & mat)
        {
            col0 = mat.col0;
            col1 = mat.col1;
            col2 = mat.col2;
            col3 = mat.col3;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> transpose(const SoaMatrix4<SoaNFloat> & mat)
        {
            return SoaMatrix4<SoaNFloat>(
                SoaVector4<SoaNFloat>(mat.getCol0().getX(), mat.getCol1().getX(), mat.getCol2().getX(), mat.getCol3().getX()),
                SoaVector4<SoaNFloat>(mat.getCol0().getY(), mat.getCol1().getY(), mat.getCol2().getY(), mat.getCol3().getY()),
                SoaVector4<SoaNFloat>(mat.getCol0().getZ(), mat.getCol1().getZ(), mat.getCol2().getZ(), mat.getCol3().getZ()),
                SoaVector4<SoaNFloat>(mat.getCol0().getW(), mat.getCol1().getW(), mat.getCol2().getW(), mat.getCol3().getW())
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> inverse(const SoaMatrix4<SoaNFloat> & mat)
        {
            SoaVector4<SoaNFloat> res0, res1, res2, res3;
            SoaNFloat mA, mB, mC, mD, mE, mF, mG, mH, mI, mJ, mK, mL, mM, mN, mO, mP, tmp0, tmp1, tmp2, tmp3, tmp4, tmp5, detInv;
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
            return SoaMatrix4<SoaNFloat>(
                (res0 * detInv),
                (res1 * detInv),
                (res2 * detInv),
                (res3 * detInv)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> affineInverse(const SoaMatrix4<SoaNFloat> & mat)
        {
            SoaTransform3<SoaNFloat> affineMat;
            affineMat.setCol0(mat.getCol0().getXYZ());
            affineMat.setCol1(mat.getCol1().getXYZ());
            affineMat.setCol2(mat.getCol2().getXYZ());
            affineMat.setCol3(mat.getCol3().getXYZ());
            return SoaMatrix4<SoaNFloat>(inverse(affineMat));
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> orthoInverse(const SoaMatrix4<SoaNFloat> & mat)
        {
            SoaTransform3<SoaNFloat> affineMat;
            affineMat.setCol0(mat.getCol0().getXYZ());
            affineMat.setCol1(mat.getCol1().getXYZ());
            affineMat.setCol2(mat.getCol2().getXYZ());
            affineMat.setCol3(mat.getCol3().getXYZ());
            return SoaMatrix4<SoaNFloat>(orthoInverse(affineMat));
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat determinant(const SoaMatrix4<SoaNFloat> & mat)
        {
            SoaNFloat dx, dy, dz, dw, mA, mB, mC, mD, mE, mF, mG, mH, mI, mJ, mK, mL, mM, mN, mO, mP, tmp0, tmp1, tmp2, tmp3, tmp4, tmp5;
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

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> SoaMatrix4<SoaNFloat>::operator +(const SoaMatrix4<SoaNFloat> & mat) const
        {
            return SoaMatrix4<SoaNFloat>(
                (col0 + mat.col0),
                (col1 + mat.col1),
                (col2 + mat.col2),
                (col3 + mat.col3)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> SoaMatrix4<SoaNFloat>::operator -(const SoaMatrix4<SoaNFloat> & mat) const
        {
            return SoaMatrix4<SoaNFloat>(
                (col0 - mat.col0),
                (col1 - mat.col1),
                (col2 - mat.col2),
                (col3 - mat.col3)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix4<SoaNFloat> & SoaMatrix4<SoaNFloat>::operator +=(const SoaMatrix4<SoaNFloat> & mat)
        {
            *this = *this + mat;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix4<SoaNFloat> & SoaMatrix4<SoaNFloat>::operator -=(const SoaMatrix4<SoaNFloat> & mat)
        {
            *this = *this - mat;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> SoaMatrix4<SoaNFloat>::operator -() const
        {
            return SoaMatrix4<SoaNFloat>(
                (-col0),
                (-col1),
                (-col2),
                (-col3)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> abs(const SoaMatrix4<SoaNFloat> & mat)
        {
            return SoaMatrix4<SoaNFloat>(
                abs(mat.getCol0()),
                abs(mat.getCol1()),
                abs(mat.getCol2()),
                abs(mat.getCol3())
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> SoaMatrix4<SoaNFloat>::operator *(SoaNFloat scalar) const
        {
            return SoaMatrix4<SoaNFloat>(
                (col0 * scalar),
                (col1 * scalar),
                (col2 * scalar),
                (col3 * scalar)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix4<SoaNFloat> & SoaMatrix4<SoaNFloat>::operator *=(SoaNFloat scalar)
        {
            *this = *this * scalar;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> operator *(SoaNFloat scalar, const SoaMatrix4<SoaNFloat> & mat)
        {
            return mat * scalar;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> mul(const SoaMatrix4<SoaNFloat>& mat, const SoaVector4<SoaNFloat> & vec)
        {
            return SoaVector4<SoaNFloat>(
                ((((mat.col0.getX() * vec.getX()) + (mat.col1.getX() * vec.getY())) + (mat.col2.getX() * vec.getZ())) + (mat.col3.getX() * vec.getW())),
                ((((mat.col0.getY() * vec.getX()) + (mat.col1.getY() * vec.getY())) + (mat.col2.getY() * vec.getZ())) + (mat.col3.getY() * vec.getW())),
                ((((mat.col0.getZ() * vec.getX()) + (mat.col1.getZ() * vec.getY())) + (mat.col2.getZ() * vec.getZ())) + (mat.col3.getZ() * vec.getW())),
                ((((mat.col0.getW() * vec.getX()) + (mat.col1.getW() * vec.getY())) + (mat.col2.getW() * vec.getZ())) + (mat.col3.getW() * vec.getW()))
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> mul(const SoaMatrix4<SoaNFloat>& mat, const SoaVector3<SoaNFloat> & vec)
        {
            return SoaVector4<SoaNFloat>(
                (((mat.col0.getX() * vec.getX()) + (mat.col1.getX() * vec.getY())) + (mat.col2.getX() * vec.getZ())),
                (((mat.col0.getY() * vec.getX()) + (mat.col1.getY() * vec.getY())) + (mat.col2.getY() * vec.getZ())),
                (((mat.col0.getZ() * vec.getX()) + (mat.col1.getZ() * vec.getY())) + (mat.col2.getZ() * vec.getZ())),
                (((mat.col0.getW() * vec.getX()) + (mat.col1.getW() * vec.getY())) + (mat.col2.getW() * vec.getZ()))
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> mul(const SoaMatrix4<SoaNFloat>& mat, const SoaPoint3<SoaNFloat> & pnt)
        {
            return SoaVector4<SoaNFloat>(
                ((((mat.col0.getX() * pnt.getX()) + (mat.col1.getX() * pnt.getY())) + (mat.col2.getX() * pnt.getZ())) + mat.col3.getX()),
                ((((mat.col0.getY() * pnt.getX()) + (mat.col1.getY() * pnt.getY())) + (mat.col2.getY() * pnt.getZ())) + mat.col3.getY()),
                ((((mat.col0.getZ() * pnt.getX()) + (mat.col1.getZ() * pnt.getY())) + (mat.col2.getZ() * pnt.getZ())) + mat.col3.getZ()),
                ((((mat.col0.getW() * pnt.getX()) + (mat.col1.getW() * pnt.getY())) + (mat.col2.getW() * pnt.getZ())) + mat.col3.getW())
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> mul(const SoaMatrix4<SoaNFloat>& mat, const SoaTransform3<SoaNFloat> & tfrm)
        {
            return SoaMatrix4<SoaNFloat>(
                mul(mat, tfrm.getCol0()),
                mul(mat, tfrm.getCol1()),
                mul(mat, tfrm.getCol2()),
                mul(mat, SoaPoint3<SoaNFloat>(tfrm.getCol3()))
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> mul(const SoaMatrix4<SoaNFloat>& mat0, const SoaMatrix4<SoaNFloat> & mat1)
        {
            return SoaMatrix4<SoaNFloat>(
                mul(mat0, mat1.col0),
                mul(mat0, mat1.col1),
                mul(mat0, mat1.col2),
                mul(mat0, mat1.col3)
                );
        }


        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> SoaMatrix4<SoaNFloat>::identity()
        {
            return SoaMatrix4<SoaNFloat>(
                SoaVector4<SoaNFloat>::xAxis(),
                SoaVector4<SoaNFloat>::yAxis(),
                SoaVector4<SoaNFloat>::zAxis(),
                SoaVector4<SoaNFloat>::wAxis()
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix4<SoaNFloat> & SoaMatrix4<SoaNFloat>::setUpper3x3(const SoaMatrix3<SoaNFloat> & mat3)
        {
            col0.setXYZ(mat3.getCol0());
            col1.setXYZ(mat3.getCol1());
            col2.setXYZ(mat3.getCol2());
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> SoaMatrix4<SoaNFloat>::getUpper3x3() const
        {
            return SoaMatrix3<SoaNFloat>(
                col0.getXYZ(),
                col1.getXYZ(),
                col2.getXYZ()
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaMatrix4<SoaNFloat> & SoaMatrix4<SoaNFloat>::setTranslation(const SoaVector3<SoaNFloat> & translateVec)
        {
            col3.setXYZ(translateVec);
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaMatrix4<SoaNFloat>::getTranslation() const
        {
            return col3.getXYZ();
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> SoaMatrix4<SoaNFloat>::rotationX(SoaNFloat radians)
        {
            SoaNFloat s, c;
            sincosf(radians, &s, &c);
            return SoaMatrix4<SoaNFloat>(
                SoaVector4<SoaNFloat>::xAxis(),
                SoaVector4<SoaNFloat>(0.0f, c, s, 0.0f),
                SoaVector4<SoaNFloat>(0.0f, -s, c, 0.0f),
                SoaVector4<SoaNFloat>::wAxis()
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> SoaMatrix4<SoaNFloat>::rotationY(SoaNFloat radians)
        {
            SoaNFloat s, c;
            sincosf(radians, &s, &c);
            return SoaMatrix4<SoaNFloat>(
                SoaVector4<SoaNFloat>(c, 0.0f, -s, 0.0f),
                SoaVector4<SoaNFloat>::yAxis(),
                SoaVector4<SoaNFloat>(s, 0.0f, c, 0.0f),
                SoaVector4<SoaNFloat>::wAxis()
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> SoaMatrix4<SoaNFloat>::rotationZ(SoaNFloat radians)
        {
            SoaNFloat s, c;
            sincosf(radians, &s, &c);
            return SoaMatrix4<SoaNFloat>(
                SoaVector4<SoaNFloat>(c, s, 0.0f, 0.0f),
                SoaVector4<SoaNFloat>(-s, c, 0.0f, 0.0f),
                SoaVector4<SoaNFloat>::zAxis(),
                SoaVector4<SoaNFloat>::wAxis()
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> SoaMatrix4<SoaNFloat>::rotationZYX(const SoaVector3<SoaNFloat> & radiansXYZ)
        {
            SoaNFloat sX, cX, sY, cY, sZ, cZ, tmp0, tmp1;
            sincosf(radiansXYZ.getX(), &sX, &cX);
            sincosf(radiansXYZ.getY(), &sY, &cY);
            sincosf(radiansXYZ.getZ(), &sZ, &cZ);
            tmp0 = (cZ * sY);
            tmp1 = (sZ * sY);
            return SoaMatrix4<SoaNFloat>(
                SoaVector4<SoaNFloat>((cZ * cY), (sZ * cY), -sY, 0.0f),
                SoaVector4<SoaNFloat>(((tmp0 * sX) - (sZ * cX)), ((tmp1 * sX) + (cZ * cX)), (cY * sX), 0.0f),
                SoaVector4<SoaNFloat>(((tmp0 * cX) + (sZ * sX)), ((tmp1 * cX) - (cZ * sX)), (cY * cX), 0.0f),
                SoaVector4<SoaNFloat>::wAxis()
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> SoaMatrix4<SoaNFloat>::rotation(SoaNFloat radians, const SoaVector3<SoaNFloat> & unitVec)
        {
            SoaNFloat x, y, z, s, c, oneMinusC, xy, yz, zx;
            s = sinf(radians);
            c = cosf(radians);
            x = unitVec.getX();
            y = unitVec.getY();
            z = unitVec.getZ();
            xy = (x * y);
            yz = (y * z);
            zx = (z * x);
            oneMinusC = (1.0f - c);
            return SoaMatrix4<SoaNFloat>(
                SoaVector4<SoaNFloat>((((x * x) * oneMinusC) + c), ((xy * oneMinusC) + (z * s)), ((zx * oneMinusC) - (y * s)), 0.0f),
                SoaVector4<SoaNFloat>(((xy * oneMinusC) - (z * s)), (((y * y) * oneMinusC) + c), ((yz * oneMinusC) + (x * s)), 0.0f),
                SoaVector4<SoaNFloat>(((zx * oneMinusC) + (y * s)), ((yz * oneMinusC) - (x * s)), (((z * z) * oneMinusC) + c), 0.0f),
                SoaVector4<SoaNFloat>::wAxis()
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> SoaMatrix4<SoaNFloat>::rotation(const SoaQuat<SoaNFloat> & unitQuat)
        {
            return SoaMatrix4<SoaNFloat>(SoaTransform3<SoaNFloat>::rotation(unitQuat));
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> SoaMatrix4<SoaNFloat>::scale(const SoaVector3<SoaNFloat> & scaleVec)
        {
            return SoaMatrix4<SoaNFloat>(
                SoaVector4<SoaNFloat>(scaleVec.getX(), 0.0f, 0.0f, 0.0f),
                SoaVector4<SoaNFloat>(0.0f, scaleVec.getY(), 0.0f, 0.0f),
                SoaVector4<SoaNFloat>(0.0f, 0.0f, scaleVec.getZ(), 0.0f),
                SoaVector4<SoaNFloat>::wAxis()
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> appendScale(const SoaMatrix4<SoaNFloat> & mat, const SoaVector3<SoaNFloat> & scaleVec)
        {
            return SoaMatrix4<SoaNFloat>(
                (mat.getCol0() * scaleVec.getX()),
                (mat.getCol1() * scaleVec.getY()),
                (mat.getCol2() * scaleVec.getZ()),
                mat.getCol3()
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> prependScale(const SoaVector3<SoaNFloat> & scaleVec, const SoaMatrix4<SoaNFloat> & mat)
        {
            SoaVector4<SoaNFloat> scale4;
            scale4 = SoaVector4<SoaNFloat>(scaleVec, 1.0f);
            return SoaMatrix4<SoaNFloat>(
                mat.getCol0() * scale4,
                mat.getCol1() * scale4,
                mat.getCol2() * scale4,
                mat.getCol3() * scale4
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> SoaMatrix4<SoaNFloat>::translation(const SoaVector3<SoaNFloat> & translateVec)
        {
            return SoaMatrix4<SoaNFloat>(
                SoaVector4<SoaNFloat>::xAxis(),
                SoaVector4<SoaNFloat>::yAxis(),
                SoaVector4<SoaNFloat>::zAxis(),
                SoaVector4<SoaNFloat>(translateVec, 1.0f)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> SoaMatrix4<SoaNFloat>::lookAt(const SoaPoint3<SoaNFloat> & eyePos, const SoaPoint3<SoaNFloat> & lookAtPos, const SoaVector3<SoaNFloat> & upVec)
        {
            SoaMatrix4<SoaNFloat> m4EyeFrame;
            SoaVector3<SoaNFloat> v3X, v3Y, v3Z;
            v3Y = normalize(upVec);
            v3Z = normalize((eyePos - lookAtPos));
            v3X = normalize(cross(v3Y, v3Z));
            v3Y = cross(v3Z, v3X);
            m4EyeFrame = SoaMatrix4<SoaNFloat>(SoaVector4<SoaNFloat>(v3X), SoaVector4<SoaNFloat>(v3Y), SoaVector4<SoaNFloat>(v3Z), SoaVector4<SoaNFloat>(eyePos));
            return orthoInverse(m4EyeFrame);
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> SoaMatrix4<SoaNFloat>::perspective(SoaNFloat fovyRadians, SoaNFloat aspect, SoaNFloat zNear, SoaNFloat zFar)
        {
            SoaNFloat f, rangeInv;
            f = tanf(((SoaNFloat)(_VECTORMATH_PI_OVER_2)-(0.5f * fovyRadians)));
            rangeInv = (1.0f / (zNear - zFar));
            return SoaMatrix4<SoaNFloat>(
                SoaVector4<SoaNFloat>((f / aspect), 0.0f, 0.0f, 0.0f),
                SoaVector4<SoaNFloat>(0.0f, f, 0.0f, 0.0f),
                SoaVector4<SoaNFloat>(0.0f, 0.0f, ((zNear + zFar) * rangeInv), -1.0f),
                SoaVector4<SoaNFloat>(0.0f, 0.0f, (((zNear * zFar) * rangeInv) * 2.0f), 0.0f)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> SoaMatrix4<SoaNFloat>::frustum(SoaNFloat left, SoaNFloat right, SoaNFloat bottom, SoaNFloat top, SoaNFloat zNear, SoaNFloat zFar)
        {
            SoaNFloat sum_rl, sum_tb, sum_nf, inv_rl, inv_tb, inv_nf, n2;
            sum_rl = (right + left);
            sum_tb = (top + bottom);
            sum_nf = (zNear + zFar);
            inv_rl = (1.0f / (right - left));
            inv_tb = (1.0f / (top - bottom));
            inv_nf = (1.0f / (zNear - zFar));
            n2 = (zNear + zNear);
            return SoaMatrix4<SoaNFloat>(
                SoaVector4<SoaNFloat>((n2 * inv_rl), 0.0f, 0.0f, 0.0f),
                SoaVector4<SoaNFloat>(0.0f, (n2 * inv_tb), 0.0f, 0.0f),
                SoaVector4<SoaNFloat>((sum_rl * inv_rl), (sum_tb * inv_tb), (sum_nf * inv_nf), -1.0f),
                SoaVector4<SoaNFloat>(0.0f, 0.0f, ((n2 * inv_nf) * zFar), 0.0f)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> SoaMatrix4<SoaNFloat>::orthographic(SoaNFloat left, SoaNFloat right, SoaNFloat bottom, SoaNFloat top, SoaNFloat zNear, SoaNFloat zFar)
        {
            SoaNFloat sum_rl, sum_tb, sum_nf, inv_rl, inv_tb, inv_nf;
            sum_rl = (right + left);
            sum_tb = (top + bottom);
            sum_nf = (zNear + zFar);
            inv_rl = (1.0f / (right - left));
            inv_tb = (1.0f / (top - bottom));
            inv_nf = (1.0f / (zNear - zFar));
            return SoaMatrix4<SoaNFloat>(
                SoaVector4<SoaNFloat>((inv_rl + inv_rl), 0.0f, 0.0f, 0.0f),
                SoaVector4<SoaNFloat>(0.0f, (inv_tb + inv_tb), 0.0f, 0.0f),
                SoaVector4<SoaNFloat>(0.0f, 0.0f, (inv_nf + inv_nf), 0.0f),
                SoaVector4<SoaNFloat>((-sum_rl * inv_rl), (-sum_tb * inv_tb), (sum_nf * inv_nf), 1.0f)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> select(const SoaMatrix4<SoaNFloat> & mat0, const SoaMatrix4<SoaNFloat> & mat1, typename SoaNFloat::SoaBool select1)
        {
            return SoaMatrix4<SoaNFloat>(
                select(mat0.getCol0(), mat1.getCol0(), select1),
                select(mat0.getCol1(), mat1.getCol1(), select1),
                select(mat0.getCol2(), mat1.getCol2(), select1),
                select(mat0.getCol3(), mat1.getCol3(), select1)
                );
        }

#ifdef _VECTORMATH_DEBUG

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print(const SoaMatrix4<SoaNFloat> & mat)
        {
            print(mat.getRow(0));
            print(mat.getRow(1));
            print(mat.getRow(2));
            print(mat.getRow(3));
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print(const SoaMatrix4<SoaNFloat> & mat, const char * name)
        {
            printf("%s:\n", name);
            print(mat);
        }

#endif

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaTransform3<SoaNFloat>::SoaTransform3(const SoaTransform3<SoaNFloat> & tfrm)
        {
            col0 = tfrm.col0;
            col1 = tfrm.col1;
            col2 = tfrm.col2;
            col3 = tfrm.col3;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaTransform3<SoaNFloat>::SoaTransform3(SoaNFloat scalar)
        {
            col0 = SoaVector3<SoaNFloat>(scalar);
            col1 = SoaVector3<SoaNFloat>(scalar);
            col2 = SoaVector3<SoaNFloat>(scalar);
            col3 = SoaVector3<SoaNFloat>(scalar);
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaTransform3<SoaNFloat>::SoaTransform3(const SoaVector3<SoaNFloat> & _col0, const SoaVector3<SoaNFloat> & _col1, const SoaVector3<SoaNFloat> & _col2, const SoaVector3<SoaNFloat> & _col3)
        {
            col0 = _col0;
            col1 = _col1;
            col2 = _col2;
            col3 = _col3;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaTransform3<SoaNFloat>::SoaTransform3(const SoaMatrix3<SoaNFloat> & tfrm, const SoaVector3<SoaNFloat> & translateVec)
        {
            this->setUpper3x3(tfrm);
            this->setTranslation(translateVec);
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaTransform3<SoaNFloat>::SoaTransform3(const SoaQuat<SoaNFloat> & unitQuat, const SoaVector3<SoaNFloat> & translateVec)
        {
            this->setUpper3x3(SoaMatrix3<SoaNFloat>(unitQuat));
            this->setTranslation(translateVec);
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaTransform3<SoaNFloat> & SoaTransform3<SoaNFloat>::setCol0(const SoaVector3<SoaNFloat> & _col0)
        {
            col0 = _col0;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaTransform3<SoaNFloat> & SoaTransform3<SoaNFloat>::setCol1(const SoaVector3<SoaNFloat> & _col1)
        {
            col1 = _col1;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaTransform3<SoaNFloat> & SoaTransform3<SoaNFloat>::setCol2(const SoaVector3<SoaNFloat> & _col2)
        {
            col2 = _col2;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaTransform3<SoaNFloat> & SoaTransform3<SoaNFloat>::setCol3(const SoaVector3<SoaNFloat> & _col3)
        {
            col3 = _col3;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaTransform3<SoaNFloat> & SoaTransform3<SoaNFloat>::setCol(int col, const SoaVector3<SoaNFloat> & vec)
        {
            *(&col0 + col) = vec;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaTransform3<SoaNFloat> & SoaTransform3<SoaNFloat>::setRow(int row, const SoaVector4<SoaNFloat> & vec)
        {
            col0.setElem(row, vec.getElem(0));
            col1.setElem(row, vec.getElem(1));
            col2.setElem(row, vec.getElem(2));
            col3.setElem(row, vec.getElem(3));
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaTransform3<SoaNFloat> & SoaTransform3<SoaNFloat>::setElem(int col, int row, SoaNFloat val)
        {
            SoaVector3<SoaNFloat> tmpV3_0;
            tmpV3_0 = this->getCol(col);
            tmpV3_0.setElem(row, val);
            this->setCol(col, tmpV3_0);
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaTransform3<SoaNFloat>::getElem(int col, int row) const
        {
            return this->getCol(col).getElem(row);
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaTransform3<SoaNFloat>::getCol0() const
        {
            return col0;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaTransform3<SoaNFloat>::getCol1() const
        {
            return col1;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaTransform3<SoaNFloat>::getCol2() const
        {
            return col2;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaTransform3<SoaNFloat>::getCol3() const
        {
            return col3;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaTransform3<SoaNFloat>::getCol(int col) const
        {
            return *(&col0 + col);
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> SoaTransform3<SoaNFloat>::getRow(int row) const
        {
            return SoaVector4<SoaNFloat>(col0.getElem(row), col1.getElem(row), col2.getElem(row), col3.getElem(row));
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaTransform3<SoaNFloat> & SoaTransform3<SoaNFloat>::operator =(const SoaTransform3<SoaNFloat> & tfrm)
        {
            col0 = tfrm.col0;
            col1 = tfrm.col1;
            col2 = tfrm.col2;
            col3 = tfrm.col3;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> inverse(const SoaTransform3<SoaNFloat> & tfrm)
        {
            SoaVector3<SoaNFloat> tmp0, tmp1, tmp2, inv0, inv1, inv2;
            SoaNFloat detinv;
            tmp0 = cross(tfrm.getCol1(), tfrm.getCol2());
            tmp1 = cross(tfrm.getCol2(), tfrm.getCol0());
            tmp2 = cross(tfrm.getCol0(), tfrm.getCol1());
            detinv = (1.0f / dot(tfrm.getCol2(), tmp2));
            inv0 = SoaVector3<SoaNFloat>((tmp0.getX() * detinv), (tmp1.getX() * detinv), (tmp2.getX() * detinv));
            inv1 = SoaVector3<SoaNFloat>((tmp0.getY() * detinv), (tmp1.getY() * detinv), (tmp2.getY() * detinv));
            inv2 = SoaVector3<SoaNFloat>((tmp0.getZ() * detinv), (tmp1.getZ() * detinv), (tmp2.getZ() * detinv));
            return SoaTransform3<SoaNFloat>(
                inv0,
                inv1,
                inv2,
                SoaVector3<SoaNFloat>((-((inv0 * tfrm.getCol3().getX()) + ((inv1 * tfrm.getCol3().getY()) + (inv2 * tfrm.getCol3().getZ())))))
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> orthoInverse(const SoaTransform3<SoaNFloat> & tfrm)
        {
            SoaVector3<SoaNFloat> inv0, inv1, inv2;
            inv0 = SoaVector3<SoaNFloat>(tfrm.getCol0().getX(), tfrm.getCol1().getX(), tfrm.getCol2().getX());
            inv1 = SoaVector3<SoaNFloat>(tfrm.getCol0().getY(), tfrm.getCol1().getY(), tfrm.getCol2().getY());
            inv2 = SoaVector3<SoaNFloat>(tfrm.getCol0().getZ(), tfrm.getCol1().getZ(), tfrm.getCol2().getZ());
            return SoaTransform3<SoaNFloat>(
                inv0,
                inv1,
                inv2,
                SoaVector3<SoaNFloat>((-((inv0 * tfrm.getCol3().getX()) + ((inv1 * tfrm.getCol3().getY()) + (inv2 * tfrm.getCol3().getZ())))))
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> abs(const SoaTransform3<SoaNFloat> & tfrm)
        {
            return SoaTransform3<SoaNFloat>(
                abs(tfrm.getCol0()),
                abs(tfrm.getCol1()),
                abs(tfrm.getCol2()),
                abs(tfrm.getCol3())
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> mul(const SoaTransform3<SoaNFloat>& tfrm, const SoaVector3<SoaNFloat> & vec)
        {
            return SoaVector3<SoaNFloat>(
                (((tfrm.col0.getX() * vec.getX()) + (tfrm.col1.getX() * vec.getY())) + (tfrm.col2.getX() * vec.getZ())),
                (((tfrm.col0.getY() * vec.getX()) + (tfrm.col1.getY() * vec.getY())) + (tfrm.col2.getY() * vec.getZ())),
                (((tfrm.col0.getZ() * vec.getX()) + (tfrm.col1.getZ() * vec.getY())) + (tfrm.col2.getZ() * vec.getZ()))
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> mul(const SoaTransform3<SoaNFloat>& tfrm, const SoaPoint3<SoaNFloat> & pnt)
        {
            return SoaPoint3<SoaNFloat>(
                ((((tfrm.col0.getX() * pnt.getX()) + (tfrm.col1.getX() * pnt.getY())) + (tfrm.col2.getX() * pnt.getZ())) + tfrm.col3.getX()),
                ((((tfrm.col0.getY() * pnt.getX()) + (tfrm.col1.getY() * pnt.getY())) + (tfrm.col2.getY() * pnt.getZ())) + tfrm.col3.getY()),
                ((((tfrm.col0.getZ() * pnt.getX()) + (tfrm.col1.getZ() * pnt.getY())) + (tfrm.col2.getZ() * pnt.getZ())) + tfrm.col3.getZ())
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> mul(const SoaTransform3<SoaNFloat> & tfrm0, const SoaTransform3<SoaNFloat> & tfrm1)
        {
            return SoaTransform3<SoaNFloat>(
                mul(tfrm0, tfrm1.col0),
                mul(tfrm0, tfrm1.col1),
                mul(tfrm0, tfrm1.col2),
                SoaVector3<SoaNFloat>(mul(tfrm0, SoaPoint3<SoaNFloat>(tfrm1.col3)))
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> mulPerElem(const SoaTransform3<SoaNFloat> & tfrm0, const SoaTransform3<SoaNFloat> & tfrm1)
        {
            return SoaTransform3<SoaNFloat>(
                tfrm0.getCol0() * tfrm1.getCol0(),
                tfrm0.getCol1() * tfrm1.getCol1(),
                tfrm0.getCol2() * tfrm1.getCol2(),
                tfrm0.getCol3() * tfrm1.getCol3()
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> SoaTransform3<SoaNFloat>::identity()
        {
            return SoaTransform3<SoaNFloat>(
                SoaVector3<SoaNFloat>::xAxis(),
                SoaVector3<SoaNFloat>::yAxis(),
                SoaVector3<SoaNFloat>::zAxis(),
                SoaVector3<SoaNFloat>(0.0f)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaTransform3<SoaNFloat> & SoaTransform3<SoaNFloat>::setUpper3x3(const SoaMatrix3<SoaNFloat> & tfrm)
        {
            col0 = tfrm.getCol0();
            col1 = tfrm.getCol1();
            col2 = tfrm.getCol2();
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> SoaTransform3<SoaNFloat>::getUpper3x3() const
        {
            return SoaMatrix3<SoaNFloat>(col0, col1, col2);
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaTransform3<SoaNFloat> & SoaTransform3<SoaNFloat>::setTranslation(const SoaVector3<SoaNFloat> & translateVec)
        {
            col3 = translateVec;
            return *this;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaTransform3<SoaNFloat>::getTranslation() const
        {
            return col3;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> SoaTransform3<SoaNFloat>::rotationX(SoaNFloat radians)
        {
            SoaNFloat s, c;
            sincosf(radians, &s, &c);
            return SoaTransform3<SoaNFloat>(
                SoaVector3<SoaNFloat>::xAxis(),
                SoaVector3<SoaNFloat>(0.0f, c, s),
                SoaVector3<SoaNFloat>(0.0f, -s, c),
                SoaVector3<SoaNFloat>(0.0f)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> SoaTransform3<SoaNFloat>::rotationY(SoaNFloat radians)
        {
            SoaNFloat s, c;
            sincosf(radians, &s, &c);
            return SoaTransform3<SoaNFloat>(
                SoaVector3<SoaNFloat>(c, 0.0f, -s),
                SoaVector3<SoaNFloat>::yAxis(),
                SoaVector3<SoaNFloat>(s, 0.0f, c),
                SoaVector3<SoaNFloat>(0.0f)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> SoaTransform3<SoaNFloat>::rotationZ(SoaNFloat radians)
        {
            SoaNFloat s, c;
            sincosf(radians, &s, &c);
            return SoaTransform3<SoaNFloat>(
                SoaVector3<SoaNFloat>(c, s, 0.0f),
                SoaVector3<SoaNFloat>(-s, c, 0.0f),
                SoaVector3<SoaNFloat>::zAxis(),
                SoaVector3<SoaNFloat>(0.0f)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> SoaTransform3<SoaNFloat>::rotationZYX(const SoaVector3<SoaNFloat> & radiansXYZ)
        {
            SoaNFloat sX, cX, sY, cY, sZ, cZ, tmp0, tmp1;
            sincosf(radiansXYZ.getX(), &sX, &cX);
            sincosf(radiansXYZ.getY(), &sY, &cY);
            sincosf(radiansXYZ.getZ(), &sZ, &cZ);
            tmp0 = (cZ * sY);
            tmp1 = (sZ * sY);
            return SoaTransform3<SoaNFloat>(
                SoaVector3<SoaNFloat>((cZ * cY), (sZ * cY), -sY),
                SoaVector3<SoaNFloat>(((tmp0 * sX) - (sZ * cX)), ((tmp1 * sX) + (cZ * cX)), (cY * sX)),
                SoaVector3<SoaNFloat>(((tmp0 * cX) + (sZ * sX)), ((tmp1 * cX) - (cZ * sX)), (cY * cX)),
                SoaVector3<SoaNFloat>(0.0f)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> SoaTransform3<SoaNFloat>::rotation(SoaNFloat radians, const SoaVector3<SoaNFloat> & unitVec)
        {
            return SoaTransform3<SoaNFloat>(SoaMatrix3<SoaNFloat>::rotation(radians, unitVec), SoaVector3<SoaNFloat>(0.0f));
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> SoaTransform3<SoaNFloat>::rotation(const SoaQuat<SoaNFloat> & unitQuat)
        {
            return SoaTransform3<SoaNFloat>(SoaMatrix3<SoaNFloat>(unitQuat), SoaVector3<SoaNFloat>(0.0f));
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> SoaTransform3<SoaNFloat>::scale(const SoaVector3<SoaNFloat> & scaleVec)
        {
            return SoaTransform3<SoaNFloat>(
                SoaVector3<SoaNFloat>(scaleVec.getX(), 0.0f, 0.0f),
                SoaVector3<SoaNFloat>(0.0f, scaleVec.getY(), 0.0f),
                SoaVector3<SoaNFloat>(0.0f, 0.0f, scaleVec.getZ()),
                SoaVector3<SoaNFloat>(0.0f)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> appendScale(const SoaTransform3<SoaNFloat> & tfrm, const SoaVector3<SoaNFloat> & scaleVec)
        {
            return SoaTransform3<SoaNFloat>(
                (tfrm.getCol0() * scaleVec.getX()),
                (tfrm.getCol1() * scaleVec.getY()),
                (tfrm.getCol2() * scaleVec.getZ()),
                tfrm.getCol3()
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> prependScale(const SoaVector3<SoaNFloat> & scaleVec, const SoaTransform3<SoaNFloat> & tfrm)
        {
            return SoaTransform3<SoaNFloat>(
                tfrm.getCol0() * scaleVec,
                tfrm.getCol1() * scaleVec,
                tfrm.getCol2() * scaleVec,
                tfrm.getCol3() * scaleVec
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> SoaTransform3<SoaNFloat>::translation(const SoaVector3<SoaNFloat> & translateVec)
        {
            return SoaTransform3<SoaNFloat>(
                SoaVector3<SoaNFloat>::xAxis(),
                SoaVector3<SoaNFloat>::yAxis(),
                SoaVector3<SoaNFloat>::zAxis(),
                translateVec
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> select(const SoaTransform3<SoaNFloat> & tfrm0, const SoaTransform3<SoaNFloat> & tfrm1, typename SoaNFloat::SoaBool select1)
        {
            return SoaTransform3<SoaNFloat>(
                select(tfrm0.getCol0(), tfrm1.getCol0(), select1),
                select(tfrm0.getCol1(), tfrm1.getCol1(), select1),
                select(tfrm0.getCol2(), tfrm1.getCol2(), select1),
                select(tfrm0.getCol3(), tfrm1.getCol3(), select1)
                );
        }

#ifdef _VECTORMATH_DEBUG

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print(const SoaTransform3<SoaNFloat> & tfrm)
        {
            print(tfrm.getRow(0));
            print(tfrm.getRow(1));
            print(tfrm.getRow(2));
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print(const SoaTransform3<SoaNFloat> & tfrm, const char * name)
        {
            printf("%s:\n", name);
            print(tfrm);
        }

#endif

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaQuat<SoaNFloat>::SoaQuat(const SoaMatrix3<SoaNFloat> & tfrm)
        {
            SoaNFloat trace, radicand, scale, xx, yx, zx, xy, yy, zy, xz, yz, zz, tmpx, tmpy, tmpz, tmpw, qx, qy, qz, qw;
            SoaNFloat::SoaBool negTrace, ZgtX, ZgtY, YgtX;
            SoaNFloat::SoaBool largestXorY, largestYorZ, largestZorX;

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
            largestXorY = (!ZgtX | !ZgtY) & negTrace;
            largestYorZ = (YgtX | ZgtX) & negTrace;
            largestZorX = (ZgtY | !YgtX) & negTrace;

#if 0
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
#else
            zz = select(zz, -zz, largestXorY);
            xy = select(xy, -xy, largestXorY);

            xx = select(xx, -xx, largestYorZ);
            yz = select(yz, -yz, largestYorZ);

            yy = select(yy, -yy, largestZorX);
            zx = select(zx, -zx, largestZorX);
#endif

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

#if 0
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
#else
            qx = select(qx, tmpw, largestXorY);
            qy = select(qy, tmpz, largestXorY);
            qz = select(qz, tmpy, largestXorY);
            qw = select(qw, tmpx, largestXorY);

            tmpx = qx;
            tmpz = qz;

            qx = select(qx, qy, largestYorZ);
            qy = select(qy, tmpx, largestYorZ);
            qz = select(qz, qw, largestYorZ);
            qw = select(qw, tmpz, largestYorZ);
#endif

            x = qx;
            y = qy;
            z = qz;
            w = qw;
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> outer(const SoaVector3<SoaNFloat> & tfrm0, const SoaVector3<SoaNFloat> & tfrm1)
        {
            return SoaMatrix3<SoaNFloat>(
                (tfrm0 * tfrm1.getX()),
                (tfrm0 * tfrm1.getY()),
                (tfrm0 * tfrm1.getZ())
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> outer(const SoaVector4<SoaNFloat> & tfrm0, const SoaVector4<SoaNFloat> & tfrm1)
        {
            return SoaMatrix4<SoaNFloat>(
                (tfrm0 * tfrm1.getX()),
                (tfrm0 * tfrm1.getY()),
                (tfrm0 * tfrm1.getZ()),
                (tfrm0 * tfrm1.getW())
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> mul(const SoaVector3<SoaNFloat> & vec, const SoaMatrix3<SoaNFloat> & mat)
        {
            return SoaVector3<SoaNFloat>(
                (((vec.getX() * mat.getCol0().getX()) + (vec.getY() * mat.getCol0().getY())) + (vec.getZ() * mat.getCol0().getZ())),
                (((vec.getX() * mat.getCol1().getX()) + (vec.getY() * mat.getCol1().getY())) + (vec.getZ() * mat.getCol1().getZ())),
                (((vec.getX() * mat.getCol2().getX()) + (vec.getY() * mat.getCol2().getY())) + (vec.getZ() * mat.getCol2().getZ()))
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> crossMatrix(const SoaVector3<SoaNFloat> & vec)
        {
            return SoaMatrix3<SoaNFloat>(
                SoaVector3<SoaNFloat>(0.0f, vec.getZ(), -vec.getY()),
                SoaVector3<SoaNFloat>(-vec.getZ(), 0.0f, vec.getX()),
                SoaVector3<SoaNFloat>(vec.getY(), -vec.getX(), 0.0f)
                );
        }

        template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> crossMatrixMul(const SoaVector3<SoaNFloat> & vec, const SoaMatrix3<SoaNFloat> & mat)
        {
            return SoaMatrix3<SoaNFloat>(cross(vec, mat.getCol0()), cross(vec, mat.getCol1()), cross(vec, mat.getCol2()));
        }

} // namespace FmVectormath

#endif
