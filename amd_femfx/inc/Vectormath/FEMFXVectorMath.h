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
// Vector and matrix API with scalar, array-of-structures SIMD and structure-of-arrays
// SIMD versions
//---------------------------------------------------------------------------------------

#pragma once

#include <math.h>
#include <float.h>

// API derived from Sony Vectormath released with Bullet.
// Adds public element access and mul() instead of operator* for more similarity with HLSL.
// SSE version includes unions for element access.
#define _VECTORMATH_DEBUG
#include "vectormath_aos.h"        // scalar math implementation
#include "simd_vectormath_aos.h"   // SIMD implementation of AoS vectors
#include "soa_vectormath.h"        // SoA vector and matrix types; can template by SoA fundamental type
#include "soa_float.h"             // SoA fundamental types
#include "soa_int.h"
#include "soa_uint.h"
#include "soa_bool.h"

#define FM_NORMALIZE_MAG_SQR_TOL 1.0e-30f

#ifndef FM_FORCE_INLINE 
#define FM_FORCE_INLINE __forceinline
#endif

namespace AMD
{
// Sets default vector math types to the scalar implementation, which currently performs better.
// However SIMD vector types are still used explicitly in parts of the code.
#define FM_USE_SCALAR_VECTORMATH 1

#if FM_USE_SCALAR_VECTORMATH
    typedef FmVectormath::Vector3        FmVector3;
    typedef FmVectormath::Vector4        FmVector4;
    typedef FmVectormath::Point3         FmPoint3;
    typedef FmVectormath::Quat           FmQuat;
    typedef FmVectormath::Matrix3        FmMatrix3;
    typedef FmVectormath::Matrix4        FmMatrix4;
    typedef FmVectormath::Transform3     FmTransform3;
#else
    typedef FmVectormath::SimdVector3        FmVector3;
    typedef FmVectormath::SimdVector4        FmVector4;
    typedef FmVectormath::SimdPoint3         FmPoint3;
    typedef FmVectormath::SimdQuat           FmQuat;
    typedef FmVectormath::SimdMatrix3        FmMatrix3;
    typedef FmVectormath::SimdMatrix4        FmMatrix4;
    typedef FmVectormath::SimdTransform3     FmTransform3;
#endif

    typedef FmVectormath::SimdVector3    FmSimdVector3;
    typedef FmVectormath::SimdVector4    FmSimdVector4;
    typedef FmVectormath::SimdPoint3     FmSimdPoint3;
    typedef FmVectormath::SimdQuat       FmSimdQuat;
    typedef FmVectormath::SimdMatrix3    FmSimdMatrix3;
    typedef FmVectormath::SimdMatrix4    FmSimdMatrix4;
    typedef FmVectormath::SimdTransform3 FmSimdTransform3;
    typedef FmVectormath::Soa4Float      FmSoa4Float;
    typedef FmVectormath::Soa4Int        FmSoa4Int;
    typedef FmVectormath::Soa4Uint       FmSoa4Uint;
    typedef FmVectormath::Soa4Bool       FmSoa4Bool;
#if SIMD_UTILS_USE_AVX
    typedef FmVectormath::Soa8Float      FmSoa8Float;
    typedef FmVectormath::Soa8Int        FmSoa8Int;
    typedef FmVectormath::Soa8Uint       FmSoa8Uint;
    typedef FmVectormath::Soa8Bool       FmSoa8Bool;
#endif

    template <class T> using FmSoaVector3 = FmVectormath::SoaVector3<T>;
    template <class T> using FmSoaVector4 = FmVectormath::SoaVector4<T>;
    template <class T> using FmSoaPoint3 = FmVectormath::SoaPoint3<T>;
    template <class T> using FmSoaQuat = FmVectormath::SoaQuat<T>;
    template <class T> using FmSoaMatrix3 = FmVectormath::SoaMatrix3<T>;
    template <class T> using FmSoaMatrix4 = FmVectormath::SoaMatrix4<T>;
    template <class T> using FmSoaTransform3 = FmVectormath::SoaTransform3<T>;

    // Group of typedefs for corresponding SoA types.
    // Template argument for SoA functions.
    struct FmSoa4Types
    {
        typedef FmSoa4Float                  SoaFloat;
        typedef FmSoa4Int                    SoaInt;
        typedef FmSoa4Uint                   SoaUint;
        typedef FmSoa4Bool                   SoaBool;
        typedef FmSoaVector3<FmSoa4Float>    SoaVector3;
        typedef FmSoaVector4<FmSoa4Float>    SoaVector4;
        typedef FmSoaPoint3<FmSoa4Float>     SoaPoint3;
        typedef FmSoaQuat<FmSoa4Float>       SoaQuat;
        typedef FmSoaMatrix3<FmSoa4Float>    SoaMatrix3;
        typedef FmSoaMatrix4<FmSoa4Float>    SoaMatrix4;
        typedef FmSoaTransform3<FmSoa4Float> SoaTransform3;
        static const int                     width = 4;
    };

#if SIMD_UTILS_USE_AVX
    struct FmSoa8Types
    {
        typedef FmSoa8Float                  SoaFloat;
        typedef FmSoa8Int                    SoaInt;
        typedef FmSoa8Uint                   SoaUint;
        typedef FmSoa8Bool                   SoaBool;
        typedef FmSoaVector3<FmSoa8Float>    SoaVector3;
        typedef FmSoaVector4<FmSoa8Float>    SoaVector4;
        typedef FmSoaPoint3<FmSoa8Float>     SoaPoint3;
        typedef FmSoaQuat<FmSoa8Float>       SoaQuat;
        typedef FmSoaMatrix3<FmSoa8Float>    SoaMatrix3;
        typedef FmSoaMatrix4<FmSoa8Float>    SoaMatrix4;
        typedef FmSoaTransform3<FmSoa8Float> SoaTransform3;
        static const int                     width = 8;
    };
#endif

    typedef FmSoa8Types FmSoaTypes;

    template<class SoaTypes>
    static FM_FORCE_INLINE FmVector3 FmGetSlice(const typename SoaTypes::SoaVector3& vec, uint32_t sliceIdx)
    {
        return FmVector3(
            vec.x.getSlice(sliceIdx),
            vec.y.getSlice(sliceIdx),
            vec.z.getSlice(sliceIdx));
    }

    template<class SoaTypes>
    static FM_FORCE_INLINE void FmSetSlice(typename SoaTypes::SoaVector3* result, uint32_t sliceIdx, const FmVector3& vec)
    {
        result->x.setSlice(sliceIdx, vec.x);
        result->y.setSlice(sliceIdx, vec.y);
        result->z.setSlice(sliceIdx, vec.z);
    }

    template<class SoaTypes>
    static FM_FORCE_INLINE FmVector4 FmGetSlice(const typename SoaTypes::SoaVector4& vec, uint32_t sliceIdx)
    {
        return FmVector4(
            vec.x.getSlice(sliceIdx),
            vec.y.getSlice(sliceIdx),
            vec.z.getSlice(sliceIdx),
            vec.w.getSlice(sliceIdx));
    }

    template<class SoaTypes>
    static FM_FORCE_INLINE void FmSetSlice(typename SoaTypes::SoaVector4* result, uint32_t sliceIdx, const FmVector4& vec)
    {
        result->x.setSlice(sliceIdx, vec.x);
        result->y.setSlice(sliceIdx, vec.y);
        result->z.setSlice(sliceIdx, vec.z);
        result->w.setSlice(sliceIdx, vec.w);
    }

    template<class SoaTypes>
    static FM_FORCE_INLINE FmQuat FmGetSlice(const typename SoaTypes::SoaQuat& quat, uint32_t sliceIdx)
    {
        return FmQuat(
            quat.x.getSlice(sliceIdx),
            quat.y.getSlice(sliceIdx),
            quat.z.getSlice(sliceIdx),
            quat.w.getSlice(sliceIdx));
    }

    template<class SoaTypes>
    static FM_FORCE_INLINE void FmSetSlice(typename SoaTypes::SoaQuat* result, uint32_t sliceIdx, const FmQuat& quat)
    {
        result->x.setSlice(sliceIdx, quat.x);
        result->y.setSlice(sliceIdx, quat.y);
        result->z.setSlice(sliceIdx, quat.z);
        result->w.setSlice(sliceIdx, quat.w);
    }

    template<class SoaTypes>
    static FM_FORCE_INLINE FmMatrix3 FmGetSlice(const typename SoaTypes::SoaMatrix3& mat, uint32_t sliceIdx)
    {
        return FmMatrix3(
            FmGetSlice<SoaTypes>(mat.col0, sliceIdx),
            FmGetSlice<SoaTypes>(mat.col1, sliceIdx),
            FmGetSlice<SoaTypes>(mat.col2, sliceIdx));
    }

    template<class SoaTypes>
    static FM_FORCE_INLINE void FmSetSlice(typename SoaTypes::SoaMatrix3* result, uint32_t sliceIdx, const FmMatrix3& mat)
    {
        FmSetSlice<SoaTypes>(&result->col0, sliceIdx, mat.col0);
        FmSetSlice<SoaTypes>(&result->col1, sliceIdx, mat.col1);
        FmSetSlice<SoaTypes>(&result->col2, sliceIdx, mat.col2);
    }

    template<class SoaTypes>
    static FM_FORCE_INLINE FmMatrix4 FmGetSlice(const typename SoaTypes::SoaMatrix4& mat, uint32_t sliceIdx)
    {
        return FmMatrix4(
            FmGetSlice<SoaTypes>(mat.col0, sliceIdx),
            FmGetSlice<SoaTypes>(mat.col1, sliceIdx),
            FmGetSlice<SoaTypes>(mat.col2, sliceIdx),
            FmGetSlice<SoaTypes>(mat.col3, sliceIdx));
    }

    template<class SoaTypes>
    static FM_FORCE_INLINE void FmSetSlice(typename SoaTypes::SoaMatrix4* result, uint32_t sliceIdx, const FmMatrix4& mat)
    {
        FmSetSlice<SoaTypes>(&result->col0, sliceIdx, mat.col0);
        FmSetSlice<SoaTypes>(&result->col1, sliceIdx, mat.col1);
        FmSetSlice<SoaTypes>(&result->col2, sliceIdx, mat.col2);
        FmSetSlice<SoaTypes>(&result->col3, sliceIdx, mat.col3);
    }

    static FM_FORCE_INLINE void FmSetSoaFromUnalignedAos(FmSoa4Types::SoaVector3* result, const FmVector3* aosVecs)
    {
        Simd128Union v0, v1, v2;

        v0.f[0] = aosVecs[0].x;
        v0.f[1] = aosVecs[1].x;
        v0.f[2] = aosVecs[2].x;
        v0.f[3] = aosVecs[3].x;

        v1.f[0] = aosVecs[0].y;
        v1.f[1] = aosVecs[1].y;
        v1.f[2] = aosVecs[2].y;
        v1.f[3] = aosVecs[3].y;

        v2.f[0] = aosVecs[0].z;
        v2.f[1] = aosVecs[1].z;
        v2.f[2] = aosVecs[2].z;
        v2.f[3] = aosVecs[3].z;

        result->x = FmSoa4Float(v0.vf);
        result->y = FmSoa4Float(v1.vf);
        result->z = FmSoa4Float(v2.vf);
    }

    static FM_FORCE_INLINE void FmSetSoaFromUnalignedAos(FmSoa4Types::SoaVector4* result, const FmVector4* aosVecs)
    {
        Simd128Union v0, v1, v2, v3;

        v0.f[0] = aosVecs[0].x;
        v0.f[1] = aosVecs[1].x;
        v0.f[2] = aosVecs[2].x;
        v0.f[3] = aosVecs[3].x;

        v1.f[0] = aosVecs[0].y;
        v1.f[1] = aosVecs[1].y;
        v1.f[2] = aosVecs[2].y;
        v1.f[3] = aosVecs[3].y;

        v2.f[0] = aosVecs[0].z;
        v2.f[1] = aosVecs[1].z;
        v2.f[2] = aosVecs[2].z;
        v2.f[3] = aosVecs[3].z;

        v3.f[0] = aosVecs[0].w;
        v3.f[1] = aosVecs[1].w;
        v3.f[2] = aosVecs[2].w;
        v3.f[3] = aosVecs[3].w;

        result->x = FmSoa4Float(v0.vf);
        result->y = FmSoa4Float(v1.vf);
        result->z = FmSoa4Float(v2.vf);
        result->w = FmSoa4Float(v3.vf);
    }

#if FM_USE_SCALAR_VECTORMATH
    static FM_FORCE_INLINE void FmSetSoaFromAlignedAos(FmSoa4Types::SoaVector3* result, const FmVector3* aosVecs)
    {
        __m128 x, y, z;
        simd_set_soa_vec3(&x, &y, &z, (float*)aosVecs);
        *result = FmSoa4Types::SoaVector3(FmSoa4Float(x), FmSoa4Float(y), FmSoa4Float(z));
    }

    static FM_FORCE_INLINE void FmSetSoaFromAlignedAos(FmSoa4Types::SoaVector4* result, const FmVector4* aosVecs)
    {
        __m128 x, y, z, w;
        simd_set_soa_vec4(&x, &y, &z, &w, (float*)aosVecs);
        *result = FmSoa4Types::SoaVector4(FmSoa4Float(x), FmSoa4Float(y), FmSoa4Float(z), FmSoa4Float(w));
    }
#else
    static FM_FORCE_INLINE void FmSetSoaFromAlignedAos(FmSoa4Types::SoaVector3* result, const FmSimdVector3* aosVecs)
    {
        FmSimdMatrix4 mat;
        mat.col0 = FmSimdVector4(aosVecs[0].get128());
        mat.col1 = FmSimdVector4(aosVecs[1].get128());
        mat.col2 = FmSimdVector4(aosVecs[2].get128());
        mat.col3 = FmSimdVector4(aosVecs[3].get128());

        mat = transpose(mat);

        result->x = FmSoa4Float(mat.col0.get128());
        result->y = FmSoa4Float(mat.col1.get128());
        result->z = FmSoa4Float(mat.col2.get128());
    }

    static FM_FORCE_INLINE void FmSetSoaFromAlignedAos(FmSoa4Types::SoaVector4* result, const FmSimdVector4* aosVecs)
    {
        FmSimdMatrix4 mat;
        mat.col0 = aosVecs[0];
        mat.col1 = aosVecs[1];
        mat.col2 = aosVecs[2];
        mat.col3 = aosVecs[3];

        mat = transpose(mat);

        result->x = FmSoa4Float(mat.col0.get128());
        result->y = FmSoa4Float(mat.col1.get128());
        result->z = FmSoa4Float(mat.col2.get128());
        result->w = FmSoa4Float(mat.col3.get128());
    }
#endif

#if SIMD_UTILS_USE_AVX
#if FM_USE_SCALAR_VECTORMATH
    static FM_FORCE_INLINE void FmSetSoaFromAlignedAos(FmSoa8Types::SoaVector3* result, const FmVector3* aosVecs)
    {
        __m256 x, y, z;
        simd_set_soa_vec3(&x, &y, &z, (float*)aosVecs);
        *result = FmSoa8Types::SoaVector3(FmSoa8Float(x), FmSoa8Float(y), FmSoa8Float(z));
    }

    static FM_FORCE_INLINE void FmSetSoaFromAlignedAos(FmSoa8Types::SoaVector4* result, const FmVector4* aosVecs)
    {
        __m256 x, y, z, w;
        simd_set_soa_vec4(&x, &y, &z, &w, (float*)aosVecs);
        *result = FmSoa8Types::SoaVector4(FmSoa8Float(x), FmSoa8Float(y), FmSoa8Float(z), FmSoa8Float(w));
    }
#else
    static FM_FORCE_INLINE void FmSetSoaFromAlignedAos(FmSoa8Types::SoaVector3* result, const FmSimdVector3* aosVecs)
    {
        FmSimdMatrix4 mat0;
        mat0.col0 = FmSimdVector4(aosVecs[0].get128());
        mat0.col1 = FmSimdVector4(aosVecs[1].get128());
        mat0.col2 = FmSimdVector4(aosVecs[2].get128());
        mat0.col3 = FmSimdVector4(aosVecs[3].get128());

        mat0 = transpose(mat0);

        FmSimdMatrix4 mat1;
        mat1.col0 = FmSimdVector4(aosVecs[4].get128());
        mat1.col1 = FmSimdVector4(aosVecs[5].get128());
        mat1.col2 = FmSimdVector4(aosVecs[6].get128());
        mat1.col3 = FmSimdVector4(aosVecs[7].get128());

        mat1 = transpose(mat1);

        result->x = FmSoa8Float(_mm256_setr_m128(mat0.col0.get128(), mat1.col0.get128()));
        result->y = FmSoa8Float(_mm256_setr_m128(mat0.col1.get128(), mat1.col1.get128()));
        result->z = FmSoa8Float(_mm256_setr_m128(mat0.col2.get128(), mat1.col2.get128()));
    }

    static FM_FORCE_INLINE void FmSetSoaFromAlignedAos(FmSoa8Types::SoaVector4* result, const FmSimdVector4* aosVecs)
    {
        FmSimdMatrix4 mat0;
        mat0.col0 = aosVecs[0];
        mat0.col1 = aosVecs[1];
        mat0.col2 = aosVecs[2];
        mat0.col3 = aosVecs[3];

        mat0 = transpose(mat0);

        FmSimdMatrix4 mat1;
        mat1.col0 = aosVecs[4];
        mat1.col1 = aosVecs[5];
        mat1.col2 = aosVecs[6];
        mat1.col3 = aosVecs[7];

        mat1 = transpose(mat1);

        result->x = FmSoa8Float(_mm256_setr_m128(mat0.col0.get128(), mat1.col0.get128()));
        result->y = FmSoa8Float(_mm256_setr_m128(mat0.col1.get128(), mat1.col1.get128()));
        result->z = FmSoa8Float(_mm256_setr_m128(mat0.col2.get128(), mat1.col2.get128()));
        result->w = FmSoa8Float(_mm256_setr_m128(mat0.col3.get128(), mat1.col3.get128()));
    }
#endif
#endif

    // Init functions added for portability
    static FM_FORCE_INLINE const FmVector3 FmInitVector3(float val)
    {
        FmVector3 result;
        result.x = val;
        result.y = val;
        result.z = val;
        return result;
    }

    static FM_FORCE_INLINE const FmVector3 FmInitVector3(float x, float y, float z)
    {
        FmVector3 result;
        result.x = x;
        result.y = y;
        result.z = z;
        return result;
    }

    static FM_FORCE_INLINE const FmVector4 FmInitVector4(float val)
    {
        FmVector4 result;
        result.x = val;
        result.y = val;
        result.z = val;
        result.w = val;
        return result;
    }

    static FM_FORCE_INLINE const FmMatrix3 FmInitMatrix3(float val)
    {
        FmMatrix3 result;
        result.col0 = FmInitVector3(val);
        result.col1 = FmInitVector3(val);
        result.col2 = FmInitVector3(val);
        return result;
    }

    static FM_FORCE_INLINE const FmMatrix3 FmInitMatrix3(const FmVector3 & col0, const FmVector3 & col1, const FmVector3 & col2)
    {
        FmMatrix3 result;
        result.col0 = col0;
        result.col1 = col1;
        result.col2 = col2;
        return result;
    }

    static FM_FORCE_INLINE const FmMatrix3 FmInitMatrix3(const FmQuat &quat)
    {
        return FmMatrix3(quat);
    }

    static FM_FORCE_INLINE const FmQuat FmInitQuat(float x, float y, float z, float w)
    {
        FmQuat result;
        result.x = x;
        result.y = y;
        result.z = z;
        result.w = w;
        return result;
    }

    static FM_FORCE_INLINE const FmQuat FmInitQuat(const FmMatrix3& rotMat)
    {
        return FmQuat(rotMat);
    }

    static FM_FORCE_INLINE FmVector3 FmNormalize(float* lenSqr, const FmVector3& a)
    {
        float lSqr = dot(a, a);
        *lenSqr = lSqr;
        return a * 1.0f / sqrtf(lSqr);
    }

    static FM_FORCE_INLINE FmVector3 FmSafeNormalize(const FmVector3& a, const FmVector3& backup)
    {
        float lenSqr;
        FmVector3 result = FmNormalize(&lenSqr, a);
        return (lenSqr < FM_NORMALIZE_MAG_SQR_TOL) ? backup : result;
    }

    // Choose vector orthogonal to input and coordinate axis
    static FM_FORCE_INLINE FmVector3 FmOrthogonalVector(const FmVector3& v)
    {
        FmVector3 absVec = abs(v);
        return (absVec.x > absVec.y && absVec.x > absVec.z) ? FmInitVector3(-v.z, 0.0f, v.x) : FmInitVector3(0.0f, v.z, -v.y);
    }

    static FM_FORCE_INLINE FmVector3 FmCrossX(const FmVector3& v)
    {
        return FmInitVector3(0.0f, -v.z, v.y);
    }

    static FM_FORCE_INLINE FmVector3 FmSafeNormalizeCrossX(const FmVector3& v)
    {
        return FmSafeNormalize(FmCrossX(v), FmInitVector3(0.0f, 1.0f, 0.0f));
    }

    static FM_FORCE_INLINE FmVector3 FmCrossY(const FmVector3& v)
    {
        return FmInitVector3(v.z, 0.0f, -v.x);
    }

    static FM_FORCE_INLINE FmVector3 FmSafeNormalizeCrossY(const FmVector3& v)
    {
        return FmSafeNormalize(FmCrossY(v), FmInitVector3(0.0f, 0.0f, 1.0f));
    }

    static FM_FORCE_INLINE FmVector3 FmCrossZ(const FmVector3& v)
    {
        return FmInitVector3(-v.y, v.x, 0.0f);
    }

    static FM_FORCE_INLINE FmVector3 FmSafeNormalizeCrossZ(const FmVector3& v)
    {
        return FmSafeNormalize(FmCrossZ(v), FmInitVector3(1.0f, 0.0f, 0.0f));
    }

    static FM_FORCE_INLINE bool FmIsCloseToParallel(const FmVector3& dir1, const FmVector3& dir2, const float sinAngleThreshold = 0.01f)
    {
        return (lengthSqr(cross(dir1, dir2)) < sinAngleThreshold*sinAngleThreshold);
    }

    static FM_FORCE_INLINE bool FmIsCloseToOrthogonal(const FmVector3& dir1, const FmVector3& dir2, const float cosAngleThreshold = 0.01f)
    {
        return (fabsf(dot(dir1, dir2)) < cosAngleThreshold);
    }

    static FM_FORCE_INLINE bool FmIsZero(const FmVector3& vec)
    {
        return (vec.x == 0.0f && vec.y == 0.0f && vec.z == 0.0f);
    }

    static FM_FORCE_INLINE bool FmIsEqual(const FmVector3& vec0, const FmVector3& vec1)
    {
        return (vec0.x == vec1.x && vec0.y == vec1.y && vec0.z == vec1.z);
    }

    static FM_FORCE_INLINE const FmSimdVector3 FmInitSimdVector3(float val)
    {
        return FmSimdVector3(val);
    }

    static FM_FORCE_INLINE const FmSimdVector3 FmInitSimdVector3(float x, float y, float z)
    {
        return FmSimdVector3(x, y, z);
    }

    static FM_FORCE_INLINE const FmSimdMatrix3 FmInitSimdMatrix3(float val)
    {
        FmSimdVector3 vec = FmInitSimdVector3(val);
        FmSimdMatrix3 result;
        result.col0 = vec;
        result.col1 = vec;
        result.col2 = vec;
        return result;
    }

    static FM_FORCE_INLINE const FmSimdMatrix3 FmInitSimdMatrix3(const FmSimdVector3 & col0, const FmSimdVector3 & col1, const FmSimdVector3 & col2)
    {
        FmSimdMatrix3 result;
        result.col0 = col0;
        result.col1 = col1;
        result.col2 = col2;
        return result;
    }

#if FM_USE_SCALAR_VECTORMATH
    static FM_FORCE_INLINE FmSimdVector3 FmNormalize(float* lenSqr, const FmSimdVector3& a)
    {
        float lSqr = dot(a, a);
        *lenSqr = lSqr;
        return a * 1.0f / sqrtf(lSqr);
    }

    static FM_FORCE_INLINE FmSimdVector3 FmSafeNormalize(const FmSimdVector3& a, const FmSimdVector3& backup)
    {
        float lenSqr;
        FmSimdVector3 result = FmNormalize(&lenSqr, a);
        return (lenSqr < FM_NORMALIZE_MAG_SQR_TOL) ? backup : result;
    }

    // Choose vector orthogonal to input and coordinate axis
    static FM_FORCE_INLINE FmSimdVector3 FmOrthogonalVector(const FmSimdVector3& v)
    {
        FmSimdVector3 absVec = abs(v);
        return (absVec.x > absVec.y && absVec.x > absVec.z) ? FmInitSimdVector3(-v.z, 0.0f, v.x) : FmInitSimdVector3(0.0f, v.z, -v.y);
    }

    static FM_FORCE_INLINE FmSimdVector3 FmCrossX(const FmSimdVector3& v)
    {
        return FmInitSimdVector3(0.0f, -v.z, v.y);
    }

    static FM_FORCE_INLINE FmSimdVector3 FmSafeNormalizeCrossX(const FmSimdVector3& v)
    {
        return FmSafeNormalize(FmCrossX(v), FmInitSimdVector3(0.0f, 1.0f, 0.0f));
    }

    static FM_FORCE_INLINE FmSimdVector3 FmCrossY(const FmSimdVector3& v)
    {
        return FmInitSimdVector3(v.z, 0.0f, -v.x);
    }

    static FM_FORCE_INLINE FmSimdVector3 FmSafeNormalizeCrossY(const FmSimdVector3& v)
    {
        return FmSafeNormalize(FmCrossY(v), FmInitSimdVector3(0.0f, 0.0f, 1.0f));
    }

    static FM_FORCE_INLINE FmSimdVector3 FmCrossZ(const FmSimdVector3& v)
    {
        return FmInitSimdVector3(-v.y, v.x, 0.0f);
    }

    static FM_FORCE_INLINE FmSimdVector3 FmSafeNormalizeCrossZ(const FmSimdVector3& v)
    {
        return FmSafeNormalize(FmCrossZ(v), FmInitSimdVector3(1.0f, 0.0f, 0.0f));
    }

    static FM_FORCE_INLINE bool FmIsCloseToParallel(const FmSimdVector3& dir1, const FmSimdVector3& dir2, const float sinAngleThreshold = 0.01f)
    {
        return (lengthSqr(cross(dir1, dir2)) < sinAngleThreshold*sinAngleThreshold);
    }

    static FM_FORCE_INLINE bool FmIsCloseToOrthogonal(const FmSimdVector3& dir1, const FmSimdVector3& dir2, const float cosAngleThreshold = 0.01f)
    {
        return (fabsf(dot(dir1, dir2)) < cosAngleThreshold);
    }

    static FM_FORCE_INLINE FmSimdVector3 FmTransposeMul(const FmSimdMatrix3& mat, const FmSimdVector3& vec)
    {
        return FmSimdVector3(simd_3dot3_ps(mat.col0.get128(), mat.col1.get128(), mat.col2.get128(), vec.get128()));
    }
#endif

    static FM_FORCE_INLINE FmSimdVector3 FmInitSimdVector3(const FmVector3& vec)
    {
        __m128 m = simd_load3_unaligned_ps(&vec.x);
        return FmSimdVector3(m);
        //return FmSimdVector3(vec.x, vec.y, vec.z);
    }

    static FM_FORCE_INLINE FmSimdMatrix3 FmInitSimdMatrix3(const FmMatrix3& mat)
    {
        return FmSimdMatrix3(
            FmInitSimdVector3(mat.col0),
            FmInitSimdVector3(mat.col1),
            FmInitSimdVector3(mat.col2));
    }

    static FM_FORCE_INLINE FmVector3 FmInitVector3(const FmSimdVector3& vec)
    {
        FmVector3 result;
        simd_store3_unaligned_ps(&result.x, vec.mVec128);
        return result;
        //return FmInitVector3(vec.x, vec.y, vec.z);
    }

    static FM_FORCE_INLINE FmMatrix3 FmInitMatrix3(const FmSimdMatrix3& mat)
    {
        return FmMatrix3(FmInitVector3(mat.getCol0()), FmInitVector3(mat.getCol1()), FmInitVector3(mat.getCol2()));
    }

    template<class SoaTypes>
    static FM_FORCE_INLINE const typename SoaTypes::SoaVector3 FmInitVector3(typename SoaTypes::SoaFloat x, typename SoaTypes::SoaFloat y, typename SoaTypes::SoaFloat z)
    {
        typename SoaTypes::SoaVector3 result;
        result.x = x;
        result.y = y;
        result.z = z;
        return result;
    }

    template<class SoaTypes>
    static FM_FORCE_INLINE const typename SoaTypes::SoaMatrix3 FmInitMatrix3(const typename SoaTypes::SoaVector3 & col0, const typename SoaTypes::SoaVector3 & col1, const typename SoaTypes::SoaVector3 & col2)
    {
        typename SoaTypes::SoaMatrix3 result;
        result.col0 = col0;
        result.col1 = col1;
        result.col2 = col2;
        return result;
    }

    template<class SoaTypes>
    static FM_FORCE_INLINE const typename SoaTypes::SoaQuat FmInitQuat(const typename SoaTypes::SoaMatrix3 & rotMat)
    {
        return SoaTypes::SoaQuat(rotMat);
    }

    static FM_FORCE_INLINE FmVector3 FmTransposeMul(const FmMatrix3& mat, const FmVector3& vec)
    {
        return mul(transpose(mat), vec);
    }

    static FM_FORCE_INLINE FmQuat FmIntegrateQuat(const FmQuat& quat, const FmVector3& angVel, float dt)
    {
        FmQuat omega = FmQuat(angVel.x, angVel.y, angVel.z, 0.0f);
        FmQuat quatDot = 0.5f * mul(omega, quat);
        return normalize(quat + quatDot * dt);
    }

    // Sum of vertices weighted by barycentric values
    static FM_FORCE_INLINE FmVector3 FmInterpolate(const float bary[4], const FmVector3& vec0, const FmVector3& vec1, const FmVector3& vec2, const FmVector3& vec3)
    {
        return vec0 * bary[0] + vec1 * bary[1] + vec2 * bary[2] + vec3 * bary[3];
    }
}
