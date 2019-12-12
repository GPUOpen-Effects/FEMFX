/*
   Copyright (C) 2006, 2010 Sony Computer Entertainment Inc.
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

#ifndef _SOA_FLOAT_H
#define _SOA_FLOAT_H

#include "vectormath_utils.h"
#pragma warning(push, 0)
#ifndef USE_SSE2
#define USE_SSE2
#endif
#ifndef __AVX2__
#define __AVX2__
#endif
#include "sse_mathfun.h"
#include "avx_mathfun.h"
#pragma warning(pop)

namespace FmVectormath {

    class Soa4Int;
    class Soa4Uint;
    class Soa4Bool;

    //--------------------------------------------------------------------------------------------------
    // Soa4Float class
    //

    class Soa4Float
    {
    private:
        __m128 mData;

    public:
        SIMD_VECTORMATH_FORCE_INLINE Soa4Float() {}
        SIMD_VECTORMATH_FORCE_INLINE Soa4Float(const Soa4Float& other);

        // construct from __m128
        //
        explicit SIMD_VECTORMATH_FORCE_INLINE Soa4Float(__m128 vec);

        // construct from scalar array
        //
        explicit SIMD_VECTORMATH_FORCE_INLINE Soa4Float(const float* scalars);

        // broadcast float
        //
        SIMD_VECTORMATH_FORCE_INLINE Soa4Float(float scalar);

        // construct from four values
        //
        SIMD_VECTORMATH_FORCE_INLINE Soa4Float(float scalar0, float scalar1, float scalar2, float scalar3);

        // get vector data
        //
        SIMD_VECTORMATH_FORCE_INLINE __m128 get128() const;

        // get/set slice
        //
        SIMD_VECTORMATH_FORCE_INLINE float getSlice(uint32_t idx) const;
        SIMD_VECTORMATH_FORCE_INLINE void setSlice(uint32_t idx, float value);

        // set from aligned data
        SIMD_VECTORMATH_FORCE_INLINE void setFromAligned(float* alignedFloats);

        // operators
        // 
        SIMD_VECTORMATH_FORCE_INLINE const Soa4Float operator ++ (int);
        SIMD_VECTORMATH_FORCE_INLINE const Soa4Float operator -- (int);
        SIMD_VECTORMATH_FORCE_INLINE Soa4Float& operator ++ ();
        SIMD_VECTORMATH_FORCE_INLINE Soa4Float& operator -- ();
        SIMD_VECTORMATH_FORCE_INLINE const Soa4Float operator - () const;
        SIMD_VECTORMATH_FORCE_INLINE Soa4Float& operator = (const Soa4Float & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa4Float& operator *= (const Soa4Float & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa4Float& operator /= (const Soa4Float & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa4Float& operator += (const Soa4Float & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa4Float& operator -= (const Soa4Float & vec);

        typedef Soa4Bool SoaBool;
        typedef Soa4Int SoaInt;
        typedef Soa4Uint SoaUint;
        static const uint32_t width = 4;
    };

    //--------------------------------------------------------------------------------------------------
    // Soa4Float functions
    //

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float operator * (const Soa4Float & vec0, const Soa4Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float operator / (const Soa4Float & vec0, const Soa4Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float operator + (const Soa4Float & vec0, const Soa4Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float operator - (const Soa4Float & vec0, const Soa4Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator < (const Soa4Float & vec0, const Soa4Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator <= (const Soa4Float & vec0, const Soa4Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator > (const Soa4Float & vec0, const Soa4Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator >= (const Soa4Float & vec0, const Soa4Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator == (const Soa4Float & vec0, const Soa4Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator != (const Soa4Float & vec0, const Soa4Float & vec1);

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float max(const Soa4Float & vec0, const Soa4Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float min(const Soa4Float & vec0, const Soa4Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float sqrtf(const Soa4Float & vec);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float fabsf(const Soa4Float & vec);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float acosf(const Soa4Float & vec);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float tanf(const Soa4Float & vec);
    SIMD_VECTORMATH_FORCE_INLINE void sincosf(const Soa4Float & vec, Soa4Float* s, Soa4Float* c);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float expf(const Soa4Float & vec);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float logf(const Soa4Float & vec);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float powf(const Soa4Float & vec0, const Soa4Float & vec1);

    // select between vec0 and vec1 using Soa4Bool. false selects vec0, true selects vec1
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float select(const Soa4Float & vec0, const Soa4Float & vec1, const Soa4Bool &select_vec1);

    SIMD_VECTORMATH_FORCE_INLINE void print(const Soa4Float & vec);
    SIMD_VECTORMATH_FORCE_INLINE void print(const Soa4Float & vec, const char* name);

#if SIMD_UTILS_USE_AVX
    class Soa8Int;
    class Soa8Uint;
    class Soa8Bool;

    //--------------------------------------------------------------------------------------------------
    // Soa8Float class
    //

    class Soa8Float
    {
    private:
        __m256 mData;

    public:
        SIMD_VECTORMATH_FORCE_INLINE Soa8Float() {}
        SIMD_VECTORMATH_FORCE_INLINE Soa8Float(const Soa8Float& other);

        // construct from __m256
        //
        explicit SIMD_VECTORMATH_FORCE_INLINE Soa8Float(__m256 vec);

        // construct from scalar array
        //
        explicit SIMD_VECTORMATH_FORCE_INLINE Soa8Float(const float* scalars);

        // broadcast float
        //
        SIMD_VECTORMATH_FORCE_INLINE Soa8Float(float scalar);

        // construct from four values
        //
        SIMD_VECTORMATH_FORCE_INLINE Soa8Float(
            float scalar0, float scalar1, float scalar2, float scalar3,
            float scalar4, float scalar5, float scalar6, float scalar7);

        // matches standard type conversions
        //
        SIMD_VECTORMATH_FORCE_INLINE Soa8Float(const Soa8Bool & vec);

        // get vector data
        //
        SIMD_VECTORMATH_FORCE_INLINE __m256 get256() const;

        // get/set element
        //
        SIMD_VECTORMATH_FORCE_INLINE float getSlice(uint32_t idx) const;
        SIMD_VECTORMATH_FORCE_INLINE void setSlice(uint32_t idx, float value);

        // operators
        // 
        SIMD_VECTORMATH_FORCE_INLINE const Soa8Float operator ++ (int);
        SIMD_VECTORMATH_FORCE_INLINE const Soa8Float operator -- (int);
        SIMD_VECTORMATH_FORCE_INLINE Soa8Float& operator ++ ();
        SIMD_VECTORMATH_FORCE_INLINE Soa8Float& operator -- ();
        SIMD_VECTORMATH_FORCE_INLINE const Soa8Float operator - () const;
        SIMD_VECTORMATH_FORCE_INLINE Soa8Float& operator = (const Soa8Float & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa8Float& operator *= (const Soa8Float & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa8Float& operator /= (const Soa8Float & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa8Float& operator += (const Soa8Float & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa8Float& operator -= (const Soa8Float & vec);

        typedef Soa8Bool SoaBool;
        typedef Soa8Int SoaInt;
        typedef Soa8Uint SoaUint;
        static const uint32_t width = 8;
    };

    //--------------------------------------------------------------------------------------------------
    // Soa8Float functions
    //

    // operators
    // 
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float operator * (const Soa8Float & vec0, const Soa8Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float operator / (const Soa8Float & vec0, const Soa8Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float operator + (const Soa8Float & vec0, const Soa8Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float operator - (const Soa8Float & vec0, const Soa8Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator < (const Soa8Float & vec0, const Soa8Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator <= (const Soa8Float & vec0, const Soa8Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator > (const Soa8Float & vec0, const Soa8Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator >= (const Soa8Float & vec0, const Soa8Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator == (const Soa8Float & vec0, const Soa8Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator != (const Soa8Float & vec0, const Soa8Float & vec1);

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float max(const Soa8Float & vec0, const Soa8Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float min(const Soa8Float & vec0, const Soa8Float & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float sqrtf(const Soa8Float & vec);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float fabsf(const Soa8Float & vec);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float acosf(const Soa8Float & vec);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float tanf(const Soa8Float & vec);
    SIMD_VECTORMATH_FORCE_INLINE void sincosf(const Soa8Float& vec, Soa8Float* s, Soa8Float* c);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float expf(const Soa8Float & vec);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float logf(const Soa8Float & vec);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float powf(const Soa8Float & vec0, const Soa8Float & vec1);

    // select between vec0 and vec1 using Soa8Bool.
    // false selects vec0, true selects vec1
    //
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float select(const Soa8Float & vec0, const Soa8Float & vec1, const Soa8Bool &select_vec1);

    SIMD_VECTORMATH_FORCE_INLINE void print(const Soa8Float& vec);
#endif // SIMD_UTILS_USE_AVX

} // namespace FmVectormath

#include "soa_int.h"
#include "soa_uint.h"
#include "soa_bool.h"

namespace FmVectormath {

    //--------------------------------------------------------------------------------------------------
    // Soa4Float implementation
    //

    SIMD_VECTORMATH_FORCE_INLINE
        Soa4Float::Soa4Float(__m128 vec)
    {
        mData = vec;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Float::Soa4Float(const float* scalars)
    {
        mData = simd_load4_unaligned_ps(scalars);
    }

    SIMD_VECTORMATH_FORCE_INLINE
        Soa4Float::Soa4Float(float scalar)
    {
        mData = _mm_set1_ps(scalar);
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Float::Soa4Float(const Soa4Float & other)
    {
        mData = other.mData;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Float::Soa4Float(float scalar0, float scalar1, float scalar2, float scalar3)
    {
        mData = _mm_setr_ps(scalar0, scalar1, scalar2, scalar3);
    }

    SIMD_VECTORMATH_FORCE_INLINE __m128 Soa4Float::get128() const
    {
        return mData;
    }

    SIMD_VECTORMATH_FORCE_INLINE float Soa4Float::getSlice(uint32_t idx) const
    {
        return simd_getelem_ps(mData, (int)idx);
    }

    SIMD_VECTORMATH_FORCE_INLINE void Soa4Float::setSlice(uint32_t idx, float value)
    {
        simd_setelem_ps(mData, value, (int)idx);
    }

    SIMD_VECTORMATH_FORCE_INLINE void Soa4Float::setFromAligned(float* alignedFloats)
    {
        mData = _mm_load_ps(alignedFloats);
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float Soa4Float::operator ++ (int)
    {
        __m128 olddata = mData;
        operator ++();
        return Soa4Float(olddata);
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float Soa4Float::operator -- (int)
    {
        __m128 olddata = mData;
        operator --();
        return Soa4Float(olddata);
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Float& Soa4Float::operator ++ ()
    {
        *this += Soa4Float(_mm_set1_ps(1.0f));
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Float& Soa4Float::operator -- ()
    {
        *this -= Soa4Float(_mm_set1_ps(1.0f));
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float Soa4Float::operator - () const
    {
        return Soa4Float(_mm_sub_ps(_mm_setzero_ps(), mData));
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Float& Soa4Float::operator = (const Soa4Float & vec)
    {
        mData = vec.mData;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Float& Soa4Float::operator *= (const Soa4Float & vec)
    {
        *this = *this * vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Float& Soa4Float::operator /= (const Soa4Float & vec)
    {
        *this = *this / vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Float& Soa4Float::operator += (const Soa4Float & vec)
    {
        *this = *this + vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Float& Soa4Float::operator -= (const Soa4Float & vec)
    {
        *this = *this - vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float operator * (const Soa4Float & vec0, const Soa4Float & vec1)
    {
        return Soa4Float(_mm_mul_ps(vec0.get128(), vec1.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float operator / (const Soa4Float &num, const Soa4Float &den)
    {
        return Soa4Float(_mm_div_ps(num.get128(), den.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float operator + (const Soa4Float & vec0, const Soa4Float & vec1)
    {
        return Soa4Float(_mm_add_ps(vec0.get128(), vec1.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float operator - (const Soa4Float & vec0, const Soa4Float & vec1)
    {
        return Soa4Float(_mm_sub_ps(vec0.get128(), vec1.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator < (const Soa4Float & vec0, const Soa4Float & vec1)
    {
        return Soa4Bool(_mm_cmpgt_ps(vec1.get128(), vec0.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator <= (const Soa4Float & vec0, const Soa4Float & vec1)
    {
        return Soa4Bool(_mm_cmpge_ps(vec1.get128(), vec0.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator > (const Soa4Float & vec0, const Soa4Float & vec1)
    {
        return Soa4Bool(_mm_cmpgt_ps(vec0.get128(), vec1.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator >= (const Soa4Float & vec0, const Soa4Float & vec1)
    {
        return Soa4Bool(_mm_cmpge_ps(vec0.get128(), vec1.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator == (const Soa4Float & vec0, const Soa4Float & vec1)
    {
        return Soa4Bool(_mm_cmpeq_ps(vec0.get128(), vec1.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator != (const Soa4Float & vec0, const Soa4Float & vec1)
    {
        return Soa4Bool(_mm_cmpneq_ps(vec0.get128(), vec1.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float select(const Soa4Float & vec0, const Soa4Float & vec1, const Soa4Bool &select_vec1)
    {
        return Soa4Float(simd_select_ps(vec0.get128(), vec1.get128(), select_vec1.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float max(const Soa4Float & vec0, const Soa4Float & vec1)
    {
        return Soa4Float(_mm_max_ps(vec0.get128(), vec1.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float min(const Soa4Float & vec0, const Soa4Float & vec1)
    {
        return Soa4Float(_mm_min_ps(vec0.get128(), vec1.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float sqrtf(const Soa4Float & vec)
    {
        return Soa4Float(_mm_sqrt_ps(vec.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float fabsf(const Soa4Float & vec)
    {
        return Soa4Float(simd_abs_ps(vec.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float acosf(const Soa4Float & vec)
    {
        return Soa4Float(simd_acos_ps(vec.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float sinf(const Soa4Float & vec)
    {
        return Soa4Float(simd_sin_ps(vec.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float tanf(const Soa4Float & vec)
    {
        __m128 ms, mc;
        simd_sincos_ps(vec.get128(), &ms, &mc);
        return Soa4Float(ms) / Soa4Float(mc);
    }

    SIMD_VECTORMATH_FORCE_INLINE void sincosf(const Soa4Float & vec, Soa4Float* s, Soa4Float* c)
    {
        __m128 ms, mc;
        simd_sincos_ps(vec.get128(), &ms, &mc);
        *s = Soa4Float(ms);
        *c = Soa4Float(mc);
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float expf(const Soa4Float & vec)
    {
        return Soa4Float(exp_ps(vec.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float logf(const Soa4Float & vec)
    {
        return Soa4Float(log_ps(vec.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Float powf(const Soa4Float & vec0, const Soa4Float & vec1)
    {
        return Soa4Float(expf(vec1 * logf(vec0)));
    }

    SIMD_VECTORMATH_FORCE_INLINE void print(const Soa4Float & vec)
    {
        printf("%f %f %f %f\n", vec.getSlice(0), vec.getSlice(1), vec.getSlice(2), vec.getSlice(3));
    }

    SIMD_VECTORMATH_FORCE_INLINE void print(const Soa4Float & vec, const char* name)
    {
        printf("%s: %f %f %f %f\n", name, vec.getSlice(0), vec.getSlice(1), vec.getSlice(2), vec.getSlice(3));
    }

#if SIMD_UTILS_USE_AVX
    //--------------------------------------------------------------------------------------------------
    // Soa8Float implementation
    //

    SIMD_VECTORMATH_FORCE_INLINE Soa8Float::Soa8Float(const Soa8Float& other)
    {
        mData = other.mData;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Float::Soa8Float(__m256 vec)
    {
        mData = vec;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Float::Soa8Float(const float* scalars)
    {
        mData = simd_load8_unaligned_ps(scalars);
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Float::Soa8Float(float scalar)
    {
        mData = _mm256_set1_ps(scalar);
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Float::Soa8Float(
        float scalar0, float scalar1, float scalar2, float scalar3,
        float scalar4, float scalar5, float scalar6, float scalar7)
    {
        mData.m256_f32[0] = scalar0;
        mData.m256_f32[1] = scalar1;
        mData.m256_f32[2] = scalar2;
        mData.m256_f32[3] = scalar3;
        mData.m256_f32[4] = scalar4;
        mData.m256_f32[5] = scalar5;
        mData.m256_f32[6] = scalar6;
        mData.m256_f32[7] = scalar7;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Float::Soa8Float(const Soa8Bool & vec)
    {
        mData = simd_select_ps(_mm256_setzero_ps(), _mm256_set1_ps(1.0f), vec.get256());
    }

    SIMD_VECTORMATH_FORCE_INLINE __m256 Soa8Float::get256() const
    {
        return mData;
    }

    SIMD_VECTORMATH_FORCE_INLINE float Soa8Float::getSlice(uint32_t idx) const
    {
        return mData.m256_f32[idx];
    }

    SIMD_VECTORMATH_FORCE_INLINE void Soa8Float::setSlice(uint32_t idx, float value)
    {
        mData.m256_f32[idx] = value;
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float Soa8Float::operator ++ (int)
    {
        __m256 olddata = mData;
        operator ++();
        return Soa8Float(olddata);
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float Soa8Float::operator -- (int)
    {
        __m256 olddata = mData;
        operator --();
        return Soa8Float(olddata);
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Float& Soa8Float::operator ++ ()
    {
        *this += Soa8Float(_mm256_set1_ps(1.0f));
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Float& Soa8Float::operator -- ()
    {
        *this -= Soa8Float(_mm256_set1_ps(1.0f));
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float Soa8Float::operator - () const
    {
        return Soa8Float(_mm256_sub_ps(_mm256_setzero_ps(), mData));
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Float& Soa8Float::operator = (const Soa8Float & vec)
    {
        mData = vec.mData;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Float& Soa8Float::operator *= (const Soa8Float & vec)
    {
        *this = *this * vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Float& Soa8Float::operator /= (const Soa8Float & vec)
    {
        *this = *this / vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Float& Soa8Float::operator += (const Soa8Float & vec)
    {
        *this = *this + vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Float& Soa8Float::operator -= (const Soa8Float & vec)
    {
        *this = *this - vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float operator * (const Soa8Float & vec0, const Soa8Float & vec1)
    {
        return Soa8Float(_mm256_mul_ps(vec0.get256(), vec1.get256()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float operator / (const Soa8Float &num, const Soa8Float &den)
    {
        return Soa8Float(_mm256_div_ps(num.get256(), den.get256()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float operator + (const Soa8Float & vec0, const Soa8Float & vec1)
    {
        return Soa8Float(_mm256_add_ps(vec0.get256(), vec1.get256()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float operator - (const Soa8Float & vec0, const Soa8Float & vec1)
    {
        return Soa8Float(_mm256_sub_ps(vec0.get256(), vec1.get256()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator < (const Soa8Float & vec0, const Soa8Float & vec1)
    {
        return Soa8Bool(_mm256_cmp_ps(vec0.get256(), vec1.get256(), _CMP_LT_OQ));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator <= (const Soa8Float & vec0, const Soa8Float & vec1)
    {
        return Soa8Bool(_mm256_cmp_ps(vec0.get256(), vec1.get256(), _CMP_LE_OQ));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator > (const Soa8Float & vec0, const Soa8Float & vec1)
    {
        return Soa8Bool(_mm256_cmp_ps(vec0.get256(), vec1.get256(), _CMP_GT_OQ));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator >= (const Soa8Float & vec0, const Soa8Float & vec1)
    {
        return Soa8Bool(_mm256_cmp_ps(vec0.get256(), vec1.get256(), _CMP_GE_OQ));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator == (const Soa8Float & vec0, const Soa8Float & vec1)
    {
        return Soa8Bool(_mm256_cmp_ps(vec0.get256(), vec1.get256(), _CMP_EQ_OQ));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator != (const Soa8Float & vec0, const Soa8Float & vec1)
    {
        return Soa8Bool(_mm256_cmp_ps(vec0.get256(), vec1.get256(), _CMP_NEQ_OQ));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float select(const Soa8Float & vec0, const Soa8Float & vec1, const Soa8Bool &select_vec1)
    {
        return Soa8Float(simd_select_ps(vec0.get256(), vec1.get256(), select_vec1.get256()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float max(const Soa8Float & vec0, const Soa8Float & vec1)
    {
        return Soa8Float(_mm256_max_ps(vec0.get256(), vec1.get256()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float min(const Soa8Float & vec0, const Soa8Float & vec1)
    {
        return Soa8Float(_mm256_min_ps(vec0.get256(), vec1.get256()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float sqrtf(const Soa8Float & vec)
    {
        return Soa8Float(_mm256_sqrt_ps(vec.get256()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float fabsf(const Soa8Float & vec)
    {
        return Soa8Float(simd_abs_ps(vec.get256()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float acosf(const Soa8Float & vec)
    {
        return Soa8Float(simd_acos_ps(vec.get256()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float sinf(const Soa8Float & vec)
    {
        return Soa8Float(simd_sin_ps(vec.get256()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float tanf(const Soa8Float & vec)
    {
        __m256 ms, mc;
        simd_sincos_ps(vec.get256(), &ms, &mc);
        return Soa8Float(ms) / Soa8Float(mc);
    }

    SIMD_VECTORMATH_FORCE_INLINE void sincosf(const Soa8Float & vec, Soa8Float* s, Soa8Float* c)
    {
        __m256 ms, mc;
        simd_sincos_ps(vec.get256(), &ms, &mc);
        *s = Soa8Float(ms);
        *c = Soa8Float(mc);
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float expf(const Soa8Float & vec)
    {
        return Soa8Float(exp256_ps(vec.get256()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float logf(const Soa8Float & vec)
    {
        return Soa8Float(log256_ps(vec.get256()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Float powf(const Soa8Float & vec0, const Soa8Float & vec1)
    {
        return Soa8Float(expf(vec1 * logf(vec0)));
    }

    SIMD_VECTORMATH_FORCE_INLINE void print(const Soa8Float & vec)
    {
        printf("%f %f %f %f %f %f %f %f\n",
            vec.getSlice(0), vec.getSlice(1), vec.getSlice(2), vec.getSlice(3),
            vec.getSlice(4), vec.getSlice(5), vec.getSlice(6), vec.getSlice(7));
    }

    SIMD_VECTORMATH_FORCE_INLINE void print(const Soa8Float & vec, const char* name)
    {
        printf("%s: %f %f %f %f %f %f %f %f\n", name,
            vec.getSlice(0), vec.getSlice(1), vec.getSlice(2), vec.getSlice(3),
            vec.getSlice(4), vec.getSlice(5), vec.getSlice(6), vec.getSlice(7));
    }
#endif // SIMD_UTILS_USE_AVX

} // namespace FmVectormath

#endif // _SOA_FLOAT_H
