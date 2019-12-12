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

#ifndef _SOA_UINT_H
#define _SOA_UINT_H

#include "vectormath_utils.h"

namespace FmVectormath {

    class Soa4Float;
    class Soa4Int;
    class Soa4Bool;

    //--------------------------------------------------------------------------------------------------
    // Soa4Uint class
    //

    class Soa4Uint
    {
    private:
        __m128i mData;

    public:
        SIMD_VECTORMATH_FORCE_INLINE Soa4Uint() {}
        SIMD_VECTORMATH_FORCE_INLINE Soa4Uint(const Soa4Uint& other);

        // construct from __m128i
        //
        explicit SIMD_VECTORMATH_FORCE_INLINE Soa4Uint(__m128i vec);

        // construct from scalar array
        //
        explicit SIMD_VECTORMATH_FORCE_INLINE Soa4Uint(const uint32_t* scalars);

        // broadcast scalar
        //
        SIMD_VECTORMATH_FORCE_INLINE Soa4Uint(uint32_t scalar);
        SIMD_VECTORMATH_FORCE_INLINE Soa4Uint(int32_t scalar);

        // construct from four scalars
        //
        SIMD_VECTORMATH_FORCE_INLINE Soa4Uint(uint32_t scalar0, uint32_t scalar1, uint32_t scalar2, uint32_t scalar3);

        // matches standard type conversions
        //
        SIMD_VECTORMATH_FORCE_INLINE Soa4Uint(const Soa4Int & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa4Uint(const Soa4Bool & vec);

        // get vector data
        //
        SIMD_VECTORMATH_FORCE_INLINE __m128i get128i() const;

        // get/set slice
        //
        SIMD_VECTORMATH_FORCE_INLINE uint32_t getSlice(uint32_t idx) const;
        SIMD_VECTORMATH_FORCE_INLINE void setSlice(uint32_t idx, uint32_t value);

        // operators
        // 
        SIMD_VECTORMATH_FORCE_INLINE const Soa4Uint operator ++ (int);
        SIMD_VECTORMATH_FORCE_INLINE const Soa4Uint operator -- (int);
        SIMD_VECTORMATH_FORCE_INLINE Soa4Uint& operator ++ ();
        SIMD_VECTORMATH_FORCE_INLINE Soa4Uint& operator -- ();
        SIMD_VECTORMATH_FORCE_INLINE const Soa4Int operator - () const;
        SIMD_VECTORMATH_FORCE_INLINE Soa4Uint& operator = (const Soa4Uint & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa4Uint& operator *= (const Soa4Uint & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa4Uint& operator += (const Soa4Uint & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa4Uint& operator -= (const Soa4Uint & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa4Uint& operator &= (const Soa4Uint & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa4Uint& operator ^= (const Soa4Uint & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa4Uint& operator |= (const Soa4Uint & vec);

        typedef Soa4Float SoaFloat;
        typedef Soa4Int SoaInt;
        typedef Soa4Bool SoaBool;
        static const uint32_t width = 4;
    };

    //--------------------------------------------------------------------------------------------------
    // Soa4Uint functions
    //

    // operators
    // 
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Uint operator * (const Soa4Uint & vec0, const Soa4Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Uint operator + (const Soa4Uint & vec0, const Soa4Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Uint operator - (const Soa4Uint & vec0, const Soa4Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator < (const Soa4Uint & vec0, const Soa4Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator <= (const Soa4Uint & vec0, const Soa4Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator > (const Soa4Uint & vec0, const Soa4Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator >= (const Soa4Uint & vec0, const Soa4Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator == (const Soa4Uint & vec0, const Soa4Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator != (const Soa4Uint & vec0, const Soa4Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Uint operator & (const Soa4Uint & vec0, const Soa4Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Uint operator ^ (const Soa4Uint & vec0, const Soa4Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Uint operator | (const Soa4Uint & vec0, const Soa4Uint & vec1);

    // select between vec0 and vec1 using Soa4Bool. false selects vec0, true selects vec1
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Uint select(const Soa4Uint & vec0, const Soa4Uint & vec1, const Soa4Bool &select_vec1);

    SIMD_VECTORMATH_FORCE_INLINE void print(const Soa4Uint & vec);
    SIMD_VECTORMATH_FORCE_INLINE void print(const Soa4Uint & vec, const char* name);

#if SIMD_UTILS_USE_AVX
    class Soa8Float;
    class Soa8Int;
    class Soa8Bool;

    //--------------------------------------------------------------------------------------------------
    // Soa8Uint class
    //

    class Soa8Uint
    {
    private:
        __m256i mData;

    public:
        SIMD_VECTORMATH_FORCE_INLINE Soa8Uint() {}
        SIMD_VECTORMATH_FORCE_INLINE Soa8Uint(const Soa8Uint& other);

        // construct from __m256i
        //
        explicit SIMD_VECTORMATH_FORCE_INLINE Soa8Uint(__m256i vec);

        // construct from scalar array
        //
        explicit SIMD_VECTORMATH_FORCE_INLINE Soa8Uint(const uint32_t* scalars);

        // broadcast scalar
        //
        SIMD_VECTORMATH_FORCE_INLINE Soa8Uint(uint32_t scalar);
        SIMD_VECTORMATH_FORCE_INLINE Soa8Uint(int32_t scalar);

        // construct from four scalars
        //
        SIMD_VECTORMATH_FORCE_INLINE Soa8Uint(
            uint32_t scalar0, uint32_t scalar1, uint32_t scalar2, uint32_t scalar3,
            uint32_t scalar4, uint32_t scalar5, uint32_t scalar6, uint32_t scalar7);

        // matches standard type conversions
        //
        SIMD_VECTORMATH_FORCE_INLINE Soa8Uint(const Soa8Int & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa8Uint(const Soa8Bool & vec);

        // get vector data
        //
        SIMD_VECTORMATH_FORCE_INLINE __m256i get256i() const;

        // get/set element
        //
        SIMD_VECTORMATH_FORCE_INLINE uint32_t getSlice(uint32_t idx) const;
        SIMD_VECTORMATH_FORCE_INLINE void setSlice(uint32_t idx, uint32_t value);

        // operators
        // 
        SIMD_VECTORMATH_FORCE_INLINE const Soa8Uint operator ++ (int);
        SIMD_VECTORMATH_FORCE_INLINE const Soa8Uint operator -- (int);
        SIMD_VECTORMATH_FORCE_INLINE Soa8Uint& operator ++ ();
        SIMD_VECTORMATH_FORCE_INLINE Soa8Uint& operator -- ();
        SIMD_VECTORMATH_FORCE_INLINE const Soa8Int operator - () const;
        SIMD_VECTORMATH_FORCE_INLINE Soa8Uint& operator = (const Soa8Uint & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa8Uint& operator *= (const Soa8Uint & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa8Uint& operator += (const Soa8Uint & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa8Uint& operator -= (const Soa8Uint & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa8Uint& operator &= (const Soa8Uint & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa8Uint& operator ^= (const Soa8Uint & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa8Uint& operator |= (const Soa8Uint & vec);

        typedef Soa8Float SoaFloat;
        typedef Soa8Uint SoaUint;
        typedef Soa8Bool SoaBool;
        static const uint32_t width = 8;
    };

    //--------------------------------------------------------------------------------------------------
    // Soa8Uint functions
    //

    // operators
    // 
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Uint operator * (const Soa8Uint & vec0, const Soa8Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Uint operator + (const Soa8Uint & vec0, const Soa8Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Uint operator - (const Soa8Uint & vec0, const Soa8Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator < (const Soa8Uint & vec0, const Soa8Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator <= (const Soa8Uint & vec0, const Soa8Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator > (const Soa8Uint & vec0, const Soa8Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator >= (const Soa8Uint & vec0, const Soa8Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator == (const Soa8Uint & vec0, const Soa8Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator != (const Soa8Uint & vec0, const Soa8Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Uint operator & (const Soa8Uint & vec0, const Soa8Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Uint operator ^ (const Soa8Uint & vec0, const Soa8Uint & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Uint operator | (const Soa8Uint & vec0, const Soa8Uint & vec1);

    // select between vec0 and vec1 using Soa8Bool.
    // false selects vec0, true selects vec1
    //
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Uint select(const Soa8Uint & vec0, const Soa8Uint & vec1, const Soa8Bool &select_vec1);

    SIMD_VECTORMATH_FORCE_INLINE void print(const Soa8Int & vec);
    SIMD_VECTORMATH_FORCE_INLINE void print(const Soa8Int & vec, const char* name);
#endif // SIMD_UTILS_USE_AVX

} // namespace FmVectormath

#include "soa_float.h"
#include "soa_bool.h"
#include "soa_int.h"

namespace FmVectormath {

    //--------------------------------------------------------------------------------------------------
    // Soa4Uint implementation
    //

    SIMD_VECTORMATH_FORCE_INLINE Soa4Uint::Soa4Uint(const Soa4Uint & other)
    {
        mData = other.mData;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Uint::Soa4Uint(__m128i vec)
    {
        mData = vec;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Uint::Soa4Uint(const uint32_t* scalars)
    {
        mData = simd_load4_unaligned_ps((const int32_t*)scalars);
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Uint::Soa4Uint(uint32_t scalar)
    {
        mData = _mm_set1_epi32((int)scalar);
    }

    Soa4Uint::Soa4Uint(int32_t scalar)
    {
        mData = _mm_set1_epi32(scalar);
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Uint::Soa4Uint(uint32_t scalar0, uint32_t scalar1, uint32_t scalar2, uint32_t scalar3)
    {
        mData = _mm_setr_epi32((int32_t)scalar0, (int32_t)scalar1, (int32_t)scalar2, (int32_t)scalar3);
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Uint::Soa4Uint(const Soa4Int & vec)
    {
        mData = vec.get128i();
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Uint::Soa4Uint(const Soa4Bool & vec)
    {
        mData = _mm_sub_epi32(_mm_setzero_si128(), vec.get128i());
    }

    SIMD_VECTORMATH_FORCE_INLINE __m128i Soa4Uint::get128i() const
    {
        return mData;
    }

    SIMD_VECTORMATH_FORCE_INLINE uint32_t Soa4Uint::getSlice(uint32_t idx) const
    {
        return (uint32_t)simd_getelem_epi32(mData, (int)idx);
    }

    SIMD_VECTORMATH_FORCE_INLINE void Soa4Uint::setSlice(uint32_t idx, uint32_t value)
    {
        simd_setelem_epi32(mData, (int)value, (int)idx);
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Uint Soa4Uint::operator ++ (int)
    {
        __m128i olddata = mData;
        operator ++();
        return Soa4Uint(olddata);
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Uint Soa4Uint::operator -- (int)
    {
        __m128i olddata = mData;
        operator --();
        return Soa4Uint(olddata);
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Uint& Soa4Uint::operator ++ ()
    {
        *this += Soa4Uint(_mm_set1_epi32(1));
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Uint& Soa4Uint::operator -- ()
    {
        *this -= Soa4Uint(_mm_set1_epi32(1));
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Int Soa4Uint::operator - () const
    {
        return Soa4Int(_mm_sub_epi32(_mm_setzero_si128(), mData));
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Uint& Soa4Uint::operator = (const Soa4Uint & vec)
    {
        mData = vec.mData;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Uint& Soa4Uint::operator *= (const Soa4Uint & vec)
    {
        *this = *this * vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Uint& Soa4Uint::operator += (const Soa4Uint & vec)
    {
        *this = *this + vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Uint& Soa4Uint::operator -= (const Soa4Uint & vec)
    {
        *this = *this - vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Uint& Soa4Uint::operator &= (const Soa4Uint & vec)
    {
        *this = *this & vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Uint& Soa4Uint::operator ^= (const Soa4Uint & vec)
    {
        *this = *this ^ vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Uint& Soa4Uint::operator |= (const Soa4Uint & vec)
    {
        *this = *this | vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Uint operator * (const Soa4Uint & vec0, const Soa4Uint & vec1)
    {
        return Soa4Uint(simd_mul_epi32(vec0.get128i(), vec1.get128i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Uint operator + (const Soa4Uint & vec0, const Soa4Uint & vec1)
    {
        return Soa4Uint(_mm_add_epi32(vec0.get128i(), vec1.get128i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Uint operator - (const Soa4Uint & vec0, const Soa4Uint & vec1)
    {
        return Soa4Uint(_mm_sub_epi32(vec0.get128i(), vec1.get128i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator < (const Soa4Uint & vec0, const Soa4Uint & vec1)
    {
        return Soa4Bool(simd_cmplt_epu32(vec0.get128i(), vec1.get128i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator <= (const Soa4Uint & vec0, const Soa4Uint & vec1)
    {
        return Soa4Bool(simd_cmple_epu32(vec0.get128i(), vec1.get128i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator > (const Soa4Uint & vec0, const Soa4Uint & vec1)
    {
        return Soa4Bool(simd_cmpgt_epu32(vec0.get128i(), vec1.get128i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator >= (const Soa4Uint & vec0, const Soa4Uint & vec1)
    {
        return Soa4Bool(simd_cmpge_epu32(vec0.get128i(), vec1.get128i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator == (const Soa4Uint & vec0, const Soa4Uint & vec1)
    {
        return Soa4Bool(_mm_cmpeq_epi32(vec0.get128i(), vec1.get128i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator != (const Soa4Uint & vec0, const Soa4Uint & vec1)
    {
        return Soa4Bool(simd_not_si128(_mm_cmpeq_epi32(vec0.get128i(), vec1.get128i())));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Uint operator & (const Soa4Uint & vec0, const Soa4Uint & vec1)
    {
        return Soa4Uint(_mm_and_si128(vec0.get128i(), vec1.get128i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Uint operator ^ (const Soa4Uint & vec0, const Soa4Uint & vec1)
    {
        return Soa4Uint(_mm_xor_si128(vec0.get128i(), vec1.get128i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Uint operator | (const Soa4Uint & vec0, const Soa4Uint & vec1)
    {
        return Soa4Uint(_mm_or_si128(vec0.get128i(), vec1.get128i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Uint select(const Soa4Uint & vec0, const Soa4Uint & vec1, const Soa4Bool &select_vec1)
    {
        return Soa4Uint(simd_select_epi32(vec0.get128i(), vec1.get128i(), select_vec1.get128i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE void print(const Soa4Uint & vec)
    {
        printf("%u %u %u %u\n", vec.getSlice(0), vec.getSlice(1), vec.getSlice(2), vec.getSlice(3));
    }

    SIMD_VECTORMATH_FORCE_INLINE void print(const Soa4Uint & vec, const char* name)
    {
        printf("%s: %u %u %u %u\n", name, vec.getSlice(0), vec.getSlice(1), vec.getSlice(2), vec.getSlice(3));
    }

#if SIMD_UTILS_USE_AVX
    //--------------------------------------------------------------------------------------------------
    // Soa8Uint implementation
    //

    SIMD_VECTORMATH_FORCE_INLINE Soa8Uint::Soa8Uint(__m256i vec)
    {
        mData = vec;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Uint::Soa8Uint(const Soa8Uint & other)
    {
        mData = other.mData;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Uint::Soa8Uint(const uint32_t* scalars)
    {
        mData = simd_load8_unaligned_ps((const int32_t*)scalars);
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Uint::Soa8Uint(uint32_t scalar)
    {
        mData = _mm256_set1_epi32(scalar);
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Uint::Soa8Uint(int32_t scalar)
    {
        mData = _mm256_set1_epi32(scalar);
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Uint::Soa8Uint(
        uint32_t scalar0, uint32_t scalar1, uint32_t scalar2, uint32_t scalar3,
        uint32_t scalar4, uint32_t scalar5, uint32_t scalar6, uint32_t scalar7)
    {
        mData = _mm256_setr_epi32(
            scalar0, scalar1, scalar2, scalar3,
            scalar4, scalar5, scalar6, scalar7);
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Uint::Soa8Uint(const Soa8Int & vec)
    {
        mData = vec.get256i();
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Uint::Soa8Uint(const Soa8Bool & vec)
    {
        mData = _mm256_sub_epi32(_mm256_setzero_si256(), vec.get256i());
    }

    SIMD_VECTORMATH_FORCE_INLINE __m256i Soa8Uint::get256i() const
    {
        return mData;
    }

    SIMD_VECTORMATH_FORCE_INLINE uint32_t Soa8Uint::getSlice(uint32_t idx) const
    {
        return mData.m256i_i32[idx];
    }

    SIMD_VECTORMATH_FORCE_INLINE void Soa8Uint::setSlice(uint32_t idx, uint32_t value)
    {
        mData.m256i_i32[idx] = value;
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Uint Soa8Uint::operator ++ (int)
    {
        __m256i olddata = mData;
        operator ++();
        return Soa8Uint(olddata);
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Uint Soa8Uint::operator -- (int)
    {
        __m256i olddata = mData;
        operator --();
        return Soa8Uint(olddata);
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Uint& Soa8Uint::operator ++ ()
    {
        *this += Soa8Uint(_mm256_set1_epi32(1));
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Uint& Soa8Uint::operator -- ()
    {
        *this -= Soa8Uint(_mm256_set1_epi32(1));
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Int Soa8Uint::operator - () const
    {
        return Soa8Int(_mm256_sub_epi32(_mm256_setzero_si256(), mData));
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Uint& Soa8Uint::operator = (const Soa8Uint & vec)
    {
        mData = vec.mData;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Uint& Soa8Uint::operator *= (const Soa8Uint & vec)
    {
        *this = *this * vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Uint& Soa8Uint::operator += (const Soa8Uint & vec)
    {
        *this = *this + vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Uint& Soa8Uint::operator -= (const Soa8Uint & vec)
    {
        *this = *this - vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Uint& Soa8Uint::operator &= (const Soa8Uint & vec)
    {
        *this = *this & vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Uint& Soa8Uint::operator ^= (const Soa8Uint & vec)
    {
        *this = *this ^ vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Uint& Soa8Uint::operator |= (const Soa8Uint & vec)
    {
        *this = *this | vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Uint operator * (const Soa8Uint & vec0, const Soa8Uint & vec1)
    {
        return Soa8Uint(simd_mul_epi32(vec0.get256i(), vec1.get256i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Uint operator + (const Soa8Uint & vec0, const Soa8Uint & vec1)
    {
        return Soa8Uint(_mm256_add_epi32(vec0.get256i(), vec1.get256i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Uint operator - (const Soa8Uint & vec0, const Soa8Uint & vec1)
    {
        return Soa8Uint(_mm256_sub_epi32(vec0.get256i(), vec1.get256i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator < (const Soa8Uint & vec0, const Soa8Uint & vec1)
    {
        return Soa8Bool(simd_cmplt_epu32(vec0.get256i(), vec1.get256i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator <= (const Soa8Uint & vec0, const Soa8Uint & vec1)
    {
        return Soa8Bool(simd_cmple_epu32(vec0.get256i(), vec1.get256i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator > (const Soa8Uint & vec0, const Soa8Uint & vec1)
    {
        return Soa8Bool(simd_cmpgt_epu32(vec0.get256i(), vec1.get256i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator >= (const Soa8Uint & vec0, const Soa8Uint & vec1)
    {
        return Soa8Bool(simd_cmpge_epu32(vec0.get256i(), vec1.get256i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator == (const Soa8Uint & vec0, const Soa8Uint & vec1)
    {
        return Soa8Bool(_mm256_cmpeq_epi32(vec0.get256i(), vec1.get256i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator != (const Soa8Uint & vec0, const Soa8Uint & vec1)
    {
        return Soa8Bool(simd_not_si256(_mm256_cmpeq_epi32(vec0.get256i(), vec1.get256i())));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Uint operator & (const Soa8Uint & vec0, const Soa8Uint & vec1)
    {
        return Soa8Uint(_mm256_and_si256(vec0.get256i(), vec1.get256i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Uint operator ^ (const Soa8Uint & vec0, const Soa8Uint & vec1)
    {
        return Soa8Uint(_mm256_xor_si256(vec0.get256i(), vec1.get256i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Uint operator | (const Soa8Uint & vec0, const Soa8Uint & vec1)
    {
        return Soa8Uint(_mm256_or_si256(vec0.get256i(), vec1.get256i()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Uint select(const Soa8Uint & vec0, const Soa8Uint & vec1, const Soa8Bool &select_vec1)
    {
        return Soa8Uint(simd_select_epi32(vec0.get256i(), vec1.get256i(), select_vec1.get256i()));
    }
#endif // SIMD_UTILS_USE_AVX

} // namespace FmVectormath

#endif // _SOA_UINT_H
