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

#ifndef _SOA_BOOL_H
#define _SOA_BOOL_H

#include "vectormath_utils.h"

namespace FmVectormath {

    class Soa4Float;
    class Soa4Int;
    class Soa4Uint;

    //--------------------------------------------------------------------------------------------------
    // Soa4Bool class
    //

    class Soa4Bool
    {
    private:
        __m128 mData;

    public:
        SIMD_VECTORMATH_FORCE_INLINE Soa4Bool() {}
        SIMD_VECTORMATH_FORCE_INLINE Soa4Bool(const Soa4Bool& other);

        // construct from __m128
        explicit SIMD_VECTORMATH_FORCE_INLINE Soa4Bool(__m128 vec);
        explicit SIMD_VECTORMATH_FORCE_INLINE Soa4Bool(__m128i vec);

        // broadcast scalar 
        //
        SIMD_VECTORMATH_FORCE_INLINE Soa4Bool(bool scalar);

        // construct from scalars
        //
        SIMD_VECTORMATH_FORCE_INLINE Soa4Bool(bool scalar0, bool scalar1, bool scalar2, bool scalar3);

        // matches standard type conversions
        //
        SIMD_VECTORMATH_FORCE_INLINE Soa4Bool(const Soa4Float & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa4Bool(const Soa4Int & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa4Bool(const Soa4Uint & vec);

        // get vector data
        // bool value in each word slot is 0 (false) or -1 (true)
        //
        SIMD_VECTORMATH_FORCE_INLINE __m128 get128() const;
        SIMD_VECTORMATH_FORCE_INLINE __m128i get128i() const;

        // get/set slice
        //
        SIMD_VECTORMATH_FORCE_INLINE bool getSlice(uint32_t idx) const;
        SIMD_VECTORMATH_FORCE_INLINE void setSlice(uint32_t idx, bool value);

        // operators
        //
        SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator ! () const;
        SIMD_VECTORMATH_FORCE_INLINE Soa4Bool& operator = (const Soa4Bool & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa4Bool& operator &= (const Soa4Bool & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa4Bool& operator ^= (const Soa4Bool & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa4Bool& operator |= (const Soa4Bool & vec);

        typedef Soa4Float SoaFloat;
        typedef Soa4Int SoaInt;
        typedef Soa4Uint SoaUint;
        static const uint32_t width = 4;
    };

    //--------------------------------------------------------------------------------------------------
    // Soa4Bool functions
    //

    // operators
    //
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator == (const Soa4Bool & vec0, const Soa4Bool & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator != (const Soa4Bool & vec0, const Soa4Bool & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator & (const Soa4Bool & vec0, const Soa4Bool & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator ^ (const Soa4Bool & vec0, const Soa4Bool & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator | (const Soa4Bool & vec0, const Soa4Bool & vec1);

    // select between vec0 and vec1 using Soa4Bool. false selects vec0, true selects vec1
    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool select(const Soa4Bool & vec0, const Soa4Bool & vec1, const Soa4Bool &select_vec1);

    SIMD_VECTORMATH_FORCE_INLINE const bool all(const Soa4Bool & vec);
    SIMD_VECTORMATH_FORCE_INLINE const bool any(const Soa4Bool & vec);

    SIMD_VECTORMATH_FORCE_INLINE void print(const Soa4Bool & vec);
    SIMD_VECTORMATH_FORCE_INLINE void print(const Soa4Bool & vec, const char* name);

#if SIMD_UTILS_USE_AVX
    class Soa8Float;
    class Soa8Int;
    class Soa8Uint;

    //--------------------------------------------------------------------------------------------------
    // Soa8Bool class
    //

    class Soa8Bool
    {
    private:
        __m256 mData;

    public:
        SIMD_VECTORMATH_FORCE_INLINE Soa8Bool() {}
        SIMD_VECTORMATH_FORCE_INLINE Soa8Bool(const Soa8Bool& other);

        // construct from __m128
        explicit SIMD_VECTORMATH_FORCE_INLINE Soa8Bool(__m256 vec);
        explicit SIMD_VECTORMATH_FORCE_INLINE Soa8Bool(__m256i vec);

        // broadcast scalar 
        //
        SIMD_VECTORMATH_FORCE_INLINE Soa8Bool(bool scalar);

        // construct from scalars
        //
        SIMD_VECTORMATH_FORCE_INLINE Soa8Bool(
            bool scalar0, bool scalar1, bool scalar2, bool scalar3,
            bool scalar4, bool scalar5, bool scalar6, bool scalar7);

        // matches standard type conversions
        //
        SIMD_VECTORMATH_FORCE_INLINE Soa8Bool(const Soa8Float & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa8Bool(const Soa8Int & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa8Bool(const Soa8Uint & vec);

        // get vector data
        // bool value in each word slot is 0 (false) or -1 (true)
        //
        SIMD_VECTORMATH_FORCE_INLINE __m256 get256() const;
        SIMD_VECTORMATH_FORCE_INLINE __m256i get256i() const;

        // get/set element
        //
        SIMD_VECTORMATH_FORCE_INLINE bool getSlice(uint32_t idx) const;
        SIMD_VECTORMATH_FORCE_INLINE void setSlice(uint32_t idx, bool value);

        // operators
        //
        SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator ! () const;
        SIMD_VECTORMATH_FORCE_INLINE Soa8Bool& operator = (const Soa8Bool & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa8Bool& operator &= (const Soa8Bool & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa8Bool& operator ^= (const Soa8Bool & vec);
        SIMD_VECTORMATH_FORCE_INLINE Soa8Bool& operator |= (const Soa8Bool & vec);
    };

    //--------------------------------------------------------------------------------------------------
    // Soa8Bool functions
    //

    // operators
    //
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator == (const Soa8Bool & vec0, const Soa8Bool & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator != (const Soa8Bool & vec0, const Soa8Bool & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator & (const Soa8Bool & vec0, const Soa8Bool & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator ^ (const Soa8Bool & vec0, const Soa8Bool & vec1);
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator | (const Soa8Bool & vec0, const Soa8Bool & vec1);

    // select between vec0 and vec1 using Soa8Bool.
    // false selects vec0, true selects vec1
    //
    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool select(const Soa8Bool & vec0, const Soa8Bool & vec1, const Soa8Bool &select_vec1);

    SIMD_VECTORMATH_FORCE_INLINE const bool all(const Soa8Bool & vec);
    SIMD_VECTORMATH_FORCE_INLINE const bool any(const Soa8Bool & vec);

#endif // SIMD_UTILS_USE_AVX

} // namespace FmVectormath

#include "soa_float.h"
#include "soa_int.h"
#include "soa_uint.h"

namespace FmVectormath {

    //--------------------------------------------------------------------------------------------------
    // Soa4Bool implementation
    //

    SIMD_VECTORMATH_FORCE_INLINE Soa4Bool::Soa4Bool(const Soa4Bool & other)
    {
        mData = other.mData;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Bool::Soa4Bool(__m128 vec)
    {
        mData = vec;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Bool::Soa4Bool(__m128i vec)
    {
        union { __m128 vf; __m128i vi; } vfiunion;
        vfiunion.vi = vec;
        mData = vfiunion.vf;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Bool::Soa4Bool(bool scalar)
    {
        union { float f; int i; } fiunion;
        fiunion.i = -(int)scalar;
        mData = _mm_set1_ps(fiunion.f);
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Bool::Soa4Bool(bool scalar0, bool scalar1, bool scalar2, bool scalar3)
    {
        union { __m128 vf; __m128i vi; } vfiunion;
        vfiunion.vi = _mm_setr_epi32(-((int)scalar0), -((int)scalar1), -((int)scalar2), -((int)scalar3));
        mData = vfiunion.vf;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Bool::Soa4Bool(const Soa4Float & vec)
    {
        *this = (vec != Soa4Float(0.0f));
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Bool::Soa4Bool(const Soa4Int & vec)
    {
        *this = (vec != Soa4Int(0));
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Bool::Soa4Bool(const Soa4Uint & vec)
    {
        *this = (vec != Soa4Uint((uint32_t)0));
    }

    SIMD_VECTORMATH_FORCE_INLINE __m128 Soa4Bool::get128() const
    {
        return mData;
    }

    SIMD_VECTORMATH_FORCE_INLINE __m128i Soa4Bool::get128i() const
    {
        union { __m128 vf; __m128i vi; } vfiunion;
        vfiunion.vf = mData;
        return vfiunion.vi;
    }

    SIMD_VECTORMATH_FORCE_INLINE bool Soa4Bool::getSlice(uint32_t idx) const
    {
        return (simd_getelem_epi32(mData, (int)idx) != 0);
    }

    SIMD_VECTORMATH_FORCE_INLINE void Soa4Bool::setSlice(uint32_t idx, bool value)
    {
        simd_setelem_epi32(mData, -(int)value, (int)idx);
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool Soa4Bool::operator ! () const
    {
        return Soa4Bool(_mm_andnot_ps(mData, _mm_cmpeq_ps(_mm_setzero_ps(), _mm_setzero_ps())));
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Bool& Soa4Bool::operator = (const Soa4Bool & vec)
    {
        mData = vec.mData;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Bool& Soa4Bool::operator &= (const Soa4Bool & vec)
    {
        *this = *this & vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Bool& Soa4Bool::operator ^= (const Soa4Bool & vec)
    {
        *this = *this ^ vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa4Bool& Soa4Bool::operator |= (const Soa4Bool & vec)
    {
        *this = *this | vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator == (const Soa4Bool & vec0, const Soa4Bool & vec1)
    {
        return Soa4Bool(_mm_cmpeq_ps(vec0.get128(), vec1.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator != (const Soa4Bool & vec0, const Soa4Bool & vec1)
    {
        return Soa4Bool(_mm_cmpneq_ps(vec0.get128(), vec1.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator & (const Soa4Bool & vec0, const Soa4Bool & vec1)
    {
        return Soa4Bool(_mm_and_ps(vec0.get128(), vec1.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator ^ (const Soa4Bool & vec0, const Soa4Bool & vec1)
    {
        return Soa4Bool(_mm_xor_ps(vec0.get128(), vec1.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool operator | (const Soa4Bool & vec0, const Soa4Bool & vec1)
    {
        return Soa4Bool(_mm_or_ps(vec0.get128(), vec1.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa4Bool select(const Soa4Bool & vec0, const Soa4Bool & vec1, const Soa4Bool &select_vec1)
    {
        return Soa4Bool(simd_select_ps(vec0.get128(), vec1.get128(), select_vec1.get128()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const bool all(const Soa4Bool & vec)
    {
        return (_mm_movemask_ps(vec.get128()) == 0xf);
    }

    SIMD_VECTORMATH_FORCE_INLINE const bool any(const Soa4Bool & vec)
    {
        return (_mm_movemask_ps(vec.get128()) != 0);
    }

    SIMD_VECTORMATH_FORCE_INLINE void print(const Soa4Bool & vec)
    {
        printf("%i %i %i %i\n", (int)vec.getSlice(0), (int)vec.getSlice(1), (int)vec.getSlice(2), (int)vec.getSlice(3));
    }

    SIMD_VECTORMATH_FORCE_INLINE void print(const Soa4Bool & vec, const char* name)
    {
        printf("%s: %i %i %i %i\n", name, (int)vec.getSlice(0), (int)vec.getSlice(1), (int)vec.getSlice(2), (int)vec.getSlice(3));
    }

#if SIMD_UTILS_USE_AVX
    //--------------------------------------------------------------------------------------------------
    // Soa8Bool implementation
    //

    SIMD_VECTORMATH_FORCE_INLINE Soa8Bool::Soa8Bool(const Soa8Bool & other)
    {
        mData = other.mData;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Bool::Soa8Bool(__m256 vec)
    {
        mData = vec;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Bool::Soa8Bool(__m256i vec)
    {
        union { __m256 vf; __m256i vi; } vfiunion;
        vfiunion.vi = vec;
        mData = vfiunion.vf;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Bool::Soa8Bool(bool scalar)
    {
        union { float f; int i; } fiunion;
        fiunion.i = -(int)scalar;
        mData = _mm256_set1_ps(fiunion.f);
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Bool::Soa8Bool(
            bool scalar0, bool scalar1, bool scalar2, bool scalar3,
            bool scalar4, bool scalar5, bool scalar6, bool scalar7)
    {
        Simd256Union tmp;
        tmp.vi.m256i_i32[0] = -((int)scalar0);
        tmp.vi.m256i_i32[1] = -((int)scalar1);
        tmp.vi.m256i_i32[2] = -((int)scalar2);
        tmp.vi.m256i_i32[3] = -((int)scalar3);
        tmp.vi.m256i_i32[4] = -((int)scalar4);
        tmp.vi.m256i_i32[5] = -((int)scalar5);
        tmp.vi.m256i_i32[6] = -((int)scalar6);
        tmp.vi.m256i_i32[7] = -((int)scalar7);
        mData = tmp.vf;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Bool::Soa8Bool(const Soa8Float & vec)
    {
        *this = (vec != Soa8Float(0.0f));
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Bool::Soa8Bool(const Soa8Int & vec)
    {
        *this = (vec != Soa8Int(0));
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Bool::Soa8Bool(const Soa8Uint & vec)
    {
        *this = (vec != Soa8Uint((uint32_t)0));
    }

    SIMD_VECTORMATH_FORCE_INLINE __m256 Soa8Bool::get256() const
    {
        return mData;
    }

    SIMD_VECTORMATH_FORCE_INLINE __m256i Soa8Bool::get256i() const
    {
        union { __m256 vf; __m256i vi; } vfiunion;
        vfiunion.vf = mData;
        return vfiunion.vi;
    }

    SIMD_VECTORMATH_FORCE_INLINE bool Soa8Bool::getSlice(uint32_t idx) const
    {
        Simd256Union tmp;
        tmp.vf = mData;
        return (bool)(tmp.vi.m256i_i32[idx] != 0);
    }

    SIMD_VECTORMATH_FORCE_INLINE void Soa8Bool::setSlice(uint32_t idx, bool value)
    {
        Simd256Union tmp;
        tmp.vf = mData;
        tmp.vi.m256i_i32[idx] = -(int)value;
        mData = tmp.vf;
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool Soa8Bool::operator ! () const
    {
        return Soa8Bool(_mm256_andnot_ps(mData, _mm256_cmp_ps(_mm256_setzero_ps(), _mm256_setzero_ps(), _CMP_EQ_OQ)));
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Bool& Soa8Bool::operator = (const Soa8Bool & vec)
    {
        mData = vec.mData;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Bool& Soa8Bool::operator &= (const Soa8Bool & vec)
    {
        *this = *this & vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Bool& Soa8Bool::operator ^= (const Soa8Bool & vec)
    {
        *this = *this ^ vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE Soa8Bool& Soa8Bool::operator |= (const Soa8Bool & vec)
    {
        *this = *this | vec;
        return *this;
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator == (const Soa8Bool & vec0, const Soa8Bool & vec1)
    {
        return Soa8Bool(_mm256_cmp_ps(vec0.get256(), vec1.get256(), _CMP_EQ_OQ));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator != (const Soa8Bool & vec0, const Soa8Bool & vec1)
    {
        return Soa8Bool(_mm256_cmp_ps(vec0.get256(), vec1.get256(), _CMP_NEQ_OQ));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator & (const Soa8Bool & vec0, const Soa8Bool & vec1)
    {
        return Soa8Bool(_mm256_and_ps(vec0.get256(), vec1.get256()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator | (const Soa8Bool & vec0, const Soa8Bool & vec1)
    {
        return Soa8Bool(_mm256_or_ps(vec0.get256(), vec1.get256()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool operator ^ (const Soa8Bool & vec0, const Soa8Bool & vec1)
    {
        return Soa8Bool(_mm256_xor_ps(vec0.get256(), vec1.get256()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const Soa8Bool select(const Soa8Bool & vec0, const Soa8Bool & vec1, const Soa8Bool &select_vec1)
    {
        return Soa8Bool(simd_select_ps(vec0.get256(), vec1.get256(), select_vec1.get256()));
    }

    SIMD_VECTORMATH_FORCE_INLINE const bool all(const Soa8Bool & vec)
    {
        return (_mm256_movemask_ps(vec.get256()) == 0xff);
    }

    SIMD_VECTORMATH_FORCE_INLINE const bool any(const Soa8Bool & vec)
    {
        return (_mm256_movemask_ps(vec.get256()) != 0);
    }
#endif // SIMD_UTILS_USE_AVX

} // namespace FmVectormath

#endif // _SOA_BOOL_H
