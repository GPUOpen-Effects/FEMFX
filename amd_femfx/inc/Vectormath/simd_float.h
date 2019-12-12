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

#ifndef _SIMD_FLOAT_H
#define _SIMD_FLOAT_H

#include <math.h>
#include <xmmintrin.h>

namespace FmVectormath {

class SimdBool;

//--------------------------------------------------------------------------------------------------
// SimdFloat class
//

class SimdFloat
{
    private:
        __m128 mData;

    public:
        SIMD_VECTORMATH_FORCE_INLINE SimdFloat(__m128 vec);

        SIMD_VECTORMATH_FORCE_INLINE SimdFloat() {}

        SIMD_VECTORMATH_FORCE_INLINE SimdFloat(const SimdFloat& other);

        // matches standard type conversions
        //
        SIMD_VECTORMATH_FORCE_INLINE SimdFloat(const SimdBool & vec);

        // construct from a slot of __m128
        //
        SIMD_VECTORMATH_FORCE_INLINE SimdFloat(__m128 vec, int slot);
        
        // explicit cast from float
        //
        explicit SIMD_VECTORMATH_FORCE_INLINE SimdFloat(float scalar);

#ifdef _SIMD_VECTORMATH_NO_SCALAR_CAST
        // explicit cast to float
        // 
        SIMD_VECTORMATH_FORCE_INLINE float getAsFloat() const;
#else
        // implicit cast to float
        //
        SIMD_VECTORMATH_FORCE_INLINE operator float() const;
#endif

        // get vector data
        // float value is splatted across all word slots of vector
        //
        SIMD_VECTORMATH_FORCE_INLINE __m128 get128() const;

        // operators
        // 
        SIMD_VECTORMATH_FORCE_INLINE const SimdFloat operator ++ (int);
        SIMD_VECTORMATH_FORCE_INLINE const SimdFloat operator -- (int);
        SIMD_VECTORMATH_FORCE_INLINE SimdFloat& operator ++ ();
        SIMD_VECTORMATH_FORCE_INLINE SimdFloat& operator -- ();
        SIMD_VECTORMATH_FORCE_INLINE const SimdFloat operator - () const;
        SIMD_VECTORMATH_FORCE_INLINE SimdFloat& operator = (const SimdFloat & vec);
        SIMD_VECTORMATH_FORCE_INLINE SimdFloat& operator *= (const SimdFloat & vec);
        SIMD_VECTORMATH_FORCE_INLINE SimdFloat& operator /= (const SimdFloat & vec);
        SIMD_VECTORMATH_FORCE_INLINE SimdFloat& operator += (const SimdFloat & vec);
        SIMD_VECTORMATH_FORCE_INLINE SimdFloat& operator -= (const SimdFloat & vec);

        // friend functions
        //
        friend SIMD_VECTORMATH_FORCE_INLINE const SimdFloat operator * (const SimdFloat & vec0, const SimdFloat & vec1);
        friend SIMD_VECTORMATH_FORCE_INLINE const SimdFloat operator / (const SimdFloat & vec0, const SimdFloat & vec1);
        friend SIMD_VECTORMATH_FORCE_INLINE const SimdFloat operator + (const SimdFloat & vec0, const SimdFloat & vec1);
        friend SIMD_VECTORMATH_FORCE_INLINE const SimdFloat operator - (const SimdFloat & vec0, const SimdFloat & vec1);
        friend SIMD_VECTORMATH_FORCE_INLINE const SimdFloat select(const SimdFloat & vec0, const SimdFloat & vec1, SimdBool select_vec1);
};

//--------------------------------------------------------------------------------------------------
// SimdFloat functions
//

// operators
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat operator * (const SimdFloat & vec0, const SimdFloat & vec1);
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat operator / (const SimdFloat & vec0, const SimdFloat & vec1);
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat operator + (const SimdFloat & vec0, const SimdFloat & vec1);
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat operator - (const SimdFloat & vec0, const SimdFloat & vec1);
SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator < (const SimdFloat & vec0, const SimdFloat & vec1);
SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator <= (const SimdFloat & vec0, const SimdFloat & vec1);
SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator > (const SimdFloat & vec0, const SimdFloat & vec1);
SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator >= (const SimdFloat & vec0, const SimdFloat & vec1);
SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator == (const SimdFloat & vec0, const SimdFloat & vec1);
SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator != (const SimdFloat & vec0, const SimdFloat & vec1);

// select between vec0 and vec1 using SimdBool.
// false selects vec0, true selects vec1
//
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat select(const SimdFloat & vec0, const SimdFloat & vec1, const SimdBool &select_vec1);

} // namespace FmVectormath

//--------------------------------------------------------------------------------------------------
// SimdFloat implementation
//

#include "simd_bool.h"

namespace FmVectormath {

SIMD_VECTORMATH_FORCE_INLINE
SimdFloat::SimdFloat(__m128 vec)
{
    mData = vec;
}

SIMD_VECTORMATH_FORCE_INLINE SimdFloat::SimdFloat(const SimdFloat & other)
{
    mData = other.mData;
}

SIMD_VECTORMATH_FORCE_INLINE
SimdFloat::SimdFloat(const SimdBool & vec)
{
    mData = simd_select_ps(_mm_setzero_ps(), _mm_set1_ps(1.0f), vec.get128());
}

SIMD_VECTORMATH_FORCE_INLINE
SimdFloat::SimdFloat(__m128 vec, int slot)
{
    mData = _mm_set1_ps(simd_getelem_ps(vec, slot));
}

SIMD_VECTORMATH_FORCE_INLINE
SimdFloat::SimdFloat(float scalar)
{
    mData = _mm_set1_ps(scalar);
}

#ifdef _SIMD_VECTORMATH_NO_SCALAR_CAST
SIMD_VECTORMATH_FORCE_INLINE
float
SimdFloat::getAsFloat() const
#else
SIMD_VECTORMATH_FORCE_INLINE
SimdFloat::operator float() const
#endif
{
    return simd_getx_ps(mData);
}

SIMD_VECTORMATH_FORCE_INLINE
__m128
SimdFloat::get128() const
{
    return mData;
}

SIMD_VECTORMATH_FORCE_INLINE
const SimdFloat
SimdFloat::operator ++ (int)
{
    __m128 olddata = mData;
    operator ++();
    return SimdFloat(olddata);
}

SIMD_VECTORMATH_FORCE_INLINE
const SimdFloat
SimdFloat::operator -- (int)
{
    __m128 olddata = mData;
    operator --();
    return SimdFloat(olddata);
}

SIMD_VECTORMATH_FORCE_INLINE
SimdFloat&
SimdFloat::operator ++ ()
{
    *this += SimdFloat(_mm_set1_ps(1.0f));
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE
SimdFloat&
SimdFloat::operator -- ()
{
    *this -= SimdFloat(_mm_set1_ps(1.0f));
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE
const SimdFloat
SimdFloat::operator - () const
{
    return SimdFloat(_mm_sub_ps(_mm_setzero_ps(), mData));
}

SIMD_VECTORMATH_FORCE_INLINE
SimdFloat&
SimdFloat::operator = (const SimdFloat & vec)
{
    mData = vec.mData;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE
SimdFloat&
SimdFloat::operator *= (const SimdFloat & vec)
{
    *this = *this * vec;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE
SimdFloat&
SimdFloat::operator /= (const SimdFloat & vec)
{
    *this = *this / vec;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE
SimdFloat&
SimdFloat::operator += (const SimdFloat & vec)
{
    *this = *this + vec;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE
SimdFloat&
SimdFloat::operator -= (const SimdFloat & vec)
{
    *this = *this - vec;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE
const SimdFloat
operator * (const SimdFloat & vec0, const SimdFloat & vec1)
{
    return SimdFloat(_mm_mul_ps(vec0.get128(), vec1.get128()));
}

SIMD_VECTORMATH_FORCE_INLINE
const SimdFloat
operator / (const SimdFloat &num, const SimdFloat &den)
{
    return SimdFloat(_mm_div_ps(num.get128(), den.get128()));
}

SIMD_VECTORMATH_FORCE_INLINE
const SimdFloat
operator + (const SimdFloat & vec0, const SimdFloat & vec1)
{
    return SimdFloat(_mm_add_ps(vec0.get128(), vec1.get128()));
}

SIMD_VECTORMATH_FORCE_INLINE
const SimdFloat
operator - (const SimdFloat & vec0, const SimdFloat & vec1)
{
    return SimdFloat(_mm_sub_ps(vec0.get128(), vec1.get128()));
}

SIMD_VECTORMATH_FORCE_INLINE
const SimdBool
operator < (const SimdFloat & vec0, const SimdFloat & vec1)
{
    return SimdBool(_mm_cmpgt_ps(vec1.get128(), vec0.get128()));
}

SIMD_VECTORMATH_FORCE_INLINE
const SimdBool
operator <= (const SimdFloat & vec0, const SimdFloat & vec1)
{
    return SimdBool(_mm_cmpge_ps(vec1.get128(), vec0.get128()));
}

SIMD_VECTORMATH_FORCE_INLINE
const SimdBool
operator > (const SimdFloat & vec0, const SimdFloat & vec1)
{
    return SimdBool(_mm_cmpgt_ps(vec0.get128(), vec1.get128()));
}

SIMD_VECTORMATH_FORCE_INLINE
const SimdBool
operator >= (const SimdFloat & vec0, const SimdFloat & vec1)
{
    return SimdBool(_mm_cmpge_ps(vec0.get128(), vec1.get128()));
}

SIMD_VECTORMATH_FORCE_INLINE
const SimdBool
operator == (const SimdFloat & vec0, const SimdFloat & vec1)
{
    return SimdBool(_mm_cmpeq_ps(vec0.get128(), vec1.get128()));
}

SIMD_VECTORMATH_FORCE_INLINE
const SimdBool
operator != (const SimdFloat & vec0, const SimdFloat & vec1)
{
    return SimdBool(_mm_cmpneq_ps(vec0.get128(), vec1.get128()));
}
    
SIMD_VECTORMATH_FORCE_INLINE
const SimdFloat
select(const SimdFloat & vec0, const SimdFloat & vec1, const SimdBool &select_vec1)
{
    return SimdFloat(simd_select_ps(vec0.get128(), vec1.get128(), select_vec1.get128()));
}

} // namespace FmVectormath

#endif // SimdFloat_h
