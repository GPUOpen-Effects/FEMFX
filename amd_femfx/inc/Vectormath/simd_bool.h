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

#ifndef _SIMD_BOOL_H
#define _SIMD_BOOL_H

#include <math.h>

namespace FmVectormath {

class SimdFloat;

//--------------------------------------------------------------------------------------------------
// SimdBool class
//

class SimdBool
{
    private:
        __m128 mData;

        SIMD_VECTORMATH_FORCE_INLINE SimdBool(__m128 vec);
    public:
        SIMD_VECTORMATH_FORCE_INLINE SimdBool() {}

        SIMD_VECTORMATH_FORCE_INLINE SimdBool(const SimdBool& other);

        // matches standard type conversions
        //
        SIMD_VECTORMATH_FORCE_INLINE SimdBool(const SimdFloat & vec);

        // explicit cast from bool
        //
        explicit SIMD_VECTORMATH_FORCE_INLINE SimdBool(bool scalar);

#ifdef _SIMD_VECTORMATH_NO_SCALAR_CAST
        // explicit cast to bool
        // 
        SIMD_VECTORMATH_FORCE_INLINE bool getAsBool() const;
#else
        // implicit cast to bool
        // 
        SIMD_VECTORMATH_FORCE_INLINE operator bool() const;
#endif
        
        // get vector data
        // bool value is splatted across all word slots of vector as 0 (false) or -1 (true)
        //
        SIMD_VECTORMATH_FORCE_INLINE __m128 get128() const;

        // operators
        //
        SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator ! () const;
        SIMD_VECTORMATH_FORCE_INLINE SimdBool& operator = (const SimdBool & vec);
        SIMD_VECTORMATH_FORCE_INLINE SimdBool& operator &= (const SimdBool & vec);
        SIMD_VECTORMATH_FORCE_INLINE SimdBool& operator ^= (const SimdBool & vec);
        SIMD_VECTORMATH_FORCE_INLINE SimdBool& operator |= (const SimdBool & vec);

        // friend functions
        //
        friend SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator == (const SimdBool & vec0, const SimdBool & vec1);
        friend SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator != (const SimdBool & vec0, const SimdBool & vec1);
        friend SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator < (const SimdFloat & vec0, const SimdFloat & vec1);
        friend SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator <= (const SimdFloat & vec0, const SimdFloat & vec1);
        friend SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator > (const SimdFloat & vec0, const SimdFloat & vec1);
        friend SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator >= (const SimdFloat & vec0, const SimdFloat & vec1);
        friend SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator == (const SimdFloat & vec0, const SimdFloat & vec1);
        friend SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator != (const SimdFloat & vec0, const SimdFloat & vec1);
        friend SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator & (const SimdBool & vec0, const SimdBool & vec1);
        friend SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator ^ (const SimdBool & vec0, const SimdBool & vec1);
        friend SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator | (const SimdBool & vec0, const SimdBool & vec1);
        friend SIMD_VECTORMATH_FORCE_INLINE const SimdBool select(const SimdBool & vec0, const SimdBool & vec1, const SimdBool &select_vec1);
};

//--------------------------------------------------------------------------------------------------
// SimdBool functions
//

// operators
//
SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator == (const SimdBool & vec0, const SimdBool & vec1);
SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator != (const SimdBool & vec0, const SimdBool & vec1);
SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator & (const SimdBool & vec0, const SimdBool & vec1);
SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator ^ (const SimdBool & vec0, const SimdBool & vec1);
SIMD_VECTORMATH_FORCE_INLINE const SimdBool operator | (const SimdBool & vec0, const SimdBool & vec1);

// select between vec0 and vec1 using SimdBool.
// false selects vec0, true selects vec1
//
SIMD_VECTORMATH_FORCE_INLINE const SimdBool select(const SimdBool & vec0, const SimdBool & vec1, const SimdBool &select_vec1);

} // namespace FmVectormath

//--------------------------------------------------------------------------------------------------
// SimdBool implementation
//

#include "simd_float.h"

namespace FmVectormath {

SIMD_VECTORMATH_FORCE_INLINE
SimdBool::SimdBool(__m128 vec)
{
    mData = vec;
}

SIMD_VECTORMATH_FORCE_INLINE SimdBool::SimdBool(const SimdBool & other)
{
    mData = other.mData;
}

SIMD_VECTORMATH_FORCE_INLINE
SimdBool::SimdBool(const SimdFloat & vec)
{
    *this = (vec != SimdFloat(0.0f));
}

union SimdBool_converter
{
    __m128 m128;
    bool b;
    float f;
    int i;
    SimdBool_converter(int i) : i(i) {}
    SimdBool_converter(__m128 m128) : m128(m128) {}
    SimdBool_converter() {}
};

SIMD_VECTORMATH_FORCE_INLINE
SimdBool::SimdBool(bool scalar)
{
    mData = _mm_set1_ps(SimdBool_converter(-(int)scalar).f);
}

#ifdef _SIMD_VECTORMATH_NO_SCALAR_CAST
SIMD_VECTORMATH_FORCE_INLINE
bool
SimdBool::getAsBool() const
#else
SIMD_VECTORMATH_FORCE_INLINE
SimdBool::operator bool() const
#endif
{
    return SimdBool_converter(mData).b;
}

SIMD_VECTORMATH_FORCE_INLINE
__m128
SimdBool::get128() const
{
    return mData;
}

SIMD_VECTORMATH_FORCE_INLINE
const SimdBool
SimdBool::operator ! () const
{
    return SimdBool(_mm_andnot_ps(mData, _mm_cmpeq_ps(_mm_setzero_ps(),_mm_setzero_ps())));
}

SIMD_VECTORMATH_FORCE_INLINE
SimdBool&
SimdBool::operator = (const SimdBool & vec)
{
    mData = vec.mData;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE
SimdBool&
SimdBool::operator &= (const SimdBool & vec)
{
    *this = *this & vec;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE
SimdBool&
SimdBool::operator ^= (const SimdBool & vec)
{
    *this = *this ^ vec;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE
SimdBool&
SimdBool::operator |= (const SimdBool & vec)
{
    *this = *this | vec;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE
const SimdBool
operator == (const SimdBool & vec0, const SimdBool & vec1)
{
    return SimdBool(_mm_cmpeq_ps(vec0.get128(), vec1.get128()));
}

SIMD_VECTORMATH_FORCE_INLINE
const SimdBool
operator != (const SimdBool & vec0, const SimdBool & vec1)
{
    return SimdBool(_mm_cmpneq_ps(vec0.get128(), vec1.get128()));
}
    
SIMD_VECTORMATH_FORCE_INLINE
const SimdBool
operator & (const SimdBool & vec0, const SimdBool & vec1)
{
    return SimdBool(_mm_and_ps(vec0.get128(), vec1.get128()));
}

SIMD_VECTORMATH_FORCE_INLINE
const SimdBool
operator | (const SimdBool & vec0, const SimdBool & vec1)
{
    return SimdBool(_mm_or_ps(vec0.get128(), vec1.get128()));
}

SIMD_VECTORMATH_FORCE_INLINE
const SimdBool
operator ^ (const SimdBool & vec0, const SimdBool & vec1)
{
    return SimdBool(_mm_xor_ps(vec0.get128(), vec1.get128()));
}

SIMD_VECTORMATH_FORCE_INLINE
const SimdBool
select(const SimdBool & vec0, const SimdBool & vec1, const SimdBool &select_vec1)
{
    return SimdBool(simd_select_ps(vec0.get128(), vec1.get128(), select_vec1.get128()));
}
 
} // namespace FmVectormath

#endif // SimdBool_h
