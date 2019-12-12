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

#ifndef _VECTORMATH_UTILS_H_
#define _VECTORMATH_UTILS_H_

// Adapted from SSE version of Sony Vectormath file "vectormath_aos.h".
// Separating some of the SIMD operations for sharing between classes.
// Other operations and settings in simd_utils.h

#include "simd_utils.h"
#include <assert.h>

#define SIMD_VECTORMATH_ALIGN16      SIMD_UTILS_ALIGN16
#define SIMD_VECTORMATH_FORCE_INLINE SIMD_UTILS_FORCE_INLINE

union SSEFloat
{
    __m128i vi;
    __m128 m128;
    __m128 vf;
    uint32_t ui[4];
    uint16_t s[8];
    float f[4];
    SSEFloat(__m128 v) : m128(v) {}
    SSEFloat(__m128i v) : vi(v) {}
    SSEFloat() {}//uninitialized
};

#define simd_ror_ps(vec,i)    \
    (((i)%4) ? (_mm_shuffle_ps(vec,vec, _MM_SHUFFLE((uint8_t)(i+3)%4,(uint8_t)(i+2)%4,(uint8_t)(i+1)%4,(uint8_t)(i+0)%4))) : (vec))
#define simd_rol_ps(vec,i)    \
    (((i)%4) ? (_mm_shuffle_ps(vec,vec, _MM_SHUFFLE((uint8_t)(7-i)%4,(uint8_t)(6-i)%4,(uint8_t)(5-i)%4,(uint8_t)(4-i)%4))) : (vec))

static SIMD_VECTORMATH_FORCE_INLINE __m128 simd_cts_ps(__m128 x, int a)
{
    assert(a == 0); // Only 2^0 supported
    (void)a;
    __m128i result = _mm_cvtps_epi32(x);
    return (__m128 &)result;
}

static SIMD_VECTORMATH_FORCE_INLINE __m128 simd_ctf_ps(__m128 x, int a)
{
    assert(a == 0); // Only 2^0 supported
    (void)a;
    return _mm_cvtepi32_ps((__m128i &)x);
}

#if SIMD_UTILS_USE_AVX
static SIMD_UTILS_FORCE_INLINE __m256 simd_cts_ps(__m256 x, int a)
{
    assert(a == 0); // Only 2^0 supported
    (void)a;
    __m256i result = _mm256_cvtps_epi32(x);
    return (__m256 &)result;
}

static SIMD_UTILS_FORCE_INLINE __m256 simd_ctf_ps(__m256 x, int a)
{
    assert(a == 0); // Only 2^0 supported
    (void)a;
    return _mm256_cvtepi32_ps((__m256i &)x);
}
#endif

static SIMD_VECTORMATH_FORCE_INLINE __m128 simd_rsqrt_newtonraphson_ps(const __m128 v)
{
#define _half4 _mm_setr_ps(.5f,.5f,.5f,.5f) 
#define _three _mm_setr_ps(3.f,3.f,3.f,3.f)
    const __m128 approx = _mm_rsqrt_ps(v);
    const __m128 muls = _mm_mul_ps(_mm_mul_ps(v, approx), approx);
    return _mm_mul_ps(_mm_mul_ps(_half4, approx), _mm_sub_ps(_three, muls));
}

static SIMD_VECTORMATH_FORCE_INLINE __m128 simd_acos_ps(__m128 x)
{
    __m128 xabs = simd_abs_ps(x);
    __m128 select = _mm_cmplt_ps(x, _mm_setzero_ps());
    __m128 t1 = _mm_sqrt_ps(_mm_sub_ps(_mm_set1_ps(1.0f), xabs));

    /* Instruction counts can be reduced if the polynomial was
    * computed entirely from nested (dependent) fma's. However,
    * to reduce the number of pipeline stalls, the polygon is evaluated
    * in two halves (hi amd lo).
    */
    __m128 xabs2 = _mm_mul_ps(xabs, xabs);
    __m128 xabs4 = _mm_mul_ps(xabs2, xabs2);
    __m128 hi = simd_madd_ps(simd_madd_ps(simd_madd_ps(_mm_set1_ps(-0.0012624911f),
        xabs, _mm_set1_ps(0.0066700901f)),
        xabs, _mm_set1_ps(-0.0170881256f)),
        xabs, _mm_set1_ps(0.0308918810f));
    __m128 lo = simd_madd_ps(simd_madd_ps(simd_madd_ps(_mm_set1_ps(-0.0501743046f),
        xabs, _mm_set1_ps(0.0889789874f)),
        xabs, _mm_set1_ps(-0.2145988016f)),
        xabs, _mm_set1_ps(1.5707963050f));

    __m128 result = simd_madd_ps(hi, xabs4, lo);

    // Adjust the result if x is negactive.
    return simd_select_ps(
        _mm_mul_ps(t1, result),                                    // Positive
        simd_fnmadd_ps(t1, result, _mm_set1_ps(3.1415926535898f)),    // Negative
        select);
}

#if SIMD_UTILS_USE_AVX
static SIMD_UTILS_FORCE_INLINE __m256 simd_acos_ps(__m256 x)
{
    __m256 xabs = simd_abs_ps(x);
    __m256 select = _mm256_cmp_ps(x, _mm256_setzero_ps(), _CMP_LT_OQ);
    __m256 t1 = _mm256_sqrt_ps(_mm256_sub_ps(_mm256_set1_ps(1.0f), xabs));

    /* Instruction counts can be reduced if the polynomial was
    * computed entirely from nested (dependent) fma's. However,
    * to reduce the number of pipeline stalls, the polygon is evaluated
    * in two halves (hi amd lo).
    */
    __m256 xabs2 = _mm256_mul_ps(xabs, xabs);
    __m256 xabs4 = _mm256_mul_ps(xabs2, xabs2);
    __m256 hi = simd_madd_ps(simd_madd_ps(simd_madd_ps(_mm256_set1_ps(-0.0012624911f),
        xabs, _mm256_set1_ps(0.0066700901f)),
        xabs, _mm256_set1_ps(-0.0170881256f)),
        xabs, _mm256_set1_ps(0.0308918810f));
    __m256 lo = simd_madd_ps(simd_madd_ps(simd_madd_ps(_mm256_set1_ps(-0.0501743046f),
        xabs, _mm256_set1_ps(0.0889789874f)),
        xabs, _mm256_set1_ps(-0.2145988016f)),
        xabs, _mm256_set1_ps(1.5707963050f));

    __m256 result = simd_madd_ps(hi, xabs4, lo);

    // Adjust the result if x is negactive.
    return simd_select_ps(
        _mm256_mul_ps(t1, result),									// Positive
        simd_fnmadd_ps(t1, result, _mm256_set1_ps(3.1415926535898f)),	// Negative
        select);
}
#endif

static SIMD_VECTORMATH_FORCE_INLINE __m128 simd_sin_ps(__m128 x)
{

    //
    // Common constants used to evaluate simd_sin_ps/cosf4/tanf4
    //
#define _SINCOS_CC0  -0.0013602249f
#define _SINCOS_CC1   0.0416566950f
#define _SINCOS_CC2  -0.4999990225f
#define _SINCOS_SC0  -0.0001950727f
#define _SINCOS_SC1   0.0083320758f
#define _SINCOS_SC2  -0.1666665247f

#define _SINCOS_KC1  1.57079625129f
#define _SINCOS_KC2  7.54978995489e-8f

    __m128 xl, xl2, xl3, res;

    // Range reduction using : xl = angle * TwoOverPi;
    //  
    xl = _mm_mul_ps(x, _mm_set1_ps(0.63661977236f));

    // Find the quadrant the angle falls in
    // using:  q = (int) (ceil(abs(xl))*sign(xl))
    //
    __m128 q = simd_cts_ps(xl, 0);

    // Compute an offset based on the quadrant that the angle falls in
    // 
    __m128 offset = _mm_and_ps(q, simd_broadcast_ui_ps(0x3));

    // Remainder in range [-pi/4..pi/4]
    //
    __m128 qf = simd_ctf_ps(q, 0);
    xl = simd_fnmadd_ps(qf, _mm_set1_ps(_SINCOS_KC2), simd_fnmadd_ps(qf, _mm_set1_ps(_SINCOS_KC1), x));

    // Compute x^2 and x^3
    //
    xl2 = _mm_mul_ps(xl, xl);
    xl3 = _mm_mul_ps(xl2, xl);

    // Compute both the sin and cos of the angles
    // using a polynomial expression:
    //   cx = 1.0f + xl2 * ((C0 * xl2 + C1) * xl2 + C2), and
    //   sx = xl + xl3 * ((S0 * xl2 + S1) * xl2 + S2)
    //

    __m128 cx =
        simd_madd_ps(
            simd_madd_ps(
                simd_madd_ps(_mm_set1_ps(_SINCOS_CC0), xl2, _mm_set1_ps(_SINCOS_CC1)), xl2, _mm_set1_ps(_SINCOS_CC2)), xl2, _mm_set1_ps(1.0f));
    __m128 sx =
        simd_madd_ps(
            simd_madd_ps(
                simd_madd_ps(_mm_set1_ps(_SINCOS_SC0), xl2, _mm_set1_ps(_SINCOS_SC1)), xl2, _mm_set1_ps(_SINCOS_SC2)), xl3, xl);

    // Use the cosine when the offset is odd and the sin
    // when the offset is even
    //
    res = simd_select_ps(cx, sx, _mm_cmpeq_ps(_mm_and_ps(offset,
        simd_broadcast_ui_ps(0x1)),
        _mm_setzero_ps()));

    // Flip the sign of the result when (offset mod 4) = 1 or 2
    //
    return simd_select_ps(
        _mm_xor_ps(simd_broadcast_ui_ps(0x80000000U), res),    // Negative
        res,                                // Positive
        _mm_cmpeq_ps(_mm_and_ps(offset, simd_broadcast_ui_ps(0x2)), _mm_setzero_ps()));
}

#if SIMD_UTILS_USE_AVX
static SIMD_UTILS_FORCE_INLINE __m256 simd_sin_ps(__m256 x)
{

    //
    // Common constants used to evaluate simd_sin_ps/cosf4/tanf4
    //
#define _SINCOS_CC0  -0.0013602249f
#define _SINCOS_CC1   0.0416566950f
#define _SINCOS_CC2  -0.4999990225f
#define _SINCOS_SC0  -0.0001950727f
#define _SINCOS_SC1   0.0083320758f
#define _SINCOS_SC2  -0.1666665247f

#define _SINCOS_KC1  1.57079625129f
#define _SINCOS_KC2  7.54978995489e-8f

    __m256 xl, xl2, xl3, res;

    // Range reduction using : xl = angle * TwoOverPi;
    //  
    xl = _mm256_mul_ps(x, _mm256_set1_ps(0.63661977236f));

    // Find the quadrant the angle falls in
    // using:  q = (int) (ceil(abs(xl))*sign(xl))
    //
    __m256 q = simd_cts_ps(xl, 0);

    // Compute an offset based on the quadrant that the angle falls in
    // 
    __m256 offset = _mm256_and_ps(q, simd256_broadcast_ui_ps(0x3));

    // Remainder in range [-pi/4..pi/4]
    //
    __m256 qf = simd_ctf_ps(q, 0);
    xl = simd_fnmadd_ps(qf, _mm256_set1_ps(_SINCOS_KC2), simd_fnmadd_ps(qf, _mm256_set1_ps(_SINCOS_KC1), x));

    // Compute x^2 and x^3
    //
    xl2 = _mm256_mul_ps(xl, xl);
    xl3 = _mm256_mul_ps(xl2, xl);

    // Compute both the sin and cos of the angles
    // using a polynomial expression:
    //   cx = 1.0f + xl2 * ((C0 * xl2 + C1) * xl2 + C2), and
    //   sx = xl + xl3 * ((S0 * xl2 + S1) * xl2 + S2)
    //

    __m256 cx =
        simd_madd_ps(
            simd_madd_ps(
                simd_madd_ps(_mm256_set1_ps(_SINCOS_CC0), xl2, _mm256_set1_ps(_SINCOS_CC1)), xl2, _mm256_set1_ps(_SINCOS_CC2)), xl2, _mm256_set1_ps(1.0f));
    __m256 sx =
        simd_madd_ps(
            simd_madd_ps(
                simd_madd_ps(_mm256_set1_ps(_SINCOS_SC0), xl2, _mm256_set1_ps(_SINCOS_SC1)), xl2, _mm256_set1_ps(_SINCOS_SC2)), xl3, xl);

    // Use the cosine when the offset is odd and the sin
    // when the offset is even
    //
    res = simd_select_ps(cx, sx, _mm256_cmp_ps(_mm256_and_ps(offset,
        simd256_broadcast_ui_ps(0x1)),
        _mm256_setzero_ps(), _CMP_EQ_OQ));

    // Flip the sign of the result when (offset mod 4) = 1 or 2
    //
    return simd_select_ps(
        _mm256_xor_ps(simd256_broadcast_ui_ps(0x80000000U), res),	// Negative
        res,								// Positive
        _mm256_cmp_ps(_mm256_and_ps(offset, simd256_broadcast_ui_ps(0x2)), _mm256_setzero_ps(), _CMP_EQ_OQ));
}
#endif

static SIMD_VECTORMATH_FORCE_INLINE void simd_sincos_ps(__m128 x, __m128* s, __m128* c)
{
    __m128 xl, xl2, xl3;
    __m128   offsetSin, offsetCos;

    // Range reduction using : xl = angle * TwoOverPi;
    //  
    xl = _mm_mul_ps(x, _mm_set1_ps(0.63661977236f));

    // Find the quadrant the angle falls in
    // using:  q = (int) (ceil(abs(xl))*sign(xl))
    //
    //__m128 q = simd_cts_ps(_mm_add_ps(xl,simd_select_ps(_mm_set1_ps(0.5f),xl,(0x80000000))),0);
    __m128 q = simd_cts_ps(xl, 0);

    // Compute the offset based on the quadrant that the angle falls in.
    // Add 1 to the offset for the cosine. 
    //
    offsetSin = _mm_and_ps(q, simd_broadcast_ui_ps((int)0x3));
    __m128i temp = _mm_add_epi32(_mm_set1_epi32(1), (__m128i &)offsetSin);
    offsetCos = (__m128 &)temp;

    // Remainder in range [-pi/4..pi/4]
    //
    __m128 qf = simd_ctf_ps(q, 0);
    xl = simd_fnmadd_ps(qf, _mm_set1_ps(_SINCOS_KC2), simd_fnmadd_ps(qf, _mm_set1_ps(_SINCOS_KC1), x));

    // Compute x^2 and x^3
    //
    xl2 = _mm_mul_ps(xl, xl);
    xl3 = _mm_mul_ps(xl2, xl);

    // Compute both the sin and cos of the angles
    // using a polynomial expression:
    //   cx = 1.0f + xl2 * ((C0 * xl2 + C1) * xl2 + C2), and
    //   sx = xl + xl3 * ((S0 * xl2 + S1) * xl2 + S2)
    //
    __m128 cx =
        simd_madd_ps(
            simd_madd_ps(
                simd_madd_ps(_mm_set1_ps(_SINCOS_CC0), xl2, _mm_set1_ps(_SINCOS_CC1)), xl2, _mm_set1_ps(_SINCOS_CC2)), xl2, _mm_set1_ps(1.0f));
    __m128 sx =
        simd_madd_ps(
            simd_madd_ps(
                simd_madd_ps(_mm_set1_ps(_SINCOS_SC0), xl2, _mm_set1_ps(_SINCOS_SC1)), xl2, _mm_set1_ps(_SINCOS_SC2)), xl3, xl);

    // Use the cosine when the offset is odd and the sin
    // when the offset is even
    //
    __m128 sinMask = (__m128)_mm_cmpeq_ps(_mm_and_ps(offsetSin, simd_broadcast_ui_ps(0x1)), _mm_setzero_ps());
    __m128 cosMask = (__m128)_mm_cmpeq_ps(_mm_and_ps(offsetCos, simd_broadcast_ui_ps(0x1)), _mm_setzero_ps());
    *s = simd_select_ps(cx, sx, sinMask);
    *c = simd_select_ps(cx, sx, cosMask);

    // Flip the sign of the result when (offset mod 4) = 1 or 2
    //
    sinMask = _mm_cmpeq_ps(_mm_and_ps(offsetSin, simd_broadcast_ui_ps(0x2)), _mm_setzero_ps());
    cosMask = _mm_cmpeq_ps(_mm_and_ps(offsetCos, simd_broadcast_ui_ps(0x2)), _mm_setzero_ps());

    *s = simd_select_ps((__m128)_mm_xor_ps(simd_broadcast_ui_ps(0x80000000), (__m128)*s), *s, sinMask);
    *c = simd_select_ps((__m128)_mm_xor_ps(simd_broadcast_ui_ps(0x80000000), (__m128)*c), *c, cosMask);
}

#if SIMD_UTILS_USE_AVX
static SIMD_UTILS_FORCE_INLINE void simd_sincos_ps(__m256 x, __m256* s, __m256* c)
{
    __m256 xl, xl2, xl3;
    __m256   offsetSin, offsetCos;

    // Range reduction using : xl = angle * TwoOverPi;
    //  
    xl = _mm256_mul_ps(x, _mm256_set1_ps(0.63661977236f));

    // Find the quadrant the angle falls in
    // using:  q = (int) (ceil(abs(xl))*sign(xl))
    //
    //__m256 q = simd_cts_ps(_mm256_add_ps(xl,simd_select_ps(_mm256_set1_ps(0.5f),xl,(0x80000000))),0);
    __m256 q = simd_cts_ps(xl, 0);

    // Compute the offset based on the quadrant that the angle falls in.
    // Add 1 to the offset for the cosine. 
    //
    offsetSin = _mm256_and_ps(q, simd256_broadcast_ui_ps((int)0x3));
    __m256i temp = _mm256_add_epi32(_mm256_set1_epi32(1), (__m256i &)offsetSin);
    offsetCos = (__m256 &)temp;

    // Remainder in range [-pi/4..pi/4]
    //
    __m256 qf = simd_ctf_ps(q, 0);
    xl = simd_fnmadd_ps(qf, _mm256_set1_ps(_SINCOS_KC2), simd_fnmadd_ps(qf, _mm256_set1_ps(_SINCOS_KC1), x));

    // Compute x^2 and x^3
    //
    xl2 = _mm256_mul_ps(xl, xl);
    xl3 = _mm256_mul_ps(xl2, xl);

    // Compute both the sin and cos of the angles
    // using a polynomial expression:
    //   cx = 1.0f + xl2 * ((C0 * xl2 + C1) * xl2 + C2), and
    //   sx = xl + xl3 * ((S0 * xl2 + S1) * xl2 + S2)
    //
    __m256 cx =
        simd_madd_ps(
            simd_madd_ps(
                simd_madd_ps(_mm256_set1_ps(_SINCOS_CC0), xl2, _mm256_set1_ps(_SINCOS_CC1)), xl2, _mm256_set1_ps(_SINCOS_CC2)), xl2, _mm256_set1_ps(1.0f));
    __m256 sx =
        simd_madd_ps(
            simd_madd_ps(
                simd_madd_ps(_mm256_set1_ps(_SINCOS_SC0), xl2, _mm256_set1_ps(_SINCOS_SC1)), xl2, _mm256_set1_ps(_SINCOS_SC2)), xl3, xl);

    // Use the cosine when the offset is odd and the sin
    // when the offset is even
    //
    __m256 sinMask = (__m256)_mm256_cmp_ps(_mm256_and_ps(offsetSin, simd256_broadcast_ui_ps(0x1)), _mm256_setzero_ps(), _CMP_EQ_OQ);
    __m256 cosMask = (__m256)_mm256_cmp_ps(_mm256_and_ps(offsetCos, simd256_broadcast_ui_ps(0x1)), _mm256_setzero_ps(), _CMP_EQ_OQ);
    *s = simd_select_ps(cx, sx, sinMask);
    *c = simd_select_ps(cx, sx, cosMask);

    // Flip the sign of the result when (offset mod 4) = 1 or 2
    //
    sinMask = _mm256_cmp_ps(_mm256_and_ps(offsetSin, simd256_broadcast_ui_ps(0x2)), _mm256_setzero_ps(), _CMP_EQ_OQ);
    cosMask = _mm256_cmp_ps(_mm256_and_ps(offsetCos, simd256_broadcast_ui_ps(0x2)), _mm256_setzero_ps(), _CMP_EQ_OQ);

    *s = simd_select_ps((__m256)_mm256_xor_ps(simd256_broadcast_ui_ps(0x80000000), (__m256)*s), *s, sinMask);
    *c = simd_select_ps((__m256)_mm256_xor_ps(simd256_broadcast_ui_ps(0x80000000), (__m256)*c), *c, cosMask);
}
#endif

#endif
