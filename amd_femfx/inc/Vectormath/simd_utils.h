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

#pragma once

#include <stdint.h>
#include <math.h>
#include <xmmintrin.h>
#include <emmintrin.h>
#include <immintrin.h>
#include <pmmintrin.h>

#define SIMD_UTILS_USE_SSE3 1
#define SIMD_UTILS_USE_SSE4 1
#define SIMD_UTILS_USE_FMA  1
#define SIMD_UTILS_USE_AVX  1

#define SIMD_UTILS_UNALIGNED_LOAD        1
#define SIMD_UTILS_USE_SSE3_LDDQU        (0 && SIMD_UTILS_USE_SSE3)
#define SIMD_UTILS_USE_SSE4_DP_PS        (0 && SIMD_UTILS_USE_SSE4)
#define SIMD_UTILS_USE_SSE4_MULLO_EPI32  (1 && SIMD_UTILS_USE_SSE4)

#define SIMD_UTILS_ALIGN16      __declspec(align(16))
#define SIMD_UTILS_FORCE_INLINE __forceinline

union Simd128Union
{
    __m128i vi;
    __m128  vf;
    int32_t i[4];
    float   f[4];
};

#if SIMD_UTILS_USE_AVX
union Simd256Union
{
    __m256i vi;
    __m256  vf;
    int32_t i[8];
    float   f[8];
};
#endif

#define simd_broadcast_ps(m, idx) _mm_shuffle_ps(m, m, _MM_SHUFFLE(idx, idx, idx, idx))

static SIMD_UTILS_FORCE_INLINE float simd_getx_ps(__m128 m)
{
    return _mm_cvtss_f32(m);
}

static SIMD_UTILS_FORCE_INLINE float simd_gety_ps(__m128 m)
{
    return _mm_cvtss_f32(simd_broadcast_ps(m, 1));
}

static SIMD_UTILS_FORCE_INLINE float simd_getz_ps(__m128 m)
{
    return _mm_cvtss_f32(simd_broadcast_ps(m, 2));
}

static SIMD_UTILS_FORCE_INLINE float simd_getw_ps(__m128 m)
{
    return _mm_cvtss_f32(simd_broadcast_ps(m, 3));
}

static SIMD_UTILS_FORCE_INLINE float simd_getelem_ps(__m128 m, int idx)
{
    return m.m128_f32[idx];
}

static SIMD_UTILS_FORCE_INLINE int32_t simd_getelem_epi32(__m128 m, int idx)
{
    return m.m128_i32[idx];
}

static SIMD_UTILS_FORCE_INLINE int32_t simd_getelem_epi32(__m128i m, int idx)
{
    return m.m128i_i32[idx];
}

#if SIMD_UTILS_USE_AVX
static SIMD_UTILS_FORCE_INLINE float simd_getelem_ps(__m256 m, int idx)
{
    return m.m256_f32[idx];
}

static SIMD_UTILS_FORCE_INLINE int32_t simd_getelem_epi32(__m256 m, int idx)
{
    Simd256Union tmp;
    tmp.vf = m;
    return tmp.i[idx];
}

static SIMD_UTILS_FORCE_INLINE int32_t simd_getelem_epi32(__m256i m, int idx)
{
    return m.m256i_i32[idx];
}
#endif

static SIMD_UTILS_FORCE_INLINE __m128 simd_insertx_ps(__m128 m, float f)
{
    return _mm_move_ss(m, _mm_set_ss(f));
}

static SIMD_UTILS_FORCE_INLINE __m128 simd_inserty_ps(__m128 m, float f)
{
    __m128 y_zw = _mm_move_ss(m, _mm_set_ss(f));
    return _mm_move_ss(_mm_shuffle_ps(y_zw, y_zw, _MM_SHUFFLE(3, 2, 0, 0)), m);
}

static SIMD_UTILS_FORCE_INLINE __m128 simd_insertz_ps(__m128 m, float f)
{
    __m128 xy_w = m;
    __m128 zy_w = _mm_move_ss(m, _mm_set_ss(f));
    return _mm_shuffle_ps(xy_w, zy_w, _MM_SHUFFLE(3, 0, 1, 0));
}

static SIMD_UTILS_FORCE_INLINE __m128 simd_insertw_ps(__m128 m, float f)
{
    __m128 xyz_ = m;
    __m128 wyz_ = _mm_move_ss(m, _mm_set_ss(f));
    return _mm_shuffle_ps(xyz_, wyz_, _MM_SHUFFLE(0, 2, 1, 0));
}

static SIMD_UTILS_FORCE_INLINE __m128 simd_shuffle_xyxy(__m128 a, __m128 b)
{
    return _mm_shuffle_ps(a, b, _MM_SHUFFLE(1, 0, 1, 0));
}

static SIMD_UTILS_FORCE_INLINE __m128 simd_shuffle_yzyz(__m128 a, __m128 b)
{
    return _mm_shuffle_ps(a, b, _MM_SHUFFLE(2, 1, 2, 1));
}

static SIMD_UTILS_FORCE_INLINE __m128 simd_shuffle_zxzx(__m128 a, __m128 b)
{
    return _mm_shuffle_ps(a, b, _MM_SHUFFLE(0, 2, 0, 2));
}

// NOTE: requires aosvec3s is 16-byte-aligned
static SIMD_UTILS_FORCE_INLINE void simd_set_soa_vec3(__m128* x, __m128* y, __m128* z, float* aosvec3s)
{
    __m128 x0y0z0x1 = _mm_load_ps(&aosvec3s[0]);
    __m128 y1z1x2y2 = _mm_load_ps(&aosvec3s[4]);
    __m128 z2x3y3z3 = _mm_load_ps(&aosvec3s[8]);

    __m128 x0x1z2z3 = _mm_shuffle_ps(x0y0z0x1, z2x3y3z3, _MM_SHUFFLE(3, 0, 3, 0));
    __m128 x2y2x3y3 = _mm_shuffle_ps(y1z1x2y2, z2x3y3z3, _MM_SHUFFLE(2, 1, 3, 2));
    __m128 y0z0y1z1 = _mm_shuffle_ps(x0y0z0x1, y1z1x2y2, _MM_SHUFFLE(1, 0, 2, 1));

    __m128 x0x1x2x3 = _mm_shuffle_ps(x0x1z2z3, x2y2x3y3, _MM_SHUFFLE(2, 0, 1, 0));
    __m128 y0y1y2y3 = _mm_shuffle_ps(y0z0y1z1, x2y2x3y3, _MM_SHUFFLE(3, 1, 2, 0));
    __m128 z0z1z2z3 = _mm_shuffle_ps(y0z0y1z1, x0x1z2z3, _MM_SHUFFLE(3, 2, 3, 1));

    *x = x0x1x2x3;
    *y = y0y1y2y3;
    *z = z0z1z2z3;
}

// NOTE: requires aosvec4s is 16-byte-aligned
static SIMD_UTILS_FORCE_INLINE void simd_set_soa_vec4(__m128* x, __m128* y, __m128* z, __m128* w, float* aosvec4s)
{
    __m128 vec0 = _mm_load_ps(&aosvec4s[0]);
    __m128 vec1 = _mm_load_ps(&aosvec4s[4]);
    __m128 vec2 = _mm_load_ps(&aosvec4s[8]);
    __m128 vec3 = _mm_load_ps(&aosvec4s[12]);

    _MM_TRANSPOSE4_PS(vec0, vec1, vec2, vec3);

    *x = vec0;
    *y = vec1;
    *z = vec2;
    *w = vec3;
}

#if SIMD_UTILS_USE_AVX
// NOTE: requires aosvec3s is 16-byte-aligned
// TODO: optimize for 256-bit vectors
static SIMD_UTILS_FORCE_INLINE void simd_set_soa_vec3(__m256* x, __m256* y, __m256* z, float* aosvec3s)
{
    __m128 x0, y0, z0, x1, y1, z1;
    simd_set_soa_vec3(&x0, &y0, &z0, aosvec3s);
    simd_set_soa_vec3(&x1, &y1, &z1, &aosvec3s[12]);
    *x = _mm256_setr_m128(x0, x1);
    *y = _mm256_setr_m128(y0, y1);
    *z = _mm256_setr_m128(z0, z1);
}

// NOTE: requires aosvec4s is 16-byte-aligned
// TODO: optimize for 256-bit vectors
static SIMD_UTILS_FORCE_INLINE void simd_set_soa_vec4(__m256* x, __m256* y, __m256* z, __m256* w, float* aosvec4s)
{
    __m128 x0, y0, z0, w0, x1, y1, z1, w1;
    simd_set_soa_vec4(&x0, &y0, &z0, &w0, aosvec4s);
    simd_set_soa_vec4(&x1, &y1, &z1, &w1, &aosvec4s[16]);
    *x = _mm256_setr_m128(x0, x1);
    *y = _mm256_setr_m128(y0, y1);
    *z = _mm256_setr_m128(z0, z1);
    *w = _mm256_setr_m128(w0, w1);
}
#endif

static SIMD_UTILS_FORCE_INLINE void simd_setelem_ps(__m128& m, float value, int idx)
{
    m.m128_f32[idx] = value;
}

static SIMD_UTILS_FORCE_INLINE void simd_setelem_epi32(__m128& m, int32_t value, int idx)
{
    m.m128_i32[idx] = value;
}

static SIMD_UTILS_FORCE_INLINE void simd_setelem_epi32(__m128i& m, int32_t value, int idx)
{
    m.m128i_i32[idx] = value;
}

#if SIMD_UTILS_USE_AVX
static SIMD_UTILS_FORCE_INLINE void simd_setelem_ps(__m256& m, float value, int idx)
{
    m.m256_f32[idx] = value;
}

static SIMD_UTILS_FORCE_INLINE void simd_setelem_epi32(__m256& m, int32_t value, int idx)
{
    Simd256Union tmp;
    tmp.vf = m;
    tmp.i[idx] = value;
    m = tmp.vf;
}

static SIMD_UTILS_FORCE_INLINE void simd_setelem_epi32(__m256i& m, int32_t value, int idx)
{
    m.m256i_i32[idx] = value;
}
#endif

static SIMD_UTILS_FORCE_INLINE __m128 simd_load3_unaligned_ps(const float* fptr)
{
    __m128 m;
#if SIMD_UTILS_UNALIGNED_LOAD
#if SIMD_UTILS_USE_SSE3_LDDQU
    Simd128Union tmp;
    tmp.vi = _mm_lddqu_si128((const __m128i*)fptr);
    m = tmp.vf;
#else
    m = _mm_loadu_ps(fptr);
#endif
#else
    m.m128_f32[0] = fptr[0];
    m.m128_f32[1] = fptr[1];
    m.m128_f32[2] = fptr[2];
#endif
    m.m128_f32[3] = 0.0f;
    return m;
}

static SIMD_UTILS_FORCE_INLINE void simd_store3_unaligned_ps(float* fptr, __m128 m)
{
    fptr[0] = m.m128_f32[0];
    fptr[1] = m.m128_f32[1];
    fptr[2] = m.m128_f32[2];
}

static SIMD_UTILS_FORCE_INLINE __m128i simd_load4_unaligned_ps(const int32_t* iptr)
{
#if SIMD_UTILS_UNALIGNED_LOAD
#if SIMD_UTILS_USE_SSE3_LDDQU
    return _mm_lddqu_si128((const __m128i*)iptr);
#else
    Simd128Union tmp;
    tmp.vf = _mm_loadu_ps((float*)iptr);
    return tmp.vi;
#endif
#else
    __m128i m;
    m.m128i_i32[0] = iptr[0];
    m.m128i_i32[1] = iptr[1];
    m.m128i_i32[2] = iptr[2];
    m.m128i_i32[3] = iptr[3];
    return m;
#endif
}

static SIMD_UTILS_FORCE_INLINE __m128 simd_load4_unaligned_ps(const float* fptr)
{
#if SIMD_UTILS_UNALIGNED_LOAD
#if SIMD_UTILS_USE_SSE3_LDDQU
    Simd128Union tmp;
    tmp.vi = _mm_lddqu_si128((const __m128i*)fptr);
    return tmp.vf;
#else
    return _mm_loadu_ps(fptr);
#endif
#else
    __m128 m;
    m.m128_f32[0] = fptr[0];
    m.m128_f32[1] = fptr[1];
    m.m128_f32[2] = fptr[2];
    m.m128_f32[3] = fptr[3];
    return m;
#endif
}

static SIMD_UTILS_FORCE_INLINE void simd_store4_unaligned_ps(float* fptr, __m128 m)
{
#if SIMD_UTILS_UNALIGNED_LOAD
    _mm_storeu_ps(fptr, m);
#else
    fptr[0] = m.m128_f32[0];
    fptr[1] = m.m128_f32[1];
    fptr[2] = m.m128_f32[2];
    fptr[3] = m.m128_f32[3];
#endif
}

#if SIMD_UTILS_USE_AVX
static SIMD_UTILS_FORCE_INLINE __m256i simd_load8_unaligned_ps(const int32_t* iptr)
{
#if SIMD_UTILS_UNALIGNED_LOAD
    Simd256Union tmp;
    tmp.vf = _mm256_loadu_ps((float*)iptr);
    return tmp.vi;
#else
    __m256i m;
    m.m256i_i32[0] = iptr[0];
    m.m256i_i32[1] = iptr[1];
    m.m256i_i32[2] = iptr[2];
    m.m256i_i32[3] = iptr[3];
    m.m256i_i32[4] = iptr[4];
    m.m256i_i32[5] = iptr[5];
    m.m256i_i32[6] = iptr[6];
    m.m256i_i32[7] = iptr[7];
    return m;
#endif
}

static SIMD_UTILS_FORCE_INLINE __m256 simd_load8_unaligned_ps(const float* fptr)
{
#if SIMD_UTILS_UNALIGNED_LOAD
    return _mm256_loadu_ps(fptr);
#else
    __m256 m;
    m.m256_f32[0] = fptr[0];
    m.m256_f32[1] = fptr[1];
    m.m256_f32[2] = fptr[2];
    m.m256_f32[3] = fptr[3];
    m.m256_f32[4] = fptr[4];
    m.m256_f32[5] = fptr[5];
    m.m256_f32[6] = fptr[6];
    m.m256_f32[7] = fptr[7];
    return m;
#endif
}

static SIMD_UTILS_FORCE_INLINE void simd_store8_unaligned_ps(float* fptr, __m256 m)
{
#if SIMD_UTILS_UNALIGNED_LOAD
    _mm256_storeu_ps(fptr, m);
#else
    fptr[0] = m.m256_f32[0];
    fptr[1] = m.m256_f32[1];
    fptr[2] = m.m256_f32[2];
    fptr[3] = m.m256_f32[3];
    fptr[4] = m.m256_f32[4];
    fptr[5] = m.m256_f32[5];
    fptr[6] = m.m256_f32[6];
    fptr[7] = m.m256_f32[7];
#endif
}
#endif

static SIMD_UTILS_FORCE_INLINE __m128i simd_mul_epi32(__m128i a, __m128i b)
{
#if SIMD_UTILS_USE_SSE4
    return _mm_mullo_epi32(a, b);
#else
    __m128i t1 = _mm_mul_epu32(a, b);
    __m128i t2 = _mm_mul_epu32(_mm_srli_si128(a, 4), _mm_srli_si128(b, 4));
    return _mm_unpacklo_epi32(_mm_shuffle_epi32(t1, _MM_SHUFFLE(0, 0, 2, 0)), _mm_shuffle_epi32(t2, _MM_SHUFFLE(0, 0, 2, 0)));
#endif
}

#if SIMD_UTILS_USE_AVX
static SIMD_UTILS_FORCE_INLINE __m256i simd_mul_epi32(__m256i a, __m256i b)
{
    return _mm256_mullo_epi32(a, b);
}
#endif

static SIMD_UTILS_FORCE_INLINE __m128i simd_setones_si128()
{
    return _mm_set1_epi32(0xffffffff);
}

static SIMD_UTILS_FORCE_INLINE __m128i simd_not_si128(__m128i m)
{
    return _mm_andnot_si128(m, simd_setones_si128());
}

#if SIMD_UTILS_USE_AVX
static SIMD_UTILS_FORCE_INLINE __m256i simd_setones_si256()
{
    return _mm256_set1_epi32(0xffffffff);
}

static SIMD_UTILS_FORCE_INLINE __m256i simd_not_si256(__m256i m)
{
    return _mm256_andnot_si256(m, simd_setones_si256());
}
#endif

// vector unsigned int comparisons
static SIMD_UTILS_FORCE_INLINE __m128i simd_cmple_epu32(__m128i a, __m128i b) { return _mm_cmpeq_epi32(a, _mm_min_epu32(a, b)); }
static SIMD_UTILS_FORCE_INLINE __m128i simd_cmpge_epu32(__m128i a, __m128i b) { return _mm_cmpeq_epi32(a, _mm_max_epu32(a, b)); }
static SIMD_UTILS_FORCE_INLINE __m128i simd_cmplt_epu32(__m128i a, __m128i b) { return simd_not_si128(simd_cmpge_epu32(a, b)); }
static SIMD_UTILS_FORCE_INLINE __m128i simd_cmpgt_epu32(__m128i a, __m128i b) { return simd_not_si128(simd_cmple_epu32(a, b)); }

#if SIMD_UTILS_USE_AVX
static SIMD_UTILS_FORCE_INLINE __m256i simd_cmple_epu32(__m256i a, __m256i b) { return _mm256_cmpeq_epi32(a, _mm256_min_epu32(a, b)); }
static SIMD_UTILS_FORCE_INLINE __m256i simd_cmpge_epu32(__m256i a, __m256i b) { return _mm256_cmpeq_epi32(a, _mm256_max_epu32(a, b)); }
static SIMD_UTILS_FORCE_INLINE __m256i simd_cmplt_epu32(__m256i a, __m256i b) { return simd_not_si256(simd_cmpge_epu32(a, b)); }
static SIMD_UTILS_FORCE_INLINE __m256i simd_cmpgt_epu32(__m256i a, __m256i b) { return simd_not_si256(simd_cmple_epu32(a, b)); }
#endif

#if SIMD_UTILS_USE_FMA
static SIMD_UTILS_FORCE_INLINE __m128 simd_madd_ps(__m128 a, __m128 b, __m128 c) { return _mm_fmadd_ps(a, b, c); }
#else
static SIMD_UTILS_FORCE_INLINE __m128 simd_madd_ps(__m128 a, __m128 b, __m128 c) { return _mm_add_ps(c, _mm_mul_ps(a, b)); }
#endif

#if SIMD_UTILS_USE_AVX
#if SIMD_UTILS_USE_FMA
static SIMD_UTILS_FORCE_INLINE __m256 simd_madd_ps(__m256 a, __m256 b, __m256 c) { return _mm256_fmadd_ps(a, b, c); }
#else
static SIMD_UTILS_FORCE_INLINE __m256 simd_madd_ps(__m256 a, __m256 b, __m256 c) { return _mm256_add_ps(c, _mm256_mul_ps(a, b)); }
#endif
#endif

static SIMD_UTILS_FORCE_INLINE __m128 simd_select_ps(__m128 a, __m128 b, __m128 selectb)
{
#if SIMD_UTILS_USE_SSE4
    return _mm_blendv_ps(a, b, selectb);
#else
    return _mm_or_ps(_mm_and_ps(selectb, b), _mm_andnot_ps(selectb, a));
#endif
}

#if SIMD_UTILS_USE_AVX
static SIMD_UTILS_FORCE_INLINE __m256 simd_select_ps(__m256 a, __m256 b, __m256 selectb)
{
    return _mm256_blendv_ps(a, b, selectb);
}
#endif

static SIMD_UTILS_FORCE_INLINE __m128i simd_select_epi32(__m128i a, __m128i b, __m128i selectb)
{
#if SIMD_UTILS_USE_SSE4
    return _mm_blendv_epi8(a, b, selectb);
#else
    return _mm_or_si128(_mm_and_si128(selectb, b), _mm_andnot_si128(selectb, a));
#endif
}

#if SIMD_UTILS_USE_AVX
static SIMD_UTILS_FORCE_INLINE __m256i simd_select_epi32(__m256i a, __m256i b, __m256i selectb)
{
    return _mm256_blendv_epi8(a, b, selectb);
}
#endif

static SIMD_UTILS_FORCE_INLINE __m128 simd_broadcast_ui_ps(uint32_t x)
{
    union { uint32_t ui; float f; } uifunion;
    uifunion.ui = x;
    return _mm_set1_ps(uifunion.f);
}

#if SIMD_UTILS_USE_AVX
static SIMD_UTILS_FORCE_INLINE __m256 simd256_broadcast_ui_ps(uint32_t x)
{
    union { uint32_t ui; float f; } uifunion;
    uifunion.ui = x;
    return _mm256_set1_ps(uifunion.f);
}
#endif

static SIMD_UTILS_FORCE_INLINE __m128 simd_select_ps(__m128 a, __m128 b, bool selectb)
{
    return simd_select_ps(a, b, simd_broadcast_ui_ps((uint32_t)(-(int)selectb)));
}

static SIMD_UTILS_FORCE_INLINE __m128 simd_select_ps(__m128 a, __m128 b, const uint32_t *_mask)
{
    return simd_select_ps(a, b, _mm_load_ps((float *)_mask));
}

static SIMD_UTILS_FORCE_INLINE __m128 simd_insertx_ps(__m128 m, __m128 x)
{
    return _mm_move_ss(m, x);
}

static SIMD_UTILS_FORCE_INLINE __m128 simd_inserty_ps(__m128 m, __m128 y)
{
    Simd128Union tmp;
    tmp.vi = _mm_setr_epi32(0, -1, 0, 0);
    return simd_select_ps(m, y, tmp.vf);
}

static SIMD_UTILS_FORCE_INLINE __m128 simd_insertz_ps(__m128 m, __m128 z)
{
    Simd128Union tmp;
    tmp.vi = _mm_setr_epi32(0, 0, -1, 0);
    return simd_select_ps(m, z, tmp.vf);
}

static SIMD_UTILS_FORCE_INLINE __m128 simd_insertw_ps(__m128 m, __m128 w)
{
    Simd128Union tmp;
    tmp.vi = _mm_setr_epi32(0, 0, 0, -1);
    return simd_select_ps(m, w, tmp.vf);
}

static SIMD_UTILS_FORCE_INLINE __m128 simd_setelem_ps(__m128 dst, __m128 src, int idx)
{
    Simd128Union srcu, dstu;
    srcu.vf = src;
    dstu.vf = dst;
    dstu.f[idx] = srcu.f[idx];
    return dstu.vf;
}

#if SIMD_UTILS_USE_FMA
static SIMD_UTILS_FORCE_INLINE __m128 simd_fnmadd_ps(__m128 a, __m128 b, __m128 c) { return _mm_fnmadd_ps(a, b, c); }
#else
static SIMD_UTILS_FORCE_INLINE __m128 simd_fnmadd_ps(__m128 a, __m128 b, __m128 c) { return _mm_sub_ps(c, _mm_mul_ps(a, b)); }
#endif

#if SIMD_UTILS_USE_AVX
#if SIMD_UTILS_USE_FMA
static SIMD_UTILS_FORCE_INLINE __m256 simd_fnmadd_ps(__m256 a, __m256 b, __m256 c) { return _mm256_fnmadd_ps(a, b, c); }
#else
static SIMD_UTILS_FORCE_INLINE __m256 simd_fnmadd_ps(__m256 a, __m256 b, __m256 c) { return _mm256_sub_ps(c, _mm256_mul_ps(a, b)); }
#endif
#endif

static SIMD_UTILS_FORCE_INLINE __m128 simd_abs_ps(__m128 x)
{
    return _mm_and_ps(x, simd_broadcast_ui_ps(0x7fffffff));
}

#if SIMD_UTILS_USE_AVX
static SIMD_UTILS_FORCE_INLINE __m256 simd_abs_ps(__m256 x)
{
    return _mm256_and_ps(x, simd256_broadcast_ui_ps(0x7fffffff));
}
#endif

static SIMD_UTILS_FORCE_INLINE __m128 simd_negate_ps(__m128 x) { return _mm_sub_ps(_mm_setzero_ps(), x); }

#if SIMD_UTILS_USE_AVX
static SIMD_UTILS_FORCE_INLINE __m256 simd_negate_ps(__m256 x) { return _mm256_sub_ps(_mm256_setzero_ps(), x); }
#endif

// 3D dot product with result broadcast to all simd slots
static SIMD_UTILS_FORCE_INLINE __m128 simd_dot3_ps(__m128 a, __m128 b)
{
#if SIMD_UTILS_USE_SSE4_DP_PS
    return _mm_dp_ps(a, b, 0x7f);
#else
    __m128 prod = _mm_mul_ps(a, b);
    return _mm_add_ps(simd_broadcast_ps(prod, 0), _mm_add_ps(simd_broadcast_ps(prod, 1), simd_broadcast_ps(prod, 2)));
#endif
}

// 4D dot product with result broadcast to all simd slots
static SIMD_UTILS_FORCE_INLINE __m128 simd_dot4_ps(__m128 a, __m128 b)
{
#if SIMD_UTILS_USE_SSE4_DP_PS
    return _mm_dp_ps(a, b, 0xff);
#else
    __m128 prod = _mm_mul_ps(a, b);
    return _mm_add_ps(simd_broadcast_ps(prod, 0), _mm_add_ps(simd_broadcast_ps(prod, 1), _mm_add_ps(simd_broadcast_ps(prod, 2), simd_broadcast_ps(prod, 3))));
#endif
}

// result is { dot3(mx, v), dot3(my, v), dot3(mz, v) }
static SIMD_UTILS_FORCE_INLINE __m128 simd_3dot3_ps(__m128 mx, __m128 my, __m128 mz, __m128 v)
{
    __m128 result = simd_dot3_ps(mx, v);
    result = simd_inserty_ps(result, simd_dot3_ps(my, v));
    result = simd_insertz_ps(result, simd_dot3_ps(mz, v));
    return result;
}
