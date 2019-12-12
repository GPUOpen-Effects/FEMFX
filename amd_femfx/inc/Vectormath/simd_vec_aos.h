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

#ifndef _SIMD_VECTORMATH_VEC_AOS_CPP_H
#define _SIMD_VECTORMATH_VEC_AOS_CPP_H

//-----------------------------------------------------------------------------
// Constants

#define _SIMD_VECTORMATH_UNIT_1000 _mm_setr_ps(1.0f,0.0f,0.0f,0.0f) // (__m128){ 1.0f, 0.0f, 0.0f, 0.0f }
#define _SIMD_VECTORMATH_UNIT_0100 _mm_setr_ps(0.0f,1.0f,0.0f,0.0f) // (__m128){ 0.0f, 1.0f, 0.0f, 0.0f }
#define _SIMD_VECTORMATH_UNIT_0010 _mm_setr_ps(0.0f,0.0f,1.0f,0.0f) // (__m128){ 0.0f, 0.0f, 1.0f, 0.0f }
#define _SIMD_VECTORMATH_UNIT_0001 _mm_setr_ps(0.0f,0.0f,0.0f,1.0f) // (__m128){ 0.0f, 0.0f, 0.0f, 1.0f }
#define _SIMD_VECTORMATH_SLERP_TOL 0.999f

//-----------------------------------------------------------------------------
// Definitions

#ifndef _SIMD_VECTORMATH_INTERNAL_FUNCTIONS
#define _SIMD_VECTORMATH_INTERNAL_FUNCTIONS

#define     _vmath_shufps(a, b, immx, immy, immz, immw) _mm_shuffle_ps(a, b, _MM_SHUFFLE(immw, immz, immy, immx))
static SIMD_VECTORMATH_FORCE_INLINE __m128 _vmathVfDot3( __m128 vec0, __m128 vec1 )
{
#if SIMD_UTILS_USE_SSE4_DP_PS
    return _mm_dp_ps(vec0, vec1, 0x7F);
#else
    __m128 result = _mm_mul_ps( vec0, vec1);
    return _mm_add_ps( simd_broadcast_ps( result, 0 ), _mm_add_ps( simd_broadcast_ps( result, 1 ), simd_broadcast_ps( result, 2 ) ) );
#endif
}

static SIMD_VECTORMATH_FORCE_INLINE __m128 _vmathVfDot4( __m128 vec0, __m128 vec1 )
{
#if SIMD_UTILS_USE_SSE4_DP_PS
    return _mm_dp_ps(vec0, vec1, 0xFF);
#else
    __m128 result = _mm_mul_ps(vec0, vec1);
    return _mm_add_ps(_mm_shuffle_ps(result, result, _MM_SHUFFLE(0,0,0,0)),
            _mm_add_ps(_mm_shuffle_ps(result, result, _MM_SHUFFLE(1,1,1,1)),
            _mm_add_ps(_mm_shuffle_ps(result, result, _MM_SHUFFLE(2,2,2,2)), _mm_shuffle_ps(result, result, _MM_SHUFFLE(3,3,3,3)))));
#endif
}

static SIMD_VECTORMATH_FORCE_INLINE __m128 _vmathVfCross( __m128 vec0, __m128 vec1 )
{
    __m128 tmp0, tmp1, tmp2, tmp3, result;
    tmp0 = _mm_shuffle_ps( vec0, vec0, _MM_SHUFFLE(3,0,2,1) );
    tmp1 = _mm_shuffle_ps( vec1, vec1, _MM_SHUFFLE(3,1,0,2) );
    tmp2 = _mm_shuffle_ps( vec0, vec0, _MM_SHUFFLE(3,1,0,2) );
    tmp3 = _mm_shuffle_ps( vec1, vec1, _MM_SHUFFLE(3,0,2,1) );
    result = _mm_mul_ps( tmp0, tmp1 );
    result = simd_fnmadd_ps( tmp2, tmp3, result );
    return result;
}

#endif

namespace FmVectormath {
    
SIMD_VECTORMATH_FORCE_INLINE SimdVector3::SimdVector3(const SimdVector3& vec)
{
    set128(vec.get128());
}

SIMD_VECTORMATH_FORCE_INLINE void SimdVector3::set128(__m128 vec)
{
    mVec128 = vec;
}


SIMD_VECTORMATH_FORCE_INLINE SimdVector3::SimdVector3( float _x, float _y, float _z )
{
    mVec128 = _mm_setr_ps(_x, _y, _z, 0.0f);
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3::SimdVector3( const SimdFloat &_x, const SimdFloat &_y, const SimdFloat &_z )
{
    __m128 xz = _mm_unpacklo_ps( _x.get128(), _z.get128() );
    mVec128 = _mm_unpacklo_ps( xz, _y.get128() );
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3::SimdVector3( const SimdPoint3 & pnt )
{
    mVec128 = pnt.get128();
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3::SimdVector3( float scalar )
{
    mVec128 = SimdFloat(scalar).get128();
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3::SimdVector3( const SimdFloat &scalar )
{
    mVec128 = scalar.get128();
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3::SimdVector3( __m128 vf4 )
{
    mVec128 = vf4;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdVector3::xAxis( )
{
    return SimdVector3( _SIMD_VECTORMATH_UNIT_1000 );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdVector3::yAxis( )
{
    return SimdVector3( _SIMD_VECTORMATH_UNIT_0100 );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdVector3::zAxis( )
{
    return SimdVector3( _SIMD_VECTORMATH_UNIT_0010 );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 lerp( float t, const SimdVector3 & vec0, const SimdVector3 & vec1 )
{
    return lerp( SimdFloat(t), vec0, vec1 );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 lerp( const SimdFloat &t, const SimdVector3 & vec0, const SimdVector3 & vec1 )
{
    return ( vec0 + ( ( vec1 - vec0 ) * t ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 slerp( float t, const SimdVector3 & unitVec0, const SimdVector3 & unitVec1 )
{
    return slerp( SimdFloat(t), unitVec0, unitVec1 );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 slerp( const SimdFloat &t, const SimdVector3 & unitVec0, const SimdVector3 & unitVec1 )
{
    __m128 scales, scale0, scale1, cosAngle, angle, tttt, oneMinusT, angles, sines;
    cosAngle = _vmathVfDot3( unitVec0.get128(), unitVec1.get128() );
    __m128 selectMask = _mm_cmpgt_ps( _mm_set1_ps(_SIMD_VECTORMATH_SLERP_TOL), cosAngle );
    angle = simd_acos_ps( cosAngle );
    tttt = t.get128();
    oneMinusT = _mm_sub_ps( _mm_set1_ps(1.0f), tttt );
    angles = _mm_unpacklo_ps( _mm_set1_ps(1.0f), tttt ); // angles = 1, t, 1, t
    angles = _mm_unpacklo_ps( angles, oneMinusT );        // angles = 1, 1-t, t, 1-t
    angles = _mm_mul_ps( angles, angle );
    sines = simd_sin_ps( angles );
    scales = _mm_div_ps( sines, simd_broadcast_ps( sines, 0 ) );
    scale0 = simd_select_ps( oneMinusT, simd_broadcast_ps( scales, 1 ), selectMask );
    scale1 = simd_select_ps( tttt, simd_broadcast_ps( scales, 2 ), selectMask );
    return SimdVector3( simd_madd_ps( unitVec0.get128(), scale0, _mm_mul_ps( unitVec1.get128(), scale1 ) ) );
}

SIMD_VECTORMATH_FORCE_INLINE __m128 SimdVector3::get128( ) const
{
    return mVec128;
}

SIMD_VECTORMATH_FORCE_INLINE void loadXYZ(SimdPoint3& vec, const float* fptr)
{
    vec = SimdPoint3(simd_load3_unaligned_ps(fptr));
}

SIMD_VECTORMATH_FORCE_INLINE void loadXYZ(SimdVector3& vec, const float* fptr)
{
    vec = SimdVector3(simd_load3_unaligned_ps(fptr));
}

SIMD_VECTORMATH_FORCE_INLINE void storeXYZ( const SimdVector3 & vec, __m128 * quad )
{
    *quad = simd_insertw_ps(vec.get128(), *quad);
}

SIMD_VECTORMATH_FORCE_INLINE void storeXYZ(const SimdPoint3& vec, float* fptr)
{
    simd_store3_unaligned_ps(fptr, vec.get128());
}

SIMD_VECTORMATH_FORCE_INLINE void storeXYZ(const SimdVector3& vec, float* fptr)
{
    simd_store3_unaligned_ps(fptr, vec.get128());
}

SIMD_VECTORMATH_FORCE_INLINE void loadXYZArray( SimdVector3 & vec0, SimdVector3 & vec1, SimdVector3 & vec2, SimdVector3 & vec3, const __m128 * threeQuads )
{
    const float *quads = (float *)threeQuads;
    vec0 = SimdVector3(  _mm_load_ps(quads) );
    vec1 = SimdVector3( _mm_loadu_ps(quads + 3) );
    vec2 = SimdVector3( _mm_loadu_ps(quads + 6) );
    vec3 = SimdVector3( _mm_loadu_ps(quads + 9) );
}

SIMD_VECTORMATH_FORCE_INLINE void storeXYZArray( const SimdVector3 & vec0, const SimdVector3 & vec1, const SimdVector3 & vec2, const SimdVector3 & vec3, __m128 * threeQuads )
{
    __m128 xxxx = _mm_shuffle_ps( vec1.get128(), vec1.get128(), _MM_SHUFFLE(0, 0, 0, 0) );
    __m128 zzzz = _mm_shuffle_ps( vec2.get128(), vec2.get128(), _MM_SHUFFLE(2, 2, 2, 2) );
    SIMD_VECTORMATH_ALIGN16 unsigned int xsw[4] = {0, 0, 0, 0xffffffff};
    SIMD_VECTORMATH_ALIGN16 unsigned int zsw[4] = {0xffffffff, 0, 0, 0};
    threeQuads[0] = simd_select_ps( vec0.get128(), xxxx, xsw );
    threeQuads[1] = _mm_shuffle_ps( vec1.get128(), vec2.get128(), _MM_SHUFFLE(1, 0, 2, 1) );
    threeQuads[2] = simd_select_ps( _mm_shuffle_ps( vec3.get128(), vec3.get128(), _MM_SHUFFLE(2, 1, 0, 3) ), zzzz, zsw );
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & SimdVector3::operator =( const SimdVector3 & vec )
{
    mVec128 = vec.mVec128;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & SimdVector3::setX( float _x )
{
    mVec128 = simd_insertx_ps(mVec128, _x);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & SimdVector3::setX( const SimdFloat &_x )
{
    mVec128 = simd_insertx_ps(mVec128, _x.get128());
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdVector3::getX( ) const
{
    return SimdFloat(simd_broadcast_ps(mVec128, 0));
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & SimdVector3::setY( float _y )
{
    mVec128 = simd_inserty_ps(mVec128, _y);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & SimdVector3::setY( const SimdFloat &_y )
{
    mVec128 = simd_inserty_ps(mVec128, _y.get128());
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdVector3::getY( ) const
{
    return SimdFloat(simd_broadcast_ps(mVec128, 1));
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & SimdVector3::setZ( float _z )
{
    mVec128 = simd_insertz_ps(mVec128, _z);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & SimdVector3::setZ( const SimdFloat &_z )
{
    mVec128 = simd_insertz_ps(mVec128, _z.get128());
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdVector3::getZ( ) const
{
    return SimdFloat(simd_broadcast_ps(mVec128, 2));
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & SimdVector3::setElem( int idx, float value )
{
    simd_setelem_ps(mVec128, value, idx);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & SimdVector3::setElem( int idx, const SimdFloat & value )
{
    simd_setelem_ps(mVec128, value.get128(), idx);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdVector3::getElem( int idx ) const
{
    return SimdFloat( mVec128, idx );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdVector3::operator +( const SimdVector3 & vec ) const
{
    return SimdVector3( _mm_add_ps( mVec128, vec.mVec128 ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdVector3::operator -( const SimdVector3 & vec ) const
{
    return SimdVector3( _mm_sub_ps( mVec128, vec.mVec128 ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 SimdVector3::operator +( const SimdPoint3 & pnt ) const
{
    return SimdPoint3( _mm_add_ps( mVec128, pnt.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdVector3::operator *( float scalar ) const
{
    return *this * SimdFloat(scalar);
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdVector3::operator *( const SimdFloat &scalar ) const
{
    return SimdVector3( _mm_mul_ps( mVec128, scalar.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & SimdVector3::operator +=( const SimdVector3 & vec )
{
    *this = *this + vec;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & SimdVector3::operator -=( const SimdVector3 & vec )
{
    *this = *this - vec;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & SimdVector3::operator *=( float scalar )
{
    *this = *this * scalar;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & SimdVector3::operator *=( const SimdFloat &scalar )
{
    *this = *this * scalar;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdVector3::operator /( float scalar ) const
{
    return *this / SimdFloat(scalar);
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdVector3::operator /( const SimdFloat &scalar ) const
{
    return SimdVector3( _mm_div_ps( mVec128, scalar.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & SimdVector3::operator /=( float scalar )
{
    *this = *this / scalar;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & SimdVector3::operator /=( const SimdFloat &scalar )
{
    *this = *this / scalar;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdVector3::operator -( ) const
{
    //return SimdVector3(_mm_sub_ps( _mm_setzero_ps(), mVec128 ) );

    SIMD_VECTORMATH_ALIGN16 static const unsigned int array[] = {0x80000000, 0x80000000, 0x80000000, 0x80000000};
    __m128 NEG_MASK = SSEFloat(*(const __m128*)array).vf;
    return SimdVector3(_mm_xor_ps(get128(),NEG_MASK));
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 operator *( float scalar, const SimdVector3 & vec )
{
    return SimdFloat(scalar) * vec;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 operator *( const SimdFloat &scalar, const SimdVector3 & vec )
{
    return vec * scalar;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 operator *( const SimdVector3 & vec0, const SimdVector3 & vec1 )
{
    return SimdVector3( _mm_mul_ps( vec0.get128(), vec1.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 operator /( const SimdVector3 & vec0, const SimdVector3 & vec1 )
{
    return SimdVector3( _mm_div_ps( vec0.get128(), vec1.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 inv( const SimdVector3 & vec )
{
    return SimdVector3( _mm_rcp_ps( vec.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 sqrt( const SimdVector3 & vec )
{
    return SimdVector3( _mm_sqrt_ps( vec.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 rsqrt( const SimdVector3 & vec )
{
    return SimdVector3( _mm_rsqrt_ps( vec.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 abs( const SimdVector3 & vec )
{
    return SimdVector3( simd_abs_ps( vec.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 copySign( const SimdVector3 & vec0, const SimdVector3 & vec1 )
{
    __m128 vmask = simd_broadcast_ui_ps(0x7fffffff);
    return SimdVector3( _mm_or_ps(
        _mm_and_ps   ( vmask, vec0.get128() ),            // Value
        _mm_andnot_ps( vmask, vec1.get128() ) ) );        // Signs
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 max( const SimdVector3 & vec0, const SimdVector3 & vec1 )
{
    return SimdVector3( _mm_max_ps( vec0.get128(), vec1.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat maxElem( const SimdVector3 & vec )
{
    return SimdFloat( _mm_max_ps( _mm_max_ps( simd_broadcast_ps( vec.get128(), 0 ), simd_broadcast_ps( vec.get128(), 1 ) ), simd_broadcast_ps( vec.get128(), 2 ) ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 min( const SimdVector3 & vec0, const SimdVector3 & vec1 )
{
    return SimdVector3( _mm_min_ps( vec0.get128(), vec1.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat minElem( const SimdVector3 & vec )
{
    return SimdFloat( _mm_min_ps( _mm_min_ps( simd_broadcast_ps( vec.get128(), 0 ), simd_broadcast_ps( vec.get128(), 1 ) ), simd_broadcast_ps( vec.get128(), 2 ) ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat sum( const SimdVector3 & vec )
{
    return SimdFloat( _mm_add_ps( _mm_add_ps( simd_broadcast_ps( vec.get128(), 0 ), simd_broadcast_ps( vec.get128(), 1 ) ), simd_broadcast_ps( vec.get128(), 2 ) ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat dot( const SimdVector3 & vec0, const SimdVector3 & vec1 )
{
    return SimdFloat( _vmathVfDot3( vec0.get128(), vec1.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat lengthSqr( const SimdVector3 & vec )
{
    return SimdFloat( _vmathVfDot3( vec.get128(), vec.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat length( const SimdVector3 & vec )
{
    return SimdFloat( _mm_sqrt_ps(_vmathVfDot3( vec.get128(), vec.get128() )) );
}


SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 normalizeApprox( const SimdVector3 & vec )
{
    return SimdVector3( _mm_mul_ps( vec.get128(), _mm_rsqrt_ps( _vmathVfDot3( vec.get128(), vec.get128() ) ) ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 normalize( const SimdVector3 & vec )
{
    return SimdVector3( _mm_mul_ps( vec.get128(), simd_rsqrt_newtonraphson_ps( _vmathVfDot3( vec.get128(), vec.get128() ) ) ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 cross( const SimdVector3 & vec0, const SimdVector3 & vec1 )
{
    return SimdVector3( _vmathVfCross( vec0.get128(), vec1.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 select( const SimdVector3 & vec0, const SimdVector3 & vec1, bool select1 )
{
    return select( vec0, vec1, SimdBool(select1) );
}

SIMD_VECTORMATH_FORCE_INLINE  const SimdVector3 select(const SimdVector3& vec0, const SimdVector3& vec1, const SimdBool& select1)
{
    return SimdVector3(simd_select_ps(vec0.get128(), vec1.get128(), select1.get128()));
}

SIMD_VECTORMATH_FORCE_INLINE  const SimdVector4 select(const SimdVector4& vec0, const SimdVector4& vec1, const SimdBool& select1)
{
    return SimdVector4(simd_select_ps(vec0.get128(), vec1.get128(), select1.get128()));
}

SIMD_VECTORMATH_FORCE_INLINE void loadXYZW(SimdVector4& vec, const float* fptr)
{
    vec = SimdVector4(simd_load4_unaligned_ps(fptr));
}

SIMD_VECTORMATH_FORCE_INLINE void storeXYZW(const SimdVector4& vec, float* fptr)
{
    simd_store4_unaligned_ps(fptr, vec.get128());
}

#ifdef _VECTORMATH_DEBUG

SIMD_VECTORMATH_FORCE_INLINE void print( const SimdVector3 & vec )
{
    union { __m128 v; float s[4]; } tmp;
    tmp.v = vec.get128();
    printf( "( %f %f %f )\n", tmp.s[0], tmp.s[1], tmp.s[2] );
}

SIMD_VECTORMATH_FORCE_INLINE void print( const SimdVector3 & vec, const char * name )
{
    union { __m128 v; float s[4]; } tmp;
    tmp.v = vec.get128();
    printf( "%s: ( %f %f %f )\n", name, tmp.s[0], tmp.s[1], tmp.s[2] );
}

#endif

SIMD_VECTORMATH_FORCE_INLINE SimdVector4::SimdVector4(const SimdVector4 & vec)
{
    mVec128 = vec.mVec128;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4::SimdVector4( float _x, float _y, float _z, float _w )
{
    mVec128 = _mm_setr_ps(_x, _y, _z, _w); 
 }

SIMD_VECTORMATH_FORCE_INLINE SimdVector4::SimdVector4( const SimdFloat &_x, const SimdFloat &_y, const SimdFloat &_z, const SimdFloat &_w )
{
    mVec128 = _mm_unpacklo_ps(
        _mm_unpacklo_ps( _x.get128(), _z.get128() ),
        _mm_unpacklo_ps( _y.get128(), _w.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4::SimdVector4( const SimdVector3 &xyz, float _w )
{
    mVec128 = xyz.get128();
    mVec128 = simd_insertw_ps(mVec128, _w);
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4::SimdVector4( const SimdVector3 &xyz, const SimdFloat &_w )
{
    mVec128 = xyz.get128();
    mVec128 = simd_insertw_ps(mVec128, _w.get128());
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4::SimdVector4( const SimdVector3 & vec )
{
    mVec128 = vec.get128();
    mVec128 = simd_insertw_ps(mVec128, _mm_setzero_ps());
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4::SimdVector4( const SimdPoint3 & pnt )
{
    mVec128 = pnt.get128();
    mVec128 = simd_insertw_ps(mVec128, _mm_set1_ps(1.0f));
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4::SimdVector4( const SimdQuat & quat )
{
    mVec128 = quat.get128();
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4::SimdVector4( float scalar )
{
    mVec128 = SimdFloat(scalar).get128();
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4::SimdVector4( const SimdFloat &scalar )
{
    mVec128 = scalar.get128();
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4::SimdVector4( __m128 vf4 )
{
    mVec128 = vf4;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 SimdVector4::xAxis( )
{
    return SimdVector4( _SIMD_VECTORMATH_UNIT_1000 );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 SimdVector4::yAxis( )
{
    return SimdVector4( _SIMD_VECTORMATH_UNIT_0100 );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 SimdVector4::zAxis( )
{
    return SimdVector4( _SIMD_VECTORMATH_UNIT_0010 );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 SimdVector4::wAxis( )
{
    return SimdVector4( _SIMD_VECTORMATH_UNIT_0001 );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 lerp( float t, const SimdVector4 & vec0, const SimdVector4 & vec1 )
{
    return lerp( SimdFloat(t), vec0, vec1 );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 lerp( const SimdFloat &t, const SimdVector4 & vec0, const SimdVector4 & vec1 )
{
    return ( vec0 + ( ( vec1 - vec0 ) * t ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 slerp( float t, const SimdVector4 & unitVec0, const SimdVector4 & unitVec1 )
{
    return slerp( SimdFloat(t), unitVec0, unitVec1 );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 slerp( const SimdFloat &t, const SimdVector4 & unitVec0, const SimdVector4 & unitVec1 )
{
    __m128 scales, scale0, scale1, cosAngle, angle, tttt, oneMinusT, angles, sines;
    cosAngle = _vmathVfDot4( unitVec0.get128(), unitVec1.get128() );
    __m128 selectMask = _mm_cmpgt_ps( _mm_set1_ps(_SIMD_VECTORMATH_SLERP_TOL), cosAngle );
    angle = simd_acos_ps( cosAngle );
    tttt = t.get128();
    oneMinusT = _mm_sub_ps( _mm_set1_ps(1.0f), tttt );
    angles = _mm_unpacklo_ps( _mm_set1_ps(1.0f), tttt ); // angles = 1, t, 1, t
    angles = _mm_unpacklo_ps( angles, oneMinusT );        // angles = 1, 1-t, t, 1-t
    angles = _mm_mul_ps( angles, angle );
    sines = simd_sin_ps( angles );
    scales = _mm_div_ps( sines, simd_broadcast_ps( sines, 0 ) );
    scale0 = simd_select_ps( oneMinusT, simd_broadcast_ps( scales, 1 ), selectMask );
    scale1 = simd_select_ps( tttt, simd_broadcast_ps( scales, 2 ), selectMask );
    return SimdVector4( simd_madd_ps( unitVec0.get128(), scale0, _mm_mul_ps( unitVec1.get128(), scale1 ) ) );
}

SIMD_VECTORMATH_FORCE_INLINE __m128 SimdVector4::get128( ) const
{
    return mVec128;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & SimdVector4::operator =( const SimdVector4 & vec )
{
    mVec128 = vec.mVec128;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & SimdVector4::setXYZ( const SimdVector3 & vec )
{
    SIMD_VECTORMATH_ALIGN16 unsigned int sw[4] = {0, 0, 0, 0xffffffff};
    mVec128 = simd_select_ps( vec.get128(), mVec128, sw );
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdVector4::getXYZ( ) const
{
    return SimdVector3( mVec128 );
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & SimdVector4::setX( float _x )
{
    mVec128 = simd_insertx_ps(mVec128, _x);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & SimdVector4::setX( const SimdFloat &_x )
{
    mVec128 = simd_insertx_ps(mVec128, _x.get128());
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdVector4::getX( ) const
{
    return SimdFloat(simd_broadcast_ps(mVec128, 0));
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & SimdVector4::setY( float _y )
{
    mVec128 = simd_inserty_ps(mVec128, _y);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & SimdVector4::setY( const SimdFloat &_y )
{
    mVec128 = simd_inserty_ps(mVec128, _y.get128());
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdVector4::getY( ) const
{
    return SimdFloat(simd_broadcast_ps(mVec128, 1));
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & SimdVector4::setZ( float _z )
{
    mVec128 = simd_insertz_ps(mVec128, _z);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & SimdVector4::setZ( const SimdFloat &_z )
{
    mVec128 = simd_insertz_ps(mVec128, _z.get128());
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdVector4::getZ( ) const
{
    return SimdFloat(simd_broadcast_ps(mVec128, 2));
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & SimdVector4::setW( float _w )
{
    mVec128 = simd_insertw_ps(mVec128, _w);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & SimdVector4::setW( const SimdFloat &_w )
{
    mVec128 = simd_insertw_ps(mVec128, _w.get128());
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdVector4::getW( ) const
{
    return SimdFloat(simd_broadcast_ps(mVec128, 3));
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & SimdVector4::setElem( int idx, float value )
{
    simd_setelem_ps(mVec128, value, idx);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & SimdVector4::setElem( int idx, const SimdFloat & value )
{
    simd_setelem_ps(mVec128, value.get128(), idx);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdVector4::getElem( int idx ) const
{
    return SimdFloat( mVec128, idx );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 SimdVector4::operator +( const SimdVector4 & vec ) const
{
    return SimdVector4( _mm_add_ps( mVec128, vec.mVec128 ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 SimdVector4::operator -( const SimdVector4 & vec ) const
{
    return SimdVector4( _mm_sub_ps( mVec128, vec.mVec128 ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 SimdVector4::operator *( float scalar ) const
{
    return *this * SimdFloat(scalar);
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 SimdVector4::operator *( const SimdFloat &scalar ) const
{
    return SimdVector4( _mm_mul_ps( mVec128, scalar.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & SimdVector4::operator +=( const SimdVector4 & vec )
{
    *this = *this + vec;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & SimdVector4::operator -=( const SimdVector4 & vec )
{
    *this = *this - vec;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & SimdVector4::operator *=( float scalar )
{
    *this = *this * scalar;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & SimdVector4::operator *=( const SimdFloat &scalar )
{
    *this = *this * scalar;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 SimdVector4::operator /( float scalar ) const
{
    return *this / SimdFloat(scalar);
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 SimdVector4::operator /( const SimdFloat &scalar ) const
{
    return SimdVector4( _mm_div_ps( mVec128, scalar.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & SimdVector4::operator /=( float scalar )
{
    *this = *this / scalar;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & SimdVector4::operator /=( const SimdFloat &scalar )
{
    *this = *this / scalar;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 SimdVector4::operator -( ) const
{
    return SimdVector4(_mm_sub_ps( _mm_setzero_ps(), mVec128 ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 operator *( float scalar, const SimdVector4 & vec )
{
    return SimdFloat(scalar) * vec;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 operator *( const SimdFloat &scalar, const SimdVector4 & vec )
{
    return vec * scalar;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 operator *( const SimdVector4 & vec0, const SimdVector4 & vec1 )
{
    return SimdVector4( _mm_mul_ps( vec0.get128(), vec1.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 operator /( const SimdVector4 & vec0, const SimdVector4 & vec1 )
{
    return SimdVector4( _mm_div_ps( vec0.get128(), vec1.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 inv( const SimdVector4 & vec )
{
    return SimdVector4( _mm_rcp_ps( vec.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 sqrt( const SimdVector4 & vec )
{
    return SimdVector4( _mm_sqrt_ps( vec.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 rsqrt( const SimdVector4 & vec )
{
    return SimdVector4( _mm_rsqrt_ps( vec.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 abs( const SimdVector4 & vec )
{
    return SimdVector4( simd_abs_ps( vec.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 copySign( const SimdVector4 & vec0, const SimdVector4 & vec1 )
{
    __m128 vmask = simd_broadcast_ui_ps(0x7fffffff);
    return SimdVector4( _mm_or_ps(
        _mm_and_ps   ( vmask, vec0.get128() ),            // Value
        _mm_andnot_ps( vmask, vec1.get128() ) ) );        // Signs
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 max( const SimdVector4 & vec0, const SimdVector4 & vec1 )
{
    return SimdVector4( _mm_max_ps( vec0.get128(), vec1.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat maxElem( const SimdVector4 & vec )
{
    return SimdFloat( _mm_max_ps(
        _mm_max_ps( simd_broadcast_ps( vec.get128(), 0 ), simd_broadcast_ps( vec.get128(), 1 ) ),
        _mm_max_ps( simd_broadcast_ps( vec.get128(), 2 ), simd_broadcast_ps( vec.get128(), 3 ) ) ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 min( const SimdVector4 & vec0, const SimdVector4 & vec1 )
{
    return SimdVector4( _mm_min_ps( vec0.get128(), vec1.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat minElem( const SimdVector4 & vec )
{
    return SimdFloat( _mm_min_ps(
        _mm_min_ps( simd_broadcast_ps( vec.get128(), 0 ), simd_broadcast_ps( vec.get128(), 1 ) ),
        _mm_min_ps( simd_broadcast_ps( vec.get128(), 2 ), simd_broadcast_ps( vec.get128(), 3 ) ) ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat sum( const SimdVector4 & vec )
{
    return SimdFloat( _mm_add_ps(
        _mm_add_ps( simd_broadcast_ps( vec.get128(), 0 ), simd_broadcast_ps( vec.get128(), 1 ) ),
        _mm_add_ps( simd_broadcast_ps( vec.get128(), 2 ), simd_broadcast_ps( vec.get128(), 3 ) ) ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat dot( const SimdVector4 & vec0, const SimdVector4 & vec1 )
{
    return SimdFloat( _vmathVfDot4( vec0.get128(), vec1.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat lengthSqr( const SimdVector4 & vec )
{
    return SimdFloat( _vmathVfDot4( vec.get128(), vec.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat length( const SimdVector4 & vec )
{
    return SimdFloat( _mm_sqrt_ps(_vmathVfDot4( vec.get128(), vec.get128() )) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 normalizeApprox( const SimdVector4 & vec )
{
    return SimdVector4( _mm_mul_ps( vec.get128(), _mm_rsqrt_ps( _vmathVfDot4( vec.get128(), vec.get128() ) ) ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 normalize( const SimdVector4 & vec )
{
    return SimdVector4( _mm_mul_ps( vec.get128(), simd_rsqrt_newtonraphson_ps( _vmathVfDot4( vec.get128(), vec.get128() ) ) ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 select( const SimdVector4 & vec0, const SimdVector4 & vec1, bool select1 )
{
    return select( vec0, vec1, SimdBool(select1) );
}


#ifdef _VECTORMATH_DEBUG

SIMD_VECTORMATH_FORCE_INLINE void print( const SimdVector4 & vec )
{
    union { __m128 v; float s[4]; } tmp;
    tmp.v = vec.get128();
    printf( "( %f %f %f %f )\n", tmp.s[0], tmp.s[1], tmp.s[2], tmp.s[3] );
}

SIMD_VECTORMATH_FORCE_INLINE void print( const SimdVector4 & vec, const char * name )
{
    union { __m128 v; float s[4]; } tmp;
    tmp.v = vec.get128();
    printf( "%s: ( %f %f %f %f )\n", name, tmp.s[0], tmp.s[1], tmp.s[2], tmp.s[3] );
}

#endif

SIMD_VECTORMATH_FORCE_INLINE SimdPoint3::SimdPoint3(const SimdPoint3 & pnt)
{
    mVec128 = pnt.mVec128;
}

SIMD_VECTORMATH_FORCE_INLINE SimdPoint3::SimdPoint3( float _x, float _y, float _z )
{
    mVec128 = _mm_setr_ps(_x, _y, _z, 0.0f);
}

SIMD_VECTORMATH_FORCE_INLINE SimdPoint3::SimdPoint3( const SimdFloat &_x, const SimdFloat &_y, const SimdFloat &_z )
{
    mVec128 = _mm_unpacklo_ps( _mm_unpacklo_ps( _x.get128(), _z.get128() ), _y.get128() );
}

SIMD_VECTORMATH_FORCE_INLINE SimdPoint3::SimdPoint3( const SimdVector3 & vec )
{
    mVec128 = vec.get128();
}

SIMD_VECTORMATH_FORCE_INLINE SimdPoint3::SimdPoint3( float scalar )
{
    mVec128 = SimdFloat(scalar).get128();
}

SIMD_VECTORMATH_FORCE_INLINE SimdPoint3::SimdPoint3( const SimdFloat &scalar )
{
    mVec128 = scalar.get128();
}

SIMD_VECTORMATH_FORCE_INLINE SimdPoint3::SimdPoint3( __m128 vf4 )
{
    mVec128 = vf4;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 lerp( float t, const SimdPoint3 & pnt0, const SimdPoint3 & pnt1 )
{
    return lerp( SimdFloat(t), pnt0, pnt1 );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 lerp( const SimdFloat &t, const SimdPoint3 & pnt0, const SimdPoint3 & pnt1 )
{
    return ( pnt0 + ( ( pnt1 - pnt0 ) * t ) );
}

SIMD_VECTORMATH_FORCE_INLINE __m128 SimdPoint3::get128( ) const
{
    return mVec128;
}

SIMD_VECTORMATH_FORCE_INLINE void storeXYZ( const SimdPoint3 & pnt, __m128 * quad )
{
    *quad = simd_insertw_ps(pnt.get128(), *quad);
}

SIMD_VECTORMATH_FORCE_INLINE void loadXYZArray( SimdPoint3 & pnt0, SimdPoint3 & pnt1, SimdPoint3 & pnt2, SimdPoint3 & pnt3, const __m128 * threeQuads )
{
    const float *quads = (float *)threeQuads;
    pnt0 = SimdPoint3(  _mm_load_ps(quads) );
    pnt1 = SimdPoint3( _mm_loadu_ps(quads + 3) );
    pnt2 = SimdPoint3( _mm_loadu_ps(quads + 6) );
    pnt3 = SimdPoint3( _mm_loadu_ps(quads + 9) );
}

SIMD_VECTORMATH_FORCE_INLINE void storeXYZArray( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1, const SimdPoint3 & pnt2, const SimdPoint3 & pnt3, __m128 * threeQuads )
{
    __m128 xxxx = _mm_shuffle_ps( pnt1.get128(), pnt1.get128(), _MM_SHUFFLE(0, 0, 0, 0) );
    __m128 zzzz = _mm_shuffle_ps( pnt2.get128(), pnt2.get128(), _MM_SHUFFLE(2, 2, 2, 2) );
    SIMD_VECTORMATH_ALIGN16 unsigned int xsw[4] = {0, 0, 0, 0xffffffff};
    SIMD_VECTORMATH_ALIGN16 unsigned int zsw[4] = {0xffffffff, 0, 0, 0};
    threeQuads[0] = simd_select_ps( pnt0.get128(), xxxx, xsw );
    threeQuads[1] = _mm_shuffle_ps( pnt1.get128(), pnt2.get128(), _MM_SHUFFLE(1, 0, 2, 1) );
    threeQuads[2] = simd_select_ps( _mm_shuffle_ps( pnt3.get128(), pnt3.get128(), _MM_SHUFFLE(2, 1, 0, 3) ), zzzz, zsw );
}

SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & SimdPoint3::operator =( const SimdPoint3 & pnt )
{
    mVec128 = pnt.mVec128;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & SimdPoint3::setX( float _x )
{
    mVec128 = simd_insertx_ps(mVec128, _x);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & SimdPoint3::setX( const SimdFloat &_x )
{
    mVec128 = simd_insertx_ps(mVec128, _x.get128());
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdPoint3::getX( ) const
{
    return SimdFloat(simd_broadcast_ps(mVec128, 0));
}

SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & SimdPoint3::setY( float _y )
{
    mVec128 = simd_inserty_ps(mVec128, _y);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & SimdPoint3::setY( const SimdFloat &_y )
{
    mVec128 = simd_inserty_ps(mVec128, _y.get128());
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdPoint3::getY( ) const
{
    return SimdFloat(simd_broadcast_ps(mVec128, 1));
}

SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & SimdPoint3::setZ( float _z )
{
    mVec128 = simd_insertz_ps(mVec128, _z);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & SimdPoint3::setZ( const SimdFloat &_z )
{
    mVec128 = simd_insertz_ps(mVec128, _z.get128());
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdPoint3::getZ( ) const
{
    return SimdFloat(simd_broadcast_ps(mVec128, 2));
}

SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & SimdPoint3::setElem( int idx, float value )
{
    simd_setelem_ps(mVec128, value, idx);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & SimdPoint3::setElem( int idx, const SimdFloat & value )
{
    simd_setelem_ps(mVec128, value.get128(), idx);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdPoint3::getElem( int idx ) const
{
    return SimdFloat( mVec128, idx );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdPoint3::operator -( const SimdPoint3 & pnt ) const
{
    return SimdVector3( _mm_sub_ps( mVec128, pnt.mVec128 ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 SimdPoint3::operator +( const SimdVector3 & vec ) const
{
    return SimdPoint3( _mm_add_ps( mVec128, vec.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 SimdPoint3::operator -( const SimdVector3 & vec ) const
{
    return SimdPoint3( _mm_sub_ps( mVec128, vec.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & SimdPoint3::operator +=( const SimdVector3 & vec )
{
    *this = *this + vec;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & SimdPoint3::operator -=( const SimdVector3 & vec )
{
    *this = *this - vec;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 operator *( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1 )
{
    return SimdPoint3( _mm_mul_ps( pnt0.get128(), pnt1.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 operator /( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1 )
{
    return SimdPoint3( _mm_div_ps( pnt0.get128(), pnt1.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 inv( const SimdPoint3 & pnt )
{
    return SimdPoint3( _mm_rcp_ps( pnt.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 sqrt( const SimdPoint3 & pnt )
{
    return SimdPoint3( _mm_sqrt_ps( pnt.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 rsqrt( const SimdPoint3 & pnt )
{
    return SimdPoint3( _mm_rsqrt_ps( pnt.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 abs( const SimdPoint3 & pnt )
{
    return SimdPoint3( simd_abs_ps( pnt.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 copySign( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1 )
{
    __m128 vmask = simd_broadcast_ui_ps(0x7fffffff);
    return SimdPoint3( _mm_or_ps(
        _mm_and_ps   ( vmask, pnt0.get128() ),            // Value
        _mm_andnot_ps( vmask, pnt1.get128() ) ) );        // Signs
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 max( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1 )
{
    return SimdPoint3( _mm_max_ps( pnt0.get128(), pnt1.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat maxElem( const SimdPoint3 & pnt )
{
    return SimdFloat( _mm_max_ps( _mm_max_ps( simd_broadcast_ps( pnt.get128(), 0 ), simd_broadcast_ps( pnt.get128(), 1 ) ), simd_broadcast_ps( pnt.get128(), 2 ) ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 min( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1 )
{
    return SimdPoint3( _mm_min_ps( pnt0.get128(), pnt1.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat minElem( const SimdPoint3 & pnt )
{
    return SimdFloat( _mm_min_ps( _mm_min_ps( simd_broadcast_ps( pnt.get128(), 0 ), simd_broadcast_ps( pnt.get128(), 1 ) ), simd_broadcast_ps( pnt.get128(), 2 ) ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat sum( const SimdPoint3 & pnt )
{
    return SimdFloat( _mm_add_ps( _mm_add_ps( simd_broadcast_ps( pnt.get128(), 0 ), simd_broadcast_ps( pnt.get128(), 1 ) ), simd_broadcast_ps( pnt.get128(), 2 ) ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 scale( const SimdPoint3 & pnt, float scaleVal )
{
    return scale( pnt, SimdFloat( scaleVal ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 scale( const SimdPoint3 & pnt, const SimdFloat &scaleVal )
{
    return operator *( pnt, SimdPoint3( scaleVal ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 scale( const SimdPoint3 & pnt, const SimdVector3 & scaleVec )
{
    return operator *( pnt, SimdPoint3( scaleVec ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat projection( const SimdPoint3 & pnt, const SimdVector3 & unitVec )
{
    return SimdFloat( _vmathVfDot3( pnt.get128(), unitVec.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat distSqrFromOrigin( const SimdPoint3 & pnt )
{
    return lengthSqr( SimdVector3( pnt ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat distFromOrigin( const SimdPoint3 & pnt )
{
    return length( SimdVector3( pnt ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat distSqr( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1 )
{
    return lengthSqr( ( pnt1 - pnt0 ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat dist( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1 )
{
    return length( ( pnt1 - pnt0 ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 select( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1, bool select1 )
{
    return select( pnt0, pnt1, SimdBool(select1) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 select( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1, const SimdBool &select1 )
{
    return SimdPoint3( simd_select_ps( pnt0.get128(), pnt1.get128(), select1.get128() ) );
}

#ifdef _VECTORMATH_DEBUG

SIMD_VECTORMATH_FORCE_INLINE void print( const SimdPoint3 & pnt )
{
    union { __m128 v; float s[4]; } tmp;
    tmp.v = pnt.get128();
    printf( "( %f %f %f )\n", tmp.s[0], tmp.s[1], tmp.s[2] );
}

SIMD_VECTORMATH_FORCE_INLINE void print( const SimdPoint3 & pnt, const char * name )
{
    union { __m128 v; float s[4]; } tmp;
    tmp.v = pnt.get128();
    printf( "%s: ( %f %f %f )\n", name, tmp.s[0], tmp.s[1], tmp.s[2] );
}

#endif

} // namespace FmVectormath

#endif
