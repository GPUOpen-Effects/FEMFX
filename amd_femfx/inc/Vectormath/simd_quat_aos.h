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


#ifndef _SIMD_VECTORMATH_QUAT_AOS_CPP_H
#define _SIMD_VECTORMATH_QUAT_AOS_CPP_H

//-----------------------------------------------------------------------------
// Definitions

#ifndef _SIMD_VECTORMATH_INTERNAL_FUNCTIONS
#define _SIMD_VECTORMATH_INTERNAL_FUNCTIONS

#endif

namespace FmVectormath {

SIMD_VECTORMATH_FORCE_INLINE void SimdQuat::set128(__m128 vec)
{
    mVec128 = vec;
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat::SimdQuat( const SimdFloat &_x, const SimdFloat &_y, const SimdFloat &_z, const SimdFloat &_w )
{
    mVec128 = _mm_unpacklo_ps(
        _mm_unpacklo_ps( _x.get128(), _z.get128() ),
        _mm_unpacklo_ps( _y.get128(), _w.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat::SimdQuat( const SimdVector3 &xyz, float _w )
{
    mVec128 = xyz.get128();
    mVec128 = simd_insertw_ps(mVec128, _w);
}

SIMD_VECTORMATH_FORCE_INLINE  SimdQuat::SimdQuat(const SimdQuat& quat)
{
    mVec128 = quat.get128();
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat::SimdQuat( float _x, float _y, float _z, float _w )
{
    mVec128 = _mm_setr_ps(_x, _y, _z, _w);
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat::SimdQuat( const SimdVector3 &xyz, const SimdFloat &_w )
{
    mVec128 = xyz.get128();
    mVec128 = simd_insertw_ps(mVec128, _w.get128());
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat::SimdQuat( const SimdVector4 & vec )
{
    mVec128 = vec.get128();
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat::SimdQuat( float scalar )
{
    mVec128 = SimdFloat(scalar).get128();
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat::SimdQuat( const SimdFloat &scalar )
{
    mVec128 = scalar.get128();
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat::SimdQuat( __m128 vf4 )
{
    mVec128 = vf4;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat SimdQuat::identity( )
{
    return SimdQuat( _SIMD_VECTORMATH_UNIT_0001 );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat lerp( float t, const SimdQuat & quat0, const SimdQuat & quat1 )
{
    return lerp( SimdFloat(t), quat0, quat1 );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat lerp( const SimdFloat &t, const SimdQuat & quat0, const SimdQuat & quat1 )
{
    return ( quat0 + ( ( quat1 - quat0 ) * t ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat slerp( float t, const SimdQuat & unitSimdQuat0, const SimdQuat & unitSimdQuat1 )
{
    return slerp( SimdFloat(t), unitSimdQuat0, unitSimdQuat1 );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat slerp( const SimdFloat &t, const SimdQuat & unitSimdQuat0, const SimdQuat & unitSimdQuat1 )
{
    SimdQuat start;
    __m128 scales, scale0, scale1, cosAngle, angle, tttt, oneMinusT, angles, sines;
    __m128 selectMask;
    cosAngle = _vmathVfDot4( unitSimdQuat0.get128(), unitSimdQuat1.get128() );
    selectMask = (__m128)_mm_cmpgt_ps( _mm_setzero_ps(), cosAngle );
    cosAngle = simd_select_ps( cosAngle, simd_negate_ps( cosAngle ), selectMask );
    start = SimdQuat( simd_select_ps( unitSimdQuat0.get128(), simd_negate_ps( unitSimdQuat0.get128() ), selectMask ) );
    selectMask = (__m128)_mm_cmpgt_ps( _mm_set1_ps(_SIMD_VECTORMATH_SLERP_TOL), cosAngle );
    angle = simd_acos_ps( cosAngle );
    tttt = t.get128();
    oneMinusT = _mm_sub_ps( _mm_set1_ps(1.0f), tttt );
    angles = _mm_unpacklo_ps( _mm_set1_ps(1.0f), tttt );
    angles = _mm_unpacklo_ps( angles, oneMinusT );
    angles = simd_madd_ps( angles, angle, _mm_setzero_ps() );
    sines = simd_sin_ps( angles );
    scales = _mm_div_ps( sines, simd_broadcast_ps( sines, 0 ) );
    scale0 = simd_select_ps( oneMinusT, simd_broadcast_ps( scales, 1 ), selectMask );
    scale1 = simd_select_ps( tttt, simd_broadcast_ps( scales, 2 ), selectMask );
    return SimdQuat( simd_madd_ps( start.get128(), scale0, _mm_mul_ps( unitSimdQuat1.get128(), scale1 ) ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat squad( float t, const SimdQuat & unitSimdQuat0, const SimdQuat & unitSimdQuat1, const SimdQuat & unitSimdQuat2, const SimdQuat & unitSimdQuat3 )
{
    return squad( SimdFloat(t), unitSimdQuat0, unitSimdQuat1, unitSimdQuat2, unitSimdQuat3 );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat squad( const SimdFloat &t, const SimdQuat & unitSimdQuat0, const SimdQuat & unitSimdQuat1, const SimdQuat & unitSimdQuat2, const SimdQuat & unitSimdQuat3 )
{
    return slerp( ( ( SimdFloat(2.0f) * t ) * ( SimdFloat(1.0f) - t ) ), slerp( t, unitSimdQuat0, unitSimdQuat3 ), slerp( t, unitSimdQuat1, unitSimdQuat2 ) );
}

SIMD_VECTORMATH_FORCE_INLINE __m128 SimdQuat::get128( ) const
{
    return mVec128;
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat & SimdQuat::operator =( const SimdQuat & quat )
{
    mVec128 = quat.mVec128;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat & SimdQuat::setXYZ( const SimdVector3 & vec )
{
    SIMD_VECTORMATH_ALIGN16 unsigned int sw[4] = {0, 0, 0, 0xffffffff};
    mVec128 = simd_select_ps( vec.get128(), mVec128, sw );
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdQuat::getXYZ( ) const
{
    return SimdVector3( mVec128 );
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat & SimdQuat::setX( float _x )
{
    mVec128 = simd_insertx_ps(mVec128, _x);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat & SimdQuat::setX( const SimdFloat &_x )
{
    mVec128 = simd_insertx_ps(mVec128, _x.get128());
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdQuat::getX( ) const
{
    return SimdFloat(simd_broadcast_ps(mVec128, 0));
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat & SimdQuat::setY( float _y )
{
    mVec128 = simd_inserty_ps(mVec128, _y);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat & SimdQuat::setY( const SimdFloat &_y )
{
    mVec128 = simd_inserty_ps(mVec128, _y.get128());
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdQuat::getY( ) const
{
    return SimdFloat(simd_broadcast_ps(mVec128, 1));
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat & SimdQuat::setZ( float _z )
{
    mVec128 = simd_insertz_ps(mVec128, _z);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat & SimdQuat::setZ( const SimdFloat &_z )
{
    mVec128 = simd_insertz_ps(mVec128, _z.get128());
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdQuat::getZ( ) const
{
    return SimdFloat(simd_broadcast_ps(mVec128, 2));
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat & SimdQuat::setW( float _w )
{
    mVec128 = simd_insertw_ps(mVec128, _w);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat & SimdQuat::setW( const SimdFloat &_w )
{
    mVec128 = simd_insertw_ps(mVec128, _w.get128());
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdQuat::getW( ) const
{
    return SimdFloat(simd_broadcast_ps(mVec128, 3));
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat & SimdQuat::setElem( int idx, float value )
{
    simd_setelem_ps(mVec128, value, idx);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat & SimdQuat::setElem( int idx, const SimdFloat & value )
{
    simd_setelem_ps(mVec128, value.get128(), idx);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdQuat::getElem( int idx ) const
{
    return SimdFloat( mVec128, idx );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat SimdQuat::operator +( const SimdQuat & quat ) const
{
    return SimdQuat( _mm_add_ps( mVec128, quat.mVec128 ) );
}


SIMD_VECTORMATH_FORCE_INLINE const SimdQuat SimdQuat::operator -( const SimdQuat & quat ) const
{
    return SimdQuat( _mm_sub_ps( mVec128, quat.mVec128 ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat SimdQuat::operator *( float scalar ) const
{
    return *this * SimdFloat(scalar);
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat SimdQuat::operator *( const SimdFloat &scalar ) const
{
    return SimdQuat( _mm_mul_ps( mVec128, scalar.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat & SimdQuat::operator +=( const SimdQuat & quat )
{
    *this = *this + quat;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat & SimdQuat::operator -=( const SimdQuat & quat )
{
    *this = *this - quat;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat & SimdQuat::operator *=( float scalar )
{
    *this = *this * scalar;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat & SimdQuat::operator *=( const SimdFloat &scalar )
{
    *this = *this * scalar;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat SimdQuat::operator /( float scalar ) const
{
    return *this / SimdFloat(scalar);
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat SimdQuat::operator /( const SimdFloat &scalar ) const
{
    return SimdQuat( _mm_div_ps( mVec128, scalar.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat & SimdQuat::operator /=( float scalar )
{
    *this = *this / scalar;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdQuat & SimdQuat::operator /=( const SimdFloat &scalar )
{
    *this = *this / scalar;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat SimdQuat::operator -( ) const
{
    return SimdQuat(_mm_sub_ps( _mm_setzero_ps(), mVec128 ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat operator *( float scalar, const SimdQuat & quat )
{
    return SimdFloat(scalar) * quat;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat operator *( const SimdFloat &scalar, const SimdQuat & quat )
{
    return quat * scalar;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat dot( const SimdQuat & quat0, const SimdQuat & quat1 )
{
    return SimdFloat( _vmathVfDot4( quat0.get128(), quat1.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat norm( const SimdQuat & quat )
{
    return SimdFloat( _vmathVfDot4( quat.get128(), quat.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat length( const SimdQuat & quat )
{
    return SimdFloat( _mm_sqrt_ps(_vmathVfDot4( quat.get128(), quat.get128() )) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat normalize( const SimdQuat & quat )
{
    __m128 dot =_vmathVfDot4( quat.get128(), quat.get128());
    return SimdQuat( _mm_mul_ps( quat.get128(), simd_rsqrt_newtonraphson_ps( dot ) ) );
}


SIMD_VECTORMATH_FORCE_INLINE const SimdQuat SimdQuat::rotation( const SimdVector3 & unitVec0, const SimdVector3 & unitVec1 )
{
    SimdVector3 crossVec;
    __m128 cosAngle, cosAngleX2Plus2, recipCosHalfAngleX2, cosHalfAngleX2, res;
    cosAngle = _vmathVfDot3( unitVec0.get128(), unitVec1.get128() );
    cosAngleX2Plus2 = simd_madd_ps( cosAngle, _mm_set1_ps(2.0f), _mm_set1_ps(2.0f) );
    recipCosHalfAngleX2 = _mm_rsqrt_ps( cosAngleX2Plus2 );
    cosHalfAngleX2 = _mm_mul_ps( recipCosHalfAngleX2, cosAngleX2Plus2 );
    crossVec = cross( unitVec0, unitVec1 );
    res = _mm_mul_ps( crossVec.get128(), recipCosHalfAngleX2 );
    SIMD_VECTORMATH_ALIGN16 unsigned int sw[4] = {0, 0, 0, 0xffffffff};
    res = simd_select_ps( res, _mm_mul_ps( cosHalfAngleX2, _mm_set1_ps(0.5f) ), sw );
    return SimdQuat( res );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat SimdQuat::rotation( float radians, const SimdVector3 & unitVec )
{
    return rotation( SimdFloat(radians), unitVec );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat SimdQuat::rotation( const SimdFloat &radians, const SimdVector3 & unitVec )
{
    __m128 s, c, angle, res;
    angle = _mm_mul_ps( radians.get128(), _mm_set1_ps(0.5f) );
    simd_sincos_ps( angle, &s, &c );
    SIMD_VECTORMATH_ALIGN16 unsigned int sw[4] = {0, 0, 0, 0xffffffff};
    res = simd_select_ps( _mm_mul_ps( unitVec.get128(), s ), c, sw );
    return SimdQuat( res );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat SimdQuat::rotationX( float radians )
{
    return rotationX( SimdFloat(radians) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat SimdQuat::rotationX( const SimdFloat &radians )
{
    __m128 s, c, angle, res;
    angle = _mm_mul_ps( radians.get128(), _mm_set1_ps(0.5f) );
    simd_sincos_ps( angle, &s, &c );
    SIMD_VECTORMATH_ALIGN16 unsigned int xsw[4] = {0xffffffff, 0, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int wsw[4] = {0, 0, 0, 0xffffffff};
    res = simd_select_ps( _mm_setzero_ps(), s, xsw );
    res = simd_select_ps( res, c, wsw );
    return SimdQuat( res );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat SimdQuat::rotationY( float radians )
{
    return rotationY( SimdFloat(radians) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat SimdQuat::rotationY( const SimdFloat &radians )
{
    __m128 s, c, angle, res;
    angle = _mm_mul_ps( radians.get128(), _mm_set1_ps(0.5f) );
    simd_sincos_ps( angle, &s, &c );
    SIMD_VECTORMATH_ALIGN16 unsigned int ysw[4] = {0, 0xffffffff, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int wsw[4] = {0, 0, 0, 0xffffffff};
    res = simd_select_ps( _mm_setzero_ps(), s, ysw );
    res = simd_select_ps( res, c, wsw );
    return SimdQuat( res );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat SimdQuat::rotationZ( float radians )
{
    return rotationZ( SimdFloat(radians) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat SimdQuat::rotationZ( const SimdFloat &radians )
{
    __m128 s, c, angle, res;
    angle = _mm_mul_ps( radians.get128(), _mm_set1_ps(0.5f) );
    simd_sincos_ps( angle, &s, &c );
    SIMD_VECTORMATH_ALIGN16 unsigned int zsw[4] = {0, 0, 0xffffffff, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int wsw[4] = {0, 0, 0, 0xffffffff};
    res = simd_select_ps( _mm_setzero_ps(), s, zsw );
    res = simd_select_ps( res, c, wsw );
    return SimdQuat( res );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat mul(const SimdQuat& quat0, const SimdQuat & quat1 )
{
    __m128 ldata, rdata, qv, tmp0, tmp1, tmp2, tmp3;
    __m128 product, l_wxyz, r_wxyz, xy, qw;
    ldata = quat0.mVec128;
    rdata = quat1.mVec128;
    tmp0 = _mm_shuffle_ps( ldata, ldata, _MM_SHUFFLE(3,0,2,1) );
    tmp1 = _mm_shuffle_ps( rdata, rdata, _MM_SHUFFLE(3,1,0,2) );
    tmp2 = _mm_shuffle_ps( ldata, ldata, _MM_SHUFFLE(3,1,0,2) );
    tmp3 = _mm_shuffle_ps( rdata, rdata, _MM_SHUFFLE(3,0,2,1) );
    qv = _mm_mul_ps( simd_broadcast_ps( ldata, 3 ), rdata );
    qv = simd_madd_ps( simd_broadcast_ps( rdata, 3 ), ldata, qv );
    qv = simd_madd_ps( tmp0, tmp1, qv );
    qv = simd_fnmadd_ps( tmp2, tmp3, qv );
    product = _mm_mul_ps( ldata, rdata );
    l_wxyz = simd_ror_ps( ldata, 3 );
    r_wxyz = simd_ror_ps( rdata, 3 );
    qw = simd_fnmadd_ps( l_wxyz, r_wxyz, product );
    xy = simd_madd_ps( l_wxyz, r_wxyz, product );
    qw = _mm_sub_ps( qw, simd_ror_ps( xy, 2 ) );
    SIMD_VECTORMATH_ALIGN16 unsigned int sw[4] = {0, 0, 0, 0xffffffff};
    return SimdQuat( simd_select_ps( qv, qw, sw ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 rotate( const SimdQuat & quat, const SimdVector3 & vec )
{    __m128 qdata, vdata, product, tmp0, tmp1, tmp2, tmp3, wwww, qv, qw, res;
    qdata = quat.get128();
    vdata = vec.get128();
    tmp0 = _mm_shuffle_ps( qdata, qdata, _MM_SHUFFLE(3,0,2,1) );
    tmp1 = _mm_shuffle_ps( vdata, vdata, _MM_SHUFFLE(3,1,0,2) );
    tmp2 = _mm_shuffle_ps( qdata, qdata, _MM_SHUFFLE(3,1,0,2) );
    tmp3 = _mm_shuffle_ps( vdata, vdata, _MM_SHUFFLE(3,0,2,1) );
    wwww = simd_broadcast_ps( qdata, 3 );
    qv = _mm_mul_ps( wwww, vdata );
    qv = simd_madd_ps( tmp0, tmp1, qv );
    qv = simd_fnmadd_ps( tmp2, tmp3, qv );
    product = _mm_mul_ps( qdata, vdata );
    qw = simd_madd_ps( simd_ror_ps( qdata, 1 ), simd_ror_ps( vdata, 1 ), product );
    qw = _mm_add_ps( simd_ror_ps( product, 2 ), qw );
    tmp1 = _mm_shuffle_ps( qv, qv, _MM_SHUFFLE(3,1,0,2) );
    tmp3 = _mm_shuffle_ps( qv, qv, _MM_SHUFFLE(3,0,2,1) );
    res = _mm_mul_ps( simd_broadcast_ps( qw, 0 ), qdata );
    res = simd_madd_ps( wwww, qv, res );
    res = simd_madd_ps( tmp0, tmp1, res );
    res = simd_fnmadd_ps( tmp2, tmp3, res );
    return SimdVector3( res );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat conj( const SimdQuat & quat )
{
    SIMD_VECTORMATH_ALIGN16 unsigned int sw[4] = {0x80000000,0x80000000,0x80000000,0};
    return SimdQuat( _mm_xor_ps( quat.get128(), _mm_load_ps((float *)sw) ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat select( const SimdQuat & quat0, const SimdQuat & quat1, bool select1 )
{
    return select( quat0, quat1, SimdBool(select1) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdQuat select( const SimdQuat & quat0, const SimdQuat & quat1, const SimdBool &select1 )
{
    return SimdQuat( simd_select_ps( quat0.get128(), quat1.get128(), select1.get128() ) );
}

SIMD_VECTORMATH_FORCE_INLINE void loadXYZW(SimdQuat& quat, const float* fptr)
{
    quat = SimdQuat( simd_load4_unaligned_ps(fptr) );
}

SIMD_VECTORMATH_FORCE_INLINE void storeXYZW(const SimdQuat& quat, float* fptr)
{
    simd_store4_unaligned_ps(fptr, quat.get128());
}

#ifdef _VECTORMATH_DEBUG

SIMD_VECTORMATH_FORCE_INLINE void print( const SimdQuat & quat )
{
    union { __m128 v; float s[4]; } tmp;
    tmp.v = quat.get128();
    printf( "( %f %f %f %f )\n", tmp.s[0], tmp.s[1], tmp.s[2], tmp.s[3] );
}

SIMD_VECTORMATH_FORCE_INLINE void print( const SimdQuat & quat, const char * name )
{
    union { __m128 v; float s[4]; } tmp;
    tmp.v = quat.get128();
    printf( "%s: ( %f %f %f %f )\n", name, tmp.s[0], tmp.s[1], tmp.s[2], tmp.s[3] );
}

#endif

} // namespace FmVectormath

#endif
