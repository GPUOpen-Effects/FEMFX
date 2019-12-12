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


#ifndef _SIMD_VECTORMATH_MAT_AOS_CPP_H
#define _SIMD_VECTORMATH_MAT_AOS_CPP_H

namespace FmVectormath {

//-----------------------------------------------------------------------------
// Constants

#define _SIMD_VECTORMATH_PI_OVER_2 1.570796327f

//-----------------------------------------------------------------------------
// Definitions

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3::SimdMatrix3( const SimdMatrix3 & mat )
{
    col0 = mat.col0;
    col1 = mat.col1;
    col2 = mat.col2;
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3::SimdMatrix3( float scalar )
{
    col0 = SimdVector3( scalar );
    col1 = SimdVector3( scalar );
    col2 = SimdVector3( scalar );
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3::SimdMatrix3( const SimdFloat &scalar )
{
    col0 = SimdVector3( scalar );
    col1 = SimdVector3( scalar );
    col2 = SimdVector3( scalar );
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3::SimdMatrix3( const SimdQuat & unitSimdQuat )
{
    __m128 xyzw_2, wwww, yzxw, zxyw, yzxw_2, zxyw_2;
    __m128 tmp0, tmp1, tmp2, tmp3, tmp4, tmp5;
    SIMD_VECTORMATH_ALIGN16 unsigned int sx[4] = {0xffffffff, 0, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int sz[4] = {0, 0, 0xffffffff, 0};
    __m128 select_x = _mm_load_ps((float *)sx);
    __m128 select_z = _mm_load_ps((float *)sz);

    xyzw_2 = _mm_add_ps( unitSimdQuat.get128(), unitSimdQuat.get128() );
    wwww = _mm_shuffle_ps( unitSimdQuat.get128(), unitSimdQuat.get128(), _MM_SHUFFLE(3,3,3,3) );
    yzxw = _mm_shuffle_ps( unitSimdQuat.get128(), unitSimdQuat.get128(), _MM_SHUFFLE(3,0,2,1) );
    zxyw = _mm_shuffle_ps( unitSimdQuat.get128(), unitSimdQuat.get128(), _MM_SHUFFLE(3,1,0,2) );
    yzxw_2 = _mm_shuffle_ps( xyzw_2, xyzw_2, _MM_SHUFFLE(3,0,2,1) );
    zxyw_2 = _mm_shuffle_ps( xyzw_2, xyzw_2, _MM_SHUFFLE(3,1,0,2) );

    tmp0 = _mm_mul_ps( yzxw_2, wwww );                                    // tmp0 = 2yw, 2zw, 2xw, 2w2
    tmp1 = _mm_sub_ps( _mm_set1_ps(1.0f), _mm_mul_ps(yzxw, yzxw_2) );    // tmp1 = 1 - 2y2, 1 - 2z2, 1 - 2x2, 1 - 2w2
    tmp2 = _mm_mul_ps( yzxw, xyzw_2 );                                    // tmp2 = 2xy, 2yz, 2xz, 2w2
    tmp0 = _mm_add_ps( _mm_mul_ps(zxyw, xyzw_2), tmp0 );                // tmp0 = 2yw + 2zx, 2zw + 2xy, 2xw + 2yz, 2w2 + 2w2
    tmp1 = _mm_sub_ps( tmp1, _mm_mul_ps(zxyw, zxyw_2) );                // tmp1 = 1 - 2y2 - 2z2, 1 - 2z2 - 2x2, 1 - 2x2 - 2y2, 1 - 2w2 - 2w2
    tmp2 = _mm_sub_ps( tmp2, _mm_mul_ps(zxyw_2, wwww) );                // tmp2 = 2xy - 2zw, 2yz - 2xw, 2xz - 2yw, 2w2 -2w2

    tmp3 = simd_select_ps( tmp0, tmp1, select_x );
    tmp4 = simd_select_ps( tmp1, tmp2, select_x );
    tmp5 = simd_select_ps( tmp2, tmp0, select_x );
    col0 = SimdVector3( simd_select_ps( tmp3, tmp2, select_z ) );
    col1 = SimdVector3( simd_select_ps( tmp4, tmp0, select_z ) );
    col2 = SimdVector3( simd_select_ps( tmp5, tmp1, select_z ) );
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3::SimdMatrix3( const SimdVector3 &_col0, const SimdVector3 &_col1, const SimdVector3 &_col2 )
{
    col0 = _col0;
    col1 = _col1;
    col2 = _col2;
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & SimdMatrix3::setCol0( const SimdVector3 &_col0 )
{
    col0 = _col0;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & SimdMatrix3::setCol1( const SimdVector3 &_col1 )
{
    col1 = _col1;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & SimdMatrix3::setCol2( const SimdVector3 &_col2 )
{
    col2 = _col2;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & SimdMatrix3::setCol( int col, const SimdVector3 & vec )
{
    *(&col0 + col) = vec;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & SimdMatrix3::setRow( int row, const SimdVector3 & vec )
{
    col0.setElem( row, vec.getElem( 0 ) );
    col1.setElem( row, vec.getElem( 1 ) );
    col2.setElem( row, vec.getElem( 2 ) );
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & SimdMatrix3::setElem( int col, int row, float val )
{
    SimdVector3 colVec = getCol(col);
    colVec.setElem(row, val);
    setCol(col, colVec);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & SimdMatrix3::setElem( int col, int row, const SimdFloat & val )
{
    SimdVector3 tmpV3_0;
    tmpV3_0 = this->getCol( col );
    tmpV3_0.setElem( row, val );
    this->setCol( col, tmpV3_0 );
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdMatrix3::getElem( int col, int row ) const
{
    return this->getCol( col ).getElem( row );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdMatrix3::getCol0( ) const
{
    return col0;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdMatrix3::getCol1( ) const
{
    return col1;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdMatrix3::getCol2( ) const
{
    return col2;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdMatrix3::getCol( int col ) const
{
    return *(&col0 + col);
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdMatrix3::getRow( int row ) const
{
    return SimdVector3( col0.getElem( row ), col1.getElem( row ), col2.getElem( row ) );
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & SimdMatrix3::operator =( const SimdMatrix3 & mat )
{
    col0 = mat.col0;
    col1 = mat.col1;
    col2 = mat.col2;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 transpose( const SimdMatrix3 & mat )
{
    __m128 tmp0, tmp1, res0, res1, res2;
    tmp0 = _mm_unpacklo_ps( mat.getCol0().get128(), mat.getCol2().get128() );
    tmp1 = _mm_unpackhi_ps( mat.getCol0().get128(), mat.getCol2().get128() );
    res0 = _mm_unpacklo_ps( tmp0, mat.getCol1().get128() );
    //res1 = vec_perm( tmp0, mat.getCol1().get128(), _SIMD_VECTORMATH_PERM_ZBWX );
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    res1 = _mm_shuffle_ps( tmp0, tmp0, _MM_SHUFFLE(0,3,2,2));
    res1 = simd_select_ps(res1, mat.getCol1().get128(), select_y);
    //res2 = vec_perm( tmp1, mat.getCol1().get128(), _SIMD_VECTORMATH_PERM_XCYX );
    res2 = _mm_shuffle_ps( tmp1, tmp1, _MM_SHUFFLE(0,1,1,0));
    res2 = simd_select_ps(res2, simd_broadcast_ps(mat.getCol1().get128(), 2), select_y);
    return SimdMatrix3(
        SimdVector3( res0 ),
        SimdVector3( res1 ),
        SimdVector3( res2 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 inverse( const SimdMatrix3 & mat )
{
    __m128 tmp0, tmp1, tmp2, tmp3, tmp4, dot, invdet, inv0, inv1, inv2;
    tmp2 = _vmathVfCross( mat.getCol0().get128(), mat.getCol1().get128() );
    tmp0 = _vmathVfCross( mat.getCol1().get128(), mat.getCol2().get128() );
    tmp1 = _vmathVfCross( mat.getCol2().get128(), mat.getCol0().get128() );
    dot = _vmathVfDot3( tmp2, mat.getCol2().get128() );
    dot = simd_broadcast_ps( dot, 0 );
    invdet = _mm_rcp_ps( dot );
    tmp3 = _mm_unpacklo_ps( tmp0, tmp2 );
    tmp4 = _mm_unpackhi_ps( tmp0, tmp2 );
    inv0 = _mm_unpacklo_ps( tmp3, tmp1 );
    //inv1 = vec_perm( tmp3, tmp1, _SIMD_VECTORMATH_PERM_ZBWX );
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    inv1 = _mm_shuffle_ps( tmp3, tmp3, _MM_SHUFFLE(0,3,2,2));
    inv1 = simd_select_ps(inv1, tmp1, select_y);
    //inv2 = vec_perm( tmp4, tmp1, _SIMD_VECTORMATH_PERM_XCYX );
    inv2 = _mm_shuffle_ps( tmp4, tmp4, _MM_SHUFFLE(0,1,1,0));
    inv2 = simd_select_ps(inv2, simd_broadcast_ps(tmp1, 2), select_y);
    inv0 = _mm_mul_ps( inv0, invdet );
    inv1 = _mm_mul_ps( inv1, invdet );
    inv2 = _mm_mul_ps( inv2, invdet );
    return SimdMatrix3(
        SimdVector3( inv0 ),
        SimdVector3( inv1 ),
        SimdVector3( inv2 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat determinant( const SimdMatrix3 & mat )
{
    return dot( mat.getCol2(), cross( mat.getCol0(), mat.getCol1() ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 SimdMatrix3::operator +( const SimdMatrix3 & mat ) const
{
    return SimdMatrix3(
        ( col0 + mat.col0 ),
        ( col1 + mat.col1 ),
        ( col2 + mat.col2 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 SimdMatrix3::operator -( const SimdMatrix3 & mat ) const
{
    return SimdMatrix3(
        ( col0 - mat.col0 ),
        ( col1 - mat.col1 ),
        ( col2 - mat.col2 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & SimdMatrix3::operator +=( const SimdMatrix3 & mat )
{
    *this = *this + mat;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & SimdMatrix3::operator -=( const SimdMatrix3 & mat )
{
    *this = *this - mat;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 SimdMatrix3::operator -( ) const
{
    return SimdMatrix3(
        ( -col0 ),
        ( -col1 ),
        ( -col2 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 abs( const SimdMatrix3 & mat )
{
    return SimdMatrix3(
        abs( mat.getCol0() ),
        abs( mat.getCol1() ),
        abs( mat.getCol2() )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 SimdMatrix3::operator *( float scalar ) const
{
    return *this * SimdFloat(scalar);
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 SimdMatrix3::operator *( const SimdFloat &scalar ) const
{
    return SimdMatrix3(
        ( col0 * scalar ),
        ( col1 * scalar ),
        ( col2 * scalar )
    );
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & SimdMatrix3::operator *=( float scalar )
{
    return *this *= SimdFloat(scalar);
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & SimdMatrix3::operator *=( const SimdFloat &scalar )
{
    *this = *this * scalar;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 operator *( float scalar, const SimdMatrix3 & mat )
{
    return SimdFloat(scalar) * mat;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 operator *( const SimdFloat &scalar, const SimdMatrix3 & mat )
{
    return mat * scalar;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 mul(const SimdMatrix3 & mat, const SimdVector3 & vec )
{
    __m128 res;
    __m128 xxxx, yyyy, zzzz;
    xxxx = simd_broadcast_ps( vec.get128(), 0 );
    yyyy = simd_broadcast_ps( vec.get128(), 1 );
    zzzz = simd_broadcast_ps( vec.get128(), 2 );
    res = _mm_mul_ps( mat.col0.get128(), xxxx );
    res = simd_madd_ps( mat.col1.get128(), yyyy, res );
    res = simd_madd_ps( mat.col2.get128(), zzzz, res );
    return SimdVector3( res );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 mul(const SimdMatrix3 & mat0, const SimdMatrix3 & mat1 ) 
{
    return SimdMatrix3(
        mul( mat0, mat1.col0 ),
        mul( mat0, mat1.col1 ),
        mul( mat0, mat1.col2 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 SimdMatrix3::identity( )
{
    return SimdMatrix3(
        SimdVector3::xAxis( ),
        SimdVector3::yAxis( ),
        SimdVector3::zAxis( )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 SimdMatrix3::rotationX( float radians )
{
    return rotationX( SimdFloat(radians) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 SimdMatrix3::rotationX( const SimdFloat &radians )
{
    __m128 s, c, res1, res2;
    __m128 zero;
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_z[4] = {0, 0, 0xffffffff, 0};
    zero = _mm_setzero_ps();
    simd_sincos_ps( radians.get128(), &s, &c );
    res1 = simd_select_ps( zero, c, select_y );
    res1 = simd_select_ps( res1, s, select_z );
    res2 = simd_select_ps( zero, simd_negate_ps(s), select_y );
    res2 = simd_select_ps( res2, c, select_z );
    return SimdMatrix3(
        SimdVector3::xAxis( ),
        SimdVector3( res1 ),
        SimdVector3( res2 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 SimdMatrix3::rotationY( float radians )
{
    return rotationY( SimdFloat(radians) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 SimdMatrix3::rotationY( const SimdFloat &radians )
{
    __m128 s, c, res0, res2;
    __m128 zero;
    SIMD_VECTORMATH_ALIGN16 unsigned int select_x[4] = {0xffffffff, 0, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_z[4] = {0, 0, 0xffffffff, 0};
    zero = _mm_setzero_ps();
    simd_sincos_ps( radians.get128(), &s, &c );
    res0 = simd_select_ps( zero, c, select_x );
    res0 = simd_select_ps( res0, simd_negate_ps(s), select_z );
    res2 = simd_select_ps( zero, s, select_x );
    res2 = simd_select_ps( res2, c, select_z );
    return SimdMatrix3(
        SimdVector3( res0 ),
        SimdVector3::yAxis( ),
        SimdVector3( res2 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 SimdMatrix3::rotationZ( float radians )
{
    return rotationZ( SimdFloat(radians) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 SimdMatrix3::rotationZ( const SimdFloat &radians )
{
    __m128 s, c, res0, res1;
    __m128 zero;
    SIMD_VECTORMATH_ALIGN16 unsigned int select_x[4] = {0xffffffff, 0, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    zero = _mm_setzero_ps();
    simd_sincos_ps( radians.get128(), &s, &c );
    res0 = simd_select_ps( zero, c, select_x );
    res0 = simd_select_ps( res0, s, select_y );
    res1 = simd_select_ps( zero, simd_negate_ps(s), select_x );
    res1 = simd_select_ps( res1, c, select_y );
    return SimdMatrix3(
        SimdVector3( res0 ),
        SimdVector3( res1 ),
        SimdVector3::zAxis( )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 SimdMatrix3::rotationZYX( const SimdVector3 &radiansXYZ )
{
    __m128 angles, s, negS, c, X0, X1, Y0, Y1, Z0, Z1, tmp;
    angles = SimdVector4( radiansXYZ, 0.0f ).get128();
    simd_sincos_ps( angles, &s, &c );
    negS = simd_negate_ps( s );
    Z0 = _mm_unpackhi_ps( c, s );
    Z1 = _mm_unpackhi_ps( negS, c );
    SIMD_VECTORMATH_ALIGN16 unsigned int select_xyz[4] = {0xffffffff, 0xffffffff, 0xffffffff, 0};
    Z1 = _mm_and_ps( Z1, _mm_load_ps( (float *)select_xyz ) );
    Y0 = _mm_shuffle_ps( c, negS, _MM_SHUFFLE(0,1,1,1) );
    Y1 = _mm_shuffle_ps( s, c, _MM_SHUFFLE(0,1,1,1) );
    X0 = simd_broadcast_ps( s, 0 );
    X1 = simd_broadcast_ps( c, 0 );
    tmp = _mm_mul_ps( Z0, Y1 );
    return SimdMatrix3(
        SimdVector3( _mm_mul_ps( Z0, Y0 ) ),
        SimdVector3( simd_madd_ps( Z1, X1, _mm_mul_ps( tmp, X0 ) ) ),
        SimdVector3( simd_fnmadd_ps( Z1, X0, _mm_mul_ps( tmp, X1 ) ) )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 SimdMatrix3::rotation( float radians, const SimdVector3 & unitVec )
{
    return rotation( SimdFloat(radians), unitVec );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 SimdMatrix3::rotation( const SimdFloat &radians, const SimdVector3 & unitVec )
{
    __m128 axis, s, c, oneMinusC, axisS, negAxisS, xxxx, yyyy, zzzz, tmp0, tmp1, tmp2;
    axis = unitVec.get128();
    simd_sincos_ps( radians.get128(), &s, &c );
    xxxx = simd_broadcast_ps( axis, 0 );
    yyyy = simd_broadcast_ps( axis, 1 );
    zzzz = simd_broadcast_ps( axis, 2 );
    oneMinusC = _mm_sub_ps( _mm_set1_ps(1.0f), c );
    axisS = _mm_mul_ps( axis, s );
    negAxisS = simd_negate_ps( axisS );
    SIMD_VECTORMATH_ALIGN16 unsigned int select_x[4] = {0xffffffff, 0, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_z[4] = {0, 0, 0xffffffff, 0};
    //tmp0 = vec_perm( axisS, negAxisS, _SIMD_VECTORMATH_PERM_XZBX );
    tmp0 = _mm_shuffle_ps( axisS, axisS, _MM_SHUFFLE(0,0,2,0) );
    tmp0 = simd_select_ps(tmp0, simd_broadcast_ps(negAxisS, 1), select_z);
    //tmp1 = vec_perm( axisS, negAxisS, _SIMD_VECTORMATH_PERM_CXXX );
    tmp1 = simd_select_ps( simd_broadcast_ps(axisS, 0), simd_broadcast_ps(negAxisS, 2), select_x );
    //tmp2 = vec_perm( axisS, negAxisS, _SIMD_VECTORMATH_PERM_YAXX );
    tmp2 = _mm_shuffle_ps( axisS, axisS, _MM_SHUFFLE(0,0,0,1) );
    tmp2 = simd_select_ps(tmp2, simd_broadcast_ps(negAxisS, 0), select_y);
    tmp0 = simd_select_ps( tmp0, c, select_x );
    tmp1 = simd_select_ps( tmp1, c, select_y );
    tmp2 = simd_select_ps( tmp2, c, select_z );
    return SimdMatrix3(
        SimdVector3( simd_madd_ps( _mm_mul_ps( axis, xxxx ), oneMinusC, tmp0 ) ),
        SimdVector3( simd_madd_ps( _mm_mul_ps( axis, yyyy ), oneMinusC, tmp1 ) ),
        SimdVector3( simd_madd_ps( _mm_mul_ps( axis, zzzz ), oneMinusC, tmp2 ) )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 SimdMatrix3::rotation( const SimdQuat & unitSimdQuat )
{
    return SimdMatrix3( unitSimdQuat );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 SimdMatrix3::scale( const SimdVector3 & scaleVec )
{
    __m128 zero = _mm_setzero_ps();
    SIMD_VECTORMATH_ALIGN16 unsigned int select_x[4] = {0xffffffff, 0, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_z[4] = {0, 0, 0xffffffff, 0};
    return SimdMatrix3(
        SimdVector3( simd_select_ps( zero, scaleVec.get128(), select_x ) ),
        SimdVector3( simd_select_ps( zero, scaleVec.get128(), select_y ) ),
        SimdVector3( simd_select_ps( zero, scaleVec.get128(), select_z ) )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 appendScale( const SimdMatrix3 & mat, const SimdVector3 & scaleVec )
{
    return SimdMatrix3(
        ( mat.getCol0() * scaleVec.getX( ) ),
        ( mat.getCol1() * scaleVec.getY( ) ),
        ( mat.getCol2() * scaleVec.getZ( ) )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 prependScale( const SimdVector3 & scaleVec, const SimdMatrix3 & mat )
{
    return SimdMatrix3(
        operator *( mat.getCol0(), scaleVec ),
        operator *( mat.getCol1(), scaleVec ),
        operator *( mat.getCol2(), scaleVec )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 select( const SimdMatrix3 & mat0, const SimdMatrix3 & mat1, bool select1 )
{
    return SimdMatrix3(
        select( mat0.getCol0(), mat1.getCol0(), select1 ),
        select( mat0.getCol1(), mat1.getCol1(), select1 ),
        select( mat0.getCol2(), mat1.getCol2(), select1 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 select( const SimdMatrix3 & mat0, const SimdMatrix3 & mat1, const SimdBool &select1 )
{
    return SimdMatrix3(
        select( mat0.getCol0(), mat1.getCol0(), select1 ),
        select( mat0.getCol1(), mat1.getCol1(), select1 ),
        select( mat0.getCol2(), mat1.getCol2(), select1 )
    );
}

#ifdef _VECTORMATH_DEBUG

SIMD_VECTORMATH_FORCE_INLINE void print( const SimdMatrix3 & mat )
{
    print( mat.getRow( 0 ) );
    print( mat.getRow( 1 ) );
    print( mat.getRow( 2 ) );
}

SIMD_VECTORMATH_FORCE_INLINE void print( const SimdMatrix3 & mat, const char * name )
{
    printf("%s:\n", name);
    print( mat );
}

#endif

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4::SimdMatrix4( const SimdMatrix4 & mat )
{
    col0 = mat.col0;
    col1 = mat.col1;
    col2 = mat.col2;
    col3 = mat.col3;
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4::SimdMatrix4( float scalar )
{
    col0 = SimdVector4( scalar );
    col1 = SimdVector4( scalar );
    col2 = SimdVector4( scalar );
    col3 = SimdVector4( scalar );
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4::SimdMatrix4( const SimdFloat &scalar )
{
    col0 = SimdVector4( scalar );
    col1 = SimdVector4( scalar );
    col2 = SimdVector4( scalar );
    col3 = SimdVector4( scalar );
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4::SimdMatrix4( const SimdTransform3 & mat )
{
    col0 = SimdVector4( mat.getCol0(), 0.0f );
    col1 = SimdVector4( mat.getCol1(), 0.0f );
    col2 = SimdVector4( mat.getCol2(), 0.0f );
    col3 = SimdVector4( mat.getCol3(), 1.0f );
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4::SimdMatrix4( const SimdVector4 &_col0, const SimdVector4 &_col1, const SimdVector4 &_col2, const SimdVector4 &_col3 )
{
    col0 = _col0;
    col1 = _col1;
    col2 = _col2;
    col3 = _col3;
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4::SimdMatrix4( const SimdMatrix3 & mat, const SimdVector3 &translateVec )
{
    col0 = SimdVector4( mat.getCol0(), 0.0f );
    col1 = SimdVector4( mat.getCol1(), 0.0f );
    col2 = SimdVector4( mat.getCol2(), 0.0f );
    col3 = SimdVector4( translateVec, 1.0f );
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4::SimdMatrix4( const SimdQuat & unitSimdQuat, const SimdVector3 &translateVec )
{
    SimdMatrix3 mat;
    mat = SimdMatrix3( unitSimdQuat );
    col0 = SimdVector4( mat.getCol0(), 0.0f );
    col1 = SimdVector4( mat.getCol1(), 0.0f );
    col2 = SimdVector4( mat.getCol2(), 0.0f );
    col3 = SimdVector4( translateVec, 1.0f );
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & SimdMatrix4::setCol0( const SimdVector4 &_col0 )
{
    col0 = _col0;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & SimdMatrix4::setCol1( const SimdVector4 &_col1 )
{
    col1 = _col1;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & SimdMatrix4::setCol2( const SimdVector4 &_col2 )
{
    col2 = _col2;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & SimdMatrix4::setCol3( const SimdVector4 &_col3 )
{
    col3 = _col3;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & SimdMatrix4::setCol( int col, const SimdVector4 & vec )
{
    *(&col0 + col) = vec;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & SimdMatrix4::setRow( int row, const SimdVector4 & vec )
{
    col0.setElem( row, vec.getElem( 0 ) );
    col1.setElem( row, vec.getElem( 1 ) );
    col2.setElem( row, vec.getElem( 2 ) );
    col3.setElem( row, vec.getElem( 3 ) );
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & SimdMatrix4::setElem( int col, int row, float val )
{
    SimdVector4 colVec = getCol(col);
    colVec.setElem(row, val);
    setCol(col, colVec);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & SimdMatrix4::setElem( int col, int row, const SimdFloat & val )
{
    SimdVector4 tmpV3_0;
    tmpV3_0 = this->getCol( col );
    tmpV3_0.setElem( row, val );
    this->setCol( col, tmpV3_0 );
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdMatrix4::getElem( int col, int row ) const
{
    return this->getCol( col ).getElem( row );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 SimdMatrix4::getCol0( ) const
{
    return col0;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 SimdMatrix4::getCol1( ) const
{
    return col1;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 SimdMatrix4::getCol2( ) const
{
    return col2;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 SimdMatrix4::getCol3( ) const
{
    return col3;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 SimdMatrix4::getCol( int col ) const
{
    return *(&col0 + col);
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 SimdMatrix4::getRow( int row ) const
{
    return SimdVector4( col0.getElem( row ), col1.getElem( row ), col2.getElem( row ), col3.getElem( row ) );
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & SimdMatrix4::operator =( const SimdMatrix4 & mat )
{
    col0 = mat.col0;
    col1 = mat.col1;
    col2 = mat.col2;
    col3 = mat.col3;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 transpose( const SimdMatrix4 & mat )
{
    __m128 tmp0, tmp1, tmp2, tmp3, res0, res1, res2, res3;
    tmp0 = _mm_unpacklo_ps( mat.getCol0().get128(), mat.getCol2().get128() );
    tmp1 = _mm_unpacklo_ps( mat.getCol1().get128(), mat.getCol3().get128() );
    tmp2 = _mm_unpackhi_ps( mat.getCol0().get128(), mat.getCol2().get128() );
    tmp3 = _mm_unpackhi_ps( mat.getCol1().get128(), mat.getCol3().get128() );
    res0 = _mm_unpacklo_ps( tmp0, tmp1 );
    res1 = _mm_unpackhi_ps( tmp0, tmp1 );
    res2 = _mm_unpacklo_ps( tmp2, tmp3 );
    res3 = _mm_unpackhi_ps( tmp2, tmp3 );
    return SimdMatrix4(
        SimdVector4( res0 ),
        SimdVector4( res1 ),
        SimdVector4( res2 ),
        SimdVector4( res3 )
    );
}

// TODO: Tidy
static SIMD_VECTORMATH_ALIGN16 const unsigned int _vmathPNPN[4] = {0x00000000, 0x80000000, 0x00000000, 0x80000000};
static SIMD_VECTORMATH_ALIGN16 const unsigned int _vmathNPNP[4] = {0x80000000, 0x00000000, 0x80000000, 0x00000000};
static SIMD_VECTORMATH_ALIGN16 const float _vmathZERONE[4] = {1.0f, 0.0f, 0.0f, 1.0f};

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 inverse( const SimdMatrix4 & mat )
{
    __m128 Va,Vb,Vc;
    __m128 r1,r2,r3,tt,tt2;
    __m128 sum,Det,RDet;
    __m128 trns0,trns1,trns2,trns3;

    __m128 _L1 = mat.getCol0().get128();
    __m128 _L2 = mat.getCol1().get128();
    __m128 _L3 = mat.getCol2().get128();
    __m128 _L4 = mat.getCol3().get128();
    // Calculating the minterms for the first line.

    // simd_ror_ps is just a macro using _mm_shuffle_ps().
    tt = _L4; tt2 = simd_ror_ps(_L3,1); 
    Vc = _mm_mul_ps(tt2,simd_ror_ps(tt,0));                    // V3'dot V4
    Va = _mm_mul_ps(tt2,simd_ror_ps(tt,2));                    // V3'dot V4"
    Vb = _mm_mul_ps(tt2,simd_ror_ps(tt,3));                    // V3' dot V4^

    r1 = _mm_sub_ps(simd_ror_ps(Va,1),simd_ror_ps(Vc,2));        // V3" dot V4^ - V3^ dot V4"
    r2 = _mm_sub_ps(simd_ror_ps(Vb,2),simd_ror_ps(Vb,0));        // V3^ dot V4' - V3' dot V4^
    r3 = _mm_sub_ps(simd_ror_ps(Va,0),simd_ror_ps(Vc,1));        // V3' dot V4" - V3" dot V4'

    tt = _L2;
    Va = simd_ror_ps(tt,1);        sum = _mm_mul_ps(Va,r1);
    Vb = simd_ror_ps(tt,2);        sum = _mm_add_ps(sum,_mm_mul_ps(Vb,r2));
    Vc = simd_ror_ps(tt,3);        sum = _mm_add_ps(sum,_mm_mul_ps(Vc,r3));

    // Calculating the determinant.
    Det = _mm_mul_ps(sum,_L1);
    Det = _mm_add_ps(Det,_mm_movehl_ps(Det,Det));

    const __m128 Sign_PNPN = _mm_load_ps((float *)_vmathPNPN);
    const __m128 Sign_NPNP = _mm_load_ps((float *)_vmathNPNP);

    __m128 mtL1 = _mm_xor_ps(sum,Sign_PNPN);

    // Calculating the minterms of the second line (using previous results).
    tt = simd_ror_ps(_L1,1);        sum = _mm_mul_ps(tt,r1);
    tt = simd_ror_ps(tt,1);        sum = _mm_add_ps(sum,_mm_mul_ps(tt,r2));
    tt = simd_ror_ps(tt,1);        sum = _mm_add_ps(sum,_mm_mul_ps(tt,r3));
    __m128 mtL2 = _mm_xor_ps(sum,Sign_NPNP);

    // Testing the determinant.
    Det = _mm_sub_ss(Det,_mm_shuffle_ps(Det,Det,1));

    // Calculating the minterms of the third line.
    tt = simd_ror_ps(_L1,1);
    Va = _mm_mul_ps(tt,Vb);                                    // V1' dot V2"
    Vb = _mm_mul_ps(tt,Vc);                                    // V1' dot V2^
    Vc = _mm_mul_ps(tt,_L2);                                // V1' dot V2

    r1 = _mm_sub_ps(simd_ror_ps(Va,1),simd_ror_ps(Vc,2));        // V1" dot V2^ - V1^ dot V2"
    r2 = _mm_sub_ps(simd_ror_ps(Vb,2),simd_ror_ps(Vb,0));        // V1^ dot V2' - V1' dot V2^
    r3 = _mm_sub_ps(simd_ror_ps(Va,0),simd_ror_ps(Vc,1));        // V1' dot V2" - V1" dot V2'

    tt = simd_ror_ps(_L4,1);        sum = _mm_mul_ps(tt,r1);
    tt = simd_ror_ps(tt,1);        sum = _mm_add_ps(sum,_mm_mul_ps(tt,r2));
    tt = simd_ror_ps(tt,1);        sum = _mm_add_ps(sum,_mm_mul_ps(tt,r3));
    __m128 mtL3 = _mm_xor_ps(sum,Sign_PNPN);

    // Dividing is FASTER than rcp_nr! (Because rcp_nr causes many register-memory RWs).
    RDet = _mm_div_ss(_mm_load_ss((float *)&_vmathZERONE), Det); // TODO: just 1.0f?
    RDet = _mm_shuffle_ps(RDet,RDet,0x00);

    // Devide the first 12 minterms with the determinant.
    mtL1 = _mm_mul_ps(mtL1, RDet);
    mtL2 = _mm_mul_ps(mtL2, RDet);
    mtL3 = _mm_mul_ps(mtL3, RDet);

    // Calculate the minterms of the forth line and devide by the determinant.
    tt = simd_ror_ps(_L3,1);        sum = _mm_mul_ps(tt,r1);
    tt = simd_ror_ps(tt,1);        sum = _mm_add_ps(sum,_mm_mul_ps(tt,r2));
    tt = simd_ror_ps(tt,1);        sum = _mm_add_ps(sum,_mm_mul_ps(tt,r3));
    __m128 mtL4 = _mm_xor_ps(sum,Sign_NPNP);
    mtL4 = _mm_mul_ps(mtL4, RDet);

    // Now we just have to transpose the minterms matrix.
    trns0 = _mm_unpacklo_ps(mtL1,mtL2);
    trns1 = _mm_unpacklo_ps(mtL3,mtL4);
    trns2 = _mm_unpackhi_ps(mtL1,mtL2);
    trns3 = _mm_unpackhi_ps(mtL3,mtL4);
    _L1 = _mm_movelh_ps(trns0,trns1);
    _L2 = _mm_movehl_ps(trns1,trns0);
    _L3 = _mm_movelh_ps(trns2,trns3);
    _L4 = _mm_movehl_ps(trns3,trns2);

    return SimdMatrix4(
        SimdVector4( _L1 ),
        SimdVector4( _L2 ),
        SimdVector4( _L3 ),
        SimdVector4( _L4 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 affineInverse( const SimdMatrix4 & mat )
{
    SimdTransform3 affineMat;
    affineMat.setCol0( mat.getCol0().getXYZ( ) );
    affineMat.setCol1( mat.getCol1().getXYZ( ) );
    affineMat.setCol2( mat.getCol2().getXYZ( ) );
    affineMat.setCol3( mat.getCol3().getXYZ( ) );
    return SimdMatrix4( inverse( affineMat ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 orthoInverse( const SimdMatrix4 & mat )
{
    SimdTransform3 affineMat;
    affineMat.setCol0( mat.getCol0().getXYZ( ) );
    affineMat.setCol1( mat.getCol1().getXYZ( ) );
    affineMat.setCol2( mat.getCol2().getXYZ( ) );
    affineMat.setCol3( mat.getCol3().getXYZ( ) );
    return SimdMatrix4( orthoInverse( affineMat ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat determinant( const SimdMatrix4 & mat )
{
    __m128 Va,Vb,Vc;
    __m128 r1,r2,r3,tt,tt2;
    __m128 sum,Det;

    __m128 _L1 = mat.getCol0().get128();
    __m128 _L2 = mat.getCol1().get128();
    __m128 _L3 = mat.getCol2().get128();
    __m128 _L4 = mat.getCol3().get128();
    // Calculating the minterms for the first line.

    // simd_ror_ps is just a macro using _mm_shuffle_ps().
    tt = _L4; tt2 = simd_ror_ps(_L3,1); 
    Vc = _mm_mul_ps(tt2,simd_ror_ps(tt,0));                    // V3' dot V4
    Va = _mm_mul_ps(tt2,simd_ror_ps(tt,2));                    // V3' dot V4"
    Vb = _mm_mul_ps(tt2,simd_ror_ps(tt,3));                    // V3' dot V4^

    r1 = _mm_sub_ps(simd_ror_ps(Va,1),simd_ror_ps(Vc,2));        // V3" dot V4^ - V3^ dot V4"
    r2 = _mm_sub_ps(simd_ror_ps(Vb,2),simd_ror_ps(Vb,0));        // V3^ dot V4' - V3' dot V4^
    r3 = _mm_sub_ps(simd_ror_ps(Va,0),simd_ror_ps(Vc,1));        // V3' dot V4" - V3" dot V4'

    tt = _L2;
    Va = simd_ror_ps(tt,1);        sum = _mm_mul_ps(Va,r1);
    Vb = simd_ror_ps(tt,2);        sum = _mm_add_ps(sum,_mm_mul_ps(Vb,r2));
    Vc = simd_ror_ps(tt,3);        sum = _mm_add_ps(sum,_mm_mul_ps(Vc,r3));

    // Calculating the determinant.
    Det = _mm_mul_ps(sum,_L1);
    Det = _mm_add_ps(Det,_mm_movehl_ps(Det,Det));

    // Calculating the minterms of the second line (using previous results).
    tt = simd_ror_ps(_L1,1);        sum = _mm_mul_ps(tt,r1);
    tt = simd_ror_ps(tt,1);        sum = _mm_add_ps(sum,_mm_mul_ps(tt,r2));
    tt = simd_ror_ps(tt,1);        sum = _mm_add_ps(sum,_mm_mul_ps(tt,r3));

    // Testing the determinant.
    Det = _mm_sub_ss(Det,_mm_shuffle_ps(Det,Det,1));
    return SimdFloat(Det, 0);
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::operator +( const SimdMatrix4 & mat ) const
{
    return SimdMatrix4(
        ( col0 + mat.col0 ),
        ( col1 + mat.col1 ),
        ( col2 + mat.col2 ),
        ( col3 + mat.col3 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::operator -( const SimdMatrix4 & mat ) const
{
    return SimdMatrix4(
        ( col0 - mat.col0 ),
        ( col1 - mat.col1 ),
        ( col2 - mat.col2 ),
        ( col3 - mat.col3 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & SimdMatrix4::operator +=( const SimdMatrix4 & mat )
{
    *this = *this + mat;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & SimdMatrix4::operator -=( const SimdMatrix4 & mat )
{
    *this = *this - mat;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::operator -( ) const
{
    return SimdMatrix4(
        ( -col0 ),
        ( -col1 ),
        ( -col2 ),
        ( -col3 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 abs( const SimdMatrix4 & mat )
{
    return SimdMatrix4(
        abs( mat.getCol0() ),
        abs( mat.getCol1() ),
        abs( mat.getCol2() ),
        abs( mat.getCol3() )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::operator *( float scalar ) const
{
    return *this * SimdFloat(scalar);
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::operator *( const SimdFloat &scalar ) const
{
    return SimdMatrix4(
        ( col0 * scalar ),
        ( col1 * scalar ),
        ( col2 * scalar ),
        ( col3 * scalar )
    );
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & SimdMatrix4::operator *=( float scalar )
{
    return *this *= SimdFloat(scalar);
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & SimdMatrix4::operator *=( const SimdFloat &scalar )
{
    *this = *this * scalar;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 operator *( float scalar, const SimdMatrix4 & mat )
{
    return SimdFloat(scalar) * mat;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 operator *( const SimdFloat &scalar, const SimdMatrix4 & mat )
{
    return mat * scalar;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 mul(const SimdMatrix4 & mat, const SimdVector4 & vec )
{
    SimdVector4 col0 = mat.col0;
    SimdVector4 col1 = mat.col1;
    SimdVector4 col2 = mat.col2;
    SimdVector4 col3 = mat.col3;
    return SimdVector4(
        _mm_add_ps(
            _mm_add_ps(_mm_mul_ps(col0.get128(), _mm_shuffle_ps(vec.get128(), vec.get128(), _MM_SHUFFLE(0,0,0,0))), _mm_mul_ps(col1.get128(), _mm_shuffle_ps(vec.get128(), vec.get128(), _MM_SHUFFLE(1,1,1,1)))),
            _mm_add_ps(_mm_mul_ps(col2.get128(), _mm_shuffle_ps(vec.get128(), vec.get128(), _MM_SHUFFLE(2,2,2,2))), _mm_mul_ps(col3.get128(), _mm_shuffle_ps(vec.get128(), vec.get128(), _MM_SHUFFLE(3,3,3,3)))))
        );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 mul(const SimdMatrix4 & mat, const SimdVector3 & vec )
{
    SimdVector4 col0 = mat.col0;
    SimdVector4 col1 = mat.col1;
    SimdVector4 col2 = mat.col2;
    SimdVector4 col3 = mat.col3;
    return SimdVector4(
        _mm_add_ps(
            _mm_add_ps(_mm_mul_ps(col0.get128(), _mm_shuffle_ps(vec.get128(), vec.get128(), _MM_SHUFFLE(0,0,0,0))), _mm_mul_ps(col1.get128(), _mm_shuffle_ps(vec.get128(), vec.get128(), _MM_SHUFFLE(1,1,1,1)))),
            _mm_mul_ps(col2.get128(), _mm_shuffle_ps(vec.get128(), vec.get128(), _MM_SHUFFLE(2,2,2,2))))
        );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 mul(const SimdMatrix4 & mat, const SimdPoint3 & pnt ) 
{
    SimdVector4 col0 = mat.col0;
    SimdVector4 col1 = mat.col1;
    SimdVector4 col2 = mat.col2;
    SimdVector4 col3 = mat.col3;
    return SimdVector4(
        _mm_add_ps(
            _mm_add_ps(_mm_mul_ps(col0.get128(), _mm_shuffle_ps(pnt.get128(), pnt.get128(), _MM_SHUFFLE(0,0,0,0))), _mm_mul_ps(col1.get128(), _mm_shuffle_ps(pnt.get128(), pnt.get128(), _MM_SHUFFLE(1,1,1,1)))),
            _mm_add_ps(_mm_mul_ps(col2.get128(), _mm_shuffle_ps(pnt.get128(), pnt.get128(), _MM_SHUFFLE(2,2,2,2))), col3.get128()))
        );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 mul(const SimdMatrix4 & mat, const SimdTransform3 & tfrm )
{
    return SimdMatrix4(
        mul( mat, tfrm.getCol0() ),
        mul( mat, tfrm.getCol1() ),
        mul( mat, tfrm.getCol2() ),
        mul( mat, SimdPoint3( tfrm.getCol3() ) )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 mul(const SimdMatrix4 & mat0, const SimdMatrix4 & mat1 )
{
    return SimdMatrix4(
        mul(mat0, mat1.getCol0()),
        mul(mat0, mat1.getCol1()),
        mul(mat0, mat1.getCol2()),
        mul(mat0, mat1.getCol3())
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::identity( )
{
    return SimdMatrix4(
        SimdVector4::xAxis( ),
        SimdVector4::yAxis( ),
        SimdVector4::zAxis( ),
        SimdVector4::wAxis( )
    );
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & SimdMatrix4::setUpper3x3( const SimdMatrix3 & mat3 )
{
    col0.setXYZ( mat3.getCol0() );
    col1.setXYZ( mat3.getCol1() );
    col2.setXYZ( mat3.getCol2() );
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 SimdMatrix4::getUpper3x3( ) const
{
    return SimdMatrix3(
        col0.getXYZ( ),
        col1.getXYZ( ),
        col2.getXYZ( )
    );
}

SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & SimdMatrix4::setTranslation( const SimdVector3 &translateVec )
{
    col3.setXYZ( translateVec );
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdMatrix4::getTranslation( ) const
{
    return col3.getXYZ( );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::rotationX( float radians )
{
    return rotationX( SimdFloat(radians) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::rotationX( const SimdFloat &radians )
{
    __m128 s, c, res1, res2;
    __m128 zero;
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_z[4] = {0, 0, 0xffffffff, 0};
    zero = _mm_setzero_ps();
    simd_sincos_ps( radians.get128(), &s, &c );
    res1 = simd_select_ps( zero, c, select_y );
    res1 = simd_select_ps( res1, s, select_z );
    res2 = simd_select_ps( zero, simd_negate_ps(s), select_y );
    res2 = simd_select_ps( res2, c, select_z );
    return SimdMatrix4(
        SimdVector4::xAxis( ),
        SimdVector4( res1 ),
        SimdVector4( res2 ),
        SimdVector4::wAxis( )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::rotationY( float radians )
{
    return rotationY( SimdFloat(radians) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::rotationY( const SimdFloat &radians )
{
    __m128 s, c, res0, res2;
    __m128 zero;
    SIMD_VECTORMATH_ALIGN16 unsigned int select_x[4] = {0xffffffff, 0, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_z[4] = {0, 0, 0xffffffff, 0};
    zero = _mm_setzero_ps();
    simd_sincos_ps( radians.get128(), &s, &c );
    res0 = simd_select_ps( zero, c, select_x );
    res0 = simd_select_ps( res0, simd_negate_ps(s), select_z );
    res2 = simd_select_ps( zero, s, select_x );
    res2 = simd_select_ps( res2, c, select_z );
    return SimdMatrix4(
        SimdVector4( res0 ),
        SimdVector4::yAxis( ),
        SimdVector4( res2 ),
        SimdVector4::wAxis( )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::rotationZ( float radians )
{
    return rotationZ( SimdFloat(radians) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::rotationZ( const SimdFloat &radians )
{
    __m128 s, c, res0, res1;
    __m128 zero;
    SIMD_VECTORMATH_ALIGN16 unsigned int select_x[4] = {0xffffffff, 0, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    zero = _mm_setzero_ps();
    simd_sincos_ps( radians.get128(), &s, &c );
    res0 = simd_select_ps( zero, c, select_x );
    res0 = simd_select_ps( res0, s, select_y );
    res1 = simd_select_ps( zero, simd_negate_ps(s), select_x );
    res1 = simd_select_ps( res1, c, select_y );
    return SimdMatrix4(
        SimdVector4( res0 ),
        SimdVector4( res1 ),
        SimdVector4::zAxis( ),
        SimdVector4::wAxis( )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::rotationZYX( const SimdVector3 &radiansXYZ )
{
    __m128 angles, s, negS, c, X0, X1, Y0, Y1, Z0, Z1, tmp;
    angles = SimdVector4( radiansXYZ, 0.0f ).get128();
    simd_sincos_ps( angles, &s, &c );
    negS = simd_negate_ps( s );
    Z0 = _mm_unpackhi_ps( c, s );
    Z1 = _mm_unpackhi_ps( negS, c );
    SIMD_VECTORMATH_ALIGN16 unsigned int select_xyz[4] = {0xffffffff, 0xffffffff, 0xffffffff, 0};
    Z1 = _mm_and_ps( Z1, _mm_load_ps( (float *)select_xyz ) );
    Y0 = _mm_shuffle_ps( c, negS, _MM_SHUFFLE(0,1,1,1) );
    Y1 = _mm_shuffle_ps( s, c, _MM_SHUFFLE(0,1,1,1) );
    X0 = simd_broadcast_ps( s, 0 );
    X1 = simd_broadcast_ps( c, 0 );
    tmp = _mm_mul_ps( Z0, Y1 );
    return SimdMatrix4(
        SimdVector4( _mm_mul_ps( Z0, Y0 ) ),
        SimdVector4( simd_madd_ps( Z1, X1, _mm_mul_ps( tmp, X0 ) ) ),
        SimdVector4( simd_fnmadd_ps( Z1, X0, _mm_mul_ps( tmp, X1 ) ) ),
        SimdVector4::wAxis( )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::rotation( float radians, const SimdVector3 & unitVec )
{
    return rotation( SimdFloat(radians), unitVec );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::rotation( const SimdFloat &radians, const SimdVector3 & unitVec )
{
    __m128 axis, s, c, oneMinusC, axisS, negAxisS, xxxx, yyyy, zzzz, tmp0, tmp1, tmp2;
    axis = unitVec.get128();
    simd_sincos_ps( radians.get128(), &s, &c );
    xxxx = simd_broadcast_ps( axis, 0 );
    yyyy = simd_broadcast_ps( axis, 1 );
    zzzz = simd_broadcast_ps( axis, 2 );
    oneMinusC = _mm_sub_ps( _mm_set1_ps(1.0f), c );
    axisS = _mm_mul_ps( axis, s );
    negAxisS = simd_negate_ps( axisS );
    SIMD_VECTORMATH_ALIGN16 unsigned int select_x[4] = {0xffffffff, 0, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_z[4] = {0, 0, 0xffffffff, 0};
    //tmp0 = vec_perm( axisS, negAxisS, _SIMD_VECTORMATH_PERM_XZBX );
    tmp0 = _mm_shuffle_ps( axisS, axisS, _MM_SHUFFLE(0,0,2,0) );
    tmp0 = simd_select_ps(tmp0, simd_broadcast_ps(negAxisS, 1), select_z);
    //tmp1 = vec_perm( axisS, negAxisS, _SIMD_VECTORMATH_PERM_CXXX );
    tmp1 = simd_select_ps( simd_broadcast_ps(axisS, 0), simd_broadcast_ps(negAxisS, 2), select_x );
    //tmp2 = vec_perm( axisS, negAxisS, _SIMD_VECTORMATH_PERM_YAXX );
    tmp2 = _mm_shuffle_ps( axisS, axisS, _MM_SHUFFLE(0,0,0,1) );
    tmp2 = simd_select_ps(tmp2, simd_broadcast_ps(negAxisS, 0), select_y);
    tmp0 = simd_select_ps( tmp0, c, select_x );
    tmp1 = simd_select_ps( tmp1, c, select_y );
    tmp2 = simd_select_ps( tmp2, c, select_z );
    SIMD_VECTORMATH_ALIGN16 unsigned int select_xyz[4] = {0xffffffff, 0xffffffff, 0xffffffff, 0};
    axis = _mm_and_ps( axis, _mm_load_ps( (float *)select_xyz ) );
    tmp0 = _mm_and_ps( tmp0, _mm_load_ps( (float *)select_xyz ) );
    tmp1 = _mm_and_ps( tmp1, _mm_load_ps( (float *)select_xyz ) );
    tmp2 = _mm_and_ps( tmp2, _mm_load_ps( (float *)select_xyz ) );
    return SimdMatrix4(
        SimdVector4( simd_madd_ps( _mm_mul_ps( axis, xxxx ), oneMinusC, tmp0 ) ),
        SimdVector4( simd_madd_ps( _mm_mul_ps( axis, yyyy ), oneMinusC, tmp1 ) ),
        SimdVector4( simd_madd_ps( _mm_mul_ps( axis, zzzz ), oneMinusC, tmp2 ) ),
        SimdVector4::wAxis( )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::rotation( const SimdQuat & unitSimdQuat )
{
    return SimdMatrix4( SimdTransform3::rotation( unitSimdQuat ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::scale( const SimdVector3 & scaleVec )
{
    __m128 zero = _mm_setzero_ps();
    SIMD_VECTORMATH_ALIGN16 unsigned int select_x[4] = {0xffffffff, 0, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_z[4] = {0, 0, 0xffffffff, 0};
    return SimdMatrix4(
        SimdVector4( simd_select_ps( zero, scaleVec.get128(), select_x ) ),
        SimdVector4( simd_select_ps( zero, scaleVec.get128(), select_y ) ),
        SimdVector4( simd_select_ps( zero, scaleVec.get128(), select_z ) ),
        SimdVector4::wAxis( )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 appendScale( const SimdMatrix4 & mat, const SimdVector3 & scaleVec )
{
    return SimdMatrix4(
        ( mat.getCol0() * scaleVec.getX( ) ),
        ( mat.getCol1() * scaleVec.getY( ) ),
        ( mat.getCol2() * scaleVec.getZ( ) ),
        mat.getCol3()
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 prependScale( const SimdVector3 & scaleVec, const SimdMatrix4 & mat )
{
    SimdVector4 scale4;
    scale4 = SimdVector4( scaleVec, 1.0f );
    return SimdMatrix4(
        operator *( mat.getCol0(), scale4 ),
        operator *( mat.getCol1(), scale4 ),
        operator *( mat.getCol2(), scale4 ),
        operator *( mat.getCol3(), scale4 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::translation( const SimdVector3 &translateVec )
{
    return SimdMatrix4(
        SimdVector4::xAxis( ),
        SimdVector4::yAxis( ),
        SimdVector4::zAxis( ),
        SimdVector4( translateVec, 1.0f )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::lookAt( const SimdPoint3 &eyePos, const SimdPoint3 &lookAtPos, const SimdVector3 &upVec )
{
    SimdMatrix4 m4EyeFrame;
    SimdVector3 v3X, v3Y, v3Z;
    v3Y = normalize( upVec );
    v3Z = normalize( ( eyePos - lookAtPos ) );
    v3X = normalize( cross( v3Y, v3Z ) );
    v3Y = cross( v3Z, v3X );
    m4EyeFrame = SimdMatrix4( SimdVector4( v3X ), SimdVector4( v3Y ), SimdVector4( v3Z ), SimdVector4( eyePos ) );
    return orthoInverse( m4EyeFrame );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::perspective( float fovyRadians, float aspect, float zNear, float zFar )
{
    float f, rangeInv;
    __m128 zero, col0, col1, col2, col3;
    union { __m128 v; float s[4]; } tmp;
    f = tanf( _SIMD_VECTORMATH_PI_OVER_2 - fovyRadians * 0.5f );
    rangeInv = 1.0f / ( zNear - zFar );
    zero = _mm_setzero_ps();
    tmp.v = zero;
    tmp.s[0] = f / aspect;
    col0 = tmp.v;
    tmp.v = zero;
    tmp.s[1] = f;
    col1 = tmp.v;
    tmp.v = zero;
    tmp.s[2] = ( zNear + zFar ) * rangeInv;
    tmp.s[3] = -1.0f;
    col2 = tmp.v;
    tmp.v = zero;
    tmp.s[2] = zNear * zFar * rangeInv * 2.0f;
    col3 = tmp.v;
    return SimdMatrix4(
        SimdVector4( col0 ),
        SimdVector4( col1 ),
        SimdVector4( col2 ),
        SimdVector4( col3 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::frustum( float left, float right, float bottom, float top, float zNear, float zFar )
{
    /* function implementation based on code from STIDC SDK:           */
    /* --------------------------------------------------------------  */
    /* PLEASE DO NOT MODIFY THIS SECTION                               */
    /* This prolog section is automatically generated.                 */
    /*                                                                 */
    /* (C)Copyright                                                    */
    /* Sony Computer Entertainment, Inc.,                              */
    /* Toshiba Corporation,                                            */
    /* International Business Machines Corporation,                    */
    /* 2001,2002.                                                      */
    /* S/T/I Confidential Information                                  */
    /* --------------------------------------------------------------  */
    __m128 lbf, rtn;
    __m128 diff, sum, inv_diff;
    __m128 diagonal, column, near2;
    __m128 zero = _mm_setzero_ps();
    union { __m128 v; float s[4]; } l, f, r, n, b, t; // TODO: Union?
    l.s[0] = left;
    f.s[0] = zFar;
    r.s[0] = right;
    n.s[0] = zNear;
    b.s[0] = bottom;
    t.s[0] = top;
    lbf = _mm_unpacklo_ps( l.v, f.v );
    rtn = _mm_unpacklo_ps( r.v, n.v );
    lbf = _mm_unpacklo_ps( lbf, b.v );
    rtn = _mm_unpacklo_ps( rtn, t.v );
    diff = _mm_sub_ps( rtn, lbf );
    sum  = _mm_add_ps( rtn, lbf );
    inv_diff = _mm_rcp_ps( diff );
    near2 = simd_broadcast_ps( n.v, 0 );
    near2 = _mm_add_ps( near2, near2 );
    diagonal = _mm_mul_ps( near2, inv_diff );
    column = _mm_mul_ps( sum, inv_diff );
    SIMD_VECTORMATH_ALIGN16 unsigned int select_x[4] = {0xffffffff, 0, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_z[4] = {0, 0, 0xffffffff, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_w[4] = {0, 0, 0, 0xffffffff};
    return SimdMatrix4(
        SimdVector4( simd_select_ps( zero, diagonal, select_x ) ),
        SimdVector4( simd_select_ps( zero, diagonal, select_y ) ),
        SimdVector4( simd_select_ps( column, _mm_set1_ps(-1.0f), select_w ) ),
        SimdVector4( simd_select_ps( zero, _mm_mul_ps( diagonal, simd_broadcast_ps( f.v, 0 ) ), select_z ) )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 SimdMatrix4::orthographic( float left, float right, float bottom, float top, float zNear, float zFar )
{
    /* function implementation based on code from STIDC SDK:           */
    /* --------------------------------------------------------------  */
    /* PLEASE DO NOT MODIFY THIS SECTION                               */
    /* This prolog section is automatically generated.                 */
    /*                                                                 */
    /* (C)Copyright                                                    */
    /* Sony Computer Entertainment, Inc.,                              */
    /* Toshiba Corporation,                                            */
    /* International Business Machines Corporation,                    */
    /* 2001,2002.                                                      */
    /* S/T/I Confidential Information                                  */
    /* --------------------------------------------------------------  */
    __m128 lbf, rtn;
    __m128 diff, sum, inv_diff, neg_inv_diff;
    __m128 diagonal, column;
    __m128 zero = _mm_setzero_ps();
    union { __m128 v; float s[4]; } l, f, r, n, b, t;
    l.s[0] = left;
    f.s[0] = zFar;
    r.s[0] = right;
    n.s[0] = zNear;
    b.s[0] = bottom;
    t.s[0] = top;
    lbf = _mm_unpacklo_ps( l.v, f.v );
    rtn = _mm_unpacklo_ps( r.v, n.v );
    lbf = _mm_unpacklo_ps( lbf, b.v );
    rtn = _mm_unpacklo_ps( rtn, t.v );
    diff = _mm_sub_ps( rtn, lbf );
    sum  = _mm_add_ps( rtn, lbf );
    inv_diff = _mm_rcp_ps( diff );
    neg_inv_diff = simd_negate_ps( inv_diff );
    diagonal = _mm_add_ps( inv_diff, inv_diff );
    SIMD_VECTORMATH_ALIGN16 unsigned int select_x[4] = {0xffffffff, 0, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_z[4] = {0, 0, 0xffffffff, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_w[4] = {0, 0, 0, 0xffffffff};
    column = _mm_mul_ps( sum, simd_select_ps( neg_inv_diff, inv_diff, select_z ) ); // TODO: no madds with zero
    return SimdMatrix4(
        SimdVector4( simd_select_ps( zero, diagonal, select_x ) ),
        SimdVector4( simd_select_ps( zero, diagonal, select_y ) ),
        SimdVector4( simd_select_ps( zero, diagonal, select_z ) ),
        SimdVector4( simd_select_ps( column, _mm_set1_ps(1.0f), select_w ) )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 select( const SimdMatrix4 & mat0, const SimdMatrix4 & mat1, bool select1 )
{
    return SimdMatrix4(
        select( mat0.getCol0(), mat1.getCol0(), select1 ),
        select( mat0.getCol1(), mat1.getCol1(), select1 ),
        select( mat0.getCol2(), mat1.getCol2(), select1 ),
        select( mat0.getCol3(), mat1.getCol3(), select1 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 select( const SimdMatrix4 & mat0, const SimdMatrix4 & mat1, const SimdBool &select1 )
{
    return SimdMatrix4(
        select( mat0.getCol0(), mat1.getCol0(), select1 ),
        select( mat0.getCol1(), mat1.getCol1(), select1 ),
        select( mat0.getCol2(), mat1.getCol2(), select1 ),
        select( mat0.getCol3(), mat1.getCol3(), select1 )
    );
}

#ifdef _VECTORMATH_DEBUG

SIMD_VECTORMATH_FORCE_INLINE void print( const SimdMatrix4 & mat )
{
    print( mat.getRow( 0 ) );
    print( mat.getRow( 1 ) );
    print( mat.getRow( 2 ) );
    print( mat.getRow( 3 ) );
}

SIMD_VECTORMATH_FORCE_INLINE void print( const SimdMatrix4 & mat, const char * name )
{
    printf("%s:\n", name);
    print( mat );
}

#endif

SIMD_VECTORMATH_FORCE_INLINE SimdTransform3::SimdTransform3( const SimdTransform3 & tfrm )
{
    col0 = tfrm.col0;
    col1 = tfrm.col1;
    col2 = tfrm.col2;
    col3 = tfrm.col3;
}

SIMD_VECTORMATH_FORCE_INLINE SimdTransform3::SimdTransform3( float scalar )
{
    col0 = SimdVector3( scalar );
    col1 = SimdVector3( scalar );
    col2 = SimdVector3( scalar );
    col3 = SimdVector3( scalar );
}

SIMD_VECTORMATH_FORCE_INLINE SimdTransform3::SimdTransform3( const SimdFloat &scalar )
{
    col0 = SimdVector3( scalar );
    col1 = SimdVector3( scalar );
    col2 = SimdVector3( scalar );
    col3 = SimdVector3( scalar );
}

SIMD_VECTORMATH_FORCE_INLINE SimdTransform3::SimdTransform3( const SimdVector3 &_col0, const SimdVector3 &_col1, const SimdVector3 &_col2, const SimdVector3 &_col3 )
{
    col0 = _col0;
    col1 = _col1;
    col2 = _col2;
    col3 = _col3;
}

SIMD_VECTORMATH_FORCE_INLINE SimdTransform3::SimdTransform3( const SimdMatrix3 & tfrm, const SimdVector3 &translateVec )
{
    this->setUpper3x3( tfrm );
    this->setTranslation( translateVec );
}

SIMD_VECTORMATH_FORCE_INLINE SimdTransform3::SimdTransform3( const SimdQuat & unitSimdQuat, const SimdVector3 &translateVec )
{
    this->setUpper3x3( SimdMatrix3( unitSimdQuat ) );
    this->setTranslation( translateVec );
}

SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & SimdTransform3::setCol0( const SimdVector3 &_col0 )
{
    col0 = _col0;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & SimdTransform3::setCol1( const SimdVector3 &_col1 )
{
    col1 = _col1;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & SimdTransform3::setCol2( const SimdVector3 &_col2 )
{
    col2 = _col2;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & SimdTransform3::setCol3( const SimdVector3 &_col3 )
{
    col3 = _col3;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & SimdTransform3::setCol( int col, const SimdVector3 & vec )
{
    *(&col0 + col) = vec;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & SimdTransform3::setRow( int row, const SimdVector4 & vec )
{
    col0.setElem( row, vec.getElem( 0 ) );
    col1.setElem( row, vec.getElem( 1 ) );
    col2.setElem( row, vec.getElem( 2 ) );
    col3.setElem( row, vec.getElem( 3 ) );
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & SimdTransform3::setElem( int col, int row, float val )
{
    SimdVector3 colVec = getCol(col);
    colVec.setElem(row, val);
    setCol(col, colVec);
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & SimdTransform3::setElem( int col, int row, const SimdFloat & val )
{
    SimdVector3 tmpV3_0;
    tmpV3_0 = this->getCol( col );
    tmpV3_0.setElem( row, val );
    this->setCol( col, tmpV3_0 );
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdFloat SimdTransform3::getElem( int col, int row ) const
{
    return this->getCol( col ).getElem( row );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdTransform3::getCol0( ) const
{
    return col0;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdTransform3::getCol1( ) const
{
    return col1;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdTransform3::getCol2( ) const
{
    return col2;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdTransform3::getCol3( ) const
{
    return col3;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdTransform3::getCol( int col ) const
{
    return *(&col0 + col);
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 SimdTransform3::getRow( int row ) const
{
    return SimdVector4( col0.getElem( row ), col1.getElem( row ), col2.getElem( row ), col3.getElem( row ) );
}

SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & SimdTransform3::operator =( const SimdTransform3 & tfrm )
{
    col0 = tfrm.col0;
    col1 = tfrm.col1;
    col2 = tfrm.col2;
    col3 = tfrm.col3;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 inverse( const SimdTransform3 & tfrm )
{
    __m128 inv0, inv1, inv2, inv3;
    __m128 tmp0, tmp1, tmp2, tmp3, tmp4, dot, invdet;
    __m128 xxxx, yyyy, zzzz;
    tmp2 = _vmathVfCross( tfrm.getCol0().get128(), tfrm.getCol1().get128() );
    tmp0 = _vmathVfCross( tfrm.getCol1().get128(), tfrm.getCol2().get128() );
    tmp1 = _vmathVfCross( tfrm.getCol2().get128(), tfrm.getCol0().get128() );
    inv3 = simd_negate_ps( tfrm.getCol3().get128() );
    dot = _vmathVfDot3( tmp2, tfrm.getCol2().get128() );
    dot = simd_broadcast_ps( dot, 0 );
    invdet = _mm_rcp_ps( dot );
    tmp3 = _mm_unpacklo_ps( tmp0, tmp2 );
    tmp4 = _mm_unpackhi_ps( tmp0, tmp2 );
    inv0 = _mm_unpacklo_ps( tmp3, tmp1 );
    xxxx = simd_broadcast_ps( inv3, 0 );
    //inv1 = vec_perm( tmp3, tmp1, _SIMD_VECTORMATH_PERM_ZBWX );
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    inv1 = _mm_shuffle_ps( tmp3, tmp3, _MM_SHUFFLE(0,3,2,2));
    inv1 = simd_select_ps(inv1, tmp1, select_y);
    //inv2 = vec_perm( tmp4, tmp1, _SIMD_VECTORMATH_PERM_XCYX );
    inv2 = _mm_shuffle_ps( tmp4, tmp4, _MM_SHUFFLE(0,1,1,0));
    inv2 = simd_select_ps(inv2, simd_broadcast_ps(tmp1, 2), select_y);
    yyyy = simd_broadcast_ps( inv3, 1 );
    zzzz = simd_broadcast_ps( inv3, 2 );
    inv3 = _mm_mul_ps( inv0, xxxx );
    inv3 = simd_madd_ps( inv1, yyyy, inv3 );
    inv3 = simd_madd_ps( inv2, zzzz, inv3 );
    inv0 = _mm_mul_ps( inv0, invdet );
    inv1 = _mm_mul_ps( inv1, invdet );
    inv2 = _mm_mul_ps( inv2, invdet );
    inv3 = _mm_mul_ps( inv3, invdet );
    return SimdTransform3(
        SimdVector3( inv0 ),
        SimdVector3( inv1 ),
        SimdVector3( inv2 ),
        SimdVector3( inv3 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 orthoInverse( const SimdTransform3 & tfrm )
{
    __m128 inv0, inv1, inv2, inv3;
    __m128 tmp0, tmp1;
    __m128 xxxx, yyyy, zzzz;
    tmp0 = _mm_unpacklo_ps( tfrm.getCol0().get128(), tfrm.getCol2().get128() );
    tmp1 = _mm_unpackhi_ps( tfrm.getCol0().get128(), tfrm.getCol2().get128() );
    inv3 = simd_negate_ps( tfrm.getCol3().get128() );
    inv0 = _mm_unpacklo_ps( tmp0, tfrm.getCol1().get128() );
    xxxx = simd_broadcast_ps( inv3, 0 );
    //inv1 = vec_perm( tmp0, tfrm.getCol1().get128(), _SIMD_VECTORMATH_PERM_ZBWX );
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    inv1 = _mm_shuffle_ps( tmp0, tmp0, _MM_SHUFFLE(0,3,2,2));
    inv1 = simd_select_ps(inv1, tfrm.getCol1().get128(), select_y);
    //inv2 = vec_perm( tmp1, tfrm.getCol1().get128(), _SIMD_VECTORMATH_PERM_XCYX );
    inv2 = _mm_shuffle_ps( tmp1, tmp1, _MM_SHUFFLE(0,1,1,0));
    inv2 = simd_select_ps(inv2, simd_broadcast_ps(tfrm.getCol1().get128(), 2), select_y);
    yyyy = simd_broadcast_ps( inv3, 1 );
    zzzz = simd_broadcast_ps( inv3, 2 );
    inv3 = _mm_mul_ps( inv0, xxxx );
    inv3 = simd_madd_ps( inv1, yyyy, inv3 );
    inv3 = simd_madd_ps( inv2, zzzz, inv3 );
    return SimdTransform3(
        SimdVector3( inv0 ),
        SimdVector3( inv1 ),
        SimdVector3( inv2 ),
        SimdVector3( inv3 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 abs( const SimdTransform3 & tfrm )
{
    return SimdTransform3(
        abs( tfrm.getCol0() ),
        abs( tfrm.getCol1() ),
        abs( tfrm.getCol2() ),
        abs( tfrm.getCol3() )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 mul(const SimdTransform3 & tfrm, const SimdVector3 & vec )
{
    SimdVector3 col0 = tfrm.col0;
    SimdVector3 col1 = tfrm.col1;
    SimdVector3 col2 = tfrm.col2;
    __m128 res;
    __m128 xxxx, yyyy, zzzz;
    xxxx = simd_broadcast_ps( vec.get128(), 0 );
    yyyy = simd_broadcast_ps( vec.get128(), 1 );
    zzzz = simd_broadcast_ps( vec.get128(), 2 );
    res = _mm_mul_ps( col0.get128(), xxxx );
    res = simd_madd_ps( col1.get128(), yyyy, res );
    res = simd_madd_ps( col2.get128(), zzzz, res );
    return SimdVector3( res );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 mul(const SimdTransform3 & tfrm, const SimdPoint3 & pnt )
{
    SimdVector3 col0 = tfrm.col0;
    SimdVector3 col1 = tfrm.col1;
    SimdVector3 col2 = tfrm.col2;
    SimdVector3 col3 = tfrm.col3;
    __m128 tmp0, tmp1, res;
    __m128 xxxx, yyyy, zzzz;
    xxxx = simd_broadcast_ps( pnt.get128(), 0 );
    yyyy = simd_broadcast_ps( pnt.get128(), 1 );
    zzzz = simd_broadcast_ps( pnt.get128(), 2 );
    tmp0 = _mm_mul_ps( col0.get128(), xxxx );
    tmp1 = _mm_mul_ps( col1.get128(), yyyy );
    tmp0 = simd_madd_ps( col2.get128(), zzzz, tmp0 );
    tmp1 = _mm_add_ps( col3.get128(), tmp1 );
    res = _mm_add_ps( tmp0, tmp1 );
    return SimdPoint3( res );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 mul(const SimdTransform3 & tfrm0, const SimdTransform3 & tfrm )
{
    return SimdTransform3(
        mul( tfrm0, tfrm.col0 ),
        mul( tfrm0, tfrm.col1 ),
        mul( tfrm0, tfrm.col2 ),
        SimdVector3( mul( tfrm0, SimdPoint3( tfrm.col3 ) ) )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 SimdTransform3::identity( )
{
    return SimdTransform3(
        SimdVector3::xAxis( ),
        SimdVector3::yAxis( ),
        SimdVector3::zAxis( ),
        SimdVector3( 0.0f )
    );
}

SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & SimdTransform3::setUpper3x3( const SimdMatrix3 & tfrm )
{
    col0 = tfrm.getCol0();
    col1 = tfrm.getCol1();
    col2 = tfrm.getCol2();
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 SimdTransform3::getUpper3x3( ) const
{
    return SimdMatrix3( col0, col1, col2 );
}

SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & SimdTransform3::setTranslation( const SimdVector3 &translateVec )
{
    col3 = translateVec;
    return *this;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 SimdTransform3::getTranslation( ) const
{
    return col3;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 SimdTransform3::rotationX( float radians )
{
    return rotationX( SimdFloat(radians) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 SimdTransform3::rotationX( const SimdFloat &radians )
{
    __m128 s, c, res1, res2;
    __m128 zero;
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_z[4] = {0, 0, 0xffffffff, 0};
    zero = _mm_setzero_ps();
    simd_sincos_ps( radians.get128(), &s, &c );
    res1 = simd_select_ps( zero, c, select_y );
    res1 = simd_select_ps( res1, s, select_z );
    res2 = simd_select_ps( zero, simd_negate_ps(s), select_y );
    res2 = simd_select_ps( res2, c, select_z );
    return SimdTransform3(
        SimdVector3::xAxis( ),
        SimdVector3( res1 ),
        SimdVector3( res2 ),
        SimdVector3( _mm_setzero_ps() )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 SimdTransform3::rotationY( float radians )
{
    return rotationY( SimdFloat(radians) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 SimdTransform3::rotationY( const SimdFloat &radians )
{
    __m128 s, c, res0, res2;
    __m128 zero;
    SIMD_VECTORMATH_ALIGN16 unsigned int select_x[4] = {0xffffffff, 0, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_z[4] = {0, 0, 0xffffffff, 0};
    zero = _mm_setzero_ps();
    simd_sincos_ps( radians.get128(), &s, &c );
    res0 = simd_select_ps( zero, c, select_x );
    res0 = simd_select_ps( res0, simd_negate_ps(s), select_z );
    res2 = simd_select_ps( zero, s, select_x );
    res2 = simd_select_ps( res2, c, select_z );
    return SimdTransform3(
        SimdVector3( res0 ),
        SimdVector3::yAxis( ),
        SimdVector3( res2 ),
        SimdVector3( 0.0f )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 SimdTransform3::rotationZ( float radians )
{
    return rotationZ( SimdFloat(radians) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 SimdTransform3::rotationZ( const SimdFloat &radians )
{
    __m128 s, c, res0, res1;
    SIMD_VECTORMATH_ALIGN16 unsigned int select_x[4] = {0xffffffff, 0, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    __m128 zero = _mm_setzero_ps();
    simd_sincos_ps( radians.get128(), &s, &c );
    res0 = simd_select_ps( zero, c, select_x );
    res0 = simd_select_ps( res0, s, select_y );
    res1 = simd_select_ps( zero, simd_negate_ps(s), select_x );
    res1 = simd_select_ps( res1, c, select_y );
    return SimdTransform3(
        SimdVector3( res0 ),
        SimdVector3( res1 ),
        SimdVector3::zAxis( ),
        SimdVector3( 0.0f )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 SimdTransform3::rotationZYX( const SimdVector3 &radiansXYZ )
{
    __m128 angles, s, negS, c, X0, X1, Y0, Y1, Z0, Z1, tmp;
    angles = SimdVector4( radiansXYZ, 0.0f ).get128();
    simd_sincos_ps( angles, &s, &c );
    negS = simd_negate_ps( s );
    Z0 = _mm_unpackhi_ps( c, s );
    Z1 = _mm_unpackhi_ps( negS, c );
    SIMD_VECTORMATH_ALIGN16 unsigned int select_xyz[4] = {0xffffffff, 0xffffffff, 0xffffffff, 0};
    Z1 = _mm_and_ps( Z1, _mm_load_ps( (float *)select_xyz ) );
    Y0 = _mm_shuffle_ps( c, negS, _MM_SHUFFLE(0,1,1,1) );
    Y1 = _mm_shuffle_ps( s, c, _MM_SHUFFLE(0,1,1,1) );
    X0 = simd_broadcast_ps( s, 0 );
    X1 = simd_broadcast_ps( c, 0 );
    tmp = _mm_mul_ps( Z0, Y1 );
    return SimdTransform3(
        SimdVector3( _mm_mul_ps( Z0, Y0 ) ),
        SimdVector3( simd_madd_ps( Z1, X1, _mm_mul_ps( tmp, X0 ) ) ),
        SimdVector3( simd_fnmadd_ps( Z1, X0, _mm_mul_ps( tmp, X1 ) ) ),
        SimdVector3( 0.0f )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 SimdTransform3::rotation( float radians, const SimdVector3 & unitVec )
{
    return rotation( SimdFloat(radians), unitVec );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 SimdTransform3::rotation( const SimdFloat &radians, const SimdVector3 & unitVec )
{
    return SimdTransform3( SimdMatrix3::rotation( radians, unitVec ), SimdVector3( 0.0f ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 SimdTransform3::rotation( const SimdQuat & unitSimdQuat )
{
    return SimdTransform3( SimdMatrix3( unitSimdQuat ), SimdVector3( 0.0f ) );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 SimdTransform3::scale( const SimdVector3 & scaleVec )
{
    __m128 zero = _mm_setzero_ps();
    SIMD_VECTORMATH_ALIGN16 unsigned int select_x[4] = {0xffffffff, 0, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_z[4] = {0, 0, 0xffffffff, 0};
    return SimdTransform3(
        SimdVector3( simd_select_ps( zero, scaleVec.get128(), select_x ) ),
        SimdVector3( simd_select_ps( zero, scaleVec.get128(), select_y ) ),
        SimdVector3( simd_select_ps( zero, scaleVec.get128(), select_z ) ),
        SimdVector3( 0.0f )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 appendScale( const SimdTransform3 & tfrm, const SimdVector3 & scaleVec )
{
    return SimdTransform3(
        ( tfrm.getCol0() * scaleVec.getX( ) ),
        ( tfrm.getCol1() * scaleVec.getY( ) ),
        ( tfrm.getCol2() * scaleVec.getZ( ) ),
        tfrm.getCol3()
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 prependScale( const SimdVector3 & scaleVec, const SimdTransform3 & tfrm )
{
    return SimdTransform3(
        operator *( tfrm.getCol0(), scaleVec ),
        operator *( tfrm.getCol1(), scaleVec ),
        operator *( tfrm.getCol2(), scaleVec ),
        operator *( tfrm.getCol3(), scaleVec )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 SimdTransform3::translation( const SimdVector3 &translateVec )
{
    return SimdTransform3(
        SimdVector3::xAxis( ),
        SimdVector3::yAxis( ),
        SimdVector3::zAxis( ),
        translateVec
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 select( const SimdTransform3 & tfrm0, const SimdTransform3 & tfrm1, bool select1 )
{
    return SimdTransform3(
        select( tfrm0.getCol0(), tfrm1.getCol0(), select1 ),
        select( tfrm0.getCol1(), tfrm1.getCol1(), select1 ),
        select( tfrm0.getCol2(), tfrm1.getCol2(), select1 ),
        select( tfrm0.getCol3(), tfrm1.getCol3(), select1 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 select( const SimdTransform3 & tfrm0, const SimdTransform3 & tfrm1, const SimdBool &select1 )
{
    return SimdTransform3(
        select( tfrm0.getCol0(), tfrm1.getCol0(), select1 ),
        select( tfrm0.getCol1(), tfrm1.getCol1(), select1 ),
        select( tfrm0.getCol2(), tfrm1.getCol2(), select1 ),
        select( tfrm0.getCol3(), tfrm1.getCol3(), select1 )
    );
}

#ifdef _VECTORMATH_DEBUG

SIMD_VECTORMATH_FORCE_INLINE void print( const SimdTransform3 & tfrm )
{
    print( tfrm.getRow( 0 ) );
    print( tfrm.getRow( 1 ) );
    print( tfrm.getRow( 2 ) );
}

SIMD_VECTORMATH_FORCE_INLINE void print( const SimdTransform3 & tfrm, const char * name )
{
    printf("%s:\n", name);
    print( tfrm );
}

#endif

SIMD_VECTORMATH_FORCE_INLINE SimdQuat::SimdQuat( const SimdMatrix3 & tfrm )
{
    __m128 res;
    __m128 col0, col1, col2;
    __m128 xx_yy, xx_yy_zz_xx, yy_zz_xx_yy, zz_xx_yy_zz, diagSum, diagDiff;
    __m128 zy_xz_yx, yz_zx_xy, sum, diff;
    __m128 radicand, invSqrt, scale;
    __m128 res0, res1, res2, res3;
    __m128 xx, yy, zz;
    SIMD_VECTORMATH_ALIGN16 unsigned int select_x[4] = {0xffffffff, 0, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_z[4] = {0, 0, 0xffffffff, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_w[4] = {0, 0, 0, 0xffffffff};

    col0 = tfrm.getCol0().get128();
    col1 = tfrm.getCol1().get128();
    col2 = tfrm.getCol2().get128();

    /* four cases: */
    /* trace > 0 */
    /* else */
    /*    xx largest diagonal element */
    /*    yy largest diagonal element */
    /*    zz largest diagonal element */

    /* compute quaternion for each case */

    xx_yy = simd_select_ps( col0, col1, select_y );
    //xx_yy_zz_xx = vec_perm( xx_yy, col2, _SIMD_VECTORMATH_PERM_XYCX );
    //yy_zz_xx_yy = vec_perm( xx_yy, col2, _SIMD_VECTORMATH_PERM_YCXY );
    //zz_xx_yy_zz = vec_perm( xx_yy, col2, _SIMD_VECTORMATH_PERM_CXYC );
    xx_yy_zz_xx = _mm_shuffle_ps( xx_yy, xx_yy, _MM_SHUFFLE(0,0,1,0) );
    xx_yy_zz_xx = simd_select_ps( xx_yy_zz_xx, col2, select_z ); // TODO: Ck
    yy_zz_xx_yy = _mm_shuffle_ps( xx_yy_zz_xx, xx_yy_zz_xx, _MM_SHUFFLE(1,0,2,1) );
    zz_xx_yy_zz = _mm_shuffle_ps( xx_yy_zz_xx, xx_yy_zz_xx, _MM_SHUFFLE(2,1,0,2) );

    diagSum = _mm_add_ps( _mm_add_ps( xx_yy_zz_xx, yy_zz_xx_yy ), zz_xx_yy_zz );
    diagDiff = _mm_sub_ps( _mm_sub_ps( xx_yy_zz_xx, yy_zz_xx_yy ), zz_xx_yy_zz );
    radicand = _mm_add_ps( simd_select_ps( diagDiff, diagSum, select_w ), _mm_set1_ps(1.0f) );
 //   invSqrt = _mm_rsqrt_ps( radicand );
    invSqrt = simd_rsqrt_newtonraphson_ps( radicand );

    

    zy_xz_yx = simd_select_ps( col0, col1, select_z );                                    // zy_xz_yx = 00 01 12 03
    //zy_xz_yx = vec_perm( zy_xz_yx, col2, _SIMD_VECTORMATH_PERM_ZAYX );
    zy_xz_yx = _mm_shuffle_ps( zy_xz_yx, zy_xz_yx, _MM_SHUFFLE(0,1,2,2) );        // zy_xz_yx = 12 12 01 00
    zy_xz_yx = simd_select_ps( zy_xz_yx, simd_broadcast_ps(col2, 0), select_y );                // zy_xz_yx = 12 20 01 00
    yz_zx_xy = simd_select_ps( col0, col1, select_x );                                    // yz_zx_xy = 10 01 02 03
    //yz_zx_xy = vec_perm( yz_zx_xy, col2, _SIMD_VECTORMATH_PERM_BZXX );
    yz_zx_xy = _mm_shuffle_ps( yz_zx_xy, yz_zx_xy, _MM_SHUFFLE(0,0,2,0) );        // yz_zx_xy = 10 02 10 10
    yz_zx_xy = simd_select_ps( yz_zx_xy, simd_broadcast_ps(col2, 1), select_x );                // yz_zx_xy = 21 02 10 10

    sum = _mm_add_ps( zy_xz_yx, yz_zx_xy );
    diff = _mm_sub_ps( zy_xz_yx, yz_zx_xy );

    scale = _mm_mul_ps( invSqrt, _mm_set1_ps(0.5f) );

    //res0 = vec_perm( sum, diff, _SIMD_VECTORMATH_PERM_XZYA );
    res0 = _mm_shuffle_ps( sum, sum, _MM_SHUFFLE(0,1,2,0) );
    res0 = simd_select_ps( res0, simd_broadcast_ps(diff, 0), select_w );  // TODO: Ck
    //res1 = vec_perm( sum, diff, _SIMD_VECTORMATH_PERM_ZXXB );
    res1 = _mm_shuffle_ps( sum, sum, _MM_SHUFFLE(0,0,0,2) );
    res1 = simd_select_ps( res1, simd_broadcast_ps(diff, 1), select_w );  // TODO: Ck
    //res2 = vec_perm( sum, diff, _SIMD_VECTORMATH_PERM_YXXC );
    res2 = _mm_shuffle_ps( sum, sum, _MM_SHUFFLE(0,0,0,1) );
    res2 = simd_select_ps( res2, simd_broadcast_ps(diff, 2), select_w );  // TODO: Ck
    res3 = diff;
    res0 = simd_select_ps( res0, radicand, select_x );
    res1 = simd_select_ps( res1, radicand, select_y );
    res2 = simd_select_ps( res2, radicand, select_z );
    res3 = simd_select_ps( res3, radicand, select_w );
    res0 = _mm_mul_ps( res0, simd_broadcast_ps( scale, 0 ) );
    res1 = _mm_mul_ps( res1, simd_broadcast_ps( scale, 1 ) );
    res2 = _mm_mul_ps( res2, simd_broadcast_ps( scale, 2 ) );
    res3 = _mm_mul_ps( res3, simd_broadcast_ps( scale, 3 ) );

    /* determine case and select answer */

    xx = simd_broadcast_ps( col0, 0 );
    yy = simd_broadcast_ps( col1, 1 );
    zz = simd_broadcast_ps( col2, 2 );
    res = simd_select_ps( res0, res1, _mm_cmpgt_ps( yy, xx ) );
    res = simd_select_ps( res, res2, _mm_and_ps( _mm_cmpgt_ps( zz, xx ), _mm_cmpgt_ps( zz, yy ) ) );
    res = simd_select_ps( res, res3, _mm_cmpgt_ps( simd_broadcast_ps( diagSum, 0 ), _mm_setzero_ps() ) );
    mVec128 = res;
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 outer( const SimdVector3 &tfrm0, const SimdVector3 &tfrm1 )
{
    return SimdMatrix3(
        ( tfrm0 * tfrm1.getX( ) ),
        ( tfrm0 * tfrm1.getY( ) ),
        ( tfrm0 * tfrm1.getZ( ) )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 outer( const SimdVector4 &tfrm0, const SimdVector4 &tfrm1 )
{
    return SimdMatrix4(
        ( tfrm0 * tfrm1.getX( ) ),
        ( tfrm0 * tfrm1.getY( ) ),
        ( tfrm0 * tfrm1.getZ( ) ),
        ( tfrm0 * tfrm1.getW( ) )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 mul( const SimdVector3 & vec, const SimdMatrix3 & mat )
{
    __m128 tmp0, tmp1, mcol0, mcol1, mcol2, res;
    __m128 xxxx, yyyy, zzzz;
    tmp0 = _mm_unpacklo_ps( mat.getCol0().get128(), mat.getCol2().get128() );
    tmp1 = _mm_unpackhi_ps( mat.getCol0().get128(), mat.getCol2().get128() );
    xxxx = simd_broadcast_ps( vec.get128(), 0 );
    mcol0 = _mm_unpacklo_ps( tmp0, mat.getCol1().get128() );
    //mcol1 = vec_perm( tmp0, mat.getCol1().get128(), _SIMD_VECTORMATH_PERM_ZBWX );
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    mcol1 = _mm_shuffle_ps( tmp0, tmp0, _MM_SHUFFLE(0,3,2,2));
    mcol1 = simd_select_ps(mcol1, mat.getCol1().get128(), select_y);
    //mcol2 = vec_perm( tmp1, mat.getCol1().get128(), _SIMD_VECTORMATH_PERM_XCYX );
    mcol2 = _mm_shuffle_ps( tmp1, tmp1, _MM_SHUFFLE(0,1,1,0));
    mcol2 = simd_select_ps(mcol2, simd_broadcast_ps(mat.getCol1().get128(), 2), select_y);
    yyyy = simd_broadcast_ps( vec.get128(), 1 );
    res = _mm_mul_ps( mcol0, xxxx );
    zzzz = simd_broadcast_ps( vec.get128(), 2 );
    res = simd_madd_ps( mcol1, yyyy, res );
    res = simd_madd_ps( mcol2, zzzz, res );
    return SimdVector3( res );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 crossMatrix( const SimdVector3 & vec )
{
    __m128 neg, res0, res1, res2;
    neg = simd_negate_ps( vec.get128() );
    SIMD_VECTORMATH_ALIGN16 unsigned int select_x[4] = {0xffffffff, 0, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_y[4] = {0, 0xffffffff, 0, 0};
    SIMD_VECTORMATH_ALIGN16 unsigned int select_z[4] = {0, 0, 0xffffffff, 0};
    //res0 = vec_perm( vec.get128(), neg, _SIMD_VECTORMATH_PERM_XZBX );
    res0 = _mm_shuffle_ps( vec.get128(), vec.get128(), _MM_SHUFFLE(0,2,2,0) );
    res0 = simd_select_ps(res0, simd_broadcast_ps(neg, 1), select_z);
    //res1 = vec_perm( vec.get128(), neg, _SIMD_VECTORMATH_PERM_CXXX );
    res1 = simd_select_ps(simd_broadcast_ps(vec.get128(), 0), simd_broadcast_ps(neg, 2), select_x);
    //res2 = vec_perm( vec.get128(), neg, _SIMD_VECTORMATH_PERM_YAXX );
    res2 = _mm_shuffle_ps( vec.get128(), vec.get128(), _MM_SHUFFLE(0,0,1,1) );
    res2 = simd_select_ps(res2, simd_broadcast_ps(neg, 0), select_y);
    SIMD_VECTORMATH_ALIGN16 unsigned int filter_x[4] = {0, 0xffffffff, 0xffffffff, 0xffffffff};
    SIMD_VECTORMATH_ALIGN16 unsigned int filter_y[4] = {0xffffffff, 0, 0xffffffff, 0xffffffff};
    SIMD_VECTORMATH_ALIGN16 unsigned int filter_z[4] = {0xffffffff, 0xffffffff, 0, 0xffffffff};
    res0 = _mm_and_ps( res0, _mm_load_ps((float *)filter_x ) );
    res1 = _mm_and_ps( res1, _mm_load_ps((float *)filter_y ) );
    res2 = _mm_and_ps( res2, _mm_load_ps((float *)filter_z ) ); // TODO: Use selects?
    return SimdMatrix3(
        SimdVector3( res0 ),
        SimdVector3( res1 ),
        SimdVector3( res2 )
    );
}

SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 crossMatrixMul( const SimdVector3 & vec, const SimdMatrix3 & mat )
{
    return SimdMatrix3( cross( vec, mat.getCol0() ), cross( vec, mat.getCol1() ), cross( vec, mat.getCol2() ) );
}

} // namespace FmVectormath

#endif
