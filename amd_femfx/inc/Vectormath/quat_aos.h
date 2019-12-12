/*
   Copyright (C) 2006-2010 Sony Computer Entertainment Inc.
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

#ifndef _VECTORMATH_QUAT_AOS_CPP_H
#define _VECTORMATH_QUAT_AOS_CPP_H

//-----------------------------------------------------------------------------
// Definitions

#ifndef _VECTORMATH_INTERNAL_FUNCTIONS
#define _VECTORMATH_INTERNAL_FUNCTIONS

#endif

namespace FmVectormath {

VECTORMATH_FORCE_INLINE Quat::Quat( const Quat & quat )
{
    x = quat.x;
    y = quat.y;
    z = quat.z;
    w = quat.w;
}

VECTORMATH_FORCE_INLINE Quat::Quat( float _x, float _y, float _z, float _w )
{
    x = _x;
    y = _y;
    z = _z;
    w = _w;
}

VECTORMATH_FORCE_INLINE Quat::Quat( const Vector3 & xyz, float _w )
{
    this->setXYZ( xyz );
    this->setW( _w );
}

VECTORMATH_FORCE_INLINE Quat::Quat( const Vector4 & vec )
{
    x = vec.getX();
    y = vec.getY();
    z = vec.getZ();
    w = vec.getW();
}

VECTORMATH_FORCE_INLINE Quat::Quat( float scalar )
{
    x = scalar;
    y = scalar;
    z = scalar;
    w = scalar;
}

VECTORMATH_FORCE_INLINE const Quat Quat::identity( )
{
    return Quat( 0.0f, 0.0f, 0.0f, 1.0f );
}

VECTORMATH_FORCE_INLINE const Quat lerp( float t, const Quat & quat0, const Quat & quat1 )
{
    return ( quat0 + ( ( quat1 - quat0 ) * t ) );
}

VECTORMATH_FORCE_INLINE const Quat slerp( float t, const Quat & unitQuat0, const Quat & unitQuat1 )
{
    Quat start;
    float recipSinAngle, scale0, scale1, cosAngle, angle;
    cosAngle = dot( unitQuat0, unitQuat1 );
    if ( cosAngle < 0.0f ) {
        cosAngle = -cosAngle;
        start = ( -unitQuat0 );
    } else {
        start = unitQuat0;
    }
    if ( cosAngle < _VECTORMATH_SLERP_TOL ) {
        angle = acosf( cosAngle );
        recipSinAngle = ( 1.0f / sinf( angle ) );
        scale0 = ( sinf( ( ( 1.0f - t ) * angle ) ) * recipSinAngle );
        scale1 = ( sinf( ( t * angle ) ) * recipSinAngle );
    } else {
        scale0 = ( 1.0f - t );
        scale1 = t;
    }
    return ( ( start * scale0 ) + ( unitQuat1 * scale1 ) );
}

VECTORMATH_FORCE_INLINE const Quat squad( float t, const Quat & unitQuat0, const Quat & unitQuat1, const Quat & unitQuat2, const Quat & unitQuat3 )
{
    Quat tmp0, tmp1;
    tmp0 = slerp( t, unitQuat0, unitQuat3 );
    tmp1 = slerp( t, unitQuat1, unitQuat2 );
    return slerp( ( ( 2.0f * t ) * ( 1.0f - t ) ), tmp0, tmp1 );
}

VECTORMATH_FORCE_INLINE void loadXYZW( Quat & quat, const float * fptr )
{
    quat = Quat( fptr[0], fptr[1], fptr[2], fptr[3] );
}

VECTORMATH_FORCE_INLINE void storeXYZW( const Quat & quat, float * fptr )
{
    fptr[0] = quat.getX();
    fptr[1] = quat.getY();
    fptr[2] = quat.getZ();
    fptr[3] = quat.getW();
}

VECTORMATH_FORCE_INLINE Quat & Quat::operator =( const Quat & quat )
{
    x = quat.x;
    y = quat.y;
    z = quat.z;
    w = quat.w;
    return *this;
}

VECTORMATH_FORCE_INLINE Quat & Quat::setXYZ( const Vector3 & vec )
{
    x = vec.getX();
    y = vec.getY();
    z = vec.getZ();
    return *this;
}

VECTORMATH_FORCE_INLINE const Vector3 Quat::getXYZ( ) const
{
    return Vector3( x, y, z );
}

VECTORMATH_FORCE_INLINE Quat & Quat::setX( float _x )
{
    x = _x;
    return *this;
}

VECTORMATH_FORCE_INLINE float Quat::getX( ) const
{
    return x;
}

VECTORMATH_FORCE_INLINE Quat & Quat::setY( float _y )
{
    y = _y;
    return *this;
}

VECTORMATH_FORCE_INLINE float Quat::getY( ) const
{
    return y;
}

VECTORMATH_FORCE_INLINE Quat & Quat::setZ( float _z )
{
    z = _z;
    return *this;
}

VECTORMATH_FORCE_INLINE float Quat::getZ( ) const
{
    return z;
}

VECTORMATH_FORCE_INLINE Quat & Quat::setW( float _w )
{
    w = _w;
    return *this;
}

VECTORMATH_FORCE_INLINE float Quat::getW( ) const
{
    return w;
}

VECTORMATH_FORCE_INLINE Quat & Quat::setElem( int idx, float value )
{
    *(&x + idx) = value;
    return *this;
}

VECTORMATH_FORCE_INLINE float Quat::getElem( int idx ) const
{
    return *(&x + idx);
}

VECTORMATH_FORCE_INLINE const Quat Quat::operator +( const Quat & quat ) const
{
    return Quat(
        ( x + quat.x ),
        ( y + quat.y ),
        ( z + quat.z ),
        ( w + quat.w )
    );
}

VECTORMATH_FORCE_INLINE const Quat Quat::operator -( const Quat & quat ) const
{
    return Quat(
        ( x - quat.x ),
        ( y - quat.y ),
        ( z - quat.z ),
        ( w - quat.w )
    );
}

VECTORMATH_FORCE_INLINE const Quat Quat::operator *( float scalar ) const
{
    return Quat(
        ( x * scalar ),
        ( y * scalar ),
        ( z * scalar ),
        ( w * scalar )
    );
}

VECTORMATH_FORCE_INLINE Quat & Quat::operator +=( const Quat & quat )
{
    *this = *this + quat;
    return *this;
}

VECTORMATH_FORCE_INLINE Quat & Quat::operator -=( const Quat & quat )
{
    *this = *this - quat;
    return *this;
}

VECTORMATH_FORCE_INLINE Quat & Quat::operator *=( float scalar )
{
    *this = *this * scalar;
    return *this;
}

VECTORMATH_FORCE_INLINE const Quat Quat::operator /( float scalar ) const
{
    return Quat(
        ( x / scalar ),
        ( y / scalar ),
        ( z / scalar ),
        ( w / scalar )
    );
}

VECTORMATH_FORCE_INLINE Quat & Quat::operator /=( float scalar )
{
    *this = *this / scalar;
    return *this;
}

VECTORMATH_FORCE_INLINE const Quat Quat::operator -( ) const
{
    return Quat(
        -x,
        -y,
        -z,
        -w
    );
}

VECTORMATH_FORCE_INLINE const Quat operator *( float scalar, const Quat & quat )
{
    return quat * scalar;
}

VECTORMATH_FORCE_INLINE float dot( const Quat & quat0, const Quat & quat1 )
{
    float result;
    result = ( quat0.getX() * quat1.getX() );
    result = ( result + ( quat0.getY() * quat1.getY() ) );
    result = ( result + ( quat0.getZ() * quat1.getZ() ) );
    result = ( result + ( quat0.getW() * quat1.getW() ) );
    return result;
}

VECTORMATH_FORCE_INLINE float norm( const Quat & quat )
{
    float result;
    result = ( quat.getX() * quat.getX() );
    result = ( result + ( quat.getY() * quat.getY() ) );
    result = ( result + ( quat.getZ() * quat.getZ() ) );
    result = ( result + ( quat.getW() * quat.getW() ) );
    return result;
}

VECTORMATH_FORCE_INLINE float length( const Quat & quat )
{
    return ::sqrtf( norm( quat ) );
}

VECTORMATH_FORCE_INLINE const Quat normalize( const Quat & quat )
{
    float lenSqr, lenInv;
    lenSqr = norm( quat );
    lenInv = ( 1.0f / sqrtf( lenSqr ) );
    return Quat(
        ( quat.getX() * lenInv ),
        ( quat.getY() * lenInv ),
        ( quat.getZ() * lenInv ),
        ( quat.getW() * lenInv )
    );
}

VECTORMATH_FORCE_INLINE const Quat Quat::rotation( const Vector3 & unitVec0, const Vector3 & unitVec1 )
{
    float cosHalfAngleX2, recipCosHalfAngleX2;
    cosHalfAngleX2 = sqrtf( ( 2.0f * ( 1.0f + dot( unitVec0, unitVec1 ) ) ) );
    recipCosHalfAngleX2 = ( 1.0f / cosHalfAngleX2 );
    return Quat( ( cross( unitVec0, unitVec1 ) * recipCosHalfAngleX2 ), ( cosHalfAngleX2 * 0.5f ) );
}

VECTORMATH_FORCE_INLINE const Quat Quat::rotation( float radians, const Vector3 & unitVec )
{
    float s, c, angle;
    angle = ( radians * 0.5f );
    s = sinf( angle );
    c = cosf( angle );
    return Quat( ( unitVec * s ), c );
}

VECTORMATH_FORCE_INLINE const Quat Quat::rotationX( float radians )
{
    float s, c, angle;
    angle = ( radians * 0.5f );
    s = sinf( angle );
    c = cosf( angle );
    return Quat( s, 0.0f, 0.0f, c );
}

VECTORMATH_FORCE_INLINE const Quat Quat::rotationY( float radians )
{
    float s, c, angle;
    angle = ( radians * 0.5f );
    s = sinf( angle );
    c = cosf( angle );
    return Quat( 0.0f, s, 0.0f, c );
}

VECTORMATH_FORCE_INLINE const Quat Quat::rotationZ( float radians )
{
    float s, c, angle;
    angle = ( radians * 0.5f );
    s = sinf( angle );
    c = cosf( angle );
    return Quat( 0.0f, 0.0f, s, c );
}

VECTORMATH_FORCE_INLINE const Quat mul(const Quat & quat0, const Quat & quat1 )
{
    return Quat(
        ((((quat0.w * quat1.x) + (quat0.x * quat1.w)) + (quat0.y * quat1.z)) - (quat0.z * quat1.y)),
        ((((quat0.w * quat1.y) + (quat0.y * quat1.w)) + (quat0.z * quat1.x)) - (quat0.x * quat1.z)),
        ((((quat0.w * quat1.z) + (quat0.z * quat1.w)) + (quat0.x * quat1.y)) - (quat0.y * quat1.x)),
        ((((quat0.w * quat1.w) - (quat0.x * quat1.x)) - (quat0.y * quat1.y)) - (quat0.z * quat1.z ) )
    );
}

VECTORMATH_FORCE_INLINE const Vector3 rotate( const Quat & quat, const Vector3 & vec )
{
    float tmpX, tmpY, tmpZ, tmpW;
    tmpX = ( ( ( quat.getW() * vec.getX() ) + ( quat.getY() * vec.getZ() ) ) - ( quat.getZ() * vec.getY() ) );
    tmpY = ( ( ( quat.getW() * vec.getY() ) + ( quat.getZ() * vec.getX() ) ) - ( quat.getX() * vec.getZ() ) );
    tmpZ = ( ( ( quat.getW() * vec.getZ() ) + ( quat.getX() * vec.getY() ) ) - ( quat.getY() * vec.getX() ) );
    tmpW = ( ( ( quat.getX() * vec.getX() ) + ( quat.getY() * vec.getY() ) ) + ( quat.getZ() * vec.getZ() ) );
    return Vector3(
        ( ( ( ( tmpW * quat.getX() ) + ( tmpX * quat.getW() ) ) - ( tmpY * quat.getZ() ) ) + ( tmpZ * quat.getY() ) ),
        ( ( ( ( tmpW * quat.getY() ) + ( tmpY * quat.getW() ) ) - ( tmpZ * quat.getX() ) ) + ( tmpX * quat.getZ() ) ),
        ( ( ( ( tmpW * quat.getZ() ) + ( tmpZ * quat.getW() ) ) - ( tmpX * quat.getY() ) ) + ( tmpY * quat.getX() ) )
    );
}

VECTORMATH_FORCE_INLINE const Quat conj( const Quat & quat )
{
    return Quat( -quat.getX(), -quat.getY(), -quat.getZ(), quat.getW() );
}

VECTORMATH_FORCE_INLINE const Quat select( const Quat & quat0, const Quat & quat1, bool select1 )
{
    return Quat(
        ( select1 )? quat1.getX() : quat0.getX(),
        ( select1 )? quat1.getY() : quat0.getY(),
        ( select1 )? quat1.getZ() : quat0.getZ(),
        ( select1 )? quat1.getW() : quat0.getW()
    );
}

#ifdef _VECTORMATH_DEBUG

VECTORMATH_FORCE_INLINE void print( const Quat & quat )
{
    printf( "( %f %f %f %f )\n", quat.getX(), quat.getY(), quat.getZ(), quat.getW() );
}

VECTORMATH_FORCE_INLINE void print( const Quat & quat, const char * name )
{
    printf( "%s: ( %f %f %f %f )\n", name, quat.getX(), quat.getY(), quat.getZ(), quat.getW() );
}

#endif

} // namespace FmVectormath

#endif
