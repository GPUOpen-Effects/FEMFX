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

#ifndef _VECTORMATH_VEC_AOS_CPP_H
#define _VECTORMATH_VEC_AOS_CPP_H

//-----------------------------------------------------------------------------
// Constants

#define _VECTORMATH_SLERP_TOL 0.999f

//-----------------------------------------------------------------------------
// Definitions

#ifndef _VECTORMATH_INTERNAL_FUNCTIONS
#define _VECTORMATH_INTERNAL_FUNCTIONS

#endif

namespace FmVectormath {

VECTORMATH_FORCE_INLINE Vector3::Vector3( const Vector3 & vec )
{
    x = vec.x;
    y = vec.y;
    z = vec.z;
}

VECTORMATH_FORCE_INLINE Vector3::Vector3( float _x, float _y, float _z )
{
    x = _x;
    y = _y;
    z = _z;
}

VECTORMATH_FORCE_INLINE Vector3::Vector3( const Point3 & pnt )
{
    x = pnt.getX();
    y = pnt.getY();
    z = pnt.getZ();
}

VECTORMATH_FORCE_INLINE Vector3::Vector3( float scalar )
{
    x = scalar;
    y = scalar;
    z = scalar;
}

VECTORMATH_FORCE_INLINE const Vector3 Vector3::xAxis( )
{
    return Vector3( 1.0f, 0.0f, 0.0f );
}

VECTORMATH_FORCE_INLINE const Vector3 Vector3::yAxis( )
{
    return Vector3( 0.0f, 1.0f, 0.0f );
}

VECTORMATH_FORCE_INLINE const Vector3 Vector3::zAxis( )
{
    return Vector3( 0.0f, 0.0f, 1.0f );
}

VECTORMATH_FORCE_INLINE const Vector3 lerp( float t, const Vector3 & vec0, const Vector3 & vec1 )
{
    return ( vec0 + ( ( vec1 - vec0 ) * t ) );
}

VECTORMATH_FORCE_INLINE const Vector3 slerp( float t, const Vector3 & unitVec0, const Vector3 & unitVec1 )
{
    float recipSinAngle, scale0, scale1, cosAngle, angle;
    cosAngle = dot( unitVec0, unitVec1 );
    if ( cosAngle < _VECTORMATH_SLERP_TOL ) {
        angle = acosf( cosAngle );
        recipSinAngle = ( 1.0f / sinf( angle ) );
        scale0 = ( sinf( ( ( 1.0f - t ) * angle ) ) * recipSinAngle );
        scale1 = ( sinf( ( t * angle ) ) * recipSinAngle );
    } else {
        scale0 = ( 1.0f - t );
        scale1 = t;
    }
    return ( ( unitVec0 * scale0 ) + ( unitVec1 * scale1 ) );
}

VECTORMATH_FORCE_INLINE void loadXYZ( Vector3 & vec, const float * fptr )
{
    vec = Vector3( fptr[0], fptr[1], fptr[2] );
}

VECTORMATH_FORCE_INLINE void storeXYZ( const Vector3 & vec, float * fptr )
{
    fptr[0] = vec.getX();
    fptr[1] = vec.getY();
    fptr[2] = vec.getZ();
}

VECTORMATH_FORCE_INLINE void loadHalfFloats( Vector3 & vec, const unsigned short * hfptr )
{
    union Data32 {
        unsigned int u32;
        float f32;
    };

    for (int i = 0; i < 3; i++) {
        unsigned short fp16 = hfptr[i];
        unsigned int sign = fp16 >> 15;
        unsigned int exponent = (fp16 >> 10) & ((1 << 5) - 1);
        unsigned int mantissa = fp16 & ((1 << 10) - 1);

        if (exponent == 0) {
            // zero
            mantissa = 0;

        } else if (exponent == 31) {
            // infinity or nan -> infinity
            exponent = 255;
        mantissa = 0;

        } else {
            exponent += 127 - 15;
            mantissa <<= 13;
        }

        Data32 d;
        d.u32 = (sign << 31) | (exponent << 23) | mantissa;
        vec.setElem(i, d.f32);
    }
}

VECTORMATH_FORCE_INLINE void storeHalfFloats( const Vector3 & vec, unsigned short * hfptr )
{
    union Data32 {
        unsigned int u32;
        float f32;
    };

    for (int i = 0; i < 3; i++) {
        Data32 d;
        d.f32 = vec.getElem(i);

        unsigned int sign = d.u32 >> 31;
        unsigned int exponent = (d.u32 >> 23) & ((1 << 8) - 1);
        unsigned int mantissa = d.u32 & ((1 << 23) - 1);;

        if (exponent == 0) {
            // zero or denorm -> zero
            mantissa = 0;

        } else if (exponent == 255 && mantissa != 0) {
            // nan -> infinity
            exponent = 31;
            mantissa = 0;

        } else if (exponent >= 127 - 15 + 31) {
            // overflow or infinity -> infinity
            exponent = 31;
            mantissa = 0;

        } else if (exponent <= 127 - 15) {
            // underflow -> zero
            exponent = 0;
            mantissa = 0;

        } else {
            exponent -= 127 - 15;
            mantissa >>= 13;
        }

        hfptr[i] = (unsigned short)((sign << 15) | (exponent << 10) | mantissa);
    }
}

VECTORMATH_FORCE_INLINE Vector3 & Vector3::operator =( const Vector3 & vec )
{
    x = vec.x;
    y = vec.y;
    z = vec.z;
    return *this;
}

VECTORMATH_FORCE_INLINE Vector3 & Vector3::setX( float _x )
{
    x = _x;
    return *this;
}

VECTORMATH_FORCE_INLINE float Vector3::getX( ) const
{
    return x;
}

VECTORMATH_FORCE_INLINE Vector3 & Vector3::setY( float _y )
{
    y = _y;
    return *this;
}

VECTORMATH_FORCE_INLINE float Vector3::getY( ) const
{
    return y;
}

VECTORMATH_FORCE_INLINE Vector3 & Vector3::setZ( float _z )
{
    z = _z;
    return *this;
}

VECTORMATH_FORCE_INLINE float Vector3::getZ( ) const
{
    return z;
}

VECTORMATH_FORCE_INLINE Vector3 & Vector3::setElem( int idx, float value )
{
    *(&x + idx) = value;
    return *this;
}

VECTORMATH_FORCE_INLINE float Vector3::getElem( int idx ) const
{
    return *(&x + idx);
}

VECTORMATH_FORCE_INLINE const Vector3 Vector3::operator +( const Vector3 & vec ) const
{
    return Vector3(
        ( x + vec.x ),
        ( y + vec.y ),
        ( z + vec.z )
    );
}

VECTORMATH_FORCE_INLINE const Vector3 Vector3::operator -( const Vector3 & vec ) const
{
    return Vector3(
        ( x - vec.x ),
        ( y - vec.y ),
        ( z - vec.z )
    );
}

VECTORMATH_FORCE_INLINE const Point3 Vector3::operator +( const Point3 & pnt ) const
{
    return Point3(
        ( x + pnt.getX() ),
        ( y + pnt.getY() ),
        ( z + pnt.getZ() )
    );
}

VECTORMATH_FORCE_INLINE const Vector3 Vector3::operator *( float scalar ) const
{
    return Vector3(
        ( x * scalar ),
        ( y * scalar ),
        ( z * scalar )
    );
}

VECTORMATH_FORCE_INLINE Vector3 & Vector3::operator +=( const Vector3 & vec )
{
    *this = *this + vec;
    return *this;
}

VECTORMATH_FORCE_INLINE Vector3 & Vector3::operator -=( const Vector3 & vec )
{
    *this = *this - vec;
    return *this;
}

VECTORMATH_FORCE_INLINE Vector3 & Vector3::operator *=( float scalar )
{
    *this = *this * scalar;
    return *this;
}

VECTORMATH_FORCE_INLINE const Vector3 Vector3::operator /( float scalar ) const
{
    return Vector3(
        ( x / scalar ),
        ( y / scalar ),
        ( z / scalar )
    );
}

VECTORMATH_FORCE_INLINE Vector3 & Vector3::operator /=( float scalar )
{
    *this = *this / scalar;
    return *this;
}

VECTORMATH_FORCE_INLINE const Vector3 Vector3::operator -( ) const
{
    return Vector3(
        -x,
        -y,
        -z
    );
}

VECTORMATH_FORCE_INLINE const Vector3 operator *( float scalar, const Vector3 & vec )
{
    return vec * scalar;
}

VECTORMATH_FORCE_INLINE const Vector3 operator *( const Vector3 & vec0, const Vector3 & vec1 )
{
    return Vector3(
        ( vec0.getX() * vec1.getX() ),
        ( vec0.getY() * vec1.getY() ),
        ( vec0.getZ() * vec1.getZ() )
    );
}

VECTORMATH_FORCE_INLINE const Vector3 operator /( const Vector3 & vec0, const Vector3 & vec1 )
{
    return Vector3(
        ( vec0.getX() / vec1.getX() ),
        ( vec0.getY() / vec1.getY() ),
        ( vec0.getZ() / vec1.getZ() )
    );
}

VECTORMATH_FORCE_INLINE const Vector3 inv( const Vector3 & vec )
{
    return Vector3(
        ( 1.0f / vec.getX() ),
        ( 1.0f / vec.getY() ),
        ( 1.0f / vec.getZ() )
    );
}

VECTORMATH_FORCE_INLINE const Vector3 sqrt( const Vector3 & vec )
{
    return Vector3(
        sqrtf( vec.getX() ),
        sqrtf( vec.getY() ),
        sqrtf( vec.getZ() )
    );
}

VECTORMATH_FORCE_INLINE const Vector3 rsqrt( const Vector3 & vec )
{
    return Vector3(
        ( 1.0f / sqrtf( vec.getX() ) ),
        ( 1.0f / sqrtf( vec.getY() ) ),
        ( 1.0f / sqrtf( vec.getZ() ) )
    );
}

VECTORMATH_FORCE_INLINE const Vector3 abs( const Vector3 & vec )
{
    return Vector3(
        fabsf( vec.getX() ),
        fabsf( vec.getY() ),
        fabsf( vec.getZ() )
    );
}

VECTORMATH_FORCE_INLINE const Vector3 copySign( const Vector3 & vec0, const Vector3 & vec1 )
{
    return Vector3(
        ( vec1.getX() < 0.0f )? -fabsf( vec0.getX() ) : fabsf( vec0.getX() ),
        ( vec1.getY() < 0.0f )? -fabsf( vec0.getY() ) : fabsf( vec0.getY() ),
        ( vec1.getZ() < 0.0f )? -fabsf( vec0.getZ() ) : fabsf( vec0.getZ() )
    );
}

VECTORMATH_FORCE_INLINE const Vector3 max( const Vector3 & vec0, const Vector3 & vec1 )
{
    return Vector3(
        (vec0.getX() > vec1.getX())? vec0.getX() : vec1.getX(),
        (vec0.getY() > vec1.getY())? vec0.getY() : vec1.getY(),
        (vec0.getZ() > vec1.getZ())? vec0.getZ() : vec1.getZ()
    );
}

VECTORMATH_FORCE_INLINE float maxElem( const Vector3 & vec )
{
    float result;
    result = (vec.getX() > vec.getY())? vec.getX() : vec.getY();
    result = (vec.getZ() > result)? vec.getZ() : result;
    return result;
}

VECTORMATH_FORCE_INLINE const Vector3 min( const Vector3 & vec0, const Vector3 & vec1 )
{
    return Vector3(
        (vec0.getX() < vec1.getX())? vec0.getX() : vec1.getX(),
        (vec0.getY() < vec1.getY())? vec0.getY() : vec1.getY(),
        (vec0.getZ() < vec1.getZ())? vec0.getZ() : vec1.getZ()
    );
}

VECTORMATH_FORCE_INLINE float minElem( const Vector3 & vec )
{
    float result;
    result = (vec.getX() < vec.getY())? vec.getX() : vec.getY();
    result = (vec.getZ() < result)? vec.getZ() : result;
    return result;
}

VECTORMATH_FORCE_INLINE float sum( const Vector3 & vec )
{
    float result;
    result = ( vec.getX() + vec.getY() );
    result = ( result + vec.getZ() );
    return result;
}

VECTORMATH_FORCE_INLINE float dot( const Vector3 & vec0, const Vector3 & vec1 )
{
    float result;
    result = ( vec0.getX() * vec1.getX() );
    result = ( result + ( vec0.getY() * vec1.getY() ) );
    result = ( result + ( vec0.getZ() * vec1.getZ() ) );
    return result;
}

VECTORMATH_FORCE_INLINE float lengthSqr( const Vector3 & vec )
{
    float result;
    result = ( vec.getX() * vec.getX() );
    result = ( result + ( vec.getY() * vec.getY() ) );
    result = ( result + ( vec.getZ() * vec.getZ() ) );
    return result;
}

VECTORMATH_FORCE_INLINE float length( const Vector3 & vec )
{
    return ::sqrtf( lengthSqr( vec ) );
}

VECTORMATH_FORCE_INLINE const Vector3 normalize( const Vector3 & vec )
{
    float lenSqr, lenInv;
    lenSqr = lengthSqr( vec );
    lenInv = ( 1.0f / sqrtf( lenSqr ) );
    return Vector3(
        ( vec.getX() * lenInv ),
        ( vec.getY() * lenInv ),
        ( vec.getZ() * lenInv )
    );
}

VECTORMATH_FORCE_INLINE const Vector3 cross( const Vector3 & vec0, const Vector3 & vec1 )
{
    return Vector3(
        ( ( vec0.getY() * vec1.getZ() ) - ( vec0.getZ() * vec1.getY() ) ),
        ( ( vec0.getZ() * vec1.getX() ) - ( vec0.getX() * vec1.getZ() ) ),
        ( ( vec0.getX() * vec1.getY() ) - ( vec0.getY() * vec1.getX() ) )
    );
}

VECTORMATH_FORCE_INLINE const Vector3 select( const Vector3 & vec0, const Vector3 & vec1, bool select1 )
{
    return Vector3(
        ( select1 )? vec1.getX() : vec0.getX(),
        ( select1 )? vec1.getY() : vec0.getY(),
        ( select1 )? vec1.getZ() : vec0.getZ()
    );
}

#ifdef _VECTORMATH_DEBUG

VECTORMATH_FORCE_INLINE void print( const Vector3 & vec )
{
    printf( "( %f %f %f )\n", vec.getX(), vec.getY(), vec.getZ() );
}

VECTORMATH_FORCE_INLINE void print( const Vector3 & vec, const char * name )
{
    printf( "%s: ( %f %f %f )\n", name, vec.getX(), vec.getY(), vec.getZ() );
}

#endif

VECTORMATH_FORCE_INLINE Vector4::Vector4( const Vector4 & vec )
{
    x = vec.x;
    y = vec.y;
    z = vec.z;
    w = vec.w;
}

VECTORMATH_FORCE_INLINE Vector4::Vector4( float _x, float _y, float _z, float _w )
{
    x = _x;
    y = _y;
    z = _z;
    w = _w;
}

VECTORMATH_FORCE_INLINE Vector4::Vector4( const Vector3 & xyz, float _w )
{
    this->setXYZ( xyz );
    this->setW( _w );
}

VECTORMATH_FORCE_INLINE Vector4::Vector4( const Vector3 & vec )
{
    x = vec.getX();
    y = vec.getY();
    z = vec.getZ();
    w = 0.0f;
}

VECTORMATH_FORCE_INLINE Vector4::Vector4( const Point3 & pnt )
{
    x = pnt.getX();
    y = pnt.getY();
    z = pnt.getZ();
    w = 1.0f;
}

VECTORMATH_FORCE_INLINE Vector4::Vector4( const Quat & quat )
{
    x = quat.getX();
    y = quat.getY();
    z = quat.getZ();
    w = quat.getW();
}

VECTORMATH_FORCE_INLINE Vector4::Vector4( float scalar )
{
    x = scalar;
    y = scalar;
    z = scalar;
    w = scalar;
}

VECTORMATH_FORCE_INLINE const Vector4 Vector4::xAxis( )
{
    return Vector4( 1.0f, 0.0f, 0.0f, 0.0f );
}

VECTORMATH_FORCE_INLINE const Vector4 Vector4::yAxis( )
{
    return Vector4( 0.0f, 1.0f, 0.0f, 0.0f );
}

VECTORMATH_FORCE_INLINE const Vector4 Vector4::zAxis( )
{
    return Vector4( 0.0f, 0.0f, 1.0f, 0.0f );
}

VECTORMATH_FORCE_INLINE const Vector4 Vector4::wAxis( )
{
    return Vector4( 0.0f, 0.0f, 0.0f, 1.0f );
}

VECTORMATH_FORCE_INLINE const Vector4 lerp( float t, const Vector4 & vec0, const Vector4 & vec1 )
{
    return ( vec0 + ( ( vec1 - vec0 ) * t ) );
}

VECTORMATH_FORCE_INLINE const Vector4 slerp( float t, const Vector4 & unitVec0, const Vector4 & unitVec1 )
{
    float recipSinAngle, scale0, scale1, cosAngle, angle;
    cosAngle = dot( unitVec0, unitVec1 );
    if ( cosAngle < _VECTORMATH_SLERP_TOL ) {
        angle = acosf( cosAngle );
        recipSinAngle = ( 1.0f / sinf( angle ) );
        scale0 = ( sinf( ( ( 1.0f - t ) * angle ) ) * recipSinAngle );
        scale1 = ( sinf( ( t * angle ) ) * recipSinAngle );
    } else {
        scale0 = ( 1.0f - t );
        scale1 = t;
    }
    return ( ( unitVec0 * scale0 ) + ( unitVec1 * scale1 ) );
}

VECTORMATH_FORCE_INLINE void loadXYZW( Vector4 & vec, const float * fptr )
{
    vec = Vector4( fptr[0], fptr[1], fptr[2], fptr[3] );
}

VECTORMATH_FORCE_INLINE void storeXYZW( const Vector4 & vec, float * fptr )
{
    fptr[0] = vec.getX();
    fptr[1] = vec.getY();
    fptr[2] = vec.getZ();
    fptr[3] = vec.getW();
}

VECTORMATH_FORCE_INLINE void loadHalfFloats( Vector4 & vec, const unsigned short * hfptr )
{
    union Data32 {
        unsigned int u32;
        float f32;
    };

    for (int i = 0; i < 4; i++) {
        unsigned short fp16 = hfptr[i];
        unsigned int sign = fp16 >> 15;
        unsigned int exponent = (fp16 >> 10) & ((1 << 5) - 1);
        unsigned int mantissa = fp16 & ((1 << 10) - 1);

        if (exponent == 0) {
            // zero
            mantissa = 0;

        } else if (exponent == 31) {
            // infinity or nan -> infinity
            exponent = 255;
        mantissa = 0;

        } else {
            exponent += 127 - 15;
            mantissa <<= 13;
        }

        Data32 d;
        d.u32 = (sign << 31) | (exponent << 23) | mantissa;
        vec.setElem(i, d.f32);
    }
}

VECTORMATH_FORCE_INLINE void storeHalfFloats( const Vector4 & vec, unsigned short * hfptr )
{
    union Data32 {
        unsigned int u32;
        float f32;
    };

    for (int i = 0; i < 4; i++) {
        Data32 d;
        d.f32 = vec.getElem(i);

        unsigned int sign = d.u32 >> 31;
        unsigned int exponent = (d.u32 >> 23) & ((1 << 8) - 1);
        unsigned int mantissa = d.u32 & ((1 << 23) - 1);;

        if (exponent == 0) {
            // zero or denorm -> zero
            mantissa = 0;

        } else if (exponent == 255 && mantissa != 0) {
            // nan -> infinity
            exponent = 31;
            mantissa = 0;

        } else if (exponent >= 127 - 15 + 31) {
            // overflow or infinity -> infinity
            exponent = 31;
            mantissa = 0;

        } else if (exponent <= 127 - 15) {
            // underflow -> zero
            exponent = 0;
            mantissa = 0;

        } else {
            exponent -= 127 - 15;
            mantissa >>= 13;
        }

        hfptr[i] = (unsigned short)((sign << 15) | (exponent << 10) | mantissa);
    }
}

VECTORMATH_FORCE_INLINE Vector4 & Vector4::operator =( const Vector4 & vec )
{
    x = vec.x;
    y = vec.y;
    z = vec.z;
    w = vec.w;
    return *this;
}

VECTORMATH_FORCE_INLINE Vector4 & Vector4::setXYZ( const Vector3 & vec )
{
    x = vec.getX();
    y = vec.getY();
    z = vec.getZ();
    return *this;
}

VECTORMATH_FORCE_INLINE const Vector3 Vector4::getXYZ( ) const
{
    return Vector3( x, y, z );
}

VECTORMATH_FORCE_INLINE Vector4 & Vector4::setX( float _x )
{
    x = _x;
    return *this;
}

VECTORMATH_FORCE_INLINE float Vector4::getX( ) const
{
    return x;
}

VECTORMATH_FORCE_INLINE Vector4 & Vector4::setY( float _y )
{
    y = _y;
    return *this;
}

VECTORMATH_FORCE_INLINE float Vector4::getY( ) const
{
    return y;
}

VECTORMATH_FORCE_INLINE Vector4 & Vector4::setZ( float _z )
{
    z = _z;
    return *this;
}

VECTORMATH_FORCE_INLINE float Vector4::getZ( ) const
{
    return z;
}

VECTORMATH_FORCE_INLINE Vector4 & Vector4::setW( float _w )
{
    w = _w;
    return *this;
}

VECTORMATH_FORCE_INLINE float Vector4::getW( ) const
{
    return w;
}

VECTORMATH_FORCE_INLINE Vector4 & Vector4::setElem( int idx, float value )
{
    *(&x + idx) = value;
    return *this;
}

VECTORMATH_FORCE_INLINE float Vector4::getElem( int idx ) const
{
    return *(&x + idx);
}

VECTORMATH_FORCE_INLINE const Vector4 Vector4::operator +( const Vector4 & vec ) const
{
    return Vector4(
        ( x + vec.x ),
        ( y + vec.y ),
        ( z + vec.z ),
        ( w + vec.w )
    );
}

VECTORMATH_FORCE_INLINE const Vector4 Vector4::operator -( const Vector4 & vec ) const
{
    return Vector4(
        ( x - vec.x ),
        ( y - vec.y ),
        ( z - vec.z ),
        ( w - vec.w )
    );
}

VECTORMATH_FORCE_INLINE const Vector4 Vector4::operator *( float scalar ) const
{
    return Vector4(
        ( x * scalar ),
        ( y * scalar ),
        ( z * scalar ),
        ( w * scalar )
    );
}

VECTORMATH_FORCE_INLINE Vector4 & Vector4::operator +=( const Vector4 & vec )
{
    *this = *this + vec;
    return *this;
}

VECTORMATH_FORCE_INLINE Vector4 & Vector4::operator -=( const Vector4 & vec )
{
    *this = *this - vec;
    return *this;
}

VECTORMATH_FORCE_INLINE Vector4 & Vector4::operator *=( float scalar )
{
    *this = *this * scalar;
    return *this;
}

VECTORMATH_FORCE_INLINE const Vector4 Vector4::operator /( float scalar ) const
{
    return Vector4(
        ( x / scalar ),
        ( y / scalar ),
        ( z / scalar ),
        ( w / scalar )
    );
}

VECTORMATH_FORCE_INLINE Vector4 & Vector4::operator /=( float scalar )
{
    *this = *this / scalar;
    return *this;
}

VECTORMATH_FORCE_INLINE const Vector4 Vector4::operator -( ) const
{
    return Vector4(
        -x,
        -y,
        -z,
        -w
    );
}

VECTORMATH_FORCE_INLINE const Vector4 operator *( float scalar, const Vector4 & vec )
{
    return vec * scalar;
}

VECTORMATH_FORCE_INLINE const Vector4 operator *( const Vector4 & vec0, const Vector4 & vec1 )
{
    return Vector4(
        ( vec0.getX() * vec1.getX() ),
        ( vec0.getY() * vec1.getY() ),
        ( vec0.getZ() * vec1.getZ() ),
        ( vec0.getW() * vec1.getW() )
    );
}

VECTORMATH_FORCE_INLINE const Vector4 operator /( const Vector4 & vec0, const Vector4 & vec1 )
{
    return Vector4(
        ( vec0.getX() / vec1.getX() ),
        ( vec0.getY() / vec1.getY() ),
        ( vec0.getZ() / vec1.getZ() ),
        ( vec0.getW() / vec1.getW() )
    );
}

VECTORMATH_FORCE_INLINE const Vector4 inv( const Vector4 & vec )
{
    return Vector4(
        ( 1.0f / vec.getX() ),
        ( 1.0f / vec.getY() ),
        ( 1.0f / vec.getZ() ),
        ( 1.0f / vec.getW() )
    );
}

VECTORMATH_FORCE_INLINE const Vector4 sqrt( const Vector4 & vec )
{
    return Vector4(
        sqrtf( vec.getX() ),
        sqrtf( vec.getY() ),
        sqrtf( vec.getZ() ),
        sqrtf( vec.getW() )
    );
}

VECTORMATH_FORCE_INLINE const Vector4 rsqrt( const Vector4 & vec )
{
    return Vector4(
        ( 1.0f / sqrtf( vec.getX() ) ),
        ( 1.0f / sqrtf( vec.getY() ) ),
        ( 1.0f / sqrtf( vec.getZ() ) ),
        ( 1.0f / sqrtf( vec.getW() ) )
    );
}

VECTORMATH_FORCE_INLINE const Vector4 abs( const Vector4 & vec )
{
    return Vector4(
        fabsf( vec.getX() ),
        fabsf( vec.getY() ),
        fabsf( vec.getZ() ),
        fabsf( vec.getW() )
    );
}

VECTORMATH_FORCE_INLINE const Vector4 copySign( const Vector4 & vec0, const Vector4 & vec1 )
{
    return Vector4(
        ( vec1.getX() < 0.0f )? -fabsf( vec0.getX() ) : fabsf( vec0.getX() ),
        ( vec1.getY() < 0.0f )? -fabsf( vec0.getY() ) : fabsf( vec0.getY() ),
        ( vec1.getZ() < 0.0f )? -fabsf( vec0.getZ() ) : fabsf( vec0.getZ() ),
        ( vec1.getW() < 0.0f )? -fabsf( vec0.getW() ) : fabsf( vec0.getW() )
    );
}

VECTORMATH_FORCE_INLINE const Vector4 max( const Vector4 & vec0, const Vector4 & vec1 )
{
    return Vector4(
        (vec0.getX() > vec1.getX())? vec0.getX() : vec1.getX(),
        (vec0.getY() > vec1.getY())? vec0.getY() : vec1.getY(),
        (vec0.getZ() > vec1.getZ())? vec0.getZ() : vec1.getZ(),
        (vec0.getW() > vec1.getW())? vec0.getW() : vec1.getW()
    );
}

VECTORMATH_FORCE_INLINE float maxElem( const Vector4 & vec )
{
    float result;
    result = (vec.getX() > vec.getY())? vec.getX() : vec.getY();
    result = (vec.getZ() > result)? vec.getZ() : result;
    result = (vec.getW() > result)? vec.getW() : result;
    return result;
}

VECTORMATH_FORCE_INLINE const Vector4 min( const Vector4 & vec0, const Vector4 & vec1 )
{
    return Vector4(
        (vec0.getX() < vec1.getX())? vec0.getX() : vec1.getX(),
        (vec0.getY() < vec1.getY())? vec0.getY() : vec1.getY(),
        (vec0.getZ() < vec1.getZ())? vec0.getZ() : vec1.getZ(),
        (vec0.getW() < vec1.getW())? vec0.getW() : vec1.getW()
    );
}

VECTORMATH_FORCE_INLINE float minElem( const Vector4 & vec )
{
    float result;
    result = (vec.getX() < vec.getY())? vec.getX() : vec.getY();
    result = (vec.getZ() < result)? vec.getZ() : result;
    result = (vec.getW() < result)? vec.getW() : result;
    return result;
}

VECTORMATH_FORCE_INLINE float sum( const Vector4 & vec )
{
    float result;
    result = ( vec.getX() + vec.getY() );
    result = ( result + vec.getZ() );
    result = ( result + vec.getW() );
    return result;
}

VECTORMATH_FORCE_INLINE float dot( const Vector4 & vec0, const Vector4 & vec1 )
{
    float result;
    result = ( vec0.getX() * vec1.getX() );
    result = ( result + ( vec0.getY() * vec1.getY() ) );
    result = ( result + ( vec0.getZ() * vec1.getZ() ) );
    result = ( result + ( vec0.getW() * vec1.getW() ) );
    return result;
}

VECTORMATH_FORCE_INLINE float lengthSqr( const Vector4 & vec )
{
    float result;
    result = ( vec.getX() * vec.getX() );
    result = ( result + ( vec.getY() * vec.getY() ) );
    result = ( result + ( vec.getZ() * vec.getZ() ) );
    result = ( result + ( vec.getW() * vec.getW() ) );
    return result;
}

VECTORMATH_FORCE_INLINE float length( const Vector4 & vec )
{
    return ::sqrtf( lengthSqr( vec ) );
}

VECTORMATH_FORCE_INLINE const Vector4 normalize( const Vector4 & vec )
{
    float lenSqr, lenInv;
    lenSqr = lengthSqr( vec );
    lenInv = ( 1.0f / sqrtf( lenSqr ) );
    return Vector4(
        ( vec.getX() * lenInv ),
        ( vec.getY() * lenInv ),
        ( vec.getZ() * lenInv ),
        ( vec.getW() * lenInv )
    );
}

VECTORMATH_FORCE_INLINE const Vector4 select( const Vector4 & vec0, const Vector4 & vec1, bool select1 )
{
    return Vector4(
        ( select1 )? vec1.getX() : vec0.getX(),
        ( select1 )? vec1.getY() : vec0.getY(),
        ( select1 )? vec1.getZ() : vec0.getZ(),
        ( select1 )? vec1.getW() : vec0.getW()
    );
}

#ifdef _VECTORMATH_DEBUG

VECTORMATH_FORCE_INLINE void print( const Vector4 & vec )
{
    printf( "( %f %f %f %f )\n", vec.getX(), vec.getY(), vec.getZ(), vec.getW() );
}

VECTORMATH_FORCE_INLINE void print( const Vector4 & vec, const char * name )
{
    printf( "%s: ( %f %f %f %f )\n", name, vec.getX(), vec.getY(), vec.getZ(), vec.getW() );
}

#endif

VECTORMATH_FORCE_INLINE Point3::Point3( const Point3 & pnt )
{
    x = pnt.x;
    y = pnt.y;
    z = pnt.z;
}

VECTORMATH_FORCE_INLINE Point3::Point3( float _x, float _y, float _z )
{
    x = _x;
    y = _y;
    z = _z;
}

VECTORMATH_FORCE_INLINE Point3::Point3( const Vector3 & vec )
{
    x = vec.getX();
    y = vec.getY();
    z = vec.getZ();
}

VECTORMATH_FORCE_INLINE Point3::Point3( float scalar )
{
    x = scalar;
    y = scalar;
    z = scalar;
}

VECTORMATH_FORCE_INLINE const Point3 lerp( float t, const Point3 & pnt0, const Point3 & pnt1 )
{
    return ( pnt0 + ( ( pnt1 - pnt0 ) * t ) );
}

VECTORMATH_FORCE_INLINE void loadXYZ( Point3 & pnt, const float * fptr )
{
    pnt = Point3( fptr[0], fptr[1], fptr[2] );
}

VECTORMATH_FORCE_INLINE void storeXYZ( const Point3 & pnt, float * fptr )
{
    fptr[0] = pnt.getX();
    fptr[1] = pnt.getY();
    fptr[2] = pnt.getZ();
}

VECTORMATH_FORCE_INLINE void loadHalfFloats( Point3 & vec, const unsigned short * hfptr )
{
    union Data32 {
        unsigned int u32;
        float f32;
    };

    for (int i = 0; i < 3; i++) {
        unsigned short fp16 = hfptr[i];
        unsigned int sign = fp16 >> 15;
        unsigned int exponent = (fp16 >> 10) & ((1 << 5) - 1);
        unsigned int mantissa = fp16 & ((1 << 10) - 1);

        if (exponent == 0) {
            // zero
            mantissa = 0;

        } else if (exponent == 31) {
            // infinity or nan -> infinity
            exponent = 255;
        mantissa = 0;

        } else {
            exponent += 127 - 15;
            mantissa <<= 13;
        }

        Data32 d;
        d.u32 = (sign << 31) | (exponent << 23) | mantissa;
        vec.setElem(i, d.f32);
    }
}

VECTORMATH_FORCE_INLINE void storeHalfFloats( const Point3 & vec, unsigned short * hfptr )
{
    union Data32 {
        unsigned int u32;
        float f32;
    };

    for (int i = 0; i < 3; i++) {
        Data32 d;
        d.f32 = vec.getElem(i);

        unsigned int sign = d.u32 >> 31;
        unsigned int exponent = (d.u32 >> 23) & ((1 << 8) - 1);
        unsigned int mantissa = d.u32 & ((1 << 23) - 1);;

        if (exponent == 0) {
            // zero or denorm -> zero
            mantissa = 0;

        } else if (exponent == 255 && mantissa != 0) {
            // nan -> infinity
            exponent = 31;
            mantissa = 0;

        } else if (exponent >= 127 - 15 + 31) {
            // overflow or infinity -> infinity
            exponent = 31;
            mantissa = 0;

        } else if (exponent <= 127 - 15) {
            // underflow -> zero
            exponent = 0;
            mantissa = 0;

        } else {
            exponent -= 127 - 15;
            mantissa >>= 13;
        }

        hfptr[i] = (unsigned short)((sign << 15) | (exponent << 10) | mantissa);
    }
}

VECTORMATH_FORCE_INLINE Point3 & Point3::operator =( const Point3 & pnt )
{
    x = pnt.x;
    y = pnt.y;
    z = pnt.z;
    return *this;
}

VECTORMATH_FORCE_INLINE Point3 & Point3::setX( float _x )
{
    x = _x;
    return *this;
}

VECTORMATH_FORCE_INLINE float Point3::getX( ) const
{
    return x;
}

VECTORMATH_FORCE_INLINE Point3 & Point3::setY( float _y )
{
    y = _y;
    return *this;
}

VECTORMATH_FORCE_INLINE float Point3::getY( ) const
{
    return y;
}

VECTORMATH_FORCE_INLINE Point3 & Point3::setZ( float _z )
{
    z = _z;
    return *this;
}

VECTORMATH_FORCE_INLINE float Point3::getZ( ) const
{
    return z;
}

VECTORMATH_FORCE_INLINE Point3 & Point3::setElem( int idx, float value )
{
    *(&x + idx) = value;
    return *this;
}

VECTORMATH_FORCE_INLINE float Point3::getElem( int idx ) const
{
    return *(&x + idx);
}

VECTORMATH_FORCE_INLINE const Vector3 Point3::operator -( const Point3 & pnt ) const
{
    return Vector3(
        ( x - pnt.x ),
        ( y - pnt.y ),
        ( z - pnt.z )
    );
}

VECTORMATH_FORCE_INLINE const Point3 Point3::operator +( const Vector3 & vec ) const
{
    return Point3(
        ( x + vec.getX() ),
        ( y + vec.getY() ),
        ( z + vec.getZ() )
    );
}

VECTORMATH_FORCE_INLINE const Point3 Point3::operator -( const Vector3 & vec ) const
{
    return Point3(
        ( x - vec.getX() ),
        ( y - vec.getY() ),
        ( z - vec.getZ() )
    );
}

VECTORMATH_FORCE_INLINE Point3 & Point3::operator +=( const Vector3 & vec )
{
    *this = *this + vec;
    return *this;
}

VECTORMATH_FORCE_INLINE Point3 & Point3::operator -=( const Vector3 & vec )
{
    *this = *this - vec;
    return *this;
}

VECTORMATH_FORCE_INLINE const Point3 operator *( const Point3 & pnt0, const Point3 & pnt1 )
{
    return Point3(
        ( pnt0.getX() * pnt1.getX() ),
        ( pnt0.getY() * pnt1.getY() ),
        ( pnt0.getZ() * pnt1.getZ() )
    );
}

VECTORMATH_FORCE_INLINE const Point3 operator /( const Point3 & pnt0, const Point3 & pnt1 )
{
    return Point3(
        ( pnt0.getX() / pnt1.getX() ),
        ( pnt0.getY() / pnt1.getY() ),
        ( pnt0.getZ() / pnt1.getZ() )
    );
}

VECTORMATH_FORCE_INLINE const Point3 inv( const Point3 & pnt )
{
    return Point3(
        ( 1.0f / pnt.getX() ),
        ( 1.0f / pnt.getY() ),
        ( 1.0f / pnt.getZ() )
    );
}

VECTORMATH_FORCE_INLINE const Point3 sqrt( const Point3 & pnt )
{
    return Point3(
        sqrtf( pnt.getX() ),
        sqrtf( pnt.getY() ),
        sqrtf( pnt.getZ() )
    );
}

VECTORMATH_FORCE_INLINE const Point3 rsqrt( const Point3 & pnt )
{
    return Point3(
        ( 1.0f / sqrtf( pnt.getX() ) ),
        ( 1.0f / sqrtf( pnt.getY() ) ),
        ( 1.0f / sqrtf( pnt.getZ() ) )
    );
}

VECTORMATH_FORCE_INLINE const Point3 abs( const Point3 & pnt )
{
    return Point3(
        fabsf( pnt.getX() ),
        fabsf( pnt.getY() ),
        fabsf( pnt.getZ() )
    );
}

VECTORMATH_FORCE_INLINE const Point3 copySign( const Point3 & pnt0, const Point3 & pnt1 )
{
    return Point3(
        ( pnt1.getX() < 0.0f )? -fabsf( pnt0.getX() ) : fabsf( pnt0.getX() ),
        ( pnt1.getY() < 0.0f )? -fabsf( pnt0.getY() ) : fabsf( pnt0.getY() ),
        ( pnt1.getZ() < 0.0f )? -fabsf( pnt0.getZ() ) : fabsf( pnt0.getZ() )
    );
}

VECTORMATH_FORCE_INLINE const Point3 max( const Point3 & pnt0, const Point3 & pnt1 )
{
    return Point3(
        (pnt0.getX() > pnt1.getX())? pnt0.getX() : pnt1.getX(),
        (pnt0.getY() > pnt1.getY())? pnt0.getY() : pnt1.getY(),
        (pnt0.getZ() > pnt1.getZ())? pnt0.getZ() : pnt1.getZ()
    );
}

VECTORMATH_FORCE_INLINE float maxElem( const Point3 & pnt )
{
    float result;
    result = (pnt.getX() > pnt.getY())? pnt.getX() : pnt.getY();
    result = (pnt.getZ() > result)? pnt.getZ() : result;
    return result;
}

VECTORMATH_FORCE_INLINE const Point3 min( const Point3 & pnt0, const Point3 & pnt1 )
{
    return Point3(
        (pnt0.getX() < pnt1.getX())? pnt0.getX() : pnt1.getX(),
        (pnt0.getY() < pnt1.getY())? pnt0.getY() : pnt1.getY(),
        (pnt0.getZ() < pnt1.getZ())? pnt0.getZ() : pnt1.getZ()
    );
}

VECTORMATH_FORCE_INLINE float minElem( const Point3 & pnt )
{
    float result;
    result = (pnt.getX() < pnt.getY())? pnt.getX() : pnt.getY();
    result = (pnt.getZ() < result)? pnt.getZ() : result;
    return result;
}

VECTORMATH_FORCE_INLINE float sum( const Point3 & pnt )
{
    float result;
    result = ( pnt.getX() + pnt.getY() );
    result = ( result + pnt.getZ() );
    return result;
}

VECTORMATH_FORCE_INLINE const Point3 scale( const Point3 & pnt, float scaleVal )
{
    return pnt * Point3( scaleVal );
}

VECTORMATH_FORCE_INLINE const Point3 scale( const Point3 & pnt, const Vector3 & scaleVec )
{
    return pnt * Point3( scaleVec );
}

VECTORMATH_FORCE_INLINE float projection( const Point3 & pnt, const Vector3 & unitVec )
{
    float result;
    result = ( pnt.getX() * unitVec.getX() );
    result = ( result + ( pnt.getY() * unitVec.getY() ) );
    result = ( result + ( pnt.getZ() * unitVec.getZ() ) );
    return result;
}

VECTORMATH_FORCE_INLINE float distSqrFromOrigin( const Point3 & pnt )
{
    return lengthSqr( Vector3( pnt ) );
}

VECTORMATH_FORCE_INLINE float distFromOrigin( const Point3 & pnt )
{
    return length( Vector3( pnt ) );
}

VECTORMATH_FORCE_INLINE float distSqr( const Point3 & pnt0, const Point3 & pnt1 )
{
    return lengthSqr( ( pnt1 - pnt0 ) );
}

VECTORMATH_FORCE_INLINE float dist( const Point3 & pnt0, const Point3 & pnt1 )
{
    return length( ( pnt1 - pnt0 ) );
}

VECTORMATH_FORCE_INLINE const Point3 select( const Point3 & pnt0, const Point3 & pnt1, bool select1 )
{
    return Point3(
        ( select1 )? pnt1.getX() : pnt0.getX(),
        ( select1 )? pnt1.getY() : pnt0.getY(),
        ( select1 )? pnt1.getZ() : pnt0.getZ()
    );
}

#ifdef _VECTORMATH_DEBUG

VECTORMATH_FORCE_INLINE void print( const Point3 & pnt )
{
    printf( "( %f %f %f )\n", pnt.getX(), pnt.getY(), pnt.getZ() );
}

VECTORMATH_FORCE_INLINE void print( const Point3 & pnt, const char * name )
{
    printf( "%s: ( %f %f %f )\n", name, pnt.getX(), pnt.getY(), pnt.getZ() );
}

#endif

} // namespace FmVectormath

#endif
