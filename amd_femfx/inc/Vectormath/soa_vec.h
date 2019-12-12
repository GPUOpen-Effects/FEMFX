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

#ifndef _SOA_VECTORMATH_VEC_H
#define _SOA_VECTORMATH_VEC_H

//-----------------------------------------------------------------------------
// Constants

#define _VECTORMATH_SLERP_TOL 0.999f

//-----------------------------------------------------------------------------
// Definitions

#ifndef _VECTORMATH_INTERNAL_FUNCTIONS
#define _VECTORMATH_INTERNAL_FUNCTIONS

#endif

namespace FmVectormath {

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector3<SoaNFloat>::SoaVector3( const SoaVector3<SoaNFloat> & vec )
{
    x = vec.x;
    y = vec.y;
    z = vec.z;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector3<SoaNFloat>::SoaVector3( SoaNFloat _x, SoaNFloat _y, SoaNFloat _z )
{
    x = _x;
    y = _y;
    z = _z;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector3<SoaNFloat>::SoaVector3( const SoaPoint3<SoaNFloat> & pnt )
{
    x = pnt.getX();
    y = pnt.getY();
    z = pnt.getZ();
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector3<SoaNFloat>::SoaVector3( SoaNFloat scalar )
{
    x = scalar;
    y = scalar;
    z = scalar;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaVector3<SoaNFloat>::xAxis( )
{
    return SoaVector3<SoaNFloat>( 1.0f, 0.0f, 0.0f );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaVector3<SoaNFloat>::yAxis( )
{
    return SoaVector3<SoaNFloat>( 0.0f, 1.0f, 0.0f );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaVector3<SoaNFloat>::zAxis( )
{
    return SoaVector3<SoaNFloat>( 0.0f, 0.0f, 1.0f );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> lerp( SoaNFloat t, const SoaVector3<SoaNFloat> & vec0, const SoaVector3<SoaNFloat> & vec1 )
{
    return ( vec0 + ( ( vec1 - vec0 ) * t ) );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> slerp( SoaNFloat t, const SoaVector3<SoaNFloat> & unitVec0, const SoaVector3<SoaNFloat> & unitVec1 )
{
    SoaNFloat recipSinAngle, scale0, scale1, cosAngle, angle;
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

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void loadXYZ( SoaVector3<SoaNFloat> & vec, const SoaNFloat * fptr )
{
    vec = SoaVector3<SoaNFloat>( fptr[0], fptr[1], fptr[2] );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void storeXYZ( const SoaVector3<SoaNFloat> & vec, SoaNFloat * fptr )
{
    fptr[0] = vec.getX();
    fptr[1] = vec.getY();
    fptr[2] = vec.getZ();
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void loadHalfFloats( SoaVector3<SoaNFloat> & vec, const unsigned short * hfptr )
{
    union Data32 {
        unsigned int u32;
        SoaNFloat f32;
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
        vec[i] = d.f32;
    }
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void storeHalfFloats( const SoaVector3<SoaNFloat> & vec, unsigned short * hfptr )
{
    union Data32 {
        unsigned int u32;
        SoaNFloat f32;
    };

    for (int i = 0; i < 3; i++) {
        Data32 d;
        d.f32 = vec[i];

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

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector3<SoaNFloat> & SoaVector3<SoaNFloat>::operator =( const SoaVector3<SoaNFloat> & vec )
{
    x = vec.x;
    y = vec.y;
    z = vec.z;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector3<SoaNFloat> & SoaVector3<SoaNFloat>::setX( SoaNFloat _x )
{
    x = _x;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaVector3<SoaNFloat>::getX( ) const
{
    return x;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector3<SoaNFloat> & SoaVector3<SoaNFloat>::setY( SoaNFloat _y )
{
    y = _y;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaVector3<SoaNFloat>::getY( ) const
{
    return y;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector3<SoaNFloat> & SoaVector3<SoaNFloat>::setZ( SoaNFloat _z )
{
    z = _z;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaVector3<SoaNFloat>::getZ( ) const
{
    return z;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector3<SoaNFloat> & SoaVector3<SoaNFloat>::setElem( int idx, SoaNFloat value )
{
    *(&x + idx) = value;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaVector3<SoaNFloat>::getElem( int idx ) const
{
    return *(&x + idx);
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaVector3<SoaNFloat>::operator +( const SoaVector3<SoaNFloat> & vec ) const
{
    return SoaVector3<SoaNFloat>(
        ( x + vec.x ),
        ( y + vec.y ),
        ( z + vec.z )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaVector3<SoaNFloat>::operator -( const SoaVector3<SoaNFloat> & vec ) const
{
    return SoaVector3<SoaNFloat>(
        ( x - vec.x ),
        ( y - vec.y ),
        ( z - vec.z )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> SoaVector3<SoaNFloat>::operator +( const SoaPoint3<SoaNFloat> & pnt ) const
{
    return SoaPoint3<SoaNFloat>(
        ( x + pnt.getX() ),
        ( y + pnt.getY() ),
        ( z + pnt.getZ() )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaVector3<SoaNFloat>::operator *( SoaNFloat scalar ) const
{
    return SoaVector3<SoaNFloat>(
        ( x * scalar ),
        ( y * scalar ),
        ( z * scalar )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector3<SoaNFloat> & SoaVector3<SoaNFloat>::operator +=( const SoaVector3<SoaNFloat> & vec )
{
    *this = *this + vec;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector3<SoaNFloat> & SoaVector3<SoaNFloat>::operator -=( const SoaVector3<SoaNFloat> & vec )
{
    *this = *this - vec;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector3<SoaNFloat> & SoaVector3<SoaNFloat>::operator *=( SoaNFloat scalar )
{
    *this = *this * scalar;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaVector3<SoaNFloat>::operator /( SoaNFloat scalar ) const
{
    return SoaVector3<SoaNFloat>(
        ( x / scalar ),
        ( y / scalar ),
        ( z / scalar )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector3<SoaNFloat> & SoaVector3<SoaNFloat>::operator /=( SoaNFloat scalar )
{
    *this = *this / scalar;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaVector3<SoaNFloat>::operator -( ) const
{
    return SoaVector3<SoaNFloat>(
        -x,
        -y,
        -z
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> operator *( SoaNFloat scalar, const SoaVector3<SoaNFloat> & vec )
{
    return vec * scalar;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> operator *( const SoaVector3<SoaNFloat> & vec0, const SoaVector3<SoaNFloat> & vec1 )
{
    return SoaVector3<SoaNFloat>(
        ( vec0.getX() * vec1.getX() ),
        ( vec0.getY() * vec1.getY() ),
        ( vec0.getZ() * vec1.getZ() )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> operator /( const SoaVector3<SoaNFloat> & vec0, const SoaVector3<SoaNFloat> & vec1 )
{
    return SoaVector3<SoaNFloat>(
        ( vec0.getX() / vec1.getX() ),
        ( vec0.getY() / vec1.getY() ),
        ( vec0.getZ() / vec1.getZ() )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> inv( const SoaVector3<SoaNFloat> & vec )
{
    return SoaVector3<SoaNFloat>(
        ( 1.0f / vec.getX() ),
        ( 1.0f / vec.getY() ),
        ( 1.0f / vec.getZ() )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> sqrt( const SoaVector3<SoaNFloat> & vec )
{
    return SoaVector3<SoaNFloat>(
        sqrtf( vec.getX() ),
        sqrtf( vec.getY() ),
        sqrtf( vec.getZ() )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> rsqrt( const SoaVector3<SoaNFloat> & vec )
{
    return SoaVector3<SoaNFloat>(
        ( 1.0f / sqrtf( vec.getX() ) ),
        ( 1.0f / sqrtf( vec.getY() ) ),
        ( 1.0f / sqrtf( vec.getZ() ) )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> abs( const SoaVector3<SoaNFloat> & vec )
{
    return SoaVector3<SoaNFloat>(
        fabsf( vec.getX() ),
        fabsf( vec.getY() ),
        fabsf( vec.getZ() )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> copySign( const SoaVector3<SoaNFloat> & vec0, const SoaVector3<SoaNFloat> & vec1 )
{
    return SoaVector3<SoaNFloat>(
        select(fabsf( vec0.getX() ), -fabsf( vec0.getX() ), ( vec1.getX() < 0.0f )),
        select(fabsf( vec0.getY() ), -fabsf( vec0.getY() ), ( vec1.getY() < 0.0f )),
        select(fabsf( vec0.getZ() ), -fabsf( vec0.getZ() ), ( vec1.getZ() < 0.0f ))
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> max( const SoaVector3<SoaNFloat> & vec0, const SoaVector3<SoaNFloat> & vec1 )
{
    return SoaVector3<SoaNFloat>(
        select(vec1.getX(), vec0.getX(), (vec0.getX() > vec1.getX())),
        select(vec1.getY(), vec0.getY(), (vec0.getY() > vec1.getY())),
        select(vec1.getZ(), vec0.getZ(), (vec0.getZ() > vec1.getZ()))
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat maxElem( const SoaVector3<SoaNFloat> & vec )
{
    SoaNFloat result;
    result = select(vec.getY(), vec.getX(), (vec.getX() > vec.getY()));
    result = select(result, vec.getZ(), (vec.getZ() > result));
    return result;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> min( const SoaVector3<SoaNFloat> & vec0, const SoaVector3<SoaNFloat> & vec1 )
{
    return SoaVector3<SoaNFloat>(
        select(vec1.getX(), vec0.getX(), (vec0.getX() < vec1.getX())),
        select(vec1.getY(), vec0.getY(), (vec0.getY() < vec1.getY())),
        select(vec1.getZ(), vec0.getZ(), (vec0.getZ() < vec1.getZ()))
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat minElem( const SoaVector3<SoaNFloat> & vec )
{
    SoaNFloat result;
    result = select(vec.getY(), vec.getX(), (vec.getX() < vec.getY()));
    result = select(result, vec.getZ(), (vec.getZ() < result));
    return result;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat sum( const SoaVector3<SoaNFloat> & vec )
{
    SoaNFloat result;
    result = ( vec.getX() + vec.getY() );
    result = ( result + vec.getZ() );
    return result;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat dot( const SoaVector3<SoaNFloat> & vec0, const SoaVector3<SoaNFloat> & vec1 )
{
    SoaNFloat result;
    result = ( vec0.getX() * vec1.getX() );
    result = ( result + ( vec0.getY() * vec1.getY() ) );
    result = ( result + ( vec0.getZ() * vec1.getZ() ) );
    return result;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat lengthSqr( const SoaVector3<SoaNFloat> & vec )
{
    SoaNFloat result;
    result = ( vec.getX() * vec.getX() );
    result = ( result + ( vec.getY() * vec.getY() ) );
    result = ( result + ( vec.getZ() * vec.getZ() ) );
    return result;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat length( const SoaVector3<SoaNFloat> & vec )
{
    return sqrtf( lengthSqr( vec ) );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> normalize( const SoaVector3<SoaNFloat> & vec )
{
    SoaNFloat lenSqr, lenInv;
    lenSqr = lengthSqr( vec );
    lenInv = ( 1.0f / sqrtf( lenSqr ) );
    return SoaVector3<SoaNFloat>(
        ( vec.getX() * lenInv ),
        ( vec.getY() * lenInv ),
        ( vec.getZ() * lenInv )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> cross( const SoaVector3<SoaNFloat> & vec0, const SoaVector3<SoaNFloat> & vec1 )
{
    return SoaVector3<SoaNFloat>(
        ( ( vec0.getY() * vec1.getZ() ) - ( vec0.getZ() * vec1.getY() ) ),
        ( ( vec0.getZ() * vec1.getX() ) - ( vec0.getX() * vec1.getZ() ) ),
        ( ( vec0.getX() * vec1.getY() ) - ( vec0.getY() * vec1.getX() ) )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> select( const SoaVector3<SoaNFloat> & vec0, const SoaVector3<SoaNFloat> & vec1, typename SoaNFloat::SoaBool select1 )
{
    return SoaVector3<SoaNFloat>(
        select(vec0.getX(), vec1.getX(), ( select1 )),
        select(vec0.getY(), vec1.getY(), ( select1 )),
        select(vec0.getZ(), vec1.getZ(), ( select1 ))
    );
}

#ifdef _VECTORMATH_DEBUG

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaVector3<SoaNFloat> & vec )
{
    print(vec.getX(), "x");
    print(vec.getY(), "y");
    print(vec.getZ(), "z");
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaVector3<SoaNFloat> & vec, const char * name )
{
    printf("%s:\n", name);
    print(vec.getX(), "x");
    print(vec.getY(), "y");
    print(vec.getZ(), "z");
}

#endif

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector4<SoaNFloat>::SoaVector4( const SoaVector4<SoaNFloat> & vec )
{
    x = vec.x;
    y = vec.y;
    z = vec.z;
    w = vec.w;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector4<SoaNFloat>::SoaVector4( SoaNFloat _x, SoaNFloat _y, SoaNFloat _z, SoaNFloat _w )
{
    x = _x;
    y = _y;
    z = _z;
    w = _w;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector4<SoaNFloat>::SoaVector4( const SoaVector3<SoaNFloat> & xyz, SoaNFloat _w )
{
    this->setXYZ( xyz );
    this->setW( _w );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector4<SoaNFloat>::SoaVector4( const SoaVector3<SoaNFloat> & vec )
{
    x = vec.getX();
    y = vec.getY();
    z = vec.getZ();
    w = 0.0f;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector4<SoaNFloat>::SoaVector4( const SoaPoint3<SoaNFloat> & pnt )
{
    x = pnt.getX();
    y = pnt.getY();
    z = pnt.getZ();
    w = 1.0f;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector4<SoaNFloat>::SoaVector4( const SoaQuat<SoaNFloat> & quat )
{
    x = quat.getX();
    y = quat.getY();
    z = quat.getZ();
    w = quat.getW();
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector4<SoaNFloat>::SoaVector4( SoaNFloat scalar )
{
    x = scalar;
    y = scalar;
    z = scalar;
    w = scalar;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> SoaVector4<SoaNFloat>::xAxis( )
{
    return SoaVector4<SoaNFloat>( 1.0f, 0.0f, 0.0f, 0.0f );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> SoaVector4<SoaNFloat>::yAxis( )
{
    return SoaVector4<SoaNFloat>( 0.0f, 1.0f, 0.0f, 0.0f );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> SoaVector4<SoaNFloat>::zAxis( )
{
    return SoaVector4<SoaNFloat>( 0.0f, 0.0f, 1.0f, 0.0f );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> SoaVector4<SoaNFloat>::wAxis( )
{
    return SoaVector4<SoaNFloat>( 0.0f, 0.0f, 0.0f, 1.0f );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> lerp( SoaNFloat t, const SoaVector4<SoaNFloat> & vec0, const SoaVector4<SoaNFloat> & vec1 )
{
    return ( vec0 + ( ( vec1 - vec0 ) * t ) );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> slerp( SoaNFloat t, const SoaVector4<SoaNFloat> & unitVec0, const SoaVector4<SoaNFloat> & unitVec1 )
{
    SoaNFloat recipSinAngle, scale0, scale1, cosAngle, angle;
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

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void loadXYZW( SoaVector4<SoaNFloat> & vec, const SoaNFloat * fptr )
{
    vec = SoaVector4<SoaNFloat>( fptr[0], fptr[1], fptr[2], fptr[3] );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void storeXYZW( const SoaVector4<SoaNFloat> & vec, SoaNFloat * fptr )
{
    fptr[0] = vec.getX();
    fptr[1] = vec.getY();
    fptr[2] = vec.getZ();
    fptr[3] = vec.getW();
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void loadHalfFloats( SoaVector4<SoaNFloat> & vec, const unsigned short * hfptr )
{
    union Data32 {
        unsigned int u32;
        SoaNFloat f32;
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
        vec[i] = d.f32;
    }
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void storeHalfFloats( const SoaVector4<SoaNFloat> & vec, unsigned short * hfptr )
{
    union Data32 {
        unsigned int u32;
        SoaNFloat f32;
    };

    for (int i = 0; i < 4; i++) {
        Data32 d;
        d.f32 = vec[i];

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

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector4<SoaNFloat> & SoaVector4<SoaNFloat>::operator =( const SoaVector4<SoaNFloat> & vec )
{
    x = vec.x;
    y = vec.y;
    z = vec.z;
    w = vec.w;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector4<SoaNFloat> & SoaVector4<SoaNFloat>::setXYZ( const SoaVector3<SoaNFloat> & vec )
{
    x = vec.getX();
    y = vec.getY();
    z = vec.getZ();
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaVector4<SoaNFloat>::getXYZ( ) const
{
    return SoaVector3<SoaNFloat>( x, y, z );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector4<SoaNFloat> & SoaVector4<SoaNFloat>::setX( SoaNFloat _x )
{
    x = _x;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaVector4<SoaNFloat>::getX( ) const
{
    return x;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector4<SoaNFloat> & SoaVector4<SoaNFloat>::setY( SoaNFloat _y )
{
    y = _y;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaVector4<SoaNFloat>::getY( ) const
{
    return y;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector4<SoaNFloat> & SoaVector4<SoaNFloat>::setZ( SoaNFloat _z )
{
    z = _z;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaVector4<SoaNFloat>::getZ( ) const
{
    return z;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector4<SoaNFloat> & SoaVector4<SoaNFloat>::setW( SoaNFloat _w )
{
    w = _w;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaVector4<SoaNFloat>::getW( ) const
{
    return w;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector4<SoaNFloat> & SoaVector4<SoaNFloat>::setElem( int idx, SoaNFloat value )
{
    *(&x + idx) = value;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaVector4<SoaNFloat>::getElem( int idx ) const
{
    return *(&x + idx);
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> SoaVector4<SoaNFloat>::operator +( const SoaVector4<SoaNFloat> & vec ) const
{
    return SoaVector4<SoaNFloat>(
        ( x + vec.x ),
        ( y + vec.y ),
        ( z + vec.z ),
        ( w + vec.w )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> SoaVector4<SoaNFloat>::operator -( const SoaVector4<SoaNFloat> & vec ) const
{
    return SoaVector4<SoaNFloat>(
        ( x - vec.x ),
        ( y - vec.y ),
        ( z - vec.z ),
        ( w - vec.w )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> SoaVector4<SoaNFloat>::operator *( SoaNFloat scalar ) const
{
    return SoaVector4<SoaNFloat>(
        ( x * scalar ),
        ( y * scalar ),
        ( z * scalar ),
        ( w * scalar )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector4<SoaNFloat> & SoaVector4<SoaNFloat>::operator +=( const SoaVector4<SoaNFloat> & vec )
{
    *this = *this + vec;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector4<SoaNFloat> & SoaVector4<SoaNFloat>::operator -=( const SoaVector4<SoaNFloat> & vec )
{
    *this = *this - vec;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector4<SoaNFloat> & SoaVector4<SoaNFloat>::operator *=( SoaNFloat scalar )
{
    *this = *this * scalar;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> SoaVector4<SoaNFloat>::operator /( SoaNFloat scalar ) const
{
    return SoaVector4<SoaNFloat>(
        ( x / scalar ),
        ( y / scalar ),
        ( z / scalar ),
        ( w / scalar )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaVector4<SoaNFloat> & SoaVector4<SoaNFloat>::operator /=( SoaNFloat scalar )
{
    *this = *this / scalar;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> SoaVector4<SoaNFloat>::operator -( ) const
{
    return SoaVector4<SoaNFloat>(
        -x,
        -y,
        -z,
        -w
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> operator *( SoaNFloat scalar, const SoaVector4<SoaNFloat> & vec )
{
    return vec * scalar;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> operator *( const SoaVector4<SoaNFloat> & vec0, const SoaVector4<SoaNFloat> & vec1 )
{
    return SoaVector4<SoaNFloat>(
        ( vec0.getX() * vec1.getX() ),
        ( vec0.getY() * vec1.getY() ),
        ( vec0.getZ() * vec1.getZ() ),
        ( vec0.getW() * vec1.getW() )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> operator /( const SoaVector4<SoaNFloat> & vec0, const SoaVector4<SoaNFloat> & vec1 )
{
    return SoaVector4<SoaNFloat>(
        ( vec0.getX() / vec1.getX() ),
        ( vec0.getY() / vec1.getY() ),
        ( vec0.getZ() / vec1.getZ() ),
        ( vec0.getW() / vec1.getW() )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> inv( const SoaVector4<SoaNFloat> & vec )
{
    return SoaVector4<SoaNFloat>(
        ( 1.0f / vec.getX() ),
        ( 1.0f / vec.getY() ),
        ( 1.0f / vec.getZ() ),
        ( 1.0f / vec.getW() )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> sqrt( const SoaVector4<SoaNFloat> & vec )
{
    return SoaVector4<SoaNFloat>(
        sqrtf( vec.getX() ),
        sqrtf( vec.getY() ),
        sqrtf( vec.getZ() ),
        sqrtf( vec.getW() )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> rsqrt( const SoaVector4<SoaNFloat> & vec )
{
    return SoaVector4<SoaNFloat>(
        ( 1.0f / sqrtf( vec.getX() ) ),
        ( 1.0f / sqrtf( vec.getY() ) ),
        ( 1.0f / sqrtf( vec.getZ() ) ),
        ( 1.0f / sqrtf( vec.getW() ) )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> abs( const SoaVector4<SoaNFloat> & vec )
{
    return SoaVector4<SoaNFloat>(
        fabsf( vec.getX() ),
        fabsf( vec.getY() ),
        fabsf( vec.getZ() ),
        fabsf( vec.getW() )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> copySign( const SoaVector4<SoaNFloat> & vec0, const SoaVector4<SoaNFloat> & vec1 )
{
    return SoaVector4<SoaNFloat>(
        select(fabsf( vec0.getX() ), -fabsf( vec0.getX() ), ( vec1.getX() < 0.0f )),
        select(fabsf( vec0.getY() ), -fabsf( vec0.getY() ), ( vec1.getY() < 0.0f )),
        select(fabsf( vec0.getZ() ), -fabsf( vec0.getZ() ), ( vec1.getZ() < 0.0f )),
        select(fabsf( vec0.getW() ), -fabsf( vec0.getW() ), ( vec1.getW() < 0.0f ))
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> max( const SoaVector4<SoaNFloat> & vec0, const SoaVector4<SoaNFloat> & vec1 )
{
    return SoaVector4<SoaNFloat>(
        select(vec1.getX(), vec0.getX(), (vec0.getX() > vec1.getX())),
        select(vec1.getY(), vec0.getY(), (vec0.getY() > vec1.getY())),
        select(vec1.getZ(), vec0.getZ(), (vec0.getZ() > vec1.getZ())),
        select(vec1.getW(), vec0.getW(), (vec0.getW() > vec1.getW()))
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat maxElem( const SoaVector4<SoaNFloat> & vec )
{
    SoaNFloat result;
    result = select(vec.getY(), vec.getX(), (vec.getX() > vec.getY()));
    result = select(result, vec.getZ(), (vec.getZ() > result));
    result = select(result, vec.getW(), (vec.getW() > result));
    return result;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> min( const SoaVector4<SoaNFloat> & vec0, const SoaVector4<SoaNFloat> & vec1 )
{
    return SoaVector4<SoaNFloat>(
        select(vec1.getX(), vec0.getX(), (vec0.getX() < vec1.getX())),
        select(vec1.getY(), vec0.getY(), (vec0.getY() < vec1.getY())),
        select(vec1.getZ(), vec0.getZ(), (vec0.getZ() < vec1.getZ())),
        select(vec1.getW(), vec0.getW(), (vec0.getW() < vec1.getW()))
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat minElem( const SoaVector4<SoaNFloat> & vec )
{
    SoaNFloat result;
    result = select(vec.getY(), vec.getX(), (vec.getX() < vec.getY()));
    result = select(result, vec.getZ(), (vec.getZ() < result));
    result = select(result, vec.getW(), (vec.getW() < result));
    return result;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat sum( const SoaVector4<SoaNFloat> & vec )
{
    SoaNFloat result;
    result = ( vec.getX() + vec.getY() );
    result = ( result + vec.getZ() );
    result = ( result + vec.getW() );
    return result;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat dot( const SoaVector4<SoaNFloat> & vec0, const SoaVector4<SoaNFloat> & vec1 )
{
    SoaNFloat result;
    result = ( vec0.getX() * vec1.getX() );
    result = ( result + ( vec0.getY() * vec1.getY() ) );
    result = ( result + ( vec0.getZ() * vec1.getZ() ) );
    result = ( result + ( vec0.getW() * vec1.getW() ) );
    return result;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat lengthSqr( const SoaVector4<SoaNFloat> & vec )
{
    SoaNFloat result;
    result = ( vec.getX() * vec.getX() );
    result = ( result + ( vec.getY() * vec.getY() ) );
    result = ( result + ( vec.getZ() * vec.getZ() ) );
    result = ( result + ( vec.getW() * vec.getW() ) );
    return result;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat length( const SoaVector4<SoaNFloat> & vec )
{
    return ::sqrtf( lengthSqr( vec ) );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> normalize( const SoaVector4<SoaNFloat> & vec )
{
    SoaNFloat lenSqr, lenInv;
    lenSqr = lengthSqr( vec );
    lenInv = ( 1.0f / sqrtf( lenSqr ) );
    return SoaVector4<SoaNFloat>(
        ( vec.getX() * lenInv ),
        ( vec.getY() * lenInv ),
        ( vec.getZ() * lenInv ),
        ( vec.getW() * lenInv )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> select( const SoaVector4<SoaNFloat> & vec0, const SoaVector4<SoaNFloat> & vec1, typename SoaNFloat::SoaBool select1 )
{
    return SoaVector4<SoaNFloat>(
        select(vec0.getX(), vec1.getX(), ( select1 )),
        select(vec0.getY(), vec1.getY(), ( select1 )),
        select(vec0.getZ(), vec1.getZ(), ( select1 )),
        select(vec0.getW(), vec1.getW(), ( select1 ))
    );
}

#ifdef _VECTORMATH_DEBUG

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaVector4<SoaNFloat> & vec )
{
    print(vec.getX(), "x");
    print(vec.getY(), "y");
    print(vec.getZ(), "z");
    print(vec.getW(), "w");
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaVector4<SoaNFloat> & vec, const char * name )
{
    printf("%s:\n", name);
    print(vec.getX(), "x");
    print(vec.getY(), "y");
    print(vec.getZ(), "z");
    print(vec.getW(), "w");
}

#endif

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaPoint3<SoaNFloat>::SoaPoint3( const SoaPoint3<SoaNFloat> & pnt )
{
    x = pnt.x;
    y = pnt.y;
    z = pnt.z;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaPoint3<SoaNFloat>::SoaPoint3( SoaNFloat _x, SoaNFloat _y, SoaNFloat _z )
{
    x = _x;
    y = _y;
    z = _z;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaPoint3<SoaNFloat>::SoaPoint3( const SoaVector3<SoaNFloat> & vec )
{
    x = vec.getX();
    y = vec.getY();
    z = vec.getZ();
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaPoint3<SoaNFloat>::SoaPoint3( SoaNFloat scalar )
{
    x = scalar;
    y = scalar;
    z = scalar;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> lerp( SoaNFloat t, const SoaPoint3<SoaNFloat> & pnt0, const SoaPoint3<SoaNFloat> & pnt1 )
{
    return ( pnt0 + ( ( pnt1 - pnt0 ) * t ) );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void loadXYZ( SoaPoint3<SoaNFloat> & pnt, const SoaNFloat * fptr )
{
    pnt = SoaPoint3<SoaNFloat>( fptr[0], fptr[1], fptr[2] );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void storeXYZ( const SoaPoint3<SoaNFloat> & pnt, SoaNFloat * fptr )
{
    fptr[0] = pnt.getX();
    fptr[1] = pnt.getY();
    fptr[2] = pnt.getZ();
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void loadHalfFloats( SoaPoint3<SoaNFloat> & vec, const unsigned short * hfptr )
{
    union Data32 {
        unsigned int u32;
        SoaNFloat f32;
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
        vec[i] = d.f32;
    }
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void storeHalfFloats( const SoaPoint3<SoaNFloat> & vec, unsigned short * hfptr )
{
    union Data32 {
        unsigned int u32;
        SoaNFloat f32;
    };

    for (int i = 0; i < 3; i++) {
        Data32 d;
        d.f32 = vec[i];

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

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaPoint3<SoaNFloat> & SoaPoint3<SoaNFloat>::operator =( const SoaPoint3<SoaNFloat> & pnt )
{
    x = pnt.x;
    y = pnt.y;
    z = pnt.z;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaPoint3<SoaNFloat> & SoaPoint3<SoaNFloat>::setX( SoaNFloat _x )
{
    x = _x;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaPoint3<SoaNFloat>::getX( ) const
{
    return x;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaPoint3<SoaNFloat> & SoaPoint3<SoaNFloat>::setY( SoaNFloat _y )
{
    y = _y;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaPoint3<SoaNFloat>::getY( ) const
{
    return y;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaPoint3<SoaNFloat> & SoaPoint3<SoaNFloat>::setZ( SoaNFloat _z )
{
    z = _z;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaPoint3<SoaNFloat>::getZ( ) const
{
    return z;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaPoint3<SoaNFloat> & SoaPoint3<SoaNFloat>::setElem( int idx, SoaNFloat value )
{
    *(&x + idx) = value;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaPoint3<SoaNFloat>::getElem( int idx ) const
{
    return *(&x + idx);
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaPoint3<SoaNFloat>::operator -( const SoaPoint3<SoaNFloat> & pnt ) const
{
    return SoaVector3<SoaNFloat>(
        ( x - pnt.x ),
        ( y - pnt.y ),
        ( z - pnt.z )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> SoaPoint3<SoaNFloat>::operator +( const SoaVector3<SoaNFloat> & vec ) const
{
    return SoaPoint3<SoaNFloat>(
        ( x + vec.getX() ),
        ( y + vec.getY() ),
        ( z + vec.getZ() )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> SoaPoint3<SoaNFloat>::operator -( const SoaVector3<SoaNFloat> & vec ) const
{
    return SoaPoint3<SoaNFloat>(
        ( x - vec.getX() ),
        ( y - vec.getY() ),
        ( z - vec.getZ() )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaPoint3<SoaNFloat> & SoaPoint3<SoaNFloat>::operator +=( const SoaVector3<SoaNFloat> & vec )
{
    *this = *this + vec;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaPoint3<SoaNFloat> & SoaPoint3<SoaNFloat>::operator -=( const SoaVector3<SoaNFloat> & vec )
{
    *this = *this - vec;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> operator *( const SoaPoint3<SoaNFloat> & pnt0, const SoaPoint3<SoaNFloat> & pnt1 )
{
    return SoaPoint3<SoaNFloat>(
        ( pnt0.getX() * pnt1.getX() ),
        ( pnt0.getY() * pnt1.getY() ),
        ( pnt0.getZ() * pnt1.getZ() )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> operator /( const SoaPoint3<SoaNFloat> & pnt0, const SoaPoint3<SoaNFloat> & pnt1 )
{
    return SoaPoint3<SoaNFloat>(
        ( pnt0.getX() / pnt1.getX() ),
        ( pnt0.getY() / pnt1.getY() ),
        ( pnt0.getZ() / pnt1.getZ() )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> inv( const SoaPoint3<SoaNFloat> & pnt )
{
    return SoaPoint3<SoaNFloat>(
        ( 1.0f / pnt.getX() ),
        ( 1.0f / pnt.getY() ),
        ( 1.0f / pnt.getZ() )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> sqrt( const SoaPoint3<SoaNFloat> & pnt )
{
    return SoaPoint3<SoaNFloat>(
        sqrtf( pnt.getX() ),
        sqrtf( pnt.getY() ),
        sqrtf( pnt.getZ() )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> rsqrt( const SoaPoint3<SoaNFloat> & pnt )
{
    return SoaPoint3<SoaNFloat>(
        ( 1.0f / sqrtf( pnt.getX() ) ),
        ( 1.0f / sqrtf( pnt.getY() ) ),
        ( 1.0f / sqrtf( pnt.getZ() ) )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> abs( const SoaPoint3<SoaNFloat> & pnt )
{
    return SoaPoint3<SoaNFloat>(
        fabsf( pnt.getX() ),
        fabsf( pnt.getY() ),
        fabsf( pnt.getZ() )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> copySign( const SoaPoint3<SoaNFloat> & pnt0, const SoaPoint3<SoaNFloat> & pnt1 )
{
    return SoaPoint3<SoaNFloat>(
        select(fabsf( pnt0.getX() ), -fabsf( pnt0.getX() ), ( pnt1.getX() < 0.0f )),
        select(fabsf( pnt0.getY() ), -fabsf( pnt0.getY() ), ( pnt1.getY() < 0.0f )),
        select(fabsf( pnt0.getZ() ), -fabsf( pnt0.getZ() ), ( pnt1.getZ() < 0.0f ))
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> max( const SoaPoint3<SoaNFloat> & pnt0, const SoaPoint3<SoaNFloat> & pnt1 )
{
    return SoaPoint3<SoaNFloat>(
        select(pnt1.getX(), pnt0.getX(), (pnt0.getX() > pnt1.getX())),
        select(pnt1.getY(), pnt0.getY(), (pnt0.getY() > pnt1.getY())),
        select(pnt1.getZ(), pnt0.getZ(), (pnt0.getZ() > pnt1.getZ()))
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat maxElem( const SoaPoint3<SoaNFloat> & pnt )
{
    SoaNFloat result;
    result = select(pnt.getY(), pnt.getX(), (pnt.getX() > pnt.getY()));
    result = select(result, pnt.getZ(), (pnt.getZ() > result));
    return result;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> min( const SoaPoint3<SoaNFloat> & pnt0, const SoaPoint3<SoaNFloat> & pnt1 )
{
    return SoaPoint3<SoaNFloat>(
        select(pnt1.getX(), pnt0.getX(), (pnt0.getX() < pnt1.getX())),
        select(pnt1.getY(), pnt0.getY(), (pnt0.getY() < pnt1.getY())),
        select(pnt1.getZ(), pnt0.getZ(), (pnt0.getZ() < pnt1.getZ()))
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat minElem( const SoaPoint3<SoaNFloat> & pnt )
{
    SoaNFloat result;
    result = select(pnt.getY(), pnt.getX(), (pnt.getX() < pnt.getY()));
    result = select(result, pnt.getZ(), (pnt.getZ() < result));
    return result;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat sum( const SoaPoint3<SoaNFloat> & pnt )
{
    SoaNFloat result;
    result = ( pnt.getX() + pnt.getY() );
    result = ( result + pnt.getZ() );
    return result;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> scale( const SoaPoint3<SoaNFloat> & pnt, SoaNFloat scaleVal )
{
    return pnt * SoaPoint3<SoaNFloat>( scaleVal );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> scale( const SoaPoint3<SoaNFloat> & pnt, const SoaVector3<SoaNFloat> & scaleVec )
{
    return pnt * SoaPoint3<SoaNFloat>( scaleVec );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat projection( const SoaPoint3<SoaNFloat> & pnt, const SoaVector3<SoaNFloat> & unitVec )
{
    SoaNFloat result;
    result = ( pnt.getX() * unitVec.getX() );
    result = ( result + ( pnt.getY() * unitVec.getY() ) );
    result = ( result + ( pnt.getZ() * unitVec.getZ() ) );
    return result;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat distSqrFromOrigin( const SoaPoint3<SoaNFloat> & pnt )
{
    return lengthSqr( SoaVector3<SoaNFloat>( pnt ) );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat distFromOrigin( const SoaPoint3<SoaNFloat> & pnt )
{
    return length( SoaVector3<SoaNFloat>( pnt ) );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat distSqr( const SoaPoint3<SoaNFloat> & pnt0, const SoaPoint3<SoaNFloat> & pnt1 )
{
    return lengthSqr( ( pnt1 - pnt0 ) );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat dist( const SoaPoint3<SoaNFloat> & pnt0, const SoaPoint3<SoaNFloat> & pnt1 )
{
    return length( ( pnt1 - pnt0 ) );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> select( const SoaPoint3<SoaNFloat> & pnt0, const SoaPoint3<SoaNFloat> & pnt1, typename SoaNFloat::SoaBool select1 )
{
    return SoaPoint3<SoaNFloat>(
        select(pnt0.getX(), pnt1.getX(), ( select1 )),
        select(pnt0.getY(), pnt1.getY(), ( select1 )),
        select(pnt0.getZ(), pnt1.getZ(), ( select1 ))
    );
}

#ifdef _VECTORMATH_DEBUG

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaPoint3<SoaNFloat> & pnt )
{
    print(vec.getX(), "x");
    print(vec.getY(), "y");
    print(vec.getZ(), "z");
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaPoint3<SoaNFloat> & pnt, const char * name )
{
    printf("%s:\n", name);
    print(vec.getX(), "x");
    print(vec.getY(), "y");
    print(vec.getZ(), "z");
}

#endif

} // namespace FmVectormath

#endif
