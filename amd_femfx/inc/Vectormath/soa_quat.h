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

#ifndef _SOA_VECTORMATH_QUAT_H
#define _SOA_VECTORMATH_QUAT_H

//-----------------------------------------------------------------------------
// Definitions

#ifndef _VECTORMATH_INTERNAL_FUNCTIONS
#define _VECTORMATH_INTERNAL_FUNCTIONS

#endif

namespace FmVectormath {

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaQuat<SoaNFloat>::SoaQuat( const SoaQuat<SoaNFloat> & quat )
{
    x = quat.x;
    y = quat.y;
    z = quat.z;
    w = quat.w;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaQuat<SoaNFloat>::SoaQuat( SoaNFloat _x, SoaNFloat _y, SoaNFloat _z, SoaNFloat _w )
{
    x = _x;
    y = _y;
    z = _z;
    w = _w;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaQuat<SoaNFloat>::SoaQuat( const SoaVector3<SoaNFloat> & xyz, SoaNFloat _w )
{
    this->setXYZ( xyz );
    this->setW( _w );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaQuat<SoaNFloat>::SoaQuat( const SoaVector4<SoaNFloat> & vec )
{
    x = vec.getX();
    y = vec.getY();
    z = vec.getZ();
    w = vec.getW();
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaQuat<SoaNFloat>::SoaQuat( SoaNFloat scalar )
{
    x = scalar;
    y = scalar;
    z = scalar;
    w = scalar;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> SoaQuat<SoaNFloat>::identity( )
{
    return SoaQuat<SoaNFloat>( 0.0f, 0.0f, 0.0f, 1.0f );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> lerp( SoaNFloat t, const SoaQuat<SoaNFloat> & quat0, const SoaQuat<SoaNFloat> & quat1 )
{
    return ( quat0 + ( ( quat1 - quat0 ) * t ) );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> slerp( SoaNFloat t, const SoaQuat<SoaNFloat> & unitQuat0, const SoaQuat<SoaNFloat> & unitQuat1 )
{
    SoaQuat<SoaNFloat> start;
    SoaNFloat recipSinAngle, scale0, scale1, cosAngle, angle;
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

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> squad( SoaNFloat t, const SoaQuat<SoaNFloat> & unitQuat0, const SoaQuat<SoaNFloat> & unitQuat1, const SoaQuat<SoaNFloat> & unitQuat2, const SoaQuat<SoaNFloat> & unitQuat3 )
{
    SoaQuat<SoaNFloat> tmp0, tmp1;
    tmp0 = slerp( t, unitQuat0, unitQuat3 );
    tmp1 = slerp( t, unitQuat1, unitQuat2 );
    return slerp( ( ( 2.0f * t ) * ( 1.0f - t ) ), tmp0, tmp1 );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void loadXYZW( SoaQuat<SoaNFloat> & quat, const SoaNFloat * fptr )
{
    quat = SoaQuat<SoaNFloat>( fptr[0], fptr[1], fptr[2], fptr[3] );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void storeXYZW( const SoaQuat<SoaNFloat> & quat, SoaNFloat * fptr )
{
    fptr[0] = quat.getX();
    fptr[1] = quat.getY();
    fptr[2] = quat.getZ();
    fptr[3] = quat.getW();
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaQuat<SoaNFloat> & SoaQuat<SoaNFloat>::operator =( const SoaQuat<SoaNFloat> & quat )
{
    x = quat.x;
    y = quat.y;
    z = quat.z;
    w = quat.w;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaQuat<SoaNFloat> & SoaQuat<SoaNFloat>::setXYZ( const SoaVector3<SoaNFloat> & vec )
{
    x = vec.getX();
    y = vec.getY();
    z = vec.getZ();
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> SoaQuat<SoaNFloat>::getXYZ( ) const
{
    return SoaVector3<SoaNFloat>( x, y, z );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaQuat<SoaNFloat> & SoaQuat<SoaNFloat>::setX( SoaNFloat _x )
{
    x = _x;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaQuat<SoaNFloat>::getX( ) const
{
    return x;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaQuat<SoaNFloat> & SoaQuat<SoaNFloat>::setY( SoaNFloat _y )
{
    y = _y;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaQuat<SoaNFloat>::getY( ) const
{
    return y;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaQuat<SoaNFloat> & SoaQuat<SoaNFloat>::setZ( SoaNFloat _z )
{
    z = _z;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaQuat<SoaNFloat>::getZ( ) const
{
    return z;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaQuat<SoaNFloat> & SoaQuat<SoaNFloat>::setW( SoaNFloat _w )
{
    w = _w;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaQuat<SoaNFloat>::getW( ) const
{
    return w;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaQuat<SoaNFloat> & SoaQuat<SoaNFloat>::setElem( int idx, SoaNFloat value )
{
    *(&x + idx) = value;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat SoaQuat<SoaNFloat>::getElem( int idx ) const
{
    return *(&x + idx);
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> SoaQuat<SoaNFloat>::operator +( const SoaQuat<SoaNFloat> & quat ) const
{
    return SoaQuat<SoaNFloat>(
        ( x + quat.x ),
        ( y + quat.y ),
        ( z + quat.z ),
        ( w + quat.w )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> SoaQuat<SoaNFloat>::operator -( const SoaQuat<SoaNFloat> & quat ) const
{
    return SoaQuat<SoaNFloat>(
        ( x - quat.x ),
        ( y - quat.y ),
        ( z - quat.z ),
        ( w - quat.w )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> SoaQuat<SoaNFloat>::operator *( SoaNFloat scalar ) const
{
    return SoaQuat<SoaNFloat>(
        ( x * scalar ),
        ( y * scalar ),
        ( z * scalar ),
        ( w * scalar )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaQuat<SoaNFloat> & SoaQuat<SoaNFloat>::operator +=( const SoaQuat<SoaNFloat> & quat )
{
    *this = *this + quat;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaQuat<SoaNFloat> & SoaQuat<SoaNFloat>::operator -=( const SoaQuat<SoaNFloat> & quat )
{
    *this = *this - quat;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaQuat<SoaNFloat> & SoaQuat<SoaNFloat>::operator *=( SoaNFloat scalar )
{
    *this = *this * scalar;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> SoaQuat<SoaNFloat>::operator /( SoaNFloat scalar ) const
{
    return SoaQuat<SoaNFloat>(
        ( x / scalar ),
        ( y / scalar ),
        ( z / scalar ),
        ( w / scalar )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaQuat<SoaNFloat> & SoaQuat<SoaNFloat>::operator /=( SoaNFloat scalar )
{
    *this = *this / scalar;
    return *this;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> SoaQuat<SoaNFloat>::operator -( ) const
{
    return SoaQuat<SoaNFloat>(
        -x,
        -y,
        -z,
        -w
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> operator *( SoaNFloat scalar, const SoaQuat<SoaNFloat> & quat )
{
    return quat * scalar;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat dot( const SoaQuat<SoaNFloat> & quat0, const SoaQuat<SoaNFloat> & quat1 )
{
    SoaNFloat result;
    result = ( quat0.getX() * quat1.getX() );
    result = ( result + ( quat0.getY() * quat1.getY() ) );
    result = ( result + ( quat0.getZ() * quat1.getZ() ) );
    result = ( result + ( quat0.getW() * quat1.getW() ) );
    return result;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat norm( const SoaQuat<SoaNFloat> & quat )
{
    SoaNFloat result;
    result = ( quat.getX() * quat.getX() );
    result = ( result + ( quat.getY() * quat.getY() ) );
    result = ( result + ( quat.getZ() * quat.getZ() ) );
    result = ( result + ( quat.getW() * quat.getW() ) );
    return result;
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat length( const SoaQuat<SoaNFloat> & quat )
{
    return ::sqrtf( norm( quat ) );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> normalize( const SoaQuat<SoaNFloat> & quat )
{
    SoaNFloat lenSqr, lenInv;
    lenSqr = norm( quat );
    lenInv = ( 1.0f / sqrtf( lenSqr ) );
    return SoaQuat<SoaNFloat>(
        ( quat.getX() * lenInv ),
        ( quat.getY() * lenInv ),
        ( quat.getZ() * lenInv ),
        ( quat.getW() * lenInv )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> SoaQuat<SoaNFloat>::rotation( const SoaVector3<SoaNFloat> & unitVec0, const SoaVector3<SoaNFloat> & unitVec1 )
{
    SoaNFloat cosHalfAngleX2, recipCosHalfAngleX2;
    cosHalfAngleX2 = sqrtf( ( 2.0f * ( 1.0f + dot( unitVec0, unitVec1 ) ) ) );
    recipCosHalfAngleX2 = ( 1.0f / cosHalfAngleX2 );
    return SoaQuat<SoaNFloat>( ( cross( unitVec0, unitVec1 ) * recipCosHalfAngleX2 ), ( cosHalfAngleX2 * 0.5f ) );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> SoaQuat<SoaNFloat>::rotation( SoaNFloat radians, const SoaVector3<SoaNFloat> & unitVec )
{
    SoaNFloat s, c, angle;
    angle = ( radians * 0.5f );
    sincosf(angle, &s, &c);
    return SoaQuat<SoaNFloat>( ( unitVec * s ), c );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> SoaQuat<SoaNFloat>::rotationX( SoaNFloat radians )
{
    SoaNFloat s, c, angle;
    angle = ( radians * 0.5f );
    sincosf(angle, &s, &c);
    return SoaQuat<SoaNFloat>( s, 0.0f, 0.0f, c );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> SoaQuat<SoaNFloat>::rotationY( SoaNFloat radians )
{
    SoaNFloat s, c, angle;
    angle = ( radians * 0.5f );
    sincosf(angle, &s, &c);
    return SoaQuat<SoaNFloat>( 0.0f, s, 0.0f, c );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> SoaQuat<SoaNFloat>::rotationZ( SoaNFloat radians )
{
    SoaNFloat s, c, angle;
    angle = ( radians * 0.5f );
    sincosf(angle, &s, &c);
    return SoaQuat<SoaNFloat>( 0.0f, 0.0f, s, c );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> mul(const SoaQuat<SoaNFloat> & quat0, const SoaQuat<SoaNFloat> & quat1 )
{
    return SoaQuat<SoaNFloat>(
        ((((quat0.w * quat1.x) + (quat0.x * quat1.w)) + (quat0.y * quat1.z)) - (quat0.z * quat1.y)),
        ((((quat0.w * quat1.y) + (quat0.y * quat1.w)) + (quat0.z * quat1.x)) - (quat0.x * quat1.z)),
        ((((quat0.w * quat1.z) + (quat0.z * quat1.w)) + (quat0.x * quat1.y)) - (quat0.y * quat1.x)),
        ((((quat0.w * quat1.w) - (quat0.x * quat1.x)) - (quat0.y * quat1.y)) - (quat0.z * quat1.z ) )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> rotate( const SoaQuat<SoaNFloat> & quat, const SoaVector3<SoaNFloat> & vec )
{
    SoaNFloat tmpX, tmpY, tmpZ, tmpW;
    tmpX = ( ( ( quat.getW() * vec.getX() ) + ( quat.getY() * vec.getZ() ) ) - ( quat.getZ() * vec.getY() ) );
    tmpY = ( ( ( quat.getW() * vec.getY() ) + ( quat.getZ() * vec.getX() ) ) - ( quat.getX() * vec.getZ() ) );
    tmpZ = ( ( ( quat.getW() * vec.getZ() ) + ( quat.getX() * vec.getY() ) ) - ( quat.getY() * vec.getX() ) );
    tmpW = ( ( ( quat.getX() * vec.getX() ) + ( quat.getY() * vec.getY() ) ) + ( quat.getZ() * vec.getZ() ) );
    return SoaVector3<SoaNFloat>(
        ( ( ( ( tmpW * quat.getX() ) + ( tmpX * quat.getW() ) ) - ( tmpY * quat.getZ() ) ) + ( tmpZ * quat.getY() ) ),
        ( ( ( ( tmpW * quat.getY() ) + ( tmpY * quat.getW() ) ) - ( tmpZ * quat.getX() ) ) + ( tmpX * quat.getZ() ) ),
        ( ( ( ( tmpW * quat.getZ() ) + ( tmpZ * quat.getW() ) ) - ( tmpX * quat.getY() ) ) + ( tmpY * quat.getX() ) )
    );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> conj( const SoaQuat<SoaNFloat> & quat )
{
    return SoaQuat<SoaNFloat>( -quat.getX(), -quat.getY(), -quat.getZ(), quat.getW() );
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> select( const SoaQuat<SoaNFloat> & quat0, const SoaQuat<SoaNFloat> & quat1, typename SoaNFloat::SoaBool select1 )
{
    return SoaQuat<SoaNFloat>(
        select(quat0.getX(), quat1.getX(), ( select1 )),
        select(quat0.getY(), quat1.getY(), ( select1 )),
        select(quat0.getZ(), quat1.getZ(), ( select1 )),
        select(quat0.getW(), quat1.getW(), ( select1 ))
    );
}

#ifdef _VECTORMATH_DEBUG

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaQuat<SoaNFloat> & quat )
{
    print(quat.getX(), "x");
    print(quat.getY(), "y");
    print(quat.getZ(), "z");
    print(quat.getW(), "w");
}

template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaQuat<SoaNFloat> & quat, const char * name )
{
    printf("%s:\n", name);
    print(quat.getX(), "x");
    print(quat.getY(), "y");
    print(quat.getZ(), "z");
    print(quat.getW(), "w");
}

#endif

} // namespace FmVectormath

#endif
