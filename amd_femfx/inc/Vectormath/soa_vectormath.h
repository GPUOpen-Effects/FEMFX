/*
   Copyright (C) 2006, 2007 Sony Computer Entertainment Inc.
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

#ifndef _SOA_VECTORMATH_H
#define _SOA_VECTORMATH_H

#include <math.h>

#ifdef _VECTORMATH_DEBUG
#include <stdio.h>
#endif

#if (defined (_WIN32) && (_MSC_VER) && _MSC_VER >= 1400)
#define SOA_VECTORMATH_FORCE_INLINE __forceinline
#else
#define SOA_VECTORMATH_FORCE_INLINE inline
#endif//_WIN32

namespace FmVectormath {

//-----------------------------------------------------------------------------
// Forward Declarations
//

template< class SoaNFloat > class SoaVector3;
template< class SoaNFloat > class SoaVector4;
template< class SoaNFloat > class SoaPoint3;
template< class SoaNFloat > class SoaQuat;
template< class SoaNFloat > class SoaMatrix3;
template< class SoaNFloat > class SoaMatrix4;
template< class SoaNFloat > class SoaTransform3;

// A 3-D vector in structure-of-arrays format
//
template< class SoaNFloat >
class SoaVector3
{
public:
    SoaNFloat x;
    SoaNFloat y;
    SoaNFloat z;

    // Default constructor; does no initialization
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector3( ) { };

    // Copy a 3-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector3( const SoaVector3 & vec );

    // Construct a 3-D vector from x, y, and z elements
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector3( SoaNFloat x, SoaNFloat y, SoaNFloat z );

    // Copy elements from a 3-D point into a 3-D vector
    // 
    explicit SOA_VECTORMATH_FORCE_INLINE SoaVector3( const SoaPoint3<SoaNFloat> & pnt );

    // Set all elements of a 3-D vector to the same scalar value
    // 
    explicit SOA_VECTORMATH_FORCE_INLINE SoaVector3( SoaNFloat scalar );

    // Assign one 3-D vector to another
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector3 & operator =( const SoaVector3 & vec );

    // Set the x element of a 3-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector3 & setX( SoaNFloat x );

    // Set the y element of a 3-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector3 & setY( SoaNFloat y );

    // Set the z element of a 3-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector3 & setZ( SoaNFloat z );

    // Get the x element of a 3-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getX( ) const;

    // Get the y element of a 3-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getY( ) const;

    // Get the z element of a 3-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getZ( ) const;

    // Set an x, y, or z element of a 3-D vector by index
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector3 & setElem( int idx, SoaNFloat value );

    // Get an x, y, or z element of a 3-D vector by index
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getElem( int idx ) const;

    // Add two 3-D vectors
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3 operator +( const SoaVector3 & vec ) const;

    // Subtract a 3-D vector from another 3-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3 operator -( const SoaVector3 & vec ) const;

    // Add a 3-D vector to a 3-D point
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> operator +( const SoaPoint3<SoaNFloat> & pnt ) const;

    // Multiply a 3-D vector by a scalar
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3 operator *( SoaNFloat scalar ) const;

    // Divide a 3-D vector by a scalar
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3 operator /( SoaNFloat scalar ) const;

    // Perform compound assignment and addition with a 3-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector3 & operator +=( const SoaVector3 & vec );

    // Perform compound assignment and subtraction by a 3-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector3 & operator -=( const SoaVector3 & vec );

    // Perform compound assignment and multiplication by a scalar
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector3 & operator *=( SoaNFloat scalar );

    // Perform compound assignment and division by a scalar
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector3 & operator /=( SoaNFloat scalar );

    // Negate all elements of a 3-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3 operator -( ) const;

    // Construct x axis
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaVector3 xAxis( );

    // Construct y axis
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaVector3 yAxis( );

    // Construct z axis
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaVector3 zAxis( );

};

// Multiply a 3-D vector by a scalar
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> operator *( SoaNFloat scalar, const SoaVector3<SoaNFloat> & vec );

// Multiply two 3-D vectors per element
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> operator *( const SoaVector3<SoaNFloat> & vec0, const SoaVector3<SoaNFloat> & vec1 );

// Divide two 3-D vectors per element
// NOTE: 
// Floating-point behavior matches standard library function divf4.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> operator /( const SoaVector3<SoaNFloat> & vec0, const SoaVector3<SoaNFloat> & vec1 );

// Compute the reciprocal of a 3-D vector per element
// NOTE: 
// Floating-point behavior matches standard library function _mm_rcp_ps.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> inv( const SoaVector3<SoaNFloat> & vec );

// Compute the square root of a 3-D vector per element
// NOTE: 
// Floating-point behavior matches standard library function _mm_sqrt_ps.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> sqrt( const SoaVector3<SoaNFloat> & vec );

// Compute the reciprocal square root of a 3-D vector per element
// NOTE: 
// Floating-point behavior matches standard library function _mm_rsqrt_ps.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> rsqrt( const SoaVector3<SoaNFloat> & vec );

// Compute the absolute value of a 3-D vector per element
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> abs( const SoaVector3<SoaNFloat> & vec );

// Copy sign from one 3-D vector to another, per element
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> copySign( const SoaVector3<SoaNFloat> & vec0, const SoaVector3<SoaNFloat> & vec1 );

// Maximum of two 3-D vectors per element
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> max( const SoaVector3<SoaNFloat> & vec0, const SoaVector3<SoaNFloat> & vec1 );

// Minimum of two 3-D vectors per element
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> min( const SoaVector3<SoaNFloat> & vec0, const SoaVector3<SoaNFloat> & vec1 );

// Maximum element of a 3-D vector
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat maxElem( const SoaVector3<SoaNFloat> & vec );

// Minimum element of a 3-D vector
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat minElem( const SoaVector3<SoaNFloat> & vec );

// Compute the sum of all elements of a 3-D vector
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat sum( const SoaVector3<SoaNFloat> & vec );

// Compute the dot product of two 3-D vectors
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat dot( const SoaVector3<SoaNFloat> & vec0, const SoaVector3<SoaNFloat> & vec1 );

// Compute the square of the length of a 3-D vector
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat lengthSqr( const SoaVector3<SoaNFloat> & vec );

// Compute the length of a 3-D vector
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat length( const SoaVector3<SoaNFloat> & vec );

// Normalize a 3-D vector
// NOTE: 
// The result is unpredictable when all elements of vec are at or near zero.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> normalize( const SoaVector3<SoaNFloat> & vec );

// Compute cross product of two 3-D vectors
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> cross( const SoaVector3<SoaNFloat> & vec0, const SoaVector3<SoaNFloat> & vec1 );

// Outer product of two 3-D vectors
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> outer( const SoaVector3<SoaNFloat> & vec0, const SoaVector3<SoaNFloat> & vec1 );

// Pre-multiply a row vector by a 3x3 matrix
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> mul( const SoaVector3<SoaNFloat> & vec, const SoaMatrix3<SoaNFloat> & mat );

// Cross-product matrix of a 3-D vector
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> crossMatrix( const SoaVector3<SoaNFloat> & vec );

// Create cross-product matrix and multiply
// NOTE: 
// Faster than separately creating a cross-product matrix and multiplying.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> crossMatrixMul( const SoaVector3<SoaNFloat> & vec, const SoaMatrix3<SoaNFloat> & mat );

// Linear interpolation between two 3-D vectors
// NOTE: 
// Does not clamp t between 0 and 1.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> lerp( SoaNFloat t, const SoaVector3<SoaNFloat> & vec0, const SoaVector3<SoaNFloat> & vec1 );

// Spherical linear interpolation between two 3-D vectors
// NOTE: 
// The result is unpredictable if the vectors point in opposite directions.
// Does not clamp t between 0 and 1.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> slerp( SoaNFloat t, const SoaVector3<SoaNFloat> & unitVec0, const SoaVector3<SoaNFloat> & unitVec1 );

// Conditionally select between two 3-D vectors
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> select( const SoaVector3<SoaNFloat> & vec0, const SoaVector3<SoaNFloat> & vec1, typename SoaNFloat::SoaBool select1 );

#ifdef _VECTORMATH_DEBUG

// Print a 3-D vector
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaVector3<SoaNFloat> & vec );

// Print a 3-D vector and an associated string identifier
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaVector3<SoaNFloat> & vec, const char * name );

#endif

// A 4-D vector in structure-of-arrays format
//
template< class SoaNFloat > 
class SoaVector4
{
public:
    SoaNFloat x;
    SoaNFloat y;
    SoaNFloat z;
    SoaNFloat w;

    // Default constructor; does no initialization
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector4( ) { };

    // Copy a 4-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector4( const SoaVector4 & vec );

    // Construct a 4-D vector from x, y, z, and w elements
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector4( SoaNFloat x, SoaNFloat y, SoaNFloat z, SoaNFloat w );

    // Construct a 4-D vector from a 3-D vector and a scalar
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector4( const SoaVector3<SoaNFloat> & xyz, SoaNFloat w );

    // Copy x, y, and z from a 3-D vector into a 4-D vector, and set w to 0
    // 
    explicit SOA_VECTORMATH_FORCE_INLINE SoaVector4( const SoaVector3<SoaNFloat> & vec );

    // Copy x, y, and z from a 3-D point into a 4-D vector, and set w to 1
    // 
    explicit SOA_VECTORMATH_FORCE_INLINE SoaVector4( const SoaPoint3<SoaNFloat> & pnt );

    // Copy elements from a quaternion into a 4-D vector
    // 
    explicit SOA_VECTORMATH_FORCE_INLINE SoaVector4( const SoaQuat<SoaNFloat> & quat );

    // Set all elements of a 4-D vector to the same scalar value
    // 
    explicit SOA_VECTORMATH_FORCE_INLINE SoaVector4( SoaNFloat scalar );

    // Assign one 4-D vector to another
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector4 & operator =( const SoaVector4 & vec );

    // Set the x, y, and z elements of a 4-D vector
    // NOTE: 
    // This function does not change the w element.
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector4 & setXYZ( const SoaVector3<SoaNFloat> & vec );

    // Get the x, y, and z elements of a 4-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> getXYZ( ) const;

    // Set the x element of a 4-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector4 & setX( SoaNFloat x );

    // Set the y element of a 4-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector4 & setY( SoaNFloat y );

    // Set the z element of a 4-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector4 & setZ( SoaNFloat z );

    // Set the w element of a 4-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector4 & setW( SoaNFloat w );

    // Get the x element of a 4-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getX( ) const;

    // Get the y element of a 4-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getY( ) const;

    // Get the z element of a 4-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getZ( ) const;

    // Get the w element of a 4-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getW( ) const;

    // Set an x, y, z, or w element of a 4-D vector by index
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector4 & setElem( int idx, SoaNFloat value );

    // Get an x, y, z, or w element of a 4-D vector by index
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getElem( int idx ) const;

    // Add two 4-D vectors
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector4 operator +( const SoaVector4 & vec ) const;

    // Subtract a 4-D vector from another 4-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector4 operator -( const SoaVector4 & vec ) const;

    // Multiply a 4-D vector by a scalar
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector4 operator *( SoaNFloat scalar ) const;

    // Divide a 4-D vector by a scalar
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector4 operator /( SoaNFloat scalar ) const;

    // Perform compound assignment and addition with a 4-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector4 & operator +=( const SoaVector4 & vec );

    // Perform compound assignment and subtraction by a 4-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector4 & operator -=( const SoaVector4 & vec );

    // Perform compound assignment and multiplication by a scalar
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector4 & operator *=( SoaNFloat scalar );

    // Perform compound assignment and division by a scalar
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaVector4 & operator /=( SoaNFloat scalar );

    // Negate all elements of a 4-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector4 operator -( ) const;

    // Construct x axis
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaVector4 xAxis( );

    // Construct y axis
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaVector4 yAxis( );

    // Construct z axis
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaVector4 zAxis( );

    // Construct w axis
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaVector4 wAxis( );

};

// Multiply a 4-D vector by a scalar
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> operator *( SoaNFloat scalar, const SoaVector4<SoaNFloat> & vec );

// Multiply two 4-D vectors per element
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> operator *( const SoaVector4<SoaNFloat> & vec0, const SoaVector4<SoaNFloat> & vec1 );

// Divide two 4-D vectors per element
// NOTE: 
// Floating-point behavior matches standard library function divf4.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> operator /( const SoaVector4<SoaNFloat> & vec0, const SoaVector4<SoaNFloat> & vec1 );

// Compute the reciprocal of a 4-D vector per element
// NOTE: 
// Floating-point behavior matches standard library function _mm_rcp_ps.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> inv( const SoaVector4<SoaNFloat> & vec );

// Compute the square root of a 4-D vector per element
// NOTE: 
// Floating-point behavior matches standard library function _mm_sqrt_ps.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> sqrt( const SoaVector4<SoaNFloat> & vec );

// Compute the reciprocal square root of a 4-D vector per element
// NOTE: 
// Floating-point behavior matches standard library function _mm_rsqrt_ps.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> rsqrt( const SoaVector4<SoaNFloat> & vec );

// Compute the absolute value of a 4-D vector per element
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> abs( const SoaVector4<SoaNFloat> & vec );

// Copy sign from one 4-D vector to another, per element
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> copySign( const SoaVector4<SoaNFloat> & vec0, const SoaVector4<SoaNFloat> & vec1 );

// Maximum of two 4-D vectors per element
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> max( const SoaVector4<SoaNFloat> & vec0, const SoaVector4<SoaNFloat> & vec1 );

// Minimum of two 4-D vectors per element
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> min( const SoaVector4<SoaNFloat> & vec0, const SoaVector4<SoaNFloat> & vec1 );

// Maximum element of a 4-D vector
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat maxElem( const SoaVector4<SoaNFloat> & vec );

// Minimum element of a 4-D vector
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat minElem( const SoaVector4<SoaNFloat> & vec );

// Compute the sum of all elements of a 4-D vector
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat sum( const SoaVector4<SoaNFloat> & vec );

// Compute the dot product of two 4-D vectors
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat dot( const SoaVector4<SoaNFloat> & vec0, const SoaVector4<SoaNFloat> & vec1 );

// Compute the square of the length of a 4-D vector
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat lengthSqr( const SoaVector4<SoaNFloat> & vec );

// Compute the length of a 4-D vector
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat length( const SoaVector4<SoaNFloat> & vec );

// Normalize a 4-D vector
// NOTE: 
// The result is unpredictable when all elements of vec are at or near zero.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> normalize( const SoaVector4<SoaNFloat> & vec );

// Outer product of two 4-D vectors
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> outer( const SoaVector4<SoaNFloat> & vec0, const SoaVector4<SoaNFloat> & vec1 );

// Linear interpolation between two 4-D vectors
// NOTE: 
// Does not clamp t between 0 and 1.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> lerp( SoaNFloat t, const SoaVector4<SoaNFloat> & vec0, const SoaVector4<SoaNFloat> & vec1 );

// Spherical linear interpolation between two 4-D vectors
// NOTE: 
// The result is unpredictable if the vectors point in opposite directions.
// Does not clamp t between 0 and 1.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> slerp( SoaNFloat t, const SoaVector4<SoaNFloat> & unitVec0, const SoaVector4<SoaNFloat> & unitVec1 );

// Conditionally select between two 4-D vectors
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> select( const SoaVector4<SoaNFloat> & vec0, const SoaVector4<SoaNFloat> & vec1, typename SoaNFloat::SoaBool select1 );

#ifdef _VECTORMATH_DEBUG

// Print a 4-D vector
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaVector4<SoaNFloat> & vec );

// Print a 4-D vector and an associated string identifier
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaVector4<SoaNFloat> & vec, const char * name );

#endif

// A 3-D point in structure-of-arrays format
//
template< class SoaNFloat > 
class SoaPoint3
{
public:
    SoaNFloat x;
    SoaNFloat y;
    SoaNFloat z;

    // Default constructor; does no initialization
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaPoint3( ) { };

    // Copy a 3-D point
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaPoint3( const SoaPoint3 & pnt );

    // Construct a 3-D point from x, y, and z elements
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaPoint3( SoaNFloat x, SoaNFloat y, SoaNFloat z );

    // Copy elements from a 3-D vector into a 3-D point
    // 
    explicit SOA_VECTORMATH_FORCE_INLINE SoaPoint3( const SoaVector3<SoaNFloat> & vec );

    // Set all elements of a 3-D point to the same scalar value
    // 
    explicit SOA_VECTORMATH_FORCE_INLINE SoaPoint3( SoaNFloat scalar );

    // Assign one 3-D point to another
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaPoint3 & operator =( const SoaPoint3 & pnt );

    // Set the x element of a 3-D point
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaPoint3 & setX( SoaNFloat x );

    // Set the y element of a 3-D point
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaPoint3 & setY( SoaNFloat y );

    // Set the z element of a 3-D point
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaPoint3 & setZ( SoaNFloat z );

    // Get the x element of a 3-D point
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getX( ) const;

    // Get the y element of a 3-D point
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getY( ) const;

    // Get the z element of a 3-D point
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getZ( ) const;

    // Set an x, y, or z element of a 3-D point by index
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaPoint3 & setElem( int idx, SoaNFloat value );

    // Get an x, y, or z element of a 3-D point by index
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getElem( int idx ) const;

    // Subtract a 3-D point from another 3-D point
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> operator -( const SoaPoint3 & pnt ) const;

    // Add a 3-D point to a 3-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaPoint3 operator +( const SoaVector3<SoaNFloat> & vec ) const;

    // Subtract a 3-D vector from a 3-D point
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaPoint3 operator -( const SoaVector3<SoaNFloat> & vec ) const;

    // Perform compound assignment and addition with a 3-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaPoint3 & operator +=( const SoaVector3<SoaNFloat> & vec );

    // Perform compound assignment and subtraction by a 3-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaPoint3 & operator -=( const SoaVector3<SoaNFloat> & vec );

};

// Multiply two 3-D points per element
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> operator *( const SoaPoint3<SoaNFloat> & pnt0, const SoaPoint3<SoaNFloat> & pnt1 );

// Divide two 3-D points per element
// NOTE: 
// Floating-point behavior matches standard library function divf4.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> operator /( const SoaPoint3<SoaNFloat> & pnt0, const SoaPoint3<SoaNFloat> & pnt1 );

// Compute the reciprocal of a 3-D point per element
// NOTE: 
// Floating-point behavior matches standard library function _mm_rcp_ps.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> inv( const SoaPoint3<SoaNFloat> & pnt );

// Compute the square root of a 3-D point per element
// NOTE: 
// Floating-point behavior matches standard library function _mm_sqrt_ps.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> sqrt( const SoaPoint3<SoaNFloat> & pnt );

// Compute the reciprocal square root of a 3-D point per element
// NOTE: 
// Floating-point behavior matches standard library function _mm_rsqrt_ps.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> rsqrt( const SoaPoint3<SoaNFloat> & pnt );

// Compute the absolute value of a 3-D point per element
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> abs( const SoaPoint3<SoaNFloat> & pnt );

// Copy sign from one 3-D point to another, per element
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> copySign( const SoaPoint3<SoaNFloat> & pnt0, const SoaPoint3<SoaNFloat> & pnt1 );

// Maximum of two 3-D points per element
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> max( const SoaPoint3<SoaNFloat> & pnt0, const SoaPoint3<SoaNFloat> & pnt1 );

// Minimum of two 3-D points per element
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> min( const SoaPoint3<SoaNFloat> & pnt0, const SoaPoint3<SoaNFloat> & pnt1 );

// Maximum element of a 3-D point
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat maxElem( const SoaPoint3<SoaNFloat> & pnt );

// Minimum element of a 3-D point
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat minElem( const SoaPoint3<SoaNFloat> & pnt );

// Compute the sum of all elements of a 3-D point
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat sum( const SoaPoint3<SoaNFloat> & pnt );

// Apply uniform scale to a 3-D point
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> scale( const SoaPoint3<SoaNFloat> & pnt, SoaNFloat scaleVal );

// Apply non-uniform scale to a 3-D point
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> scale( const SoaPoint3<SoaNFloat> & pnt, const SoaVector3<SoaNFloat> & scaleVec );

// Scalar projection of a 3-D point on a unit-length 3-D vector
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat projection( const SoaPoint3<SoaNFloat> & pnt, const SoaVector3<SoaNFloat> & unitVec );

// Compute the square of the distance of a 3-D point from the coordinate-system origin
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat distSqrFromOrigin( const SoaPoint3<SoaNFloat> & pnt );

// Compute the distance of a 3-D point from the coordinate-system origin
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat distFromOrigin( const SoaPoint3<SoaNFloat> & pnt );

// Compute the square of the distance between two 3-D points
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat distSqr( const SoaPoint3<SoaNFloat> & pnt0, const SoaPoint3<SoaNFloat> & pnt1 );

// Compute the distance between two 3-D points
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat dist( const SoaPoint3<SoaNFloat> & pnt0, const SoaPoint3<SoaNFloat> & pnt1 );

// Linear interpolation between two 3-D points
// NOTE: 
// Does not clamp t between 0 and 1.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> lerp( SoaNFloat t, const SoaPoint3<SoaNFloat> & pnt0, const SoaPoint3<SoaNFloat> & pnt1 );

// Conditionally select between two 3-D points
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> select( const SoaPoint3<SoaNFloat> & pnt0, const SoaPoint3<SoaNFloat> & pnt1, typename SoaNFloat::SoaBool select1 );

#ifdef _VECTORMATH_DEBUG

// Print a 3-D point
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaPoint3<SoaNFloat> & pnt );

// Print a 3-D point and an associated string identifier
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaPoint3<SoaNFloat> & pnt, const char * name );

#endif

// A quaternion in structure-of-arrays format
//
template< class SoaNFloat > 
class SoaQuat
{
public:
    SoaNFloat x;
    SoaNFloat y;
    SoaNFloat z;
    SoaNFloat w;

    // Default constructor; does no initialization
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaQuat( ) { };

    // Copy a quaternion
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaQuat( const SoaQuat & quat );

    // Construct a quaternion from x, y, z, and w elements
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaQuat( SoaNFloat x, SoaNFloat y, SoaNFloat z, SoaNFloat w );

    // Construct a quaternion from a 3-D vector and a scalar
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaQuat( const SoaVector3<SoaNFloat> & xyz, SoaNFloat w );

    // Copy elements from a 4-D vector into a quaternion
    // 
    explicit SOA_VECTORMATH_FORCE_INLINE SoaQuat( const SoaVector4<SoaNFloat> & vec );

    // Convert a rotation matrix to a unit-length quaternion
    // 
    explicit SOA_VECTORMATH_FORCE_INLINE SoaQuat( const SoaMatrix3<SoaNFloat> & rotMat );

    // Set all elements of a quaternion to the same scalar value
    // 
    explicit SOA_VECTORMATH_FORCE_INLINE SoaQuat( SoaNFloat scalar );

    // Assign one quaternion to another
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaQuat & operator =( const SoaQuat & quat );

    // Set the x, y, and z elements of a quaternion
    // NOTE: 
    // This function does not change the w element.
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaQuat & setXYZ( const SoaVector3<SoaNFloat> & vec );

    // Get the x, y, and z elements of a quaternion
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> getXYZ( ) const;

    // Set the x element of a quaternion
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaQuat & setX( SoaNFloat x );

    // Set the y element of a quaternion
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaQuat & setY( SoaNFloat y );

    // Set the z element of a quaternion
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaQuat & setZ( SoaNFloat z );

    // Set the w element of a quaternion
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaQuat & setW( SoaNFloat w );

    // Get the x element of a quaternion
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getX( ) const;

    // Get the y element of a quaternion
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getY( ) const;

    // Get the z element of a quaternion
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getZ( ) const;

    // Get the w element of a quaternion
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getW( ) const;

    // Set an x, y, z, or w element of a quaternion by index
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaQuat & setElem( int idx, SoaNFloat value );

    // Get an x, y, z, or w element of a quaternion by index
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getElem( int idx ) const;

    // Add two quaternions
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaQuat operator +( const SoaQuat & quat ) const;

    // Subtract a quaternion from another quaternion
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaQuat operator -( const SoaQuat & quat ) const;

    // Multiply a quaternion by a scalar
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaQuat operator *( SoaNFloat scalar ) const;

    // Divide a quaternion by a scalar
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaQuat operator /( SoaNFloat scalar ) const;

    // Perform compound assignment and addition with a quaternion
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaQuat & operator +=( const SoaQuat & quat );

    // Perform compound assignment and subtraction by a quaternion
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaQuat & operator -=( const SoaQuat & quat );

    // Perform compound assignment and multiplication by a scalar
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaQuat & operator *=( SoaNFloat scalar );

    // Perform compound assignment and division by a scalar
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaQuat & operator /=( SoaNFloat scalar );

    // Negate all elements of a quaternion
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaQuat operator -( ) const;

    // Construct an identity quaternion
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaQuat identity( );

    // Construct a quaternion to rotate between two unit-length 3-D vectors
    // NOTE: 
    // The result is unpredictable if unitVec0 and unitVec1 point in opposite directions.
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaQuat rotation( const SoaVector3<SoaNFloat> & unitVec0, const SoaVector3<SoaNFloat> & unitVec1 );

    // Construct a quaternion to rotate around a unit-length 3-D vector
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaQuat rotation( SoaNFloat radians, const SoaVector3<SoaNFloat> & unitVec );

    // Construct a quaternion to rotate around the x axis
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaQuat rotationX( SoaNFloat radians );

    // Construct a quaternion to rotate around the y axis
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaQuat rotationY( SoaNFloat radians );

    // Construct a quaternion to rotate around the z axis
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaQuat rotationZ( SoaNFloat radians );

};

// Multiply two quaternions
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> mul(const SoaQuat<SoaNFloat> & q0, const SoaQuat<SoaNFloat> & q1);

// Multiply a quaternion by a scalar
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> operator *( SoaNFloat scalar, const SoaQuat<SoaNFloat> & quat );

// Compute the conjugate of a quaternion
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> conj( const SoaQuat<SoaNFloat> & quat );

// Use a unit-length quaternion to rotate a 3-D vector
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> rotate( const SoaQuat<SoaNFloat> & unitQuat, const SoaVector3<SoaNFloat> & vec );

// Compute the dot product of two quaternions
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat dot( const SoaQuat<SoaNFloat> & quat0, const SoaQuat<SoaNFloat> & quat1 );

// Compute the norm of a quaternion
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat norm( const SoaQuat<SoaNFloat> & quat );

// Compute the length of a quaternion
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat length( const SoaQuat<SoaNFloat> & quat );

// Normalize a quaternion
// NOTE: 
// The result is unpredictable when all elements of quat are at or near zero.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> normalize( const SoaQuat<SoaNFloat> & quat );

// Linear interpolation between two quaternions
// NOTE: 
// Does not clamp t between 0 and 1.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> lerp( SoaNFloat t, const SoaQuat<SoaNFloat> & quat0, const SoaQuat<SoaNFloat> & quat1 );

// Spherical linear interpolation between two quaternions
// NOTE: 
// Interpolates along the shortest path between orientations.
// Does not clamp t between 0 and 1.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> slerp( SoaNFloat t, const SoaQuat<SoaNFloat> & unitQuat0, const SoaQuat<SoaNFloat> & unitQuat1 );

// Spherical quadrangle interpolation
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> squad( SoaNFloat t, const SoaQuat<SoaNFloat> & unitQuat0, const SoaQuat<SoaNFloat> & unitQuat1, const SoaQuat<SoaNFloat> & unitQuat2, const SoaQuat<SoaNFloat> & unitQuat3 );

// Conditionally select between two quaternions
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaQuat<SoaNFloat> select( const SoaQuat<SoaNFloat> & quat0, const SoaQuat<SoaNFloat> & quat1, typename SoaNFloat::SoaBool select1 );

#ifdef _VECTORMATH_DEBUG

// Print a quaternion
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaQuat<SoaNFloat> & quat );

// Print a quaternion and an associated string identifier
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaQuat<SoaNFloat> & quat, const char * name );

#endif

// A 3x3 matrix in structure-of-arrays format
//
template< class SoaNFloat > 
class SoaMatrix3
{
public:
    SoaVector3<SoaNFloat> col0;
    SoaVector3<SoaNFloat> col1;
    SoaVector3<SoaNFloat> col2;

    // Default constructor; does no initialization
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix3( ) { };

    // Copy a 3x3 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix3( const SoaMatrix3 & mat );

    // Construct a 3x3 matrix containing the specified columns
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix3( const SoaVector3<SoaNFloat> & col0, const SoaVector3<SoaNFloat> & col1, const SoaVector3<SoaNFloat> & col2 );

    // Construct a 3x3 rotation matrix from a unit-length quaternion
    // 
    explicit SOA_VECTORMATH_FORCE_INLINE SoaMatrix3( const SoaQuat<SoaNFloat> & unitSoaQuat );

    // Set all elements of a 3x3 matrix to the same scalar value
    // 
    explicit SOA_VECTORMATH_FORCE_INLINE SoaMatrix3( SoaNFloat scalar );

    // Assign one 3x3 matrix to another
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix3 & operator =( const SoaMatrix3 & mat );

    // Set column 0 of a 3x3 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix3 & setCol0( const SoaVector3<SoaNFloat> & col0 );

    // Set column 1 of a 3x3 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix3 & setCol1( const SoaVector3<SoaNFloat> & col1 );

    // Set column 2 of a 3x3 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix3 & setCol2( const SoaVector3<SoaNFloat> & col2 );

    // Get column 0 of a 3x3 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> getCol0( ) const;

    // Get column 1 of a 3x3 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> getCol1( ) const;

    // Get column 2 of a 3x3 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> getCol2( ) const;

    // Set the column of a 3x3 matrix referred to by the specified index
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix3 & setCol( int col, const SoaVector3<SoaNFloat> & vec );

    // Set the row of a 3x3 matrix referred to by the specified index
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix3 & setRow( int row, const SoaVector3<SoaNFloat> & vec );

    // Get the column of a 3x3 matrix referred to by the specified index
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> getCol( int col ) const;

    // Get the row of a 3x3 matrix referred to by the specified index
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> getRow( int row ) const;

    // Set the element of a 3x3 matrix referred to by column and row indices
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix3 & setElem( int col, int row, SoaNFloat val );

    // Get the element of a 3x3 matrix referred to by column and row indices
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getElem( int col, int row ) const;

    // Add two 3x3 matrices
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3 operator +( const SoaMatrix3 & mat ) const;

    // Subtract a 3x3 matrix from another 3x3 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3 operator -( const SoaMatrix3 & mat ) const;

    // Negate all elements of a 3x3 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3 operator -( ) const;

    // Multiply a 3x3 matrix by a scalar
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3 operator *( SoaNFloat scalar ) const;

    // Perform compound assignment and addition with a 3x3 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix3 & operator +=( const SoaMatrix3 & mat );

    // Perform compound assignment and subtraction by a 3x3 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix3 & operator -=( const SoaMatrix3 & mat );

    // Perform compound assignment and multiplication by a scalar
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix3 & operator *=( SoaNFloat scalar );

    // Construct an identity 3x3 matrix
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3 identity( );

    // Construct a 3x3 matrix to rotate around the x axis
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3 rotationX( SoaNFloat radians );

    // Construct a 3x3 matrix to rotate around the y axis
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3 rotationY( SoaNFloat radians );

    // Construct a 3x3 matrix to rotate around the z axis
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3 rotationZ( SoaNFloat radians );

    // Construct a 3x3 matrix to rotate around the x, y, and z axes
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3 rotationZYX( const SoaVector3<SoaNFloat> & radiansXYZ );

    // Construct a 3x3 matrix to rotate around a unit-length 3-D vector
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3 rotation( SoaNFloat radians, const SoaVector3<SoaNFloat> & unitVec );

    // Construct a rotation matrix from a unit-length quaternion
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3 rotation( const SoaQuat<SoaNFloat> & unitSoaQuat );

    // Construct a 3x3 matrix to perform scaling
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3 scale( const SoaVector3<SoaNFloat> & scaleVec );

};

// Multiply a 3x3 matrix by a 3-D vector
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> mul(const SoaMatrix3<SoaNFloat> & mat, const SoaVector3<SoaNFloat> & vec);

// Multiply two 3x3 matrices
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> mul(const SoaMatrix3<SoaNFloat> & mat0, const SoaMatrix3<SoaNFloat> & mat1);

// Multiply a 3x3 matrix by a scalar
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> operator *( SoaNFloat scalar, const SoaMatrix3<SoaNFloat> & mat );

// Append (post-multiply) a scale transformation to a 3x3 matrix
// NOTE: 
// Faster than creating and multiplying a scale transformation matrix.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> appendScale( const SoaMatrix3<SoaNFloat> & mat, const SoaVector3<SoaNFloat> & scaleVec );

// Prepend (pre-multiply) a scale transformation to a 3x3 matrix
// NOTE: 
// Faster than creating and multiplying a scale transformation matrix.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> prependScale( const SoaVector3<SoaNFloat> & scaleVec, const SoaMatrix3<SoaNFloat> & mat );

// Compute the absolute value of a 3x3 matrix per element
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> abs( const SoaMatrix3<SoaNFloat> & mat );

// Transpose of a 3x3 matrix
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> transpose( const SoaMatrix3<SoaNFloat> & mat );

// Compute the inverse of a 3x3 matrix
// NOTE: 
// Result is unpredictable when the determinant of mat is equal to or near 0.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> inverse( const SoaMatrix3<SoaNFloat> & mat );

// Determinant of a 3x3 matrix
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat determinant( const SoaMatrix3<SoaNFloat> & mat );

// Conditionally select between two 3x3 matrices
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> select( const SoaMatrix3<SoaNFloat> & mat0, const SoaMatrix3<SoaNFloat> & mat1, typename SoaNFloat::SoaBool select1 );

#ifdef _VECTORMATH_DEBUG

// Print a 3x3 matrix
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaMatrix3<SoaNFloat> & mat );

// Print a 3x3 matrix and an associated string identifier
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaMatrix3<SoaNFloat> & mat, const char * name );

#endif

// A 4x4 matrix in structure-of-arrays format
//
template< class SoaNFloat > 
class SoaMatrix4
{
public:
    SoaVector4<SoaNFloat> col0;
    SoaVector4<SoaNFloat> col1;
    SoaVector4<SoaNFloat> col2;
    SoaVector4<SoaNFloat> col3;

    // Default constructor; does no initialization
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix4( ) { };

    // Copy a 4x4 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix4( const SoaMatrix4 & mat );

    // Construct a 4x4 matrix containing the specified columns
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix4( const SoaVector4<SoaNFloat> & col0, const SoaVector4<SoaNFloat> & col1, const SoaVector4<SoaNFloat> & col2, const SoaVector4<SoaNFloat> & col3 );

    // Construct a 4x4 matrix from a 3x4 transformation matrix
    // 
    explicit SOA_VECTORMATH_FORCE_INLINE SoaMatrix4( const SoaTransform3<SoaNFloat> & mat );

    // Construct a 4x4 matrix from a 3x3 matrix and a 3-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix4( const SoaMatrix3<SoaNFloat> & mat, const SoaVector3<SoaNFloat> & translateVec );

    // Construct a 4x4 matrix from a unit-length quaternion and a 3-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix4( const SoaQuat<SoaNFloat> & unitSoaQuat, const SoaVector3<SoaNFloat> & translateVec );

    // Set all elements of a 4x4 matrix to the same scalar value
    // 
    explicit SOA_VECTORMATH_FORCE_INLINE SoaMatrix4( SoaNFloat scalar );

    // Assign one 4x4 matrix to another
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix4 & operator =( const SoaMatrix4 & mat );

    // Set the upper-left 3x3 submatrix
    // NOTE: 
    // This function does not change the bottom row elements.
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix4 & setUpper3x3( const SoaMatrix3<SoaNFloat> & mat3 );

    // Get the upper-left 3x3 submatrix of a 4x4 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> getUpper3x3( ) const;

    // Set translation component
    // NOTE: 
    // This function does not change the bottom row elements.
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix4 & setTranslation( const SoaVector3<SoaNFloat> & translateVec );

    // Get the translation component of a 4x4 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> getTranslation( ) const;

    // Set column 0 of a 4x4 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix4 & setCol0( const SoaVector4<SoaNFloat> & col0 );

    // Set column 1 of a 4x4 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix4 & setCol1( const SoaVector4<SoaNFloat> & col1 );

    // Set column 2 of a 4x4 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix4 & setCol2( const SoaVector4<SoaNFloat> & col2 );

    // Set column 3 of a 4x4 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix4 & setCol3( const SoaVector4<SoaNFloat> & col3 );

    // Get column 0 of a 4x4 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> getCol0( ) const;

    // Get column 1 of a 4x4 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> getCol1( ) const;

    // Get column 2 of a 4x4 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> getCol2( ) const;

    // Get column 3 of a 4x4 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> getCol3( ) const;

    // Set the column of a 4x4 matrix referred to by the specified index
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix4 & setCol( int col, const SoaVector4<SoaNFloat> & vec );

    // Set the row of a 4x4 matrix referred to by the specified index
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix4 & setRow( int row, const SoaVector4<SoaNFloat> & vec );

    // Get the column of a 4x4 matrix referred to by the specified index
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> getCol( int col ) const;

    // Get the row of a 4x4 matrix referred to by the specified index
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> getRow( int row ) const;

    // Set the element of a 4x4 matrix referred to by column and row indices
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix4 & setElem( int col, int row, SoaNFloat val );

    // Get the element of a 4x4 matrix referred to by column and row indices
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getElem( int col, int row ) const;

    // Add two 4x4 matrices
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4 operator +( const SoaMatrix4 & mat ) const;

    // Subtract a 4x4 matrix from another 4x4 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4 operator -( const SoaMatrix4 & mat ) const;

    // Negate all elements of a 4x4 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4 operator -( ) const;

    // Multiply a 4x4 matrix by a scalar
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4 operator *( SoaNFloat scalar ) const;

    // Perform compound assignment and addition with a 4x4 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix4 & operator +=( const SoaMatrix4 & mat );

    // Perform compound assignment and subtraction by a 4x4 matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix4 & operator -=( const SoaMatrix4 & mat );

    // Perform compound assignment and multiplication by a scalar
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaMatrix4 & operator *=( SoaNFloat scalar );

    // Construct an identity 4x4 matrix
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4 identity( );

    // Construct a 4x4 matrix to rotate around the x axis
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4 rotationX( SoaNFloat radians );

    // Construct a 4x4 matrix to rotate around the y axis
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4 rotationY( SoaNFloat radians );

    // Construct a 4x4 matrix to rotate around the z axis
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4 rotationZ( SoaNFloat radians );

    // Construct a 4x4 matrix to rotate around the x, y, and z axes
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4 rotationZYX( const SoaVector3<SoaNFloat> & radiansXYZ );

    // Construct a 4x4 matrix to rotate around a unit-length 3-D vector
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4 rotation( SoaNFloat radians, const SoaVector3<SoaNFloat> & unitVec );

    // Construct a rotation matrix from a unit-length quaternion
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4 rotation( const SoaQuat<SoaNFloat> & unitSoaQuat );

    // Construct a 4x4 matrix to perform scaling
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4 scale( const SoaVector3<SoaNFloat> & scaleVec );

    // Construct a 4x4 matrix to perform translation
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4 translation( const SoaVector3<SoaNFloat> & translateVec );

    // Construct viewing matrix based on eye position, position looked at, and up direction
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4 lookAt( const SoaPoint3<SoaNFloat> & eyePos, const SoaPoint3<SoaNFloat> & lookAtPos, const SoaVector3<SoaNFloat> & upVec );

    // Construct a perspective projection matrix
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4 perspective( SoaNFloat fovyRadians, SoaNFloat aspect, SoaNFloat zNear, SoaNFloat zFar );

    // Construct a perspective projection matrix based on frustum
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4 frustum( SoaNFloat left, SoaNFloat right, SoaNFloat bottom, SoaNFloat top, SoaNFloat zNear, SoaNFloat zFar );

    // Construct an orthographic projection matrix
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4 orthographic( SoaNFloat left, SoaNFloat right, SoaNFloat bottom, SoaNFloat top, SoaNFloat zNear, SoaNFloat zFar );

};

// Multiply a 4x4 matrix by a 4-D vector
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> mul(const SoaMatrix4<SoaNFloat>& mat, const SoaVector4<SoaNFloat> & vec);

// Multiply a 4x4 matrix by a 3-D vector
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> mul(const SoaMatrix4<SoaNFloat>& mat, const SoaVector3<SoaNFloat> & vec);

// Multiply a 4x4 matrix by a 3-D point
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> mul(const SoaMatrix4<SoaNFloat>& mat, const SoaPoint3<SoaNFloat> & pnt);

// Multiply two 4x4 matrices
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> mul(const SoaMatrix4<SoaNFloat>& mat0, const SoaMatrix4<SoaNFloat> & mat1);

// Multiply a 4x4 matrix by a 3x4 transformation matrix
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> mul(const SoaMatrix4<SoaNFloat>& mat, const SoaTransform3<SoaNFloat> & tfrm);

// Multiply a 4x4 matrix by a scalar
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> operator *( SoaNFloat scalar, const SoaMatrix4<SoaNFloat> & mat );

// Append (post-multiply) a scale transformation to a 4x4 matrix
// NOTE: 
// Faster than creating and multiplying a scale transformation matrix.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> appendScale( const SoaMatrix4<SoaNFloat> & mat, const SoaVector3<SoaNFloat> & scaleVec );

// Prepend (pre-multiply) a scale transformation to a 4x4 matrix
// NOTE: 
// Faster than creating and multiplying a scale transformation matrix.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> prependScale( const SoaVector3<SoaNFloat> & scaleVec, const SoaMatrix4<SoaNFloat> & mat );

// Compute the absolute value of a 4x4 matrix per element
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> abs( const SoaMatrix4<SoaNFloat> & mat );

// Transpose of a 4x4 matrix
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> transpose( const SoaMatrix4<SoaNFloat> & mat );

// Compute the inverse of a 4x4 matrix
// NOTE: 
// Result is unpredictable when the determinant of mat is equal to or near 0.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> inverse( const SoaMatrix4<SoaNFloat> & mat );

// Compute the inverse of a 4x4 matrix, which is expected to be an affine matrix
// NOTE: 
// This can be used to achieve better performance than a general inverse when the specified 4x4 matrix meets the given restrictions.  The result is unpredictable when the determinant of mat is equal to or near 0.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> affineInverse( const SoaMatrix4<SoaNFloat> & mat );

// Compute the inverse of a 4x4 matrix, which is expected to be an affine matrix with an orthogonal upper-left 3x3 submatrix
// NOTE: 
// This can be used to achieve better performance than a general inverse when the specified 4x4 matrix meets the given restrictions.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> orthoInverse( const SoaMatrix4<SoaNFloat> & mat );

// Determinant of a 4x4 matrix
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE SoaNFloat determinant( const SoaMatrix4<SoaNFloat> & mat );

// Conditionally select between two 4x4 matrices
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaMatrix4<SoaNFloat> select( const SoaMatrix4<SoaNFloat> & mat0, const SoaMatrix4<SoaNFloat> & mat1, typename SoaNFloat::SoaBool select1 );

#ifdef _VECTORMATH_DEBUG

// Print a 4x4 matrix
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaMatrix4<SoaNFloat> & mat );

// Print a 4x4 matrix and an associated string identifier
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaMatrix4<SoaNFloat> & mat, const char * name );

#endif

// A 3x4 transformation matrix in structure-of-arrays format
//
template< class SoaNFloat > 
class SoaTransform3
{
public:
    SoaVector3<SoaNFloat> col0;
    SoaVector3<SoaNFloat> col1;
    SoaVector3<SoaNFloat> col2;
    SoaVector3<SoaNFloat> col3;

    // Default constructor; does no initialization
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaTransform3( ) { };

    // Copy a 3x4 transformation matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaTransform3( const SoaTransform3 & tfrm );

    // Construct a 3x4 transformation matrix containing the specified columns
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaTransform3( const SoaVector3<SoaNFloat> & col0, const SoaVector3<SoaNFloat> & col1, const SoaVector3<SoaNFloat> & col2, const SoaVector3<SoaNFloat> & col3 );

    // Construct a 3x4 transformation matrix from a 3x3 matrix and a 3-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaTransform3( const SoaMatrix3<SoaNFloat> & tfrm, const SoaVector3<SoaNFloat> & translateVec );

    // Construct a 3x4 transformation matrix from a unit-length quaternion and a 3-D vector
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaTransform3( const SoaQuat<SoaNFloat> & unitSoaQuat, const SoaVector3<SoaNFloat> & translateVec );

    // Set all elements of a 3x4 transformation matrix to the same scalar value
    // 
    explicit SOA_VECTORMATH_FORCE_INLINE SoaTransform3( SoaNFloat scalar );

    // Assign one 3x4 transformation matrix to another
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaTransform3 & operator =( const SoaTransform3 & tfrm );

    // Set the upper-left 3x3 submatrix
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaTransform3 & setUpper3x3( const SoaMatrix3<SoaNFloat> & mat3 );

    // Get the upper-left 3x3 submatrix of a 3x4 transformation matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaMatrix3<SoaNFloat> getUpper3x3( ) const;

    // Set translation component
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaTransform3 & setTranslation( const SoaVector3<SoaNFloat> & translateVec );

    // Get the translation component of a 3x4 transformation matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> getTranslation( ) const;

    // Set column 0 of a 3x4 transformation matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaTransform3 & setCol0( const SoaVector3<SoaNFloat> & col0 );

    // Set column 1 of a 3x4 transformation matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaTransform3 & setCol1( const SoaVector3<SoaNFloat> & col1 );

    // Set column 2 of a 3x4 transformation matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaTransform3 & setCol2( const SoaVector3<SoaNFloat> & col2 );

    // Set column 3 of a 3x4 transformation matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaTransform3 & setCol3( const SoaVector3<SoaNFloat> & col3 );

    // Get column 0 of a 3x4 transformation matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> getCol0( ) const;

    // Get column 1 of a 3x4 transformation matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> getCol1( ) const;

    // Get column 2 of a 3x4 transformation matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> getCol2( ) const;

    // Get column 3 of a 3x4 transformation matrix
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> getCol3( ) const;

    // Set the column of a 3x4 transformation matrix referred to by the specified index
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaTransform3 & setCol( int col, const SoaVector3<SoaNFloat> & vec );

    // Set the row of a 3x4 transformation matrix referred to by the specified index
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaTransform3 & setRow( int row, const SoaVector4<SoaNFloat> & vec );

    // Get the column of a 3x4 transformation matrix referred to by the specified index
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> getCol( int col ) const;

    // Get the row of a 3x4 transformation matrix referred to by the specified index
    // 
    SOA_VECTORMATH_FORCE_INLINE const SoaVector4<SoaNFloat> getRow( int row ) const;

    // Set the element of a 3x4 transformation matrix referred to by column and row indices
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaTransform3 & setElem( int col, int row, SoaNFloat val );

    // Get the element of a 3x4 transformation matrix referred to by column and row indices
    // 
    SOA_VECTORMATH_FORCE_INLINE SoaNFloat getElem( int col, int row ) const;

    // Construct an identity 3x4 transformation matrix
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaTransform3 identity( );

    // Construct a 3x4 transformation matrix to rotate around the x axis
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaTransform3 rotationX( SoaNFloat radians );

    // Construct a 3x4 transformation matrix to rotate around the y axis
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaTransform3 rotationY( SoaNFloat radians );

    // Construct a 3x4 transformation matrix to rotate around the z axis
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaTransform3 rotationZ( SoaNFloat radians );

    // Construct a 3x4 transformation matrix to rotate around the x, y, and z axes
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaTransform3 rotationZYX( const SoaVector3<SoaNFloat> & radiansXYZ );

    // Construct a 3x4 transformation matrix to rotate around a unit-length 3-D vector
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaTransform3 rotation( SoaNFloat radians, const SoaVector3<SoaNFloat> & unitVec );

    // Construct a rotation matrix from a unit-length quaternion
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaTransform3 rotation( const SoaQuat<SoaNFloat> & unitSoaQuat );

    // Construct a 3x4 transformation matrix to perform scaling
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaTransform3 scale( const SoaVector3<SoaNFloat> & scaleVec );

    // Construct a 3x4 transformation matrix to perform translation
    // 
    static SOA_VECTORMATH_FORCE_INLINE const SoaTransform3 translation( const SoaVector3<SoaNFloat> & translateVec );

};

// Multiply a 3x4 transformation matrix by a 3-D vector
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaVector3<SoaNFloat> mul(const SoaTransform3<SoaNFloat> & tfrm, const SoaVector3<SoaNFloat> & vec);

// Multiply a 3x4 transformation matrix by a 3-D point
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaPoint3<SoaNFloat> mul(const SoaTransform3<SoaNFloat> & tfrm, const SoaPoint3<SoaNFloat> & pnt);

// Multiply two 3x4 transformation matrices
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> mul(const SoaTransform3<SoaNFloat> & tfrm0, const SoaTransform3<SoaNFloat> & tfrm1);

// Append (post-multiply) a scale transformation to a 3x4 transformation matrix
// NOTE: 
// Faster than creating and multiplying a scale transformation matrix.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> appendScale( const SoaTransform3<SoaNFloat> & tfrm, const SoaVector3<SoaNFloat> & scaleVec );

// Prepend (pre-multiply) a scale transformation to a 3x4 transformation matrix
// NOTE: 
// Faster than creating and multiplying a scale transformation matrix.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> prependScale( const SoaVector3<SoaNFloat> & scaleVec, const SoaTransform3<SoaNFloat> & tfrm );

// Multiply two 3x4 transformation matrices per element
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> operator *( const SoaTransform3<SoaNFloat> & tfrm0, const SoaTransform3<SoaNFloat> & tfrm1 );

// Compute the absolute value of a 3x4 transformation matrix per element
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> abs( const SoaTransform3<SoaNFloat> & tfrm );

// Inverse of a 3x4 transformation matrix
// NOTE: 
// Result is unpredictable when the determinant of the left 3x3 submatrix is equal to or near 0.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> inverse( const SoaTransform3<SoaNFloat> & tfrm );

// Compute the inverse of a 3x4 transformation matrix, expected to have an orthogonal upper-left 3x3 submatrix
// NOTE: 
// This can be used to achieve better performance than a general inverse when the specified 3x4 transformation matrix meets the given restrictions.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> orthoInverse( const SoaTransform3<SoaNFloat> & tfrm );

// Conditionally select between two 3x4 transformation matrices
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE const SoaTransform3<SoaNFloat> select( const SoaTransform3<SoaNFloat> & tfrm0, const SoaTransform3<SoaNFloat> & tfrm1, typename SoaNFloat::SoaBool select1 );

#ifdef _VECTORMATH_DEBUG

// Print a 3x4 transformation matrix
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaTransform3<SoaNFloat> & tfrm );

// Print a 3x4 transformation matrix and an associated string identifier
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
template< class SoaNFloat > SOA_VECTORMATH_FORCE_INLINE void print( const SoaTransform3<SoaNFloat> & tfrm, const char * name );

#endif

} // namespace FmVectormath

#include "soa_vec.h"
#include "soa_quat.h"
#include "soa_mat.h"

#endif
