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


#ifndef _SIMD_VECTORMATH_AOS_CPP_SSE_H
#define _SIMD_VECTORMATH_AOS_CPP_SSE_H

#include "vectormath_utils.h"
#include "simd_float.h"
#include "simd_bool.h"

#ifdef _VECTORMATH_DEBUG
#include <stdio.h>
#endif

#pragma warning(disable : 4201)

namespace FmVectormath {

//-----------------------------------------------------------------------------
// Forward Declarations
//

class SimdVector3;
class SimdVector4;
class SimdPoint3;
class SimdQuat;
class SimdMatrix3;
class SimdMatrix4;
class SimdTransform3;

// A 3-D vector in array-of-structures format
//
class SimdVector3
{
public:
    union 
    {
        struct
        {
            float x, y, z, _unused;
        };
        __m128 mVec128;
    };

    SIMD_VECTORMATH_FORCE_INLINE void set128(__m128 vec);
     
    // Default constructor; does no initialization
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector3( ) { };

    // Copy a 3-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector3( const SimdVector3 & vec );

    // Construct a 3-D vector from x, y, and z elements
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector3( float x, float y, float z );

    // Construct a 3-D vector from x, y, and z elements (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector3( const SimdFloat &x, const SimdFloat &y, const SimdFloat &z );

    // Copy elements from a 3-D point into a 3-D vector
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdVector3( const SimdPoint3 & pnt );

    // Set all elements of a 3-D vector to the same scalar value
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdVector3( float scalar );

    // Set all elements of a 3-D vector to the same scalar value (scalar data contained in vector data type)
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdVector3( const SimdFloat &scalar );

    // Set vector float data in a 3-D vector
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdVector3( __m128 vf4 );

    // Get vector float data from a 3-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE __m128 get128( ) const;

    // Assign one 3-D vector to another
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & operator =( const SimdVector3 & vec );

    // Set the x element of a 3-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & setX( float x );

    // Set the y element of a 3-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & setY( float y );

    // Set the z element of a 3-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & setZ( float z );

    // Set the x element of a 3-D vector (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & setX( const SimdFloat &x );

    // Set the y element of a 3-D vector (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & setY( const SimdFloat &y );

    // Set the z element of a 3-D vector (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & setZ( const SimdFloat &z );

    // Get the x element of a 3-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getX( ) const;

    // Get the y element of a 3-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getY( ) const;

    // Get the z element of a 3-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getZ( ) const;

    // Set an x, y, or z element of a 3-D vector by index
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & setElem( int idx, float value );

    // Set an x, y, or z element of a 3-D vector by index (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & setElem( int idx, const SimdFloat & value );

    // Get an x, y, or z element of a 3-D vector by index
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getElem( int idx ) const;

    // Add two 3-D vectors
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 operator +( const SimdVector3 & vec ) const;

    // Subtract a 3-D vector from another 3-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 operator -( const SimdVector3 & vec ) const;

    // Add a 3-D vector to a 3-D point
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 operator +( const SimdPoint3 & pnt ) const;

    // Multiply a 3-D vector by a scalar
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 operator *( float scalar ) const;

    // Divide a 3-D vector by a scalar
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 operator /( float scalar ) const;

    // Multiply a 3-D vector by a scalar (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 operator *( const SimdFloat &scalar ) const;

    // Divide a 3-D vector by a scalar (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 operator /( const SimdFloat &scalar ) const;

    // Perform compound assignment and addition with a 3-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & operator +=( const SimdVector3 & vec );

    // Perform compound assignment and subtraction by a 3-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & operator -=( const SimdVector3 & vec );

    // Perform compound assignment and multiplication by a scalar
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & operator *=( float scalar );

    // Perform compound assignment and division by a scalar
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & operator /=( float scalar );

    // Perform compound assignment and multiplication by a scalar (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & operator *=( const SimdFloat &scalar );

    // Perform compound assignment and division by a scalar (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector3 & operator /=( const SimdFloat &scalar );

    // Negate all elements of a 3-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 operator -( ) const;

    // Construct x axis
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 xAxis( );

    // Construct y axis
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 yAxis( );

    // Construct z axis
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 zAxis( );

};

// Multiply a 3-D vector by a scalar
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 operator *( float scalar, const SimdVector3 & vec );

// Multiply a 3-D vector by a scalar (scalar data contained in vector data type)
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 operator *( const SimdFloat &scalar, const SimdVector3 & vec );

// Multiply two 3-D vectors per element
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 operator *( const SimdVector3 & vec0, const SimdVector3 & vec1 );

// Divide two 3-D vectors per element
// NOTE: 
// Floating-point behavior matches standard library function divf4.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 operator /( const SimdVector3 & vec0, const SimdVector3 & vec1 );

// Compute the reciprocal of a 3-D vector per element
// NOTE: 
// Floating-point behavior matches standard library function _mm_rcp_ps.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 inv( const SimdVector3 & vec );

// Compute the square root of a 3-D vector per element
// NOTE: 
// Floating-point behavior matches standard library function _mm_sqrt_ps.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 sqrt( const SimdVector3 & vec );

// Compute the reciprocal square root of a 3-D vector per element
// NOTE: 
// Floating-point behavior matches standard library function _mm_rsqrt_ps.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 rsqrt( const SimdVector3 & vec );

// Compute the absolute value of a 3-D vector per element
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 abs( const SimdVector3 & vec );

// Copy sign from one 3-D vector to another, per element
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 copySign( const SimdVector3 & vec0, const SimdVector3 & vec1 );

// Maximum of two 3-D vectors per element
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 max( const SimdVector3 & vec0, const SimdVector3 & vec1 );

// Minimum of two 3-D vectors per element
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 min( const SimdVector3 & vec0, const SimdVector3 & vec1 );

// Maximum element of a 3-D vector
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat maxElem( const SimdVector3 & vec );

// Minimum element of a 3-D vector
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat minElem( const SimdVector3 & vec );

// Compute the sum of all elements of a 3-D vector
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat sum( const SimdVector3 & vec );

// Compute the dot product of two 3-D vectors
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat dot( const SimdVector3 & vec0, const SimdVector3 & vec1 );

// Compute the square of the length of a 3-D vector
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat lengthSqr( const SimdVector3 & vec );

// Compute the length of a 3-D vector
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat length( const SimdVector3 & vec );

// Normalize a 3-D vector
// NOTE: 
// The result is unpredictable when all elements of vec are at or near zero.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 normalize( const SimdVector3 & vec );

// Compute cross product of two 3-D vectors
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 cross( const SimdVector3 & vec0, const SimdVector3 & vec1 );

// Outer product of two 3-D vectors
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 outer( const SimdVector3 & vec0, const SimdVector3 & vec1 );

// Pre-multiply a row vector by a 3x3 matrix
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 mul( const SimdVector3 & vec, const SimdMatrix3 & mat );

// Cross-product matrix of a 3-D vector
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 crossMatrix( const SimdVector3 & vec );

// Create cross-product matrix and multiply
// NOTE: 
// Faster than separately creating a cross-product matrix and multiplying.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 crossMatrixMul( const SimdVector3 & vec, const SimdMatrix3 & mat );

// Linear interpolation between two 3-D vectors
// NOTE: 
// Does not clamp t between 0 and 1.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 lerp( float t, const SimdVector3 & vec0, const SimdVector3 & vec1 );

// Linear interpolation between two 3-D vectors (scalar data contained in vector data type)
// NOTE: 
// Does not clamp t between 0 and 1.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 lerp( const SimdFloat &t, const SimdVector3 & vec0, const SimdVector3 & vec1 );

// Spherical linear interpolation between two 3-D vectors
// NOTE: 
// The result is unpredictable if the vectors point in opposite directions.
// Does not clamp t between 0 and 1.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 slerp( float t, const SimdVector3 & unitVec0, const SimdVector3 & unitVec1 );

// Spherical linear interpolation between two 3-D vectors (scalar data contained in vector data type)
// NOTE: 
// The result is unpredictable if the vectors point in opposite directions.
// Does not clamp t between 0 and 1.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 slerp( const SimdFloat &t, const SimdVector3 & unitVec0, const SimdVector3 & unitVec1 );

// Conditionally select between two 3-D vectors
// NOTE: 
// This function uses a conditional select instruction to avoid a branch.
// However, the transfer of select1 to a VMX register may use more processing time than a branch.
// Use the SimdBool version for better performance.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 select( const SimdVector3 & vec0, const SimdVector3 & vec1, bool select1 );

// Conditionally select between two 3-D vectors (scalar data contained in vector data type)
// NOTE: 
// This function uses a conditional select instruction to avoid a branch.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 select( const SimdVector3 & vec0, const SimdVector3 & vec1, const SimdBool &select1 );

// Load x, y, and z elements from the first three words of a float array.
// 
// 
SIMD_VECTORMATH_FORCE_INLINE void loadXYZ( SimdVector3 & vec, const float * fptr );

// Store x, y, and z elements of 3-D vector in first three words of a quadword, preserving fourth word
// 
SIMD_VECTORMATH_FORCE_INLINE void storeXYZ( const SimdVector3 & vec, __m128 * quad );

// Load four three-float 3-D vectors, stored in three quadwords
// 
SIMD_VECTORMATH_FORCE_INLINE void loadXYZArray( SimdVector3 & vec0, SimdVector3 & vec1, SimdVector3 & vec2, SimdVector3 & vec3, const __m128 * threeQuads );

// Store four 3-D vectors in three quadwords
// 
SIMD_VECTORMATH_FORCE_INLINE void storeXYZArray( const SimdVector3 & vec0, const SimdVector3 & vec1, const SimdVector3 & vec2, const SimdVector3 & vec3, __m128 * threeQuads );

#ifdef _VECTORMATH_DEBUG

// Print a 3-D vector
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
SIMD_VECTORMATH_FORCE_INLINE void print( const SimdVector3 & vec );

// Print a 3-D vector and an associated string identifier
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
SIMD_VECTORMATH_FORCE_INLINE void print( const SimdVector3 & vec, const char * name );

#endif

// A 4-D vector in array-of-structures format
//
class SimdVector4
{
public:
    union
    {
        struct
        {
            float x, y, z, w;
        };
        __m128 mVec128;
    };

    // Default constructor; does no initialization
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4( ) { };

    // Copy a 4-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4( const SimdVector4 & vec );

    // Construct a 4-D vector from x, y, z, and w elements
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4( float x, float y, float z, float w );

    // Construct a 4-D vector from x, y, z, and w elements (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4( const SimdFloat &x, const SimdFloat &y, const SimdFloat &z, const SimdFloat &w );

    // Construct a 4-D vector from a 3-D vector and a scalar
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4( const SimdVector3 & xyz, float w );

    // Construct a 4-D vector from a 3-D vector and a scalar (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4( const SimdVector3 &xyz, const SimdFloat &w );

    // Copy x, y, and z from a 3-D vector into a 4-D vector, and set w to 0
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdVector4( const SimdVector3 & vec );

    // Copy x, y, and z from a 3-D point into a 4-D vector, and set w to 1
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdVector4( const SimdPoint3 & pnt );

    // Copy elements from a quaternion into a 4-D vector
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdVector4( const SimdQuat & quat );

    // Set all elements of a 4-D vector to the same scalar value
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdVector4( float scalar );

    // Set all elements of a 4-D vector to the same scalar value (scalar data contained in vector data type)
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdVector4( const SimdFloat &scalar );

    // Set vector float data in a 4-D vector
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdVector4( __m128 vf4 );

    // Get vector float data from a 4-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE __m128 get128( ) const;

    // Assign one 4-D vector to another
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & operator =( const SimdVector4 & vec );

    // Set the x, y, and z elements of a 4-D vector
    // NOTE: 
    // This function does not change the w element.
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & setXYZ( const SimdVector3 & vec );

    // Get the x, y, and z elements of a 4-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 getXYZ( ) const;

    // Set the x element of a 4-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & setX( float x );

    // Set the y element of a 4-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & setY( float y );

    // Set the z element of a 4-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & setZ( float z );

    // Set the w element of a 4-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & setW( float w );

    // Set the x element of a 4-D vector (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & setX( const SimdFloat &x );

    // Set the y element of a 4-D vector (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & setY( const SimdFloat &y );

    // Set the z element of a 4-D vector (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & setZ( const SimdFloat &z );

    // Set the w element of a 4-D vector (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & setW( const SimdFloat &w );

    // Get the x element of a 4-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getX( ) const;

    // Get the y element of a 4-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getY( ) const;

    // Get the z element of a 4-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getZ( ) const;

    // Get the w element of a 4-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getW( ) const;

    // Set an x, y, z, or w element of a 4-D vector by index
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & setElem( int idx, float value );

    // Set an x, y, z, or w element of a 4-D vector by index (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & setElem( int idx, const SimdFloat & value );

    // Get an x, y, z, or w element of a 4-D vector by index
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getElem( int idx ) const;

    // Add two 4-D vectors
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 operator +( const SimdVector4 & vec ) const;

    // Subtract a 4-D vector from another 4-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 operator -( const SimdVector4 & vec ) const;

    // Multiply a 4-D vector by a scalar
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 operator *( float scalar ) const;

    // Divide a 4-D vector by a scalar
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 operator /( float scalar ) const;

    // Multiply a 4-D vector by a scalar (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 operator *( const SimdFloat &scalar ) const;

    // Divide a 4-D vector by a scalar (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 operator /( const SimdFloat &scalar ) const;

    // Perform compound assignment and addition with a 4-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & operator +=( const SimdVector4 & vec );

    // Perform compound assignment and subtraction by a 4-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & operator -=( const SimdVector4 & vec );

    // Perform compound assignment and multiplication by a scalar
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & operator *=( float scalar );

    // Perform compound assignment and division by a scalar
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & operator /=( float scalar );

    // Perform compound assignment and multiplication by a scalar (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & operator *=( const SimdFloat &scalar );

    // Perform compound assignment and division by a scalar (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdVector4 & operator /=( const SimdFloat &scalar );

    // Negate all elements of a 4-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 operator -( ) const;

    // Construct x axis
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 xAxis( );

    // Construct y axis
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 yAxis( );

    // Construct z axis
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 zAxis( );

    // Construct w axis
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 wAxis( );

};

// Multiply a 4-D vector by a scalar
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 operator *( float scalar, const SimdVector4 & vec );

// Multiply a 4-D vector by a scalar (scalar data contained in vector data type)
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 operator *( const SimdFloat &scalar, const SimdVector4 & vec );

// Multiply two 4-D vectors per element
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 operator *( const SimdVector4 & vec0, const SimdVector4 & vec1 );

// Divide two 4-D vectors per element
// NOTE: 
// Floating-point behavior matches standard library function divf4.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 operator /( const SimdVector4 & vec0, const SimdVector4 & vec1 );

// Compute the reciprocal of a 4-D vector per element
// NOTE: 
// Floating-point behavior matches standard library function _mm_rcp_ps.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 inv( const SimdVector4 & vec );

// Compute the square root of a 4-D vector per element
// NOTE: 
// Floating-point behavior matches standard library function _mm_sqrt_ps.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 sqrt( const SimdVector4 & vec );

// Compute the reciprocal square root of a 4-D vector per element
// NOTE: 
// Floating-point behavior matches standard library function _mm_rsqrt_ps.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 rsqrt( const SimdVector4 & vec );

// Compute the absolute value of a 4-D vector per element
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 abs( const SimdVector4 & vec );

// Copy sign from one 4-D vector to another, per element
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 copySign( const SimdVector4 & vec0, const SimdVector4 & vec1 );

// Maximum of two 4-D vectors per element
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 max( const SimdVector4 & vec0, const SimdVector4 & vec1 );

// Minimum of two 4-D vectors per element
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 min( const SimdVector4 & vec0, const SimdVector4 & vec1 );

// Maximum element of a 4-D vector
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat maxElem( const SimdVector4 & vec );

// Minimum element of a 4-D vector
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat minElem( const SimdVector4 & vec );

// Compute the sum of all elements of a 4-D vector
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat sum( const SimdVector4 & vec );

// Compute the dot product of two 4-D vectors
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat dot( const SimdVector4 & vec0, const SimdVector4 & vec1 );

// Compute the square of the length of a 4-D vector
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat lengthSqr( const SimdVector4 & vec );

// Compute the length of a 4-D vector
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat length( const SimdVector4 & vec );

// Normalize a 4-D vector
// NOTE: 
// The result is unpredictable when all elements of vec are at or near zero.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 normalize( const SimdVector4 & vec );

// Outer product of two 4-D vectors
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 outer( const SimdVector4 & vec0, const SimdVector4 & vec1 );

// Linear interpolation between two 4-D vectors
// NOTE: 
// Does not clamp t between 0 and 1.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 lerp( float t, const SimdVector4 & vec0, const SimdVector4 & vec1 );

// Linear interpolation between two 4-D vectors (scalar data contained in vector data type)
// NOTE: 
// Does not clamp t between 0 and 1.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 lerp( const SimdFloat &t, const SimdVector4 & vec0, const SimdVector4 & vec1 );

// Spherical linear interpolation between two 4-D vectors
// NOTE: 
// The result is unpredictable if the vectors point in opposite directions.
// Does not clamp t between 0 and 1.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 slerp( float t, const SimdVector4 & unitVec0, const SimdVector4 & unitVec1 );

// Spherical linear interpolation between two 4-D vectors (scalar data contained in vector data type)
// NOTE: 
// The result is unpredictable if the vectors point in opposite directions.
// Does not clamp t between 0 and 1.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 slerp( const SimdFloat &t, const SimdVector4 & unitVec0, const SimdVector4 & unitVec1 );

// Conditionally select between two 4-D vectors
// NOTE: 
// This function uses a conditional select instruction to avoid a branch.
// However, the transfer of select1 to a VMX register may use more processing time than a branch.
// Use the SimdBool version for better performance.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 select( const SimdVector4 & vec0, const SimdVector4 & vec1, bool select1 );

// Conditionally select between two 4-D vectors (scalar data contained in vector data type)
// NOTE: 
// This function uses a conditional select instruction to avoid a branch.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 select( const SimdVector4 & vec0, const SimdVector4 & vec1, const SimdBool &select1 );

// Load x, y, z, and w elements from the first four words of a float array.
// 
// 
SIMD_VECTORMATH_FORCE_INLINE void loadXYZW(SimdVector4 & vec, const float * fptr);

// Store x, y, z, and w elements of a 4-D vector in the first four words of a float array.
// Memory area of previous 16 bytes and next 32 bytes from fptr might be accessed
// 
SIMD_VECTORMATH_FORCE_INLINE void storeXYZW(const SimdVector4 & vec, float * fptr);

#ifdef _VECTORMATH_DEBUG

// Print a 4-D vector
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
SIMD_VECTORMATH_FORCE_INLINE void print( const SimdVector4 & vec );

// Print a 4-D vector and an associated string identifier
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
SIMD_VECTORMATH_FORCE_INLINE void print( const SimdVector4 & vec, const char * name );

#endif

// A 3-D point in array-of-structures format
//
class SimdPoint3
{
public:
    union
    {
        struct
        {
            float x, y, z, _unused;
        };
        __m128 mVec128;
    };

    // Default constructor; does no initialization
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdPoint3( ) { };

    // Copy a 3-D point
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdPoint3( const SimdPoint3 & pnt );

    // Construct a 3-D point from x, y, and z elements
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdPoint3( float x, float y, float z );

    // Construct a 3-D point from x, y, and z elements (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdPoint3( const SimdFloat &x, const SimdFloat &y, const SimdFloat &z );

    // Copy elements from a 3-D vector into a 3-D point
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdPoint3( const SimdVector3 & vec );

    // Set all elements of a 3-D point to the same scalar value
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdPoint3( float scalar );

    // Set all elements of a 3-D point to the same scalar value (scalar data contained in vector data type)
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdPoint3( const SimdFloat &scalar );

    // Set vector float data in a 3-D point
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdPoint3( __m128 vf4 );

    // Get vector float data from a 3-D point
    // 
    SIMD_VECTORMATH_FORCE_INLINE __m128 get128( ) const;

    // Assign one 3-D point to another
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & operator =( const SimdPoint3 & pnt );

    // Set the x element of a 3-D point
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & setX( float x );

    // Set the y element of a 3-D point
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & setY( float y );

    // Set the z element of a 3-D point
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & setZ( float z );

    // Set the x element of a 3-D point (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & setX( const SimdFloat &x );

    // Set the y element of a 3-D point (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & setY( const SimdFloat &y );

    // Set the z element of a 3-D point (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & setZ( const SimdFloat &z );

    // Get the x element of a 3-D point
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getX( ) const;

    // Get the y element of a 3-D point
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getY( ) const;

    // Get the z element of a 3-D point
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getZ( ) const;

    // Set an x, y, or z element of a 3-D point by index
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & setElem( int idx, float value );

    // Set an x, y, or z element of a 3-D point by index (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & setElem( int idx, const SimdFloat & value );

    // Get an x, y, or z element of a 3-D point by index
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getElem( int idx ) const;

    // Subtract a 3-D point from another 3-D point
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 operator -( const SimdPoint3 & pnt ) const;

    // Add a 3-D point to a 3-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 operator +( const SimdVector3 & vec ) const;

    // Subtract a 3-D vector from a 3-D point
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 operator -( const SimdVector3 & vec ) const;

    // Perform compound assignment and addition with a 3-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & operator +=( const SimdVector3 & vec );

    // Perform compound assignment and subtraction by a 3-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdPoint3 & operator -=( const SimdVector3 & vec );

};

// Multiply two 3-D points per element
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 operator *( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1 );

// Divide two 3-D points per element
// NOTE: 
// Floating-point behavior matches standard library function divf4.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 operator /( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1 );

// Compute the reciprocal of a 3-D point per element
// NOTE: 
// Floating-point behavior matches standard library function _mm_rcp_ps.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 inv( const SimdPoint3 & pnt );

// Compute the square root of a 3-D point per element
// NOTE: 
// Floating-point behavior matches standard library function _mm_sqrt_ps.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 sqrt( const SimdPoint3 & pnt );

// Compute the reciprocal square root of a 3-D point per element
// NOTE: 
// Floating-point behavior matches standard library function _mm_rsqrt_ps.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 rsqrt( const SimdPoint3 & pnt );

// Compute the absolute value of a 3-D point per element
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 abs( const SimdPoint3 & pnt );

// Copy sign from one 3-D point to another, per element
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 copySign( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1 );

// Maximum of two 3-D points per element
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 max( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1 );

// Minimum of two 3-D points per element
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 min( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1 );

// Maximum element of a 3-D point
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat maxElem( const SimdPoint3 & pnt );

// Minimum element of a 3-D point
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat minElem( const SimdPoint3 & pnt );

// Compute the sum of all elements of a 3-D point
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat sum( const SimdPoint3 & pnt );

// Apply uniform scale to a 3-D point
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 scale( const SimdPoint3 & pnt, float scaleVal );

// Apply uniform scale to a 3-D point (scalar data contained in vector data type)
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 scale( const SimdPoint3 & pnt, const SimdFloat &scaleVal );

// Apply non-uniform scale to a 3-D point
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 scale( const SimdPoint3 & pnt, const SimdVector3 & scaleVec );

// Scalar projection of a 3-D point on a unit-length 3-D vector
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat projection( const SimdPoint3 & pnt, const SimdVector3 & unitVec );

// Compute the square of the distance of a 3-D point from the coordinate-system origin
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat distSqrFromOrigin( const SimdPoint3 & pnt );

// Compute the distance of a 3-D point from the coordinate-system origin
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat distFromOrigin( const SimdPoint3 & pnt );

// Compute the square of the distance between two 3-D points
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat distSqr( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1 );

// Compute the distance between two 3-D points
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat dist( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1 );

// Linear interpolation between two 3-D points
// NOTE: 
// Does not clamp t between 0 and 1.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 lerp( float t, const SimdPoint3 & pnt0, const SimdPoint3 & pnt1 );

// Linear interpolation between two 3-D points (scalar data contained in vector data type)
// NOTE: 
// Does not clamp t between 0 and 1.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 lerp( const SimdFloat &t, const SimdPoint3 & pnt0, const SimdPoint3 & pnt1 );

// Conditionally select between two 3-D points
// NOTE: 
// This function uses a conditional select instruction to avoid a branch.
// However, the transfer of select1 to a VMX register may use more processing time than a branch.
// Use the SimdBool version for better performance.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 select( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1, bool select1 );

// Conditionally select between two 3-D points (scalar data contained in vector data type)
// NOTE: 
// This function uses a conditional select instruction to avoid a branch.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 select( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1, const SimdBool &select1 );

// Load x, y, and z elements from the first three words of a float array.
// 
// 
SIMD_VECTORMATH_FORCE_INLINE void loadXYZ( SimdPoint3 & pnt, const float * fptr );

// Store x, y, and z elements of 3-D point in first three words of a quadword, preserving fourth word
// 
SIMD_VECTORMATH_FORCE_INLINE void storeXYZ( const SimdPoint3 & pnt, __m128 * quad );

// Load four three-float 3-D points, stored in three quadwords
// 
SIMD_VECTORMATH_FORCE_INLINE void loadXYZArray( SimdPoint3 & pnt0, SimdPoint3 & pnt1, SimdPoint3 & pnt2, SimdPoint3 & pnt3, const __m128 * threeQuads );

// Store four 3-D points in three quadwords
// 
SIMD_VECTORMATH_FORCE_INLINE void storeXYZArray( const SimdPoint3 & pnt0, const SimdPoint3 & pnt1, const SimdPoint3 & pnt2, const SimdPoint3 & pnt3, __m128 * threeQuads );

#ifdef _VECTORMATH_DEBUG

// Print a 3-D point
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
SIMD_VECTORMATH_FORCE_INLINE void print( const SimdPoint3 & pnt );

// Print a 3-D point and an associated string identifier
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
SIMD_VECTORMATH_FORCE_INLINE void print( const SimdPoint3 & pnt, const char * name );

#endif

// A quaternion in array-of-structures format
//
class SimdQuat
{
public:
    union
    {
        struct
        {
            float x, y, z, w;
        };
        __m128 mVec128;
    };

    // Default constructor; does no initialization
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat( ) { };

    // Copy a quaternion
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat( const SimdQuat & quat );

    // Construct a quaternion from x, y, z, and w elements
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat( float x, float y, float z, float w );

    // Construct a quaternion from x, y, z, and w elements (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat( const SimdFloat &x, const SimdFloat &y, const SimdFloat &z, const SimdFloat &w );

    // Construct a quaternion from a 3-D vector and a scalar
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat( const SimdVector3 & xyz, float w );

    // Construct a quaternion from a 3-D vector and a scalar (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat( const SimdVector3 &xyz, const SimdFloat &w );

    // Copy elements from a 4-D vector into a quaternion
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdQuat( const SimdVector4 & vec );

    // Convert a rotation matrix to a unit-length quaternion
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdQuat( const SimdMatrix3 & rotMat );

    // Set all elements of a quaternion to the same scalar value
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdQuat( float scalar );

    // Set all elements of a quaternion to the same scalar value (scalar data contained in vector data type)
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdQuat( const SimdFloat &scalar );

    // Set vector float data in a quaternion
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdQuat( __m128 vf4 );

    // Get vector float data from a quaternion
    // 
    SIMD_VECTORMATH_FORCE_INLINE __m128 get128( ) const;

    // Set a quaterion from vector float data
    //
    SIMD_VECTORMATH_FORCE_INLINE void set128(__m128 vec);

    // Assign one quaternion to another
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat & operator =( const SimdQuat & quat );

    // Set the x, y, and z elements of a quaternion
    // NOTE: 
    // This function does not change the w element.
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat & setXYZ( const SimdVector3 & vec );

    // Get the x, y, and z elements of a quaternion
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 getXYZ( ) const;

    // Set the x element of a quaternion
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat & setX( float x );

    // Set the y element of a quaternion
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat & setY( float y );

    // Set the z element of a quaternion
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat & setZ( float z );

    // Set the w element of a quaternion
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat & setW( float w );

    // Set the x element of a quaternion (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat & setX( const SimdFloat &x );

    // Set the y element of a quaternion (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat & setY( const SimdFloat &y );

    // Set the z element of a quaternion (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat & setZ( const SimdFloat &z );

    // Set the w element of a quaternion (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat & setW( const SimdFloat &w );

    // Get the x element of a quaternion
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getX( ) const;

    // Get the y element of a quaternion
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getY( ) const;

    // Get the z element of a quaternion
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getZ( ) const;

    // Get the w element of a quaternion
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getW( ) const;

    // Set an x, y, z, or w element of a quaternion by index
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat & setElem( int idx, float value );

    // Set an x, y, z, or w element of a quaternion by index (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat & setElem( int idx, const SimdFloat & value );

    // Get an x, y, z, or w element of a quaternion by index
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getElem( int idx ) const;

    // Add two quaternions
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdQuat operator +( const SimdQuat & quat ) const;

    // Subtract a quaternion from another quaternion
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdQuat operator -( const SimdQuat & quat ) const;

    // Multiply two quaternions
    // 
    //SIMD_VECTORMATH_FORCE_INLINE const SimdQuat operator *( const SimdQuat & quat ) const;

    // Multiply a quaternion by a scalar
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdQuat operator *( float scalar ) const;

    // Divide a quaternion by a scalar
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdQuat operator /( float scalar ) const;

    // Multiply a quaternion by a scalar (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdQuat operator *( const SimdFloat &scalar ) const;

    // Divide a quaternion by a scalar (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdQuat operator /( const SimdFloat &scalar ) const;

    // Perform compound assignment and addition with a quaternion
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat & operator +=( const SimdQuat & quat );

    // Perform compound assignment and subtraction by a quaternion
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat & operator -=( const SimdQuat & quat );

    // Perform compound assignment and multiplication by a quaternion
    // 
    //SIMD_VECTORMATH_FORCE_INLINE SimdQuat & operator *=( const SimdQuat & quat );

    // Perform compound assignment and multiplication by a scalar
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat & operator *=( float scalar );

    // Perform compound assignment and division by a scalar
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat & operator /=( float scalar );

    // Perform compound assignment and multiplication by a scalar (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat & operator *=( const SimdFloat &scalar );

    // Perform compound assignment and division by a scalar (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdQuat & operator /=( const SimdFloat &scalar );

    // Negate all elements of a quaternion
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdQuat operator -( ) const;

    // Construct an identity quaternion
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdQuat identity( );

    // Construct a quaternion to rotate between two unit-length 3-D vectors
    // NOTE: 
    // The result is unpredictable if unitVec0 and unitVec1 point in opposite directions.
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdQuat rotation( const SimdVector3 & unitVec0, const SimdVector3 & unitVec1 );

    // Construct a quaternion to rotate around a unit-length 3-D vector
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdQuat rotation( float radians, const SimdVector3 & unitVec );

    // Construct a quaternion to rotate around a unit-length 3-D vector (scalar data contained in vector data type)
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdQuat rotation( const SimdFloat &radians, const SimdVector3 & unitVec );

    // Construct a quaternion to rotate around the x axis
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdQuat rotationX( float radians );

    // Construct a quaternion to rotate around the y axis
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdQuat rotationY( float radians );

    // Construct a quaternion to rotate around the z axis
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdQuat rotationZ( float radians );

    // Construct a quaternion to rotate around the x axis (scalar data contained in vector data type)
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdQuat rotationX( const SimdFloat &radians );

    // Construct a quaternion to rotate around the y axis (scalar data contained in vector data type)
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdQuat rotationY( const SimdFloat &radians );

    // Construct a quaternion to rotate around the z axis (scalar data contained in vector data type)
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdQuat rotationZ( const SimdFloat &radians );

};

// Multiply two quaternions
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdQuat mul(const SimdQuat & q0, const SimdQuat & q1);

// Multiply a quaternion by a scalar
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdQuat operator *( float scalar, const SimdQuat & quat );

// Multiply a quaternion by a scalar (scalar data contained in vector data type)
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdQuat operator *( const SimdFloat &scalar, const SimdQuat & quat );

// Compute the conjugate of a quaternion
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdQuat conj( const SimdQuat & quat );

// Use a unit-length quaternion to rotate a 3-D vector
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 rotate( const SimdQuat & unitSimdQuat, const SimdVector3 & vec );

// Compute the dot product of two quaternions
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat dot( const SimdQuat & quat0, const SimdQuat & quat1 );

// Compute the norm of a quaternion
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat norm( const SimdQuat & quat );

// Compute the length of a quaternion
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat length( const SimdQuat & quat );

// Normalize a quaternion
// NOTE: 
// The result is unpredictable when all elements of quat are at or near zero.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdQuat normalize( const SimdQuat & quat );

// Linear interpolation between two quaternions
// NOTE: 
// Does not clamp t between 0 and 1.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdQuat lerp( float t, const SimdQuat & quat0, const SimdQuat & quat1 );

// Linear interpolation between two quaternions (scalar data contained in vector data type)
// NOTE: 
// Does not clamp t between 0 and 1.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdQuat lerp( const SimdFloat &t, const SimdQuat & quat0, const SimdQuat & quat1 );

// Spherical linear interpolation between two quaternions
// NOTE: 
// Interpolates along the shortest path between orientations.
// Does not clamp t between 0 and 1.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdQuat slerp( float t, const SimdQuat & unitSimdQuat0, const SimdQuat & unitSimdQuat1 );

// Spherical linear interpolation between two quaternions (scalar data contained in vector data type)
// NOTE: 
// Interpolates along the shortest path between orientations.
// Does not clamp t between 0 and 1.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdQuat slerp( const SimdFloat &t, const SimdQuat & unitSimdQuat0, const SimdQuat & unitSimdQuat1 );

// Spherical quadrangle interpolation
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdQuat squad( float t, const SimdQuat & unitSimdQuat0, const SimdQuat & unitSimdQuat1, const SimdQuat & unitSimdQuat2, const SimdQuat & unitSimdQuat3 );

// Spherical quadrangle interpolation (scalar data contained in vector data type)
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdQuat squad( const SimdFloat &t, const SimdQuat & unitSimdQuat0, const SimdQuat & unitSimdQuat1, const SimdQuat & unitSimdQuat2, const SimdQuat & unitSimdQuat3 );

// Conditionally select between two quaternions
// NOTE: 
// This function uses a conditional select instruction to avoid a branch.
// However, the transfer of select1 to a VMX register may use more processing time than a branch.
// Use the SimdBool version for better performance.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdQuat select( const SimdQuat & quat0, const SimdQuat & quat1, bool select1 );

// Conditionally select between two quaternions (scalar data contained in vector data type)
// NOTE: 
// This function uses a conditional select instruction to avoid a branch.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdQuat select( const SimdQuat & quat0, const SimdQuat & quat1, const SimdBool &select1 );


#ifdef _VECTORMATH_DEBUG

// Print a quaternion
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
SIMD_VECTORMATH_FORCE_INLINE void print( const SimdQuat & quat );

// Print a quaternion and an associated string identifier
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
SIMD_VECTORMATH_FORCE_INLINE void print( const SimdQuat & quat, const char * name );

#endif

// A 3x3 matrix in array-of-structures format
//
class SimdMatrix3
{
public:
    SimdVector3 col0;
    SimdVector3 col1;
    SimdVector3 col2;

    // Default constructor; does no initialization
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3( ) { };

    // Copy a 3x3 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3( const SimdMatrix3 & mat );

    // Construct a 3x3 matrix containing the specified columns
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3( const SimdVector3 & col0, const SimdVector3 & col1, const SimdVector3 & col2 );

    // Construct a 3x3 rotation matrix from a unit-length quaternion
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3( const SimdQuat & unitSimdQuat );

    // Set all elements of a 3x3 matrix to the same scalar value
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3( float scalar );

    // Set all elements of a 3x3 matrix to the same scalar value (scalar data contained in vector data type)
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3( const SimdFloat &scalar );

    // Assign one 3x3 matrix to another
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & operator =( const SimdMatrix3 & mat );

    // Set column 0 of a 3x3 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & setCol0( const SimdVector3 & col0 );

    // Set column 1 of a 3x3 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & setCol1( const SimdVector3 & col1 );

    // Set column 2 of a 3x3 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & setCol2( const SimdVector3 & col2 );

    // Get column 0 of a 3x3 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 getCol0( ) const;

    // Get column 1 of a 3x3 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 getCol1( ) const;

    // Get column 2 of a 3x3 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 getCol2( ) const;

    // Set the column of a 3x3 matrix referred to by the specified index
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & setCol( int col, const SimdVector3 & vec );

    // Set the row of a 3x3 matrix referred to by the specified index
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & setRow( int row, const SimdVector3 & vec );

    // Get the column of a 3x3 matrix referred to by the specified index
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 getCol( int col ) const;

    // Get the row of a 3x3 matrix referred to by the specified index
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 getRow( int row ) const;

    // Set the element of a 3x3 matrix referred to by column and row indices
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & setElem( int col, int row, float val );

    // Set the element of a 3x3 matrix referred to by column and row indices (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & setElem( int col, int row, const SimdFloat & val );

    // Get the element of a 3x3 matrix referred to by column and row indices
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getElem( int col, int row ) const;

    // Add two 3x3 matrices
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 operator +( const SimdMatrix3 & mat ) const;

    // Subtract a 3x3 matrix from another 3x3 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 operator -( const SimdMatrix3 & mat ) const;

    // Negate all elements of a 3x3 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 operator -( ) const;

    // Multiply a 3x3 matrix by a scalar
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 operator *( float scalar ) const;

    // Multiply a 3x3 matrix by a scalar
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 operator *(const SimdFloat& scalar) const;

    // Perform compound assignment and addition with a 3x3 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & operator +=( const SimdMatrix3 & mat );

    // Perform compound assignment and subtraction by a 3x3 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & operator -=( const SimdMatrix3 & mat );

    // Perform compound assignment and multiplication by a scalar
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & operator *=( float scalar );

    // Perform compound assignment and multiplication by a scalar (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix3 & operator *=( const SimdFloat &scalar );

    // Construct an identity 3x3 matrix
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 identity( );

    // Construct a 3x3 matrix to rotate around the x axis
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 rotationX( float radians );

    // Construct a 3x3 matrix to rotate around the y axis
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 rotationY( float radians );

    // Construct a 3x3 matrix to rotate around the z axis
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 rotationZ( float radians );

    // Construct a 3x3 matrix to rotate around the x axis (scalar data contained in vector data type)
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 rotationX( const SimdFloat &radians );

    // Construct a 3x3 matrix to rotate around the y axis (scalar data contained in vector data type)
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 rotationY( const SimdFloat &radians );

    // Construct a 3x3 matrix to rotate around the z axis (scalar data contained in vector data type)
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 rotationZ( const SimdFloat &radians );

    // Construct a 3x3 matrix to rotate around the x, y, and z axes
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 rotationZYX( const SimdVector3 &radiansXYZ );

    // Construct a 3x3 matrix to rotate around a unit-length 3-D vector
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 rotation( float radians, const SimdVector3 & unitVec );

    // Construct a 3x3 matrix to rotate around a unit-length 3-D vector (scalar data contained in vector data type)
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 rotation( const SimdFloat &radians, const SimdVector3 & unitVec );

    // Construct a rotation matrix from a unit-length quaternion
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 rotation( const SimdQuat & unitSimdQuat );

    // Construct a 3x3 matrix to perform scaling
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 scale( const SimdVector3 & scaleVec );

};

// Multiply a 3x3 matrix by a 3-D vector
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 mul(const SimdMatrix3 & mat, const SimdVector3 & vec);

// Multiply two 3x3 matrices
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 mul(const SimdMatrix3 & mat0, const SimdMatrix3 & mat1);

// Multiply a 3x3 matrix by a scalar
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 operator *( float scalar, const SimdMatrix3 & mat );

// Multiply a 3x3 matrix by a scalar (scalar data contained in vector data type)
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 operator *( const SimdFloat &scalar, const SimdMatrix3 & mat );

// Append (post-multiply) a scale transformation to a 3x3 matrix
// NOTE: 
// Faster than creating and multiplying a scale transformation matrix.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 appendScale( const SimdMatrix3 & mat, const SimdVector3 & scaleVec );

// Prepend (pre-multiply) a scale transformation to a 3x3 matrix
// NOTE: 
// Faster than creating and multiplying a scale transformation matrix.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 prependScale( const SimdVector3 & scaleVec, const SimdMatrix3 & mat );

// Compute the absolute value of a 3x3 matrix per element
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 abs( const SimdMatrix3 & mat );

// Transpose of a 3x3 matrix
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 transpose( const SimdMatrix3 & mat );

// Compute the inverse of a 3x3 matrix
// NOTE: 
// Result is unpredictable when the determinant of mat is equal to or near 0.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 inverse( const SimdMatrix3 & mat );

// Determinant of a 3x3 matrix
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat determinant( const SimdMatrix3 & mat );

// Conditionally select between two 3x3 matrices
// NOTE: 
// This function uses a conditional select instruction to avoid a branch.
// However, the transfer of select1 to a VMX register may use more processing time than a branch.
// Use the SimdBool version for better performance.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 select( const SimdMatrix3 & mat0, const SimdMatrix3 & mat1, bool select1 );

// Conditionally select between two 3x3 matrices (scalar data contained in vector data type)
// NOTE: 
// This function uses a conditional select instruction to avoid a branch.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 select( const SimdMatrix3 & mat0, const SimdMatrix3 & mat1, const SimdBool &select1 );

#ifdef _VECTORMATH_DEBUG

// Print a 3x3 matrix
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
SIMD_VECTORMATH_FORCE_INLINE void print( const SimdMatrix3 & mat );

// Print a 3x3 matrix and an associated string identifier
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
SIMD_VECTORMATH_FORCE_INLINE void print( const SimdMatrix3 & mat, const char * name );

#endif

// A 4x4 matrix in array-of-structures format
//
class SimdMatrix4
{
public:
    SimdVector4 col0;
    SimdVector4 col1;
    SimdVector4 col2;
    SimdVector4 col3;

    // Default constructor; does no initialization
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4( ) { };

    // Copy a 4x4 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4( const SimdMatrix4 & mat );

    // Construct a 4x4 matrix containing the specified columns
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4( const SimdVector4 & col0, const SimdVector4 & col1, const SimdVector4 & col2, const SimdVector4 & col3 );

    // Construct a 4x4 matrix from a 3x4 transformation matrix
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4( const SimdTransform3 & mat );

    // Construct a 4x4 matrix from a 3x3 matrix and a 3-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4( const SimdMatrix3 & mat, const SimdVector3 & translateVec );

    // Construct a 4x4 matrix from a unit-length quaternion and a 3-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4( const SimdQuat & unitSimdQuat, const SimdVector3 & translateVec );

    // Set all elements of a 4x4 matrix to the same scalar value
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4( float scalar );

    // Set all elements of a 4x4 matrix to the same scalar value (scalar data contained in vector data type)
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4( const SimdFloat &scalar );

    // Assign one 4x4 matrix to another
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & operator =( const SimdMatrix4 & mat );

    // Set the upper-left 3x3 submatrix
    // NOTE: 
    // This function does not change the bottom row elements.
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & setUpper3x3( const SimdMatrix3 & mat3 );

    // Get the upper-left 3x3 submatrix of a 4x4 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 getUpper3x3( ) const;

    // Set translation component
    // NOTE: 
    // This function does not change the bottom row elements.
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & setTranslation( const SimdVector3 & translateVec );

    // Get the translation component of a 4x4 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 getTranslation( ) const;

    // Set column 0 of a 4x4 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & setCol0( const SimdVector4 & col0 );

    // Set column 1 of a 4x4 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & setCol1( const SimdVector4 & col1 );

    // Set column 2 of a 4x4 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & setCol2( const SimdVector4 & col2 );

    // Set column 3 of a 4x4 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & setCol3( const SimdVector4 & col3 );

    // Get column 0 of a 4x4 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 getCol0( ) const;

    // Get column 1 of a 4x4 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 getCol1( ) const;

    // Get column 2 of a 4x4 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 getCol2( ) const;

    // Get column 3 of a 4x4 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 getCol3( ) const;

    // Set the column of a 4x4 matrix referred to by the specified index
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & setCol( int col, const SimdVector4 & vec );

    // Set the row of a 4x4 matrix referred to by the specified index
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & setRow( int row, const SimdVector4 & vec );

    // Get the column of a 4x4 matrix referred to by the specified index
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 getCol( int col ) const;

    // Get the row of a 4x4 matrix referred to by the specified index
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 getRow( int row ) const;

    // Set the element of a 4x4 matrix referred to by column and row indices
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & setElem( int col, int row, float val );

    // Set the element of a 4x4 matrix referred to by column and row indices (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & setElem( int col, int row, const SimdFloat & val );

    // Get the element of a 4x4 matrix referred to by column and row indices
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getElem( int col, int row ) const;

    // Add two 4x4 matrices
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 operator +( const SimdMatrix4 & mat ) const;

    // Subtract a 4x4 matrix from another 4x4 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 operator -( const SimdMatrix4 & mat ) const;

    // Negate all elements of a 4x4 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 operator -( ) const;

    // Multiply a 4x4 matrix by a scalar
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 operator *( float scalar ) const;

    // Multiply a 4x4 matrix by a scalar (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 operator *( const SimdFloat &scalar ) const;

    // Perform compound assignment and addition with a 4x4 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & operator +=( const SimdMatrix4 & mat );

    // Perform compound assignment and subtraction by a 4x4 matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & operator -=( const SimdMatrix4 & mat );

    // Perform compound assignment and multiplication by a scalar
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & operator *=( float scalar );

    // Perform compound assignment and multiplication by a scalar (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdMatrix4 & operator *=( const SimdFloat &scalar );

    // Construct an identity 4x4 matrix
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 identity( );

    // Construct a 4x4 matrix to rotate around the x axis
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 rotationX( float radians );

    // Construct a 4x4 matrix to rotate around the y axis
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 rotationY( float radians );

    // Construct a 4x4 matrix to rotate around the z axis
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 rotationZ( float radians );

    // Construct a 4x4 matrix to rotate around the x axis (scalar data contained in vector data type)
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 rotationX( const SimdFloat &radians );

    // Construct a 4x4 matrix to rotate around the y axis (scalar data contained in vector data type)
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 rotationY( const SimdFloat &radians );

    // Construct a 4x4 matrix to rotate around the z axis (scalar data contained in vector data type)
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 rotationZ( const SimdFloat &radians );

    // Construct a 4x4 matrix to rotate around the x, y, and z axes
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 rotationZYX( const SimdVector3 &radiansXYZ );

    // Construct a 4x4 matrix to rotate around a unit-length 3-D vector
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 rotation( float radians, const SimdVector3 & unitVec );

    // Construct a 4x4 matrix to rotate around a unit-length 3-D vector (scalar data contained in vector data type)
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 rotation( const SimdFloat &radians, const SimdVector3 & unitVec );

    // Construct a rotation matrix from a unit-length quaternion
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 rotation( const SimdQuat & unitSimdQuat );

    // Construct a 4x4 matrix to perform scaling
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 scale( const SimdVector3 & scaleVec );

    // Construct a 4x4 matrix to perform translation
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 translation( const SimdVector3 & translateVec );

    // Construct viewing matrix based on eye position, position looked at, and up direction
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 lookAt( const SimdPoint3 & eyePos, const SimdPoint3 & lookAtPos, const SimdVector3 & upVec );

    // Construct a perspective projection matrix
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 perspective( float fovyRadians, float aspect, float zNear, float zFar );

    // Construct a perspective projection matrix based on frustum
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 frustum( float left, float right, float bottom, float top, float zNear, float zFar );

    // Construct an orthographic projection matrix
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 orthographic( float left, float right, float bottom, float top, float zNear, float zFar );

};

// Multiply a 4x4 matrix by a 4-D vector
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 mul(const SimdMatrix4& mat, const SimdVector4 & vec);

// Multiply a 4x4 matrix by a 3-D vector
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 mul(const SimdMatrix4& mat, const SimdVector3 & vec);

// Multiply a 4x4 matrix by a 3-D point
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 mul(const SimdMatrix4& mat, const SimdPoint3 & pnt);

// Multiply two 4x4 matrices
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 mul(const SimdMatrix4& mat0, const SimdMatrix4 & mat1);

// Multiply a 4x4 matrix by a 3x4 transformation matrix
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 mul(const SimdMatrix4& mat, const SimdTransform3 & tfrm);

// Multiply a 4x4 matrix by a scalar
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 operator *( float scalar, const SimdMatrix4 & mat );

// Multiply a 4x4 matrix by a scalar (scalar data contained in vector data type)
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 operator *( const SimdFloat &scalar, const SimdMatrix4 & mat );

// Append (post-multiply) a scale transformation to a 4x4 matrix
// NOTE: 
// Faster than creating and multiplying a scale transformation matrix.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 appendScale( const SimdMatrix4 & mat, const SimdVector3 & scaleVec );

// Prepend (pre-multiply) a scale transformation to a 4x4 matrix
// NOTE: 
// Faster than creating and multiplying a scale transformation matrix.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 prependScale( const SimdVector3 & scaleVec, const SimdMatrix4 & mat );

// Compute the absolute value of a 4x4 matrix per element
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 abs( const SimdMatrix4 & mat );

// Transpose of a 4x4 matrix
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 transpose( const SimdMatrix4 & mat );

// Compute the inverse of a 4x4 matrix
// NOTE: 
// Result is unpredictable when the determinant of mat is equal to or near 0.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 inverse( const SimdMatrix4 & mat );

// Compute the inverse of a 4x4 matrix, which is expected to be an affine matrix
// NOTE: 
// This can be used to achieve better performance than a general inverse when the specified 4x4 matrix meets the given restrictions.  The result is unpredictable when the determinant of mat is equal to or near 0.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 affineInverse( const SimdMatrix4 & mat );

// Compute the inverse of a 4x4 matrix, which is expected to be an affine matrix with an orthogonal upper-left 3x3 submatrix
// NOTE: 
// This can be used to achieve better performance than a general inverse when the specified 4x4 matrix meets the given restrictions.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 orthoInverse( const SimdMatrix4 & mat );

// Determinant of a 4x4 matrix
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdFloat determinant( const SimdMatrix4 & mat );

// Conditionally select between two 4x4 matrices
// NOTE: 
// This function uses a conditional select instruction to avoid a branch.
// However, the transfer of select1 to a VMX register may use more processing time than a branch.
// Use the SimdBool version for better performance.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 select( const SimdMatrix4 & mat0, const SimdMatrix4 & mat1, bool select1 );

// Conditionally select between two 4x4 matrices (scalar data contained in vector data type)
// NOTE: 
// This function uses a conditional select instruction to avoid a branch.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix4 select( const SimdMatrix4 & mat0, const SimdMatrix4 & mat1, const SimdBool &select1 );

#ifdef _VECTORMATH_DEBUG

// Print a 4x4 matrix
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
SIMD_VECTORMATH_FORCE_INLINE void print( const SimdMatrix4 & mat );

// Print a 4x4 matrix and an associated string identifier
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
SIMD_VECTORMATH_FORCE_INLINE void print( const SimdMatrix4 & mat, const char * name );

#endif

// A 3x4 transformation matrix in array-of-structures format
//
class SimdTransform3
{
public:
    SimdVector3 col0;
    SimdVector3 col1;
    SimdVector3 col2;
    SimdVector3 col3;

    // Default constructor; does no initialization
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdTransform3( ) { };

    // Copy a 3x4 transformation matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdTransform3( const SimdTransform3 & tfrm );

    // Construct a 3x4 transformation matrix containing the specified columns
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdTransform3( const SimdVector3 & col0, const SimdVector3 & col1, const SimdVector3 & col2, const SimdVector3 & col3 );

    // Construct a 3x4 transformation matrix from a 3x3 matrix and a 3-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdTransform3( const SimdMatrix3 & tfrm, const SimdVector3 & translateVec );

    // Construct a 3x4 transformation matrix from a unit-length quaternion and a 3-D vector
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdTransform3( const SimdQuat & unitSimdQuat, const SimdVector3 & translateVec );

    // Set all elements of a 3x4 transformation matrix to the same scalar value
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdTransform3( float scalar );

    // Set all elements of a 3x4 transformation matrix to the same scalar value (scalar data contained in vector data type)
    // 
    explicit SIMD_VECTORMATH_FORCE_INLINE SimdTransform3( const SimdFloat & scalar );

    // Assign one 3x4 transformation matrix to another
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & operator =( const SimdTransform3 & tfrm );

    // Set the upper-left 3x3 submatrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & setUpper3x3( const SimdMatrix3 & mat3 );

    // Get the upper-left 3x3 submatrix of a 3x4 transformation matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdMatrix3 getUpper3x3( ) const;

    // Set translation component
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & setTranslation( const SimdVector3 & translateVec );

    // Get the translation component of a 3x4 transformation matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 getTranslation( ) const;

    // Set column 0 of a 3x4 transformation matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & setCol0( const SimdVector3 & col0 );

    // Set column 1 of a 3x4 transformation matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & setCol1( const SimdVector3 & col1 );

    // Set column 2 of a 3x4 transformation matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & setCol2( const SimdVector3 & col2 );

    // Set column 3 of a 3x4 transformation matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & setCol3( const SimdVector3 & col3 );

    // Get column 0 of a 3x4 transformation matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 getCol0( ) const;

    // Get column 1 of a 3x4 transformation matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 getCol1( ) const;

    // Get column 2 of a 3x4 transformation matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 getCol2( ) const;

    // Get column 3 of a 3x4 transformation matrix
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 getCol3( ) const;

    // Set the column of a 3x4 transformation matrix referred to by the specified index
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & setCol( int col, const SimdVector3 & vec );

    // Set the row of a 3x4 transformation matrix referred to by the specified index
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & setRow( int row, const SimdVector4 & vec );

    // Get the column of a 3x4 transformation matrix referred to by the specified index
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 getCol( int col ) const;

    // Get the row of a 3x4 transformation matrix referred to by the specified index
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdVector4 getRow( int row ) const;

    // Set the element of a 3x4 transformation matrix referred to by column and row indices
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & setElem( int col, int row, float val );

    // Set the element of a 3x4 transformation matrix referred to by column and row indices (scalar data contained in vector data type)
    // 
    SIMD_VECTORMATH_FORCE_INLINE SimdTransform3 & setElem( int col, int row, const SimdFloat & val );

    // Get the element of a 3x4 transformation matrix referred to by column and row indices
    // 
    SIMD_VECTORMATH_FORCE_INLINE const SimdFloat getElem( int col, int row ) const;


    // Construct an identity 3x4 transformation matrix
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 identity( );

    // Construct a 3x4 transformation matrix to rotate around the x axis
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 rotationX( float radians );

    // Construct a 3x4 transformation matrix to rotate around the y axis
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 rotationY( float radians );

    // Construct a 3x4 transformation matrix to rotate around the z axis
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 rotationZ( float radians );

    // Construct a 3x4 transformation matrix to rotate around the x axis (scalar data contained in vector data type)
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 rotationX( const SimdFloat & radians );

    // Construct a 3x4 transformation matrix to rotate around the y axis (scalar data contained in vector data type)
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 rotationY( const SimdFloat & radians );

    // Construct a 3x4 transformation matrix to rotate around the z axis (scalar data contained in vector data type)
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 rotationZ( const SimdFloat & radians );

    // Construct a 3x4 transformation matrix to rotate around the x, y, and z axes
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 rotationZYX( const SimdVector3 & radiansXYZ );

    // Construct a 3x4 transformation matrix to rotate around a unit-length 3-D vector
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 rotation( float radians, const SimdVector3 & unitVec );

    // Construct a 3x4 transformation matrix to rotate around a unit-length 3-D vector (scalar data contained in vector data type)
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 rotation( const SimdFloat &radians, const SimdVector3 & unitVec );

    // Construct a rotation matrix from a unit-length quaternion
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 rotation( const SimdQuat & unitSimdQuat );

    // Construct a 3x4 transformation matrix to perform scaling
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 scale( const SimdVector3 & scaleVec );

    // Construct a 3x4 transformation matrix to perform translation
    // 
    static SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 translation( const SimdVector3 & translateVec );

};

// Multiply a 3x4 transformation matrix by a 3-D vector
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdVector3 mul(const SimdTransform3 & tfrm, const SimdVector3 & vec);

// Multiply a 3x4 transformation matrix by a 3-D point
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdPoint3 mul(const SimdTransform3 & tfrm, const SimdPoint3 & pnt);

// Multiply two 3x4 transformation matrices
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 mul(const SimdTransform3 & tfrm0, const SimdTransform3 & tfrm1);

// Append (post-multiply) a scale transformation to a 3x4 transformation matrix
// NOTE: 
// Faster than creating and multiplying a scale transformation matrix.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 appendScale( const SimdTransform3 & tfrm, const SimdVector3 & scaleVec );

// Prepend (pre-multiply) a scale transformation to a 3x4 transformation matrix
// NOTE: 
// Faster than creating and multiplying a scale transformation matrix.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 prependScale( const SimdVector3 & scaleVec, const SimdTransform3 & tfrm );

// Compute the absolute value of a 3x4 transformation matrix per element
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 abs( const SimdTransform3 & tfrm );

// Inverse of a 3x4 transformation matrix
// NOTE: 
// Result is unpredictable when the determinant of the left 3x3 submatrix is equal to or near 0.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 inverse( const SimdTransform3 & tfrm );

// Compute the inverse of a 3x4 transformation matrix, expected to have an orthogonal upper-left 3x3 submatrix
// NOTE: 
// This can be used to achieve better performance than a general inverse when the specified 3x4 transformation matrix meets the given restrictions.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 orthoInverse( const SimdTransform3 & tfrm );

// Conditionally select between two 3x4 transformation matrices
// NOTE: 
// This function uses a conditional select instruction to avoid a branch.
// However, the transfer of select1 to a VMX register may use more processing time than a branch.
// Use the SimdBool version for better performance.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 select( const SimdTransform3 & tfrm0, const SimdTransform3 & tfrm1, bool select1 );

// Conditionally select between two 3x4 transformation matrices (scalar data contained in vector data type)
// NOTE: 
// This function uses a conditional select instruction to avoid a branch.
// 
SIMD_VECTORMATH_FORCE_INLINE const SimdTransform3 select( const SimdTransform3 & tfrm0, const SimdTransform3 & tfrm1, const SimdBool &select1 );

#ifdef _VECTORMATH_DEBUG

// Print a 3x4 transformation matrix
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
SIMD_VECTORMATH_FORCE_INLINE void print( const SimdTransform3 & tfrm );

// Print a 3x4 transformation matrix and an associated string identifier
// NOTE: 
// Function is only defined when _VECTORMATH_DEBUG is defined.
// 
SIMD_VECTORMATH_FORCE_INLINE void print( const SimdTransform3 & tfrm, const char * name );

#endif

} // namespace FmVectormath

#pragma warning(default : 4201)

#include "simd_vec_aos.h"
#include "simd_quat_aos.h"
#include "simd_mat_aos.h"

#endif
