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

//---------------------------------------------------------------------------------------
// Approximate 3x3 SVD
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXVectorMath.h"

namespace AMD
{
    // Compute eigenvectors and values for symmetric matrix, using Jacobi iteration with fixed cycle of (row, col) pairs
    void FmEigenSymm3x3CyclicJacobi(FmVector3* eigenvals, FmMatrix3* eigenvectors, const FmMatrix3& symmMat);

    // SVD(mat) = U * Sigma * V^T
    // Reference: McAdams et al., "Computing the Singular Value Decomposition of 3 x 3 matrices with minimal branching and elementary floating point operations"
    void FmSvd3x3(FmMatrix3* U, FmVector3* sigma, FmMatrix3* V, const FmMatrix3& mat, float givensTolerance = FLT_EPSILON);

    // 3x3 matrix pseudo-inverse based on SVD
    FmMatrix3 FmPseudoInverse(const FmMatrix3& mat);
}