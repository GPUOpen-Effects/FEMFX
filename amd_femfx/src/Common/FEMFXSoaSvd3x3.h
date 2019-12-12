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
// Structure-of-arrays SIMD implementation of 3x3 SVD
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXVectorMath.h"

namespace AMD
{
    // Reference: McAdams et al., "Computing the Singular Value Decomposition of 3 x 3 matrices with minimal branching and elementary floating point operations"

    // Approximation of sin,cos values needed for Givens rotation used in Jacobi iteration
    template<class T>
    void FmApproxGivens(typename T::SoaFloat* s, typename T::SoaFloat* c, typename T::SoaFloat a00, typename T::SoaFloat a01, typename T::SoaFloat a11)
    {
        typename T::SoaFloat a00m11 = a00 - a11;
        typename T::SoaFloat a00m11_2 = a00m11*a00m11;
        typename T::SoaFloat a01_2 = a01*a01;

        typename T::SoaBool useOmega = (a01_2 < a00m11_2);
        typename T::SoaFloat omega = 1.0f / sqrtf(a01_2 + a00m11_2);

        const typename T::SoaFloat sqrtOneHalf = 0.70710678118654752440084436210485f; // sqrtf(0.5f);

        *s = select(sqrtOneHalf, omega*a01, useOmega);
        *c = select(sqrtOneHalf, omega*a00m11, useOmega);
    }

    // Compute eigenvectors and values for symmetric matrix, using Jacobi iteration with fixed cycle of (row, col) pairs
    template<class T>
    void FmEigenSymm3x3CyclicJacobi(typename T::SoaVector3* eigenvals, typename T::SoaMatrix3* eigenvectors, const typename T::SoaMatrix3& symmMat)
    {
        typename T::SoaFloat s00 = symmMat.col0.x;
        typename T::SoaFloat s01 = symmMat.col1.x;
        typename T::SoaFloat s02 = symmMat.col2.x;
        typename T::SoaFloat s11 = symmMat.col1.y;
        typename T::SoaFloat s12 = symmMat.col2.y;
        typename T::SoaFloat s22 = symmMat.col2.z;

        const int numIterations = 4;

        typename T::SoaMatrix3 V = typename T::SoaMatrix3::identity();

        for (int i = 0; i < numIterations; i++)
        {
            // (p, q) = (0, 1)
            typename T::SoaFloat s, c;
            FmApproxGivens<T>(&s, &c, s00, s01, s11);
            typename T::SoaMatrix3 GivensMat = FmInitMatrix3<T>(FmInitVector3<T>(c, s, 0.0f), FmInitVector3<T>(-s, c, 0.0f), FmInitVector3<T>(0.0f, 0.0f, 1.0f));
            V = mul(V, GivensMat);

            typename T::SoaFloat s00_next = s*s*s11 + s*c*s01 + c*s*s01 + c*c*s00;
            typename T::SoaFloat s01_next = s*c*s11 - s*s*s01 + c*c*s01 - c*s*s00;
            typename T::SoaFloat s02_next = s*s12 + c*s02;
            typename T::SoaFloat s11_next = c*c*s11 - c*s*s01 - s*c*s01 + s*s*s00;
            typename T::SoaFloat s12_next = c*s12 - s*s02;
            typename T::SoaFloat s22_next; // = s22;

            s00 = s00_next;
            s01 = s01_next;
            s02 = s02_next;
            s11 = s11_next;
            s12 = s12_next;
            //s22 = s22_next;

            // (p, q) = (0, 2)
            FmApproxGivens<T>(&s, &c, s00, s02, s22);
            GivensMat = FmInitMatrix3<T>(FmInitVector3<T>(c, 0.0f, s), FmInitVector3<T>(0.0f, 1.0f, 0.0f), FmInitVector3<T>(-s, 0.0f, c));
            V = mul(V, GivensMat);

            s00_next = s*s*s22 + s*c*s02 + c*s*s02 + c*c*s00;
            s01_next = s*s12 + c*s01;
            s02_next = s*c*s22 - s*s*s02 + c*c*s02 - c*s*s00;
            //s11_next = s11;
            s12_next = c*s12 - s*s01;
            s22_next = c*c*s22 - c*s*s02 - s*c*s02 + s*s*s00;

            s00 = s00_next;
            s01 = s01_next;
            s02 = s02_next;
            //s11 = s11_next;
            s12 = s12_next;
            s22 = s22_next;

            // (p, q) = (1, 2)
            FmApproxGivens<T>(&s, &c, s11, s12, s22);
            GivensMat = FmInitMatrix3<T>(FmInitVector3<T>(1.0f, 0.0f, 0.0f), FmInitVector3<T>(0.0f, c, s), FmInitVector3<T>(0.0f, -s, c));
            V = mul(V, GivensMat);

            //s00_next = s00;
            s01_next = s*s02 + c*s01;
            s02_next = c*s02 - s*s01;
            s11_next = s*s*s22 + s*c*s12 + c*s*s12 + c*c*s11;
            s12_next = s*c*s22 - s*s*s12 + c*c*s12 - c*s*s11;
            s22_next = c*c*s22 - c*s*s12 - s*c*s12 + s*s*s11;

            //s00 = s00_next;
            s01 = s01_next;
            s02 = s02_next;
            s11 = s11_next;
            s12 = s12_next;
            s22 = s22_next;
        }

        // Sort eigenvalues in order of decreasing magnitude, and swap eigenvectors to match.
        // Signs flipped during swaps to preserve rotation.
        typename T::SoaVector3 eigenvec0 = V.col0;
        typename T::SoaVector3 eigenvec1 = V.col1;
        typename T::SoaVector3 eigenvec2 = V.col2;
        typename T::SoaVector3 eigenvectemp;
        typename T::SoaFloat eigenval0 = s00;
        typename T::SoaFloat eigenval1 = s11;
        typename T::SoaFloat eigenval2 = s22;
        typename T::SoaFloat eigenvaltemp;
        typename T::SoaFloat abseigenval0 = fabsf(eigenval0);
        typename T::SoaFloat abseigenval1 = fabsf(eigenval1);
        typename T::SoaFloat abseigenval2 = fabsf(eigenval2);
        typename T::SoaFloat abseigenvaltemp;

        typename T::SoaBool swap = (abseigenval0 < abseigenval1);
        eigenvectemp = -eigenvec0;
        eigenvec0 = select(eigenvec0, eigenvec1, swap);
        eigenvec1 = select(eigenvec1, eigenvectemp, swap);
        eigenvaltemp = -eigenval0;
        eigenval0 = select(eigenval0, eigenval1, swap);
        eigenval1 = select(eigenval1, eigenvaltemp, swap);
        abseigenvaltemp = abseigenval0;
        abseigenval0 = select(abseigenval0, abseigenval1, swap);
        abseigenval1 = select(abseigenval1, abseigenvaltemp, swap);

        swap = (abseigenval0 < abseigenval2);
        eigenvectemp = -eigenvec0;
        eigenvec0 = select(eigenvec0, eigenvec2, swap);
        eigenvec2 = select(eigenvec2, eigenvectemp, swap);
        eigenvaltemp = -eigenval0;
        eigenval0 = select(eigenval0, eigenval2, swap);
        eigenval2 = select(eigenval2, eigenvaltemp, swap);
        abseigenvaltemp = abseigenval0;
        //abseigenval0 = select(abseigenval0, abseigenval2, swap);
        abseigenval2 = select(abseigenval2, abseigenvaltemp, swap);

        swap = (abseigenval1 < abseigenval2);
        eigenvectemp = -eigenvec1;
        eigenvec1 = select(eigenvec1, eigenvec2, swap);
        eigenvec2 = select(eigenvec2, eigenvectemp, swap);
        eigenvaltemp = -eigenval1;
        eigenval1 = select(eigenval1, eigenval2, swap);
        eigenval2 = select(eigenval2, eigenvaltemp, swap);
        //abseigenvaltemp = abseigenval1;
        //abseigenval1 = select(abseigenval1, abseigenval2, swap);
        //abseigenval2 = select(abseigenval2, abseigenvaltemp, swap);

        *eigenvectors = FmInitMatrix3<T>(eigenvec0, eigenvec1, eigenvec2);
        *eigenvals = FmInitVector3<T>(eigenval0, eigenval1, eigenval2);
    }

    // Givens rotation used in QR decomposition
    template<class T>
    void FmQRGivens(typename T::SoaFloat& s, typename T::SoaFloat& c, typename T::SoaFloat apq, typename T::SoaFloat aqq, typename T::SoaFloat tolerance)
    {
        typename T::SoaFloat sum = apq*apq + aqq*aqq;
        typename T::SoaFloat denomInv = 1.0f / sqrtf(sum);
        s = select(apq * denomInv, 0.0f, (sum < tolerance));
        c = select(aqq * denomInv, 1.0f, (sum < tolerance));
    }

    // SVD(mat) = U * Sigma * V^T
    // Reference: McAdams et al., "Computing the Singular Value Decomposition of 3 x 3 matrices with minimal branching and elementary floating point operations"
    template<class T>
    void FmSvd3x3(typename T::SoaMatrix3* U, typename T::SoaVector3* sigma, typename T::SoaMatrix3* V, const typename T::SoaMatrix3& mat, typename T::SoaFloat givensTolerance = typename T::SoaFloat(FLT_EPSILON))
    {
        // Compute V^T * Sigma^2 * V as Eigendecomposition of Mat^T * Mat
        typename T::SoaMatrix3 symmMat = mul(transpose(mat), mat);

        typename T::SoaVector3 eigenvals;
        typename T::SoaMatrix3 eigenvecs;
        FmEigenSymm3x3CyclicJacobi<T>(&eigenvals, &eigenvecs, symmMat);

        // B = U * Sigma = Mat * V
        typename T::SoaMatrix3 B = mul(mat, eigenvecs);

        // Decompose into U and Sigma with Givens rotations
        typename T::SoaMatrix3 Q, GivensMat;
        typename T::SoaFloat s, c;

        typename T::SoaFloat B_00, B_10, B_20, B_21, B_11, B_22;

        B_10 = B.getElem(0, 1);
        B_00 = B.getElem(0, 0);
        FmQRGivens<T>(s, c, B_10, B_00, givensTolerance);
        GivensMat = FmInitMatrix3<T>(FmInitVector3<T>(c, s, 0.0f), FmInitVector3<T>(-s, c, 0.0f), FmInitVector3<T>(0.0f, 0.0f, 1.0f));

        Q = GivensMat;
        B = mul(transpose(GivensMat), B);

        B_20 = B.getElem(0, 2);
        B_00 = B.getElem(0, 0);
        FmQRGivens<T>(s, c, B_20, B_00, givensTolerance);
        GivensMat = FmInitMatrix3<T>(FmInitVector3<T>(c, 0.0f, s), FmInitVector3<T>(0.0f, 1.0f, 0.0f), FmInitVector3<T>(-s, 0.0f, c));

        Q = mul(Q, GivensMat);
        B = mul(transpose(GivensMat), B);

        B_21 = B.getElem(1, 2);
        B_11 = B.getElem(1, 1);
        FmQRGivens<T>(s, c, B_21, B_11, givensTolerance);
        GivensMat = FmInitMatrix3<T>(FmInitVector3<T>(1.0f, 0.0f, 0.0f), FmInitVector3<T>(0.0f, c, s), FmInitVector3<T>(0.0f, -s, c));

        Q = mul(Q, GivensMat);
        B = mul(transpose(GivensMat), B);

        B_00 = B.getElem(0, 0);
        B_11 = B.getElem(1, 1);
        B_22 = B.getElem(2, 2);
        *U = Q;
        sigma->x = B_00;
        sigma->y = B_11;
        sigma->z = B_22;
        *V = eigenvecs;
    }

    template<class T>
    typename T::SoaMatrix3 FmPseudoInverse(const typename T::SoaMatrix3& mat)
    {
        typename T::SoaMatrix3 U, V;
        typename T::SoaVector3 sigma;
        FmSvd3x3(&U, &sigma, &V, mat);

        // Tolerance from https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_pseudoinverse
        typename T::SoaVector3 absSigma = abs(sigma);

        typename T::SoaFloat tol = FLT_EPSILON * 3.0f * FmMaxFloat(absSigma.x, FmMaxFloat(absSigma.y, absSigma.z));

        sigma.x = select(0.0f, 1.0f / sigma.x, absSigma.x > tol);
        sigma.y = select(0.0f, 1.0f / sigma.y, absSigma.y > tol);
        sigma.z = select(0.0f, 1.0f / sigma.z, absSigma.z > tol);

        return mul(V, mul(typename T::SoaMatrix3::scale(sigma), transpose(U)));
    }
}
