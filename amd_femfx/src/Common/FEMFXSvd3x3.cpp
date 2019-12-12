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

#include "FEMFXCommonInternal.h"
#include "FEMFXTypes.h"
#include "FEMFXSvd3x3.h"

namespace AMD
{
    // Reference: McAdams et al., "Computing the Singular Value Decomposition of 3 x 3 matrices with minimal branching and elementary floating point operations"

    // Approximation of sin,cos values needed for Givens rotation used in Jacobi iteration
    void FmApproxGivens(float* res_s, float* res_c, float a00, float a01, float a11)
    {
        float a00m11 = a00 - a11;
        float a00m11_2 = a00m11*a00m11;
        float a01_2 = a01*a01;

        bool useOmega = (a01_2 < a00m11_2);
        float omega = 1.0f / sqrtf(a01_2 + a00m11_2);

        const float sqrtOneHalf = 0.70710678118654752440084436210485f; // sqrtf(0.5f);

        float s = sqrtOneHalf;
        float c = sqrtOneHalf;
        if (useOmega)
        {
            s = omega * a01;
            c = omega * a00m11;
        }

        *res_s = s;
        *res_c = c;
    }

    // Compute eigenvectors and values for symmetric matrix, using Jacobi iteration with fixed cycle of (row, col) pairs
    void FmEigenSymm3x3CyclicJacobi(FmVector3* eigenvals, FmMatrix3* eigenvectors, const FmMatrix3& symmMat)
    {
        float s00 = symmMat.col0.x;
        float s01 = symmMat.col1.x;
        float s02 = symmMat.col2.x;
        float s11 = symmMat.col1.y;
        float s12 = symmMat.col2.y;
        float s22 = symmMat.col2.z;

        const int numIterations = 4;

        FmMatrix3 V = FmMatrix3::identity();

        for (int i = 0; i < numIterations; i++)
        {
            // (p, q) = (0, 1)
            float s, c;
            FmApproxGivens(&s, &c, s00, s01, s11);
            FmMatrix3 GivensMat = FmInitMatrix3(FmInitVector3(c, s, 0.0f), FmInitVector3(-s, c, 0.0f), FmInitVector3(0.0f, 0.0f, 1.0f));
            V = mul(V, GivensMat);

            float s00_next = s*s*s11 + s*c*s01 + c*s*s01 + c*c*s00;
            float s01_next = s*c*s11 - s*s*s01 + c*c*s01 - c*s*s00;
            float s02_next = s*s12 + c*s02;
            float s11_next = c*c*s11 - c*s*s01 - s*c*s01 + s*s*s00;
            float s12_next = c*s12 - s*s02;
            float s22_next; // = s22;

            s00 = s00_next;
            s01 = s01_next;
            s02 = s02_next;
            s11 = s11_next;
            s12 = s12_next;
            //s22 = s22_next;

            // (p, q) = (0, 2)
            FmApproxGivens(&s, &c, s00, s02, s22);
            GivensMat = FmInitMatrix3(FmInitVector3(c, 0.0f, s), FmInitVector3(0.0f, 1.0f, 0.0f), FmInitVector3(-s, 0.0f, c));
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
            FmApproxGivens(&s, &c, s11, s12, s22);
            GivensMat = FmInitMatrix3(FmInitVector3(1.0f, 0.0f, 0.0f), FmInitVector3(0.0f, c, s), FmInitVector3(0.0f, -s, c));
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
        FmVector3 eigenvec0 = V.col0;
        FmVector3 eigenvec1 = V.col1;
        FmVector3 eigenvec2 = V.col2;
        FmVector3 eigenvectemp;
        float eigenval0 = s00;
        float eigenval1 = s11;
        float eigenval2 = s22;
        float eigenvaltemp;
        float abseigenval0 = fabsf(eigenval0);
        float abseigenval1 = fabsf(eigenval1);
        float abseigenval2 = fabsf(eigenval2);
        float abseigenvaltemp;

        bool swap = (abseigenval0 < abseigenval1);
        eigenvectemp = -eigenvec0;
        eigenvec0 = swap ? eigenvec1 : eigenvec0;
        eigenvec1 = swap ? eigenvectemp : eigenvec1;
        eigenvaltemp = -eigenval0;
        eigenval0 = swap ? eigenval1 : eigenval0;
        eigenval1 = swap ? eigenvaltemp : eigenval1;
        abseigenvaltemp = abseigenval0;
        abseigenval0 = swap ? abseigenval1 : abseigenval0;
        abseigenval1 = swap ? abseigenvaltemp : abseigenval1;

        swap = (abseigenval0 < abseigenval2);
        eigenvectemp = -eigenvec0;
        eigenvec0 = swap ? eigenvec2 : eigenvec0;
        eigenvec2 = swap ? eigenvectemp : eigenvec2;
        eigenvaltemp = -eigenval0;
        eigenval0 = swap ? eigenval2 : eigenval0;
        eigenval2 = swap ? eigenvaltemp : eigenval2;
        abseigenvaltemp = abseigenval0;
        //abseigenval0 = swap ? abseigenval2 : abseigenval0;
        abseigenval2 = swap ? abseigenvaltemp : abseigenval2;

        swap = (abseigenval1 < abseigenval2);
        eigenvectemp = -eigenvec1;
        eigenvec1 = swap ? eigenvec2 : eigenvec1;
        eigenvec2 = swap ? eigenvectemp : eigenvec2;
        eigenvaltemp = -eigenval1;
        eigenval1 = swap ? eigenval2 : eigenval1;
        eigenval2 = swap ? eigenvaltemp : eigenval2;
        //abseigenvaltemp = abseigenval1;
        //abseigenval1 = swap ? abseigenval2 : abseigenval1;
        //abseigenval2 = swap ? abseigenvaltemp : abseigenval2;

        *eigenvectors = FmInitMatrix3(eigenvec0, eigenvec1, eigenvec2);
        *eigenvals = FmInitVector3(eigenval0, eigenval1, eigenval2);
    }

    // Givens rotation used in QR decomposition
    void FmQRGivens(float& s, float& c, float apq, float aqq, float tolerance)
    {
        float sum = apq*apq + aqq*aqq;
        float denomInv = 1.0f / sqrtf(sum);
        s = (sum < tolerance) ? 0.0f : apq * denomInv;
        c = (sum < tolerance) ? 1.0f : aqq * denomInv;
    }

    // SVD(mat) = U * Sigma * V^T
    // Reference: McAdams et al., "Computing the Singular Value Decomposition of 3 x 3 matrices with minimal branching and elementary floating point operations"
    void FmSvd3x3(FmMatrix3* U, FmVector3* sigma, FmMatrix3* V, const FmMatrix3& mat, float givensTolerance /* = FLT_EPSILON */)
    {
        // Compute V^T * Sigma^2 * V as Eigendecomposition of Mat^T * Mat
        FmMatrix3 symmMat = mul(transpose(mat), mat);

        FmVector3 eigenvals;
        FmMatrix3 eigenvecs;
        FmEigenSymm3x3CyclicJacobi(&eigenvals, &eigenvecs, symmMat);

        // B = U * Sigma = Mat * V
        FmMatrix3 B = mul(mat, eigenvecs);

        // Decompose into U and Sigma with Givens rotations
        FmMatrix3 Q, GivensMat;
        float s, c;

        float B_00, B_10, B_20, B_21, B_11, B_22;

        B_10 = B.getElem(0, 1);
        B_00 = B.getElem(0, 0);
        FmQRGivens(s, c, B_10, B_00, givensTolerance);
        GivensMat = FmInitMatrix3(FmInitVector3(c, s, 0.0f), FmInitVector3(-s, c, 0.0f), FmInitVector3(0.0f, 0.0f, 1.0f));

        Q = GivensMat;
        B = mul(transpose(GivensMat), B);

        B_20 = B.getElem(0, 2);
        B_00 = B.getElem(0, 0);
        FmQRGivens(s, c, B_20, B_00, givensTolerance);
        GivensMat = FmInitMatrix3(FmInitVector3(c, 0.0f, s), FmInitVector3(0.0f, 1.0f, 0.0f), FmInitVector3(-s, 0.0f, c));

        Q = mul(Q, GivensMat);
        B = mul(transpose(GivensMat), B);

        B_21 = B.getElem(1, 2);
        B_11 = B.getElem(1, 1);
        FmQRGivens(s, c, B_21, B_11, givensTolerance);
        GivensMat = FmInitMatrix3(FmInitVector3(1.0f, 0.0f, 0.0f), FmInitVector3(0.0f, c, s), FmInitVector3(0.0f, -s, c));

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

    FmMatrix3 FmPseudoInverse(const FmMatrix3& mat)
    {
        FmMatrix3 U, V;
        FmVector3 sigma;
        FmSvd3x3(&U, &sigma, &V, mat);

        // Tolerance from https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_pseudoinverse
        FmVector3 absSigma = abs(sigma);

        float tol = FLT_EPSILON * 3.0f * FmMaxFloat(absSigma.x, FmMaxFloat(absSigma.y, absSigma.z));

        sigma.x = absSigma.x > tol ? 1.0f / sigma.x : 0.0f;
        sigma.y = absSigma.y > tol ? 1.0f / sigma.y : 0.0f;
        sigma.z = absSigma.z > tol ? 1.0f / sigma.z : 0.0f;

        return mul(V, mul(FmMatrix3::scale(sigma), transpose(U)));
    }

}
