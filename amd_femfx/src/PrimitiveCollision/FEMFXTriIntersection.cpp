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
// Functions to compute the intersection of a pair of triangles
//---------------------------------------------------------------------------------------

#include "FEMFXTypes.h"
#include "FEMFXTriIntersection.h"

namespace AMD
{

    static FM_FORCE_INLINE void FmIntersectionX12TetsAddVert(
        uint& numVerts,
        FmVector3* verts,
        int X12Vals[6][4],
        uint vertIds[6][4],
        uint edgeA, uint faceB,
        const FmVector3& eA0, const FmVector3& eA1,
        const FmVector3& fB0, const FmVector3& fB1, const FmVector3& fB2)
    {
        FmVector3 intersectionPoint;
        int X12Val = FmIntersectionX12(&intersectionPoint, eA0, eA1, fB0, fB1, fB2);

        X12Vals[edgeA][faceB] = X12Val;
        vertIds[edgeA][faceB] = numVerts;
        verts[numVerts] = intersectionPoint;
        numVerts = (X12Val != 0) ? numVerts + 1 : numVerts;
    }

    static FM_FORCE_INLINE void FmIntersectionX12TriTetAddVert(
        uint& numVerts,
        FmVector3* verts,
        int X12Vals[3][4],
        uint vertIds[3][4],
        uint edgeA, uint faceB,
        const FmVector3& eA0, const FmVector3& eA1,
        const FmVector3& fB0, const FmVector3& fB1, const FmVector3& fB2)
    {
        FmVector3 intersectionPoint;
        int X12Val = FmIntersectionX12(&intersectionPoint, eA0, eA1, fB0, fB1, fB2);

        X12Vals[edgeA][faceB] = X12Val;
        vertIds[edgeA][faceB] = numVerts;
        verts[numVerts] = intersectionPoint;
        numVerts = (X12Val != 0) ? numVerts + 1 : numVerts;
    }

    static FM_FORCE_INLINE void FmIntersectionX12TetTriAddVert(
        uint& numVerts,
        FmVector3* verts,
        int X12Vals[6],
        uint vertIds[6],
        uint edgeA,
        const FmVector3& eA0, const FmVector3& eA1,
        const FmVector3& fB0, const FmVector3& fB1, const FmVector3& fB2)
    {
        FmVector3 intersectionPoint;
        int X12Val = FmIntersectionX12(&intersectionPoint, eA0, eA1, fB0, fB1, fB2);

        X12Vals[edgeA] = X12Val;
        vertIds[edgeA] = numVerts;
        verts[numVerts] = intersectionPoint;
        numVerts = (X12Val != 0) ? numVerts + 1 : numVerts;
    }

    static FM_FORCE_INLINE void FmIntersectionX21TetsAddVert(
        uint& numVerts,
        FmVector3* verts,
        int X21Vals[4][6],
        uint vertIds[4][6],
        uint faceA, uint edgeB,
        const FmVector3& fA0, const FmVector3& fA1, const FmVector3& fA2,
        const FmVector3& eB0, const FmVector3& eB1)
    {
        FmVector3 intersectionPoint;
        int X21Val = FmIntersectionX21(&intersectionPoint, fA0, fA1, fA2, eB0, eB1);

        X21Vals[faceA][edgeB] = X21Val;
        vertIds[faceA][edgeB] = numVerts;
        verts[numVerts] = intersectionPoint;
        numVerts = (X21Val != 0) ? numVerts + 1 : numVerts;
    }

    static FM_FORCE_INLINE void FmIntersectionX21TriTetAddVert(
        uint& numVerts,
        FmVector3* verts,
        int X21Vals[6],
        uint vertIds[6],
        uint edgeB,
        const FmVector3& fA0, const FmVector3& fA1, const FmVector3& fA2,
        const FmVector3& eB0, const FmVector3& eB1)
    {
        FmVector3 intersectionPoint;
        int X21Val = FmIntersectionX21(&intersectionPoint, fA0, fA1, fA2, eB0, eB1);

        X21Vals[edgeB] = X21Val;
        vertIds[edgeB] = numVerts;
        verts[numVerts] = intersectionPoint;
        numVerts = (X21Val != 0) ? numVerts + 1 : numVerts;
    }

    static FM_FORCE_INLINE void FmIntersectionX21TetTriAddVert(
        uint& numVerts,
        FmVector3* verts,
        int X21Vals[4][3],
        uint vertIds[4][3],
        uint faceA, uint edgeB,
        const FmVector3& fA0, const FmVector3& fA1, const FmVector3& fA2,
        const FmVector3& eB0, const FmVector3& eB1)
    {
        FmVector3 intersectionPoint;
        int X21Val = FmIntersectionX21(&intersectionPoint, fA0, fA1, fA2, eB0, eB1);

        X21Vals[faceA][edgeB] = X21Val;
        vertIds[faceA][edgeB] = numVerts;
        verts[numVerts] = intersectionPoint;
        numVerts = (X21Val != 0) ? numVerts + 1 : numVerts;
    }

    static FM_FORCE_INLINE void FmIntersectionX03AddVert(
        uint& numVerts,
        FmVector3* verts,
        int* X03Vals,
        uint* vertIds,
        uint vertA,
        const FmVector3& vA,
        const FmVector3& tB0, const FmVector3& tB1, const FmVector3& tB2, const FmVector3& tB3)
    {
        int X03Val = FmIntersectionX03(vA, tB0, tB1, tB2, tB3);

        X03Vals[vertA] = X03Val;
        vertIds[vertA] = numVerts;
        verts[numVerts] = vA;
        numVerts = (X03Val != 0) ? numVerts + 1 : numVerts;
    }

    static FM_FORCE_INLINE void FmIntersectionX30AddVert(
        uint& numVerts,
        FmVector3* verts,
        int* X30Vals,
        uint* vertIds,
        uint vertB,
        const FmVector3& tA0, const FmVector3& tA1, const FmVector3& tA2, const FmVector3& tA3,
        const FmVector3& vB)
    {
        int X30Val = FmIntersectionX30(tA0, tA1, tA2, tA3, vB);

        X30Vals[vertB] = X30Val;
        vertIds[vertB] = numVerts;
        verts[numVerts] = vB;
        numVerts = (X30Val != 0) ? numVerts + 1 : numVerts;
    }

    // Intersection function for edge A and edge B intersecting in XY plane
    int FmIntersectionX11(
        FmVector3* intersectionPointA, FmVector3* intersectionPointB,
        const FmVector3& eA0, const FmVector3& eA1, const FmVector3& eB0, const FmVector3& eB1)
    {
        FmVector3 intersectionPointBwithA0, intersectionPointBwithA1, intersectionPointAwithB0, intersectionPointAwithB1;
        int X01ValBwithA0 = FmIntersectionX01(&intersectionPointBwithA0, eA0, eB0, eB1);
        int X01ValBwithA1 = FmIntersectionX01(&intersectionPointBwithA1, eA1, eB0, eB1);
        int X10ValAwithB0 = FmIntersectionX10(&intersectionPointAwithB0, eA0, eA1, eB0);
        int X10ValAwithB1 = FmIntersectionX10(&intersectionPointAwithB1, eA0, eA1, eB1);

        int S01ValBonA0 = FmShadowFunction(X01ValBwithA0, eA0.y, intersectionPointBwithA0.y);
        int S01ValBonA1 = FmShadowFunction(X01ValBwithA1, eA1.y, intersectionPointBwithA1.y);
        int S10ValB0onA = FmShadowFunction(X10ValAwithB0, intersectionPointAwithB0.y, eB0.y);
        int S10ValB1onA = FmShadowFunction(X10ValAwithB1, intersectionPointAwithB1.y, eB1.y);

        int intersectionVal = S01ValBonA1 - S01ValBonA0 + S10ValB1onA - S10ValB0onA;

        FmVector3 xBp = intersectionPointBwithA0;
        FmVector3 xAp = eA0;
        FmVector3 xBm = intersectionPointBwithA0;
        FmVector3 xAm = eA0;

        // xBp and xAp are intersection points where xBp shadows xAp
        xBp = (S01ValBonA1 != 0) ? intersectionPointBwithA1 : xBp;
        xAp = (S01ValBonA1 != 0) ? eA1 : xAp;
        xBp = (S10ValB0onA != 0) ? eB0 : xBp;
        xAp = (S10ValB0onA != 0) ? intersectionPointAwithB0 : xAp;
        xBp = (S10ValB1onA != 0) ? eB1 : xBp;
        xAp = (S10ValB1onA != 0) ? intersectionPointAwithB1 : xAp;

        // xBm and xAm are intersection points where xBm does not shadow xAm
        xBm = (X01ValBwithA1 != 0 && S01ValBonA1 == 0) ? intersectionPointBwithA1 : xBm;
        xAm = (X01ValBwithA1 != 0 && S01ValBonA1 == 0) ? eA1 : xAm;
        xBm = (X10ValAwithB0 != 0 && S10ValB0onA == 0) ? eB0 : xBm;
        xAm = (X10ValAwithB0 != 0 && S10ValB0onA == 0) ? intersectionPointAwithB0 : xAm;
        xBm = (X10ValAwithB1 != 0 && S10ValB1onA == 0) ? eB1 : xBm;
        xAm = (X10ValAwithB1 != 0 && S10ValB1onA == 0) ? intersectionPointAwithB1 : xAm;

        float deltam = xBm.y - xAm.y;
        float deltap = xBp.y - xAp.y;
        float t = deltap / (deltap - deltam);

        FmVector3 vA = xAp - t * (xAp - xAm);
        //FmVector3 vB = xBp - t * (xBp - xBm);
        FmVector3 vB;
        vB.x = vA.x;
        vB.y = vA.y;
        vB.z = xBp.z - t * (xBp.z - xBm.z);
        *intersectionPointA = vA;
        *intersectionPointB = vB;

        return intersectionVal;
    }

    // Intersection function for vertex A and triangle B intersecting in XY plane
    int FmIntersectionX02(FmVector3* intersectionPointB, const FmVector3& vA, const FmVector3& fB0, const FmVector3& fB1, const FmVector3& fB2)
    {
        FmVector3 intersectionPointBe0withA, intersectionPointBe1withA, intersectionPointBe2withA;
        int X01ValBe0withA = FmIntersectionX01(&intersectionPointBe0withA, vA, fB0, fB1);
        int X01ValBe1withA = FmIntersectionX01(&intersectionPointBe1withA, vA, fB1, fB2);
        int X01ValBe2withA = FmIntersectionX01(&intersectionPointBe2withA, vA, fB2, fB0);

        int S01ValB0onA = FmShadowFunction(X01ValBe0withA, vA.y, intersectionPointBe0withA.y);
        int S01ValB1onA = FmShadowFunction(X01ValBe1withA, vA.y, intersectionPointBe1withA.y);
        int S01ValB2onA = FmShadowFunction(X01ValBe2withA, vA.y, intersectionPointBe2withA.y);

        int intersectionVal = -(S01ValB0onA + S01ValB1onA + S01ValB2onA);

        FmVector3 xBp = intersectionPointBe0withA;
        FmVector3 xBm = intersectionPointBe0withA;

        // xBp and xAp are intersection points where xBp shadows xAp
        xBp = (S01ValB1onA != 0) ? intersectionPointBe1withA : xBp;
        xBp = (S01ValB2onA != 0) ? intersectionPointBe2withA : xBp;

        // xBm and xAm are intersection points where xBm does not shadow xAm
        xBm = (X01ValBe1withA != 0 && S01ValB1onA == 0) ? intersectionPointBe1withA : xBm;
        xBm = (X01ValBe2withA != 0 && S01ValB2onA == 0) ? intersectionPointBe2withA : xBm;

        float t = (xBp.y - vA.y) / (xBp.y - xBm.y);

        FmVector3 vB;
        vB.x = vA.x;
        vB.y = vA.y;
        vB.z = xBp.z - t * (xBp.z - xBm.z);
        *intersectionPointB = vB;

        return intersectionVal;
    }

    // Intersection function for triangle A and vertex B intersecting in XY plane
    int FmIntersectionX20(FmVector3* intersectionPointA, const FmVector3& fA0, const FmVector3& fA1, const FmVector3& fA2, const FmVector3& vB)
    {
        FmVector3 intersectionPointAe0withB, intersectionPointAe1withB, intersectionPointAe2withB;
        int X10ValAe0withB = FmIntersectionX10(&intersectionPointAe0withB, fA0, fA1, vB);
        int X10ValAe1withB = FmIntersectionX10(&intersectionPointAe1withB, fA1, fA2, vB);
        int X10ValAe2withB = FmIntersectionX10(&intersectionPointAe2withB, fA2, fA0, vB);

        int S10ValBonAe0 = FmShadowFunction(X10ValAe0withB, intersectionPointAe0withB.y, vB.y);
        int S10ValBonAe1 = FmShadowFunction(X10ValAe1withB, intersectionPointAe1withB.y, vB.y);
        int S10ValBonAe2 = FmShadowFunction(X10ValAe2withB, intersectionPointAe2withB.y, vB.y);

        int intersectionVal = S10ValBonAe0 + S10ValBonAe1 + S10ValBonAe2;

        FmVector3 xAp = intersectionPointAe0withB;
        FmVector3 xAm = intersectionPointAe0withB;

        // xBp and xAp are intersection points where xBp shadows xAp
        xAp = (S10ValBonAe1 != 0) ? intersectionPointAe1withB : xAp;
        xAp = (S10ValBonAe2 != 0) ? intersectionPointAe2withB : xAp;

        // xBm and xAm are intersection points where xBm does not shadow xAm
        xAm = (X10ValAe1withB != 0 && S10ValBonAe1 == 0) ? intersectionPointAe1withB : xAm;
        xAm = (X10ValAe2withB != 0 && S10ValBonAe2 == 0) ? intersectionPointAe2withB : xAm;

        float t = (vB.y - xAp.y) / (xAm.y - xAp.y);

        FmVector3 vA;
        vA.x = vB.x;
        vA.y = vB.y;
        vA.z = xAp.z - t * (xAp.z - xAm.z);
        *intersectionPointA = vA;

        return intersectionVal;
    }

    // Intersection function for edge A and triangle B intersecting in 3D
    int FmIntersectionX12(
        FmVector3* intersectionPoint,
        const FmVector3& eA0, const FmVector3& eA1,
        const FmVector3& fB0, const FmVector3& fB1, const FmVector3& fB2)
    {
        FmVector3 intersectionPointBwithA0, intersectionPointBwithA1;
        FmVector3 intersectionPointAwithBe0, intersectionPointBe0withA;
        FmVector3 intersectionPointAwithBe1, intersectionPointBe1withA;
        FmVector3 intersectionPointAwithBe2, intersectionPointBe2withA;
        int X02ValBwithA0 = FmIntersectionX02(&intersectionPointBwithA0, eA0, fB0, fB1, fB2);
        int X02ValBwithA1 = FmIntersectionX02(&intersectionPointBwithA1, eA1, fB0, fB1, fB2);
        int X11ValAwithBe0 = FmIntersectionX11(&intersectionPointAwithBe0, &intersectionPointBe0withA, eA0, eA1, fB0, fB1);
        int X11ValAwithBe1 = FmIntersectionX11(&intersectionPointAwithBe1, &intersectionPointBe1withA, eA0, eA1, fB1, fB2);
        int X11ValAwithBe2 = FmIntersectionX11(&intersectionPointAwithBe2, &intersectionPointBe2withA, eA0, eA1, fB2, fB0);

        int S02ValBonA0 = FmShadowFunction(X02ValBwithA0, eA0.z, intersectionPointBwithA0.z);
        int S02ValBonA1 = FmShadowFunction(X02ValBwithA1, eA1.z, intersectionPointBwithA1.z);
        int S11ValBe0onA = FmShadowFunction(X11ValAwithBe0, intersectionPointAwithBe0.z, intersectionPointBe0withA.z);
        int S11ValBe1onA = FmShadowFunction(X11ValAwithBe1, intersectionPointAwithBe1.z, intersectionPointBe1withA.z);
        int S11ValBe2onA = FmShadowFunction(X11ValAwithBe2, intersectionPointAwithBe2.z, intersectionPointBe2withA.z);

        int intersectionVal = -S02ValBonA1 + S02ValBonA0 - S11ValBe0onA - S11ValBe1onA - S11ValBe2onA;

        float xBp_z = intersectionPointBwithA0.z;
        FmVector3 xAp = eA0;
        float xBm_z = intersectionPointBwithA0.z;
        FmVector3 xAm = eA0;

        // xBp and xAp are intersection points where xBp shadows xAp
        xBp_z = (S02ValBonA1 != 0) ? intersectionPointBwithA1.z : xBp_z;
        xAp = (S02ValBonA1 != 0) ? eA1 : xAp;
        xBp_z = (S11ValBe0onA != 0) ? intersectionPointBe0withA.z : xBp_z;
        xAp = (S11ValBe0onA != 0) ? intersectionPointAwithBe0 : xAp;
        xBp_z = (S11ValBe1onA != 0) ? intersectionPointBe1withA.z : xBp_z;
        xAp = (S11ValBe1onA != 0) ? intersectionPointAwithBe1 : xAp;
        xBp_z = (S11ValBe2onA != 0) ? intersectionPointBe2withA.z : xBp_z;
        xAp = (S11ValBe2onA != 0) ? intersectionPointAwithBe2 : xAp;

        // xBm and xAm are intersection points where xBm does not shadow xAm
        xBm_z = (X02ValBwithA1 != 0 && S02ValBonA1 == 0) ? intersectionPointBwithA1.z : xBm_z;
        xAm = (X02ValBwithA1 != 0 && S02ValBonA1 == 0) ? eA1 : xAm;
        xBm_z = (X11ValAwithBe0 != 0 && S11ValBe0onA == 0) ? intersectionPointBe0withA.z : xBm_z;
        xAm = (X11ValAwithBe0 != 0 && S11ValBe0onA == 0) ? intersectionPointAwithBe0 : xAm;
        xBm_z = (X11ValAwithBe1 != 0 && S11ValBe1onA == 0) ? intersectionPointBe1withA.z : xBm_z;
        xAm = (X11ValAwithBe1 != 0 && S11ValBe1onA == 0) ? intersectionPointAwithBe1 : xAm;
        xBm_z = (X11ValAwithBe2 != 0 && S11ValBe2onA == 0) ? intersectionPointBe2withA.z : xBm_z;
        xAm = (X11ValAwithBe2 != 0 && S11ValBe2onA == 0) ? intersectionPointAwithBe2 : xAm;

        float deltam = xBm_z - xAm.z;
        float deltap = xBp_z - xAp.z;
        float t = deltap / (deltap - deltam);

        FmVector3 vA = xAp - t * (xAp - xAm);
        //FmVector3 vB = xBp - t * (xBp - xBm);
        *intersectionPoint = vA;

        return intersectionVal;
    }

    // Intersection function for triangle A and edge B intersecting in 3D
    int FmIntersectionX21(
        FmVector3* intersectionPoint,
        const FmVector3& fA0, const FmVector3& fA1, const FmVector3& fA2,
        const FmVector3& eB0, const FmVector3& eB1)
    {
        FmVector3 intersectionPointAwithB0, intersectionPointAwithB1;
        FmVector3 intersectionPointAe0withB, intersectionPointBwithAe0;
        FmVector3 intersectionPointAe1withB, intersectionPointBwithAe1;
        FmVector3 intersectionPointAe2withB, intersectionPointBwithAe2;
        int X20ValAwithB0 = FmIntersectionX20(&intersectionPointAwithB0, fA0, fA1, fA2, eB0);
        int X20ValAwithB1 = FmIntersectionX20(&intersectionPointAwithB1, fA0, fA1, fA2, eB1);
        int X11ValAe0withB = FmIntersectionX11(&intersectionPointAe0withB, &intersectionPointBwithAe0, fA0, fA1, eB0, eB1);
        int X11ValAe1withB = FmIntersectionX11(&intersectionPointAe1withB, &intersectionPointBwithAe1, fA1, fA2, eB0, eB1);
        int X11ValAe2withB = FmIntersectionX11(&intersectionPointAe2withB, &intersectionPointBwithAe2, fA2, fA0, eB0, eB1);

        int S20ValB0onA = FmShadowFunction(X20ValAwithB0, intersectionPointAwithB0.z, eB0.z);
        int S20ValB1onA = FmShadowFunction(X20ValAwithB1, intersectionPointAwithB1.z, eB1.z);
        int S11ValBonAe0 = FmShadowFunction(X11ValAe0withB, intersectionPointAe0withB.z, intersectionPointBwithAe0.z);
        int S11ValBonAe1 = FmShadowFunction(X11ValAe1withB, intersectionPointAe1withB.z, intersectionPointBwithAe1.z);
        int S11ValBonAe2 = FmShadowFunction(X11ValAe2withB, intersectionPointAe2withB.z, intersectionPointBwithAe2.z);

        int intersectionVal = S20ValB1onA - S20ValB0onA - S11ValBonAe0 - S11ValBonAe1 - S11ValBonAe2;

        float xBp_z = eB0.z;
        FmVector3 xAp = intersectionPointAwithB0;
        float xBm_z = eB0.z;
        FmVector3 xAm = intersectionPointAwithB0;

        // xBp and xAp are intersection points where xBp shadows xAp
        xBp_z = (S20ValB1onA != 0) ? eB1.z : xBp_z;
        xAp = (S20ValB1onA != 0) ? intersectionPointAwithB1 : xAp;
        xBp_z = (S11ValBonAe0 != 0) ? intersectionPointBwithAe0.z : xBp_z;
        xAp = (S11ValBonAe0 != 0) ? intersectionPointAe0withB : xAp;
        xBp_z = (S11ValBonAe1 != 0) ? intersectionPointBwithAe1.z : xBp_z;
        xAp = (S11ValBonAe1 != 0) ? intersectionPointAe1withB : xAp;
        xBp_z = (S11ValBonAe2 != 0) ? intersectionPointBwithAe2.z : xBp_z;
        xAp = (S11ValBonAe2 != 0) ? intersectionPointAe2withB : xAp;

        // xBm and xAm are intersection points where xBm does not shadow xAm
        xBm_z = (X20ValAwithB1 != 0 && S20ValB1onA == 0) ? eB1.z : xBm_z;
        xAm = (X20ValAwithB1 != 0 && S20ValB1onA == 0) ? intersectionPointAwithB1 : xAm;
        xBm_z = (X11ValAe0withB != 0 && S11ValBonAe0 == 0) ? intersectionPointBwithAe0.z : xBm_z;
        xAm = (X11ValAe0withB != 0 && S11ValBonAe0 == 0) ? intersectionPointAe0withB : xAm;
        xBm_z = (X11ValAe1withB != 0 && S11ValBonAe1 == 0) ? intersectionPointBwithAe1.z : xBm_z;
        xAm = (X11ValAe1withB != 0 && S11ValBonAe1 == 0) ? intersectionPointAe1withB : xAm;
        xBm_z = (X11ValAe2withB != 0 && S11ValBonAe2 == 0) ? intersectionPointBwithAe2.z : xBm_z;
        xAm = (X11ValAe2withB != 0 && S11ValBonAe2 == 0) ? intersectionPointAe2withB : xAm;

        float deltam = xBm_z - xAm.z;
        float deltap = xBp_z - xAp.z;
        float t = deltap / (deltap - deltam);

        FmVector3 vA = xAp - t * (xAp - xAm);
        //FmVector3 vB = xBp - t * (xBp - xBm);
        *intersectionPoint = vA;

        return intersectionVal;
    }

    // Intersection function for vertex A and tetrahedron B intersecting in 3D
    int FmIntersectionX03(
        const FmVector3& vA,
        const FmVector3& tB0, const FmVector3& tB1, const FmVector3& tB2, const FmVector3& tB3)
    {
        FmVector3 intersectionPointBf0withA, intersectionPointBf1withA, intersectionPointBf2withA, intersectionPointBf3withA;
        int X02ValBf0withA = FmIntersectionX02(&intersectionPointBf0withA, vA, tB0, tB2, tB1);
        int X02ValBf1withA = FmIntersectionX02(&intersectionPointBf1withA, vA, tB1, tB3, tB0);
        int X02ValBf2withA = FmIntersectionX02(&intersectionPointBf2withA, vA, tB2, tB0, tB3);
        int X02ValBf3withA = FmIntersectionX02(&intersectionPointBf3withA, vA, tB3, tB1, tB2);

        int S02ValBf0onA = FmShadowFunction(X02ValBf0withA, vA.z, intersectionPointBf0withA.z);
        int S02ValBf1onA = FmShadowFunction(X02ValBf1withA, vA.z, intersectionPointBf1withA.z);
        int S02ValBf2onA = FmShadowFunction(X02ValBf2withA, vA.z, intersectionPointBf2withA.z);
        int S02ValBf3onA = FmShadowFunction(X02ValBf3withA, vA.z, intersectionPointBf3withA.z);

        int intersectionVal = S02ValBf0onA + S02ValBf1onA + S02ValBf2onA + S02ValBf3onA;
        return intersectionVal;
    }

    // Intersection function for tetrahedron A and vertex B intersecting in 3D
    int FmIntersectionX30(
        const FmVector3& tA0, const FmVector3& tA1, const FmVector3& tA2, const FmVector3& tA3,
        const FmVector3& vB)
    {
        FmVector3 intersectionPointAf0withB, intersectionPointAf1withB, intersectionPointAf2withB, intersectionPointAf3withB;
        int X20ValAf0withB = FmIntersectionX20(&intersectionPointAf0withB, tA0, tA2, tA1, vB);
        int X20ValAf1withB = FmIntersectionX20(&intersectionPointAf1withB, tA1, tA3, tA0, vB);
        int X20ValAf2withB = FmIntersectionX20(&intersectionPointAf2withB, tA2, tA0, tA3, vB);
        int X20ValAf3withB = FmIntersectionX20(&intersectionPointAf3withB, tA3, tA1, tA2, vB);

        int S20ValBonAf0 = FmShadowFunction(X20ValAf0withB, intersectionPointAf0withB.z, vB.z);
        int S20ValBonAf1 = FmShadowFunction(X20ValAf1withB, intersectionPointAf1withB.z, vB.z);
        int S20ValBonAf2 = FmShadowFunction(X20ValAf2withB, intersectionPointAf2withB.z, vB.z);
        int S20ValBonAf3 = FmShadowFunction(X20ValAf3withB, intersectionPointAf3withB.z, vB.z);

        int intersectionVal = -S20ValBonAf0 - S20ValBonAf1 - S20ValBonAf2 - S20ValBonAf3;
        return intersectionVal;
    }

    bool FmComputeTriIntersection(FmTriIntersectionPoints* triIntersectionPoints, const FmVector3 fA[3], const FmVector3 fB[3])
    {
        int X12Val0, X12Val1, X12Val2;
        int X21Val0, X21Val1, X21Val2;
        FmVector3 edgeFacePoint0, edgeFacePoint1, edgeFacePoint2;
        FmVector3 faceEdgePoint0, faceEdgePoint1, faceEdgePoint2;

        X12Val0 = FmIntersectionX12(&edgeFacePoint0, fA[0], fA[1], fB[0], fB[1], fB[2]);
        X12Val1 = FmIntersectionX12(&edgeFacePoint1, fA[1], fA[2], fB[0], fB[1], fB[2]);
        X12Val2 = FmIntersectionX12(&edgeFacePoint2, fA[2], fA[0], fB[0], fB[1], fB[2]);

        X21Val0 = FmIntersectionX21(&faceEdgePoint0, fA[0], fA[1], fA[2], fB[0], fB[1]);
        X21Val1 = FmIntersectionX21(&faceEdgePoint1, fA[0], fA[1], fA[2], fB[1], fB[2]);
        X21Val2 = FmIntersectionX21(&faceEdgePoint2, fA[0], fA[1], fA[2], fB[2], fB[0]);

        triIntersectionPoints->XVals[0] = X12Val0;
        triIntersectionPoints->XVals[1] = X12Val1;
        triIntersectionPoints->XVals[2] = X12Val2;
        triIntersectionPoints->XVals[3] = X21Val0;
        triIntersectionPoints->XVals[4] = X21Val1;
        triIntersectionPoints->XVals[5] = X21Val2;
        triIntersectionPoints->points[0] = edgeFacePoint0;
        triIntersectionPoints->points[1] = edgeFacePoint1;
        triIntersectionPoints->points[2] = edgeFacePoint2;
        triIntersectionPoints->points[3] = faceEdgePoint0;
        triIntersectionPoints->points[4] = faceEdgePoint1;
        triIntersectionPoints->points[5] = faceEdgePoint2;

        int netEndCount12Val0, netEndCount12Val1, netEndCount12Val2;
        int netEndCount21Val0, netEndCount21Val1, netEndCount21Val2;
        netEndCount12Val0 = -X12Val0;
        netEndCount12Val1 = -X12Val1;
        netEndCount12Val2 = -X12Val2;
        netEndCount21Val0 = X21Val0;
        netEndCount21Val1 = X21Val1;
        netEndCount21Val2 = X21Val2;

        uint numStart = 0, numEnd = 0;
        uint startPoints[3] = { 0 };
        uint endPoints[3] = { 0 };

        while (netEndCount12Val0 < 0 && numStart < 3)
        {
            startPoints[numStart++] = 0;
            netEndCount12Val0++;
        }
        while (netEndCount12Val0 > 0 && numEnd < 3)
        {
            endPoints[numEnd++] = 0;
            netEndCount12Val0--;
        }
        while (netEndCount12Val1 < 0 && numStart < 3)
        {
            startPoints[numStart++] = 1;
            netEndCount12Val1++;
        }
        while (netEndCount12Val1 > 0 && numEnd < 3)
        {
            endPoints[numEnd++] = 1;
            netEndCount12Val1--;
        }
        while (netEndCount12Val2 < 0 && numStart < 3)
        {
            startPoints[numStart++] = 2;
            netEndCount12Val2++;
        }
        while (netEndCount12Val2 > 0 && numEnd < 3)
        {
            endPoints[numEnd++] = 2;
            netEndCount12Val2--;
        }
        while (netEndCount21Val0 < 0 && numStart < 3)
        {
            startPoints[numStart++] = 3;
            netEndCount21Val0++;
        }
        while (netEndCount21Val0 > 0 && numEnd < 3)
        {
            endPoints[numEnd++] = 3;
            netEndCount21Val0--;
        }
        while (netEndCount21Val1 < 0 && numStart < 3)
        {
            startPoints[numStart++] = 4;
            netEndCount21Val1++;
        }
        while (netEndCount21Val1 > 0 && numEnd < 3)
        {
            endPoints[numEnd++] = 4;
            netEndCount21Val1--;
        }
        while (netEndCount21Val2 < 0 && numStart < 3)
        {
            startPoints[numStart++] = 5;
            netEndCount21Val2++;
        }
        while (netEndCount21Val2 > 0 && numEnd < 3)
        {
            endPoints[numEnd++] = 5;
            netEndCount21Val2--;
        }

        FM_ASSERT(numStart == numEnd);

        triIntersectionPoints->numSegments = numStart;
        triIntersectionPoints->startPoints[0] = startPoints[0];
        triIntersectionPoints->startPoints[1] = startPoints[1];
        triIntersectionPoints->startPoints[2] = startPoints[2];
        triIntersectionPoints->endPoints[0] = endPoints[0];
        triIntersectionPoints->endPoints[1] = endPoints[1];
        triIntersectionPoints->endPoints[2] = endPoints[2];

        return (numStart > 0);
    }

    float FmComputeEdgeFaceIntersectionVolumeAndGradientContribution(
        FmVector3 dVdp_contribution[3],
        const FmVector3& triPos0, const FmVector3& triPos1, const FmVector3& triPos2, const FmVector3& edge1Pos,
        bool isRetainedEdgeStart)
    {
        float barycentrics[3];
        FmBarycentricCoords(barycentrics, edge1Pos, triPos0, triPos1, triPos2);

        // Integrate barycentric coordinates across polygon
        const FmVector3& pos0 = triPos0;
        float z0 = pos0.z;

        FmVector3 veca, vecb, areas;
        float zi, zi1, zavg;
        float baryavg0, baryavg1, baryavg2;

        const FmVector3& posi = triPos1;
        const FmVector3& posi1 = edge1Pos;

        // Find volume and gradient contribution to triangle A
        zi = posi.z;
        zi1 = posi1.z;

        veca = posi - pos0;
        vecb = posi1 - pos0;
        areas = cross(veca, vecb) * 0.5f;

        zavg = (z0 + zi + zi1) * (1.0f / 3.0f);

        float V_contribution = areas.z * zavg;

        baryavg0 = (1.0f + 0.0f + barycentrics[0]) * (1.0f / 3.0f);
        baryavg1 = (0.0f + 1.0f + barycentrics[1]) * (1.0f / 3.0f);
        baryavg2 = (0.0f + 0.0f + barycentrics[2]) * (1.0f / 3.0f);

        areas = isRetainedEdgeStart ? -areas : areas;
        V_contribution = isRetainedEdgeStart ? -V_contribution : V_contribution;

        dVdp_contribution[0] = areas * baryavg0;
        dVdp_contribution[1] = areas * baryavg1;
        dVdp_contribution[2] = areas * baryavg2;

        return V_contribution;
    }

    float FmComputePartialEdgeFaceIntersectionVolumeAndGradientContribution(
        bool* isFullFaceIntegration,
        int* numFullFaceIntegrations,
        FmVector3 dVdp_contribution[3],
        const FmVector3& triPos0, const FmVector3& triPos1, const FmVector3& triPos2, const FmVector3& edge1Pos,
        bool isRetainedEdgeStart)
    {
        if (FmIsEqual(edge1Pos, triPos1))
        {
            *isFullFaceIntegration = true;
            *numFullFaceIntegrations = 0;
            dVdp_contribution[0] = FmInitVector3(0.0f);
            dVdp_contribution[1] = FmInitVector3(0.0f);
            dVdp_contribution[2] = FmInitVector3(0.0f);
            return 0.0f;
        }
        else if (FmIsEqual(edge1Pos, triPos2))
        {
            *isFullFaceIntegration = true;
            *numFullFaceIntegrations = isRetainedEdgeStart ? 1 : -1;
            dVdp_contribution[0] = FmInitVector3(0.0f);
            dVdp_contribution[1] = FmInitVector3(0.0f);
            dVdp_contribution[2] = FmInitVector3(0.0f);
            return 0.0f;
        }
        else
        {
            *isFullFaceIntegration = false;
            *numFullFaceIntegrations = 0;
        }

        float barycentrics[3];
        FmBarycentricCoords(barycentrics, edge1Pos, triPos0, triPos1, triPos2);

        // Integrate barycentric coordinates across polygon
        const FmVector3& pos0 = triPos0;
        float z0 = pos0.z;

        FmVector3 veca, vecb, areas;
        float zi, zi1, zavg;
        float baryavg0, baryavg1, baryavg2;

        const FmVector3& posi = triPos1;
        const FmVector3& posi1 = edge1Pos;

        // Find volume and gradient contribution to triangle A
        zi = posi.z;
        zi1 = posi1.z;

        veca = posi - pos0;
        vecb = posi1 - pos0;
        areas = cross(veca, vecb) * 0.5f;

        zavg = (z0 + zi + zi1) * (1.0f / 3.0f);

        float V_contribution = areas.z * zavg;

        baryavg0 = (1.0f + 0.0f + barycentrics[0]) * (1.0f / 3.0f);
        baryavg1 = (0.0f + 1.0f + barycentrics[1]) * (1.0f / 3.0f);
        baryavg2 = (0.0f + 0.0f + barycentrics[2]) * (1.0f / 3.0f);

        areas = isRetainedEdgeStart ? -areas : areas;
        V_contribution = isRetainedEdgeStart ? -V_contribution : V_contribution;

        dVdp_contribution[0] = areas * baryavg0;
        dVdp_contribution[1] = areas * baryavg1;
        dVdp_contribution[2] = areas * baryavg2;

        return V_contribution;
    }

    float FmComputeFaceIntersectionVolumeAndGradientContribution(
        FmVector3 dVdp_contribution[3],
        const FmVector3& triPos0, const FmVector3& triPos1, const FmVector3& triPos2,
        bool isRetainedEdgeStart)
    {
        // Integrate barycentric coordinates across polygon
        const FmVector3& pos0 = triPos0;
        float z0 = pos0.z;

        FmVector3 veca, vecb, areas;
        float zi, zi1, zavg;

        const FmVector3& posi = triPos1;
        const FmVector3& posi1 = triPos2;

        // Find volume and gradient contribution to triangle A
        zi = posi.z;
        zi1 = posi1.z;

        veca = posi - pos0;
        vecb = posi1 - pos0;
        areas = cross(veca, vecb) * 0.5f;

        zavg = (z0 + zi + zi1) * (1.0f / 3.0f);

        float V_contribution = areas.z * zavg;

        areas = isRetainedEdgeStart ? -areas : areas;
        V_contribution = isRetainedEdgeStart ? -V_contribution : V_contribution;

        dVdp_contribution[0] = areas * (1.0f / 3.0f);
        dVdp_contribution[1] = areas * (1.0f / 3.0f);
        dVdp_contribution[2] = areas * (1.0f / 3.0f);

        return V_contribution;
    }

    void FmComputeTriIntersectionVolumeAndGradientContribution(
        FmVector3 dVdp_contributionA[3],
        FmVector3 dVdp_contributionB[3],
        float* V_contributionA,
        float* V_contributionB,
        const FmTriIntersectionPoints& triIntersectionPoints,
        const FmVector3& triAPos0, const FmVector3& triAPos1, const FmVector3& triAPos2,
        const FmVector3& triBPos0, const FmVector3& triBPos1, const FmVector3& triBPos2)
    {
        if (triIntersectionPoints.numSegments == 0)
        {
            dVdp_contributionA[0] = FmInitVector3(0.0f, 0.0f, 0.0f);
            dVdp_contributionA[1] = FmInitVector3(0.0f, 0.0f, 0.0f);
            dVdp_contributionA[2] = FmInitVector3(0.0f, 0.0f, 0.0f);
            dVdp_contributionB[0] = FmInitVector3(0.0f, 0.0f, 0.0f);
            dVdp_contributionB[1] = FmInitVector3(0.0f, 0.0f, 0.0f);
            dVdp_contributionB[2] = FmInitVector3(0.0f, 0.0f, 0.0f);

            *V_contributionA = 0.0f;
            *V_contributionB = 0.0f;

            return;
        }

        FmVector3 dVdp_contribA[3];
        FmVector3 dVdp_contribB[3];
        dVdp_contribA[0] = FmInitVector3(0.0f, 0.0f, 0.0f);
        dVdp_contribA[1] = FmInitVector3(0.0f, 0.0f, 0.0f);
        dVdp_contribA[2] = FmInitVector3(0.0f, 0.0f, 0.0f);
        dVdp_contribB[0] = FmInitVector3(0.0f, 0.0f, 0.0f);
        dVdp_contribB[1] = FmInitVector3(0.0f, 0.0f, 0.0f);
        dVdp_contribB[2] = FmInitVector3(0.0f, 0.0f, 0.0f);

        float V_contribA = 0.0f;
        float V_contribB = 0.0f;

        float barycentricsA[6][3];
        float barycentricsB[6][3];

        // Compute barycentric coordinates of intersection verts
        for (uint i = 0; i < 6; i++)
        {
            if (triIntersectionPoints.XVals[i] == 0)
                continue;

            const FmVector3& pos = triIntersectionPoints.points[i];
            FmBarycentricCoords(barycentricsA[i], pos, triAPos0, triAPos1, triAPos2);
            FmBarycentricCoords(barycentricsB[i], pos, triBPos0, triBPos1, triBPos2);
        }

        // Integrate barycentric coordinates across polygon
        const FmVector3& posA0 = triAPos0;
        float zA0 = posA0.z;
        float baryA00 = 1.0f;
        float baryA01 = 0.0f;
        float baryA02 = 0.0f;
        const FmVector3& posB0 = triBPos0;
        float zB0 = posB0.z;
        float baryB00 = 1.0f;
        float baryB01 = 0.0f;
        float baryB02 = 0.0f;

        uint numSegments = triIntersectionPoints.numSegments;
        FmVector3 veca, vecb, areas;

        for (uint i = 0; i < numSegments; i++)
        {
            // Form retained edge of tri intersection.
            uint startIdx = triIntersectionPoints.startPoints[i];
            uint endIdx = triIntersectionPoints.endPoints[i];

            FM_ASSERT(startIdx < 6 && endIdx < 6);

            const FmVector3& posi = triIntersectionPoints.points[startIdx];
            const FmVector3& posi1 = triIntersectionPoints.points[endIdx];

            // Find volume and gradient contribution to triangle A
            float zi, zi1, zavg;
            float baryavg0, baryavg1, baryavg2;

            zi = posi.z;
            zi1 = posi1.z;

            veca = posi - posA0;
            vecb = posi1 - posA0;
            areas = cross(veca, vecb) * 0.5f;

            zavg = (zA0 + zi + zi1) * (1.0f / 3.0f);

            V_contribA += areas.z * zavg;

            baryavg0 = (baryA00 + barycentricsA[startIdx][0] + barycentricsA[endIdx][0]) * (1.0f / 3.0f);
            baryavg1 = (baryA01 + barycentricsA[startIdx][1] + barycentricsA[endIdx][1]) * (1.0f / 3.0f);
            baryavg2 = (baryA02 + barycentricsA[startIdx][2] + barycentricsA[endIdx][2]) * (1.0f / 3.0f);

            dVdp_contribA[0] += areas * baryavg0;
            dVdp_contribA[1] += areas * baryavg1;
            dVdp_contribA[2] += areas * baryavg2;

            // Find volume and gradient contribution to triangle B.
            // Segment direction is reversed for B.
            zi = posi1.z;
            zi1 = posi.z;
            veca = posi1 - posB0;
            vecb = posi - posB0;
            areas = cross(veca, vecb) * 0.5f;

            zavg = (zB0 + zi + zi1) * (1.0f / 3.0f);

            V_contribB += areas.z * zavg;

            baryavg0 = (baryB00 + barycentricsB[startIdx][0] + barycentricsB[endIdx][0]) * (1.0f / 3.0f);
            baryavg1 = (baryB01 + barycentricsB[startIdx][1] + barycentricsB[endIdx][1]) * (1.0f / 3.0f);
            baryavg2 = (baryB02 + barycentricsB[startIdx][2] + barycentricsB[endIdx][2]) * (1.0f / 3.0f);

            dVdp_contribB[0] += areas * baryavg0;
            dVdp_contribB[1] += areas * baryavg1;
            dVdp_contribB[2] += areas * baryavg2;
        }

        dVdp_contributionA[0] = dVdp_contribA[0];
        dVdp_contributionA[1] = dVdp_contribA[1];
        dVdp_contributionA[2] = dVdp_contribA[2];
        dVdp_contributionB[0] = dVdp_contribB[0];
        dVdp_contributionB[1] = dVdp_contribB[1];
        dVdp_contributionB[2] = dVdp_contribB[2];

        *V_contributionA = V_contribA;
        *V_contributionB = V_contribB;
    }

    // Perform integrations for any segments that aren't across the entire triangle area.
    // Output number of full area integrations, to be completed later.
    // Also output whether only full area integrations were computed (dVdp and V results are all zero).
    void FmComputePartialTriIntersectionVolumeAndGradientContribution(
        bool* onlyFullIntegrationsA,
        bool* onlyFullIntegrationsB,
        int* numFullIntegrationsA,
        int* numFullIntegrationsB,
        FmVector3 dVdp_contributionA[3],
        FmVector3 dVdp_contributionB[3],
        float* V_contributionA,
        float* V_contributionB,
        const FmTriIntersectionPoints& triIntersectionPoints,
        const FmVector3& triAPos0, const FmVector3& triAPos1, const FmVector3& triAPos2,
        const FmVector3& triBPos0, const FmVector3& triBPos1, const FmVector3& triBPos2)
    {
        if (triIntersectionPoints.numSegments == 0)
        {
            *onlyFullIntegrationsA = true;
            *onlyFullIntegrationsB = true;

            *numFullIntegrationsA = 0;
            *numFullIntegrationsB = 0;

            dVdp_contributionA[0] = FmInitVector3(0.0f, 0.0f, 0.0f);
            dVdp_contributionA[1] = FmInitVector3(0.0f, 0.0f, 0.0f);
            dVdp_contributionA[2] = FmInitVector3(0.0f, 0.0f, 0.0f);
            dVdp_contributionB[0] = FmInitVector3(0.0f, 0.0f, 0.0f);
            dVdp_contributionB[1] = FmInitVector3(0.0f, 0.0f, 0.0f);
            dVdp_contributionB[2] = FmInitVector3(0.0f, 0.0f, 0.0f);

            *V_contributionA = 0.0f;
            *V_contributionB = 0.0f;

            return;
        }

        FmVector3 dVdp_contribA[3];
        FmVector3 dVdp_contribB[3];
        dVdp_contribA[0] = FmInitVector3(0.0f, 0.0f, 0.0f);
        dVdp_contribA[1] = FmInitVector3(0.0f, 0.0f, 0.0f);
        dVdp_contribA[2] = FmInitVector3(0.0f, 0.0f, 0.0f);
        dVdp_contribB[0] = FmInitVector3(0.0f, 0.0f, 0.0f);
        dVdp_contribB[1] = FmInitVector3(0.0f, 0.0f, 0.0f);
        dVdp_contribB[2] = FmInitVector3(0.0f, 0.0f, 0.0f);

        float V_contribA = 0.0f;
        float V_contribB = 0.0f;

        bool fullFaceA = true;
        bool fullFaceB = true;

        int numFullFaceA = 0;
        int numFullFaceB = 0;

        int cornerIndicesA[6] = { -1, -1, -1, -1, -1, -1 };
        int cornerIndicesB[6] = { -1, -1, -1, -1, -1, -1 };
        float barycentricsA[6][3];
        float barycentricsB[6][3];

        // Compute barycentric coordinates of intersection verts
        for (uint i = 0; i < 6; i++)
        {
            if (triIntersectionPoints.XVals[i] == 0)
                continue;

            const FmVector3& pos = triIntersectionPoints.points[i];
            cornerIndicesA[i] = FmBarycentricCoordsCheckIfCorner(barycentricsA[i], pos, triAPos0, triAPos1, triAPos2);
            cornerIndicesB[i] = FmBarycentricCoordsCheckIfCorner(barycentricsB[i], pos, triBPos0, triBPos1, triBPos2);
        }

        // Integrate barycentric coordinates across polygon
        const FmVector3& posA0 = triAPos0;
        float zA0 = posA0.z;
        float baryA00 = 1.0f;
        float baryA01 = 0.0f;
        float baryA02 = 0.0f;
        const FmVector3& posB0 = triBPos0;
        float zB0 = posB0.z;
        float baryB00 = 1.0f;
        float baryB01 = 0.0f;
        float baryB02 = 0.0f;

        uint numSegments = triIntersectionPoints.numSegments;
        FmVector3 veca, vecb, areas;

        for (uint i = 0; i < numSegments; i++)
        {
            // Form retained edge of tri intersection.
            uint startIdx = triIntersectionPoints.startPoints[i];
            uint endIdx = triIntersectionPoints.endPoints[i];

            FM_ASSERT(startIdx < 6 && endIdx < 6);

            const FmVector3& posi = triIntersectionPoints.points[startIdx];
            const FmVector3& posi1 = triIntersectionPoints.points[endIdx];

            // Skip if same corner
            int cornerIndexStartA = cornerIndicesA[startIdx];
            int cornerIndexEndA = cornerIndicesA[endIdx];
            int cornerIndexStartB = cornerIndicesB[startIdx];
            int cornerIndexEndB = cornerIndicesB[endIdx];

            if ((cornerIndexStartA >= 0 && cornerIndexStartA == cornerIndexEndA) || (cornerIndexStartB >= 0 && cornerIndexStartB == cornerIndexEndB))
            {
                continue;
            }

            if (cornerIndexStartA == 1 && cornerIndexEndA == 2)
            {
                // full negative area
                numFullFaceA--;
            }
            else if (cornerIndexStartA == 2 && cornerIndexEndA == 1)
            {
                // full positive area
                numFullFaceA++;
            }
            else if (
                (cornerIndexStartA == 0 && cornerIndexEndA == 1)
                || (cornerIndexStartA == 1 && cornerIndexEndA == 0)
                || (cornerIndexStartA == 0 && cornerIndexEndA == 2)
                || (cornerIndexStartA == 2 && cornerIndexEndA == 0))
            {
                // zero area
            }
            else
            {
                // partial area
                fullFaceA = false;

                // Find volume and gradient contribution to triangle A
                float zi, zi1, zavg;
                float baryavg0, baryavg1, baryavg2;

                zi = posi.z;
                zi1 = posi1.z;

                veca = posi - posA0;
                vecb = posi1 - posA0;
                areas = cross(veca, vecb) * 0.5f;

                zavg = (zA0 + zi + zi1) * (1.0f / 3.0f);

                V_contribA += areas.z * zavg;

                baryavg0 = (baryA00 + barycentricsA[startIdx][0] + barycentricsA[endIdx][0]) * (1.0f / 3.0f);
                baryavg1 = (baryA01 + barycentricsA[startIdx][1] + barycentricsA[endIdx][1]) * (1.0f / 3.0f);
                baryavg2 = (baryA02 + barycentricsA[startIdx][2] + barycentricsA[endIdx][2]) * (1.0f / 3.0f);

                dVdp_contribA[0] += areas * baryavg0;
                dVdp_contribA[1] += areas * baryavg1;
                dVdp_contribA[2] += areas * baryavg2;
            }

            if (cornerIndexStartB == 1 && cornerIndexEndB == 2)
            {
                // full positive area
                numFullFaceB++;
            }
            else if (cornerIndexStartB == 2 && cornerIndexEndB == 1)
            {
                // full negative area
                numFullFaceB--;
            }
            else if (
                (cornerIndexStartB == 0 && cornerIndexEndB == 1)
                || (cornerIndexStartB == 1 && cornerIndexEndB == 0)
                || (cornerIndexStartB == 0 && cornerIndexEndB == 2)
                || (cornerIndexStartB == 2 && cornerIndexEndB == 0))
            {
                // zero area
            }
            else
            {
                // partial area
                fullFaceB = false;

                // Find volume and gradient contribution to triangle B.
                // Segment direction is reversed for B.
                float zi, zi1, zavg;
                float baryavg0, baryavg1, baryavg2;

                zi = posi1.z;
                zi1 = posi.z;
                veca = posi1 - posB0;
                vecb = posi - posB0;
                areas = cross(veca, vecb) * 0.5f;

                zavg = (zB0 + zi + zi1) * (1.0f / 3.0f);

                V_contribB += areas.z * zavg;

                baryavg0 = (baryB00 + barycentricsB[startIdx][0] + barycentricsB[endIdx][0]) * (1.0f / 3.0f);
                baryavg1 = (baryB01 + barycentricsB[startIdx][1] + barycentricsB[endIdx][1]) * (1.0f / 3.0f);
                baryavg2 = (baryB02 + barycentricsB[startIdx][2] + barycentricsB[endIdx][2]) * (1.0f / 3.0f);

                dVdp_contribB[0] += areas * baryavg0;
                dVdp_contribB[1] += areas * baryavg1;
                dVdp_contribB[2] += areas * baryavg2;
            }
        }

        *onlyFullIntegrationsA = fullFaceA;
        *onlyFullIntegrationsB = fullFaceB;

        *numFullIntegrationsA = numFullFaceA;
        *numFullIntegrationsB = numFullFaceB;

        dVdp_contributionA[0] = dVdp_contribA[0];
        dVdp_contributionA[1] = dVdp_contribA[1];
        dVdp_contributionA[2] = dVdp_contribA[2];
        dVdp_contributionB[0] = dVdp_contribB[0];
        dVdp_contributionB[1] = dVdp_contribB[1];
        dVdp_contributionB[2] = dVdp_contribB[2];

        *V_contributionA = V_contribA;
        *V_contributionB = V_contribB;
    }
}
