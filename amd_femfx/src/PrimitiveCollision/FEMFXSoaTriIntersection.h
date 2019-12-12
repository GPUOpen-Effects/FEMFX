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
// Structure-of-arrays SIMD implementation of triangle intersection
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXTypes.h"
#include "FEMFXTriIntersection.h"

#define FM_SOA_INTERSECTION_MASK_INVALID 0

namespace AMD
{

    // Intersection (X) and Shadow (S) functions:
    //
    // Xij(featureA, featureB) is intersection of dimension i feature with dimension j feature (dimension of vert = 0, edge = 1, facet = 2) 
    // in coordinates dimensions up to (i + j).
    // E.g., X11 gives intersection state of edges (projections) in XY plane.
    // Nonzero result is intersecting, with sign affected by order of edge vertices.
    // 
    // Sij(featureA, featureB) returns Xij result when feature B "shadows" feature A, else 0.
    // Shadowing means intersection according to Xij (i+j < 3) and that at intersection point, point on B >= point on A in dimension (i + j + 1).
    // E.g., S11 = X11 if at the edge intersection in XY plane the point on edge B has z >= z of point on edge A, else 0
    //
    // Feature-pair intersection test in 3D is broken down into intersection tests in lower dimensions.
    // Intersection points are computed by interpolation of two lower-dimensional intersection points, one that shadows and one that 
    // intersects but does not shadow other feature.
    //
    // Intersection status and points of Xij(featureA, featureB) are identical regardless of order of feature vertices, with
    // sign of Xij return value affected by the order.
    //
    // Intersection status/points may be different between Xij(featureA, featureB) and Xji(featureB, featureA)
    // 
    // Reference: Smith and Dodgson, "A topologically robust algorithm for Boolean operations on polyhedral shapes using approximate arithmetic"

    // Shadow function is intersection value if the value is non-zero and coord dimension of b >= a, else 0
    template< class T >
    static FM_FORCE_INLINE typename T::SoaInt FmShadowFunction(typename T::SoaInt intersectionVal, typename T::SoaFloat a, typename T::SoaFloat b)
    {
        typedef typename T::SoaInt SoaInt;
        //return select(SoaInt(0), intersectionVal, (intersectionVal != SoaInt(0) & b >= a));
        return select(SoaInt(0), intersectionVal, (b >= a));
    }

    // Intersection function for vertex A and edge B intersecting on X axis
    template< class T >
    static FM_FORCE_INLINE typename T::SoaInt FmIntersectionX01(typename T::SoaVector3* intersectionPointB, const typename T::SoaVector3& vA, const typename T::SoaVector3& eB0, const typename T::SoaVector3& eB1)
    {
        typedef typename T::SoaVector3 SoaVector3;
        typedef typename T::SoaFloat SoaFloat;
        typedef typename T::SoaInt SoaInt;

        SoaInt S00B0onA = SoaInt(eB0.x >= vA.x);
        SoaInt S00B1onA = SoaInt(eB1.x >= vA.x);

        // Get point on B that intersects A on X axis.
        // Intersection point is same regardless of order of edge vertices, when return value nonzero. 
        SoaVector3 xBp = select(eB1, eB0, (S00B0onA != SoaInt(0)));
        SoaVector3 xBm = select(eB0, eB1, (S00B0onA != SoaInt(0)));

        SoaFloat t = (xBp.x - vA.x) / (xBp.x - xBm.x);

        SoaVector3 vB;
        vB.x = vA.x;
        vB.y = xBp.y - t * (xBp.y - xBm.y);
        vB.z = xBp.z - t * (xBp.z - xBm.z);
        *intersectionPointB = vB;

        SoaInt intersectionVal = S00B1onA - S00B0onA;
#if FM_SOA_INTERSECTION_MASK_INVALID
        *intersectionPointB = select(*intersectionPointB, SoaVector3(0.0f), (intersectionVal == 0));
#endif
        return intersectionVal;
    }

    // Intersection function for edge A and vertex B intersecting on X axis
    template< class T >
    static FM_FORCE_INLINE typename T::SoaInt FmIntersectionX10(typename T::SoaVector3* intersectionPointA, const typename T::SoaVector3& eA0, const typename T::SoaVector3& eA1, const typename T::SoaVector3& vB)
    {
        typedef typename T::SoaVector3 SoaVector3;
        typedef typename T::SoaFloat SoaFloat;
        typedef typename T::SoaInt SoaInt;

        SoaInt S00BonA0 = SoaInt(vB.x >= eA0.x);
        SoaInt S00BonA1 = SoaInt(vB.x >= eA1.x);

        // Get point on B that intersects A on X axis.
        // Intersection point is same regardless of order of edge vertices, when return value nonzero. 
        SoaVector3 xAp = select(eA1, eA0, (S00BonA0 != SoaInt(0)));
        SoaVector3 xAm = select(eA0, eA1, (S00BonA0 != SoaInt(0)));

        SoaFloat t = (vB.x - xAp.x) / (xAm.x - xAp.x);

        SoaVector3 vA;
        vA.x = vB.x;
        vA.y = xAp.y - t * (xAp.y - xAm.y);
        vA.z = xAp.z - t * (xAp.z - xAm.z);
        *intersectionPointA = vA;

        SoaInt intersectionVal = S00BonA0 - S00BonA1;
#if FM_SOA_INTERSECTION_MASK_INVALID
        *intersectionPointA = select(*intersectionPointA, SoaVector3(0.0f), (intersectionVal == 0));
#endif
        return intersectionVal;
    }

    // Intersection function for edge A and edge B intersecting in XY plane
    template< class T >
    typename T::SoaInt FmIntersectionX11(
        typename T::SoaVector3* intersectionPointA, typename T::SoaVector3* intersectionPointB,
        const typename T::SoaVector3& eA0, const typename T::SoaVector3& eA1, const typename T::SoaVector3& eB0, const typename T::SoaVector3& eB1)
    {
        typedef typename T::SoaVector3 SoaVector3;
        typedef typename T::SoaFloat SoaFloat;
        typedef typename T::SoaInt SoaInt;

        SoaVector3 intersectionPointBwithA0, intersectionPointBwithA1, intersectionPointAwithB0, intersectionPointAwithB1;
        SoaInt X01ValBwithA0 = FmIntersectionX01<T>(&intersectionPointBwithA0, eA0, eB0, eB1);
        SoaInt X01ValBwithA1 = FmIntersectionX01<T>(&intersectionPointBwithA1, eA1, eB0, eB1);
        SoaInt X10ValAwithB0 = FmIntersectionX10<T>(&intersectionPointAwithB0, eA0, eA1, eB0);
        SoaInt X10ValAwithB1 = FmIntersectionX10<T>(&intersectionPointAwithB1, eA0, eA1, eB1);

        SoaInt S01ValBonA0 = FmShadowFunction<T>(X01ValBwithA0, eA0.y, intersectionPointBwithA0.y);
        SoaInt S01ValBonA1 = FmShadowFunction<T>(X01ValBwithA1, eA1.y, intersectionPointBwithA1.y);
        SoaInt S10ValB0onA = FmShadowFunction<T>(X10ValAwithB0, intersectionPointAwithB0.y, eB0.y);
        SoaInt S10ValB1onA = FmShadowFunction<T>(X10ValAwithB1, intersectionPointAwithB1.y, eB1.y);

        SoaInt intersectionVal = S01ValBonA1 - S01ValBonA0 + S10ValB1onA - S10ValB0onA;

        SoaVector3 xBp = intersectionPointBwithA0;
        SoaVector3 xAp = eA0;
        SoaVector3 xBm = intersectionPointBwithA0;
        SoaVector3 xAm = eA0;

        // xBp and xAp are intersection points where xBp shadows xAp
        xBp = select(xBp, intersectionPointBwithA1, (S01ValBonA1 != SoaInt(0)));
        xAp = select(xAp, eA1, (S01ValBonA1 != SoaInt(0)));
        xBp = select(xBp, eB0, (S10ValB0onA != SoaInt(0)));
        xAp = select(xAp, intersectionPointAwithB0, (S10ValB0onA != SoaInt(0)));
        xBp = select(xBp, eB1, (S10ValB1onA != SoaInt(0)));
        xAp = select(xAp, intersectionPointAwithB1, (S10ValB1onA != SoaInt(0)));

        // xBm and xAm are intersection points where xBm does not shadow xAm
        xBm = select(xBm, intersectionPointBwithA1, ((X01ValBwithA1 != SoaInt(0)) & (S01ValBonA1 == SoaInt(0))));
        xAm = select(xAm, eA1, ((X01ValBwithA1 != SoaInt(0)) & (S01ValBonA1 == SoaInt(0))));
        xBm = select(xBm, eB0, ((X10ValAwithB0 != SoaInt(0)) & (S10ValB0onA == SoaInt(0))));
        xAm = select(xAm, intersectionPointAwithB0, ((X10ValAwithB0 != SoaInt(0)) & (S10ValB0onA == SoaInt(0))));
        xBm = select(xBm, eB1, ((X10ValAwithB1 != SoaInt(0)) & (S10ValB1onA == SoaInt(0))));
        xAm = select(xAm, intersectionPointAwithB1, ((X10ValAwithB1 != SoaInt(0)) & (S10ValB1onA == SoaInt(0))));

        SoaFloat deltam = xBm.y - xAm.y;
        SoaFloat deltap = xBp.y - xAp.y;
        SoaFloat t = deltap / (deltap - deltam);

        SoaVector3 vA = xAp - t * (xAp - xAm);
        //SoaVector3 vB = xBp - t * (xBp - xBm);
        SoaVector3 vB;
        vB.x = vA.x;
        vB.y = vA.y;
        vB.z = xBp.z - t * (xBp.z - xBm.z);
        *intersectionPointA = vA;
        *intersectionPointB = vB;
#if FM_SOA_INTERSECTION_MASK_INVALID
        *intersectionPointA = select(*intersectionPointA, SoaVector3(0.0f), (intersectionVal == 0));
        *intersectionPointB = select(*intersectionPointB, SoaVector3(0.0f), (intersectionVal == 0));
#endif

        return intersectionVal;
    }

    // Intersection function for vertex A and triangle B intersecting in XY plane
    template< class T >
    typename T::SoaInt FmIntersectionX02(
        typename T::SoaVector3* intersectionPointB, const typename T::SoaVector3& vA, const typename T::SoaVector3& fB0, const typename T::SoaVector3& fB1, const typename T::SoaVector3& fB2)
    {
        typedef typename T::SoaVector3 SoaVector3;
        typedef typename T::SoaFloat SoaFloat;
        typedef typename T::SoaInt SoaInt;

        SoaVector3 intersectionPointBe0withA, intersectionPointBe1withA, intersectionPointBe2withA;
        SoaInt X01ValBe0withA = FmIntersectionX01<T>(&intersectionPointBe0withA, vA, fB0, fB1);
        SoaInt X01ValBe1withA = FmIntersectionX01<T>(&intersectionPointBe1withA, vA, fB1, fB2);
        SoaInt X01ValBe2withA = FmIntersectionX01<T>(&intersectionPointBe2withA, vA, fB2, fB0);

        SoaInt S01ValB0onA = FmShadowFunction<T>(X01ValBe0withA, vA.y, intersectionPointBe0withA.y);
        SoaInt S01ValB1onA = FmShadowFunction<T>(X01ValBe1withA, vA.y, intersectionPointBe1withA.y);
        SoaInt S01ValB2onA = FmShadowFunction<T>(X01ValBe2withA, vA.y, intersectionPointBe2withA.y);

        SoaInt intersectionVal = -(S01ValB0onA + S01ValB1onA + S01ValB2onA);

        SoaVector3 xBp = intersectionPointBe0withA;
        SoaVector3 xBm = intersectionPointBe0withA;

        // xBp and xAp are intersection points where xBp shadows xAp
        xBp = select(xBp, intersectionPointBe1withA, (S01ValB1onA != SoaInt(0)));
        xBp = select(xBp, intersectionPointBe2withA, (S01ValB2onA != SoaInt(0)));

        // xBm and xAm are intersection points where xBm does not shadow xAm
        xBm = select(xBm, intersectionPointBe1withA, (X01ValBe1withA != SoaInt(0) & S01ValB1onA == SoaInt(0)));
        xBm = select(xBm, intersectionPointBe2withA, (X01ValBe2withA != SoaInt(0) & S01ValB2onA == SoaInt(0)));

        SoaFloat t = (xBp.y - vA.y) / (xBp.y - xBm.y);

        SoaVector3 vB;
        vB.x = vA.x;
        vB.y = vA.y;
        vB.z = xBp.z - t * (xBp.z - xBm.z);
        *intersectionPointB = vB;
#if FM_SOA_INTERSECTION_MASK_INVALID
        *intersectionPointB = select(*intersectionPointB, SoaVector3(0.0f), (intersectionVal == 0));
#endif

        return intersectionVal;
    }

    // Intersection function for triangle A and vertex B intersecting in XY plane
    template< class T >
    typename T::SoaInt FmIntersectionX20(typename T::SoaVector3* intersectionPointA, const typename T::SoaVector3& fA0, const typename T::SoaVector3& fA1, const typename T::SoaVector3& fA2, const typename T::SoaVector3& vB)
    {
        typedef typename T::SoaVector3 SoaVector3;
        typedef typename T::SoaFloat SoaFloat;
        typedef typename T::SoaInt SoaInt;

        SoaVector3 intersectionPointAe0withB, intersectionPointAe1withB, intersectionPointAe2withB;
        SoaInt X10ValAe0withB = FmIntersectionX10<T>(&intersectionPointAe0withB, fA0, fA1, vB);
        SoaInt X10ValAe1withB = FmIntersectionX10<T>(&intersectionPointAe1withB, fA1, fA2, vB);
        SoaInt X10ValAe2withB = FmIntersectionX10<T>(&intersectionPointAe2withB, fA2, fA0, vB);

        SoaInt S10ValBonAe0 = FmShadowFunction<T>(X10ValAe0withB, intersectionPointAe0withB.y, vB.y);
        SoaInt S10ValBonAe1 = FmShadowFunction<T>(X10ValAe1withB, intersectionPointAe1withB.y, vB.y);
        SoaInt S10ValBonAe2 = FmShadowFunction<T>(X10ValAe2withB, intersectionPointAe2withB.y, vB.y);

        SoaInt intersectionVal = S10ValBonAe0 + S10ValBonAe1 + S10ValBonAe2;

        SoaVector3 xAp = intersectionPointAe0withB;
        SoaVector3 xAm = intersectionPointAe0withB;

        // xBp and xAp are intersection points where xBp shadows xAp
        xAp = select(xAp, intersectionPointAe1withB, (S10ValBonAe1 != SoaInt(0)));
        xAp = select(xAp, intersectionPointAe2withB, (S10ValBonAe2 != SoaInt(0)));

        // xBm and xAm are intersection points where xBm does not shadow xAm
        xAm = select(xAm, intersectionPointAe1withB, (X10ValAe1withB != SoaInt(0) & S10ValBonAe1 == SoaInt(0)));
        xAm = select(xAm, intersectionPointAe2withB, (X10ValAe2withB != SoaInt(0) & S10ValBonAe2 == SoaInt(0)));

        SoaFloat t = (vB.y - xAp.y) / (xAm.y - xAp.y);

        SoaVector3 vA;
        vA.x = vB.x;
        vA.y = vB.y;
        vA.z = xAp.z - t * (xAp.z - xAm.z);
        *intersectionPointA = vA;
#if FM_SOA_INTERSECTION_MASK_INVALID
        *intersectionPointA = select(*intersectionPointA, SoaVector3(0.0f), (intersectionVal == 0));
#endif

        return intersectionVal;
    }

    // Intersection function for edge A and triangle B intersecting in 3D
    template< class T >
    typename T::SoaInt FmIntersectionX12(
        typename T::SoaVector3* intersectionPoint,
        const typename T::SoaVector3& eA0, const typename T::SoaVector3& eA1,
        const typename T::SoaVector3& fB0, const typename T::SoaVector3& fB1, const typename T::SoaVector3& fB2)
    {
        typedef typename T::SoaVector3 SoaVector3;
        typedef typename T::SoaFloat SoaFloat;
        typedef typename T::SoaInt SoaInt;

        SoaVector3 intersectionPointBwithA0, intersectionPointBwithA1;
        SoaVector3 intersectionPointAwithBe0, intersectionPointBe0withA;
        SoaVector3 intersectionPointAwithBe1, intersectionPointBe1withA;
        SoaVector3 intersectionPointAwithBe2, intersectionPointBe2withA;
        SoaInt X02ValBwithA0 = FmIntersectionX02<T>(&intersectionPointBwithA0, eA0, fB0, fB1, fB2);
        SoaInt X02ValBwithA1 = FmIntersectionX02<T>(&intersectionPointBwithA1, eA1, fB0, fB1, fB2);
        SoaInt X11ValAwithBe0 = FmIntersectionX11<T>(&intersectionPointAwithBe0, &intersectionPointBe0withA, eA0, eA1, fB0, fB1);
        SoaInt X11ValAwithBe1 = FmIntersectionX11<T>(&intersectionPointAwithBe1, &intersectionPointBe1withA, eA0, eA1, fB1, fB2);
        SoaInt X11ValAwithBe2 = FmIntersectionX11<T>(&intersectionPointAwithBe2, &intersectionPointBe2withA, eA0, eA1, fB2, fB0);

        SoaInt S02ValBonA0 = FmShadowFunction<T>(X02ValBwithA0, eA0.z, intersectionPointBwithA0.z);
        SoaInt S02ValBonA1 = FmShadowFunction<T>(X02ValBwithA1, eA1.z, intersectionPointBwithA1.z);
        SoaInt S11ValBe0onA = FmShadowFunction<T>(X11ValAwithBe0, intersectionPointAwithBe0.z, intersectionPointBe0withA.z);
        SoaInt S11ValBe1onA = FmShadowFunction<T>(X11ValAwithBe1, intersectionPointAwithBe1.z, intersectionPointBe1withA.z);
        SoaInt S11ValBe2onA = FmShadowFunction<T>(X11ValAwithBe2, intersectionPointAwithBe2.z, intersectionPointBe2withA.z);

        SoaInt intersectionVal = -S02ValBonA1 + S02ValBonA0 - S11ValBe0onA - S11ValBe1onA - S11ValBe2onA;

        SoaFloat xBp_z = intersectionPointBwithA0.z;
        SoaVector3 xAp = eA0;
        SoaFloat xBm_z = intersectionPointBwithA0.z;
        SoaVector3 xAm = eA0;

        // xBp and xAp are intersection points where xBp shadows xAp
        xBp_z = select(xBp_z, intersectionPointBwithA1.z, (S02ValBonA1 != SoaInt(0)));
        xAp = select(xAp, eA1, (S02ValBonA1 != SoaInt(0)));
        xBp_z = select(xBp_z, intersectionPointBe0withA.z, (S11ValBe0onA != SoaInt(0)));
        xAp = select(xAp, intersectionPointAwithBe0, (S11ValBe0onA != SoaInt(0)));
        xBp_z = select(xBp_z, intersectionPointBe1withA.z, (S11ValBe1onA != SoaInt(0)));
        xAp = select(xAp, intersectionPointAwithBe1, (S11ValBe1onA != SoaInt(0)));
        xBp_z = select(xBp_z, intersectionPointBe2withA.z, (S11ValBe2onA != SoaInt(0)));
        xAp = select(xAp, intersectionPointAwithBe2, (S11ValBe2onA != SoaInt(0)));

        // xBm and xAm are intersection points where xBm does not shadow xAm
        xBm_z = select(xBm_z, intersectionPointBwithA1.z, ((X02ValBwithA1 != SoaInt(0)) & (S02ValBonA1 == SoaInt(0))));
        xAm = select(xAm, eA1, ((X02ValBwithA1 != SoaInt(0)) & (S02ValBonA1 == SoaInt(0))));
        xBm_z = select(xBm_z, intersectionPointBe0withA.z, ((X11ValAwithBe0 != SoaInt(0)) & (S11ValBe0onA == SoaInt(0))));
        xAm = select(xAm, intersectionPointAwithBe0, ((X11ValAwithBe0 != SoaInt(0)) & (S11ValBe0onA == SoaInt(0))));
        xBm_z = select(xBm_z, intersectionPointBe1withA.z, ((X11ValAwithBe1 != SoaInt(0)) & (S11ValBe1onA == SoaInt(0))));
        xAm = select(xAm, intersectionPointAwithBe1, ((X11ValAwithBe1 != SoaInt(0)) & (S11ValBe1onA == SoaInt(0))));
        xBm_z = select(xBm_z, intersectionPointBe2withA.z, ((X11ValAwithBe2 != SoaInt(0)) & (S11ValBe2onA == SoaInt(0))));
        xAm = select(xAm, intersectionPointAwithBe2, ((X11ValAwithBe2 != SoaInt(0)) & (S11ValBe2onA == SoaInt(0))));

        SoaFloat deltam = xBm_z - xAm.z;
        SoaFloat deltap = xBp_z - xAp.z;
        SoaFloat t = deltap / (deltap - deltam);

        SoaVector3 vA = xAp - t * (xAp - xAm);
        //SoaVector3 vB = xBp - t * (xBp - xBm);
        *intersectionPoint = vA;
#if FM_SOA_INTERSECTION_MASK_INVALID
        *intersectionPoint = select(*intersectionPoint, SoaVector3(0.0f), (intersectionVal == 0));
#endif

        return intersectionVal;
    }

    // Intersection function for triangle A and edge B intersecting in 3D
    template< class T >
    typename T::SoaInt FmIntersectionX21(
        typename T::SoaVector3* intersectionPoint,
        const typename T::SoaVector3& fA0, const typename T::SoaVector3& fA1, const typename T::SoaVector3& fA2,
        const typename T::SoaVector3& eB0, const typename T::SoaVector3& eB1)
    {
        typedef typename T::SoaVector3 SoaVector3;
        typedef typename T::SoaFloat SoaFloat;
        typedef typename T::SoaInt SoaInt;

        SoaVector3 intersectionPointAwithB0, intersectionPointAwithB1;
        SoaVector3 intersectionPointAe0withB, intersectionPointBwithAe0;
        SoaVector3 intersectionPointAe1withB, intersectionPointBwithAe1;
        SoaVector3 intersectionPointAe2withB, intersectionPointBwithAe2;
        SoaInt X20ValAwithB0 = FmIntersectionX20<T>(&intersectionPointAwithB0, fA0, fA1, fA2, eB0);
        SoaInt X20ValAwithB1 = FmIntersectionX20<T>(&intersectionPointAwithB1, fA0, fA1, fA2, eB1);
        SoaInt X11ValAe0withB = FmIntersectionX11<T>(&intersectionPointAe0withB, &intersectionPointBwithAe0, fA0, fA1, eB0, eB1);
        SoaInt X11ValAe1withB = FmIntersectionX11<T>(&intersectionPointAe1withB, &intersectionPointBwithAe1, fA1, fA2, eB0, eB1);
        SoaInt X11ValAe2withB = FmIntersectionX11<T>(&intersectionPointAe2withB, &intersectionPointBwithAe2, fA2, fA0, eB0, eB1);

        SoaInt S20ValB0onA = FmShadowFunction<T>(X20ValAwithB0, intersectionPointAwithB0.z, eB0.z);
        SoaInt S20ValB1onA = FmShadowFunction<T>(X20ValAwithB1, intersectionPointAwithB1.z, eB1.z);
        SoaInt S11ValBonAe0 = FmShadowFunction<T>(X11ValAe0withB, intersectionPointAe0withB.z, intersectionPointBwithAe0.z);
        SoaInt S11ValBonAe1 = FmShadowFunction<T>(X11ValAe1withB, intersectionPointAe1withB.z, intersectionPointBwithAe1.z);
        SoaInt S11ValBonAe2 = FmShadowFunction<T>(X11ValAe2withB, intersectionPointAe2withB.z, intersectionPointBwithAe2.z);

        SoaInt intersectionVal = S20ValB1onA - S20ValB0onA - S11ValBonAe0 - S11ValBonAe1 - S11ValBonAe2;

        SoaFloat xBp_z = eB0.z;
        SoaVector3 xAp = intersectionPointAwithB0;
        SoaFloat xBm_z = eB0.z;
        SoaVector3 xAm = intersectionPointAwithB0;

        // xBp and xAp are intersection points where xBp shadows xAp
        xBp_z = select(xBp_z, eB1.z, (S20ValB1onA != SoaInt(0)));
        xAp = select(xAp, intersectionPointAwithB1, (S20ValB1onA != SoaInt(0)));
        xBp_z = select(xBp_z, intersectionPointBwithAe0.z, (S11ValBonAe0 != SoaInt(0)));
        xAp = select(xAp, intersectionPointAe0withB, (S11ValBonAe0 != SoaInt(0)));
        xBp_z = select(xBp_z, intersectionPointBwithAe1.z, (S11ValBonAe1 != SoaInt(0)));
        xAp = select(xAp, intersectionPointAe1withB, (S11ValBonAe1 != SoaInt(0)));
        xBp_z = select(xBp_z, intersectionPointBwithAe2.z, (S11ValBonAe2 != SoaInt(0)));
        xAp = select(xAp, intersectionPointAe2withB, (S11ValBonAe2 != SoaInt(0)));

        // xBm and xAm are intersection points where xBm does not shadow xAm
        xBm_z = select(xBm_z, eB1.z, ((X20ValAwithB1 != SoaInt(0)) & (S20ValB1onA == SoaInt(0))));
        xAm = select(xAm, intersectionPointAwithB1, ((X20ValAwithB1 != SoaInt(0)) & (S20ValB1onA == SoaInt(0))));
        xBm_z = select(xBm_z, intersectionPointBwithAe0.z, ((X11ValAe0withB != SoaInt(0)) & (S11ValBonAe0 == SoaInt(0))));
        xAm = select(xAm, intersectionPointAe0withB, ((X11ValAe0withB != SoaInt(0)) & (S11ValBonAe0 == SoaInt(0))));
        xBm_z = select(xBm_z, intersectionPointBwithAe1.z, ((X11ValAe1withB != SoaInt(0)) & (S11ValBonAe1 == SoaInt(0))));
        xAm = select(xAm, intersectionPointAe1withB, ((X11ValAe1withB != SoaInt(0)) & (S11ValBonAe1 == SoaInt(0))));
        xBm_z = select(xBm_z, intersectionPointBwithAe2.z, ((X11ValAe2withB != SoaInt(0)) & (S11ValBonAe2 == SoaInt(0))));
        xAm = select(xAm, intersectionPointAe2withB, ((X11ValAe2withB != SoaInt(0)) & (S11ValBonAe2 == SoaInt(0))));

        SoaFloat deltam = xBm_z - xAm.z;
        SoaFloat deltap = xBp_z - xAp.z;
        SoaFloat t = deltap / (deltap - deltam);

        SoaVector3 vA = xAp - t * (xAp - xAm);
        //SoaVector3 vB = xBp - t * (xBp - xBm);
        *intersectionPoint = vA;
#if FM_SOA_INTERSECTION_MASK_INVALID
        *intersectionPoint = select(*intersectionPoint, SoaVector3(0.0f), (intersectionVal == 0));
#endif

        return intersectionVal;
    }

    // Intersection function for vertex A and tetrahedron B intersecting in 3D
    template< class T >
    typename T::SoaInt FmIntersectionX03(
        const typename T::SoaVector3& vA,
        const typename T::SoaVector3& tB0, const typename T::SoaVector3& tB1, const typename T::SoaVector3& tB2, const typename T::SoaVector3& tB3)
    {
        typedef typename T::SoaVector3 SoaVector3;
        typedef typename T::SoaInt SoaInt;

        SoaVector3 intersectionPointBf0withA, intersectionPointBf1withA, intersectionPointBf2withA, intersectionPointBf3withA;
        SoaInt X02ValBf0withA = FmIntersectionX02<T>(&intersectionPointBf0withA, vA, tB0, tB2, tB1);
        SoaInt X02ValBf1withA = FmIntersectionX02<T>(&intersectionPointBf1withA, vA, tB1, tB3, tB0);
        SoaInt X02ValBf2withA = FmIntersectionX02<T>(&intersectionPointBf2withA, vA, tB2, tB0, tB3);
        SoaInt X02ValBf3withA = FmIntersectionX02<T>(&intersectionPointBf3withA, vA, tB3, tB1, tB2);

        SoaInt S02ValBf0onA = FmShadowFunction<T>(X02ValBf0withA, vA.z, intersectionPointBf0withA.z);
        SoaInt S02ValBf1onA = FmShadowFunction<T>(X02ValBf1withA, vA.z, intersectionPointBf1withA.z);
        SoaInt S02ValBf2onA = FmShadowFunction<T>(X02ValBf2withA, vA.z, intersectionPointBf2withA.z);
        SoaInt S02ValBf3onA = FmShadowFunction<T>(X02ValBf3withA, vA.z, intersectionPointBf3withA.z);

        SoaInt intersectionVal = S02ValBf0onA + S02ValBf1onA + S02ValBf2onA + S02ValBf3onA;
        return intersectionVal;
    }

    // Intersection function for tetrahedron A and vertex B intersecting in 3D
    template< class T >
    typename T::SoaInt FmIntersectionX30(
        const typename T::SoaVector3& tA0, const typename T::SoaVector3& tA1, const typename T::SoaVector3& tA2, const typename T::SoaVector3& tA3,
        const typename T::SoaVector3& vB)
    {
        typedef typename T::SoaVector3 SoaVector3;
        typedef typename T::SoaInt SoaInt;

        SoaVector3 intersectionPointAf0withB, intersectionPointAf1withB, intersectionPointAf2withB, intersectionPointAf3withB;
        SoaInt X20ValAf0withB = FmIntersectionX20<T>(intersectionPointAf0withB, tA0, tA2, tA1, vB);
        SoaInt X20ValAf1withB = FmIntersectionX20<T>(intersectionPointAf1withB, tA1, tA3, tA0, vB);
        SoaInt X20ValAf2withB = FmIntersectionX20<T>(intersectionPointAf2withB, tA2, tA0, tA3, vB);
        SoaInt X20ValAf3withB = FmIntersectionX20<T>(intersectionPointAf3withB, tA3, tA1, tA2, vB);

        SoaInt S20ValBonAf0 = FmShadowFunction<T>(X20ValAf0withB, intersectionPointAf0withB.z, vB.z);
        SoaInt S20ValBonAf1 = FmShadowFunction<T>(X20ValAf1withB, intersectionPointAf1withB.z, vB.z);
        SoaInt S20ValBonAf2 = FmShadowFunction<T>(X20ValAf2withB, intersectionPointAf2withB.z, vB.z);
        SoaInt S20ValBonAf3 = FmShadowFunction<T>(X20ValAf3withB, intersectionPointAf3withB.z, vB.z);

        SoaInt intersectionVal = -S20ValBonAf0 - S20ValBonAf1 - S20ValBonAf2 - S20ValBonAf3;
        return intersectionVal;
    }

    // Returns nonzero value == X02 when face B shadows vertex A
    template< class T >
    static FM_FORCE_INLINE typename T::SoaInt FmShadowS02(
        const typename T::SoaVector3& vA,
        const typename T::SoaVector3& fB0, const typename T::SoaVector3& fB1, const typename T::SoaVector3& fB2)
    {
        typename T::SoaVector3 intersectionPointBwithA;
        typename T::SoaInt X02ValBwithA = FmIntersectionX02<T>(intersectionPointBwithA, vA, fB0, fB1, fB2);
        return FmShadowFunction<T>(X02ValBwithA, vA.z, intersectionPointBwithA.z);
    }

    // Returns nonzero value == X20 when vertex B shadows face A
    template< class T >
    static FM_FORCE_INLINE typename T::SoaInt FmShadowS20(
        const typename T::SoaVector3& fA0, const typename T::SoaVector3& fA1, const typename T::SoaVector3& fA2,
        const typename T::SoaVector3& vB)
    {
        typename T::SoaVector3 intersectionPointAwithB;
        typename T::SoaInt X20ValAwithB = FmIntersectionX20<T>(intersectionPointAwithB, fA0, fA1, fA2, vB);
        return FmShadowFunction<T>(X20ValAwithB, intersectionPointAwithB.z, vB.z);
    }

    template< class T >
    static FM_FORCE_INLINE void
        FmBarycentricCoords(typename T::SoaFloat barycentricCoords[3], const typename T::SoaVector3& pos, const typename T::SoaVector3& triPos0, const typename T::SoaVector3& triPos1, const typename T::SoaVector3& triPos2)
    {
        typedef typename T::SoaVector3 SoaVector3;
        typedef typename T::SoaFloat SoaFloat;

        // Requires pos on triangle for valid result.
        // Compute barycentric values using ratio of sub-triangle area to triangle area.
        // Check for degenerate input and constrain results.
        SoaVector3 u = triPos1 - triPos0;
        SoaVector3 v = triPos2 - triPos0;
        SoaVector3 w = pos - triPos0;

        SoaVector3 uxv = cross(u, v);
        SoaVector3 uxw = cross(u, w);
        SoaVector3 vxw = cross(v, w);

        SoaFloat len_uxv = length(uxv);
        SoaFloat len_uxw = length(uxw);
        SoaFloat len_vxw = length(vxw);

        SoaFloat bary_x, bary_y, bary_z;

        bary_y = select(len_vxw / len_uxv, 1.0f, (len_uxv == 0.0f));
        bary_z = select(len_uxw / len_uxv, 1.0f, (len_uxv == 0.0f));

        bary_y = select(bary_y, 1.0f, (bary_y > 1.0f));
        bary_z = select(bary_z, 1.0f, (bary_z > 1.0f));

        bary_y = select(bary_y, 1.0f - bary_z, (bary_y + bary_z > 1.0f));
        bary_x = 1.0f - bary_y - bary_z;

        barycentricCoords[0] = bary_x;
        barycentricCoords[1] = bary_y;
        barycentricCoords[2] = bary_z;
    }

    // Compute intersection points between two triangles (edge/face and face/edge) and the X intersection value for each.
    // Link start and end points into segments.
    template< class T >
    struct FmSoaTriIntersectionPoints
    {
        typename T::SoaVector3 points[6];  // 3 edge/face + 3 face/edge
        typename T::SoaInt     XVals[6];
        typename T::SoaInt     startOrEndCount[6]; // for each point:
                                                   // if == 0: not involved in intersection
                                                   // if < 0: number of times point is a start vertex
                                                   // if > 0: number of times point is an end vertex
    };

    template<class T>
    typename T::SoaBool FmComputeTriIntersection(FmSoaTriIntersectionPoints<T>* triIntersectionPoints, const typename T::SoaVector3 fA[3], const typename T::SoaVector3 fB[3])
    {
        typedef typename T::SoaVector3 SoaVector3;
        typedef typename T::SoaFloat SoaFloat;
        typedef typename T::SoaInt SoaInt;

        SoaInt X12Val0, X12Val1, X12Val2;
        SoaInt X21Val0, X21Val1, X21Val2;
        SoaVector3 edgeFacePoint0, edgeFacePoint1, edgeFacePoint2;
        SoaVector3 faceEdgePoint0, faceEdgePoint1, faceEdgePoint2;

        X12Val0 = FmIntersectionX12<T>(&edgeFacePoint0, fA[0], fA[1], fB[0], fB[1], fB[2]);
        X12Val1 = FmIntersectionX12<T>(&edgeFacePoint1, fA[1], fA[2], fB[0], fB[1], fB[2]);
        X12Val2 = FmIntersectionX12<T>(&edgeFacePoint2, fA[2], fA[0], fB[0], fB[1], fB[2]);

        X21Val0 = FmIntersectionX21<T>(&faceEdgePoint0, fA[0], fA[1], fA[2], fB[0], fB[1]);
        X21Val1 = FmIntersectionX21<T>(&faceEdgePoint1, fA[0], fA[1], fA[2], fB[1], fB[2]);
        X21Val2 = FmIntersectionX21<T>(&faceEdgePoint2, fA[0], fA[1], fA[2], fB[2], fB[0]);

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

        SoaInt netEndCount12Val0, netEndCount12Val1, netEndCount12Val2;
        SoaInt netEndCount21Val0, netEndCount21Val1, netEndCount21Val2;
        netEndCount12Val0 = -X12Val0;
        netEndCount12Val1 = -X12Val1;
        netEndCount12Val2 = -X12Val2;
        netEndCount21Val0 = X21Val0;
        netEndCount21Val1 = X21Val1;
        netEndCount21Val2 = X21Val2;

        SoaInt isIntersection =
            netEndCount12Val0 |
            netEndCount12Val1 |
            netEndCount12Val2 |
            netEndCount21Val0 |
            netEndCount21Val1 |
            netEndCount21Val2;

        triIntersectionPoints->startOrEndCount[0] = netEndCount12Val0;
        triIntersectionPoints->startOrEndCount[1] = netEndCount12Val1;
        triIntersectionPoints->startOrEndCount[2] = netEndCount12Val2;
        triIntersectionPoints->startOrEndCount[3] = netEndCount21Val0;
        triIntersectionPoints->startOrEndCount[4] = netEndCount21Val1;
        triIntersectionPoints->startOrEndCount[5] = netEndCount21Val2;

        return (isIntersection != SoaInt(0));
    }

    template< class T >
    static inline void FmConvertSoaToAos(bool* pairIntersects, FmTriIntersectionPoints** pTriIntersectionPoints,
        typename T::SoaBool soaPairIntersects, const FmSoaTriIntersectionPoints<T>& soaTriIntersectionPoints, uint num)
    {
        for (uint i = 0; i < num; i++)
        {
            pairIntersects[i] = soaPairIntersects.getSlice(i);

            if (pairIntersects[i])
            {
                FmTriIntersectionPoints& points = *pTriIntersectionPoints[i];
                points.numSegments = 0;
                points.XVals[0] = soaTriIntersectionPoints.XVals[0].getSlice(i);
                points.XVals[1] = soaTriIntersectionPoints.XVals[1].getSlice(i);
                points.XVals[2] = soaTriIntersectionPoints.XVals[2].getSlice(i);
                points.XVals[3] = soaTriIntersectionPoints.XVals[3].getSlice(i);
                points.XVals[4] = soaTriIntersectionPoints.XVals[4].getSlice(i);
                points.XVals[5] = soaTriIntersectionPoints.XVals[5].getSlice(i);

                uint startIdx = 0;
                uint endIdx = 0;
                for (uint p = 0; p < 6; p++)
                {
                    int startOrEndCount = soaTriIntersectionPoints.startOrEndCount[p].getSlice(i);
                    while (startOrEndCount < 0 && startIdx < 3)
                    {
                        points.startPoints[startIdx] = p;
                        points.points[p] = FmGetSlice<T>(soaTriIntersectionPoints.points[p], i);
                        startIdx++;
                        startOrEndCount++;
                        points.numSegments++;
                    }
                    while (startOrEndCount > 0 && endIdx < 3)
                    {
                        points.endPoints[endIdx] = p;
                        points.points[p] = FmGetSlice<T>(soaTriIntersectionPoints.points[p], i);
                        endIdx++;
                        startOrEndCount--;
                    }
                }
            }
        }
    }

}
