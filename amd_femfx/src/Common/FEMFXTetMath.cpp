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
// Tetrahedron-specific types and operations for the FEM model
//---------------------------------------------------------------------------------------

#include "FEMFXTypes.h"
#include "FEMFXTetMath.h"
#include "FEMFXSvd3x3.h"

namespace AMD
{
    void FmComputeShapeParams(
        FmTetShapeParams* resultShapeParams,
        const FmVector3& p0,
        const FmVector3& p1,
        const FmVector3& p2,
        const FmVector3& p3)
    {
        FmTetShapeParams shapeParams;

        float x0 = p0.x;
        float y0 = p0.y;
        float z0 = p0.z;
        float x1 = p1.x;
        float y1 = p1.y;
        float z1 = p1.z;
        float x2 = p2.x;
        float y2 = p2.y;
        float z2 = p2.z;
        float x3 = p3.x;
        float y3 = p3.y;
        float z3 = p3.z;

        float x01 = x0 - x1;
        float x02 = x0 - x2;
        float x03 = x0 - x3;
        float x10 = -x01;
        float x12 = x1 - x2;
        float x13 = x1 - x3;
        float x20 = -x02;
        float x21 = -x12;
        float x23 = x2 - x3;
        float x30 = -x03;
        float x31 = -x13;
        float x32 = -x23;

        float y01 = y0 - y1;
        float y02 = y0 - y2;
        float y03 = y0 - y3;
        float y10 = -y01;
        float y12 = y1 - y2;
        float y13 = y1 - y3;
        float y20 = -y02;
        float y21 = -y12;
        float y23 = y2 - y3;
        float y30 = -y03;
        float y31 = -y13;
        float y32 = -y23;

        // determinant of M = [  1  1  1  1 ] = 6.0 x signed volume
        //                    [ p0 p1 p2 p3 ] 

        float det = (x10*y2 + x02*y1 + x21*y0)*z3 + (x01*y3 + x30*y1 + x13*y0)*z2 + (x20*y3 + x03*y2 + x32*y0)*z1 +
            (x12*y3 + x31*y2 + x23*y1)*z0;

        float m00 = (x1*y2 - x2*y1)*z3 + (x3*y1 - x1*y3)*z2 + (x2*y3 - x3*y2)*z1;
        float m01 = y12*z3 + y31*z2 + y23*z1;
        float m02 = x21*z3 + x13*z2 + x32*z1;
        float m03 = x12*y3 + x31*y2 + x23*y1;
        float m10 = (x2*y0 - x0*y2)*z3 + (x0*y3 - x3*y0)*z2 + (x3*y2 - x2*y3)*z0;
        float m11 = y20*z3 + y03*z2 + y32*z0;
        float m12 = x02*z3 + x30*z2 + x23*z0;
        float m13 = x20*y3 + x03*y2 + x32*y0;
        float m20 = (x0*y1 - x1*y0)*z3 + (x3*y0 - x0*y3)*z1 + (x1*y3 - x3*y1)*z0;
        float m21 = y01*z3 + y30*z1 + y13*z0;
        float m22 = x10*z3 + x03*z1 + x31*z0;
        float m23 = x01*y3 + x30*y1 + x13*y0;
        float m30 = (x1*y0 - x0*y1)*z2 + (x0*y2 - x2*y0)*z1 + (x2*y1 - x1*y2)*z0;
        float m31 = y10*z2 + y02*z1 + y21*z0;
        float m32 = x01*z2 + x20*z1 + x12*z0;
        float m33 = x10*y2 + x02*y1 + x21*y0;

        shapeParams.det = det;

        // baryMatrix transforms [x, y, z, 1]^T to barycentric coords

        float invDet = 1.0f / det;

        shapeParams.baryMatrix = FmMatrix4(
            FmVector4(
                m01 * invDet,
                m11 * invDet,
                m21 * invDet,
                m31 * invDet),
            FmVector4(
                m02 * invDet,
                m12 * invDet,
                m22 * invDet,
                m32 * invDet),
            FmVector4(
                m03 * invDet,
                m13 * invDet,
                m23 * invDet,
                m33 * invDet),
            FmVector4(
                m00 * invDet,
                m10 * invDet,
                m20 * invDet,
                m30 * invDet));

        *resultShapeParams = shapeParams;
    }

    // Ratio of maximum edge length to minimum height of vertex from plane containing other vertices.
    // Measure of tetrahedron quality.
    float FmComputeTetAspectRatio(const FmVector3& position0, const FmVector3& position1, const FmVector3& position2, const FmVector3& position3)
    {
        FmVector3 edge0 = position0 - position1;
        FmVector3 edge1 = position0 - position2;
        FmVector3 edge2 = position0 - position3;
        FmVector3 edge3 = position1 - position2;
        FmVector3 edge4 = position1 - position3;
        FmVector3 edge5 = position2 - position3;

        float planeDist0 = dot(edge0, normalize(cross(edge5, edge4)));
        float planeDist1 = dot(edge4, normalize(cross(edge2, edge5)));
        float planeDist2 = dot(edge5, normalize(cross(edge4, edge2)));
        float planeDist3 = dot(edge2, normalize(cross(edge3, edge1)));

        float maxEdgeLength =
            FmMaxFloat(FmMaxFloat(FmMaxFloat(FmMaxFloat(FmMaxFloat(length(edge0), length(edge1)), length(edge2)), length(edge3)), length(edge4)), length(edge5));

        float minPlaneDist =
            FmMinFloat(FmMinFloat(FmMinFloat(planeDist0, planeDist1), planeDist2), planeDist3);

        return maxEdgeLength / minPlaneDist;
    }

    // Ratio of maximum edge length to minimum height of vertex from plane containing other vertices.
    // Measure of tetrahedron quality.
    float FmComputeTetAspectRatio(const FmVector3 positions[4])
    {
        return FmComputeTetAspectRatio(positions[0], positions[1], positions[2], positions[3]);
    }
        
    // If tetrahedron aspect ratio is greater than the specified maximum, modifies the positions to improve aspect ratio, 
    // maintaining the original volume and centroid.
    bool FmEquilateralize(FmVector3* position0, FmVector3* position1, FmVector3* position2, FmVector3* position3, float maxAspectRatio, float minVolume)
    {
        const uint maxIterations = 100;
        const float factor = 0.02f;

        const float minAspectRatio = 1.0f / (sqrtf(2.0f / 3.0f));

        maxAspectRatio = FmMaxFloat(maxAspectRatio, minAspectRatio);

        FmVector3 newp0 = *position0;
        FmVector3 newp1 = *position1;
        FmVector3 newp2 = *position2;
        FmVector3 newp3 = *position3;

        // Get original centroid and volume
        FmVector3 centroid = 0.25f * (newp0 + newp1 + newp2 + newp3);

        FmTetShapeParams shapeParams;
        FmComputeShapeParams(&shapeParams, newp0, newp1, newp2, newp3);
        float volume = shapeParams.GetVolume();

        volume = FmMaxFloat(volume, minVolume);

        bool modified = false;
        for (uint iteration = 0; iteration < maxIterations; iteration++)
        {
            // Form six edges and normals for each face
            FmVector3 edge0 = newp0 - newp1;
            FmVector3 edge1 = newp0 - newp2;
            FmVector3 edge2 = newp0 - newp3;
            FmVector3 edge3 = newp1 - newp2;
            FmVector3 edge4 = newp1 - newp3;
            FmVector3 edge5 = newp2 - newp3;

            FmVector3 planeNormal0 = normalize(cross(edge5, edge4));
            FmVector3 planeNormal1 = normalize(cross(edge2, edge5));
            FmVector3 planeNormal2 = normalize(cross(edge4, edge2));
            FmVector3 planeNormal3 = normalize(cross(edge3, edge1));

            float planeDist0 = dot(edge0, planeNormal0);
            float planeDist1 = dot(edge4, planeNormal1);
            float planeDist2 = dot(edge5, planeNormal2);
            float planeDist3 = dot(edge2, planeNormal3);

            // Compute max edge length and min plane distance, to compute aspect ratio
            float edgeLength0 = length(edge0);
            float edgeLength1 = length(edge1);
            float edgeLength2 = length(edge2);
            float edgeLength3 = length(edge3);
            float edgeLength4 = length(edge4);
            float edgeLength5 = length(edge5);

            float maxEdgeLength =
                FmMaxFloat(FmMaxFloat(FmMaxFloat(FmMaxFloat(FmMaxFloat(edgeLength0, edgeLength1), edgeLength2), edgeLength3), edgeLength4), edgeLength5);

            float minPlaneDist = planeDist0;
            uint minPlaneIdx = 0;
            if (planeDist1 < minPlaneDist)
            {
                minPlaneDist = planeDist1;
                minPlaneIdx = 1;
            }
            if (planeDist2 < minPlaneDist)
            {
                minPlaneDist = planeDist2;
                minPlaneIdx = 2;
            }
            if (planeDist3 < minPlaneDist)
            {
                minPlaneDist = planeDist3;
                minPlaneIdx = 3;
            }

            float aspectRatio = maxEdgeLength / minPlaneDist;

            if (aspectRatio > maxAspectRatio)
            {
                modified = true;

                // Adjust the tetrahedron shape

                // First increase the plane distance
                float distDelta = fabsf(maxEdgeLength / maxAspectRatio - minPlaneDist);
                if (minPlaneIdx == 0)
                {
                    newp0 += planeNormal0 * distDelta * factor;
                }
                else if (minPlaneIdx == 1)
                {
                    newp1 += planeNormal1 * distDelta * factor;
                }
                else if (minPlaneIdx == 2)
                {
                    newp2 += planeNormal2 * distDelta * factor;
                }
                else
                {
                    newp3 -= planeNormal3 * distDelta * factor;
                }

                // Shift edge lengths closer to the average
                float edgeLengthTotal = edgeLength0 + edgeLength1 + edgeLength2 + edgeLength3 + edgeLength4 + edgeLength5;
                float edgeLengthAverage = edgeLengthTotal / 6.0f;

                FmVector3 delta, avgPos;

                edge0 = newp0 - newp1;
                delta = edge0 * (edgeLengthAverage / length(edge0) - 1.0f) * factor * 0.5f;
                newp0 += delta;
                newp1 -= delta;

                edge1 = newp0 - newp2;
                delta = edge1 * (edgeLengthAverage / length(edge1) - 1.0f) * factor * 0.5f;
                newp0 += delta;
                newp2 -= delta;

                edge2 = newp0 - newp3;
                delta = edge2 * (edgeLengthAverage / length(edge2) - 1.0f) * factor * 0.5f;
                newp0 += delta;
                newp3 -= delta;

                edge3 = newp1 - newp2;
                delta = edge3 * (edgeLengthAverage / length(edge3) - 1.0f) * factor * 0.5f;
                newp1 += delta;
                newp2 -= delta;

                edge4 = newp1 - newp3;
                delta = edge4 * (edgeLengthAverage / length(edge4) - 1.0f) * factor * 0.5f;
                newp1 += delta;
                newp3 -= delta;

                edge5 = newp2 - newp3;
                delta = edge5 * (edgeLengthAverage / length(edge5) - 1.0f) * factor * 0.5f;
                newp2 += delta;
                newp3 -= delta;
            }
            else
            {
                break;
            }
        }

        if (modified)
        {
            FmVector3 newCentroid = 0.25f * (newp0 + newp1 + newp2 + newp3);

            FmComputeShapeParams(&shapeParams, newp0, newp1, newp2, newp3);
            float newVolume = shapeParams.GetVolume();

            float volumeScale = powf(volume/newVolume, 1.0f / 3.0f);

            newp0 = centroid + (newp0 - newCentroid) * volumeScale;
            newp1 = centroid + (newp1 - newCentroid) * volumeScale;
            newp2 = centroid + (newp2 - newCentroid) * volumeScale;
            newp3 = centroid + (newp3 - newCentroid) * volumeScale;

            *position0 = newp0;
            *position1 = newp1;
            *position2 = newp2;
            *position3 = newp3;
        }

        return modified;
    }

    void FmComputeTetStrainMatrix(FmTetStrainMatrix* strainMat, const FmTetShapeParams& shapeParams)
    {
        float a0 = shapeParams.GetA0();
        float a1 = shapeParams.GetA1();
        float a2 = shapeParams.GetA2();
        float a3 = shapeParams.GetA3();
        float b0 = shapeParams.GetB0();
        float b1 = shapeParams.GetB1();
        float b2 = shapeParams.GetB2();
        float b3 = shapeParams.GetB3();
        float c0 = shapeParams.GetC0();
        float c1 = shapeParams.GetC1();
        float c2 = shapeParams.GetC2();
        float c3 = shapeParams.GetC3();

        strainMat->Be00 = FmMatrix3(FmVector3(a0, 0.0f, 0.0f), FmVector3(0.0f, b0, 0.0f), FmVector3(0.0f, 0.0f, c0));
        strainMat->Be01 = FmMatrix3(FmVector3(a1, 0.0f, 0.0f), FmVector3(0.0f, b1, 0.0f), FmVector3(0.0f, 0.0f, c1));
        strainMat->Be02 = FmMatrix3(FmVector3(a2, 0.0f, 0.0f), FmVector3(0.0f, b2, 0.0f), FmVector3(0.0f, 0.0f, c2));
        strainMat->Be03 = FmMatrix3(FmVector3(a3, 0.0f, 0.0f), FmVector3(0.0f, b3, 0.0f), FmVector3(0.0f, 0.0f, c3));
        strainMat->Be10 = FmMatrix3(FmVector3(b0, 0.0f, c0), FmVector3(a0, c0, 0.0f), FmVector3(0.0f, b0, a0));
        strainMat->Be11 = FmMatrix3(FmVector3(b1, 0.0f, c1), FmVector3(a1, c1, 0.0f), FmVector3(0.0f, b1, a1));
        strainMat->Be12 = FmMatrix3(FmVector3(b2, 0.0f, c2), FmVector3(a2, c2, 0.0f), FmVector3(0.0f, b2, a2));
        strainMat->Be13 = FmMatrix3(FmVector3(b3, 0.0f, c3), FmVector3(a3, c3, 0.0f), FmVector3(0.0f, b3, a3));
    }

    void FmComputeTetStrain(
        FmVector3 strain[2],
        const FmTetShapeParams& shapeParams,
        const FmVector3& unrotatedOffset0,
        const FmVector3& unrotatedOffset1,
        const FmVector3& unrotatedOffset2,
        const FmVector3& unrotatedOffset3)
    {
        float a0 = shapeParams.GetA0();
        float a1 = shapeParams.GetA1();
        float a2 = shapeParams.GetA2();
        float a3 = shapeParams.GetA3();
        float b0 = shapeParams.GetB0();
        float b1 = shapeParams.GetB1();
        float b2 = shapeParams.GetB2();
        float b3 = shapeParams.GetB3();
        float c0 = shapeParams.GetC0();
        float c1 = shapeParams.GetC1();
        float c2 = shapeParams.GetC2();
        float c3 = shapeParams.GetC3();

        float x0 = unrotatedOffset0.x;
        float y0 = unrotatedOffset0.y;
        float z0 = unrotatedOffset0.z;
        float x1 = unrotatedOffset1.x;
        float y1 = unrotatedOffset1.y;
        float z1 = unrotatedOffset1.z;
        float x2 = unrotatedOffset2.x;
        float y2 = unrotatedOffset2.y;
        float z2 = unrotatedOffset2.z;
        float x3 = unrotatedOffset3.x;
        float y3 = unrotatedOffset3.y;
        float z3 = unrotatedOffset3.z;        
        
        strain[0].x = a3*x3 + a2*x2 + a1*x1 + a0*x0;
        strain[0].y = b3*y3 + b2*y2 + b1*y1 + b0*y0;
        strain[0].z = c3*z3 + c2*z2 + c1*z1 + c0*z0;
        strain[1].x = a3*y3 + a2*y2 + a1*y1 + a0*y0 + b3*x3 + b2*x2 + b1*x1 + b0*x0;
        strain[1].y = b3*z3 + b2*z2 + b1*z1 + b0*z0 + c3*y3 + c2*y2 + c1*y1 + c0*y0;
        strain[1].z = a3*z3 + a2*z2 + a1*z1 + a0*z0 + c3*x3 + c2*x2 + c1*x1 + c0*x0;
    }

    void FmComputeTetStrainMatrixTransposedSubmats(FmTetStrainMatrix* strainMat, const FmTetShapeParams& shapeParams)
    {
        float a0 = shapeParams.GetA0();
        float a1 = shapeParams.GetA1();
        float a2 = shapeParams.GetA2();
        float a3 = shapeParams.GetA3();
        float b0 = shapeParams.GetB0();
        float b1 = shapeParams.GetB1();
        float b2 = shapeParams.GetB2();
        float b3 = shapeParams.GetB3();
        float c0 = shapeParams.GetC0();
        float c1 = shapeParams.GetC1();
        float c2 = shapeParams.GetC2();
        float c3 = shapeParams.GetC3();

        strainMat->Be00 = transpose(FmMatrix3(FmVector3(a0, 0.0f, 0.0f), FmVector3(0.0f, b0, 0.0f), FmVector3(0.0f, 0.0f, c0)));
        strainMat->Be01 = transpose(FmMatrix3(FmVector3(a1, 0.0f, 0.0f), FmVector3(0.0f, b1, 0.0f), FmVector3(0.0f, 0.0f, c1)));
        strainMat->Be02 = transpose(FmMatrix3(FmVector3(a2, 0.0f, 0.0f), FmVector3(0.0f, b2, 0.0f), FmVector3(0.0f, 0.0f, c2)));
        strainMat->Be03 = transpose(FmMatrix3(FmVector3(a3, 0.0f, 0.0f), FmVector3(0.0f, b3, 0.0f), FmVector3(0.0f, 0.0f, c3)));
        strainMat->Be10 = transpose(FmMatrix3(FmVector3(b0, 0.0f, c0), FmVector3(a0, c0, 0.0f), FmVector3(0.0f, b0, a0)));
        strainMat->Be11 = transpose(FmMatrix3(FmVector3(b1, 0.0f, c1), FmVector3(a1, c1, 0.0f), FmVector3(0.0f, b1, a1)));
        strainMat->Be12 = transpose(FmMatrix3(FmVector3(b2, 0.0f, c2), FmVector3(a2, c2, 0.0f), FmVector3(0.0f, b2, a2)));
        strainMat->Be13 = transpose(FmMatrix3(FmVector3(b3, 0.0f, c3), FmVector3(a3, c3, 0.0f), FmVector3(0.0f, b3, a3)));
    }

    void FmComputeTetStressMatrix(
        FmTetStressMatrix* stressMat,
        const FmVector4& baryMatrixCol0,
        const FmVector4& baryMatrixCol1,
        const FmVector4& baryMatrixCol2,
        float youngsModulus,
        float poissonsRatio)
    {
        float a0 = baryMatrixCol0.x;
        float a1 = baryMatrixCol0.y;
        float a2 = baryMatrixCol0.z;
        float a3 = baryMatrixCol0.w;
        float b0 = baryMatrixCol1.x;
        float b1 = baryMatrixCol1.y;
        float b2 = baryMatrixCol1.z;
        float b3 = baryMatrixCol1.w;
        float c0 = baryMatrixCol2.x;
        float c1 = baryMatrixCol2.y;
        float c2 = baryMatrixCol2.z;
        float c3 = baryMatrixCol2.w;

        float scale = (youngsModulus / ((1.0f + poissonsRatio)*(1.0f - 2.0f * poissonsRatio)));

        float v0 = scale * (1.0f - poissonsRatio);
        float v1 = scale * poissonsRatio;
        float v2 = scale * (0.5f - poissonsRatio);

        float a0_v0 = a0 * v0;
        float a0_v1 = a0 * v1;
        float a0_v2 = a0 * v2;
        float a1_v0 = a1 * v0;
        float a1_v1 = a1 * v1;
        float a1_v2 = a1 * v2;
        float a2_v0 = a2 * v0;
        float a2_v1 = a2 * v1;
        float a2_v2 = a2 * v2;
        float a3_v0 = a3 * v0;
        float a3_v1 = a3 * v1;
        float a3_v2 = a3 * v2;
        float b0_v0 = b0 * v0;
        float b0_v1 = b0 * v1;
        float b0_v2 = b0 * v2;
        float b1_v0 = b1 * v0;
        float b1_v1 = b1 * v1;
        float b1_v2 = b1 * v2;
        float b2_v0 = b2 * v0;
        float b2_v1 = b2 * v1;
        float b2_v2 = b2 * v2;
        float b3_v0 = b3 * v0;
        float b3_v1 = b3 * v1;
        float b3_v2 = b3 * v2;
        float c0_v0 = c0 * v0;
        float c0_v1 = c0 * v1;
        float c0_v2 = c0 * v2;
        float c1_v0 = c1 * v0;
        float c1_v1 = c1 * v1;
        float c1_v2 = c1 * v2;
        float c2_v0 = c2 * v0;
        float c2_v1 = c2 * v1;
        float c2_v2 = c2 * v2;
        float c3_v0 = c3 * v0;
        float c3_v1 = c3 * v1;
        float c3_v2 = c3 * v2;

        stressMat->EBe00 = FmMatrix3(FmVector3(a0_v0, a0_v1, a0_v1), FmVector3(b0_v1, b0_v0, b0_v1), FmVector3(c0_v1, c0_v1, c0_v0));
        stressMat->EBe01 = FmMatrix3(FmVector3(a1_v0, a1_v1, a1_v1), FmVector3(b1_v1, b1_v0, b1_v1), FmVector3(c1_v1, c1_v1, c1_v0));
        stressMat->EBe02 = FmMatrix3(FmVector3(a2_v0, a2_v1, a2_v1), FmVector3(b2_v1, b2_v0, b2_v1), FmVector3(c2_v1, c2_v1, c2_v0));
        stressMat->EBe03 = FmMatrix3(FmVector3(a3_v0, a3_v1, a3_v1), FmVector3(b3_v1, b3_v0, b3_v1), FmVector3(c3_v1, c3_v1, c3_v0));
        stressMat->EBe10 = FmMatrix3(FmVector3(b0_v2, 0.0f, c0_v2), FmVector3(a0_v2, c0_v2, 0.0f), FmVector3(0.0f, b0_v2, a0_v2));
        stressMat->EBe11 = FmMatrix3(FmVector3(b1_v2, 0.0f, c1_v2), FmVector3(a1_v2, c1_v2, 0.0f), FmVector3(0.0f, b1_v2, a1_v2));
        stressMat->EBe12 = FmMatrix3(FmVector3(b2_v2, 0.0f, c2_v2), FmVector3(a2_v2, c2_v2, 0.0f), FmVector3(0.0f, b2_v2, a2_v2));
        stressMat->EBe13 = FmMatrix3(FmVector3(b3_v2, 0.0f, c3_v2), FmVector3(a3_v2, c3_v2, 0.0f), FmVector3(0.0f, b3_v2, a3_v2));
    }

    void FmComputeTetStressAndStiffnessMatrix(
        FmTetStressMatrix* stressMat, 
        FmTetStiffnessMatrix* stiffnessMat, 
        const FmVector4& baryMatrixCol0,
        const FmVector4& baryMatrixCol1,
        const FmVector4& baryMatrixCol2,
        float volume,
        float youngsModulus,
        float poissonsRatio)
    {
        float a0 = baryMatrixCol0.x;
        float a1 = baryMatrixCol0.y;
        float a2 = baryMatrixCol0.z;
        float a3 = baryMatrixCol0.w;
        float b0 = baryMatrixCol1.x;
        float b1 = baryMatrixCol1.y;
        float b2 = baryMatrixCol1.z;
        float b3 = baryMatrixCol1.w;
        float c0 = baryMatrixCol2.x;
        float c1 = baryMatrixCol2.y;
        float c2 = baryMatrixCol2.z;
        float c3 = baryMatrixCol2.w;

        float scale = (youngsModulus / ((1.0f + poissonsRatio)*(1.0f - 2.0f * poissonsRatio)));

        float v0 = scale * (1.0f - poissonsRatio);
        float v1 = scale * poissonsRatio;
        float v2 = scale * (0.5f - poissonsRatio);

        float a0_v0 = a0*v0;
        float a0_v1 = a0*v1;
        float a0_v2 = a0*v2;
        float a1_v0 = a1*v0;
        float a1_v1 = a1*v1;
        float a1_v2 = a1*v2;
        float a2_v0 = a2*v0;
        float a2_v1 = a2*v1;
        float a2_v2 = a2*v2;
        float a3_v0 = a3*v0;
        float a3_v1 = a3*v1;
        float a3_v2 = a3*v2;
        float b0_v0 = b0*v0;
        float b0_v1 = b0*v1;
        float b0_v2 = b0*v2;
        float b1_v0 = b1*v0;
        float b1_v1 = b1*v1;
        float b1_v2 = b1*v2;
        float b2_v0 = b2*v0;
        float b2_v1 = b2*v1;
        float b2_v2 = b2*v2;
        float b3_v0 = b3*v0;
        float b3_v1 = b3*v1;
        float b3_v2 = b3*v2;
        float c0_v0 = c0*v0;
        float c0_v1 = c0*v1;
        float c0_v2 = c0*v2;
        float c1_v0 = c1*v0;
        float c1_v1 = c1*v1;
        float c1_v2 = c1*v2;
        float c2_v0 = c2*v0;
        float c2_v1 = c2*v1;
        float c2_v2 = c2*v2;
        float c3_v0 = c3*v0;
        float c3_v1 = c3*v1;
        float c3_v2 = c3*v2;

        stressMat->EBe00 = FmMatrix3(FmVector3(a0_v0, a0_v1, a0_v1), FmVector3(b0_v1, b0_v0, b0_v1), FmVector3(c0_v1, c0_v1, c0_v0));
        stressMat->EBe01 = FmMatrix3(FmVector3(a1_v0, a1_v1, a1_v1), FmVector3(b1_v1, b1_v0, b1_v1), FmVector3(c1_v1, c1_v1, c1_v0));
        stressMat->EBe02 = FmMatrix3(FmVector3(a2_v0, a2_v1, a2_v1), FmVector3(b2_v1, b2_v0, b2_v1), FmVector3(c2_v1, c2_v1, c2_v0));
        stressMat->EBe03 = FmMatrix3(FmVector3(a3_v0, a3_v1, a3_v1), FmVector3(b3_v1, b3_v0, b3_v1), FmVector3(c3_v1, c3_v1, c3_v0));
        stressMat->EBe10 = FmMatrix3(FmVector3(b0_v2, 0.0f, c0_v2), FmVector3(a0_v2, c0_v2, 0.0f), FmVector3(0.0f, b0_v2, a0_v2));
        stressMat->EBe11 = FmMatrix3(FmVector3(b1_v2, 0.0f, c1_v2), FmVector3(a1_v2, c1_v2, 0.0f), FmVector3(0.0f, b1_v2, a1_v2));
        stressMat->EBe12 = FmMatrix3(FmVector3(b2_v2, 0.0f, c2_v2), FmVector3(a2_v2, c2_v2, 0.0f), FmVector3(0.0f, b2_v2, a2_v2));
        stressMat->EBe13 = FmMatrix3(FmVector3(b3_v2, 0.0f, c3_v2), FmVector3(a3_v2, c3_v2, 0.0f), FmVector3(0.0f, b3_v2, a3_v2));

        stiffnessMat->Ke_diag0 = volume * FmMatrix3(FmVector3(c0*c0_v2 + b0*b0_v2 + a0*a0_v0, a0*b0_v2 + a0*b0_v1, a0*c0_v2 + a0*c0_v1), FmVector3(a0*b0_v2 + a0*b0_v1, c0*c0_v2 + a0*a0_v2 + b0*b0_v0, b0*c0_v2 + b0*c0_v1), FmVector3(a0*c0_v2 + a0*c0_v1, b0*c0_v2 + b0*c0_v1, b0*b0_v2 + a0*a0_v2 + c0*c0_v0));
        stiffnessMat->Ke_diag1 = volume * FmMatrix3(FmVector3(c1*c1_v2 + b1*b1_v2 + a1*a1_v0, a1*b1_v2 + a1*b1_v1, a1*c1_v2 + a1*c1_v1), FmVector3(a1*b1_v2 + a1*b1_v1, c1*c1_v2 + a1*a1_v2 + b1*b1_v0, b1*c1_v2 + b1*c1_v1), FmVector3(a1*c1_v2 + a1*c1_v1, b1*c1_v2 + b1*c1_v1, b1*b1_v2 + a1*a1_v2 + c1*c1_v0));
        stiffnessMat->Ke_diag2 = volume * FmMatrix3(FmVector3(c2*c2_v2 + b2*b2_v2 + a2*a2_v0, a2*b2_v2 + a2*b2_v1, a2*c2_v2 + a2*c2_v1), FmVector3(a2*b2_v2 + a2*b2_v1, c2*c2_v2 + a2*a2_v2 + b2*b2_v0, b2*c2_v2 + b2*c2_v1), FmVector3(a2*c2_v2 + a2*c2_v1, b2*c2_v2 + b2*c2_v1, b2*b2_v2 + a2*a2_v2 + c2*c2_v0));
        stiffnessMat->Ke_diag3 = volume * FmMatrix3(FmVector3(c3*c3_v2 + b3*b3_v2 + a3*a3_v0, a3*b3_v2 + a3*b3_v1, a3*c3_v2 + a3*c3_v1), FmVector3(a3*b3_v2 + a3*b3_v1, c3*c3_v2 + a3*a3_v2 + b3*b3_v0, b3*c3_v2 + b3*c3_v1), FmVector3(a3*c3_v2 + a3*c3_v1, b3*c3_v2 + b3*c3_v1, b3*b3_v2 + a3*a3_v2 + c3*c3_v0));
        stiffnessMat->Ke_lower0 = volume * FmMatrix3(FmVector3(c0*c1_v2 + b0*b1_v2 + a0*a1_v0, a1*b0_v2 + a0*b1_v1, a1*c0_v2 + a0*c1_v1), FmVector3(a0*b1_v2 + a1*b0_v1, c0*c1_v2 + a0*a1_v2 + b0*b1_v0, b1*c0_v2 + b0*c1_v1), FmVector3(a0*c1_v2 + a1*c0_v1, b0*c1_v2 + b1*c0_v1, b0*b1_v2 + a0*a1_v2 + c0*c1_v0));
        stiffnessMat->Ke_lower1 = volume * FmMatrix3(FmVector3(c0*c2_v2 + b0*b2_v2 + a0*a2_v0, a2*b0_v2 + a0*b2_v1, a2*c0_v2 + a0*c2_v1), FmVector3(a0*b2_v2 + a2*b0_v1, c0*c2_v2 + a0*a2_v2 + b0*b2_v0, b2*c0_v2 + b0*c2_v1), FmVector3(a0*c2_v2 + a2*c0_v1, b0*c2_v2 + b2*c0_v1, b0*b2_v2 + a0*a2_v2 + c0*c2_v0));
        stiffnessMat->Ke_lower2 = volume * FmMatrix3(FmVector3(c0*c3_v2 + b0*b3_v2 + a0*a3_v0, a3*b0_v2 + a0*b3_v1, a3*c0_v2 + a0*c3_v1), FmVector3(a0*b3_v2 + a3*b0_v1, c0*c3_v2 + a0*a3_v2 + b0*b3_v0, b3*c0_v2 + b0*c3_v1), FmVector3(a0*c3_v2 + a3*c0_v1, b0*c3_v2 + b3*c0_v1, b0*b3_v2 + a0*a3_v2 + c0*c3_v0));
        stiffnessMat->Ke_lower3 = volume * FmMatrix3(FmVector3(c1*c2_v2 + b1*b2_v2 + a1*a2_v0, a2*b1_v2 + a1*b2_v1, a2*c1_v2 + a1*c2_v1), FmVector3(a1*b2_v2 + a2*b1_v1, c1*c2_v2 + a1*a2_v2 + b1*b2_v0, b2*c1_v2 + b1*c2_v1), FmVector3(a1*c2_v2 + a2*c1_v1, b1*c2_v2 + b2*c1_v1, b1*b2_v2 + a1*a2_v2 + c1*c2_v0));
        stiffnessMat->Ke_lower4 = volume * FmMatrix3(FmVector3(c1*c3_v2 + b1*b3_v2 + a1*a3_v0, a3*b1_v2 + a1*b3_v1, a3*c1_v2 + a1*c3_v1), FmVector3(a1*b3_v2 + a3*b1_v1, c1*c3_v2 + a1*a3_v2 + b1*b3_v0, b3*c1_v2 + b1*c3_v1), FmVector3(a1*c3_v2 + a3*c1_v1, b1*c3_v2 + b3*c1_v1, b1*b3_v2 + a1*a3_v2 + c1*c3_v0));
        stiffnessMat->Ke_lower5 = volume * FmMatrix3(FmVector3(c2*c3_v2 + b2*b3_v2 + a2*a3_v0, a3*b2_v2 + a2*b3_v1, a3*c2_v2 + a2*c3_v1), FmVector3(a2*b3_v2 + a3*b2_v1, c2*c3_v2 + a2*a3_v2 + b2*b3_v0, b3*c2_v2 + b2*c3_v1), FmVector3(a2*c3_v2 + a3*c2_v1, b2*c3_v2 + b3*c2_v1, b2*b3_v2 + a2*a3_v2 + c2*c3_v0));
    }

    void FmComputeRotatedStiffnessMatrix(
        FmTetRotatedStiffnessMatrix* rotatedStiffnessMat,
        const FmTetStiffnessMatrix& stiffnessMat,
        const FmVector3 tetRestPosition0,
        const FmVector3 tetRestPosition1,
        const FmVector3 tetRestPosition2,
        const FmVector3 tetRestPosition3,
        const FmMatrix3& tetRotation)
    {
        FmMatrix3 ReKe[4][4];

        FmMatrix3 tetRotationInv = transpose(tetRotation);

        ReKe[0][0] = mul(tetRotation, stiffnessMat.Ke_diag0);
        ReKe[0][1] = mul(tetRotation, transpose(stiffnessMat.Ke_lower0));
        ReKe[0][2] = mul(tetRotation, transpose(stiffnessMat.Ke_lower1));
        ReKe[0][3] = mul(tetRotation, transpose(stiffnessMat.Ke_lower2));
        ReKe[1][0] = mul(tetRotation, stiffnessMat.Ke_lower0);
        ReKe[1][1] = mul(tetRotation, stiffnessMat.Ke_diag1);
        ReKe[1][2] = mul(tetRotation, transpose(stiffnessMat.Ke_lower3));
        ReKe[1][3] = mul(tetRotation, transpose(stiffnessMat.Ke_lower4));
        ReKe[2][0] = mul(tetRotation, stiffnessMat.Ke_lower1);
        ReKe[2][1] = mul(tetRotation, stiffnessMat.Ke_lower3);
        ReKe[2][2] = mul(tetRotation, stiffnessMat.Ke_diag2);
        ReKe[2][3] = mul(tetRotation, transpose(stiffnessMat.Ke_lower5));
        ReKe[3][0] = mul(tetRotation, stiffnessMat.Ke_lower2);
        ReKe[3][1] = mul(tetRotation, stiffnessMat.Ke_lower4);
        ReKe[3][2] = mul(tetRotation, stiffnessMat.Ke_lower5);
        ReKe[3][3] = mul(tetRotation, stiffnessMat.Ke_diag3);

        FmVector3 vec0 = -tetRestPosition0;
        FmVector3 vec1 = -tetRestPosition1;
        FmVector3 vec2 = -tetRestPosition2;
        FmVector3 vec3 = -tetRestPosition3;

        rotatedStiffnessMat->f0eprime0 = mul(ReKe[0][0], vec0) + mul(ReKe[0][1], vec1) + mul(ReKe[0][2], vec2) + mul(ReKe[0][3], vec3);
        rotatedStiffnessMat->f0eprime1 = mul(ReKe[1][0], vec0) + mul(ReKe[1][1], vec1) + mul(ReKe[1][2], vec2) + mul(ReKe[1][3], vec3);
        rotatedStiffnessMat->f0eprime2 = mul(ReKe[2][0], vec0) + mul(ReKe[2][1], vec1) + mul(ReKe[2][2], vec2) + mul(ReKe[2][3], vec3);
        rotatedStiffnessMat->f0eprime3 = mul(ReKe[3][0], vec0) + mul(ReKe[3][1], vec1) + mul(ReKe[3][2], vec2) + mul(ReKe[3][3], vec3);

        rotatedStiffnessMat->Keprime_diag0 = mul(ReKe[0][0], tetRotationInv);
        rotatedStiffnessMat->Keprime_lower0 = mul(ReKe[1][0], tetRotationInv);
        rotatedStiffnessMat->Keprime_lower1 = mul(ReKe[2][0], tetRotationInv);
        rotatedStiffnessMat->Keprime_lower2 = mul(ReKe[3][0], tetRotationInv);
        rotatedStiffnessMat->Keprime_diag1 = mul(ReKe[1][1], tetRotationInv);
        rotatedStiffnessMat->Keprime_lower3 = mul(ReKe[2][1], tetRotationInv);
        rotatedStiffnessMat->Keprime_lower4 = mul(ReKe[3][1], tetRotationInv);

        rotatedStiffnessMat->Keprime_diag2 = mul(ReKe[2][2], tetRotationInv);
        rotatedStiffnessMat->Keprime_lower5 = mul(ReKe[3][2], tetRotationInv);

        rotatedStiffnessMat->Keprime_diag3 = mul(ReKe[3][3], tetRotationInv);
    }

    static FM_FORCE_INLINE float FmFrobeniusNorm(const FmMatrix3& mat)
    {
        float m00 = mat.getElem(0, 0);
        float m01 = mat.getElem(1, 0);
        float m02 = mat.getElem(2, 0);
        float m10 = mat.getElem(0, 1);
        float m11 = mat.getElem(1, 1);
        float m12 = mat.getElem(2, 1);
        float m20 = mat.getElem(0, 2);
        float m21 = mat.getElem(1, 2);
        float m22 = mat.getElem(2, 2);

        return sqrtf(
            m00*m00 +
            m01*m01 +
            m02*m02 +
            m10*m10 +
            m11*m11 +
            m12*m12 +
            m20*m20 +
            m21*m21 +
            m22*m22);
    }

    // Reference: Higham, "Computing the Polar Decomposition -- with Applications"
    FmMatrix3 FmComputeTetRotationPolarDecomp(
        const FmVector3& tetDeformedPosition0,
        const FmVector3& tetDeformedPosition1,
        const FmVector3& tetDeformedPosition2,
        const FmVector3& tetDeformedPosition3,
        const FmMatrix4& tetRestBaryMatrix)
    {
        // Deformation of any point in tetrahedron = Barycentric2DeformedPos * OriginalPos2Barycentric
        FmMatrix4 deformationTransform = mul(FmMatrix4(
            FmVector4(tetDeformedPosition0, 1.0f),
            FmVector4(tetDeformedPosition1, 1.0f),
            FmVector4(tetDeformedPosition2, 1.0f),
            FmVector4(tetDeformedPosition3, 1.0f)), tetRestBaryMatrix);

        FmMatrix3 U = deformationTransform.getUpper3x3();

        const int numIterations = 15;
        for (int i = 0; i < numIterations; i++)
        {
            float scale = sqrtf(FmFrobeniusNorm(inverse(U)) / FmFrobeniusNorm(U));
            U = (scale * U + (1.0f / scale) * inverse(transpose(U))) * 0.5f;
        }

        return U;
    }

    FmMatrix3 FmComputeTetRotationPolarDecompSvd(
        const FmVector3& tetDeformedPosition0,
        const FmVector3& tetDeformedPosition1,
        const FmVector3& tetDeformedPosition2,
        const FmVector3& tetDeformedPosition3,
        const FmMatrix4& tetRestBaryMatrix)
    {
        // Deformation of any point in tetrahedron = Barycentric2DeformedPos * OriginalPos2Barycentric
        FmMatrix4 deformationTransform = mul(FmMatrix4(
            FmVector4(tetDeformedPosition0, 1.0f),
            FmVector4(tetDeformedPosition1, 1.0f),
            FmVector4(tetDeformedPosition2, 1.0f),
            FmVector4(tetDeformedPosition3, 1.0f)), tetRestBaryMatrix);

        FmMatrix3 U, V;
        FmVector3 sigma;
        FmSvd3x3(&U, &sigma, &V, deformationTransform.getUpper3x3());

        return mul(U, transpose(V));
    }
}
