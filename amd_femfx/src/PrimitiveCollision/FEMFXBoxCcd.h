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
// Continuous collision detection for boxes moving with constant linear and angular
// velocity in timestep
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXTypes.h"
#include "FEMFXRigidBodyState.h"
#include "FEMFXTriCcd.h"

namespace AMD
{

    struct FmBox
    {
        FmRigidBodyState state;
        float            halfWidths[3];
    };

    struct FmBoxFeature
    {
        uint signBits; // zyx signs
        uint axisMask; // 3 bits set = point, 2 set = edge, 1 set = face
        uint dim;      // dimension of face or edge feature

        inline void SetPoint(bool posX, bool posY, bool posZ)
        {
            axisMask = 0x7;
            signBits = ((uint)posX) | (((uint)posY) << 1) | (((uint)posZ) << 2);
            dim = 0;
        }
        inline void SetXEdge(bool posY, bool posZ)
        {
            axisMask = 0x6;
            signBits = (((uint)posY) << 1) | (((uint)posZ) << 2);
            dim = 0;
        }
        inline void SetYEdge(bool posZ, bool posX)
        {
            axisMask = 0x5;
            signBits = ((uint)posX) | (((uint)posZ) << 2);
            dim = 1;
        }
        inline void SetZEdge(bool posX, bool posY)
        {
            axisMask = 0x3;
            signBits = ((uint)posX) | (((uint)posY) << 1);
            dim = 2;
        }
        inline void SetXFace(bool posX)
        {
            axisMask = 0x1;
            signBits = ((uint)posX);
            dim = 0;
        }
        inline void SetYFace(bool posY)
        {
            axisMask = 0x2;
            signBits = (((uint)posY) << 1);
            dim = 1;
        }
        inline void SetZFace(bool posZ)
        {
            axisMask = 0x4;
            signBits = (((uint)posZ) << 2);
            dim = 2;
        }
        inline void SetDim()
        {
            dim = 0;
            dim = (axisMask == 0x2) ? 1 : dim;
            dim = (axisMask == 0x4) ? 2 : dim;
            dim = (axisMask == 0x6) ? 0 : dim;
            dim = (axisMask == 0x5) ? 1 : dim;
            dim = (axisMask == 0x3) ? 2 : dim;
        }
        inline void GetScalars(float& maskX, float& maskY, float& maskZ, float& signX, float& signY, float& signZ) const
        {
            maskX = (axisMask & 0x1) ? 1.0f : 0.0f;
            maskY = (axisMask & 0x2) ? 1.0f : 0.0f;
            maskZ = (axisMask & 0x4) ? 1.0f : 0.0f;
            signX = (signBits & 0x1) ? 1.0f : -1.0f;
            signY = (signBits & 0x2) ? 1.0f : -1.0f;
            signZ = (signBits & 0x4) ? 1.0f : -1.0f;
        }
        inline bool IsValid() const
        {
            return (axisMask == 0x1 && dim == 0) ||
                (axisMask == 0x2 && dim == 1) ||
                (axisMask == 0x4 && dim == 2) ||
                (axisMask == 0x6 && dim == 0) ||
                (axisMask == 0x5 && dim == 1) ||
                (axisMask == 0x3 && dim == 2) ||
                axisMask == 0x7;
        }
        inline bool IsPoint() const
        {
            return axisMask == 0x7;
        }
        inline bool IsEdge() const
        {
            return axisMask == 0x3 || axisMask == 0x5 || axisMask == 0x6;
        }
        inline bool IsFace() const
        {
            return axisMask == 0x1 || axisMask == 0x2 || axisMask == 0x4;
        }
        inline bool operator == (const FmBoxFeature& rhs) const
        {
            return (signBits == rhs.signBits && axisMask == rhs.axisMask && dim == rhs.dim);
        }
    };

    FmAabb FmComputeBoxAabb(const FmBox& box, float deltaTime, float padding);

    void FmBoxFacePointCcd(FmCcdResult* contact,
        const FmBox& box,
        uint boxFaceDim, uint boxFaceSign,
        const FmVector3& pointPos, const FmVector3& pointVel,
        uint j,
        float deltaTime, const FmCcdTerminationConditions& conditions);

    void FmBoxEdgeSegmentCcd(FmCcdResult* contact,
        const FmBox& box,
        const FmBoxFeature& boxFeature,
        const FmVector3& boxEdgePos0BoxSpace, const FmVector3& boxEdgePos1BoxSpace,
        const FmVector3& segmentPos0, const FmVector3& segmentVel0,
        const FmVector3& segmentPos1, const FmVector3& segmentVel1,
        uint j0, uint j1,
        float deltaTime, const FmCcdTerminationConditions& conditions);

    void FmBoxVertexTriCcd(FmCcdResult* contact,
        const FmBox& box, uint boxVertexId, const FmVector3& boxVertexPosBoxSpace, const FmTri& tri,
        float deltaTime, const FmCcdTerminationConditions& conditions);

    void FmBoxTriClosestPoints(FmDistanceResult* contact, const FmBox& box, const FmVector3 triPos[3]);

    bool FmBoxAndTriIntersect(const FmBox& box, const FmVector3 triPos[3]);

    void FmBoxFaceBoxVertexCcd(FmCcdResult* contact,
        const FmBox& boxA, uint boxAFaceDim, uint boxAFaceSign,
        const FmBox & boxB, uint boxBVertexId, const FmVector3& boxBVertexPosBoxSpace,
        float deltaTime, const FmCcdTerminationConditions& conditions);

    void FmBoxVertexBoxFaceCcd(FmCcdResult* contact,
        const FmBox& boxA, uint boxAVertexId, const FmVector3& boxAVertexPosBoxSpace,
        const FmBox& boxB, uint boxBFaceDim, uint boxBFaceSign,
        float deltaTime, const FmCcdTerminationConditions& conditions);

    void FmBoxEdgeBoxEdgeCcd(FmCcdResult* contact,
        const FmBox& boxA, const FmBoxFeature& boxAFeature, const FmVector3& boxAEdgePos0BoxSpace, const FmVector3& boxAEdgePos1BoxSpace,
        const FmBox& boxB, const FmBoxFeature& boxBFeature, const FmVector3& boxBEdgePos0BoxSpace, const FmVector3& boxBEdgePos1BoxSpace,
        float deltaTime, const FmCcdTerminationConditions& conditions);

    bool FmBoxesIntersect(const FmBox& boxA, const FmBox& boxB);

}
