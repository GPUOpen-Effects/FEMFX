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
// Dynamic state of rigid bodies
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXVectormath.h"

namespace AMD
{

    struct FmRigidBodyState
    {
        FmVector3 pos;
        FmQuat    quat;
        FmVector3 vel;
        FmVector3 angVel;

        FmRigidBodyState()
        {
            pos = FmInitVector3(0.0f);
            quat = FmInitQuat(0.0f, 0.0f, 0.0f, 1.0f);
            vel = FmInitVector3(0.0f);
            angVel = FmInitVector3(0.0f);
        }
    };

    // Set mass and body-relative inertial tensor for uniform density box
    static inline FmMatrix3 FmComputeBodyInertiaTensorForBox(float hx, float hy, float hz, float inMass)
    {
        FmMatrix3 bodyInertiaTensor;

        float massDiv12 = inMass / 12.0f;
        float dx = hx * 2.0f;
        float dy = hy * 2.0f;
        float dz = hz * 2.0f;
        float dxSqr = dx * dx;
        float dySqr = dy * dy;
        float dzSqr = dz * dz;
        bodyInertiaTensor.col0 = FmInitVector3(massDiv12 * (dySqr + dzSqr), 0.0f, 0.0f);
        bodyInertiaTensor.col1 = FmInitVector3(0.0f, massDiv12 * (dzSqr + dxSqr), 0.0f);
        bodyInertiaTensor.col2 = FmInitVector3(0.0f, 0.0f, massDiv12 * (dxSqr + dySqr));

        return bodyInertiaTensor;
    }
}
