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

#pragma once

#include "FEMFXCommonInternal.h"
#include "FEMFXBvh.h"

namespace AMD
{
    // Callback on leaf pair intersection
    typedef void(*FmBvhLeafPairCallback) (void* userData, uint primIdA, uint primIdB);

    // CCD query between BVH and itself
    void FmBvhSelfCcd(FmBvhLeafPairCallback leafPairCallback, void* userData, const FmBvh& bvh, float timestep);

    // CCD query between pair of BVHs
    void FmBvhPairCcd(FmBvhLeafPairCallback leafPairCallback, void* userData, const FmBvh& bvhA, const FmBvh& bvhB, float timestep);

    // CCD query between object BV and BVH
    void FmObjectBvhCcd(FmBvhLeafPairCallback leafPairCallback, void* userData, const FmAabb& objectBvA, uint objectIdA, const FmBvh& bvh, float timestep);
}