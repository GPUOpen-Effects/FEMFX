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

#include "FEMFXCommon.h"

namespace AMD
{
    struct FmTetMeshBuffer;
    struct FmScene;
    struct FmRigidBody;

    struct FmSerializedSceneCounts
    {
        size_t bufferSize;
        uint numTetMeshBuffers;
        uint numRigidBodies;
        uint numGlueConstraints;
        uint numPlaneConstraints;
        uint numRigidBodyAngleConstraints;
    };

    // Allocate a buffer and serialize all tet meshes, rigid bodies, and constraints of the scene into it.
    uint8_t* FmSerializeScene(size_t* serializationBufferSize, const FmScene& scene);

    // Get the number of tet meshes, rigid bodies, and constraints from serialized buffer.
    void FmGetSerializedSceneCounts(FmSerializedSceneCounts* sceneCounts, const uint8_t* pSerializationBuffer);

    // Deserialize tet meshes, rigid bodies, and constraints into a scene.  (Note, will first clear the scene).
    // Returns false if scene is not large enough for all objects.
    bool FmDeserializeScene(FmScene* scene, FmTetMeshBuffer** tetMeshBufferPtrs, FmRigidBody** rigidBodies, const uint8_t* pSerializationBuffer);
}