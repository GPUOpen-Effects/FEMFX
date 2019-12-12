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
// Pool of memory buffers used by worker threads for temporary allocations
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXCommonInternal.h"

namespace AMD
{
    // Per-thread memory used for temporary buffers when colliding mesh pairs
    struct FmThreadTempMemoryBuffer
    {
        uint8_t* pBuffer;
        size_t bufferNumBytes;

        uint8_t**                      buffers;
        uint                           numBuffers;
        size_t                         numBytesPerBuffer;
        uint                           maxTetMeshBufferFeatures;
    };

    struct FmThreadTempMemoryBufferSetupParams
    {
        uint numWorkerThreads;
        uint maxTetMeshBufferFeatures;          // max verts, tets, or exterior faces of tet mesh buffer
        size_t numBytesPerThread;               // if nonzero, determines size of buffer per thread, otherwise it's calculated from other parameters

        FmThreadTempMemoryBufferSetupParams()
        {
            numWorkerThreads = 0;
            maxTetMeshBufferFeatures = 0;
            numBytesPerThread = 0;
        }
    };

    // Get total bytes needed for FmThreadTempMemoryBuffer struct and all arrays.
    size_t FmGetThreadTempMemoryBufferSize(const FmThreadTempMemoryBufferSetupParams& params);

    // Suballocate memory from pBuffer to create FmThreadTempMemoryBuffer and arrays.
    // NOTE: pBuffer must be 64-byte-aligned
    FmThreadTempMemoryBuffer* FmSetupThreadTempMemoryBuffer(const FmThreadTempMemoryBufferSetupParams& params, uint8_t*& pBuffer64ByteAligned, size_t bufferNumBytes);
}
