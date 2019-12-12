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

#include "FEMFXThreadTempMemory.h"
#include "FEMFXCollisionPairData.h"
#include "FEMFXConstraints.h"
#include "FEMFXTetMesh.h"
#include "FEMFXMpcgSolver.h"
#include "FEMFXFracture.h"

namespace AMD
{

    static inline size_t FmGetNumBytes(const FmThreadTempMemoryBufferSetupParams& params)
    {
        size_t numBytes = params.numBytesPerThread;

        if (numBytes == 0)
        {
            uint maxElementsA, maxElementsB, maxSetsA, maxSetsB;
            FmGetExpandableHashSetSizes(&maxElementsA, &maxSetsA, params.maxTetMeshBufferFeatures);
            FmGetExpandableHashSetSizes(&maxElementsB, &maxSetsB, params.maxTetMeshBufferFeatures);
            uint maxElements = maxElementsA + maxElementsB;
            uint maxSets = maxSetsA + maxSetsB;

            size_t numBytesCollisionDetection =
                FM_PAD_16(sizeof(FmVolumeContactWorkspace))
                + FM_PAD_16(sizeof(FmVolumeContactVertSet) * maxSets)
                + FM_PAD_16(sizeof(FmVolumeContactVertSetElement) * maxElements)
                + FM_PAD_16(sizeof(FmVolumeContactFace) * maxElements)
#if FM_SURFACE_INTERSECTION_CONTACTS
                + FM_PAD_16(sizeof(FmIntersectingFacePair) * FM_SURFACE_INTERSECTION_MAX_CONTACTS)
#endif
                + FM_PAD_16(sizeof(FmMeshCollisionTriPair) * FM_MAX_MESH_COLLISION_TRI_PAIR)
#if FM_CONTACT_REDUCTION
                + FM_PAD_16(sizeof(FmContactReductionWorkspace))
                + FM_PAD_16(sizeof(FmVertKeptContactSet) * params.maxTetMeshBufferFeatures)
                + FM_PAD_16(sizeof(FmVertKeptContactSet) * params.maxTetMeshBufferFeatures)
#endif
                + FM_PAD_16(sizeof(FmDistanceContactPairInfo) * FM_MAX_TEMP_DISTANCE_CONTACTS)
                + FM_PAD_16(sizeof(FmDistanceContact) * FM_MAX_TEMP_DISTANCE_CONTACTS)
                + FM_PAD_16(sizeof(FmDistanceContactTetVertIds) * FM_MAX_TEMP_DISTANCE_CONTACTS)
                + FM_PAD_16(sizeof(FmBoxBoxCcdTemps));


            numBytes = numBytesCollisionDetection;

            size_t numBytesMeshConnectedComponents =
                FM_PAD_16(sizeof(uint) * params.maxTetMeshBufferFeatures)
                + FM_PAD_16(sizeof(FmReorderInfo) * params.maxTetMeshBufferFeatures)
                + FM_PAD_16(sizeof(FmReorderInfo) * params.maxTetMeshBufferFeatures)
                + FM_PAD_16(sizeof(FmReorderInfo) * params.maxTetMeshBufferFeatures);
            if (numBytesMeshConnectedComponents > numBytes)
            {
                numBytes = numBytesMeshConnectedComponents;
            }

            size_t numBytesMpcgTemps = FmMpcgSolverDataTemps::GetNumBytes(params.maxTetMeshBufferFeatures);
            if (numBytesMpcgTemps > numBytes)
            {
                numBytes = numBytesMpcgTemps;
            }

            const uint maxTetIdMapElements = FM_MAX_VERT_INCIDENT_TETS * 2;
            size_t splitFaceConnectedSize =
                FM_PAD_16(sizeof(FmSplitFaceConnectedLocalTet)*FM_MAX_VERT_INCIDENT_TETS)
                + FM_PAD_16(sizeof(FmFractureTetIdMapElement)*maxTetIdMapElements)
                + FM_PAD_16(sizeof(uint)*FM_MAX_VERT_INCIDENT_TETS);
            size_t makeSplitsSize = 
                FM_PAD_16(sizeof(FmFractureTetIdMapElement)*maxTetIdMapElements)
                + FM_PAD_16(sizeof(FmMakeSplitsLocalTet)*FM_MAX_VERT_INCIDENT_TETS)
                + FM_PAD_16(sizeof(uint)*FM_MAX_VERT_INCIDENT_TETS);
            size_t numBytesFracture = (splitFaceConnectedSize > makeSplitsSize) ? splitFaceConnectedSize : makeSplitsSize;

            if (numBytesFracture > numBytes)
            {
                numBytes = numBytesFracture;
            }
        }

        return FM_PAD_64(numBytes);
    }

    size_t FmGetThreadTempMemoryBufferSize(const FmThreadTempMemoryBufferSetupParams& params)
    {
        uint numBuffers = params.numWorkerThreads;
        size_t numBytesPerBuffer = FmGetNumBytes(params);

        size_t numBytes =
            FM_PAD_64(sizeof(*FmThreadTempMemoryBuffer::buffers) * params.numWorkerThreads) +
            FM_PAD_64(numBytesPerBuffer) * numBuffers +
            FM_PAD_16(sizeof(FmThreadTempMemoryBuffer));

        return FM_PAD_64(numBytes);
    }

    FmThreadTempMemoryBuffer* FmSetupThreadTempMemoryBuffer(const FmThreadTempMemoryBufferSetupParams& params, uint8_t*& pBuffer, size_t bufferNumBytes)
    {
        FM_ASSERT(((uintptr_t)pBuffer & 0x3f) == 0);

        uint numBuffers = params.numWorkerThreads;
        uint maxTetMeshBufferFeatures = params.maxTetMeshBufferFeatures;
        size_t numBytesPerBuffer = FmGetNumBytes(params);

        uint8_t* pBufferStart = pBuffer;
        uint8_t* pBufferEnd = pBuffer + bufferNumBytes;

        uint8_t** buffers = FmAllocFromBuffer64<uint8_t*>(&pBuffer, numBuffers, pBufferEnd);

        for (uint tIdx = 0; tIdx < numBuffers; tIdx++)
        {
            buffers[tIdx] = FmAllocFromBuffer64<uint8_t>(&pBuffer, numBytesPerBuffer, pBufferEnd);
        }

        FmThreadTempMemoryBuffer* pTempMemoryBuffer = FmAllocFromBuffer<FmThreadTempMemoryBuffer>(&pBuffer, 1, pBufferEnd);
        pTempMemoryBuffer->pBuffer = pBufferStart;
        pTempMemoryBuffer->bufferNumBytes = bufferNumBytes;

        pTempMemoryBuffer->buffers = buffers;

        pTempMemoryBuffer->numBuffers = numBuffers;
        pTempMemoryBuffer->numBytesPerBuffer = numBytesPerBuffer;
        pTempMemoryBuffer->maxTetMeshBufferFeatures = maxTetMeshBufferFeatures;

        pBuffer = (uint8_t*)FM_PAD_64((uintptr_t)pBuffer);
        FM_ASSERT(((uintptr_t)pBuffer & 0x3f) == 0);

        return pTempMemoryBuffer;
    }
}
