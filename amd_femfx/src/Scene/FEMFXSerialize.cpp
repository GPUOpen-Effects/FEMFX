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
// Functions to serialize and deserialize tet mesh buffers, rigid bodies, and constraints
// of a scene.
//---------------------------------------------------------------------------------------

#include "AMD_FEMFX.h"
#include "FEMFXScene.h"
#include "FEMFXConstraints.h"
#include "FEMFXMpcgSolver.h"
#include "FEMFXSerialize.h"

namespace AMD
{
    void FmResetTetMeshBufferSceneState(FmTetMeshBuffer* tetMeshBuffer);
    void FmResetRigidBodySceneState(FmRigidBody* rigidBody);

    template<class T>
    void FmConvertPtrToOffset(T*& ptr, uint8_t* basePtr)
    {
        ptr = (ptr == NULL) ? reinterpret_cast<T*>(0xFFFFFFFFFFFFFFFF) : reinterpret_cast<T*>((uint8_t*)ptr - basePtr);
    }

    template<class T>
    void FmConvertOffsetToPtr(T*& offset, uint8_t* basePtr)
    {
        offset = (offset == reinterpret_cast<T*>(0xFFFFFFFFFFFFFFFF)) ? NULL : (T*)(basePtr + reinterpret_cast<uintptr_t>(offset));
    }

    void FmConvertTetMeshPtrsToOffsets(FmTetMesh* tetMesh, uint8_t* pBufferBase)
    {
        FmConvertPtrToOffset(tetMesh->vertsNeighbors, pBufferBase);
        FmConvertPtrToOffset(tetMesh->vertsMass, pBufferBase);
        FmConvertPtrToOffset(tetMesh->vertsFlags, pBufferBase);
        FmConvertPtrToOffset(tetMesh->vertsIndex0, pBufferBase);
        FmConvertPtrToOffset(tetMesh->vertsRestPos, pBufferBase);
        FmConvertPtrToOffset(tetMesh->vertsPos, pBufferBase);
        FmConvertPtrToOffset(tetMesh->vertsVel, pBufferBase);
        FmConvertPtrToOffset(tetMesh->vertsExtForce, pBufferBase);
        FmConvertPtrToOffset(tetMesh->vertsTetValues, pBufferBase);
        FmConvertPtrToOffset(tetMesh->vertConnectivity.incidentTets, pBufferBase);
        FmConvertPtrToOffset(tetMesh->tetsMass, pBufferBase);
        FmConvertPtrToOffset(tetMesh->tetsFrictionCoeff, pBufferBase);
        FmConvertPtrToOffset(tetMesh->tetsFlags, pBufferBase);
        FmConvertPtrToOffset(tetMesh->tetsShapeParams, pBufferBase);
        FmConvertPtrToOffset(tetMesh->tetsRestDensity, pBufferBase);
        FmConvertPtrToOffset(tetMesh->tetsMaxUnconstrainedSolveIterations, pBufferBase);
        FmConvertPtrToOffset(tetMesh->tetsStressMaterialParams, pBufferBase);
        FmConvertPtrToOffset(tetMesh->tetsDeformationMaterialParams, pBufferBase);
        FmConvertPtrToOffset(tetMesh->tetsFractureMaterialParams, pBufferBase);
        FmConvertPtrToOffset(tetMesh->tetsPlasticityMaterialParams, pBufferBase);
        FmConvertPtrToOffset(tetMesh->tetsStrainMag, pBufferBase);
        FmConvertPtrToOffset(tetMesh->tetsIndex0, pBufferBase);
        FmConvertPtrToOffset(tetMesh->tetsRotation, pBufferBase);
        FmConvertPtrToOffset(tetMesh->tetsFaceIncidentTetIds, pBufferBase);
        FmConvertPtrToOffset(tetMesh->tetsVertIds, pBufferBase);
        FmConvertPtrToOffset(tetMesh->tetsStiffness, pBufferBase);
        FmConvertPtrToOffset(tetMesh->exteriorFaces, pBufferBase);
        FmConvertPtrToOffset(tetMesh->tetsToFracture, pBufferBase);
        FmConvertPtrToOffset(tetMesh->tetsPlasticity, pBufferBase);
        FmConvertPtrToOffset(tetMesh->solverData, pBufferBase);
        FmConvertPtrToOffset(tetMesh->bvh.nodes, pBufferBase);
        FmConvertPtrToOffset(tetMesh->bvh.primBoxes, pBufferBase);
        FmConvertPtrToOffset(tetMesh->bvh.mortonCodesSorted, pBufferBase);
        FmConvertPtrToOffset(tetMesh->bvh.primIndicesSorted, pBufferBase);
    }

    void FmConvertTetMeshOffsetsToPtrs(FmTetMesh* tetMesh, uint8_t* pBufferBase)
    {
        FmConvertOffsetToPtr(tetMesh->vertsNeighbors, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->vertsMass, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->vertsFlags, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->vertsIndex0, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->vertsRestPos, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->vertsPos, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->vertsVel, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->vertsExtForce, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->vertsTetValues, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->vertConnectivity.incidentTets, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->tetsMass, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->tetsFrictionCoeff, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->tetsFlags, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->tetsShapeParams, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->tetsRestDensity, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->tetsMaxUnconstrainedSolveIterations, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->tetsStressMaterialParams, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->tetsDeformationMaterialParams, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->tetsFractureMaterialParams, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->tetsPlasticityMaterialParams, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->tetsStrainMag, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->tetsIndex0, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->tetsRotation, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->tetsFaceIncidentTetIds, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->tetsVertIds, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->tetsStiffness, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->exteriorFaces, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->tetsToFracture, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->tetsPlasticity, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->solverData, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->bvh.nodes, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->bvh.primBoxes, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->bvh.mortonCodesSorted, pBufferBase);
        FmConvertOffsetToPtr(tetMesh->bvh.primIndicesSorted, pBufferBase);
    }

    void FmConvertSolverDataPtrsToOffsets(FmMpcgSolverData* solverData, uint8_t* pBufferBase)
    {
        FmConvertPtrToOffset(solverData->kinematicFlags, pBufferBase);
        FmConvertPtrToOffset(solverData->PInvDiag, pBufferBase);
        FmConvertPtrToOffset(solverData->A.indices, pBufferBase);
        FmConvertPtrToOffset(solverData->A.rowStarts, pBufferBase);
        FmConvertPtrToOffset(solverData->A.submats, pBufferBase);
        FmConvertPtrToOffset(solverData->b, pBufferBase);
        FmConvertPtrToOffset(solverData->mass, pBufferBase);
    }

    void FmConvertSolverDataOffsetsToPtrs(FmMpcgSolverData* solverData, uint8_t* pBufferBase)
    {
        FmConvertOffsetToPtr(solverData->kinematicFlags, pBufferBase);
        FmConvertOffsetToPtr(solverData->PInvDiag, pBufferBase);
        FmConvertOffsetToPtr(solverData->A.indices, pBufferBase);
        FmConvertOffsetToPtr(solverData->A.rowStarts, pBufferBase);
        FmConvertOffsetToPtr(solverData->A.submats, pBufferBase);
        FmConvertOffsetToPtr(solverData->b, pBufferBase);
        FmConvertOffsetToPtr(solverData->mass, pBufferBase);
    }

    void FmConvertTetMeshBufferPtrsToOffsets(FmTetMeshBuffer* tetMeshBuffer, uint8_t* pBufferBase)
    {
        FmConvertPtrToOffset(tetMeshBuffer->tetMeshes, pBufferBase);
        FmConvertPtrToOffset(tetMeshBuffer->solverData, pBufferBase);
        FmConvertPtrToOffset(tetMeshBuffer->fractureGroupCounts, pBufferBase);
        FmConvertPtrToOffset(tetMeshBuffer->visitedFractureGroup, pBufferBase);
        FmConvertPtrToOffset(tetMeshBuffer->vertReferences, pBufferBase);
        FmConvertPtrToOffset(tetMeshBuffer->tetReferences, pBufferBase);
    }

    void FmConvertTetMeshBufferOffsetsToPtrs(FmTetMeshBuffer* tetMeshBuffer, uint8_t* pBufferBase)
    {
        FmConvertOffsetToPtr(tetMeshBuffer->tetMeshes, pBufferBase);
        FmConvertOffsetToPtr(tetMeshBuffer->solverData, pBufferBase);
        FmConvertOffsetToPtr(tetMeshBuffer->fractureGroupCounts, pBufferBase);
        FmConvertOffsetToPtr(tetMeshBuffer->visitedFractureGroup, pBufferBase);
        FmConvertOffsetToPtr(tetMeshBuffer->vertReferences, pBufferBase);
        FmConvertOffsetToPtr(tetMeshBuffer->tetReferences, pBufferBase);
    }

    // Serialize tet mesh buffer contents after it has been removed from a FmScene.
    // Must provide buffer at least as large as FmTetMeshBuffer->bufferNumBytes.
    void FmSerializeTetMeshBuffer(uint8_t* dstBuffer, const FmTetMeshBuffer& srcTetMeshBuffer)
    {
        // Copy buffer
        memcpy(dstBuffer, &srcTetMeshBuffer, srcTetMeshBuffer.bufferNumBytes);

        // Convert pointers to offsets
        FmTetMeshBuffer& dstTetMeshBuffer = *(FmTetMeshBuffer*)dstBuffer;
        uint8_t* srcBasePtr = (uint8_t*)&srcTetMeshBuffer;

        FmConvertTetMeshBufferPtrsToOffsets(&dstTetMeshBuffer, srcBasePtr);

        // Convert tet mesh and solver data pointers to offsets
        uint maxTetMeshes = dstTetMeshBuffer.maxTetMeshes;
        for (uint tetMeshIdx = 0; tetMeshIdx < maxTetMeshes; tetMeshIdx++)
        {
            FmTetMesh* dstTetMeshes = (FmTetMesh*)(dstBuffer + reinterpret_cast<uintptr_t>(dstTetMeshBuffer.tetMeshes));
            FmMpcgSolverData* dstSolverData = (FmMpcgSolverData*)(dstBuffer + reinterpret_cast<uintptr_t>(dstTetMeshBuffer.solverData));
            FmConvertTetMeshPtrsToOffsets(&dstTetMeshes[tetMeshIdx], srcBasePtr);
            FmConvertSolverDataPtrsToOffsets(&dstSolverData[tetMeshIdx], srcBasePtr);
        }
    }

    // Deserialize a serialized tet mesh buffer for addition to a FmScene.
    void FmDeserializeTetMeshBuffer(FmTetMeshBuffer* dstTetMeshBuffer, const uint8_t* srcBuffer, size_t srcBufferNumBytes)
    {
        FM_ASSERT(((FmTetMeshBuffer*)srcBuffer)->bufferNumBytes == srcBufferNumBytes);

        // Copy buffer
        memcpy(dstTetMeshBuffer, srcBuffer, srcBufferNumBytes);

        // Convert offsets to pointers
        uint8_t* dstBasePtr = (uint8_t*)dstTetMeshBuffer;

        FmConvertTetMeshBufferOffsetsToPtrs(dstTetMeshBuffer, dstBasePtr);

        uint maxTetMeshes = dstTetMeshBuffer->maxTetMeshes;
        for (uint tetMeshIdx = 0; tetMeshIdx < maxTetMeshes; tetMeshIdx++)
        {
            FmTetMesh& tetMesh = dstTetMeshBuffer->tetMeshes[tetMeshIdx];
            FmMpcgSolverData& solverData = dstTetMeshBuffer->solverData[tetMeshIdx];
            FmConvertTetMeshOffsetsToPtrs(&tetMesh, dstBasePtr);
            FmConvertSolverDataOffsetsToPtrs(&solverData, dstBasePtr);

            FM_ASSERT(tetMesh.solverData == &solverData);
        }
    }

    size_t FmGetSceneSerializedTetMeshBufferSize(const FmScene& scene)
    {
        uint numTetMeshBufferSlots = scene.numTetMeshBufferSlots;
        size_t totalSize = 0;

        totalSize += sizeof(uint); // for number of slots

        for (uint bufferIdx = 0; bufferIdx < numTetMeshBufferSlots; bufferIdx++)
        {
            const FmTetMeshBuffer* tetMeshBuffer = scene.tetMeshBuffers[bufferIdx];
            if (tetMeshBuffer)
            {
                totalSize += sizeof(size_t) + tetMeshBuffer->bufferNumBytes; // size plus data
            }
            else
            {
                totalSize += sizeof(size_t); // recording deleted slots by setting size 0
            }
        }
        return totalSize;
    }

    // Serialize all tet mesh buffers and return number of tet mesh buffer slots
    uint FmSerializeSceneTetMeshBuffers(uint8_t* dstTetMeshBuffers, const FmScene& scene)
    {
        uint numTetMeshBufferSlots = scene.numTetMeshBufferSlots;

        uint8_t* pDstBuffer = dstTetMeshBuffers;

        // Save number of tet mesh buffers
        *((uint*)pDstBuffer) = numTetMeshBufferSlots;
        pDstBuffer += sizeof(uint);

        // Copy all tet mesh buffers
        for (uint bufferIdx = 0; bufferIdx < numTetMeshBufferSlots; bufferIdx++)
        {
            const FmTetMeshBuffer* tetMeshBuffer = scene.tetMeshBuffers[bufferIdx];
            if (tetMeshBuffer)
            {
                // Save size of tet mesh buffer
                *((size_t*)pDstBuffer) = tetMeshBuffer->bufferNumBytes;
                pDstBuffer += sizeof(size_t);

                FmSerializeTetMeshBuffer(pDstBuffer, *tetMeshBuffer);
                pDstBuffer += tetMeshBuffer->bufferNumBytes;
            }
            else
            {
                *((size_t*)pDstBuffer) = 0; // recording deleted slots by setting size 0
                pDstBuffer += sizeof(size_t);
            }
        }

        return numTetMeshBufferSlots;
    }

    // Get number of tet mesh buffer slots that were serialized
    uint FmGetSerializedNumTetMeshBufferSlots(const uint8_t* srcTetMeshBuffers)
    {
        uint numTetMeshBufferSlots = *((uint*)srcTetMeshBuffers);
        return numTetMeshBufferSlots;
    }

    // Pass in array of numTetMeshBufferSlots pointers to be allocated for tet mesh buffers.
    // The number of valid tet mesh buffers is returned.
    uint FmAllocTetMeshBuffersForDeserialization(uint8_t** dstTetMeshBuffers, uint8_t* srcTetMeshBuffers, uint numTetMeshBufferSlots)
    {
        FM_ASSERT(*((uint*)srcTetMeshBuffers) == numTetMeshBufferSlots);
        srcTetMeshBuffers += sizeof(uint);

        uint dstTetMeshBufferIdx = 0;
        for (uint bufferIdx = 0; bufferIdx < numTetMeshBufferSlots; bufferIdx++)
        {
            size_t srcTetMeshBufferNumBytes = *((size_t*)srcTetMeshBuffers);
            srcTetMeshBuffers += sizeof(size_t);

            if (srcTetMeshBufferNumBytes > 0)
            {
                dstTetMeshBuffers[dstTetMeshBufferIdx] = (uint8_t*)FmAlignedMalloc(srcTetMeshBufferNumBytes, 64);
                srcTetMeshBuffers += srcTetMeshBufferNumBytes;

                dstTetMeshBufferIdx++;
            }
        }
        return dstTetMeshBufferIdx;
    }

    // Deserialize serialized scene tet mesh buffers into dstTetMeshBuffers and add to scene
    size_t FmDeserializeSceneTetMeshBuffers(FmTetMeshBuffer** dstTetMeshBuffers, FmScene* scene, const uint8_t* srcTetMeshBuffers, uint numTetMeshBufferSlots)
    {
        if (numTetMeshBufferSlots > scene->maxTetMeshBuffers)
        {
            return 0;
        }

        const uint8_t* srcTetMeshBuffersStart = srcTetMeshBuffers;

        // Save number of tet mesh buffers
        FM_ASSERT(*((uint*)srcTetMeshBuffers) == numTetMeshBufferSlots);
        srcTetMeshBuffers += sizeof(uint);

        // Remove any tet mesh buffers in scene.
        // NOTE: does not free the memory associated with these buffers, which is left to the application
        uint sceneNumTetMeshBufferSlots = scene->numTetMeshBufferSlots;
        for (uint bufferIdx = 0; bufferIdx < sceneNumTetMeshBufferSlots; bufferIdx++)
        {
            const FmTetMeshBuffer* tetMeshBuffer = scene->tetMeshBuffers[bufferIdx];
            if (tetMeshBuffer)
            {
                uint bufferId = tetMeshBuffer->bufferId;
                FmRemoveTetMeshBufferFromScene(scene, bufferId);
            }
        }
        FmResetFreeIds(&scene->freeTetMeshIds);

        uint dstTetMeshBufferIdx = 0;
        for (uint bufferIdx = 0; bufferIdx < numTetMeshBufferSlots; bufferIdx++)
        {
            size_t srcTetMeshBufferNumBytes = *((size_t*)srcTetMeshBuffers);
            srcTetMeshBuffers += sizeof(size_t);

            if (srcTetMeshBufferNumBytes > 0)
            {
                if (dstTetMeshBuffers[dstTetMeshBufferIdx])
                {
                    FmAlignedFree(dstTetMeshBuffers[dstTetMeshBufferIdx]);
                }

                dstTetMeshBuffers[dstTetMeshBufferIdx] = (FmTetMeshBuffer*)FmAlignedMalloc(srcTetMeshBufferNumBytes, 64);

                FmTetMeshBuffer* dstTetMeshBuffer = dstTetMeshBuffers[dstTetMeshBufferIdx];

                FmDeserializeTetMeshBuffer(dstTetMeshBuffer, srcTetMeshBuffers, srcTetMeshBufferNumBytes);
                srcTetMeshBuffers += srcTetMeshBufferNumBytes;

                dstTetMeshBufferIdx++;

                FmAddTetMeshBufferToScene(scene, dstTetMeshBuffer);
            }
        }

        return srcTetMeshBuffers - srcTetMeshBuffersStart;
    }

    // Serialize rigid body contents after it has been removed from a FmScene.
    // Must provide buffer at least as large as FmRigidBody->bufferNumBytes.
    void FmSerializeRigidBody(uint8_t* dstBuffer, const FmRigidBody& srcRigidBody)
    {
        *((FmRigidBody*)dstBuffer) = srcRigidBody;
    }

    // Deserialize a serialized rigid body for addition to a FmScene.
    void FmDeserializeRigidBody(FmRigidBody* dstRigidBody, const uint8_t* srcBuffer, size_t srcBufferNumBytes)
    {
        FM_ASSERT(sizeof(FmRigidBody) == srcBufferNumBytes);

        // Copy buffer
        memcpy(dstRigidBody, srcBuffer, srcBufferNumBytes);
    }

    size_t FmGetSceneSerializedRigidBodiesSize(const FmScene& scene)
    {
        uint numRigidBodySlots = scene.numRigidBodySlots;
        size_t totalSize = 0;

        totalSize += sizeof(uint); // for number of slots

        for (uint rbIdx = 0; rbIdx < numRigidBodySlots; rbIdx++)
        {
            const FmRigidBody* rigidBody = scene.rigidBodies[rbIdx];
            if (rigidBody)
            {
                totalSize += sizeof(size_t) + sizeof(FmRigidBody); // size plus data
            }
            else
            {
                totalSize += sizeof(size_t); // recording deleted slots by setting size 0
            }
        }
        return totalSize;
    }

    // Serialize all rigid bodies and return number of rigid body slots
    uint FmSerializeSceneRigidBodies(uint8_t* dstRigidBodiesBuffer, const FmScene& scene)
    {
        uint numRigidBodySlots = scene.numRigidBodySlots;

        uint8_t* pDstBuffer = dstRigidBodiesBuffer;

        // Save number of rigid bodies
        *((uint*)pDstBuffer) = numRigidBodySlots;
        pDstBuffer += sizeof(uint);

        // Copy all rigid bodies
        for (uint rbIdx = 0; rbIdx < numRigidBodySlots; rbIdx++)
        {
            const FmRigidBody* rigidBody = scene.rigidBodies[rbIdx];
            if (rigidBody)
            {
                // Save size of rigid body
                *((size_t*)pDstBuffer) = sizeof(FmRigidBody);
                pDstBuffer += sizeof(size_t);

                // Save rigid body data
                FmSerializeRigidBody(pDstBuffer, *rigidBody);
                pDstBuffer += sizeof(FmRigidBody);
            }
            else
            {
                // Record deleted slots by setting size 0
                *((size_t*)pDstBuffer) = 0; 
                pDstBuffer += sizeof(size_t);
            }
        }

        return numRigidBodySlots;
    }

    // Get number of rigid body slots that were serialized
    uint FmGetSerializedNumRigidBodySlots(const uint8_t* srcRigidBodies)
    {
        uint numRigidBodySlots = *((uint*)srcRigidBodies);
        return numRigidBodySlots;
    }

    // Pass in array of numRigidBodySlots pointers to be allocated for rigid bodies.
    // The number of valid rigid bodies is returned.
    uint FmAllocRigidBodiesForDeserialization(uint8_t** dstRigidBodies, uint8_t* srcRigidBodiesBuffer, uint numRigidBodySlots)
    {
        FM_ASSERT(*((uint*)srcRigidBodiesBuffer) == numRigidBodySlots);
        srcRigidBodiesBuffer += sizeof(uint);

        uint dstRigidBodyIdx = 0;
        for (uint rbIdx = 0; rbIdx < numRigidBodySlots; rbIdx++)
        {
            size_t srcRigidBodyNumBytes = *((size_t*)srcRigidBodiesBuffer);
            srcRigidBodiesBuffer += sizeof(size_t);

            if (srcRigidBodyNumBytes > 0)
            {
                dstRigidBodies[dstRigidBodyIdx] = (uint8_t*)FmAlignedMalloc(srcRigidBodyNumBytes, 64);
                srcRigidBodiesBuffer += srcRigidBodyNumBytes;

                dstRigidBodyIdx++;
            }
        }
        return dstRigidBodyIdx;
    }

    // Deserialize serialized scene rigid bodies into dstRigidBodies and add to scene
    size_t FmDeserializeSceneRigidBodies(FmRigidBody** dstRigidBodies, FmScene* scene, const uint8_t* srcRigidBodies, uint numRigidBodySlots)
    {
        if (numRigidBodySlots > scene->maxRigidBodies)
        {
            return 0;
        }

        const uint8_t* srcRigidBodiesStart = srcRigidBodies;

        // Save number of rigid bodies
        FM_ASSERT(*((uint*)srcRigidBodies) == numRigidBodySlots);
        srcRigidBodies += sizeof(uint);

        // Remove any rigid bodies in scene.
        // NOTE: does not free the memory associated with these buffers, which is left to the application
        uint sceneNumRigidBodySlots = scene->numRigidBodySlots;
        for (uint bufferIdx = 0; bufferIdx < sceneNumRigidBodySlots; bufferIdx++)
        {
            const FmRigidBody* rigidBody = scene->rigidBodies[bufferIdx];
            if (rigidBody)
            {
                uint objectId = rigidBody->objectId;
                FmRemoveRigidBodyFromScene(scene, objectId);
            }
        }
        FmResetFreeIds(&scene->freeRigidBodyIds, FM_RB_FLAG);

        uint dstRigidBodyIdx = 0;
        for (uint bufferIdx = 0; bufferIdx < numRigidBodySlots; bufferIdx++)
        {
            size_t srcRigidBodyNumBytes = *((size_t*)srcRigidBodies);
            srcRigidBodies += sizeof(size_t);

            if (srcRigidBodyNumBytes > 0)
            {
                if (dstRigidBodies[dstRigidBodyIdx])
                {
                    FmDestroyRigidBody(dstRigidBodies[dstRigidBodyIdx]);
                }

                dstRigidBodies[dstRigidBodyIdx] = (FmRigidBody*)FmAlignedMalloc(srcRigidBodyNumBytes, 64);

                FmRigidBody* dstRigidBody = dstRigidBodies[dstRigidBodyIdx];

                FmDeserializeRigidBody(dstRigidBody, srcRigidBodies, srcRigidBodyNumBytes);
                srcRigidBodies += srcRigidBodyNumBytes;

                dstRigidBodyIdx++;

                FmCreateRigidBodyBoxCollObj(dstRigidBody);
                FmAddRigidBodyToScene(scene, dstRigidBody);
            }
        }

        return srcRigidBodies - srcRigidBodiesStart;
    }

    size_t FmGetSceneSerializedConstraintsSize(const FmScene& scene)
    {
        return sizeof(uint) + sizeof(uint) + sizeof(uint)
            + sizeof(FmGlueConstraint) * scene.constraintsBuffer->numGlueConstraints
            + sizeof(FmPlaneConstraint) * scene.constraintsBuffer->numPlaneConstraints
            + sizeof(FmRigidBodyAngleConstraint) * scene.constraintsBuffer->numRigidBodyAngleConstraints;
    }

    // Serialize all constraints and return numbers of constraints
    void FmSerializeSceneConstraints(uint8_t* dstConstraintsBuffer, 
        uint* outNumGlueConstraints, 
        uint* outNumPlaneConstraints,
        uint* outNumRigidBodyAngleConstraints, 
        const FmScene& scene)
    {
        uint numGlueConstraints = scene.constraintsBuffer->numGlueConstraints;
        uint numPlaneConstraints = scene.constraintsBuffer->numPlaneConstraints;
        uint numRigidBodyAngleConstraints = scene.constraintsBuffer->numRigidBodyAngleConstraints;

        // Save counts
        (*(uint*)dstConstraintsBuffer) = numGlueConstraints;
        dstConstraintsBuffer += sizeof(uint);

        (*(uint*)dstConstraintsBuffer) = numPlaneConstraints;
        dstConstraintsBuffer += sizeof(uint);

        (*(uint*)dstConstraintsBuffer) = numRigidBodyAngleConstraints;
        dstConstraintsBuffer += sizeof(uint);

        // Glue constraints
        uint glueConstraintsNumBytes = sizeof(FmGlueConstraint)*numGlueConstraints;
        memcpy(dstConstraintsBuffer, scene.constraintsBuffer->glueConstraints, glueConstraintsNumBytes);
        dstConstraintsBuffer += glueConstraintsNumBytes;

        // Plane constraints
        uint planeConstraintsNumBytes = sizeof(FmPlaneConstraint)*numPlaneConstraints;
        memcpy(dstConstraintsBuffer, scene.constraintsBuffer->planeConstraints, planeConstraintsNumBytes);
        dstConstraintsBuffer += planeConstraintsNumBytes;

        // Rigid body angle constraints
        uint rigidBodyAngleConstraintsNumBytes = sizeof(FmRigidBodyAngleConstraint)*numRigidBodyAngleConstraints;
        memcpy(dstConstraintsBuffer, scene.constraintsBuffer->rigidBodyAngleConstraints, rigidBodyAngleConstraintsNumBytes);
        dstConstraintsBuffer += rigidBodyAngleConstraintsNumBytes;

        *outNumGlueConstraints = numGlueConstraints;
        *outNumPlaneConstraints = numPlaneConstraints;
        *outNumRigidBodyAngleConstraints = numRigidBodyAngleConstraints;
    }

    size_t FmDeserializeSceneConstraints(FmScene* scene, const uint8_t* srcConstraintsBuffer)
    {
        const uint8_t* srcConstraintsBufferStart = srcConstraintsBuffer;

        uint srcNumGlueConstraints = *(uint*)srcConstraintsBuffer;
        srcConstraintsBuffer += sizeof(uint);

        uint srcNumPlaneConstraints = *(uint*)srcConstraintsBuffer;
        srcConstraintsBuffer += sizeof(uint);

        uint srcNumRigidBodyAngleConstraints = *(uint*)srcConstraintsBuffer;
        srcConstraintsBuffer += sizeof(uint);

        if (srcNumGlueConstraints > scene->constraintsBuffer->maxGlueConstraints
            || srcNumPlaneConstraints > scene->constraintsBuffer->maxPlaneConstraints
            || srcNumRigidBodyAngleConstraints > scene->constraintsBuffer->maxRigidBodyAngleConstraints)
        {
            return 0;
        }

        // Remove any constraints in scene
        uint sceneNumGlueConstraints = scene->constraintsBuffer->numGlueConstraints;
        for (uint cIdx = 0; cIdx < sceneNumGlueConstraints; cIdx++)
        {
            FmRemoveGlueConstraintFromScene(scene, cIdx);
        }
        uint sceneNumPlaneConstraints = scene->constraintsBuffer->numPlaneConstraints;
        for (uint cIdx = 0; cIdx < sceneNumPlaneConstraints; cIdx++)
        {
            FmRemovePlaneConstraintFromScene(scene, cIdx);
        }
        uint sceneNumRigidBodyAngleConstraints = scene->constraintsBuffer->numRigidBodyAngleConstraints;
        for (uint cIdx = 0; cIdx < sceneNumRigidBodyAngleConstraints; cIdx++)
        {
            FmRemoveRigidBodyAngleConstraintFromScene(scene, cIdx);
        }
        FmResetFreeIds(&scene->constraintsBuffer->freeGlueConstraintIds);
        FmResetFreeIds(&scene->constraintsBuffer->freePlaneConstraintIds);
        FmResetFreeIds(&scene->constraintsBuffer->freeRigidBodyAngleConstraintIds);

        // Add constraints
        FmGlueConstraint* srcGlueConstraints = (FmGlueConstraint*)srcConstraintsBuffer;
        for (uint cIdx = 0; cIdx < srcNumGlueConstraints; cIdx++)
        {
            FmAddGlueConstraintToScene(scene, srcGlueConstraints[cIdx]);
        }
        srcConstraintsBuffer += sizeof(FmGlueConstraint) * srcNumGlueConstraints;

        FmPlaneConstraint* srcPlaneConstraints = (FmPlaneConstraint*)srcConstraintsBuffer;
        for (uint cIdx = 0; cIdx < srcNumPlaneConstraints; cIdx++)
        {
            FmAddPlaneConstraintToScene(scene, srcPlaneConstraints[cIdx]);
        }
        srcConstraintsBuffer += sizeof(FmPlaneConstraint) * srcNumPlaneConstraints;

        FmRigidBodyAngleConstraint* srcRigidBodyAngleConstraints = (FmRigidBodyAngleConstraint*)srcConstraintsBuffer;
        for (uint cIdx = 0; cIdx < srcNumRigidBodyAngleConstraints; cIdx++)
        {
            FmAddRigidBodyAngleConstraintToScene(scene, srcRigidBodyAngleConstraints[cIdx]);
        }
        srcConstraintsBuffer += sizeof(FmRigidBodyAngleConstraint) * srcNumRigidBodyAngleConstraints;

        return srcConstraintsBuffer - srcConstraintsBufferStart;
    }

    uint8_t* FmSerializeScene(size_t* serializationBufferSize, const FmScene& scene)
    {
        size_t sceneTetMeshBuffersSize = FmGetSceneSerializedTetMeshBufferSize(scene);
        size_t sceneRigidBodiesSize = FmGetSceneSerializedRigidBodiesSize(scene);
        size_t sceneConstraintsSize = FmGetSceneSerializedConstraintsSize(scene);
        size_t sceneSize = FM_PAD_64(sizeof(FmSerializedSceneCounts) + sceneTetMeshBuffersSize + sceneRigidBodiesSize + sceneConstraintsSize);

        *serializationBufferSize = sceneSize;

        uint8_t* pSerializationBufferStart = (uint8_t*)FmAlignedMalloc(sceneSize, 64);

        uint8_t* pSerializationBuffer = pSerializationBufferStart;

        pSerializationBuffer += sizeof(FmSerializedSceneCounts);

        FmSceneControlParams& sceneControlParams = *(FmSceneControlParams*)pSerializationBuffer;
        sceneControlParams = scene.params;
        pSerializationBuffer += sizeof(FmSceneControlParams);

        uint numTetMeshBufferSlots = FmSerializeSceneTetMeshBuffers(pSerializationBuffer, scene);
        pSerializationBuffer += sceneTetMeshBuffersSize;

        uint numRigidBodySlots = FmSerializeSceneRigidBodies(pSerializationBuffer, scene);
        pSerializationBuffer += sceneRigidBodiesSize;

        uint numGlueConstraints, numPlaneConstraints, numRigidBodyAngleConstraints;
        FmSerializeSceneConstraints(pSerializationBuffer, &numGlueConstraints, &numPlaneConstraints, &numRigidBodyAngleConstraints, scene);
        pSerializationBuffer += sceneConstraintsSize;

        pSerializationBuffer = pSerializationBufferStart;

        FmSerializedSceneCounts& sceneCounts = *(FmSerializedSceneCounts*)pSerializationBufferStart;
        sceneCounts.bufferSize = sceneSize;
        sceneCounts.numTetMeshBuffers = numTetMeshBufferSlots;
        sceneCounts.numRigidBodies = numRigidBodySlots;
        sceneCounts.numGlueConstraints = numGlueConstraints;
        sceneCounts.numPlaneConstraints = numPlaneConstraints;
        sceneCounts.numRigidBodyAngleConstraints = numRigidBodyAngleConstraints;

        return pSerializationBufferStart;
    }

    void FmGetSerializedSceneCounts(FmSerializedSceneCounts* sceneCounts, const uint8_t* pSerializationBuffer)
    {
        *sceneCounts = *(FmSerializedSceneCounts*)pSerializationBuffer;
    }

    bool FmDeserializeScene(FmScene* scene, FmTetMeshBuffer** tetMeshBufferPtrs, FmRigidBody** rigidBodies, const uint8_t* pSerializationBuffer)
    {
        pSerializationBuffer += sizeof(FmSerializedSceneCounts);

        FmSceneControlParams& sceneControlParams = *(FmSceneControlParams*)pSerializationBuffer;
        scene->params = sceneControlParams;
        pSerializationBuffer += sizeof(FmSceneControlParams);

        uint numSlots = FmGetSerializedNumTetMeshBufferSlots(pSerializationBuffer);

        size_t sceneTetMeshBuffersSize = FmDeserializeSceneTetMeshBuffers(tetMeshBufferPtrs, scene, pSerializationBuffer, numSlots);

        if (sceneTetMeshBuffersSize == 0)
        {
            return false;
        }

        pSerializationBuffer += sceneTetMeshBuffersSize;

        uint numRigidBodySlots = FmGetSerializedNumRigidBodySlots(pSerializationBuffer);

        size_t sceneRigidBodiesSize = FmDeserializeSceneRigidBodies(rigidBodies, scene, pSerializationBuffer, numRigidBodySlots);

        if (sceneRigidBodiesSize == 0)
        {
            return false;
        }

        pSerializationBuffer += sceneRigidBodiesSize;

        size_t sceneConstraintsSize = FmDeserializeSceneConstraints(scene, pSerializationBuffer);

        if (sceneConstraintsSize == 0)
        {
            return false;
        }

        pSerializationBuffer += sceneConstraintsSize;

        return true;
    }
}