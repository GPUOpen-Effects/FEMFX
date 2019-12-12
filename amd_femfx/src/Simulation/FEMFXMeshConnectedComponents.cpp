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
// Find connected connected components of all tet meshes following fracture
//---------------------------------------------------------------------------------------

#include "AMD_FEMFX.h"
#include "FEMFXAtomicOps.h"
#include "FEMFXScene.h"
#include "FEMFXMpcgSolver.h"
#include "FEMFXThreadTempMemory.h"

namespace AMD
{
    // Vertex data collected for partitioning data after fracture
    struct FmCCVertData
    {
        uint            vertId;
        FmVertNeighbors vertNeighbors;
        float           vertMass;
        uint16_t        vertFlags;
        uint            vertIndex0;
        FmVector3       vertRestPos;
        FmVector3       vertPos;
        FmVector3       vertVel;
        FmVector3       vertExtForce;
        FmVertTetValues vertTetValues;
    };

    // Tet data collected for partitioning data after fracture
    struct FmCCTetData
    {
        float                   tetMass;
        float                   tetFrictionCoeff;
        uint16_t                tetFlags;
        FmTetShapeParams        tetShapeParams;
        float                          tetRestDensity;
        uint                           tetMaxUnconstrainedSolveIterations;
        FmTetStressMaterialParams      tetStressMaterialParams;
        FmTetDeformationMaterialParams tetDeformationMaterialParams;
        FmTetPlasticityMaterialParams  tetPlasticityMaterialParams;
        FmTetFractureMaterialParams    tetFractureMaterialParams;
        float                   tetStrainMag;
        uint                    tetIndex0;
        FmMatrix3               tetRotation;
        FmTetFaceIncidentTetIds tetFaceIncidentTetIds;
        FmTetVertIds            tetVertIds;
        FmTetStiffnessState     tetStiffnessState;
        FmTetPlasticityState    tetPlasticityState;
        bool                    enablePlasticity;
        bool                    enableFracture;

        FmCCTetData(): enablePlasticity(false), enableFracture(false) {}
    };

    struct FmCCExteriorFaceData
    {
        FmExteriorFace exteriorFace;
    };

    static inline void FmGetVertData(FmTetMesh* tetMesh, uint srcId, FmCCVertData& vertData)
    {
        vertData.vertId = srcId;
        vertData.vertNeighbors = tetMesh->vertsNeighbors[srcId];
        vertData.vertMass = tetMesh->vertsMass[srcId];
        vertData.vertFlags = tetMesh->vertsFlags[srcId];
        vertData.vertIndex0 = tetMesh->vertsIndex0[srcId];
        vertData.vertRestPos = tetMesh->vertsRestPos[srcId];
        vertData.vertPos = tetMesh->vertsPos[srcId];
        vertData.vertVel = tetMesh->vertsVel[srcId];
        vertData.vertExtForce = tetMesh->vertsExtForce[srcId];
        vertData.vertTetValues = tetMesh->vertsTetValues[srcId];
    }

    static inline void FmSetVertData(FmTetMesh* tetMesh, uint dstId, FmCCVertData& vertData)
    {
        tetMesh->vertsNeighbors[dstId] = vertData.vertNeighbors;
        tetMesh->vertsMass[dstId] = vertData.vertMass;
        tetMesh->vertsFlags[dstId] = vertData.vertFlags;
        tetMesh->vertsIndex0[dstId] = vertData.vertIndex0;
        tetMesh->vertsRestPos[dstId] = vertData.vertRestPos;
        tetMesh->vertsPos[dstId] = vertData.vertPos;
        tetMesh->vertsVel[dstId] = vertData.vertVel;
        tetMesh->vertsExtForce[dstId] = vertData.vertExtForce;
        tetMesh->vertsTetValues[dstId] = vertData.vertTetValues;
    }

    static inline void FmGetTetData(FmTetMesh* tetMesh, uint srcId, FmCCTetData& tetData)
    {
        tetData.tetMass = tetMesh->tetsMass[srcId];
        tetData.tetFrictionCoeff = tetMesh->tetsFrictionCoeff[srcId];
        tetData.tetFlags = tetMesh->tetsFlags[srcId];
        tetData.tetShapeParams = tetMesh->tetsShapeParams[srcId];

        tetData.tetRestDensity = tetMesh->tetsRestDensity[srcId];
        tetData.tetMaxUnconstrainedSolveIterations = tetMesh->tetsMaxUnconstrainedSolveIterations[srcId];
        tetData.tetStressMaterialParams = tetMesh->tetsStressMaterialParams[srcId];
        tetData.tetDeformationMaterialParams = tetMesh->tetsDeformationMaterialParams[srcId];

        tetData.tetStrainMag = tetMesh->tetsStrainMag[srcId];
        tetData.tetIndex0 = tetMesh->tetsIndex0[srcId];
        tetData.tetRotation = tetMesh->tetsRotation[srcId];
        tetData.tetFaceIncidentTetIds = tetMesh->tetsFaceIncidentTetIds[srcId];
        tetData.tetVertIds = tetMesh->tetsVertIds[srcId];
        tetData.tetStiffnessState = tetMesh->tetsStiffness[srcId];
        tetData.enablePlasticity = (tetMesh->tetsPlasticity != NULL);
        tetData.enableFracture = (tetMesh->tetsToFracture != NULL);
        if (tetData.enableFracture)
        {
            tetData.tetFractureMaterialParams = tetMesh->tetsFractureMaterialParams[srcId];
        }
        if (tetData.enablePlasticity)
        {
            tetData.tetPlasticityState = tetMesh->tetsPlasticity[srcId];
            tetData.tetPlasticityMaterialParams = tetMesh->tetsPlasticityMaterialParams[srcId];
        }
    }

    static inline void FmSetTetData(FmTetMesh* tetMesh, uint dstId, FmCCTetData& tetData)
    {
        tetMesh->tetsMass[dstId] = tetData.tetMass;
        tetMesh->tetsFrictionCoeff[dstId] = tetData.tetFrictionCoeff;
        tetMesh->tetsFlags[dstId] = tetData.tetFlags;
        tetMesh->tetsShapeParams[dstId] = tetData.tetShapeParams;

        tetMesh->tetsRestDensity[dstId] = tetData.tetRestDensity;
        tetMesh->tetsMaxUnconstrainedSolveIterations[dstId] = tetData.tetMaxUnconstrainedSolveIterations;
        tetMesh->tetsStressMaterialParams[dstId] = tetData.tetStressMaterialParams;
        tetMesh->tetsDeformationMaterialParams[dstId] = tetData.tetDeformationMaterialParams;

        tetMesh->tetsStrainMag[dstId] = tetData.tetStrainMag;
        tetMesh->tetsIndex0[dstId] = tetData.tetIndex0;
        tetMesh->tetsRotation[dstId] = tetData.tetRotation;
        tetMesh->tetsFaceIncidentTetIds[dstId] = tetData.tetFaceIncidentTetIds;
        tetMesh->tetsVertIds[dstId] = tetData.tetVertIds;
        tetMesh->tetsStiffness[dstId] = tetData.tetStiffnessState;
        if (tetData.enableFracture)
        {
            tetMesh->tetsFractureMaterialParams[dstId] = tetData.tetFractureMaterialParams;
        }
        if (tetData.enablePlasticity)
        {
            tetMesh->tetsPlasticity[dstId] = tetData.tetPlasticityState;
            tetMesh->tetsPlasticityMaterialParams[dstId] = tetData.tetPlasticityMaterialParams;
        }
    }

    static inline void FmGetExteriorFaceData(FmTetMesh* tetMesh, uint srcId, FmCCExteriorFaceData& extFaceData)
    {
        extFaceData.exteriorFace = tetMesh->exteriorFaces[srcId];
    }

    static inline void FmSetExteriorFaceData(FmTetMesh* tetMesh, uint dstId, FmCCExteriorFaceData& extFaceData)
    {
        tetMesh->exteriorFaces[dstId] = extFaceData.exteriorFace;
    }

    void FmUpdateAdjacentVertOffsets(FmTetMesh* tetMesh);

    // Update the tet meshes for a tet mesh buffer, separating any fractured connected components into new meshes.
    // Reuses existing memory by partitioning the original FmTetMesh arrays for each new mesh.
    // TODO: Need to analyze this choice more -- it fragments data for each new mesh, however a test with more 
    // contiguous allocations didn't give a benefit in benchmarks.
    // TODO: Parallelize
    void FmFindTetMeshBufferConnectedComponents(FmScene* scene, FmTetMeshBuffer* pTetMeshBuffer)
    {
        FmTetMeshBuffer& tetMeshBuffer = *pTetMeshBuffer;
        uint numMeshes = tetMeshBuffer.numTetMeshes;
        uint nextFreeMeshIdx = numMeshes;

        int workerIndex = scene->taskSystemCallbacks.GetTaskSystemWorkerIndex();
        uint8_t* pTempBufferStart = scene->threadTempMemoryBuffer->buffers[workerIndex];
        uint8_t* pTempBufferEnd = pTempBufferStart + scene->threadTempMemoryBuffer->numBytesPerBuffer;

        for (uint fractureGroupId = 0; fractureGroupId < tetMeshBuffer.maxTetMeshes; fractureGroupId++)
        {
            tetMeshBuffer.visitedFractureGroup[fractureGroupId] = false;
        }

        for (uint meshIdx = 0; meshIdx < numMeshes; meshIdx++)
        {
            FmTetMesh& tetMesh = tetMeshBuffer.tetMeshes[meshIdx];
            FmMpcgSolverData& solverData = tetMeshBuffer.solverData[meshIdx];

            // Run connected component if the need flag has been set by fracture (or after init),
            // and if the object is not sleeping or disabled.
            if (FM_IS_SET(tetMesh.flags, FM_OBJECT_FLAG_NEEDS_CONNECTED_COMPONENTS)
                && FM_NONE_SET(tetMesh.flags, FM_OBJECT_FLAG_SLEEPING | FM_OBJECT_FLAG_SIMULATION_DISABLED))
            {
                // First step is flood-filling to determine a reordering of the verts, tets, and exterior faces within the 
                // tet mesh's existing memory.   Keep a queue of verts, and for each search incident tets and verts/exterior 
                // faces incident on those, marking each visited feature with its destination index.  When queue is empty, 
                // have reached the end of a connected component and can create a new mesh.
                // Search verts sequentially to find unvisited one for next connected component.
                uint newMeshesStart = nextFreeMeshIdx;
                uint numNewMeshesCreated = 0;
                uint newMeshVertOffset = 0;
                uint newMeshTetOffset = 0;
                uint newMeshExteriorFaceOffset = 0;
                uint newMeshBvhNodeOffset = 0;
                uint newMeshRowStartOffset = 0;
                uint newMeshSubmatOffset = 0;
                uint newMeshNumVerts = 1;
                uint newMeshNumTets = 0;
                uint newMeshNumExteriorFaces = 0;
                uint newMeshNumIncidentTets = 0;

                FmFractureGroupCounts newMeshCounts;
                newMeshCounts.Zero();

                // Save initial tetMesh counts since they'll be changed when creating first new mesh
                uint initialTetMeshNumVerts = tetMesh.numVerts;
                uint initialTetMeshNumTets = tetMesh.numTets;
                uint initialTetMeshNumExteriorFaces = tetMesh.numExteriorFaces;
#if FM_DEBUG_CHECKS || _DEBUG
                uint initialTetMeshMaxVerts = tetMesh.maxVerts;
                uint initialTetMeshMaxExteriorFaces = tetMesh.maxExteriorFaces;
                uint initialSolverDataMaxVertAdjacentVerts = solverData.maxVertAdjacentVerts;
#endif

#if FM_DEBUG_CHECKS
                bool* seenFractureGroup = new bool[tetMeshBuffer.maxTetMeshes];
                for (uint i = 0; i < tetMeshBuffer.maxTetMeshes; i++)
                {
                    seenFractureGroup[i] = false;
                }
                FmFractureGroupCounts counts;
                for (uint tetId = 0; tetId < initialTetMeshNumTets; tetId++)
                {
                    uint tetIndex0 = tetMesh.tetsIndex0[tetId];
                    uint fractureGroupId = tetMeshBuffer.tetReferences[tetIndex0].fractureGroupId;
                    FM_ASSERT(fractureGroupId < tetMeshBuffer.maxTetMeshes);

                    if (!seenFractureGroup[fractureGroupId])
                    {
                        counts.Add(tetMeshBuffer.fractureGroupCounts[fractureGroupId]);
                        seenFractureGroup[fractureGroupId] = true;
                    }
                }

                FM_ASSERT(counts.numVerts == initialTetMeshMaxVerts);
                FM_ASSERT(counts.numTets == initialTetMeshNumTets);
                for (uint i = 0; i < tetMeshBuffer.maxTetMeshes; i++)
                {
                    seenFractureGroup[i] = false;
                }
                delete [] seenFractureGroup;
#endif

                // Start with vert 0 on queue, set all other features unvisited.
                uint8_t* pTempBuffer = pTempBufferStart;
                uint* connectedVertsQueue = FmAllocFromBuffer<uint>(&pTempBuffer, tetMesh.maxVerts, pTempBufferEnd);
                FmReorderInfo* vertReorderInfo = FmAllocFromBuffer<FmReorderInfo>(&pTempBuffer, tetMesh.maxVerts, pTempBufferEnd);
                FmReorderInfo* tetReorderInfo = FmAllocFromBuffer<FmReorderInfo>(&pTempBuffer, tetMesh.numTets, pTempBufferEnd);
                FmReorderInfo* exteriorFaceReorderInfo = FmAllocFromBuffer<FmReorderInfo>(&pTempBuffer, tetMesh.maxExteriorFaces, pTempBufferEnd);

                uint queueIdx = 0;
                uint queueSize = 1;
                connectedVertsQueue[0] = 0;
                vertReorderInfo[0].dstIdx = 0;
                vertReorderInfo[0].remapIdx = 0;
                uint nextConnectedComponentVert = 1;
                for (uint vertId = 1; vertId < initialTetMeshNumVerts; vertId++)
                {
                    vertReorderInfo[vertId].dstIdx = FM_INVALID_ID;
                    vertReorderInfo[vertId].remapIdx = FM_INVALID_ID;
                }
                for (uint tetId = 0; tetId < initialTetMeshNumTets; tetId++)
                {
                    tetReorderInfo[tetId].dstIdx = FM_INVALID_ID;
                    tetReorderInfo[tetId].remapIdx = FM_INVALID_ID;
                }
                for (uint exteriorFaceId = 0; exteriorFaceId < initialTetMeshNumExteriorFaces; exteriorFaceId++)
                {
                    exteriorFaceReorderInfo[exteriorFaceId].dstIdx = FM_INVALID_ID;
                    exteriorFaceReorderInfo[exteriorFaceId].remapIdx = FM_INVALID_ID;
                }

                newMeshCounts.Zero();

                while (queueIdx < initialTetMeshNumVerts)
                {
                    uint vId = connectedVertsQueue[queueIdx];

                    // Search tet and vert neighbors
                    FmVertNeighbors& vertNeighbors = tetMesh.vertsNeighbors[vId];
                    uint* vertIncidentTets = &tetMesh.vertConnectivity.incidentTets[vertNeighbors.incidentTetsStart];
                    uint numIncidentTets = vertNeighbors.numIncidentTets;

                    // Need sum of all incident tets for new mesh.
                    newMeshNumIncidentTets += numIncidentTets;

                    for (uint itIdx = 0; itIdx < numIncidentTets; itIdx++)
                    {
                        uint incidentTetId = vertIncidentTets[itIdx];

                        uint tetIndex0 = tetMesh.tetsIndex0[incidentTetId];
                        FmTetVertIds tetVerts = tetMesh.tetsVertIds[incidentTetId];

                        // Add tet to list for new mesh
                        if (tetReorderInfo[incidentTetId].dstIdx == FM_INVALID_ID)
                        {
                            uint dstIdx = newMeshTetOffset + newMeshNumTets;
                            tetReorderInfo[incidentTetId].dstIdx = dstIdx;
                            tetReorderInfo[incidentTetId].remapIdx = newMeshNumTets;
                            if (dstIdx >= initialTetMeshNumTets)
                            {
                                tetReorderInfo[dstIdx].dstIdx = FM_INVALID_ID;  // Marks no valid data at destination
                                tetReorderInfo[dstIdx].remapIdx = FM_INVALID_ID;
                            }

                            // Check fracture group for this tet and add counts to bound required memory
                            uint fractureGroupId = tetMeshBuffer.tetReferences[tetIndex0].fractureGroupId;

                            if (!tetMeshBuffer.visitedFractureGroup[fractureGroupId])
                            {
                                tetMeshBuffer.visitedFractureGroup[fractureGroupId] = true;
                                newMeshCounts.Add(tetMeshBuffer.fractureGroupCounts[fractureGroupId]);
                            }

                            newMeshNumTets++;
                        }

                        // Add exterior faces to list for new mesh
                        FmTetFaceIncidentTetIds tetFaceIncidentTets = tetMesh.tetsFaceIncidentTetIds[incidentTetId];
                        for (uint faceIdx = 0; faceIdx < 4; faceIdx++)
                        {
                            if (FmIsExteriorFaceId(tetFaceIncidentTets.ids[faceIdx]))
                            {
                                uint efId = FmGetExteriorFaceId(tetFaceIncidentTets.ids[faceIdx]);
                                if (exteriorFaceReorderInfo[efId].dstIdx == FM_INVALID_ID)
                                {
                                    uint dstIdx = newMeshExteriorFaceOffset + newMeshNumExteriorFaces;
                                    exteriorFaceReorderInfo[efId].dstIdx = dstIdx;
                                    exteriorFaceReorderInfo[efId].remapIdx = newMeshNumExteriorFaces;
                                    if (dstIdx >= initialTetMeshNumExteriorFaces)
                                    {
                                        exteriorFaceReorderInfo[dstIdx].dstIdx = FM_INVALID_ID; // Marks no valid data at destination
                                        exteriorFaceReorderInfo[dstIdx].remapIdx = FM_INVALID_ID;
                                    }
                                    newMeshNumExteriorFaces++;
                                }
                            }
                        }

                        // Add neighbor verts to list for new mesh, and to queue
                        for (uint tetVertIdx = 0; tetVertIdx < 4; tetVertIdx++)
                        {
                            uint nvId = tetVerts.ids[tetVertIdx];

                            if (vertReorderInfo[nvId].dstIdx == FM_INVALID_ID)
                            {
                                FM_ASSERT(queueSize < initialTetMeshNumVerts);
                                connectedVertsQueue[queueSize] = nvId;
                                uint dstIdx = newMeshVertOffset + newMeshNumVerts;
                                vertReorderInfo[nvId].dstIdx = dstIdx;
                                vertReorderInfo[nvId].remapIdx = newMeshNumVerts;
                                if (dstIdx >= initialTetMeshNumVerts)
                                {
                                    vertReorderInfo[dstIdx].dstIdx = FM_INVALID_ID; // Marks no valid data at destination
                                    vertReorderInfo[dstIdx].remapIdx = FM_INVALID_ID;
                                }
                                queueSize++;
                                newMeshNumVerts++;
                            }
                        }
                    }

                    queueIdx++;

                    if (queueIdx == queueSize)
                    {
                        // If queueIdx equals queueSize, have consumed all vertices in a connected component.
                        // Init the arrays and counts in the new mesh.
                        // However, should only split off this new mesh if:
                        // - There is at least one spare mesh structure for any remainder,
                        //   or else this is the last new mesh
                        // - This isn't the only new mesh (otherwise can keep original)

                        bool isNewMesh = (numNewMeshesCreated > 0);
                        bool isBufferMeshFree = !isNewMesh || (nextFreeMeshIdx < tetMeshBuffer.maxTetMeshes);
                        bool isSceneMeshFree = !isNewMesh || (scene->numTetMeshesTotal < scene->maxTetMeshes);

                        // Report if have run out of space for a new mesh
                        if (!isBufferMeshFree)
                        {
                            FmAtomicOr(&scene->warningsReport.flags.val, FM_WARNING_FLAG_HIT_LIMIT_TET_MESH_BUFFER_MESHES);
                            FmAtomicWrite(&scene->warningsReport.tetMeshId.val, tetMeshBuffer.tetMeshes[0].objectId);
                        }
                        if (!isSceneMeshFree)
                        {
                            FmAtomicOr(&scene->warningsReport.flags.val, FM_WARNING_FLAG_HIT_LIMIT_SCENE_TET_MESHES);
                        }

                        uint spareSceneMeshIdx = isNewMesh ? scene->numTetMeshesTotal + 1 : scene->numTetMeshesTotal;
                        uint spareBufferMeshIdx = isNewMesh ? nextFreeMeshIdx + 1 : nextFreeMeshIdx;

                        bool isSpareBufferMesh = (spareBufferMeshIdx < tetMeshBuffer.maxTetMeshes) && (spareSceneMeshIdx < scene->maxTetMeshes);
                        bool isLastNewMesh = (queueIdx == initialTetMeshNumVerts);
                        bool isOnlyNewMesh = isLastNewMesh && !isNewMesh;

                        // Create new mesh
                        if (isBufferMeshFree
                            && isSceneMeshFree
                            && (isSpareBufferMesh || isLastNewMesh)
                            && !isOnlyNewMesh)
                        {
                            numNewMeshesCreated++;

                            // If first new mesh, modify original tetMesh, otherwise use next free buffer mesh
                            uint newMeshIdx = meshIdx;
                            if (isNewMesh)
                            {
                                newMeshIdx = nextFreeMeshIdx;
                                nextFreeMeshIdx++;
                                scene->numTetMeshesTotal++;
                            }

                            FM_ASSERT(newMeshIdx < tetMeshBuffer.maxTetMeshes);
                            FM_ASSERT(scene->numTetMeshesTotal <= scene->maxTetMeshes);

                            FmTetMesh& newTetMesh = tetMeshBuffer.tetMeshes[newMeshIdx];
                            FmMpcgSolverData& newSolverData = tetMeshBuffer.solverData[newMeshIdx];

                            uint newMeshMaxVerts = newMeshCounts.numVerts;
                            uint newMeshMaxAdjacentVerts = newMeshCounts.numVertAdjacentVerts;
                            uint newMeshMaxExteriorFaces = newMeshCounts.numExteriorFaces;
                            uint newMeshMaxBvhNodes = FmNumBvhNodes(newMeshMaxExteriorFaces);

                            // Init new mesh array pointers using the computed offsets.
                            // Not moving the incident tets for each vert, so all new meshes get the same incident tets array.
                            newTetMesh.vertsNeighbors = tetMesh.vertsNeighbors + newMeshVertOffset;
                            newTetMesh.vertsMass = tetMesh.vertsMass + newMeshVertOffset;
                            newTetMesh.vertsFlags = tetMesh.vertsFlags + newMeshVertOffset;
                            newTetMesh.vertsIndex0 = tetMesh.vertsIndex0 + newMeshVertOffset;
                            newTetMesh.vertsRestPos = tetMesh.vertsRestPos + newMeshVertOffset;
                            newTetMesh.vertsPos = tetMesh.vertsPos + newMeshVertOffset;
                            newTetMesh.vertsVel = tetMesh.vertsVel + newMeshVertOffset;
                            newTetMesh.vertsExtForce = tetMesh.vertsExtForce + newMeshVertOffset;
                            newTetMesh.vertsTetValues = tetMesh.vertsTetValues + newMeshVertOffset;
                            newTetMesh.vertConnectivity.incidentTets = tetMesh.vertConnectivity.incidentTets;
                            newTetMesh.vertConnectivity.numIncidentTets = newMeshNumIncidentTets;
                            newTetMesh.vertConnectivity.numAdjacentVerts = newMeshMaxAdjacentVerts; // Set upper bound; this will be recomputed before use in solver
                            newTetMesh.vertConnectivity.numIncidentTetsTotal = tetMesh.vertConnectivity.numIncidentTetsTotal;

                            newTetMesh.tetsMass = tetMesh.tetsMass + newMeshTetOffset;
                            newTetMesh.tetsFrictionCoeff = tetMesh.tetsFrictionCoeff + newMeshTetOffset;
                            newTetMesh.tetsFlags = tetMesh.tetsFlags + newMeshTetOffset;
                            newTetMesh.tetsShapeParams = tetMesh.tetsShapeParams + newMeshTetOffset;

                            newTetMesh.tetsRestDensity = tetMesh.tetsRestDensity + newMeshTetOffset;
                            newTetMesh.tetsMaxUnconstrainedSolveIterations = tetMesh.tetsMaxUnconstrainedSolveIterations + newMeshTetOffset;
                            newTetMesh.tetsStressMaterialParams = tetMesh.tetsStressMaterialParams + newMeshTetOffset;
                            newTetMesh.tetsDeformationMaterialParams = tetMesh.tetsDeformationMaterialParams + newMeshTetOffset;
                            newTetMesh.tetsFractureMaterialParams = tetMesh.tetsFractureMaterialParams ? tetMesh.tetsFractureMaterialParams + newMeshTetOffset : NULL;
                            newTetMesh.tetsPlasticityMaterialParams = tetMesh.tetsPlasticityMaterialParams ? tetMesh.tetsPlasticityMaterialParams + newMeshTetOffset : NULL;
                            newTetMesh.tetsStrainMag = tetMesh.tetsStrainMag + newMeshTetOffset;
                            newTetMesh.tetsIndex0 = tetMesh.tetsIndex0 + newMeshTetOffset;
                            newTetMesh.tetsRotation = tetMesh.tetsRotation + newMeshTetOffset;
                            newTetMesh.tetsFaceIncidentTetIds = tetMesh.tetsFaceIncidentTetIds + newMeshTetOffset;
                            newTetMesh.tetsVertIds = tetMesh.tetsVertIds + newMeshTetOffset;
                            newTetMesh.tetsStiffness = tetMesh.tetsStiffness + newMeshTetOffset;
                            newTetMesh.tetsToFracture = tetMesh.tetsToFracture ? tetMesh.tetsToFracture + newMeshTetOffset : NULL;
                            newTetMesh.tetsPlasticity = tetMesh.tetsPlasticity ? tetMesh.tetsPlasticity + newMeshTetOffset : NULL;

                            newTetMesh.exteriorFaces = tetMesh.exteriorFaces + newMeshExteriorFaceOffset;
                            newTetMesh.bvh.nodes = tetMesh.bvh.nodes + newMeshBvhNodeOffset;
                            newTetMesh.bvh.primBoxes = tetMesh.bvh.primBoxes + newMeshExteriorFaceOffset;
                            newTetMesh.bvh.mortonCodesSorted = tetMesh.bvh.mortonCodesSorted + newMeshExteriorFaceOffset;
                            newTetMesh.bvh.primIndicesSorted = tetMesh.bvh.primIndicesSorted + newMeshExteriorFaceOffset;
                            newTetMesh.bvh.numPrims = newMeshNumExteriorFaces;

                            newTetMesh.numVerts = newMeshNumVerts;
                            newTetMesh.numTets = newMeshNumTets;
                            newTetMesh.numExteriorFaces = newMeshNumExteriorFaces;
                            newTetMesh.maxVerts = newMeshMaxVerts;
                            newTetMesh.maxExteriorFaces = newMeshMaxExteriorFaces;
                            newTetMesh.numTetsToFracture = 0;
                            newTetMesh.numNewExteriorFaces = 0;

                            // Propagate parameters to all pieces
                            newTetMesh.gravityVector = tetMesh.gravityVector;
                            newTetMesh.frictionCoeff = tetMesh.frictionCoeff;
                            newTetMesh.removeKinematicStressThreshold = tetMesh.removeKinematicStressThreshold;
                            newTetMesh.extForceSpeedLimit = tetMesh.extForceSpeedLimit;
                            newTetMesh.resetSpeedLimit = tetMesh.resetSpeedLimit;
                            newTetMesh.sleepMaxSpeedThreshold = tetMesh.sleepMaxSpeedThreshold;
                            newTetMesh.sleepAvgSpeedThreshold = tetMesh.sleepAvgSpeedThreshold;
                            newTetMesh.sleepStableCount = tetMesh.sleepStableCount;

                            FmInitVelStats(&newTetMesh.velStats);

                            newTetMesh.collisionGroup = tetMesh.collisionGroup;
                            newTetMesh.flags = tetMesh.flags & ~FM_OBJECT_FLAG_NEEDS_CONNECTED_COMPONENTS;

                            newSolverData.kinematicFlags = solverData.kinematicFlags + newMeshVertOffset;
                            newSolverData.PInvDiag = solverData.PInvDiag + newMeshVertOffset;
                            newSolverData.A.rowStarts = solverData.A.rowStarts + newMeshRowStartOffset;
                            newSolverData.A.submats = solverData.A.submats + newMeshVertOffset + newMeshSubmatOffset;
                            newSolverData.A.indices = solverData.A.indices + newMeshVertOffset + newMeshSubmatOffset;
                            newSolverData.A.numRows = newMeshNumVerts;
                            newSolverData.b = solverData.b + newMeshVertOffset;
                            newSolverData.mass = solverData.mass + newMeshVertOffset;
                            newSolverData.maxVertAdjacentVerts = newMeshMaxAdjacentVerts;

                            newMeshVertOffset += newMeshMaxVerts;
                            newMeshTetOffset += newMeshNumTets;
                            newMeshExteriorFaceOffset += newMeshMaxExteriorFaces;
                            newMeshBvhNodeOffset += newMeshMaxBvhNodes;
                            newMeshRowStartOffset += newMeshMaxVerts * 2;  // extra rowStart to mark end of last row, 2x maxVerts worst case
                            newMeshSubmatOffset += newMeshMaxAdjacentVerts;

                            newMeshNumVerts = 0;
                            newMeshNumTets = 0;
                            newMeshNumExteriorFaces = 0;
                            newMeshNumIncidentTets = 0;
                        }

                        // Need to add new vert (next connected component) on queue.
                        // Continue a sequential search of verts to find next unvisited one.
                        if (queueIdx < initialTetMeshNumVerts)
                        {
                            newMeshCounts.Zero();

                            uint nextStartId = FM_INVALID_ID;
                            for (; nextConnectedComponentVert < initialTetMeshNumVerts; nextConnectedComponentVert++)
                            {
                                if (vertReorderInfo[nextConnectedComponentVert].dstIdx == FM_INVALID_ID)
                                {
                                    nextStartId = nextConnectedComponentVert;
                                    nextConnectedComponentVert++;
                                    break;
                                }
                            }
                            FM_ASSERT(nextStartId != FM_INVALID_ID);
                            connectedVertsQueue[queueIdx] = nextStartId;
                            uint dstIdx = newMeshVertOffset + newMeshNumVerts;
                            vertReorderInfo[nextStartId].dstIdx = dstIdx;
                            vertReorderInfo[nextStartId].remapIdx = newMeshNumVerts;
                            if (dstIdx >= initialTetMeshNumVerts)
                            {
                                vertReorderInfo[dstIdx].dstIdx = FM_INVALID_ID; // Marks no valid data at destination
                                vertReorderInfo[dstIdx].remapIdx = FM_INVALID_ID;
                            }
                            queueSize++;
                            newMeshNumVerts++;
                        }
                    }
                }

                // New mesh arrays/counts are initialized.
                // Remap all ids, then reorder feature data in place.  When writing data to a destination, swap out the data there 
                // and repeat on that feature.  Reset the dest index value to FM_INVALID_ID as each feature is moved.  Can just overwrite 
                // data in destination if data there is already moved, or was not valid data to start with (part of the padding).

                if (numNewMeshesCreated > 0)
                {
                    FM_ASSERT(newMeshVertOffset == initialTetMeshMaxVerts);
                    FM_ASSERT(newMeshTetOffset == initialTetMeshNumTets);
                    FM_ASSERT(newMeshExteriorFaceOffset == initialTetMeshMaxExteriorFaces);
                    FM_ASSERT(newMeshSubmatOffset == initialSolverDataMaxVertAdjacentVerts);

                    // Remap all ids.
                    // This relies on tetMesh pointers all being unchanged even though counts have changed.

                    // Vert tet ids
                    for (uint vId = 0; vId < initialTetMeshNumVerts; vId++)
                    {
                        FmVertNeighbors& vertNeighbors = tetMesh.vertsNeighbors[vId];
                        uint* vertIncidentTets = &tetMesh.vertConnectivity.incidentTets[vertNeighbors.incidentTetsStart];
                        uint numIncidentTets = vertNeighbors.numIncidentTets;
                        FM_ASSERT(vertNeighbors.incidentTetsStart + vertNeighbors.numIncidentTets <= tetMesh.vertConnectivity.numIncidentTetsTotal);

                        for (uint itIdx = 0; itIdx < numIncidentTets; itIdx++)
                        {
                            vertIncidentTets[itIdx] = tetReorderInfo[vertIncidentTets[itIdx]].remapIdx;
                        }
                    }

                    // Tet vert ids and incident tet ids
                    for (uint tId = 0; tId < initialTetMeshNumTets; tId++)
                    {
                        FmTetVertIds& tetVerts = tetMesh.tetsVertIds[tId];
                        tetVerts.ids[0] = vertReorderInfo[tetVerts.ids[0]].remapIdx;
                        tetVerts.ids[1] = vertReorderInfo[tetVerts.ids[1]].remapIdx;
                        tetVerts.ids[2] = vertReorderInfo[tetVerts.ids[2]].remapIdx;
                        tetVerts.ids[3] = vertReorderInfo[tetVerts.ids[3]].remapIdx;

                        uint id0 = tetMesh.tetsFaceIncidentTetIds[tId].ids[0];
                        uint id1 = tetMesh.tetsFaceIncidentTetIds[tId].ids[1];
                        uint id2 = tetMesh.tetsFaceIncidentTetIds[tId].ids[2];
                        uint id3 = tetMesh.tetsFaceIncidentTetIds[tId].ids[3];

                        tetMesh.tetsFaceIncidentTetIds[tId].ids[0] =
                            FmIsExteriorFaceId(id0) ?
                            FmMakeExteriorFaceId(exteriorFaceReorderInfo[FmGetExteriorFaceId(id0)].remapIdx) :
                            tetReorderInfo[id0].remapIdx;

                        tetMesh.tetsFaceIncidentTetIds[tId].ids[1] =
                            FmIsExteriorFaceId(id1) ?
                            FmMakeExteriorFaceId(exteriorFaceReorderInfo[FmGetExteriorFaceId(id1)].remapIdx) :
                            tetReorderInfo[id1].remapIdx;

                        tetMesh.tetsFaceIncidentTetIds[tId].ids[2] =
                            FmIsExteriorFaceId(id2) ?
                            FmMakeExteriorFaceId(exteriorFaceReorderInfo[FmGetExteriorFaceId(id2)].remapIdx) :
                            tetReorderInfo[id2].remapIdx;

                        tetMesh.tetsFaceIncidentTetIds[tId].ids[3] =
                            FmIsExteriorFaceId(id3) ?
                            FmMakeExteriorFaceId(exteriorFaceReorderInfo[FmGetExteriorFaceId(id3)].remapIdx) :
                            tetReorderInfo[id3].remapIdx;
                    }

                    // Exterior face id, tetId, edgeIncidentFaceIds
                    for (uint fId = 0; fId < initialTetMeshNumExteriorFaces; fId++)
                    {
                        FmExteriorFace& extFace = tetMesh.exteriorFaces[fId];
                        FM_ASSERT(extFace.GetId() == fId);
                        extFace.SetId(exteriorFaceReorderInfo[fId].remapIdx);
                        extFace.tetId = tetReorderInfo[extFace.tetId].remapIdx;
                        extFace.edgeIncidentFaceIds[0] = exteriorFaceReorderInfo[extFace.edgeIncidentFaceIds[0]].remapIdx;
                        extFace.edgeIncidentFaceIds[1] = exteriorFaceReorderInfo[extFace.edgeIncidentFaceIds[1]].remapIdx;
                        extFace.edgeIncidentFaceIds[2] = exteriorFaceReorderInfo[extFace.edgeIncidentFaceIds[2]].remapIdx;
                    }

                    // Move all features to destination positions

                    // Move verts
                    uint numVertsMoved = 0;
                    uint srcVertId = 0;
                    uint nextVertId = 1;
                    FmCCVertData srcVertData, dstVertData;
                    FmGetVertData(&tetMesh, srcVertId, srcVertData);

                    while (true)
                    {
                        uint dstVertId = vertReorderInfo[srcVertId].dstIdx;

                        FM_ASSERT(dstVertId != FM_INVALID_ID);

                        if (srcVertId != dstVertId)
                        {
                            // Save dst data if it hasn't been moved already
                            if (vertReorderInfo[dstVertId].dstIdx != FM_INVALID_ID)
                            {
                                FmGetVertData(&tetMesh, dstVertId, dstVertData);
                            }

                            // Move src vert data to dst
                            FmSetVertData(&tetMesh, dstVertId, srcVertData);
                        }

                        // Mark as moved
                        vertReorderInfo[srcVertId].dstIdx = FM_INVALID_ID;
                        numVertsMoved++;

                        if (numVertsMoved >= initialTetMeshNumVerts)
                            break;

                        // Next src data is either dst that was saved, or the next unmoved vert in line.
                        if (vertReorderInfo[dstVertId].dstIdx == FM_INVALID_ID)
                        {
                            srcVertId = FM_INVALID_ID;
                            for (; nextVertId < initialTetMeshNumVerts; nextVertId++)
                            {
                                if (vertReorderInfo[nextVertId].dstIdx != FM_INVALID_ID)
                                {
                                    srcVertId = nextVertId;
                                    nextVertId++;
                                    break;
                                }
                            }
                            FM_ASSERT(srcVertId != FM_INVALID_ID);
                            FmGetVertData(&tetMesh, srcVertId, srcVertData);
                        }
                        else
                        {
                            srcVertId = dstVertId;
                            srcVertData = dstVertData;
                        }
                    }

                    // Move tets
                    uint numTetsMoved = 0;
                    uint srcTetId = 0;
                    uint nextTetId = 1;
                    FmCCTetData srcTetData, dstTetData;
                    FmGetTetData(&tetMesh, srcTetId, srcTetData);
                    while (true)
                    {
                        uint dstTetId = tetReorderInfo[srcTetId].dstIdx;

                        if (srcTetId != dstTetId)
                        {
                            // Save dst data if it hasn't been moved already
                            if (tetReorderInfo[dstTetId].dstIdx != FM_INVALID_ID)
                            {
                                FmGetTetData(&tetMesh, dstTetId, dstTetData);
                            }

                            // Move src tet data to dst
                            FmSetTetData(&tetMesh, dstTetId, srcTetData);
                        }

                        // Mark as moved
                        tetReorderInfo[srcTetId].dstIdx = FM_INVALID_ID;
                        numTetsMoved++;

                        if (numTetsMoved >= initialTetMeshNumTets)
                            break;

                        // Next src data is either dst that was saved, or the next unmoved tet in line.
                        if (tetReorderInfo[dstTetId].dstIdx == FM_INVALID_ID)
                        {
                            srcTetId = FM_INVALID_ID;
                            for (; nextTetId < initialTetMeshNumTets; nextTetId++)
                            {
                                if (tetReorderInfo[nextTetId].dstIdx != FM_INVALID_ID)
                                {
                                    srcTetId = nextTetId;
                                    nextTetId++;
                                    break;
                                }
                            }
                            FM_ASSERT(srcTetId != FM_INVALID_ID);
                            FmGetTetData(&tetMesh, srcTetId, srcTetData);
                        }
                        else
                        {
                            srcTetId = dstTetId;
                            srcTetData = dstTetData;
                        }
                    }

                    // Move exterior faces
                    uint numExteriorFacesMoved = 0;
                    uint srcExteriorFaceId = 0;
                    uint nextExteriorFaceId = 1;
                    FmCCExteriorFaceData srcExteriorFaceData, dstExteriorFaceData;
                    FmGetExteriorFaceData(&tetMesh, srcExteriorFaceId, srcExteriorFaceData);
                    while (true)
                    {
                        uint dstExteriorFaceId = exteriorFaceReorderInfo[srcExteriorFaceId].dstIdx;

                        if (srcExteriorFaceId != dstExteriorFaceId)
                        {
                            // Save dst data if it hasn't been moved already
                            if (exteriorFaceReorderInfo[dstExteriorFaceId].dstIdx != FM_INVALID_ID)
                            {
                                FmGetExteriorFaceData(&tetMesh, dstExteriorFaceId, dstExteriorFaceData);
                            }

                            // Move src exterior face data to dst
                            FmSetExteriorFaceData(&tetMesh, dstExteriorFaceId, srcExteriorFaceData);
                        }

                        // Mark as moved
                        exteriorFaceReorderInfo[srcExteriorFaceId].dstIdx = FM_INVALID_ID;
                        numExteriorFacesMoved++;

                        if (numExteriorFacesMoved >= initialTetMeshNumExteriorFaces)
                            break;

                        // Next src data is either dst that was saved, or the next unmoved exterior face in line.
                        if (exteriorFaceReorderInfo[dstExteriorFaceId].dstIdx == FM_INVALID_ID)
                        {
                            srcExteriorFaceId = FM_INVALID_ID;
                            for (; nextExteriorFaceId < initialTetMeshNumExteriorFaces; nextExteriorFaceId++)
                            {
                                if (exteriorFaceReorderInfo[nextExteriorFaceId].dstIdx != FM_INVALID_ID)
                                {
                                    srcExteriorFaceId = nextExteriorFaceId;
                                    nextExteriorFaceId++;
                                    break;
                                }
                            }
                            FM_ASSERT(srcExteriorFaceId != FM_INVALID_ID);
                            FmGetExteriorFaceData(&tetMesh, srcExteriorFaceId, srcExteriorFaceData);
                        }
                        else
                        {
                            srcExteriorFaceId = dstExteriorFaceId;
                            srcExteriorFaceData = dstExteriorFaceData;
                        }
                    }
                }

                // Update adjacent vert offsets for all the new meshes created from tetMesh.
                // Update tet mesh buffer's maps to new mesh verts and tets.
                for (uint newMeshesIdx = 0; newMeshesIdx < numNewMeshesCreated; newMeshesIdx++)
                {
                    uint newMeshIdx = (newMeshesIdx == 0) ? meshIdx : newMeshesStart + newMeshesIdx - 1;

                    FmTetMesh& newTetMesh = tetMeshBuffer.tetMeshes[newMeshIdx];

                    newTetMesh.flags |= FM_OBJECT_FLAG_NEEDS_ADJACENT_VERT_OFFSETS;

                    // For newly created meshes, assign id and update map
                    if (newMeshesIdx > 0)
                    {
                        uint objectId = FmReserveId(&scene->freeTetMeshIds);
                        newTetMesh.objectId = objectId;
                        FmSetTetMeshPtrById(scene, objectId, &newTetMesh);
                        FmSetTetMeshIdxById(scene, objectId, FM_INVALID_ID);  // Will set in CollectTetMeshIds
                    }
                    else
                    {
                        FmSetTetMeshIdxById(scene, newTetMesh.objectId, FM_INVALID_ID);  // Will set in CollectTetMeshIds
                    }

                    // Update references to new mesh verts and tets
                    for (uint vertId = 0; vertId < newTetMesh.numVerts; vertId++)
                    {
                        if (!(newTetMesh.vertsFlags[vertId] & FM_VERT_FLAG_FRACTURE_COPY)) // Point to the original vertex, not the copies from fracture
                        {
                            uint vertIdx0 = newTetMesh.vertsIndex0[vertId];
                            FmVertReference& vertRef = tetMeshBuffer.vertReferences[vertIdx0];
                            vertRef.meshIdx = newMeshIdx;
                            vertRef.vertId = vertId;
                        }
                    }

                    for (uint tetId = 0; tetId < newTetMesh.numTets; tetId++)
                    {
                        uint tetIdx0 = newTetMesh.tetsIndex0[tetId];
                        FmTetReference& tetRef = tetMeshBuffer.tetReferences[tetIdx0];
                        tetRef.meshIdx = newMeshIdx;
                        tetRef.tetId = tetId;
                    }

#if FM_DEBUG_CHECKS
                    uint numGroups = 0;
                    FmFractureGroupCounts total;
                    for (uint i = 0; i < tetMeshBuffer.maxTetMeshes; i++)
                    {
                        seenFractureGroup[i] = false;
                    }
                    for (uint tetId = 0; tetId < newTetMesh.numTets; tetId++)
                    {
                        uint tetIdx0 = newTetMesh.tetsIndex0[tetId];
                        FmTetReference& tetRef = tetMeshBuffer.tetReferences[tetIdx0];
                        
                        uint fractureGroupId = tetRef.fractureGroupId;
                        if (!seenFractureGroup[fractureGroupId])
                        {
                            numGroups++;
                            FmFractureGroupCounts& fractureGroupCounts = tetMeshBuffer.fractureGroupCounts[fractureGroupId];
                            total.Add(fractureGroupCounts);
                            seenFractureGroup[fractureGroupId] = true;
                        }
                    }

                    if (numGroups > 0)
                    {
                        FM_ASSERT(newTetMesh.maxVerts == total.numVerts);
                        FM_ASSERT(newTetMesh.maxExteriorFaces == total.numExteriorFaces);

                        FmFractureGroupCounts current;
                        current.numVerts = newTetMesh.numVerts;
                        current.numTets = newTetMesh.numTets;
                        current.numExteriorFaces = newTetMesh.numExteriorFaces;
                        current.numVertAdjacentVerts = newTetMesh.vertConnectivity.numAdjacentVerts;
                        current.numVertIncidentTets = newTetMesh.vertConnectivity.numIncidentTets;

                        if (!total.GreaterEqual(current))
                        {
                            printf("numGroups: %u\n", numGroups);
                            printf("total: v: %u t: %u e: %u av: %u, it: %u\n",
                                total.numVerts, total.numTets, total.numExteriorFaces, total.numVertAdjacentVerts, total.numVertIncidentTets);
                            printf("current: v: %u t: %u e: %u av: %u, it: %u\n",
                                current.numVerts, current.numTets, current.numExteriorFaces, current.numVertAdjacentVerts, current.numVertIncidentTets);

                            FM_ASSERT(total.GreaterEqual(current));
                        }
                    }
#endif
                }

                tetMesh.flags &= ~FM_OBJECT_FLAG_NEEDS_CONNECTED_COMPONENTS;
            }
        }

        tetMeshBuffer.numTetMeshes = nextFreeMeshIdx;
    }

    // Update the scene's array of tet meshes, separating any fractured connected components into new meshes.
    void FmFindMeshConnectedComponents(FmScene* scene)
    {
        FmTetMeshBuffer** tetMeshBuffers = scene->tetMeshBuffers;
        uint numTetMeshBufferSlots = scene->numTetMeshBufferSlots;

        for (uint meshBufferIdx = 0; meshBufferIdx < numTetMeshBufferSlots; meshBufferIdx++)
        {
            FmTetMeshBuffer* pTetMeshBuffer = tetMeshBuffers[meshBufferIdx];

            if (!pTetMeshBuffer) // has been removed
            {
                continue;
            }

            FmFindTetMeshBufferConnectedComponents(scene, pTetMeshBuffer);
        }
    }
}