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
// Fracturing of FEM tetrahedral meshes on face boundaries
//---------------------------------------------------------------------------------------

#include "FEMFXCommonInternal.h"
#include "FEMFXFracture.h"
#include "FEMFXScene.h"
#include "FEMFXThreadTempMemory.h"
#include "FEMFXSolverMath.h"
#include "FEMFXSort.h"

namespace AMD
{
    // Break kinematic vertex constraints when no connected face is constrained
    void FmBreakVertConstraints(FmTetMesh* tetMesh)
    {
        bool removedKinematicFlag = false;

        uint numVerts = tetMesh->numVerts;
        for (uint vertId = 0; vertId < numVerts; vertId++)
        {
            if (FM_ALL_SET(tetMesh->vertsFlags[vertId], FM_VERT_FLAG_KINEMATIC | FM_VERT_FLAG_KINEMATIC_REMOVABLE))
            {
                // Gather data for tets incident to vertex
                FmVertNeighbors& vertNeighbors = tetMesh->vertsNeighbors[vertId];
                uint* vertIncidentTets = &tetMesh->vertConnectivity.incidentTets[vertNeighbors.incidentTetsStart];
                uint numIncidentTets = vertNeighbors.numIncidentTets;

                FM_ASSERT(numIncidentTets > 0);

                bool removeConstraint = true;
                for (uint itId = 0; itId < numIncidentTets; itId++)
                {
                    uint incidentTetId = vertIncidentTets[itId];

                    uint16_t tetFlags = tetMesh->tetsFlags[incidentTetId];

                    if (FM_IS_SET(tetFlags, FM_TET_FLAG_KINEMATIC))
                    {
                        removeConstraint = false;
                        break;
                    }

                    FmTetVertIds tetVerts = tetMesh->tetsVertIds[incidentTetId];

                    uint numConstrainedVerts =
                        (uint)((bool)(tetMesh->vertsFlags[tetVerts.ids[0]] & FM_VERT_FLAG_KINEMATIC) +
                        (bool)(tetMesh->vertsFlags[tetVerts.ids[1]] & FM_VERT_FLAG_KINEMATIC) +
                            (bool)(tetMesh->vertsFlags[tetVerts.ids[2]] & FM_VERT_FLAG_KINEMATIC) +
                            (bool)(tetMesh->vertsFlags[tetVerts.ids[3]] & FM_VERT_FLAG_KINEMATIC));

                    if (numConstrainedVerts >= 3)
                    {
                        removeConstraint = false;
                        break;
                    }
                }

                if (removeConstraint)
                {
                    tetMesh->vertsFlags[vertId] &= ~FM_VERT_FLAG_KINEMATIC;
                    removedKinematicFlag = true;
                }
            }
        }

        if (removedKinematicFlag)
        {
            tetMesh->flags |= FM_OBJECT_FLAG_REMOVED_KINEMATIC;
        }
    }

    static inline void FmInitSplitVertData(FmTetMesh* tetMesh, uint dstVertId, uint srcVertId)
    {
        tetMesh->vertsNeighbors[dstVertId] = tetMesh->vertsNeighbors[srcVertId];
        tetMesh->vertsMass[dstVertId] = 0.0f;
        tetMesh->vertsFlags[dstVertId] = FM_VERT_FLAG_FRACTURED | FM_VERT_FLAG_FRACTURE_COPY;
        tetMesh->vertsIndex0[dstVertId] = tetMesh->vertsIndex0[srcVertId];
        tetMesh->vertsRestPos[dstVertId] = tetMesh->vertsRestPos[srcVertId];
        tetMesh->vertsPos[dstVertId] = tetMesh->vertsPos[srcVertId];
        tetMesh->vertsVel[dstVertId] = tetMesh->vertsVel[srcVertId];
        tetMesh->vertsExtForce[dstVertId] = FmInitVector3(0.0f);
        tetMesh->vertsTetValues[dstVertId] = tetMesh->vertsTetValues[srcVertId];
    }

    // Post-process after stress-based fracture making additional splits between groups of tets 
    // incident on a vert that are not connected by a face.  
    // This breaks one-vertex or two-vertex connections between these groups, so pieces don't 
    // hang by a thread.
    //
    // NOTE:
    // This still leaves a case where multiple exterior surfaces meet at an edge, when
    // all tets connected to this edge are connected by faces (around both verts of the edge).  
    // Later the edge may be broken if further fracture breaks face connections around either 
    // edge vert.   This case complicates updating the edge links between exterior faces and 
    // assigning edge features.
    //
    // Currently am treating this case as a hole in the mesh (although zero area), and connecting 
    // the exterior faces through it.  Also requiring the edge feature to be redundantly assigned in 
    // all of the meeting surfaces.   Then if the edge is broken by later fracture, no updates to the 
    // edge links or assignments are needed.
    // 
    // This means later code can restrict updates to topology and assignments to new exterior faces 
    // (in the above case, the edge may be split without any new exterior face containing the edge).

    void FmSplitFaceConnectedGroups(FmTetMesh* tetMesh, uint8_t* pTempBuffer, uint8_t* pTempBufferEnd)
    {
        FmSplitFaceConnectedLocalTet* localTets = FmAllocFromBuffer<FmSplitFaceConnectedLocalTet>(&pTempBuffer, FM_MAX_VERT_INCIDENT_TETS, pTempBufferEnd);
        uint* componentTetCounts = FmAllocFromBuffer<uint>(&pTempBuffer, FM_MAX_VERT_INCIDENT_TETS, pTempBufferEnd);

        // Hash map to map the adjacent tet ids to local ids
        FmFractureTetIdMap tetIdMap;
        const uint maxTetIdMapElements = FM_MAX_VERT_INCIDENT_TETS * 2;
        FmFractureTetIdMapElement* tetIdMapElements = FmAllocFromBuffer<FmFractureTetIdMapElement>(&pTempBuffer, maxTetIdMapElements, pTempBufferEnd);

        uint numVerts = tetMesh->numVerts;
        uint maxVerts = tetMesh->maxVerts;
        for (uint vertId = 0; vertId < numVerts; vertId++)
        {
            // Gather data for tets incident to vertex
            FmVertNeighbors& vertNeighbors = tetMesh->vertsNeighbors[vertId];
            uint* vertIncidentTets = &tetMesh->vertConnectivity.incidentTets[vertNeighbors.incidentTetsStart];
            uint numIncidentTets = vertNeighbors.numIncidentTets;

            FM_ASSERT(numIncidentTets > 0);

            // Create map from global to local tet ids
            FmInitHashSet(&tetIdMap, tetIdMapElements, maxTetIdMapElements);

            for (uint localTetId = 0; localTetId < numIncidentTets; localTetId++)
            {
                uint globalTetId = vertIncidentTets[localTetId];
                bool foundInSet;
                FmInsertElement(&foundInSet, &tetIdMap, globalTetId)->id = localTetId;
            }

            for (uint itId = 0; itId < numIncidentTets; itId++)
            {
                uint incidentTetId = vertIncidentTets[itId];

                float tetMass = tetMesh->tetsMass[incidentTetId];
                uint16_t tetFlags = tetMesh->tetsFlags[incidentTetId];
                FmTetVertIds tetVertIds = tetMesh->tetsVertIds[incidentTetId];
                FmTetFaceIncidentTetIds tetFaceIncidentTetIds = tetMesh->tetsFaceIncidentTetIds[incidentTetId];

                FmSplitFaceConnectedLocalTet& localTet = localTets[itId];
                localTet.tetId = incidentTetId;
                localTet.componentId = FM_INVALID_ID;
                localTet.tetMass = tetMass;
                localTet.tetVerts = tetVertIds;
                localTet.tetFaceIncidentTets = tetFaceIncidentTetIds;
                localTet.isKinematic = FM_IS_SET(tetFlags, FM_TET_FLAG_KINEMATIC);

                uint splitCorner = FmFindTetCornerWithVertId(vertId, localTet.tetVerts);

                // Iterate over the faces incident to the split vertex (excluding face id = split corner)
                localTet.localTetFaceIncidentTets.ids[0] = FM_INVALID_ID;
                localTet.localTetFaceIncidentTets.ids[1] = FM_INVALID_ID;
                localTet.localTetFaceIncidentTets.ids[2] = FM_INVALID_ID;
                localTet.localTetFaceIncidentTets.ids[3] = FM_INVALID_ID;

                for (uint faceOffset = 1; faceOffset < 4; faceOffset++)
                {
                    uint faceId = (splitCorner + faceOffset) % 4;
                    uint globalTetId = localTet.tetFaceIncidentTets.ids[faceId];

                    if (FmIsExteriorFaceId(globalTetId))
                    {
                        localTet.localTetFaceIncidentTets.ids[faceId] = FM_INVALID_ID;
                    }
                    else
                    {
                        bool foundInSet;
                        uint localTetId = FmInsertElement(&foundInSet, &tetIdMap, globalTetId)->id;
                        FM_ASSERT(foundInSet);
                        localTet.localTetFaceIncidentTets.ids[faceId] = localTetId;
                    }
                }

                componentTetCounts[itId] = 0;
            }

            // Find connected components of tets connected by faces
            uint* tetsQueue = FmAllocFromBuffer<uint>(&pTempBuffer, FM_MAX_VERT_INCIDENT_TETS, pTempBufferEnd);

            uint numComponents = 0;
            uint numTetsReached = 0;
            uint originalVertexComponentId = 0; // one component with FM_TET_FLAG_KINEMATIC will be attached to original vertex

            for (uint itId = 0; itId < numIncidentTets; itId++)
            {
                // If reached a tet without a component, start a new component
                if (localTets[itId].componentId == FM_INVALID_ID)
                {
                    // Init queue with first tet
                    tetsQueue[0] = itId;
                    uint numTetsOnQueue = 1;

                    // Set component, marking as reached
                    localTets[itId].componentId = numComponents;
                    numTetsReached++;

                    bool componentHasKinematicTet = localTets[itId].isKinematic;

                    // Process queue tets, adding unreached connected tets to queue.
                    // Compute average projection of component to decide which side of fracture to put it on.
                    uint queueIdx = 0;
                    while (queueIdx < numTetsOnQueue)
                    {
                        uint itAId = tetsQueue[queueIdx];
                        FmSplitFaceConnectedLocalTet& tetA = localTets[itAId];

                        for (uint faceA = 0; faceA < 4; faceA++)
                        {
                            uint itBId = tetA.localTetFaceIncidentTets.ids[faceA];

                            if (itBId != FM_INVALID_ID)  // if connects to a tet on this face
                            {
                                FmSplitFaceConnectedLocalTet& tetB = localTets[itBId];

                                // Will not add to queue if already reached
                                bool unreached = tetB.componentId == FM_INVALID_ID;

                                if (unreached)
                                {
                                    // Put connected tet on queue
                                    tetsQueue[numTetsOnQueue] = itBId;
                                    numTetsOnQueue++;

                                    // Set connected component, marking as reached
                                    tetB.componentId = numComponents;
                                    numTetsReached++;

                                    componentHasKinematicTet = componentHasKinematicTet || tetB.isKinematic;
                                }
                            }
                        }

                        queueIdx++;
                    }

                    if (componentHasKinematicTet)
                    {
                        originalVertexComponentId = numComponents;
                    }

                    numComponents++;
                }
            }

            FM_ASSERT(numTetsReached == numIncidentTets);

            FM_ASSERT(numComponents > 0);

            int numNewVerts = numComponents - 1;
            uint newVertsStartId = tetMesh->numVerts;

            // Should be guaranteed by bounds checked in FmMakeMeshSplits
            FM_ASSERT(tetMesh->numVerts + numNewVerts <= maxVerts);
            if (numNewVerts > 0 && tetMesh->numVerts + numNewVerts <= maxVerts)
            {
                tetMesh->vertsFlags[vertId] |= FM_VERT_FLAG_FRACTURED;

                // Create new vertices for the additional groups
                for (int nId = 0; nId < numNewVerts; nId++)
                {
                    uint newVertId = tetMesh->numVerts;
                    tetMesh->numVerts++;

                    FmInitSplitVertData(tetMesh, newVertId, vertId);
                }

                // Loop over tets, change vert ids in split-off groups and distribute tets to original and new vertices.
                tetMesh->vertsMass[vertId] = 0.0f;

                for (uint itId = 0; itId < numIncidentTets; itId++)
                {
                    FmSplitFaceConnectedLocalTet& localTet = localTets[itId];

                    uint tetId = localTet.tetId;

                    // Pick vertex for tet connected component.
                    uint assignVertId = vertId; // original vertex
                    if (localTet.componentId < originalVertexComponentId)
                    {
                        assignVertId = newVertsStartId + localTet.componentId;
                    }
                    else if (localTet.componentId > originalVertexComponentId)
                    {
                        assignVertId = newVertsStartId + localTet.componentId - 1;
                    }

                    FM_ASSERT(localTet.componentId >= 0 && localTet.componentId < numComponents);
                    FM_ASSERT(assignVertId < tetMesh->numVerts);

                    // Replace vert in tet
                    FmTetVertIds newVertIds;
                    newVertIds.ids[0] = localTet.tetVerts.ids[0] == vertId ? assignVertId : localTet.tetVerts.ids[0];
                    newVertIds.ids[1] = localTet.tetVerts.ids[1] == vertId ? assignVertId : localTet.tetVerts.ids[1];
                    newVertIds.ids[2] = localTet.tetVerts.ids[2] == vertId ? assignVertId : localTet.tetVerts.ids[2];
                    newVertIds.ids[3] = localTet.tetVerts.ids[3] == vertId ? assignVertId : localTet.tetVerts.ids[3];
                    tetMesh->tetsVertIds[tetId] = newVertIds;

                    tetMesh->vertsMass[assignVertId] += 0.25f * localTet.tetMass;

                    // Get counts of incident tets, to later partition the original incident tet array
                    componentTetCounts[localTet.componentId]++;
                }

                // Setup incident tet arrays for new verts
                vertNeighbors.numIncidentTets = componentTetCounts[originalVertexComponentId];
                vertNeighbors.numAdjacentVerts = 0;  // Recalculated in FmUpdateAdjacentVertOffsets
                uint incidentTetsStart = vertNeighbors.incidentTetsStart + vertNeighbors.numIncidentTets;

                for (int nId = 0; nId < numNewVerts; nId++)
                {
                    FmVertNeighbors& newVertNeighbors = tetMesh->vertsNeighbors[newVertsStartId + nId];
                    newVertNeighbors.incidentTetsStart = incidentTetsStart;
                    uint componentId = (nId >= (int)originalVertexComponentId) ? nId + 1 : nId;
                    newVertNeighbors.numIncidentTets = componentTetCounts[componentId];
                    newVertNeighbors.numAdjacentVerts = 0;
                    incidentTetsStart += componentTetCounts[componentId];
                }

                // Distribute the tets
                uint* incidentTetsArray = tetMesh->vertConnectivity.incidentTets;
                for (uint itId = 0; itId < numIncidentTets; itId++)
                {
                    FmSplitFaceConnectedLocalTet& localTet = localTets[itId];

                    uint assignVertId = vertId;
                    if (localTet.componentId < originalVertexComponentId)
                    {
                        assignVertId = newVertsStartId + localTet.componentId;
                    }
                    else if (localTet.componentId > originalVertexComponentId)
                    {
                        assignVertId = newVertsStartId + localTet.componentId - 1;
                    }
                    FmVertNeighbors& assignVertNeighbors = tetMesh->vertsNeighbors[assignVertId];

                    uint incidentTetIndex = assignVertNeighbors.incidentTetsStart + assignVertNeighbors.numIncidentTets - componentTetCounts[localTet.componentId];

                    incidentTetsArray[incidentTetIndex] = localTet.tetId;

                    componentTetCounts[localTet.componentId]--;
                }
            }
        }
    }

    class FmCompareTetsToFracture
    {
    public:

        FmCompareTetsToFracture(void* inUserData) { (void)inUserData; }

        inline bool operator ()(const FmTetToFracture& tetA, const FmTetToFracture& tetB)
        {
            return tetA.tetId < tetB.tetId;
        }

        inline int Compare(const FmTetToFracture& tetA, const FmTetToFracture& tetB)
        {
            return FM_QSORT_INCREASING_RETVAL(tetA.tetId, tetB.tetId);
        }
    };

    // Compute fracture on all tets in tetsToFracture list.
    // As in "Interactive Virtual Materials", a tet vertex is chosen as the location of the split, a duplicate vertex is created and incident tets are 
    // divided at faces and distributed between the original and new vertex according to which side of the chosen split plane they fall on.  
    // Vertices are preferred for splitting if they continue previous fractures, marked as "crack tips".   Applying this by itself can result in groups 
    // of tets connected only by a single vertex or edge.  A post-process is included to make additional vertex splits to avoid that.

    bool FmMakeMeshSplits(FmScene* scene, FmTetMesh* tetMesh)
    {
        bool didSplit = false;
        bool removedKinematicFlag = FM_IS_SET(tetMesh->flags, FM_OBJECT_FLAG_REMOVED_KINEMATIC);

        const int minTets = 3;

        if (tetMesh->numTets < 2*minTets)
        {
            return false;
        }

        // Get temp memory for fracture processing at each vertex.
        uint workerIndex = scene->taskSystemCallbacks.GetTaskSystemWorkerIndex();
        uint8_t* pTempBufferStart = scene->threadTempMemoryBuffer->buffers[workerIndex];
        uint8_t* pTempBufferEnd = pTempBufferStart + scene->threadTempMemoryBuffer->numBytesPerBuffer;
        uint8_t* pTempBuffer = pTempBufferStart;

        tetMesh->numNewExteriorFaces = 0;

        // Hash map to map the adjacent tet ids to local ids
        FmFractureTetIdMap tetIdMap;
        const uint maxTetIdMapElements = FM_MAX_VERT_INCIDENT_TETS * 2;
        FmFractureTetIdMapElement* tetIdMapElements = FmAllocFromBuffer<FmFractureTetIdMapElement>(&pTempBuffer, maxTetIdMapElements, pTempBufferEnd);

        FmMakeSplitsLocalTet* localTets = FmAllocFromBuffer<FmMakeSplitsLocalTet>(&pTempBuffer, FM_MAX_VERT_INCIDENT_TETS, pTempBufferEnd);

        FmRandomState randomState(tetMesh->objectId);

        // Compute fracture on all tets in tetsToFracture list, as in "Interactive Virtual Materials".
        uint maxVerts = tetMesh->maxVerts;
        uint numTetsToFracture = FmAtomicRead(&tetMesh->numTetsToFracture.val);

        // Needed for determinism with parallel fracture tests
        FmSort<FmTetToFracture, FmCompareTetsToFracture>(tetMesh->tetsToFracture, numTetsToFracture, NULL);

        for (uint tId = 0; tId < numTetsToFracture; tId++)
        {
            FmTetToFracture fractureTet = tetMesh->tetsToFracture[tId];
            uint tetId = fractureTet.tetId;

            FmTetVertIds fractureTetVertIds = tetMesh->tetsVertIds[tetId];

#define FM_RANDOMIZE_FRACTURE_DIR 1
#if FM_RANDOMIZE_FRACTURE_DIR 
            const float deg2rad = 3.14159265359f / 180.0f;
            float rotationAngle = randomState.RandomFloatMinusOneToOne() * 45.0f * deg2rad;
            float normX = randomState.RandomFloatZeroToOne();
            float normY = randomState.RandomFloatZeroToOne();
            float normZ = randomState.RandomFloatZeroToOne();
            FmVector3 rotationAxis = normalize(FmInitVector3(normX, normY, normZ));
            fractureTet.fractureDirection = mul(FmMatrix3::rotation(rotationAngle, rotationAxis), fractureTet.fractureDirection);
#endif

            bool crackTipFlags[4];
            crackTipFlags[0] = (tetMesh->vertsFlags[fractureTetVertIds.ids[0]] & FM_VERT_FLAG_CRACK_TIP) != 0;
            crackTipFlags[1] = (tetMesh->vertsFlags[fractureTetVertIds.ids[1]] & FM_VERT_FLAG_CRACK_TIP) != 0;
            crackTipFlags[2] = (tetMesh->vertsFlags[fractureTetVertIds.ids[2]] & FM_VERT_FLAG_CRACK_TIP) != 0;
            crackTipFlags[3] = (tetMesh->vertsFlags[fractureTetVertIds.ids[3]] & FM_VERT_FLAG_CRACK_TIP) != 0;

            uint16_t tetFlags = tetMesh->tetsFlags[tetId];
            bool onFracturableFace[4];
            onFracturableFace[0] = !FM_ALL_SET(tetFlags, FM_TET_FLAG_FACE1_FRACTURE_DISABLED | FM_TET_FLAG_FACE2_FRACTURE_DISABLED | FM_TET_FLAG_FACE3_FRACTURE_DISABLED);
            onFracturableFace[1] = !FM_ALL_SET(tetFlags, FM_TET_FLAG_FACE0_FRACTURE_DISABLED | FM_TET_FLAG_FACE2_FRACTURE_DISABLED | FM_TET_FLAG_FACE3_FRACTURE_DISABLED);
            onFracturableFace[2] = !FM_ALL_SET(tetFlags, FM_TET_FLAG_FACE0_FRACTURE_DISABLED | FM_TET_FLAG_FACE1_FRACTURE_DISABLED | FM_TET_FLAG_FACE3_FRACTURE_DISABLED);
            onFracturableFace[3] = !FM_ALL_SET(tetFlags, FM_TET_FLAG_FACE0_FRACTURE_DISABLED | FM_TET_FLAG_FACE1_FRACTURE_DISABLED | FM_TET_FLAG_FACE2_FRACTURE_DISABLED);

            // Find vertex on fracturable face, preferring crack tip.
            // Start search on random corner.
            uint baseTetCorner = randomState.RandomUint() % 4;
            uint splitTetCorner = FM_INVALID_ID;
            for (uint offset = 0; offset < 4; offset++)
            {
                uint corner = (baseTetCorner + offset) % 4;

                if (onFracturableFace[corner])
                {
                    splitTetCorner = corner;

                    if (crackTipFlags[corner])
                    {
                        break;
                    }
                }
            }

            if (splitTetCorner == FM_INVALID_ID)
            {
                continue;
            }

            uint splitVertId = fractureTetVertIds.ids[splitTetCorner];

            // Skip vertex if fracture already run on it
            if (FM_IS_SET(tetMesh->vertsFlags[splitVertId], FM_VERT_FLAG_FRACTURED))
            {
                continue;
            }

            // Using undeformed positions, because fracture direction was left in space of undeformed tet
            FmVector3 splitVertPos = tetMesh->vertsRestPos[splitVertId];

            // Use fracture direction to split tets incident on vert.
            // Gather data from incident tets.
            FmVertNeighbors& splitVertNeighbors = tetMesh->vertsNeighbors[splitVertId];

            uint* splitVertIncidentTets = &tetMesh->vertConnectivity.incidentTets[splitVertNeighbors.incidentTetsStart];
            uint numIncidentTets = splitVertNeighbors.numIncidentTets;

            // Create map from global to local tet ids
            FmInitHashSet(&tetIdMap, tetIdMapElements, maxTetIdMapElements);

            for (uint localTetId = 0; localTetId < numIncidentTets; localTetId++)
            {
                uint globalTetId = splitVertIncidentTets[localTetId];
                bool foundInSet;
                FmInsertElement(&foundInSet, &tetIdMap, globalTetId)->id = localTetId;
            }

            // Initialize local tet data including localTetFaceIncidentTets which connects tets using local tet ids.
            bool isKinematicTet = false;
            for (uint itId = 0; itId < numIncidentTets; itId++)
            {
                uint incidentTetId = splitVertIncidentTets[itId];

                float incidentTetMass = tetMesh->tetsMass[incidentTetId];
                uint16_t incidentTetFlags = tetMesh->tetsFlags[incidentTetId];

                FmTetVertIds incidentTetVertIds = tetMesh->tetsVertIds[incidentTetId];
                FmVector3 incidentTetCentroid =
                    (tetMesh->vertsRestPos[incidentTetVertIds.ids[0]] +
                        tetMesh->vertsRestPos[incidentTetVertIds.ids[1]] +
                        tetMesh->vertsRestPos[incidentTetVertIds.ids[2]] +
                        tetMesh->vertsRestPos[incidentTetVertIds.ids[3]]) * 0.25f;

                float incidentTetProj = dot(incidentTetCentroid - splitVertPos, fractureTet.fractureDirection);

                FmTetFaceIncidentTetIds tetFaceIncidentTets = tetMesh->tetsFaceIncidentTetIds[incidentTetId];

                FmMakeSplitsLocalTet& localTet = localTets[itId];
                localTet.tetId = incidentTetId;
                localTet.tetVerts = incidentTetVertIds;

                uint splitCorner = FmFindTetCornerWithVertId(splitVertId, localTet.tetVerts);

                FM_ASSERT(localTet.tetVerts.ids[splitCorner] == splitVertId);

                localTet.componentId = FM_INVALID_ID;
                localTet.newTetFaceIncidentTets = tetFaceIncidentTets;

                // Iterate over the faces incident to the split vertex (excluding face id = split corner)
                localTet.localTetFaceIncidentTets.ids[0] = FM_INVALID_ID;
                localTet.localTetFaceIncidentTets.ids[1] = FM_INVALID_ID;
                localTet.localTetFaceIncidentTets.ids[2] = FM_INVALID_ID;
                localTet.localTetFaceIncidentTets.ids[3] = FM_INVALID_ID;

                for (uint faceOffset = 1; faceOffset < 4; faceOffset++)
                {
                    uint faceId = (splitCorner + faceOffset) % 4;
                    uint globalTetId = tetFaceIncidentTets.ids[faceId];

                    if (FmIsExteriorFaceId(globalTetId))
                    {
                        localTet.localTetFaceIncidentTets.ids[faceId] = FM_INVALID_ID;
                    }
                    else
                    {
                        bool foundInSet;
                        uint localTetId = FmInsertElement(&foundInSet, &tetIdMap, globalTetId)->id;
                        FM_ASSERT(foundInSet);
                        localTet.localTetFaceIncidentTets.ids[faceId] = localTetId;
                    }
                }

                localTet.tetMass = incidentTetMass;
                localTet.projectionOnFractureDir = incidentTetProj;
                localTet.faceFractureDisabled = (uint16_t)((incidentTetFlags / FM_TET_FLAG_FACE0_FRACTURE_DISABLED) & 0xf);
                localTet.isKinematic = FM_IS_SET(incidentTetFlags, FM_TET_FLAG_KINEMATIC);
                localTet.crackTipId0 = FM_INVALID_ID;
                localTet.crackTipId1 = FM_INVALID_ID;
                localTet.positiveSide = (incidentTetProj > 0.0f);

                // Set if kinematic tet found and which side it's on.
                // Later will ensure kinematic tets are on the negative side, which stays connected to the original vertex.
                if (localTet.isKinematic)
                {
                    isKinematicTet = true;
                }
            }

            // Find connected components of tets which can't be split due to fracture disabled flags.
            // Later will allow fracture only between tets of different components.
            // Add face-connected tets to queue - component complete when all queue indices processed.
            uint* tetsQueue = FmAllocFromBuffer<uint>(&pTempBuffer, FM_MAX_VERT_INCIDENT_TETS, pTempBufferEnd);

            uint numComponents = 0;
            uint numTetsReached = 0;

            bool isPositiveTet = false;
            bool isNegativeTet = false;
            for (uint itId = 0; itId < numIncidentTets; itId++)
            {
                // If reached a tet without a component, start a new component
                if (localTets[itId].componentId == FM_INVALID_ID)
                {
                    // Init queue with first tet
                    tetsQueue[0] = itId;
                    uint numTetsOnQueue = 1;

                    // Set component, marking as reached
                    localTets[itId].componentId = numComponents;
                    numTetsReached++;

                    bool componentHasKinematicTet = localTets[itId].isKinematic;

                    // Process queue tets, adding unreached connected tets to queue.
                    // Compute average projection of component to decide which side of fracture to put it on.
                    uint queueIdx = 0;
                    float componentProjSum = 0.0f;

                    while (queueIdx < numTetsOnQueue)
                    {
                        uint itAId = tetsQueue[queueIdx];
                        FmMakeSplitsLocalTet& tetA = localTets[itAId];

                        // Compute average projection of group
                        componentProjSum += tetA.projectionOnFractureDir;

                        for (uint faceA = 0; faceA < 4; faceA++)
                        {
                            uint itBId = tetA.localTetFaceIncidentTets.ids[faceA];

                            if (itBId != FM_INVALID_ID)  // if connects to a tet on this face
                            {
                                FmMakeSplitsLocalTet& tetB = localTets[itBId];

                                // Will not add to queue if already reached
                                bool unreached = tetB.componentId == FM_INVALID_ID;

                                // Check if fracture disabled in either direction
                                uint faceB = FmFindTetFaceWithIncidentTetId(itAId, tetB.localTetFaceIncidentTets);

                                bool fractureDisabled =
                                    (tetA.faceFractureDisabled & (1 << faceA))
                                    || (tetB.faceFractureDisabled & (1 << faceB))
                                    || (tetA.isKinematic && tetB.isKinematic);

                                if (unreached && fractureDisabled)
                                {
                                    // Put connected tet on queue
                                    tetsQueue[numTetsOnQueue] = itBId;
                                    numTetsOnQueue++;

                                    // Set connected component, marking as reached
                                    tetB.componentId = numComponents;
                                    numTetsReached++;

                                    componentHasKinematicTet = componentHasKinematicTet || tetB.isKinematic;
                                }
                            }
                        }

                        queueIdx++;
                    }

                    // Component finished 
                    numComponents++;

                    bool positiveSide;

                    if (isKinematicTet)
                    {
                        // All components with kinematic tets placed on negative side, to stay connected to original vertex.
                        // Putting other components on the positive side.
                        positiveSide = !componentHasKinematicTet;
                    }
                    else
                    {
                        // Assign side of fracture based on component projection.
                        positiveSide = (componentProjSum > 0.0f);

                        // Ensure some fracture will occur, when fracture disabled on some faces
                        if (numTetsReached == numIncidentTets    // this is last component
                            && numComponents < numIncidentTets)  // fracture was disabled on one of the incident faces
                        {
                            if (!isPositiveTet)
                            {
                                positiveSide = true;
                            }
                            else if (!isNegativeTet)
                            {
                                positiveSide = false;
                            }
                        }
                    }

                    // Assign side to all tets of component
                    for (uint i = 0; i < numTetsOnQueue; i++)
                    {
                        localTets[tetsQueue[i]].positiveSide = positiveSide;
                    }

                    isPositiveTet = isPositiveTet || positiveSide;
                    isNegativeTet = isNegativeTet || !positiveSide;
                }
            }

            FM_ASSERT(numTetsReached == numIncidentTets);

            // No split possible if only one non-fracturing component, or if all tets on one side
            if (numComponents <= 1 || !isPositiveTet || !isNegativeTet)
            {
                continue;
            }

            // Divide the tets on the positive side of the fracture plane from the negative side.
            // Validity depends on being able to divide all the positive and negative tets.
            // The grouping into connected components above ensures that none of these divisions are
            // prevented by the flags disabling fracture between tets.

            uint newVertId = tetMesh->numVerts;
            uint exteriorFaceIdStart = tetMesh->numExteriorFaces + tetMesh->numNewExteriorFaces;
            uint numSplitExteriorFaces = 0;

            // Init new vert
            FmInitSplitVertData(tetMesh, newVertId, splitVertId);

            bool fractureDisabledFace = false;
            bool exceededMaxExteriorFaces = false;

            // For face-connected tets on opposite sides, break face connections and mark crack tip verts.
            for (uint itAId = 0; itAId < numIncidentTets && !exceededMaxExteriorFaces && !fractureDisabledFace; itAId++)
            {
                FmMakeSplitsLocalTet& tetA = localTets[itAId];

                // Break face connections and mark new crack tips
                for (uint faceIdx = 0; faceIdx < 4; faceIdx++)
                {
                    uint itBId = tetA.localTetFaceIncidentTets.ids[faceIdx];

                    if (itBId == FM_INVALID_ID)
                    {
                        continue;
                    }

                    FmMakeSplitsLocalTet& tetB = localTets[itBId];

                    uint faceA, faceB;
                    bool matchingFace = FmHasMatchingFace(&faceA, &faceB, itAId, itBId, tetA.localTetFaceIncidentTets, tetB.localTetFaceIncidentTets);
                    bool fractureDisabled =
                        (tetA.faceFractureDisabled & (1 << faceA))
                        || (tetB.faceFractureDisabled & (1 << faceB))
                        || (tetA.isKinematic && tetB.isKinematic);

                    if (matchingFace
                        && !fractureDisabled
                        && tetA.positiveSide != tetB.positiveSide)
                    {
                        FM_ASSERT(tetA.componentId != tetB.componentId);

                        // Mark crack tips
                        FmFaceVertIds tetAFaceVerts;
                        FmGetFaceVertIds(&tetAFaceVerts, faceA, tetA.tetVerts);
                        tetA.crackTipId0 = (tetAFaceVerts.ids[0] == splitVertId) ? tetAFaceVerts.ids[1] : tetAFaceVerts.ids[0];
                        tetA.crackTipId1 = (tetAFaceVerts.ids[2] == splitVertId) ? tetAFaceVerts.ids[1] : tetAFaceVerts.ids[2];

                        // Break face links.
                        // Set the incident ids to exterior faces ids instead.
                        uint tetAExteriorFaceId = exteriorFaceIdStart + numSplitExteriorFaces++;
                        uint tetBExteriorFaceId = exteriorFaceIdStart + numSplitExteriorFaces++;

                        if (tetBExteriorFaceId >= tetMesh->maxExteriorFaces)
                        {
                            exceededMaxExteriorFaces = true;
                            break;
                        }

                        tetA.newTetFaceIncidentTets.ids[faceA] = FmMakeExteriorFaceId(tetAExteriorFaceId);
                        tetB.newTetFaceIncidentTets.ids[faceB] = FmMakeExteriorFaceId(tetBExteriorFaceId);

                        // Tentatively create new exterior faces.
                        FmExteriorFace& tetAExteriorFace = tetMesh->exteriorFaces[tetAExteriorFaceId];
                        FmExteriorFace& tetBExteriorFace = tetMesh->exteriorFaces[tetBExteriorFaceId];

                        tetAExteriorFace.SetId(tetAExteriorFaceId);
                        tetAExteriorFace.ClearAssignments();
                        tetAExteriorFace.tetId = tetA.tetId;
                        tetAExteriorFace.faceId = faceA;
                        tetAExteriorFace.opposingTetId = tetB.tetId;
                        tetAExteriorFace.edgeIncidentFaceIds[0] = FM_INVALID_ID;
                        tetAExteriorFace.edgeIncidentFaceIds[1] = FM_INVALID_ID;
                        tetAExteriorFace.edgeIncidentFaceIds[2] = FM_INVALID_ID;

                        tetBExteriorFace.SetId(tetBExteriorFaceId);
                        tetBExteriorFace.ClearAssignments();
                        tetBExteriorFace.tetId = tetB.tetId;
                        tetBExteriorFace.faceId = faceB;
                        tetBExteriorFace.opposingTetId = tetA.tetId;
                        tetBExteriorFace.edgeIncidentFaceIds[0] = FM_INVALID_ID;
                        tetBExteriorFace.edgeIncidentFaceIds[1] = FM_INVALID_ID;
                        tetBExteriorFace.edgeIncidentFaceIds[2] = FM_INVALID_ID;

                        tetA.localTetFaceIncidentTets.ids[faceA] = FM_INVALID_ID;
                        tetB.localTetFaceIncidentTets.ids[faceB] = FM_INVALID_ID;
                    }
                }
            }

            if (exceededMaxExteriorFaces)
            {
                tetMesh->flags |= FM_OBJECT_FLAG_HIT_MAX_EXTERIOR_FACES;
            }

            if (!exceededMaxExteriorFaces && !fractureDisabledFace)
            {
                // Test for >= minTets before committing split
                bool commitSplit = true;

                for (uint itId = 0; itId < numIncidentTets; itId++)
                {
                    FmMakeSplitsLocalTet& localTet = localTets[itId];
                    uint numTets =
                        1UL +
                        (!FmIsExteriorFaceId(localTet.newTetFaceIncidentTets.ids[0])) +
                        (!FmIsExteriorFaceId(localTet.newTetFaceIncidentTets.ids[1])) +
                        (!FmIsExteriorFaceId(localTet.newTetFaceIncidentTets.ids[2])) +
                        (!FmIsExteriorFaceId(localTet.newTetFaceIncidentTets.ids[3]));

                    for (uint fId = 0; fId < 4 && numTets < minTets; fId++)
                    {
                        uint localId = localTet.localTetFaceIncidentTets.ids[fId];
                        uint globalId = localTet.newTetFaceIncidentTets.ids[fId];

                        if (localId != FM_INVALID_ID)
                        {
                            FM_ASSERT(localId < numIncidentTets);
                            FmTetFaceIncidentTetIds neighborTetFaceIncidentTets = localTets[localId].newTetFaceIncidentTets;
                            numTets +=
                                (!FmIsExteriorFaceId(neighborTetFaceIncidentTets.ids[0])) +
                                (!FmIsExteriorFaceId(neighborTetFaceIncidentTets.ids[1])) +
                                (!FmIsExteriorFaceId(neighborTetFaceIncidentTets.ids[2])) +
                                (!FmIsExteriorFaceId(neighborTetFaceIncidentTets.ids[3])) - 1;
                        }
                        else if (!FmIsExteriorFaceId(globalId))
                        {
                            FmTetFaceIncidentTetIds neighborTetFaceIncidentTets = tetMesh->tetsFaceIncidentTetIds[globalId];
                            numTets +=
                                (!FmIsExteriorFaceId(neighborTetFaceIncidentTets.ids[0])) +
                                (!FmIsExteriorFaceId(neighborTetFaceIncidentTets.ids[1])) +
                                (!FmIsExteriorFaceId(neighborTetFaceIncidentTets.ids[2])) +
                                (!FmIsExteriorFaceId(neighborTetFaceIncidentTets.ids[3])) - 1;
                        }
                    }

                    if (numTets < minTets)
                    {
                        commitSplit = false;
                        break;
                    }
                }

                FM_ASSERT(tetMesh->numVerts < maxVerts);

                if (commitSplit && tetMesh->numVerts < maxVerts)
                {
                    didSplit = true;

                    tetMesh->vertsFlags[splitVertId] |= FM_VERT_FLAG_FRACTURED;

                    tetMesh->numVerts++;
                    tetMesh->numNewExteriorFaces += numSplitExteriorFaces;

                    // Partition tets between original and new vertex.
                    uint splitVertTetIdx = 0;
                    uint numSplitVertTets = 0;
                    uint newVertTetIdx = numIncidentTets - 1;   // fill in tets for new vertex from end of existing array
                    uint numNewVertTets = 0;
                    float newVertMass = 0.0f;
                    float splitVertMass = 0.0f;

                    FmVertNeighbors& newVertNeighbors = tetMesh->vertsNeighbors[newVertId];

                    for (uint itId = 0; itId < numIncidentTets; itId++)
                    {
                        FmMakeSplitsLocalTet& localTet = localTets[itId];

                        // Substitute new vertex id for tets on positive side
                        if (localTet.positiveSide)
                        {
                            FmTetVertIds newVertIds;
                            newVertIds.ids[0] = localTet.tetVerts.ids[0] == splitVertId ? newVertId : localTet.tetVerts.ids[0];
                            newVertIds.ids[1] = localTet.tetVerts.ids[1] == splitVertId ? newVertId : localTet.tetVerts.ids[1];
                            newVertIds.ids[2] = localTet.tetVerts.ids[2] == splitVertId ? newVertId : localTet.tetVerts.ids[2];
                            newVertIds.ids[3] = localTet.tetVerts.ids[3] == splitVertId ? newVertId : localTet.tetVerts.ids[3];
                            tetMesh->tetsVertIds[localTet.tetId] = newVertIds;

                            splitVertIncidentTets[newVertTetIdx] = localTet.tetId;
                            newVertTetIdx--;
                            numNewVertTets++;
                            newVertMass += 0.25f * localTet.tetMass;
                        }
                        else
                        {
                            splitVertIncidentTets[splitVertTetIdx] = localTet.tetId;
                            splitVertTetIdx++;
                            numSplitVertTets++;
                            splitVertMass += 0.25f * localTet.tetMass;
                        }

                        tetMesh->tetsFaceIncidentTetIds[localTet.tetId] = localTet.newTetFaceIncidentTets;
                        if (localTet.crackTipId0 != FM_INVALID_ID)
                        {
                            tetMesh->vertsFlags[localTet.crackTipId0] |= FM_VERT_FLAG_CRACK_TIP;
                            tetMesh->vertsFlags[localTet.crackTipId1] |= FM_VERT_FLAG_CRACK_TIP;
                        }
                    }

                    // Set incident tet arrays on verts, mark for updating adjacent vertex count/offsets after fracture complete
                    splitVertNeighbors.numIncidentTets = numSplitVertTets;

                    newVertNeighbors.incidentTetsStart = splitVertNeighbors.incidentTetsStart + numIncidentTets - numNewVertTets;
                    newVertNeighbors.numIncidentTets = numNewVertTets;
                    newVertNeighbors.numAdjacentVerts = 1;

                    tetMesh->vertsFlags[splitVertId] &= ~FM_VERT_FLAG_CRACK_TIP;

                    tetMesh->vertsMass[newVertId] = newVertMass;
                    tetMesh->vertsMass[splitVertId] = splitVertMass;
                }
            }
        }

        if (didSplit)
        {
            FmSplitFaceConnectedGroups(tetMesh, pTempBufferStart, pTempBufferEnd);
        }

        if (didSplit || removedKinematicFlag)
        {
            FmBreakVertConstraints(tetMesh);
        }

        return didSplit;
    }

}