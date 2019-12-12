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
// Functions supporting creation of tet mesh topology and computing memory bounds for 
// fracture.
//---------------------------------------------------------------------------------------

#include "FEMFXScene.h"
#include "FEMFXFracture.h"
#include "FEMFXTetMesh.h"
#include "FEMFXRigidBody.h"
#include "FEMFXMeshCreation.h"

namespace AMD
{
    // Add tet id to the incident tets of a vertex
    void FmAddIncidentTetToSet(FmArray<uint>& vertTetIds, uint tetId)
    {
        uint numElems = (uint)vertTetIds.GetNumElems();
        for (uint i = 0; i < numElems; i++)
        {
            uint vTetId = vertTetIds[i];
            if (vTetId == tetId)
                return;
        }
        vertTetIds.Add(tetId);
    }

    // Finds set of tets incident to each vertex, based on tetVertIds array.
    // vertIncidentTets array of vectors must be sized >= numVerts.
    void FmFindVertIncidentTets(
        FmArray<uint>* vertIncidentTets,
        const FmTetVertIds* tetVertIds,
        uint numVerts, uint numTets)
    {
        for (uint vId = 0; vId < numVerts; vId++)
        {
            vertIncidentTets[vId].Clear();
        }
        for (uint tetId = 0; tetId < numTets; tetId++)
        {
            FmTetVertIds tetVerts = tetVertIds[tetId];
            FM_ASSERT(tetVerts.ids[0] < numVerts);
            FM_ASSERT(tetVerts.ids[1] < numVerts);
            FM_ASSERT(tetVerts.ids[2] < numVerts);
            FM_ASSERT(tetVerts.ids[3] < numVerts);
            FmAddIncidentTetToSet(vertIncidentTets[tetVerts.ids[0]], tetId);
            FmAddIncidentTetToSet(vertIncidentTets[tetVerts.ids[1]], tetId);
            FmAddIncidentTetToSet(vertIncidentTets[tetVerts.ids[2]], tetId);
            FmAddIncidentTetToSet(vertIncidentTets[tetVerts.ids[3]], tetId);
        }
    }

    void FmCreateBoxMesh(FmArray<uint>* vertIncidentTets)
    {
        FmAddIncidentTetToSet(vertIncidentTets[0], 0);
        FmAddIncidentTetToSet(vertIncidentTets[4], 0);
        FmAddIncidentTetToSet(vertIncidentTets[5], 0);
        FmAddIncidentTetToSet(vertIncidentTets[6], 0);

        FmAddIncidentTetToSet(vertIncidentTets[5], 1);
        FmAddIncidentTetToSet(vertIncidentTets[0], 1);
        FmAddIncidentTetToSet(vertIncidentTets[6], 1);
        FmAddIncidentTetToSet(vertIncidentTets[7], 1);

        FmAddIncidentTetToSet(vertIncidentTets[0], 2);
        FmAddIncidentTetToSet(vertIncidentTets[7], 2);
        FmAddIncidentTetToSet(vertIncidentTets[2], 2);
        FmAddIncidentTetToSet(vertIncidentTets[6], 2);

        FmAddIncidentTetToSet(vertIncidentTets[0], 3);
        FmAddIncidentTetToSet(vertIncidentTets[5], 3);
        FmAddIncidentTetToSet(vertIncidentTets[1], 3);
        FmAddIncidentTetToSet(vertIncidentTets[7], 3);

        FmAddIncidentTetToSet(vertIncidentTets[0], 4);
        FmAddIncidentTetToSet(vertIncidentTets[7], 4);
        FmAddIncidentTetToSet(vertIncidentTets[1], 4);
        FmAddIncidentTetToSet(vertIncidentTets[2], 4);

        FmAddIncidentTetToSet(vertIncidentTets[1], 5);
        FmAddIncidentTetToSet(vertIncidentTets[2], 5);
        FmAddIncidentTetToSet(vertIncidentTets[7], 5);
        FmAddIncidentTetToSet(vertIncidentTets[3], 5);
    }

    void FmInitBoxVerts(FmVector3* vertRestPositions, FmTetVertIds* tetVertIds, float cubeDimX, float cubeDimY, float cubeDimZ)
    {
        vertRestPositions[0] = FmInitVector3(-cubeDimX * 0.5f, -cubeDimY * 0.5f, -cubeDimZ * 0.5f);
        vertRestPositions[1] = FmInitVector3(cubeDimX*0.5f, -cubeDimY * 0.5f, -cubeDimZ * 0.5f);
        vertRestPositions[2] = FmInitVector3(-cubeDimX * 0.5f, cubeDimY*0.5f, -cubeDimZ * 0.5f);
        vertRestPositions[3] = FmInitVector3(cubeDimX*0.5f, cubeDimY*0.5f, -cubeDimZ * 0.5f);
        vertRestPositions[4] = FmInitVector3(-cubeDimX * 0.5f, -cubeDimY * 0.5f, cubeDimZ*0.5f);
        vertRestPositions[5] = FmInitVector3(cubeDimX*0.5f, -cubeDimY * 0.5f, cubeDimZ*0.5f);
        vertRestPositions[6] = FmInitVector3(-cubeDimX * 0.5f, cubeDimY*0.5f, cubeDimZ*0.5f);
        vertRestPositions[7] = FmInitVector3(cubeDimX*0.5f, cubeDimY*0.5f, cubeDimZ*0.5f);

        tetVertIds[0].ids[0] = 0;
        tetVertIds[0].ids[1] = 4;
        tetVertIds[0].ids[2] = 5;
        tetVertIds[0].ids[3] = 6;

        tetVertIds[1].ids[0] = 5;
        tetVertIds[1].ids[1] = 0;
        tetVertIds[1].ids[2] = 6;
        tetVertIds[1].ids[3] = 7;

        tetVertIds[2].ids[0] = 0;
        tetVertIds[2].ids[1] = 7;
        tetVertIds[2].ids[2] = 2;
        tetVertIds[2].ids[3] = 6;

        tetVertIds[3].ids[0] = 0;
        tetVertIds[3].ids[1] = 5;
        tetVertIds[3].ids[2] = 1;
        tetVertIds[3].ids[3] = 7;

        tetVertIds[4].ids[0] = 0;
        tetVertIds[4].ids[1] = 7;
        tetVertIds[4].ids[2] = 1;
        tetVertIds[4].ids[3] = 2;

        tetVertIds[5].ids[0] = 1;
        tetVertIds[5].ids[1] = 2;
        tetVertIds[5].ids[2] = 7;
        tetVertIds[5].ids[3] = 3;
    }

    FmTetMeshBuffer* FmCreateBoxTetMeshBuffer(const FmRigidBody& rigidBody)
    {
        // Create tet mesh for volume contacts
        float hx = rigidBody.dims[0];
        float hy = rigidBody.dims[1];
        float hz = rigidBody.dims[2];

        uint numVerts = 8;
        uint numTets = 6;

        FmArray<uint> vertIncidentTets[8];
        FmTetVertIds tetVertIds[6];
        FmVector3 vertRestPositions[8];

        FmCreateBoxMesh(vertIncidentTets);

        FmInitBoxVerts(vertRestPositions, tetVertIds, hx*2.0f, hy*2.0f, hz*2.0f);

        FmTetMeshBufferBounds bounds;
        FmComputeTetMeshBufferBounds(
            &bounds,
            NULL, NULL,
            vertIncidentTets, tetVertIds, NULL,
            numVerts, numTets, false);

        FmTetMeshBufferSetupParams tetMeshBufferParams;
        tetMeshBufferParams.numVerts = bounds.numVerts;
        tetMeshBufferParams.numTets = bounds.numTets;
        tetMeshBufferParams.maxVerts = bounds.maxVerts;
        tetMeshBufferParams.maxVertAdjacentVerts = bounds.maxVertAdjacentVerts;
        tetMeshBufferParams.numVertIncidentTets = bounds.numVertIncidentTets;
        tetMeshBufferParams.maxExteriorFaces = bounds.maxExteriorFaces;
        tetMeshBufferParams.maxTetMeshes = bounds.maxTetMeshes;
        tetMeshBufferParams.enablePlasticity = false;
        tetMeshBufferParams.enableFracture = false;
        tetMeshBufferParams.isKinematic = false;

        FmTetMesh* rbTetMesh;
        FmTetMeshBuffer* rbTetMeshBuffer = FmCreateTetMeshBuffer(tetMeshBufferParams, NULL, NULL, &rbTetMesh);

        FmInitVertState(rbTetMesh, vertRestPositions, FmMatrix3(rigidBody.state.quat), rigidBody.state.pos);

        FmTetMaterialParams materialParams;
        FmInitTetState(rbTetMesh, tetVertIds, materialParams);

        FmInitConnectivity(rbTetMesh, vertIncidentTets);

        return rbTetMeshBuffer;
    }

    void FmCreateRigidBodyBoxCollObj(FmRigidBody* rigidBody)
    {
        rigidBody->collisionObj = FmCreateBoxTetMeshBuffer(*rigidBody);
    }

    void FmDestroyRigidBodyCollObj(FmRigidBody* rigidBody)
    {
        if (rigidBody->collisionObj)
        {
            FmAlignedFree((FmTetMeshBuffer*)rigidBody->collisionObj);
            rigidBody->collisionObj = NULL;
        }
    }

    // Compute worst case number of adjacent vertices at a vertex.  The worst case is if all possible fractures occur at the vertex.
    // This can be analyzed by using the same process as in FmMakeMeshSplits(), detecting non-fracturing connected-components of tets that are incident to the vertex, 
    // and assuming each of these components will split off, and computing the number of adjacent verts for each component.
    struct FmComputeMaxAdjacentLocalTet
    {
        uint                       tetId;
        FmTetVertIds            tetVerts;
        uint                       componentId;
        FmTetFaceIncidentTetIds localTetFaceIncidentTets;  // indices of tets within tet assignment array
        uint16_t                   faceFractureDisabled;
        bool                       isKinematic;
    };

    struct FmComputeMaxVertAdjacentVertsWorkspace
    {
        FmFractureTetIdMapElement* tetIdMapElements;
        uint maxTetIdMapElements;
        FmComputeMaxAdjacentLocalTet* localTets;
        uint* tetsQueue;
        uint* componentIncidentTets;
    };

    void FmComputeMaxVertAdjacentVertsAfterFracture(
        uint* outMaxVerts,
        uint* outMaxVertAdjacentVerts,
        uint* outMaxVertIncidentTets,
        uint vertId, const uint* vertIncidentTets, uint numIncidentTets,
        uint fractureGroupId,
        const uint* tetFractureGroupIds,
        const FmTetVertIds* tetVertIds,
        const FmTetFaceIncidentTetIds* tetFaceIncidentTetIds,
        const uint16_t* tetsFlags,
        FmComputeMaxVertAdjacentVertsWorkspace& workspace)
    {
        uint maxVerts = 0;
        uint maxVertAdjacentVerts = 0;
        uint maxVertIncidentTets = 0;

        // Create map from global to local tet ids
        FmFractureTetIdMap tetIdMap;
        FmInitHashSet(&tetIdMap, workspace.tetIdMapElements, workspace.maxTetIdMapElements);

        for (uint localTetId = 0; localTetId < numIncidentTets; localTetId++)
        {
            uint globalTetId = vertIncidentTets[localTetId];
            bool foundInSet;
            FmInsertElement(&foundInSet, &tetIdMap, globalTetId)->id = localTetId;
        }

        // Initialize local tet data including localTetFaceIncidentTets which connects tets using local tet ids.
        for (uint itId = 0; itId < numIncidentTets; itId++)
        {
            uint incidentTetId = vertIncidentTets[itId];

            FmTetVertIds incidentTetVertIds = tetVertIds[incidentTetId];
            FmTetFaceIncidentTetIds tetFaceIncidentTets = tetFaceIncidentTetIds[incidentTetId];

            FmComputeMaxAdjacentLocalTet& localTet = workspace.localTets[itId];
            localTet.tetId = incidentTetId;
            localTet.tetVerts = incidentTetVertIds;

            uint vertCorner = FmFindTetCornerWithVertId(vertId, localTet.tetVerts);

            FM_ASSERT(localTet.tetVerts.ids[vertCorner] == vertId);

            localTet.componentId = FM_INVALID_ID;

            // Iterate over the faces incident to the input vertex (excluding face opposite)
            localTet.localTetFaceIncidentTets.ids[0] = FM_INVALID_ID;
            localTet.localTetFaceIncidentTets.ids[1] = FM_INVALID_ID;
            localTet.localTetFaceIncidentTets.ids[2] = FM_INVALID_ID;
            localTet.localTetFaceIncidentTets.ids[3] = FM_INVALID_ID;

            for (uint faceOffset = 1; faceOffset < 4; faceOffset++)
            {
                uint faceId = (vertCorner + faceOffset) % 4;
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

            if (tetsFlags)
            {
                uint16_t tetFlags = tetsFlags[incidentTetId];
                localTet.faceFractureDisabled = (uint16_t)((tetFlags / FM_TET_FLAG_FACE0_FRACTURE_DISABLED) & 0xf);
                localTet.isKinematic = FM_IS_SET(tetFlags, FM_TET_FLAG_KINEMATIC);
            }
            else
            {
                localTet.faceFractureDisabled = false;
                localTet.isKinematic = false;
            }
        }

        // Find connected components of tets which can't be split due to fracture disabled flags.
        // Later will allow fracture only between tets of different components.
        // Add face-connected tets to queue - component complete when all queue indices processed.

        uint numComponents = 0;
        uint numTetsReached = 0;

        for (uint itId = 0; itId < numIncidentTets; itId++)
        {
            // If reached a tet without a component, start a new component
            if (workspace.localTets[itId].componentId == FM_INVALID_ID)
            {
                // Init queue with first tet
                workspace.tetsQueue[0] = itId;
                uint numTetsOnQueue = 1;

                // Set component, marking as reached
                workspace.localTets[itId].componentId = numComponents;
                numTetsReached++;

                // Process queue tets, adding unreached connected tets to queue.
                uint queueIdx = 0;

                while (queueIdx < numTetsOnQueue)
                {
                    uint itAId = workspace.tetsQueue[queueIdx];
                    FmComputeMaxAdjacentLocalTet& tetA = workspace.localTets[itAId];

                    for (uint faceA = 0; faceA < 4; faceA++)
                    {
                        uint itBId = tetA.localTetFaceIncidentTets.ids[faceA];

                        if (itBId != FM_INVALID_ID)  // if connects to a tet on this face
                        {
                            FmComputeMaxAdjacentLocalTet& tetB = workspace.localTets[itBId];

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
                                workspace.tetsQueue[numTetsOnQueue] = itBId;
                                numTetsOnQueue++;

                                // Set connected component, marking as reached
                                tetB.componentId = numComponents;
                                numTetsReached++;
                            }
                        }
                    }

                    queueIdx++;
                }

                // Component finished 
                numComponents++;

                // Have a component of incident tets, but only want counts if it belongs to the fracture group being tallied.
                if (fractureGroupId == FM_INVALID_ID ||
                    fractureGroupId == tetFractureGroupIds[workspace.localTets[workspace.tetsQueue[0]].tetId])
                {
                    // One vertex copy needed for each component
                    maxVerts++;

                    // Add to the incident tets for the vertex
                    maxVertIncidentTets += numTetsOnQueue;

                    // Create incident tets list from the component and call GetNumVertAdjacentVerts().
                    // This gives the total adjacent verts for the component after splitting off.
                    for (uint i = 0; i < numTetsOnQueue; i++)
                    {
                        workspace.componentIncidentTets[i] = workspace.localTets[workspace.tetsQueue[i]].tetId;
                    }
                    maxVertAdjacentVerts += FmGetVertNumAdjacentVerts(workspace.componentIncidentTets, numTetsOnQueue, tetVertIds);
                }
            }
        }

        FM_ASSERT(numTetsReached == numIncidentTets);

        *outMaxVerts = maxVerts;
        *outMaxVertAdjacentVerts = maxVertAdjacentVerts;
        *outMaxVertIncidentTets = maxVertIncidentTets;
    }

    void FmComputeFractureGroupConnectivityCounts(
        FmFractureGroupCounts* outFractureGroupCounts,
        uint* vertsVisited,
        const FmArray<uint>* vertIncidentTets,
        uint fractureGroupId, const uint* fractureGroupTetIds, uint numTetsInFractureGroup,
        const uint* tetFractureGroupIds,
        const FmTetVertIds* tetVertIds,
        const FmTetFaceIncidentTetIds* tetFaceIncidentTetIds,
        const uint16_t* tetsFlags)
    {
        uint maxVertsInComponent = 0;
        uint maxVertIncidentTetsInComponent = 0;
        uint maxVertAdjacentVertsInComponent = 0;

        // Allocate workspace data
        FmComputeMaxVertAdjacentVertsWorkspace workspace;

        workspace.maxTetIdMapElements = FM_MAX_VERT_INCIDENT_TETS * 2;
        workspace.tetIdMapElements = (FmFractureTetIdMapElement*)FmAlignedMalloc(sizeof(FmFractureTetIdMapElement)*workspace.maxTetIdMapElements, FM_ALIGN_OF(FmFractureTetIdMapElement));
        workspace.localTets = (FmComputeMaxAdjacentLocalTet*)FmAlignedMalloc(sizeof(FmComputeMaxAdjacentLocalTet)*FM_MAX_VERT_INCIDENT_TETS, FM_ALIGN_OF(FmComputeMaxAdjacentLocalTet));

        workspace.tetsQueue = (uint*)FmAlignedMalloc(sizeof(uint)*FM_MAX_VERT_INCIDENT_TETS, FM_ALIGN_OF(uint));
        workspace.componentIncidentTets = (uint*)FmAlignedMalloc(sizeof(uint)*FM_MAX_VERT_INCIDENT_TETS, FM_ALIGN_OF(uint));

        // Mark verts of component tets as unvisited
        for (uint compTetId = 0; compTetId < numTetsInFractureGroup; compTetId++)
        {
            uint tetId = fractureGroupTetIds[compTetId];

            for (uint cornerId = 0; cornerId < 4; cornerId++)
            {
                uint vId = tetVertIds[tetId].ids[cornerId];

                vertsVisited[vId] = 0;
            }
        }

        // Process unvisited verts
        for (uint compTetId = 0; compTetId < numTetsInFractureGroup; compTetId++)
        {
            uint tetId = fractureGroupTetIds[compTetId];

            for (uint cornerId = 0; cornerId < 4; cornerId++)
            {
                uint vId = tetVertIds[tetId].ids[cornerId];

                if (vertsVisited[vId] == 0)
                {
                    uint maxVerts, maxVertAdjacentVerts, maxVertIncidentTets;

                    uint numIncidentTets = vertIncidentTets[vId].GetNumElems();
                    const uint* incidentTets = &vertIncidentTets[vId][0];

                    FmComputeMaxVertAdjacentVertsAfterFracture(
                        &maxVerts,
                        &maxVertAdjacentVerts,
                        &maxVertIncidentTets,
                        vId, incidentTets, numIncidentTets,
                        fractureGroupId,
                        tetFractureGroupIds,
                        tetVertIds,
                        tetFaceIncidentTetIds,
                        tetsFlags,
                        workspace);

                    maxVertsInComponent += maxVerts;
                    maxVertAdjacentVertsInComponent += maxVertAdjacentVerts;
                    maxVertIncidentTetsInComponent += maxVertIncidentTets;

                    vertsVisited[vId] = 1;
                }
            }
        }

        FmAlignedFree(workspace.tetIdMapElements);
        FmAlignedFree(workspace.localTets);
        FmAlignedFree(workspace.tetsQueue);
        FmAlignedFree(workspace.componentIncidentTets);

        outFractureGroupCounts->numVerts = maxVertsInComponent;
        outFractureGroupCounts->numVertAdjacentVerts = maxVertAdjacentVertsInComponent;
        outFractureGroupCounts->numVertIncidentTets = maxVertIncidentTetsInComponent;
    }

    void FmComputeBoundsAfterFracture(
        uint* outMaxVerts,
        uint* outMaxVertAdjacentVerts,
        uint* outMaxExteriorFaces,
        uint* outMaxTetMeshes,
        FmFractureGroupCounts* outFractureGroups,
        uint* outTetFractureGroupIds,
        FmArray<uint>* vertIncidentTets,
        const FmTetVertIds* tetVertIds,
        const FmTetFaceIncidentTetIds* tetFaceIncidentTetIds,
        const uint16_t* tetFlags,
        uint numVerts, uint numTets)
    {
        uint* vertsVisited = (uint*)FmAlignedMalloc(sizeof(uint)*numVerts, FM_ALIGN_OF(uint));
        uint* tetsQueue = (uint*)FmAlignedMalloc(sizeof(uint)*numTets, FM_ALIGN_OF(uint));

        uint maxVertAdjacentVerts = 0;
        uint maxVerts = 0;
        uint maxExteriorFaces = 0;
        uint maxTetMeshes = 0;

        uint numComponents = 0;
        uint numTetsReached = 0;

        // Mark tet component ids invalid
        for (uint tetId = 0; tetId < numTets; tetId++)
        {
            outTetFractureGroupIds[tetId] = FM_INVALID_ID;
        }

        for (uint tetId = 0; tetId < numTets; tetId++)
        {
            // If reached a tet without a component, start a new component
            if (outTetFractureGroupIds[tetId] == FM_INVALID_ID)
            {
                // Init queue with first tet
                tetsQueue[0] = tetId;
                uint numTetsOnQueue = 1;

                // Set component, marking as reached
                outTetFractureGroupIds[tetId] = numComponents;
                numTetsReached++;

                uint maxExteriorFacesInComponent = 0;

                // Process queue tets, adding unreached connected tets to queue.
                // Looking for connected components that remain after fracture, so traversing tets when non-fracture flag set.
                uint queueIdx = 0;

                while (queueIdx < numTetsOnQueue)
                {
                    uint tetIdA = tetsQueue[queueIdx];

                    for (uint faceA = 0; faceA < 4; faceA++)
                    {
                        uint tetIdB = tetFaceIncidentTetIds[tetIdA].ids[faceA];

                        if (FmIsExteriorFaceId(tetIdB))
                        {
                            maxExteriorFacesInComponent++;
                        }
                        else
                        {
                            // Will not add to queue if already reached
                            bool unreached = (outTetFractureGroupIds[tetIdB] == FM_INVALID_ID);

                            // Check if fracture disabled in either direction
                            uint faceB = FmFindTetFaceWithIncidentTetId(tetIdA, tetFaceIncidentTetIds[tetIdB]);

                            bool fractureDisabled = (tetFlags &&
                                ((tetFlags[tetIdA] & (FM_TET_FLAG_FACE0_FRACTURE_DISABLED << faceA))
                                    || (tetFlags[tetIdB] & (FM_TET_FLAG_FACE0_FRACTURE_DISABLED << faceB))
                                    || ((tetFlags[tetIdA] & FM_TET_FLAG_KINEMATIC) && (tetFlags[tetIdB] & FM_TET_FLAG_KINEMATIC))));

                            if (!fractureDisabled)
                            {
                                // Fracture-enabled face can become an exterior face
                                maxExteriorFacesInComponent++;
                            }

                            if (unreached && fractureDisabled)
                            {
                                // Put connected tet on queue
                                tetsQueue[numTetsOnQueue] = tetIdB;
                                numTetsOnQueue++;

                                // Set connected component, marking as reached
                                outTetFractureGroupIds[tetIdB] = numComponents;
                                numTetsReached++;
                            }
                        }
                    }

                    queueIdx++;
                }

                // Component (fracture group) finished
                uint fractureGroupId = numComponents;
                numComponents++;

                FmFractureGroupCounts& fractureGroupCounts = outFractureGroups[fractureGroupId];

                fractureGroupCounts.numTets = numTetsOnQueue;
                fractureGroupCounts.numExteriorFaces = maxExteriorFacesInComponent;

                FmComputeFractureGroupConnectivityCounts(
                    &fractureGroupCounts,
                    vertsVisited,
                    vertIncidentTets,
                    fractureGroupId, tetsQueue, numTetsOnQueue,
                    outTetFractureGroupIds,
                    tetVertIds,
                    tetFaceIncidentTetIds,
                    tetFlags);

                maxVerts += fractureGroupCounts.numVerts;
                maxVertAdjacentVerts += fractureGroupCounts.numVertAdjacentVerts;
                maxExteriorFaces += fractureGroupCounts.numExteriorFaces;
                maxTetMeshes += 1;
            }
        }

        FM_ASSERT(numTetsReached == numTets);

        FmAlignedFree(vertsVisited);
        FmAlignedFree(tetsQueue);

        *outMaxVerts = maxVerts;
        *outMaxVertAdjacentVerts = maxVertAdjacentVerts;
        *outMaxExteriorFaces = maxExteriorFaces;
        *outMaxTetMeshes = maxTetMeshes;
    }

    // Compute bounds on maxVerts, maxVertAdjacentVerts, maxExteriorFaces, and maxTetMeshes assuming maximum amount of fracture allowed by the supplied tet flags.
    // If enableFracture true, outFractureGroupCounts and outTetFractureGroupIds should be sized to the number of tets, otherwise may be NULL
    // tetFlags expected to be an OR of FM_TET_FLAG_* values, or may be NULL signifying flags 0
    void FmComputeTetMeshBufferBounds(
        FmTetMeshBufferBounds* outBounds,
        FmFractureGroupCounts* outFractureGroups,
        uint* outTetFractureGroupIds,
        FmArray<uint>* vertIncidentTets,
        const FmTetVertIds* tetVertIds,
        const uint16_t* tetFlags,
        uint numVerts, uint numTets, bool enableFracture)
    {
        outBounds->numVerts = numVerts;
        outBounds->numTets = numTets;

        FmTetFaceIncidentTetIds* tetFaceIncidentTetIds = (FmTetFaceIncidentTetIds*)FmAlignedMalloc(sizeof(FmTetFaceIncidentTetIds)*numTets, FM_ALIGN_OF(FmTetFaceIncidentTetIds));

        // Create tet face incident tets, and get initial number of exterior faces
        uint numExteriorFaces = FmFindTetFaceIncidentTets(tetFaceIncidentTetIds, vertIncidentTets, tetVertIds, numTets);

        // Total vert incident tets is a constant
        uint numVertIncidentTets = 0;
        for (uint vId = 0; vId < numVerts; vId++)
        {
            numVertIncidentTets += vertIncidentTets[vId].GetNumElems();
        }

        outBounds->numVertIncidentTets = numVertIncidentTets;

        if (enableFracture)
        {
            FM_ASSERT(outFractureGroups != NULL);
            FM_ASSERT(outTetFractureGroupIds != NULL);

            FmComputeBoundsAfterFracture(
                &outBounds->maxVerts,
                &outBounds->maxVertAdjacentVerts,
                &outBounds->maxExteriorFaces,
                &outBounds->maxTetMeshes,
                outFractureGroups,
                outTetFractureGroupIds,
                vertIncidentTets,
                tetVertIds,
                tetFaceIncidentTetIds,
                tetFlags,
                numVerts, numTets);
        }
        else
        {
            // For non-fracturing case, sum initial vert adjacent verts
            uint numVertAdjacentVerts = 0;
            for (uint vId = 0; vId < numVerts; vId++)
            {
                uint numIncidentTets = vertIncidentTets[vId].GetNumElems();
                uint* incidentTets = &vertIncidentTets[vId][0];

                numVertAdjacentVerts += FmGetVertNumAdjacentVerts(incidentTets, numIncidentTets, tetVertIds);
            }
            outBounds->maxVertAdjacentVerts = numVertAdjacentVerts;

            outBounds->maxVerts = numVerts;
            outBounds->maxExteriorFaces = numExteriorFaces;
            outBounds->maxTetMeshes = 1;

            // One fracture group, with same counts as totals
            if (outFractureGroups)
            {
                outFractureGroups[0].numVerts = numVerts;
                outFractureGroups[0].numVertAdjacentVerts = numVertAdjacentVerts;
                outFractureGroups[0].numVertIncidentTets = numVertIncidentTets;
                outFractureGroups[0].numTets = numTets;
                outFractureGroups[0].numExteriorFaces = numExteriorFaces;
            }

            if (outTetFractureGroupIds)
            {
                for (uint tId = 0; tId < numTets; tId++)
                {
                    outTetFractureGroupIds[tId] = 0;
                }
            }
        }

        FmAlignedFree(tetFaceIncidentTetIds);
    }
}