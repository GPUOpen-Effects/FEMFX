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
// Implementation of the simulation pipeline
//---------------------------------------------------------------------------------------

#include "FEMFXSvd3x3.h"
#include "FEMFXMpcgSolverSetup.h"
#include "FEMFXParallelFor.h"
#include "FEMFXBoxCcd.h"
#include "FEMFXScene.h"
#include "FEMFXSleeping.h"
#include "FEMFXFracture.h"
#include "FEMFXUpdateTetState.h"
#include "FEMFXThreadTempMemory.h"
#include "FEMFXConstraintSolver.h"
#include "FEMFXSort.h"

#if FM_SOA_TET_MATH
#include "FEMFXSoaTetMath.h"
#endif

#if FM_USE_TRACE
bool gFEMFXTraceEnabled = false;
#endif

int FmAssertBreak()
{
#ifdef _DEBUG
    assert(0);
#endif
    return -1;
}

namespace AMD
{
#if FM_TIMINGS
    double gFEMFXStartTime = 0.0;
    double gFEMFXMeshComponentsTime = 0.0;
    double gFEMFXImplicitStepTime = 0.0;
    double gFEMFXBroadPhaseTime = 0.0;
    double gFEMFXMeshContactsTime = 0.0;
    double gFEMFXConstraintIslandsTime = 0.0;
    double gFEMFXConstraintSolveTime = 0.0;
    double gFEMFXTotalStepTime = 0.0;
#endif

#if FM_DEBUG_MESHES
    FmTetMesh* gFEMFXPreConstraintSolveMeshes = NULL;
    FmRigidBody* gFEMFXPreConstraintSolveRigidBodies = NULL;
    uint gFEMFXNumPreConstraintSolveMeshes = 0;
    uint gFEMFXNumPreConstraintSolveRigidBodies = 0;
#endif

    // Sort by decreasing num verts
    class FmCompareSolverSize
    {
        FmScene* scene;

    public:
        FmCompareSolverSize(void* inScene)
        {
            scene = (FmScene*)inScene;
        }

        inline bool operator ()(uint tetMeshIdA, uint tetMeshIdB)
        {
            FmTetMesh& tetMeshA = *FmGetTetMeshPtrById(*scene, tetMeshIdA);
            FmTetMesh& tetMeshB = *FmGetTetMeshPtrById(*scene, tetMeshIdB);

            return tetMeshA.numVerts > tetMeshB.numVerts;
        }

        inline int Compare(uint tetMeshIdA, uint tetMeshIdB)
        {
            FmTetMesh& tetMeshA = *FmGetTetMeshPtrById(*scene, tetMeshIdA);
            FmTetMesh& tetMeshB = *FmGetTetMeshPtrById(*scene, tetMeshIdB);

            return FM_QSORT_DECREASING_RETVAL(tetMeshA.numVerts, tetMeshB.numVerts);
        }
    };

    void FmCollectTetMeshIds(FmScene* scene)
    {
        FmTetMeshBuffer** tetMeshBuffers = scene->tetMeshBuffers;
        uint numTetMeshBufferSlots = scene->numTetMeshBufferSlots;
        uint maxTetMeshes = scene->maxTetMeshes;

        uint numAwakeTetMeshes = 0;
        uint numSleepingTetMeshes = 0;
        for (uint meshBufferIdx = 0; meshBufferIdx < numTetMeshBufferSlots; meshBufferIdx++)
        {
            FmTetMeshBuffer* pTetMeshBuffer = tetMeshBuffers[meshBufferIdx];

            if (!pTetMeshBuffer) // has been removed
            {
                continue;
            }

            FmTetMeshBuffer& tetMeshBuffer = *pTetMeshBuffer;

            for (uint meshIdx = 0; meshIdx < tetMeshBuffer.numTetMeshes; meshIdx++)
            {
                FmTetMesh& tetMesh = tetMeshBuffer.tetMeshes[meshIdx];
                FmSetTetMeshIdxById(scene, tetMesh.objectId, FM_INVALID_ID); // set below after sort

                if (FM_IS_SET(tetMesh.flags, FM_OBJECT_FLAG_SIMULATION_DISABLED))
                {
                    continue;
                }

                // Add id to list of ids for this step.
                // Update map from id to tet mesh to include any new tet meshes.
                if (FM_IS_SET(tetMesh.flags, FM_OBJECT_FLAG_SLEEPING))
                {
                    scene->awakeTetMeshIds[maxTetMeshes - 1 - numSleepingTetMeshes] = tetMesh.objectId;
                    numSleepingTetMeshes++;
                }
                else
                {
                    scene->awakeTetMeshIds[numAwakeTetMeshes] = tetMesh.objectId;
                    numAwakeTetMeshes++;
                }
                FmSetTetMeshPtrById(scene, tetMesh.objectId, &tetMesh);
            }
        }

        scene->numAwakeTetMeshes = numAwakeTetMeshes;
        scene->numSleepingTetMeshes = numSleepingTetMeshes;
        scene->sleepingTetMeshIds = scene->awakeTetMeshIds + maxTetMeshes - numSleepingTetMeshes;

        // Sort by descreasing number of vertices
        FmSort<uint, FmCompareSolverSize>(scene->awakeTetMeshIds, numAwakeTetMeshes, scene);
        FmSort<uint, FmCompareSolverSize>(scene->sleepingTetMeshIds, numSleepingTetMeshes, scene);

        // Set index in tet meshes that will locate the corresponding id in awakeTetMeshIds or sleepingTetMeshIds arrays
        for (uint meshIdx = 0; meshIdx < numAwakeTetMeshes; meshIdx++)
        {
            uint meshId = scene->awakeTetMeshIds[meshIdx];
            FmSetTetMeshIdxById(scene, meshId, meshIdx);
        }
        for (uint meshIdx = 0; meshIdx < numSleepingTetMeshes; meshIdx++)
        {
            uint idx = maxTetMeshes - 1 - meshIdx;
            uint meshId = scene->awakeTetMeshIds[idx];
            FmSetTetMeshIdxById(scene, meshId, idx);
        }
    }

    bool FmValidateMesh(const FmTetMesh& tetMesh)
    {
        bool validated = true;

        uint numVerts = tetMesh.numVerts;
        uint numTets = tetMesh.numTets;
        uint numExteriorFaces = tetMesh.numExteriorFaces;

        // Check counts <= max
        if (numVerts > tetMesh.maxVerts)
        {
            FM_PRINT(("ERROR in ValidateTetMesh: num verts greater than max: %u max: %u\n", numVerts, tetMesh.maxVerts));
            validated = false;
        }
        if (numExteriorFaces > tetMesh.maxExteriorFaces)
        {
            FM_PRINT(("ERROR in ValidateTetMesh: num exterior faces greater than max: %u max: %u\n", numExteriorFaces, tetMesh.maxExteriorFaces));
            validated = false;
        }

        struct TetGroupAssignment
        {
            uint                       tetId;
            int                        groupId;
            float                      tetMass;
            FmTetVertIds            tetVerts;
            FmTetFaceIncidentTetIds tetFaceIncidentTets;
        };

        TetGroupAssignment tetGroupAssignments[FM_MAX_VERT_INCIDENT_TETS];

        uint numTetsCounted = 0;
        bool* tetsVisited = new bool[tetMesh.numTets];
        for (uint i = 0; i < tetMesh.numTets; i++)
        {
            tetsVisited[i] = false;
        }

        uint totalNumAdjacentVerts = 0;

        for (uint vertId = 0; vertId < numVerts; vertId++)
        {
            // Gather data for tets incident to vertex
            const FmVertNeighbors& vertNeighbors = tetMesh.vertsNeighbors[vertId];
            uint* vertIncidentTets = &tetMesh.vertConnectivity.incidentTets[vertNeighbors.incidentTetsStart];
            uint numIncidentTets = vertNeighbors.numIncidentTets;

            totalNumAdjacentVerts += vertNeighbors.numAdjacentVerts;

            if (vertNeighbors.numIncidentTets > FM_MAX_VERT_INCIDENT_TETS)
            {
                FM_PRINT(("ERROR in ValidateTetMesh: num incident tets %u > max %u\n", vertNeighbors.numIncidentTets, FM_MAX_VERT_INCIDENT_TETS));
                validated = false;
            }

            if (vertNeighbors.incidentTetsStart + vertNeighbors.numIncidentTets > tetMesh.vertConnectivity.numIncidentTetsTotal)
            {
                FM_PRINT(("ERROR in ValidateTetMesh: incident tets start + num incident tets > total incident tets\n"));
                validated = false;
            }

            for (uint itId = 0; itId < numIncidentTets; itId++)
            {
                uint incidentTetId = vertIncidentTets[itId];
                FmTetVertIds tetVerts = tetMesh.tetsVertIds[incidentTetId];

                if (!tetsVisited[incidentTetId])
                {
                    numTetsCounted++;
                    tetsVisited[incidentTetId] = true;
                }

                if (incidentTetId >= tetMesh.numTets)
                {
                    FM_PRINT(("ERROR in ValidateTetMesh: vert incident tet id out of range: %u num: %u\n", incidentTetId, tetMesh.numTets));
                    validated = false;
                }

                if (!FmIsTetIncident(vertId, tetVerts))
                {
                    FM_PRINT(("ERROR in ValidateTetMesh: tet incident on vertex does not include vertex\n"));
                    validated = false;
                }
            }

            for (uint itId = 0; itId < numIncidentTets; itId++)
            {
                uint incidentTetId = vertIncidentTets[itId];

                TetGroupAssignment& tet = tetGroupAssignments[itId];
                tet.tetId = incidentTetId;
                tet.groupId = -1;
                tet.tetMass = tetMesh.tetsMass[incidentTetId];
                tet.tetVerts = tetMesh.tetsVertIds[incidentTetId];
                tet.tetFaceIncidentTets = tetMesh.tetsFaceIncidentTetIds[incidentTetId];
            }

            // Loop over pairs of tets, assign or merge different groups
            int numGroups = 0;
            for (uint itAId = 0; itAId < numIncidentTets; itAId++)
            {
                TetGroupAssignment& tetA = tetGroupAssignments[itAId];

                if (tetA.groupId == -1)
                {
                    tetA.groupId = numGroups;
                    numGroups++;
                }

                for (uint itBId = itAId + 1; itBId < numIncidentTets; itBId++)
                {
                    TetGroupAssignment& tetB = tetGroupAssignments[itBId];

                    uint faceA, faceB;
                    if (FmHasMatchingFace(&faceA, &faceB, tetA.tetId, tetB.tetId, tetA.tetFaceIncidentTets, tetB.tetFaceIncidentTets))
                    {
                        if (tetB.groupId == -1)
                        {
                            tetB.groupId = tetA.groupId;
                        }
                        else if (tetB.groupId != tetA.groupId)
                        {
                            // merging groups
                            numGroups--;

                            // replace group ids to merge the groups, and compact others
                            int tetAGroupId = tetA.groupId;
                            int tetBGroupId = tetB.groupId;
                            int minGroupId = FmMinUint(tetAGroupId, tetBGroupId);
                            int maxGroupId = FmMaxUint(tetAGroupId, tetBGroupId);
                            for (uint itId = 0; itId < numIncidentTets; itId++)
                            {
                                TetGroupAssignment& tet = tetGroupAssignments[itId];

                                // set any tets in a or b group to min id
                                tet.groupId = (tet.groupId == tetAGroupId || tet.groupId == tetBGroupId) ? minGroupId : tet.groupId;

                                // decrement groups > max id, to compact group ids (all ids < numGroups)
                                tet.groupId = (tet.groupId > maxGroupId) ? tet.groupId - 1 : tet.groupId;

                                FM_ASSERT(tet.groupId < numGroups);
                            }
                        }
                    }
                }
            }

            if (numGroups > 1)
            {
                FM_PRINT(("ERROR in ValidateTetMesh: vertex has multiple face-connected-components\n"));
                validated = false;
            }
        }

        if (!FM_IS_SET(tetMesh.flags, FM_OBJECT_FLAG_NEEDS_ADJACENT_VERT_OFFSETS))
        {
            if (totalNumAdjacentVerts != tetMesh.vertConnectivity.numAdjacentVerts)
            {
                FM_PRINT(("ERROR in ValidateTetMesh: sum of adjacent verts not equal to total\n"));
                validated = false;
            }
        }

        if (numTetsCounted != tetMesh.numTets)
        {
            FM_PRINT(("ERROR in ValidateTetMesh: num tets reached by incident vertices not equal to mesh num verts\n"));
            validated = false;
        }

        delete[] tetsVisited;

        for (uint tetId = 0; tetId < numTets; tetId++)
        {
            FmTetVertIds tetVerts = tetMesh.tetsVertIds[tetId];
            FmTetFaceIncidentTetIds tetIncidentTets = tetMesh.tetsFaceIncidentTetIds[tetId];

            for (uint cornerIdx = 0; cornerIdx < 4; cornerIdx++)
            {
                uint vId = tetVerts.ids[cornerIdx];

                if (vId >= tetMesh.numVerts)
                {
                    FM_PRINT(("ERROR in ValidateTetMesh: tet vert id out of range: %u num: %u\n", vId, tetMesh.numVerts));
                    validated = false;
                }
            }

            for (uint faceId = 0; faceId < 4; faceId++)
            {
                uint id = tetIncidentTets.ids[faceId];
                if (FmIsExteriorFaceId(id))
                {
                    uint extFaceId = FmGetExteriorFaceId(id);

                    // Check in range
                    if (extFaceId >= tetMesh.numExteriorFaces)
                    {
                        FM_PRINT(("ERROR in ValidateTetMesh: tet's exterior face id out of range: %u num: %u\n", extFaceId, tetMesh.numExteriorFaces));
                        validated = false;
                    }

                    const FmExteriorFace& exteriorFace = tetMesh.exteriorFaces[extFaceId];

                    if (exteriorFace.tetId != tetId)
                    {
                        FM_PRINT(("ERROR in ValidateTetMesh: exterior face tet id doesn't match\n"));
                        validated = false;
                    }
                }
                else
                {
                    if (id >= tetMesh.numTets)
                    {
                        FM_PRINT(("ERROR in ValidateTetMesh: tet face incident tet id out of range: %u %u\n", id, tetMesh.numTets));
                        validated = false;
                    }

                    FmTetVertIds incidentTetVerts = tetMesh.tetsVertIds[id];

                    uint faceA, faceB;
                    if (!FmHasMatchingFace(&faceA, &faceB, tetVerts, incidentTetVerts))
                    {
                        FM_PRINT(("ERROR in ValidateTetMesh: incident tet doesn't include tet\n"));
                        validated = false;
                    }
                }
            }
        }

        uint* incidentTetsArray = tetMesh.vertConnectivity.incidentTets;

        for (uint exteriorFaceId = 0; exteriorFaceId < numExteriorFaces; exteriorFaceId++)
        {
            const FmExteriorFace& exteriorFace = tetMesh.exteriorFaces[exteriorFaceId];

            uint extFaceId = exteriorFace.GetId();

            // Check ids in range
            if (extFaceId >= tetMesh.numExteriorFaces)
            {
                FM_PRINT(("ERROR in ValidateTetMesh: exterior face id out of range: %u num: %u\n", extFaceId, tetMesh.numExteriorFaces));
                validated = false;
            }

            if (exteriorFace.faceId >= 4)
            {
                FM_PRINT(("ERROR in ValidateTetMesh: exterior face tet face index out of range: %u\n", exteriorFace.faceId));
                validated = false;
            }

            if (exteriorFace.tetId >= tetMesh.numTets)
            {
                FM_PRINT(("ERROR in ValidateTetMesh: exterior face tet id out of range: %u num: %u\n", exteriorFace.tetId, tetMesh.numTets));
                validated = false;
            }

            uint faceId = exteriorFace.faceId;
            uint tetId = exteriorFace.tetId;
            FmTetVertIds tetVerts = tetMesh.tetsVertIds[tetId];

            FmFaceVertIds faceVerts;
            FmGetFaceVertIds(&faceVerts, faceId, tetVerts);

            // Check for duplicate ownership
            bool duplicateEdgeAssignment[3] = { false, false, false };
            for (uint otherFaceIndex = exteriorFaceId + 1; otherFaceIndex < numExteriorFaces; otherFaceIndex++)
            {
                const FmExteriorFace& otherExteriorFace = tetMesh.exteriorFaces[otherFaceIndex];

                uint otherFaceId = otherExteriorFace.faceId;
                uint otherTetId = otherExteriorFace.tetId;
                FmTetVertIds otherTetVerts = tetMesh.tetsVertIds[otherTetId];

                FmFaceVertIds otherFaceVerts;
                FmGetFaceVertIds(&otherFaceVerts, otherFaceId, otherTetVerts);

                for (uint faceCorner = 0; faceCorner < 3; faceCorner++)
                {
                    int vertInTri = -1;
                    vertInTri = otherFaceVerts.ids[0] == faceVerts.ids[faceCorner] ? 0 : vertInTri;
                    vertInTri = otherFaceVerts.ids[1] == faceVerts.ids[faceCorner] ? 1 : vertInTri;
                    vertInTri = otherFaceVerts.ids[2] == faceVerts.ids[faceCorner] ? 2 : vertInTri;

                    if (vertInTri >= 0 && exteriorFace.OwnsVert(faceCorner) && otherExteriorFace.OwnsVert(vertInTri))
                    {
                        FM_PRINT(("ERROR in ValidateTetMesh: two exterior faces own vertex\n"));
                        validated = false;
                    }
                }

                uint edgeA, edgeB;
                if (FmHasMatchingEdge(edgeA, edgeB, faceVerts, otherFaceVerts))
                {
                    if (exteriorFace.OwnsEdge(edgeA) && otherExteriorFace.OwnsEdge(edgeB))
                    {
                        duplicateEdgeAssignment[edgeA] = true;
                    }
                }
            }

            // Check that all features owned by this or incident exterior face.
            bool vertIsAssigned[3];
            vertIsAssigned[0] = exteriorFace.OwnsVert(0);
            vertIsAssigned[1] = exteriorFace.OwnsVert(1);
            vertIsAssigned[2] = exteriorFace.OwnsVert(2);

            bool edgeIsAssigned[3];
            edgeIsAssigned[0] = exteriorFace.OwnsEdge(0);
            edgeIsAssigned[1] = exteriorFace.OwnsEdge(1);
            edgeIsAssigned[2] = exteriorFace.OwnsEdge(2);

            uint foundIncidentExteriorFace[3];
            foundIncidentExteriorFace[0] = FM_INVALID_ID;
            foundIncidentExteriorFace[1] = FM_INVALID_ID;
            foundIncidentExteriorFace[2] = FM_INVALID_ID;
            bool multipleEdgeIncidentSurfaces[3] = { false, false, false };

            for (uint faceCornerId = 0; faceCornerId < 3; faceCornerId++)
            {
                uint vId = faceVerts.ids[faceCornerId];

                const FmVertNeighbors& vertNeighbors = tetMesh.vertsNeighbors[vId];
                uint* vertIncidentTets = &incidentTetsArray[vertNeighbors.incidentTetsStart];
                uint numIncidentTets = vertNeighbors.numIncidentTets;

                for (uint itId = 0; itId < numIncidentTets; itId++)
                {
                    uint incidentTetId = vertIncidentTets[itId];
                    FmTetVertIds incidentTetVerts = tetMesh.tetsVertIds[incidentTetId];
                    FmTetFaceIncidentTetIds faceIncidentTets = tetMesh.tetsFaceIncidentTetIds[incidentTetId];

                    // Check exterior faces of incident tet
                    for (uint tetFaceId = 0; tetFaceId < 4; tetFaceId++)
                    {
                        if (FmIsExteriorFaceId(faceIncidentTets.ids[tetFaceId]))
                        {
                            uint incidentFaceId = FmGetExteriorFaceId(faceIncidentTets.ids[tetFaceId]);
                            FM_ASSERT(incidentFaceId < tetMesh.numExteriorFaces);

                            if (exteriorFaceId == incidentFaceId)
                                continue;

                            const FmExteriorFace& incidentFace = tetMesh.exteriorFaces[incidentFaceId];
                            FmFaceVertIds incidentFaceVerts;
                            FmGetFaceVertIds(&incidentFaceVerts, tetFaceId, incidentTetVerts);

                            int vertInTri = -1;
                            vertInTri = incidentFaceVerts.ids[0] == vId ? 0 : vertInTri;
                            vertInTri = incidentFaceVerts.ids[1] == vId ? 1 : vertInTri;
                            vertInTri = incidentFaceVerts.ids[2] == vId ? 2 : vertInTri;

                            if (vertInTri >= 0 && incidentFace.OwnsVert(vertInTri))
                            {
                                vertIsAssigned[faceCornerId] = true;
                            }

                            uint edgeA, edgeB;
                            if (FmHasMatchingEdge(edgeA, edgeB, faceVerts, incidentFaceVerts))
                            {
                                if (foundIncidentExteriorFace[edgeA] != FM_INVALID_ID &&
                                    foundIncidentExteriorFace[edgeA] != incidentFaceId)
                                {
                                    multipleEdgeIncidentSurfaces[edgeA] = true;
                                }

                                foundIncidentExteriorFace[edgeA] = incidentFaceId;

                                if (incidentFace.OwnsEdge(edgeB))
                                {
                                    edgeIsAssigned[edgeA] = true;
                                }
                            }
                        }
                    }
                }
            }

            if (!vertIsAssigned[0] || !vertIsAssigned[1] || !vertIsAssigned[2])
            {
                FM_PRINT(("ERROR in ValidateTetMesh: vert of exterior face not assigned\n"));
                validated = false;
            }

            if (!edgeIsAssigned[0] || !edgeIsAssigned[1] || !edgeIsAssigned[2])
            {
                FM_PRINT(("ERROR in ValidateTetMesh: edge of exterior face not assigned\n"));
                validated = false;
            }

            if ((duplicateEdgeAssignment[0] && !multipleEdgeIncidentSurfaces[0]) ||
                (duplicateEdgeAssignment[1] && !multipleEdgeIncidentSurfaces[1]) ||
                (duplicateEdgeAssignment[2] && !multipleEdgeIncidentSurfaces[2]))
            {
                FM_PRINT(("ERROR in ValidateTetMesh: edge assignment duplicated without multiple surfaces incident on edge\n"));
                validated = false;
            }
        }

        FM_ASSERT(validated);
        return validated;
    }

#if FM_DEBUG_CHECKS
    bool FmValidateMeshBuffer(const FmTetMeshBuffer& tetMeshBuffer)
    {
        bool validated = true;

        for (uint vertId = 0; vertId < tetMeshBuffer.numVerts; vertId++)
        {
            FmVertReference& vertRef = tetMeshBuffer.vertReferences[vertId];

            FmTetMesh& tetMesh = tetMeshBuffer.tetMeshes[vertRef.meshIdx];

            if (tetMesh.vertsIndex0[vertRef.vertId] != vertId)
            {
                FM_PRINT(("ERROR in ValidateMeshBuffer: vert referenced does not point back to same vertex\n"));
                validated = false;
            }
        }
        for (uint tetId = 0; tetId < tetMeshBuffer.numTets; tetId++)
        {
            FmTetReference& tetRef = tetMeshBuffer.tetReferences[tetId];

            FmTetMesh& tetMesh = tetMeshBuffer.tetMeshes[tetRef.meshIdx];

            if (tetMesh.tetsIndex0[tetRef.tetId] != tetId)
            {
                FM_PRINT(("ERROR in ValidateMeshBuffer: tet referenced does not point back to same vertex\n"));
                validated = false;
            }
        }
        for (uint meshIdx = 0; meshIdx < tetMeshBuffer.numTetMeshes; meshIdx++)
        {
            FmTetMesh& tetMesh = tetMeshBuffer.tetMeshes[meshIdx];

            if (!FmValidateMesh(tetMesh))
            {
                validated = false;
            }

            // Check map from TetMeshBuffer to TetMesh verts and tets
            uint tetMeshBufferId = tetMesh.bufferId;
            if (tetMeshBufferId != tetMeshBuffer.bufferId)
            {
                FM_PRINT(("ERROR in ValidateTetMesh: bufferId not valid\n"));
                validated = false;
            }
            else
            {
                uint numVerts = tetMesh.numVerts;
                for (uint vertId = 0; vertId < numVerts; vertId++)
                {
                    uint index0 = tetMesh.vertsIndex0[vertId];
                    if (FM_NOT_SET(tetMesh.vertsFlags[vertId], FM_VERT_FLAG_FRACTURE_COPY))
                    {
                        if (vertId != tetMeshBuffer.vertReferences[index0].vertId)
                        {
                            FM_PRINT(("ERROR in ValidateTetMesh: inconsistent map between TetMeshBuffer and TetMesh vert\n"));
                            validated = false;
                        }
                    }
                }
                uint numTets = tetMesh.numTets;
                for (uint tetId = 0; tetId < numTets; tetId++)
                {
                    uint index0 = tetMesh.tetsIndex0[tetId];
                    if (tetId != tetMeshBuffer.tetReferences[index0].tetId)
                    {
                        FM_PRINT(("ERROR in ValidateTestMesh: inconsistent map between TetMeshBuffer and TetMesh tet\n"));
                        validated = false;
                    }
                }
            }

            if (tetMesh.bufferId == FM_INVALID_ID || tetMesh.bufferId != tetMeshBuffer.bufferId)
            {
                FM_PRINT(("ERROR in ValidateMeshBuffer: invalid tet mesh bufferId\n"));
                validated = false;
            }
        }

        FM_ASSERT(validated);
        return validated;
    }

#define FM_SCENE_UPDATE_PHASE_START                      0
#define FM_SCENE_UPDATE_PHASE_POST_CONNECTED_COMPONENTS  1
#define FM_SCENE_UPDATE_PHASE_POST_COLLISION             2
#define FM_SCENE_UPDATE_PHASE_POST_ISLANDS               3

    bool FmValidateScene(const FmScene& scene, uint phase)
    {
        bool validated = true;

        const FmConstraintsBuffer& constraintsBuffer = *scene.constraintsBuffer;

        if (phase >= FM_SCENE_UPDATE_PHASE_POST_CONNECTED_COMPONENTS)
        {
            // Count simulated tet meshes
            uint numSimulatedTetMeshes = 0;
            for (uint i = 0; i < scene.numTetMeshBufferSlots; i++)
            {
                if (!scene.tetMeshBuffers[i])
                {
                    continue;
                }

                const FmTetMeshBuffer& tetMeshBuffer = *scene.tetMeshBuffers[i];

                for (uint mId = 0; mId < tetMeshBuffer.numTetMeshes; mId++)
                {
                    const FmTetMesh& tetMesh = tetMeshBuffer.tetMeshes[mId];

                    if (!FM_IS_SET(tetMesh.flags, FM_OBJECT_FLAG_SIMULATION_DISABLED))
                    {
                        numSimulatedTetMeshes++;

                        uint tetMeshIdx = FmGetTetMeshIdxById(scene, tetMesh.objectId);
                        if (tetMeshIdx == FM_INVALID_ID || tetMeshIdx >= scene.maxTetMeshes)
                        {
                            FM_PRINT(("ERROR in ValidateMeshBuffer: invalid tet mesh idx\n"));
                            validated = false;
                        }
                        else
                        {
                            uint meshId = scene.awakeTetMeshIds[tetMeshIdx];

                            if (tetMesh.objectId != meshId)
                            {
                                FM_PRINT(("ERROR in ValidateMeshBuffer: objectId (%u) does not match awakeTetMeshIds[%u] (%u)\n", tetMesh.objectId, tetMeshIdx, meshId));
                                validated = false;
                            }
                        }
                    }
                }
            }

            // Check active and sleeping id arrays
            if (scene.numAwakeTetMeshes + scene.numSleepingTetMeshes != numSimulatedTetMeshes)
            {
                FM_PRINT(("ERROR in ValidateScene: sum of active and sleeping meshes (%u) != total (%u)\n", scene.numAwakeTetMeshes + scene.numSleepingTetMeshes, numSimulatedTetMeshes));
                validated = false;
            }
        }

        if (scene.sleepingTetMeshIds + scene.numSleepingTetMeshes > scene.awakeTetMeshIds + scene.maxTetMeshes
            || scene.awakeTetMeshIds + scene.numAwakeTetMeshes > scene.sleepingTetMeshIds
            || scene.sleepingTetMeshIds != scene.awakeTetMeshIds + scene.maxTetMeshes - scene.numSleepingTetMeshes)
        {
            FM_PRINT(("ERROR in ValidateScene: invalid tetMeshIds array\n"));
            validated = false;
        }

        // Validate tet mesh ids
        for (uint meshIdx = 0; meshIdx < scene.numAwakeTetMeshes; meshIdx++)
        {
            uint meshId = scene.awakeTetMeshIds[meshIdx];

            if (meshId == FM_INVALID_ID || meshId >= scene.maxTetMeshes)
            {
                FM_PRINT(("ERROR in ValidateScene: mesh id (%u) in awakeTetMeshIds is invalid; max: %u\n", meshId, scene.maxTetMeshes));
                validated = false;
                continue;
            }

            if (scene.tetMeshPtrFromId[meshId] == NULL)
            {
                FM_PRINT(("ERROR in ValidateScene: tetMeshPtrFromId of mesh in awakeTetMeshIds is NULL\n"));
                validated = false;
                continue;
            }

            const FmTetMesh& tetMesh = *FmGetTetMeshPtrById(scene, meshId);
            uint tetMeshIdx = FmGetTetMeshIdxById(scene, meshId);

            if (FM_IS_SET(tetMesh.flags, FM_OBJECT_FLAG_SLEEPING))
            {
                FM_PRINT(("ERROR in ValidateScene: awake tet mesh objectId (%u) has sleeping flag\n", tetMesh.objectId));
                validated = false;
            }

            // Check object id
            if (tetMesh.objectId != meshId)
            {
                FM_PRINT(("ERROR in ValidateScene: tet mesh objectId (%u) does not match id of lookup (%u)\n", tetMesh.objectId, meshId));
                validated = false;
            }

            // Check idx
            if (tetMeshIdx != meshIdx)
            {
                FM_PRINT(("ERROR in ValidateScene: tet mesh idx does not match location of id in tetMeshIds\n"));
                validated = false;
            }

            // Check that id is not also in sleeping list
            for (uint sleepingMeshIdx = 0; sleepingMeshIdx < scene.numSleepingTetMeshes; sleepingMeshIdx++)
            {
                uint sleepingMeshId = scene.sleepingTetMeshIds[sleepingMeshIdx];

                if (meshId == sleepingMeshId)
                {
                    FM_PRINT(("ERROR in ValidateScene: tet mesh id present in active and sleeping arrays\n"));
                    validated = false;
                }
            }
        }
        for (uint sleepingMeshIdx = 0; sleepingMeshIdx < scene.numSleepingTetMeshes; sleepingMeshIdx++)
        {
            uint meshIdx = scene.maxTetMeshes - scene.numSleepingTetMeshes + sleepingMeshIdx;
            uint meshId = scene.sleepingTetMeshIds[sleepingMeshIdx];

            if (meshId == FM_INVALID_ID || meshId >= scene.maxTetMeshes)
            {
                FM_PRINT(("ERROR in ValidateScene: mesh id (%u) in sleepingTetMeshIds is invalid, max: %u\n", meshId, scene.maxTetMeshes));
                validated = false;
                continue;
            }

            const FmTetMesh& tetMesh = *FmGetTetMeshPtrById(scene, meshId);
            uint tetMeshIdx = FmGetTetMeshIdxById(scene, meshId);

            if (FM_NOT_SET(tetMesh.flags, FM_OBJECT_FLAG_SLEEPING))
            {
                FM_PRINT(("ERROR in ValidateScene: asleep tet mesh objectId (%u) does not have sleeping flag\n", tetMesh.objectId));
                validated = false;
            }

            if (tetMeshIdx != meshIdx)
            {
                FM_PRINT(("ERROR in ValidateScene: tet mesh idx does not match location of id in awakeTetMeshIds\n"));
                validated = false;
            }
        }

        // Check active and sleeping id arrays
        if (scene.sleepingRigidBodyIds + scene.numSleepingRigidBodies > scene.awakeRigidBodyIds + scene.maxRigidBodies
            || scene.awakeRigidBodyIds + scene.numAwakeRigidBodies > scene.sleepingRigidBodyIds
            || scene.sleepingRigidBodyIds != scene.awakeRigidBodyIds + scene.maxRigidBodies - scene.numSleepingRigidBodies)
        {
            FM_PRINT(("ERROR in ValidateScene: invalid rigidBodyIds array\n"));
            validated = false;
        }

        // Validate the rigid body idx 
        for (uint rbIdx = 0; rbIdx < scene.numAwakeRigidBodies; rbIdx++)
        {
            uint rbId = scene.awakeRigidBodyIds[rbIdx];

            if (rbId == FM_INVALID_ID || (rbId & ~FM_RB_FLAG) >= scene.maxRigidBodies)
            {
                FM_PRINT(("ERROR in ValidateScene: id (%u) in rigidBodyIds is invalid, max: %u\n", rbId, scene.maxRigidBodies));
                validated = false;
                continue;
            }

            FmRigidBody* pRigidBody = FmGetRigidBodyPtrById(scene, rbId);

            if (pRigidBody == NULL)
            {
                FM_PRINT(("ERROR in ValidateScene: rigid body in rigidBodyIds is deleted\n"));
                validated = false;
                continue;
            }

            const FmRigidBody& rigidBody = *pRigidBody;
            uint rigidBodyIdx = FmGetRigidBodyIdxById(scene, rigidBody.objectId);

            if (rigidBodyIdx != rbIdx)
            {
                FM_PRINT(("ERROR in ValidateScene: rigid body idx does not match location of id in rigidBodyIds\n"));
                validated = false;
            }

            // Check that id is not also in sleeping list
            for (uint sleepingRbIdx = 0; sleepingRbIdx < scene.numSleepingRigidBodies; sleepingRbIdx++)
            {
                uint sleepingRbId = scene.sleepingRigidBodyIds[sleepingRbIdx];

                if (rbId == sleepingRbId)
                {
                    FM_PRINT(("ERROR in ValidateScene: rigid body id present in active and sleeping arrays\n"));
                    validated = false;
                }
            }
        }
        for (uint sleepingRbIdx = 0; sleepingRbIdx < scene.numSleepingRigidBodies; sleepingRbIdx++)
        {
            uint rbIdx = scene.maxRigidBodies - scene.numSleepingRigidBodies + sleepingRbIdx;
            uint rbId = scene.sleepingRigidBodyIds[sleepingRbIdx];

            const FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(scene, rbId);
            uint rigidBodyIdx = FmGetRigidBodyIdxById(scene, rigidBody.objectId);

            if (rigidBodyIdx != rbIdx)
            {
                FM_PRINT(("ERROR in ValidateScene: rigid body idx does not match location of id in rigidBodyIds\n"));
                validated = false;
            }
        }

        // Check islands for valid objects
        for (uint islandIdx = 0; islandIdx < constraintsBuffer.numConstraintIslands; islandIdx++)
        {
            FmConstraintIsland& constraintIsland = constraintsBuffer.constraintIslands[islandIdx];

            for (uint islandMeshIdx = 0; islandMeshIdx < constraintIsland.numTetMeshes; islandMeshIdx++)
            {
                uint meshId = constraintIsland.tetMeshIds[islandMeshIdx];

                if (meshId == FM_INVALID_ID || meshId >= scene.maxTetMeshes)
                {
                    FM_PRINT(("ERROR in ValidateScene: id (%u) in island tetMeshIds is invalid; max: %u\n", meshId, scene.maxTetMeshes));
                    validated = false;
                    continue;
                }

                if (scene.tetMeshPtrFromId[meshId] == NULL)
                {
                    FM_PRINT(("ERROR in ValidateScene: tetMeshPtrFromId of mesh in island tetMeshIds is NULL\n"));
                    validated = false;
                    continue;
                }
            }

            for (uint islandRbIdx = 0; islandRbIdx < constraintIsland.numRigidBodiesConnected; islandRbIdx++)
            {
                uint rbId = constraintIsland.rigidBodyIds[islandRbIdx];

                if (rbId == FM_INVALID_ID || (rbId & ~FM_RB_FLAG) >= scene.maxRigidBodies)
                {
                    FM_PRINT(("ERROR in ValidateScene: id (%u) in island rigidBodyIds is invalid; max: %u\n", rbId, scene.maxRigidBodies));
                    validated = false;
                    continue;
                }

                FmRigidBody* pRigidBody = FmGetRigidBodyPtrById(scene, rbId);

                if (pRigidBody == NULL)
                {
                    FM_PRINT(("ERROR in ValidateScene: rigid body in island rigidBodyIds is deleted\n"));
                    validated = false;
                    continue;
                }
            }
        }

        // Check sleeping islands for valid objects
        for (uint islandIdx = 0; islandIdx < constraintsBuffer.numSleepingConstraintIslands; islandIdx++)
        {
            FmSleepingConstraintIsland& constraintIsland = constraintsBuffer.sleepingConstraintIslands[islandIdx];

            if (phase > FM_SCENE_UPDATE_PHASE_START)
            {
                if (constraintIsland.numTetMeshes == 0 && constraintIsland.numRigidBodiesConnected == 0)
                {
                    FM_PRINT(("ERROR in ValidateScene: sleeping island has no objects\n"));
                    validated = false;
                }
            }

            for (uint islandMeshIdx = 0; islandMeshIdx < constraintIsland.numTetMeshes; islandMeshIdx++)
            {
                uint meshId = constraintIsland.tetMeshIds[islandMeshIdx];

                if (meshId == FM_INVALID_ID || meshId >= scene.maxTetMeshes)
                {
                    FM_PRINT(("ERROR in ValidateScene: id (%u) in sleeping island tetMeshIds is invalid; max: %u\n", meshId, scene.maxTetMeshes));
                    validated = false;
                    continue;
                }

                if (scene.tetMeshPtrFromId[meshId] == NULL)
                {
                    FM_PRINT(("ERROR in ValidateScene: tetMeshPtrFromId of mesh in sleeping island tetMeshIds is NULL\n"));
                    validated = false;
                    continue;
                }
            }

            for (uint islandRbIdx = 0; islandRbIdx < constraintIsland.numRigidBodiesConnected; islandRbIdx++)
            {
                uint rbId = constraintIsland.rigidBodyIds[islandRbIdx];

                if (rbId == FM_INVALID_ID || (rbId & ~FM_RB_FLAG) >= scene.maxRigidBodies)
                {
                    FM_PRINT(("ERROR in ValidateScene: id (%u) in sleeping island rigidBodyIds is invalid; max: %u\n", rbId, scene.maxRigidBodies));
                    validated = false;
                    continue;
                }

                FmRigidBody* pRigidBody = FmGetRigidBodyPtrById(scene, rbId);

                if (pRigidBody == NULL)
                {
                    FM_PRINT(("ERROR in ValidateScene: rigid body in sleeping island rigidBodyIds is deleted\n"));
                    validated = false;
                    continue;
                }
            }
        }

        // Check all objects
        for (uint meshBufferIdx = 0; meshBufferIdx < scene.numTetMeshBufferSlots; meshBufferIdx++)
        {
            if (scene.tetMeshBuffers[meshBufferIdx] == NULL)
            {
                continue;
            }

            FmTetMeshBuffer& tetMeshBuffer = *scene.tetMeshBuffers[meshBufferIdx];

            if (!FmValidateMeshBuffer(tetMeshBuffer))
            {
                validated = false;
            }

            for (uint meshIdx = 0; meshIdx < tetMeshBuffer.numTetMeshes; meshIdx++)
            {
                FmTetMesh& tetMesh = tetMeshBuffer.tetMeshes[meshIdx];
                uint tetMeshIdx = FmGetTetMeshIdxById(scene, tetMesh.objectId);

                if (FM_IS_SET(tetMesh.flags, FM_OBJECT_FLAG_SIMULATION_DISABLED))
                {
                    if (tetMeshIdx != FM_INVALID_ID || tetMesh.islandId != FM_INVALID_ID)
                    {
                        FM_PRINT(("ERROR in ValidateScene: sim disabled tet mesh has idx or islandId\n"));
                        validated = false;
                    }
                }
                else if (FM_IS_SET(tetMesh.flags, FM_OBJECT_FLAG_SLEEPING))
                {
                    // Check sleeping object in sleeping island
                    FmSleepingConstraintIsland& island = *FmGetSleepingConstraintIslandById(scene, tetMesh.islandId);
                    bool found = false;
                    for (uint i = 0; i < island.numTetMeshes; i++)
                    {
                        if (island.tetMeshIds[i] == tetMesh.objectId)
                        {
                            found = true;
                            break;
                        }
                    }
                    if (!found)
                    {
                        FM_PRINT(("ERROR in ValidateScene: tet mesh not found in sleeping island\n"));
                        validated = false;
                    }
                }
                else
                {
                    // For active objects after islands found, check membership in island
                    if (phase >= FM_SCENE_UPDATE_PHASE_POST_ISLANDS)
                    {
                        FmConstraintIsland& island = constraintsBuffer.constraintIslands[tetMesh.islandId];
                        bool found = false;
                        for (uint i = 0; i < island.numTetMeshes; i++)
                        {
                            if (island.tetMeshIds[i] == tetMesh.objectId)
                            {
                                found = true;
                                break;
                            }
                        }
                        if (!found)
                        {
                            FM_PRINT(("ERROR in ValidateScene: tet mesh not found in island\n"));
                            validated = false;
                        }
                    }
                }
            }
        }

        for (uint rbIdx = 0; rbIdx < scene.numRigidBodySlots; rbIdx++)
        {
            FmRigidBody* pRigidBody = scene.rigidBodies[rbIdx];

            if (pRigidBody == NULL)
            {
                continue;
            }

            const FmRigidBody& rigidBody = *pRigidBody;
            uint rigidBodyIdx = FmGetRigidBodyIdxById(scene, rigidBody.objectId);

            if (rigidBody.objectId != (rbIdx | FM_RB_FLAG))
            {
                FM_PRINT(("ERROR in ValidateScene: rigid body objectId (%u) not equal to idx (%u)\n", rbIdx, rbIdx | FM_RB_FLAG));
                validated = false;
            }

            if (FM_IS_SET(rigidBody.flags, FM_OBJECT_FLAG_SIMULATION_DISABLED))
            {
                if (rigidBodyIdx != FM_INVALID_ID || rigidBody.femIslandId != FM_INVALID_ID)
                {
                    FM_PRINT(("ERROR in ValidateScene: sim disabled rigid body has idx or femIslandId\n"));
                    validated = false;
                }
            }
            else if (FM_IS_SET(rigidBody.flags, FM_OBJECT_FLAG_SLEEPING))
            {
                FmSleepingConstraintIsland& island = *FmGetSleepingConstraintIslandById(scene, rigidBody.femIslandId);
                bool found = false;
                for (uint i = 0; i < island.numRigidBodiesConnected; i++)
                {
                    if (island.rigidBodyIds[i] == rigidBody.objectId)
                    {
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    FM_PRINT(("ERROR in ValidateScene: rigid body not found in sleeping island\n"));
                    validated = false;
                }
            }
            else
            {
                // For active objects after islands found, check membership in island
                if (phase >= FM_SCENE_UPDATE_PHASE_POST_ISLANDS)
                {
                    if (rigidBody.femIslandId == FM_INVALID_ID)
                    {
                        if (rigidBody.foundInConstraint)
                        {
                            FM_PRINT(("ERROR in ValidateScene: rigid body has no island but is in a constraint\n"));
                            validated = false;
                        }
                    }
                    else
                    {
                        // Check membership in island
                        FmConstraintIsland& island = constraintsBuffer.constraintIslands[rigidBody.femIslandId];
                        bool found = false;
                        for (uint i = 0; i < island.numRigidBodiesConnected; i++)
                        {
                            if (island.rigidBodyIds[i] == rigidBody.objectId)
                            {
                                found = true;
                                break;
                            }
                        }
                        if (!found)
                        {
                            FM_PRINT(("ERROR in ValidateScene: rigid body not found in island\n"));
                            validated = false;
                        }
                    }
                }
            }
        }

        FM_ASSERT(validated);
        return validated;
    }
#endif

    FmMatrix4 FmComputeTetBarycentricMatrix(const FmVector3& tetRestPosition0, const FmVector3& tetRestPosition1, const FmVector3& tetRestPosition2, const FmVector3& tetRestPosition3)
    {
        FmTetShapeParams shapeParams;
        FmComputeShapeParams(&shapeParams, tetRestPosition0, tetRestPosition1, tetRestPosition2, tetRestPosition3);
        return shapeParams.baryMatrix;
    }

    FmMatrix4 FmComputeTetBarycentricMatrix(const FmVector3* vertRestPositions, const FmTetVertIds& tetVerts)
    {
        FmVector3 tetRestPosition0 = vertRestPositions[tetVerts.ids[0]];
        FmVector3 tetRestPosition1 = vertRestPositions[tetVerts.ids[1]];
        FmVector3 tetRestPosition2 = vertRestPositions[tetVerts.ids[2]];
        FmVector3 tetRestPosition3 = vertRestPositions[tetVerts.ids[3]];

        return FmComputeTetBarycentricMatrix(tetRestPosition0, tetRestPosition1, tetRestPosition2, tetRestPosition3);
    }

    FmVector4 FmComputeBarycentricCoords(const FmVector3& tetRestPosition0, const FmVector3& tetRestPosition1, const FmVector3& tetRestPosition2, const FmVector3& tetRestPosition3, const FmVector3& point)
    {
        return mul(FmComputeTetBarycentricMatrix(tetRestPosition0, tetRestPosition1, tetRestPosition2, tetRestPosition3), FmVector4(point, 1.0f));
    }

    FmVector4 FmComputeBarycentricCoords(const FmVector3* vertRestPositions, const FmTetVertIds& tetVerts, const FmVector3& point)
    {
        return mul(FmComputeTetBarycentricMatrix(vertRestPositions, tetVerts), FmVector4(point, 1.0f));
    }

    void FmResetFromRestPositions(FmScene* scene, FmTetMesh* tetMesh, const FmMatrix3& rotation, const FmVector3& translation, const FmVector3& velocity)
    {
        uint numVerts = tetMesh->numVerts;
        for (uint vId = 0; vId < numVerts; vId++)
        {
            tetMesh->vertsPos[vId] = translation + mul(rotation, tetMesh->vertsRestPos[vId]);
            tetMesh->vertsVel[vId] = velocity;
        }

        // For plastic meshes, reset the plastic deformation and tet dynamics matrices
        if (tetMesh->tetsPlasticity)
        {
            uint numTets = tetMesh->numTets;
            for (uint tId = 0; tId < numTets; tId++)
            {
                FmTetMaterialParams tetMaterialParams;
                FmInitTetMaterialParams(&tetMaterialParams, *tetMesh, tId);
                FmUpdateTetMaterialParams(scene, tetMesh, tId, tetMaterialParams, 0.0f);
            }
        }

        FmUpdateBoundsAndCenterOfMass(tetMesh);
        FmUpdateTetStateAndFracture(scene, tetMesh);

        // If part of sleeping island, mark for waking
        FmMarkIslandOfObjectForWaking(scene, tetMesh->objectId);
    }

    void FmFractureMesh(FmScene* scene, FmTetMesh* tetMesh)
    {
        uint numTetsToFracture = FmAtomicRead(&tetMesh->numTetsToFracture.val);
        if (tetMesh->tetsToFracture && (numTetsToFracture > 0 || FM_IS_SET(tetMesh->flags, FM_OBJECT_FLAG_REMOVED_KINEMATIC)))
        {
            if (FmMakeMeshSplits(scene, tetMesh))
            {
                tetMesh->flags |= (FM_OBJECT_FLAG_NEEDS_CONNECTED_COMPONENTS | FM_OBJECT_FLAG_NEEDS_ADJACENT_VERT_OFFSETS);

                FmAssignFeaturesToExteriorFaces(tetMesh, tetMesh->numExteriorFaces, tetMesh->numNewExteriorFaces);
                tetMesh->numExteriorFaces += tetMesh->numNewExteriorFaces;

                // Add object to fracture report
                FmFractureReport& fractureReport = scene->fractureReport;
                if (fractureReport.tetMeshReportsBuffer != NULL)
                {
                    if (FmAtomicRead(&fractureReport.numTetMeshReports.val) < fractureReport.maxTetMeshReports)
                    {
                        uint reportIdx = FmAtomicIncrement(&fractureReport.numTetMeshReports.val) - 1;

                        if (reportIdx < fractureReport.maxTetMeshReports)
                        {
                            FmTetMeshFractureReport tetMeshReport;
                            tetMeshReport.objectId = tetMesh->objectId;
                            tetMeshReport.numFractureFaces = tetMesh->numNewExteriorFaces;
                            tetMeshReport.numTotalFaces = tetMesh->numExteriorFaces;

                            fractureReport.tetMeshReportsBuffer[reportIdx] = tetMeshReport;
                        }
                        else
                        {
                            FmAtomicWrite(&fractureReport.numTetMeshReports.val, fractureReport.maxTetMeshReports);
                        }
                    }
                }
            }

            // Add to warnings report     
            if (FM_IS_SET(tetMesh->flags, FM_OBJECT_FLAG_HIT_MAX_VERTS))
            {
                FmAtomicOr(&scene->warningsReport.flags.val, FM_WARNING_FLAG_HIT_LIMIT_TET_MESH_VERTS);
                FmAtomicWrite(&scene->warningsReport.tetMeshId.val, tetMesh->objectId);
            }
            if (FM_IS_SET(tetMesh->flags, FM_OBJECT_FLAG_HIT_MAX_EXTERIOR_FACES))
            {
                FmAtomicOr(&scene->warningsReport.flags.val, FM_WARNING_FLAG_HIT_LIMIT_TET_MESH_EXTERIOR_FACES);
                FmAtomicWrite(&scene->warningsReport.tetMeshId.val, tetMesh->objectId);
            }
        }
    }

    // Take a simulation step using current dynamic state, external forces, constraints set in tetMeshState.
    // Also resets vertex flag FM_VERT_FLAG_FRACTURED and tetQuatSum
    void FmStepVelocityImplicitEuler(
        FmScene* scene,
        FmTetMesh* tetMesh, 
        FmMpcgSolverData* solverData,
        const FmVector3& gravityVector, 
        float kRayleighMassDamping, float kRayleighStiffnessDamping, float extForceSpeedLimit, float timestep, float epsilonCG)
    {
        uint workerIndex = scene->taskSystemCallbacks.GetTaskSystemWorkerIndex();
        uint8_t* tempBuffer = scene->threadTempMemoryBuffer->buffers[workerIndex];
        uint8_t* tempBufferEnd = tempBuffer + scene->threadTempMemoryBuffer->numBytesPerBuffer;
        FmMpcgSolverDataTemps solverTemps;
        solverTemps.Alloc(&tempBuffer, tempBufferEnd, tetMesh->numVerts);

        // Set up MPCG solve with current state including vertex constraints
        FmSetupMpcgSolve(solverTemps.solution, solverData, tetMesh,
            gravityVector,
            kRayleighMassDamping,
            kRayleighStiffnessDamping,
            extForceSpeedLimit,
            timestep); // Initial est is current velocity

        // Run MPCG to compute velocities at end of step, for linear implicit Euler step.
        FmRunMpcgSolve(solverTemps.solution, solverData, &solverTemps, epsilonCG, tetMesh->maxUnconstrainedSolveIterations);

        // Reset vertOffset
        solverData->solverStateOffset = FM_INVALID_ID;

        // Update to the end-of-step velocities, which give movement of vertices during the
        // step before contacts are considered.  Don't update position until contacts found and 
        // contact corrections to velocities are computed.
        uint numVerts = tetMesh->numVerts;
        for (uint vId = 0; vId < numVerts; vId++)
        {
#if FM_SOLVE_DELTAV
            FmVector3 endOfStepVelocity = tetMesh->vertsVel[vId] + FmInitVector3(solverTemps.solution[vId]);
#else
            FmVector3 endOfStepVelocity = FmInitVector3(solverTemps.solution[vId]);
#endif
            FmVertTetValues& vertTetValues = tetMesh->vertsTetValues[vId];

            tetMesh->vertsVel[vId] = endOfStepVelocity;

            // Reset fractured flag and tetQuatSum
            vertTetValues.tetStrainMagAvg = 0.0f;
            vertTetValues.tetQuatSum = FmInitQuat(0.0f, 0.0f, 0.0f, 0.0f);
            tetMesh->vertsFlags[vId] &= ~FM_VERT_FLAG_FRACTURED;
        }

        tetMesh->flags |= FM_OBJECT_FLAG_VEL_CHANGED;
    }

    void FmResetKinematicObject(FmTetMesh* tetMesh, FmMpcgSolverData* solverData)
    {
        // Reset vertOffset
        solverData->solverStateOffset = FM_INVALID_ID;

        uint numVerts = tetMesh->numVerts;
        for (uint vId = 0; vId < numVerts; vId++)
        {
            FmVertTetValues& vertTetValues = tetMesh->vertsTetValues[vId];
            vertTetValues.tetStrainMagAvg = 0.0f;
            vertTetValues.tetQuatSum = FmInitQuat(0.0f, 0.0f, 0.0f, 0.0f);
        }
    }

    void FmAddDeformationConstraints(FmTetMesh* tetMesh, FmConstraintsBuffer* constraintsBuffer, float timestep);

    class FmTaskDataStepVelocityRebuildBvh : public FmAsyncTaskData
    {
    public:
        FM_CLASS_NEW_DELETE(FmTaskDataStepVelocityRebuildBvh)

        FmScene* scene;

        uint numTetMeshes;
        uint numRigidBodies;

        FmTaskDataStepVelocityRebuildBvh(FmScene* inScene, uint inNumTetMeshes, uint inNumRigidBodies)
        {
            scene = inScene;
            numTetMeshes = inNumTetMeshes;
            numRigidBodies = inNumRigidBodies;
        }
    };

    void FmTaskFuncTetMeshStepVelocityRebuildBvh(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT(MESH_STEP_VELOCITY_REBUILD_BVH);

        FmTaskDataStepVelocityRebuildBvh* taskData = (FmTaskDataStepVelocityRebuildBvh*)inTaskData;
        FmScene* scene = taskData->scene;

        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;
        FmSceneControlParams& sceneParams = scene->params;
        FmVector3 gravityVector = sceneParams.gravityVector;
        float timestep = sceneParams.timestep;
        float kRayleighMassDamping = sceneParams.kRayleighMassDamping;
        float kRayleighStiffnessDamping = sceneParams.kRayleighStiffnessDamping;
        float aabbPadding = sceneParams.distContactThreshold * 0.5f;

        uint beginIdx = (uint)inTaskBeginIndex;
        uint endIdx = (uint)inTaskBeginIndex + 1;
        for (uint i = beginIdx; i < endIdx; i++)
        {
            FmTetMesh* tetMesh = FmGetTetMeshPtrById(*scene, scene->awakeTetMeshIds[i]);
            FmMpcgSolverData* solverData = FmGetSolverDataById(*scene, scene->awakeTetMeshIds[i]);

            // Reset for this step
            tetMesh->numNewExteriorFaces = 0;

            if (FM_IS_SET(tetMesh->flags, FM_OBJECT_FLAG_KINEMATIC))
            {
                if (FM_ANY_SET(tetMesh->flags, FM_OBJECT_FLAG_POS_CHANGED | FM_OBJECT_FLAG_VEL_CHANGED))
                {
                    FmResetKinematicObject(tetMesh, solverData);

                    FmBuildHierarchy(tetMesh, timestep, aabbPadding);
                }
            }
            else
            {
                FmStepVelocityImplicitEuler(scene, tetMesh, solverData, gravityVector, kRayleighMassDamping, kRayleighStiffnessDamping, tetMesh->extForceSpeedLimit, timestep, scene->params.epsilonCg);

                FmBuildHierarchy(tetMesh, timestep, aabbPadding);

                FmAddDeformationConstraints(tetMesh, constraintsBuffer, timestep);
            }
        }

        taskData->progress.TaskIsFinished(taskData);
    }

    void FmTaskFuncRbStepVelocityRebuildBvh(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT(RB_STEP_VELOCITY_REBUILD_BVH);

        FmTaskDataStepVelocityRebuildBvh* taskData = (FmTaskDataStepVelocityRebuildBvh*)inTaskData;
        FmScene* scene = taskData->scene;

        FmSceneControlParams& sceneParams = scene->params;
        float timestep = sceneParams.timestep;
        FmVector3 sceneGravityDeltaVel = sceneParams.gravityVector * timestep;
        float aabbPadding = sceneParams.distContactThreshold * 0.5f;

        uint beginIdx = (uint)inTaskBeginIndex;
        uint endIdx = (uint)inTaskBeginIndex + 1;
        for (uint i = beginIdx; i < endIdx; i++)
        {
            FmRigidBody* pRigidBody = FmGetRigidBodyPtrById(*scene, scene->awakeRigidBodyIds[i]);

            FM_ASSERT(pRigidBody != NULL);

            FmRigidBody& rigidBody = *pRigidBody;

            bool isKinematic = FM_IS_SET(rigidBody.flags, FM_OBJECT_FLAG_KINEMATIC);

            if (!isKinematic)
            {
                // Integrate gravity force
                rigidBody.state.vel += sceneGravityDeltaVel + rigidBody.gravityVector * timestep;
            }

            // Update FmAabb
            FmBox box;
            box.halfWidths[0] = rigidBody.dims[0];
            box.halfWidths[1] = rigidBody.dims[1];
            box.halfWidths[2] = rigidBody.dims[2];
            box.state = rigidBody.state;

            rigidBody.aabb = FmComputeBoxAabb(box, timestep, aabbPadding);

            // Update tet mesh used for collision detection
            FmVector3 boxPos = rigidBody.state.pos;
            FmVector3 boxVel = rigidBody.state.vel;
            FmVector3 boxAngVel = rigidBody.state.angVel;
            FmMatrix3 boxRot = FmInitMatrix3(rigidBody.state.quat);

            if (rigidBody.collisionObj)
            {
                FmTetMesh& rbTetMesh = *FmGetTetMesh(*(FmTetMeshBuffer*)rigidBody.collisionObj, 0);
                uint numVerts = rbTetMesh.numVerts;
                for (uint vId = 0; vId < numVerts; vId++)
                {
                    rbTetMesh.vertsPos[vId] = mul(boxRot, rbTetMesh.vertsRestPos[vId]) + boxPos;
                    rbTetMesh.vertsVel[vId] = boxVel + cross(boxAngVel, rbTetMesh.vertsPos[vId]);
                    if (isKinematic)
                    {
                        rbTetMesh.vertsFlags[vId] |= FM_VERT_FLAG_KINEMATIC;
                    }
                    else
                    {
                        rbTetMesh.vertsFlags[vId] &= ~FM_VERT_FLAG_KINEMATIC;
                    }
                }

                rbTetMesh.minPosition = rigidBody.aabb.pmin;
                rbTetMesh.maxPosition = rigidBody.aabb.pmax;

                // Update the tet mesh hierarchy
                FmBuildHierarchy(&rbTetMesh, timestep, aabbPadding);
            }

            // Reset the delta state values which are used in the constraint solve
            rigidBody.deltaVel = FmInitVector3(0.0f);
            rigidBody.deltaAngVel = FmInitVector3(0.0f);
            rigidBody.deltaPos = FmInitVector3(0.0f);
            rigidBody.deltaAngPos = FmInitVector3(0.0f);
        }

        taskData->progress.TaskIsFinished(taskData);
    }

    FM_WRAPPED_TASK_FUNC(FmTaskFuncStepVelocityRebuildBvh)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FmTaskDataStepVelocityRebuildBvh* taskData = (FmTaskDataStepVelocityRebuildBvh*)inTaskData;

        uint objIndex = taskData->progress.GetNextIndex();

        if (objIndex < taskData->numTetMeshes)
        {
            FmTaskFuncTetMeshStepVelocityRebuildBvh(inTaskData, objIndex, objIndex + 1);
        }
        else
        {           
            objIndex -= taskData->numTetMeshes;
            FmTaskFuncRbStepVelocityRebuildBvh(inTaskData, objIndex, objIndex + 1);
        }
    }

    void FmUpdatePositionsFracturePlasticity(FmScene* scene, FmTetMesh* tetMesh, float timestep, FmAsyncTaskData* parentTaskData)
    {
        if (FM_IS_SET(tetMesh->flags, FM_OBJECT_FLAG_SLEEPING))
        {
            return;
        }

        if (FM_IS_SET(tetMesh->flags, FM_OBJECT_FLAG_KINEMATIC))
        {
            if (FM_ANY_SET(tetMesh->flags, FM_OBJECT_FLAG_POS_CHANGED | FM_OBJECT_FLAG_VEL_CHANGED))
            {
                // Reset vel changed flag
                tetMesh->flags &= ~FM_OBJECT_FLAG_VEL_CHANGED;

                FmUpdateVertPositions(tetMesh, timestep);

                FmUpdateTetStateAndFracture(scene, tetMesh, false, false, parentTaskData);
            }
        }
        else
        {
            FmFailsafeReset(tetMesh, tetMesh->resetSpeedLimit);

            FmUpdateVertPositions(tetMesh, timestep);

            FmUpdateTetStateAndFracture(scene, tetMesh, true, true, parentTaskData);
        }
    }

    void FmUpdatePositionsFracturePlasticity(FmScene* scene, FmConstraintIsland* constraintIsland)
    {
        float timestep = scene->params.timestep;
        for (uint i = 0; i < constraintIsland->numTetMeshes; i++)
        {
            FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, constraintIsland->tetMeshIds[i]);
            FmUpdatePositionsFracturePlasticity(scene, &tetMesh, timestep, NULL);
        }

        if (!scene->params.rigidBodiesExternal)
        {
            for (uint i = 0; i < constraintIsland->numRigidBodiesConnected; i++)
            {
                FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, constraintIsland->rigidBodyIds[i]);
                FmStepState(rigidBody.state, timestep);
            }
        }
    }

    class FmTaskDataConstraintIslandSolve : public FmAsyncTaskData
    {
    public:
        FM_CLASS_NEW_DELETE(FmTaskDataConstraintIslandSolve)

        FmScene* scene;

        FmTaskDataConstraintIslandSolve(FmScene* inScene, FmTaskFuncCallback inFollowTaskFunc, void* inFollowTaskData)
        {
            scene = inScene;
            followTask.func = inFollowTaskFunc;
            followTask.data = inFollowTaskData;
        }
    };

    void FmTaskFuncConstraintIslandSolveBegin(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    void FmTaskFuncConstraintIslandSolve(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE);

        FmTaskDataConstraintIslandSolve* taskData = (FmTaskDataConstraintIslandSolve*)inTaskData;
        FmScene* scene = taskData->scene;

        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;
        FmConstraintSolverBuffer* constraintSolverBuffer = scene->constraintSolverBuffer;
        float timestep = scene->params.timestep;

        uint beginIdx = (uint)inTaskBeginIndex;
        uint endIdx = (uint)inTaskBeginIndex + 1;
        for (uint i = beginIdx; i < endIdx; i++)
        {
            uint sortedi = taskData->progress.GetNextIndex();
            FmConstraintIsland* constraintIsland = &constraintsBuffer->constraintIslands[sortedi];
            FmConstraintSolverData* constraintSolverData = &constraintSolverBuffer->islandSolverData[sortedi];

            if (!constraintSolverData->isAllocated)
            {
                continue;
            }

            if (constraintIsland->numConstraints == 0)
            {
                FmTestSleepingAndUpdateStats(scene, *constraintIsland);
                FmUpdatePositionsFracturePlasticity(scene, constraintIsland);
                continue;
            }

            FmSetupConstraintSolve(scene, constraintSolverData, constraintIsland, timestep);

            FmRunConstraintSolve(scene, constraintSolverData, *constraintIsland);

            FmUpdateConstraintLambdas(constraintSolverData, constraintsBuffer, *constraintIsland);

#if FM_CONSTRAINT_STABILIZATION_SOLVE
            FmSetupConstraintStabilization(scene, constraintSolverData, constraintIsland, timestep);

            FmRunConstraintSolve(scene, constraintSolverData, *constraintIsland);
#endif

            FmApplyConstraintSolveDeltasAndTestSleeping(scene, constraintSolverData, *constraintIsland);

            FmShutdownConstraintSolve(constraintSolverData);

            if (constraintIsland->islandCompletedCallback && (constraintIsland->numRigidBodiesInFEMSolve > 0 || constraintIsland->numUserRigidBodyIslands > 0))
            {
                constraintIsland->islandCompletedCallback(
                    scene, constraintIsland->userData,
                    constraintIsland->rigidBodyIds, constraintIsland->numRigidBodiesInFEMSolve,
                    constraintIsland->userRigidBodyIslandIndices, constraintIsland->numUserRigidBodyIslands);
            }
        }
    }

#if FM_ASYNC_THREADING
    void FmTaskFuncConstraintIslandSolveRunSolve(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    FM_WRAPPED_TASK_FUNC(FmTaskFuncConstraintIslandSolveBegin)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE_SETUP);

        FmTaskDataConstraintIslandSolve* taskData = (FmTaskDataConstraintIslandSolve*)inTaskData;
        FmScene* scene = taskData->scene;
        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;
        FmConstraintSolverBuffer* constraintSolverBuffer = scene->constraintSolverBuffer;

        float timestep = scene->params.timestep;

        uint islandIdx = taskData->progress.GetNextIndex();
        FmConstraintIsland* constraintIsland = &constraintsBuffer->constraintIslands[islandIdx];
        FmConstraintSolverData* constraintSolverData = &constraintSolverBuffer->islandSolverData[islandIdx];

        if (!constraintSolverData->isAllocated)
        {
            taskData->progress.TaskIsFinished();
            return;
        }

        if (constraintIsland->numConstraints == 0)
        {
            FmTestSleepingAndUpdateStats(scene, *constraintIsland);
            FmUpdatePositionsFracturePlasticity(scene, constraintIsland);
            FmShutdownConstraintSolve(constraintSolverData);

            taskData->progress.TaskIsFinished();
            return;
        }

        FmSetupConstraintSolve(scene, constraintSolverData, constraintIsland, timestep, FmTaskFuncConstraintIslandSolveRunSolve, taskData);
    }

    void FmTaskFuncConstraintIslandSolveSetupStabilization(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);
    void FmTaskFuncConstraintIslandSolveApplyDeltasTestSleeping(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    void FmTaskFuncConstraintIslandSolveRunSolve(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;

        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE_RUN_SOLVE);

        FmTaskDataConstraintIslandSolve* taskData = (FmTaskDataConstraintIslandSolve*)inTaskData;
        FmScene* scene = taskData->scene;
        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;
        FmConstraintSolverBuffer* constraintSolverBuffer = scene->constraintSolverBuffer;

        uint islandIdx = (uint)inTaskBeginIndex;
        FmConstraintIsland* constraintIsland = &constraintsBuffer->constraintIslands[islandIdx];
        FmConstraintSolverData* constraintSolverData = &constraintSolverBuffer->islandSolverData[islandIdx];

#if FM_CONSTRAINT_STABILIZATION_SOLVE
        FmRunConstraintSolve(scene, constraintSolverData, *constraintIsland, FmTaskFuncConstraintIslandSolveSetupStabilization, taskData);
#else
        FmRunConstraintSolve(scene, constraintSolverData, *constraintIsland, FmTaskFuncConstraintIslandSolveApplyDeltasTestSleeping, taskData);
#endif
    }

#if FM_CONSTRAINT_STABILIZATION_SOLVE
    void FmTaskFuncConstraintIslandSolveRunStabilization(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    void FmTaskFuncConstraintIslandSolveSetupStabilization(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;

        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE_SETUP_STABILIZATION);

        FmTaskDataConstraintIslandSolve* taskData = (FmTaskDataConstraintIslandSolve*)inTaskData;
        FmScene* scene = taskData->scene;
        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;
        FmConstraintSolverBuffer* constraintSolverBuffer = scene->constraintSolverBuffer;

        float timestep = scene->params.timestep;

        uint islandIdx = (uint)inTaskBeginIndex;
        FmConstraintIsland* constraintIsland = &constraintsBuffer->constraintIslands[islandIdx];
        FmConstraintSolverData* constraintSolverData = &constraintSolverBuffer->islandSolverData[islandIdx];

        FmUpdateConstraintLambdas(constraintSolverData, constraintsBuffer, *constraintIsland);

        FmSetupConstraintStabilization(scene, constraintSolverData, constraintIsland, timestep, FmTaskFuncConstraintIslandSolveRunStabilization, taskData);
    }

    void FmTaskFuncConstraintIslandSolveRunStabilization(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;

        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE_RUN_STABILIZATION);

        FmTaskDataConstraintIslandSolve* taskData = (FmTaskDataConstraintIslandSolve*)inTaskData;
        FmScene* scene = taskData->scene;
        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;
        FmConstraintSolverBuffer* constraintSolverBuffer = scene->constraintSolverBuffer;

        uint islandIdx = (uint)inTaskBeginIndex;
        FmConstraintIsland* constraintIsland = &constraintsBuffer->constraintIslands[islandIdx];
        FmConstraintSolverData* constraintSolverData = &constraintSolverBuffer->islandSolverData[islandIdx];

        FmRunConstraintSolve(scene, constraintSolverData, *constraintIsland, FmTaskFuncConstraintIslandSolveApplyDeltasTestSleeping, taskData);
    }
#endif

    void FmTaskFuncConstraintIslandSolveEnd(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    void FmTaskFuncConstraintIslandSolveApplyDeltasTestSleeping(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;

        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE_END);

        FmTaskDataConstraintIslandSolve* taskData = (FmTaskDataConstraintIslandSolve*)inTaskData;
        FmScene* scene = taskData->scene;
        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;
        FmConstraintSolverBuffer* constraintSolverBuffer = scene->constraintSolverBuffer;

        uint islandIdx = (uint)inTaskBeginIndex;
        FmConstraintIsland* constraintIsland = &constraintsBuffer->constraintIslands[islandIdx];
        FmConstraintSolverData* constraintSolverData = &constraintSolverBuffer->islandSolverData[islandIdx];

#if !FM_CONSTRAINT_STABILIZATION_SOLVE
        FmUpdateConstraintLambdas(constraintSolverData, constraintsBuffer, *constraintIsland);
#endif

        FmApplyConstraintSolveDeltasAndTestSleeping(scene, constraintSolverData, *constraintIsland, FmTaskFuncConstraintIslandSolveEnd, taskData);
    }

    void FmTaskFuncConstraintIslandSolveEnd(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;

        FM_TRACE_SCOPED_EVENT(ISLAND_SOLVE_END);

        FmTaskDataConstraintIslandSolve* taskData = (FmTaskDataConstraintIslandSolve*)inTaskData;
        FmScene* scene = taskData->scene;
        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;

        uint islandIdx = (uint)inTaskBeginIndex;
        FmConstraintIsland* constraintIsland = &constraintsBuffer->constraintIslands[islandIdx];

        FmConstraintSolverData* constraintSolverData = &scene->constraintSolverBuffer->islandSolverData[islandIdx];
        FmShutdownConstraintSolve(constraintSolverData);

        if (FmAtomicRead(&constraintIsland->isIslandStable.val) == 1
            && (constraintIsland->numFixedAttachments + FmAtomicRead(&constraintIsland->numFixedPoints.val)) >= 3)
        {
            FmMarkIslandForSleeping(scene, constraintIsland->islandId);
        }

        if (constraintIsland->islandCompletedCallback && (constraintIsland->numRigidBodiesInFEMSolve > 0 || constraintIsland->numUserRigidBodyIslands > 0))
        {
            constraintIsland->islandCompletedCallback(
                scene, constraintIsland->userData,
                constraintIsland->rigidBodyIds, constraintIsland->numRigidBodiesInFEMSolve,
                constraintIsland->userRigidBodyIslandIndices, constraintIsland->numUserRigidBodyIslands);
        }

        taskData->progress.TaskIsFinished();
    }
#endif

    void FmCollectRigidBodyIds(FmScene* scene)
    {
        // Collect rigid body ids for this simulation step
        uint numAwakeRigidBodies = 0;
        uint numSleepingRigidBodies = 0;
        uint numRigidBodySlots = scene->numRigidBodySlots;
        for (uint rbIdx = 0; rbIdx < numRigidBodySlots; rbIdx++)
        {
            FmRigidBody* pRigidBody = scene->rigidBodies[rbIdx];
            if (pRigidBody == NULL)
            {
                continue;
            }

            FmRigidBody& rigidBody = *pRigidBody;

            FmSetRigidBodyIdxById(scene, rigidBody.objectId, FM_INVALID_ID);

            if (FM_IS_SET(rigidBody.flags, FM_OBJECT_FLAG_SIMULATION_DISABLED))
            {
                continue;
            }

            if (FM_IS_SET(rigidBody.flags, FM_OBJECT_FLAG_SLEEPING))
            {
                uint idx = scene->maxRigidBodies - 1 - numSleepingRigidBodies;
                FmSetRigidBodyIdxById(scene, rigidBody.objectId, idx);
                scene->awakeRigidBodyIds[idx] = rigidBody.objectId;
                numSleepingRigidBodies++;
            }
            else
            {
                FmSetRigidBodyIdxById(scene, rigidBody.objectId, numAwakeRigidBodies);
                scene->awakeRigidBodyIds[numAwakeRigidBodies] = rigidBody.objectId;
                numAwakeRigidBodies++;
            }
        }
        scene->numAwakeRigidBodies = numAwakeRigidBodies;
        scene->numSleepingRigidBodies = numSleepingRigidBodies;

        scene->sleepingRigidBodyIds = scene->awakeRigidBodyIds + (scene->maxRigidBodies - numSleepingRigidBodies);
    }

    void FmWakeIslandsChangedBetweenSteps(FmScene* scene)
    {
        // Finish waking islands that may have been marked between steps by moving or deletion.
        FmWakeMarkedIslands(scene);

        // Zero out awakened object counts which are used only on the next waking pass (to find new contacts)
        scene->numAwakenedTetMeshes = 0;
        scene->numAwakenedRigidBodies = 0;
    }

    void FmUpdateUnconstrained(FmScene* scene, float timestep, FmTaskFuncCallback followTaskFunc, void* followTaskData)
    {
        FM_SET_START_TIME();

        FmSceneControlParams& sceneParams = scene->params;
        sceneParams.timestep = timestep;
        sceneParams.numThreads = scene->taskSystemCallbacks.GetTaskSystemNumThreads();
        
        // Override some settings when rigid bodies not external
        if (!sceneParams.rigidBodiesExternal)
        {
            sceneParams.includeRigidBodiesInBroadPhase = true;
            sceneParams.createZeroConstraintRigidBodyIslands = true;
        }

#if FM_DEBUG_CHECKS
        FmValidateScene(*scene, FM_SCENE_UPDATE_PHASE_START);
#endif

        FM_TRACE_START_EVENT(CONNECTED_COMPONENTS);
        // Complete waking of any islands marked between steps
        FmWakeIslandsChangedBetweenSteps(scene);

        // Recompute mesh connected components after any fracture in the last step.
        // This will create new tet mesh objects, and must be done before contact determination so that contact report tet and object ids are valid
        FmFindMeshConnectedComponents(scene);

        // Make arrays of active and sleeping object ids
        FmCollectTetMeshIds(scene);
        FmCollectRigidBodyIds(scene);
        FM_TRACE_STOP_EVENT(CONNECTED_COMPONENTS);

        FM_SET_MESH_COMPONENTS_TIME();

        FM_ASSERT(scene->numAwakeTetMeshes <= scene->maxTetMeshes);

#if FM_DEBUG_CHECKS
        FmValidateScene(*scene, FM_SCENE_UPDATE_PHASE_POST_CONNECTED_COMPONENTS);
#endif

        // Reset count before finding new deformation constraints.
        FmAtomicWrite(&scene->constraintsBuffer->numDeformationConstraints.val, 0);

        // Reset number of fracture reports
        FmAtomicWrite(&scene->fractureReport.numTetMeshReports.val, 0);

        FM_SET_START_TIME();

        uint numTetMeshes = scene->numAwakeTetMeshes;
        uint numRigidBodies = scene->numAwakeRigidBodies;

        uint numTasks = numTetMeshes;
        
        if (!scene->params.rigidBodiesExternal)
        {
            // FEMFX library handling all rigid body updating; add tasks to step velocity and rebuild BVH
            numTasks += numRigidBodies;
        }

#if FM_ASYNC_THREADING
        if (followTaskFunc)
        {
            if (numTasks > 0)
            {
                FmTaskDataStepVelocityRebuildBvh* taskData = new FmTaskDataStepVelocityRebuildBvh(scene, numTetMeshes, numRigidBodies);
                taskData->progress.Init(numTasks, followTaskFunc, followTaskData);

                FmParallelForAsync("StepVelocityRebuildBvh", FM_TASK_AND_WRAPPED_TASK_ARGS(FmTaskFuncStepVelocityRebuildBvh), NULL, taskData, numTasks, scene->taskSystemCallbacks.SubmitAsyncTask, scene->params.numThreads);
            }
            else
            {
                FmSetNextTask(followTaskFunc, followTaskData, 0, 1);
            }
        }
#else
        FmTaskDataStepVelocityRebuildBvh taskData(scene, numTetMeshes, numRigidBodies);
        scene->taskSystemCallbacks.ParallelFor("ImplicitSolveBvhBuild", FmTaskFuncStepVelocityRebuildBvh, &taskData, numTasks);

        // Add to warnings report if hit deformation constraint limit
        if (FmAtomicRead(&scene->constraintsBuffer->numDeformationConstraints.val) >= scene->constraintsBuffer->maxDeformationConstraints)
        {
            FmAtomicOr(&scene->warningsReport.flags.val, FM_WARNING_FLAG_HIT_LIMIT_SCENE_DEFORMATION_CONSTRAINTS);
        }

        FM_SET_IMPLICIT_STEP_TIME();
#endif
    }

    // For each glue constraint, use the current object state to update delta between glue attachment points.
    void FmUpdateGlueConstraints(
        FmScene* scene,
        FmGluedObjectPairSet& gluedObjectPairSet,
        FmGlueConstraint* glueConstraints,
        uint numGlueConstraints)
    {
        FmInitHashSet(&gluedObjectPairSet, gluedObjectPairSet.elements, numGlueConstraints);

        FmTetMeshBuffer** tetMeshBuffers = scene->tetMeshBuffers;

        for (uint glueConstraintIdx = 0; glueConstraintIdx < numGlueConstraints; glueConstraintIdx++)
        {
            FmGlueConstraint& glueConstraint = glueConstraints[glueConstraintIdx];

            FmVector3 posA, posB;
            uint dynamicFlagsA = 0, dynamicFlagsB = 0, movingFlagsA = 0, movingFlagsB = 0;

            if (FM_ANY_SET(glueConstraint.flags, (FM_CONSTRAINT_FLAG_DISABLED | FM_CONSTRAINT_FLAG_DELETED)))
            {
                glueConstraint.objectIdA = FM_INVALID_ID;
                glueConstraint.objectIdB = FM_INVALID_ID;
                continue;
            }

            if (glueConstraint.bufferIdA & FM_RB_FLAG)
            {
                // If rigid body, bufferId interpreted as objectId
                glueConstraint.objectIdA = glueConstraint.bufferIdA;

                FmRigidBody* pRigidBodyA = FmGetRigidBodyPtrById(*scene, glueConstraint.objectIdA);
                if (pRigidBodyA == NULL)
                {
                    glueConstraint.objectIdA = FM_INVALID_ID;
                    glueConstraint.objectIdB = FM_INVALID_ID;
                    glueConstraint.flags |= FM_CONSTRAINT_FLAG_DISABLED;
                    continue;
                }

                FmRigidBody& rigidBodyA = *pRigidBodyA;

                FmVector3 posBodySpaceA = FmInitVector3(glueConstraint.posBodySpaceA[0], glueConstraint.posBodySpaceA[1], glueConstraint.posBodySpaceA[2]);

                glueConstraint.comToPosA = rotate(rigidBodyA.state.quat, posBodySpaceA);

                posA = rigidBodyA.state.pos + glueConstraint.comToPosA;

                if (FM_NONE_SET(rigidBodyA.flags, FM_OBJECT_FLAG_SLEEPING | FM_OBJECT_FLAG_SIMULATION_DISABLED))
                {
                    FmDynamicAndMovingFlags(&dynamicFlagsA, &movingFlagsA, rigidBodyA);
                }
            }
            else
            {
                // Skip constraint if tet mesh buffer has been removed.
                FmTetMeshBuffer* pTetMeshBuffer = tetMeshBuffers[glueConstraint.bufferIdA];
                if (pTetMeshBuffer == NULL)
                {
                    glueConstraint.objectIdA = FM_INVALID_ID;
                    glueConstraint.objectIdB = FM_INVALID_ID;
                    glueConstraint.flags |= FM_CONSTRAINT_FLAG_DISABLED;
                    continue;
                }

                // Find the appropriate tet mesh object given the buffer and tet.
                FmTetMeshBuffer& tetMeshBufferA = *pTetMeshBuffer;
                FmTetReference tetRefA = tetMeshBufferA.tetReferences[glueConstraint.bufferTetIdA];
                FmTetMesh& tetMeshA = tetMeshBufferA.tetMeshes[tetRefA.meshIdx];
                uint tetIdA = tetRefA.tetId;
                FmTetVertIds tetVertIdsA = tetMeshA.tetsVertIds[tetIdA];

                glueConstraint.objectIdA = tetMeshA.objectId;
                glueConstraint.tetIdA = tetIdA;

                posA = FmInterpolate(glueConstraint.posBaryA,
                    tetMeshA.vertsPos[tetVertIdsA.ids[0]],
                    tetMeshA.vertsPos[tetVertIdsA.ids[1]],
                    tetMeshA.vertsPos[tetVertIdsA.ids[2]],
                    tetMeshA.vertsPos[tetVertIdsA.ids[3]]);

                if (FM_NONE_SET(tetMeshA.flags, FM_OBJECT_FLAG_SLEEPING | FM_OBJECT_FLAG_SIMULATION_DISABLED))
                {
                    FmDynamicAndMovingFlags(&dynamicFlagsA, &movingFlagsA, tetMeshA, tetVertIdsA, glueConstraint.posBaryA);
                }
            }

            if (glueConstraint.bufferIdB == FM_INVALID_ID)
            {
                // bufferIdB invalid interpreted as point in world
                glueConstraint.objectIdB = FM_INVALID_ID;

                posB = FmInitVector3(glueConstraint.posWorldB[0], glueConstraint.posWorldB[1], glueConstraint.posWorldB[2]);
            }
            else if (glueConstraint.bufferIdB & FM_RB_FLAG)
            {
                // If rigid body, bufferId interpreted as objectId
                glueConstraint.objectIdB = glueConstraint.bufferIdB;

                FmRigidBody* pRigidBodyB = FmGetRigidBodyPtrById(*scene, glueConstraint.objectIdB);
                if (pRigidBodyB == NULL)
                {
                    glueConstraint.objectIdA = FM_INVALID_ID;
                    glueConstraint.objectIdB = FM_INVALID_ID;
                    glueConstraint.flags |= FM_CONSTRAINT_FLAG_DISABLED;
                    continue;
                }

                FmRigidBody& rigidBodyB = *pRigidBodyB;

                FmVector3 posBodySpaceB = FmInitVector3(glueConstraint.posBodySpaceB[0], glueConstraint.posBodySpaceB[1], glueConstraint.posBodySpaceB[2]);

                glueConstraint.comToPosB = rotate(rigidBodyB.state.quat, posBodySpaceB);

                posB = rigidBodyB.state.pos + glueConstraint.comToPosB;

                if (FM_NONE_SET(rigidBodyB.flags, FM_OBJECT_FLAG_SLEEPING | FM_OBJECT_FLAG_SIMULATION_DISABLED))
                {
                    FmDynamicAndMovingFlags(&dynamicFlagsB, &movingFlagsB, rigidBodyB);
                }
            }
            else
            {
                // Skip constraint if tet mesh buffer has been removed.
                FmTetMeshBuffer* pTetMeshBuffer = tetMeshBuffers[glueConstraint.bufferIdB];
                if (pTetMeshBuffer == NULL)
                {
                    glueConstraint.objectIdA = FM_INVALID_ID;
                    glueConstraint.objectIdB = FM_INVALID_ID;
                    glueConstraint.flags |= FM_CONSTRAINT_FLAG_DISABLED;
                    continue;
                }

                // Find the appropriate tet mesh object given the buffer and tet.
                FmTetMeshBuffer& tetMeshBufferB = *pTetMeshBuffer;
                FmTetReference tetRefB = tetMeshBufferB.tetReferences[glueConstraint.bufferTetIdB];
                FmTetMesh& tetMeshB = tetMeshBufferB.tetMeshes[tetRefB.meshIdx];
                uint tetIdB = tetRefB.tetId;
                FmTetVertIds tetVertIdsB = tetMeshB.tetsVertIds[tetIdB];

                glueConstraint.objectIdB = tetMeshB.objectId;
                glueConstraint.tetIdB = tetIdB;

                posB = FmInterpolate(glueConstraint.posBaryB,
                    tetMeshB.vertsPos[tetVertIdsB.ids[0]],
                    tetMeshB.vertsPos[tetVertIdsB.ids[1]],
                    tetMeshB.vertsPos[tetVertIdsB.ids[2]],
                    tetMeshB.vertsPos[tetVertIdsB.ids[3]]);

                if (FM_NONE_SET(tetMeshB.flags, FM_OBJECT_FLAG_SLEEPING | FM_OBJECT_FLAG_SIMULATION_DISABLED))
                {
                    FmDynamicAndMovingFlags(&dynamicFlagsB, &movingFlagsB, tetMeshB, tetVertIdsB, glueConstraint.posBaryB);
                }
            }

            FmSetConstraintFlags(&glueConstraint.flags, &glueConstraint.dynamicFlags, &glueConstraint.movingFlags, dynamicFlagsA, dynamicFlagsB, movingFlagsA, movingFlagsB);

            if (FM_ALL_SET(glueConstraint.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED | FM_CONSTRAINT_FLAG_OBJECTB_FIXED))
            {
                continue;
            }

            // If breaking enabled, check last impulse calculated against the breaking threshold
            if (glueConstraint.breakThreshold > 0.0f)
            {
                FmVector3 lambda(glueConstraint.lambdaX, glueConstraint.lambdaY, glueConstraint.lambdaZ);
                if (length(lambda) > glueConstraint.breakThreshold)
                {
                    glueConstraint.flags |= FM_CONSTRAINT_FLAG_DISABLED;
                }
            }

            // Count the number of enabled glue constraints between object pairs.
            // This does not count glue that's disabled, deleted, or between fixed/sleeping objects.
            if (FM_NOT_SET(glueConstraint.flags, FM_CONSTRAINT_FLAG_DISABLED))
            {
                // Now that objectIds updated, add to hash set to count number of glue contacts per object pair
                FmGluedObjectPairKey pairKey;
                pairKey.objectIdA = glueConstraint.objectIdA;
                pairKey.objectIdB = glueConstraint.objectIdB;
                bool foundInSet;
                FmGluedObjectPairElement* pairElement;
                pairElement = FmInsertElement(&foundInSet, &gluedObjectPairSet, pairKey);
                pairElement->glueCount++;
            }

            glueConstraint.deltaPos = posA - posB;
        }

        // Disable glue constraints based on minimum number required between objects.
        // Note, there's only one pass, so after removing the current ones < min, there may be some
        // constraints kept even though the new count is < their min, but these will be removed in 
        // subsequent steps.
        for (uint glueConstraintIdx = 0; glueConstraintIdx < numGlueConstraints; glueConstraintIdx++)
        {
            FmGlueConstraint& glueConstraint = glueConstraints[glueConstraintIdx];

            // Skip already removed glue
            if (FM_ANY_SET(glueConstraint.flags, FM_CONSTRAINT_FLAG_DISABLED | FM_CONSTRAINT_FLAG_DELETED))
            {
                continue;
            }

            // Skip fixed/fixed constraints, which includes those between sleeping objects.
            // Glue constraints aren't counted in this case.
            if (FM_ALL_SET(glueConstraint.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED | FM_CONSTRAINT_FLAG_OBJECTB_FIXED))
            {
                continue;
            }

            // Check if count of constraints is less than minimum
            // Count will include this constraint, so always be at least 1.
            if (glueConstraint.minGlueConstraints > 1)  
            {
                FmGluedObjectPairKey pairKey;
                pairKey.objectIdA = glueConstraint.objectIdA;
                pairKey.objectIdB = glueConstraint.objectIdB;
                bool foundInSet;
                FmGluedObjectPairElement* pairElement;
                pairElement = FmInsertElement(&foundInSet, &gluedObjectPairSet, pairKey);
                if (foundInSet)
                {
                    if (pairElement->glueCount < glueConstraint.minGlueConstraints)
                    {
                        glueConstraint.flags |= FM_CONSTRAINT_FLAG_DISABLED;
                    }
                }
            }
        }
    }

    // For each plane constraint, use the current object state to update projections.
    void FmUpdatePlaneConstraints(
        FmScene* scene,
        FmPlaneConstraint* planeConstraints,
        uint numPlaneConstraints)
    {
        FmTetMeshBuffer** tetMeshBuffers = scene->tetMeshBuffers;

        for (uint planeConstraintIdx = 0; planeConstraintIdx < numPlaneConstraints; planeConstraintIdx++)
        {
            FmPlaneConstraint& planeConstraint = planeConstraints[planeConstraintIdx];

            if (FM_ANY_SET(planeConstraint.flags, (FM_CONSTRAINT_FLAG_DISABLED | FM_CONSTRAINT_FLAG_DELETED)))
            {
                planeConstraint.objectIdA = FM_INVALID_ID;
                planeConstraint.objectIdB = FM_INVALID_ID;
                continue;
            }

            FmVector3 posA, posB;
            uint dynamicFlagsA = 0, dynamicFlagsB = 0, movingFlagsA = 0, movingFlagsB = 0;

            if (planeConstraint.bufferIdA & FM_RB_FLAG)
            {
                // If rigid body, bufferId interpreted as objectId
                planeConstraint.objectIdA = planeConstraint.bufferIdA;

                FmRigidBody* pRigidBodyA = FmGetRigidBodyPtrById(*scene, planeConstraint.objectIdA);
                if (pRigidBodyA == NULL)
                {
                    planeConstraint.objectIdA = FM_INVALID_ID;
                    planeConstraint.objectIdB = FM_INVALID_ID;
                    planeConstraint.flags |= FM_CONSTRAINT_FLAG_DISABLED;
                    continue;
                }

                FmRigidBody& rigidBodyA = *pRigidBodyA;

                FmVector3 posBodySpaceA = FmInitVector3(planeConstraint.posBodySpaceA[0], planeConstraint.posBodySpaceA[1], planeConstraint.posBodySpaceA[2]);

                planeConstraint.comToPosA = rotate(rigidBodyA.state.quat, posBodySpaceA);

                posA = rigidBodyA.state.pos + planeConstraint.comToPosA;

                if (FM_NONE_SET(rigidBodyA.flags, FM_OBJECT_FLAG_SLEEPING | FM_OBJECT_FLAG_SIMULATION_DISABLED))
                {
                    FmDynamicAndMovingFlags(&dynamicFlagsA, &movingFlagsA, rigidBodyA);
                }
            }
            else
            {
                // Skip constraint if tet mesh buffer has been removed.
                FmTetMeshBuffer* pTetMeshBuffer = tetMeshBuffers[planeConstraint.bufferIdA];
                if (pTetMeshBuffer == NULL)
                {
                    planeConstraint.objectIdA = FM_INVALID_ID;
                    planeConstraint.objectIdB = FM_INVALID_ID;
                    planeConstraint.flags |= FM_CONSTRAINT_FLAG_DISABLED;
                    continue;
                }

                // Find the appropriate tet mesh object given the buffer and tet.
                FmTetMeshBuffer& tetMeshBufferA = *tetMeshBuffers[planeConstraint.bufferIdA];
                FmTetReference tetRefA = tetMeshBufferA.tetReferences[planeConstraint.bufferTetIdA];
                FmTetMesh& tetMeshA = tetMeshBufferA.tetMeshes[tetRefA.meshIdx];
                uint tetIdA = tetRefA.tetId;
                FmTetVertIds tetVertIdsA = tetMeshA.tetsVertIds[tetIdA];

                planeConstraint.objectIdA = tetMeshA.objectId;
                planeConstraint.tetIdA = tetIdA;

                posA = FmInterpolate(planeConstraint.posBaryA,
                    tetMeshA.vertsPos[tetVertIdsA.ids[0]],
                    tetMeshA.vertsPos[tetVertIdsA.ids[1]],
                    tetMeshA.vertsPos[tetVertIdsA.ids[2]],
                    tetMeshA.vertsPos[tetVertIdsA.ids[3]]);

                if (FM_NONE_SET(tetMeshA.flags, FM_OBJECT_FLAG_SLEEPING | FM_OBJECT_FLAG_SIMULATION_DISABLED))
                {
                    FmDynamicAndMovingFlags(&dynamicFlagsA, &movingFlagsA, tetMeshA, tetVertIdsA, planeConstraint.posBaryA);
                }
            }

            if (planeConstraint.bufferIdB == FM_INVALID_ID)
            {
                // bufferIdB invalid interpreted as point in world
                planeConstraint.objectIdB = FM_INVALID_ID;

                posB = FmInitVector3(planeConstraint.posWorldB[0], planeConstraint.posWorldB[1], planeConstraint.posWorldB[2]);
            }
            else if (planeConstraint.bufferIdB & FM_RB_FLAG)
            {
                // If rigid body, bufferId interpreted as objectId
                planeConstraint.objectIdB = planeConstraint.bufferIdB;

                FmRigidBody* pRigidBodyB = FmGetRigidBodyPtrById(*scene, planeConstraint.objectIdB);
                if (pRigidBodyB == NULL)
                {
                    planeConstraint.objectIdA = FM_INVALID_ID;
                    planeConstraint.objectIdB = FM_INVALID_ID;
                    planeConstraint.flags |= FM_CONSTRAINT_FLAG_DISABLED;
                    continue;
                }

                FmRigidBody& rigidBodyB = *pRigidBodyB;

                FmVector3 posBodySpaceB = FmInitVector3(planeConstraint.posBodySpaceB[0], planeConstraint.posBodySpaceB[1], planeConstraint.posBodySpaceB[2]);

                planeConstraint.comToPosB = rotate(rigidBodyB.state.quat, posBodySpaceB);

                posB = rigidBodyB.state.pos + planeConstraint.comToPosB;

                if (FM_NONE_SET(rigidBodyB.flags, FM_OBJECT_FLAG_SLEEPING | FM_OBJECT_FLAG_SIMULATION_DISABLED))
                {
                    FmDynamicAndMovingFlags(&dynamicFlagsB, &movingFlagsB, rigidBodyB);
                }
            }
            else
            {
                // Skip constraint if tet mesh buffer has been removed.
                FmTetMeshBuffer* pTetMeshBuffer = tetMeshBuffers[planeConstraint.bufferIdB];
                if (pTetMeshBuffer == NULL)
                {
                    planeConstraint.objectIdA = FM_INVALID_ID;
                    planeConstraint.objectIdB = FM_INVALID_ID;
                    planeConstraint.flags |= FM_CONSTRAINT_FLAG_DISABLED;
                    continue;
                }

                // Find the appropriate tet mesh object given the buffer and tet.
                FmTetMeshBuffer& tetMeshBufferB = *tetMeshBuffers[planeConstraint.bufferIdB];
                FmTetReference tetRefB = tetMeshBufferB.tetReferences[planeConstraint.bufferTetIdB];
                FmTetMesh& tetMeshB = tetMeshBufferB.tetMeshes[tetRefB.meshIdx];
                uint tetIdB = tetRefB.tetId;
                FmTetVertIds tetVertIdsB = tetMeshB.tetsVertIds[tetIdB];

                planeConstraint.objectIdB = tetMeshB.objectId;
                planeConstraint.tetIdB = tetIdB;

                posB = FmInterpolate(planeConstraint.posBaryB,
                    tetMeshB.vertsPos[tetVertIdsB.ids[0]],
                    tetMeshB.vertsPos[tetVertIdsB.ids[1]],
                    tetMeshB.vertsPos[tetVertIdsB.ids[2]],
                    tetMeshB.vertsPos[tetVertIdsB.ids[3]]);

                if (FM_NONE_SET(tetMeshB.flags, FM_OBJECT_FLAG_SLEEPING | FM_OBJECT_FLAG_SIMULATION_DISABLED))
                {
                    FmDynamicAndMovingFlags(&dynamicFlagsB, &movingFlagsB, tetMeshB, tetVertIdsB, planeConstraint.posBaryB);
                }
            }

            FmSetConstraintFlags(&planeConstraint.flags, &planeConstraint.dynamicFlags, &planeConstraint.movingFlags, dynamicFlagsA, dynamicFlagsB, movingFlagsA, movingFlagsB);

            if (FM_ALL_SET(planeConstraint.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED | FM_CONSTRAINT_FLAG_OBJECTB_FIXED))
            {
                continue;
            }

            FmVector3 deltaPos = posA - posB;
            planeConstraint.projection0 = dot(deltaPos, planeConstraint.planeNormal0);
            planeConstraint.projection1 = dot(deltaPos, planeConstraint.planeNormal1);
            planeConstraint.projection2 = dot(deltaPos, planeConstraint.planeNormal2);
        }
    }

    float FmComputeHingeRelAngVel(const FmScene& scene, const FmRigidBodyAngleConstraint& hingeAngleConstraint)
    {
        // Body hinge axes transformed to world
        FmVector3 hingeAxisA, hingeAxisB;
        FmVector3 angVelA, angVelB;

        if (FM_ANY_SET(hingeAngleConstraint.flags, FM_CONSTRAINT_FLAG_DISABLED | FM_CONSTRAINT_FLAG_DELETED))
        {
            return 0.0f;
        }

        const FmRigidBody& rigidBodyA = *FmGetRigidBodyPtrById(scene, hingeAngleConstraint.objectIdA);

        hingeAxisA = rotate(rigidBodyA.state.quat, hingeAngleConstraint.axisBodySpaceA);
        angVelA = rigidBodyA.state.angVel;

        if (hingeAngleConstraint.objectIdB == FM_INVALID_ID)  // Hinge is attached to environment
        {
            // axisBodySpaceB is actually world space vector
            hingeAxisB = hingeAngleConstraint.axisBodySpaceB;
            angVelB = FmInitVector3(0.0f);
        }
        else
        {
            const FmRigidBody& rigidBodyB = *FmGetRigidBodyPtrById(scene, hingeAngleConstraint.objectIdB);

            hingeAxisB = rotate(rigidBodyB.state.quat, hingeAngleConstraint.axisBodySpaceB);
            angVelB = rigidBodyB.state.angVel;
        }

        // Compute world space hinge axis as average
        FmVector3 hingeAxis = FmSafeNormalize((hingeAxisA + hingeAxisB) * 0.5f, hingeAxisA);

        return dot(hingeAxis, angVelA - angVelB);
    }

    float FmComputeHingeRelAngVel(const FmScene& scene, uint rigidBodyAngleConstraintId)
    {
        FmRigidBodyAngleConstraint* pConstraint = FmGetRigidBodyAngleConstraint(scene, rigidBodyAngleConstraintId);

        if (pConstraint == NULL)
        {
            return 0.0f;
        }

        return FmComputeHingeRelAngVel(scene, *pConstraint);
    }

    // Assuming valid constraint object ids, axisBodySpaceA, and axisBodySpaceB
    // update the Jacobian and error values used in constraint solver. 
    void FmUpdateHingeAngleConstraint(FmScene* scene, FmRigidBodyAngleConstraint* constraint)
    {
        FmRigidBodyAngleConstraint& hingeAngleConstraint = *constraint;

        // Body hinge axes transformed to world
        FmVector3 hingeAxisA, hingeAxisB;
        uint dynamicFlagsA = 0, dynamicFlagsB = 0, movingFlagsA = 0, movingFlagsB = 0;

        FmRigidBody* pRigidBodyA = FmGetRigidBodyPtrById(*scene, hingeAngleConstraint.objectIdA);
        if (pRigidBodyA == NULL)
        {
            hingeAngleConstraint.objectIdA = FM_INVALID_ID;
            hingeAngleConstraint.objectIdB = FM_INVALID_ID;
            hingeAngleConstraint.flags |= FM_CONSTRAINT_FLAG_DISABLED;
            return;
        }

        FmRigidBody& rigidBodyA = *pRigidBodyA;

        hingeAxisA = rotate(rigidBodyA.state.quat, hingeAngleConstraint.axisBodySpaceA);

        if (FM_NONE_SET(rigidBodyA.flags, FM_OBJECT_FLAG_SLEEPING | FM_OBJECT_FLAG_SIMULATION_DISABLED))
        {
            FmDynamicAndMovingFlags(&dynamicFlagsA, &movingFlagsA, rigidBodyA);
        }

        if (hingeAngleConstraint.objectIdB == FM_INVALID_ID)  // Hinge is attached to environment
        {
            // axisBodySpaceB is actually world space vector
            hingeAxisB = hingeAngleConstraint.axisBodySpaceB;

            hingeAngleConstraint.flags |= FM_CONSTRAINT_FLAG_OBJECTB_FIXED;
        }
        else
        {
            FmRigidBody* pRigidBodyB = FmGetRigidBodyPtrById(*scene, hingeAngleConstraint.objectIdB);
            if (pRigidBodyB == NULL)
            {
                hingeAngleConstraint.objectIdA = FM_INVALID_ID;
                hingeAngleConstraint.objectIdB = FM_INVALID_ID;
                hingeAngleConstraint.flags |= FM_CONSTRAINT_FLAG_DISABLED;
                return;
            }

            FmRigidBody& rigidBodyB = *pRigidBodyB;

            hingeAxisB = rotate(rigidBodyB.state.quat, hingeAngleConstraint.axisBodySpaceB);

            if (FM_NONE_SET(rigidBodyB.flags, FM_OBJECT_FLAG_SLEEPING | FM_OBJECT_FLAG_SIMULATION_DISABLED))
            {
                FmDynamicAndMovingFlags(&dynamicFlagsB, &movingFlagsB, rigidBodyB);
            }
        }

        FmSetConstraintFlags(&hingeAngleConstraint.flags, &hingeAngleConstraint.dynamicFlags, &hingeAngleConstraint.movingFlags, dynamicFlagsA, dynamicFlagsB, movingFlagsA, movingFlagsB);

        if (FM_ALL_SET(hingeAngleConstraint.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED | FM_CONSTRAINT_FLAG_OBJECTB_FIXED))
        {
            return;
        }

        // Compute world space hinge axis as average
        FmVector3 hingeAxis = FmSafeNormalize((hingeAxisA + hingeAxisB) * 0.5f, hingeAxisA);
        FmVector3 tangent1, tangent2;

        tangent1 = normalize(FmOrthogonalVector(hingeAxis));
        tangent2 = cross(hingeAxis, tangent1);

        FmMatrix3 jacobianA;
        jacobianA.col0 = hingeAxis;
        jacobianA.col1 = tangent1;
        jacobianA.col2 = tangent2;
        jacobianA = transpose(jacobianA);

        FmMatrix3 jacobianB;
        jacobianB.col0 = -hingeAxis;
        jacobianB.col1 = -tangent1;
        jacobianB.col2 = -tangent2;
        jacobianB = transpose(jacobianB);

        hingeAngleConstraint.jacobianA = jacobianA;
        hingeAngleConstraint.jacobianB = jacobianB;

        FmVector3 errorAxis = cross(hingeAxisB, hingeAxisA);
        hingeAngleConstraint.error0 = 0.0f;
        hingeAngleConstraint.error1 = dot(tangent1, errorAxis);
        hingeAngleConstraint.error2 = dot(tangent2, errorAxis);
    }

    // For each rigidBodyAngle constraint, use the current object state to update projections.
    void FmUpdateRigidBodyAngleConstraints(
        FmScene* scene,
        FmRigidBodyAngleConstraint* rigidBodyAngleConstraints,
        uint numRigidBodyAngleConstraints)
    {
        for (uint angleConstraintIdx = 0; angleConstraintIdx < numRigidBodyAngleConstraints; angleConstraintIdx++)
        {
            FmRigidBodyAngleConstraint& angleConstraint = rigidBodyAngleConstraints[angleConstraintIdx];

            // Skip constraint if objectIdA invalid
            if (FM_ANY_SET(angleConstraint.flags, (FM_CONSTRAINT_FLAG_DISABLED | FM_CONSTRAINT_FLAG_DELETED)))
            {
                continue;
            }

            FM_ASSERT(angleConstraint.objectIdA & FM_RB_FLAG);
            FM_ASSERT(angleConstraint.objectIdB == FM_INVALID_ID || (angleConstraint.objectIdB & FM_RB_FLAG));

            if (angleConstraint.type == FM_RB_JOINT_TYPE_HINGE)
            {
                FmUpdateHingeAngleConstraint(scene, &angleConstraint);
            }
        }
    }

    // Find the constraint islands prior to solve.
    // Include the user rigid bodies and contacts/constraints.
    void FmFindConstraintIslands(
        FmScene* scene,
        const FmUserConstraints* userConstraints)
    {
        FM_TRACE_SCOPED_EVENT(CONSTRAINT_ISLANDS);

        FM_SET_START_TIME();

        FmConstraintsBuffer& constraintsBuffer = *scene->constraintsBuffer;

        FM_ASSERT(constraintsBuffer.numGlueConstraints <= constraintsBuffer.maxGlueConstraints);
        FM_ASSERT(constraintsBuffer.numPlaneConstraints <= constraintsBuffer.maxPlaneConstraints);

        if (userConstraints)
        {
            // Copy rigid/FEM constraints into FEM scene arrays

            constraintsBuffer.innerIterationCallback = userConstraints->innerIterationCallback;
            constraintsBuffer.islandCompletedCallback = userConstraints->islandCompletedCallback;
            constraintsBuffer.userData = userConstraints->userData;
            constraintsBuffer.numUserRigidBodyIslands = userConstraints->numIslands;

            uint appendIdx = FmAtomicRead(&constraintsBuffer.numDistanceContacts.val);
            for (uint contactIdx = 0; contactIdx < userConstraints->numDistanceContacts && appendIdx < constraintsBuffer.maxDistanceContacts; contactIdx++)
            {
                constraintsBuffer.distanceContactsPairInfo[appendIdx] = userConstraints->distanceContactsPairInfo[contactIdx];
                constraintsBuffer.distanceContacts[appendIdx] = userConstraints->distanceContacts[contactIdx];
                FmAtomicIncrement(&constraintsBuffer.numDistanceContacts.val);
                appendIdx++;
            }

            if (FmAtomicRead(&constraintsBuffer.numVolumeContactVerts.val) + userConstraints->numVolumeContactVerts < constraintsBuffer.maxVolumeContactVerts)
            {
                uint baseVolContactIdx = FmAtomicRead(&constraintsBuffer.numVolumeContactVerts.val);

                appendIdx = baseVolContactIdx;
                for (uint contactIdx = 0; contactIdx < userConstraints->numVolumeContactVerts; contactIdx++)
                {
                    constraintsBuffer.volumeContactVerts[appendIdx] = userConstraints->volumeContactVerts[contactIdx];
                    FmAtomicIncrement(&constraintsBuffer.numVolumeContactVerts.val);
                    appendIdx++;
                }

                appendIdx = FmAtomicRead(&constraintsBuffer.numVolumeContacts.val);
                for (uint contactIdx = 0; contactIdx < userConstraints->numVolumeContacts && appendIdx < constraintsBuffer.maxVolumeContacts; contactIdx++)
                {
                    constraintsBuffer.volumeContacts[appendIdx] = userConstraints->volumeContacts[contactIdx];
                    constraintsBuffer.volumeContacts[appendIdx].volVertsStartA += baseVolContactIdx;
                    constraintsBuffer.volumeContacts[appendIdx].volVertsStartB += baseVolContactIdx;
                    FmAtomicIncrement(&constraintsBuffer.numVolumeContacts.val);
                    appendIdx++;
                }

            }

            appendIdx = constraintsBuffer.numGlueConstraints;
            for (uint contactIdx = 0; contactIdx < userConstraints->numGlueConstraints && appendIdx < constraintsBuffer.maxGlueConstraints; contactIdx++)
            {
                constraintsBuffer.glueConstraints[appendIdx] = userConstraints->glueConstraints[contactIdx];
                constraintsBuffer.numGlueConstraints++;
                appendIdx++;
            }
            appendIdx = constraintsBuffer.numPlaneConstraints;
            for (uint contactIdx = 0; contactIdx < userConstraints->numPlaneConstraints && appendIdx < constraintsBuffer.maxPlaneConstraints; contactIdx++)
            {
                constraintsBuffer.planeConstraints[appendIdx] = userConstraints->planeConstraints[contactIdx];
                constraintsBuffer.numPlaneConstraints++;
                appendIdx++;
            }
            appendIdx = constraintsBuffer.numRigidBodyAngleConstraints;
            for (uint contactIdx = 0; contactIdx < userConstraints->numRigidBodyAngleConstraints && appendIdx < constraintsBuffer.maxRigidBodyAngleConstraints; contactIdx++)
            {
                constraintsBuffer.rigidBodyAngleConstraints[appendIdx] = userConstraints->rigidBodyAngleConstraints[contactIdx];
                constraintsBuffer.numRigidBodyAngleConstraints++;
                appendIdx++;
            }
        }
        else
        {
            scene->constraintsBuffer->innerIterationCallback = NULL;
            scene->constraintsBuffer->islandCompletedCallback = NULL;
            scene->constraintsBuffer->numUserRigidBodyIslands = 0;
        }

        FmUpdateGlueConstraints(scene, constraintsBuffer.gluedObjectPairs, constraintsBuffer.glueConstraints, constraintsBuffer.numGlueConstraints);

        FmUpdatePlaneConstraints(scene, constraintsBuffer.planeConstraints, constraintsBuffer.numPlaneConstraints);

        FmUpdateRigidBodyAngleConstraints(scene, constraintsBuffer.rigidBodyAngleConstraints, constraintsBuffer.numRigidBodyAngleConstraints);

        FmFindConstraintIslands(&constraintsBuffer, scene);
        FmAllocConstraintIslandSolverData(scene);

#if FM_DEBUG_CHECKS
        FmValidateScene(*scene, FM_SCENE_UPDATE_PHASE_POST_ISLANDS);
#endif

        FM_SET_CONSTRAINT_ISLANDS_TIME();
    }

    void FmTaskFuncSceneConstraintSolvePutIslandsToSleep(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    void FmSceneConstraintSolve(FmScene* scene, FmTaskFuncCallback followTaskFunc, void* followTaskData)
    {
#if !FM_ASYNC_THREADING
        (void)followTaskFunc;
        (void)followTaskData;
        FM_TRACE_SCOPED_EVENT(CONSTRAINT_SOLVE);
#endif

        FM_SET_START_TIME();

        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;

#if FM_ASYNC_THREADING
        if (followTaskFunc)
        {
            FmTaskDataConstraintIslandSolve* taskData = new FmTaskDataConstraintIslandSolve(scene, followTaskFunc, followTaskData);

            uint numTasks = constraintsBuffer->numConstraintIslands;

            taskData->progress.Init(numTasks, FmTaskFuncSceneConstraintSolvePutIslandsToSleep, taskData);

            if (numTasks > 0)
            {
                FmParallelForAsync("ConstraintIslandSolve", FM_TASK_AND_WRAPPED_TASK_ARGS(FmTaskFuncConstraintIslandSolveBegin), NULL, taskData, numTasks, scene->taskSystemCallbacks.SubmitAsyncTask, scene->params.numThreads);
            }
            else
            {
                FmSetNextTask(FmTaskFuncSceneConstraintSolvePutIslandsToSleep, taskData, 0, 1);
            }
        }
#else
        FmTaskDataConstraintIslandSolve taskData(scene, NULL, NULL);
        scene->taskSystemCallbacks.ParallelFor("ConstraintIslandSolve", FmTaskFuncConstraintIslandSolve, &taskData, (int)constraintsBuffer->numConstraintIslands);

        FmPutMarkedIslandsToSleep(scene, NULL);

        FM_SET_CONSTRAINT_SOLVE_TIME();
#endif
    }

#if FM_ASYNC_THREADING
    void FmTaskFuncSceneConstraintSolveEnd(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    FM_WRAPPED_TASK_FUNC(FmTaskFuncSceneConstraintSolvePutIslandsToSleep)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FmTaskDataConstraintIslandSolve* taskData = (FmTaskDataConstraintIslandSolve*)inTaskData;
        FmScene* scene = taskData->scene;

        // Reuse the taskData->progress for putting islands to sleep.  
        // Initialize to 1 so FmPutMarkedIslandsToSleep is completed, in addition to all 
        // the dispatches for islands, before follow task is triggered.
        taskData->progress.Init(1, FmTaskFuncSceneConstraintSolveEnd, taskData);

        FmPutMarkedIslandsToSleep(scene, taskData);

        taskData->progress.TaskIsFinished();
    }

    void FmTaskFuncSceneConstraintSolveEnd(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FmTaskDataConstraintIslandSolve* taskData = (FmTaskDataConstraintIslandSolve*)inTaskData;

        FM_SET_CONSTRAINT_SOLVE_TIME();

        FmSetNextTask(taskData->followTask.func, taskData->followTask.data, 0, 1);

        delete taskData;
    }
#endif

    void FmUpdateSceneAsync(FmScene* scene, float timestep);

    void FmUpdateScene(FmScene* scene, float timestep)
    {
#if FM_ASYNC_THREADING
        FmUpdateSceneAsync(scene, timestep);
#else
        FM_TRACE_START_FRAME();
        FM_TRACE_START_EVENT(SCENE_UPDATE);

        double t0;
        FM_GET_TIME(t0);

        FmUpdateUnconstrained(scene, timestep, NULL, NULL);

        FmFindContacts(scene, NULL, NULL);

        FmWakeCollidedIslandsAndFindContacts(scene, NULL, NULL);

#if FM_DEBUG_MESHES
        for (uint meshIdx = 0; meshIdx < gFEMFXNumPreConstraintSolveMeshes; meshIdx++)
        {
            FmFreeTetMeshData(&gFEMFXPreConstraintSolveMeshes[meshIdx]);
        }
        delete[] gFEMFXPreConstraintSolveMeshes;
        delete[] gFEMFXPreConstraintSolveRigidBodies;

        uint numSceneTetMeshes = FmGetNumEnabledTetMeshes(*scene);
        uint numSceneRigidBodies = FmGetNumEnabledRigidBodies(*scene);
        gFEMFXPreConstraintSolveMeshes = new FmTetMesh[numSceneTetMeshes];
        gFEMFXPreConstraintSolveRigidBodies = new FmRigidBody[numSceneRigidBodies];
        gFEMFXNumPreConstraintSolveMeshes = numSceneTetMeshes;
        gFEMFXNumPreConstraintSolveRigidBodies = numSceneRigidBodies;

        for (uint meshIdx = 0; meshIdx < numSceneTetMeshes; meshIdx++)
        {
            FmTetMesh* tetMesh = FmGetTetMeshPtrById(*scene, FmGetEnabledTetMeshId(*scene, meshIdx));
            FmInitTetMesh(&gFEMFXPreConstraintSolveMeshes[meshIdx]);
            FmAllocTetMeshData(&gFEMFXPreConstraintSolveMeshes[meshIdx], *tetMesh);
            FmCopyTetMesh(&gFEMFXPreConstraintSolveMeshes[meshIdx], *tetMesh);
        }
        for (uint rigidBodyIdx = 0; rigidBodyIdx < numSceneRigidBodies; rigidBodyIdx++)
        {
            gFEMFXPreConstraintSolveRigidBodies[rigidBodyIdx] = *FmGetRigidBodyPtrById(*scene, FmGetEnabledRigidBodyId(*scene, rigidBodyIdx));
        }
#endif

        FmFindConstraintIslands(scene, NULL);

        FmSceneConstraintSolve(scene, NULL, NULL);

        FM_TRACE_STOP_EVENT(SCENE_UPDATE);
        FM_TRACE_STOP_FRAME();
        FM_DISABLE_TRACE();

        FM_SET_TOTAL_STEP_TIME(t0);

        scene->params.simTime += timestep;
#endif
    }

#if FM_ASYNC_THREADING
    void FmTaskFuncUpdateSceneFindContacts(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    void FmTaskFuncUpdateSceneStartAux(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FmTaskDataUpdateScene* taskData = (FmTaskDataUpdateScene*)inTaskData;

        FmScene* scene = taskData->scene;
        float timestep = taskData->timestep;

        FmUpdateUnconstrained(scene, timestep, FmTaskFuncUpdateSceneFindContacts, taskData);
    }

    void FmTaskFuncUpdateSceneStart(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        FmExecuteTask(FmTaskFuncUpdateSceneStartAux, inTaskData, inTaskBeginIndex, inTaskEndIndex);
    }

    void FmTaskFuncUpdateSceneWakeAndFindContacts(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    void FmTaskFuncUpdateSceneFindContacts(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FmTaskDataUpdateScene* taskData = (FmTaskDataUpdateScene*)inTaskData;

        FmScene* scene = taskData->scene;

        // Measure time for FmUpdateUnconstrained()
        FM_SET_IMPLICIT_STEP_TIME();

        FmFindContacts(scene, FmTaskFuncUpdateSceneWakeAndFindContacts, taskData);
    }

    void FmTaskFuncUpdateSceneConstraintSolve(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    void FmTaskFuncUpdateSceneWakeAndFindContacts(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FmTaskDataUpdateScene* taskData = (FmTaskDataUpdateScene*)inTaskData;

        FmScene* scene = taskData->scene;

        // Measure time for FmFindContacts()
        FM_ADD_MESH_CONTACTS_TIME();
        FM_SET_START_TIME();

        FmWakeCollidedIslandsAndFindContacts(scene, FmTaskFuncUpdateSceneConstraintSolve, taskData);
    }

    void FmTaskFuncUpdateSceneEnd(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    void FmTaskFuncUpdateSceneConstraintSolve(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FmTaskDataUpdateScene* taskData = (FmTaskDataUpdateScene*)inTaskData;

        FmScene* scene = taskData->scene;

        FM_ADD_MESH_CONTACTS_TIME();

#if FM_DEBUG_MESHES
        for (uint meshIdx = 0; meshIdx < gFEMFXNumPreConstraintSolveMeshes; meshIdx++)
        {
            FmFreeTetMeshData(&gFEMFXPreConstraintSolveMeshes[meshIdx]);
        }
        delete[] gFEMFXPreConstraintSolveMeshes;
        gFEMFXPreConstraintSolveMeshes = NULL;
        delete[] gFEMFXPreConstraintSolveRigidBodies;
        gFEMFXPreConstraintSolveRigidBodies = NULL;

        uint numSceneTetMeshes = FmGetNumEnabledTetMeshes(*scene);
        uint numSceneRigidBodies = FmGetNumEnabledRigidBodies(*scene);
        gFEMFXPreConstraintSolveMeshes = new FmTetMesh[numSceneTetMeshes];
        gFEMFXPreConstraintSolveRigidBodies = new FmRigidBody[numSceneRigidBodies];
        gFEMFXNumPreConstraintSolveMeshes = numSceneTetMeshes;
        gFEMFXNumPreConstraintSolveRigidBodies = numSceneRigidBodies;

        for (uint meshIdx = 0; meshIdx < numSceneTetMeshes; meshIdx++)
        {
            FmTetMesh* tetMesh = FmGetTetMeshPtrById(*scene, FmGetEnabledTetMeshId(*scene, meshIdx));
            FmInitTetMesh(&gFEMFXPreConstraintSolveMeshes[meshIdx]);
            FmAllocTetMeshData(&gFEMFXPreConstraintSolveMeshes[meshIdx], *tetMesh);
            FmCopyTetMesh(&gFEMFXPreConstraintSolveMeshes[meshIdx], *tetMesh);
        }
        for (uint rigidBodyIdx = 0; rigidBodyIdx < numSceneRigidBodies; rigidBodyIdx++)
        {
            gFEMFXPreConstraintSolveRigidBodies[rigidBodyIdx] = *FmGetRigidBodyPtrById(*scene, FmGetEnabledRigidBodyId(*scene, rigidBodyIdx));
        }
#endif

        FmFindConstraintIslands(scene, NULL);

        FmSceneConstraintSolve(scene, FmTaskFuncUpdateSceneEnd, taskData);
    }

    void FmTaskFuncUpdateSceneEnd(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FmTaskDataUpdateScene* taskData = (FmTaskDataUpdateScene*)inTaskData;

        FmScene* scene = taskData->scene;

        delete taskData;

        if (scene->postUpdateSceneCallback)
        {
            scene->postUpdateSceneCallback(scene->postUpdateSceneData, 0, 1);
        }
    }

    struct FmUpdateSceneFinishedData
    {
        FmSyncEvent* taskEvent;
        FmScene* scene;

        FmUpdateSceneFinishedData(FmSyncEvent* inTaskEvent, FmScene* inScene)
        {
            taskEvent = inTaskEvent;
            scene = inScene;
        }
    };

    static void FmTaskFuncUpdateSceneFinished(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FmUpdateSceneFinishedData* taskData = (FmUpdateSceneFinishedData*)inTaskData;
        taskData->scene->taskSystemCallbacks.TriggerSyncEvent(taskData->taskEvent);
    }

    void FmUpdateSceneAsync(FmScene* scene, float timestep)
    {
        FmSyncEvent* updateSceneFinishedEvent = scene->taskSystemCallbacks.CreateSyncEvent();

        FM_TRACE_START_FRAME();
        FM_TRACE_START_EVENT(SCENE_UPDATE);

        double t0;
        FM_GET_TIME(t0);

        FmUpdateSceneFinishedData updateSceneFinishedData(updateSceneFinishedEvent, scene);
        scene->postUpdateSceneCallback = FmTaskFuncUpdateSceneFinished;
        scene->postUpdateSceneData = &updateSceneFinishedData;

        FmTaskDataUpdateScene* taskData = new FmTaskDataUpdateScene(scene, timestep);

        scene->taskSystemCallbacks.SubmitAsyncTask("FEMFXTaskFuncUpdateSceneStart", FmTaskFuncUpdateSceneStart, taskData, 0, 1);

        scene->taskSystemCallbacks.WaitForSyncEvent(updateSceneFinishedEvent);

        scene->taskSystemCallbacks.DestroySyncEvent(updateSceneFinishedEvent);

        FM_TRACE_STOP_EVENT(SCENE_UPDATE);
        FM_TRACE_STOP_FRAME();
        FM_DISABLE_TRACE();

        FM_SET_TOTAL_STEP_TIME(t0);

        scene->params.simTime += timestep;
    }
#endif
}
