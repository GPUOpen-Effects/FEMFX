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

#include "FEMFXInternal.h"
#include "FEMFXTriCCD.h"
#include "FEMFXSort.h"

namespace AMD
{
    // The self-collision method uses a Boolean difference between a mesh and itself.  The procedure is similar to the mesh intersection
    // method.
    // Reference: Smith and Dodgson, "A topologically robust algorithm for Boolean operations on polyhedral shapes using approximate arithmetic".
    //
    // NOTE: This method is expensive - possibly some shortcuts can be found, and multithreading optimizations may be an option.

    // Insert vertices of exterior face into mesh A set of vertices.
    // This is needed prior to FmSelfCollCountFullIntegrations()
    void FmSelfCollInsertFaceVertsInSet(FmCollidedObjectPair* objectPair, uint exteriorFaceIdA)
    {
        FM_ASSERT(exteriorFaceIdA != FM_INVALID_ID);

        FmTetMesh* meshA = objectPair->tetMeshA;
        FmVolumeContactWorkspace* volContactWorkSpace = objectPair->temps.volContactWorkspace;

        // Gather positions exterior faces
        FmExteriorFace& exteriorFaceA = meshA->exteriorFaces[exteriorFaceIdA];

        uint tetIdA = exteriorFaceA.tetId;

        FmFaceVertIds faceVertIdsA;
        FmGetFaceVertIds(&faceVertIdsA, exteriorFaceA.faceId, meshA->tetsVertIds[tetIdA]);

        FmVolumeContactFace* face = &volContactWorkSpace->meshAFaces[exteriorFaceIdA];
        
        for (uint cornerIdx = 0; cornerIdx < 3; cornerIdx++)
        {
            if (exteriorFaceA.OwnsVert(cornerIdx))
            {
                face->vertIds[cornerIdx] = faceVertIdsA.ids[cornerIdx];
            }
            else
            {
                face->vertIds[cornerIdx] = FM_INVALID_ID;
            }
        }

        face->intersectionVals[0] = 0;
        face->intersectionVals[1] = 0;
        face->intersectionVals[2] = 0;
        face->intersectionValsB[0] = 0;
        face->intersectionValsB[1] = 0;
        face->intersectionValsB[2] = 0;
        
        face->partialFacedVdp[0] = FmInitVector3(0.0f);
        face->partialFacedVdp[1] = FmInitVector3(0.0f);
        face->partialFacedVdp[2] = FmInitVector3(0.0f);
        face->partialFaceV = 0.0f;

        face->numFullFaceIntegrations = 0;
        face->normalScale = 0;
        face->isFullFace = true;
    }

    // Check X02 and X20 values for all vert/face pairings and update total intersection values, which will give
    // inside/outside status for these verts.
    void FmSelfCollUpdateVertIntersectionVals(FmCollidedObjectPair* objectPair, uint exteriorFaceIdA, uint exteriorFaceIdB)
    {
        FM_ASSERT(exteriorFaceIdA != FM_INVALID_ID && exteriorFaceIdB != FM_INVALID_ID);

        FmTetMesh* meshA = objectPair->tetMeshA;
        FmTetMesh* meshB = objectPair->tetMeshB;
        FmVolumeContactWorkspace* volContactWorkSpace = objectPair->temps.volContactWorkspace;
        FmVector3 volContactCenter = objectPair->volContactCenter;
        FmVector3 objectACenterPos = objectPair->volContactObjectACenterPos;
        FmVector3 objectBCenterPos = objectPair->volContactObjectBCenterPos;

        // Gather positions exterior faces
        FmExteriorFace& exteriorFaceA = meshA->exteriorFaces[exteriorFaceIdA];

        uint tetIdA = exteriorFaceA.tetId;

        FmFaceVertIds tetCornersA;
        FmGetFaceTetCorners(&tetCornersA, exteriorFaceA.faceId);

        FmFaceVertIds faceVertIdsA;
        FmGetFaceVertIds(&faceVertIdsA, exteriorFaceA.faceId, meshA->tetsVertIds[tetIdA]);

        FmVector3 triAPos[3];
        triAPos[0] = meshA->vertsPos[faceVertIdsA.ids[0]] - volContactCenter;
        triAPos[1] = meshA->vertsPos[faceVertIdsA.ids[1]] - volContactCenter;
        triAPos[2] = meshA->vertsPos[faceVertIdsA.ids[2]] - volContactCenter;

        FmExteriorFace& exteriorFaceB = meshB->exteriorFaces[exteriorFaceIdB];

        uint tetIdB = exteriorFaceB.tetId;

        FmFaceVertIds tetCornersB;
        FmGetFaceTetCorners(&tetCornersB, exteriorFaceB.faceId);

        FmFaceVertIds faceVertIdsB;
        FmGetFaceVertIds(&faceVertIdsB, exteriorFaceB.faceId, meshB->tetsVertIds[tetIdB]);

        FmVector3 triBPos[3];
        triBPos[0] = meshB->vertsPos[faceVertIdsB.ids[0]] - volContactCenter;
        triBPos[1] = meshB->vertsPos[faceVertIdsB.ids[1]] - volContactCenter;
        triBPos[2] = meshB->vertsPos[faceVertIdsB.ids[2]] - volContactCenter;

        FmVolumeContactFace* faceA = &volContactWorkSpace->meshAFaces[exteriorFaceIdA];

        for (uint cornerIdx = 0; cornerIdx < 3; cornerIdx++)
        {
            if (exteriorFaceA.OwnsVert(cornerIdx))
            {
                FM_ASSERT(faceA->vertIds[cornerIdx] != FM_INVALID_ID);

                int S02Val = FmShadowS02(triAPos[cornerIdx], triBPos[0], triBPos[1], triBPos[2]);

                // Compute second intersection value for vertex as if from second mesh of the Boolean difference
                int S20Val = FmShadowS20(triBPos[0], triBPos[1], triBPos[2], triAPos[cornerIdx]);

                if (S02Val || S20Val)
                {
                    faceA->intersectionVals[cornerIdx] += S02Val;
                    faceA->intersectionValsB[cornerIdx] -= S20Val;
                }
            }
        }
    }

    // For inside vertices, count the full integrations that must be performed on incident faces.
    // Assumes that the intersection value of the vertices has been computed, from which the "inclusion value" can be
    // determined.  Each vertex has two intersection values, each for one instance of a vertex in the difference of 
    // the mesh with itself.
    void FmSelfCollIncidentFaceIntegrations(FmCollidedObjectPair* objectPair, FmTetMesh* tetMesh)
    {
        FmVolumeContactWorkspace* volContactWorkSpace = objectPair->temps.volContactWorkspace;
        FmVector3 volContactCenter = objectPair->volContactCenter;

        FmVolumeContactFace* faces = volContactWorkSpace->meshAFaces;

        uint numFaces = tetMesh->numExteriorFaces;
        for (uint faceId = 0; faceId < numFaces; faceId++)
        {
            FmVolumeContactFace& face = faces[faceId];

            for (uint cornerIdx = 0; cornerIdx < 3; cornerIdx++)
            {
                uint vId = face.vertIds[cornerIdx];
                if (vId != FM_INVALID_ID)
                {

                    // Compute "inclusion value" from Smith-Dodgson method for mesh difference.
                    // (Actually negating this value to get the desired sign of volume)
                    int inclusionValueA = -1 + face.intersectionVals[cornerIdx];
                    int inclusionValueB = face.intersectionValsB[cornerIdx];

                    if (inclusionValueA != 0 || inclusionValueB != 0)
                    {
                        // Found inside vert.
                        // Will contribute to face integrals of incident faces.
                        // The integral is over a triangle formed from face vertex 0, 1, and the inside vertex.
                        // Thus the contribution is zero unless the inside vertex is vertex 2 of the face.

                        uint ownerExteriorFaceId = faceId;
                        FM_ASSERT(ownerExteriorFaceId != FM_INVALID_ID);

                        uint exteriorFaceId = ownerExteriorFaceId;
                        do
                        {
                            FmExteriorFace& exteriorFace = tetMesh->exteriorFaces[exteriorFaceId];
                            uint tetId = exteriorFace.tetId;

                            FmFaceVertIds tetCorners;
                            FmGetFaceTetCorners(&tetCorners, exteriorFace.faceId);

                            FmFaceVertIds faceVertIds;
                            FmGetFaceVertIds(&faceVertIds, exteriorFace.faceId, tetMesh->tetsVertIds[tetId]);

                            // Volume and gradient contributions from this vertex to the triangle are only nonzero when it is vertex 2.
                            // The reference point for the triangle is vertex0 (same convention is used for the tri-intersection contributions).
                            if (faceVertIds.ids[2] == vId)
                            {
                                FmVector3 triPos0 = tetMesh->vertsPos[faceVertIds.ids[0]] - volContactCenter;
                                FmVector3 triPos1 = tetMesh->vertsPos[faceVertIds.ids[1]] - volContactCenter;
                                FmVector3 triPos2 = tetMesh->vertsPos[faceVertIds.ids[2]] - volContactCenter;

                                if (inclusionValueA || inclusionValueB)
                                {
                                    FmVolumeContactFace* incidentFace = &faces[exteriorFaceId];

                                    if (inclusionValueA)
                                    {
                                        int numFullIntegrations = (inclusionValueA < 0) ? 1 : -1;
                                        incidentFace->numFullFaceIntegrations += numFullIntegrations;
                                        incidentFace->normalScale += numFullIntegrations;
                                    }

                                    if (inclusionValueB)
                                    {
                                        int numFullIntegrations = (inclusionValueB < 0) ? 1 : -1;
                                        incidentFace->numFullFaceIntegrations += numFullIntegrations;
                                        incidentFace->normalScale -= numFullIntegrations;
                                    }
                                }
                            }

                            // Step to next incident face
                            uint edgeId = 0;
                            edgeId = faceVertIds.ids[1] == vId ? 1 : edgeId;
                            edgeId = faceVertIds.ids[2] == vId ? 2 : edgeId;

                            exteriorFaceId = exteriorFace.edgeIncidentFaceIds[edgeId];

                        } while (exteriorFaceId != ownerExteriorFaceId);
                    }
                }
            }
        }
    }

    // Apply volume and gradient contributions from integrating over intersecting triangles.
    // The contributions are split into two parts, partial area integrations, and full area integrations.
    // The partial integrations are computed in this function and summed with the face data.
    // Full area integrations are just counted to be computed later.
    // This is important for ensuring that triangles that have net zero integrations will end up with zero volume and 
    // gradient values, and so can be excluded from the Boolean difference result.
    void FmSelfCollApplyTriIntersectionVolumeAndGradientContributions(
        FmCollidedObjectPair* objectPair,
        FmMeshCollisionTriPair& triPair,
        FmVolumeContactWorkspace* volContactWorkspace)
    {
        FmVector3 objectACenterPos = objectPair->volContactObjectACenterPos;
        FmVector3 objectBCenterPos = objectPair->volContactObjectBCenterPos;

        bool fullFaceA;
        bool fullFaceB;
        int numFullFaceA;
        int numFullFaceB;
        FmVector3 partial_dVdp_contributionA[3];
        FmVector3 partial_dVdp_contributionB[3];
        float partial_V_contributionA, partial_V_contributionB;

        // For each triangle, integrate over area between triangle vertex 0 and intersection segment
        FmComputePartialTriIntersectionVolumeAndGradientContribution(
            &fullFaceA,
            &fullFaceB,
            &numFullFaceA,
            &numFullFaceB,
            partial_dVdp_contributionA,
            partial_dVdp_contributionB,
            &partial_V_contributionA,
            &partial_V_contributionB,
            triPair.triIntersectionPoints,
            triPair.triA.pos0, triPair.triA.pos1, triPair.triA.pos2,
            triPair.triB.pos0, triPair.triB.pos1, triPair.triB.pos2);

        // For each triangle, integrate over area between triangle vertex 0, vertex 1, and segment endpoint on triangle edge 1
        const uint edgeAIdx = 1;
        int inclusionValueA = triPair.triIntersectionPoints.XVals[edgeAIdx];
        if (inclusionValueA)
        {
            bool edgeA_fullFaceIntegrationA;
            int edgeA_numFullFaceIntegrationsA;
            FmVector3 edgeA_partial_dVdp_contribution[3];
            float edgeA_partial_V_contribution = FmComputePartialEdgeFaceIntersectionVolumeAndGradientContribution(
                &edgeA_fullFaceIntegrationA,
                &edgeA_numFullFaceIntegrationsA,
                edgeA_partial_dVdp_contribution,
                triPair.triA.pos0, triPair.triA.pos1, triPair.triA.pos2, triPair.triIntersectionPoints.points[edgeAIdx], inclusionValueA < 0);

            partial_dVdp_contributionA[0] += edgeA_partial_dVdp_contribution[0];
            partial_dVdp_contributionA[1] += edgeA_partial_dVdp_contribution[1];
            partial_dVdp_contributionA[2] += edgeA_partial_dVdp_contribution[2];

            partial_V_contributionA += edgeA_partial_V_contribution;

            if (!edgeA_fullFaceIntegrationA)
            {
                fullFaceA = false;
            }
            numFullFaceA += edgeA_numFullFaceIntegrationsA;
        }

        const uint edgeBIdx = 4;
        int inclusionValueB = triPair.triIntersectionPoints.XVals[edgeBIdx];
        if (inclusionValueB)
        {
            bool edgeB_fullFaceIntegrationB;
            int edgeB_numFullFaceIntegrationsB;
            FmVector3 edgeB_partial_dVdp_contribution[3];
            float edgeB_partial_V_contribution = FmComputePartialEdgeFaceIntersectionVolumeAndGradientContribution(
                &edgeB_fullFaceIntegrationB,
                &edgeB_numFullFaceIntegrationsB,
                edgeB_partial_dVdp_contribution,
                triPair.triB.pos0, triPair.triB.pos1, triPair.triB.pos2, triPair.triIntersectionPoints.points[edgeBIdx], inclusionValueB < 0);

            partial_dVdp_contributionB[0] += edgeB_partial_dVdp_contribution[0];
            partial_dVdp_contributionB[1] += edgeB_partial_dVdp_contribution[1];
            partial_dVdp_contributionB[2] += edgeB_partial_dVdp_contribution[2];

            partial_V_contributionB += edgeB_partial_V_contribution;

            if (!edgeB_fullFaceIntegrationB)
            {
                fullFaceB = false;
            }
            numFullFaceB += edgeB_numFullFaceIntegrationsB;
        }

        FmVolumeContactFace* facesA = objectPair->temps.volContactWorkspace->meshAFaces;

        if (!fullFaceA || numFullFaceA != 0)
        {
            FmVolumeContactFace* faceA = &facesA[triPair.exteriorFaceIdA];

            if (!fullFaceA)
            {
                faceA->isFullFace = false;
            }
            faceA->numFullFaceIntegrations += numFullFaceA;

            faceA->partialFacedVdp[0] -= partial_dVdp_contributionA[0];
            faceA->partialFacedVdp[1] -= partial_dVdp_contributionA[1];
            faceA->partialFacedVdp[2] -= partial_dVdp_contributionA[2];
            faceA->partialFaceV -= partial_V_contributionA;

            faceA->normalScale += numFullFaceA;
        }

        FmVolumeContactFace* facesB = facesA;

        if (!fullFaceB || numFullFaceB != 0)
        {
            FmVolumeContactFace* faceB = &facesB[triPair.exteriorFaceIdB];

            if (!fullFaceB)
            {
                faceB->isFullFace = false;
            }

            faceB->numFullFaceIntegrations += numFullFaceB;

            faceB->partialFacedVdp[0] -= partial_dVdp_contributionB[0];
            faceB->partialFacedVdp[1] -= partial_dVdp_contributionB[1];
            faceB->partialFacedVdp[2] -= partial_dVdp_contributionB[2];
            faceB->partialFaceV -= partial_V_contributionB;

            faceB->normalScale -= numFullFaceB;
        }

#if FM_SURFACE_INTERSECTION_CONTACTS
        if ((!fullFaceA || !fullFaceB)
            && volContactWorkspace->numIntersectingFacePairs < volContactWorkspace->maxIntersectingFacePairs
            && !triPair.isTetInverted  // disabling these contacts if tet is inverted since they may interfere with contact resolution
            )
        {
            FmVector3 centroid = FmInitVector3(0.0f);
            for (uint i = 0; i < triPair.triIntersectionPoints.numSegments; i++)
            {
                centroid +=
                    triPair.triIntersectionPoints.points[triPair.triIntersectionPoints.startPoints[i]] +
                    triPair.triIntersectionPoints.points[triPair.triIntersectionPoints.endPoints[i]];
            }
            centroid /= (float)(triPair.triIntersectionPoints.numSegments * 2);

            FmIntersectingFacePair& facePair = volContactWorkspace->intersectingFacePairs[volContactWorkspace->numIntersectingFacePairs];
            facePair.exteriorFaceIdA = triPair.exteriorFaceIdA;
            facePair.exteriorFaceIdB = triPair.exteriorFaceIdB;
            facePair.pos = centroid;

            volContactWorkspace->numIntersectingFacePairs++;
        }
#endif
    }

    // Apply volume and gradient contributions from faces to vertices
    void FmSelfCollApplyFaceVolumeAndGradientContributionsToVertices(FmCollidedObjectPair* objectPair, const FmTetMesh& tetMesh)
    {
        FmVolumeContactWorkspace* volContactWorkspace = objectPair->temps.volContactWorkspace;

        FmVolumeContactVertSetExpandable& vertSet = volContactWorkspace->meshAVertSets;
        const FmVolumeContactFace* faces = volContactWorkspace->meshAFaces;

        FmVector3 objectACenterPos = objectPair->volContactObjectACenterPos;

        uint numElements = tetMesh.numExteriorFaces;
        for (uint elemIdx = 0; elemIdx < numElements; elemIdx++)
        {
            const FmVolumeContactFace& element = faces[elemIdx];

            if (!element.isFullFace || element.numFullFaceIntegrations != 0 || element.normalScale != 0)
            {
                FmExteriorFace& exteriorFace = tetMesh.exteriorFaces[elemIdx];
                uint tetId = exteriorFace.tetId;

                FmFaceVertIds tetCorners;
                FmGetFaceTetCorners(&tetCorners, exteriorFace.faceId);

                FmFaceVertIds faceVertIds;
                FmGetFaceVertIds(&faceVertIds, exteriorFace.faceId, tetMesh.tetsVertIds[tetId]);

                FmVolumeContactVertSetElement* vert0 = FmInsertElement(&vertSet, faceVertIds.ids[0]);
                FmVolumeContactVertSetElement* vert1 = FmInsertElement(&vertSet, faceVertIds.ids[1]);
                FmVolumeContactVertSetElement* vert2 = FmInsertElement(&vertSet, faceVertIds.ids[2]);

                FmVector3 triPos0 = tetMesh.vertsPos[faceVertIds.ids[0]] - objectPair->volContactCenter;
                FmVector3 triPos1 = tetMesh.vertsPos[faceVertIds.ids[1]] - objectPair->volContactCenter;
                FmVector3 triPos2 = tetMesh.vertsPos[faceVertIds.ids[2]] - objectPair->volContactCenter;

                if (element.vertIds[0] != FM_INVALID_ID)
                {
                    vert0->centerToVert = triPos0 - objectACenterPos;
                }
                if (element.vertIds[1] != FM_INVALID_ID)
                {
                    vert1->centerToVert = triPos1 - objectACenterPos;
                }
                if (element.vertIds[2] != FM_INVALID_ID)
                {
                    vert2->centerToVert = triPos2 - objectACenterPos;
                }

                float elementV = 0.0f;

                if (!element.isFullFace)
                {
                    vert0->dVdp += element.partialFacedVdp[0];
                    vert1->dVdp += element.partialFacedVdp[1];
                    vert2->dVdp += element.partialFacedVdp[2];
                    elementV += element.partialFaceV;
                }

                if (element.numFullFaceIntegrations != 0 || element.normalScale != 0)
                {
                    FmVector3 dVdp_contribution[3];
                    float V_contribution = FmComputeFaceIntersectionVolumeAndGradientContribution(
                        dVdp_contribution,
                        triPos0, triPos1, triPos2, true);

                    if (element.numFullFaceIntegrations != 0)
                    {
                        float scale = (float)element.numFullFaceIntegrations;

                        vert0->dVdp -= dVdp_contribution[0] * scale;
                        vert1->dVdp -= dVdp_contribution[1] * scale;
                        vert2->dVdp -= dVdp_contribution[2] * scale;
                        elementV -= V_contribution * scale;
                    }

                    objectPair->volContactNormal -= (dVdp_contribution[0] + dVdp_contribution[1] + dVdp_contribution[2]) * (float)element.normalScale;
                }

                vert2->V += elementV;

                objectPair->volContactV += elementV;
            }
        }
    }

    void FmSelfCollideRecursive(FmCollidedObjectPair* objectPair, int indexA, int indexB)
    {
        FmBvh* hierarchyA = objectPair->objectAHierarchy;
        FmBvh* hierarchyB = objectPair->objectBHierarchy;
        float timestep = objectPair->timestep;

        FmBvhNode* nodeA = &hierarchyA->nodes[indexA];
        FmBvhNode* nodeB = &hierarchyB->nodes[indexB];
        FmAabb* boxA = &hierarchyA->nodes[indexA].box;
        FmAabb* boxB = &hierarchyB->nodes[indexB].box;
        bool nodeAIsLeaf = nodeA->left == nodeA->right;
        bool nodeBIsLeaf = nodeB->left == nodeB->right;
        float impactTime;

        if ((indexA == indexB) || FmAabbCcd(impactTime, *boxA, *boxB, timestep))
        {
            if (nodeAIsLeaf && nodeBIsLeaf)
            {
#if FM_SOA_TRI_INTERSECTION
                {
                    FmMeshCollisionTriPair& pair = objectPair->temps.meshCollisionTriPairs[objectPair->temps.numMeshCollisionTriPairs];
                    pair.exteriorFaceIdA = (uint)nodeA->left;
                    pair.exteriorFaceIdB = (uint)nodeB->left;
                    objectPair->temps.numMeshCollisionTriPairs++;
                    if (objectPair->temps.numMeshCollisionTriPairs >= FM_MAX_MESH_COLLISION_TRI_PAIR)
                    {
                        FmGenerateContactsFromMeshCollisionTriPairs(objectPair);
                        objectPair->temps.numMeshCollisionTriPairs = 0;
                    }
                }

                if (indexA != indexB)
                {
                    FmMeshCollisionTriPair& pair = objectPair->temps.meshCollisionTriPairs[objectPair->temps.numMeshCollisionTriPairs];
                    pair.exteriorFaceIdA = (uint)nodeB->left;
                    pair.exteriorFaceIdB = (uint)nodeA->left;
                    objectPair->temps.numMeshCollisionTriPairs++;
                    if (objectPair->temps.numMeshCollisionTriPairs >= FM_MAX_MESH_COLLISION_TRI_PAIR)
                    {
                        FmGenerateContactsFromMeshCollisionTriPairs(objectPair);
                        objectPair->temps.numMeshCollisionTriPairs = 0;
                    }
                }
#else
                FmGenerateContacts(objectPair, nodeA->left, nodeB->left);

                if (indexA != indexB)
                {
                    FmGenerateContacts(objectPair, nodeB->left, nodeA->left);
                }
#endif
            }
            else if (nodeAIsLeaf)
            {
                FmSelfCollideRecursive(objectPair, indexA, nodeB->left);
                FmSelfCollideRecursive(objectPair, indexA, nodeB->right);
            }
            else if (nodeBIsLeaf)
            {
                FmSelfCollideRecursive(objectPair, nodeA->left, indexB);
                FmSelfCollideRecursive(objectPair, nodeA->right, indexB);
            }
            else
            {
                if (indexA == indexB)
                {
                    // If the same tree node, test LL, LR, and RR pairs (not RL) to find unique pairs
                    FmSelfCollideRecursive(objectPair, nodeA->left, nodeB->left);
                    FmSelfCollideRecursive(objectPair, nodeA->left, nodeB->right);
                    FmSelfCollideRecursive(objectPair, nodeA->right, nodeB->right);
                }
                else
                {
                    float sizeA = lengthSqr(boxA->pmax - boxA->pmin);
                    float sizeB = lengthSqr(boxB->pmax - boxB->pmin);

                    // Split the larger node
                    if (sizeA > sizeB)
                    {
                        FmSelfCollideRecursive(objectPair, nodeA->left, indexB);
                        FmSelfCollideRecursive(objectPair, nodeA->right, indexB);
                    }
                    else
                    {
                        FmSelfCollideRecursive(objectPair, indexA, nodeB->left);
                        FmSelfCollideRecursive(objectPair, indexA, nodeB->right);
                    }
                }
            }
        }
    }

    void FmSelfCollideHierarchies(FmCollidedObjectPair* meshPair)
    {
        FmSelfCollideRecursive(meshPair, 0, 0);

#if FM_SOA_TRI_INTERSECTION
        FmGenerateContactsFromMeshCollisionTriPairs(meshPair);
        meshPair->temps.numMeshCollisionTriPairs = 0;
#endif
    }

    static FM_FORCE_INLINE bool FmAabbOverlapXY(const FmAabb& aabb0, const FmAabb& aabb1)
    {
        return
            aabb0.pmin.x <= aabb1.pmax.x &&
            aabb0.pmin.y <= aabb1.pmax.y &&
            aabb1.pmin.x <= aabb0.pmax.x &&
            aabb1.pmin.y <= aabb0.pmax.y;
    }

    void FmSelfCollFindInsideVertsRecursive(FmCollidedObjectPair* objectPair, int indexA, int indexB)
    {
        FmBvh* hierarchyA = objectPair->objectAHierarchy;
        FmBvh* hierarchyB = objectPair->objectBHierarchy;

        FmBvhNode* nodeA = &hierarchyA->nodes[indexA];
        FmBvhNode* nodeB = &hierarchyB->nodes[indexB];
        FmAabb* boxA = &hierarchyA->nodes[indexA].box;
        FmAabb* boxB = &hierarchyB->nodes[indexB].box;
        bool nodeAIsLeaf = nodeA->left == nodeA->right;
        bool nodeBIsLeaf = nodeB->left == nodeB->right;

        if ((indexA == indexB) || FmAabbOverlapXY(*boxA, *boxB))
        {
            if (nodeAIsLeaf && nodeBIsLeaf)
            {
                FmSelfCollUpdateVertIntersectionVals(objectPair, (uint)nodeA->left, (uint)nodeB->left);

                if (indexA != indexB)
                {
                    FmSelfCollUpdateVertIntersectionVals(objectPair, (uint)nodeB->left, (uint)nodeA->left);
                }
            }
            else if (nodeAIsLeaf)
            {
                FmSelfCollFindInsideVertsRecursive(objectPair, indexA, nodeB->left);
                FmSelfCollFindInsideVertsRecursive(objectPair, indexA, nodeB->right);
            }
            else if (nodeBIsLeaf)
            {
                FmSelfCollFindInsideVertsRecursive(objectPair, nodeA->left, indexB);
                FmSelfCollFindInsideVertsRecursive(objectPair, nodeA->right, indexB);
            }
            else
            {
                if (indexA == indexB)
                {
                    // If the same tree node, test LL, LR, and RR pairs (not RL) to find unique pairs
                    FmSelfCollFindInsideVertsRecursive(objectPair, nodeA->left, nodeB->left);
                    FmSelfCollFindInsideVertsRecursive(objectPair, nodeA->left, nodeB->right);
                    FmSelfCollFindInsideVertsRecursive(objectPair, nodeA->right, nodeB->right);
                }
                else
                {
                    float sizeA = lengthSqr(boxA->pmax - boxA->pmin);
                    float sizeB = lengthSqr(boxB->pmax - boxB->pmin);

                    // Split the larger node
                    if (sizeA > sizeB)
                    {
                        FmSelfCollFindInsideVertsRecursive(objectPair, nodeA->left, indexB);
                        FmSelfCollFindInsideVertsRecursive(objectPair, nodeA->right, indexB);
                    }
                    else
                    {
                        FmSelfCollFindInsideVertsRecursive(objectPair, indexA, nodeB->left);
                        FmSelfCollFindInsideVertsRecursive(objectPair, indexA, nodeB->right);
                    }
                }
            }
        }
    }

    void FmSelfCollFindInsideVerts(FmCollidedObjectPair* meshPair)
    {
        FmSelfCollFindInsideVertsRecursive(meshPair, 0, 0);
    }

    uint FmFindSelfContacts(FmCollidedObjectPair* objectPair)
    {
        FM_ASSERT(objectPair->tetMeshA == objectPair->tetMeshB);

        FmTetMesh& tetMesh = *objectPair->tetMeshA;

        uint numFaces = tetMesh.numExteriorFaces;
        for (uint fId = 0; fId < numFaces; fId++)
        {
            FmSelfCollInsertFaceVertsInSet(objectPair, fId);
        }

        FmSelfCollFindInsideVerts(objectPair);

        // For inside vertices, apply volume and gradient contributions to any incident faces
        FmSelfCollIncidentFaceIntegrations(objectPair, &tetMesh);

        FmSelfCollideHierarchies(objectPair);

        FmSelfCollApplyFaceVolumeAndGradientContributionsToVertices(objectPair, tetMesh);

        uint numVolumeContacts = FmFinalizeVolumeContact(objectPair);

        // Copy any temp contacts to global list
        if (objectPair->temps.numDistanceContacts > 0)
        {
            FmCopyTempDistanceContacts(objectPair);
        }

        return objectPair->numDistanceContacts + numVolumeContacts;
    }
}

