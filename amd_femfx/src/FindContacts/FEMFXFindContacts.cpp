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
// Generation of distance and volume contacts between FEM objects
//---------------------------------------------------------------------------------------

#include "FEMFXFindContacts.h"
#include "FEMFXBvhCollision.h"
#include "FEMFXCollisionPairData.h"
#include "FEMFXScene.h"
#include "FEMFXBroadPhase.h"
#include "FEMFXParallelFor.h"
#include "FEMFXThreadTempMemory.h"
#include "FEMFXSleeping.h"

#if FM_SOA_TRI_INTERSECTION
#include "FEMFXSoATriIntersection.h"
#endif
#if FM_SOA_TRI_CCD
#include "FEMFXSoaTriCcd.h"
#endif

using namespace AMD;

// Use an early rejection for triangle overlap for a performance benefit. 
// NOTE: For volume contact V and dVdp values, it's important to not reject an overlap that would be 
// found by FmComputeTriIntersection().  Need to verify this can't occur.
#define FM_TRI_REJECTION_TEST 1

namespace AMD
{
    // Check X02 and X20 values for all vert/face pairings and update total intersection values, which will give
    // inside/outside status for these verts.
    void FmUpdateVertIntersectionVals(FmCollidedObjectPair* objectPair, uint exteriorFaceIdA, uint exteriorFaceIdB,
        bool includeFaceA, bool includeFaceB)
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

        FmVector3 intersectionPoint;
        for (uint cornerIdx = 0; cornerIdx < 3; cornerIdx++)
        {
            if (includeFaceA && exteriorFaceA.OwnsVert(cornerIdx))
            {
                int S02Val = FmShadowS02(triAPos[cornerIdx], triBPos[0], triBPos[1], triBPos[2]);

                if (S02Val)
                {
                    FmVolumeContactVertSetElement* element = FmInsertElement(&volContactWorkSpace->meshAVertSets, faceVertIdsA.ids[cornerIdx]);
                    element->centerToVert = triAPos[cornerIdx] - objectACenterPos;
                    element->intersectionValue += S02Val;
                    element->ownerExteriorFaceId = exteriorFaceIdA;
                }
            }

            if (includeFaceB && exteriorFaceB.OwnsVert(cornerIdx))
            {
                int S20Val = FmShadowS20(triAPos[0], triAPos[1], triAPos[2], triBPos[cornerIdx]);

                if (S20Val)
                {
                    FmVolumeContactVertSetElement* element = FmInsertElement(&volContactWorkSpace->meshBVertSets, faceVertIdsB.ids[cornerIdx]);
                    element->centerToVert = triBPos[cornerIdx] - objectBCenterPos;
                    element->intersectionValue -= S20Val;
                    element->ownerExteriorFaceId = exteriorFaceIdB;
                }
            }
        }
    }

    // For inside vertices, apply volume and gradient contributions to any incident faces.
    // Assumes that the inside vertices in the vertex set have a nonzero intersectionValue.
    void FmApplyInsideVertVolumeAndGradientContributions(
        FmCollidedObjectPair* objectPair, FmTetMesh* tetMesh, FmVolumeContactVertSetExpandable& volContactVerts, float normalScale)
    {
        FmVector3 volContactCenter = objectPair->volContactCenter;

        uint numElements = volContactVerts.numElementsUsed;
        for (uint elemIdx = 0; elemIdx < numElements; elemIdx++)
        {
            FmVolumeContactVertSetElement& element = volContactVerts.elements[elemIdx];
            if (element.key != FM_INVALID_ID && element.intersectionValue)
            {
                // Found inside vert.
                // Will contribute to face integrals of incident faces.
                // The integral is over a triangle formed from face vertex 0, 1, and the inside vertex.
                // Thus the contribution is zero unless the inside vertex is vertex 2 of the face.

                uint vId = element.key;
                uint ownerExteriorFaceId = element.ownerExteriorFaceId;
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

                        FmVector3 dVdp_contribution[3];
                        float V_contribution = FmComputeFaceIntersectionVolumeAndGradientContribution(
                            dVdp_contribution,
                            triPos0, triPos1, triPos2, element.intersectionValue < 0);

                        FmVolumeContactVertSetElement* vert0 = FmInsertElement(&volContactVerts, faceVertIds.ids[0]);
                        vert0->dVdp -= dVdp_contribution[0];
                        FmVolumeContactVertSetElement* vert1 = FmInsertElement(&volContactVerts, faceVertIds.ids[1]);
                        vert1->dVdp -= dVdp_contribution[1];
                        FmVolumeContactVertSetElement* vert2 = FmInsertElement(&volContactVerts, faceVertIds.ids[2]);
                        vert2->dVdp -= dVdp_contribution[2];

                        vert2->V -= V_contribution;

                        objectPair->volContactNormal -= (dVdp_contribution[0] + dVdp_contribution[1] + dVdp_contribution[2]) * normalScale;
                        objectPair->volContactV -= V_contribution;
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

    // Apply volume and gradient contributions from integrating over intersecting triangles.
    void FmApplyTriIntersectionVolumeAndGradientContributions(
        FmCollidedObjectPair* objectPair,
        FmMeshCollisionTriPair& triPair,
        FmVolumeContactWorkspace* volContactWorkspace)
    {
        FmVector3 objectACenterPos = objectPair->volContactObjectACenterPos;
        FmVector3 objectBCenterPos = objectPair->volContactObjectBCenterPos;

#if FM_SURFACE_INTERSECTION_CONTACTS
        if (volContactWorkspace->numIntersectingFacePairs < volContactWorkspace->maxIntersectingFacePairs
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

        // For each triangle, integrate over area between triangle vertex 0 and intersection segment
        FmVector3 dVdp_contributionA[3];
        FmVector3 dVdp_contributionB[3];
        float V_contributionA, V_contributionB;
        FmComputeTriIntersectionVolumeAndGradientContribution(
            dVdp_contributionA, dVdp_contributionB,
            &V_contributionA, &V_contributionB,
            triPair.triIntersectionPoints,
            triPair.triA.pos0, triPair.triA.pos1, triPair.triA.pos2,
            triPair.triB.pos0, triPair.triB.pos1, triPair.triB.pos2);

        // For each triangle, integrate over area between triangle vertex 0, vertex 1, and segment endpoint on triangle edge 1
        const uint edgeAIdx = 1;
        if (triPair.triIntersectionPoints.XVals[edgeAIdx])
        {
            FmVector3 edgeA_dVdp_contribution[3];
            float edgeA_V_contribution = FmComputeEdgeFaceIntersectionVolumeAndGradientContribution(
                edgeA_dVdp_contribution,
                triPair.triA.pos0, triPair.triA.pos1, triPair.triA.pos2, triPair.triIntersectionPoints.points[edgeAIdx], triPair.triIntersectionPoints.XVals[edgeAIdx] < 0);

            dVdp_contributionA[0] += edgeA_dVdp_contribution[0];
            dVdp_contributionA[1] += edgeA_dVdp_contribution[1];
            dVdp_contributionA[2] += edgeA_dVdp_contribution[2];
            V_contributionA += edgeA_V_contribution;
        }

        const uint edgeBIdx = 4;
        if (triPair.triIntersectionPoints.XVals[edgeBIdx])
        {
            FmVector3 edgeB_dVdp_contribution[3];
            float edgeB_V_contribution = FmComputeEdgeFaceIntersectionVolumeAndGradientContribution(
                edgeB_dVdp_contribution,
                triPair.triB.pos0, triPair.triB.pos1, triPair.triB.pos2, triPair.triIntersectionPoints.points[edgeBIdx], triPair.triIntersectionPoints.XVals[edgeBIdx] < 0);

            dVdp_contributionB[0] += edgeB_dVdp_contribution[0];
            dVdp_contributionB[1] += edgeB_dVdp_contribution[1];
            dVdp_contributionB[2] += edgeB_dVdp_contribution[2];
            V_contributionB += edgeB_V_contribution;
        }

        // Apply contributions to volume and volume gradient at each vertex
        FmVolumeContactVertSetElement* element0, *element1, *element2;
        element0 = FmInsertElement(&volContactWorkspace->meshAVertSets, triPair.faceVertIdsA.ids[0]);
        element0->dVdp -= dVdp_contributionA[0];
        element0->centerToVert = triPair.triA.pos0 - objectACenterPos;
        element1 = FmInsertElement(&volContactWorkspace->meshAVertSets, triPair.faceVertIdsA.ids[1]);
        element1->dVdp -= dVdp_contributionA[1];
        element1->centerToVert = triPair.triA.pos1 - objectACenterPos;
        element2 = FmInsertElement(&volContactWorkspace->meshAVertSets, triPair.faceVertIdsA.ids[2]);
        element2->dVdp -= dVdp_contributionA[2];
        element2->centerToVert = triPair.triA.pos2 - objectACenterPos;
        element2->V -= V_contributionA; // Apply volume contribution just to vertex 2

        element0 = FmInsertElement(&volContactWorkspace->meshBVertSets, triPair.faceVertIdsB.ids[0]);
        element0->dVdp -= dVdp_contributionB[0];
        element0->centerToVert = triPair.triB.pos0 - objectBCenterPos;
        element1 = FmInsertElement(&volContactWorkspace->meshBVertSets, triPair.faceVertIdsB.ids[1]);
        element1->dVdp -= dVdp_contributionB[1];
        element1->centerToVert = triPair.triB.pos1 - objectBCenterPos;
        element2 = FmInsertElement(&volContactWorkspace->meshBVertSets, triPair.faceVertIdsB.ids[2]);
        element2->dVdp -= dVdp_contributionB[2];
        element2->centerToVert = triPair.triB.pos2 - objectBCenterPos;
        element2->V -= V_contributionB; // Apply volume contribution just to vertex 2

        // Apply contribution to total volume, and contact normal
        objectPair->volContactNormal -= (dVdp_contributionA[0] + dVdp_contributionA[1] + dVdp_contributionA[2]);
        objectPair->volContactNormal += (dVdp_contributionB[0] + dVdp_contributionB[1] + dVdp_contributionB[2]);
        objectPair->volContactV -= (V_contributionA + V_contributionB);
    }

#if FM_CONTACT_REDUCTION
    FmVertKeptContact* FmAddToKeptContacts(
        FmCollidedObjectPairTemps* temps, FmVertKeptContactSet* vertsContacts, FmDistanceContactPairInfo* contactsArray,
        uint vertId, float impactTime, float distanceSqr)
    {
        uint tempContactsIdx = temps->numDistanceContacts;
        FmDistanceContactPairInfo* tempContactsArray = temps->distanceContactPairInfoBuffer;

        FmVertKeptContactSet& vertContacts = vertsContacts[vertId];

        // Remove contacts with FM_INVALID_ID idx, which would occur if ran out of distance contact memory
        uint dstIdx = 0;
        uint numContactsTimeWeighted = vertContacts.numContactsTimeWeighted;
        for (uint i = 0; i < numContactsTimeWeighted; i++)
        {
            if (vertContacts.contactsTimeWeighted[i].idx != FM_INVALID_ID)
            {
                if (dstIdx != i)
                {
                    vertContacts.contactsTimeWeighted[dstIdx] = vertContacts.contactsTimeWeighted[i];
                }
                dstIdx++;
            }
        }
        vertContacts.numContactsTimeWeighted = dstIdx;

        dstIdx = 0;
        uint numContactsDistWeighted = vertContacts.numContactsDistWeighted;
        for (uint i = 0; i < numContactsDistWeighted; i++)
        {
            if (vertContacts.contactsDistanceWeighted[i].idx != FM_INVALID_ID)
            {
                if (dstIdx != i)
                {
                    vertContacts.contactsDistanceWeighted[dstIdx] = vertContacts.contactsDistanceWeighted[i];
                }
                dstIdx++;
            }
        }
        vertContacts.numContactsDistWeighted = dstIdx;

        FmVertKeptContact contact;
        contact.idx = tempContactsIdx;  // contact first added to temp buffer
        contact.distanceSqr = distanceSqr;
        contact.impactTime = impactTime;
        contact.isTempContact = true;

        // Add contact if empty space in either contact set
        numContactsTimeWeighted = vertContacts.numContactsTimeWeighted;
        if (numContactsTimeWeighted < FM_MAX_TIME_WEIGHTED_CONTACTS_PER_VERTEX)
        {
            FmVertKeptContact& destContact = vertContacts.contactsTimeWeighted[numContactsTimeWeighted];
            destContact = contact;
            vertContacts.numContactsTimeWeighted++;
            return &destContact;
        }

        numContactsDistWeighted = vertContacts.numContactsDistWeighted;
        if (numContactsDistWeighted < FM_MAX_DIST_WEIGHTED_CONTACTS_PER_VERTEX)
        {
            FmVertKeptContact& destContact = vertContacts.contactsDistanceWeighted[numContactsDistWeighted];
            destContact = contact;
            vertContacts.numContactsDistWeighted++;
            return &destContact;
        }

        // Check if a contact should be replaced.

        // Find contact with max impact time
        uint maxImpactTimeIdx = 0;
        float maxImpactTime = vertContacts.contactsTimeWeighted[0].impactTime;
        for (uint i = 1; i < numContactsTimeWeighted; i++)
        {
            float contactImpactTime = vertContacts.contactsTimeWeighted[i].impactTime;
            if (contactImpactTime > maxImpactTime)
            {
                maxImpactTimeIdx = i;
                maxImpactTime = contactImpactTime;
            }
        }

        // Find contact with max distance
        uint maxDistanceSqrIdx = 0;
        float maxDistanceSqr = vertContacts.contactsDistanceWeighted[0].distanceSqr;
        for (uint i = 1; i < numContactsDistWeighted; i++)
        {
            float contactDistanceSqr = vertContacts.contactsDistanceWeighted[i].distanceSqr;
            if (contactDistanceSqr > maxDistanceSqr)
            {
                maxDistanceSqrIdx = i;
                maxDistanceSqr = contactDistanceSqr;
            }
        }

        // If contact impact time less than max, evict max
        if (contact.impactTime < maxImpactTime)
        {
            FmVertKeptContact& destContact = vertContacts.contactsTimeWeighted[maxImpactTimeIdx];

            // First check if contact should be moved to other set
            float destDistance = destContact.distanceSqr;
            if (destDistance < maxDistanceSqr)
            {
                // Move max impact time contact to distance weighted contacts

                // Decrement ref count for replaced contact
                FmVertKeptContact& moveDestContact = vertContacts.contactsDistanceWeighted[maxDistanceSqrIdx];
                uint contactIdx = moveDestContact.idx;
                FM_ASSERT(contactIdx != FM_INVALID_ID);

                if (moveDestContact.isTempContact)
                {
                    FM_ASSERT(tempContactsArray[contactIdx].refCount > 0);
                    tempContactsArray[contactIdx].refCount--;
                    if (tempContactsArray[contactIdx].refCount == 0)
                    {
                        temps->numDistanceContactsNonzeroRefCount--;
                    }
                }
                else
                {
                    FM_ASSERT(contactsArray[contactIdx].refCount > 0);
                    contactsArray[contactIdx].refCount--;
                }

                moveDestContact = vertContacts.contactsTimeWeighted[maxImpactTimeIdx];
            }
            else
            {
                // Decrement ref count for replaced contact
                uint contactIdx = destContact.idx;
                FM_ASSERT(contactIdx != FM_INVALID_ID);

                if (destContact.isTempContact)
                {
                    FM_ASSERT(tempContactsArray[contactIdx].refCount > 0);
                    tempContactsArray[contactIdx].refCount--;
                    if (tempContactsArray[contactIdx].refCount == 0)
                    {
                        temps->numDistanceContactsNonzeroRefCount--;
                    }
                }
                else
                {
                    FM_ASSERT(contactsArray[contactIdx].refCount > 0);
                    contactsArray[contactIdx].refCount--;
                }
            }

            // Replace contact
            destContact = contact;
            return &destContact;
        }

        if (contact.distanceSqr < maxDistanceSqr)
        {
            FmVertKeptContact& destContact = vertContacts.contactsDistanceWeighted[maxDistanceSqrIdx];

            // Decrement ref count for replaced contact
            uint contactIdx = destContact.idx;
            FM_ASSERT(contactIdx != FM_INVALID_ID);

            if (destContact.isTempContact)
            {
                FM_ASSERT(tempContactsArray[contactIdx].refCount > 0);
                tempContactsArray[contactIdx].refCount--;
                if (tempContactsArray[contactIdx].refCount == 0)
                {
                    temps->numDistanceContactsNonzeroRefCount--;
                }
            }
            else
            {
                FM_ASSERT(contactsArray[contactIdx].refCount > 0);
                contactsArray[contactIdx].refCount--;
            }

            // Replace contact
            destContact = contact;
            return &destContact;
        }

        return NULL;
    }

    void FmReplaceKeptContactTempIndex(FmVertKeptContactSet* vertsContacts, uint vId, uint tempIdx, uint contactIdx)
    {
        FmVertKeptContactSet& vertContacts = vertsContacts[vId];

        uint numContactsTimeWeighted = vertContacts.numContactsTimeWeighted;
        for (uint i = 0; i < numContactsTimeWeighted; i++)
        {
            FmVertKeptContact& keptContact = vertContacts.contactsTimeWeighted[i];

            if (keptContact.isTempContact && keptContact.idx == tempIdx)
            {
                keptContact.idx = contactIdx;
                keptContact.isTempContact = false;
                return;
            }
        }

        uint numContactsDistWeighted = vertContacts.numContactsDistWeighted;
        for (uint i = 0; i < numContactsDistWeighted; i++)
        {
            FmVertKeptContact& keptContact = vertContacts.contactsDistanceWeighted[i];

            if (keptContact.isTempContact && keptContact.idx == tempIdx)
            {
                keptContact.idx = contactIdx;
                keptContact.isTempContact = false;
                return;
            }
        }
    }
#endif

    void FmComputeLineSegmentBarycentricCoords(float barycentricCoords[2], const FmVector3& pos, const FmVector3& segmentPos0, const FmVector3& segmentPos1)
    {
        // Requires pos on line segment for valid result.
        // Compute barycentric values using ratio of lengths.
        // Check for degenerate input and constrain results.
        FmVector3 v = segmentPos1 - segmentPos0;
        FmVector3 w = pos - segmentPos0;

        float len_v = length(v);
        float len_w = length(w);

        float bary_y = (len_v == 0.0f) ? 1.0f : len_w / len_v;
        bary_y = (bary_y > 1.0f) ? 1.0f : bary_y;

        float bary_x = 1.0f - bary_y;

        barycentricCoords[0] = bary_x;
        barycentricCoords[1] = bary_y;
    }

    void FmComputeTriBarycentricCoords(float barycentricCoords[3], const FmVector3& pos, const FmVector3& triPos0, const FmVector3& triPos1, const FmVector3& triPos2)
    {
        // Requires pos on triangle for valid result.
        // Compute barycentric values using ratio of sub-triangle area to triangle area.
        // Check for degenerate input and constrain results.
        FmVector3 u = triPos1 - triPos0;
        FmVector3 v = triPos2 - triPos0;
        FmVector3 w = pos - triPos0;

        FmVector3 uxv = cross(u, v);
        FmVector3 uxw = cross(u, w);
        FmVector3 vxw = cross(v, w);

        float len_uxv = length(uxv);
        float len_uxw = length(uxw);
        float len_vxw = length(vxw);

        float bary_x, bary_y, bary_z;

        bary_y = (len_uxv == 0.0f) ? 1.0f : len_vxw / len_uxv;
        bary_z = (len_uxv == 0.0f) ? 1.0f : len_uxw / len_uxv;

        bary_y = (bary_y > 1.0f) ? 1.0f : bary_y;
        bary_z = (bary_z > 1.0f) ? 1.0f : bary_z;

        bary_y = (bary_y + bary_z > 1.0f) ? 1.0f - bary_z : bary_y;
        bary_x = 1.0f - bary_y - bary_z;

        barycentricCoords[0] = bary_x;
        barycentricCoords[1] = bary_y;
        barycentricCoords[2] = bary_z;
    }

    // Decide if a contact normal direction points to exterior of the mesh, based on local surface features.
    bool FmIsEdgeNormalExterior(const FmTetMesh& tetMesh, uint exteriorFaceId, uint edgeId, const FmVector3& direction)
    {
        const FmExteriorFace& exteriorFace = tetMesh.exteriorFaces[exteriorFaceId];

        if (dot(FmExteriorFaceCrossProd(tetMesh, exteriorFaceId), direction) > 0.0f) return true;

        uint incidentFaceId = exteriorFace.edgeIncidentFaceIds[edgeId];

        return (dot(FmExteriorFaceCrossProd(tetMesh, incidentFaceId), direction) > 0.0f);
    }

    FmVector3 FmComputeEdgeNormal(const FmTetMesh& tetMesh, uint exteriorFaceId, uint edgeId)
    {
        const FmExteriorFace& exteriorFace = tetMesh.exteriorFaces[exteriorFaceId];

        FmVector3 normal = normalize(FmExteriorFaceCrossProd(tetMesh, exteriorFaceId));

        uint incidentFaceId = exteriorFace.edgeIncidentFaceIds[edgeId];

        normal += normalize(FmExteriorFaceCrossProd(tetMesh, incidentFaceId));

        return normalize(normal);
    }

    // Decide if a contact normal direction points to exterior of the mesh, based on local surface features.
    bool FmIsVertNormalExterior(const FmTetMesh& tetMesh, uint exteriorFaceId, uint faceCornerId, const FmVector3& direction)
    {
#if 1
        // Heuristic based on total projection of adjacent verts on contact normal
        const FmExteriorFace& exteriorFace = tetMesh.exteriorFaces[exteriorFaceId];

        FmFaceVertIds faceVertIds;
        FmGetExteriorFaceVertIds(&faceVertIds, tetMesh, exteriorFaceId);

        uint vertId = faceVertIds.ids[faceCornerId];
        FmVector3 vertPos = tetMesh.vertsPos[vertId];

        uint adjacentVertId = faceVertIds.ids[(faceCornerId + 1) % 3];

        float totalProjection = dot(direction, tetMesh.vertsPos[adjacentVertId] - vertPos);

        uint edgeId = faceCornerId;

        uint previousFaceId = exteriorFaceId;
        uint incidentFaceId = exteriorFace.edgeIncidentFaceIds[edgeId];

        do
        {
            const FmExteriorFace& incidentExteriorFace = tetMesh.exteriorFaces[incidentFaceId];

            FmFaceVertIds incidentFaceVertIds;
            FmGetExteriorFaceVertIds(&incidentFaceVertIds, tetMesh, incidentFaceId);

            adjacentVertId = incidentFaceVertIds.ids[0];
            adjacentVertId = (incidentFaceVertIds.ids[0] == vertId) ? incidentFaceVertIds.ids[1] : adjacentVertId;
            adjacentVertId = (incidentFaceVertIds.ids[1] == vertId) ? incidentFaceVertIds.ids[2] : adjacentVertId;

            totalProjection += dot(direction, tetMesh.vertsPos[adjacentVertId] - vertPos);

            uint incidentEdgeId = 0;
            incidentEdgeId = (incidentExteriorFace.edgeIncidentFaceIds[1] == previousFaceId) ? 1 : incidentEdgeId;
            incidentEdgeId = (incidentExteriorFace.edgeIncidentFaceIds[2] == previousFaceId) ? 2 : incidentEdgeId;

            previousFaceId = incidentFaceId;
            incidentFaceId = incidentExteriorFace.edgeIncidentFaceIds[(incidentEdgeId + 1) % 3];

        } while (incidentFaceId != exteriorFaceId);

        return (totalProjection < 0.0f);
#else
        // Define direction from a vertex as exterior if exterior to any incident face
        FmExteriorFace& exteriorFace = tetMesh.exteriorFaces[exteriorFaceId];

        if (dot(FmExteriorFaceCrossProd(tetMesh, exteriorFaceId), direction) > 0.0f) return true;

        uint edgeId = faceCornerId;

        uint previousFaceId = exteriorFaceId;
        uint incidentFaceId = exteriorFace.edgeIncidentFaceIds[edgeId];

        do
        {
            FmExteriorFace& incidentExteriorFace = tetMesh.exteriorFaces[incidentFaceId];

            if (dot(FmExteriorFaceCrossProd(tetMesh, incidentFaceId), direction) > 0.0f) return true;

            uint incidentEdgeId = 0;
            incidentEdgeId = (incidentExteriorFace.edgeIncidentFaceIds[1] == previousFaceId) ? 1 : incidentEdgeId;
            incidentEdgeId = (incidentExteriorFace.edgeIncidentFaceIds[2] == previousFaceId) ? 2 : incidentEdgeId;

            previousFaceId = incidentFaceId;
            incidentFaceId = incidentExteriorFace.edgeIncidentFaceIds[(incidentEdgeId + 1) % 3];

        } while (incidentFaceId != exteriorFaceId);

        return false;
#endif
    }

    FmVector3 FmComputeVertNormal(const FmTetMesh& tetMesh, uint exteriorFaceId, uint vertId)
    {
        const FmExteriorFace& exteriorFace = tetMesh.exteriorFaces[exteriorFaceId];

        FmVector3 normal = normalize(FmExteriorFaceCrossProd(tetMesh, exteriorFaceId));

        uint edgeId = vertId;

        uint previousFaceId = exteriorFaceId;
        uint incidentFaceId = exteriorFace.edgeIncidentFaceIds[edgeId];

        do
        {
            const FmExteriorFace& incidentExteriorFace = tetMesh.exteriorFaces[incidentFaceId];

            normal += normalize(FmExteriorFaceCrossProd(tetMesh, incidentFaceId));

            uint incidentEdgeId = 0;
            incidentEdgeId = (incidentExteriorFace.edgeIncidentFaceIds[1] == previousFaceId) ? 1 : incidentEdgeId;
            incidentEdgeId = (incidentExteriorFace.edgeIncidentFaceIds[2] == previousFaceId) ? 2 : incidentEdgeId;

            previousFaceId = incidentFaceId;
            incidentFaceId = incidentExteriorFace.edgeIncidentFaceIds[(incidentEdgeId + 1) % 3];

        } while (incidentFaceId != exteriorFaceId);

        return normalize(normal);
    }

    // Copy the temp buffer of distance contacts into the constraints buffer array.
    // Update the contact reduction data.
    bool FmCopyTempDistanceContacts(FmCollidedObjectPair* collidedPair)
    {
        FmConstraintsBuffer* constraintsBuffer = collidedPair->constraintsBuffer;
        FmCollisionReport* collisionReport = collidedPair->collisionReport;

        uint numTempDistanceContacts = collidedPair->temps.numDistanceContacts;
        uint numTempDistanceContactsNonZeroRefCount = collidedPair->temps.numDistanceContactsNonzeroRefCount;

        if (FmAtomicRead(&constraintsBuffer->numDistanceContacts.val) + numTempDistanceContactsNonZeroRefCount <= constraintsBuffer->maxDistanceContacts)
        {
            uint endContactIdx = FmAtomicAdd(&constraintsBuffer->numDistanceContacts.val, numTempDistanceContactsNonZeroRefCount);

            if (endContactIdx <= constraintsBuffer->maxDistanceContacts)
            {
                uint dstContactIdx = endContactIdx - numTempDistanceContactsNonZeroRefCount;

                for (uint srcIdx = 0; srcIdx < numTempDistanceContacts; srcIdx++)
                {
                    const FmDistanceContactPairInfo& srcContactPairInfo = collidedPair->temps.distanceContactPairInfoBuffer[srcIdx];
                    const FmDistanceContact& srcContact = collidedPair->temps.distanceContactBuffer[srcIdx];

                    if (srcContactPairInfo.refCount == 0)
                    {
                        continue;
                    }

                    const FmDistanceContactTetVertIds& srcContactTetVertIds = collidedPair->temps.distanceContactTetVertIds[srcIdx];

                    constraintsBuffer->distanceContactsPairInfo[dstContactIdx] = srcContactPairInfo;
                    constraintsBuffer->distanceContacts[dstContactIdx] = srcContact;

#if FM_CONTACT_REDUCTION
                    FmContactReductionWorkspace* contactReductionWorkspace = collidedPair->temps.contactReductionWorkspace;
                    if (contactReductionWorkspace && contactReductionWorkspace->numVertsA > 0 && contactReductionWorkspace->numVertsB > 0)
                    {
                        FmReplaceKeptContactTempIndex(collidedPair->temps.contactReductionWorkspace->vertsContactsA, srcContactTetVertIds.tetVertIdsA.ids[0], srcIdx, dstContactIdx);
                        FmReplaceKeptContactTempIndex(collidedPair->temps.contactReductionWorkspace->vertsContactsA, srcContactTetVertIds.tetVertIdsA.ids[1], srcIdx, dstContactIdx);
                        FmReplaceKeptContactTempIndex(collidedPair->temps.contactReductionWorkspace->vertsContactsA, srcContactTetVertIds.tetVertIdsA.ids[2], srcIdx, dstContactIdx);
                        FmReplaceKeptContactTempIndex(collidedPair->temps.contactReductionWorkspace->vertsContactsA, srcContactTetVertIds.tetVertIdsA.ids[3], srcIdx, dstContactIdx);
                        FmReplaceKeptContactTempIndex(collidedPair->temps.contactReductionWorkspace->vertsContactsB, srcContactTetVertIds.tetVertIdsB.ids[0], srcIdx, dstContactIdx);
                        FmReplaceKeptContactTempIndex(collidedPair->temps.contactReductionWorkspace->vertsContactsB, srcContactTetVertIds.tetVertIdsB.ids[1], srcIdx, dstContactIdx);
                        FmReplaceKeptContactTempIndex(collidedPair->temps.contactReductionWorkspace->vertsContactsB, srcContactTetVertIds.tetVertIdsB.ids[2], srcIdx, dstContactIdx);
                        FmReplaceKeptContactTempIndex(collidedPair->temps.contactReductionWorkspace->vertsContactsB, srcContactTetVertIds.tetVertIdsB.ids[3], srcIdx, dstContactIdx);
                    }
#endif

                    dstContactIdx++;

                    // Collision reporting
                    if (collisionReport
                        && collisionReport->distanceContactBuffer
                        && FmAtomicRead(&collisionReport->numDistanceContacts.val) < collisionReport->maxDistanceContacts
                        && collidedPair->numDistanceContactReports < collisionReport->maxDistanceContactsPerObjectPair
                        && srcContact.normalProjRelVel > collisionReport->minContactRelVel)
                    {
                        uint reportContactIdx = FmAtomicIncrement(&collisionReport->numDistanceContacts.val) - 1;
                        if (reportContactIdx < collisionReport->maxDistanceContacts)
                        {
                            collidedPair->numDistanceContactReports++;

                            FmCollisionReportDistanceContact& reportContact = collisionReport->distanceContactBuffer[reportContactIdx];
                            reportContact.objectIdA = srcContactPairInfo.objectIdA;
                            reportContact.objectIdB = srcContactPairInfo.objectIdB;
                            reportContact.tetIdA = srcContact.tetIdA;
                            reportContact.tetIdB = srcContact.tetIdB;
                            reportContact.posBaryA[0] = srcContact.posBaryA[0];
                            reportContact.posBaryA[1] = srcContact.posBaryA[1];
                            reportContact.posBaryA[2] = srcContact.posBaryA[2];
                            reportContact.posBaryA[3] = srcContact.posBaryA[3];
                            reportContact.posBaryB[0] = srcContact.posBaryB[0];
                            reportContact.posBaryB[1] = srcContact.posBaryB[1];
                            reportContact.posBaryB[2] = srcContact.posBaryB[2];
                            reportContact.posBaryB[3] = srcContact.posBaryB[3];
                            reportContact.normal = srcContact.normal;
                        }
                        else
                        {
                            FmAtomicWrite(&collisionReport->numDistanceContacts.val, collisionReport->maxDistanceContacts);
                        }
                    }
                }

                collidedPair->temps.numDistanceContacts = 0;
                collidedPair->temps.numDistanceContactsNonzeroRefCount = 0;

                FM_ASSERT(dstContactIdx == endContactIdx);

                return true;
            }
            else
            {
                FmAtomicWrite(&constraintsBuffer->numDistanceContacts.val, constraintsBuffer->maxDistanceContacts);
            }
        }

#if FM_CONTACT_REDUCTION
        // Failed to add, invalidate contact reduction data
        if (collidedPair->temps.contactReductionWorkspace)
        {
            for (uint srcIdx = 0; srcIdx < numTempDistanceContacts; srcIdx++)
            {
                const FmDistanceContactTetVertIds& srcContactTetVertIds = collidedPair->temps.distanceContactTetVertIds[srcIdx];

                FmReplaceKeptContactTempIndex(collidedPair->temps.contactReductionWorkspace->vertsContactsA, srcContactTetVertIds.tetVertIdsA.ids[0], srcIdx, FM_INVALID_ID);
                FmReplaceKeptContactTempIndex(collidedPair->temps.contactReductionWorkspace->vertsContactsA, srcContactTetVertIds.tetVertIdsA.ids[1], srcIdx, FM_INVALID_ID);
                FmReplaceKeptContactTempIndex(collidedPair->temps.contactReductionWorkspace->vertsContactsA, srcContactTetVertIds.tetVertIdsA.ids[2], srcIdx, FM_INVALID_ID);
                FmReplaceKeptContactTempIndex(collidedPair->temps.contactReductionWorkspace->vertsContactsA, srcContactTetVertIds.tetVertIdsA.ids[3], srcIdx, FM_INVALID_ID);
                FmReplaceKeptContactTempIndex(collidedPair->temps.contactReductionWorkspace->vertsContactsB, srcContactTetVertIds.tetVertIdsB.ids[0], srcIdx, FM_INVALID_ID);
                FmReplaceKeptContactTempIndex(collidedPair->temps.contactReductionWorkspace->vertsContactsB, srcContactTetVertIds.tetVertIdsB.ids[1], srcIdx, FM_INVALID_ID);
                FmReplaceKeptContactTempIndex(collidedPair->temps.contactReductionWorkspace->vertsContactsB, srcContactTetVertIds.tetVertIdsB.ids[2], srcIdx, FM_INVALID_ID);
                FmReplaceKeptContactTempIndex(collidedPair->temps.contactReductionWorkspace->vertsContactsB, srcContactTetVertIds.tetVertIdsB.ids[3], srcIdx, FM_INVALID_ID);
            }
        }
#endif

        collidedPair->temps.numDistanceContacts = 0;
        collidedPair->temps.numDistanceContactsNonzeroRefCount = 0;

        return false;
    }

    struct FmContactReductionInputs
    {
        float timeOfImpact;
        float distanceA0;
        float distanceA1;
        float distanceA2;
        float distanceA3;
        float distanceB0;
        float distanceB1;
        float distanceB2;
        float distanceB3;
    };

    // Add contact to temp contacts buffer
    bool FmAddTempDistanceContact(
        FmCollidedObjectPair* collidedPair,
        FmDistanceContactPairInfo* contactPairInfo,
        FmDistanceContact* contact,
        const FmTetVertIds& tetVertIdsA,
        const FmTetVertIds& tetVertIdsB,
        uint dynamicFlagsA,
        uint dynamicFlagsB,
        FmContactReductionInputs* contactReductionInputs)  // only supported for tet mesh pair
    {
        FmCollidedObjectPairTemps& temps = collidedPair->temps;
        uint tempContactIdx = FM_INVALID_ID;

        // If temp buffer full, copy contacts to global list.
        // Will also update contact reduction data and report contacts.
        if (temps.numDistanceContacts == FM_MAX_TEMP_DISTANCE_CONTACTS)
        {
            if (!FmCopyTempDistanceContacts(collidedPair))
            {
                return false;
            }
        }

        tempContactIdx = temps.numDistanceContacts;

        FmConstraintsBuffer* constraintsBuffer = collidedPair->constraintsBuffer;

#if FM_CONTACT_REDUCTION
        if (contactReductionInputs)
        {
            // Check caches of contacts per vertex to decide if keeping
            float timeOfImpact = contactReductionInputs->timeOfImpact;
            FmVertKeptContact* kept0 = FmAddToKeptContacts(&temps, temps.contactReductionWorkspace->vertsContactsA, constraintsBuffer->distanceContactsPairInfo, tetVertIdsA.ids[0], timeOfImpact, contactReductionInputs->distanceA0);
            FmVertKeptContact* kept1 = FmAddToKeptContacts(&temps, temps.contactReductionWorkspace->vertsContactsA, constraintsBuffer->distanceContactsPairInfo, tetVertIdsA.ids[1], timeOfImpact, contactReductionInputs->distanceA1);
            FmVertKeptContact* kept2 = FmAddToKeptContacts(&temps, temps.contactReductionWorkspace->vertsContactsA, constraintsBuffer->distanceContactsPairInfo, tetVertIdsA.ids[2], timeOfImpact, contactReductionInputs->distanceA2);
            FmVertKeptContact* kept3 = FmAddToKeptContacts(&temps, temps.contactReductionWorkspace->vertsContactsA, constraintsBuffer->distanceContactsPairInfo, tetVertIdsA.ids[3], timeOfImpact, contactReductionInputs->distanceA3);
            FmVertKeptContact* kept4 = FmAddToKeptContacts(&temps, temps.contactReductionWorkspace->vertsContactsB, constraintsBuffer->distanceContactsPairInfo, tetVertIdsB.ids[0], timeOfImpact, contactReductionInputs->distanceB0);
            FmVertKeptContact* kept5 = FmAddToKeptContacts(&temps, temps.contactReductionWorkspace->vertsContactsB, constraintsBuffer->distanceContactsPairInfo, tetVertIdsB.ids[1], timeOfImpact, contactReductionInputs->distanceB1);
            FmVertKeptContact* kept6 = FmAddToKeptContacts(&temps, temps.contactReductionWorkspace->vertsContactsB, constraintsBuffer->distanceContactsPairInfo, tetVertIdsB.ids[2], timeOfImpact, contactReductionInputs->distanceB2);
            FmVertKeptContact* kept7 = FmAddToKeptContacts(&temps, temps.contactReductionWorkspace->vertsContactsB, constraintsBuffer->distanceContactsPairInfo, tetVertIdsB.ids[3], timeOfImpact, contactReductionInputs->distanceB3);

            uint numTimesAdded =
                (uint)(kept0 != NULL)
                + (uint)(kept1 != NULL)
                + (uint)(kept2 != NULL)
                + (uint)(kept3 != NULL)
                + (uint)(kept4 != NULL)
                + (uint)(kept5 != NULL)
                + (uint)(kept6 != NULL)
                + (uint)(kept7 != NULL);

            if (numTimesAdded == 0)
            {
                return false;
            }

            // Set number of references from vertices
            contactPairInfo->refCount = (uint8_t)numTimesAdded;
        }
#endif

        collidedPair->numDistanceContacts++;

        if (dynamicFlagsA)
        {
            collidedPair->objectAContactWithDynamicVerts = true;
        }
        if (dynamicFlagsB)
        {
            collidedPair->objectBContactWithDynamicVerts = true;
        }

        temps.distanceContactPairInfoBuffer[tempContactIdx] = *contactPairInfo;
        temps.distanceContactBuffer[tempContactIdx] = *contact;
        temps.distanceContactTetVertIds[tempContactIdx].tetVertIdsA = tetVertIdsA;
        temps.distanceContactTetVertIds[tempContactIdx].tetVertIdsB = tetVertIdsB;
        temps.numDistanceContacts++;
        temps.numDistanceContactsNonzeroRefCount++;

        return true;
    }

    bool FmFinalizeDistanceContact(FmCollidedObjectPair* collidedPair, const FmMeshCollisionTriPair& triPair, const FmCcdResult& triContact)
    {
        FmTetMesh* meshA = collidedPair->tetMeshA;
        FmTetMesh* meshB = collidedPair->tetMeshB;
        float tetAFrictionCoeff = meshA->tetsFrictionCoeff[triPair.tetIdA];
        float tetBFrictionCoeff = meshB->tetsFrictionCoeff[triPair.tetIdB];
        FmTetVertIds tetVertIdsA = meshA->tetsVertIds[triPair.tetIdA];
        FmTetVertIds tetVertIdsB = meshB->tetsVertIds[triPair.tetIdB];
        float distContactBias = collidedPair->distContactBias;

        FM_ASSERT(!(triContact.retVal == FM_CCD_RET_NO_IMPACT || triContact.distance <= 0.0f));

        // Create tet pair from tri contact
        FmDistanceContactPairInfo contactPairInfo;
        contactPairInfo.idInObjectPair = collidedPair->numDistanceContacts;
        contactPairInfo.objectIdA = collidedPair->objectAId;
        contactPairInfo.objectIdB = collidedPair->objectBId;
        contactPairInfo.flags = 0;

        FmDistanceContact contact;
        contact.posBaryA[0] = 0.0f;
        contact.posBaryA[1] = 0.0f;
        contact.posBaryA[2] = 0.0f;
        contact.posBaryA[3] = 0.0f;
        contact.posBaryB[0] = 0.0f;
        contact.posBaryB[1] = 0.0f;
        contact.posBaryB[2] = 0.0f;
        contact.posBaryB[3] = 0.0f;
        contact.tetIdA = triPair.tetIdA;
        contact.tetIdB = triPair.tetIdB;
        contact.frictionCoeff = FmMinFloat(tetAFrictionCoeff, tetBFrictionCoeff);

        // Found a contact, calculate the tet barycentric coords

        if (triContact.featurePair.itype == FM_FEATURE_TYPE_VERTEX)
        {
            contact.posBaryA[triPair.tetCornersA.ids[triContact.featurePair.i0]] = 1.0f;
        }


        if (triContact.featurePair.jtype == FM_FEATURE_TYPE_VERTEX)
        {
            contact.posBaryB[triPair.tetCornersB.ids[triContact.featurePair.j0]] = 1.0f;
        }

        if (triContact.featurePair.itype == FM_FEATURE_TYPE_EDGE)
        {
            float bary[2];
            FmComputeLineSegmentBarycentricCoords(
                bary, triContact.posi,
                triPair.triA.Pos(triContact.featurePair.i0, triContact.time),
                triPair.triA.Pos(triContact.featurePair.i1, triContact.time));

            contact.posBaryA[triPair.tetCornersA.ids[triContact.featurePair.i0]] = bary[0];
            contact.posBaryA[triPair.tetCornersA.ids[triContact.featurePair.i1]] = bary[1];
        }

        if (triContact.featurePair.jtype == FM_FEATURE_TYPE_EDGE)
        {
            float bary[2];
            FmComputeLineSegmentBarycentricCoords(
                bary, triContact.posj,
                triPair.triB.Pos(triContact.featurePair.j0, triContact.time),
                triPair.triB.Pos(triContact.featurePair.j1, triContact.time));

            contact.posBaryB[triPair.tetCornersB.ids[triContact.featurePair.j0]] = bary[0];
            contact.posBaryB[triPair.tetCornersB.ids[triContact.featurePair.j1]] = bary[1];
        }

        if (triContact.featurePair.itype == FM_FEATURE_TYPE_FACE)
        {
            float bary[3];
            FmComputeTriBarycentricCoords(
                bary, triContact.posi,
                triPair.triA.Pos(0, triContact.time),
                triPair.triA.Pos(1, triContact.time),
                triPair.triA.Pos(2, triContact.time));

            contact.posBaryA[triPair.tetCornersA.ids[0]] = bary[0];
            contact.posBaryA[triPair.tetCornersA.ids[1]] = bary[1];
            contact.posBaryA[triPair.tetCornersA.ids[2]] = bary[2];
        }

        if (triContact.featurePair.jtype == FM_FEATURE_TYPE_FACE)
        {
            float bary[3];
            FmComputeTriBarycentricCoords(
                bary, triContact.posj,
                triPair.triB.Pos(0, triContact.time),
                triPair.triB.Pos(1, triContact.time),
                triPair.triB.Pos(2, triContact.time));

            contact.posBaryB[triPair.tetCornersB.ids[0]] = bary[0];
            contact.posBaryB[triPair.tetCornersB.ids[1]] = bary[1];
            contact.posBaryB[triPair.tetCornersB.ids[2]] = bary[2];
        }

        FmVector3 vertPosA0 = meshA->vertsPos[tetVertIdsA.ids[0]];
        FmVector3 vertPosA1 = meshA->vertsPos[tetVertIdsA.ids[1]];
        FmVector3 vertPosA2 = meshA->vertsPos[tetVertIdsA.ids[2]];
        FmVector3 vertPosA3 = meshA->vertsPos[tetVertIdsA.ids[3]];

        FmVector3 vertPosB0 = meshB->vertsPos[tetVertIdsB.ids[0]];
        FmVector3 vertPosB1 = meshB->vertsPos[tetVertIdsB.ids[1]];
        FmVector3 vertPosB2 = meshB->vertsPos[tetVertIdsB.ids[2]];
        FmVector3 vertPosB3 = meshB->vertsPos[tetVertIdsB.ids[3]];

        FmVector3 vertVelA0 = meshA->vertsVel[tetVertIdsA.ids[0]];
        FmVector3 vertVelA1 = meshA->vertsVel[tetVertIdsA.ids[1]];
        FmVector3 vertVelA2 = meshA->vertsVel[tetVertIdsA.ids[2]];
        FmVector3 vertVelA3 = meshA->vertsVel[tetVertIdsA.ids[3]];

        FmVector3 vertVelB0 = meshB->vertsVel[tetVertIdsB.ids[0]];
        FmVector3 vertVelB1 = meshB->vertsVel[tetVertIdsB.ids[1]];
        FmVector3 vertVelB2 = meshB->vertsVel[tetVertIdsB.ids[2]];
        FmVector3 vertVelB3 = meshB->vertsVel[tetVertIdsB.ids[3]];

        FmVector3 posA = FmInterpolate(contact.posBaryA, vertPosA0, vertPosA1, vertPosA2, vertPosA3);
        FmVector3 velA = FmInterpolate(contact.posBaryA, vertVelA0, vertVelA1, vertVelA2, vertVelA3);

        FmVector3 posB = FmInterpolate(contact.posBaryB, vertPosB0, vertPosB1, vertPosB2, vertPosB3);
        FmVector3 velB = FmInterpolate(contact.posBaryB, vertVelB0, vertVelB1, vertVelB2, vertVelB3);

        contact.normal = -triContact.direction;
        contact.normalProjDistance = dot(contact.normal, posA - posB) - distContactBias;
        contact.normalProjRelVel = dot(contact.normal, velB - velA);
        contact.tangent1 = FmGetContactTangent(contact.normal, velA, velB);

        // Set dynamic bits, used to find dependencies between contacts and size the constraint Jacobian
        uint dynamicFlagsA, movingFlagsA, dynamicFlagsB, movingFlagsB;

        FmDynamicAndMovingFlags(&dynamicFlagsA, &movingFlagsA, *meshA, tetVertIdsA, contact.posBaryA);
        FmDynamicAndMovingFlags(&dynamicFlagsB, &movingFlagsB, *meshB, tetVertIdsB, contact.posBaryB);

        FmSetConstraintFlags(&contactPairInfo.flags, &contactPairInfo.dynamicFlags, &contact.movingFlags, dynamicFlagsA, dynamicFlagsB, movingFlagsA, movingFlagsB);

        // If no dynamic vertices, nothing to constrain
        if (contactPairInfo.dynamicFlags == 0)
        {
            return false;
        }

        bool exterior = true;

        // Test normal
        if (triContact.featurePair.itype == FM_FEATURE_TYPE_VERTEX)
        {
            if (!FmIsVertNormalExterior(*meshA, triPair.triA.GetTriId(), triContact.featurePair.i0, -contact.normal))
            {
                exterior = false;
            }
        }
        if (triContact.featurePair.jtype == FM_FEATURE_TYPE_VERTEX)
        {
            if (!FmIsVertNormalExterior(*meshB, triPair.triB.GetTriId(), triContact.featurePair.j0, contact.normal))
            {
                exterior = false;
            }
        }
        if (triContact.featurePair.itype == FM_FEATURE_TYPE_EDGE)
        {
            if (!FmIsEdgeNormalExterior(*meshA, triPair.triA.GetTriId(), triContact.featurePair.i0, -contact.normal))
            {
                exterior = false;
            }
        }
        if (triContact.featurePair.jtype == FM_FEATURE_TYPE_EDGE)
        {
            if (!FmIsEdgeNormalExterior(*meshB, triPair.triB.GetTriId(), triContact.featurePair.j0, contact.normal))
            {
                exterior = false;
            }
        }
        if (triContact.featurePair.itype == FM_FEATURE_TYPE_FACE)
        {
            if (dot(contact.normal, FmExteriorFaceCrossProd(*meshA, triPair.triA.GetTriId())) >= 0.0f)
            {
                exterior = false;
            }
        }
        if (triContact.featurePair.jtype == FM_FEATURE_TYPE_FACE)
        {
            if (dot(contact.normal, FmExteriorFaceCrossProd(*meshB, triPair.triB.GetTriId())) <= 0.0f)
            {
                exterior = false;
            }
        }

        if (!exterior)
        {
            return false;
        }
        
        FmContactReductionInputs contactReductionInputs;
        contactReductionInputs.timeOfImpact = triContact.time;
        contactReductionInputs.distanceA0 = lengthSqr(posA - vertPosA0);
        contactReductionInputs.distanceA1 = lengthSqr(posA - vertPosA1);
        contactReductionInputs.distanceA2 = lengthSqr(posA - vertPosA2);
        contactReductionInputs.distanceA3 = lengthSqr(posA - vertPosA3);
        contactReductionInputs.distanceB0 = lengthSqr(posB - vertPosB0);
        contactReductionInputs.distanceB1 = lengthSqr(posB - vertPosB1);
        contactReductionInputs.distanceB2 = lengthSqr(posB - vertPosB2);
        contactReductionInputs.distanceB3 = lengthSqr(posB - vertPosB3);

        return FmAddTempDistanceContact(collidedPair, &contactPairInfo, &contact, tetVertIdsA, tetVertIdsB, dynamicFlagsA, dynamicFlagsB, &contactReductionInputs);
    }

    static FM_FORCE_INLINE bool FmAcceptCcdContact(const FmCcdResult& contact, float distanceContactThreshold)
    {
        return (contact.retVal != FM_CCD_RET_NO_IMPACT && contact.distance > 0.0f && contact.distance <= distanceContactThreshold);
    }

#if FM_SOA_TRI_CCD
    template<class SoaTypes>
    struct FmFeaturePairVFBatch
    {
        FmMeshCollisionTriPair* triPair[SoaTypes::width];  // pointer to triangle pair data
        uint numPairs;                              // number of pairs batched
        FmSoaVFPairData<SoaTypes> pairData;

        // AoS pair data
        FM_ALIGN(16) FmVector3 vApos[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 vAvel[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 fBpos0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 fBpos1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 fBpos2[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 fBvel0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 fBvel1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 fBvel2[SoaTypes::width] FM_ALIGN_END(16);
        uint i[SoaTypes::width];

        FmFeaturePairVFBatch()
        {
            numPairs = 0;
            for (uint sliceIdx = 0; sliceIdx < SoaTypes::width; sliceIdx++)
            {
                triPair[sliceIdx] = NULL;
                vApos[sliceIdx] = FmInitVector3(0.0f);
                vAvel[sliceIdx] = FmInitVector3(0.0f);
                fBpos1[sliceIdx] = FmInitVector3(0.0f);
                fBpos2[sliceIdx] = FmInitVector3(0.0f);
                fBvel0[sliceIdx] = FmInitVector3(0.0f);
                fBvel1[sliceIdx] = FmInitVector3(0.0f);
                fBvel2[sliceIdx] = FmInitVector3(0.0f);
                i[sliceIdx] = 0;
            }
        }
    };

    template<class SoaTypes>
    struct FmFeaturePairFVBatch
    {
        FmMeshCollisionTriPair* triPair[SoaTypes::width];  // pointer to triangle pair data
        uint numPairs;                              // number of pairs batched
        FmSoaFVPairData<SoaTypes> pairData;

        // AoS pair data
        FM_ALIGN(16) FmVector3 fApos0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 fApos1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 fApos2[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 fAvel0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 fAvel1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 fAvel2[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 vBpos[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 vBvel[SoaTypes::width] FM_ALIGN_END(16);
        uint j[SoaTypes::width];

        FmFeaturePairFVBatch()
        {
            numPairs = 0;
            for (uint sliceIdx = 0; sliceIdx < SoaTypes::width; sliceIdx++)
            {
                triPair[sliceIdx] = NULL;
                fApos0[sliceIdx] = FmInitVector3(0.0f);
                fApos1[sliceIdx] = FmInitVector3(0.0f);
                fApos2[sliceIdx] = FmInitVector3(0.0f);
                fAvel0[sliceIdx] = FmInitVector3(0.0f);
                fAvel1[sliceIdx] = FmInitVector3(0.0f);
                fAvel2[sliceIdx] = FmInitVector3(0.0f);
                vBpos[sliceIdx] = FmInitVector3(0.0f);
                vBvel[sliceIdx] = FmInitVector3(0.0f);
                j[sliceIdx] = 0;
            }
        }

    };

    template<class SoaTypes>
    struct FmFeaturePairEEBatch
    {
        FmMeshCollisionTriPair* triPair[SoaTypes::width];  // pointer to triangle pair data
        uint numPairs;                              // number of pairs batched
        FmSoaEEPairData<SoaTypes> pairData;

        // AoS pair data
        FM_ALIGN(16) FmVector3 eApos0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 eApos1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 eAvel0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 eAvel1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 eBpos0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 eBpos1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 eBvel0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 eBvel1[SoaTypes::width] FM_ALIGN_END(16);
        uint i[SoaTypes::width];
        uint j[SoaTypes::width];

        FmFeaturePairEEBatch()
        {
            numPairs = 0;
            for (uint sliceIdx = 0; sliceIdx < SoaTypes::width; sliceIdx++)
            {
                triPair[sliceIdx] = NULL;
                eApos0[sliceIdx] = FmInitVector3(0.0f);
                eApos1[sliceIdx] = FmInitVector3(0.0f);
                eAvel0[sliceIdx] = FmInitVector3(0.0f);
                eAvel1[sliceIdx] = FmInitVector3(0.0f);
                eBpos0[sliceIdx] = FmInitVector3(0.0f);
                eBpos1[sliceIdx] = FmInitVector3(0.0f);
                eBvel0[sliceIdx] = FmInitVector3(0.0f);
                eBvel1[sliceIdx] = FmInitVector3(0.0f);
                i[sliceIdx] = 0;
                j[sliceIdx] = 0;
            }
        }
    };

    template<class SoaTypes>
    void FmProcessVFBatch(
        FmCollidedObjectPair* collidedPair,
        FmFeaturePairVFBatch<SoaTypes>& vfBatch,
        const FmSoaCcdTerminationConditions<SoaTypes>& soaConditions)
    {
        FmSoaCcdResult<SoaTypes> result;

        FmSetSoaFromAlignedAos(&vfBatch.pairData.vApos, vfBatch.vApos);
        FmSetSoaFromAlignedAos(&vfBatch.pairData.vAvel, vfBatch.vAvel);
        FmSetSoaFromAlignedAos(&vfBatch.pairData.fBpos[0], vfBatch.fBpos0);
        FmSetSoaFromAlignedAos(&vfBatch.pairData.fBpos[1], vfBatch.fBpos1);
        FmSetSoaFromAlignedAos(&vfBatch.pairData.fBpos[2], vfBatch.fBpos2);
        FmSetSoaFromAlignedAos(&vfBatch.pairData.fBvel[0], vfBatch.fBvel0);
        FmSetSoaFromAlignedAos(&vfBatch.pairData.fBvel[1], vfBatch.fBvel1);
        FmSetSoaFromAlignedAos(&vfBatch.pairData.fBvel[2], vfBatch.fBvel2);
        vfBatch.pairData.i = SoaTypes::SoaUint(vfBatch.i);

        FmVertexFaceCcd(&result, vfBatch.pairData, collidedPair->timestep, soaConditions);

        for (uint idx = 0; idx < vfBatch.numPairs; idx++)
        {
            FmCcdResult contact;
            FmGetSlice(&contact, result, idx);

            if (FmAcceptCcdContact(contact, collidedPair->distContactThreshold) && contact.featurePair.jtype == FM_FEATURE_TYPE_FACE)
            {
                FmFinalizeDistanceContact(collidedPair, *vfBatch.triPair[idx], contact);
            }
        }

        vfBatch.numPairs = 0;
    }

    template<class SoaTypes>
    void FmProcessFVBatch(
        FmCollidedObjectPair* collidedPair,
        FmFeaturePairFVBatch<SoaTypes>& fvBatch,
        const FmSoaCcdTerminationConditions<SoaTypes>& soaConditions)
    {
        FmSoaCcdResult<SoaTypes> result;

        FmSetSoaFromAlignedAos(&fvBatch.pairData.fApos[0], fvBatch.fApos0);
        FmSetSoaFromAlignedAos(&fvBatch.pairData.fApos[1], fvBatch.fApos1);
        FmSetSoaFromAlignedAos(&fvBatch.pairData.fApos[2], fvBatch.fApos2);
        FmSetSoaFromAlignedAos(&fvBatch.pairData.fAvel[0], fvBatch.fAvel0);
        FmSetSoaFromAlignedAos(&fvBatch.pairData.fAvel[1], fvBatch.fAvel1);
        FmSetSoaFromAlignedAos(&fvBatch.pairData.fAvel[2], fvBatch.fAvel2);
        FmSetSoaFromAlignedAos(&fvBatch.pairData.vBpos, fvBatch.vBpos);
        FmSetSoaFromAlignedAos(&fvBatch.pairData.vBvel, fvBatch.vBvel);
        fvBatch.pairData.j = SoaTypes::SoaUint(fvBatch.j);

        FmFaceVertexCcd(&result, fvBatch.pairData, collidedPair->timestep, soaConditions);

        for (uint idx = 0; idx < fvBatch.numPairs; idx++)
        {
            FmCcdResult contact;
            FmGetSlice(&contact, result, idx);

            if (FmAcceptCcdContact(contact, collidedPair->distContactThreshold) && contact.featurePair.itype == FM_FEATURE_TYPE_FACE)
            {
                FmFinalizeDistanceContact(collidedPair, *fvBatch.triPair[idx], contact);
            }
        }

        fvBatch.numPairs = 0;
    }

    template<class SoaTypes>
    void FmProcessEEBatch(
        FmCollidedObjectPair* collidedPair,
        FmFeaturePairEEBatch<SoaTypes>& eeBatch,
        const FmSoaCcdTerminationConditions<SoaTypes>& soaConditions)
    {
        FmSoaCcdResult<SoaTypes> result;

        FmSetSoaFromAlignedAos(&eeBatch.pairData.eApos[0], eeBatch.eApos0);
        FmSetSoaFromAlignedAos(&eeBatch.pairData.eApos[1], eeBatch.eApos1);
        FmSetSoaFromAlignedAos(&eeBatch.pairData.eAvel[0], eeBatch.eAvel0);
        FmSetSoaFromAlignedAos(&eeBatch.pairData.eAvel[1], eeBatch.eAvel1);
        FmSetSoaFromAlignedAos(&eeBatch.pairData.eBpos[0], eeBatch.eBpos0);
        FmSetSoaFromAlignedAos(&eeBatch.pairData.eBpos[1], eeBatch.eBpos1);
        FmSetSoaFromAlignedAos(&eeBatch.pairData.eBvel[0], eeBatch.eBvel0);
        FmSetSoaFromAlignedAos(&eeBatch.pairData.eBvel[1], eeBatch.eBvel1);
        eeBatch.pairData.i = SoaTypes::SoaUint(eeBatch.i);
        eeBatch.pairData.j = SoaTypes::SoaUint(eeBatch.j);

        FmEdgeEdgeCcd(&result, eeBatch.pairData, collidedPair->timestep, soaConditions);

        for (uint idx = 0; idx < eeBatch.numPairs; idx++)
        {
            FmMeshCollisionTriPair& pair = *eeBatch.triPair[idx];

            FmCcdResult contact;
            FmGetSlice(&contact, result, idx);

            if (FmAcceptCcdContact(contact, collidedPair->distContactThreshold) &&
                !((contact.featurePair.itype == FM_FEATURE_TYPE_VERTEX && !pair.triA.OwnsVert(contact.featurePair.i0))
                    ||
                    (contact.featurePair.jtype == FM_FEATURE_TYPE_VERTEX && !pair.triB.OwnsVert(contact.featurePair.j0))))
            {
                uint bit = 1UL << (contact.featurePair.i0 * 3 + contact.featurePair.j0);

                uint found = 0;
                if (contact.featurePair.itype == FM_FEATURE_TYPE_VERTEX && contact.featurePair.jtype == FM_FEATURE_TYPE_VERTEX)
                {
                    found = pair.vertVertFound & bit;
                    pair.vertVertFound |= bit;
                }
                else if (contact.featurePair.itype == FM_FEATURE_TYPE_VERTEX && contact.featurePair.jtype == FM_FEATURE_TYPE_EDGE)
                {
                    found = pair.vertEdgeFound & bit;
                    pair.vertEdgeFound |= bit;
                }
                else if (contact.featurePair.itype == FM_FEATURE_TYPE_EDGE && contact.featurePair.jtype == FM_FEATURE_TYPE_VERTEX)
                {
                    found = pair.edgeVertFound & bit;
                    pair.edgeVertFound |= bit;
                }

                if (!found)
                {
                    FmFinalizeDistanceContact(collidedPair, pair, contact);
                }
            }
        }

        eeBatch.numPairs = 0;
    }

    template<class SoaTypes>
    void FmGenerateCcdContacts(
        FmCollidedObjectPair* collidedPair,
        FmMeshCollisionTriPair& triPair,
        FmFeaturePairVFBatch<SoaTypes>& vfBatch,
        FmFeaturePairFVBatch<SoaTypes>& fvBatch,
        FmFeaturePairEEBatch<SoaTypes>& eeBatch)
    {
        FmTetMesh* meshA = collidedPair->tetMeshA;
        FmTetMesh* meshB = collidedPair->tetMeshB;
        float distContactBias = collidedPair->distContactBias;
        float distContactThreshold = collidedPair->distContactThreshold;
        uint meshIdA = meshA->objectId;
        uint meshIdB = meshB->objectId;

        // Perform CCD checks with features of triangles
        FmSoaCcdTerminationConditions<SoaTypes> soaConditions;
        soaConditions.impactGap = distContactBias;
        soaConditions.contactGap = distContactThreshold;
        soaConditions.maxIterations = 4;

        // Zero found flags for CCD
        triPair.vertVertFound = 0;
        triPair.vertEdgeFound = 0;
        triPair.edgeVertFound = 0;

        for (uint i = 0; i < 3; i++)
        {
            // Vertex/face pairs

            // If self-collision, skip feature pair with incident verts
            bool incidentVerts = (meshIdA == meshIdB &&
                (triPair.faceVertIdsA.ids[i] == triPair.faceVertIdsB.ids[0] ||
                    triPair.faceVertIdsA.ids[i] == triPair.faceVertIdsB.ids[1] ||
                    triPair.faceVertIdsA.ids[i] == triPair.faceVertIdsB.ids[2]));

            if (triPair.triA.OwnsVert(i)
                && !incidentVerts
                && dot(triPair.triA.Pos(i) - triPair.triB.pos0, cross(triPair.triB.pos1 - triPair.triB.pos0, triPair.triB.pos2 - triPair.triB.pos0)) > 0.0f // test vert on outside side
                )
            {
                uint sliceIdx = vfBatch.numPairs;

                vfBatch.vApos[sliceIdx] = triPair.triA.Pos(i);
                vfBatch.vAvel[sliceIdx] = triPair.triA.Vel(i);
                vfBatch.fBpos0[sliceIdx] = triPair.triB.Pos(0);
                vfBatch.fBvel0[sliceIdx] = triPair.triB.Vel(0);
                vfBatch.fBpos1[sliceIdx] = triPair.triB.Pos(1);
                vfBatch.fBvel1[sliceIdx] = triPair.triB.Vel(1);
                vfBatch.fBpos2[sliceIdx] = triPair.triB.Pos(2);
                vfBatch.fBvel2[sliceIdx] = triPair.triB.Vel(2);
                vfBatch.i[sliceIdx] = i;

                vfBatch.triPair[vfBatch.numPairs] = &triPair;
                vfBatch.numPairs++;

                if (vfBatch.numPairs == SoaTypes::width)
                {
                    FmProcessVFBatch(collidedPair, vfBatch, soaConditions);
                }
            }

            // Face/vertex pairs

            incidentVerts = (meshIdA == meshIdB &&
                (triPair.faceVertIdsB.ids[i] == triPair.faceVertIdsA.ids[0] ||
                    triPair.faceVertIdsB.ids[i] == triPair.faceVertIdsA.ids[1] ||
                    triPair.faceVertIdsB.ids[i] == triPair.faceVertIdsA.ids[2]));

            if (triPair.triB.OwnsVert(i)
                && !incidentVerts
                && dot(triPair.triB.Pos(i) - triPair.triA.pos0, cross(triPair.triA.pos1 - triPair.triA.pos0, triPair.triA.pos2 - triPair.triA.pos0)) > 0.0f // test vert on outside side
                )
            {
                uint sliceIdx = fvBatch.numPairs;

                fvBatch.fApos0[sliceIdx] = triPair.triA.Pos(0);
                fvBatch.fAvel0[sliceIdx] = triPair.triA.Vel(0);
                fvBatch.fApos1[sliceIdx] = triPair.triA.Pos(1);
                fvBatch.fAvel1[sliceIdx] = triPair.triA.Vel(1);
                fvBatch.fApos2[sliceIdx] = triPair.triA.Pos(2);
                fvBatch.fAvel2[sliceIdx] = triPair.triA.Vel(2);
                fvBatch.vBpos[sliceIdx] = triPair.triB.Pos(i);
                fvBatch.vBvel[sliceIdx] = triPair.triB.Vel(i);
                fvBatch.j[sliceIdx] = i;

                fvBatch.triPair[fvBatch.numPairs] = &triPair;
                fvBatch.numPairs++;

                if (fvBatch.numPairs == SoaTypes::width)
                {
                    FmProcessFVBatch(collidedPair, fvBatch, soaConditions);
                }
            }

            // Edge/edge pairs

            uint i1 = (i + 1) % 3;

            for (uint j = 0; j < 3; j++)
            {
                uint j1 = (j + 1) % 3;

                incidentVerts = (meshIdA == meshIdB &&
                    (triPair.faceVertIdsA.ids[i] == triPair.faceVertIdsB.ids[j] ||
                        triPair.faceVertIdsA.ids[i] == triPair.faceVertIdsB.ids[j1] ||
                        triPair.faceVertIdsA.ids[i1] == triPair.faceVertIdsB.ids[j] ||
                        triPair.faceVertIdsA.ids[i1] == triPair.faceVertIdsB.ids[j1]));

                // only test if edges not already intersecting with face
                bool edgeIntersectsFace = triPair.pairIntersects && (triPair.triIntersectionPoints.XVals[i] != 0 || triPair.triIntersectionPoints.XVals[j + 3] != 0);

                if (triPair.triA.OwnsEdge(i) && triPair.triB.OwnsEdge(j)           // only one tri pair will test this edge pair
                    && !incidentVerts
                    && !edgeIntersectsFace)
                {
                    uint sliceIdx = eeBatch.numPairs;

                    eeBatch.eApos0[sliceIdx] = triPair.triA.Pos(i);
                    eeBatch.eAvel0[sliceIdx] = triPair.triA.Vel(i);
                    eeBatch.eApos1[sliceIdx] = triPair.triA.Pos(i1);
                    eeBatch.eAvel1[sliceIdx] = triPair.triA.Vel(i1);
                    eeBatch.eBpos0[sliceIdx] = triPair.triB.Pos(j);
                    eeBatch.eBvel0[sliceIdx] = triPair.triB.Vel(j);
                    eeBatch.eBpos1[sliceIdx] = triPair.triB.Pos(j1);
                    eeBatch.eBvel1[sliceIdx] = triPair.triB.Vel(j1);
                    eeBatch.i[sliceIdx] = i;
                    eeBatch.j[sliceIdx] = j;

                    eeBatch.triPair[eeBatch.numPairs] = &triPair;
                    eeBatch.numPairs++;

                    if (eeBatch.numPairs == SoaTypes::width)
                    {
                        FmProcessEEBatch(collidedPair, eeBatch, soaConditions);
                    }
                }
            }
        }
    }
#else
    void FmGenerateCcdContacts(FmCollidedObjectPair* collidedPair, FmMeshCollisionTriPair& triPair)
    {
        FmTetMesh* meshA = collidedPair->tetMeshA;
        FmTetMesh* meshB = collidedPair->tetMeshB;
        float distContactBias = collidedPair->distContactBias;
        float distContactThreshold = collidedPair->distContactThreshold;
        uint meshIdA = meshA->objectId;
        uint meshIdB = meshB->objectId;
        float timestep = collidedPair->timestep;

        // Perform CCD checks with features of triangles
        FmCcdTerminationConditions conditions;
        conditions.impactGap = distContactBias;
        conditions.contactGap = distContactThreshold;
        conditions.maxIterations = 4;

        // Zero found flags for CCD
        uint16_t vertVertFound = 0;
        uint16_t vertEdgeFound = 0;
        uint16_t edgeVertFound = 0;

        for (uint i = 0; i < 3; i++)
        {
            // Vertex/face pairs

            // If self-collision, skip feature pair with incident verts
            bool incidentVerts = (meshIdA == meshIdB &&
                (triPair.faceVertIdsA.ids[i] == triPair.faceVertIdsB.ids[0] ||
                    triPair.faceVertIdsA.ids[i] == triPair.faceVertIdsB.ids[1] ||
                    triPair.faceVertIdsA.ids[i] == triPair.faceVertIdsB.ids[2]));

            if (triPair.triA.OwnsVert(i)
                && !incidentVerts
                && dot(triPair.triA.Pos(i) - triPair.triB.pos0, cross(triPair.triB.pos1 - triPair.triB.pos0, triPair.triB.pos2 - triPair.triB.pos0)) > 0.0f // test vert on outside side
                )
            {
                FmVertexFacePairData vfPair;
                vfPair.vApos = triPair.triA.Pos(i);
                vfPair.vAvel = triPair.triA.Vel(i);
                vfPair.fBpos[0] = triPair.triB.Pos(0);
                vfPair.fBvel[0] = triPair.triB.Vel(0);
                vfPair.fBpos[1] = triPair.triB.Pos(1);
                vfPair.fBvel[1] = triPair.triB.Vel(1);
                vfPair.fBpos[2] = triPair.triB.Pos(2);
                vfPair.fBvel[2] = triPair.triB.Vel(2);
                vfPair.i = i;

                FmCcdResult contact;

                FmVertexFaceCcd(&contact, vfPair, timestep, conditions);

                if (FmAcceptCcdContact(contact, collidedPair->distContactThreshold) && contact.featurePair.jtype == FM_FEATURE_TYPE_FACE)
                {
                    FmFinalizeDistanceContact(collidedPair, triPair, contact);
                }
            }

            // Face/vertex pairs

            incidentVerts = (meshIdA == meshIdB &&
                (triPair.faceVertIdsB.ids[i] == triPair.faceVertIdsA.ids[0] ||
                    triPair.faceVertIdsB.ids[i] == triPair.faceVertIdsA.ids[1] ||
                    triPair.faceVertIdsB.ids[i] == triPair.faceVertIdsA.ids[2]));

            if (triPair.triB.OwnsVert(i)
                && !incidentVerts
                && dot(triPair.triB.Pos(i) - triPair.triA.pos0, cross(triPair.triA.pos1 - triPair.triA.pos0, triPair.triA.pos2 - triPair.triA.pos0)) > 0.0f // test vert on outside side
                )
            {
                FmFaceVertexPairData fvPair;
                fvPair.fApos[0] = triPair.triA.Pos(0);
                fvPair.fAvel[0] = triPair.triA.Vel(0);
                fvPair.fApos[1] = triPair.triA.Pos(1);
                fvPair.fAvel[1] = triPair.triA.Vel(1);
                fvPair.fApos[2] = triPair.triA.Pos(2);
                fvPair.fAvel[2] = triPair.triA.Vel(2);
                fvPair.vBpos = triPair.triB.Pos(i);
                fvPair.vBvel = triPair.triB.Vel(i);
                fvPair.j = i;

                FmCcdResult contact;

                FmFaceVertexCcd(&contact, fvPair, timestep, conditions);

                if (FmAcceptCcdContact(contact, collidedPair->distContactThreshold) && contact.featurePair.itype == FM_FEATURE_TYPE_FACE)
                {
                    FmFinalizeDistanceContact(collidedPair, triPair, contact);
                }
            }

            // Edge/edge pairs

            uint i1 = (i + 1) % 3;

            for (uint j = 0; j < 3; j++)
            {
                uint j1 = (j + 1) % 3;

                incidentVerts = (meshIdA == meshIdB &&
                    (triPair.faceVertIdsA.ids[i] == triPair.faceVertIdsB.ids[j] ||
                        triPair.faceVertIdsA.ids[i] == triPair.faceVertIdsB.ids[j1] ||
                        triPair.faceVertIdsA.ids[i1] == triPair.faceVertIdsB.ids[j] ||
                        triPair.faceVertIdsA.ids[i1] == triPair.faceVertIdsB.ids[j1]));

                // only test if edges not already intersecting with face
                bool edgeIntersectsFace = triPair.pairIntersects && (triPair.triIntersectionPoints.XVals[i] != 0 || triPair.triIntersectionPoints.XVals[j + 3] != 0);

                if (triPair.triA.OwnsEdge(i) && triPair.triB.OwnsEdge(j)           // only one tri pair will test this edge pair
                    && !incidentVerts
                    && !edgeIntersectsFace)
                {
                    FmEdgeEdgePairData eePair;
                    eePair.eApos[0] = triPair.triA.Pos(i);
                    eePair.eAvel[0] = triPair.triA.Vel(i);
                    eePair.eApos[1] = triPair.triA.Pos(i1);
                    eePair.eAvel[1] = triPair.triA.Vel(i1);
                    eePair.eBpos[0] = triPair.triB.Pos(j);
                    eePair.eBvel[0] = triPair.triB.Vel(j);
                    eePair.eBpos[1] = triPair.triB.Pos(j1);
                    eePair.eBvel[1] = triPair.triB.Vel(j1);
                    eePair.i = i;
                    eePair.j = j;

                    FmCcdResult contact;

                    FmEdgeEdgeCcd(&contact, eePair, timestep, conditions);

                    if (FmAcceptCcdContact(contact, collidedPair->distContactThreshold) &&
                        !((contact.featurePair.itype == FM_FEATURE_TYPE_VERTEX && !triPair.triA.OwnsVert(contact.featurePair.i0))
                            ||
                            (contact.featurePair.jtype == FM_FEATURE_TYPE_VERTEX && !triPair.triB.OwnsVert(contact.featurePair.j0))))
                    {
                        uint bit = 1 << (contact.featurePair.i0 * 3 + contact.featurePair.j0);

                        uint found = 0;
                        if (contact.featurePair.itype == FM_FEATURE_TYPE_VERTEX && contact.featurePair.jtype == FM_FEATURE_TYPE_VERTEX)
                        {
                            found = vertVertFound & bit;
                            vertVertFound |= bit;
                        }
                        else if (contact.featurePair.itype == FM_FEATURE_TYPE_VERTEX && contact.featurePair.jtype == FM_FEATURE_TYPE_EDGE)
                        {
                            found = vertEdgeFound & bit;
                            vertEdgeFound |= bit;
                        }
                        else if (contact.featurePair.itype == FM_FEATURE_TYPE_EDGE && contact.featurePair.jtype == FM_FEATURE_TYPE_VERTEX)
                        {
                            found = edgeVertFound & bit;
                            edgeVertFound |= bit;
                        }

                        if (!found)
                        {
                            FmFinalizeDistanceContact(collidedPair, triPair, contact);
                        }
                    }
                }
            }
        }
    }
#endif

#if FM_SOA_TRI_INTERSECTION
    void FmGenerateContactsFromMeshCollisionTriPairs(FmCollidedObjectPair* objectPair)
    {
#if 0
        for (uint pairIdx = 0; pairIdx < objectPair->temps.numMeshCollisionTriPairs; pairIdx++)
        {
            FmMeshCollisionTriPair& pair = objectPair->temps.meshCollisionTriPairs[pairIdx];

            FmGenerateContacts(objectPair, pair.exteriorFaceIdA, pair.exteriorFaceIdB, true);
        }
#else

        typedef FmSoaTypes SoaTypes;

        FmTetMesh* meshA = objectPair->tetMeshA;
        FmTetMesh* meshB = objectPair->tetMeshB;
        FmVolumeContactWorkspace* volContactWorkspace = objectPair->temps.volContactWorkspace;
        FmVector3 volContactCenter = objectPair->volContactCenter;

        // Traverse mesh collision pairs and gather groups for SoA processing
        FmSoaTriIntersectionPoints<SoaTypes> soaTriIntersectionPoints;
        SoaTypes::SoaVector3 soaTriPosA[3];
        SoaTypes::SoaVector3 soaTriPosB[3];
        soaTriPosA[0] = soaTriPosA[1] = soaTriPosA[2] = SoaTypes::SoaVector3(0.0f);
        soaTriPosB[0] = soaTriPosB[1] = soaTriPosB[2] = SoaTypes::SoaVector3(0.0f);
        FM_ALIGN(16) FmVector3 aosTriPosA0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosTriPosA1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosTriPosA2[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosTriPosB0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosTriPosB1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosTriPosB2[SoaTypes::width] FM_ALIGN_END(16);

        uint groupPairIndices[SoaTypes::width];
        bool groupPairIntersects[SoaTypes::width];
        FmTriIntersectionPoints* groupTriIntersectionPoints[SoaTypes::width];
        uint groupNumPairs = 0;

        for (uint sliceIdx = 0; sliceIdx < SoaTypes::width; sliceIdx++)
        {
            aosTriPosA0[sliceIdx] = FmInitVector3(0.0f);
            aosTriPosA1[sliceIdx] = FmInitVector3(0.0f);
            aosTriPosA2[sliceIdx] = FmInitVector3(0.0f);
            aosTriPosB0[sliceIdx] = FmInitVector3(0.0f);
            aosTriPosB1[sliceIdx] = FmInitVector3(0.0f);
            aosTriPosB2[sliceIdx] = FmInitVector3(0.0f);
        }

        for (uint pairIdx = 0; pairIdx < objectPair->temps.numMeshCollisionTriPairs; pairIdx++)
        {
            FmMeshCollisionTriPair& triPair = objectPair->temps.meshCollisionTriPairs[pairIdx];

            // Gather positions and velocities of exterior faces
            FmExteriorFace& exteriorFaceA = meshA->exteriorFaces[triPair.exteriorFaceIdA];

            triPair.tetIdA = exteriorFaceA.tetId;

            FmTetVertIds tetVertIdsA = meshA->tetsVertIds[triPair.tetIdA];

            FmGetFaceVertIds(&triPair.faceVertIdsA, exteriorFaceA.faceId, tetVertIdsA);
            FmGetFaceTetCorners(&triPair.tetCornersA, exteriorFaceA.faceId);

            triPair.triA.id = exteriorFaceA.id;

            FmVector3 tetPositionsA[4];
            tetPositionsA[0] = meshA->vertsPos[tetVertIdsA.ids[0]];
            tetPositionsA[1] = meshA->vertsPos[tetVertIdsA.ids[1]];
            tetPositionsA[2] = meshA->vertsPos[tetVertIdsA.ids[2]];
            tetPositionsA[3] = meshA->vertsPos[tetVertIdsA.ids[3]];

            FmVector3 vertPosA0 = meshA->vertsPos[triPair.faceVertIdsA.ids[0]];
            FmVector3 vertPosA1 = meshA->vertsPos[triPair.faceVertIdsA.ids[1]];
            FmVector3 vertPosA2 = meshA->vertsPos[triPair.faceVertIdsA.ids[2]];

            FmVector3 vertVelA0 = meshA->vertsVel[triPair.faceVertIdsA.ids[0]];
            FmVector3 vertVelA1 = meshA->vertsVel[triPair.faceVertIdsA.ids[1]];
            FmVector3 vertVelA2 = meshA->vertsVel[triPair.faceVertIdsA.ids[2]];

            triPair.triA.pos0 = vertPosA0 - volContactCenter;
            triPair.triA.pos1 = vertPosA1 - volContactCenter;
            triPair.triA.pos2 = vertPosA2 - volContactCenter;
            triPair.triA.vel0 = vertVelA0;
            triPair.triA.vel1 = vertVelA1;
            triPair.triA.vel2 = vertVelA2;

            FmExteriorFace& exteriorFaceB = meshB->exteriorFaces[triPair.exteriorFaceIdB];

            triPair.tetIdB = exteriorFaceB.tetId;

            FmTetVertIds tetVertIdsB = meshB->tetsVertIds[triPair.tetIdB];

            FmGetFaceVertIds(&triPair.faceVertIdsB, exteriorFaceB.faceId, tetVertIdsB);
            FmGetFaceTetCorners(&triPair.tetCornersB, exteriorFaceB.faceId);

            triPair.triB.id = exteriorFaceB.id;

            FmVector3 tetPositionsB[4];
            tetPositionsB[0] = meshB->vertsPos[tetVertIdsB.ids[0]];
            tetPositionsB[1] = meshB->vertsPos[tetVertIdsB.ids[1]];
            tetPositionsB[2] = meshB->vertsPos[tetVertIdsB.ids[2]];
            tetPositionsB[3] = meshB->vertsPos[tetVertIdsB.ids[3]];

            triPair.isTetInverted = FmIsTetInverted(tetPositionsA) || FmIsTetInverted(tetPositionsB);

            FmVector3 vertPosB0 = meshB->vertsPos[triPair.faceVertIdsB.ids[0]];
            FmVector3 vertPosB1 = meshB->vertsPos[triPair.faceVertIdsB.ids[1]];
            FmVector3 vertPosB2 = meshB->vertsPos[triPair.faceVertIdsB.ids[2]];

            FmVector3 vertVelB0 = meshB->vertsVel[triPair.faceVertIdsB.ids[0]];
            FmVector3 vertVelB1 = meshB->vertsVel[triPair.faceVertIdsB.ids[1]];
            FmVector3 vertVelB2 = meshB->vertsVel[triPair.faceVertIdsB.ids[2]];

            triPair.triB.pos0 = vertPosB0 - volContactCenter;
            triPair.triB.pos1 = vertPosB1 - volContactCenter;
            triPair.triB.pos2 = vertPosB2 - volContactCenter;
            triPair.triB.vel0 = vertVelB0;
            triPair.triB.vel1 = vertVelB1;
            triPair.triB.vel2 = vertVelB2;

            // Test for intersection
            triPair.pairIntersects = false;

            FmVector3 minPosA = min(triPair.triA.pos0, min(triPair.triA.pos1, triPair.triA.pos2));
            FmVector3 maxPosA = max(triPair.triA.pos0, max(triPair.triA.pos1, triPair.triA.pos2));
            FmVector3 minPosB = min(triPair.triB.pos0, min(triPair.triB.pos1, triPair.triB.pos2));
            FmVector3 maxPosB = max(triPair.triB.pos0, max(triPair.triB.pos1, triPair.triB.pos2));

            if (FmAabbOverlap(minPosA, maxPosA, minPosB, maxPosB)
#if FM_TRI_REJECTION_TEST
                && !FmRejectTriOverlap(
                    triPair.triA.pos0, triPair.triA.pos1, triPair.triA.pos2,
                    triPair.triB.pos0, triPair.triB.pos1, triPair.triB.pos2, 1.0e-4f)
#endif
                )
            {
                groupPairIndices[groupNumPairs] = pairIdx;
                groupTriIntersectionPoints[groupNumPairs] = &triPair.triIntersectionPoints;
                aosTriPosA0[groupNumPairs] = triPair.triA.pos0;
                aosTriPosA1[groupNumPairs] = triPair.triA.pos1;
                aosTriPosA2[groupNumPairs] = triPair.triA.pos2;
                aosTriPosB0[groupNumPairs] = triPair.triB.pos0;
                aosTriPosB1[groupNumPairs] = triPair.triB.pos1;
                aosTriPosB2[groupNumPairs] = triPair.triB.pos2;
                groupNumPairs++;

                if (groupNumPairs >= SoaTypes::width)
                {
                    FmSetSoaFromAlignedAos(&soaTriPosA[0], aosTriPosA0);
                    FmSetSoaFromAlignedAos(&soaTriPosA[1], aosTriPosA1);
                    FmSetSoaFromAlignedAos(&soaTriPosA[2], aosTriPosA2);
                    FmSetSoaFromAlignedAos(&soaTriPosB[0], aosTriPosB0);
                    FmSetSoaFromAlignedAos(&soaTriPosB[1], aosTriPosB1);
                    FmSetSoaFromAlignedAos(&soaTriPosB[2], aosTriPosB2);

                    SoaTypes::SoaBool soaPairIntersects = FmComputeTriIntersection<SoaTypes>(&soaTriIntersectionPoints, soaTriPosA, soaTriPosB);
                    FmConvertSoaToAos(groupPairIntersects, groupTriIntersectionPoints, soaPairIntersects, soaTriIntersectionPoints, groupNumPairs);
                    for (uint i = 0; i < groupNumPairs; i++)
                    {
                        FmMeshCollisionTriPair& groupPair = objectPair->temps.meshCollisionTriPairs[groupPairIndices[i]];
                        groupPair.pairIntersects = groupPairIntersects[i];
                    }

                    groupNumPairs = 0;
                }
            }
        }

        if (groupNumPairs > 0)
        {
            FmSetSoaFromAlignedAos(&soaTriPosA[0], aosTriPosA0);
            FmSetSoaFromAlignedAos(&soaTriPosA[1], aosTriPosA1);
            FmSetSoaFromAlignedAos(&soaTriPosA[2], aosTriPosA2);
            FmSetSoaFromAlignedAos(&soaTriPosB[0], aosTriPosB0);
            FmSetSoaFromAlignedAos(&soaTriPosB[1], aosTriPosB1);
            FmSetSoaFromAlignedAos(&soaTriPosB[2], aosTriPosB2);

            SoaTypes::SoaBool soaPairIntersects = FmComputeTriIntersection<SoaTypes>(&soaTriIntersectionPoints, soaTriPosA, soaTriPosB);
            FmConvertSoaToAos(groupPairIntersects, groupTriIntersectionPoints, soaPairIntersects, soaTriIntersectionPoints, groupNumPairs);
            for (uint i = 0; i < groupNumPairs; i++)
            {
                FmMeshCollisionTriPair& groupPair = objectPair->temps.meshCollisionTriPairs[groupPairIndices[i]];
                groupPair.pairIntersects = groupPairIntersects[i];
            }

            groupNumPairs = 0;
        }

#if FM_SOA_TRI_CCD
        typedef FmSoaTypes CcdSoaTypes;
        FmFeaturePairVFBatch<CcdSoaTypes> vfBatch;
        FmFeaturePairFVBatch<CcdSoaTypes> fvBatch;
        FmFeaturePairEEBatch<CcdSoaTypes> eeBatch;

        bool isSelfCollision = (meshA == meshB);
        uint numTriPairs = objectPair->temps.numMeshCollisionTriPairs;

        if (isSelfCollision)
        {
            for (uint pairIdx = 0; pairIdx < numTriPairs; pairIdx++)
            {
                FmMeshCollisionTriPair& triPair = objectPair->temps.meshCollisionTriPairs[pairIdx];

                // Apply contributions to volume and gradient from tri intersection
                if (triPair.pairIntersects)
                {
                    FmSelfCollApplyTriIntersectionVolumeAndGradientContributions(objectPair, triPair, volContactWorkspace);
                }

                if (triPair.exteriorFaceIdA < triPair.exteriorFaceIdB)
                {
                    FmGenerateCcdContacts(objectPair, triPair, vfBatch, fvBatch, eeBatch);
                }
            }
        }
        else
        {
            for (uint pairIdx = 0; pairIdx < numTriPairs; pairIdx++)
            {
                FmMeshCollisionTriPair& triPair = objectPair->temps.meshCollisionTriPairs[pairIdx];

                // Apply contributions to volume and gradient from tri intersection
                if (triPair.pairIntersects)
                {
                    FmApplyTriIntersectionVolumeAndGradientContributions(objectPair, triPair, volContactWorkspace);
                }

                FmGenerateCcdContacts(objectPair, triPair, vfBatch, fvBatch, eeBatch);
            }
        }

        FmSoaCcdTerminationConditions<CcdSoaTypes> soaConditions;
        soaConditions.impactGap = objectPair->distContactBias;
        soaConditions.contactGap = objectPair->distContactThreshold;
        soaConditions.maxIterations = 4;

        if (vfBatch.numPairs)
        {
            FmProcessVFBatch(objectPair, vfBatch, soaConditions);
        }

        if (fvBatch.numPairs)
        {
            FmProcessFVBatch(objectPair, fvBatch, soaConditions);
        }

        if (eeBatch.numPairs)
        {
            FmProcessEEBatch(objectPair, eeBatch, soaConditions);
        }

#else
        for (uint pairIdx = 0; pairIdx < objectPair->temps.numMeshCollisionTriPairs; pairIdx++)
        {
            FmMeshCollisionTriPair& triPair = objectPair->temps.meshCollisionTriPairs[pairIdx];

            // Apply contributions to volume and gradient from tri intersection
            if (triPair.pairIntersects && volContactWorkspace)
            {
                FmApplyTriIntersectionVolumeAndGradientContributions(objectPair, triPair, volContactWorkspace);
            }

            FmGenerateCcdContacts(objectPair, triPair);
        }
#endif
#endif
    }
#endif

#if !FM_SOA_TRI_INTERSECTION
    void FmGenerateContacts(FmCollidedObjectPair* collidedPair, uint exteriorFaceIdA, uint exteriorFaceIdB)
    {
        FmTetMesh* meshA = collidedPair->tetMeshA;
        FmTetMesh* meshB = collidedPair->tetMeshB;
        FmVolumeContactWorkspace* volContactWorkspace = collidedPair->temps.volContactWorkspace;
        FmVector3 volContactCenter = collidedPair->volContactCenter;

        FmMeshCollisionTriPair triPair;
        triPair.exteriorFaceIdA = exteriorFaceIdA;
        triPair.exteriorFaceIdB = exteriorFaceIdB;

        // Gather positions and velocities of exterior faces
        FmExteriorFace& exteriorFaceA = meshA->exteriorFaces[exteriorFaceIdA];

        triPair.tetIdA = exteriorFaceA.tetId;

        FmGetFaceTetCorners(&triPair.tetCornersA, exteriorFaceA.faceId);

        FmGetFaceVertIds(&triPair.faceVertIdsA, exteriorFaceA.faceId, meshA->tetsVertIds[triPair.tetIdA]);

        triPair.triA.id = exteriorFaceA.id;
        triPair.triA.pos0 = meshA->vertsPos[triPair.faceVertIdsA.ids[0]] - volContactCenter;
        triPair.triA.pos1 = meshA->vertsPos[triPair.faceVertIdsA.ids[1]] - volContactCenter;
        triPair.triA.pos2 = meshA->vertsPos[triPair.faceVertIdsA.ids[2]] - volContactCenter;
        triPair.triA.vel0 = meshA->vertsVel[triPair.faceVertIdsA.ids[0]];
        triPair.triA.vel1 = meshA->vertsVel[triPair.faceVertIdsA.ids[1]];
        triPair.triA.vel2 = meshA->vertsVel[triPair.faceVertIdsA.ids[2]];

        FmExteriorFace& exteriorFaceB = meshB->exteriorFaces[exteriorFaceIdB];

        triPair.tetIdB = exteriorFaceB.tetId;

        FmGetFaceTetCorners(&triPair.tetCornersB, exteriorFaceB.faceId);

        FmGetFaceVertIds(&triPair.faceVertIdsB, exteriorFaceB.faceId, meshB->tetsVertIds[triPair.tetIdB]);

        triPair.triB.id = exteriorFaceB.id;
        triPair.triB.pos0 = meshB->vertsPos[triPair.faceVertIdsB.ids[0]] - volContactCenter;
        triPair.triB.pos1 = meshB->vertsPos[triPair.faceVertIdsB.ids[1]] - volContactCenter;
        triPair.triB.pos2 = meshB->vertsPos[triPair.faceVertIdsB.ids[2]] - volContactCenter;
        triPair.triB.vel0 = meshB->vertsVel[triPair.faceVertIdsB.ids[0]];
        triPair.triB.vel1 = meshB->vertsVel[triPair.faceVertIdsB.ids[1]];
        triPair.triB.vel2 = meshB->vertsVel[triPair.faceVertIdsB.ids[2]];

        // Test for intersection
        triPair.pairIntersects = false;

        FmVector3 minPosA = min(triPair.triA.pos0, min(triPair.triA.pos1, triPair.triA.pos2));
        FmVector3 maxPosA = max(triPair.triA.pos0, max(triPair.triA.pos1, triPair.triA.pos2));
        FmVector3 minPosB = min(triPair.triB.pos0, min(triPair.triB.pos1, triPair.triB.pos2));
        FmVector3 maxPosB = max(triPair.triB.pos0, max(triPair.triB.pos1, triPair.triB.pos2));

        if (FmAabbOverlap(minPosA, maxPosA, minPosB, maxPosB))
        {
            triPair.pairIntersects = FmComputeTriIntersection(&triPair.triIntersectionPoints, &triPair.triA.pos0, &triPair.triB.pos0);
        }

        // Apply contributions to volume and gradient from tri intersection
        bool isSelfCollision = (meshA == meshB);

        if (isSelfCollision)
        {
            if (triPair.pairIntersects)
            {
                FmSelfCollApplyTriIntersectionVolumeAndGradientContributions(collidedPair, triPair, volContactWorkspace);
            }

            if (triPair.exteriorFaceIdA < triPair.exteriorFaceIdB)
            {
                FmGenerateCcdContacts(collidedPair, triPair);
            }
        }
        else
        {
            if (triPair.pairIntersects)
            {
                FmApplyTriIntersectionVolumeAndGradientContributions(collidedPair, triPair, volContactWorkspace);
            }

            FmGenerateCcdContacts(collidedPair, triPair);
        }
    }
#endif

    void FmInitFindContacts(FmConstraintsBuffer* constraintsBuffer, FmCollisionReport* collisionReport)
    {
        FmAtomicWrite(&constraintsBuffer->foundContacts.val, 0);
        FmAtomicWrite(&constraintsBuffer->numDistanceContacts.val, 0);
        FmAtomicWrite(&constraintsBuffer->numVolumeContactVerts.val, 0);
        FmAtomicWrite(&constraintsBuffer->numVolumeContacts.val, 0);
        FmAtomicWrite(&constraintsBuffer->exceededVolumeContactVerts.val, 0);
        FmAtomicWrite(&collisionReport->numDistanceContacts.val, 0);
        FmAtomicWrite(&collisionReport->numVolumeContacts.val, 0);
    }

    uint FmFindDistanceContacts(FmCollidedObjectPair* objectPair)
    {
        FmCollideHierarchies(objectPair);

        return objectPair->numDistanceContacts;
    }

    // Create volume contact collecting the vertices of each mesh affecting the intersection volume (nonzero dVdp values)
    uint FmFinalizeVolumeContact(FmCollidedObjectPair* objectPair)
    {
        FmTetMesh* meshA = objectPair->tetMeshA;
        FmTetMesh* meshB = objectPair->tetMeshB;
        FmConstraintsBuffer* constraintsBuffer = objectPair->constraintsBuffer;
        FmVolumeContactWorkspace* volContactWorkspace = objectPair->temps.volContactWorkspace;
        FmVector3 volContactCenter = objectPair->volContactCenter;
        FmVector3 volContactNormal = objectPair->volContactNormal;
        float volContactV = objectPair->volContactV;
        float volContactBias = objectPair->volContactBias;
        float volContactThreshold = objectPair->volContactThreshold;
        float distContactBias = objectPair->distContactBias;

        uint numEntriesA = FmGetNumEntries(volContactWorkspace->meshAVertSets);
        uint numEntriesB = FmGetNumEntries(volContactWorkspace->meshBVertSets);

        uint numContacts = 0;

        float lenSqrVolContactNormal = lengthSqr(volContactNormal);
        const float lenSqrThreshold = 1.0e-12f;

        bool isSelfCollision = (meshA == meshB);
        FmVolumeContactVertSetExpandable& vertSetA = volContactWorkspace->meshAVertSets;
        FmVolumeContactVertSetExpandable& vertSetB = isSelfCollision? vertSetA : volContactWorkspace->meshBVertSets;

#if FM_SURFACE_INTERSECTION_CONTACTS
        if (volContactV < volContactThreshold
            && lenSqrVolContactNormal > lenSqrThreshold)
        {
            uint numIntersectingFacePairs = volContactWorkspace->numIntersectingFacePairs;
            for (uint pairIdx = 0; pairIdx < numIntersectingFacePairs; pairIdx++)
            {
                FmIntersectingFacePair& facePair = volContactWorkspace->intersectingFacePairs[pairIdx];
                FmExteriorFace& exteriorFaceA = meshA->exteriorFaces[facePair.exteriorFaceIdA];
                FmExteriorFace& exteriorFaceB = meshB->exteriorFaces[facePair.exteriorFaceIdB];

                float tetAFrictionCoeff = meshA->tetsFrictionCoeff[exteriorFaceA.tetId];
                float tetBFrictionCoeff = meshB->tetsFrictionCoeff[exteriorFaceB.tetId];

                FmTetVertIds tetVertIdsA = meshA->tetsVertIds[exteriorFaceA.tetId];
                FmTetVertIds tetVertIdsB = meshB->tetsVertIds[exteriorFaceB.tetId];

                FmFaceVertIds faceVertIdsA, faceVertIdsB;
                FmGetFaceVertIds(&faceVertIdsA, exteriorFaceA.faceId, tetVertIdsA);
                FmGetFaceVertIds(&faceVertIdsB, exteriorFaceB.faceId, tetVertIdsB);

                FmFaceVertIds tetCornersA;
                FmGetFaceTetCorners(&tetCornersA, exteriorFaceA.faceId);

                FmFaceVertIds tetCornersB;
                FmGetFaceTetCorners(&tetCornersB, exteriorFaceB.faceId);

                FmVector3 facePosA0 = meshA->vertsPos[faceVertIdsA.ids[0]] - volContactCenter;
                FmVector3 facePosA1 = meshA->vertsPos[faceVertIdsA.ids[1]] - volContactCenter;
                FmVector3 facePosA2 = meshA->vertsPos[faceVertIdsA.ids[2]] - volContactCenter;

                float baryA[3];
                FmComputeTriBarycentricCoords(
                    baryA, facePair.pos,
                    facePosA0,
                    facePosA1,
                    facePosA2);

                FmVector3 facePosB0 = meshB->vertsPos[faceVertIdsB.ids[0]] - volContactCenter;
                FmVector3 facePosB1 = meshB->vertsPos[faceVertIdsB.ids[1]] - volContactCenter;
                FmVector3 facePosB2 = meshB->vertsPos[faceVertIdsB.ids[2]] - volContactCenter;

                float baryB[3];
                FmComputeTriBarycentricCoords(
                    baryB, facePair.pos,
                    facePosB0,
                    facePosB1,
                    facePosB2);

                // Create tet pair from tri contact
                FmDistanceContactPairInfo contactPairInfo;
                contactPairInfo.idInObjectPair = objectPair->numDistanceContacts;
                contactPairInfo.objectIdA = objectPair->objectAId;
                contactPairInfo.objectIdB = objectPair->objectBId;

                FmDistanceContact contact;
                contact.posBaryA[0] = 0.0f;
                contact.posBaryA[1] = 0.0f;
                contact.posBaryA[2] = 0.0f;
                contact.posBaryA[3] = 0.0f;
                contact.posBaryB[0] = 0.0f;
                contact.posBaryB[1] = 0.0f;
                contact.posBaryB[2] = 0.0f;
                contact.posBaryB[3] = 0.0f;
                contact.tetIdA = exteriorFaceA.tetId;
                contact.tetIdB = exteriorFaceB.tetId;
                contact.frictionCoeff = FmMinFloat(tetAFrictionCoeff, tetBFrictionCoeff);

                contact.posBaryA[tetCornersA.ids[0]] = baryA[0];
                contact.posBaryA[tetCornersA.ids[1]] = baryA[1];
                contact.posBaryA[tetCornersA.ids[2]] = baryA[2];

                contact.posBaryB[tetCornersB.ids[0]] = baryB[0];
                contact.posBaryB[tetCornersB.ids[1]] = baryB[1];
                contact.posBaryB[tetCornersB.ids[2]] = baryB[2];

                FmVector3 vertVelA0 = meshA->vertsVel[tetVertIdsA.ids[0]];
                FmVector3 vertVelA1 = meshA->vertsVel[tetVertIdsA.ids[1]];
                FmVector3 vertVelA2 = meshA->vertsVel[tetVertIdsA.ids[2]];
                FmVector3 vertVelA3 = meshA->vertsVel[tetVertIdsA.ids[3]];

                FmVector3 vertVelB0 = meshB->vertsVel[tetVertIdsB.ids[0]];
                FmVector3 vertVelB1 = meshB->vertsVel[tetVertIdsB.ids[1]];
                FmVector3 vertVelB2 = meshB->vertsVel[tetVertIdsB.ids[2]];
                FmVector3 vertVelB3 = meshB->vertsVel[tetVertIdsB.ids[3]];

                FmVector3 posA = facePair.pos + volContactCenter;
                FmVector3 velA = FmInterpolate(contact.posBaryA, vertVelA0, vertVelA1, vertVelA2, vertVelA3);

                FmVector3 posB = posA;
                FmVector3 velB = FmInterpolate(contact.posBaryB, vertVelB0, vertVelB1, vertVelB2, vertVelB3);

                if (objectPair->objectAId & FM_RB_FLAG)
                {
                    FmRigidBody* rigidBodyA = (FmRigidBody*)objectPair->objectA;

                    FmVector3 comToContact = posA - rigidBodyA->state.pos;
                    contact.comToPosA[0] = comToContact.x;
                    contact.comToPosA[1] = comToContact.y;
                    contact.comToPosA[2] = comToContact.z;
                }

                if (objectPair->objectBId & FM_RB_FLAG)
                {
                    FmRigidBody* rigidBodyB = (FmRigidBody*)objectPair->objectB;

                    FmVector3 comToContact = posB - rigidBodyB->state.pos;
                    contact.comToPosB[0] = comToContact.x;
                    contact.comToPosB[1] = comToContact.y;
                    contact.comToPosB[2] = comToContact.z;
                }

                FmVolumeContactVertSetElement* elementA0 = FmInsertElement(&vertSetA, faceVertIdsA.ids[0]);
                FmVolumeContactVertSetElement* elementA1 = FmInsertElement(&vertSetA, faceVertIdsA.ids[1]);
                FmVolumeContactVertSetElement* elementA2 = FmInsertElement(&vertSetA, faceVertIdsA.ids[2]);

                FmVolumeContactVertSetElement* elementB0 = FmInsertElement(&vertSetB, faceVertIdsB.ids[0]);
                FmVolumeContactVertSetElement* elementB1 = FmInsertElement(&vertSetB, faceVertIdsB.ids[1]);
                FmVolumeContactVertSetElement* elementB2 = FmInsertElement(&vertSetB, faceVertIdsB.ids[2]);

                contact.normal =
                    elementA0->dVdp * baryA[0]
                    + elementA1->dVdp * baryA[1]
                    + elementA2->dVdp * baryA[2]
                    - elementB0->dVdp * baryB[0]
                    - elementB1->dVdp * baryB[1]
                    - elementB2->dVdp * baryB[2];

                bool valid = true;
                if (lengthSqr(contact.normal) < float(1.0e-10))
                {
                    valid = false;
                }

                contact.normal = normalize(contact.normal);

                contact.normalProjDistance = dot(contact.normal, posA - posB) - distContactBias;
                contact.normalProjRelVel = dot(contact.normal, velB - velA);
                contact.tangent1 = FmGetContactTangent(contact.normal, velA, velB);

                // Set dynamic bits, used to find dependencies between contacts and size the constraint Jacobian
                uint dynamicFlagsA, movingFlagsA, dynamicFlagsB, movingFlagsB;

                if (objectPair->objectAId & FM_RB_FLAG)
                {
                    FmRigidBody* rigidBodyA = (FmRigidBody*)objectPair->objectA;
                    FmDynamicAndMovingFlags(&dynamicFlagsA, &movingFlagsA, *rigidBodyA);
                }
                else
                {
                    FmDynamicAndMovingFlags(&dynamicFlagsA, &movingFlagsA, *meshA, tetVertIdsA, contact.posBaryA);
                }

                if (objectPair->objectBId & FM_RB_FLAG)
                {
                    FmRigidBody* rigidBodyB = (FmRigidBody*)objectPair->objectB;
                    FmDynamicAndMovingFlags(&dynamicFlagsB, &movingFlagsB, *rigidBodyB);
                }
                else
                {
                    FmDynamicAndMovingFlags(&dynamicFlagsB, &movingFlagsB, *meshB, tetVertIdsB, contact.posBaryB);
                }

                FmSetConstraintFlags(&contactPairInfo.flags, &contactPairInfo.dynamicFlags, &contact.movingFlags, dynamicFlagsA, dynamicFlagsB, movingFlagsA, movingFlagsB);

                if (contactPairInfo.dynamicFlags && valid)
                {
                    FmAddTempDistanceContact(objectPair, &contactPairInfo, &contact, tetVertIdsA, tetVertIdsB, dynamicFlagsA, dynamicFlagsB, NULL);
                }
            }
        }
#endif

        bool spaceInVolumeContactVerts = (FmAtomicRead(&constraintsBuffer->numVolumeContactVerts.val) + numEntriesA + numEntriesB <= constraintsBuffer->maxVolumeContactVerts);

        // For warnings report
        if (!spaceInVolumeContactVerts)
        {
            FmAtomicWrite(&constraintsBuffer->exceededVolumeContactVerts.val, 1);
        }

        if (volContactV <= volContactThreshold
            && lenSqrVolContactNormal > lenSqrThreshold   // Small values can cause numerical issue in solver.
            && spaceInVolumeContactVerts
            && FmAtomicRead(&constraintsBuffer->numVolumeContacts.val) + 1 <= constraintsBuffer->maxVolumeContacts
            )
        {
            // Applying a scaling so that sum of dVdp vectors is closer to 1.0, which affects scale 
            // of block diagonal inverse done in the constraint solves.  
            // Must apply same scaling to rhs of constraint equation: J * vel = (vol_bias - vol)/dt.
            float scale = 2.0f / sqrtf(lenSqrVolContactNormal);

            // Atomically claim space for contact vertex data.
            // TODO: Make this allocation tighter or avoid when possible.
            uint numAllocVerts = numEntriesA + numEntriesB;
            uint numTotalVerts = FmAtomicAdd(&constraintsBuffer->numVolumeContactVerts.val, numAllocVerts);
            uint volContactVertStartIdx = numTotalVerts - numAllocVerts;

            if (numTotalVerts <= constraintsBuffer->maxVolumeContactVerts)
            {
                // Compute normal and tangents for volume contact
                FmVector3 backupDir = FmInitVector3(0.0f, 1.0f, 0.0f);
                volContactNormal = FmSafeNormalize(volContactNormal, backupDir);

                float V = 0.0f;

                FmVector3 velRel = FmInitVector3(0.0f, 0.0f, 0.0f);

                uint finalVertsStartOffsetA = volContactVertStartIdx;
                FmVolumeContactVert* volContactVerts = &constraintsBuffer->volumeContactVerts[finalVertsStartOffsetA];
                uint numVolVertsA = 0;
                uint numDynamicVertsA = 0;
                uint numElements = volContactWorkspace->meshAVertSets.numElementsUsed;
                for (uint elemIdx = 0; elemIdx < numElements; elemIdx++)
                {
                    FmVolumeContactVertSetElement& element = volContactWorkspace->meshAVertSets.elements[elemIdx];
                    if (element.key != FM_INVALID_ID)
                    {
                        FmVolumeContactVert& volContactVert = volContactVerts[numVolVertsA];
                        volContactVert = FmVolumeContactVert();

                        FmVector3 dVdp = element.dVdp;
                        FmVector3 centerToVert = element.centerToVert;
                        uint vertId = element.key;
                        FmVector3 vel = meshA->vertsVel[vertId];

                        volContactVert.dVdp = dVdp * scale;
                        volContactVert.centerToVert = centerToVert;
                        volContactVert.vertId = vertId;
                        volContactVert.tangent = FmGetContactTangent(FmSafeNormalize(dVdp, backupDir), -vel);

                        bool isDynamic = FM_NOT_SET(meshA->vertsFlags[vertId], FM_VERT_FLAG_KINEMATIC);
                        bool isMoving = !FmIsZero(vel);
                        bool isNonzeroGrad = !FmIsZero(dVdp);

                        volContactVert.dynamic = isDynamic;
                        volContactVert.moving = isMoving;

                        numVolVertsA += (uint)(isNonzeroGrad && (isDynamic || isMoving));
                        numDynamicVertsA += (uint)(isNonzeroGrad && isDynamic);

                        velRel -= vel * dot(volContactNormal, dVdp);

                        V += element.V;
                    }
                }

                FM_ASSERT(numVolVertsA <= numEntriesA);

                uint finalVertsStartOffsetB = volContactVertStartIdx + numVolVertsA;
                volContactVerts = &constraintsBuffer->volumeContactVerts[finalVertsStartOffsetB];
                uint numVolVertsB = 0;
                uint numDynamicVertsB = 0;
                numElements = volContactWorkspace->meshBVertSets.numElementsUsed;
                for (uint elemIdx = 0; elemIdx < numElements; elemIdx++)
                {
                    FmVolumeContactVertSetElement& element = volContactWorkspace->meshBVertSets.elements[elemIdx];
                    if (element.key != FM_INVALID_ID)
                    {
                        FmVolumeContactVert& volContactVert = volContactVerts[numVolVertsB];
                        volContactVert = FmVolumeContactVert();

                        FmVector3 dVdp = element.dVdp;
                        FmVector3 centerToVert = element.centerToVert;
                        uint vertId = element.key;
                        FmVector3 vel = meshB->vertsVel[vertId];

                        volContactVert.dVdp = dVdp * scale;
                        volContactVert.centerToVert = centerToVert;
                        volContactVert.vertId = vertId;
                        volContactVert.tangent = FmGetContactTangent(FmSafeNormalize(dVdp, backupDir), -vel);

                        bool isDynamic = FM_NOT_SET(meshB->vertsFlags[vertId], FM_VERT_FLAG_KINEMATIC);
                        bool isMoving = !FmIsZero(vel);
                        bool isNonzeroGrad = !FmIsZero(dVdp);

                        volContactVert.dynamic = isDynamic;
                        volContactVert.moving = isMoving;

                        numVolVertsB += (uint)(isNonzeroGrad && (isDynamic || isMoving));
                        numDynamicVertsB += (uint)(isNonzeroGrad && isDynamic);

                        velRel -= vel * dot(volContactNormal, dVdp);

                        V += element.V;
                    }
                }

                FM_ASSERT(numVolVertsB <= numEntriesB);

                if (numDynamicVertsA + numDynamicVertsB > 0)
                {
                    if (FmAtomicRead(&constraintsBuffer->numVolumeContacts.val) < constraintsBuffer->maxVolumeContacts)
                    {
                        // Atomically claim space for contact
                        uint volContactIdx = FmAtomicIncrement(&constraintsBuffer->numVolumeContacts.val) - 1;

                        if (volContactIdx < constraintsBuffer->maxVolumeContacts)
                        {
                            objectPair->numVolumeContacts = 1;

                            if (numDynamicVertsA > 0)
                            {
                                objectPair->objectAContactWithDynamicVerts = true;
                            }
                            if (numDynamicVertsB > 0)
                            {
                                objectPair->objectBContactWithDynamicVerts = true;
                            }

                            FmVolumeContact& contact = constraintsBuffer->volumeContacts[volContactIdx];

                            FmVector3 tangent1 = FmGetContactTangent(volContactNormal, velRel);

                            contact = FmVolumeContact();

                            contact.idInObjectPair = objectPair->numVolumeContacts;
                            objectPair->numVolumeContacts++;

                            contact.objectIdA = meshA->objectId;
                            contact.objectIdB = meshB->objectId;
                            contact.V = (V - volContactBias) * scale;
                            contact.normalProjRelVel = dot(volContactNormal, velRel);
                            contact.volVertsStartA = finalVertsStartOffsetA;
                            contact.volVertsStartB = finalVertsStartOffsetB;
                            contact.numVolVertsA = numVolVertsA;
                            contact.numVolVertsB = numVolVertsB;
                            contact.numDynamicVertsA = numDynamicVertsA;
                            contact.numDynamicVertsB = numDynamicVertsB;
                            contact.frictionCoeff = FmMinFloat(meshA->frictionCoeff, meshB->frictionCoeff); // TODO: base on tet friction coeffs
                            contact.normal = volContactNormal;
                            contact.tangent1 = tangent1;

                            contact.flags |= (numDynamicVertsA == 0) ? FM_CONSTRAINT_FLAG_OBJECTA_FIXED : 0;
                            contact.flags |= (numDynamicVertsB == 0) ? FM_CONSTRAINT_FLAG_OBJECTB_FIXED : 0;

                            // Collision reporting
                            FmCollisionReport* collisionReport = objectPair->collisionReport;
                            if (collisionReport
                                && collisionReport->volumeContactBuffer
                                && FmAtomicRead(&collisionReport->numVolumeContacts.val) < collisionReport->maxVolumeContacts
                                && objectPair->numVolumeContactReports < collisionReport->maxVolumeContactsPerObjectPair
                                && contact.normalProjRelVel > collisionReport->minContactRelVel)
                            {
                                uint reportContactIdx = FmAtomicIncrement(&collisionReport->numVolumeContacts.val) - 1;
                                if (reportContactIdx < collisionReport->maxVolumeContacts)
                                {
                                    objectPair->numVolumeContactReports++;

                                    FmCollisionReportVolumeContact& reportContact = collisionReport->volumeContactBuffer[reportContactIdx];
                                    reportContact.objectIdA = contact.objectIdA;
                                    reportContact.objectIdB = contact.objectIdB;
                                    reportContact.volume = V;
                                    reportContact.normal = contact.normal;
                                }
                                else
                                {
                                    FmAtomicWrite(&collisionReport->numVolumeContacts.val, collisionReport->maxVolumeContacts);
                                }
                            }
                        }
                    }
                    else
                    {
                        FmAtomicWrite(&constraintsBuffer->numVolumeContacts.val, constraintsBuffer->maxVolumeContacts);
                    }

                    numContacts++;
                }
            }
            else
            {
                FmAtomicWrite(&constraintsBuffer->numVolumeContactVerts.val, constraintsBuffer->maxVolumeContactVerts);
                FmAtomicWrite(&constraintsBuffer->exceededVolumeContactVerts.val, 1);
            }
        }

        return numContacts;
    }

    // Accumulating the integrals for the volume contacts without 
    // creating faces.  Can sum contribution from each boundary line segment found
    // during collision detection.  Segments are formed from face/face intersections, and 
    // edge/solid intersections.  Can't easily produce the segments for the edge/solid case
    // without having all the edge/face intersections at once, but for this case 
    // can compute a contribution for each intersection point, whose sign is determined by
    // whether the point is a segment start or end.
    // 
    // To find the inside/outside status for surface verts, using a separate specialized 
    // tree traversal.

    uint FmFindContacts(FmCollidedObjectPair* objectPair)
    {
        FmTetMesh* meshA = objectPair->tetMeshA;
        FmTetMesh* meshB = objectPair->tetMeshB;
        FmVolumeContactWorkspace* volContactWorkspace = objectPair->temps.volContactWorkspace;

        // Sum the point vs. face "shadow" results to determine whether each vertex is inside other mesh.
        FmFindInsideVerts(objectPair);

        // For inside vertices, apply volume and gradient contributions to any incident faces
        FmApplyInsideVertVolumeAndGradientContributions(objectPair, meshA, volContactWorkspace->meshAVertSets, 1.0f);
        FmApplyInsideVertVolumeAndGradientContributions(objectPair, meshB, volContactWorkspace->meshBVertSets, -1.0f);

        // Collide triangles to find CCD contacts, and apply volume and gradient contributions of intersecting triangles.
        FmCollideHierarchies(objectPair);

        // Create volume contact.
        uint numVolumeContacts = FmFinalizeVolumeContact(objectPair);

        // Copy any temp contacts to global list
        if (objectPair->temps.numDistanceContacts > 0)
        {
            FmCopyTempDistanceContacts(objectPair);
        }

        return objectPair->numDistanceContacts + numVolumeContacts;
    }

    bool FmAabbSceneCollisionPlanesPotentialContact(bool results[6], const FmAabb& aabb, const FmSceneCollisionPlanes& planes, float timestep, float threshold)
    {
        FmVector3 minPos = aabb.pmin + aabb.vmin * timestep;
        FmVector3 maxPos = aabb.pmax + aabb.vmax * timestep;

        bool result0 = (minPos.x <= planes.minX + threshold);
        bool result1 = (maxPos.x >= planes.maxX - threshold);
        bool result2 = (minPos.y <= planes.minY + threshold);
        bool result3 = (maxPos.y >= planes.maxY - threshold);
        bool result4 = (minPos.z <= planes.minZ + threshold);
        bool result5 = (maxPos.z >= planes.maxZ - threshold);

        results[0] = result0;
        results[1] = result1;
        results[2] = result2;
        results[3] = result3;
        results[4] = result4;
        results[5] = result5;

        return result0 || result1 || result2 || result3 || result4 || result5;
    }

    bool FmAddPlaneContact(
        FmCollidedObjectPair* objectPair,
        uint tetId, uint tetCornerId, const FmTetVertIds& tetVertIds,
        const FmVector3& vertPos, const FmVector3& vertVel,
        const FmVector3& planeNormal, const FmVector3& planePos)
    {
        const FmTetMesh& tetMesh = *objectPair->tetMeshA;
        float distContactBias = objectPair->distContactBias;

        FmDistanceContactPairInfo contactPairInfo;
        contactPairInfo.idInObjectPair = objectPair->numDistanceContacts;
        contactPairInfo.objectIdA = tetMesh.objectId;
        contactPairInfo.objectIdB = FM_INVALID_ID;

        FmDistanceContact contact;

        contact.tetIdA = tetId;
        contact.tetIdB = FM_INVALID_ID;
        contact.normal = planeNormal;
        contact.posBaryA[0] = 0.0f;
        contact.posBaryA[1] = 0.0f;
        contact.posBaryA[2] = 0.0f;
        contact.posBaryA[3] = 0.0f;
        contact.posBaryA[tetCornerId] = 1.0f;
        contact.frictionCoeff = tetMesh.tetsFrictionCoeff[tetId];

        uint dynamicFlagsA, movingFlagsA;

        FmDynamicAndMovingFlags(&dynamicFlagsA, &movingFlagsA, tetMesh, tetVertIds, contact.posBaryA);

        FmSetConstraintFlags(&contactPairInfo.flags, &contactPairInfo.dynamicFlags, &contact.movingFlags, dynamicFlagsA, 0, movingFlagsA, 0);
        contactPairInfo.flags |= FM_CONSTRAINT_FLAG_OBJECTB_COLLISION_PLANE;

        contact.normalProjDistance = dot(vertPos - planePos, planeNormal) - distContactBias;
        contact.normalProjRelVel = -dot(planeNormal, vertVel);

        FmVector3 tangentVelBA = FmGetContactTangent(planeNormal, vertVel);

        contact.tangent1 = tangentVelBA;

        return FmAddTempDistanceContact(objectPair, &contactPairInfo, &contact, tetVertIds, tetVertIds, dynamicFlagsA, 0, NULL);
    }

    uint FmGenerateFaceCollisionPlaneContacts(
        FmCollidedObjectPair* objectPair,
        bool canCollide[6],
        uint extFaceId, const FmSceneCollisionPlanes& collisionPlanes)
    {
        (void)canCollide;
        uint numContacts = 0;

        const FmTetMesh& tetMesh = *objectPair->tetMeshA;
        const FmExteriorFace& extFace = tetMesh.exteriorFaces[extFaceId];

        float timestep = objectPair->timestep;
        float distContactThreshold = objectPair->distContactThreshold;

        uint tetId = extFace.tetId;
        FmTetVertIds tetVertIds = tetMesh.tetsVertIds[tetId];

        FmFaceVertIds extFaceVertIds;
        FmGetFaceVertIds(&extFaceVertIds, extFace.faceId, tetVertIds);

        FmFaceVertIds extFaceTetCorners;
        FmGetFaceTetCorners(&extFaceTetCorners, extFace.faceId);

        for (uint cornerId = 0; cornerId < 3; cornerId++)
        {
            if (extFace.OwnsVert(cornerId))
            {
                uint vId = extFaceVertIds.ids[cornerId];

                if (!(tetMesh.vertsFlags[vId] & FM_VERT_FLAG_KINEMATIC))
                {
                    uint tetCornerId = extFaceTetCorners.ids[cornerId];

                    FmVector3 vel = tetMesh.vertsVel[vId];
                    FmVector3 posStart = tetMesh.vertsPos[vId];
                    FmVector3 posEnd = posStart + vel * timestep;

                    if (posEnd.x <= collisionPlanes.minX + distContactThreshold)
                    {
                        numContacts += FmAddPlaneContact(objectPair, tetId, tetCornerId, tetVertIds, posStart, vel, FmInitVector3(1.0f, 0.0f, 0.0f), FmInitVector3(collisionPlanes.minX, 0.0f, 0.0f));
                    }
                    if (posEnd.x >= collisionPlanes.maxX - distContactThreshold)
                    {
                        numContacts += FmAddPlaneContact(objectPair, tetId, tetCornerId, tetVertIds, posStart, vel, FmInitVector3(-1.0f, 0.0f, 0.0f), FmInitVector3(collisionPlanes.maxX, 0.0f, 0.0f));
                    }
                    if (posEnd.y <= collisionPlanes.minY + distContactThreshold)
                    {
                        numContacts += FmAddPlaneContact(objectPair, tetId, tetCornerId, tetVertIds, posStart, vel, FmInitVector3(0.0f, 1.0f, 0.0f), FmInitVector3(0.0f, collisionPlanes.minY, 0.0f));
                    }
                    if (posEnd.y >= collisionPlanes.maxY - distContactThreshold)
                    {
                        numContacts += FmAddPlaneContact(objectPair, tetId, tetCornerId, tetVertIds, posStart, vel, FmInitVector3(0.0f, -1.0f, 0.0f), FmInitVector3(0.0f, collisionPlanes.maxY, 0.0f));
                    }
                    if (posEnd.z <= collisionPlanes.minZ + distContactThreshold)
                    {
                        numContacts += FmAddPlaneContact(objectPair, tetId, tetCornerId, tetVertIds, posStart, vel, FmInitVector3(0.0f, 0.0f, 1.0f), FmInitVector3(0.0f, 0.0f, collisionPlanes.minZ));
                    }
                    if (posEnd.z >= collisionPlanes.maxZ - distContactThreshold)
                    {
                        numContacts += FmAddPlaneContact(objectPair, tetId, tetCornerId, tetVertIds, posStart, vel, FmInitVector3(0.0f, 0.0f, -1.0f), FmInitVector3(0.0f, 0.0f, collisionPlanes.maxZ));
                    }
                }
            }
        }

        return numContacts;
    }

    uint FmFindSceneCollisionPlanesContacts(
        FmCollidedObjectPair* objectPair,
        const FmSceneCollisionPlanes& collisionPlanes)
    {
        uint numContacts;
        if (objectPair->objectAId & FM_RB_FLAG)
        {
            const FmRigidBody& rigidBody = *(FmRigidBody *)objectPair->objectA;

            if (FM_IS_SET(rigidBody.flags, FM_OBJECT_FLAG_KINEMATIC))
            {
                return 0;
            }

            numContacts = FmFindSceneCollisionPlanesContactsRb(objectPair, rigidBody, collisionPlanes);
        }
        else
        {
            const FmTetMesh& tetMesh = *objectPair->tetMeshA;

            if (FM_IS_SET(tetMesh.flags, FM_OBJECT_FLAG_KINEMATIC))
            {
                return 0;
            }

            numContacts = FmFindSceneCollisionPlanesContactsQuery(objectPair, collisionPlanes);
        }

        // Copy any temp contacts to global list
        if (objectPair->temps.numDistanceContacts > 0)
        {
            FmCopyTempDistanceContacts(objectPair);
        }

        return numContacts;
    }

    void FmFindSurfaceCollisionCallbackContacts(FmScene* scene, FmCollidedObjectPair* objectPair)
    {
        const FmTetMesh& tetMesh = *objectPair->tetMeshA;
        float timestep = objectPair->timestep;
        float distContactThreshold = objectPair->distContactThreshold;

        if (FM_IS_SET(tetMesh.flags, FM_OBJECT_FLAG_KINEMATIC)
            || FM_IS_SET(tetMesh.flags, FM_OBJECT_FLAG_SURFACE_COLLISION_CALLBACK_DISABLED)
            || scene->surfaceCollisionCallback == NULL)
        {
            return;
        }

        for (uint extFaceId = 0; extFaceId < tetMesh.numExteriorFaces; extFaceId++)
        {
            const FmExteriorFace& extFace = tetMesh.exteriorFaces[extFaceId];

            uint tetId = extFace.tetId;
            FmTetVertIds tetVertIds = tetMesh.tetsVertIds[tetId];

            FmFaceVertIds extFaceVertIds;
            FmGetFaceVertIds(&extFaceVertIds, extFace.faceId, tetVertIds);

            FmFaceVertIds extFaceTetCorners;
            FmGetFaceTetCorners(&extFaceTetCorners, extFace.faceId);

            for (uint cornerId = 0; cornerId < 3; cornerId++)
            {
                if (extFace.OwnsVert(cornerId))
                {
                    uint vId = extFaceVertIds.ids[cornerId];

                    if (!(tetMesh.vertsFlags[vId] & FM_VERT_FLAG_KINEMATIC))
                    {
                        uint tetCornerId = extFaceTetCorners.ids[cornerId];

                        FmVector3 vel = tetMesh.vertsVel[vId];
                        FmVector3 posStart = tetMesh.vertsPos[vId];
                        FmVector3 posEnd = posStart + vel * timestep;

                        FmVector3 surfacePos;
                        FmVector3 surfaceNormal;
                        bool contactDetected = scene->surfaceCollisionCallback(&surfacePos, &surfaceNormal, posStart, posEnd, tetMesh.objectId, vId);

                        if (contactDetected && dot(posEnd - surfacePos, surfaceNormal) <= distContactThreshold)
                        {
                            FmAddPlaneContact(objectPair, tetId, tetCornerId, tetVertIds, posStart, vel, surfaceNormal, surfacePos);
                        }
                    }
                }
            }
        }

        // Copy any temp contacts to global list
        if (objectPair->temps.numDistanceContacts > 0)
        {
            FmCopyTempDistanceContacts(objectPair);
        }
    }

    void FmSetFindContactsWarnings(FmScene* scene);

    void FmTaskFuncFindSceneCollisionPlanesContacts(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT(FIND_CONTACTS_WORK);

        FmTaskDataFindContacts* taskData = (FmTaskDataFindContacts*)inTaskData;
        FmScene* scene = taskData->scene;
        uint* tetMeshIds = taskData->tetMeshIds;
        uint* rigidBodyIds = taskData->rigidBodyIds;
        uint numTetMeshes = taskData->numTetMeshes;

        FmThreadTempMemoryBuffer* threadTempMemBuffer = scene->threadTempMemoryBuffer;
        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;
        FmSceneControlParams& sceneParams = scene->params;
        float timestep = sceneParams.timestep;
        float distContactBias = sceneParams.distContactBias;
        float distContactThreshold = sceneParams.distContactThreshold;

        uint taskIndex = (uint)inTaskBeginIndex;

        int workerIndex = scene->taskSystemCallbacks.GetTaskSystemWorkerIndex();
        uint8_t* pTempMemBufferStart = threadTempMemBuffer->buffers[workerIndex];
        uint8_t* pTempMemBufferEnd = pTempMemBufferStart + threadTempMemBuffer->numBytesPerBuffer;

        uint numObjectContacts = 0;

        if (taskIndex < numTetMeshes)
        {
            FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, tetMeshIds[inTaskBeginIndex]);

            FmCollidedObjectPair objectPair;
            objectPair.objectA = &tetMesh;
            objectPair.objectB = NULL;
            objectPair.tetMeshA = &tetMesh;
            objectPair.tetMeshB = NULL;
            objectPair.objectAId = tetMesh.objectId;
            objectPair.objectBId = FM_INVALID_ID;
            objectPair.objectAHierarchy = &tetMesh.bvh;
            objectPair.objectBHierarchy = NULL;
            objectPair.objectAMinPosition = tetMesh.minPosition;
            objectPair.objectAMaxPosition = tetMesh.maxPosition;
            objectPair.objectBMinPosition = FmInitVector3(0.0f);
            objectPair.objectBMaxPosition = FmInitVector3(0.0f);
            objectPair.constraintsBuffer = constraintsBuffer;
            objectPair.timestep = timestep;
            objectPair.distContactBias = distContactBias;
            objectPair.distContactThreshold = distContactThreshold;
            objectPair.volContactBias = 0.0f;
            objectPair.volContactThreshold = 0.0f;
            objectPair.numDistanceContacts = 0;
            objectPair.numVolumeContacts = 0;
            objectPair.volContactCenter = FmInitVector3(0.0f);
            objectPair.volContactObjectACenterPos = FmInitVector3(0.0f);
            objectPair.volContactObjectBCenterPos = FmInitVector3(0.0f);
            objectPair.volContactNormal = FmInitVector3(0.0f);
            objectPair.volContactV = 0.0f;
            objectPair.collisionReport = &scene->collisionReport;
            objectPair.numDistanceContactReports = 0;
            objectPair.numVolumeContactReports = 0;

            uint8_t* pTempMemBuffer = pTempMemBufferStart;

            // Allocate temp memory for object pair
            if (FmAllocCollidedObjectPairTemps(&objectPair.temps, pTempMemBuffer, pTempMemBufferEnd, threadTempMemBuffer->maxTetMeshBufferFeatures,
                0, 0, 0, 0, false
#if FM_SURFACE_INTERSECTION_CONTACTS
                , 0
#endif
            ))
            {
                FmFindSceneCollisionPlanesContacts(&objectPair, sceneParams.collisionPlanes);

                // Add contacts at vertices from collision surface callback
                FmFindSurfaceCollisionCallbackContacts(scene, &objectPair);

                numObjectContacts = objectPair.numDistanceContacts;
            }

            if (FM_IS_SET(tetMesh.flags, FM_OBJECT_FLAG_ENABLE_SELF_COLLISION))
            {
                objectPair.objectA = &tetMesh;
                objectPair.objectB = &tetMesh;
                objectPair.tetMeshA = &tetMesh;
                objectPair.tetMeshB = &tetMesh;
                objectPair.objectAId = tetMesh.objectId;
                objectPair.objectBId = tetMesh.objectId;
                objectPair.objectAHierarchy = &tetMesh.bvh;
                objectPair.objectBHierarchy = &tetMesh.bvh;
                objectPair.objectAMinPosition = tetMesh.minPosition;
                objectPair.objectAMaxPosition = tetMesh.maxPosition;
                objectPair.objectBMinPosition = tetMesh.minPosition;
                objectPair.objectBMaxPosition = tetMesh.maxPosition;
                objectPair.constraintsBuffer = constraintsBuffer;
                objectPair.timestep = timestep;
                objectPair.distContactBias = distContactBias;
                objectPair.distContactThreshold = distContactThreshold;
                objectPair.volContactBias = 0.0f;
                objectPair.volContactThreshold = 0.0f;
                objectPair.numDistanceContacts = 0;
                objectPair.numVolumeContacts = 0;
                objectPair.volContactCenter = tetMesh.centerOfMass;
                objectPair.volContactObjectACenterPos = FmInitVector3(0.0f);
                objectPair.volContactObjectBCenterPos = FmInitVector3(0.0f);
                objectPair.volContactNormal = FmInitVector3(0.0f);
                objectPair.volContactV = 0.0f;
                objectPair.collisionReport = &scene->collisionReport;
                objectPair.numDistanceContactReports = 0;
                objectPair.numVolumeContactReports = 0;

                pTempMemBuffer = pTempMemBufferStart;

                // Allocate temp memory for object pair
                if (FmAllocCollidedObjectPairTemps(&objectPair.temps, pTempMemBuffer, pTempMemBufferEnd, threadTempMemBuffer->maxTetMeshBufferFeatures,
                    tetMesh.numVerts * 2, tetMesh.numVerts * 2, tetMesh.numExteriorFaces, tetMesh.numExteriorFaces, true
#if FM_SURFACE_INTERSECTION_CONTACTS
                    , FmMinUint(tetMesh.numExteriorFaces + tetMesh.numExteriorFaces, FM_SURFACE_INTERSECTION_MAX_CONTACTS)
#endif
                ))
                {
                    FmFindSelfContacts(&objectPair);
                }
            }
        }
        else
        {
            FmRigidBody* pRigidBody = FmGetRigidBodyPtrById(*scene, rigidBodyIds[taskIndex - numTetMeshes]);

            FmCollidedObjectPair objectPair;
            objectPair.objectA = pRigidBody;
            objectPair.objectB = NULL;
            objectPair.tetMeshA = NULL;
            objectPair.tetMeshB = NULL;
            objectPair.objectAId = pRigidBody->objectId;
            objectPair.objectBId = FM_INVALID_ID;
            objectPair.objectAHierarchy = NULL;
            objectPair.objectBHierarchy = NULL;
            objectPair.objectAMinPosition = FmInitVector3(0.0f);
            objectPair.objectAMaxPosition = FmInitVector3(0.0f);
            objectPair.objectBMinPosition = FmInitVector3(0.0f);
            objectPair.objectBMaxPosition = FmInitVector3(0.0f);
            objectPair.constraintsBuffer = constraintsBuffer;
            objectPair.timestep = timestep;
            objectPair.distContactBias = distContactBias;
            objectPair.distContactThreshold = distContactThreshold;
            objectPair.volContactBias = 0.0f;
            objectPair.volContactThreshold = 0.0f;
            objectPair.numDistanceContacts = 0;
            objectPair.numVolumeContacts = 0;
            objectPair.volContactCenter = FmInitVector3(0.0f);
            objectPair.volContactObjectACenterPos = FmInitVector3(0.0f);
            objectPair.volContactObjectBCenterPos = FmInitVector3(0.0f);
            objectPair.volContactNormal = FmInitVector3(0.0f);
            objectPair.volContactV = 0.0f;
            objectPair.collisionReport = &scene->collisionReport;
            objectPair.numDistanceContactReports = 0;
            objectPair.numVolumeContactReports = 0;

            uint8_t* pTempMemBuffer = pTempMemBufferStart;

            // Allocate temp memory for object pair
            if (FmAllocCollidedObjectPairTemps(&objectPair.temps, pTempMemBuffer, pTempMemBufferEnd, threadTempMemBuffer->maxTetMeshBufferFeatures,
                0, 0, 0, 0, false
#if FM_SURFACE_INTERSECTION_CONTACTS
                , 0
#endif
            ))
            {
                FmFindSceneCollisionPlanesContactsRb(&objectPair, *pRigidBody, sceneParams.collisionPlanes);

                numObjectContacts = objectPair.numDistanceContacts;
            }
        }

        if (numObjectContacts)
        {
            FmAtomicOr(&constraintsBuffer->foundContacts.val, 1);
        }

        if (taskData->progress.TaskIsFinished(taskData))
        {
            FmSetFindContactsWarnings(scene);
        }
    }

    void FmTaskFuncFindContacts(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT(FIND_CONTACTS_WORK);

        FmTaskDataFindContacts* taskData = (FmTaskDataFindContacts*)inTaskData;
        FmScene* scene = taskData->scene;

        FmConstraintsBuffer& constraintsBuffer = *scene->constraintsBuffer;
        FmThreadTempMemoryBuffer* threadTempMemBuffer = scene->threadTempMemoryBuffer;
        FmSceneControlParams& sceneParams = scene->params;
        float timestep = sceneParams.timestep;
        float distContactBias = sceneParams.distContactBias;
        float distContactThreshold = sceneParams.distContactThreshold;
        float volContactBias = sceneParams.volContactBias;
        float volContactThreshold = sceneParams.volContactThreshold;

        int workerIndex = scene->taskSystemCallbacks.GetTaskSystemWorkerIndex();
        uint8_t* pTempMemBufferStart = threadTempMemBuffer->buffers[workerIndex];
        uint8_t* pTempMemBufferEnd = pTempMemBufferStart + threadTempMemBuffer->numBytesPerBuffer;

        uint pairIdxStart = (uint)inTaskBeginIndex;
        uint pairIdxEnd = pairIdxStart + 1;

        for (uint pairIdx = pairIdxStart; pairIdx < pairIdxEnd; pairIdx++)
        {
            FmBroadPhasePair& pair = constraintsBuffer.broadPhasePairs[pairIdx];

            if (FM_IS_SET(pair.objectIdA, FM_RB_FLAG) && FM_IS_SET(pair.objectIdB, FM_RB_FLAG))
            {
                FmRigidBody* rigidBodyA = FmGetRigidBodyPtrById(*scene, pair.objectIdA);
                FmRigidBody* rigidBodyB = FmGetRigidBodyPtrById(*scene, pair.objectIdB);

                FM_ASSERT(rigidBodyA != NULL);
                FM_ASSERT(rigidBodyB != NULL);

                uint collisionGroupA = rigidBodyA->collisionGroup;
                uint collisionGroupB = rigidBodyB->collisionGroup;

                // Exclude kinematic vs kinematic collisions
                if ((FM_IS_SET(rigidBodyA->flags, FM_OBJECT_FLAG_KINEMATIC)
                    && FM_IS_SET(rigidBodyB->flags, FM_OBJECT_FLAG_KINEMATIC))
                    || !FmGroupsCanCollide(*scene, collisionGroupA, collisionGroupB))
                {
                    continue;
                }

                FmTetMesh* meshA = FmGetTetMesh(*(FmTetMeshBuffer*)rigidBodyA->collisionObj, 0);
                FmTetMesh* meshB = FmGetTetMesh(*(FmTetMeshBuffer*)rigidBodyB->collisionObj, 0);

                FmCollidedObjectPair objectPair;
                objectPair.objectA = rigidBodyA;
                objectPair.objectB = rigidBodyB;
                objectPair.tetMeshA = meshA;
                objectPair.tetMeshB = meshB;
                objectPair.objectAId = meshA->objectId;
                objectPair.objectBId = meshB->objectId;
                objectPair.objectAHierarchy = &meshA->bvh;
                objectPair.objectBHierarchy = &meshB->bvh;
                objectPair.objectAMinPosition = meshA->minPosition;
                objectPair.objectAMaxPosition = meshA->maxPosition;
                objectPair.objectBMinPosition = meshB->minPosition;
                objectPair.objectBMaxPosition = meshB->maxPosition;
                objectPair.constraintsBuffer = scene->constraintsBuffer;
                objectPair.timestep = timestep;
                objectPair.distContactBias = sceneParams.distContactBias;
                objectPair.distContactThreshold = sceneParams.distContactThreshold;
                objectPair.volContactBias = sceneParams.volContactBias;
                objectPair.volContactThreshold = sceneParams.volContactThreshold;
                objectPair.numDistanceContacts = 0;
                objectPair.numVolumeContacts = 0;

                FmVector3 objectACenterPos = rigidBodyA->state.pos;
                FmVector3 objectBCenterPos = rigidBodyB->state.pos;
                objectPair.volContactCenter = (objectACenterPos + objectBCenterPos) * 0.5f;
                objectPair.volContactObjectACenterPos = objectACenterPos - objectPair.volContactCenter;
                objectPair.volContactObjectBCenterPos = objectBCenterPos - objectPair.volContactCenter;

                objectPair.volContactNormal = FmInitVector3(0.0f);
                objectPair.volContactV = 0.0f;
                objectPair.collisionReport = &scene->collisionReport;
                objectPair.numDistanceContactReports = 0;
                objectPair.numVolumeContactReports = 0;

                uint8_t* pTempMemBuffer = pTempMemBufferStart;

                if (!FmAllocCollidedObjectPairTemps(&objectPair.temps, pTempMemBuffer, pTempMemBufferEnd, threadTempMemBuffer->maxTetMeshBufferFeatures, 
                    meshA->numVerts, meshB->numVerts, 0, 0, false
#if FM_SURFACE_INTERSECTION_CONTACTS
                    , FmMinUint(meshA->numExteriorFaces + meshB->numExteriorFaces, FM_SURFACE_INTERSECTION_MAX_CONTACTS)
#endif
                ))
                {
                    continue;
                }

                FmBoxBoxCcdTemps* ccdTemps = FmAllocBoxCcdTemps(pTempMemBuffer, (size_t)(pTempMemBufferEnd - pTempMemBuffer));
                if (ccdTemps == NULL)
                {
                    continue;
                }

                uint numContacts = FmFindContactsBoxBox(&objectPair, ccdTemps);

                if (numContacts)
                {
                    FmAtomicOr(&constraintsBuffer.foundContacts.val, 1);
                }

                if (objectPair.objectAContactWithDynamicVerts)
                {
                    FmMarkIslandOfObjectForWaking(scene, rigidBodyA->objectId);
                }

                if (objectPair.objectBContactWithDynamicVerts)
                {
                    FmMarkIslandOfObjectForWaking(scene, rigidBodyB->objectId);
                }
            }
            else if (FM_IS_SET(pair.objectIdB, FM_RB_FLAG))
            {
                FmRigidBody* rigidBodyB = FmGetRigidBodyPtrById(*scene, pair.objectIdB);

                FmTetMesh* meshA = FmGetTetMesh(*scene, pair.objectIdA);
                FmTetMesh* meshB = FmGetTetMesh(*(FmTetMeshBuffer*)rigidBodyB->collisionObj, 0);

                uint collisionGroupA = meshA->collisionGroup;
                uint collisionGroupB = rigidBodyB->collisionGroup;

                if (!FmGroupsCanCollide(*scene, collisionGroupA, collisionGroupB))
                {
                    continue;
                }

                FmCollidedObjectPair objectPair;
                objectPair.objectA = meshA;
                objectPair.objectB = rigidBodyB;
                objectPair.tetMeshA = meshA;
                objectPair.tetMeshB = meshB;
                objectPair.objectAId = meshA->objectId;
                objectPair.objectBId = meshB->objectId;
                objectPair.objectAHierarchy = &meshA->bvh;
                objectPair.objectBHierarchy = &meshB->bvh;
                objectPair.objectAMinPosition = meshA->minPosition;
                objectPair.objectAMaxPosition = meshA->maxPosition;
                objectPair.objectBMinPosition = meshB->minPosition;
                objectPair.objectBMaxPosition = meshB->maxPosition;
                objectPair.constraintsBuffer = scene->constraintsBuffer;
                objectPair.timestep = timestep;
                objectPair.distContactBias = sceneParams.distContactBias;
                objectPair.distContactThreshold = sceneParams.distContactThreshold;
                objectPair.volContactBias = sceneParams.volContactBias;
                objectPair.volContactThreshold = sceneParams.volContactThreshold;
                objectPair.numDistanceContacts = 0;
                objectPair.numVolumeContacts = 0;

                FmVector3 objectACenterPos = meshA->centerOfMass;
                FmVector3 objectBCenterPos = rigidBodyB->state.pos;
                objectPair.volContactCenter = (objectACenterPos + objectBCenterPos) * 0.5f;
                objectPair.volContactObjectACenterPos = objectACenterPos - objectPair.volContactCenter;
                objectPair.volContactObjectBCenterPos = objectBCenterPos - objectPair.volContactCenter;

                objectPair.volContactNormal = FmInitVector3(0.0f);
                objectPair.volContactV = 0.0f;
                objectPair.collisionReport = &scene->collisionReport;
                objectPair.numDistanceContactReports = 0;
                objectPair.numVolumeContactReports = 0;

                uint8_t* pTempMemBuffer = pTempMemBufferStart;

                if (!FmAllocCollidedObjectPairTemps(&objectPair.temps, pTempMemBuffer, pTempMemBufferEnd, threadTempMemBuffer->maxTetMeshBufferFeatures, 
                    meshA->numVerts, meshB->numVerts, 0, 0, false
#if FM_SURFACE_INTERSECTION_CONTACTS
                    , FmMinUint(meshA->numExteriorFaces + meshB->numExteriorFaces, FM_SURFACE_INTERSECTION_MAX_CONTACTS)
#endif
                ))
                {
                    continue;
                }

                uint numContacts = FmFindContactsTetMeshBox(&objectPair);

                if (numContacts)
                {
                    FmAtomicOr(&constraintsBuffer.foundContacts.val, 1);
                }

                if (objectPair.objectAContactWithDynamicVerts)
                {
                    FmMarkIslandOfObjectForWaking(scene, meshA->objectId);
                }

                if (objectPair.objectBContactWithDynamicVerts)
                {
                    FmMarkIslandOfObjectForWaking(scene, rigidBodyB->objectId);
                }
            }
            else
            {
                FM_ASSERT(pair.objectIdA < scene->maxTetMeshes && pair.objectIdB < scene->maxTetMeshes);

                FmTetMesh* meshA = FmGetTetMeshPtrById(*scene, pair.objectIdA);
                FmTetMesh* meshB = FmGetTetMeshPtrById(*scene, pair.objectIdB);

                // Skip kinematic vs kinematic collisions
                if (FM_IS_SET(meshA->flags, FM_OBJECT_FLAG_KINEMATIC)
                    && FM_IS_SET(meshB->flags, FM_OBJECT_FLAG_KINEMATIC))
                {
                    continue;
                }

                FmCollidedObjectPair objectPair;
                objectPair.objectA = meshA;
                objectPair.objectB = meshB;
                objectPair.tetMeshA = meshA;
                objectPair.tetMeshB = meshB;
                objectPair.objectAId = meshA->objectId;
                objectPair.objectBId = meshB->objectId;
                objectPair.objectAHierarchy = &meshA->bvh;
                objectPair.objectBHierarchy = &meshB->bvh;
                objectPair.objectAMinPosition = meshA->minPosition;
                objectPair.objectAMaxPosition = meshA->maxPosition;
                objectPair.objectBMinPosition = meshB->minPosition;
                objectPair.objectBMaxPosition = meshB->maxPosition;
                objectPair.constraintsBuffer = &constraintsBuffer;
                objectPair.timestep = timestep;
                objectPair.distContactBias = distContactBias;
                objectPair.distContactThreshold = distContactThreshold;
                objectPair.volContactBias = volContactBias;
                objectPair.volContactThreshold = volContactThreshold;
                objectPair.numDistanceContacts = 0;
                objectPair.numVolumeContacts = 0;

                FmVector3 objectACenterPos = meshA->centerOfMass;
                FmVector3 objectBCenterPos = meshB->centerOfMass;
                objectPair.volContactCenter = (objectACenterPos + objectBCenterPos) * 0.5f;
                objectPair.volContactObjectACenterPos = objectACenterPos - objectPair.volContactCenter;
                objectPair.volContactObjectBCenterPos = objectBCenterPos - objectPair.volContactCenter;

                objectPair.volContactNormal = FmInitVector3(0.0f);
                objectPair.volContactV = 0.0f;
                objectPair.collisionReport = &scene->collisionReport;
                objectPair.numDistanceContactReports = 0;
                objectPair.numVolumeContactReports = 0;

                uint8_t* pTempMemBuffer = pTempMemBufferStart;

                // Allocate temp memory for object pair
                if (!FmAllocCollidedObjectPairTemps(&objectPair.temps, pTempMemBuffer, pTempMemBufferEnd, threadTempMemBuffer->maxTetMeshBufferFeatures,
                    meshA->numVerts, meshB->numVerts, 0, 0, true
#if FM_SURFACE_INTERSECTION_CONTACTS
                    , FmMinUint(meshA->numExteriorFaces + meshB->numExteriorFaces, FM_SURFACE_INTERSECTION_MAX_CONTACTS)
#endif
                ))
                {
                    continue;
                }

                uint numMeshPairContacts = FmFindContacts(&objectPair);

                if (numMeshPairContacts)
                {
                    FmAtomicOr(&constraintsBuffer.foundContacts.val, 1);
                }

                if (objectPair.objectAContactWithDynamicVerts)
                {
                    FmMarkIslandOfObjectForWaking(scene, meshA->objectId);
                }

                if (objectPair.objectBContactWithDynamicVerts)
                {
                    FmMarkIslandOfObjectForWaking(scene, meshB->objectId);
                }
            }
        }

        if (taskData->progress.TaskIsFinished(taskData))
        {
            FmSetFindContactsWarnings(scene);
        }
    }

    FM_WRAPPED_TASK_FUNC(FmTaskFuncFindContactsMeshPairsAndCollisionPlanes)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FmTaskDataFindContacts* taskData = (FmTaskDataFindContacts*)inTaskData;

        uint numBroadPhasePairs = taskData->numBroadPhasePairs;

        int32_t taskIndex = taskData->progress.GetNextIndex();

        if (taskIndex < (int32_t)numBroadPhasePairs)
        {
            FmTaskFuncFindContacts(inTaskData, taskIndex, taskIndex + 1);
        }
        else
        {
            taskIndex -= (int32_t)numBroadPhasePairs;
            FmTaskFuncFindSceneCollisionPlanesContacts(inTaskData, taskIndex, taskIndex + 1);
        }
    }

    void FmFindContactsMidAndNarrowPhase(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    void FmFindContacts(FmScene* scene, FmTaskFuncCallback followTaskFunc, void* followTaskData)
    {
#if !FM_ASYNC_THREADING
        (void)followTaskFunc;
        (void)followTaskData;
        FM_TRACE_SCOPED_EVENT(FIND_CONTACTS);
#endif
        FM_SET_START_TIME();
        FM_RESET_BROAD_PHASE_TIME();
        FM_RESET_MESH_CONTACTS_TIME();

        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;
        FmSceneControlParams& sceneParams = scene->params;
        bool includeRigidBodiesInBroadPhase = sceneParams.includeRigidBodiesInBroadPhase;
        bool collideRigidBodiesWithPlanes = !sceneParams.rigidBodiesExternal;
        bool testWithSleepingObjects = true;

        FmInitFindContacts(constraintsBuffer, &scene->collisionReport);

        FmTaskDataFindContacts* taskData = new FmTaskDataFindContacts(0, scene, scene->params.timestep, 
            scene->awakeTetMeshIds, scene->awakeRigidBodyIds,
            scene->numAwakeTetMeshes, scene->numAwakeRigidBodies,
            includeRigidBodiesInBroadPhase, collideRigidBodiesWithPlanes, testWithSleepingObjects);

        taskData->followTask.func = followTaskFunc;
        taskData->followTask.data = followTaskData;

#if FM_PARALLEL_BROAD_PHASE_TRAVERSAL
        FmBroadPhase(taskData, FmFindContactsMidAndNarrowPhase, taskData);
#else
        FmBroadPhase(taskData, NULL, NULL);
        FmFindContactsMidAndNarrowPhase(taskData, 0, 1);
#endif
    }

    void FmSetFindContactsWarnings(FmScene* scene)
    {
        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;

        // Add to warnings report if hit a contact limit
        if (FmAtomicRead(&constraintsBuffer->numDistanceContacts.val) >= constraintsBuffer->maxDistanceContacts)
        {
            FmAtomicOr(&scene->warningsReport.flags.val, FM_WARNING_FLAG_HIT_LIMIT_SCENE_DISTANCE_CONTACTS);
        }
        if (FmAtomicRead(&constraintsBuffer->numVolumeContacts.val) >= constraintsBuffer->maxVolumeContacts)
        {
            FmAtomicOr(&scene->warningsReport.flags.val, FM_WARNING_FLAG_HIT_LIMIT_SCENE_VOLUME_CONTACTS);
        }
        if (FmAtomicRead(&constraintsBuffer->exceededVolumeContactVerts.val))
        {
            FmAtomicOr(&scene->warningsReport.flags.val, FM_WARNING_FLAG_HIT_LIMIT_SCENE_VOLUME_CONTACT_VERTS);
        }
    }

    void FmWakeCollidedIslandsAndFindContacts(FmScene* scene, FmTaskFuncCallback followTaskFunc, void* followTaskData)
    {
        FM_TRACE_SCOPED_EVENT(WAKE_COLLIDED_FIND_CONTACTS);

        FmWakeMarkedIslands(scene);

        // Build broad phase hierarchies on awakened objects

        uint numAwakenedTetMeshes = scene->numAwakenedTetMeshes;
        uint numAwakenedRigidBodies = scene->numAwakenedRigidBodies;

        uint* awakenedTetMeshIds = scene->awakeTetMeshIds + scene->numAwakeTetMeshes - numAwakenedTetMeshes;
        uint* awakenedRigidBodyIds = scene->awakeRigidBodyIds + scene->numAwakeRigidBodies - numAwakenedRigidBodies;

        const FmSceneControlParams& sceneParams = scene->params;
        bool includeRigidBodiesInBroadPhase = sceneParams.includeRigidBodiesInBroadPhase;
        bool collideRigidBodiesWithPlanes = !sceneParams.rigidBodiesExternal;

        bool testWithSleepingObjects = false;

        FmTaskDataFindContacts* taskData = new FmTaskDataFindContacts(0, scene, scene->params.timestep,
            awakenedTetMeshIds, awakenedRigidBodyIds,
            numAwakenedTetMeshes, numAwakenedRigidBodies,
            includeRigidBodiesInBroadPhase, collideRigidBodiesWithPlanes, testWithSleepingObjects);

        taskData->followTask.func = followTaskFunc;
        taskData->followTask.data = followTaskData;

#if FM_PARALLEL_BROAD_PHASE_TRAVERSAL
        FmBroadPhase(taskData, FmFindContactsMidAndNarrowPhase, taskData);
#else
        FmBroadPhase(taskData, NULL, NULL);
        FmFindContactsMidAndNarrowPhase(taskData, 0, 1);
#endif
    }

    void FmFindContactsMidAndNarrowPhase(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FmTaskDataFindContacts* taskData = (FmTaskDataFindContacts *)inTaskData;

        FM_SET_START_TIME();

        FmScene* scene = taskData->scene;
        FmConstraintsBuffer& constraintsBuffer = *scene->constraintsBuffer;
        uint numTetMeshes = taskData->numTetMeshes;
        uint numRigidBodies = taskData->numRigidBodies;
        bool collideRigidBodiesWithPlanes = taskData->collideRigidBodiesWithPlanes;

        uint numPairs = FmAtomicRead(&constraintsBuffer.numBroadPhasePairs.val);

        // Perform contact finding with awakened bodies
        uint numTasks = numPairs + numTetMeshes;
        if (collideRigidBodiesWithPlanes)
        {
            numTasks += numRigidBodies;
        }

        // Reset number of awakened bodies
        scene->numAwakenedTetMeshes = 0;
        scene->numAwakenedRigidBodies = 0;

        taskData->numBroadPhasePairs = numPairs;
        taskData->progress.ResetNextIndex();

#if FM_ASYNC_THREADING
        if (taskData->followTask.func)
        {
            if (numTasks > 0)
            {
                taskData->progress.Init(numTasks, taskData->followTask.func, taskData->followTask.data);

                FmParallelForAsync("FindContacts", FM_TASK_AND_WRAPPED_TASK_ARGS(FmTaskFuncFindContactsMeshPairsAndCollisionPlanes), NULL, taskData, numTasks, scene->taskSystemCallbacks.SubmitAsyncTask, scene->params.numThreads);
            }
            else
            {
                FmSetNextTask(taskData->followTask.func, taskData->followTask.data, 0, 1);
                delete taskData;
            }
        }
#else
        scene->taskSystemCallbacks.ParallelFor("FindContacts", FmTaskFuncFindContactsMeshPairsAndCollisionPlanes, taskData, numTasks);

        FM_ADD_MESH_CONTACTS_TIME();

        delete taskData;
#endif
    }

    // Query FmCollisionGroupPairFlags if two collision groups have collision enabled.
    static inline bool FmGroupsCanCollide(const FmCollisionGroupPairFlags& pairFlags, uint i, uint j)
    {
        FM_ASSERT(i < 32 && j < 32);
        return ((pairFlags.flags[i] & (1 << j)) != 0);
    }

    // Set in FmCollisionGroupPairFlags whether two collision groups can collide.
    static inline void FmSetGroupsCanCollide(FmCollisionGroupPairFlags* pairFlags, uint i, uint j, bool canCollide)
    {
        FM_ASSERT(i < 32 && j < 32);
        if (canCollide)
        {
            pairFlags->flags[i] |= (1 << j);
            pairFlags->flags[j] |= (1 << i);
        }
        else
        {
            pairFlags->flags[i] &= ~(1 << j);
            pairFlags->flags[j] &= ~(1 << i);
        }
    }

    // Query scene if two collision groups have collision enabled.
    bool FmGroupsCanCollide(const FmScene& scene, uint i, uint j)
    {
        return FmGroupsCanCollide(scene.constraintsBuffer->collisionGroupPairs, i, j);
    }

    // Set in scene whether two collision groups can collide.
    void FmSetGroupsCanCollide(FmScene* scene, uint i, uint j, bool canCollide)
    {
        FmSetGroupsCanCollide(&scene->constraintsBuffer->collisionGroupPairs, i, j, canCollide);
    }
}