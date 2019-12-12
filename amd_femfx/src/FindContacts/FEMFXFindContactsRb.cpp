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

#include "FEMFXFindContacts.h"
#include "FEMFXCollisionPairData.h"
#include "FEMFXBoxCcd.h"
#if FM_SOA_TRI_INTERSECTION
#include "FEMFXSoATriIntersection.h"
#endif
#include "FEMFXScene.h"
#include "FEMFXSleeping.h"

using namespace AMD;

namespace AMD
{
    void FmGenerateContactsTriBox(FmCollidedObjectPair* objectPair, uint exteriorFaceIdA)
    {
        FmTetMesh* meshA = objectPair->tetMeshA;
        FmRigidBody* rigidBodyB = (FmRigidBody*)objectPair->objectB;
        FmTetMesh* meshB = objectPair->tetMeshB;
        FmVolumeContactWorkspace* volContactWorkspace = objectPair->temps.volContactWorkspace;
        FmVector3 volContactCenter = objectPair->volContactCenter;
        float timestep = objectPair->timestep;
        float distContactBias = objectPair->distContactBias;
        float distContactThreshold = objectPair->distContactThreshold;
        uint meshIdA = meshA->objectId;
        uint rigidBodyIdB = rigidBodyB->objectId;

        uint numDistanceContacts = 0;

        // Gather positions and velocities of exterior faces
        FmExteriorFace& exteriorFaceA = meshA->exteriorFaces[exteriorFaceIdA];

        uint tetIdA = exteriorFaceA.tetId;

        FmFaceVertIds tetCornersA;
        FmGetFaceTetCorners(&tetCornersA, exteriorFaceA.faceId);

        FmFaceVertIds faceVertIdsA;
        FmGetFaceVertIds(&faceVertIdsA, exteriorFaceA.faceId, meshA->tetsVertIds[tetIdA]);

        FmTri triA;
        triA.id = exteriorFaceA.id;
        triA.pos0 = meshA->vertsPos[faceVertIdsA.ids[0]] - volContactCenter;
        triA.pos1 = meshA->vertsPos[faceVertIdsA.ids[1]] - volContactCenter;
        triA.pos2 = meshA->vertsPos[faceVertIdsA.ids[2]] - volContactCenter;
        triA.vel0 = meshA->vertsVel[faceVertIdsA.ids[0]];
        triA.vel1 = meshA->vertsVel[faceVertIdsA.ids[1]];
        triA.vel2 = meshA->vertsVel[faceVertIdsA.ids[2]];

        FmBox boxB;
        boxB.state = rigidBodyB->state;
        boxB.state.pos -= volContactCenter;
        boxB.halfWidths[0] = rigidBodyB->dims[0];
        boxB.halfWidths[1] = rigidBodyB->dims[1];
        boxB.halfWidths[2] = rigidBodyB->dims[2];

        FmMatrix3 boxRot = FmInitMatrix3(boxB.state.quat);

        // Test for intersection
        bool boxTriIntersection = FmBoxAndTriIntersect(boxB, &triA.pos0);

        // Apply contributions to volume and gradient
        if (boxTriIntersection)
        {
            if (volContactWorkspace)
            {
                uint numExteriorFacesB = meshB->numExteriorFaces;
                for (uint exteriorFaceIdB = 0; exteriorFaceIdB < numExteriorFacesB; exteriorFaceIdB++)
                {
                    FmMeshCollisionTriPair triPair;
                    triPair.exteriorFaceIdA = exteriorFaceIdA;
                    triPair.exteriorFaceIdB = exteriorFaceIdB;

                    triPair.tetIdA = tetIdA;
                    triPair.tetCornersA = tetCornersA;
                    triPair.faceVertIdsA = faceVertIdsA;
                    triPair.triA = triA;

                    FmExteriorFace& exteriorFaceB = meshB->exteriorFaces[triPair.exteriorFaceIdB];

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

                    triPair.pairIntersects = FmComputeTriIntersection(&triPair.triIntersectionPoints, &triPair.triA.pos0, &triPair.triB.pos0);

                    if (triPair.pairIntersects)
                    {
                        FmApplyTriIntersectionVolumeAndGradientContributions(objectPair, triPair, volContactWorkspace);
                    }
                }
            }
        }

        // Perform CCD checks with features of triangles
        FmCcdTerminationConditions conditions;
        conditions.impactGap = distContactBias;
        conditions.contactGap = distContactThreshold;
        conditions.maxIterations = FM_DEFAULT_MAX_CCD_ITERATIONS;

        const int maxContacts = 3 * 6 + 3 * 12 + 1 * 8;
        FmCcdResult boxTriContacts[maxContacts];
        uint numCcdContacts = 0;

        for (uint i = 0; i < 3; i++)  // tri verts
        {
            if (triA.OwnsVert(i))
            {
                FmVector3 triPos = triA.Pos(i);
                FmVector3 triVel = triA.Vel(i);

                for (uint boxFaceDim = 0; boxFaceDim < 3; boxFaceDim++)  // box faces
                {
                    for (uint boxFaceSign = 0; boxFaceSign < 2; boxFaceSign++)
                    {
                        FmCcdResult& contact = boxTriContacts[numCcdContacts];

                        FmBoxFacePointCcd(&contact, boxB, boxFaceDim, boxFaceSign, triPos, triVel, i, timestep, conditions);

                        if (contact.retVal != FM_CCD_RET_NO_IMPACT && contact.distance > 0.0f && contact.featurePair.itype == FM_FEATURE_TYPE_FACE)
                        {
                            numCcdContacts++;
                        }
                    }
                }
            }
        }

        FmVector3 boxComponents[3];
        boxComponents[0] = FmInitVector3(boxB.halfWidths[0], 0.0f, 0.0f);
        boxComponents[1] = FmInitVector3(0.0f, boxB.halfWidths[1], 0.0f);
        boxComponents[2] = FmInitVector3(0.0f, 0.0f, boxB.halfWidths[2]);

        for (uint i = 0; i < 3; i++)  // tri edges
        {
            uint i1 = (i + 1) % 3;

            if (triA.OwnsEdge(i))
            {
                FmVector3 triPos0 = triA.Pos(i);
                FmVector3 triVel0 = triA.Vel(i);
                FmVector3 triPos1 = triA.Pos(i1);
                FmVector3 triVel1 = triA.Vel(i1);

                FmBoxFeature edgeFeature;
                edgeFeature.axisMask = 0x7;

                for (uint boxDim = 0; boxDim < 3; boxDim++)  // box edges
                {
                    for (uint sign1 = 0; sign1 < 2; sign1++)
                    {
                        for (uint sign2 = 0; sign2 < 2; sign2++)
                        {
                            uint dim1 = (boxDim + 1) % 3;
                            uint dim2 = (boxDim + 2) % 3;

                            edgeFeature.axisMask = (uint)(~(1 << boxDim) & 0x7);
                            edgeFeature.signBits = (sign1 << dim1) | (sign2 << dim2);
                            edgeFeature.dim = boxDim;

                            float fsign1 = sign1 ? 1.0f : -1.0f;
                            float fsign2 = sign2 ? 1.0f : -1.0f;

                            FmVector3 boxEdgePos0 = -boxComponents[boxDim] + boxComponents[dim1] * fsign1 + boxComponents[dim2] * fsign2;
                            FmVector3 boxEdgePos1 = boxComponents[boxDim] + boxComponents[dim1] * fsign1 + boxComponents[dim2] * fsign2;

                            FmCcdResult& contact = boxTriContacts[numCcdContacts];

                            FmBoxEdgeSegmentCcd(&contact, boxB, edgeFeature, boxEdgePos0, boxEdgePos1, triPos0, triVel0, triPos1, triVel1, i, i1, timestep, conditions);

                            if (contact.retVal != FM_CCD_RET_NO_IMPACT && contact.distance > 0.0f)
                            {
                                numCcdContacts++;
                            }
                        }
                    }
                }
            }
        }

        for (uint boxSignX = 0; boxSignX < 2; boxSignX++)  // box points
        {
            for (uint boxSignY = 0; boxSignY < 2; boxSignY++)  // box points
            {
                for (uint boxSignZ = 0; boxSignZ < 2; boxSignZ++)  // box points
                {
                    uint vertexId = boxSignX | (boxSignY << 1) | (boxSignZ << 2);

                    FmCcdResult& contact = boxTriContacts[numCcdContacts];

                    float signX = boxSignX ? 1.0f : -1.0f;
                    float signY = boxSignY ? 1.0f : -1.0f;
                    float signZ = boxSignZ ? 1.0f : -1.0f;

                    FmVector3 boxVertexPos = boxComponents[0] * signX + boxComponents[1] * signY + boxComponents[2] * signZ;

                    FmBoxVertexTriCcd(&contact, boxB, vertexId, boxVertexPos, triA, timestep, conditions);

                    if (contact.retVal != FM_CCD_RET_NO_IMPACT && contact.distance > 0.0f)
                    {
                        numCcdContacts++;
                    }
                }
            }
        }

        for (uint cId = 0; cId < numCcdContacts; cId++)
        {
            FmCcdResult& boxTriContact = boxTriContacts[cId];
            FmCcdResult triBoxContact;
            triBoxContact.distance = boxTriContact.distance;
            triBoxContact.numIterations = boxTriContact.numIterations;
            triBoxContact.time = boxTriContact.time;
            triBoxContact.direction = -boxTriContact.direction;
            triBoxContact.posi = boxTriContact.posj;
            triBoxContact.posj = boxTriContact.posi;
            triBoxContact.featurePair = FmReverseFeaturePair(boxTriContact.featurePair);
            triBoxContact.retVal = boxTriContact.retVal;

            assert(!(triBoxContact.retVal == FM_CCD_RET_NO_IMPACT || triBoxContact.distance <= 0.0f));

            // exclude contacts with vertex if not owned by triangle
            if (triBoxContact.featurePair.itype == FM_FEATURE_TYPE_VERTEX && !triA.OwnsVert(triBoxContact.featurePair.i0))
            {
                continue;
            }

            FmTetVertIds tetVertIdsA = meshA->tetsVertIds[tetIdA];

            // Create tet pair from tri contact
            FmDistanceContactPairInfo contactPairInfo;
            contactPairInfo.idInObjectPair = objectPair->numDistanceContacts;
            contactPairInfo.objectIdA = meshIdA;
            contactPairInfo.objectIdB = rigidBodyIdB;
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
            contact.tetIdA = tetIdA;
            contact.tetIdB = (triBoxContact.featurePair.j1 << 3) | triBoxContact.featurePair.j0;  // box feature
            contact.frictionCoeff = FmMinFloat(meshA->tetsFrictionCoeff[tetIdA], rigidBodyB->frictionCoeff);

            // Found a contact, calculate the tet barycentric coords

            if (triBoxContact.featurePair.itype == FM_FEATURE_TYPE_VERTEX)
            {
                contact.posBaryA[tetCornersA.ids[triBoxContact.featurePair.i0]] = 1.0f;
            }
            else if (triBoxContact.featurePair.itype == FM_FEATURE_TYPE_EDGE)
            {
                float bary[2];
                FmComputeLineSegmentBarycentricCoords(
                    bary, triBoxContact.posi,
                    triA.Pos(triBoxContact.featurePair.i0, triBoxContact.time),
                    triA.Pos(triBoxContact.featurePair.i1, triBoxContact.time));

                contact.posBaryA[tetCornersA.ids[triBoxContact.featurePair.i0]] = bary[0];
                contact.posBaryA[tetCornersA.ids[triBoxContact.featurePair.i1]] = bary[1];
            }
            else if (triBoxContact.featurePair.itype == FM_FEATURE_TYPE_FACE)
            {
                float bary[3];
                FmComputeTriBarycentricCoords(
                    bary, triBoxContact.posi,
                    triA.Pos(0, triBoxContact.time),
                    triA.Pos(1, triBoxContact.time),
                    triA.Pos(2, triBoxContact.time));

                contact.posBaryA[tetCornersA.ids[0]] = bary[0];
                contact.posBaryA[tetCornersA.ids[1]] = bary[1];
                contact.posBaryA[tetCornersA.ids[2]] = bary[2];
            }

            FmVector3 posAdv = boxB.state.pos + boxB.state.vel * triBoxContact.time;
            FmQuat    quatAdv = FmIntegrateQuat(boxB.state.quat, boxB.state.angVel, triBoxContact.time);
            FmVector3 comToContactAdv = triBoxContact.posj - posAdv;
            FmVector3 comToContactAdvBoxSpace = rotate(conj(quatAdv), comToContactAdv);
            FmVector3 comToContact = rotate(boxB.state.quat, comToContactAdvBoxSpace);

            contact.comToPosB[0] = comToContact.x;
            contact.comToPosB[1] = comToContact.y;
            contact.comToPosB[2] = comToContact.z;

            FmVector3 posA = FmInterpolate(contact.posBaryA,
                meshA->vertsPos[tetVertIdsA.ids[0]],
                meshA->vertsPos[tetVertIdsA.ids[1]],
                meshA->vertsPos[tetVertIdsA.ids[2]],
                meshA->vertsPos[tetVertIdsA.ids[3]]) - volContactCenter;  // Translate mesh position to same origin as box
            FmVector3 velA = FmInterpolate(contact.posBaryA,
                meshA->vertsVel[tetVertIdsA.ids[0]],
                meshA->vertsVel[tetVertIdsA.ids[1]],
                meshA->vertsVel[tetVertIdsA.ids[2]],
                meshA->vertsVel[tetVertIdsA.ids[3]]);

            FmVector3 posB = boxB.state.pos + comToContact;
            FmVector3 velB = boxB.state.vel + cross(boxB.state.angVel, comToContact);

            contact.normal = -triBoxContact.direction;
            contact.normalProjDistance = dot(contact.normal, posA - posB) - distContactBias;
            contact.normalProjRelVel = dot(contact.normal, velB - velA);
            contact.tangent1 = FmGetContactTangent(contact.normal, velA, velB);

            uint dynamicFlagsA, movingFlagsA, dynamicFlagsB, movingFlagsB;

            FmDynamicAndMovingFlags(&dynamicFlagsA, &movingFlagsA, *meshA, tetVertIdsA, contact.posBaryA);
            FmDynamicAndMovingFlags(&dynamicFlagsB, &movingFlagsB, *rigidBodyB);

            FmSetConstraintFlags(&contactPairInfo.flags, &contactPairInfo.dynamicFlags, &contact.movingFlags, dynamicFlagsA, dynamicFlagsB, movingFlagsA, movingFlagsB);

            bool exterior = true;

            // Test normal
            if (triBoxContact.featurePair.itype == FM_FEATURE_TYPE_VERTEX)
            {
                if (!FmIsVertNormalExterior(*meshA, triA.GetTriId(), triBoxContact.featurePair.i0, -contact.normal))
                {
                    exterior = false;
                }
            }
            if (triBoxContact.featurePair.itype == FM_FEATURE_TYPE_EDGE)
            {
                if (!FmIsEdgeNormalExterior(*meshA, triA.GetTriId(), triBoxContact.featurePair.i0, -contact.normal))
                {
                    exterior = false;
                }
            }
            if (triBoxContact.featurePair.itype == FM_FEATURE_TYPE_FACE)
            {
                if (dot(contact.normal, FmExteriorFaceCrossProd(*meshA, triA.GetTriId())) >= 0.0f)
                {
                    exterior = false;
                }
            }

            bool exteriorToBoxFace = false;

            uint boxSignBits = triBoxContact.featurePair.j0;
            uint boxAxisMask = triBoxContact.featurePair.j1;

            float boxSignX = (boxSignBits & 0x1) ? 1.0f : -1.0f;
            float boxSignY = (boxSignBits & 0x2) ? 1.0f : -1.0f;
            float boxSignZ = (boxSignBits & 0x4) ? 1.0f : -1.0f;

            if ((boxAxisMask & 0x1) && dot(contact.normal, boxSignX * boxRot.col0) > 0.0f)
            {
                exteriorToBoxFace = true;
            }
            if ((boxAxisMask & 0x2) && dot(contact.normal, boxSignY * boxRot.col1) > 0.0f)
            {
                exteriorToBoxFace = true;
            }
            if ((boxAxisMask & 0x4) && dot(contact.normal, boxSignZ * boxRot.col2) > 0.0f)
            {
                exteriorToBoxFace = true;
            }

            if (!exteriorToBoxFace)
            {
                exterior = false;
            }

            if ((FM_NOT_SET(contactPairInfo.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED) || FM_NOT_SET(contactPairInfo.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED))
                && exterior)
            {
                FmTetVertIds tmpVertIdsA, tmpVertIdsB;
                numDistanceContacts += FmAddTempDistanceContact(objectPair, &contactPairInfo, &contact, tmpVertIdsA, tmpVertIdsB, dynamicFlagsA, dynamicFlagsB, NULL);
            }
        }
    }

    void FmGenerateContactsBoxBox(FmCollidedObjectPair* objectPair, FmBoxBoxCcdTemps* ccdTemps)
    {
        FmRigidBody* rigidBodyA = (FmRigidBody*)objectPair->objectA;
        FmRigidBody* rigidBodyB = (FmRigidBody*)objectPair->objectB;
        FmTetMesh* meshA = objectPair->tetMeshA;
        FmTetMesh* meshB = objectPair->tetMeshB;
        FmVolumeContactWorkspace* volContactWorkspace = objectPair->temps.volContactWorkspace;
        FmVector3 volContactCenter = objectPair->volContactCenter;
        float timestep = objectPair->timestep;
        float distContactBias = objectPair->distContactBias;
        float distContactThreshold = objectPair->distContactThreshold;
        uint rigidBodyIdA = rigidBodyA->objectId;
        uint rigidBodyIdB = rigidBodyB->objectId;

        uint numDistanceContacts = 0;

        // Gather positions and velocities of exterior faces
        FmBox boxA;
        boxA.state = rigidBodyA->state;
        boxA.state.pos -= volContactCenter;
        boxA.halfWidths[0] = rigidBodyA->dims[0];
        boxA.halfWidths[1] = rigidBodyA->dims[1];
        boxA.halfWidths[2] = rigidBodyA->dims[2];

        FmBox boxB;
        boxB.state = rigidBodyB->state;
        boxB.state.pos -= volContactCenter;
        boxB.halfWidths[0] = rigidBodyB->dims[0];
        boxB.halfWidths[1] = rigidBodyB->dims[1];
        boxB.halfWidths[2] = rigidBodyB->dims[2];

        // Test for intersection
        bool boxIntersection = FmBoxesIntersect(boxA, boxB);

        // Apply contributions to volume and gradient
        if (boxIntersection)
        {
            if (volContactWorkspace)
            {
                uint numExteriorFacesA = meshA->numExteriorFaces;
                uint numExteriorFacesB = meshB->numExteriorFaces;
                for (uint exteriorFaceIdA = 0; exteriorFaceIdA < numExteriorFacesA; exteriorFaceIdA++)
                {
                    FmMeshCollisionTriPair triPair;
                    triPair.exteriorFaceIdA = exteriorFaceIdA;

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

                    for (uint exteriorFaceIdB = 0; exteriorFaceIdB < numExteriorFacesB; exteriorFaceIdB++)
                    {
                        triPair.exteriorFaceIdB = exteriorFaceIdB;

                        FmExteriorFace& exteriorFaceB = meshB->exteriorFaces[triPair.exteriorFaceIdB];

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

                        triPair.pairIntersects = FmComputeTriIntersection(&triPair.triIntersectionPoints, &triPair.triA.pos0, &triPair.triB.pos0);

                        if (triPair.pairIntersects)
                        {
                            FmApplyTriIntersectionVolumeAndGradientContributions(objectPair, triPair, volContactWorkspace);
                        }
                    }
                }
            }
        }

        // Perform CCD checks
        FmCcdTerminationConditions conditions;
        conditions.impactGap = distContactBias;
        conditions.contactGap = distContactThreshold;
        conditions.maxIterations = FM_DEFAULT_MAX_CCD_ITERATIONS;

        ccdTemps->Clear();

        FmVector3 boxAComponents[3];
        boxAComponents[0] = FmInitVector3(boxA.halfWidths[0], 0.0f, 0.0f);
        boxAComponents[1] = FmInitVector3(0.0f, boxA.halfWidths[1], 0.0f);
        boxAComponents[2] = FmInitVector3(0.0f, 0.0f, boxA.halfWidths[2]);

        FmVector3 boxBComponents[3];
        boxBComponents[0] = FmInitVector3(boxB.halfWidths[0], 0.0f, 0.0f);
        boxBComponents[1] = FmInitVector3(0.0f, boxB.halfWidths[1], 0.0f);
        boxBComponents[2] = FmInitVector3(0.0f, 0.0f, boxB.halfWidths[2]);

        // Box A faces, box B verts
        for (uint boxAFaceDim = 0; boxAFaceDim < 3; boxAFaceDim++)
        {
            for (uint boxAFaceSign = 0; boxAFaceSign < 2; boxAFaceSign++)
            {
                for (uint boxBSignX = 0; boxBSignX < 2; boxBSignX++)
                {
                    for (uint boxBSignY = 0; boxBSignY < 2; boxBSignY++)
                    {
                        for (uint boxBSignZ = 0; boxBSignZ < 2; boxBSignZ++)
                        {
                            uint boxBVertexId = boxBSignX | (boxBSignY << 1) | (boxBSignZ << 2);

                            float signX = boxBSignX ? 1.0f : -1.0f;
                            float signY = boxBSignY ? 1.0f : -1.0f;
                            float signZ = boxBSignZ ? 1.0f : -1.0f;

                            FmVector3 boxBVertexPos = boxBComponents[0] * signX + boxBComponents[1] * signY + boxBComponents[2] * signZ;

                            FmCcdResult& contact = ccdTemps->boxBoxContacts[ccdTemps->numCcdContacts];

                            FmBoxFaceBoxVertexCcd(&contact, boxA, boxAFaceDim, boxAFaceSign, boxB, boxBVertexId, boxBVertexPos, timestep, conditions);

                            if (contact.retVal != FM_CCD_RET_NO_IMPACT && contact.distance > 0.0f && contact.featurePair.itype == FM_FEATURE_TYPE_FACE)
                            {
                                ccdTemps->numCcdContacts++;
                            }
                        }
                    }
                }
            }
        }

        // Box B faces, box A verts
        for (uint boxBFaceDim = 0; boxBFaceDim < 3; boxBFaceDim++)
        {
            for (uint boxBFaceSign = 0; boxBFaceSign < 2; boxBFaceSign++)
            {
                for (uint boxASignX = 0; boxASignX < 2; boxASignX++)
                {
                    for (uint boxASignY = 0; boxASignY < 2; boxASignY++)
                    {
                        for (uint boxASignZ = 0; boxASignZ < 2; boxASignZ++)
                        {
                            uint boxAVertexId = boxASignX | (boxASignY << 1) | (boxASignZ << 2);

                            float signX = boxASignX ? 1.0f : -1.0f;
                            float signY = boxASignY ? 1.0f : -1.0f;
                            float signZ = boxASignZ ? 1.0f : -1.0f;

                            FmVector3 boxAVertexPos = boxAComponents[0] * signX + boxAComponents[1] * signY + boxAComponents[2] * signZ;

                            FmCcdResult boxBAContact;

                            FmBoxFaceBoxVertexCcd(&boxBAContact, boxB, boxBFaceDim, boxBFaceSign, boxA, boxAVertexId, boxAVertexPos, timestep, conditions);

                            if (boxBAContact.retVal != FM_CCD_RET_NO_IMPACT && boxBAContact.distance > 0.0f && boxBAContact.featurePair.itype == FM_FEATURE_TYPE_FACE)
                            {
                                FmCcdResult& contact = ccdTemps->boxBoxContacts[ccdTemps->numCcdContacts];
                                contact.distance = boxBAContact.distance;
                                contact.numIterations = boxBAContact.numIterations;
                                contact.time = boxBAContact.time;
                                contact.direction = -boxBAContact.direction;
                                contact.posi = boxBAContact.posj;
                                contact.posj = boxBAContact.posi;
                                contact.featurePair = FmReverseFeaturePair(boxBAContact.featurePair);
                                contact.retVal = boxBAContact.retVal;

                                ccdTemps->numCcdContacts++;
                            }
                        }
                    }
                }
            }
        }

        // Box edge pairs
        FmBoxFeature boxAEdgeFeature;
        boxAEdgeFeature.axisMask = 0x7;

        for (uint boxADim = 0; boxADim < 3; boxADim++)
        {
            for (uint boxASign1 = 0; boxASign1 < 2; boxASign1++)
            {
                for (uint boxASign2 = 0; boxASign2 < 2; boxASign2++)
                {
                    uint boxADim1 = (boxADim + 1) % 3;
                    uint boxADim2 = (boxADim + 2) % 3;

                    boxAEdgeFeature.axisMask = (uint)(~(1 << boxADim) & 0x7);
                    boxAEdgeFeature.signBits = (boxASign1 << boxADim1) | (boxASign2 << boxADim2);
                    boxAEdgeFeature.dim = boxADim;

                    float boxAfsign1 = boxASign1 ? 1.0f : -1.0f;
                    float boxAfsign2 = boxASign2 ? 1.0f : -1.0f;

                    FmVector3 boxAEdgePos0 = -boxAComponents[boxADim] + boxAComponents[boxADim1] * boxAfsign1 + boxAComponents[boxADim2] * boxAfsign2;
                    FmVector3 boxAEdgePos1 = boxAComponents[boxADim] + boxAComponents[boxADim1] * boxAfsign1 + boxAComponents[boxADim2] * boxAfsign2;

                    FmBoxFeature boxBEdgeFeature;
                    boxBEdgeFeature.axisMask = 0x7;

                    for (uint boxBDim = 0; boxBDim < 3; boxBDim++)
                    {
                        for (uint boxBSign1 = 0; boxBSign1 < 2; boxBSign1++)
                        {
                            for (uint boxBSign2 = 0; boxBSign2 < 2; boxBSign2++)
                            {
                                uint boxBDim1 = (boxBDim + 1) % 3;
                                uint boxBDim2 = (boxBDim + 2) % 3;

                                boxBEdgeFeature.axisMask = (uint)(~(1 << boxBDim) & 0x7);
                                boxBEdgeFeature.signBits = (boxBSign1 << boxBDim1) | (boxBSign2 << boxBDim2);
                                boxBEdgeFeature.dim = boxBDim;

                                float boxBfsign1 = boxBSign1 ? 1.0f : -1.0f;
                                float boxBfsign2 = boxBSign2 ? 1.0f : -1.0f;

                                FmVector3 boxBEdgePos0 = -boxBComponents[boxBDim] + boxBComponents[boxBDim1] * boxBfsign1 + boxBComponents[boxBDim2] * boxBfsign2;
                                FmVector3 boxBEdgePos1 = boxBComponents[boxBDim] + boxBComponents[boxBDim1] * boxBfsign1 + boxBComponents[boxBDim2] * boxBfsign2;

                                FmCcdResult& contact = ccdTemps->boxBoxContacts[ccdTemps->numCcdContacts];

                                FmBoxEdgeBoxEdgeCcd(&contact, boxA, boxAEdgeFeature, boxAEdgePos0, boxAEdgePos1, boxB, boxBEdgeFeature, boxBEdgePos0, boxBEdgePos1, timestep, conditions);

                                if (contact.retVal != FM_CCD_RET_NO_IMPACT && contact.distance > 0.0f)
                                {
                                    ccdTemps->numCcdContacts++;
                                }
                            }
                        }
                    }
                }
            }
        }

        for (uint cId = 0; cId < ccdTemps->numCcdContacts; cId++)
        {
            FmCcdResult& boxBoxContact = ccdTemps->boxBoxContacts[cId];

            assert(!(boxBoxContact.retVal == FM_CCD_RET_NO_IMPACT || boxBoxContact.distance <= 0.0f));

            // Create tet pair from tri contact
            FmDistanceContactPairInfo contactPairInfo;
            contactPairInfo.idInObjectPair = objectPair->numDistanceContacts;
            contactPairInfo.objectIdA = rigidBodyIdA;
            contactPairInfo.objectIdB = rigidBodyIdB;
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
            contact.tetIdA = (boxBoxContact.featurePair.i1 << 3) | boxBoxContact.featurePair.i0;  // box feature
            contact.tetIdB = (boxBoxContact.featurePair.j1 << 3) | boxBoxContact.featurePair.j0;  // box feature
            contact.frictionCoeff = FmMinFloat(rigidBodyA->frictionCoeff, rigidBodyB->frictionCoeff);

            // Found a contact, calculate COM to contact vectors
            FmVector3 boxAPosAdv = boxA.state.pos + boxA.state.vel * boxBoxContact.time;
            FmQuat    boxAQuatAdv = FmIntegrateQuat(boxA.state.quat, boxA.state.angVel, boxBoxContact.time);
            FmVector3 boxAComToContactAdv = boxBoxContact.posi - boxAPosAdv;
            FmVector3 boxAComToContactAdvBoxSpace = rotate(conj(boxAQuatAdv), boxAComToContactAdv);
            FmVector3 boxAComToContact = rotate(boxA.state.quat, boxAComToContactAdvBoxSpace);

            contact.comToPosA[0] = boxAComToContact.x;
            contact.comToPosA[1] = boxAComToContact.y;
            contact.comToPosA[2] = boxAComToContact.z;

            FmVector3 posA = boxA.state.pos + boxAComToContact;
            FmVector3 velA = boxA.state.vel + cross(boxA.state.angVel, boxAComToContact);

            FmVector3 boxBPosAdv = boxB.state.pos + boxB.state.vel * boxBoxContact.time;
            FmQuat    boxBQuatAdv = FmIntegrateQuat(boxB.state.quat, boxB.state.angVel, boxBoxContact.time);
            FmVector3 boxBComToContactAdv = boxBoxContact.posj - boxBPosAdv;
            FmVector3 boxBComToContactAdvBoxSpace = rotate(conj(boxBQuatAdv), boxBComToContactAdv);
            FmVector3 boxBComToContact = rotate(boxB.state.quat, boxBComToContactAdvBoxSpace);

            contact.comToPosB[0] = boxBComToContact.x;
            contact.comToPosB[1] = boxBComToContact.y;
            contact.comToPosB[2] = boxBComToContact.z;

            FmVector3 posB = boxB.state.pos + boxBComToContact;
            FmVector3 velB = boxB.state.vel + cross(boxB.state.angVel, boxBComToContact);

            contact.normal = -boxBoxContact.direction;
            contact.normalProjDistance = dot(contact.normal, posA - posB) - distContactBias;
            contact.normalProjRelVel = dot(contact.normal, velB - velA);
            contact.tangent1 = FmGetContactTangent(contact.normal, velA, velB);

            uint dynamicFlagsA, movingFlagsA, dynamicFlagsB, movingFlagsB;

            FmDynamicAndMovingFlags(&dynamicFlagsA, &movingFlagsA, *rigidBodyA);
            FmDynamicAndMovingFlags(&dynamicFlagsB, &movingFlagsB, *rigidBodyB);

            FmSetConstraintFlags(&contactPairInfo.flags, &contactPairInfo.dynamicFlags, &contact.movingFlags, dynamicFlagsA, dynamicFlagsB, movingFlagsA, movingFlagsB);

            bool exterior = true;

            bool exteriorToBoxAFace = false;

            uint boxASignBits = boxBoxContact.featurePair.i0;
            uint boxAAxisMask = boxBoxContact.featurePair.i1;

            float boxASignX = (boxASignBits & 0x1) ? 1.0f : -1.0f;
            float boxASignY = (boxASignBits & 0x2) ? 1.0f : -1.0f;
            float boxASignZ = (boxASignBits & 0x4) ? 1.0f : -1.0f;

            FmMatrix3 boxARot = FmInitMatrix3(boxA.state.quat);

            if ((boxAAxisMask & 0x1) && dot(contact.normal, boxASignX * boxARot.col0) < 0.0f)
            {
                exteriorToBoxAFace = true;
            }
            if ((boxAAxisMask & 0x2) && dot(contact.normal, boxASignY * boxARot.col1) < 0.0f)
            {
                exteriorToBoxAFace = true;
            }
            if ((boxAAxisMask & 0x4) && dot(contact.normal, boxASignZ * boxARot.col2) < 0.0f)
            {
                exteriorToBoxAFace = true;
            }

            bool exteriorToBoxBFace = false;

            uint boxBSignBits = boxBoxContact.featurePair.j0;
            uint boxBAxisMask = boxBoxContact.featurePair.j1;

            float boxBSignX = (boxBSignBits & 0x1) ? 1.0f : -1.0f;
            float boxBSignY = (boxBSignBits & 0x2) ? 1.0f : -1.0f;
            float boxBSignZ = (boxBSignBits & 0x4) ? 1.0f : -1.0f;

            FmMatrix3 boxBRot = FmInitMatrix3(boxB.state.quat);

            if ((boxBAxisMask & 0x1) && dot(contact.normal, boxBSignX * boxBRot.col0) > 0.0f)
            {
                exteriorToBoxBFace = true;
            }
            if ((boxBAxisMask & 0x2) && dot(contact.normal, boxBSignY * boxBRot.col1) > 0.0f)
            {
                exteriorToBoxBFace = true;
            }
            if ((boxBAxisMask & 0x4) && dot(contact.normal, boxBSignZ * boxBRot.col2) > 0.0f)
            {
                exteriorToBoxBFace = true;
            }

            exterior = exteriorToBoxAFace && exteriorToBoxBFace;

            if (exterior)
            {
                FmTetVertIds tmpVertIdsA, tmpVertIdsB;
                numDistanceContacts += FmAddTempDistanceContact(objectPair, &contactPairInfo, &contact, tmpVertIdsA, tmpVertIdsB, dynamicFlagsA, dynamicFlagsB, NULL);
            }
        }
    }

    uint FmGenerateBoxCollisionPlaneContacts(
        FmCollidedObjectPair* objectPair,
        const FmRigidBody& rigidBody, const FmBox& box, const FmVector3 boxPointsBoxSpace[8],
        const FmVector3& planeNormal, const FmVector3& planePos)
    {
        float distContactBias = objectPair->distContactBias;
        float distContactThreshold = objectPair->distContactThreshold;
        float timestep = objectPair->timestep;

        FmVector3 planeTriPos = rigidBody.state.pos - planeNormal * dot(rigidBody.state.pos - planePos, planeNormal);
        FmVector3 planeTangent1 = normalize(FmOrthogonalVector(planeNormal));
        FmVector3 planeTangent2 = cross(planeNormal, planeTangent1);

        FmTri planeTri;
        planeTri.pos0 = planeTriPos - planeTangent1 * 1000.0f - planeTangent2 * 1000.0f;
        planeTri.pos1 = planeTriPos + planeTangent1 * 1000.0f - planeTangent2 * 1000.0f;
        planeTri.pos2 = planeTriPos + planeTangent1 * 1000.0f + planeTangent2 * 1000.0f;
        planeTri.vel0 = planeTri.vel1 = planeTri.vel2 = FmInitVector3(0.0f);

        FmCcdTerminationConditions conditions;
        conditions.impactGap = distContactBias;
        conditions.contactGap = distContactThreshold;
        conditions.maxIterations = FM_DEFAULT_MAX_CCD_ITERATIONS;

        FmCcdResult ccdContact;

        uint numContacts = 0;

        for (uint i = 0; i < 8; i++)
        {
            FmVector3 comToContact = rotate(rigidBody.state.quat, boxPointsBoxSpace[i]);
            FmVector3 boxPointStart = rigidBody.state.pos + comToContact;
            float normalProjDistance = dot(boxPointStart - planePos, planeNormal);

            bool isContact = false;
            if (normalProjDistance <= distContactThreshold)
            {
                isContact = true;
            }
            else
            {
                FmBoxVertexTriCcd(&ccdContact, box, i, boxPointsBoxSpace[i], planeTri, timestep, conditions);
                if (ccdContact.retVal != FM_CCD_RET_NO_IMPACT)
                {
                    isContact = true;

                    FmVector3 posAdv = rigidBody.state.pos + rigidBody.state.vel * ccdContact.time;
                    FmQuat    quatAdv = FmIntegrateQuat(rigidBody.state.quat, rigidBody.state.angVel, ccdContact.time);
                    FmVector3 comToContactAdv = ccdContact.posi - posAdv;
                    FmVector3 comToContactAdvBoxSpace = rotate(conj(quatAdv), comToContactAdv);
                    comToContact = rotate(rigidBody.state.quat, comToContactAdvBoxSpace);
                }
            }

            if (isContact)
            {
                FmVector3 posStart = rigidBody.state.pos + comToContact;
                FmVector3 vel = rigidBody.state.vel + cross(rigidBody.state.angVel, comToContact);

                FmDistanceContactPairInfo contactPairInfo;
                contactPairInfo.idInObjectPair = objectPair->numDistanceContacts;
                contactPairInfo.objectIdA = rigidBody.objectId;
                contactPairInfo.objectIdB = FM_INVALID_ID;

                FmDistanceContact contact;
                contact.tetIdA = (0x7 << 3) | i; // box feature
                contact.tetIdB = FM_INVALID_ID;
                contact.normal = planeNormal;
                contact.comToPosA[0] = comToContact.x;
                contact.comToPosA[1] = comToContact.y;
                contact.comToPosA[2] = comToContact.z;
                contact.frictionCoeff = rigidBody.frictionCoeff;

                contact.normalProjDistance = normalProjDistance - distContactBias;
                contact.normalProjRelVel = -dot(planeNormal, vel);

                FmVector3 tangentVelBA = FmGetContactTangent(planeNormal, vel);

                contact.tangent1 = tangentVelBA;

                uint dynamicFlagsA, movingFlagsA;

                FmDynamicAndMovingFlags(&dynamicFlagsA, &movingFlagsA, rigidBody);

                FmSetConstraintFlags(&contactPairInfo.flags, &contactPairInfo.dynamicFlags, &contact.movingFlags, dynamicFlagsA, 0, movingFlagsA, 0);
                contactPairInfo.flags |= FM_CONSTRAINT_FLAG_OBJECTB_COLLISION_PLANE;

                FmTetVertIds tmpVertIds;
                numContacts += FmAddTempDistanceContact(objectPair, &contactPairInfo, &contact, tmpVertIds, tmpVertIds, dynamicFlagsA, 0, NULL);
            }
        }

        return numContacts;
    }

    uint FmFindSceneCollisionPlanesContactsRb(
        FmCollidedObjectPair* objectPair,
        const FmRigidBody& rigidBody,
        const FmSceneCollisionPlanes& collisionPlanes)
    {
        float timestep = objectPair->timestep;
        float distContactThreshold = objectPair->distContactThreshold;

        if (FM_IS_SET(rigidBody.flags, FM_OBJECT_FLAG_KINEMATIC))
        {
            return 0;
        }

        FmBox box;
        box.state = rigidBody.state;
        box.halfWidths[0] = rigidBody.dims[0];
        box.halfWidths[1] = rigidBody.dims[1];
        box.halfWidths[2] = rigidBody.dims[2];

        FmAabb aabb = FmComputeBoxAabb(box, timestep, 0.0f);

        bool canCollide[6];
        if (!FmAabbSceneCollisionPlanesPotentialContact(canCollide, aabb, collisionPlanes, timestep, distContactThreshold))
        {
            return 0;
        }

        FmVector3 boxPointsBoxSpace[8];

        boxPointsBoxSpace[0] = FmInitVector3(-rigidBody.dims[0], -rigidBody.dims[1], -rigidBody.dims[2]);
        boxPointsBoxSpace[1] = FmInitVector3(rigidBody.dims[0], -rigidBody.dims[1], -rigidBody.dims[2]);
        boxPointsBoxSpace[2] = FmInitVector3(-rigidBody.dims[0], rigidBody.dims[1], -rigidBody.dims[2]);
        boxPointsBoxSpace[3] = FmInitVector3(rigidBody.dims[0], rigidBody.dims[1], -rigidBody.dims[2]);
        boxPointsBoxSpace[4] = FmInitVector3(-rigidBody.dims[0], -rigidBody.dims[1], rigidBody.dims[2]);
        boxPointsBoxSpace[5] = FmInitVector3(rigidBody.dims[0], -rigidBody.dims[1], rigidBody.dims[2]);
        boxPointsBoxSpace[6] = FmInitVector3(-rigidBody.dims[0], rigidBody.dims[1], rigidBody.dims[2]);
        boxPointsBoxSpace[7] = FmInitVector3(rigidBody.dims[0], rigidBody.dims[1], rigidBody.dims[2]);

        uint numContacts = 0;

        if (canCollide[0])
        {
            numContacts += FmGenerateBoxCollisionPlaneContacts(objectPair, 
                rigidBody, box, boxPointsBoxSpace, FmInitVector3(1.0f, 0.0f, 0.0f), FmInitVector3(collisionPlanes.minX, 0.0f, 0.0f));
        }
        if (canCollide[1])
        {
            numContacts += FmGenerateBoxCollisionPlaneContacts(objectPair,
                rigidBody, box, boxPointsBoxSpace, FmInitVector3(-1.0f, 0.0f, 0.0f), FmInitVector3(collisionPlanes.maxX, 0.0f, 0.0f));
        }
        if (canCollide[2])
        {
            numContacts += FmGenerateBoxCollisionPlaneContacts(objectPair,
                rigidBody, box, boxPointsBoxSpace, FmInitVector3(0.0f, 1.0f, 0.0f), FmInitVector3(0.0f, collisionPlanes.minY, 0.0f));
        }
        if (canCollide[3])
        {
            numContacts += FmGenerateBoxCollisionPlaneContacts(objectPair,
                rigidBody, box, boxPointsBoxSpace, FmInitVector3(0.0f, -1.0f, 0.0f), FmInitVector3(0.0f, collisionPlanes.maxY, 0.0f));
        }
        if (canCollide[4])
        {
            numContacts += FmGenerateBoxCollisionPlaneContacts(objectPair,
                rigidBody, box, boxPointsBoxSpace, FmInitVector3(0.0f, 0.0f, 1.0f), FmInitVector3(0.0f, 0.0f, collisionPlanes.minZ));
        }
        if (canCollide[5])
        {
            numContacts += FmGenerateBoxCollisionPlaneContacts(objectPair,
                rigidBody, box, boxPointsBoxSpace, FmInitVector3(0.0f, 0.0f, -1.0f), FmInitVector3(0.0f, 0.0f, collisionPlanes.maxZ));
        }

        // Copy any temp contacts to global list
        if (objectPair->temps.numDistanceContacts > 0)
        {
            FmCopyTempDistanceContacts(objectPair);
        }

        return numContacts;
    }

    uint FmFindContactsTetMeshBox(FmCollidedObjectPair* objectPair)
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
        for (uint exteriorFaceIdA = 0; exteriorFaceIdA < meshA->numExteriorFaces; exteriorFaceIdA++)
        {
            FmGenerateContactsTriBox(objectPair, exteriorFaceIdA);
        }

        // Create volume contact.
        uint numVolumeContacts = FmFinalizeVolumeContact(objectPair);

        // Copy any temp contacts to global list
        if (objectPair->temps.numDistanceContacts > 0)
        {
            FmCopyTempDistanceContacts(objectPair);
        }

        return objectPair->numDistanceContacts + numVolumeContacts;
    }

    uint FmFindContactsBoxBox(FmCollidedObjectPair* objectPair, FmBoxBoxCcdTemps* ccdTemps)
    {
        FmTetMesh* meshA = objectPair->tetMeshA;
        FmTetMesh* meshB = objectPair->tetMeshB;
        FmVolumeContactWorkspace* volContactWorkspace = objectPair->temps.volContactWorkspace;

        // Sum the point vs. face "shadow" results to determine whether each vertex is inside other mesh.
        FmFindInsideVerts(objectPair);

        // For inside vertices, apply volume and gradient contributions to any incident faces
        FmApplyInsideVertVolumeAndGradientContributions(objectPair, meshA, volContactWorkspace->meshAVertSets, 1.0f);
        FmApplyInsideVertVolumeAndGradientContributions(objectPair, meshB, volContactWorkspace->meshBVertSets, -1.0f);

        FmGenerateContactsBoxBox(objectPair, ccdTemps);

        // Create volume contact.
        uint numVolumeContacts = FmFinalizeVolumeContact(objectPair);

        // Copy any temp contacts to global list
        if (objectPair->temps.numDistanceContacts > 0)
        {
            FmCopyTempDistanceContacts(objectPair);
        }

        return objectPair->numDistanceContacts + numVolumeContacts;
    }

}

