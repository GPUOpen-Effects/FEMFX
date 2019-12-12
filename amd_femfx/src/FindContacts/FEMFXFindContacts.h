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

#pragma once

#include "FEMFXCommonInternal.h"
#include "FEMFXCollisionPairData.h"
#include "FEMFXTetMesh.h"
#include "FEMFXAsyncThreading.h"

namespace AMD
{
    struct FmSceneCollisionPlanes;
    struct FmBoxBoxCcdTemps;
    struct FmConstraintsBuffer;
    struct FmCollisionReport;
    struct FmRigidBody;
    struct FmScene;
    struct FmContactReductionInputs;

    class FmTaskDataFindContacts : public FmAsyncTaskData
    {
    public:
        FM_CLASS_NEW_DELETE(FmTaskDataFindContacts)

        uint numBroadPhasePairs;
        FmScene* scene;
        float timestep;
        uint* tetMeshIds;
        uint* rigidBodyIds;
        uint numTetMeshes;
        uint numRigidBodies;

        uint numObjects;
        uint numTasks;

        bool includeRigidBodiesInBroadPhase;
        bool collideRigidBodiesWithPlanes;
        bool testWithSleepingObjects;

        FmTaskDataFindContacts(
            uint inNumBroadPhasePairs,
            FmScene* inScene,
            float inTimestep,
            uint* inTetMeshIds, uint* inRigidBodyIds,
            uint inNumTetMeshes, uint inNumRigidBodies,
            bool inIncludeRigidBodiesInBroadPhase,
            bool inCollideRigidBodiesWithPlanes,
            bool inTestWithSleepingObjects)
        {
            numBroadPhasePairs = inNumBroadPhasePairs;
            scene = inScene;
            timestep = inTimestep;
            tetMeshIds = inTetMeshIds;
            rigidBodyIds = inRigidBodyIds;
            numTetMeshes = inNumTetMeshes;
            numRigidBodies = inNumRigidBodies;

            includeRigidBodiesInBroadPhase = inIncludeRigidBodiesInBroadPhase;
            collideRigidBodiesWithPlanes = inCollideRigidBodiesWithPlanes;
            testWithSleepingObjects = inTestWithSleepingObjects;
        }
    };

    static FM_FORCE_INLINE void FmGetExteriorFaceVertIds(FmFaceVertIds* faceVertIds, const FmTetMesh& tetMesh, uint exteriorFaceId)
    {
        const FmExteriorFace& exteriorFace = tetMesh.exteriorFaces[exteriorFaceId];

        FmTetVertIds tetVerts = tetMesh.tetsVertIds[exteriorFace.tetId];
        FmFaceVertIds faceVerts;
        FmGetFaceVertIds(&faceVerts, exteriorFace.faceId, tetVerts);

        *faceVertIds = faceVerts;
    }

    static FM_FORCE_INLINE FmVector3 FmExteriorFaceCrossProd(const FmTetMesh& tetMesh, uint exteriorFaceId)
    {
        const FmExteriorFace& exteriorFace = tetMesh.exteriorFaces[exteriorFaceId];

        FmTetVertIds tetVerts = tetMesh.tetsVertIds[exteriorFace.tetId];
        FmFaceVertIds faceVerts;
        FmGetFaceVertIds(&faceVerts, exteriorFace.faceId, tetVerts);

        FmVector3 pos0 = tetMesh.vertsPos[faceVerts.ids[0]];
        FmVector3 pos1 = tetMesh.vertsPos[faceVerts.ids[1]];
        FmVector3 pos2 = tetMesh.vertsPos[faceVerts.ids[2]];
        return cross(pos1 - pos0, pos2 - pos0);
    }

    bool FmIsVertNormalExterior(const FmTetMesh& tetMesh, uint exteriorFaceId, uint vertId, const FmVector3& direction);
    bool FmIsEdgeNormalExterior(const FmTetMesh& tetMesh, uint exteriorFaceId, uint edgeId, const FmVector3& direction);
    FmVector3 FmComputeVertNormal(const FmTetMesh& tetMesh, uint exteriorFaceId, uint vertId);
    FmVector3 FmComputeEdgeNormal(const FmTetMesh& tetMesh, uint exteriorFaceId, uint edgeId);

    bool FmAabbSceneCollisionPlanesPotentialContact(bool results[6], const FmAabb& aabb, const FmSceneCollisionPlanes& planes, float timestep, float contactThreshold);

    void FmUpdateVertIntersectionVals(FmCollidedObjectPair* collidedPair, uint exteriorFaceIdA, uint exteriorFaceIdB, bool includeA, bool includeB);

    void FmGenerateContacts(FmCollidedObjectPair* collidedPair, uint exteriorFaceIdA, uint exteriorFaceIdB);

    void FmGenerateContactsFromMeshCollisionTriPairs(FmCollidedObjectPair* collidedPair);

    uint FmGenerateFaceCollisionPlaneContacts(
        FmCollidedObjectPair* objectPair,
        bool canCollide[6],
        uint extFaceId, const FmSceneCollisionPlanes& collisionPlanes);

    uint FmFindSceneCollisionPlanesContactsQuery(
        FmCollidedObjectPair* objectPair,
        const FmSceneCollisionPlanes& collisionPlanes);

    uint FmFindSceneCollisionPlanesContactsRb(
        FmCollidedObjectPair* objectPair,
        const FmRigidBody& rigidBody,
        const FmSceneCollisionPlanes& collisionPlanes);

    uint FmFindContactsBoxBox(FmCollidedObjectPair* objectPair, FmBoxBoxCcdTemps* ccdTemps);

    uint FmFindContactsTetMeshBox(FmCollidedObjectPair* objectPair);

    // Functions to obtain barycentric coordinates of a point on a line segment or triangle.
    // Used to go from triangle contact points to the tetrahedra barycentic coords used in Contact.
    // Assumes points are on features.
    void FmComputeLineSegmentBarycentricCoords(float barycentricCoords[2], const FmVector3& pos, const FmVector3& segmentPos0, const FmVector3& segmentPos1);
    void FmComputeTriBarycentricCoords(float barycentricCoords[3], const FmVector3& pos, const FmVector3& triPos0, const FmVector3& triPos1, const FmVector3& triPos2);

    // Collide two BVHs to find verts of each mesh inside the other.
    void FmFindInsideVerts(FmCollidedObjectPair* meshPair);

    // Collide two BVHs to find primitive pairs whose boxes overlap
    void FmCollideHierarchies(FmCollidedObjectPair* meshPair);

    // Add contact to temp contacts buffer
    bool FmAddTempDistanceContact(
        FmCollidedObjectPair* collidedPair,
        FmDistanceContactPairInfo* contactPairInfo,
        FmDistanceContact* contact,
        const FmTetVertIds& tetVertIdsA,
        const FmTetVertIds& tetVertIdsB,
        uint dynamicFlagsA,
        uint dynamicFlagsB,
        FmContactReductionInputs* contactReductionInputs);  // only supported for tet mesh pair

    // Copy the temp buffer of distance contacts into the constraints buffer array.
    // Update the contact reduction data.
    bool FmCopyTempDistanceContacts(FmCollidedObjectPair* collidedPair);

    void FmApplyInsideVertVolumeAndGradientContributions(FmCollidedObjectPair* objectPair, FmTetMesh* tetMesh, FmVolumeContactVertSetExpandable& volContactVerts, float normalScale);
    void FmApplyTriIntersectionVolumeAndGradientContributions(FmCollidedObjectPair* objectPair, FmMeshCollisionTriPair& triPair, FmVolumeContactWorkspace* volContactWorkspace);
    uint FmFinalizeVolumeContact(FmCollidedObjectPair* meshPair);

    void FmSelfCollideHierarchies(FmCollidedObjectPair* meshPair);

    void FmSelfCollFindInsideVerts(FmCollidedObjectPair* meshPair);
    void FmSelfCollApplyTriIntersectionVolumeAndGradientContributions(FmCollidedObjectPair* objectPair, FmMeshCollisionTriPair& triPair, FmVolumeContactWorkspace* volContactWorkspace);
    uint FmFindSelfContacts(FmCollidedObjectPair* meshPair);
}