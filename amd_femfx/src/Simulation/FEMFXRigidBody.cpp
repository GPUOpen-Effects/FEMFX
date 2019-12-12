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

#include "AMD_FEMFX.h"
#include "FEMFXRigidBody.h"
#include "FEMFXSleeping.h"

namespace AMD
{
    // Allocate and initialize rigid body
    FmRigidBody* FmCreateRigidBody(const FmRigidBodySetupParams& setupParams)
    {
        FmRigidBody* rigidBody = (FmRigidBody*)FmAlignedMalloc(sizeof(FmRigidBody), 64);

        if (rigidBody == NULL)
        {
            return NULL;
        }

        rigidBody->Init();
        rigidBody->mass = setupParams.mass;
        rigidBody->bodyInertiaTensor = setupParams.bodyInertiaTensor;
        rigidBody->state = setupParams.state;
        rigidBody->SetDimensions(setupParams.halfDimX, setupParams.halfDimY, setupParams.halfDimZ);

        if (setupParams.isKinematic)
        {
            rigidBody->flags |= FM_OBJECT_FLAG_KINEMATIC;
        }

        rigidBody->collisionGroup = setupParams.collisionGroup;

        FmCreateRigidBodyBoxCollObj(rigidBody);

        return rigidBody;
    }

    // Free rigid body memory
    void FmDestroyRigidBody(FmRigidBody* rigidBody)
    {
        if (rigidBody)
        {
            FmDestroyRigidBodyCollObj(rigidBody);
            FmAlignedFree(rigidBody);
        }
    }

    uint FmGetObjectId(const FmRigidBody& rigidBody)
    {
        return rigidBody.objectId;
    }

    FmVector3 FmGetPosition(const FmRigidBody& rigidBody)
    {
        return rigidBody.state.pos;
    }

    FmVector3 FmGetVelocity(const FmRigidBody& rigidBody)
    {
        return rigidBody.state.vel;
    }

    FmQuat FmGetRotation(const FmRigidBody& rigidBody)
    {
        return rigidBody.state.quat;
    }

    FmVector3 FmGetAngularVelocity(const FmRigidBody& rigidBody)
    {
        return rigidBody.state.angVel;
    }

    FmRigidBodyState FmGetState(const FmRigidBody& rigidBody)
    {
        return rigidBody.state;
    }

    void FmSetPosition(FmScene* scene, FmRigidBody* rigidBody, const FmVector3& position)
    {
        if (rigidBody == NULL)
        {
            return;
        }

        rigidBody->state.pos = position;

        if (scene)
        {
            FmMarkIslandOfObjectForWaking(scene, rigidBody->objectId);
        }
    }

    void FmSetVelocity(FmScene* scene, FmRigidBody* rigidBody, const FmVector3& velocity)
    {
        if (rigidBody == NULL)
        {
            return;
        }

        rigidBody->state.vel = velocity;

        if (scene)
        {
            FmMarkIslandOfObjectForWaking(scene, rigidBody->objectId);
        }
    }

    void FmSetRotation(FmScene* scene, FmRigidBody* rigidBody, const FmQuat& rotation)
    {
        if (rigidBody == NULL)
        {
            return;
        }

        rigidBody->state.quat = rotation;

        if (scene)
        {
            FmMarkIslandOfObjectForWaking(scene, rigidBody->objectId);
        }
    }

    void FmSetAngularVelocity(FmScene* scene, FmRigidBody* rigidBody, const FmVector3& angularVelocity)
    {
        if (rigidBody == NULL)
        {
            return;
        }

        rigidBody->state.angVel = angularVelocity;

        if (scene)
        {
            FmMarkIslandOfObjectForWaking(scene, rigidBody->objectId);
        }
    }

    void FmSetState(FmScene* scene, FmRigidBody* rigidBody, const FmRigidBodyState& state)
    {
        if (rigidBody == NULL)
        {
            return;
        }

        rigidBody->state = state;

        if (scene)
        {
            // If part of sleeping island, mark for waking
            FmMarkIslandOfObjectForWaking(scene, rigidBody->objectId);
        }
    }

    void FmSetCollisionGroup(FmRigidBody* rigidBody, uint8_t collisionGroup)
    {
        if (rigidBody == NULL)
        {
            return;
        }

        rigidBody->collisionGroup = collisionGroup;
    }

    void FmSetGravity(FmRigidBody* rigidBody, const FmVector3& gravityVector)
    {
        if (rigidBody == NULL)
        {
            return;
        }

        rigidBody->gravityVector = gravityVector;
    }

    void FmSetAabb(FmRigidBody* rigidBody, const FmAabb& aabb)
    {
        if (rigidBody)
        {
            rigidBody->aabb = aabb;
        }
    }

    void FmSetRigidBodyIsland(FmRigidBody* rigidBody, uint rigidBodyIslandId)
    {
        if (rigidBody)
        {
            rigidBody->rbIslandId = rigidBodyIslandId;
        }
    }
}