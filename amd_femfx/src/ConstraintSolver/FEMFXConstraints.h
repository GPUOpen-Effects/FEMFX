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
// Contact and other constraints types supported by the constraint solve
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXCommonInternal.h"
#include "FEMFXVectorMath.h"
#include "FEMFXTetMeshConnectivity.h"
#include "FEMFXRandom.h"

#define FM_CONSTRAINT_FLAG_OBJECTA_FIXED            0x1          // Object A state in constraint cannot be changed
#define FM_CONSTRAINT_FLAG_OBJECTB_FIXED            0x2          // Object B state in constraint cannot be changed
#define FM_CONSTRAINT_FLAG_OBJECTA_ZEROVEL          0x4          // Object A state in constraint has zero velocity
#define FM_CONSTRAINT_FLAG_OBJECTB_ZEROVEL          0x8          // Object B state in constraint has zero velocity
#define FM_CONSTRAINT_FLAG_1D                       0x10         // Constraint is 1D (3D is default)
#define FM_CONSTRAINT_FLAG_2D                       0x20         // Constraint is 2D (3D is default)
#define FM_CONSTRAINT_FLAG_NONNEG0                  0x40         // Multiplier for dimension 0 of constraint must be non-negative (otherwise not limited)
#define FM_CONSTRAINT_FLAG_NONNEG1                  0x80         // Multiplier for dimension 1 of constraint must be non-negative (otherwise not limited)
#define FM_CONSTRAINT_FLAG_NONNEG2                  0x100        // Multiplier for dimension 2 of constraint must be non-negative (otherwise not limited)
#define FM_CONSTRAINT_FLAG_DISABLED                 0x200        // User-controlled flag to disable glue, plane, or angle constraints
#define FM_CONSTRAINT_FLAG_DELETED                  0x400        // Marks constraint as deleted
#define FM_CONSTRAINT_FLAG_OBJECTB_COLLISION_PLANE  0x800        // Marks contact with collision plane

namespace AMD
{
    // Constraint type enum
    // Fracture contacts are distance contacts added at tet faces where fracture occurs.
    // They exist for only one step but may help reduce interpenetration that would occur
    // immediately following fracture.
    enum FmConstraintType
    {
        // Managed by library
        FM_CONSTRAINT_TYPE_DISTANCE_CONTACT,
        FM_CONSTRAINT_TYPE_VOLUME_CONTACT,
        FM_CONSTRAINT_TYPE_DEFORMATION,

        // Managed by application
        FM_CONSTRAINT_TYPE_GLUE,
        FM_CONSTRAINT_TYPE_PLANE,
        FM_CONSTRAINT_TYPE_RIGID_BODY_ANGLE,

        FM_NUM_CONSTRAINT_TYPES
    };

    // Contact type used to constrain distance along a normal direction.
    struct FmDistanceContactPairInfo
    {
        uint         idInObjectPair;       // Id within the object pair, to help with sorting
        uint         objectIdA;            // Tet mesh id, or rigid body id if FM_RB_FLAG set
        uint         objectIdB;            // Tet mesh id, or rigid body id if FM_RB_FLAG set, or fixed point if FM_INVALID_ID
        uint16_t     flags;                // Bitwise or of FM_CONSTRAINT_FLAG_* values
        uint8_t      dynamicFlags;         // Bits set if tet vertices are dynamic and barycentric != 0
        uint8_t      refCount;             // 0-8 reference count of vertices used in contact reduction

        FmDistanceContactPairInfo()
        {
            idInObjectPair = FM_INVALID_ID;
            objectIdA = FM_INVALID_ID;
            objectIdB = FM_INVALID_ID;
            flags = 0;
            dynamicFlags = 0;
            refCount = 1;
        }
    };

    // Contact type used to constrain distance along a normal direction.
    struct FmDistanceContact
    {
        union
        {
            float    posBaryA[4];          // Barycentric coordinate of contact point on tet mesh A
            float    comToPosA[4];         // Vector from center of mass to contact point on rigid body A
        };
        union
        {
            float    posBaryB[4];          // Barycentric coordinate of contact point on tet mesh B
            float    comToPosB[4];         // Vector from center of mass to contact point on rigid body B
        };
        FmVector3    normal;               // Normal of contact plane pointing out from body B (multiple of this is normal force on body A)
        FmVector3    tangent1;             // Direction of posB-posA movement tangent to contact plane (multiple of this is friction force on body A)
        float        normalProjDistance;   // Initial distance of contact points projected on normal (dot(normal, posA - posB)) - goal distance
        float        normalProjRelVel;     // Relative velocity of contact points projected on normal (dot(normal, velB - velA)); positive value means approaching
        float        frictionCoeff;        // Currently same for static or dynamic case
        uint         tetIdA;               // If object A is tet mesh, contains id of tet with contact point
        uint         tetIdB;               // If object B is tet mesh, contains id of tet with contact point
        uint8_t      movingFlags;          // Bits set if tet vertices are moving and barycentric != 0

        inline FmDistanceContact()
        {
            tetIdA = FM_INVALID_ID;
            tetIdB = FM_INVALID_ID;
            posBaryA[0] = 0.0f;
            posBaryA[1] = 0.0f;
            posBaryA[2] = 0.0f;
            posBaryA[3] = 0.0f;
            posBaryB[0] = 0.0f;
            posBaryB[1] = 0.0f;
            posBaryB[2] = 0.0f;
            posBaryB[3] = 0.0f;
            normal = FmInitVector3(0.0f);
            tangent1 = FmInitVector3(0.0f);
            normalProjDistance = 0.0f;
            normalProjRelVel = 0.0f;
            frictionCoeff = FM_DEFAULT_FRICTION_COEFF;
            movingFlags = 0;
        }
    };

    // Tet mesh vertex that is part of a volume contact defined below
    struct FmVolumeContactVert
    {
        uint      vertId;        // id of vertex
        FmVector3 dVdp;          // Gradient of volume with respect to vertex position
        FmVector3 tangent;       // Tangent computed per vertex
        FmVector3 centerToVert;  // Used for rigid bodies only, give vector from center of mass to vertex
        bool      dynamic;
        bool      moving;

        inline FmVolumeContactVert()
        {
            vertId = FM_INVALID_ID;
            dVdp = FmInitVector3(0.0f);
            tangent = FmInitVector3(0.0f);
            centerToVert = FmInitVector3(0.0f);
            dynamic = false;
            moving = false; 
        }
    };

    // Contact type used to constrain volume of intersection of the mesh polyhedra.
    // Defining signed volume V as the negative of the volume of intersection.  
    // Need to compute dV/dp for each vertex to determine direction of fastest volume reduction.
    // To find V and dV/dp values, must find the faces of the intersection volume and integrate over them.
    // Collect verts with nonzero dVdp into subarray and create FmVolumeContact referencing verts.
    // Compute normal from combined dVdp values, overall relative velocity from normal and vert velocities.
    // For solve, init Jacobian from dVdp and tangent directions.
    // Reference: Allard et al., "Volume Contact Constraints at Arbitrary Resolution"
    struct FmVolumeContact
    {
        uint      idInObjectPair;          // Id within the object pair, to help with sorting
        uint      objectIdA;               // Tet mesh id, or rigid body if FM_RB_FLAG set
        uint      objectIdB;               // Tet mesh id, or rigid body if FM_RB_FLAG set
        uint      volVertsStartA;          // Start index of vertices of meshA with nonzero dVdp
        uint      volVertsStartB;          // Start index of vertices of meshB with nonzero dVdp
        uint      numVolVertsA;            // Number of vertices of meshA with nonzero dVdp
        uint      numVolVertsB;            // Number of vertices of meshB with nonzero dVdp
        uint      numDynamicVertsA;        // Number of vertices of meshA that are dynamic, which will determine size of constraint Jacobian row
        uint      numDynamicVertsB;        // Number of vertices of meshB that are dynamic, which will determine size of constraint Jacobian row
        FmVector3 normal;                  // Normal derived from average dVdp of each mesh
        FmVector3 tangent1;                // Tangent derived from normal and weighted average of vert velocities
        float     V;                       // Signed volume of intersection (negative for intersections)
        float     normalProjRelVel;        // Relative velocity of contact points projected on normal (dot(normal, velB - velA)); positive value means approaching
        float     frictionCoeff;           // Currently same for static or dynamic case
        uint16_t  flags;                   // Bitwise or of FM_CONSTRAINT_FLAG_* values

        inline FmVolumeContact()
        {
            idInObjectPair = FM_INVALID_ID;
            objectIdA = FM_INVALID_ID;
            objectIdB = FM_INVALID_ID;
            volVertsStartA = 0;
            volVertsStartB = 0;
            numVolVertsA = 0;
            numVolVertsB = 0;
            numDynamicVertsA = 0;
            numDynamicVertsB = 0;
            normal = FmInitVector3(0.0f);
            tangent1 = FmInitVector3(0.0f);
            V = 0.0f;
            frictionCoeff = FM_DEFAULT_FRICTION_COEFF;
            normalProjRelVel = 0.0f;
            flags = 0;
        }
    };

    // Constraint to prevent excessive tet deformation.
    // Forms constraint on singular values of deformation gradient, which limits stretch or compression.
    // If tet positions are pos0, pos1, pos2, pos3
    // Volume matrix is defined as
    // X = ( pos0 - pos3, pos1 - pos3, pos2 - pos3 )
    // Rest state volume matrix is X0
    // Deformation gradient is:
    // G = X * inverse(X0)
    // SVD(G) is computed as U * S * V^T
    // Reference: Perez et al., "Strain Limiting for Soft Finger Contact Simulation"
    struct FmDeformationConstraint
    {
        uint         idInObject;        // Id within the object, to help with sorting
        uint         objectId;
        uint         tetId;
        FmVector3    deformationDeltas;    // Correction to deformation value
        FmMatrix3    jacobian0;            // Derivative of deformation value wrt pos0 = inverse(X0).row0 * V.col_i * U.col_i^T  (where i is singular value being constrained)
        FmMatrix3    jacobian1;            // Derivative of deformation value wrt pos1 = inverse(X0).row1 * V.col_i * U.col_i^T
        FmMatrix3    jacobian2;            // Derivative of deformation value wrt pos2 = inverse(X0).row2 * V.col_i * U.col_i^T
        FmMatrix3    jacobian3;            // Derivative of deformation value wrt pos3 = -(inverse(X0).row0 + inverse(X0).row1 + inverse(X0).row2) * V.coli * U.coli^T
        float        kVelCorrection;       // Fraction to correct in constraint solve (velocity solve)
        float        kPosCorrection;       // Fraction to correct in stabilization (position solve)
        uint16_t     flags;
        uint8_t      dynamicFlags;         // Bits set if tet vertices are dynamic and barycentric != 0
        uint8_t      movingFlags;          // Bits set if tet vertices are moving and barycentric != 0

        inline FmDeformationConstraint()
        {
            idInObject = FM_INVALID_ID;
            objectId = FM_INVALID_ID;
            tetId = FM_INVALID_ID;
            deformationDeltas = FmInitVector3(0.0f);
            kVelCorrection = 0.0f;  // Just restrict further deformation
            kPosCorrection = 0.05f; // Slight correction for stability
            dynamicFlags = 0;
            movingFlags = 0;
            flags = 0;
        }
    };

    // Constraint used to glue object points together.
    struct FmGlueConstraint
    {
        uint         bufferIdA;            // Tet mesh buffer id, or rigid body if FM_RB_FLAG set
        uint         bufferIdB;            // Tet mesh buffer id, or rigid body if FM_RB_FLAG set, or world position if FM_INVALID_ID
        uint         bufferTetIdA;         // If object A is tet mesh, contains id of tet in mesh buffer
        uint         bufferTetIdB;         // If object B is tet mesh, contains id of tet in mesh buffer

                                           // Object id, tet id, tet vert ids updated automatically, since they may change with fracture
        uint         objectIdA;            // Tet mesh id, or rigid body if FM_RB_FLAG set
        uint         objectIdB;            // Tet mesh id, or rigid body if FM_RB_FLAG set
        uint         tetIdA;               // If object A is tet mesh, contains id of tet with contact point
        uint         tetIdB;               // If object B is tet mesh, contains id of tet with contact point
        union
        {
            float    posBaryA[4];          // Barycentric coordinate of contact point on tet mesh A
            float    posBodySpaceA[4];     // Vector from center of mass to contact point on rigid body A, in body space
        };
        union
        {
            float    posBaryB[4];          // Barycentric coordinate of contact point on tet mesh B
            float    posBodySpaceB[4];     // Vector from center of mass to contact point on rigid body B, in body space
            float    posWorldB[4];
        };
        FmVector3    comToPosA;            // Vector from center of mass to contact point on rigid body A
        FmVector3    comToPosB;            // Vector from center of mass to contact point on rigid body B
        FmVector3    deltaPos;             // Vector from posB to posA
        float        lambdaX;              // Cached multipliers
        float        lambdaY;
        float        lambdaZ;
        float        breakThreshold;       // If nonzero, is threshold in magnitude of impulse at which glue is automatically broken (disabled)
        float        kVelCorrection;       // Fraction to correct in constraint solve (velocity solve)
        float        kPosCorrection;       // Fraction to correct in stabilization (position solve)
        uint16_t     flags;                // Bitwise or of FM_CONSTRAINT_FLAG_* values
        uint8_t      dynamicFlags;         // Bits set if tet vertices are dynamic and barycentric != 0
        uint8_t      movingFlags;          // Bits set if tet vertices are moving and barycentric != 0
        uint8_t      minGlueConstraints;   // Minimum number of glue constraints required between the glued objects.  This constraint will be broken/disabled if < this value.

        inline FmGlueConstraint()
        {
            bufferIdA = FM_INVALID_ID;
            bufferIdB = FM_INVALID_ID;
            bufferTetIdA = FM_INVALID_ID;
            bufferTetIdB = FM_INVALID_ID;
            objectIdA = FM_INVALID_ID;
            objectIdB = FM_INVALID_ID;
            tetIdA = FM_INVALID_ID;
            tetIdB = FM_INVALID_ID;
            posBaryA[0] = 0.0f;
            posBaryA[1] = 0.0f;
            posBaryA[2] = 0.0f;
            posBaryA[3] = 0.0f;
            posBaryB[0] = 0.0f;
            posBaryB[1] = 0.0f;
            posBaryB[2] = 0.0f;
            posBaryB[3] = 0.0f;
            comToPosA = FmInitVector3(0.0f);
            comToPosB = FmInitVector3(0.0f);
            deltaPos = FmInitVector3(0.0f);
            lambdaX = 0.0f;
            lambdaY = 0.0f;
            lambdaZ = 0.0f;
            breakThreshold = 0.0f;
            kVelCorrection = 0.1f;
            kPosCorrection = 1.0f;
            flags = 0;
            dynamicFlags = 0;
            movingFlags = 0;
            minGlueConstraints = 1;  // Default allows this glue constraint to be the only one attaching two objects
        }
    };

    // Constrains points on or above 1 to 3 planes.
    // Can limit to fewer planes with FM_CONSTRAINT_FLAG_1D or FM_CONSTRAINT_FLAG_2D and
    // specify non-negative inequality with FM_CONSTRAINT_FLAG_NONNEG flags.
    // Can use to create a line, pin, or distance constraint (by updating normal).
    // This can be used for glue but is slower.
    struct FmPlaneConstraint
    {
        uint         bufferIdA;            // Tet mesh buffer id, or rigid body if FM_RB_FLAG set
        uint         bufferIdB;            // Tet mesh buffer id, or rigid body if FM_RB_FLAG set, or world position if FM_INVALID_ID
        uint         bufferTetIdA;         // If object A is tet mesh, contains id of tet in mesh buffer
        uint         bufferTetIdB;         // If object B is tet mesh, contains id of tet in mesh buffer

                                           // Object id, tet id, tet vert ids updated automatically, since they may change with fracture
        uint         objectIdA;            // Tet mesh id, or rigid body if FM_RB_FLAG set
        uint         objectIdB;            // Tet mesh id, or rigid body if FM_RB_FLAG set
        uint         tetIdA;               // If object A is tet mesh, contains id of tet with contact point
        uint         tetIdB;               // If object B is tet mesh, contains id of tet with contact point
        union
        {
            float    posBaryA[4];          // Barycentric coordinate of contact point on tet mesh A
            float    posBodySpaceA[4];     // Vector from center of mass to contact point on rigid body A, in body space
        };
        union
        {
            float    posBaryB[4];          // Barycentric coordinate of contact point on tet mesh B
            float    posBodySpaceB[4];     // Vector from center of mass to contact point on rigid body B, in body space
            float    posWorldB[4];
        };
        FmVector3    comToPosA;            // Vector from center of mass to contact point on rigid body A
        FmVector3    comToPosB;            // Vector from center of mass to contact point on rigid body B
        FmVector3    planeNormal0;         // Plane 0 normal in world space
        FmVector3    planeNormal1;         // Plane 1 normal in world space
        FmVector3    planeNormal2;         // Plane 2 normal in world space
        float        projection0;          // Computed as dot(planeNormal, posA - posB)
        float        projection1;
        float        projection2;
        float        bias0;
        float        bias1;
        float        bias2;
        float        lambda0;
        float        lambda1;
        float        lambda2;
        float        kVelCorrection;       // Fraction to correct in constraint solve (velocity solve)
        float        kPosCorrection;       // Fraction to correct in stabilization (position solve)
        uint16_t     flags;                // Bitwise or of FM_CONSTRAINT_FLAG_* values
        uint8_t      dynamicFlags;         // Bits set if tet vertices are dynamic and barycentric != 0
        uint8_t      movingFlags;          // Bits set if tet vertices are moving and barycentric != 0

        inline FmPlaneConstraint()
        {
            bufferIdA = FM_INVALID_ID;
            bufferIdB = FM_INVALID_ID;
            bufferTetIdA = FM_INVALID_ID;
            bufferTetIdB = FM_INVALID_ID;
            objectIdA = FM_INVALID_ID;
            objectIdB = FM_INVALID_ID;
            tetIdA = FM_INVALID_ID;
            tetIdB = FM_INVALID_ID;
            posBaryA[0] = 0.0f;
            posBaryA[1] = 0.0f;
            posBaryA[2] = 0.0f;
            posBaryA[3] = 0.0f;
            posBaryB[0] = 0.0f;
            posBaryB[1] = 0.0f;
            posBaryB[2] = 0.0f;
            posBaryB[3] = 0.0f;
            comToPosA = FmInitVector3(0.0f);
            comToPosB = FmInitVector3(0.0f);
            planeNormal0 = FmInitVector3(1.0f, 0.0f, 0.0f);
            planeNormal1 = FmInitVector3(0.0f, 1.0f, 0.0f);
            planeNormal2 = FmInitVector3(0.0f, 0.0f, 1.0f);
            projection0 = 0.0f;
            projection1 = 0.0f;
            projection2 = 0.0f;
            bias0 = 0.0f;
            bias1 = 0.0f;
            bias2 = 0.0f;
            lambda0 = 0.0f;
            lambda1 = 0.0f;
            lambda2 = 0.0f;
            kVelCorrection = 0.1f;
            kPosCorrection = 1.0f;
            flags = 0;
            dynamicFlags = 0;
            movingFlags = 0;
        }
    };

    // Angular constraint on rigid bodies which can be used to implement joints.
    struct FmRigidBodyAngleConstraint
    {
        uint         objectIdA;            // Rigid body id with FM_RB_FLAG set
        uint         objectIdB;            // Rigid body id with FM_RB_FLAG set, or world position if FM_INVALID_ID
        FmVector3    axisBodySpaceA;       // Body-relative axis defining angle constraint
        FmVector3    axisBodySpaceB;       // Body-relative axis defining angle constraint
        FmMatrix3    jacobianA;
        FmMatrix3    jacobianB;
        float        error0;
        float        error1;
        float        error2;
        float        lambda0;
        float        lambda1;
        float        lambda2;
        float        frictionCoeff;
        float        kVelCorrection;       // Fraction to correct in constraint solve (velocity solve)
        float        kPosCorrection;       // Fraction to correct in stabilization (position solve)
        uint16_t     flags;                // Bitwise or of FM_CONSTRAINT_FLAG_* values
        uint8_t      dynamicFlags;         // Bits set if tet vertices are dynamic and barycentric != 0
        uint8_t      movingFlags;          // Bits set if tet vertices are moving and barycentric != 0
        uint8_t      type;                 // One of FmRigidBodyAngleConstraintTypes

        inline FmRigidBodyAngleConstraint()
        {
            objectIdA = FM_INVALID_ID;
            objectIdB = FM_INVALID_ID;
            axisBodySpaceA = FmInitVector3(0.0f);
            axisBodySpaceB = FmInitVector3(0.0f);
            jacobianA = FmInitMatrix3(0.0f);
            jacobianB = FmInitMatrix3(0.0f);
            error0 = 0.0f;
            error1 = 0.0f;
            error2 = 0.0f;
            lambda0 = 0.0f;
            lambda1 = 0.0f;
            lambda2 = 0.0f;
            frictionCoeff = FM_DEFAULT_FRICTION_COEFF;
            kVelCorrection = 0.1f;
            kPosCorrection = 1.0f;
            flags = 0;
            dynamicFlags = 0;
            movingFlags = 0;
            type = FM_RB_JOINT_TYPE_HINGE;
        }
    };

    // Set fixed flags in constraint
    static FM_FORCE_INLINE void FmSetConstraintFlags(
        uint16_t* flags,
        uint8_t* dynamicFlags,
        uint8_t* movingFlags,
        uint dynamicFlagsA, uint dynamicFlagsB,
        uint movingFlagsA, uint movingFlagsB)
    {
        *dynamicFlags = (uint8_t)(dynamicFlagsA | (dynamicFlagsB << 4));
        *movingFlags = (uint8_t)(movingFlagsA | (movingFlagsB << 4));

        uint16_t resultFlags = *flags;
        if (dynamicFlagsA == 0)
        {
            resultFlags |= FM_CONSTRAINT_FLAG_OBJECTA_FIXED;
        }
        else
        {
            resultFlags &= ~FM_CONSTRAINT_FLAG_OBJECTA_FIXED;
        }

        if (dynamicFlagsB == 0)
        {
            resultFlags |= FM_CONSTRAINT_FLAG_OBJECTB_FIXED;
        }
        else
        {
            resultFlags &= ~FM_CONSTRAINT_FLAG_OBJECTB_FIXED;
        }

        if (movingFlagsA == 0)
        {
            resultFlags |= FM_CONSTRAINT_FLAG_OBJECTA_ZEROVEL;
        }
        else
        {
            resultFlags &= ~FM_CONSTRAINT_FLAG_OBJECTA_ZEROVEL;
        }

        if (movingFlagsB == 0)
        {
            resultFlags |= FM_CONSTRAINT_FLAG_OBJECTB_ZEROVEL;
        }
        else
        {
            resultFlags &= ~FM_CONSTRAINT_FLAG_OBJECTB_ZEROVEL;
        }

        *flags = resultFlags;
    }

    static FM_FORCE_INLINE uint FmGetNumJacobianSubmats(const FmDistanceContactPairInfo& contact)
    {
        uint numSubmats = 0;
        numSubmats += (uint)FM_IS_SET(contact.dynamicFlags, 0x1);
        numSubmats += (uint)FM_IS_SET(contact.dynamicFlags, 0x2);
        numSubmats += (uint)FM_IS_SET(contact.dynamicFlags, 0x4);
        numSubmats += (uint)FM_IS_SET(contact.dynamicFlags, 0x8);

        numSubmats += (uint)FM_IS_SET(contact.dynamicFlags, 0x10);
        numSubmats += (uint)FM_IS_SET(contact.dynamicFlags, 0x20);
        numSubmats += (uint)FM_IS_SET(contact.dynamicFlags, 0x40);
        numSubmats += (uint)FM_IS_SET(contact.dynamicFlags, 0x80);
        return numSubmats;
    }

    static FM_FORCE_INLINE uint FmGetNumJacobianSubmats(const FmVolumeContact& contact)
    {
        uint numSubmatsA = (contact.objectIdA & FM_RB_FLAG) ? contact.numDynamicVertsA * 2 : contact.numDynamicVertsA;
        uint numSubmatsB = (contact.objectIdB & FM_RB_FLAG) ? contact.numDynamicVertsB * 2 : contact.numDynamicVertsB;
        return numSubmatsA + numSubmatsB;
    }

    static FM_FORCE_INLINE uint FmGetNumJacobianSubmats(const FmDeformationConstraint& deformationConstraint)
    {
        uint numSubmats = 0;
        numSubmats += (uint)FM_IS_SET(deformationConstraint.dynamicFlags, 0x1);
        numSubmats += (uint)FM_IS_SET(deformationConstraint.dynamicFlags, 0x2);
        numSubmats += (uint)FM_IS_SET(deformationConstraint.dynamicFlags, 0x4);
        numSubmats += (uint)FM_IS_SET(deformationConstraint.dynamicFlags, 0x8);
        return numSubmats;
    }

    static FM_FORCE_INLINE uint FmGetNumJacobianSubmats(const FmGlueConstraint& glueConstraint)
    {
        uint numSubmats = 0;
        numSubmats += (uint)FM_IS_SET(glueConstraint.dynamicFlags, 0x1);
        numSubmats += (uint)FM_IS_SET(glueConstraint.dynamicFlags, 0x2);
        numSubmats += (uint)FM_IS_SET(glueConstraint.dynamicFlags, 0x4);
        numSubmats += (uint)FM_IS_SET(glueConstraint.dynamicFlags, 0x8);

        numSubmats += (uint)FM_IS_SET(glueConstraint.dynamicFlags, 0x10);
        numSubmats += (uint)FM_IS_SET(glueConstraint.dynamicFlags, 0x20);
        numSubmats += (uint)FM_IS_SET(glueConstraint.dynamicFlags, 0x40);
        numSubmats += (uint)FM_IS_SET(glueConstraint.dynamicFlags, 0x80);
        return numSubmats;
    }

    static FM_FORCE_INLINE uint FmGetNumJacobianSubmats(const FmPlaneConstraint& planeConstraint)
    {
        uint numSubmats = 0;
        numSubmats += (uint)FM_IS_SET(planeConstraint.dynamicFlags, 0x1);
        numSubmats += (uint)FM_IS_SET(planeConstraint.dynamicFlags, 0x2);
        numSubmats += (uint)FM_IS_SET(planeConstraint.dynamicFlags, 0x4);
        numSubmats += (uint)FM_IS_SET(planeConstraint.dynamicFlags, 0x8);

        numSubmats += (uint)FM_IS_SET(planeConstraint.dynamicFlags, 0x10);
        numSubmats += (uint)FM_IS_SET(planeConstraint.dynamicFlags, 0x20);
        numSubmats += (uint)FM_IS_SET(planeConstraint.dynamicFlags, 0x40);
        numSubmats += (uint)FM_IS_SET(planeConstraint.dynamicFlags, 0x80);
        return numSubmats;
    }

    static FM_FORCE_INLINE uint FmGetNumJacobianSubmats(const FmRigidBodyAngleConstraint& rigidBodyAngleConstraint)
    {
        return
            (uint)FM_NOT_SET(rigidBodyAngleConstraint.flags, FM_CONSTRAINT_FLAG_OBJECTA_FIXED) +
            (uint)FM_NOT_SET(rigidBodyAngleConstraint.flags, FM_CONSTRAINT_FLAG_OBJECTB_FIXED);
    }

    // Get direction of relative velocity on contact plane
    static FM_FORCE_INLINE FmVector3 FmGetContactTangent(const FmVector3& normal, const FmVector3& relVel)
    {
#if FM_PROJECT_FRICTION_TO_CIRCLE
        (void)relVel;

        // Tangent direction may be less important with this type of projection, so using an arbitrary orthogonal direction
        return normalize(FmOrthogonalVector(normal));
#else
        FmVector3 tangentVel = relVel - normal * dot(normal, relVel);
        //tangentVel = cross(normal, cross(tangentVel, normal));

        // If too small choose arbitrary vector orthogonal to normal
        float lenSqr;
        const float tol = 1.0e-30f;
        lenSqr = dot(tangentVel, tangentVel);

        return (lenSqr > tol) ? tangentVel * (1.0f / sqrtf(lenSqr)) : normalize(FmOrthogonalVector(normal));
#endif
    }

    // Get direction of relative velocity on contact plane
    static FM_FORCE_INLINE FmVector3 FmGetContactTangent(const FmVector3& normal, const FmVector3& velA, const FmVector3& velB)
    {
        return FmGetContactTangent(normal, velB - velA);
    }

}