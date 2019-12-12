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
// FEMFX library public API and overview
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXCommon.h"
#include "FEMFXTypes.h"
#include "FEMFXArray.h"
#include "FEMFXTaskSystemInterface.h"
#include "FEMFXTetMaterialParams.h"
#include "FEMFXTetMeshConnectivity.h"
#include "FEMFXRigidBodyState.h"
#include "FEMFXSerialize.h"


// Introduction:
// FEMFX is a mutithreaded-CPU library for real-time Finite Element Method (FEM) physics, 
// simulating deformation and fracture of solids.  Objects are represented as a mesh of 
// tetrahedra, and each has parameters from a material physics model to control behavior like 
// flexibility or resistance to fracture.  This supports a range of material types from elastic 
// soft-bodies to stiffer flexible materials such as wood or metal.
//
// The system is still experimental and not a complete solution.  However, we hope that the 
// release provides useful information and sample code for developers interested in physics, 
// collision detection, and multithreading.  The implementation has extensive multithreading
// to utilize multicore CPUs and benefit from the trend of increasing CPU core counts.
//
// Components:
// - Co-rotational FEM model
// - Multiplicative plastic deformation model
// - Implicit integration
// - A constraint solver
// - Continuous collision detection (CCD) (Scalar and SIMD implementations)
// - Triangle mesh intersection (Scalar and SIMD implementations)
// - 3x3 SVD (Scalar and SIMD implementations)
// - Volume-based contacts
// - Deformation constraints
// - Glue, plane, and rigid body angle constraints
// - Connected components for finding constraint islands, and sub-meshes after fracture
// - Analysis of constraint islands for parallelism: partitioning, graph coloring, and building 
//   a task dependency graph
// - AABB-trees adapted from Radeon Rays
// - A fixed-memory implementation of fracture (splitting between tetrahedral faces only)
// - Functions supporting asynchronous threading
// 
// Included in the library samples is a sample task system implementation.
// 
// See amd_femfx\inc\FEMFXTaskSystemInterface.h for a callback API to integrate a user task system.
// See amd_femfx\inc\Vectormath\simd_utils.h file for configuration of SIMD intrinsics use.
//
// References:
// - Allard et al., “Volume Contact Constraints at Arbitrary Resolution”, 2012
// - Ascher and Boxerman, “On the modified conjugate gradient method in cloth simulation”, 2003
// - Baraff and Witkin, “Large Steps in Cloth Simulation”, 1998
// - Bargteil et al., "A Finite Element Method for Animating Large Viscoplastic Flow", 2007
// - Cline and Pai, “Post - stabilization for Rigid Body Simulation with Contactand Constraints”, 2003
// - Curtis et al., “Fast Collision Detection for Deformable Models using Representative - Triangles”, 2008
// - Georgii and Westerman, “Corotated Finite Elements Made Fastand Stable”, 2008
// - Harada, “Two - Level Constraint Solver and Pipelined Local Batching for Rigid Body Simulation on GPUs”, 2008
// - Irving et al., “Invertible Finite Elements for Robust Simulation of Large Deformation”, 2004
// - Kushida, "Condition Number Estimation of Preconditioned Matrices", 2015
// - Lazarevych et al. “Decomposing the Contact Linear Complementarity Problem into Separate Contact Regions”, 2010
// - McAdams et al., “Computing the Singular Value Decomposition of 3x3 Matrices with Minimal Branching and Elementary Floating Point Operations”, 2011
// - Miguel and Otaduy, “Efficient Simulation of Contact Between Rigidand Deformable Objects”, 2011
// - Mirtich, "Impulse Based Dynamic Simulation of Rigid Body Systems", 1996
// - Müller and Gross, “Interactive Virtual Materials”, 2004
// - O’Brien et al., “Graphical modelingand animation of ductile fracture”, 2002
// - Otaduy et al., “Implicit Contact Handling for Deformable Objects”, 2009
// - Parker and O’Brien], “Real - Time Deformation and Fracture in a Game Environment”, 2009
// - Perez et al., "Strain Limiting for Soft Finger Contact Simulation", 2013
// - Radeon Rays, https://gpuopen.com/gaming-product/radeon-rays/
// - Smith and Dodgson, “A topologically robust algorithm for Boolean operations on polyhedral shapes using approximate arithmetic”, 2006
//
// Implementation notes:
// The implementation tends towards a more C-like style.  Some conventions used:
// Using an "Fm" prefix on classes and function names, and "FM_" prefix on macros.
// Placing a function's output parameters at the start of the arguments list.
// Passing pointers to output parameters, and value or const reference arguments otherwise.
// 
// The vector library API has been mostly limited to syntax allowed in shader languages, for
// greater portability.
// 
// The implementation has been designed to reduce dynamic allocations for more predictable memory 
// usage.  There is a fixed-size linear memory buffer used for each tet mesh object, and one for 
// the entire scene.  Fracture occurs within an object's fixed memory.  To support this the user
// has to provide information that can be used to bound required memory.  However, internal to the
// library there is dynamic allocation used in the task graph implemention and to allocate shared
// task data for asynchronous tasks.  Typically we use "struct" for types that are placed in 
// suballocated memory and are initialized afterward by setting members or use of an assignment
// operator.
// 
// Task-based parallelism is implemented at several stages:
// - Across meshes for the update of tetrahedral state, fracture, integration, and BVH rebuild
// - Across pairs of meshes for CCD and intersection traversals, contact generation
// - Across constraint islands solves
// - Across "independent partitions" computed within a constraint island
//
// The library is expected to give deterministic results regardless of number of threads used.
//
// API:
// To initialize the scene or tet mesh objects, the application has to first query required memory, 
// then allocate a 64-byte-aligned buffer and provide it to the library.  Here is an example for 
// the scene:
// 
//    FmSceneSetupParams sceneParams;
//    sceneParams.maxTetMeshBuffers = APP_MAX_MESH_BUFFERS;
//    sceneParams.maxTetMeshes = APP_MAX_MESHES;
//    sceneParams.maxRigidBodies = APP_MAX_RIGID_BODIES;
//    sceneParams.maxDistanceContacts = APP_MAX_DISTANCE_CONTACTS;
//    sceneParams.maxVolumeContacts = APP_MAX_VOLUME_CONTACTS;
//    sceneParams.maxVolumeContactVerts = APP_MAX_VOLUME_CONTACT_VERTS;
//    sceneParams.maxDeformationConstraints = APP_MAX_DEFORMATION_CONSTRAINTS;
//    sceneParams.maxGlueConstraints = APP_MAX_GLUE_CONSTRAINTS;
//    sceneParams.maxPlaneConstraints = APP_MAX_PLANE_CONSTRAINTS;
//    sceneParams.maxRigidBodyAngleConstraints = APP_MAX_RIGID_BODY_ANGLE_CONSTRAINTS;
//    sceneParams.maxBroadPhasePairs = APP_MAX_BROAD_PHASE_PAIRS;
//    sceneParams.maxRigidBodyBroadPhasePairs = APP_MAX_BROAD_PHASE_PAIRS;
//    sceneParams.maxSceneVerts = APP_MAX_SCENE_VERTS;
//    sceneParams.maxConstraintSolverDataSize = APP_MAX_CONSTRAINT_SOLVER_SIZE;
//    sceneParams.numWorkerThreads = APP_NUM_WORKER_THREADS;
//    sceneParams.rigidBodiesExternal = false;
// 
//    gScene = FmCreateScene(sceneParams);
// 
// A FmTetMeshBuffer is a single contiguous buffer that will hold the initial tet mesh geometry
// and any new tet meshes created by fracture.  The memory for this can be bounded by the tet mesh
// connectivity, and flags that enable/disable fracture at tet faces.
//
// The application needs to create an array of the tet ids incident to each vertex, e.g.:
// 
//    // Array of incident tets for each vertex
//    FmArray<uint>* vertIncidentTetIds = new FmArray<uint>[numVerts]; 
//    
//    // Fill in incident tet ids for each vertex
//    ...
//
//    // Create arrays for fracture group information
//    FmFractureGroupCounts* fractureGroupCounts = new FmFractureGroupCounts[numTets];
//    uint* tetFractureGroupIds = new uint[numTets];
//
//    // Get bounds for computing required memory, based on whether fracture enabled, and settings
//    // of tetFractureFlags, which must be set the same as when the mesh is later initialized.  
//    // This includes settings such as FM_TET_FLAG_FACE*_FRACTURE_DISABLED and 
//    // FM_TET_FLAG_KINEMATIC.
//    FmTetMeshBufferBounds bounds;
//    FmComputeTetMeshBufferBounds(
//        &bounds,
//        fractureGroupCounts,
//        tetFractureGroupIds,
//        vertIncidentTetIds, tetVertIds, tetFractureFlags,
//        numVerts, numTets, enableFracture);
//
//    // Set the parameters for tet mesh creation
//    FmTetMeshBufferSetupParams tetMeshBufferParams;
//    tetMeshBufferParams.numVerts = bounds.numVerts;
//    tetMeshBufferParams.numTets = bounds.numTets;
//    tetMeshBufferParams.numVertIncidentTets = bounds.numVertIncidentTets;
//    tetMeshBufferParams.maxVertAdjacentVerts = bounds.maxVertAdjacentVerts;
//    tetMeshBufferParams.maxVerts = bounds.maxVerts;
//    tetMeshBufferParams.maxExteriorFaces = bounds.maxExteriorFaces;
//    tetMeshBufferParams.maxTetMeshes = bounds.maxTetMeshes;
//    tetMeshBufferParams.collisionGroup = 0;
//    tetMeshBufferParams.enablePlasticity = enablePlasticity;
//    tetMeshBufferParams.enableFracture = enableFracture;
//    tetMeshBufferParams.isKinematic = isKinematic;
//
//    // Create the buffer
//    FmTetMesh* tetMeshPtr;
//    FmTetMeshBuffer* tetMeshBuffer = FmCreateTetMeshBuffer(tetMeshBufferParams, 
//        fractureGroupCounts, tetFractureGroupIds, &tetMeshPtr);
//    FmTetMesh& tetMesh = *tetMeshPtr;
//
// The FmTetMesh* is returned for the user to initialize the geometry and simulation parameters.
//
//    FmInitVertState(&tetMesh, vertRestPositionsArray, rotation, translation, 
//        vertMass,  // typically replaced in later calls
//        vertInitialVelocity);
// 
//    FmInitTetState((&tetMesh, tetVertIdsArray, defaultMaterialParams);
// 
// After tet vertex ids, material properties and vertex rest positions are set, this call will
// initialize various matrices associated with the FEM model.  It also returns total rest volume.
//
//    float totalRestVolume = FmComputeMeshConstantMatrices(&tetMesh);
//
//    FmFinishConnectivityFromVertIncidentTets(&tetMesh);
//
//
// After vertex connectivity has been initialized, one can initialize masses.
// The application can use vertex masses to initialize tet masses:
//    
//    FmSetVertMass(&tetMesh, 0, mass);  // if changing the default mass value in FmInitVertState
//    ...
//
//    FmDistributeVertMassesToTets(&tetMesh);
//
// Alternatively, one can use an API call that computes tet mass from rest density and distributes
// to vertices.  If the restDensity argument is nonzero, it is applied to all tetrahedra, otherwise
// rest density from tet material parameters is used.
// 
//    FmSetMassesFromRestDensities(&tetMesh, restDensity);
//
// Finally the application should call:
// 
//    FmFinishTetMeshInit(&tetMesh);
//
// and add the completed tet mesh buffer to the scene:
//
//    uint tetMeshBufferId = FmAddTetMeshBufferToScene(gScene, tetMeshBuffer);
//
// To run the simulation the simplest API call is:
//
//   void FmUpdateScene(FmScene* scene, float timestep);
//
// Gravity acceleration can be controlled by setting FmSceneControlParams::gravityVector 
//
// API is also described below that breaks the scene update into phases and allows mixing with
// rigid bodies.

// Bit flags used with FEM objects
// NOTE: other bit positions reserved for implementation

#define FM_VERT_FLAG_KINEMATIC                            0x1          // Kinematically driven vertex
#define FM_VERT_FLAG_KINEMATIC_REMOVABLE                  0x2          // Let fracture remove kinematic flags based on stress or when exterior face is constrained only by one or two vertices
#define FM_VERT_FLAG_FRACTURED                            0x4          // Marks a vertex where a fracture occurred, reset on scene update
#define FM_VERT_FLAG_FRACTURE_COPY                        0x8          // Marks a vertex that was created by fracture

#define FM_TET_FLAG_FACE0_FRACTURE_DISABLED               0x1          // Fracture disabled for face 0 of tetrahedron
#define FM_TET_FLAG_FACE1_FRACTURE_DISABLED               0x2          // Fracture disabled for face 1 of tetrahedron
#define FM_TET_FLAG_FACE2_FRACTURE_DISABLED               0x4          // Fracture disabled for face 2 of tetrahedron
#define FM_TET_FLAG_FACE3_FRACTURE_DISABLED               0x8          // Fracture disabled for face 3 of tetrahedron
#define FM_TET_FLAG_KINEMATIC                             0x10         // Marks tet as kinematic; fracture will leave connected to original vertices and not remove kinematic flags from vertices, as long as these are in face-connected groupings
#define FM_TET_FLAG_PLASTICITY_DISABLED                   0x20         // Disable plasticity for tetrahedron material
#define FM_TET_FLAG_VOLUME_PRESERVING_PLASTICITY          0x40         // Constrain plastic deformation to preserve volume (note can cause an irregular surface)

#define FM_WARNING_FLAG_HIT_LIMIT_SCENE_TET_MESHES                    0x1     // Hit limit determined by FmSceneSetupParams::maxTetMeshes
#define FM_WARNING_FLAG_HIT_LIMIT_SCENE_BROAD_PHASE_PAIRS             0x2     // Hit limit determined by FmSceneSetupParams::maxBroadPhasePairs
#define FM_WARNING_FLAG_HIT_LIMIT_SCENE_RIGID_BODY_BROAD_PHASE_PAIRS  0x4     // Hit limit determined by FmSceneSetupParams::maxRigidBodyBroadPhasePairs
#define FM_WARNING_FLAG_HIT_LIMIT_SCENE_DISTANCE_CONTACTS             0x8     // Hit limit determined by FmSceneSetupParams::maxDistanceContacts
#define FM_WARNING_FLAG_HIT_LIMIT_SCENE_VOLUME_CONTACTS               0x10    // Hit limit determined by FmSceneSetupParams::maxVolumeContacts
#define FM_WARNING_FLAG_HIT_LIMIT_SCENE_VOLUME_CONTACT_VERTS          0x20    // Hit limit determined by FmSceneSetupParams::maxVolumeContactVerts
#define FM_WARNING_FLAG_HIT_LIMIT_SCENE_DEFORMATION_CONSTRAINTS       0x40    // Hit limit determined by FmSceneSetupParams::maxDeformationConstraints
#define FM_WARNING_FLAG_HIT_LIMIT_SCENE_CONSTRAINT_SOLVER_MEMORY      0x80    // Hit limit determined by combination of max objects, contacts, and FmSceneSetupParams::maxVerts,maxJacobianSubmats
#define FM_WARNING_FLAG_HIT_LIMIT_TET_MESH_BUFFER_MESHES              0x100   // Hit limit determined by FmTetMeshBufferSetupParams::maxTetMeshes
#define FM_WARNING_FLAG_HIT_LIMIT_TET_MESH_VERTS                      0x200   // Hit limit determined by FmTetMeshBufferSetupParams::maxVerts
#define FM_WARNING_FLAG_HIT_LIMIT_TET_MESH_EXTERIOR_FACES             0x400   // Hit limit determined by FmTetMeshBufferSetupParams::maxExteriorFaces

namespace AMD
{
    struct FmScene;
    struct FmTetMeshBuffer;
    struct FmTetMesh;
    struct FmRigidBody;
    struct FmUserConstraints;
    struct FmBvh;

    // Parameters needed to allocate memory for tet mesh
    struct FmTetMeshBufferBounds
    {
        uint numVerts;                // Initial total number of verts, may grow with fracture
        uint numTets;                 // Total number of tets, unchanged by fracture
        uint numVertIncidentTets;     // Total incident tets of all verts; bounds the memory needed for topology
        uint maxVertAdjacentVerts;    // Total adjacent verts of all verts; bounds the memory needed for topology
        uint maxVerts;                // Limit on number of verts including original mesh verts and those created by fracture
        uint maxExteriorFaces;        // Limit on number of exterior faces
        uint maxTetMeshes;            // Limit on number of meshes including original and pieces created by fracture
    };

    struct FmTetMeshBufferSetupParams
    {
        uint numVerts;                // Initial total number of verts, may grow with fracture
        uint numTets;                 // Total number of tets, unchanged by fracture
        uint numVertIncidentTets;     // Total incident tets of all verts; bounds the memory needed for topology
        uint maxVertAdjacentVerts;    // Total adjacent verts of all verts; bounds the memory needed for topology
        uint maxVerts;                // Limit on number of verts including original mesh verts and those created by fracture
        uint maxExteriorFaces;        // Limit on number of exterior faces
        uint maxTetMeshes;            // Limit on number of meshes including original and pieces created by fracture
        uint collisionGroup;          // Initial collision group for all meshes created from this (must be < 32)
        bool enablePlasticity;        // Whether mesh has plasticity
        bool enableFracture;          // Whether mesh can be fractured
        bool isKinematic;             // Whether mesh is only kinematic

        inline FmTetMeshBufferSetupParams() : numVerts(0), numTets(0), numVertIncidentTets(0), maxVertAdjacentVerts(0), maxVerts(0), 
            maxExteriorFaces(0), maxTetMeshes(0), collisionGroup(0), enablePlasticity(false), enableFracture(false), isKinematic(false) {}
    };

    // Collision-enabled status for all pairs of collision groups
    struct FmCollisionGroupPairFlags
    {
        uint flags[32];    // flags[i] non-zero bits give collision-enabled status between group i and other groups.
    };

    struct FmCollisionReportDistanceContact
    {
        uint      objectIdA;            // Tet mesh id, or rigid body id if FM_RB_FLAG set
        uint      objectIdB;            // Tet mesh id, or rigid body id if FM_RB_FLAG set, or fixed point if FM_INVALID_ID

        uint      tetIdA;               // Tet id, if object is a tet mesh
        uint      tetIdB;               // Tet id, if object is a tet mesh

        union
        {
            float posBaryA[4];          // Barycentric coordinate of contact point on tet mesh A
            float comToPosA[4];         // Vector from center of mass to contact point on rigid body A
        };        
        union     
        {         
            float posBaryB[4];          // Barycentric coordinate of contact point on tet mesh B
            float comToPosB[4];         // Vector from center of mass to contact point on rigid body B
        };

        FmVector3 normal;               // Normal of contact plane pointing out from body B

        FmCollisionReportDistanceContact() : objectIdA(FM_INVALID_ID), objectIdB(FM_INVALID_ID), tetIdA(FM_INVALID_ID), tetIdB(FM_INVALID_ID),
            normal(FmInitVector3(0.0f))
        {
            posBaryA[0] = 0.0f;
            posBaryA[1] = 0.0f;
            posBaryA[2] = 0.0f;
            posBaryA[3] = 0.0f;
            posBaryB[0] = 0.0f;
            posBaryB[1] = 0.0f;
            posBaryB[2] = 0.0f;
            posBaryB[3] = 0.0f;
        }
    };

    struct FmCollisionReportVolumeContact
    {
        uint      objectIdA;            // Tet mesh id, or rigid body id if FM_RB_FLAG set
        uint      objectIdB;            // Tet mesh id, or rigid body id if FM_RB_FLAG set
                  
        float     volume;               // Overlap volume

        FmVector3 normal;               // Normal of contact plane pointing out from body B

        FmCollisionReportVolumeContact() : objectIdA(FM_INVALID_ID), objectIdB(FM_INVALID_ID), volume(0.0f), normal(FmInitVector3(0.0f)) {}
    };

    // For reporting contacts to the application.
    // To use must set buffers for the contacts, size of buffers, and velocity threshold.
    struct FmCollisionReport
    {
        FmAtomicUint numDistanceContacts;
        FmAtomicUint numVolumeContacts;

        uint  maxDistanceContactsPerObjectPair;
        uint  maxVolumeContactsPerObjectPair;         // Currently there is at most one volume contact per object pair
        float minContactRelVel;                       // Minimum relative speed of approach at contact (along normal) 

        FmCollisionReportDistanceContact* distanceContactBuffer;  // Set to an application array with space for maxDistanceContacts
        FmCollisionReportVolumeContact*   volumeContactBuffer;    // Set to an application array with space for maxVolumeContacts

        uint maxDistanceContacts;
        uint maxVolumeContacts;

        FmCollisionReport() :
            numDistanceContacts(0), numVolumeContacts(0), maxDistanceContactsPerObjectPair(0), maxVolumeContactsPerObjectPair(0),
            minContactRelVel(FLT_MAX), distanceContactBuffer(NULL), volumeContactBuffer(NULL), maxDistanceContacts(0), maxVolumeContacts(0) {}
    };

    struct FmTetMeshFractureReport
    {
        uint objectId;
        uint numFractureFaces;
        uint numTotalFaces;

        FmTetMeshFractureReport() : objectId(FM_INVALID_ID), numFractureFaces(0), numTotalFaces(0) {}
    };

    // For reporting fracture to the application.
    // To use must set a buffer for the tet mesh reports and size of buffer.
    struct FmFractureReport
    {
        FmAtomicUint numTetMeshReports;

        uint                     maxTetMeshReports;
        FmTetMeshFractureReport* tetMeshReportsBuffer;     // Set to an application array with space for maxTetMeshReports

        FmFractureReport()
        {
            maxTetMeshReports = 0;
            tetMeshReportsBuffer = NULL;
            numTetMeshReports = 0;
        }
    };

    // Warnings recorded from within scene update
    struct FmWarningsReport
    {
        FmAtomicUint tetMeshId;    // Saves id of one tet mesh with a warning; bufferId will indicate the tet mesh buffer
        FmAtomicUint flags;        // OR of FM_WARNING_* flags; cleared by user

        FmWarningsReport() : tetMeshId(FM_INVALID_ID), flags(0) {}
    };

    // Parameters used to initialize a rigid body
    struct FmRigidBodySetupParams
    {
        FmRigidBodyState state;
        FmMatrix3        bodyInertiaTensor;
        float            mass;
        float            halfDimX;
        float            halfDimY;
        float            halfDimZ;
        bool             isKinematic;
        uint8_t          collisionGroup;

        inline FmRigidBodySetupParams() : bodyInertiaTensor(FmMatrix3::identity()), mass(1.0f),
            halfDimX(0.5f), halfDimY(0.5f), halfDimZ(0.5f), isKinematic(false), collisionGroup(0) {}
    };

    // Allocate and initialize rigid body
    FmRigidBody* FmCreateRigidBody(const FmRigidBodySetupParams& setupParams);

    // Free rigid body memory
    void FmDestroyRigidBody(FmRigidBody* rigidBody);

    // Positions of axis-aligned planes forming box around the scene
    struct FmSceneCollisionPlanes
    {
        float minX;
        float maxX;
        float minY;
        float maxY;
        float minZ;
        float maxZ;
    };

    // Solver pass indices
#define FM_CG_PASS_IDX 0
#define FM_GS_PASS_IDX 1

    struct FmConstraintSolverControlParams
    {
        // Fraction of correction to apply to contacts in constraint (velocity) solve or stabilization
        float kDistanceCorrection;          // Applies to any interpenetration contacts (positive error); for pre-emptive CCD contacts full correction used
        float kVolumeCorrection;

        // Parameters for one of the two passes
        struct PassParams
        {
            uint  maxOuterIterations;              // Max iterations of process alternating impulses and response
            uint  maxInnerIterations;              // Max iterations of PGS solving constraint impulses
            uint  currentMaxInnerIterations;       // Inner iterations that may be reduced as solve progresses
            bool  useInnerIterationsFalloff;       // Reduce max inner iterations as outer iterations progress; faster but less stable
            bool  useOuterAlternatingDirections;   // Alternate row direction each iteration (GS pass only).
            bool  useInnerAlternatingDirections;   // Alternate row direction each iteration of PGS.

            // Constant between 0 and 2; 1 for Gauss-Seidel, < 1 for under-relaxation, > 1 for over-relaxation.
            // Default is 0.8 which resolves instability or jitter in a few examples, especially with useInnerIterationsFalloff true.
            float kPgsRelaxationOmega;

            // Termination thresholds (not currently supported)
            float outerRelErrorEpsilon;
            float innerRelErrorEpsilon;

            PassParams() : maxOuterIterations(0), maxInnerIterations(0), currentMaxInnerIterations(0),
                useInnerIterationsFalloff(false), useOuterAlternatingDirections(false), useInnerAlternatingDirections(false),
                kPgsRelaxationOmega(0.0f), outerRelErrorEpsilon(0.0f), innerRelErrorEpsilon(0.0f) {}
        };

        PassParams passParams[2];          // Different parameters for CG or GS based passes

        uint  maxCgIterationsCgPass;       // Max iterations using CG to compute FEM response to constraint impulses
        float epsilonCgPass;               // Tolerance for CG; residual mag should be < eps * right-hand-side mag

        FmConstraintSolverControlParams() : kDistanceCorrection(0.0f), kVolumeCorrection(0.0f), maxCgIterationsCgPass(0), epsilonCgPass(FLT_EPSILON) {}
    };

    // Set of parameters that control all collision detection and solving done in the scene.
    struct FmSceneControlParams
    {
        FmVector3 gravityVector;  // Gravity acceleration applied as external force to all objects in scene.

        FmSceneCollisionPlanes collisionPlanes;  // Positions of axis-aligned planes forming box around the scene

        uint numThreads; // Number of worker threads processing step update, set at start of frame.

        float timestep; // Timestep used in integration
        float simTime;  // Elapsed time of simulation

        // Bias is the distance or volume value that is the goal for the contact constraint.
        // For CCD, will find time of impact at this distance.
        // Threshold is a larger separation value at which a contact is added.
        // Partly this is meant to help keep contacts more stable when things are 
        // resting and contacts are near the goal separation.
        // (The solver itself though may disable different contacts.)
        // Also in CCD, if features start out closer than the contact threshold, the
        // contact is returned without further iteration.
        // volContactBias and volContactThreshold should be <= 0.0; 0.0 is recommended.
        float distContactBias;
        float distContactThreshold;
        float volContactBias;
        float volContactThreshold;

        // Rayleigh damping terms applied to FEM dynamics integration solve:
        // Damping matrix C = kRayleighMassDamping * M + kRayleighStiffnessDamping * K' 
        float kRayleighMassDamping;
        float kRayleighStiffnessDamping;

        // Options for rigid body interfacing
        bool rigidBodiesExternal;                   // If rigid bodies external, collision detection and integration for rigid bodies are skipped (only box rigid bodies supported internally)
        bool includeRigidBodiesInBroadPhase;        // Whether to add rigid bodies to broad phase and produce broad phase pairs; if included and rigidBodiesExternal true, will output to rigidBodyBroadPhasePairs.
        bool createZeroConstraintRigidBodyIslands;  // Applies when rigidBodiesExternal true: whether to create islands for groups of rigid bodies without any (FEM-lib-visible) constraints.
        
        float epsilonCg;              // Tolerance for CG; residual mag should be < eps * right-hand-side mag
        uint defaultMaxCgIterations;  // Initial setting for use in FEM dynamics integration solve which can be overridden by material settings

        FmConstraintSolverControlParams constraintSolveParams;         // Parameters for constraint solve (velocity-based)
        FmConstraintSolverControlParams constraintStabilizationParams; // Parameters for constraint stabilization (position-based)

        FmSceneControlParams()
        {
            // Defaults for scene control parameters
            gravityVector = FmInitVector3(0.0f, -9.88f, 0.0f);
            collisionPlanes.minX = -FLT_MAX;
            collisionPlanes.maxX = FLT_MAX;
            collisionPlanes.minY = 0.0f;
            collisionPlanes.maxY = FLT_MAX;
            collisionPlanes.minZ = -FLT_MAX;
            collisionPlanes.maxZ = FLT_MAX;
            numThreads = 1;
            timestep = (1.0f / 60.0f);
            simTime = 0.0f;
            distContactBias = 0.002f;
            distContactThreshold = 0.003f;
            volContactBias = 0.0f;
            volContactThreshold = 0.0f;
            kRayleighMassDamping = 0.15f;
            kRayleighStiffnessDamping = 0.004f;
            rigidBodiesExternal = false;
            includeRigidBodiesInBroadPhase = true;
            createZeroConstraintRigidBodyIslands = true;
            epsilonCg = FLT_EPSILON;
            defaultMaxCgIterations = FM_DEFAULT_MAX_CG_ITERATIONS;

#if FM_CONSTRAINT_STABILIZATION_SOLVE
            constraintSolveParams.kDistanceCorrection = 0.0f;     // corrections occur during stabilization
            constraintSolveParams.kVolumeCorrection = 0.0f;       // corrections occur during stabilization
#else
            constraintSolveParams.kDistanceCorrection = 0.1f;
            constraintSolveParams.kVolumeCorrection = 0.05f; 
#endif

            constraintSolveParams.passParams[FM_CG_PASS_IDX].maxOuterIterations = 8;
            constraintSolveParams.passParams[FM_CG_PASS_IDX].maxInnerIterations = 1;
            constraintSolveParams.passParams[FM_CG_PASS_IDX].useInnerIterationsFalloff = false;
            constraintSolveParams.passParams[FM_CG_PASS_IDX].useOuterAlternatingDirections = false;
            constraintSolveParams.passParams[FM_CG_PASS_IDX].useInnerAlternatingDirections = false;
            constraintSolveParams.passParams[FM_CG_PASS_IDX].kPgsRelaxationOmega = 0.8f;
            constraintSolveParams.passParams[FM_CG_PASS_IDX].outerRelErrorEpsilon = 0.001f;
            constraintSolveParams.passParams[FM_CG_PASS_IDX].innerRelErrorEpsilon = 0.001f;
            constraintSolveParams.maxCgIterationsCgPass = 15;
            constraintSolveParams.epsilonCgPass = FLT_EPSILON;

            constraintSolveParams.passParams[FM_GS_PASS_IDX].maxOuterIterations = 14;
            constraintSolveParams.passParams[FM_GS_PASS_IDX].maxInnerIterations = 1;
            constraintSolveParams.passParams[FM_GS_PASS_IDX].useInnerIterationsFalloff = true;
            constraintSolveParams.passParams[FM_GS_PASS_IDX].useOuterAlternatingDirections = true;
            constraintSolveParams.passParams[FM_GS_PASS_IDX].useInnerAlternatingDirections = false;
            constraintSolveParams.passParams[FM_GS_PASS_IDX].kPgsRelaxationOmega = 0.8f;
            constraintSolveParams.passParams[FM_GS_PASS_IDX].outerRelErrorEpsilon = 0.001f;
            constraintSolveParams.passParams[FM_GS_PASS_IDX].innerRelErrorEpsilon = 0.001f;

#if FM_CONSTRAINT_STABILIZATION_SOLVE
            constraintStabilizationParams.kDistanceCorrection = 1.0f;
            constraintStabilizationParams.kVolumeCorrection = 0.05f;  // For volume contacts need a low value for stability

            constraintStabilizationParams.passParams[FM_CG_PASS_IDX].maxOuterIterations = 0;
            constraintStabilizationParams.passParams[FM_CG_PASS_IDX].maxInnerIterations = 1;
            constraintStabilizationParams.passParams[FM_CG_PASS_IDX].useInnerIterationsFalloff = false;
            constraintStabilizationParams.passParams[FM_CG_PASS_IDX].useOuterAlternatingDirections = false;
            constraintStabilizationParams.passParams[FM_CG_PASS_IDX].useInnerAlternatingDirections = false;
            constraintStabilizationParams.passParams[FM_CG_PASS_IDX].kPgsRelaxationOmega = 0.8f;
            constraintStabilizationParams.passParams[FM_CG_PASS_IDX].outerRelErrorEpsilon = 0.001f;
            constraintStabilizationParams.passParams[FM_CG_PASS_IDX].innerRelErrorEpsilon = 0.001f;
            constraintStabilizationParams.maxCgIterationsCgPass = 15;
            constraintStabilizationParams.epsilonCgPass = FLT_EPSILON;
            constraintStabilizationParams.passParams[FM_GS_PASS_IDX].maxOuterIterations = 4;
            constraintStabilizationParams.passParams[FM_GS_PASS_IDX].maxInnerIterations = 1;
            constraintStabilizationParams.passParams[FM_GS_PASS_IDX].useInnerIterationsFalloff = true;
            constraintStabilizationParams.passParams[FM_GS_PASS_IDX].useOuterAlternatingDirections = true;
            constraintStabilizationParams.passParams[FM_GS_PASS_IDX].useInnerAlternatingDirections = false;
            constraintStabilizationParams.passParams[FM_GS_PASS_IDX].kPgsRelaxationOmega = 0.8f;
            constraintStabilizationParams.passParams[FM_GS_PASS_IDX].outerRelErrorEpsilon = 0.001f;
            constraintStabilizationParams.passParams[FM_GS_PASS_IDX].innerRelErrorEpsilon = 0.001f;
#endif
        }
    };

    // Parameters needed to allocate scene
    struct FmSceneSetupParams
    {
        uint maxTetMeshBuffers;             // Limit on buffers created for tet meshes and their fractured tet mesh pieces.
        uint maxTetMeshes;                  // Limit on total tet meshes
        uint maxRigidBodies;                // Limit on total rigid bodies
        uint maxDistanceContacts;
        uint maxVolumeContacts;
        uint maxVolumeContactVerts;
        uint maxDeformationConstraints;
        uint maxGlueConstraints;
        uint maxPlaneConstraints;
        uint maxRigidBodyAngleConstraints;
        uint maxBroadPhasePairs;
        uint maxRigidBodyBroadPhasePairs;
        uint maxSceneVerts;                 // Limit on total verts of all tet meshes
        uint maxTetMeshBufferFeatures;      // Limit on verts, tets, or exterior faces per tet mesh buffer, to size temp memory
        size_t maxConstraintSolverDataSize; // Limit on preallocated size of constraint solver data (more dynamically allocated if needed)
        uint numWorkerThreads;
        bool rigidBodiesExternal;           // Set whether rigid bodies are owned by external system; NOTE: cannot be changed after scene creation

        inline FmSceneSetupParams() : maxTetMeshBuffers(0), maxTetMeshes(0), maxRigidBodies(0), maxDistanceContacts(0), maxVolumeContacts(0),
            maxVolumeContactVerts(0), maxDeformationConstraints(0), maxGlueConstraints(0), maxPlaneConstraints(0), maxRigidBodyAngleConstraints(0),
            maxBroadPhasePairs(0), maxRigidBodyBroadPhasePairs(0), maxSceneVerts(0), maxTetMeshBufferFeatures(0), maxConstraintSolverDataSize(0),
            numWorkerThreads(1), rigidBodiesExternal(false) {}
    };

    // Callback typedefs

    // Called to notify rigid body system of FEM library waking one of its islands containing rigid bodies.
    typedef void(*FmCallbackNotifyIslandWaking) (
        FmScene* scene, void* userData, uint* rigidBodyIds, uint numRigidBodies);

    // Called to notify rigid body system of FEM library putting to sleep one of its islands containing rigid bodies.
    typedef void(*FmCallbackNotifyIslandSleeping) (
        FmScene* scene, void* userData, uint* rigidBodyIds, uint numRigidBodies);

    // Query a static collision surface and return information to create a contact with an FEM mesh vertex.
    // vertStartPosition is FEM vertex position at start of step.
    // vertEndPosition is predicted FEM vertex position at end of step.
    // If contact with surface is detected, should output contacted position on the surface, and outward normal of the surface.
    // Return value is whether collision was detected.
    // Note, contact will be added only if projected distance along normal is less than FmSceneControlParams::distContactThreshold
    typedef bool(*FmCallbackSurfaceCollision) (FmVector3* outPosition, FmVector3* outNormal,
        const FmVector3& vertStartPosition, const FmVector3& vertEndPosition, uint tetMeshId, uint vertId);

    // Constraint used to glue object points together.
    struct FmGlueConstraintSetupParams
    {
        uint            bufferIdA;            // Tet mesh buffer id, or rigid body if FM_RB_FLAG set
        uint            bufferIdB;            // Tet mesh buffer id, or rigid body if FM_RB_FLAG set, or world position if FM_INVALID_ID
        uint            bufferTetIdA;         // If object A is tet mesh, contains id of tet in mesh buffer
        uint            bufferTetIdB;         // If object B is tet mesh, contains id of tet in mesh buffer

        union
        {
            float       posBaryA[4];          // Barycentric coordinate of contact point on tet mesh A
            float       posBodySpaceA[4];     // Vector from center of mass to contact point on rigid body A, in body space
        };
        union
        {
            float       posBaryB[4];          // Barycentric coordinate of contact point on tet mesh B
            float       posBodySpaceB[4];     // Vector from center of mass to contact point on rigid body B, in body space
            float       posWorldB[4];         // Position in world, for 
        };
        float           breakThreshold;       // If nonzero, is threshold in magnitude of impulse at which glue is automatically broken (disabled)
        float           kVelCorrection;       // Fraction to correct in constraint solve (velocity solve)
        float           kPosCorrection;       // Fraction to correct in stabilization (position solve)
        uint8_t         minGlueConstraints;   // Minimum number of glue constraints required between the glued objects.  This constraint will be broken/disabled if < this value.
        bool            enabled;              // Whether constraint is enabled initially

        inline FmGlueConstraintSetupParams() : bufferIdA(FM_INVALID_ID), bufferIdB(FM_INVALID_ID), bufferTetIdA(FM_INVALID_ID), bufferTetIdB(FM_INVALID_ID),
            breakThreshold(0.0f), kVelCorrection(0.1f), kPosCorrection(1.0f),
            minGlueConstraints(1), // Default allows this glue constraint to be the only one attaching two objects
            enabled(true)
        {
            posBaryA[0] = 0.0f;
            posBaryA[1] = 0.0f;
            posBaryA[2] = 0.0f;
            posBaryA[3] = 0.0f;
            posBaryB[0] = 0.0f;
            posBaryB[1] = 0.0f;
            posBaryB[2] = 0.0f;
            posBaryB[3] = 0.0f;
        }
    };

    // Constrains points on or above 1 to 3 planes.
    // Can use to create a line, pin, or distance constraint (update normal with UpdatePlaneConstraint()).
    struct FmPlaneConstraintSetupParams
    {
        uint        bufferIdA;            // Tet mesh buffer id, or rigid body if FM_RB_FLAG set
        uint        bufferIdB;            // Tet mesh buffer id, or rigid body if FM_RB_FLAG set, or world position if FM_INVALID_ID
        uint        bufferTetIdA;         // If object A is tet mesh, contains id of tet in mesh buffer
        uint        bufferTetIdB;         // If object B is tet mesh, contains id of tet in mesh buffer

        union
        {
            float   posBaryA[4];          // Barycentric coordinate of contact point on tet mesh A
            float   posBodySpaceA[4];     // Vector from center of mass to contact point on rigid body A, in body space
        };
        union
        {
            float   posBaryB[4];          // Barycentric coordinate of contact point on tet mesh B
            float   posBodySpaceB[4];     // Vector from center of mass to contact point on rigid body B, in body space
            float   posWorldB[4];
        };
        FmVector3   planeNormal0;         // Initial plane 0 normal in world space
        FmVector3   planeNormal1;         // Initial plane 1 normal in world space
        FmVector3   planeNormal2;         // Initial plane 2 normal in world space
        float       bias0;                // Initial distance bias to plane 0
        float       bias1;                // Initial distance bias to plane 1
        float       bias2;                // Initial distance bias to plane 2
        float       kVelCorrection;       // Fraction to correct in constraint solve (velocity solve)
        float       kPosCorrection;       // Fraction to correct in stabilization (position solve)

        uint        numDimensions;        // 1-3 planes supported
        bool        nonNeg0;              // If constraint on plane 0 is limited to non-negative (repulsive) forces
        bool        nonNeg1;              // If constraint on plane 1 is limited to non-negative (repulsive) forces
        bool        nonNeg2;              // If constraint on plane 2 is limited to non-negative (repulsive) forces
        bool        enabled;              // Whether constraint is enabled initially

        inline FmPlaneConstraintSetupParams() : bufferIdA(FM_INVALID_ID), bufferIdB(FM_INVALID_ID), bufferTetIdA(FM_INVALID_ID), bufferTetIdB(FM_INVALID_ID),
            planeNormal0(FmInitVector3(1.0f, 0.0f, 0.0f)), planeNormal1(FmInitVector3(0.0f, 1.0f, 0.0f)), planeNormal2(FmInitVector3(0.0f, 0.0f, 1.0f)),
            bias0(0.0f), bias1(0.0f), bias2(0.0f), kVelCorrection(0.1f), kPosCorrection(1.0f), numDimensions(3), nonNeg0(false), nonNeg1(false), nonNeg2(false),
            enabled(true)
        {
            posBaryA[0] = 0.0f;
            posBaryA[1] = 0.0f;
            posBaryA[2] = 0.0f;
            posBaryA[3] = 0.0f;
            posBaryB[0] = 0.0f;
            posBaryB[1] = 0.0f;
            posBaryB[2] = 0.0f;
            posBaryB[3] = 0.0f;
        }
    };

    // Angular constraint on rigid bodies which can be used to implement joints.
    struct FmRigidBodyAngleConstraintSetupParams
    {
        uint         objectIdA;            // Rigid body id with FM_RB_FLAG set
        uint         objectIdB;            // Rigid body id with FM_RB_FLAG set, or world position if FM_INVALID_ID
        FmVector3    axisBodySpaceA;       // Body-relative axis defining angle constraint
        FmVector3    axisBodySpaceB;       // Body-relative axis defining angle constraint
        float        frictionCoeff;        // Friction on joint
        float        kVelCorrection;       // Fraction to correct in constraint solve (velocity solve)
        float        kPosCorrection;       // Fraction to correct in stabilization (position solve)
        uint8_t      type;                 // One of FmRigidBodyAngleConstraintTypes (NOTE: currently only hinge supported)
        bool         enabled;              // Whether constraint is enabled initially

        inline FmRigidBodyAngleConstraintSetupParams() : objectIdA(FM_INVALID_ID), objectIdB(FM_INVALID_ID),
            axisBodySpaceA(FmInitVector3(0.0f)), axisBodySpaceB(FmInitVector3(0.0f)), frictionCoeff(FM_DEFAULT_FRICTION_COEFF),
            kVelCorrection(0.1f), kPosCorrection(1.0f), type(FM_RB_JOINT_TYPE_HINGE), enabled(true) {}
    };

    // Return the current world position inside a tet that corresponds to the input barycentric values
    FmVector3 FmGetInterpolatedPosition(const float bary[4], const FmTetMesh& tetMesh, uint tetId);

    // Return the current world velocity inside a tet that corresponds to the input barycentric values
    FmVector3 FmGetInterpolatedVelocity(const float bary[4], const FmTetMesh& tetMesh, uint tetId);

    // Get the tet vertex ids of a tet
    FmTetVertIds FmGetTetVertIds(const FmTetMesh& tetMesh, uint tetId);

    // Get the ids of tets incident to tet faces
    FmTetFaceIncidentTetIds FmGetTetFaceIncidentTetIds(const FmTetMesh& tetMesh, uint tetId);

    // Get the current world positions of the four corners a tet
    void FmGetTetPositions(FmVector3 outPositions[4], const FmTetMesh& tetMesh, uint tetId);

    // Get the current world positions of the three corners a tet face
    void FmGetTetFacePositions(FmVector3 outPositions[3], const FmTetMesh& tetMesh, uint tetId, uint faceId);

    // Get average of the current world positions of the corners
    FmVector3 FmGetTetCenter(const FmTetMesh& tetMesh, uint tetId);

    // Get the rotation of a tet
    FmMatrix3 FmGetTetRotation(const FmTetMesh& tetMesh, uint tetId);

    // Get the matrix derived from the tet rest positions that transforms a position in homogeneous coordinates (x,y,z,1) into barycentric coordinates.
    FmMatrix4 FmGetTetRestBaryMatrix(const FmTetMesh& tetMesh, uint tetId);

    // Get the material parameters of a tet
    FmTetMaterialParams FmGetTetMaterialParams(const FmTetMesh& tetMesh, uint tetId);

    // Estimates total number of bytes for constraint solver data based on FmSceneSetupParams parameters (all besides maxConstraintSolverDataSize).
    size_t FmEstimateSceneConstraintSolverDataSize(const FmSceneSetupParams& sceneSetupParams);

    // Allocate and initialize scene
    FmScene* FmCreateScene(const FmSceneSetupParams& params);

    // Free scene memory
    void FmDestroyScene(FmScene* scene);

    // Get size in bytes of scene
    size_t FmGetSceneSize(const FmScene& scene);

    // Get number of bytes sub-allocated for solver data and high water mark
    void FmGetSceneConstraintSolverDataSize(size_t* numBytes, size_t* highWaterMark, const FmScene& scene);

    // Get current scene control params
    const FmSceneControlParams& FmGetSceneControlParams(const FmScene& scene);

    // Set scene control params
    void FmSetSceneControlParams(FmScene* scene, const FmSceneControlParams& sceneControlParams);

    // Set task system callbacks
    void FmSetSceneTaskSystemCallbacks(FmScene* scene, const FmTaskSystemCallbacks& callbacks);

    // Set function called after scene update.
    // NOTE: If using FmUpdateScene() function, this is set to a default function used for synchronization.
    void FmSetPostSceneUpdateCallback(FmScene* scene, FmTaskFuncCallback postSceneUpdateCallback, void* postSceneUpdateData);

    // Set callback used to collide FEM object vertices with a user-defined surface
    void FmSetSurfaceCollisionCallback(FmScene* scene, FmCallbackSurfaceCollision surfaceCollisionCallback);

    // Set callbacks used if handling rigid bodies externally (setting FmSceneControlParams::rigidBodiesExternal true)
    void FmSetExternalRigidBodiesCallbacks(
        FmScene* scene,
        FmCallbackNotifyIslandWaking islandWakingCallback, 
        FmCallbackNotifyIslandSleeping islandSleepingCallback, 
        void* rigidBodiesUserData);

    // Access scene collision report
    FmCollisionReport& FmGetSceneCollisionReportRef(FmScene* scene);

    // Access scene fracture report
    FmFractureReport& FmGetSceneFractureReportRef(FmScene* scene);

    // Access scene warnings report
    FmWarningsReport& FmGetSceneWarningsReportRef(FmScene* scene);

    // Find tets incident to tet faces, using vertIncidentTets and tetVertIds.
    // Output to outTetFaceIncidentTetIdsArray, which should be sized to numTets.
    // Returns number of exterior faces.
    uint FmFindTetFaceIncidentTets(
        FmTetFaceIncidentTetIds* outTetFaceIncidentTetIds,
        FmArray<uint>* vertIncidentTets,
        const FmTetVertIds* tetVertIds,
        uint numTets);

    // Compute bounds on maxVerts, maxVertAdjacentVerts, maxExteriorFaces, and maxTetMeshes assuming maximum amount of fracture allowed by the supplied tet flags.
    // If enableFracture:
    //   - tetFlags expected to be an OR of FM_TET_FLAG_* values.
    //   - outFractureGroupCounts and outTetFractureGroupIds should be sized to the number of tets.
    // otherwise outFractureGroupCounts, outTetFractureGroupIds, tetFlags may be NULL.
    // NOTE: Any tetFlags set here must also be applied to tet mesh tets and not changed after initialization.
    void FmComputeTetMeshBufferBounds(
        FmTetMeshBufferBounds* outBounds,
        FmFractureGroupCounts* outFractureGroupCounts,
        uint* outTetFractureGroupIds,
        FmArray<uint>* vertIncidentTets,
        const FmTetVertIds* tetVertIds,
        const uint16_t* tetFlags,
        uint numVerts, uint numTets, bool enableFracture);

    // Allocates using FmTetMeshBufferSetupParams, but application must still initialize most data including topology, state, matrices.
    // Also sets the pTetMeshPtr to the initial FmTetMesh (before fracture occurs).
    FmTetMeshBuffer* FmCreateTetMeshBuffer(
        const FmTetMeshBufferSetupParams& params,
        const FmFractureGroupCounts* fractureGroupCounts,
        const uint* tetFractureGroupIds,
        FmTetMesh** pTetMeshPtr);

    // Free tet mesh buffer memory
    void FmDestroyTetMeshBuffer(FmTetMeshBuffer* tetMeshBuffer);

    // Get size in bytes of tet mesh buffer
    size_t FmGetTetMeshBufferSize(const FmTetMeshBuffer& tetMeshBuffer);

    // Add a set up tet mesh buffer to the scene.
    // Returns a buffer id (which is the index where the buffer pointer is stored in FmScene::tetMeshBuffers).
    // If maximum tet mesh buffers used, returns FM_INVALID_ID
    uint FmAddTetMeshBufferToScene(FmScene* scene, FmTetMeshBuffer* tetMeshBuffer);

    // Remove a tet mesh buffer from the scene.
    void FmRemoveTetMeshBufferFromScene(FmScene* scene, uint tetMeshBufferId);

    // After adding tet mesh buffers and rigid body scene, group them into a set of objects that 
    // will not be actively simulated until an event such as collision or deletion.
    bool FmCreateSleepingIsland(FmScene* scene, uint* tetMeshIds, uint numTetMeshes, uint* rigidBodyIds, uint numRigidBodies);

    // Puts all currently active scene objects into sleeping state and in same island.
    bool FmSetAllSceneObjectsSleeping(FmScene* scene);

    // Get a FmTetMeshBuffer* by buffer id; returns NULL if not found.
    FmTetMeshBuffer* FmGetTetMeshBuffer(const FmScene& scene, uint tetMeshBufferId);

    // Get a FmTetMesh* by object id; returns NULL if not found.
    FmTetMesh* FmGetTetMesh(const FmScene& scene, uint objectId);

    // Get id of tet mesh buffer
    uint FmGetTetMeshBufferId(const FmTetMesh& tetMesh);
    uint FmGetTetMeshBufferId(const FmTetMeshBuffer& tetMeshBuffer);

    // Get number of tetrahedra contained in tet mesh buffer
    uint FmGetNumTets(const FmTetMeshBuffer& tetMeshBuffer);

    // Get fracture group and flags for tet in tet mesh buffer
    void FmGetTetMeshBufferTetInfo(uint* fractureGroupId, uint16_t* tetFlags, const FmTetMeshBuffer& tetMeshBuffer, uint bufferTetId);

    // Get number of tet meshes contained in tet mesh buffer
    uint FmGetNumTetMeshes(const FmTetMeshBuffer& tetMeshBuffer);

    // Get maximum number of tet meshes contained in tet mesh buffer after all fracture
    uint FmGetMaxTetMeshes(const FmTetMeshBuffer& tetMeshBuffer);

    // Get a FmTetMesh* by index within a FmTetMeshBuffer; returns NULL if not found.
    // Use FmGetNumTetMeshes(const FmTetMeshBuffer& tetMeshBuffer) to get number.
    FmTetMesh* FmGetTetMesh(const FmTetMeshBuffer& tetMeshBuffer, uint meshIdx);

    // From a FmVertMeshBuffer and original vertex id (pre-fracture), return the FmVertMesh containing this vertex.
    // Set meshVertId to the id within the tet mesh, and bufferVertMeshIdx to index of sub tet mesh within buffer.
    FmTetMesh* FmGetTetMeshContainingVert(uint* meshVertId, uint* bufferVertMeshIdx, const FmTetMeshBuffer& tetMeshBuffer, uint bufferVertId);

    // From a FmTetMeshBuffer and original tet id (pre-fracture), return the FmTetMesh containing this tet.
    // Set meshTetId to the id within the tet mesh, and bufferTetMeshIdx to index of sub tet mesh within buffer.
    FmTetMesh* FmGetTetMeshContainingTet(uint* meshTetId, uint* bufferTetMeshIdx, const FmTetMeshBuffer& tetMeshBuffer, uint bufferTetId);

    // Add rigid body to the scene and return an id.
    uint FmAddRigidBodyToScene(FmScene* scene, FmRigidBody* inRigidBody);

    // Remove rigid body with the given id.
    void FmRemoveRigidBodyFromScene(FmScene* scene, uint rigidBodyId);

    // Get object id of rigid body
    uint FmGetObjectId(const FmRigidBody& rigidBody);

    // Get state of rigid body
    FmVector3 FmGetPosition(const FmRigidBody& rigidBody);
    FmVector3 FmGetVelocity(const FmRigidBody& rigidBody);
    FmQuat FmGetRotation(const FmRigidBody& rigidBody);
    FmVector3 FmGetAngularVelocity(const FmRigidBody& rigidBody);
    FmRigidBodyState FmGetState(const FmRigidBody& rigidBody);

    // Set state of rigid body
    // If non-NULL scene provided, will wake rigid body if asleep.
    void FmSetPosition(FmScene* scene, FmRigidBody* rigidBody, const FmVector3& position);
    void FmSetVelocity(FmScene* scene, FmRigidBody* rigidBody, const FmVector3& velocity);
    void FmSetRotation(FmScene* scene, FmRigidBody* rigidBody, const FmQuat& rotation);
    void FmSetAngularVelocity(FmScene* scene, FmRigidBody* rigidBody, const FmVector3& angularVelocity);
    void FmSetState(FmScene* scene, FmRigidBody* rigidBody, const FmRigidBodyState& state);

    // Set collision group
    void FmSetCollisionGroup(FmRigidBody* rigidBody, uint8_t collisionGroup);

    // Set a gravity value for a rigid body that will sum with the scene gravity.  Default value is zero.
    void FmSetGravity(FmRigidBody* rigidBody, const FmVector3& gravityVector);

    // Notify of object waking due to external change or collision event, in order to wake FEM island.
    // Not necessary if calling FmSetState.
    void FmNotifyObjectWaking(FmScene* scene, uint objectId);

    // Notify of rigid bodies solved outside FEM system being put to sleep.
    bool FmNotifyRigidBodiesSleeping(FmScene* scene, uint* rigidBodyIds, uint numRigidBodies);

    // Get a rigid body by id; returns NULL if not found.
    FmRigidBody* FmGetRigidBody(const FmScene& scene, uint rigidBodyId);

    // Get total number of tet meshes, awake or asleep, which are enabled for simulation
    uint FmGetNumEnabledTetMeshes(const FmScene& scene);

    // Get object id for tet mesh in range 0..FmGetNumEnabledTetMeshes()-1
    uint FmGetEnabledTetMeshId(const FmScene& scene, uint enabledIdx);

    // Get total number of rigid bodies, awake or asleep, which are enabled for simulation
    uint FmGetNumEnabledRigidBodies(const FmScene& scene);

    // Get object id for rigid body in range 0..FmGetNumEnabledRigidBodies()-1
    uint FmGetEnabledRigidBodyId(const FmScene& scene, uint enabledIdx);

    // Add a constraint to the scene; Returns an id, or if maximum constraints created, FM_INVALID_ID
    uint FmAddGlueConstraintToScene(FmScene* scene, const FmGlueConstraintSetupParams& glueConstraintSetupParams);
    uint FmAddPlaneConstraintToScene(FmScene* scene, const FmPlaneConstraintSetupParams& planeConstraintSetupParams);
    uint FmAddRigidBodyAngleConstraintToScene(FmScene* scene, const FmRigidBodyAngleConstraintSetupParams& rigidBodyAngleConstraintSetupParams);

    // Remove a constraint from the scene.
    void FmRemoveGlueConstraintFromScene(FmScene* scene, uint glueConstraintId);
    void FmRemovePlaneConstraintFromScene(FmScene* scene, uint planeConstraintId);
    void FmRemoveRigidBodyAngleConstraintFromScene(FmScene* scene, uint rigidBodyAngleConstraintId);

    // Get the current constraint parameters
    FmGlueConstraintSetupParams FmGetGlueConstraintParams(const FmScene& scene, uint glueConstraintId);
    FmPlaneConstraintSetupParams FmGetPlaneConstraintParams(const FmScene& scene, uint planeConstraintId);
    FmRigidBodyAngleConstraintSetupParams FmGetRigidBodyAngleConstraintParams(const FmScene& scene, uint rigidBodyAngleConstraintId);

    // Get the last impulse applied to maintain the glue constraint
    FmVector3 FmGetGlueConstraintImpulse(const FmScene& scene, uint glueConstraintId);

    // Enable/disable constraints in the scene
    void FmEnableGlueConstraint(FmScene* scene, uint glueConstraintId, bool enabled);
    void FmEnablePlaneConstraint(FmScene* scene, uint planeConstraintId, bool enabled);
    void FmEnableRigidBodyAngleConstraint(FmScene* scene, uint rigidBodyAngleConstraintId, bool enabled);

    // Get current enabled or disabled status
    bool FmGetGlueConstraintEnabled(const FmScene& scene, uint glueConstraintId);
    bool FmGetPlaneConstraintEnabled(const FmScene& scene, uint planeConstraintId);
    bool FmGetRigidBodyAngleConstraintEnabled(const FmScene& scene, uint rigidBodyAngleConstraintId);

    // Set plane constraint normals
    void FmSetPlaneConstraintNormals(FmScene* scene, uint planeConstraintId, const FmVector3& planeNormal0, const FmVector3& planeNormal1, const FmVector3& planeNormal2);

    // Set plane constraint biases
    void FmSetPlaneConstraintBiases(FmScene* scene, uint planeConstraintId, float bias0, float bias1, float bias2);

    // Set friction coefficient
    void FmSetRigidBodyAngleConstraintFrictionCoeff(FmScene* scene, uint rigidBodyAngleConstraintId, float frictionCoeff);

    // Get id of tet mesh
    uint FmGetObjectId(const FmTetMesh& tetMesh);

    // Get number of vertices in tet mesh.
    uint FmGetNumVerts(const FmTetMesh& tetMesh);

    // Get number of tetrahedra in tet mesh.
    uint FmGetNumTets(const FmTetMesh& tetMesh);

    // Get number of exterior faces in tet mesh.
    uint FmGetNumExteriorFaces(const FmTetMesh& tetMesh);

    // Get the tet id and face id (0..3) for the exterior face in a tet mesh.
    // Use FmGetNumExteriorFaces() to get number.
    void FmGetExteriorFace(uint* tetId, uint* faceId, const FmTetMesh& tetMesh, uint extFaceIdx);

    // Get number of new exterior faces created by fractures in last simulation update.
    uint FmGetNumNewExteriorFaces(const FmTetMesh& tetMesh);

    // Get the tet id and face id (0..3) for the new exterior face in a tet mesh.
    // Use FmGetNumNewExteriorFaces() to get number.
    void FmGetNewExteriorFace(uint* tetId, uint* faceId, const FmTetMesh& tetMesh, uint newExtFaceIdx);

    // Get the tet id and face in the original unfractured tet mesh buffer that corresponds to a new exterior face.
    // Use FmGetNumNewExteriorFaces() to get number.
    void FmGetNewExteriorFaceInTetMeshBuffer(uint* bufferTetId, uint* bufferTetFaceId, const FmTetMesh& tetMesh, uint newExtFaceIdx);

    // Get min position of tet mesh.
    FmVector3 FmGetMinPosition(const FmTetMesh& tetMesh);

    // Get max position of tet mesh.
    FmVector3 FmGetMaxPosition(const FmTetMesh& tetMesh);

    // Get center of mass of tet mesh.
    FmVector3 FmGetCenterOfMass(const FmTetMesh& tetMesh);

    // Threshold for vertex speed beyond which external forces are not applied
    void FmSetExternalForceSpeedLimit(FmTetMesh* tetMesh, float speedLimit);

    // Set stress threshold used to remove kinematic vertex property, when vertices are marked FM_VERT_FLAG_KINEMATIC_REMOVABLE
    void FmSetRemoveKinematicThreshold(FmTetMesh* tetMesh, float stressThreshold);

    // Set collision group
    void FmSetCollisionGroup(FmTetMesh* tetMesh, uint8_t collisionGroup);

    // Set maximum speed below which mesh considered stable
    void FmSetSleepMaxSpeedThreshold(FmTetMesh* tetMesh, float threshold);

    // Set average speed below which mesh considered stable
    void FmSetSleepAvgSpeedThreshold(FmTetMesh* tetMesh, float threshold);

    // Set required count for mesh to be stable before sleeping
    void FmSetSleepStableCount(FmTetMesh* tetMesh, uint count);

    // Set a gravity value for a mesh that will sum with the scene gravity.  Default value is zero.
    void FmSetGravity(FmTetMesh* tetMesh, const FmVector3& gravityVector);

    // Compute volume from rest position array, to help with setup of material parameters.
    float FmComputeTetMeshVolume(const FmVector3* vertRestPositions, const FmTetVertIds* tetVertIds, uint numTets);

    // Add tet id to the incident tets of a vertex
    void FmAddIncidentTetToSet(FmArray<uint>& vertTetIds, uint tetId);

    // Initialize the vertex rest positions, positions (as a transformation of rest positions), mass, and velocity.
    // Mass will be replaced later if using FmSetMassesFromRestDensities().
    // Uses one velocity value to initialize all vertices; can loop over vertices for more custom settings.
    void FmInitVertState(
        FmTetMesh* tetMesh,
        const FmVector3* vertRestPositions,
        const FmMatrix3& rotation, const FmVector3& translation,
        float mass = 1.0f,
        const FmVector3& velocity = FmVector3(0.0f));

    // Initialize the tet vertex ids and material parameters.
    // Uses one material parameters value to initialize all tets; can loop over tets for more custom settings.
    // Sets tet mesh default removeKinematicStressThreshold to 4X of fractureStressThreshold.
    void FmInitTetState(
        FmTetMesh* tetMesh,
        const FmTetVertIds* tetVertIds,
        const FmTetMaterialParams& materialParams,
        float frictionCoeff = FM_DEFAULT_FRICTION_COEFF);

    // Use tets incident on verts to complete mesh connectivity.
    void FmFinishConnectivityFromVertIncidentTets(FmTetMesh* tetMesh);

    // Compute constant stiffness, elasticity, strain and stress matrices for each tet of mesh.
    // Assumes vert rest positions, tet indices, and material params initialized.
    // Returns rest volume.
    float FmComputeMeshConstantMatrices(FmTetMesh* tetMesh);

    // Sets mass of each tet from its rest density and volume, and distributes mass evenly to tet vertices.
    // If restDensity non-zero, uses this for density of all tets.
    void FmSetMassesFromRestDensities(FmTetMesh* tetMesh, float restDensity = 0.0f);

    // Use lists of tets incident to verts to initialize tet mesh connectivity.
    // Returns whether completed; fails if any vertex has more than FM_MAX_VERT_INCIDENT_TETS
    bool FmInitConnectivity(FmTetMesh* tetMesh, const FmArray<uint>* vertIncidentTets);
       
    // Distributes vert masses to incident tets.
    void FmDistributeVertMassesToTets(FmTetMesh* tetMesh);

    // Shift rest positions so center of mass is at origin, and return original center of mass.
    FmVector3 FmMoveRestPositionCenterOfMassToOrigin(FmTetMesh* tetMesh);

    // Scale all vertex masses to achieve new total mass.
    void FmSetTotalMass(FmTetMesh* tetMesh, float mass);

    // Enable or disable the surface collision for individual tet mesh (enabled by default)
    void FmEnableSurfaceCollisionCallback(FmTetMesh* tetMesh, bool isEnabled);

    // Enable or disable simulation of object.
    void FmEnableSimulation(FmScene* scene, FmTetMesh* tetMesh, bool isEnabled);
    void FmEnableSimulation(FmScene* scene, FmRigidBody* rigidBody, bool isEnabled);

    // Enable or disable sleeping of object.
    void FmEnableSleeping(FmScene* scene, FmTetMesh* tetMesh, bool isEnabled);
    void FmEnableSleeping(FmScene* scene, FmRigidBody* rigidBody, bool isEnabled);

    // Enable or disable computing strain measure on all tets.
    void FmEnableStrainMagComputation(FmTetMesh* tetMesh, bool isEnabled);

    // Enable or disable self collision of tet mesh.
    // NOTE: experimental and not optimized
    void FmEnableSelfCollision(FmTetMesh* tetMesh, bool isEnabled);

    // Checks a variety of requirements for tet mesh validity.
    bool FmValidateMesh(const FmTetMesh& tetMesh);

    // Call after all initialization of tet mesh data and before scene update.
    // Returns:
    // 0 if successful
    // -1 if any tets have aspect ratio > FM_MAX_TET_ASPECT_RATIO (higher aspect ratios can cause instability)
    // -2 if any tets are inverted
    int FmFinishTetMeshInit(FmTetMesh* tetMesh);

    // Returns estimated condition number of Implicit Euler system matrix, given specified scene parameters.
    float FmCheckTetMeshCondition(FmTetMesh* tetMesh, const FmSceneControlParams& sceneControlParams);

    // Returns the maximum estimated condition number of Implicit Euler system matrix over all fracture groups, given specified scene parameters.
    // Should use after setup of tet mesh buffer and before any fracture has occurred.
    float FmCheckMaxTetMeshCondition(FmTetMeshBuffer* tetMeshBuffer, const FmSceneControlParams& sceneControlParams);

    // Set positions to be a rigid transformation of the rest positions, and undo plastic deformation.
    // If non-NULL scene provided, will wake tet mesh if asleep.
    void FmResetFromRestPositions(FmScene* scene, FmTetMesh* tetMesh, const FmMatrix3& rotation, const FmVector3& translation, const FmVector3& velocity = FmVector3(0.0f));

    // Set vertex mass
    void FmSetVertMass(FmTetMesh* tetMesh, uint vertId, float mass);

    // Get FM_VERT_FLAG_* values
    uint16_t FmGetVertFlags(const FmTetMesh& tetMesh, uint vertId);

    // Set FM_VERT_FLAG_* values
    void FmSetVertFlags(FmTetMesh* tetMesh, uint vertId, uint16_t flags);

    // Add FM_VERT_FLAG_* values into vertex flags
    void FmAddVertFlags(FmTetMesh* tetMesh, uint vertId, uint16_t flags);

    // Remove FM_VERT_FLAG_* values from vertex flags
    void FmRemoveVertFlags(FmTetMesh* tetMesh, uint vertId, uint16_t flags);

    // Set FM_TET_FLAG_* values
    void FmSetTetFlags(FmTetMesh* tetMesh, uint tetId, uint16_t flags);

    // Add FM_TET_FLAG_* values into tet flags
    void FmAddTetFlags(FmTetMesh* tetMesh, uint tetId, uint16_t flags);

    // Remove FM_TET_FLAG_* values from tet flags
    void FmRemoveTetFlags(FmTetMesh* tetMesh, uint tetId, uint16_t flags);

    // Get position or velocity of vertex.
    FmVector3 FmGetVertRestPosition(const FmTetMesh& tetMesh, uint vertId);
    FmVector3 FmGetVertPosition(const FmTetMesh& tetMesh, uint vertId);
    FmVector3 FmGetVertVelocity(const FmTetMesh& tetMesh, uint vertId);

    // Get external force
    FmVector3 FmGetVertExternalForce(const FmTetMesh& tetMesh, uint vertId);

    // Get mass
    float FmGetVertMass(const FmTetMesh& tetMesh, uint vertId);

    // Get neighbor data of vertex
    FmVertNeighbors FmGetVertNeighbors(const FmTetMesh& tetMesh, uint vertId);

    // Get the original index of vertex when FmTetMeshBuffer created
    uint FmGetVertIndex0(const FmTetMesh& tetMesh, uint vertId);

    // Get magnitude of elastic strain, either lifetime max or last frame average
    float FmGetVertTetStrainMagAvg(const FmTetMesh& tetMesh, uint vertId);
    float FmGetVertTetStrainMagMax(const FmTetMesh& tetMesh, uint vertId);

    // Get the sum of quaternion rotations of all tets incident at a vertex.  Can normalize to get an average rotation for normal calculations.
    FmQuat FmGetVertTetQuatSum(const FmTetMesh& tetMesh, uint vertId);

    // Set position or velocity of vertex.
    // If non-NULL scene provided, will wake tet mesh if asleep.
    void FmSetVertPosition(FmScene* scene, FmTetMesh* tetMesh, uint vertId, const FmVector3& position);
    void FmSetVertVelocity(FmScene* scene, FmTetMesh* tetMesh, uint vertId, const FmVector3& velocity);

    // Add external force to vertex.
    // NOTE: external forces are reset to 0 at end of simulation step
    // If non-NULL scene provided, will wake tet mesh if asleep.
    void FmAddForceToVert(FmScene* scene, FmTetMesh* tetMesh, uint vertId, const FmVector3& force);

    // Update material of tet; use if modifying the tet material dynamically during simulation, to properly update internal data.
    // If non-NULL scene provided, will wake tet mesh if asleep.
    void FmUpdateTetMaterialParams(FmScene* scene, FmTetMesh* tetMesh, uint meshTetId, const FmTetMaterialParams& newMaterialParams, float plasticDeformationAttenuation = 1.0f, bool volumePreservingPlasticity = false);

    // Update material for entire tet mesh or buffer; use if modifying the tet material dynamically during simulation, to properly update internal data.
    // If non-NULL scene provided, will wake tet mesh if asleep.
    void FmUpdateAllTetMaterialParams(FmScene* scene, FmTetMesh* tetMesh, const FmTetMaterialParams& newMaterialParams, float plasticDeformationAttenuation = 1.0f, bool volumePreservingPlasticity = false);

    // Update scene without rigid bodies.
    void FmUpdateScene(FmScene* scene, float timestep);

    // Ratio of maximum edge length to minimum height of vertex from plane containing other vertices.
    // Measure of tetrahedron quality, with 1 being optimal value.
    float FmComputeTetAspectRatio(const FmVector3 tetRestPositions[4]);

    // Scene update broken down for use with external rigid bodies.

    // Rebuild meshes after fracture, solve end velocity without regard for constraints, and rebuild hierarchies.
    // If FM_ASYNC_THREADING and followTaskFunc non-NULL, this call will execute asynchronously, and may return before tasks are complete. 
    // These tasks will execute followTaskFunc with followTaskData (and index 0) when complete.
    void FmUpdateUnconstrained(FmScene* scene, float timestep, FmTaskFuncCallback followTaskFunc, void* followTaskData);

    // Find contacts from intersections at start of step and CCD.
    // If FM_ASYNC_THREADING and followTaskFunc non-NULL, this call will execute asynchronously, and may return before tasks are complete. 
    // These tasks will execute followTaskFunc with followTaskData (and index 0) when complete.
    void FmFindContacts(FmScene* scene, FmTaskFuncCallback followTaskFunc, void* followTaskData);

    // Wake islands flagged due contacts found in FmFindContacts, and find new contacts between awakened objects
    // If FM_ASYNC_THREADING and followTaskFunc non-NULL, this call will execute asynchronously, and may return before tasks are complete. 
    // These tasks will execute followTaskFunc with followTaskData (and index 0) when complete.
    void FmWakeCollidedIslandsAndFindContacts(FmScene* scene, FmTaskFuncCallback followTaskFunc, void* followTaskData);

    // Query scene if two collision groups have collision enabled.
    bool FmGroupsCanCollide(const FmScene& scene, uint i, uint j);

    // Set in scene whether two collision groups can collide.
    void FmSetGroupsCanCollide(FmScene* scene, uint i, uint j, bool canCollide);

    // For the given tetrahedron positions, compute 4x4 matrix transforming points to barycentric coordinates
    FmMatrix4 FmComputeTetBarycentricMatrix(const FmVector3& tetRestPosition0, const FmVector3& tetRestPosition1, const FmVector3& tetRestPosition2, const FmVector3& tetRestPosition3);
    FmMatrix4 FmComputeTetBarycentricMatrix(const FmVector3* vertRestPositions, const FmTetVertIds& tetVerts);

    // Get barycentric coordinates of point
    FmVector4 FmComputeBarycentricCoords(const FmVector3& tetRestPosition0, const FmVector3& tetRestPosition1, const FmVector3& tetRestPosition2, const FmVector3& tetRestPosition3, const FmVector3& point);
    FmVector4 FmComputeBarycentricCoords(const FmVector3* vertRestPositions, const FmTetVertIds& tetVerts, const FmVector3& point);

    // Create or destroy an FmBvh
    FmBvh* FmCreateBvh(uint numPrims);
    void FmDestroyBvh(FmBvh* bvh);

    // Get top level bounding box of BVH
    void FmGetBoundingBox(FmVector3* minPosition, FmVector3* maxPosition, const FmBvh& bvh);

    // Build BVH over rest mesh tetrahedra for nearest tetrahedron query.
    // bvh expected to be preallocated for tetMesh->numTets leaves
    // (see FmCreateBvh() and FmDestroyBvh()).
    void FmBuildRestMeshTetBvh(FmBvh* bvh, const FmTetMesh& tetMesh);
    void FmBuildRestMeshTetBvh(FmBvh* bvh, const FmVector3* vertRestPositions, const FmTetVertIds* tetVertIds, uint numTets);

    // Results of FmFindClosestTet
    struct FmClosestTetResult
    {
        FmVector3 position; // Closest position on closest tet
        float posBary[4];      // Barycentric coordinates of closest position
        float distance;        // Distance to closest tet
        uint tetId;            // Closest tet id
        uint faceId;           // Closest face if query point outside tet
        bool insideTet;        // According to Smith and Dodgson robust intersection

        FmClosestTetResult()
        {
            position = FmInitVector3(0.0f);
            posBary[0] = 0.0f;
            posBary[1] = 0.0f;
            posBary[2] = 0.0f;
            posBary[3] = 0.0f;
            distance = 0.0f;
            tetId = FM_INVALID_ID;
            faceId = FM_INVALID_ID;
            insideTet = false;
        }
    };

    // Queries a tet mesh at its rest positions to find the closest tet and barycentric values corresponding to the input position.
    // Requires a BVH built with FmBuildRestMeshTetBvh.
    void FmFindClosestTet(FmClosestTetResult* closestTet, const FmTetMesh* tetMesh, const FmBvh* bvh, const FmVector3& queryPoint);
    void FmFindClosestTet(FmClosestTetResult* closestTet, const FmVector3* vertRestPositions, const FmTetVertIds* tetVertIds, const FmBvh* bvh, const FmVector3& queryPoint);

    // Queries a tet mesh at its rest positions to find the tet which is intersected and barycentric values corresponding to the input position.
    void FmFindIntersectedTet(FmClosestTetResult* closestTet, const FmTetMesh* tetMesh, const FmBvh* bvh, const FmVector3& queryPoint);
    void FmFindIntersectedTet(FmClosestTetResult* closestTet, const FmVector3* vertRestPositions, const FmTetVertIds* tetVertIds, const FmBvh* bvh, const FmVector3& queryPoint);

    // Find all the rest mesh tetrahedra that intersect the box.
    // Saves output to tetsIntersectingBox array, expected to be large enough for all tets of the mesh.
    // Returns the number of intersected tets.
    uint FmFindTetsIntersectingBox(
        uint* tetsIntersectingBox,
        const FmTetMesh* tetMesh, const FmBvh* bvh,
        const FmVector3& boxHalfDimensions,
        const FmVector3& boxCenterPos,
        const FmMatrix3& boxRotation);
    uint FmFindTetsIntersectingBox(
        uint* tetsIntersectingBox,
        const FmVector3* vertRestPositions, const FmTetVertIds* tetVertIds, const FmBvh* bvh,
        const FmVector3& boxHalfDimensions,
        const FmVector3& boxCenterPos,
        const FmMatrix3& boxRotation);

    // Compute the relative angular velocity of rigid bodies around the hinge axis.
    float FmComputeHingeRelAngVel(const FmScene& scene, uint rigidBodyAngleConstraintId);

    // Find islands of objects (FEM meshes and supplied rigid bodies) which are connected by
    // constraints.  Each island must be solved as a unit during constraint solving.  
    // Optional argument allows user to provide constraints from external system.  See FmUserConstraints in FEMFXInternal.h 
    void FmFindConstraintIslands(
        FmScene* scene,
        const FmUserConstraints* userConstraints = NULL);

    // Do a full solve and stabilization of scene's constraints.
    // Step positions, recompute tet rotations and center of mass.
    // If applicable update plasticity state or fracture meshes.
    // If FM_ASYNC_THREADING and followTaskFunc non-NULL, this call will execute asynchronously, and may return before tasks are complete. 
    // These tasks will execute followTaskFunc with followTaskData (and index 0) when complete.
    void FmSceneConstraintSolve(FmScene* scene, FmTaskFuncCallback followTaskFunc, void* followTaskData);

#if FM_ASYNC_THREADING
    // To manually control synchronization of scene update, must register a postUpdateSceneCallback and postUpdateSceneData, 
    // allocate a FmTaskDataUpdateScene with new, then use a FmSubmitTaskCallback to submit FmTaskFuncUpdateSceneStart with this data.
    // When the scene update is finished, it will delete FmTaskDataUpdateScene and call the postUpdateSceneCallback.
    class FmTaskDataUpdateScene
    {
    public:
        FM_CLASS_NEW_DELETE(FmTaskDataUpdateScene)

        FmScene* scene;
        float timestep;

        FmTaskDataUpdateScene(FmScene* inScene, float inTimestep)
        {
            scene = inScene;
            timestep = inTimestep;
        }
    };

    // Example use:
    // FmTaskDataUpdateScene* taskDataUpdateScene = new FmTaskDataUpdateScene(scene, timestep);
    // SubmitAsyncTask(FmTaskFuncUpdateSceneStart, taskDataUpdateScene, 0);
    void FmTaskFuncUpdateSceneStart(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);
#endif
}
