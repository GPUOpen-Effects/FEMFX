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

#include "AMD_FEMFX.h"
#include "FEMFXArray.h"

// Scene options - select only one

#ifndef FEMFX_VIEWER_ENV_SET
// Three wood panels fractured by projectile ('f' or 'h')
#define WOOD_PANELS_SCENE        0

// Pile of softbody blocks
#define BLOCKS_SCENE             0

// Sends a car through wood panels and stacks of tractor tires
#define CARS_PANELS_TIRES_SCENE  1

// Pile of softbody ducks
#define DUCKS_SCENE              0

// Rectangular panel with adjustable material parameters (FmDemo only)
#define MATERIAL_DEMO_SCENE      0

// Load object from .FEM file
#define LOAD_FEM_ASSET           0

// Load serialized scene
#define LOAD_SERIALIZED_SCENE    0

// Test stack of FEM and rigid bodies
#define RIGIDBODY_TEST_SCENE     0
#endif

// Other options

#define SELF_COLLISION_TEST      (0 && BLOCKS_SCENE)

#define KINEMATIC_TEST           0

// Enable proof of concept interfacing with external system handling rigid body collision detection, constraint solving, and integration.
// Only supported with RIGIDBODY_TEST_SCENE
#define EXTERNAL_RIGIDBODIES     (0 && RIGIDBODY_TEST_SCENE)

// With RIGIDBODY_TEST_SCENE 1, demonstrates FEM and rigid body glued together by constraints
#define GLUE_TEST                (0 && RIGIDBODY_TEST_SCENE)

// Configures performance test in FEXFXViewer.
// Test also requires FM_TIMINGS 1 in AMD_FEMFX.h to include the timing calls in the library.
#ifndef FEMFX_VIEWER_ENV_SET
#define PERF_TEST                0
#endif

// Run the same simulation repeatedly, instead of randomizing initial setup.
#define FIX_INITIAL_CONDITIONS   (0 || PERF_TEST)

// Set demo constants
#define CARS_IN_SCENE        (CARS_SCENE || CARS_PANELS_TIRES_SCENE)
#define WOOD_PANELS_IN_SCENE (WOOD_PANELS_SCENE || CARS_PANELS_TIRES_SCENE)
#define TIRES_IN_SCENE       (CARS_PANELS_TIRES_SCENE)
#define PROJECTILE_IN_SCENE  (CARS_SCENE || CARS_PANELS_TIRES_SCENE || WOOD_PANELS_SCENE || MATERIAL_DEMO_SCENE || LOAD_FEM_ASSET || LOAD_SERIALIZED_SCENE)

#if EXTERNAL_RIGIDBODIES
#include "ExampleRigidBodies.h"
extern AMD::ExampleRigidBodiesScene* gRbScene;
#endif

// Update objects, different depending on test scene
void UpdateObjects();

// Geometry and settings used to create an mesh instance
struct TetMeshBufferTemplate
{
    AMD::FmVector3*                 vertRestPositions;
    AMD::FmTetVertIds*              tetVertIds;
    AMD::FmArray<AMD::uint>*        vertIncidentTets;
    AMD::FmTetMeshBufferSetupParams setupParams;
    AMD::FmBvh*                     tetsBvh;
    AMD::FmTetMaterialParams        defaultMaterialParams;
    AMD::FmFractureGroupCounts*     fractureGroupCounts;
    AMD::uint*                      tetFractureGroupIds;
    uint16_t*                       tetFlags;

    TetMeshBufferTemplate()
    {
        vertRestPositions = NULL;
        tetVertIds = NULL;
        vertIncidentTets = NULL;
        tetsBvh = NULL;
        fractureGroupCounts = NULL;
        tetFractureGroupIds = NULL;
        tetFlags = NULL;
    }

    void Destroy()
    {
        FM_DELETE_ARRAY(vertRestPositions);
        FM_DELETE_ARRAY(tetVertIds);
        FM_DELETE_ARRAY(vertIncidentTets);
        FM_DELETE_ARRAY(fractureGroupCounts);
        FM_DELETE_ARRAY(tetFractureGroupIds);
        FM_DELETE_ARRAY(tetFlags);
        FmDestroyBvh(tetsBvh);
    }
};

#if CARS_PANELS_TIRES_SCENE
#define NUM_INSTANCES 3
#define NUM_CARS_PER_INSTANCE 1
#define NUM_WOOD_PANELS_PER_INSTANCE 3
#else
#define NUM_CARS 16
#define NUM_WOOD_PANELS_PER_ROW 5
#define NUM_WOOD_PANELS_ROWS    4
#define NUM_WOOD_PANELS (NUM_WOOD_PANELS_PER_ROW*NUM_WOOD_PANELS_ROWS)
#endif

#if WOOD_PANELS_IN_SCENE
#define PANEL_HEIGHT 3.0f
#define PANEL_WIDTH 3.0f
#define PANEL_DEPTH 0.1f
extern TetMeshBufferTemplate* gWoodPanelBufferTemplate;
#endif

#if CARS_IN_SCENE
#define NUM_CAR_TEMPLATES 8
extern TetMeshBufferTemplate* gCarTemplates[NUM_CAR_TEMPLATES];
#endif

#if TIRES_IN_SCENE
#define TRACTOR_TIRE_RADIUS 0.88f
#define TRACTOR_TIRE_HALF_DEPTH 0.34f
extern TetMeshBufferTemplate* gTractorTireBufferTemplate;
#endif

#if CARS_PANELS_TIRES_SCENE
#define NUM_TRACTOR_TIRE_STACKS 2
#define TRACTOR_TIRE_STACK_BASE 3
#define TRACTOR_TIRE_STACK_NUM_TIRES (2 * (TRACTOR_TIRE_STACK_BASE*(TRACTOR_TIRE_STACK_BASE + 1))/2)

#define NUM_TRACTOR_TIRES (TRACTOR_TIRE_STACK_NUM_TIRES*NUM_TRACTOR_TIRE_STACKS)

#define PROJECTILE_ID        0
#define CAR_PARTS_START_ID   1
#define WOOD_PANELS_START_ID (CAR_PARTS_START_ID + NUM_CARS_PER_INSTANCE*NUM_CAR_TEMPLATES*NUM_INSTANCES)
#define TRACTOR_TIRE_START_ID (WOOD_PANELS_START_ID + NUM_WOOD_PANELS_PER_INSTANCE*NUM_INSTANCES)
 
#define CAR_BODY_MESH_OFFSET       0
#define CAR_HOOD_MESH_OFFSET       1
#define CAR_WHEEL0_MESH_OFFSET     2
#define CAR_WHEEL1_MESH_OFFSET     3
#define CAR_WHEEL2_MESH_OFFSET     4
#define CAR_WHEEL3_MESH_OFFSET     5
#define CAR_SEATBACK_L_MESH_OFFSET 6
#define CAR_SEATBACK_R_MESH_OFFSET 7

void LaunchCarSimObject(AMD::uint carId, float speed);
void StartExplosionAtProjectile(float force);
void ApplyExplosion();
void ResetExplosion();
#endif

#define MAX_MESH_BUFFERS 1024
#define MAX_BUFFER_TETMESHES 128
#define MAX_MESHES (MAX_MESH_BUFFERS*MAX_BUFFER_TETMESHES)
#define MAX_RIGID_BODIES 256
#define MAX_VERTS_PER_MESH_BUFFER 4096
#define MAX_DISTANCE_CONTACTS (4096*16*2)
#define MAX_FRACTURE_CONTACTS (4096)
#define MAX_VOLUME_CONTACTS (4096*2)
#define MAX_VOLUME_CONTACT_VERTS (4096*64*2)
#define MAX_GLUE_CONSTRAINTS (4096)
#define MAX_ANGLE_CONSTRAINTS (4096)
#define MAX_CONTACTS (MAX_DISTANCE_CONTACTS + MAX_FRACTURE_CONTACTS + MAX_VOLUME_CONTACTS)
#define MAX_BROAD_PHASE_PAIRS (4096*2)

extern AMD::FmScene* gScene;
extern AMD::FmTetMeshBuffer* gTetMeshBuffers[MAX_MESH_BUFFERS];
extern AMD::uint gNumTetMeshBuffers;
extern AMD::FmRigidBody* gRigidBodies[MAX_RIGID_BODIES];
extern AMD::uint gNumRigidBodies;
extern AMD::uint gNumObjects;
extern bool gMaterialDemoEnablePlasticity;
extern bool gMaterialDemoEnableFracture;
extern AMD::FmTetMaterialParams gMaterialDemoMaterialParams;
extern bool gFired;

extern void FreeScene();
extern void InitScene(const char* modelsPath, const char* timingsPath, int numThreads, int randomSeed);
extern void FireProjectile(const AMD::FmMatrix3& viewRotation, const AMD::FmVector3& viewPosition, float speed, bool fixedPosForWoodPanels = false);

extern void TestSerialization();
