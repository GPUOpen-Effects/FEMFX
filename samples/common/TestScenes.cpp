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

#define _VECTORMATH_DEBUG
#include "TestScenes.h"
#include "AMD_FEMFX.h"
#include "LoadFemFile.h"
#include "LoadNodeEleMesh.h"
#include "PartitionFemMesh.h"
#include "ExplosionForce.h"
#include "SampleTaskSystem.h"
#include <stdarg.h>
#include <string>

using namespace AMD;

// Include condition number checks on created meshes
#define TEST_CONDITION_NUMBERS 0

// Required allocator definitions
void* FmAlignedMalloc(size_t size, size_t alignment)
{
    return _aligned_malloc(size, alignment);
}

void FmAlignedFree(void* ptr)
{
    _aligned_free(ptr);
}

// Debug print definition
int FmDebugPrint(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    return 0;
}

FILE* gTimingsFile = NULL;

static void WriteTimings()
{
#if FM_TIMINGS
    if (gTimingsFile)
    {
        fprintf(gTimingsFile, "%lf,", gFEMFXMeshComponentsTime);
        fprintf(gTimingsFile, "%lf,", gFEMFXImplicitStepTime);
        fprintf(gTimingsFile, "%lf,", gFEMFXBroadPhaseTime);
        fprintf(gTimingsFile, "%lf,", gFEMFXMeshContactsTime);
        fprintf(gTimingsFile, "%lf,", gFEMFXConstraintIslandsTime);
        fprintf(gTimingsFile, "%lf,", gFEMFXConstraintSolveTime);
        fprintf(gTimingsFile, "%lf\n", gFEMFXTotalStepTime);
    }
#endif
}

// Saved pointers to scene and object data

FmTetMeshBuffer* gTetMeshBuffers[MAX_MESH_BUFFERS] = { NULL };
uint             gNumTetMeshBuffers = 0;

FmRigidBody*     gRigidBodies[MAX_RIGID_BODIES] = { NULL };
uint             gNumRigidBodies = 0;


FmScene*         gScene = NULL;

uint             gProjectileMeshBufferId = 0;
#if WOOD_PANELS_SCENE
uint             gWoodPanelProjectileMeshBufferIds[NUM_WOOD_PANELS_ROWS] = { 0 };
#endif
bool             gFired = false;

#if EXTERNAL_RIGIDBODIES
ExampleRigidBodiesScene* gRbScene = NULL;
#endif

static float randfloat()
{
    return (float)rand() / RAND_MAX;
}

static float randfloat2()
{
    return randfloat() * 2.0f - 1.0f;
}

void GetBlockMeshCounts(uint* numVerts, uint* numTets, uint numCubesX, uint numCubesY, uint numCubesZ);
void CreateBlockMesh(FmArray<uint>* vertIncidentTets, uint numCubesX, uint numCubesY, uint numCubesZ);
void InitBlockVerts(FmVector3* vertRestPositions, FmTetVertIds* tetVertIds,
    uint numCubesX, uint numCubesY, uint numCubesZ,
    float cubeDimX, float cubeDimY, float cubeDimZ, float scale, bool randomize);

// Init TetMeshBufferTemplate from .node and .ele files
void InitTetMeshBufferTemplate(
    TetMeshBufferTemplate* bufferTemplate, 
    const FmTetMaterialParams& materialParams,
    uint collisionGroup,
    const char* modelsPath, const char* modelName,
    bool enablePlasticity, bool enableFracture, bool isKinematic)
{
    uint numVerts;
    uint numTets;

    FmVector3 position = FmInitVector3(0.0f);
    FmVector3 velocity = FmInitVector3(0.0f);
    FmMatrix3 rotation = FmMatrix3::identity();

    // Get number of vertices and tetrahedra in the model, and initialize vertex incident tets
    std::string nodeFile = std::string(modelsPath) + std::string(modelName) + std::string(".node");
    std::string eleFile = std::string(modelsPath) + std::string(modelName) + std::string(".ele");

    int loadRet = LoadNodeEleMeshNumVerts(nodeFile.c_str());
    if (loadRet < 0)
    {
        exit(-1);
    }
    numVerts = loadRet;

    bufferTemplate->vertIncidentTets = new FmArray<uint>[numVerts];

    loadRet = LoadNodeEleMeshNumTets(eleFile.c_str(), bufferTemplate->vertIncidentTets, numVerts);
    if (loadRet < 0)
    {
        exit(-1);
    }
    numTets = loadRet;

    // Initialize rest positions and tet vertex ids.
    bufferTemplate->vertRestPositions = new FmVector3[numVerts];
    bufferTemplate->tetVertIds = new FmTetVertIds[numTets];

    LoadNodeEleMeshData(nodeFile.c_str(), eleFile.c_str(), bufferTemplate->vertRestPositions, bufferTemplate->tetVertIds);

#if PARTITION_MESH
    FmArray<uint> vertPermutation, tetPermutation;
    SortByPartition(vertPermutation, tetPermutation, bufferTemplate->vertRestPositions, bufferTemplate->tetVertIds, bufferTemplate->vertIncidentTets, numVerts, numTets);
#if 0
    StoreNodeEleMeshData((nodeFile + std::string("2")).c_str(), (eleFile + std::string("2")).c_str(), bufferTemplate->vertRestPositions, bufferTemplate->tetVertIds, numVerts, numTets);
#endif
#endif

    bufferTemplate->fractureGroupCounts = new FmFractureGroupCounts[numTets];
    bufferTemplate->tetFractureGroupIds = new uint[numTets];

    FmTetMeshBufferBounds bounds;
    FmComputeTetMeshBufferBounds(
        &bounds,
        bufferTemplate->fractureGroupCounts,
        bufferTemplate->tetFractureGroupIds,
        bufferTemplate->vertIncidentTets, bufferTemplate->tetVertIds, NULL,
        numVerts, numTets, enableFracture);

    FmTetMeshBufferSetupParams& setupParams = bufferTemplate->setupParams;
    setupParams.numVerts = bounds.numVerts;
    setupParams.maxVerts = bounds.maxVerts;
    setupParams.numTets = bounds.numTets;
    setupParams.maxVertAdjacentVerts = bounds.maxVertAdjacentVerts;
    setupParams.numTets = bounds.numTets;
    setupParams.numVertIncidentTets = bounds.numVertIncidentTets;
    setupParams.maxExteriorFaces = bounds.maxExteriorFaces;
    setupParams.maxTetMeshes = bounds.maxTetMeshes;
    setupParams.collisionGroup = collisionGroup;
    setupParams.enablePlasticity = enablePlasticity;
    setupParams.enableFracture = enableFracture;
    setupParams.isKinematic = isKinematic;

    bufferTemplate->defaultMaterialParams = materialParams;

    bufferTemplate->tetsBvh = FmCreateBvh(bufferTemplate->setupParams.numTets);
    FmBuildRestMeshTetBvh(bufferTemplate->tetsBvh, bufferTemplate->vertRestPositions, bufferTemplate->tetVertIds, bufferTemplate->setupParams.numTets);
}

// Init block-shaped TetMeshBufferTemplate
void InitTetMeshBufferTemplate(
    TetMeshBufferTemplate* bufferTemplate,
    const FmTetMaterialParams& materialParams,
    uint collisionGroup,
    uint numCubesX, uint numCubesY, uint numCubesZ,
    float widthX, float widthY, float widthZ, bool randomize,
    bool enablePlasticity, bool enableFracture, bool isKinematic)
{
    // Get number of vertices and tetrahedra in the model, and initialize vertex incident tets

    uint numVerts, numTets;
    GetBlockMeshCounts(&numVerts, &numTets, numCubesX, numCubesY, numCubesZ);
    bufferTemplate->vertIncidentTets = new FmArray<uint>[numVerts];
    CreateBlockMesh(bufferTemplate->vertIncidentTets, numCubesX, numCubesY, numCubesZ);

    // Initialize rest positions and tet vertex ids.
    bufferTemplate->vertRestPositions = new FmVector3[numVerts];
    bufferTemplate->tetVertIds = new FmTetVertIds[numTets];

    InitBlockVerts(bufferTemplate->vertRestPositions, bufferTemplate->tetVertIds, numCubesX, numCubesY, numCubesZ,
        widthX / numCubesX,
        widthY / numCubesY,
        widthZ / numCubesZ, 1.0f, randomize);

    bufferTemplate->fractureGroupCounts = new FmFractureGroupCounts[numTets];
    bufferTemplate->tetFractureGroupIds = new uint[numTets];

    FmTetMeshBufferBounds bounds;
    FmComputeTetMeshBufferBounds(
        &bounds,
        bufferTemplate->fractureGroupCounts,
        bufferTemplate->tetFractureGroupIds,
        bufferTemplate->vertIncidentTets, bufferTemplate->tetVertIds, bufferTemplate->tetFlags,
        numVerts, numTets, enableFracture);

    FmTetMeshBufferSetupParams& setupParams = bufferTemplate->setupParams;
    setupParams.numVerts = bounds.numVerts;
    setupParams.maxVerts = bounds.maxVerts;
    setupParams.numTets = bounds.numTets;
    setupParams.maxVertAdjacentVerts = bounds.maxVertAdjacentVerts;
    setupParams.numTets = bounds.numTets;
    setupParams.numVertIncidentTets = bounds.numVertIncidentTets;
    setupParams.maxExteriorFaces = bounds.maxExteriorFaces;
    setupParams.maxTetMeshes = bounds.maxTetMeshes;
    setupParams.collisionGroup = collisionGroup;
    setupParams.enablePlasticity = enablePlasticity;
    setupParams.enableFracture = enableFracture;
    setupParams.isKinematic = isKinematic;

    bufferTemplate->defaultMaterialParams = materialParams;

    bufferTemplate->tetsBvh = FmCreateBvh(bufferTemplate->setupParams.numTets);
    FmBuildRestMeshTetBvh(bufferTemplate->tetsBvh, bufferTemplate->vertRestPositions, bufferTemplate->tetVertIds, bufferTemplate->setupParams.numTets);
}

#if LOAD_FEM_ASSET
FmTetMeshBuffer* CreateTetMeshBuffer(FComponentResources& componentResources, 
    const FmTetMaterialParams& defaultMaterial,
    const FmVector3& position, const FmMatrix3& rotation, const FmVector3& velocity, float massOverride,
    bool enablePlasticity, bool enableFracture, bool isKinematic,
    uint setKinematicVerts,  // 0 for none, 1 for base, 2 for top 
    float kinYThreshold)
{
    // Set up a single memory buffer to hold all tet mesh data

    FmArray<uint>* vertIncidentTets = new FmArray<uint>[componentResources.NumVerts];
    uint vitPos = 0;
    for (uint vidx = 0; vidx < (uint)componentResources.NumVerts; vidx++)
    {
        uint numTets = componentResources.vertIncidentTets[vitPos++];
        for (uint tidx = 0; tidx < numTets; tidx++)
        {
            vertIncidentTets[vidx].Add(componentResources.vertIncidentTets[vitPos++]);
        }
    }

    FmVector3* vertRestPositions = new FmVector3[componentResources.NumVerts];
    uint vertRestPosBytePos = 0;
    for (uint vidx = 0; vidx < (uint)componentResources.NumVerts; vidx++)
    {
        memcpy(&vertRestPositions[vidx], &componentResources.restPositions[vertRestPosBytePos], sizeof(float) * 3);
        vertRestPosBytePos += sizeof(float) * 3;
    }

    FmTetVertIds* tetVertIds = new FmTetVertIds[componentResources.NumTets];
    uint16_t* tetFlags = new uint16_t[componentResources.NumTets];
    uint tetIdBytePos = 0;
    for (uint tidx = 0; tidx < (uint)componentResources.NumTets; tidx++)
    {
        tetFlags[tidx] = 0;
        memcpy(&tetVertIds[tidx], &componentResources.tetVertIds[tetIdBytePos], sizeof(FmTetVertIds));
        tetIdBytePos += sizeof(FmTetVertIds);        
    }

    //SortByPartition(componentResources.VertPermutation, componentResources.TetPermutation,
    //    vertRestPositions,
    //    tetVertIds,
    //    vertIncidentTets, componentResources.NumVerts, componentResources.NumTets);

    for (int i = 0; i < (int)componentResources.Materials.size(); i++)
    {
        for (int j = 0; j < (int)componentResources.Materials[i].TetIds.size(); j++)
        {
            if (componentResources.Materials[i].NoFractureFaces.size() > 0)
            {
                int flags = componentResources.Materials[i].NoFractureFaces[j];
                uint tetId = componentResources.Materials[i].TetIds[j];

                tetFlags[tetId] |= flags;
            }
        }
    }

    FmFractureGroupCounts* fractureGroupCounts = new FmFractureGroupCounts[componentResources.NumTets];
    uint* tetFractureGroupIds = new uint[componentResources.NumTets];

    FmTetMeshBufferBounds bounds;
    FmComputeTetMeshBufferBounds(
        &bounds,
        fractureGroupCounts,
        tetFractureGroupIds,
        vertIncidentTets, tetVertIds, tetFlags,
        componentResources.NumVerts, componentResources.NumTets, enableFracture);

    FmTetMeshBufferSetupParams setupParams;
    setupParams.numVerts = bounds.numVerts;
    setupParams.maxVerts = bounds.maxVerts;
    setupParams.numTets = bounds.numTets;
    setupParams.maxVertAdjacentVerts = bounds.maxVertAdjacentVerts;
    setupParams.numTets = bounds.numTets;
    setupParams.numVertIncidentTets = bounds.numVertIncidentTets;
    setupParams.maxExteriorFaces = bounds.maxExteriorFaces;
    setupParams.maxTetMeshes = bounds.maxTetMeshes;
    setupParams.enablePlasticity = enablePlasticity;
    setupParams.enableFracture = enableFracture;
    setupParams.isKinematic = isKinematic;

    FmTetMesh* tetMeshPtr = NULL;
    FmTetMeshBuffer* tetMeshBuffer = FmCreateTetMeshBuffer(setupParams, fractureGroupCounts, tetFractureGroupIds, &tetMeshPtr);
    FmTetMesh& tetMesh = *tetMeshPtr;

    gTetMeshBuffers[gNumTetMeshBuffers] = tetMeshBuffer;
    gNumTetMeshBuffers++;

    FmInitVertState(&tetMesh, vertRestPositions, rotation, position, 1.0f, velocity);

    FmInitTetState(&tetMesh, tetVertIds, defaultMaterial);

    FmComputeMeshConstantMatrices(&tetMesh);

    FmSetMassesFromRestDensities(&tetMesh);

    FmInitConnectivity(&tetMesh, vertIncidentTets);

    FmFinishTetMeshInit(&tetMesh);

    uint maxIncident = 0;
    uint numVerts = FmGetNumVerts(tetMesh);
    uint numTets = FmGetNumTets(tetMesh);
    for (uint vId = 0; vId < numVerts; vId++)
    {
        if (vertIncidentTets[vId].GetNumElems() > maxIncident)
        {
            maxIncident = (uint)vertIncidentTets[vId].GetNumElems();
        }
    }
    printf("Num verts: %u Num tets: %u Max incident tets: %u\n", numVerts, numTets, maxIncident);

    for (int i = 0; i < (int)componentResources.Materials.size(); i++)
    {
        for (int j = 0; j < (int)componentResources.Materials[i].TetIds.size(); j++)
        {
            if (componentResources.Materials[i].NoFractureFaces.size() > 0)
            {
                uint16_t flags = (uint16_t)componentResources.Materials[i].NoFractureFaces[j];
                uint tetId = componentResources.Materials[i].TetIds[j];

                FmSetTetFlags(&tetMesh, tetId, flags);
            }
        }
    }

    if (massOverride > 0.0f)
    {
        FmSetTotalMass(&tetMesh, massOverride);
    }

#if TEST_CONDITION_NUMBERS
    AMD::FmSceneControlParams defaultParams;
    float meshCondition = AMD::FmCheckTetMeshCondition(&tetMesh, defaultParams);
    printf("Mesh condition number: %f\n", meshCondition);

    if (enableFracture)
    {
        float maxCondition = AMD::FmCheckMaxTetMeshCondition(tetMeshBuffer, defaultParams);
        printf("Max of fracture groups: %f\n", maxCondition);
    }
#endif

    if (setKinematicVerts)
    {
        if (setKinematicVerts == 1)
        {
            float minY = FmGetVertRestPosition(tetMesh, 0).y;

            for (uint i = 1; i < numVerts; i++)
            {
                if (FmGetVertRestPosition(tetMesh, i).y < minY)
                {
                    minY = FmGetVertRestPosition(tetMesh, i).y;
                }
            }

            for (uint i = 0; i < numVerts; i++)
            {
                if (FmGetVertRestPosition(tetMesh, i).y < minY + kinYThreshold)
                {
                    FmSetVertFlags(&tetMesh, i, FM_VERT_FLAG_KINEMATIC | FM_VERT_FLAG_KINEMATIC_REMOVABLE);
                }
            }
        }
        else
        {
            float maxY = FmGetVertRestPosition(tetMesh, 0).y;

            for (uint i = 1; i < numVerts; i++)
            {
                if (FmGetVertRestPosition(tetMesh, i).y > maxY)
                {
                    maxY = FmGetVertRestPosition(tetMesh, i).y;
                }
            }

            for (uint i = 0; i < numVerts; i++)
            {
                if (FmGetVertRestPosition(tetMesh, i).y > maxY - kinYThreshold)
                {
                    FmSetVertFlags(&tetMesh, i, FM_VERT_FLAG_KINEMATIC | FM_VERT_FLAG_KINEMATIC_REMOVABLE);
                }
            }
        }
    }

    delete[] fractureGroupCounts;
    delete[] tetFractureGroupIds;
    delete[] vertIncidentTets;
    delete[] vertRestPositions;
    delete[] tetVertIds;
    delete[] tetFlags;

    return tetMeshBuffer;

}
#endif

#if CARS_IN_SCENE
// Setup for car objects

struct CarSimObject
{
    static const uint numTetMeshBuffers = 2 + 4 + 2;  // body, hood, 4 wheels, 2 seatbacks
    static const uint numRigidBodies = 8;             // rim, axel attached to car
    //static const uint numBodyHoodGlueConstraints = 3;
    static const uint numBodyHoodGlueConstraints = 2;
    static const uint numRimCirclePoints = 6;
    static const uint numWheelRimGlueConstraints = numRimCirclePoints * 2 * 4;
    static const uint numBodyAxelGlueConstraints = 8 * 4;
    static const uint numRimAxelGlueConstraints = 4;
    static const uint numBodySeatGlueConstraints = 16;
    static const uint numGlueConstraints = numBodyHoodGlueConstraints + numWheelRimGlueConstraints + numBodyAxelGlueConstraints + numRimAxelGlueConstraints + numBodySeatGlueConstraints;
    static const uint numRimAxelAngleConstraints = 4;
    static const uint numPlaneConstraints = 1 + 10;

    // Tet mesh buffers to be added to scene
    FmTetMeshBuffer*            tetMeshBuffers[numTetMeshBuffers];
    uint                           tetMeshBufferIds[numTetMeshBuffers];

    // Will be copied into scene
    FmRigidBodySetupParams      rigidBodySetupParams[numRigidBodies];

    float hoodHingeMaxImpulseMag;
    float hoodLatchMaxImpulseMag;

    // Pointers to scene objects when added to scene
    FmRigidBody*  rigidBodies[numRigidBodies];
    uint          rigidBodyIds[numRigidBodies];
    uint          axelAngleConstraints[numRimAxelAngleConstraints];
    uint          bodyHoodGlueConstraints[numBodyHoodGlueConstraints];
    uint          wheelRimGlueConstraints[numWheelRimGlueConstraints];
    uint          bodyAxelGlueConstraints[numBodyAxelGlueConstraints];
    uint          rimAxelGlueConstraints[numRimAxelGlueConstraints];
    uint          bodySeatGlueConstraints[numBodySeatGlueConstraints];
    uint          planeConstraints[numPlaneConstraints];

    FmVector3 initialPosition;
    FmMatrix3 initialRotation;

    CarSimObject()
    {
        memset(this, 0, sizeof(CarSimObject));

        hoodHingeMaxImpulseMag = 0.0f;
        hoodLatchMaxImpulseMag = 0.0f;
    }
};

// Contains shared car geometry and parameters to set up car instance.
struct CarSimObjectTemplate
{
    TetMeshBufferTemplate tetMeshBufferTemplates[CarSimObject::numTetMeshBuffers];
    FmRigidBodySetupParams rigidBodySetupParams[CarSimObject::numRigidBodies];

    FmVector3 wheelMinPositions[4];
    FmVector3 wheelMaxPositions[4];
    FmVector3 wheelCenterPositions[4];
    FmVector3 wheelAnchorPositions[4];
    FmVector3 wheelAxelBodyPositions[4];
    FmVector3 wheelAxelBodyHalfDims;
};

void InitCarSimObjectTemplate(CarSimObjectTemplate* carTemplate, const char* modelsPath)
{
    // Load geometry and set parameters for FEM tet mesh buffers

    for (uint carSubIdx = 0; carSubIdx < CarSimObject::numTetMeshBuffers; carSubIdx++)
    {
        bool enablePlasticity = false;
        bool enableFracture = false;
        bool isKinematic = false;

        const char* fileName = NULL;

        if (carSubIdx == 0)
        {
            fileName = "car-body-tets";
            enablePlasticity = true;
        }
        else if (carSubIdx == 1)
        {
            fileName = "car-hood-tets";
            enablePlasticity = true;
        }
        else if (carSubIdx == 2)
        {
            fileName = "car-wheel0-tets";
        }
        else if (carSubIdx == 3)
        {
            fileName = "car-wheel1-tets";
        }
        else if (carSubIdx == 4)
        {
            fileName = "car-wheel2-tets";
        }
        else if (carSubIdx == 5)
        {
            fileName = "car-wheel3-tets";
        }
        else if (carSubIdx == 6)
        {
            fileName = "car-seatback-l-tets";
        }
        else if (carSubIdx == 7)
        {
            fileName = "car-seatback-r-tets";
        }

        uint collisionGroup = 0;
        FmTetMaterialParams materialParams;
        if (carSubIdx < 2 || carSubIdx >= 6)
        {
            float carBodyDensity = 1031.951904296875f;
            materialParams.restDensity = carBodyDensity;
            materialParams.poissonsRatio = 0.15f;
            materialParams.youngsModulus = 2.5e7f;
            materialParams.plasticCreep = 1.0f;
            materialParams.plasticYieldThreshold = 2.5e6f;
            materialParams.plasticMin = 0.5f;
            materialParams.plasticMax = 1.2f;
        }
        else
        {
            collisionGroup = 1;

            materialParams.restDensity = 1768.1546630859375f;
            //materialParams.poissonsRatio = 0.25f;
            materialParams.poissonsRatio = 0.15f;
            //materialParams.youngsModulus = 2e7f;
            materialParams.youngsModulus = 2e7f;
        }

        TetMeshBufferTemplate& meshBufferTemplate = carTemplate->tetMeshBufferTemplates[carSubIdx];

        InitTetMeshBufferTemplate(&meshBufferTemplate, materialParams, collisionGroup, modelsPath, fileName, enablePlasticity, enableFracture, isKinematic);

        FmVector3 minPos = meshBufferTemplate.vertRestPositions[0];
        FmVector3 maxPos = meshBufferTemplate.vertRestPositions[0];
        uint numVerts = meshBufferTemplate.setupParams.numVerts;
        for (uint vIdx = 0; vIdx < numVerts; vIdx++)
        {
            minPos = min(minPos, meshBufferTemplate.vertRestPositions[vIdx]);
            maxPos = max(maxPos, meshBufferTemplate.vertRestPositions[vIdx]);
        }

        if (carSubIdx >= 2 && carSubIdx < 6)
        {
            uint wheelIdx = carSubIdx - 2;
            carTemplate->wheelMinPositions[wheelIdx] = minPos;
            carTemplate->wheelMaxPositions[wheelIdx] = maxPos;
            carTemplate->wheelCenterPositions[wheelIdx] = (minPos + maxPos) * 0.5f;
            carTemplate->wheelAnchorPositions[wheelIdx] = carTemplate->wheelCenterPositions[wheelIdx];

            FmVector3 offset = FmInitVector3(0.0f, 0.0f, 0.5f);

            if (wheelIdx == 0 || wheelIdx == 2)
            {
                carTemplate->wheelAxelBodyPositions[wheelIdx] = carTemplate->wheelCenterPositions[wheelIdx] + offset;
                carTemplate->wheelAnchorPositions[wheelIdx].z = carTemplate->wheelMaxPositions[wheelIdx].z;
            }
            else
            {
                carTemplate->wheelAxelBodyPositions[wheelIdx] = carTemplate->wheelCenterPositions[wheelIdx] - offset;
                carTemplate->wheelAnchorPositions[wheelIdx].z = carTemplate->wheelMinPositions[wheelIdx].z;
            }
        }
    }

    // Init rigid bodies
    carTemplate->wheelAxelBodyHalfDims = FmInitVector3(0.2f, 0.2f, 0.25f);

    for (uint rbIdx = 0; rbIdx < CarSimObject::numRigidBodies; rbIdx++)
    {
        uint wheelIdx = rbIdx / 2;
        uint wheelSubIdx = rbIdx % 2;

        float hx, hy, hz;

        FmRigidBodyState rigidBodyState;

        if (wheelSubIdx == 0)
        {
            rigidBodyState.pos = carTemplate->wheelCenterPositions[wheelIdx];
            rigidBodyState.quat = FmInitQuat(0.0f, 0.0f, 0.0f, 1.0f);
            rigidBodyState.vel = FmInitVector3(0.0f, 0.0f, 0.0f);
            rigidBodyState.angVel = FmInitVector3(0.0f);

            hx = 0.15f;
            hy = 0.15f;
            hz = (carTemplate->wheelMaxPositions[wheelIdx].z - carTemplate->wheelMinPositions[wheelIdx].z) * 0.5f;
        }
        else
        {
            rigidBodyState.pos = carTemplate->wheelAxelBodyPositions[wheelIdx];
            rigidBodyState.quat = FmInitQuat(0.0f, 0.0f, 0.0f, 1.0f);
            rigidBodyState.vel = FmInitVector3(0.0f, 0.0f, 0.0f);
            rigidBodyState.angVel = FmInitVector3(0.0f);

            hx = carTemplate->wheelAxelBodyHalfDims.x;
            hy = carTemplate->wheelAxelBodyHalfDims.y;
            hz = carTemplate->wheelAxelBodyHalfDims.z;
        }


        float mass = 200.0f;

        FmRigidBodySetupParams& rigidBodySetupParams = carTemplate->rigidBodySetupParams[rbIdx];
        rigidBodySetupParams.state = rigidBodyState;
        rigidBodySetupParams.halfDimX = hx;
        rigidBodySetupParams.halfDimY = hy;
        rigidBodySetupParams.halfDimZ = hz;
        rigidBodySetupParams.mass = mass;
        rigidBodySetupParams.collisionGroup = 2;
        rigidBodySetupParams.isKinematic = false;
        rigidBodySetupParams.bodyInertiaTensor = FmComputeBodyInertiaTensorForBox(hx, hy, hz, mass);
    }
}

void FreeCarSimObjectTemplate(CarSimObjectTemplate* carTemplate)
{
    for (uint i = 0; i < CarSimObject::numTetMeshBuffers; i++)
    {
        carTemplate->tetMeshBufferTemplates[i].Destroy();
    }
}

void CreateCarSimObject(CarSimObject* car, CarSimObjectTemplate& carTemplate, const FmVector3& position, const FmMatrix3& rotation, const FmVector3& velocity)
{
    car->initialPosition = position;
    car->initialRotation = rotation;

    for (uint carSubIdx = 0; carSubIdx < CarSimObject::numTetMeshBuffers; carSubIdx++)
    {
        TetMeshBufferTemplate& meshBufferTemplate = carTemplate.tetMeshBufferTemplates[carSubIdx];

        uint numVerts = meshBufferTemplate.setupParams.numVerts;

        FmTetMesh* tetMeshPtr;
        FmTetMeshBuffer* tetMeshBuffer = FmCreateTetMeshBuffer(meshBufferTemplate.setupParams, meshBufferTemplate.fractureGroupCounts, meshBufferTemplate.tetFractureGroupIds, &tetMeshPtr);

        gTetMeshBuffers[gNumTetMeshBuffers] = tetMeshBuffer;
        gNumTetMeshBuffers++;

        FmTetMesh& tetMesh = *tetMeshPtr;

        FmVector3 minPos = meshBufferTemplate.vertRestPositions[0];
        FmVector3 maxPos = meshBufferTemplate.vertRestPositions[0];
        for (uint vIdx = 0; vIdx < numVerts; vIdx++)
        {
            minPos = min(minPos, meshBufferTemplate.vertRestPositions[vIdx]);
            maxPos = max(maxPos, meshBufferTemplate.vertRestPositions[vIdx]);
        }

        FmInitVertState(&tetMesh, meshBufferTemplate.vertRestPositions, rotation, position, 1.0f, velocity);

        if (carSubIdx < 2)
        {
            FmSetExternalForceSpeedLimit(&tetMesh, 300.0f);
        }

        FmInitTetState(&tetMesh, meshBufferTemplate.tetVertIds, meshBufferTemplate.defaultMaterialParams, 0.6f);

        FmComputeMeshConstantMatrices(&tetMesh);

        FmSetMassesFromRestDensities(&tetMesh);

        FmInitConnectivity(&tetMesh, meshBufferTemplate.vertIncidentTets);

        FmFinishTetMeshInit(&tetMesh);

#if TEST_CONDITION_NUMBERS
        AMD::FmSceneControlParams defaultParams;
        float meshCondition = AMD::FmCheckMaxTetMeshCondition(tetMeshBuffer, defaultParams);
        printf("Mesh condition number: %f\n", meshCondition);
#endif

        car->tetMeshBuffers[carSubIdx] = tetMeshBuffer;
    }

    FmQuat quat = FmQuat(rotation);

    for (uint rbIdx = 0; rbIdx < CarSimObject::numRigidBodies; rbIdx++)
    {
        FmRigidBodySetupParams& rigidBodySetupParams = car->rigidBodySetupParams[rbIdx];

        rigidBodySetupParams = carTemplate.rigidBodySetupParams[rbIdx];
        rigidBodySetupParams.state.pos = position + mul(rotation, rigidBodySetupParams.state.pos);
        rigidBodySetupParams.state.vel = velocity;
        rigidBodySetupParams.state.quat = mul(quat, rigidBodySetupParams.state.quat);
    }
}

void AddCarSimObjectToScene(CarSimObject* car, const CarSimObjectTemplate& carTemplate, FmScene* scene)
{
    for (uint meshBufferIdx = 0; meshBufferIdx < CarSimObject::numTetMeshBuffers; meshBufferIdx++)
    {
        uint bufferId = FmAddTetMeshBufferToScene(scene, car->tetMeshBuffers[meshBufferIdx]);
        car->tetMeshBufferIds[meshBufferIdx] = bufferId;
        FM_ASSERT(bufferId != FM_INVALID_ID);
    }

    for (uint rbIdx = 0; rbIdx < CarSimObject::numRigidBodies; rbIdx++)
    {
        FmRigidBody* rigidBody = FmCreateRigidBody(car->rigidBodySetupParams[rbIdx]);

        car->rigidBodies[rbIdx] = rigidBody;

        car->rigidBodyIds[rbIdx] = FmAddRigidBodyToScene(gScene, rigidBody);

        gRigidBodies[gNumRigidBodies] = rigidBody;
        gNumRigidBodies++;
    }

    // Add constraints holding components together

    // First find points on hood
    FmClosestTetResult bodyHoodGluePoints[3];
    FmClosestTetResult hoodGluePoints[3];
    FmVector4 bodyHoodGlueBarycentrics[3];
    FmVector4 hoodHoodGlueBarycentrics[3];
    FmClosestTetResult wheelAxelPoints[4];
    FmClosestTetResult bodyAxelPoints[4];
    FmVector4 wheelAxelBarycentrics[4];
    FmVector4 bodyAxelBarycentrics[4];

    FmClosestTetResult wheelAxel0Points[4];
    FmClosestTetResult wheelAxel1Points[4];
    FmClosestTetResult wheelAxel2Points[4];
    FmVector4 wheelAxel0Barycentrics[4];
    FmVector4 wheelAxel1Barycentrics[4];
    FmVector4 wheelAxel2Barycentrics[4];

    FmClosestTetResult seatbackLGluePoints[4];
    FmClosestTetResult seatbackRGluePoints[4];
    FmClosestTetResult bodySeatbackLGluePoints[4];
    FmClosestTetResult bodySeatbackRGluePoints[4];
    FmVector4 bodySeatbackLBarycentrics[4];
    FmVector4 bodySeatbackRbarycentrics[4];

    uint* tetsIntersected = NULL;

    uint bodyBufferId = car->tetMeshBufferIds[0];
    uint hoodBufferId = car->tetMeshBufferIds[1];
    uint seatbackLBufferId = car->tetMeshBufferIds[6];
    uint seatbackRbufferId = car->tetMeshBufferIds[7];
    uint wheelBufferIds[4];
    wheelBufferIds[0] = car->tetMeshBufferIds[2];
    wheelBufferIds[1] = car->tetMeshBufferIds[3];
    wheelBufferIds[2] = car->tetMeshBufferIds[4];
    wheelBufferIds[3] = car->tetMeshBufferIds[5];

    FmTetMesh& carBodyMesh = *FmGetTetMesh(*car->tetMeshBuffers[0], 0);
    FmTetMesh& carHoodMesh = *FmGetTetMesh(*car->tetMeshBuffers[1], 0);
    FmTetMesh* carWheelMeshes[4];
    carWheelMeshes[0] = FmGetTetMesh(*car->tetMeshBuffers[2], 0);
    carWheelMeshes[1] = FmGetTetMesh(*car->tetMeshBuffers[3], 0);
    carWheelMeshes[2] = FmGetTetMesh(*car->tetMeshBuffers[4], 0);
    carWheelMeshes[3] = FmGetTetMesh(*car->tetMeshBuffers[5], 0);

    const TetMeshBufferTemplate& bodyTemplate = carTemplate.tetMeshBufferTemplates[0];
    const TetMeshBufferTemplate& hoodTemplate = carTemplate.tetMeshBufferTemplates[1];
    const TetMeshBufferTemplate* wheelTemplates[4];
    wheelTemplates[0] = &carTemplate.tetMeshBufferTemplates[2];
    wheelTemplates[1] = &carTemplate.tetMeshBufferTemplates[3];
    wheelTemplates[2] = &carTemplate.tetMeshBufferTemplates[4];
    wheelTemplates[3] = &carTemplate.tetMeshBufferTemplates[5];
    const TetMeshBufferTemplate& seatbackLTemplate = carTemplate.tetMeshBufferTemplates[6];
    const TetMeshBufferTemplate& seatbackRTemplate = carTemplate.tetMeshBufferTemplates[7];

    uint numTets = FmGetNumTets(carBodyMesh);
    tetsIntersected = new uint[numTets];

    uint rimRigidBodyIds[4];
    uint axelRigidBodyIds[4];
    rimRigidBodyIds[0] = car->rigidBodyIds[0];
    rimRigidBodyIds[1] = car->rigidBodyIds[2];
    rimRigidBodyIds[2] = car->rigidBodyIds[4];
    rimRigidBodyIds[3] = car->rigidBodyIds[6];
    uint axelRigidBodyIdx[4];
    axelRigidBodyIdx[0] = 1;
    axelRigidBodyIdx[1] = 3;
    axelRigidBodyIdx[2] = 5;
    axelRigidBodyIdx[3] = 7;
    axelRigidBodyIds[0] = car->rigidBodyIds[1];
    axelRigidBodyIds[1] = car->rigidBodyIds[3];
    axelRigidBodyIds[2] = car->rigidBodyIds[5];
    axelRigidBodyIds[3] = car->rigidBodyIds[7];

    uint wheelRimIdx = 0;
    uint bodyAxelIdx = 0;
    for (uint wheelIdx = 0; wheelIdx < 4; wheelIdx++)
    {
        // Create rigid body hinge joint
        FmRigidBodyAngleConstraintSetupParams angleConstraint;
        angleConstraint.kVelCorrection = 1.0f;
        angleConstraint.kPosCorrection = 1.0f;
        angleConstraint.frictionCoeff = 0.6f;

        angleConstraint.objectIdA = rimRigidBodyIds[wheelIdx];
        angleConstraint.objectIdB = axelRigidBodyIds[wheelIdx];

        FmRigidBodySetupParams& axelRigidBodyParams = car->rigidBodySetupParams[axelRigidBodyIdx[wheelIdx]];

        angleConstraint.axisBodySpaceA = FmInitVector3(0.0f, 0.0f, 1.0f);
        angleConstraint.axisBodySpaceB = FmInitVector3(0.0f, 0.0f, 1.0f);

        uint angleConstraintId = FmAddRigidBodyAngleConstraintToScene(scene, angleConstraint);
        car->axelAngleConstraints[wheelIdx] = angleConstraintId;

        FmGlueConstraintSetupParams glueConstraint;

        // Glue rim to axel
        glueConstraint.kVelCorrection = 1.0f;
        glueConstraint.kPosCorrection = 1.0f;

        glueConstraint.bufferIdA = rimRigidBodyIds[wheelIdx];
        glueConstraint.bufferIdB = axelRigidBodyIds[wheelIdx];

        FmVector3 posBodySpaceA = carTemplate.wheelAnchorPositions[wheelIdx] - carTemplate.wheelCenterPositions[wheelIdx];
        glueConstraint.posBodySpaceA[0] = posBodySpaceA.x;
        glueConstraint.posBodySpaceA[1] = posBodySpaceA.y;
        glueConstraint.posBodySpaceA[2] = posBodySpaceA.z;
        glueConstraint.posBodySpaceA[3] = 1.0f;

        FmVector3 posBodySpaceB = carTemplate.wheelAnchorPositions[wheelIdx] - carTemplate.wheelAxelBodyPositions[wheelIdx];
        glueConstraint.posBodySpaceB[0] = posBodySpaceB.x;
        glueConstraint.posBodySpaceB[1] = posBodySpaceB.y;
        glueConstraint.posBodySpaceB[2] = posBodySpaceB.z;
        glueConstraint.posBodySpaceB[3] = 1.0f;

        uint glueId = FmAddGlueConstraintToScene(scene, glueConstraint);
        car->rimAxelGlueConstraints[wheelIdx] = glueId;

        // Glue wheel to rim
        glueConstraint.bufferIdA = wheelBufferIds[wheelIdx];
        glueConstraint.bufferIdB = rimRigidBodyIds[wheelIdx];

        const float radius = 0.25f;
        const int numCirclePnts = CarSimObject::numRimCirclePoints;
        for (uint pntIdx = 0; pntIdx < numCirclePnts; pntIdx++)
        {
            float angle = ((float)pntIdx / (float)numCirclePnts) * 2.0f * 3.14159265f;
            FmVector3 rimGlueCenterPos[2];
            FmVector3 rimGlueRestPos[2];

            rimGlueCenterPos[0] = carTemplate.wheelCenterPositions[wheelIdx];
            rimGlueCenterPos[0].z = carTemplate.wheelMinPositions[wheelIdx].z;

            rimGlueCenterPos[1] = carTemplate.wheelCenterPositions[wheelIdx];
            rimGlueCenterPos[1].z = carTemplate.wheelMaxPositions[wheelIdx].z;

            rimGlueRestPos[0] = rimGlueCenterPos[0] + FmInitVector3(cosf(angle), sinf(angle), 0.0f) * radius;
            rimGlueRestPos[1] = rimGlueCenterPos[1] + FmInitVector3(cosf(angle), sinf(angle), 0.0f) * radius;

            for (uint subIdx = 0; subIdx < 2; subIdx++)
            {
                FmClosestTetResult result;
                FmFindClosestTet(&result, wheelTemplates[wheelIdx]->vertRestPositions, wheelTemplates[wheelIdx]->tetVertIds, wheelTemplates[wheelIdx]->tetsBvh, rimGlueRestPos[subIdx]);

                glueConstraint.bufferTetIdA = result.tetId;
                glueConstraint.bufferTetIdB = FM_INVALID_ID;

                glueConstraint.posBaryA[0] = result.posBary[0];
                glueConstraint.posBaryA[1] = result.posBary[1];
                glueConstraint.posBaryA[2] = result.posBary[2];
                glueConstraint.posBaryA[3] = result.posBary[3];

                FmVector3 posBoxSpace = result.position - carTemplate.wheelCenterPositions[wheelIdx];
                glueConstraint.posBodySpaceB[0] = posBoxSpace.x;
                glueConstraint.posBodySpaceB[1] = posBoxSpace.y;
                glueConstraint.posBodySpaceB[2] = posBoxSpace.z;
                glueConstraint.posBodySpaceB[3] = 1.0f;

                posBodySpaceB = FmInitVector3(glueConstraint.posBodySpaceB[0], glueConstraint.posBodySpaceB[1], glueConstraint.posBodySpaceB[2]);

                glueId = FmAddGlueConstraintToScene(scene, glueConstraint);
                car->wheelRimGlueConstraints[wheelRimIdx] = glueId;
                wheelRimIdx++;
            }
        }

        // Glue rigid bodies to car
        glueConstraint.bufferIdA = bodyBufferId;
        glueConstraint.bufferIdB = axelRigidBodyIds[wheelIdx];

        uint numTetsIntersected = FmFindTetsIntersectingBox(tetsIntersected, bodyTemplate.vertRestPositions, bodyTemplate.tetVertIds, bodyTemplate.tetsBvh, carTemplate.wheelAxelBodyHalfDims, carTemplate.wheelAxelBodyPositions[wheelIdx], FmMatrix3::identity());
        for (uint itetId = 0; itetId < numTetsIntersected; itetId++)
        {
            FmTetMaterialParams tetMaterialParams = FmGetTetMaterialParams(carBodyMesh, tetsIntersected[itetId]);
            FmUpdateTetMaterialParams(gScene, &carBodyMesh, tetsIntersected[itetId], tetMaterialParams);
            FmAddTetFlags(&carBodyMesh, tetsIntersected[itetId], FM_TET_FLAG_PLASTICITY_DISABLED);
        }

        for (uint boxCornerIdx = 0; boxCornerIdx < 8; boxCornerIdx++)
        {
            FmVector3 boxRelPos = FmInitVector3(
                (boxCornerIdx & 0x1) ? -axelRigidBodyParams.halfDimX : axelRigidBodyParams.halfDimX,
                (boxCornerIdx & 0x2) ? -axelRigidBodyParams.halfDimY : axelRigidBodyParams.halfDimY,
                (boxCornerIdx & 0x4) ? -axelRigidBodyParams.halfDimZ : axelRigidBodyParams.halfDimZ);

            FmVector3 boxCornerRestPos = carTemplate.wheelAxelBodyPositions[wheelIdx] + boxRelPos;

            FmClosestTetResult result;
            FmFindClosestTet(&result, bodyTemplate.vertRestPositions, bodyTemplate.tetVertIds, bodyTemplate.tetsBvh, boxCornerRestPos);

            glueConstraint.bufferTetIdA = result.tetId;
            glueConstraint.bufferTetIdB = FM_INVALID_ID;

            glueConstraint.posBaryA[0] = result.posBary[0];
            glueConstraint.posBaryA[1] = result.posBary[1];
            glueConstraint.posBaryA[2] = result.posBary[2];
            glueConstraint.posBaryA[3] = result.posBary[3];

            FmVector3 posBoxSpace = result.position - carTemplate.wheelAxelBodyPositions[wheelIdx];
            glueConstraint.posBodySpaceB[0] = posBoxSpace.x;
            glueConstraint.posBodySpaceB[1] = posBoxSpace.y;
            glueConstraint.posBodySpaceB[2] = posBoxSpace.z;
            glueConstraint.posBodySpaceB[3] = 1.0f;

            posBodySpaceB = FmInitVector3(glueConstraint.posBodySpaceB[0], glueConstraint.posBodySpaceB[1], glueConstraint.posBodySpaceB[2]);

            glueId = FmAddGlueConstraintToScene(scene, glueConstraint);
            car->bodyAxelGlueConstraints[bodyAxelIdx] = glueId;
            bodyAxelIdx++;
        }
    }

    // Find points of attachment for hood on car
    FmFindClosestTet(&hoodGluePoints[0], hoodTemplate.vertRestPositions, hoodTemplate.tetVertIds, hoodTemplate.tetsBvh, FmInitVector3(0.0f, 2.0f, 1.0f));
    FmFindClosestTet(&hoodGluePoints[1], hoodTemplate.vertRestPositions, hoodTemplate.tetVertIds, hoodTemplate.tetsBvh, FmInitVector3(0.0f, 2.0f, -1.0f));
    FmFindClosestTet(&hoodGluePoints[2], hoodTemplate.vertRestPositions, hoodTemplate.tetVertIds, hoodTemplate.tetsBvh, FmInitVector3(4.0f, 0.0f, 0.0f));

    // Then find nearest tets on body for attachement
    FmFindClosestTet(&bodyHoodGluePoints[0], bodyTemplate.vertRestPositions, bodyTemplate.tetVertIds, bodyTemplate.tetsBvh, hoodGluePoints[0].position);
    FmFindClosestTet(&bodyHoodGluePoints[1], bodyTemplate.vertRestPositions, bodyTemplate.tetVertIds, bodyTemplate.tetsBvh, hoodGluePoints[1].position);
    FmFindClosestTet(&bodyHoodGluePoints[2], bodyTemplate.vertRestPositions, bodyTemplate.tetVertIds, bodyTemplate.tetsBvh, hoodGluePoints[2].position);

    // Compute barycentric coords
    hoodHoodGlueBarycentrics[0] = mul(FmGetTetRestBaryMatrix(carHoodMesh, hoodGluePoints[0].tetId), FmVector4(hoodGluePoints[0].position, 1.0f));
    hoodHoodGlueBarycentrics[1] = mul(FmGetTetRestBaryMatrix(carHoodMesh, hoodGluePoints[1].tetId), FmVector4(hoodGluePoints[1].position, 1.0f));
    hoodHoodGlueBarycentrics[2] = mul(FmGetTetRestBaryMatrix(carHoodMesh, hoodGluePoints[2].tetId), FmVector4(hoodGluePoints[2].position, 1.0f));
    bodyHoodGlueBarycentrics[0] = mul(FmGetTetRestBaryMatrix(carBodyMesh, bodyHoodGluePoints[0].tetId), FmVector4(hoodGluePoints[0].position, 1.0f));
    bodyHoodGlueBarycentrics[1] = mul(FmGetTetRestBaryMatrix(carBodyMesh, bodyHoodGluePoints[1].tetId), FmVector4(hoodGluePoints[1].position, 1.0f));
    bodyHoodGlueBarycentrics[2] = mul(FmGetTetRestBaryMatrix(carBodyMesh, bodyHoodGluePoints[2].tetId), FmVector4(hoodGluePoints[2].position, 1.0f));

    // Find glue points of seatback meshes
    FmFindClosestTet(&seatbackLGluePoints[0], seatbackLTemplate.vertRestPositions, seatbackLTemplate.tetVertIds, seatbackLTemplate.tetsBvh, FmInitVector3(-10.0f, -10.0f, -10.0f));
    FmFindClosestTet(&seatbackLGluePoints[1], seatbackLTemplate.vertRestPositions, seatbackLTemplate.tetVertIds, seatbackLTemplate.tetsBvh, FmInitVector3(-10.0f, -10.0f, 10.0f));
    FmFindClosestTet(&seatbackLGluePoints[2], seatbackLTemplate.vertRestPositions, seatbackLTemplate.tetVertIds, seatbackLTemplate.tetsBvh, FmInitVector3(10.0f, -10.0f, -10.0f));
    FmFindClosestTet(&seatbackLGluePoints[3], seatbackLTemplate.vertRestPositions, seatbackLTemplate.tetVertIds, seatbackLTemplate.tetsBvh, FmInitVector3(10.0f, -10.0f, 10.0f));
    FmFindClosestTet(&seatbackRGluePoints[0], seatbackRTemplate.vertRestPositions, seatbackRTemplate.tetVertIds, seatbackRTemplate.tetsBvh, FmInitVector3(-10.0f, -10.0f, -10.0f));
    FmFindClosestTet(&seatbackRGluePoints[1], seatbackRTemplate.vertRestPositions, seatbackRTemplate.tetVertIds, seatbackRTemplate.tetsBvh, FmInitVector3(-10.0f, -10.0f, 10.0f));
    FmFindClosestTet(&seatbackRGluePoints[2], seatbackRTemplate.vertRestPositions, seatbackRTemplate.tetVertIds, seatbackRTemplate.tetsBvh, FmInitVector3(10.0f, -10.0f, -10.0f));
    FmFindClosestTet(&seatbackRGluePoints[3], seatbackRTemplate.vertRestPositions, seatbackRTemplate.tetVertIds, seatbackRTemplate.tetsBvh, FmInitVector3(10.0f, -10.0f, 10.0f));

    // Find corresponding points on body
    FmFindClosestTet(&bodySeatbackLGluePoints[0], bodyTemplate.vertRestPositions, bodyTemplate.tetVertIds, bodyTemplate.tetsBvh, seatbackLGluePoints[0].position);
    FmFindClosestTet(&bodySeatbackLGluePoints[1], bodyTemplate.vertRestPositions, bodyTemplate.tetVertIds, bodyTemplate.tetsBvh, seatbackLGluePoints[1].position);
    FmFindClosestTet(&bodySeatbackLGluePoints[2], bodyTemplate.vertRestPositions, bodyTemplate.tetVertIds, bodyTemplate.tetsBvh, seatbackLGluePoints[2].position);
    FmFindClosestTet(&bodySeatbackLGluePoints[3], bodyTemplate.vertRestPositions, bodyTemplate.tetVertIds, bodyTemplate.tetsBvh, seatbackLGluePoints[2].position);
    bodySeatbackLBarycentrics[0] = mul(FmGetTetRestBaryMatrix(carBodyMesh, bodySeatbackLGluePoints[0].tetId), FmVector4(seatbackLGluePoints[0].position, 1.0f));
    bodySeatbackLBarycentrics[1] = mul(FmGetTetRestBaryMatrix(carBodyMesh, bodySeatbackLGluePoints[1].tetId), FmVector4(seatbackLGluePoints[1].position, 1.0f));
    bodySeatbackLBarycentrics[2] = mul(FmGetTetRestBaryMatrix(carBodyMesh, bodySeatbackLGluePoints[2].tetId), FmVector4(seatbackLGluePoints[2].position, 1.0f));
    bodySeatbackLBarycentrics[3] = mul(FmGetTetRestBaryMatrix(carBodyMesh, bodySeatbackLGluePoints[3].tetId), FmVector4(seatbackLGluePoints[3].position, 1.0f));

    FmFindClosestTet(&bodySeatbackRGluePoints[0], bodyTemplate.vertRestPositions, bodyTemplate.tetVertIds, bodyTemplate.tetsBvh, seatbackRGluePoints[0].position);
    FmFindClosestTet(&bodySeatbackRGluePoints[1], bodyTemplate.vertRestPositions, bodyTemplate.tetVertIds, bodyTemplate.tetsBvh, seatbackRGluePoints[1].position);
    FmFindClosestTet(&bodySeatbackRGluePoints[2], bodyTemplate.vertRestPositions, bodyTemplate.tetVertIds, bodyTemplate.tetsBvh, seatbackRGluePoints[2].position);
    FmFindClosestTet(&bodySeatbackRGluePoints[3], bodyTemplate.vertRestPositions, bodyTemplate.tetVertIds, bodyTemplate.tetsBvh, seatbackRGluePoints[2].position);
    bodySeatbackRbarycentrics[0] = mul(FmGetTetRestBaryMatrix(carBodyMesh, bodySeatbackRGluePoints[0].tetId), FmVector4(seatbackRGluePoints[0].position, 1.0f));
    bodySeatbackRbarycentrics[1] = mul(FmGetTetRestBaryMatrix(carBodyMesh, bodySeatbackRGluePoints[1].tetId), FmVector4(seatbackRGluePoints[1].position, 1.0f));
    bodySeatbackRbarycentrics[2] = mul(FmGetTetRestBaryMatrix(carBodyMesh, bodySeatbackRGluePoints[2].tetId), FmVector4(seatbackRGluePoints[2].position, 1.0f));
    bodySeatbackRbarycentrics[3] = mul(FmGetTetRestBaryMatrix(carBodyMesh, bodySeatbackRGluePoints[3].tetId), FmVector4(seatbackRGluePoints[3].position, 1.0f));

    car->hoodHingeMaxImpulseMag = 10000.0f;
    car->hoodLatchMaxImpulseMag = 5000.0f;

    // Glue hood to car
    for (uint hoodGlueIdx = 0; hoodGlueIdx < CarSimObject::numBodyHoodGlueConstraints; hoodGlueIdx++)
    {
        FmGlueConstraintSetupParams glueConstraint;

        glueConstraint.bufferIdA = bodyBufferId;
        glueConstraint.bufferIdB = hoodBufferId;

        glueConstraint.bufferTetIdA = bodyHoodGluePoints[hoodGlueIdx].tetId;
        glueConstraint.bufferTetIdB = hoodGluePoints[hoodGlueIdx].tetId;

        glueConstraint.posBaryA[0] = bodyHoodGlueBarycentrics[hoodGlueIdx].x;
        glueConstraint.posBaryA[1] = bodyHoodGlueBarycentrics[hoodGlueIdx].y;
        glueConstraint.posBaryA[2] = bodyHoodGlueBarycentrics[hoodGlueIdx].z;
        glueConstraint.posBaryA[3] = bodyHoodGlueBarycentrics[hoodGlueIdx].w;
        glueConstraint.posBaryB[0] = hoodHoodGlueBarycentrics[hoodGlueIdx].x;
        glueConstraint.posBaryB[1] = hoodHoodGlueBarycentrics[hoodGlueIdx].y;
        glueConstraint.posBaryB[2] = hoodHoodGlueBarycentrics[hoodGlueIdx].z;
        glueConstraint.posBaryB[3] = hoodHoodGlueBarycentrics[hoodGlueIdx].w;

        glueConstraint.breakThreshold = car->hoodHingeMaxImpulseMag;

        uint glueId = FmAddGlueConstraintToScene(scene, glueConstraint);
        car->bodyHoodGlueConstraints[hoodGlueIdx] = glueId;

        if (hoodGlueIdx < 2)
        {
            FmTetMaterialParams tetMaterialParams = FmGetTetMaterialParams(carHoodMesh, hoodGluePoints[hoodGlueIdx].tetId);
            FmUpdateTetMaterialParams(gScene, &carHoodMesh, hoodGluePoints[hoodGlueIdx].tetId, tetMaterialParams);
            FmAddTetFlags(&carHoodMesh, hoodGluePoints[hoodGlueIdx].tetId, FM_TET_FLAG_PLASTICITY_DISABLED);
        }
    }

    // Glue seats to car
    uint bodySeatIdx = 0;
    for (uint seatGlueIdx = 0; seatGlueIdx < 4; seatGlueIdx++)
    {
        FmGlueConstraintSetupParams glueConstraint;

        glueConstraint.bufferIdA = bodyBufferId;
        glueConstraint.bufferIdB = seatbackLBufferId;

        glueConstraint.bufferTetIdA = bodySeatbackLGluePoints[seatGlueIdx].tetId;
        glueConstraint.bufferTetIdB = seatbackLGluePoints[seatGlueIdx].tetId;

        glueConstraint.posBaryA[0] = bodySeatbackLBarycentrics[seatGlueIdx].x;
        glueConstraint.posBaryA[1] = bodySeatbackLBarycentrics[seatGlueIdx].y;
        glueConstraint.posBaryA[2] = bodySeatbackLBarycentrics[seatGlueIdx].z;
        glueConstraint.posBaryA[3] = bodySeatbackLBarycentrics[seatGlueIdx].w;
        glueConstraint.posBaryB[0] = seatbackLGluePoints[seatGlueIdx].posBary[0];
        glueConstraint.posBaryB[1] = seatbackLGluePoints[seatGlueIdx].posBary[1];
        glueConstraint.posBaryB[2] = seatbackLGluePoints[seatGlueIdx].posBary[2];
        glueConstraint.posBaryB[3] = seatbackLGluePoints[seatGlueIdx].posBary[3];

        uint glueId = FmAddGlueConstraintToScene(scene, glueConstraint);
        car->bodySeatGlueConstraints[bodySeatIdx] = glueId;
        bodySeatIdx++;

        glueConstraint.bufferIdA = bodyBufferId;
        glueConstraint.bufferIdB = seatbackRbufferId;

        glueConstraint.bufferTetIdA = bodySeatbackRGluePoints[seatGlueIdx].tetId;
        glueConstraint.bufferTetIdB = seatbackRGluePoints[seatGlueIdx].tetId;

        glueConstraint.posBaryA[0] = bodySeatbackRbarycentrics[seatGlueIdx].x;
        glueConstraint.posBaryA[1] = bodySeatbackRbarycentrics[seatGlueIdx].y;
        glueConstraint.posBaryA[2] = bodySeatbackRbarycentrics[seatGlueIdx].z;
        glueConstraint.posBaryA[3] = bodySeatbackRbarycentrics[seatGlueIdx].w;
        glueConstraint.posBaryB[0] = seatbackRGluePoints[seatGlueIdx].posBary[0];
        glueConstraint.posBaryB[1] = seatbackRGluePoints[seatGlueIdx].posBary[1];
        glueConstraint.posBaryB[2] = seatbackRGluePoints[seatGlueIdx].posBary[2];
        glueConstraint.posBaryB[3] = seatbackRGluePoints[seatGlueIdx].posBary[3];

        glueId = FmAddGlueConstraintToScene(scene, glueConstraint);
        car->bodySeatGlueConstraints[bodySeatIdx] = glueId;
        bodySeatIdx++;
    }

    {
        FmPlaneConstraintSetupParams planeConstraint;
        planeConstraint.kVelCorrection = 1.0f;
        planeConstraint.kPosCorrection = 1.0f;

        FmVector3 bodyMinPosition;
        FmVector3 bodyMaxPosition;
        FmGetBoundingBox(&bodyMinPosition, &bodyMaxPosition, *bodyTemplate.tetsBvh);

        FmClosestTetResult bodyClosestTetRoof;
        FmFindClosestTet(&bodyClosestTetRoof, bodyTemplate.vertRestPositions, bodyTemplate.tetVertIds, bodyTemplate.tetsBvh,
            FmInitVector3(0.0f, bodyMaxPosition.y - 0.5f, 0.0f));

        FmClosestTetResult bodyClosestTetFloor;
        FmFindClosestTet(&bodyClosestTetFloor, bodyTemplate.vertRestPositions, bodyTemplate.tetVertIds, bodyTemplate.tetsBvh,
            FmInitVector3(0.0f, bodyMinPosition.y + 0.5f, 0.0f));

        planeConstraint.bufferIdA = bodyBufferId;
        planeConstraint.bufferIdB = bodyBufferId;
        planeConstraint.bufferTetIdA = bodyClosestTetRoof.tetId;
        planeConstraint.bufferTetIdB = bodyClosestTetFloor.tetId;

        planeConstraint.posBaryA[0] = bodyClosestTetRoof.posBary[0];
        planeConstraint.posBaryA[1] = bodyClosestTetRoof.posBary[1];
        planeConstraint.posBaryA[2] = bodyClosestTetRoof.posBary[2];
        planeConstraint.posBaryA[3] = bodyClosestTetRoof.posBary[3];

        planeConstraint.posBaryB[0] = bodyClosestTetFloor.posBary[0];
        planeConstraint.posBaryB[1] = bodyClosestTetFloor.posBary[1];
        planeConstraint.posBaryB[2] = bodyClosestTetFloor.posBary[2];
        planeConstraint.posBaryB[3] = bodyClosestTetFloor.posBary[3];

        planeConstraint.bias0 = 0.25f;
        planeConstraint.planeNormal0 = FmInitVector3(0.0f, 1.0f, 0.0f);
        planeConstraint.numDimensions = 1;
        planeConstraint.nonNeg0 = true;

        uint carPlaneIdx = 0;
        uint planeId = FmAddPlaneConstraintToScene(scene, planeConstraint);
        car->planeConstraints[carPlaneIdx] = planeId;
        carPlaneIdx++;

        // Set up plane constraints to keep the hood out of the car.

        // All start disabled and are enabled based on proximity
        planeConstraint.enabled = false;

        // Create 5 points on hood
        FmVector3 hoodBoxMinPosition;
        FmVector3 hoodBoxMaxPosition;
        FmGetBoundingBox(&hoodBoxMinPosition, &hoodBoxMaxPosition, *bodyTemplate.tetsBvh);
        FmVector3 hoodCenter = (hoodBoxMinPosition + hoodBoxMaxPosition) * 0.5f;
        FmClosestTetResult hoodCollisionPoints[5];
        FmFindClosestTet(&hoodCollisionPoints[0], hoodTemplate.vertRestPositions, hoodTemplate.tetVertIds, hoodTemplate.tetsBvh, hoodCenter);
        FmFindClosestTet(&hoodCollisionPoints[1], hoodTemplate.vertRestPositions, hoodTemplate.tetVertIds, hoodTemplate.tetsBvh, FmInitVector3(hoodBoxMinPosition.x, hoodCenter.y, hoodBoxMinPosition.z));
        FmFindClosestTet(&hoodCollisionPoints[2], hoodTemplate.vertRestPositions, hoodTemplate.tetVertIds, hoodTemplate.tetsBvh, FmInitVector3(hoodBoxMinPosition.x, hoodCenter.y, hoodBoxMaxPosition.z));
        FmFindClosestTet(&hoodCollisionPoints[3], hoodTemplate.vertRestPositions, hoodTemplate.tetVertIds, hoodTemplate.tetsBvh, FmInitVector3(hoodBoxMaxPosition.x, hoodCenter.y, hoodBoxMinPosition.z));
        FmFindClosestTet(&hoodCollisionPoints[4], hoodTemplate.vertRestPositions, hoodTemplate.tetVertIds, hoodTemplate.tetsBvh, FmInitVector3(hoodBoxMaxPosition.x, hoodCenter.y, hoodBoxMaxPosition.z));

        // Use bodyClosestTetFloor and create new body point in engine compartment
        FmClosestTetResult bodyClosestTetEngineCompartment;
        FmFindClosestTet(&bodyClosestTetEngineCompartment, bodyTemplate.vertRestPositions, bodyTemplate.tetVertIds, bodyTemplate.tetsBvh,
            hoodCenter - FmInitVector3(0.0f, 0.25f, 0.0f));

        // Constraints for engine compartment
        for (uint i = 0; i < 5; i++)
        {
            planeConstraint.bufferIdA = hoodBufferId;
            planeConstraint.bufferIdB = bodyBufferId;
            planeConstraint.bufferTetIdA = hoodCollisionPoints[i].tetId;
            planeConstraint.bufferTetIdB = bodyClosestTetEngineCompartment.tetId;

            planeConstraint.posBaryA[0] = hoodCollisionPoints[i].posBary[0];
            planeConstraint.posBaryA[1] = hoodCollisionPoints[i].posBary[1];
            planeConstraint.posBaryA[2] = hoodCollisionPoints[i].posBary[2];
            planeConstraint.posBaryA[3] = hoodCollisionPoints[i].posBary[3];

            planeConstraint.posBaryB[0] = bodyClosestTetEngineCompartment.posBary[0];
            planeConstraint.posBaryB[1] = bodyClosestTetEngineCompartment.posBary[1];
            planeConstraint.posBaryB[2] = bodyClosestTetEngineCompartment.posBary[2];
            planeConstraint.posBaryB[3] = bodyClosestTetEngineCompartment.posBary[3];

            planeConstraint.bias0 = 0.5f;

            planeId = FmAddPlaneConstraintToScene(scene, planeConstraint);
            car->planeConstraints[carPlaneIdx] = planeId;
            carPlaneIdx++;
        }

        // Constraints for cabin
        for (uint i = 0; i < 5; i++)
        {
            planeConstraint.bufferIdA = hoodBufferId;
            planeConstraint.bufferIdB = bodyBufferId;
            planeConstraint.bufferTetIdA = hoodCollisionPoints[i].tetId;
            planeConstraint.bufferTetIdB = bodyClosestTetFloor.tetId;

            planeConstraint.posBaryA[0] = hoodCollisionPoints[i].posBary[0];
            planeConstraint.posBaryA[1] = hoodCollisionPoints[i].posBary[1];
            planeConstraint.posBaryA[2] = hoodCollisionPoints[i].posBary[2];
            planeConstraint.posBaryA[3] = hoodCollisionPoints[i].posBary[3];

            planeConstraint.posBaryB[0] = bodyClosestTetFloor.posBary[0];
            planeConstraint.posBaryB[1] = bodyClosestTetFloor.posBary[1];
            planeConstraint.posBaryB[2] = bodyClosestTetFloor.posBary[2];
            planeConstraint.posBaryB[3] = bodyClosestTetFloor.posBary[3];

            planeConstraint.bias0 = 0.75f;

            planeId = FmAddPlaneConstraintToScene(scene, planeConstraint);
            car->planeConstraints[carPlaneIdx] = planeId;
            carPlaneIdx++;
        }

        FM_ASSERT(carPlaneIdx == CarSimObject::numPlaneConstraints);
    }

    delete[] tetsIntersected;
}

#if CARS_PANELS_TIRES_SCENE
const uint gNumInstances = NUM_INSTANCES;
const uint gNumCarsPerInstance = NUM_CARS_PER_INSTANCE;
uint gNumCars = gNumCarsPerInstance*gNumInstances;
#else
uint gNumCars = NUM_CARS;
#endif
CarSimObject* gCars = NULL;
CarSimObjectTemplate gCarTemplate;
TetMeshBufferTemplate* gCarTemplates[8] = { NULL };
#endif

#if CARS_PANELS_TIRES_SCENE
TetMeshBufferTemplate* gWoodPanelBufferTemplate = NULL;
TetMeshBufferTemplate* gTractorTireBufferTemplate = NULL;

void LaunchCarSimObject(uint carId, float speed)
{
    CarSimObject* car = &gCars[carId];

    FmVector3 velocity = car->initialRotation.col0 * speed;

    for (uint carSubIdx = 0; carSubIdx < CarSimObject::numTetMeshBuffers; carSubIdx++)
    {
        FmTetMesh& tetMesh = *FmGetTetMesh(*FmGetTetMeshBuffer(*gScene, car->tetMeshBufferIds[carSubIdx]), 0);

        uint numVerts = FmGetNumVerts(tetMesh);
        for (uint vIdx = 0; vIdx < numVerts; vIdx++)
        {
            if (length(FmGetVertVelocity(tetMesh, vIdx)) < 100.0f)
            {
                FmSetVertVelocity(gScene, &tetMesh, vIdx, velocity);
            }
        }
    }

    for (uint rbIdx = 0; rbIdx < CarSimObject::numRigidBodies; rbIdx++)
    {
        uint rbId = car->rigidBodyIds[rbIdx];

        FmSetVelocity(gScene, FmGetRigidBody(*gScene, rbId), velocity);
    }
}

AMD::FmVector3 gExplosionOrigin(0.0f);
float gExplosionStartTime = -FLT_MAX;
float gExplosionDuration = 2.0f;
float gExplosionSpeed = 50.0f;
float gExplosionPressure = 2e7f;

void StartExplosion(const FmVector3& origin, float force, float pressure, float duration)
{
    const FmSceneControlParams& sceneParams = FmGetSceneControlParams(*gScene);
    gExplosionStartTime = sceneParams.simTime;

    gExplosionOrigin = origin;
    gExplosionPressure = force;
    gExplosionSpeed = pressure;
    gExplosionDuration = duration;
}

void StartExplosionAtProjectile(float force)
{
    gExplosionPressure = force;
    FmTetMeshBuffer* pTetMeshBuffer = FmGetTetMeshBuffer(*gScene, 0);
    FmTetMesh& tetMesh = *FmGetTetMesh(*pTetMeshBuffer, 0);
    StartExplosion(FmGetCenterOfMass(tetMesh), 2e7f, 50.0f, 2.0f);
    FmResetFromRestPositions(gScene, &tetMesh, FmMatrix3::identity(), FmInitVector3(-1000.0f, 0.0f, 0.0f));
}

void ApplyExplosion()
{
    const FmSceneControlParams& sceneParams = FmGetSceneControlParams(*gScene);

    if (sceneParams.simTime - gExplosionStartTime < gExplosionDuration)
    {
        uint numSceneMeshes = FmGetNumEnabledTetMeshes(*gScene);

        for (uint i = 0; i < numSceneMeshes; i++)
        {
            ApplyExplosionForce(gScene, FmGetTetMesh(*gScene, FmGetEnabledTetMeshId(*gScene, i)), gExplosionOrigin,
                gExplosionPressure,
                1.0f,
                gExplosionSpeed, sceneParams.timestep, sceneParams.simTime - gExplosionStartTime);
        }
    }
    else
    {
        gExplosionStartTime = -FLT_MAX;
    }
}

void ResetExplosion()
{
    gExplosionStartTime = -FLT_MAX;
}
#endif

#if WOOD_PANELS_IN_SCENE
struct WoodPanelSimObjectTemplate
{
    TetMeshBufferTemplate bufferTemplate;
    uint numCubesX;
    uint numCubesY;
    uint numCubesZ;
    float widthX;
    float widthY;
    float widthZ;
};

struct WoodPanelSimObject
{
    FmTetMeshBuffer* tetMeshBuffer;

#if KINEMATIC_TEST
    FmVector3 position;
    FmMatrix3 rotation;
    std::vector<uint> movingVerts;
    std::vector<FmVector3> movingVertOrigPos;
#endif
};

void InitWoodPanelSimObjectTemplate(WoodPanelSimObjectTemplate* panelTemplate,
    uint numCubesX, uint numCubesY, uint numCubesZ, float widthX, float widthY, float widthZ)
{
    panelTemplate->numCubesX = numCubesX;
    panelTemplate->numCubesY = numCubesY;
    panelTemplate->numCubesZ = numCubesZ;
    panelTemplate->widthX = widthX;
    panelTemplate->widthY = widthY;
    panelTemplate->widthZ = widthZ;

    bool enablePlasticity = false;
#if KINEMATIC_TEST
    bool enableFracture = true;
#else
    bool enableFracture = true;
#endif
    bool isKinematic = false;

    FmVector3 position = FmInitVector3(0.0f);
    FmVector3 velocity = FmInitVector3(0.0f);
    FmMatrix3 rotation = FmMatrix3::identity();

    FmTetMaterialParams materialParams;
    materialParams.restDensity = 1000.0f;
    materialParams.youngsModulus = 4.0e6f;
    materialParams.poissonsRatio = 0.25f;
    materialParams.fractureStressThreshold = 5.0e5f;

    uint collisionGroup = 0;
    InitTetMeshBufferTemplate(&panelTemplate->bufferTemplate, materialParams, collisionGroup, numCubesX, numCubesY, numCubesZ, widthX, widthY, widthZ, true, enablePlasticity, enableFracture, isKinematic);
}

#if KINEMATIC_TEST
float GetZOffset(float time)
{
    const float amp = 1.0f;
    const float period = 3.0f;
    const float timestep = (1.0f / 60.0f);
    return amp * sinf(2.0f * AMD_PI * (time / period));
}
#endif

void CreateWoodPanelSimObject(WoodPanelSimObject* panel, const WoodPanelSimObjectTemplate& panelTemplate, const FmVector3& position, const FmMatrix3& rotation, const FmVector3& velocity)
{
    // Set up a single memory buffer to hold all tet mesh data

    FmTetMesh* tetMeshPtr = NULL;
    panel->tetMeshBuffer = FmCreateTetMeshBuffer(panelTemplate.bufferTemplate.setupParams, panelTemplate.bufferTemplate.fractureGroupCounts, panelTemplate.bufferTemplate.tetFractureGroupIds, &tetMeshPtr);
    FmTetMesh& tetMesh = *tetMeshPtr;

    gTetMeshBuffers[gNumTetMeshBuffers] = panel->tetMeshBuffer;
    gNumTetMeshBuffers++;

    FmVector3 panelPos = position;
    panelPos += mul(rotation, FmInitVector3(-panelTemplate.widthX * 0.5f, 0.0f, 0.0f));

    FmInitVertState(&tetMesh, panelTemplate.bufferTemplate.vertRestPositions, rotation, panelPos, 1.0f, velocity);

    FmInitTetState(&tetMesh, panelTemplate.bufferTemplate.tetVertIds, panelTemplate.bufferTemplate.defaultMaterialParams);

    FmComputeMeshConstantMatrices(&tetMesh);

    FmSetMassesFromRestDensities(&tetMesh);

    FmInitConnectivity(&tetMesh, panelTemplate.bufferTemplate.vertIncidentTets);

    FmFinishTetMeshInit(&tetMesh);

#if TEST_CONDITION_NUMBERS
    AMD::FmSceneControlParams defaultParams;
    float meshCondition = AMD::FmCheckTetMeshCondition(&tetMesh, defaultParams);
    float maxCondition = AMD::FmCheckMaxTetMeshCondition(panel->tetMeshBuffer, defaultParams);
    printf("Mesh condition number: %f Max of fracture groups: %f\n", meshCondition, maxCondition);
#endif

#if KINEMATIC_TEST
    panel->position = position;
    panel->rotation = rotation;
    panel->movingVerts.clear();
    panel->movingVertOrigPos.clear();
#endif
    uint numVerts = FmGetNumVerts(tetMesh);

    for (uint vId = 0; vId < numVerts; vId++)
    {
        if (FmGetVertRestPosition(tetMesh, vId).y == 0.0f)
        {
#if KINEMATIC_TEST
            const FmSceneControlParams& sceneParams = FmGetSceneControlParams(*gScene);

            FmSetVertFlags(&tetMesh, vId, FM_VERT_FLAG_KINEMATIC | FM_VERT_FLAG_KINEMATIC_REMOVABLE);

            float offset = GetZOffset(sceneParams.simTime);
            float nextOffset = GetZOffset(sceneParams.simTime + sceneParams.timestep);
            float goalVel = (nextOffset - offset) / sceneParams.timestep;

            FmSetVertVelocity(gScene, &tetMesh, vId, FmInitVector3(0.0f, 0.0f, goalVel));

            panel->movingVerts.push_back(vId);
            panel->movingVertOrigPos.push_back(FmGetVertPosition(tetMesh, vId));
#else
            FmSetVertFlags(&tetMesh, vId, FM_VERT_FLAG_KINEMATIC | FM_VERT_FLAG_KINEMATIC_REMOVABLE);
            FmSetVertVelocity(NULL, &tetMesh, vId, FmInitVector3(0.0f));
#endif
        }
    }

    if (panelTemplate.bufferTemplate.tetFlags)
    {
        uint numTets = FmGetNumTets(tetMesh);
        for (uint tId = 0; tId < numTets; tId++)
        {
            FmSetTetFlags(&tetMesh, tId, panelTemplate.bufferTemplate.tetFlags[tId]);
        }
    }

    FmSetRemoveKinematicThreshold(&tetMesh, 1.5e6f);
}

void AddWoodPanelSimObjectToScene(WoodPanelSimObject* panel, FmScene* scene)
{
    FmAddTetMeshBufferToScene(scene, panel->tetMeshBuffer);
}

void FreeWoodPanelSimObjectTemplate(WoodPanelSimObjectTemplate* panelTemplate)
{
    panelTemplate->bufferTemplate.Destroy();
}

#if CARS_PANELS_TIRES_SCENE
const uint gNumWoodPanelsPerInstance = NUM_WOOD_PANELS_PER_INSTANCE;
const uint gNumWoodPanels = gNumWoodPanelsPerInstance*gNumInstances;
#else
const uint gNumWoodPanels = NUM_WOOD_PANELS;
#endif
WoodPanelSimObject gWoodPanels[gNumWoodPanels];
WoodPanelSimObjectTemplate gWoodPanelTemplate;
#endif

#if CARS_PANELS_TIRES_SCENE
struct TractorTireSimObjectTemplate
{
    TetMeshBufferTemplate bufferTemplate;
};

struct TractorTireSimObject
{
    FmTetMeshBuffer* tetMeshBuffer;
};

void InitTractorTireSimObjectTemplate(TractorTireSimObjectTemplate* tireTemplate, const char* modelsPath)
{
    FmVector3 position = FmInitVector3(0.0f);
    FmVector3 velocity = FmInitVector3(0.0f);
    FmMatrix3 rotation = FmMatrix3::identity();

    FmTetMaterialParams materialParams;
    materialParams.restDensity = 500.0f;
    materialParams.youngsModulus = 6.5e5f;//7.0e5f;
    materialParams.poissonsRatio = 0.3f;
    materialParams.lowerDeformationLimit = 0.7f;
    materialParams.upperDeformationLimit = 1.25f;

    uint collisionGroup = 0;
    InitTetMeshBufferTemplate(&tireTemplate->bufferTemplate, materialParams, collisionGroup, modelsPath, "tractor_tire_tets.1", false, false, false);
}

void CreateTractorTireSimObject(TractorTireSimObject* tire, const TractorTireSimObjectTemplate& tireTemplate, const FmVector3& position, const FmMatrix3& rotation, const FmVector3& velocity)
{
    // Set up a single memory buffer to hold all tet mesh data

    FmTetMesh* tetMeshPtr = NULL;
    tire->tetMeshBuffer = FmCreateTetMeshBuffer(tireTemplate.bufferTemplate.setupParams, tireTemplate.bufferTemplate.fractureGroupCounts, tireTemplate.bufferTemplate.tetFractureGroupIds, &tetMeshPtr);
    FmTetMesh& tetMesh = *tetMeshPtr;

    gTetMeshBuffers[gNumTetMeshBuffers] = tire->tetMeshBuffer;
    gNumTetMeshBuffers++;

    FmInitVertState(&tetMesh, tireTemplate.bufferTemplate.vertRestPositions, rotation, position, 1.0f, velocity);

    FmInitTetState(&tetMesh, tireTemplate.bufferTemplate.tetVertIds, tireTemplate.bufferTemplate.defaultMaterialParams);

    FmComputeMeshConstantMatrices(&tetMesh);

    FmSetMassesFromRestDensities(&tetMesh);

    FmInitConnectivity(&tetMesh, tireTemplate.bufferTemplate.vertIncidentTets);

    FmFinishTetMeshInit(&tetMesh);

#if TEST_CONDITION_NUMBERS
    AMD::FmSceneControlParams defaultParams;
    float meshCondition = AMD::FmCheckMaxTetMeshCondition(tire->tetMeshBuffer, defaultParams);
    printf("Mesh condition number: %f\n", meshCondition);
#endif
}

void AddTractorTireSimObjectToScene(TractorTireSimObject* panel, FmScene* scene)
{
    FmAddTetMeshBufferToScene(scene, panel->tetMeshBuffer);
}

void FreeTractorTireSimObjectTemplate(TractorTireSimObjectTemplate* panelTemplate)
{
    delete[] panelTemplate->bufferTemplate.vertRestPositions;
    delete[] panelTemplate->bufferTemplate.tetVertIds;
    delete[] panelTemplate->bufferTemplate.vertIncidentTets;

    FmDestroyBvh(panelTemplate->bufferTemplate.tetsBvh);
}

const uint gNumTractorTiresPerInstance = NUM_TRACTOR_TIRES;
const uint gNumTractorTires = gNumTractorTiresPerInstance*gNumInstances;
TractorTireSimObject gTractorTires[gNumTractorTires];
TractorTireSimObjectTemplate gTractorTireTemplate;
#endif

#if DUCKS_SCENE
uint gNumObjects = 75;
#elif RIGIDBODY_TEST_SCENE
#if GLUE_TEST
uint gNumObjects = 1;
#else
uint gNumObjects = 3;
#endif
#elif BLOCKS_SCENE
#if SELF_COLLISION_TEST
const uint gNumBlocksPerColumn = 1;
const uint gNumColumns = 1;
#else
const uint gNumBlocksPerColumn = 80;
const uint gNumColumns = 3;
#endif
uint gNumObjects = gNumBlocksPerColumn * gNumColumns;
#else
uint gNumObjects = 1;
#endif

float gDeltaY = 1.5f;

bool gMaterialDemoEnablePlasticity = true;
bool gMaterialDemoEnableFracture = false;

FmTetMaterialParams gMaterialDemoMaterialParams(50.0f, 4.0e6f, 0.3f, 1.0e4f, 0.5f, 0.01f, 2.0f, 5.0f, 60);

#if MATERIAL_DEMO_SCENE
uint gNumCubesX = 6;
uint gNumCubesY = 7;
uint gNumCubesZ = 1;
const float gCubeX = 1.0f;
const float gCubeY = 1.0f;
const float gCubeZ = 0.8f;
#elif WOOD_PANELS_IN_SCENE
uint gNumCubesX = 12;
uint gNumCubesY = 4;
uint gNumCubesZ = 1;
const float gCubeX = PANEL_WIDTH / gNumCubesX;
const float gCubeY = PANEL_HEIGHT / gNumCubesY;
const float gCubeZ = PANEL_DEPTH / gNumCubesZ;
#elif BLOCKS_SCENE || RIGIDBODY_TEST_SCENE
#if SELF_COLLISION_TEST
uint        gNumCubesX = 2;
uint        gNumCubesY = 50;
uint        gNumCubesZ = 2;
const float gCubeX = 0.2f;
const float gCubeY = 0.2f;
const float gCubeZ = 0.2f;
#else
uint        gNumCubesX = 4;
uint        gNumCubesY = 1;
uint        gNumCubesZ = 1;
const float gCubeX = 1.0f;
const float gCubeY = 1.0f;
const float gCubeZ = 1.0f;
#endif
#endif

#if WOOD_PANELS_SCENE
static const float gWoodPanelsRowSpacing = 4.0f;
#endif

#if MATERIAL_DEMO_SCENE
static const float gProjectileX = 1.0f;
static const float gProjectileY = 1.0f;
static const float gProjectileZ = 1.0f;
#else
static const float gProjectileX = 0.5f;
static const float gProjectileY = 0.5f;
static const float gProjectileZ = 0.5f;
#endif

void GetBlockMeshCounts(uint* numVerts, uint* numTets, uint numCubesX, uint numCubesY, uint numCubesZ)
{
    *numVerts = (numCubesX + 1) * (numCubesY + 1) * (numCubesZ + 1);
    *numTets = 6 * numCubesX*numCubesY*numCubesZ;
}

void CreateBlockMesh(FmArray<uint>* vertIncidentTets, uint numCubesX, uint numCubesY, uint numCubesZ)
{
    for (uint k = 0; k < numCubesZ; k++)
    {
        for (uint j = 0; j < numCubesY; j++)
        {
            for (uint i = 0; i < numCubesX; i++)
            {
                int v0 = (i + j*(numCubesX + 1) + k*(numCubesX + 1)*(numCubesY + 1));
                int v1 = ((i + 1) + j*(numCubesX + 1) + k*(numCubesX + 1)*(numCubesY + 1));
                int v2 = ((i + 1) + j*(numCubesX + 1) + (k + 1)*(numCubesX + 1)*(numCubesY + 1));
                int v3 = (i + j*(numCubesX + 1) + (k + 1)*(numCubesX + 1)*(numCubesY + 1));
                int v4 = (i + (j + 1)*(numCubesX + 1) + k*(numCubesX + 1)*(numCubesY + 1));
                int v5 = ((i + 1) + (j + 1)*(numCubesX + 1) + k*(numCubesX + 1)*(numCubesY + 1));
                int v6 = ((i + 1) + (j + 1)*(numCubesX + 1) + (k + 1)*(numCubesX + 1)*(numCubesY + 1));
                int v7 = (i + (j + 1)*(numCubesX + 1) + (k + 1)*(numCubesX + 1)*(numCubesY + 1));

                int tetId = 6 * (i + j*numCubesX + k*(numCubesX*numCubesY));
                FmAddIncidentTetToSet(vertIncidentTets[v0], tetId);
                FmAddIncidentTetToSet(vertIncidentTets[v1], tetId);
                FmAddIncidentTetToSet(vertIncidentTets[v3], tetId);
                FmAddIncidentTetToSet(vertIncidentTets[v4], tetId);

                tetId = 6 * (i + j*numCubesX + k*(numCubesX*numCubesY)) + 1;
                FmAddIncidentTetToSet(vertIncidentTets[v1], tetId);
                FmAddIncidentTetToSet(vertIncidentTets[v3], tetId);
                FmAddIncidentTetToSet(vertIncidentTets[v4], tetId);
                FmAddIncidentTetToSet(vertIncidentTets[v5], tetId);

                tetId = 6 * (i + j*numCubesX + k*(numCubesX*numCubesY)) + 2;
                FmAddIncidentTetToSet(vertIncidentTets[v3], tetId);
                FmAddIncidentTetToSet(vertIncidentTets[v5], tetId);
                FmAddIncidentTetToSet(vertIncidentTets[v7], tetId);
                FmAddIncidentTetToSet(vertIncidentTets[v4], tetId);

                tetId = 6 * (i + j*numCubesX + k*(numCubesX*numCubesY)) + 3;
                FmAddIncidentTetToSet(vertIncidentTets[v1], tetId);
                FmAddIncidentTetToSet(vertIncidentTets[v2], tetId);
                FmAddIncidentTetToSet(vertIncidentTets[v3], tetId);
                FmAddIncidentTetToSet(vertIncidentTets[v5], tetId);

                tetId = 6 * (i + j*numCubesX + k*(numCubesX*numCubesY)) + 4;
                FmAddIncidentTetToSet(vertIncidentTets[v3], tetId);
                FmAddIncidentTetToSet(vertIncidentTets[v2], tetId);
                FmAddIncidentTetToSet(vertIncidentTets[v7], tetId);
                FmAddIncidentTetToSet(vertIncidentTets[v5], tetId);

                tetId = 6 * (i + j*numCubesX + k*(numCubesX*numCubesY)) + 5;
                FmAddIncidentTetToSet(vertIncidentTets[v2], tetId);
                FmAddIncidentTetToSet(vertIncidentTets[v7], tetId);
                FmAddIncidentTetToSet(vertIncidentTets[v5], tetId);
                FmAddIncidentTetToSet(vertIncidentTets[v6], tetId);
            }
        }
    }
}

void InitBlockVerts(FmVector3* vertRestPositions, FmTetVertIds* tetVertIds, 
    uint numCubesX, uint numCubesY, uint numCubesZ,
    float cubeDimX, float cubeDimY, float cubeDimZ, float scale, bool randomize)
{
    for (uint i = 0; i < numCubesX + 1; i++)
    {
        float biasY = 0.2f*cubeDimY*randfloat2();

        for (uint k = 0; k < numCubesZ + 1; k++)
        {
            for (uint j = 0; j < numCubesY + 1; j++)
            {
                int v0 = (i + j*(numCubesX + 1) + k*(numCubesX + 1)*(numCubesY + 1));

                float x = cubeDimX * i;
                float y = cubeDimY * j;
                float z = -cubeDimZ * k;

                if (randomize)
                {
                    if (i > 0 && i < numCubesX)
                    {
                        x += 0.2f*cubeDimX*randfloat2();

                        if (x < 0.0f)
                        {
                            x = 0.0f;
                        }
                        if (x > cubeDimX * numCubesX)
                        {
                            x = cubeDimX * numCubesX;
                        }
                    }

                    if (j > 0 && j < numCubesY)
                    {
                        y += 0.2f*cubeDimY*randfloat2() + biasY;

                        if (y < 0.0f)
                        {
                            y = 0.0f;
                        }
                        if (y > cubeDimY * numCubesY)
                        {
                            y = cubeDimY * numCubesY;
                        }
                    }
                }

                vertRestPositions[v0] = FmInitVector3(x, y, z) * scale;
            }
        }
    }

    for (uint k = 0; k < numCubesZ; k++)
    {
        for (uint j = 0; j < numCubesY; j++)
        {
            for (uint i = 0; i < numCubesX; i++)
            {
                int v0 = (i + j*(numCubesX + 1) + k*(numCubesX + 1)*(numCubesY + 1));
                int v1 = ((i + 1) + j*(numCubesX + 1) + k*(numCubesX + 1)*(numCubesY + 1));
                int v2 = ((i + 1) + j*(numCubesX + 1) + (k + 1)*(numCubesX + 1)*(numCubesY + 1));
                int v3 = (i + j*(numCubesX + 1) + (k + 1)*(numCubesX + 1)*(numCubesY + 1));
                int v4 = (i + (j + 1)*(numCubesX + 1) + k*(numCubesX + 1)*(numCubesY + 1));
                int v5 = ((i + 1) + (j + 1)*(numCubesX + 1) + k*(numCubesX + 1)*(numCubesY + 1));
                int v6 = ((i + 1) + (j + 1)*(numCubesX + 1) + (k + 1)*(numCubesX + 1)*(numCubesY + 1));
                int v7 = (i + (j + 1)*(numCubesX + 1) + (k + 1)*(numCubesX + 1)*(numCubesY + 1));

                int tetId = 6 * (i + j*numCubesX + k*(numCubesX*numCubesY));
                tetVertIds[tetId].ids[0] = v0;
                tetVertIds[tetId].ids[1] = v1;
                tetVertIds[tetId].ids[2] = v3;
                tetVertIds[tetId].ids[3] = v4;

                tetId = 6 * (i + j*numCubesX + k*(numCubesX*numCubesY)) + 1;
                tetVertIds[tetId].ids[0] = v1;
                tetVertIds[tetId].ids[1] = v3;
                tetVertIds[tetId].ids[2] = v4;
                tetVertIds[tetId].ids[3] = v5;

                tetId = 6 * (i + j*numCubesX + k*(numCubesX*numCubesY)) + 2;
                tetVertIds[tetId].ids[0] = v3;
                tetVertIds[tetId].ids[1] = v5;
                tetVertIds[tetId].ids[2] = v7;
                tetVertIds[tetId].ids[3] = v4;

                tetId = 6 * (i + j*numCubesX + k*(numCubesX*numCubesY)) + 3;
                tetVertIds[tetId].ids[0] = v1;
                tetVertIds[tetId].ids[1] = v2;
                tetVertIds[tetId].ids[2] = v3;
                tetVertIds[tetId].ids[3] = v5;

                tetId = 6 * (i + j*numCubesX + k*(numCubesX*numCubesY)) + 4;
                tetVertIds[tetId].ids[0] = v3;
                tetVertIds[tetId].ids[1] = v2;
                tetVertIds[tetId].ids[2] = v7;
                tetVertIds[tetId].ids[3] = v5;

                tetId = 6 * (i + j*numCubesX + k*(numCubesX*numCubesY)) + 5;
                tetVertIds[tetId].ids[0] = v2;
                tetVertIds[tetId].ids[1] = v7;
                tetVertIds[tetId].ids[2] = v5;
                tetVertIds[tetId].ids[3] = v6;
            }
        }
    }
}

#if PROJECTILE_IN_SCENE
void FireProjectile(const FmMatrix3& viewRotation, const FmVector3& viewPosition, float speed, bool fixedPosForWoodPanels)
{
#if WOOD_PANELS_SCENE
    for (uint meshIdx = 0; meshIdx < NUM_WOOD_PANELS_ROWS; meshIdx++)
    {
        uint meshId = gWoodPanelProjectileMeshBufferIds[meshIdx];
#else
    uint meshId = gProjectileMeshBufferId;
    {
#endif
        FmTetMeshBuffer* pTetMeshBuffer = FmGetTetMeshBuffer(*gScene, meshId);
        if (!pTetMeshBuffer)
        {
            return;
        }

        FmTetMesh& tetMesh = *FmGetTetMesh(*pTetMeshBuffer, 0);

        const float cubeX = gProjectileX;
        const float cubeY = gProjectileY;
        const float cubeZ = gProjectileZ;

        FmVector3 posOffset;
        FmMatrix3 rotation;
        FmVector3 direction;
        if (fixedPosForWoodPanels)
        {
            posOffset = FmInitVector3(0.0f, 1.75f, -10.0f);
#if WOOD_PANELS_SCENE
            posOffset += FmInitVector3(gWoodPanelsRowSpacing, 0.0f, 0.0f) * (float)meshId;
#endif
            rotation = FmMatrix3::identity();
            direction = normalize(FmInitVector3(randfloat2()*0.05f, randfloat2()*0.05f, 1.0f));
        }
        else
        {
            rotation = FmMatrix3(
                FmInitVector3(viewRotation.col0.x, viewRotation.col0.y, viewRotation.col0.z),
                FmInitVector3(viewRotation.col1.x, viewRotation.col1.y, viewRotation.col1.z),
                FmInitVector3(viewRotation.col2.x, viewRotation.col2.y, viewRotation.col2.z));
            posOffset = FmInitVector3(viewPosition.x, viewPosition.y, viewPosition.z) + mul(rotation, FmInitVector3(-0.5f*cubeX, -0.5f*cubeY, 0.5f*cubeZ));
            direction = FmInitVector3(-viewRotation.col2.x, -viewRotation.col2.y, -viewRotation.col2.z);
        }

        FmResetFromRestPositions(gScene, &tetMesh, rotation, posOffset, direction * speed);
    }
}
#else
void FireProjectile(const FmMatrix3& viewRotation, const FmVector3& viewPosition, float speed, bool fixedPosForWoodPanels)
{
    (void)viewRotation;
    (void)viewPosition;
    (void)speed;
    (void)fixedPosForWoodPanels;
}
#endif

void InitScene(const char* modelsPath, const char* timingsPath, int numThreads, int randomSeed)
{
    (void)modelsPath;
    (void)timingsPath;

    if (gScene)
    {
        return;
    }

    // Set up task scheduler
    SampleInitTaskSystem(numThreads);

    numThreads = SampleGetTaskSystemNumThreads();

#if PERF_TEST
    printf("Random seed = %i Num threads = %i\n", randomSeed, numThreads);
#endif

    // Open timings file
#if FM_TIMINGS
    static char timingsFilename[1024];
#if BLOCKS_SCENE
    sprintf_s(timingsFilename, "%s/femtimings_%ublocks_%iseed_%ithreads.csv", timingsPath, gNumObjects, randomSeed, numThreads);
#elif DUCKS_SCENE
    sprintf_s(timingsFilename, "%s/femtimings_%uducks_%iseed_%ithreads.csv", timingsPath, gNumObjects, randomSeed, numThreads);
#elif WOOD_PANELS_SCENE
    sprintf_s(timingsFilename, "%s/femtimings_%uwoodpanels_%iseed_%ithreads.csv", timingsPath, gNumWoodPanels, randomSeed, numThreads);
#elif MATERIAL_DEMO_SCENE
    sprintf_s(timingsFilename, "%s/femtimings_materialdemo_%iseed_%ithreads.csv", timingsPath, randomSeed, numThreads);
#elif CARS_PANELS_TIRES_SCENE
    sprintf_s(timingsFilename, "%s/femtimings_%ucars_panels_tires_%iseed_%ithreads.csv", timingsPath, gNumCars, randomSeed, numThreads);
#else
    sprintf_s(timingsFilename, "%s/femtimings_%iseed_%ithreads.csv", timingsPath, randomSeed, numThreads);
#endif
    fopen_s(&gTimingsFile, timingsFilename, "w");

    if (gTimingsFile)
    {
        fprintf(gTimingsFile, "MeshComponents (%u), ImplicitStep (%u), BroadPhase (%u), MeshContacts (%u), ConstraintIslands (%u), ConstraintSolve (%u), Total(%u)\n",
            numThreads, numThreads, numThreads, numThreads,
            numThreads, numThreads, numThreads);
    }
#endif

    srand(randomSeed);

    // Create templates for assets
#if CARS_IN_SCENE
    gCars = new CarSimObject[gNumCars];

    InitCarSimObjectTemplate(&gCarTemplate, modelsPath);

    for (uint i = 0; i < NUM_CAR_TEMPLATES; i++)
    {
        gCarTemplates[i] = &gCarTemplate.tetMeshBufferTemplates[i];
    }
#endif
#if WOOD_PANELS_IN_SCENE
    InitWoodPanelSimObjectTemplate(&gWoodPanelTemplate, gNumCubesX, gNumCubesY, gNumCubesZ, PANEL_WIDTH, PANEL_HEIGHT, PANEL_DEPTH);
#endif
#if TIRES_IN_SCENE
    InitTractorTireSimObjectTemplate(&gTractorTireTemplate, modelsPath);
    gTractorTireBufferTemplate = &gTractorTireTemplate.bufferTemplate;
#endif

    // reset after external call to srand in setup
    srand(randomSeed);

    gFired = false;

    // Setup scene
    FmSceneSetupParams sceneParams;
    sceneParams.maxTetMeshBuffers = MAX_MESH_BUFFERS;
    sceneParams.maxTetMeshes = MAX_MESHES;
    sceneParams.maxRigidBodies = MAX_RIGID_BODIES;
    sceneParams.maxDistanceContacts = MAX_DISTANCE_CONTACTS;
    sceneParams.maxVolumeContacts = MAX_VOLUME_CONTACTS;
    sceneParams.maxVolumeContactVerts = MAX_VOLUME_CONTACT_VERTS;
    sceneParams.maxDeformationConstraints = MAX_MESH_BUFFERS * 64;
    sceneParams.maxGlueConstraints = MAX_GLUE_CONSTRAINTS;
    sceneParams.maxPlaneConstraints = MAX_GLUE_CONSTRAINTS;
    sceneParams.maxRigidBodyAngleConstraints = MAX_ANGLE_CONSTRAINTS;
    sceneParams.maxBroadPhasePairs = MAX_BROAD_PHASE_PAIRS;
    sceneParams.maxRigidBodyBroadPhasePairs = MAX_BROAD_PHASE_PAIRS;
    sceneParams.maxSceneVerts = MAX_VERTS_PER_MESH_BUFFER * MAX_MESH_BUFFERS;
    sceneParams.maxTetMeshBufferFeatures = MAX_VERTS_PER_MESH_BUFFER*4;
    sceneParams.numWorkerThreads = numThreads;

    sceneParams.maxConstraintSolverDataSize = 100000000;// FmEstimateSceneConstraintSolverDataSize(sceneParams);

    gScene = FmCreateScene(sceneParams);

    FmSetGroupsCanCollide(gScene, 2, 2, false); // no collisions between rigid bodies

    FmSceneControlParams params;

#if EXTERNAL_RIGIDBODIES
    params.rigidBodiesExternal = true;
#else
    params.rigidBodiesExternal = false;
#endif

    FmSetSceneControlParams(gScene, params);

    FmTaskSystemCallbacks taskSystemCallbacks;

    taskSystemCallbacks.SetCallbacks(
        SampleGetTaskSystemNumThreads,
        SampleGetTaskSystemWorkerIndex,
        SampleAsyncTask,
        SampleCreateSyncEvent,
        SampleDestroySyncEvent,
        SampleWaitForSyncEvent,
        SampleTriggerSyncEvent
#if !FM_ASYNC_THREADING
        , SampleCreateTaskWaitCounter,
        SampleWaitForTaskWaitCounter,
        SampleDestroyTaskWaitCounter,
        SampleSubmitTask,
        SampleParallelFor
#endif
    );

    FmSetSceneTaskSystemCallbacks(gScene, taskSystemCallbacks);

#if LOAD_SERIALIZED_SCENE
    // Load objects from serialized scene

    FmSetGroupsCanCollide(gScene, 1, 1, false); // no collisions between rigid bodies
    FmSetGroupsCanCollide(gScene, 0, 3, true);
    FmSetGroupsCanCollide(gScene, 2, 3, true);

    {
        FILE* sceneFile = fopen("femscene.bin", "rb");
        size_t serializationBufferSize;
        fread(&serializationBufferSize, sizeof(size_t), 1, sceneFile);
        fseek(sceneFile, 0, 0);

        uint8_t* pSerializationBuffer = (uint8_t*)FmAlignedMalloc(serializationBufferSize, 64);
        fread(pSerializationBuffer, serializationBufferSize, 1, sceneFile);
        fclose(sceneFile);

        FmDeserializeScene(gScene, gTetMeshBuffers, gRigidBodies, pSerializationBuffer);
        FmAlignedFree(pSerializationBuffer);
    }
#endif

    // Load or procedurally create some objects depending on scene setting

#if MATERIAL_DEMO_SCENE
    uint numMeshBuffers = 2;  // Slab and projectile
#elif DUCKS_SCENE || BLOCKS_SCENE || RIGIDBODY_TEST_SCENE
    uint numMeshBuffers = gNumObjects;
#elif WOOD_PANELS_SCENE
    uint numMeshBuffers = NUM_WOOD_PANELS_ROWS;  // A projectile for each row
#else
    uint numMeshBuffers = 1;  // Projectile
#endif

    FmVector3 posOffset(0.0f);
    FmVector3 velOffset(0.0f);
    FmMatrix3 rotation = FmMatrix3::identity();

    FmVector3 rbPos = FmInitVector3(0.0f, 4.0f, 0.0f);
    FmQuat rbRotation = normalize(FmQuat(randfloat2(), randfloat2(), randfloat2(), randfloat()));

    for (uint meshBufferIdx = 0; meshBufferIdx < numMeshBuffers; meshBufferIdx++)
    {
        uint numVerts = 0;
        uint numTets = 0;

        bool enablePlasticity = false;
        bool enableFracture = false;
        bool isKinematic = false;
#if BLOCKS_SCENE && KINEMATIC_TEST
        if (meshBufferIdx == 0)
        {
            isKinematic = true;
        }
#endif

        posOffset = FmInitVector3(0.0f);
        velOffset = FmInitVector3(0.0f);
        rotation = FmMatrix3::identity();

        // Get number of vertices and tetrahedra in the model, and initialize vertex incident tets

        FmArray<uint>* vertIncidentTets = NULL;

#if MATERIAL_DEMO_SCENE
        std::string nodeFile = std::string(modelsPath) + std::string("materialblock.1.node");
        std::string eleFile = std::string(modelsPath) + std::string("materialblock.1.ele");

        if (meshBufferIdx == 0)
        {
            GetBlockMeshCounts(&numVerts, &numTets, 1, 1, 1);
            vertIncidentTets = new FmArray<uint>[numVerts];
            CreateBlockMesh(vertIncidentTets, 1, 1, 1);
        }
        else
        {
            enableFracture = gMaterialDemoEnableFracture;
            enablePlasticity = gMaterialDemoEnablePlasticity;

            int loadRet = LoadNodeEleMeshNumVerts(nodeFile.c_str());
            if (loadRet < 0)
            {
                exit(-1);
            }
            numVerts = loadRet;

            vertIncidentTets = new FmArray<uint>[numVerts];

            loadRet = LoadNodeEleMeshNumTets(eleFile.c_str(), vertIncidentTets, numVerts);
            if (loadRet < 0)
            {
                exit(-1);
            }
            numTets = loadRet;
        }
#elif DUCKS_SCENE
        std::string nodeFile = std::string(modelsPath) + std::string("duck.1.node");
        std::string eleFile = std::string(modelsPath) + std::string("duck.1.ele");

        int loadRet = LoadNodeEleMeshNumVerts(nodeFile.c_str());
        if (loadRet < 0)
        {
            exit(-1);
        }
        numVerts = loadRet;

        vertIncidentTets = new FmArray<uint>[numVerts];

        loadRet = LoadNodeEleMeshNumTets(eleFile.c_str(), vertIncidentTets, numVerts);
        if (loadRet < 0)
        {
            exit(-1);
        }
        numTets = loadRet;
#elif PROJECTILE_IN_SCENE
        GetBlockMeshCounts(&numVerts, &numTets, 1, 1, 1);

        vertIncidentTets = new FmArray<uint>[numVerts];

        CreateBlockMesh(vertIncidentTets, 1, 1, 1);
#elif BLOCKS_SCENE || RIGIDBODY_TEST_SCENE
        GetBlockMeshCounts(&numVerts, &numTets, gNumCubesX, gNumCubesY, gNumCubesZ);
        vertIncidentTets = new FmArray<uint>[numVerts];
        CreateBlockMesh(vertIncidentTets, gNumCubesX, gNumCubesY, gNumCubesZ);
#endif

        // Initialize rest positions and tet vertex ids.

        FmVector3* vertRestPositions = new FmVector3[numVerts];
        FmTetVertIds* tetVertIds = new FmTetVertIds[numTets];

#if MATERIAL_DEMO_SCENE
        if (meshBufferIdx == 0)
        {
            InitBlockVerts(vertRestPositions, tetVertIds, 1, 1, 1, gProjectileX, gProjectileY, gProjectileZ, 1.0f, false);
        }
        else
        {
            LoadNodeEleMeshData(nodeFile.c_str(), eleFile.c_str(), vertRestPositions, tetVertIds);
        }
#elif DUCKS_SCENE
        LoadNodeEleMeshData(nodeFile.c_str(), eleFile.c_str(), vertRestPositions, tetVertIds);
#elif PROJECTILE_IN_SCENE
        InitBlockVerts(vertRestPositions, tetVertIds, 1, 1, 1, gProjectileX, gProjectileY, gProjectileZ, 1.0f, false);
#elif BLOCKS_SCENE || RIGIDBODY_TEST_SCENE
        InitBlockVerts(vertRestPositions, tetVertIds, gNumCubesX, gNumCubesY, gNumCubesZ, gCubeX, gCubeY, gCubeZ, 1.0f, false);
#endif

        // Set up a single memory buffer to hold all tet mesh data

        FmFractureGroupCounts* fractureGroupCounts = new FmFractureGroupCounts[numTets];
        uint* tetFractureGroupIds = new uint[numTets];

        FmTetMeshBufferBounds bounds;
        FmComputeTetMeshBufferBounds(
            &bounds,
            fractureGroupCounts,
            tetFractureGroupIds,
            vertIncidentTets, tetVertIds, NULL,
            numVerts, numTets, enableFracture);

        FmTetMeshBufferSetupParams tetMeshBufferParams;
        tetMeshBufferParams.numVerts = bounds.numVerts;
        tetMeshBufferParams.maxVerts = bounds.maxVerts;
        tetMeshBufferParams.numTets = bounds.numTets;
        tetMeshBufferParams.maxVertAdjacentVerts = bounds.maxVertAdjacentVerts;
        tetMeshBufferParams.numTets = bounds.numTets;
        tetMeshBufferParams.numVertIncidentTets = bounds.numVertIncidentTets;
        tetMeshBufferParams.maxExteriorFaces = bounds.maxExteriorFaces;
        tetMeshBufferParams.maxTetMeshes = bounds.maxTetMeshes;
        tetMeshBufferParams.enablePlasticity = enablePlasticity;
        tetMeshBufferParams.enableFracture = enableFracture;
        tetMeshBufferParams.isKinematic = isKinematic;

        FmTetMesh* tetMeshPtr = NULL;
        FmTetMeshBuffer* tetMeshBuffer = FmCreateTetMeshBuffer(tetMeshBufferParams, fractureGroupCounts, tetFractureGroupIds, &tetMeshPtr);

        gTetMeshBuffers[gNumTetMeshBuffers] = tetMeshBuffer;
        gNumTetMeshBuffers++;

        delete[] fractureGroupCounts;
        delete[] tetFractureGroupIds;

        FmTetMesh& tetMesh = *tetMeshPtr;

        // Copy mesh data into FmTetMesh structure and set material parameters.

#if MATERIAL_DEMO_SCENE
        if (meshBufferIdx == 0)
        {
            rotation = FmMatrix3::identity();
            posOffset += FmInitVector3(1000.0f, 0.0f, 1000.0f);

            FmInitVertState(&tetMesh, vertRestPositions, rotation, posOffset);

            FmTetMaterialParams materialParams;

            materialParams.restDensity = 300.0f;
            materialParams.poissonsRatio = 0.4f;
            materialParams.youngsModulus = 1.0e6f;

            FmInitTetState(&tetMesh, tetVertIds, materialParams);

            FmComputeMeshConstantMatrices(&tetMesh);

            FmSetMassesFromRestDensities(&tetMesh);
        }
        else
        {
            rotation = FmMatrix3::identity();
            posOffset += FmInitVector3(float(-gCubeX * gNumCubesX * 0.5f), float(params.distContactThreshold * 1.1), 0.0f);

            FmInitVertState(&tetMesh, vertRestPositions, rotation, posOffset, 1.0f, velOffset);

            FmTetMaterialParams materialParams;
            materialParams = gMaterialDemoMaterialParams;
            //materialParams.lowerDeformationLimit = 0.5f;

            FmInitTetState(&tetMesh, tetVertIds, materialParams);

            FmComputeMeshConstantMatrices(&tetMesh);

            FmSetMassesFromRestDensities(&tetMesh);

            for (uint vId = 0; vId < numVerts; vId++)
            {
                if (FmGetVertRestPosition(tetMesh, vId).y < 0.05f)
                {
                    FmSetVertFlags(&tetMesh, vId, FM_VERT_FLAG_KINEMATIC);
                    FmSetVertVelocity(NULL, &tetMesh, vId, FmInitVector3(0.0f));
                }
            }
        }
#elif DUCKS_SCENE
        posOffset = FmInitVector3(0.0f, 1.75f, 0.0f) * (float)meshBufferIdx;

        posOffset.x += randfloat2() * 0.2f;
        posOffset.y += randfloat2() * 0.2f;
        posOffset.z += randfloat2() * 0.2f;

        rotation = FmMatrix3::identity();

        FmInitVertState(&tetMesh, vertRestPositions, rotation, posOffset, 1.0f);

        FmTetMaterialParams materialParams;
        materialParams.restDensity = 20.0f;
        materialParams.poissonsRatio = 0.4f;
        materialParams.youngsModulus = 3.0e4f;

        FmInitTetState(&tetMesh, tetVertIds, materialParams);

        FmComputeMeshConstantMatrices(&tetMesh);
#elif PROJECTILE_IN_SCENE
        const float cubeX = gProjectileX;
        const float cubeY = gProjectileY;
        const float cubeZ = gProjectileZ;

        rotation = FmMatrix3::identity();
        posOffset += FmInitVector3(-1000.0f, 0.0f, 0.0f) + FmInitVector3(10.0f, 0.0f, 0.0f)*(float)meshBufferIdx;

        FmInitVertState(&tetMesh, vertRestPositions, rotation, posOffset, 1.0f, velOffset);

        FmTetMaterialParams materialParams;
        materialParams.restDensity = 1.0f;
        materialParams.poissonsRatio = 0.3f;
        materialParams.youngsModulus = 1.0e7f;
        materialParams.lowerDeformationLimit = 0.95f;
        materialParams.upperDeformationLimit = 1.05f;

        FmInitTetState(&tetMesh, tetVertIds, materialParams);

        float restVolume = FmComputeMeshConstantMatrices(&tetMesh);

        float mass = 2000.0f;
        FmSetMassesFromRestDensities(&tetMesh, mass / restVolume);
#elif BLOCKS_SCENE
        uint columnIdx = meshBufferIdx / gNumBlocksPerColumn;
        uint blockIdx = meshBufferIdx % gNumBlocksPerColumn;

#if KINEMATIC_TEST
        posOffset = FmInitVector3(0.0f, 1.00299f, 0.0f) * (float)blockIdx + FmInitVector3(8.0f, 0.0f, 0.0f) * (float)((int)columnIdx - (int)gNumColumns / 2);
#else
        posOffset = FmInitVector3(0.0f, 2.0f, 0.0f) * (float)blockIdx + FmInitVector3(8.0f, 0.0f, 0.0f) * (float)((int)columnIdx - (int)gNumColumns/2);
        posOffset.x += randfloat2() * 0.2f;
        posOffset.y += randfloat2() * 0.2f;
        posOffset.z += randfloat2() * 0.2f;
#endif
        rotation = FmMatrix3::rotationY(FM_PI/2.0f);

        velOffset = FmInitVector3(0.0f, 0.0f, 0.0f);

        FmInitVertState(&tetMesh, vertRestPositions, rotation, posOffset, 1.0f, velOffset);

        FmTetMaterialParams materialParams;
        materialParams.restDensity = 20.0f;
        materialParams.poissonsRatio = 0.4f;
#if SELF_COLLISION_TEST
        FmEnableSelfCollision(&tetMesh, true);
        materialParams.youngsModulus = 8.0e4f;
#else
        materialParams.youngsModulus = 8.0e3f;
#endif

        FmInitTetState(&tetMesh, tetVertIds, materialParams);

        FmComputeMeshConstantMatrices(&tetMesh);

#elif RIGIDBODY_TEST_SCENE
#if GLUE_TEST
        rotation = FmMatrix3(rbRotation);
        posOffset = rbPos + FmInitVector3(0.0f, 4.0f, 0.0f) * (float)(meshBufferIdx);
        posOffset += rotate(rbRotation, FmInitVector3(0.5f, 0.0f, 0.0f) - FmInitVector3(gCubeX, gCubeY, -gCubeZ) * 0.5f);
        velOffset = FmInitVector3(0.0f, 10.0f, 0.0f);
#else
        rotation = FmMatrix3::identity();
        posOffset = FmInitVector3(-2.0f, 0.5f, 0.0f) + FmInitVector3(0.0f, 3.0f * gDeltaY, 0.0f) * (float)meshBufferIdx;
        posOffset += FmInitVector3(0.0f, 1.1f, 0.0f);
        velOffset = FmInitVector3(0.0f, 0.0f, 0.0f);
#endif

        FmInitVertState(&tetMesh, vertRestPositions, rotation, posOffset, 1.0f, velOffset);

        FmTetMaterialParams materialParams;
        materialParams.restDensity = 20.0f;
        materialParams.poissonsRatio = 0.4f;
        materialParams.youngsModulus = 8.0e3f;

        FmInitTetState(&tetMesh, tetVertIds, materialParams);

        FmComputeMeshConstantMatrices(&tetMesh);

        uint numMeshVerts = FmGetNumVerts(tetMesh);
        for (uint i = 0; i < numMeshVerts; i++)
        {
            FmSetVertMass(&tetMesh, i, 1.0f);
            FmSetVertPosition(NULL, &tetMesh, i, posOffset + mul(rotation, FmGetVertRestPosition(tetMesh, i)));
            FmSetVertVelocity(NULL, &tetMesh, i, FmInitVector3(0.0f) + velOffset);
        }
#endif

        FmInitConnectivity(&tetMesh, vertIncidentTets);

        FmFinishTetMeshInit(&tetMesh);

#if TEST_CONDITION_NUMBERS
        AMD::FmSceneControlParams defaultParams;
        float meshCondition = AMD::FmCheckTetMeshCondition(&tetMesh, defaultParams);
        printf("Mesh condition number: %f\n", meshCondition);

        if (enableFracture)
        {
            float maxCondition = AMD::FmCheckMaxTetMeshCondition(tetMeshBuffer, defaultParams);
            printf("Max of fracture groups: %f\n", maxCondition);
        }
#endif

        FmAddTetMeshBufferToScene(gScene, tetMeshBuffer);

#if WOOD_PANELS_SCENE
        gWoodPanelProjectileMeshBufferIds[meshBufferIdx] = FmGetTetMeshBufferId(*tetMeshBuffer);
#elif PROJECTILE_IN_SCENE
        if (meshBufferIdx == 0)
        {
            gProjectileMeshBufferId = FmGetTetMeshBufferId(*tetMeshBuffer);
        }
#endif

        delete[] vertRestPositions;
        delete[] tetVertIds;
        delete[] vertIncidentTets;
    }

#if CARS_PANELS_TIRES_SCENE
    uint numRigidBodies = CarSimObject::numRigidBodies * gNumCars;
#elif GLUE_TEST
    uint numRigidBodies = 1;
#elif RIGIDBODY_TEST_SCENE
    uint numRigidBodies = gNumObjects * 2;
#else
    uint numRigidBodies = 0;
#endif
    (void)numRigidBodies;
#if EXTERNAL_RIGIDBODIES
    // Create external scene with rigid bodies to test coordinated solve

    ExampleRigidBodiesSceneSetupParams rbSceneParams;

    rbSceneParams.maxRigidBodies = std::max(numRigidBodies, (uint)MAX_RIGID_BODIES);

    rbSceneParams.rigidRigidConstraintsSetupParams.maxDistanceContacts = MAX_DISTANCE_CONTACTS;
    rbSceneParams.rigidRigidConstraintsSetupParams.maxFractureContacts = MAX_FRACTURE_CONTACTS;
    rbSceneParams.rigidRigidConstraintsSetupParams.maxVolumeContacts = MAX_VOLUME_CONTACTS;
    rbSceneParams.rigidRigidConstraintsSetupParams.maxVolumeContactVerts = MAX_VOLUME_CONTACT_VERTS;
    rbSceneParams.rigidRigidConstraintsSetupParams.maxGlueConstraints = 128;
    rbSceneParams.rigidRigidConstraintsSetupParams.maxBroadPhasePairs = 0; // Using FEM lib broad phase
    rbSceneParams.rigidFEMConstraintsSetupParams.maxDistanceContacts = MAX_DISTANCE_CONTACTS;
    rbSceneParams.rigidFEMConstraintsSetupParams.maxFractureContacts = MAX_FRACTURE_CONTACTS;
    rbSceneParams.rigidFEMConstraintsSetupParams.maxVolumeContacts = MAX_VOLUME_CONTACTS;
    rbSceneParams.rigidFEMConstraintsSetupParams.maxVolumeContactVerts = MAX_VOLUME_CONTACT_VERTS;
    rbSceneParams.rigidFEMConstraintsSetupParams.maxGlueConstraints = 128;
    rbSceneParams.rigidFEMConstraintsSetupParams.maxBroadPhasePairs = 0; // Using FEM lib broad phase
    rbSceneParams.maxObjectPairTriPairs = MAX_VERTS_PER_MESH_BUFFER * 2;
    rbSceneParams.maxObjectPairVolContactVerts = MAX_VERTS_PER_MESH_BUFFER * 2;
    rbSceneParams.maxJacobianSubmats = MAX_CONTACTS * 8;

    gRbScene = new ExampleRigidBodiesScene(rbSceneParams);
#endif

#if RIGIDBODY_TEST_SCENE
    // Set up rigid body objects

    for (uint rbIdx = 0; rbIdx < numRigidBodies; rbIdx++)
    {
        FmRigidBodyState rigidBodyState;

#if GLUE_TEST
        rigidBodyState.pos = rbPos;
        rigidBodyState.quat = rbRotation;
        //rigidBodyState.quat = normalize(FmQuat(0.0f, 0.0f, 0.0f, 1.0f));
        rigidBodyState.vel = FmInitVector3(0.0f, 0.0f, 0.0f);
        rigidBodyState.angvel = FmInitVector3(0.0f);

        float hx = 1.8f * 0.6f;
        float hy = 1.0f * 0.6f;
        float hz = 1.6f * 0.6f;
#else
        float offsetY = (rbIdx / 2) * 3.0f * gDeltaY + (rbIdx % 2) * gDeltaY;
        rigidBodyState.pos = FmInitVector3(0.0f, 3.5f, -0.5f) + FmInitVector3(0.0, offsetY, 0.0f);// *(float)rbIdx;
        rigidBodyState.pos += FmInitVector3(0.5f*randfloat2(), 0.0f, 0.0f);
        rigidBodyState.quat = normalize(FmQuat(0.0f, 0.0f, 0.0f, 1.0f));
        rigidBodyState.vel = FmInitVector3(0.0f);
        rigidBodyState.angVel = FmInitVector3(0.0f);

        float hx = 1.0f * 0.6f;
        float hy = 1.0f * 0.6f;
        float hz = 1.0f * 0.6f;
#endif

        float mass = 10.0f;

        FmRigidBodySetupParams setupParams;
        setupParams.halfDimX = hx;
        setupParams.halfDimY = hy;
        setupParams.halfDimZ = hz;
        setupParams.collisionGroup = 0;
        setupParams.state = rigidBodyState;
#if KINEMATIC_TEST
        setupParams.isKinematic = (rbIdx == 0);
#else
        setupParams.isKinematic = false;
#endif
        setupParams.mass = mass;
        setupParams.bodyInertiaTensor = FmComputeBodyInertiaTensorForBox(hx, hy, hz, mass);

        FmRigidBody* rigidBody = FmCreateRigidBody(setupParams);

#if EXTERNAL_RIGIDBODIES
        AddRigidBodyToScenes(gRbScene, gScene, rigidBody);
#else
        FmAddRigidBodyToScene(gScene, rigidBody);
#endif
        gRigidBodies[gNumRigidBodies] = rigidBody;
        gNumRigidBodies++;
    }
#endif

#if LOAD_FEM_ASSET
    // Load objects from a .FEM file

    FmSetGroupsCanCollide(gScene, 1, 1, false); // no collisions between rigid bodies
    FmSetGroupsCanCollide(gScene, 0, 3, true);
    FmSetGroupsCanCollide(gScene, 2, 3, true);

    const char* FEMFilename = "FEMFiles/FEM_SimpleSquare.fem";

    FEMResource resource;
    if (!LoadFEMFile_v1_0(&resource, FEMFilename))
    {
        fprintf(stderr, "LoadFEMContent could not load %s\n", FEMFilename);
        exit(-1);
    }

    FmVector3 modelPosition = FmInitVector3(0.0f, 2.0f, 0.0f);
    FmMatrix3 modelRotation = FmMatrix3::identity();

    for (int i = 0; i < (int)resource.ComponentResources.size(); i++)
    {
        bool isKinematic = false;

        FmTetMaterialParams materialParams;

        materialParams.restDensity = 1000.0f;
        materialParams.youngsModulus = 1.0e7f;
        materialParams.fractureStressThreshold = 1.0e6f;

        int setKinematic = 0;

        FmTetMeshBuffer* tetMeshBuffer = CreateTetMeshBuffer(resource.ComponentResources[i], materialParams, modelPosition, modelRotation, FmInitVector3(0.0f), 0.0f, false, true, isKinematic, setKinematic, 0.05f);
        FmAddTetMeshBufferToScene(gScene, tetMeshBuffer);
    }

    for (int i = 0; i < (int)resource.ActorResource.RigidBodies.size(); i++)
    {
        AMD::FmRigidBodySetupParams bodyParams;
        bodyParams.collisionGroup = 1;

        FRigidBody temp = resource.ActorResource.RigidBodies[i];

        bodyParams.state.pos = AMD::FmInitVector3(temp.Position.X, temp.Position.Y, temp.Position.Z);
        bodyParams.state.quat = AMD::FmInitQuat(temp.Rotation.X, temp.Rotation.Y, temp.Rotation.Z, temp.Rotation.W);
        bodyParams.state.vel = AMD::FmInitVector3(0.0f, 0.0f, 0.0f);
        bodyParams.state.angVel = AMD::FmInitVector3(0.0f, 0.0f, 0.0f);

        bodyParams.halfDimX = temp.Dimensions.X;
        bodyParams.halfDimY = temp.Dimensions.Y;
        bodyParams.halfDimZ = temp.Dimensions.Z;

        bodyParams.mass = temp.mass;

        AMD::FmVector3 col1 = AMD::FmInitVector3(temp.BodyInertiaTensor[0], temp.BodyInertiaTensor[1], temp.BodyInertiaTensor[2]);
        AMD::FmVector3 col2 = AMD::FmInitVector3(temp.BodyInertiaTensor[3], temp.BodyInertiaTensor[4], temp.BodyInertiaTensor[5]);
        AMD::FmVector3 col3 = AMD::FmInitVector3(temp.BodyInertiaTensor[6], temp.BodyInertiaTensor[7], temp.BodyInertiaTensor[8]);

        bodyParams.bodyInertiaTensor = AMD::FmInitMatrix3(col1, col2, col3);

        bodyParams.state.pos = modelPosition + mul(rotation, bodyParams.state.pos);
        bodyParams.state.quat = mul(FmQuat(modelRotation), bodyParams.state.quat);

        FmRigidBody* rigidBody = FmCreateRigidBody(bodyParams);
        gRigidBodies[gNumRigidBodies] = rigidBody;
        gNumRigidBodies++;

        FmAddRigidBodyToScene(gScene, rigidBody);
    }

    for (int i = 0; i < (int)resource.ActorResource.AngleConstraints.size(); i++)
    {
        FAngleConstraint* it = &resource.ActorResource.AngleConstraints[i];
        AMD::FmRigidBodyAngleConstraintSetupParams constraint;
        constraint.kVelCorrection = 1.0f;
        constraint.kPosCorrection = 1.0f;
        constraint.frictionCoeff = 0.6f;

        constraint.objectIdA = it->BodyA | FM_RB_FLAG;
        constraint.objectIdB = it->BodyB | FM_RB_FLAG;

        constraint.axisBodySpaceA = AMD::FmInitVector3(it->AxisBodySpaceA.X, it->AxisBodySpaceA.Y, it->AxisBodySpaceA.Z);
        constraint.axisBodySpaceB = AMD::FmInitVector3(it->AxisBodySpaceB.X, it->AxisBodySpaceB.Y, it->AxisBodySpaceB.Z);

        AMD::FmAddRigidBodyAngleConstraintToScene(gScene, constraint);
    }

    for (int i = 0; i < (int)resource.ActorResource.GlueConstraints.size(); i++)
    {
        FGlueConstraint* it = &resource.ActorResource.GlueConstraints[i];
        FmGlueConstraintSetupParams constraint;

        constraint.kVelCorrection = 1.0f;
        constraint.kPosCorrection = 1.0f;

        constraint.bufferIdA = (it->IsRigidBodyA) ? (it->BodyA | FM_RB_FLAG) : it->BodyA + 1;
        constraint.bufferIdB = (it->IsRigidBodyB) ? (it->BodyB | FM_RB_FLAG) : it->BodyB + 1;

        uint compIdA = it->BodyA;
        uint compIdB = it->BodyB;

        constraint.bufferTetIdA = (it->IsRigidBodyA) ? it->TetIdA : resource.ComponentResources[compIdA].TetPermutation[it->TetIdA];
        constraint.bufferTetIdB = (it->IsRigidBodyB) ? it->TetIdB : resource.ComponentResources[compIdB].TetPermutation[it->TetIdB];

        constraint.posBodySpaceA[0] = it->PosBodySpaceA.X;
        constraint.posBodySpaceA[1] = it->PosBodySpaceA.Y;
        constraint.posBodySpaceA[2] = it->PosBodySpaceA.Z;
        constraint.posBodySpaceA[3] = it->PosBodySpaceA.W;

        constraint.posBodySpaceB[0] = it->PosBodySpaceB.X;
        constraint.posBodySpaceB[1] = it->PosBodySpaceB.Y;
        constraint.posBodySpaceB[2] = it->PosBodySpaceB.Z;
        constraint.posBodySpaceB[3] = it->PosBodySpaceB.W;

        constraint.breakThreshold = it->BreakThreshold;
        constraint.minGlueConstraints = (uint8_t)it->MinGlueConstraints;

        AMD::FmAddGlueConstraintToScene(gScene, constraint);
    }
#endif

#if CARS_PANELS_TIRES_SCENE
    const float instanceSpacingX = 10.0f;

    const float woodPanelsSpacingZ = 4.0f;
    const float woodPanelsHalfZ = 0.5f * (float)(gNumWoodPanelsPerInstance - 1) * woodPanelsSpacingZ;
    const float woodPanelsMinZ = -woodPanelsHalfZ;
    const float woodPanelsMaxZ = woodPanelsHalfZ;
#endif

#if CARS_IN_SCENE
    // Create car instances and place in scene
    for (uint carIdx = 0; carIdx < gNumCars; carIdx++)
    {
        FmVector3 position, velocity;
#if CARS_PANELS_TIRES_SCENE
        int instanceIdx = carIdx / gNumCarsPerInstance;
        position = FmInitVector3((float)(instanceIdx - (int)(gNumInstances / 2)) * instanceSpacingX, 0.0f, woodPanelsMinZ - 4.0f);
        rotation = FmMatrix3::rotationY(-FM_PI * 0.5f + 0.025f * FM_PI * randfloat2());
        velocity = FmInitVector3(0.0f);
#else
        // collisions
        const float spacing = 5.0f;
        uint spaces = (gNumCars > 1) ? (gNumCars / 2 - 1) : 0;
        float totalWidth = spacing * spaces;
        const float speed = (gNumCars == 1) ? 0.0f : 35.0f;
        if (carIdx < gNumCars / 2)
        {
            position = FmInitVector3(-totalWidth * 0.5f, 0.0f, 5.0f) + FmInitVector3(spacing, 0.0f, 0.0f) * (float)carIdx;
            //rotation = FmMatrix3::rotationY(0.05f * 3.14159265f * randfloat2());
            rotation = FmMatrix3::rotationY(3.14159265f * 0.5f + 0.05f * 3.14159265f * randfloat2());
            //rotation = FmMatrix3::rotationY(3.14159265f * 0.5f);//FmMatrix3::identity();
            velocity = rotation.col0 * speed;
        }
        else
        {
            position = FmInitVector3(-totalWidth * 0.5f, 0.0f, -5.0f) + FmInitVector3(spacing, 0.0f, 0.0f) * (float)((int)carIdx - (int)gNumCars / 2);
            //rotation = FmMatrix3::rotationY(-3.14159265f + 0.05f * 3.14159265f * randfloat2());
            rotation = FmMatrix3::rotationY(-3.14159265f * 0.5f + 0.05f * 3.14159265f * randfloat2());
            //rotation = FmMatrix3::rotationY(-3.14159265f * 0.5f);// FmMatrix3::rotationY(-3.14159265f);
            velocity = rotation.col0 * speed;
        }
#endif

        CreateCarSimObject(&gCars[carIdx], gCarTemplate, position, rotation, velocity);
        AddCarSimObjectToScene(&gCars[carIdx], gCarTemplate, gScene);
    }
#endif

#if WOOD_PANELS_IN_SCENE
    // Create wood panels instances and place in scene
    for (uint i = 0; i < gNumWoodPanels; i++)
    {
#if CARS_PANELS_TIRES_SCENE
        int instanceIdx = i / gNumWoodPanelsPerInstance;
        uint rowIdx = i % gNumWoodPanelsPerInstance;
        FmVector3 position = FmInitVector3((float)(instanceIdx - (int)(gNumInstances / 2)) * instanceSpacingX, params.distContactThreshold * 1.1f, woodPanelsSpacingZ * ((float)rowIdx - 0.5f * (gNumWoodPanelsPerInstance - 1)));
#else
        uint rowIdx = i / NUM_WOOD_PANELS_PER_ROW;
        uint colIdx = i % NUM_WOOD_PANELS_PER_ROW;
        FmVector3 position = FmInitVector3(gWoodPanelsRowSpacing * (float)rowIdx, params.distContactThreshold * 1.1f, 2.5f * (float)((int)colIdx - 2));
#endif
        CreateWoodPanelSimObject(&gWoodPanels[i], gWoodPanelTemplate, position, FmMatrix3::identity(), FmInitVector3(0.0f));

        AddWoodPanelSimObjectToScene(&gWoodPanels[i], gScene);
    }
#endif

#if CARS_PANELS_TIRES_SCENE
    // Create tractor tire instances and place in scene
    uint numTireStacksPerInstance = NUM_TRACTOR_TIRE_STACKS;
    uint numTireStacks = numTireStacksPerInstance*gNumInstances;

    uint tireObjectIdx = 0;
    float tireStackSpacing = 4.0f;
    for (uint i = 0; i < numTireStacks; i++)
    {
        int instanceIdx = i / numTireStacksPerInstance;
        uint stackIdx = i % numTireStacksPerInstance;

        rotation = FmMatrix3::identity();
        FmVector3 basePosition = FmInitVector3((float)(instanceIdx - (int)(gNumInstances / 2)) * instanceSpacingX, TRACTOR_TIRE_HALF_DEPTH, woodPanelsMaxZ + tireStackSpacing + tireStackSpacing * stackIdx);

        for (uint levelIdx = 0; levelIdx < TRACTOR_TIRE_STACK_BASE; levelIdx++)
        {
            uint numOnLevel = TRACTOR_TIRE_STACK_BASE - levelIdx;
            //float height = TRACTOR_TIRE_HALF_DEPTH*2.0f * levelIdx;
            float gapY = TRACTOR_TIRE_HALF_DEPTH*2.0f;
            float height = gapY * levelIdx * 2.0f;
            float gapX = TRACTOR_TIRE_RADIUS * 2.05f;
            float minX = -0.5f * (numOnLevel - 1) * gapX;

            for (uint ti = 0; ti < 2; ti++)
            {
                for (uint tireIdx = 0; tireIdx < numOnLevel; tireIdx++)
                {
                    //FmVector3 position = basePosition + FmInitVector3(minX + gapX * tireIdx, height, 0.0f);
                    FmVector3 position = basePosition + FmInitVector3(minX + gapX * tireIdx, height + ti * gapY, 0.0f);
                    CreateTractorTireSimObject(&gTractorTires[tireObjectIdx], gTractorTireTemplate, position, rotation, FmInitVector3(0.0f));
                    AddTractorTireSimObjectToScene(&gTractorTires[tireObjectIdx], gScene);
                    tireObjectIdx++;
                }
            }
        }
    }
#endif

#if GLUE_TEST
    // Set up glue constraints between tet mesh and rigid body
    FmTetMesh& tetMeshA = *FmGetTetMesh(*FmGetTetMeshBuffer(*gScene, 0), 0);
    FmSetCollisionGroup(&tetMeshA, 0);

    FmRigidBody& rigidBodyB = *FmGetRigidBody(*gScene, 0 | FM_RB_FLAG);
    FmSetCollisionGroup(&rigidBodyB, 1);

    FmGlueConstraintSetupParams glueConstraintParams[8];

    {
        glueConstraintParams[0].bufferIdA = 0;
        glueConstraintParams[0].bufferIdB = 0 | FM_RB_FLAG;
        glueConstraintParams[0].bufferTetIdB = FM_INVALID_ID;
        glueConstraintParams[0].bufferTetIdA = 0;
        glueConstraintParams[0].posBaryA[0] = 1.0f;
        glueConstraintParams[0].posBaryA[1] = 0.0f;
        glueConstraintParams[0].posBaryA[2] = 0.0f;
        glueConstraintParams[0].posBaryA[3] = 0.0f;

        glueConstraintParams[1].bufferIdA = 0;
        glueConstraintParams[1].bufferIdB = 0 | FM_RB_FLAG;
        glueConstraintParams[1].bufferTetIdB = FM_INVALID_ID;
        glueConstraintParams[1].bufferTetIdA = 1;
        glueConstraintParams[1].posBaryA[0] = 1.0f;
        glueConstraintParams[1].posBaryA[1] = 0.0f;
        glueConstraintParams[1].posBaryA[2] = 0.0f;
        glueConstraintParams[1].posBaryA[3] = 0.0f;

        glueConstraintParams[2].bufferIdA = 0;
        glueConstraintParams[2].bufferIdB = 0 | FM_RB_FLAG;
        glueConstraintParams[2].bufferTetIdB = FM_INVALID_ID;
        glueConstraintParams[2].bufferTetIdA = 3;
        glueConstraintParams[2].posBaryA[0] = 0.0f;
        glueConstraintParams[2].posBaryA[1] = 1.0f;
        glueConstraintParams[2].posBaryA[2] = 0.0f;
        glueConstraintParams[2].posBaryA[3] = 0.0f;

        glueConstraintParams[3].bufferIdA = 0;
        glueConstraintParams[3].bufferIdB = 0 | FM_RB_FLAG;
        glueConstraintParams[3].bufferTetIdB = FM_INVALID_ID;
        glueConstraintParams[3].bufferTetIdA = 3;
        glueConstraintParams[3].posBaryA[0] = 0.0f;
        glueConstraintParams[3].posBaryA[1] = 0.0f;
        glueConstraintParams[3].posBaryA[2] = 1.0f;
        glueConstraintParams[3].posBaryA[3] = 0.0f;

        glueConstraintParams[4].bufferIdA = 0;
        glueConstraintParams[4].bufferIdB = 0 | FM_RB_FLAG;
        glueConstraintParams[4].bufferTetIdB = FM_INVALID_ID;
        glueConstraintParams[4].bufferTetIdA = 0;
        glueConstraintParams[4].posBaryA[0] = 0.0f;
        glueConstraintParams[4].posBaryA[1] = 0.0f;
        glueConstraintParams[4].posBaryA[2] = 0.0f;
        glueConstraintParams[4].posBaryA[3] = 1.0f;

        glueConstraintParams[5].bufferIdA = 0;
        glueConstraintParams[5].bufferIdB = 0 | FM_RB_FLAG;
        glueConstraintParams[5].bufferTetIdB = FM_INVALID_ID;
        glueConstraintParams[5].bufferTetIdA = 1;
        glueConstraintParams[5].posBaryA[0] = 0.0f;
        glueConstraintParams[5].posBaryA[1] = 0.0f;
        glueConstraintParams[5].posBaryA[2] = 0.0f;
        glueConstraintParams[5].posBaryA[3] = 1.0f;

        glueConstraintParams[6].bufferIdA = 0;
        glueConstraintParams[6].bufferIdB = 0 | FM_RB_FLAG;
        glueConstraintParams[6].bufferTetIdB = FM_INVALID_ID;
        glueConstraintParams[6].bufferTetIdA = 5;
        glueConstraintParams[6].posBaryA[0] = 0.0f;
        glueConstraintParams[6].posBaryA[1] = 0.0f;
        glueConstraintParams[6].posBaryA[2] = 0.0f;
        glueConstraintParams[6].posBaryA[3] = 1.0f;

        glueConstraintParams[7].bufferIdA = 0;
        glueConstraintParams[7].bufferIdB = 0 | FM_RB_FLAG;
        glueConstraintParams[7].bufferTetIdB = FM_INVALID_ID;
        glueConstraintParams[7].bufferTetIdA = 5;
        glueConstraintParams[7].posBaryA[0] = 0.0f;
        glueConstraintParams[7].posBaryA[1] = 1.0f;
        glueConstraintParams[7].posBaryA[2] = 0.0f;
        glueConstraintParams[7].posBaryA[3] = 0.0f;
    }

    for (uint i = 0; i < 8; i++)
    {
        FmGlueConstraintSetupParams& glueConstraint = glueConstraintParams[i];

        FmTetVertIds tetVertIdsA = FmGetTetVertIds(tetMeshA, glueConstraint.bufferTetIdA);

        FmVector3 worldPos = FmInterpolate(glueConstraint.posBaryA,
                FmGetVertPosition(tetMeshA, tetVertIdsA.ids[0]),
                FmGetVertPosition(tetMeshA, tetVertIdsA.ids[1]),
                FmGetVertPosition(tetMeshA, tetVertIdsA.ids[2]),
                FmGetVertPosition(tetMeshA, tetVertIdsA.ids[3]));

        FmRigidBodyState state = FmGetState(*FmGetRigidBody(*gScene, 0 | FM_RB_FLAG));
        FmVector3 posBodySpaceB = rotate(conj(state.quat), worldPos - state.pos);
        glueConstraint.posBodySpaceB[0] = posBodySpaceB.x;
        glueConstraint.posBodySpaceB[1] = posBodySpaceB.y;
        glueConstraint.posBodySpaceB[2] = posBodySpaceB.z;
        glueConstraint.posBodySpaceB[3] = 1.0f;

        FmAddGlueConstraintToScene(gScene, glueConstraint);
    }
#endif

#if FIX_INITIAL_CONDITIONS && WOOD_PANELS_SCENE
    FireProjectile(FmMatrix3::identity(), FmInitVector3(0.0f), 60.0f, true);
#endif
#if FIX_INITIAL_CONDITIONS && CARS_PANELS_TIRES_SCENE
    for (uint i = 0; i < gNumInstances; i++)
    {
        LaunchCarSimObject(i, 36.0f);
    }
#endif
}

void UpdateObjects()
{
    WriteTimings();

    if (!gScene)
    {
        return;
    }
#if WOOD_PANELS_SCENE && KINEMATIC_TEST
    const FmSceneControlParams& sceneParams = FmGetSceneControlParams(*gScene);

    for (uint i = 0; i < gNumWoodPanels; i++)
    {
        WoodPanelSimObject& simObject = gWoodPanels[i];
        for (uint mvIdx = 0; mvIdx < simObject.movingVerts.size(); mvIdx++)
        {
            uint meshVertId;
            uint bufferVertId;
            FmTetMesh* tetMesh = FmGetTetMeshContainingVert(&meshVertId, &bufferVertId, *simObject.tetMeshBuffer, simObject.movingVerts[mvIdx]);

            if (FmGetVertFlags(*tetMesh, meshVertId) & FM_VERT_FLAG_KINEMATIC)
            {
                float offset = GetZOffset(sceneParams.simTime);
                float nextOffset = GetZOffset(sceneParams.simTime + sceneParams.timestep);
                float goalVel = (nextOffset - offset) / sceneParams.timestep;

                FmVector3 pos = simObject.movingVertOrigPos[mvIdx];
                pos.z += offset;

                FmSetVertPosition(gScene, tetMesh, meshVertId, pos);
                FmSetVertVelocity(gScene, tetMesh, meshVertId, FmInitVector3(0.0f, 0.0f, goalVel));
            }
        }
    }
#endif
#if BLOCKS_SCENE && KINEMATIC_TEST
    const float period = 3.0f;
    const float amp = 1.0f;

    const FmSceneControlParams& params = FmGetSceneControlParams(*gScene);

    float offset = amp * sinf(2.0f * AMD_PI * (params.simTime / period));
    float nextOffset = amp * sinf(2.0f * AMD_PI * ((params.simTime + params.timestep) / period));

    FmVector3 vel = FmInitVector3(0.0f, (nextOffset - offset) / params.timestep, 0.0f);
    //FmVector3 vel = FmInitVector3(0.0f);

    FmTetMesh* tetMesh = FmGetTetMesh(*gScene, 0);

    uint numMeshVerts = FmGetNumVerts(*tetMesh);
    for (uint i = 0; i < numMeshVerts; i++)
    {
        FmSetVertVelocity(gScene, tetMesh, i, vel);
    }

#endif
#if RIGIDBODY_TEST_SCENE && KINEMATIC_TEST
    const FmSceneControlParams& params = FmGetSceneControlParams(*gScene);

    const float period = 3.0f;
    const float amp = 1.0f;

    float offset = amp * sinf(2.0f * AMD_PI * (params.simTime / period));
    float nextOffset = amp * sinf(2.0f * AMD_PI * ((params.simTime + params.timestep) / period));

    FmRigidBody* rigidBody = FmGetRigidBody(*gScene, 0 | FM_RB_FLAG);
    FmRigidBodyState state = FmGetState(*rigidBody);
    state.vel = FmInitVector3(0.0f, (nextOffset - offset) / params.timestep, 0.0f);

    FmSetState(gScene, rigidBody, state);
#endif
#if CARS_IN_SCENE
    for (uint carIdx = 0; carIdx < gNumCars; carIdx++)
    {
        CarSimObject& car = gCars[carIdx];

        for (uint wheelIdx = 0; wheelIdx < 4; wheelIdx++)
        {
            float relAngVel = FmComputeHingeRelAngVel(*gScene, car.axelAngleConstraints[wheelIdx]);

            if (fabsf(relAngVel) < 10.0f)
            {
                FmSetRigidBodyAngleConstraintFrictionCoeff(gScene, car.axelAngleConstraints[wheelIdx], 0.9f);
            }
            else
            {
                FmSetRigidBodyAngleConstraintFrictionCoeff(gScene, car.axelAngleConstraints[wheelIdx], 0.6f);
            }
        }

        {
            uint planeConstraintId = car.planeConstraints[0];
            FmPlaneConstraintSetupParams planeConstraint = FmGetPlaneConstraintParams(*gScene, planeConstraintId);

            uint tetId, meshIdx;
            FmTetMesh* tetMesh = FmGetTetMeshContainingTet(&tetId, &meshIdx, *FmGetTetMeshBuffer(*gScene, car.tetMeshBufferIds[0]), planeConstraint.bufferTetIdB);

            FmVector3 planeNormal0 = mul(FmGetTetRotation(*tetMesh, tetId), FmInitVector3(0.0f, 1.0f, 0.0f));
            FmSetPlaneConstraintNormals(gScene, planeConstraintId, planeNormal0, planeNormal0, planeNormal0);
        }

        uint engineConstraintsStartIdx = 1;
        uint cabinConstraintsStartIdx = 1 + 5;

        // Get rotation of the constraint point in engine compartment
        FmVector3 engineConstraintPosition;
        FmMatrix3 engineConstraintRotation;
        {
            FmPlaneConstraintSetupParams planeConstraint = FmGetPlaneConstraintParams(*gScene, car.planeConstraints[engineConstraintsStartIdx]);

            uint tetId, meshIdx;
            FmTetMesh* tetMesh = FmGetTetMeshContainingTet(&tetId, &meshIdx, *FmGetTetMeshBuffer(*gScene, car.tetMeshBufferIds[0]), planeConstraint.bufferTetIdB);

            engineConstraintPosition = FmGetInterpolatedPosition(planeConstraint.posBaryB, *tetMesh, tetId);
            engineConstraintRotation = FmGetTetRotation(*tetMesh, tetId);
        }

        FmVector3 cabinConstraintPosition;
        FmMatrix3 cabinConstraintRotation;
        {
            FmPlaneConstraintSetupParams planeConstraint = FmGetPlaneConstraintParams(*gScene, car.planeConstraints[cabinConstraintsStartIdx]);

            uint tetId, meshIdx;
            FmTetMesh* tetMesh = FmGetTetMeshContainingTet(&tetId, &meshIdx, *FmGetTetMeshBuffer(*gScene, car.tetMeshBufferIds[0]), planeConstraint.bufferTetIdB);

            cabinConstraintPosition = FmGetInterpolatedPosition(planeConstraint.posBaryB, *tetMesh, tetId);
            cabinConstraintRotation = FmGetTetRotation(*tetMesh, tetId);
        }

        for (uint i = 0; i < 5; i++)
        {
            uint planeConstraintId = car.planeConstraints[engineConstraintsStartIdx + i];
            FmPlaneConstraintSetupParams planeConstraint = FmGetPlaneConstraintParams(*gScene, planeConstraintId);

            FmSetPlaneConstraintNormals(gScene, planeConstraintId, engineConstraintRotation.col1, engineConstraintRotation.col1, engineConstraintRotation.col1);

            // Enable some of the constraints only if the glue is broken
            if (i == 1)
            {
                if (FmGetGlueConstraintEnabled(*gScene, car.bodyHoodGlueConstraints[1]))
                {
                    FmEnablePlaneConstraint(gScene, planeConstraintId, false);
                    continue;
                }
            }
            else if (i == 2)
            {
                if (FmGetGlueConstraintEnabled(*gScene, car.bodyHoodGlueConstraints[0]))
                {
                    FmEnablePlaneConstraint(gScene, planeConstraintId, false);
                    continue;
                }
            }

            uint tetId, meshIdx;
            FmTetMesh* hoodTetMesh = FmGetTetMeshContainingTet(&tetId, &meshIdx, *FmGetTetMeshBuffer(*gScene, car.tetMeshBufferIds[1]), planeConstraint.bufferTetIdA);

            FmVector3 hoodPoint = FmGetInterpolatedPosition(planeConstraint.posBaryA, *hoodTetMesh, tetId);

            // Apply constraint only if hood point within a box near point on the car body
            FmVector3 hoodPointEngine = mul(transpose(engineConstraintRotation), hoodPoint - engineConstraintPosition);

            if (hoodPointEngine.x > -0.75f && hoodPointEngine.x < 0.75f
                && hoodPointEngine.y > 0.05f && hoodPointEngine.y <= planeConstraint.bias0 * 1.25f
                && hoodPointEngine.z > -0.75f && hoodPointEngine.z < 0.75f)
            {
                FmEnablePlaneConstraint(gScene, planeConstraintId, true);
            }
            else
            {
                FmEnablePlaneConstraint(gScene, planeConstraintId, false);
            }
        }

        for (uint i = 0; i < 5; i++)
        {
            uint planeConstraintId = car.planeConstraints[cabinConstraintsStartIdx + i];

            FmPlaneConstraintSetupParams planeConstraint = FmGetPlaneConstraintParams(*gScene, planeConstraintId);

            FmSetPlaneConstraintNormals(gScene, planeConstraintId, cabinConstraintRotation.col1, cabinConstraintRotation.col1, cabinConstraintRotation.col1);

            // Enable some of the constraints only if the glue is broken
            if (i == 1)
            {
                if (FmGetGlueConstraintEnabled(*gScene, car.bodyHoodGlueConstraints[1]))
                {
                    FmEnablePlaneConstraint(gScene, planeConstraintId, false);
                    continue;
                }
            }
            else if (i == 2)
            {
                if (FmGetGlueConstraintEnabled(*gScene, car.bodyHoodGlueConstraints[0]))
                {
                    FmEnablePlaneConstraint(gScene, planeConstraintId, false);
                    continue;
                }
            }

            uint tetId, meshIdx;
            FmTetMesh* hoodTetMesh = FmGetTetMeshContainingTet(&tetId, &meshIdx, *FmGetTetMeshBuffer(*gScene, car.tetMeshBufferIds[1]), planeConstraint.bufferTetIdA);

            FmVector3 hoodPoint = FmGetInterpolatedPosition(planeConstraint.posBaryA, *hoodTetMesh, tetId);

            // Apply constraint only if hood point within a box near point on the car body
            FmVector3 hoodPointCabin = mul(transpose(cabinConstraintRotation), hoodPoint - cabinConstraintPosition);

            if (hoodPointCabin.x > -0.75f && hoodPointCabin.x < 0.75f
                && hoodPointCabin.y > 0.05f && hoodPointCabin.y <= planeConstraint.bias0 * 1.25f
                && hoodPointCabin.z > -0.75f && hoodPointCabin.z < 0.75f)
            {
                FmEnablePlaneConstraint(gScene, planeConstraintId, true);
            }
            else
            {
                FmEnablePlaneConstraint(gScene, planeConstraintId, false);
            }
        }
    }
#endif
}

void FreeScene()
{
#if EXTERNAL_RIGIDBODIES
    if (gRbScene)
    {
        delete gRbScene;
        gRbScene = NULL;
    }
#endif

    if (gScene)
    {
#if FM_TIMINGS
        if (gTimingsFile)
        {
            fclose(gTimingsFile);
            gTimingsFile = NULL;
        }
#endif

        for (uint meshBufferIdx = 0; meshBufferIdx < gNumTetMeshBuffers; meshBufferIdx++)
        {
            FmDestroyTetMeshBuffer(gTetMeshBuffers[meshBufferIdx]);
            gTetMeshBuffers[meshBufferIdx] = NULL;
        }
        gNumTetMeshBuffers = 0;

        for (uint rbIdx = 0; rbIdx < gNumRigidBodies; rbIdx++)
        {
            FmDestroyRigidBody(gRigidBodies[rbIdx]);
            gRigidBodies[rbIdx] = NULL;
        }
        gNumRigidBodies = 0;

        FmDestroyScene(gScene);
        gScene = NULL;
    }

#if CARS_IN_SCENE
    delete[] gCars;
    FreeCarSimObjectTemplate(&gCarTemplate);
    for (uint i = 0; i < NUM_CAR_TEMPLATES; i++)
    {
        gCarTemplates[i] = NULL;
    }
#endif
#if WOOD_PANELS_IN_SCENE
    FreeWoodPanelSimObjectTemplate(&gWoodPanelTemplate);
#endif
#if CARS_PANELS_TIRES_SCENE
    FreeTractorTireSimObjectTemplate(&gTractorTireTemplate);
#endif

    SampleDestroyTaskSystem();
}
