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
// Container for data loaded from .FEM file.
// Adapted from UE4 plugin code.
//---------------------------------------------------------------------------------------
#pragma once

#include "AMD_FEMFX.h"
#include "FEMFXArray.h"

#include <vector>
#include <string>

namespace AMD
{
    struct FVector
    {
        float X, Y, Z;

        FVector()
        {
        }
        FVector(float inX, float inY, float inZ)
        {
            X = inX;
            Y = inY;
            Z = inZ;
        }
    };

    struct FVector4
    {
        float X, Y, Z, W;

        FVector4()
        {
        }
        FVector4(float inX, float inY, float inZ, float inW)
        {
            X = inX;
            Y = inY;
            Z = inZ;
            W = inW;
        }
    };

    /**
    * This class is going to mimick the .fem file structure
    */
    struct FTet
    {
        int TetIndex;
        std::vector<int> Indices;
    };


    struct FNodeResource
    {
        bool IsBoundaryMarker;
        int NumAttributes;
        int NumDimensions;
        int NumPoints;
        std::vector<float> Data;
    };

    struct FEleResource
    {
        bool IsRegionAttribute;
        int NumNodesPerTets;
        int NumTetrahedra;
        std::vector<FTet> Data;
    };

    struct FNameIndexMap
    {
        std::string Name;
        std::vector<uint32_t> TetIds;
        std::vector<uint32_t> NoFractureFaces;

        FNameIndexMap()
        {
            Name = "";
        }
    };

    struct FRigidBody
    {
        std::string Name;
        FVector Position;
        FVector Dimensions;
        FVector4 Rotation;
        float mass;
        std::vector<float> BodyInertiaTensor;
    };

    struct FAngleConstraint
    {
        std::string Name;
        unsigned int BodyA;
        unsigned int BodyB;
        FVector AxisBodySpaceA;
        FVector AxisBodySpaceB;
    };

    struct FGlueConstraint
    {
        std::string Name;
        unsigned int BodyA;
        bool IsRigidBodyA;
        unsigned int BodyB;
        bool IsRigidBodyB;
        FVector4 PosBodySpaceA;
        FVector4 PosBodySpaceB;
        unsigned int TetIdA;
        unsigned int TetIdB;
        float BreakThreshold;
        unsigned int MinGlueConstraints;
    };

    struct FFEMPlane
    {
        float Bias;
        bool NonNegative;
        FVector PlaneNormal;
    };

    struct FPlaneConstraint : public FGlueConstraint
    {
        int NumberOfPlanes;
        std::vector<FFEMPlane> Planes;
    };


    struct FComponent
    {
        int32_t NumberOfCornersPerShard;
        std::string Name;
        bool IsFracturable;
        int CollisionGroup;
        int NumTags;
        std::vector<FNameIndexMap> Tags;
        int NumMaterials;
        std::vector<FNameIndexMap> Materials;
        FEleResource EleFile;
        FNodeResource NodeFile;
        int NumFBXFiles;
        std::vector<std::string> FBXFiles;
        std::vector<int> AssignedTetFace;
        std::vector<float> Barycentrics;
        std::vector<int32_t> BarycentricsPosIds;
        std::vector<int> TetAssignment;
        std::vector<float> VertexColor;
        std::vector<float> VertexNormal;
        std::vector<float> VertexPosition;
        std::vector<float> VertexTangent;
        std::vector<float> VertexUVs;
        std::vector<int32_t> ShardIds;
        std::vector<int32_t> Triangles;
        std::vector<float> Centroids;
        int32_t NumberOfShards;
    };

    struct FTetVertIds
    {
        unsigned int Ids[4];
    };

    struct FTetIdxToMaterial
    {
        unsigned int tetIndex;

    };

    struct FBVSizes
    {
        int totalSize;
        int nodeSize;
        int primBoxesSize;
        int boxesSize;
        int mortonCodesSize;
        int mortonCodesSortedSize;
        int primIndicesSortedSize;
        int numPrimsSize;

        FBVSizes()
        {
            totalSize = 0;
            nodeSize = 0;
            primBoxesSize = 0;
            boxesSize = 0;
            mortonCodesSize = 0;
            mortonCodesSortedSize = 0;
            primIndicesSortedSize = 0;
            numPrimsSize = sizeof(unsigned int);
        }

        void CalculateTotalSize()
        {
            totalSize += nodeSize;
            totalSize += primBoxesSize;
            totalSize += boxesSize;
            totalSize += mortonCodesSize;
            totalSize += mortonCodesSortedSize;
            totalSize += primIndicesSortedSize;
            totalSize += numPrimsSize;
        }
    };

    struct FTetMeshBufferInfo
    {
        uint32_t pBufferOffset;
        uint32_t tetMeshOffset;
        uint32_t solverDataOffset;
        uint32_t tetMeshVertOffsetsOffset;
        uint32_t vertReferenceOffset;
        uint32_t tetReferenceOffset;
    };

    struct FComponentResources
    {
        int32_t NumberOfCornersPerShard;
        std::string Name;
        int CollisionGroup;
        std::vector<FNameIndexMap> Tags;
        std::vector<FNameIndexMap> Materials;
        std::vector<std::string> FBXFiles;
        int NumVerts;
        int NumTets;
        float RestVolume;
        FVector minPos;
        FVector maxPos;
        int MaxVerts;
        std::vector<uint8_t> restPositions;
        std::vector<uint8_t> tetVertIds;
        std::vector<unsigned int> vertIncidentTets;
        std::vector<int> VertexIndices;
        std::vector<int> AssignedTetFace;
        std::vector<float> Barycentrics;
        std::vector<int> TetAssignment;
        std::vector<int> BarycentricsPosIds;
        std::vector<float> VertexColor;
        std::vector<float> VertexNormal;
        std::vector<float> VertexPosition;
        std::vector<float> VertexTangent;
        std::vector<float> VertexUVs;
        std::vector<int> ShardIds;
        std::vector<int> Triangles;
        std::vector<float> Centroids;
        AMD::FmArray<unsigned int> VertPermutation;
        AMD::FmArray<unsigned int> TetPermutation;
        int32_t NumberOfShards;
    };

    struct FActorResource
    {
        std::vector<FRigidBody> RigidBodies;
        std::vector<FAngleConstraint> AngleConstraints;
        std::vector<FGlueConstraint> GlueConstraints;
        std::vector<FPlaneConstraint> PlaneConstraints;
    };

    class FEMResource
    {
    public:
        std::string Version;
        std::vector<FComponentResources> ComponentResources;
        FActorResource ActorResource;
        static FComponentResources ProcessResource(
            std::vector<unsigned int>& vertPermutation, std::vector<unsigned int>& tetPermutation,
            AMD::FmVector3* restPositions, AMD::FmTetVertIds* tetVertIds, std::vector<unsigned int>* vertIncidentTets, int numVerts, int numTets);
        void ProcessResource();
        void AddComponent(FComponent comp);

    private:

        std::vector<FComponent> Components;
    };

}