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
// Code to parse .FEM file and load data into a FEMResource.
// Adapted from UE4 plugin code.
//---------------------------------------------------------------------------------------
#include "LoadFemFile.h"
#include "FEMResource.h"

#include <fstream>
#include <sstream>
#include <vector>
#pragma warning(push, 0)
#include "nlohmann\json.hpp"
#pragma warning(pop)

using namespace std;
using json = nlohmann::json;

namespace AMD
{
    bool LoadFEMFile_v1_0(
        FEMResource* Resource,
        const char* dotFEMFile)
    {
        ifstream inputStream(dotFEMFile);

        if (inputStream.fail())
        {
            return false;
        }

        json JsonObject;
        inputStream >> JsonObject;

        std::string version = JsonObject["Version"].get<std::string>();

        std::istringstream versionStream(version);

        string subString;
        getline(versionStream, subString, '.');
        uint32_t majorVersion = stoi(subString);

        getline(versionStream, subString, '.');
        uint32_t minorVersion = stoi(subString);

        if (!(majorVersion == 1 && minorVersion == 0))
        {
            return false;
        }

        std::vector<json> JsonMeshComponents = JsonObject["FEMMeshComponents"];

        for (int idx = 0; idx < (int)JsonMeshComponents.size(); ++idx)
        {
            FComponent component;

            component.Name = JsonMeshComponents[idx]["Name"].get<std::string>();
            component.NumFBXFiles = JsonMeshComponents[idx]["NumFBXFiles"].get<int>();
            if (JsonMeshComponents[idx].find("NumCornersPerShard") == JsonMeshComponents[idx].end())
            {
                component.NumberOfCornersPerShard = 0;
            }
            else
            {
                component.NumberOfCornersPerShard = JsonMeshComponents[idx]["NumCornersPerShard"].get<int>();
            }
            component.CollisionGroup = JsonMeshComponents[idx]["CollisionGroup"].get<int>();
            component.IsFracturable = JsonMeshComponents[idx]["IsFracturable"].get<int>();
            component.NumTags = JsonMeshComponents[idx]["NumTags"].get<int>();
            component.NumMaterials = JsonMeshComponents[idx]["NumMaterials"].get<int>();

            std::vector<json> Tags = JsonMeshComponents[idx]["Tags"];
            for (int i = 0; i < (int)Tags.size(); i++)
            {
                FNameIndexMap map;
                map.Name = Tags[i]["Tag"].get<std::string>();

                std::vector<json> ids = Tags[i]["TetIds"];
                for (int j = 0; j < (int)ids.size(); ++j)
                {
                    map.TetIds.push_back(ids[j].get<int>());
                }
                component.Tags.push_back(map);
            }

            std::vector<json> Materials = JsonMeshComponents[idx]["Materials"];
            for (int i = 0; i < (int)Materials.size(); i++)
            {
                FNameIndexMap map;
                map.Name = Materials[i]["MaterialName"].get<std::string>();

                std::vector<json> ids = Materials[i]["TetIds"];

                for (int j = 0; j < (int)ids.size(); ++j)
                {
                    map.TetIds.push_back(ids[j].get<int>());
                }

                std::vector<json> faces = Materials[i]["NoFractureFaces"];

                for (int j = 0; j < (int)faces.size(); ++j)
                {
                    map.NoFractureFaces.push_back(faces[j].get<int>());
                }

                component.Materials.push_back(map);
            }

#if 0
            std::vector<json> FBXFiles = JsonMeshComponents[idx]["FbxFiles"];
            for (int i = 0; i < (int)FBXFiles.size(); i++)
            {
                component.FBXFiles.push_back(FBXFiles[i].get<std::string>());
            }
#endif

            FNodeResource node;
            json JsonNode = JsonMeshComponents[idx]["Node"];
            node.IsBoundaryMarker = JsonNode["IsBoundaryMarker"].get<int>();
            node.NumAttributes = JsonNode["NumAttributes"].get<int>();
            node.NumDimensions = JsonNode["NumDimensions"].get<int>();
            node.NumPoints = JsonNode["NumPoints"].get<int>();
            std::vector<json> points = JsonNode["Data"];
            for (int i = 0; i < node.NumPoints; i++)
            {
                for (int j = 0; j < node.NumDimensions + 1; ++j)
                {
                    node.Data.push_back(points[i * 4 + j].get<float>());
                }
            }

            component.NodeFile = node;

            FEleResource ele;
            json JsonEle = JsonMeshComponents[idx]["Ele"];
            ele.IsRegionAttribute = JsonEle["IsRegionAttribute"].get<int>();
            ele.NumNodesPerTets = JsonEle["NumNodesPerTets"].get<int>();
            ele.NumTetrahedra = JsonEle["NumTetrahedra"].get<int>();
            std::vector<json> tets = JsonEle["Data"];
            for (int i = 0; i < ele.NumTetrahedra; i++)
            {
                json JsonTet = tets[i];
                FTet tet;
                tet.TetIndex = JsonTet["TetIndex"].get<int>();

                std::vector<json> indicies = JsonTet["Indices"];
                for (int j = 0; j < ele.NumNodesPerTets; j++)
                {
                    tet.Indices.push_back(indicies[j].get<int>());
                }

                ele.Data.push_back(tet);
            }

            component.EleFile = ele;

#if 0
            json JsonRenderMesh = JsonMeshComponents[idx]["RenderMesh"];

            std::vector<json> assignedTetFace = JsonRenderMesh["AssignedTetFaceBuffer"];
            for (int i = 0; i < (int)assignedTetFace.size(); i++)
            {
                component.AssignedTetFace.push_back(assignedTetFace[i].get<int>());
            }

            std::vector<json> barycentrics = JsonRenderMesh["BarycentricCoordsBuffer"];
            for (int i = 0; i < (int)barycentrics.size(); i++)
            {
                component.Barycentrics.push_back(barycentrics[i].get<float>());
            }

            std::vector<json> barycentricsPosIds = JsonRenderMesh["BarycentricPosIds"];
            for (int i = 0; i < (int)barycentricsPosIds.size(); i++)
            {
                component.BarycentricsPosIds.push_back(barycentricsPosIds[i].get<int>());
            }

            std::vector<json> tetAssignment = JsonRenderMesh["TetAssignmentBuffer"];
            for (int i = 0; i < (int)tetAssignment.size(); i++)
            {
                component.TetAssignment.push_back(tetAssignment[i].get<int>());
            }

            std::vector<json> vertColor = JsonRenderMesh["ColorBuffer"];
            for (int i = 0; i < (int)vertColor.size(); i++)
            {
                component.VertexColor.push_back(vertColor[i].get<float>());
            }

            std::vector<json> vertNormal = JsonRenderMesh["NormalBuffer"];
            for (int i = 0; i < (int)vertNormal.size(); i++)
            {
                component.VertexNormal.push_back(vertNormal[i].get<float>());
            }

            std::vector<json> vertPos = JsonRenderMesh["PositionBuffer"];
            for (int i = 0; i < (int)vertPos.size(); i++)
            {
                component.VertexPosition.push_back(vertPos[i].get<float>());
            }

            std::vector<json> vertTangent = JsonRenderMesh["TangentBuffer"];
            for (int i = 0; i < (int)vertTangent.size(); i++)
            {
                component.VertexTangent.push_back(vertTangent[i].get<float>());
            }

            std::vector<json> vertUVs = JsonRenderMesh["UVsBuffer"];
            for (int i = 0; i < (int)vertUVs.size(); i++)
            {
                component.VertexUVs.push_back(vertUVs[i].get<float>());
            }

            std::vector<json> shardIds = JsonRenderMesh["ShardIds"];
            for (int i = 0; i < (int)shardIds.size(); i++)
            {
                component.ShardIds.push_back(shardIds[i].get<int>());
            }

            std::vector<json> triangles = JsonRenderMesh["Triangles"];
            for (int i = 0; i < (int)triangles.size(); i++)
            {
                component.Triangles.push_back(triangles[i].get<int>());
            }

            std::vector<json> centroids = JsonRenderMesh["Centroids"];
            for (int i = 0; i < (int)centroids.size(); i++)
            {
                component.Centroids.push_back(centroids[i].get<float>());
            }

            component.NumberOfShards = JsonRenderMesh["NumberOfShards"].get<int>();
#endif

            Resource->AddComponent(component);
        }

        // Setup The Component Resource
        Resource->ProcessResource();

        // Read Rigid Body values out from file
        std::vector<json> JsonRigidBodyArray = JsonObject["RigidBodies"];
        for (auto it = JsonRigidBodyArray.begin(); it != JsonRigidBodyArray.end(); it++)
        {
            FRigidBody rb;
            json obj = *it;
            rb.mass = obj["Mass"].get<float>();

            std::vector<json> Pos = obj["Position"];
            FVector position = FVector(Pos[0].get<float>(), Pos[1].get<float>(), Pos[2].get<float>());
            rb.Position = position;

            std::vector<json> Dim = obj["Dimensions"];
            FVector dimension = FVector(Dim[0].get<float>(), Dim[1].get<float>(), Dim[2].get<float>());
            rb.Dimensions = FVector(dimension.X * 0.5f, dimension.Y * 0.5f, dimension.Z * 0.5f); // Library dimensions are half-widths

            std::vector<json> Rot = obj["Rotation"];
            FVector4 rotation = FVector4(Rot[0].get<float>(), Rot[1].get<float>(), Rot[2].get<float>(), Rot[3].get<float>());
            rb.Rotation = rotation;

            std::vector<json> bit = obj["BodyInertiaTensor"];
            for (auto itr = bit.begin(); itr != bit.end(); itr++)
            {
                rb.BodyInertiaTensor.push_back(itr->get<float>());
            }

            Resource->ActorResource.RigidBodies.push_back(rb);
        }

        // Read Angle Constraints out from file
        std::vector<json> JsonAngleConstraints = JsonObject["RBAngleConstraints"];
        for (auto it = JsonAngleConstraints.begin(); it != JsonAngleConstraints.end(); it++)
        {
            FAngleConstraint ac;
            json obj = *it;

            if (obj.find("Name") != obj.end())
                ac.Name = obj["Name"].get<std::string>();

            ac.BodyA = obj["BodyA"].get<int>();
            ac.BodyB = obj["BodyB"].get<int>();

            std::vector<json> absA = obj["AxisBodySpaceA"];
            ac.AxisBodySpaceA = FVector(absA[0].get<float>(), absA[1].get<float>(), absA[2].get<float>());

            std::vector<json> absB = obj["AxisBodySpaceB"];
            ac.AxisBodySpaceB = FVector(absB[0].get<float>(), absB[1].get<float>(), absB[2].get<float>());

            Resource->ActorResource.AngleConstraints.push_back(ac);
        }

        // Read Glue Constraints
        std::vector<json> JsonGlueConstraints = JsonObject["GlueConstraints"];
        for (auto it = JsonGlueConstraints.begin(); it != JsonGlueConstraints.end(); it++)
        {
            FGlueConstraint gc;
            json obj = *it;

            if (obj.find("Name") != obj.end())
                gc.Name = obj["Name"].get<std::string>();
            gc.BodyA = obj["BodyA"].get<int>();
            gc.BodyB = obj["BodyB"].get<int>();
            gc.IsRigidBodyA = obj["IsRigidBodyA"].get<int>();
            gc.IsRigidBodyB = obj["IsRigidBodyB"].get<int>();

            int tetIdA = obj["TetIdA"].get<int>();
            int tetIdB = obj["TetIdB"].get<int>();

            if (gc.IsRigidBodyA)
            {
                gc.TetIdA = (uint)-1;
            }
            else
            {
                gc.TetIdA = Resource->ComponentResources[gc.BodyA].TetPermutation[tetIdA];
            }

            if (gc.IsRigidBodyB)
            {
                gc.TetIdB = (uint)-1;
            }
            else
            {
                gc.TetIdB = Resource->ComponentResources[gc.BodyB].TetPermutation[tetIdB];
            }

            gc.BreakThreshold = obj["BreakThreshold"].get<float>();
            gc.MinGlueConstraints = obj["MinGlueConstraints"].get<unsigned int>();

            std::vector<json> pbsA = obj["PosBodySpaceA"];
            gc.PosBodySpaceA = FVector4(pbsA[0].get<float>(), pbsA[1].get<float>(), pbsA[2].get<float>(), pbsA[3].get<float>());

            std::vector<json> pbsB = obj["PosBodySpaceB"];
            gc.PosBodySpaceB = FVector4(pbsB[0].get<float>(), pbsB[1].get<float>(), pbsB[2].get<float>(), pbsB[3].get<float>());


            Resource->ActorResource.GlueConstraints.push_back(gc);
        }

        // Read Plane Constraints
        std::vector<json> JsonPlaneConstraints = JsonObject["PlaneConstraints"];
        for (auto it = JsonPlaneConstraints.begin(); it != JsonPlaneConstraints.end(); it++)
        {
            FPlaneConstraint pc;
            json obj = *it;

            if (obj.find("Name") != obj.end())
                pc.Name = obj["Name"].get<std::string>();
            pc.BodyA = obj["BodyA"].get<int>();
            pc.BodyB = obj["BodyB"].get<int>();
            pc.IsRigidBodyA = obj["IsRigidBodyA"].get<int>();
            pc.IsRigidBodyB = obj["IsRigidBodyB"].get<int>();
            pc.TetIdA = Resource->ComponentResources[pc.BodyA].TetPermutation[obj["TetIdA"].get<int>()];
            pc.TetIdB = Resource->ComponentResources[pc.BodyB].TetPermutation[obj["TetIdB"].get<int>()];

            std::vector<json> pbsA = obj["PosBodySpaceA"];
            pc.PosBodySpaceA = FVector4(pbsA[0].get<float>(), pbsA[1].get<float>(), pbsA[2].get<float>(), pbsA[3].get<float>());

            std::vector<json> pbsB = obj["PosBodySpaceB"];
            pc.PosBodySpaceB = FVector4(pbsB[0].get<float>(), pbsB[1].get<float>(), pbsB[2].get<float>(), pbsB[3].get<float>());

            pc.NumberOfPlanes = obj["NumberOfPlanes"].get<int>();
            std::vector<json> PlanesArray = obj["Planes"];
            for (auto itr = PlanesArray.begin(); itr != PlanesArray.end(); itr++)
            {
                FFEMPlane plane;
                json planeObj = *itr;

                plane.Bias = planeObj["Bias"].get<float>();
                plane.NonNegative = planeObj["NonNegative"].get<int>();

                std::vector<json> norm = planeObj["PlaneNormal"];
                plane.PlaneNormal = FVector(norm[0].get<float>(), norm[1].get<float>(), norm[2].get<float>());

                pc.Planes.push_back(plane);
            }

            Resource->ActorResource.PlaneConstraints.push_back(pc);
        }

        return true;
    }
}