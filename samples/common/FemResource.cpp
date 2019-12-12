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

#include "FemResource.h"
#include "PartitionFemMesh.h"

namespace AMD
{

    void FEMResource::ProcessResource()
    {

        for (int compIdx = 0; compIdx < (int)Components.size(); ++compIdx)
        {

            FComponentResources comp;

            FNodeResource& nodeResource = Components[compIdx].NodeFile;
            FEleResource& eleResource = Components[compIdx].EleFile;

            comp.Tags = Components[compIdx].Tags;
            comp.Materials = Components[compIdx].Materials;
            comp.NumberOfCornersPerShard = Components[compIdx].NumberOfCornersPerShard;
            comp.FBXFiles = Components[compIdx].FBXFiles;
            comp.Name = Components[compIdx].Name;
            comp.CollisionGroup = Components[compIdx].CollisionGroup;
            comp.NumVerts = nodeResource.NumPoints;
            comp.NumTets = eleResource.NumTetrahedra;
            comp.AssignedTetFace = Components[compIdx].AssignedTetFace;
            comp.Barycentrics = Components[compIdx].Barycentrics;
            comp.BarycentricsPosIds = Components[compIdx].BarycentricsPosIds;
            comp.TetAssignment = Components[compIdx].TetAssignment;
            comp.VertexColor = Components[compIdx].VertexColor;
            comp.VertexNormal = Components[compIdx].VertexNormal;
            comp.VertexPosition = Components[compIdx].VertexPosition;
            comp.VertexTangent = Components[compIdx].VertexTangent;
            comp.VertexUVs = Components[compIdx].VertexUVs;
            comp.ShardIds = Components[compIdx].ShardIds;
            comp.Triangles = Components[compIdx].Triangles;
            comp.Centroids = Components[compIdx].Centroids;
            comp.NumberOfShards = Components[compIdx].NumberOfShards;

            AMD::FmVector3* restPositions = new AMD::FmVector3[comp.NumVerts];
            AMD::FmTetVertIds* tetVertIds = new AMD::FmTetVertIds[comp.NumTets];
            AMD::FmArray<unsigned int>* vertIncidentTets = new AMD::FmArray<unsigned int>[comp.NumVerts];

            for (int i = 0; i < nodeResource.Data.size(); i += 4)
            {
                // This is part of the code that is hard coded to only support 3 dimensional tets
                //int numDimensions = nodeResource.NumDimensions;
                restPositions[i / 4].x = nodeResource.Data[i + 1];
                restPositions[i / 4].y = nodeResource.Data[i + 2];
                restPositions[i / 4].z = nodeResource.Data[i + 3];
            }

            for (int eleTetIndex = 0; eleTetIndex < comp.NumTets; ++eleTetIndex)
            {
                FTet& tet = eleResource.Data[eleTetIndex];
                int nodeIdx0, nodeIdx1, nodeIdx2, nodeIdx3;

                nodeIdx0 = tet.Indices[0];
                nodeIdx1 = tet.Indices[1];
                nodeIdx2 = tet.Indices[2];
                nodeIdx3 = tet.Indices[3];

                // number from 0
                nodeIdx0--;
                nodeIdx1--;
                nodeIdx2--;
                nodeIdx3--;

                if (nodeIdx0 < 0 || nodeIdx0 >= comp.NumVerts
                    || nodeIdx1 < 0 || nodeIdx1 >= comp.NumVerts
                    || nodeIdx2 < 0 || nodeIdx2 >= comp.NumVerts
                    || nodeIdx3 < 0 || nodeIdx3 >= comp.NumVerts)
                {
                    // Something Is Very Wrong
                    return;
                }

                FmAddIncidentTetToSet(vertIncidentTets[nodeIdx0], eleTetIndex);
                FmAddIncidentTetToSet(vertIncidentTets[nodeIdx1], eleTetIndex);
                FmAddIncidentTetToSet(vertIncidentTets[nodeIdx2], eleTetIndex);
                FmAddIncidentTetToSet(vertIncidentTets[nodeIdx3], eleTetIndex);

                tetVertIds[eleTetIndex].ids[0] = nodeIdx0;
                tetVertIds[eleTetIndex].ids[1] = nodeIdx1;
                tetVertIds[eleTetIndex].ids[2] = nodeIdx2;
                tetVertIds[eleTetIndex].ids[3] = nodeIdx3;
            }

#if PARTITION_MESH
            if (!(comp.Tags.size() > 0))
            {
                SortByPartition(comp.VertPermutation, comp.TetPermutation, restPositions, tetVertIds, vertIncidentTets, comp.NumVerts, comp.NumTets);
            }
            else
#endif
            {
                comp.VertPermutation.Clear();
                for (int i = 0; i < comp.NumVerts; i++)
                {
                    comp.VertPermutation.Add(i);
                }

                comp.TetPermutation.Clear();
                for (int i = 0; i < comp.NumTets; i++)
                {
                    comp.TetPermutation.Add(i);
                }
            }

            comp.minPos = FVector();
            comp.maxPos = FVector();
            AMD::FmVector3 minP = restPositions[0];
            AMD::FmVector3 maxP = restPositions[0];

            for (int vIdx = 0; vIdx < comp.NumVerts; vIdx++)
            {
                minP = min(minP, restPositions[vIdx]);
                maxP = max(maxP, restPositions[vIdx]);
            }

            comp.minPos.X = minP.x;
            comp.minPos.Y = minP.y;
            comp.minPos.Z = minP.z;

            comp.maxPos.X = maxP.x;
            comp.maxPos.Y = maxP.y;
            comp.maxPos.Z = maxP.z;

            comp.RestVolume = AMD::FmComputeTetMeshVolume(restPositions, tetVertIds, comp.NumTets);

            int restPosSize = sizeof(AMD::FmVector3) * comp.NumVerts;
            int tetVertIdsSize = sizeof(AMD::FmTetVertIds) * comp.NumTets;

            comp.restPositions.reserve(restPosSize);
            for (int i = 0; i < restPosSize; i++) comp.restPositions.push_back(0);

            memcpy(comp.restPositions.data(), restPositions, restPosSize);

            comp.tetVertIds.reserve(tetVertIdsSize);
            for (int i = 0; i < tetVertIdsSize; i++) comp.tetVertIds.push_back(0);
            memcpy(comp.tetVertIds.data(), tetVertIds, tetVertIdsSize);

            for (int i = 0; i < comp.NumVerts; ++i)
            {
                uint numElems = vertIncidentTets[i].GetNumElems();
                comp.vertIncidentTets.push_back(numElems);
                for (uint j = 0; j < numElems; ++j)
                {
                    comp.vertIncidentTets.push_back(vertIncidentTets[i][j]);
                }
            }

            delete[] restPositions;
            delete[] tetVertIds;
            delete[] vertIncidentTets;

            ComponentResources.push_back(comp);
        }
    }

    FComponentResources FEMResource::ProcessResource(
        std::vector<unsigned int>& vertPermutation, std::vector<unsigned int>& tetPermutation, 
        AMD::FmVector3* restPositions, AMD::FmTetVertIds* tetVertIds, std::vector<unsigned int>* vertIncidentTets, int numVerts, int numTets)
    {
        (void)vertPermutation;
        (void)tetPermutation;

        FComponentResources comp;
        comp.NumVerts = numVerts;
        comp.NumTets = numTets;
        comp.CollisionGroup = 0;

        //SortByPartition(vertPermutation, tetPermutation, restPositions, tetVertIds, vertIncidentTets, comp.NumVerts, comp.NumTets);

        comp.minPos = FVector();
        comp.maxPos = FVector();
        AMD::FmVector3 minP = restPositions[0];
        AMD::FmVector3 maxP = restPositions[0];

        for (int vIdx = 0; vIdx < comp.NumVerts; vIdx++)
        {
            minP = min(minP, restPositions[vIdx]);
            maxP = max(maxP, restPositions[vIdx]);
        }

        comp.minPos.X = minP.x;
        comp.minPos.Y = minP.y;
        comp.minPos.Z = minP.z;

        comp.maxPos.X = maxP.x;
        comp.maxPos.Y = maxP.y;
        comp.maxPos.Z = maxP.z;

        comp.RestVolume = AMD::FmComputeTetMeshVolume(restPositions, tetVertIds, comp.NumTets);

        int restPosSize = sizeof(AMD::FmVector3) * comp.NumVerts;
        int tetVertIdsSize = sizeof(AMD::FmTetVertIds) * comp.NumTets;

        comp.restPositions.reserve(restPosSize);
        for (int i = 0; i < restPosSize; i++) comp.restPositions.push_back(0);
        memcpy(comp.restPositions.data(), restPositions, restPosSize);

        comp.tetVertIds.reserve(tetVertIdsSize);
        for (int i = 0; i < tetVertIdsSize; i++) comp.tetVertIds.push_back(0);
        memcpy(comp.tetVertIds.data(), tetVertIds, tetVertIdsSize);

        for (int i = 0; i < comp.NumVerts; ++i)
        {
            comp.vertIncidentTets.push_back((uint)vertIncidentTets[i].size());
            for (int j = 0; j < vertIncidentTets[i].size(); ++j)
            {
                comp.vertIncidentTets.push_back(vertIncidentTets[i][j]);
            }
        }

        return comp;
    }

    void FEMResource::AddComponent(FComponent comp)
    {
        Components.push_back(comp);
    }

}
