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
// Sample code to reorganize mesh data using partitions computed by METIS
//---------------------------------------------------------------------------------------
#include "PartitionFemMesh.h"

#if PARTITION_MESH
#define _MSC_STDINT_H_
#include "metisbin.h"

namespace AMD
{

    int PartitionMesh(
        uint* vertPartitionOffsets,  // Array of size numPartitions + 1 with starting offset of each partition
        uint* vertPartitionIndices,  // Concatenated ararys of vertex indices of each partition
        uint* tetPartitionOffsets,   // Array of size numPartitions + 1 with starting offset of each partition
        uint* tetPartitionIndices,   // Concatenated ararys of vertex indices of each partition
        const PartitionMeshInput& meshInput)
    {
        if (meshInput.numPartitions <= 1)
        {
            return -1;
        }

        mesh_t metisMesh;
        metisMesh.ne = meshInput.numTets;
        metisMesh.nn = meshInput.numVerts;
        metisMesh.ncon = 0;
        metisMesh.eptr = new idx_t[meshInput.numTets + 1];  // start of node indices of element
        metisMesh.eind = new idx_t[meshInput.numTets*4];    // all element node indices
        metisMesh.ewgt = NULL;

        uint nodeIdx = 0;
        for (uint elemIdx = 0; elemIdx < meshInput.numTets; elemIdx++)
        {
            metisMesh.eptr[elemIdx] = nodeIdx;
            metisMesh.eind[nodeIdx + 0] = meshInput.tetVertIndices[elemIdx].ids[0];
            metisMesh.eind[nodeIdx + 1] = meshInput.tetVertIndices[elemIdx].ids[1];
            metisMesh.eind[nodeIdx + 2] = meshInput.tetVertIndices[elemIdx].ids[2];
            metisMesh.eind[nodeIdx + 3] = meshInput.tetVertIndices[elemIdx].ids[3];
            nodeIdx += 4;
        }
        metisMesh.eptr[meshInput.numTets] = nodeIdx;

        idx_t* metisNodePartitions = new idx_t[meshInput.numVerts];
        idx_t* metisElemPartitions = new idx_t[meshInput.numTets];

        idx_t metisOptions[METIS_NOPTIONS];
        idx_t metisObjVal;
        idx_t metisNumPartitions = meshInput.numPartitions;

        METIS_SetDefaultOptions(metisOptions);

        metisOptions[METIS_OPTION_GTYPE] = METIS_GTYPE_NODAL;
        metisOptions[METIS_OPTION_PTYPE] = METIS_PTYPE_KWAY;
        metisOptions[METIS_OPTION_OBJTYPE] = METIS_OBJTYPE_CUT;
        metisOptions[METIS_OPTION_CTYPE] = METIS_CTYPE_SHEM;
        metisOptions[METIS_OPTION_IPTYPE] = METIS_IPTYPE_GROW;
        metisOptions[METIS_OPTION_RTYPE] = -1;
        metisOptions[METIS_OPTION_DBGLVL] = 0;
        metisOptions[METIS_OPTION_UFACTOR] = -1;
        metisOptions[METIS_OPTION_MINCONN] = 0;
        metisOptions[METIS_OPTION_CONTIG] = 0;
        metisOptions[METIS_OPTION_SEED] = -1;
        metisOptions[METIS_OPTION_NITER] = 10;
        metisOptions[METIS_OPTION_NCUTS] = 1;
        metisOptions[METIS_OPTION_NUMBERING] = 0;

        METIS_PartMeshNodal(&metisMesh.ne, &metisMesh.nn, metisMesh.eptr, metisMesh.eind,
            NULL, NULL, &metisNumPartitions, NULL, metisOptions, &metisObjVal,
            metisElemPartitions, metisNodePartitions);

        // Find size of each partition
        for (uint partIdx = 0; partIdx < meshInput.numPartitions; partIdx++)
        {
            vertPartitionOffsets[partIdx] = 0;
            tetPartitionOffsets[partIdx] = 0;
        }
        for (uint vertIdx = 0; vertIdx < meshInput.numVerts; vertIdx++)
        {
            uint partIndex = metisNodePartitions[vertIdx];
            vertPartitionOffsets[partIndex]++;
        }
        for (uint elemIdx = 0; elemIdx < meshInput.numTets; elemIdx++)
        {
            uint partIdx = metisElemPartitions[elemIdx];
            tetPartitionOffsets[partIdx]++;
        }

        // Prefix sum to compute partition offsets
        uint vertSum = vertPartitionOffsets[0];
        uint tetSum = tetPartitionOffsets[0];
        vertPartitionOffsets[0] = 0;
        tetPartitionOffsets[0] = 0;
        for (uint partIdx = 1; partIdx < meshInput.numPartitions; partIdx++)
        {
            uint prevVertSum = vertSum;
            uint prevTetSum = tetSum;
            vertSum += vertPartitionOffsets[partIdx];
            tetSum += tetPartitionOffsets[partIdx];
            vertPartitionOffsets[partIdx] = prevVertSum;
            tetPartitionOffsets[partIdx] = prevTetSum;
        }
        vertPartitionOffsets[meshInput.numPartitions] = vertSum;
        tetPartitionOffsets[meshInput.numPartitions] = tetSum;

        // Fill partitions
        for (uint vertIdx = 0; vertIdx < meshInput.numVerts; vertIdx++)
        {
            uint partIdx = metisNodePartitions[vertIdx];
            uint offset = vertPartitionOffsets[partIdx];
            vertPartitionIndices[offset] = vertIdx;
            vertPartitionOffsets[partIdx]++;
        }
        for (uint tetIdx = 0; tetIdx < meshInput.numTets; tetIdx++)
        {
            uint partIdx = metisElemPartitions[tetIdx];
            uint offset = tetPartitionOffsets[partIdx];
            tetPartitionIndices[offset] = tetIdx;
            tetPartitionOffsets[partIdx]++;
        }

        // Reset offsets
        for (int partIdx = meshInput.numPartitions - 1; partIdx >= 1; partIdx--)
        {
            vertPartitionOffsets[partIdx] = vertPartitionOffsets[partIdx - 1];
            tetPartitionOffsets[partIdx] = tetPartitionOffsets[partIdx - 1];
        }
        vertPartitionOffsets[0] = 0;
        tetPartitionOffsets[0] = 0;

        delete[] metisMesh.eptr;
        delete[] metisMesh.eind;
        delete[] metisNodePartitions;
        delete[] metisElemPartitions;

        return 0;
    }

    void SortByPartition(
        FmArray<uint>& vertPermutation,
        FmArray<uint>& tetPermutation,
        FmVector3* vertRestPositions,
        FmTetVertIds* tetVertIds,
        FmArray<uint>* vertIncidentTets,
        uint numVerts, uint numTets)
    {
        // Init permutations
        vertPermutation.Clear();
        for (uint i = 0; i < numVerts; i++)
        {
            vertPermutation.Add(i);
        }

        tetPermutation.Clear();
        for (uint i = 0; i < numTets; i++)
        {
            tetPermutation.Add(i);
        }

        PartitionMeshInput partitionMeshInput;
        partitionMeshInput.tetVertIndices = tetVertIds;
        partitionMeshInput.vertIncidentTets = vertIncidentTets;
        partitionMeshInput.numVerts = numVerts;
        partitionMeshInput.numTets = numTets;
        uint numPartitions = numVerts / 64;
        partitionMeshInput.numPartitions = numPartitions;

        if (partitionMeshInput.numPartitions <= 1)
            return;

        uint* vertPartitionOffsets = new uint[numPartitions + 1];
        uint* vertPartitionIndices = new uint[numVerts];
        uint* tetPartitionOffsets = new uint[numPartitions + 1];
        uint* tetPartitionIndices = new uint[numTets];

        PartitionMesh(vertPartitionOffsets, vertPartitionIndices, tetPartitionOffsets, tetPartitionIndices, partitionMeshInput);

        // Order verts in partition order
        uint* remapVertIds = new uint[numVerts];
        for (uint i = 0; i < numVerts; i++)
        {
            remapVertIds[vertPartitionIndices[i]] = i;
        }

        // Order tets by visiting from verts
        uint* remapTetIds = new uint[numTets];
        bool* tetAdded = new bool[numTets];
        uint numTetsAdded = 0;
        for (uint i = 0; i < numTets; i++)
        {
            tetAdded[i] = false;
            remapTetIds[i] = i;
        }

        for (uint i = 0; i < numVerts; i++)
        {
            uint vertIdx = vertPartitionIndices[i];
            uint numElems = vertIncidentTets[vertIdx].GetNumElems();
            for (uint t = 0; t < numElems; t++)
            {
                uint tetIdx = vertIncidentTets[vertIdx][t];
                if (!tetAdded[tetIdx])
                {
                    remapTetIds[tetIdx] = numTetsAdded;
                    tetAdded[tetIdx] = true;
                    numTetsAdded++;
                }
            }
        }

        // Reorder all the FEM mesh inputs
        FmVector3* sortedRestPositions = new FmVector3[numVerts];
        FmTetVertIds* sortedTetVertIds = new FmTetVertIds[numTets];
        FmArray<uint>* sortedVertIncidentTets = new FmArray<uint>[numVerts];

        for (uint i = 0; i < numVerts; i++)
        {
            sortedRestPositions[i] = vertRestPositions[vertPartitionIndices[i]];
            sortedVertIncidentTets[i] = vertIncidentTets[vertPartitionIndices[i]];

            // Remap tet ids in incident tets array
            uint numElems = sortedVertIncidentTets[i].GetNumElems();
            for (uint t = 0; t < numElems; t++)
            {
                sortedVertIncidentTets[i][t] = remapTetIds[sortedVertIncidentTets[i][t]];
            }
        }

        for (uint i = 0; i < numVerts; i++)
        {
            vertRestPositions[i] = sortedRestPositions[i];
            vertIncidentTets[i] = sortedVertIncidentTets[i];
        }

        for (uint i = 0; i < numTets; i++)
        {
            sortedTetVertIds[remapTetIds[i]] = tetVertIds[i];
        }

        for (uint i = 0; i < numTets; i++)
        {
            tetVertIds[i] = sortedTetVertIds[i];
        }

        // Remap tet vert ids
        for (uint i = 0; i < numTets; i++)
        {
            tetVertIds[i].ids[0] = remapVertIds[tetVertIds[i].ids[0]];
            tetVertIds[i].ids[1] = remapVertIds[tetVertIds[i].ids[1]];
            tetVertIds[i].ids[2] = remapVertIds[tetVertIds[i].ids[2]];
            tetVertIds[i].ids[3] = remapVertIds[tetVertIds[i].ids[3]];
        }

        vertPermutation.Clear();
        for (uint i = 0; i < numVerts; i++)
        {
            vertPermutation.Add(remapVertIds[i]);
        }

        tetPermutation.Clear();
        for (uint i = 0; i < numTets; i++)
        {
            tetPermutation.Add(remapTetIds[i]);
        }

        delete[] vertPartitionOffsets;
        delete[] vertPartitionIndices;
        delete[] tetPartitionOffsets;
        delete[] tetPartitionIndices;
        delete[] remapVertIds;
        delete[] tetAdded;
        delete[] remapTetIds;
        delete[] sortedRestPositions;
        delete[] sortedTetVertIds;
        delete[] sortedVertIncidentTets;
    }
}
#endif