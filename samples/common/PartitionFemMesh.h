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
#pragma once

#include "AMD_FEMFX.h"
#include "FEMFXArray.h"

#define PARTITION_MESH 0

#if PARTITION_MESH
namespace AMD
{

    struct PartitionMeshInput
    {
        //FmVector3*      vertPositions;
        FmTetVertIds*   tetVertIndices;
        FmArray<uint>*  vertIncidentTets;
        uint               numVerts;
        uint               numTets;
        uint               numPartitions;
    };

    int PartitionMesh(
        uint* vertPartitionOffsets,  // Array of size numPartitions + 1 with starting offset of each partition
        uint* vertPartitionIndices,  // Concatenated ararys of vertex indices of each partition
        uint* tetPartitionOffsets,   // Array of size numPartitions + 1 with starting offset of each partition
        uint* tetPartitionIndices,   // Concatenated ararys of vertex indices of each partition
        const PartitionMeshInput& meshInput);

    // Reorder the positions, tet vert ids, and vert incident tets in order of vertex partitions created by PartitionMesh
    void SortByPartition(
        FmArray<unsigned int>& vertPermutation, FmArray<unsigned int>& tetPermutation,
        FmVector3* vertRestPositions,
        FmTetVertIds* tetVertIds,
        FmArray<uint>* vertIncidentTets,
        uint numVerts, uint numTets);

}
#endif
