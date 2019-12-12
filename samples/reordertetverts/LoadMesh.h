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

#ifndef _LOAD_MESH_H_
#define _LOAD_MESH_H_

#include <vector>

namespace AMD
{

    // Load vert and tet data from Tetgen .node and .ele files

    struct NodeEleTetVertIds
    {
        int ids[4];
    };

    struct NodeEleVector3
    {
        float x, y, z;
    };

    // Get number of verts, needed for sizing buffers
    int LoadNodeEleMeshNumVerts(const char* nodeFile);

    // Get number of tets, and get list of tets incident on each vert, also needed for sizing buffers.
    // vertIncidentTets array assumed to have >= number of verts elements.
    // Each element is vector containing incident tets for a vertex.
    int LoadNodeEleMeshNumTets(const char* eleFile, std::vector<unsigned int>* vertIncidentTets, int numVerts);

    // vertPositions arary assumed to have >= number of verts elements.
    // tets arary assumed to have >= number of tets elements.
    int LoadNodeEleMeshData(const char* nodeFile, const char* eleFile, NodeEleVector3* vertPositions, NodeEleTetVertIds* tets);

    // Store vertex and tetrahedra data to .node and .ele files.
    int StoreNodeEleMeshData(const char* nodeFile, const char* eleFile, NodeEleVector3* vertPositions, NodeEleTetVertIds* tets, int numPoints, int numTetrahedra);

    // Cull any vertices without tetrahedra referencing them.  Returns new number of vertices.
    int RemoveUnreferencedVertices(NodeEleVector3* vertPositions, std::vector<unsigned int>* vertIncidentTets, int numVerts, NodeEleTetVertIds* tets, int numTets);

    // Switches vertices 0 and 1, which will convert between TetGen and Stellar conventions.
    void ReorderTetVertIds(NodeEleTetVertIds* tets, int numTets);
}

#endif
