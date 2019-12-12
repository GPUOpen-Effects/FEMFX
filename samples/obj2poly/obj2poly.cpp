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

#include <stdio.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

bool WritePolyFile(const char* objFilename, const char* polyFilename)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &err, objFilename))
    {
        fprintf(stderr, "Could not load %s\n", objFilename);
        return false;
    }

    int numVerts = (int)attrib.vertices.size() / 3;
    float* verts = &attrib.vertices[0];
    float* normals = &attrib.normals[0];
    float* texcoords = &attrib.texcoords[0];

    tinyobj::index_t* indices = &shapes[0].mesh.indices[0];
    int numIndices = (int)shapes[0].mesh.indices.size();

    int numDims = 3;
    int numAttributes = 0;
    int isBoundaryMarker = 1;
    FILE* fp = fopen(polyFilename, "w");
    if (fp == NULL)
    {
        fprintf(stderr, "Could not open %s\n", polyFilename);
        return false;
    }

    fprintf(fp, "%d  %d  %d  %d\n", numVerts, numDims, numAttributes, isBoundaryMarker);

    int boundaryMarker = 1;
    for (int i = 0; i < numVerts; i++)
    {
        fprintf(fp, "%d  %f  %f  %f  %d\n", i + 1, verts[0], verts[1], verts[2], boundaryMarker);
        verts += 3;
    }

    int numFaces = numIndices / 3;

    fprintf(fp, "%d  %d\n", numFaces, isBoundaryMarker);

    for (int i = 0; i < numFaces; i++)
    {
        int numCorners = 3;
        int numPolys = 1;
        int numHoles = 0;
        fprintf(fp, "%d  %d  %d  # %d\n", numPolys, numHoles, boundaryMarker, i + 1);
        fprintf(fp, "%d    %d  %d  %d  \n", numCorners, indices[0].vertex_index+1, indices[1].vertex_index+1, indices[2].vertex_index+1);
        indices += 3;
    }

    fclose(fp);

    return true;
}

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        fprintf(stderr, "Usage obj2tetgen objfile polyfile\n");
        return -1;
    }

    char* objFilename = argv[1];
    char* polyFilename = argv[2];

    WritePolyFile(objFilename, polyFilename);

    return 0;
}
