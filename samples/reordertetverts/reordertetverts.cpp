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

#include "LoadMesh.h"
#include <string>

using namespace::AMD;

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        fprintf(stderr, "Usage %s src_filebase dst_filebase\n", argv[0]);
        return -1;
    }

    char* srcFilebase = argv[1];
    char* dstFilebase = argv[2];

    std::string srcNodeFilename = std::string(srcFilebase) + std::string(".node");
    std::string srcEleFilename = std::string(srcFilebase) + std::string(".ele");

    std::string dstNodeFilename = std::string(dstFilebase) + std::string(".node");
    std::string dstEleFilename = std::string(dstFilebase) + std::string(".ele");

    int numVerts, numTets;
    int loadRet = LoadNodeEleMeshNumVerts(srcNodeFilename.c_str());
    if (loadRet < 0)
    {
        return -1;
    }
    numVerts = loadRet;

    std::vector<unsigned int>* vertIncidentTets = new std::vector<unsigned int>[numVerts];

    loadRet = LoadNodeEleMeshNumTets(srcEleFilename.c_str(), vertIncidentTets, numVerts);
    if (loadRet < 0)
    {
        delete[] vertIncidentTets;
        return -1;
    }
    numTets = loadRet;

    NodeEleVector3* vertPositions = new NodeEleVector3[numVerts];
    NodeEleTetVertIds* tets = new NodeEleTetVertIds[numTets];

    LoadNodeEleMeshData(srcNodeFilename.c_str(), srcEleFilename.c_str(), vertPositions, tets);

    numVerts = RemoveUnreferencedVertices(vertPositions, vertIncidentTets, numVerts, tets, numTets);
    ReorderTetVertIds(tets, numTets);

    int ret = StoreNodeEleMeshData(dstNodeFilename.c_str(), dstEleFilename.c_str(), vertPositions, tets, numVerts, numTets);

    delete[] vertIncidentTets;
    delete[] vertPositions;
    delete[] tets;

    return ret;
}
