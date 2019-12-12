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
// Functions supporting creation of tet mesh topology and computing memory bounds for 
// fracture.
//---------------------------------------------------------------------------------------
#pragma once

#include "FEMFXCommonInternal.h"
#include "FEMFXArray.h"

namespace AMD
{
    struct FmTetVertIds;

    // Search incident tets of a vertex to find number of adjacent vertices 
    uint FmGetVertNumAdjacentVerts(const uint* vertexIncidentTets, uint numIncidentTets, const FmTetVertIds* tetVertIds);

    // Add incident tets to vertex
    bool FmAddVertIncidentTets(FmTetMesh* tetMesh, uint* incidentTetsIndex, uint vId, const uint* tetsArray, uint numTets);

    // Finds set of tets incident to each vertex, based on tetVertIds array.
    // vertIncidentTets array of vectors must be sized >= numVerts.
    void FmFindVertIncidentTets(
        FmArray<uint>* vertIncidentTets,
        const FmTetVertIds* tetVertIds,
        uint numVerts, uint numTets);


}