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
// Structures used for fracture updates at each vertex.
//---------------------------------------------------------------------------------------
#pragma once

#include "FEMFXTetMeshConnectivity.h"
#include "FEMFXHashSet.h"

namespace AMD
{
    struct FmScene;
    struct FmTetMesh;

    struct FmFractureTetIdMapElement
    {
        uint key;  // global tet id
        uint id;   // local tet id
    };

    struct FmFractureTetIdRemapSetFuncs
    {
        static bool IsValidKey(uint key)
        {
            return (key != FM_INVALID_ID);
        }

        static bool KeysEqual(uint keyA, uint keyB)
        {
            return (keyA == keyB);
        }

        static void SetInvalidKey(uint& key)
        {
            key = FM_INVALID_ID;
        }

        static uint HashFunc(uint key)
        {
            return FmComputeHash(key);
        }

        static void InitValue(FmFractureTetIdMapElement& elem)
        {
            elem.id = 0;
        }
    };

    typedef FmHashSet<FmFractureTetIdMapElement, FmFractureTetIdRemapSetFuncs> FmFractureTetIdMap;

    struct FmSplitFaceConnectedLocalTet
    {
        uint                       tetId;
        uint                       componentId;
        float                      tetMass;
        FmTetVertIds            tetVerts;
        FmTetFaceIncidentTetIds tetFaceIncidentTets;
        FmTetFaceIncidentTetIds localTetFaceIncidentTets;
        bool                       isKinematic;
    };

    struct FmMakeSplitsLocalTet
    {
        uint                       tetId;
        FmTetVertIds            tetVerts;
        uint                       componentId;
        FmTetFaceIncidentTetIds newTetFaceIncidentTets;
        FmTetFaceIncidentTetIds localTetFaceIncidentTets;  // indices of tets within tet assignment array
        float                      tetMass;
        float                      projectionOnFractureDir;
        uint                       crackTipId0;
        uint                       crackTipId1;
        uint16_t                   faceFractureDisabled;
        bool                       isKinematic;
        bool                       positiveSide;
    };

    // Find tet corner with vert id
    static FM_FORCE_INLINE uint FmFindTetCornerWithVertId(uint vertId, const FmTetVertIds& tetVertIds)
    {
        return 1 * (uint)(tetVertIds.ids[1] == vertId)
            + 2 * (uint)(tetVertIds.ids[2] == vertId)
            + 3 * (uint)(tetVertIds.ids[3] == vertId);
    }

    // Find tet face connected to given tet
    static FM_FORCE_INLINE uint FmFindTetFaceWithIncidentTetId(uint incidentTetId, const FmTetFaceIncidentTetIds& faceIncidentTets)
    {
        return 1 * (uint)(faceIncidentTets.ids[1] == incidentTetId)
            + 2 * (uint)(faceIncidentTets.ids[2] == incidentTetId)
            + 3 * (uint)(faceIncidentTets.ids[3] == incidentTetId);
    }

    // Split mesh at vertices of fracturing tets.  
    // Creates new vertices and new exterior faces, and updates incident tets of vertices.
    // Returns whether split was made.
    bool FmMakeMeshSplits(FmScene* scene, FmTetMesh* tetMesh);

    // Update the scene's array of tet meshes, separating any fractured connected components into new sub-meshes.
    void FmFindMeshConnectedComponents(FmScene* scene);
}
