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
// Types defining the connectivity of a FEMFX tet mesh and some basic operations
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXCommon.h"

namespace AMD
{
    // Vertex ids, arranged so v0, v1, v2 in CCW order seen from v3
    struct FmTetVertIds
    {
        uint ids[4];
    };

    // Faces CCW from exterior defined as: 312, 203, 130, 021
    // Face i is face opposite to corner i
    struct FmFaceVertIds
    {
        uint ids[3];
    };

    // Get the tet vertex ids for a face
    static FM_FORCE_INLINE void FmGetFaceVertIds(FmFaceVertIds* faceVertIds, uint face, const FmTetVertIds& tetVertIds)
    {
        faceVertIds->ids[0] = tetVertIds.ids[3 - face];
        faceVertIds->ids[1] = tetVertIds.ids[(5 - face) % 4];
        faceVertIds->ids[2] = tetVertIds.ids[(face + 2) % 4];
    }

    // Get the indices of tet corners for a face
    static FM_FORCE_INLINE void FmGetFaceTetCorners(FmFaceVertIds* tetCorners, uint face)
    {
        tetCorners->ids[0] = 3 - face;
        tetCorners->ids[1] = (5 - face) % 4;
        tetCorners->ids[2] = (face + 2) % 4;
    }

    // Neighbor features of a vertex.  Keeping only an array of incident tets, and finding adjacent vertices through those.
    struct FmVertNeighbors
    {
        uint numAdjacentVerts;   // Number of adjacent verts, including self (also number of non-zero submatrices in block row of stiffness matrix)
        uint incidentTetsStart;  // Start index of incident tets array
        uint numIncidentTets;    // Number of incident tets

        FmVertNeighbors() : numAdjacentVerts(0), incidentTetsStart(0), numIncidentTets(0) {}
    };

    // Incident tets and number of adjacent verts for all vertices.
    // (Not storing adjacent verts but finding them from incident tets.)
    // With fracture on tet boundaries, a mesh's total number of incident tets does not change --
    // fracture replicates a vertex, then distributes its incident tets between it and the new vertices.
    // Fracture can stay within the original memory by partitioning that vertex's incident tets in place 
    // to make arrays for the new vertices. 
    struct FmVertConnectivity
    {
        uint* incidentTets;           // All tet mesh buffer incident tets
        uint  numIncidentTets;        // Num incident tets of tet mesh
        uint  numIncidentTetsTotal;   // Total incident tets in tet mesh buffer
        uint  numAdjacentVerts;       // Num adjacent verts of tet mesh (determines size of sparse system matrix)

        FmVertConnectivity() : incidentTets(NULL), numIncidentTets(0), numIncidentTetsTotal(0), numAdjacentVerts(0) {}
    };

    // Ids of tets that share faces with a tet.
    // For exterior tet faces, id is an exterior face id with MSB bit set.
    struct FmTetFaceIncidentTetIds
    {
        uint ids[4];
    };

    static FM_FORCE_INLINE bool FmIsExteriorFaceId(uint id) { return ((id & 0x80000000UL) != 0); }
    static FM_FORCE_INLINE uint FmMakeExteriorFaceId(uint exteriorFaceId) { return exteriorFaceId | 0x80000000UL; }
    static FM_FORCE_INLINE uint FmGetExteriorFaceId(uint id) { return id & ~0x80000000UL; }

    struct FmEdgeIncidentFaces
    {
        uint faceIds[3];
    };

    // Get the tet faces incident to a face on each of its three edges.
    static FM_FORCE_INLINE void FmGetTetFacesAtEdges(FmEdgeIncidentFaces* edgeIncidentFaces, uint face)
    {
        edgeIncidentFaces->faceIds[0] = (face + 2) % 4;
        edgeIncidentFaces->faceIds[1] = 3 - face;
        edgeIncidentFaces->faceIds[2] = (5 - face) % 4;
    }

    // Check if two tet faces match, assuming CCW order of vertices
    static FM_FORCE_INLINE bool FmFacesAreMatching(const FmFaceVertIds& faceVertIdsA, const FmFaceVertIds& faceVertIdsB)
    {
        return (
            (faceVertIdsA.ids[0] == faceVertIdsB.ids[2] &&
                faceVertIdsA.ids[1] == faceVertIdsB.ids[1] &&
                faceVertIdsA.ids[2] == faceVertIdsB.ids[0]) ||
                (faceVertIdsA.ids[0] == faceVertIdsB.ids[1] &&
                    faceVertIdsA.ids[1] == faceVertIdsB.ids[0] &&
                    faceVertIdsA.ids[2] == faceVertIdsB.ids[2]) ||
                    (faceVertIdsA.ids[0] == faceVertIdsB.ids[0] &&
                        faceVertIdsA.ids[1] == faceVertIdsB.ids[2] &&
                        faceVertIdsA.ids[2] == faceVertIdsB.ids[1]));
    }

    // Check if two edges match
    static FM_FORCE_INLINE bool FmEdgesAreMatching(uint edgeA, uint edgeB, const FmFaceVertIds& faceVertIdsA, const FmFaceVertIds& faceVertIdsB)
    {
        FM_ASSERT(edgeA < 3 && edgeB < 3);

        uint vA0 = edgeA;
        uint vA1 = (edgeA + 1) % 3;
        uint vB0 = edgeB;
        uint vB1 = (edgeB + 1) % 3;

        return
            (faceVertIdsA.ids[vA0] == faceVertIdsB.ids[vB0] && faceVertIdsA.ids[vA1] == faceVertIdsB.ids[vB1]) ||
            (faceVertIdsA.ids[vA1] == faceVertIdsB.ids[vB0] && faceVertIdsA.ids[vA0] == faceVertIdsB.ids[vB1]);
    }

    static FM_FORCE_INLINE bool FmHasMatchingFace(uint* matchingFaceA, uint* matchingFaceB, uint tetAId, uint tetBId, const FmTetFaceIncidentTetIds& tetATets, const FmTetFaceIncidentTetIds& tetBTets)
    {
        uint faceA, faceB;
        faceA = faceB = FM_INVALID_ID;
        faceA = tetATets.ids[0] == tetBId ? 0 : faceA;
        faceA = tetATets.ids[1] == tetBId ? 1 : faceA;
        faceA = tetATets.ids[2] == tetBId ? 2 : faceA;
        faceA = tetATets.ids[3] == tetBId ? 3 : faceA;
        faceB = tetBTets.ids[0] == tetAId ? 0 : faceB;
        faceB = tetBTets.ids[1] == tetAId ? 1 : faceB;
        faceB = tetBTets.ids[2] == tetAId ? 2 : faceB;
        faceB = tetBTets.ids[3] == tetAId ? 3 : faceB;
        FM_ASSERT((faceA != FM_INVALID_ID && faceB != FM_INVALID_ID) || (faceA == FM_INVALID_ID && faceB == FM_INVALID_ID));
        *matchingFaceA = faceA;
        *matchingFaceB = faceB;
        return (faceA != FM_INVALID_ID);
    }

    bool FmHasMatchingFace(uint* matchingFaceA, uint* matchingFaceB, const FmTetVertIds& tetAVerts, const FmTetVertIds& tetBVerts);

    static inline bool FmHasMatchingEdge(uint& edgeA, uint& edgeB, const FmFaceVertIds& faceAVerts, const FmFaceVertIds& faceBVerts)
    {
        edgeA = edgeB = FM_INVALID_ID;
        bool match;
        match = FmEdgesAreMatching(0, 0, faceAVerts, faceBVerts);
        edgeA = match ? 0 : edgeA;
        edgeB = match ? 0 : edgeB;
        match = FmEdgesAreMatching(0, 1, faceAVerts, faceBVerts);
        edgeA = match ? 0 : edgeA;
        edgeB = match ? 1 : edgeB;
        match = FmEdgesAreMatching(0, 2, faceAVerts, faceBVerts);
        edgeA = match ? 0 : edgeA;
        edgeB = match ? 2 : edgeB;
        match = FmEdgesAreMatching(1, 0, faceAVerts, faceBVerts);
        edgeA = match ? 1 : edgeA;
        edgeB = match ? 0 : edgeB;
        match = FmEdgesAreMatching(1, 1, faceAVerts, faceBVerts);
        edgeA = match ? 1 : edgeA;
        edgeB = match ? 1 : edgeB;
        match = FmEdgesAreMatching(1, 2, faceAVerts, faceBVerts);
        edgeA = match ? 1 : edgeA;
        edgeB = match ? 2 : edgeB;
        match = FmEdgesAreMatching(2, 0, faceAVerts, faceBVerts);
        edgeA = match ? 2 : edgeA;
        edgeB = match ? 0 : edgeB;
        match = FmEdgesAreMatching(2, 1, faceAVerts, faceBVerts);
        edgeA = match ? 2 : edgeA;
        edgeB = match ? 1 : edgeB;
        match = FmEdgesAreMatching(2, 2, faceAVerts, faceBVerts);
        edgeA = match ? 2 : edgeA;
        edgeB = match ? 2 : edgeB;
        return (edgeA != FM_INVALID_ID);
    }

    static FM_FORCE_INLINE bool FmIsTetIncident(uint vertId, const FmTetVertIds& tetVertIds)
    {
        return (tetVertIds.ids[0] == vertId || tetVertIds.ids[1] == vertId || tetVertIds.ids[2] == vertId || tetVertIds.ids[3] == vertId);
    }

    // Specifies the size of a tet mesh needed for a fracture group, a non-fracturing region of the original tet mesh
    struct FmFractureGroupCounts
    {
        uint numVerts;
        uint numTets;
        uint numExteriorFaces;
        uint numVertAdjacentVerts;
        uint numVertIncidentTets;

        FmFractureGroupCounts() : numVerts(0), numTets(0), numExteriorFaces(0), numVertAdjacentVerts(0), numVertIncidentTets(0) { }

        inline void Zero()
        {
            numVerts = 0;
            numTets = 0;
            numExteriorFaces = 0;
            numVertAdjacentVerts = 0;
            numVertIncidentTets = 0;
        }

        inline void Add(const FmFractureGroupCounts& inCounts)
        {
            numVerts += inCounts.numVerts;
            numTets += inCounts.numTets;
            numExteriorFaces += inCounts.numExteriorFaces;
            numVertAdjacentVerts += inCounts.numVertAdjacentVerts;
            numVertIncidentTets += inCounts.numVertIncidentTets;
        }

        inline bool GreaterEqual(const FmFractureGroupCounts& inCounts) const
        {
            return numVerts >= inCounts.numVerts
                && numTets >= inCounts.numTets
                && numExteriorFaces >= inCounts.numExteriorFaces
                && numVertAdjacentVerts >= inCounts.numVertAdjacentVerts
                && numVertIncidentTets >= inCounts.numVertIncidentTets;
        }
    };
}

