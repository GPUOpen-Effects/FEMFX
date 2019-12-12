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

#pragma once

#include "FEMFXCommon.h"

#include <algorithm>

#ifdef _WIN32
#include <malloc.h>
#include <intrin.h> // for __lzcnt
#endif

// Bit flags
#define FM_VERT_FLAG_KINEMATIC                            0x1          // Kinematically driven vertex
#define FM_VERT_FLAG_KINEMATIC_REMOVABLE                  0x2          // Let fracture remove kinematic flags based on stress or when exterior face is constrained only by one or two vertices
#define FM_VERT_FLAG_FRACTURED                            0x4          // Marks a vertex where a fracture occurred, reset on scene update
#define FM_VERT_FLAG_FRACTURE_COPY                        0x8          // Marks a vertex that was created by fracture
#define FM_VERT_FLAG_CRACK_TIP                            0x10         // Marked as crack tip for propagating fracture
#define FM_VERT_FLAG_ASSIGNMENT_UPDATED                   0x20         // Flag used in updating assignments of verts to exterior faces
#define FM_VERT_FLAG_EXT_FORCE_SET                        0x40         // Marks that external force was set on vertex (cleared each frame)

#define FM_TET_FLAG_FACE0_FRACTURE_DISABLED               0x1          // Fracture disabled for face 0 of tetrahedron
#define FM_TET_FLAG_FACE1_FRACTURE_DISABLED               0x2          // Fracture disabled for face 1 of tetrahedron
#define FM_TET_FLAG_FACE2_FRACTURE_DISABLED               0x4          // Fracture disabled for face 2 of tetrahedron
#define FM_TET_FLAG_FACE3_FRACTURE_DISABLED               0x8          // Fracture disabled for face 3 of tetrahedron
#define FM_TET_FLAG_KINEMATIC                             0x10         // Marks tet as kinematic; fracture will leave connected to original vertices and not remove kinematic flags from vertices, as long as these are in face-connected groupings
#define FM_TET_FLAG_PLASTICITY_DISABLED                   0x20         // Disable plasticity for tetrahedron material
#define FM_TET_FLAG_VOLUME_PRESERVING_PLASTICITY          0x40         // Constrain plastic deformation to preserve volume (note can cause an irregular surface)

#define FM_OBJECT_FLAG_KINEMATIC                           0x1          // Entire object is kinematically driven
#define FM_OBJECT_FLAG_POS_CHANGED                         0x2          // Set when vertex position changed, used for kinematic objects
#define FM_OBJECT_FLAG_VEL_CHANGED                         0x4          // Set when vertex velocity changed, used for kinematic objects
#define FM_OBJECT_FLAG_NEEDS_CONNECTED_COMPONENTS          0x8          // Needs connected components processing, after initialization or fracture
#define FM_OBJECT_FLAG_NEEDS_ADJACENT_VERT_OFFSETS         0x10         // Needs update of adjacent vert offsets, after creation of new mesh
#define FM_OBJECT_FLAG_REMOVED_KINEMATIC                   0x20         // Set if kinematic flag removed at vertex (only when FM_VERT_FLAG_KINEMATIC_REMOVABLE set and threshold exceeded)
#define FM_OBJECT_FLAG_SLEEPING                            0x40         // Set object as sleeping, still enough to set velocity to zero and disable simulation
#define FM_OBJECT_FLAG_SLEEPING_DISABLED                   0x80         // Disable sleeping for the object
#define FM_OBJECT_FLAG_HIT_MAX_VERTS                       0x100        // Hit limit of verts during fracture
#define FM_OBJECT_FLAG_HIT_MAX_EXTERIOR_FACES              0x200        // Hit limit of exterior faces during fracture
#define FM_OBJECT_FLAG_SURFACE_COLLISION_CALLBACK_DISABLED 0x400        // Disable call to scene's surfaceCollisionCallback (if non-NULL) for a tet mesh
#define FM_OBJECT_FLAG_SIMULATION_DISABLED                 0x800        // Object (tet mesh, rigid body) should not be simulated
#define FM_OBJECT_FLAG_COMPUTE_TET_STRAIN_MAG              0x1000       // Enable computing tetStrainMagAvg and tetStrainMagMax deformation measures
#define FM_OBJECT_FLAG_ENABLE_SELF_COLLISION               0x2000       // Enable computing self collision

#define FM_ISLAND_FLAG_SOLVE_IS_SETUP                     0x1          // Constraints and objects have been processed to setup solve
#define FM_ISLAND_FLAG_STABILIZATION_IS_SETUP             0x2          // Constraints and objects have been processed to setup stabilization
#define FM_ISLAND_FLAG_MARKED_FOR_SLEEPING                0x4          // Used during constraint solve to mark island for sleeping
#define FM_ISLAND_FLAG_MARKED_FOR_WAKING                  0x8          // Used to mark island for waking when contact occurs, or one of its objects is moved or deleted
#define FM_ISLAND_FLAG_SLEEPING                           0x10         // Sleeping status of island
#define FM_ISLAND_FLAG_USER                               0x8000

// Pad value to next largest multiple of n, power of 2
#define FM_PAD_POW2(n, x) (((x) + (n) - 1)&~((n)-1))
#define FM_PAD_16(x) FM_PAD_POW2(16, x)
#define FM_PAD_64(x) FM_PAD_POW2(64, x)

// Test bits in flags
#define FM_ANY_SET(flags, bit) (((flags) & (bit)) != 0)
#define FM_IS_SET FM_ANY_SET
#define FM_NONE_SET(flags, bit) (((flags) & (bit)) == 0)
#define FM_NOT_SET FM_NONE_SET
#define FM_ALL_SET(flags, bits) (((flags) & (bits)) == (bits))

namespace AMD
{
    // Count leading zeros
    static FM_FORCE_INLINE uint FmCountLeadingZeros(uint x)
    {
#ifdef WIN32
        return __lzcnt(x);
#endif
    }

    // Count set bits
    static FM_FORCE_INLINE uint FmCountSetBits(uint x)
    {
#ifdef WIN32
        return (int)_mm_popcnt_u32(x);
#endif
    }

    static FM_FORCE_INLINE uint FmIntLog2(uint x)
    {
        return 32 - FmCountLeadingZeros(x) - 1;
    }

    static FM_FORCE_INLINE uint FmNextPowerOf2(uint x)
    {
        return 1 << FmIntLog2(x + x - 1);
    }
}

namespace AMD
{
    // 16-byte aligned suballocations
    template<class T>
    static FM_FORCE_INLINE T* FmAllocFromBuffer(uint8_t** ppBuffer, size_t numElements, uint8_t* pBufferEnd)
    {
        (void)pBufferEnd;
        size_t size = sizeof(T) * numElements;
        size_t paddedSize = (size + 15) & ~(15);  // all ptrs/offsets at 16 byte alignment
        uint8_t* pAlloc = *ppBuffer;
        *ppBuffer += paddedSize;
        FM_STATIC_ASSERT(FM_ALIGN_OF(T) <= 16);
        FM_ASSERT(*ppBuffer <= pBufferEnd);
        FM_ASSERT(((uintptr_t)pAlloc & 0xf) == 0);

        return reinterpret_cast<T*>(pAlloc);
    }

    // 64-byte aligned suballocations
    template<class T>
    static FM_FORCE_INLINE T* FmAllocFromBuffer64(uint8_t** ppBuffer, size_t numElements, uint8_t* pBufferEnd)
    {
        (void)pBufferEnd;
        size_t size = sizeof(T) * numElements;
        size_t paddedSize = (size + 63) & ~(63);  // all ptrs/offsets at 64 byte alignment
        uint8_t* pAlloc = *ppBuffer;
        *ppBuffer += paddedSize;
        FM_STATIC_ASSERT(FM_ALIGN_OF(T) <= 64);
        FM_ASSERT(*ppBuffer <= pBufferEnd);
        FM_ASSERT(((uintptr_t)pAlloc & 0x3f) == 0);

        return reinterpret_cast<T*>(pAlloc);
    }
}

#define FM_DEBUG_MESHES                 0  // Saves a copy of meshes prior to constraint solve

#if FM_DEBUG_MESHES
#include <vector>
#endif

#define FM_USE_TRACE                    0
#if FM_USE_TRACE
extern bool gFEMFXTraceEnabled;
#include "FEMTrace.h"
#else
#define FM_INIT_TRACE()     
#define FM_SHUTDOWN_TRACE() 
#define FM_ENABLE_TRACE()   
#define FM_DISABLE_TRACE()  
#define FM_TRACE_START_EVENT(name_)
#define FM_TRACE_STOP_EVENT(name_)
#define FM_TRACE_SCOPED_EVENT(name_)
#define FM_TRACE_INC_COUNTER(name_, val_)
#define FM_TRACE_START_FRAME()
#define FM_TRACE_STOP_FRAME()
#endif

#if FM_TIMINGS
#include "Windows.h"
namespace AMD
{
    // Get system time in millseconds
    static FM_FORCE_INLINE double FmGetTimeMsec()
    {
        LARGE_INTEGER freq, counter;
        QueryPerformanceFrequency(&freq);
        QueryPerformanceCounter(&counter);

        uint64_t timeseconds = counter.QuadPart / freq.QuadPart;
        uint64_t timeremainder = counter.QuadPart % freq.QuadPart;

        return ((double)timeseconds + (double)timeremainder / (double)freq.QuadPart) * 1000.0;
    }
}
#define FM_GET_TIME(t) t = FmGetTimeMsec()
#define FM_SET_START_TIME() gFEMFXStartTime = FmGetTimeMsec()
#define FM_SET_MESH_COMPONENTS_TIME() gFEMFXMeshComponentsTime = FmGetTimeMsec() - gFEMFXStartTime
#define FM_SET_IMPLICIT_STEP_TIME() gFEMFXImplicitStepTime = FmGetTimeMsec() - gFEMFXStartTime
#define FM_SET_CONSTRAINT_ISLANDS_TIME() gFEMFXConstraintIslandsTime = FmGetTimeMsec() - gFEMFXStartTime
#define FM_SET_CONSTRAINT_SOLVE_TIME() gFEMFXConstraintSolveTime = FmGetTimeMsec() - gFEMFXStartTime
#define FM_SET_TOTAL_STEP_TIME(tStart) gFEMFXTotalStepTime = FmGetTimeMsec() - tStart

#define FM_RESET_BROAD_PHASE_TIME() gFEMFXBroadPhaseTime = 0.0
#define FM_RESET_MESH_CONTACTS_TIME() gFEMFXMeshContactsTime = 0.0
#define FM_ADD_BROAD_PHASE_TIME() gFEMFXBroadPhaseTime += FmGetTimeMsec() - gFEMFXStartTime
#define FM_ADD_MESH_CONTACTS_TIME() gFEMFXMeshContactsTime += FmGetTimeMsec() - gFEMFXStartTime
#else
#define FM_GET_TIME(t) (void)t
#define FM_SET_START_TIME()
#define FM_SET_MESH_COMPONENTS_TIME()
#define FM_SET_IMPLICIT_STEP_TIME()
#define FM_SET_CONSTRAINT_ISLANDS_TIME()
#define FM_SET_CONSTRAINT_SOLVE_TIME()
#define FM_SET_TOTAL_STEP_TIME(tStart) (void)tStart

#define FM_RESET_BROAD_PHASE_TIME()
#define FM_RESET_MESH_CONTACTS_TIME()
#define FM_ADD_BROAD_PHASE_TIME()
#define FM_ADD_MESH_CONTACTS_TIME()
#endif
