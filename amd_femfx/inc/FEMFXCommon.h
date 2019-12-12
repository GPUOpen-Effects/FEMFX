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
// Common constants, compiler attributes, and configuration options for library
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXTypes.h"

// Library constants
#define FM_INVALID_ID                                     0xffffffff   // Indicates uninitialized or invalid id
#define FM_RB_FLAG                                        0x80000000   // Bit set in object ids as flag for rigid body
#define FM_DEFAULT_MAX_CG_ITERATIONS                      60           // Default initialized number for maximum CG iterations 
#define FM_DEFAULT_FRICTION_COEFF                         0.4f         // Default initialized value for friction
#define FM_MAX_VERT_INCIDENT_TETS                         64           // Maximum tets allowed to be incident on a vertex; bounds some array sizes
#define FM_MAX_TET_ASPECT_RATIO                           25.0f        // Maximum aspect ratio for warnings; lower than this is preferred; higher aspect ratios harm stability

// Aligned malloc and free, which must be defined by the application
extern void* FmAlignedMalloc(size_t size, size_t alignment);
extern void FmAlignedFree(void *);

template <class T>
static inline T* FmMalloc()
{
    return reinterpret_cast<T*>(FmAlignedMalloc(sizeof(T), FM_ALIGN_OF(T)));
}

template <class T>
static inline T* FmMallocArray(size_t numElements)
{
    return reinterpret_cast<T*>(FmAlignedMalloc(sizeof(T) * numElements, FM_ALIGN_OF(T)));
}

// Class override of new and delete to use FmAligned functions
#define FM_CLASS_NEW_DELETE(T) \
    inline void* operator new(size_t size) \
    { \
        return FmAlignedMalloc(size, FM_ALIGN_OF(T)); \
    } \
    inline void operator delete(void* ptr) \
    { \
        FmAlignedFree(ptr); \
    } \
    inline void* operator new [](size_t size) \
    { \
        return FmAlignedMalloc(size, FM_ALIGN_OF(T)); \
    } \
    inline void operator delete[] (void* ptr) \
    { \
        FmAlignedFree(ptr); \
    }

// Enable a few checks and warnings
#define FM_DEBUG_CHECKS 0
#if FM_DEBUG_CHECKS
extern int FmDebugPrint(const char *format, ...);  // Must be defined in application
#define FM_PRINT(x) FmDebugPrint x
int FmAssertBreak();
#ifdef FM_ASSERT
#undef FM_ASSERT
#endif
#define FM_ASSERT(x) if (!(x)) { FM_PRINT(("FEMFX assert failed: " #x "\n")); FmAssertBreak(); }
#else
#define FM_PRINT(x)
#endif

namespace AMD
{
    // TODO: more angle constraint types
    enum FmRigidBodyAngleConstraintTypes
    {
        FM_RB_JOINT_TYPE_HINGE
    };
}

// Save per phase and total step timings each step
#define FM_TIMINGS 0

#if FM_TIMINGS
namespace AMD
{
    extern double gFEMFXStartTime;
    extern double gFEMFXMeshComponentsTime;
    extern double gFEMFXImplicitStepTime;
    extern double gFEMFXBroadPhaseTime;
    extern double gFEMFXMeshContactsTime;
    extern double gFEMFXConstraintIslandsTime;
    extern double gFEMFXConstraintSolveTime;
    extern double gFEMFXTotalStepTime;
}
#endif

// Code path options to configure library

// Run a separate stabilization solve to correct constraint error.
#define FM_CONSTRAINT_STABILIZATION_SOLVE     1

// Option to solve for delta velocity in implicit solve
#define FM_SOLVE_DELTAV                       0

// Option to assemble system matrix by iterating over tets to compute and add stiffness submats into matrix.
// Otherwise will iterate over verts, applying submats from incident tets list.
#define FM_MATRIX_ASSEMBLY_BY_TETS            1

// Compute a relative rotation for plastically deformed rest positions, in order to compute an "elastic-only" tet rotation for computing elastic stress.
// In practice this seems to have a small effect, so may be possible to disable for CPU and mem savings.
#define FM_COMPUTE_PLASTIC_REL_ROTATION       0

// Option to sort adjacent vertices by id in each row of implicit solve system matrix.
#define FM_SORT_MATRIX_ROW_VERTS              0

// Use structure-of-arrays style SIMD to run multiple triangle-pair intersections in parallel.
#define FM_SOA_TRI_INTERSECTION               1

// Use SoA implementations of CCD functions.
#define FM_SOA_TRI_CCD                        (1 && FM_SOA_TRI_INTERSECTION)

// Use SoA implementation to compute tet matrices.
#define FM_SOA_TET_MATH                       1

// Create distance contacts on intersection of tet mesh surfaces.
// Contacts are placed at intersecting triangles, and normals are derived from the volume contact gradients.
// This gives better quality contact for intersecting meshes, but with added cost.
// Using a limit of contacts per object pair (see FEMFXCollisionPairData.h) 
#define FM_SURFACE_INTERSECTION_CONTACTS      1

// Use task dependency graph within a constraint island to allow better scheduling.
// This gives a significant benefit in some cases, but seems neutral in other cases.
// There is some overhead in constructing the graph, but it reduces barrier syncs and allows some of the 
// solver tasks to be scheduled earlier reducing idle time.
#define FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH 1

// Maintains a limited set of distance contacts for each tet mesh vertex based on time of impact or distance, 
// and culls a contact if no vertex references it.
#define FM_CONTACT_REDUCTION                  1

// Option to project friction to circle in PGS (technique described by Erwin Coumans)
// Otherwise friction forces are projected to a box, aligned to relative velocity on the contact plane.
#define FM_PROJECT_FRICTION_TO_CIRCLE         0

// Compute norms for convergence test (not yet supported)
// Reference: Eberlen, "Physics Based Animation"
#define FM_CONSTRAINT_SOLVER_CONVERGENCE_TEST 0
