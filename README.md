AMD FEMFX
v0.1.0

FEMFX is a multithreaded CPU library for deformable material physics, using 
the Finite Element Method (FEM).  Solid objects are represented as a mesh of 
tetrahedral elements, and each element has material parameters that control 
stiffness, how volume changes with deformation, and stress limits where 
fracture or plastic (permanent) deformation occur.  The model supports a 
wide range of materials and interactions between materials.  We intend for 
these features to complement rather than replace traditional rigid body 
physics.  The system is designed with the following considerations:

 * Fidelity: realistic-looking wood, metal, plastic, even glass, because 
   they bend and break according to stress as real materials do.
 * Deformation effects: non-rigid use cases such as soft-body objects, 
   bending or warping objects.   It is not just a visual effect, but 
   materials will resist or push back on other objects.
 * Changing material on the fly: you can change the settings to make the 
   same object behave very differently, e.g., turn gelatinous or melt
 * Interesting physics interactions for gameplay or puzzles

The library uses extensive multithreading to utilize multicore CPUs and benefit 
from the trend of increasing CPU core counts.

Included in this release
* Source code for FEMFX library
* Houdini plugin for content creation
* Sample code for loading and preprocessing content for rendering
* Sample threading utilities
* A basic graphical sample

In a separate release, we provide a plugin for Unreal Engine and a demo that 
demonstrates more advanced rendering of FEM objects:
https://github.com/GPUOpenSoftware/UnrealEngine/tree/FEMFX-4.18

Dependencies
The sample code has some dependencies on external frameworks which must be 
downloaded separately:
* JSON for Modern C++: https://github.com/nlohmann/json
* GLFW: https://www.glfw.org/download.html

System requirements
* AMD Ryzen™ 7 2700X Processor or equivalent
* Windows® 10
* Visual Studio 2017 or Visual Studio 2019

Files:
1. Documentation
   * amd_femfx\docs\
2. Library
   * amd_femfx\inc\ : Public API (Look first at AMD_FEMFX.h)
   * amd_femfx\src\ : Implementation files
3. Sample code
   * samples\FEMFXViewer\ : Basic sample; displays tetrahedral meshes and debug 
     information
   * samples\common\TestScenes.*: Setup for tech demo scenes used by FEMFXViewer
   * samples\common\FemResource.* : Container for the data loaded from a .FEM 
     file
   * samples\common\LoadFem.*: Code for parsing a .FEM and loading into an 
     FEMResource
   * samples\common\RenderTetAssignment.* : Assigns rendering mesh vertices to 
     tetrahedra for skinning
   * samples\ExampleRigidBodies\ : Basic demonstration of interfacing an 
     external rigid body system with FEMFX library
   * samples\sample_task_system\ : Sample task system library interfaced with 
     FEMFX 
4. Houdini plugin for content creation
   * houdini16.5\AMD_FEM_Assets.otl
5. External dependencies
   * external\glfw\ : Path to install GLFW for viewer; can download Windows 
     pre-compiled binaries from https://www.glfw.org/download.html
   * external\json-develop\ : Path to install json parser used in LoadFemFile.*; 
     can download from https://github.com/nlohmann/json
6. Some individual components
   * amd_femfx\inc\Vectormath\ : 3D vector/matrix math
   * amd_femfx\src\PrimitiveCollision\ : Triangle and box collision 
     operations, intersection and CCD
   * amd_femfx\src\Common\FEMFXSvd3x3.* : 3x3 SVD
   * amd_femfx\src\Threading\ : Async parallel-for and task graph support
   * samples\sample_task_system\TL* : Async task system implementation

