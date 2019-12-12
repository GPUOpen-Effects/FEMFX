FEMFX_BASE_DIR = "../"
FEMFX_SRC_DIR = FEMFX_BASE_DIR.."amd_femfx/src/"
FEMFX_INC_DIR = FEMFX_BASE_DIR.."amd_femfx/inc/"
FEMFX_LIB_DIR = FEMFX_BASE_DIR.."amd_femfx/lib/"
VECTORMATH_INC_DIR = FEMFX_BASE_DIR.."amd_femfx/inc/Vectormath/"
TRACE_DIR = FEMFX_BASE_DIR.."external/trace/"
SAMPLES_COMMON_DIR = FEMFX_BASE_DIR.."samples/common/"

    project (FEMFX_LIB_NAME)
        filename('%{prj.name}_'.._ACTION)
        kind "StaticLib"
        language "C++"
        warnings "Extra"
        objdir ('obj/'.._ACTION..'/%{cfg.architecture}')
        targetdir ( FEMFX_LIB_DIR )
        files { FEMFX_INC_DIR.."*.h" }
        files { FEMFX_SRC_DIR.."Bvh/*.cpp", FEMFX_SRC_DIR.."Bvh/*.h" }
        files { FEMFX_SRC_DIR.."Common/*.cpp", FEMFX_SRC_DIR.."Common/*.h" }
        files { FEMFX_SRC_DIR.."ConstraintSolver/*.cpp", FEMFX_SRC_DIR.."ConstraintSolver/*.h" }
        files { FEMFX_SRC_DIR.."FindContacts/*.cpp", FEMFX_SRC_DIR.."FindContacts/*.h" }
        files { FEMFX_SRC_DIR.."PrimitiveCollision/*.cpp", FEMFX_SRC_DIR.."PrimitiveCollision/*.h" }
        files { FEMFX_SRC_DIR.."Scene/*.cpp", FEMFX_SRC_DIR.."Scene/*.h" }
        files { FEMFX_SRC_DIR.."Simulation/*.cpp", FEMFX_SRC_DIR.."Simulation/*.h" }
        files { FEMFX_SRC_DIR.."SparseSolvers/*.cpp", FEMFX_SRC_DIR.."SparseSolvers/*.h" }
        files { FEMFX_SRC_DIR.."Threading/*.cpp", FEMFX_SRC_DIR.."Threading/*.h" }
        files { VECTORMATH_INC_DIR.."*.h" }
        files { TRACE_DIR.."**.cpp", TRACE_DIR.."**.h" }
        includedirs { FEMFX_INC_DIR, VECTORMATH_INC_DIR, TRACE_DIR }
        includedirs { FEMFX_SRC_DIR.."Bvh/" }
        includedirs { FEMFX_SRC_DIR.."Common/" }
        includedirs { FEMFX_SRC_DIR.."ConstraintSolver/" }
        includedirs { FEMFX_SRC_DIR.."FindContacts/" }
        includedirs { FEMFX_SRC_DIR.."PrimitiveCollision/" }
        includedirs { FEMFX_SRC_DIR.."Scene/" }
        includedirs { FEMFX_SRC_DIR.."Simulation/" }
        includedirs { FEMFX_SRC_DIR.."SparseSolvers/" }
        includedirs { FEMFX_SRC_DIR.."Threading/" }
        vpaths { [("inc")] = {FEMFX_INC_DIR.."*.h"} }
        vpaths { [("src")] = {FEMFX_SRC_DIR.."*.cpp", FEMFX_SRC_DIR.."*.h"} }
        vpaths { [("inc/Vectormath")] = {VECTORMATH_INC_DIR.."*.h"} }
        vpaths { [("Trace")] = {TRACE_DIR.."**.h", TRACE_DIR.."**.cpp"} }
