SOLUTION_NAME = "FEMFXViewer"
FEMFX_LIB_NAME = "AMD_FEMFX"
TASK_SYSTEM_NAME = "sample_task_system"

solution ( SOLUTION_NAME )

FEMFX_BASE_DIR = "../../"
FEMFX_SRC_DIR = FEMFX_BASE_DIR.."amd_femfx/src/"
FEMFX_INC_DIR = FEMFX_BASE_DIR.."amd_femfx/inc/"
FEMFX_LIB_DIR = FEMFX_BASE_DIR.."amd_femfx/lib/"
VECTORMATH_INC_DIR = FEMFX_BASE_DIR.."amd_femfx/inc/Vectormath/"
TRACE_DIR = FEMFX_BASE_DIR.."external/trace/"
SAMPLES_COMMON_DIR = FEMFX_BASE_DIR.."samples/common/"
METIS_DIR = FEMFX_BASE_DIR.."external/metis-5.1.0/"
JSON_INC_DIR = FEMFX_BASE_DIR.."external/json-develop/include"
TASK_SYSTEM_DIR = FEMFX_BASE_DIR.."samples/sample_task_system/"

-- Solution-wide config

    filename(SOLUTION_NAME.."_".._ACTION)
    configurations { "Debug", "Release" }
    platforms { "x64" }
    symbols "On"
    architecture "x86_64"
    flags { "MultiProcessorCompile", "NoBufferSecurityCheck" }
    floatingpoint "Fast"
    vectorextensions "AVX2"
    exceptionhandling ("Off")
    --callingconvention ("VectorCall")

    defines { 
        "WIN32", 
        "NOMINMAX"
	}

    objdir ('obj/'.._ACTION..'/%{cfg.architecture}')
    targetdir ( 'build/'.._ACTION..'/%{cfg.architecture}/%{cfg.buildcfg}' )

    filter "configurations:Debug" 
        defines { "_DEBUG" }
        targetsuffix("_d")

    filter "configurations:Release"
        defines { "NDEBUG" }
        optimize "Speed"

	filter "action:vs2017"
		systemversion "10.0.17763.0"
    
    EXE_NAME = "FEMFXViewer"
    SRC_DIR = ""
    EXAMPLE_RB_DIR = FEMFX_BASE_DIR.."samples/ExampleRigidBodies/"
    GLFW_INCLUDE_DIR = FEMFX_BASE_DIR.."external/glfw/include/"
    GLFW_LIB_DIR = FEMFX_BASE_DIR.."external/glfw/lib-vc2017/"
    project (EXE_NAME)
        filename('%{prj.name}_'.._ACTION)
        targetname(EXE_NAME)
        kind "ConsoleApp"
        language "C++"
        warnings "Extra"
		buildoptions { '$(FEMFXViewerOptionsEnvVar)' }
        files { SRC_DIR.."*.cpp", SRC_DIR.."*.h", SAMPLES_COMMON_DIR.."*.cpp", SAMPLES_COMMON_DIR.."*.h" }
        files { EXAMPLE_RB_DIR.."*.cpp", EXAMPLE_RB_DIR.."*.h" }     
        vpaths { [("ExampleRigidBodies")] = {EXAMPLE_RB_DIR.."**.h", EXAMPLE_RB_DIR.."**.cpp"} }
        vpaths { [("")] = { SRC_DIR.."*.cpp", SRC_DIR.."*.h", SAMPLES_COMMON_DIR.."*.cpp", SAMPLES_COMMON_DIR.."*.h" } }
        includedirs { FEMFX_INC_DIR, FEMFX_SRC_DIR.."Common/", FEMFX_SRC_DIR.."Bvh/",FEMFX_SRC_DIR.."PrimitiveCollision/", FEMFX_SRC_DIR.."FindContacts/", FEMFX_SRC_DIR.."Scene/", FEMFX_SRC_DIR.."SparseSolvers/", FEMFX_SRC_DIR.."ConstraintSolver/", FEMFX_SRC_DIR.."Simulation/", FEMFX_SRC_DIR.."Threading/" }
		includedirs { VECTORMATH_INC_DIR, TRACE_DIR, EXAMPLE_RB_DIR, TASK_SYSTEM_DIR, JSON_INC_DIR }
        includedirs { GLFW_INCLUDE_DIR, GLFW_DEPS_DIR }
        includedirs { SRC_DIR, SAMPLES_COMMON_DIR }
		libdirs { FEMFX_LIB_DIR, GLFW_LIB_DIR }
        links { FEMFX_LIB_NAME, TASK_SYSTEM_NAME, "glfw3.lib", "odbc32.lib", "odbccp32.lib", "opengl32.lib", "glu32.lib" }
        --includedirs { METIS_DIR.."programs", METIS_DIR.."GKlib", METIS_DIR.."include", METIS_DIR.."libmetis" }
        --defines { "MSC", "USE_GKREGEX", "__thread=__declspec(thread)" } --for metis
        --links { "metis.lib" }
        --filter "configurations:Debug" 
        --    libdirs { METIS_DIR.."libmetis/Debug" }

        --filter "configurations:Release"
        --    libdirs { METIS_DIR.."libmetis/Release" }

include "../../amd_femfx"
include "../../samples/sample_task_system"

