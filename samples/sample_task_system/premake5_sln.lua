SOLUTION_NAME = "sample_task_system"

solution ( SOLUTION_NAME )

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
    targetdir ( 'build/'.._ACTION..'/%{cfg.architecture}/%{cfg.platform}/%{cfg.buildcfg}' )

    filter "kind:StaticLib"
        defines "_LIB"

    filter "configurations:Debug" 
        defines { "_DEBUG" }
        targetsuffix("_d")

    filter "configurations:Release"
        defines { "NDEBUG" }
        optimize "Speed"

	filter "action:vs2017"
		systemversion "10.0.17763.0"
    
include "."

