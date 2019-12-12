
SOLUTION_NAME = "obj2poly"

solution ( SOLUTION_NAME )

    filename(SOLUTION_NAME.."_".._ACTION)
    configurations { "Debug", "Release" }
    platforms { "Win32" }
    symbols "On"
    architecture "x86"
    flags { "MultiProcessorCompile" }
    systemversion "10.0.15063.0"

    defines { 
        "WIN32", 
        "WINDOWS", 
        "_WINDOWS",
        "NOMINMAX",
        "_CRT_NONSTDC_NO_DEPRECATE",
        "_CRT_SECURE_NO_DEPRECATE",
        "_CRT_NONSTDC_NO_WARNINGS",       
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
      
    EXE_NAME = "obj2poly"
    SRC_DIR = ""
    project (EXE_NAME)
        filename('%{prj.name}_'.._ACTION)
        targetname(EXE_NAME)
        kind "ConsoleApp"
        language "C++"
        files { SRC_DIR.."*.cpp", SRC_DIR.."*.h" }
        includedirs { SRC_DIR }
