    project ('sample_task_system')
        filename('%{prj.name}_'.._ACTION)
        kind "StaticLib"
        language "C++"
        warnings "Extra"
		buildoptions { '$(SampleTaskSysOptionsEnvVar)' }
        floatingpoint "Fast"
        vectorextensions "AVX2"
        objdir ('obj/'.._ACTION..'/%{cfg.architecture}')
        targetdir ( '../../amd_femfx/lib/' )
        files { "*.cpp", "*.h" }
        vpaths { [("src")] = {"*.cpp"} }
        vpaths { [("inc")] = {"*.h"} }
