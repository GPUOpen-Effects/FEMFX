FEMFX Viewer

A basic graphical sample for simulations of several FEM test scenes.
Starts with simulation paused and can be interacted with using controls below.

Run "premake5 vs2019" in this directory to create the visual studio files.
(https://premake.github.io/download.html)

Several external libraries are required in the external directory:

glfw\         : Download Windows pre-compiled binaries from https://www.glfw.org/download.html
json-develop\ : Download from https://github.com/nlohmann/json

Configuration options for the scenes are in TestScenes.[h,cpp]

Controls:
Left mouse drag: rotate the scene
Middle mouse drag: pan
Right mouse drag down/up: zoom in/out
'p': toggle paused/running
'r': reset simulation
'q': quit
'f': fire projectile (WOOD_PANELS_SCENE or CARS_PANELS_TIRES_SCENE)
'h': fire higher-speed projectile (WOOD_PANELS_SCENE or CARS_PANELS_TIRES_SCENE)
'l': launch cars in CARS_PANELS_TIRES_SCENE
'c': display distance contact debug
'v': display volume contact debug
'w': toggle wireframe
']': step simulation forward
'[': step simulation backward (reruns to previous frame, requires FIX_INITIAL_CONDITIONS in TestScenes.h)
