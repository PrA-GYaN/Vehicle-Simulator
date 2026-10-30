cmake_minimum_required(VERSION 3.20)
project(VehicleSimulator VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(VS_USE_SYSTEM_GLM      "Use system GLM if available"      ON)
option(VS_USE_SYSTEM_YAMLCPP  "Use system yaml-cpp if available" ON)
option(VS_USE_SYSTEM_GLFW     "Use system GLFW if available"     ON)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Dependencies.cmake)

# --------------------------------------------------------------------------
# Simulation core library.
#
# IMPORTANT ARCHITECTURAL RULE:
#   `vsim_core` contains configuration, physics, vehicle, simulation,
#   environment, telemetry and experiments.  It links ONLY against GLM and
#   yaml-cpp.  It has no knowledge of OpenGL, GLFW or ImGui.
#
#   This is enforced by the link libraries below - if anyone ever tries to
#   #include a rendering header from the simulation side, the build breaks.
# --------------------------------------------------------------------------
add_library(vsim_core STATIC
    src/core/Units.cpp
    src/config/Config.cpp
    src/physics/RoadLoad.cpp
    src/physics/SuspensionModel.cpp
    src/physics/TireModel.cpp
    src/physics/KinematicBicycleModel.cpp
    src/physics/FourWheelModel.cpp
    src/physics/VehicleDynamicsFactory.cpp
    src/physics/Engine.cpp
    src/physics/ResolvedEngine.cpp
    src/physics/EngineDyno.cpp
    src/physics/enginecore/enginecore.c
    src/physics/Gearbox.cpp
    src/physics/Powertrain.cpp
    src/vehicle/Vehicle.cpp
    src/simulation/Simulation.cpp
    src/environment/Track.cpp
    src/environment/circuits/ApexGrandPrix.cpp
    src/environment/TerrainHeightField.cpp
    src/telemetry/Telemetry.cpp
    src/experiments/Experiment.cpp
)
target_include_directories(vsim_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(vsim_core PUBLIC glm::glm yaml-cpp::yaml-cpp)

# --------------------------------------------------------------------------
# The resolved engine's cylinder core.
#
# src/physics/enginecore/enginecore.c is EngineSim's compiled inner loop,
# VENDORED unchanged by tools/sync_enginecore.py (with fields.h and
# gas_tables.inc) - see physics/ResolvedEngine.h. Two things follow:
#
#   * It is built with the flags EngineSim builds it with (native_core.py):
#     no FMA contraction, no fast-math. EngineSim's parity tests prove the C
#     against its Python reference under exactly these, and tools/engine_parity
#     proves this port against EngineSim; either relaxation changes rounding
#     and breaks that chain. ResolvedEngine.cpp gets the same flags, because
#     it is the other half of the same arithmetic.
#   * It is third-party code as far as warnings go: not ours to fix here.
# --------------------------------------------------------------------------
if(MSVC)
    set(VS_ENGINECORE_FLAGS /fp:precise)
else()
    set(VS_ENGINECORE_FLAGS -ffp-contract=off -fno-fast-math)
endif()
set_source_files_properties(src/physics/enginecore/enginecore.c PROPERTIES
    COMPILE_OPTIONS "${VS_ENGINECORE_FLAGS};$<IF:$<BOOL:${MSVC}>,/w,-w>")
set_source_files_properties(src/physics/ResolvedEngine.cpp PROPERTIES
    COMPILE_OPTIONS "${VS_ENGINECORE_FLAGS}")
# EngineSim's sound synthesiser, transcribed: checked sample for sample
# against the JavaScript original, so the same no-contraction rule applies.
set_source_files_properties(src/audio/EngineSimSynth.cpp PROPERTIES
    COMPILE_OPTIONS "${VS_ENGINECORE_FLAGS}")
if(NOT WIN32)
    target_link_libraries(vsim_core PUBLIC m)
endif()

# Say so at configure time if the vendored core has fallen behind EngineSim.
# A warning, not an error: the simulator builds and runs standalone, without
# EngineSim or Python present.
find_package(Python3 QUIET COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../EngineSim/engine_sim/native/enginecore.c)
    execute_process(
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/sync_enginecore.py --check
        RESULT_VARIABLE VS_ENGINECORE_STALE OUTPUT_QUIET ERROR_QUIET)
    if(VS_ENGINECORE_STALE EQUAL 1)
        message(WARNING "src/physics/enginecore/ is out of date with ../EngineSim. "
                        "Run tools/sync_enginecore.py, rebuild, and re-run engine_parity "
                        "(README: 'Keeping the port in step with EngineSim').")
    endif()
endif()

# --------------------------------------------------------------------------
# Presentation layer + executable.
#   Rendering / input / UI / application may depend on the simulation core,
#   but never the other way around.
# --------------------------------------------------------------------------
add_executable(VehicleSimulator
    src/main.cpp
    src/application/Application.cpp
    src/input/Input.cpp
    src/input/AndroidUsbInput.cpp
    src/rendering/GLLoader.cpp
    src/rendering/Shader.cpp
    src/rendering/Mesh.cpp
    src/rendering/MeshFactory.cpp
    src/rendering/CarMesh.cpp
    src/rendering/ShadowMap.cpp
    src/rendering/TrackMesh.cpp
    src/rendering/Venue.cpp
    src/rendering/ApexVenueKit.cpp
    src/rendering/ApexVenue.cpp
    src/rendering/ApexVenueSurfaces.cpp
    src/rendering/ApexVenueTrackside.cpp
    src/rendering/ApexVenueBuildings.cpp
    src/rendering/ApexVenueLandscape.cpp
    src/rendering/Texture.cpp
    src/rendering/GltfModel.cpp
    src/rendering/GltfLoaderImpl.cpp
    src/rendering/Camera.cpp
    src/rendering/DebugDraw.cpp
    src/rendering/Renderer.cpp
    src/ui/DebugUI.cpp
    src/audio/SoundSynth.cpp
    src/audio/EngineSimSynth.cpp
    src/audio/SoundStateBuilder.cpp
    src/audio/AudioOutput.cpp
    src/audio/MiniaudioImpl.cpp
)
# Third-party implementation unit: not our warnings to fix.
set_source_files_properties(src/audio/MiniaudioImpl.cpp PROPERTIES COMPILE_OPTIONS "-w")
# --------------------------------------------------------------------------
# Offline tools.
#
# Not part of the simulator and not on any startup path: these are run by hand
# and their OUTPUT is committed. See tools/extract_track_centreline.cpp for
# why a reference scan is turned into a centreline table rather than shipped.
# Built with the default target because it is a few hundred lines and keeps
# itself honest that way.
# --------------------------------------------------------------------------
add_executable(extract_track_centreline tools/extract_track_centreline.cpp
                                        src/rendering/GltfLoaderImpl.cpp)
target_link_libraries(extract_track_centreline PRIVATE vendor_gltf)

# Engine tools: a dynamometer for a vehicle's detailed engine, and the check
# of the resolved-engine port against a trace recorded from EngineSim. Both
# link only the simulation core - no window, no audio.
add_executable(engine_dyno tools/engine_dyno.cpp)
target_link_libraries(engine_dyno PRIVATE vsim_core)
# Drives a lap of a circuit with an autopilot through the real simulation and
# reports whether the car stayed on the road. See tools/lap_test.cpp.
add_executable(lap_test tools/lap_test.cpp)
target_link_libraries(lap_test PRIVATE vsim_core)
add_executable(engine_parity tools/engine_parity.cpp)
target_link_libraries(engine_parity PRIVATE vsim_core)
# The same check for the sound: audio/EngineSimSynth against EngineSim's
# worklet rendered under Node. Needs the two synthesiser sources but no device.
add_executable(engine_sound_parity tools/engine_sound_parity.cpp
                                   src/audio/EngineSimSynth.cpp src/audio/SoundSynth.cpp)
target_link_libraries(engine_sound_parity PRIVATE vsim_core)

find_package(Threads REQUIRED)
target_link_libraries(VehicleSimulator PRIVATE vsim_core glfw imgui vendor_gltf vendor_miniaudio
                                               OpenGL::GL Threads::Threads ${CMAKE_DL_LIBS})
if(WIN32)
    target_link_libraries(VehicleSimulator PRIVATE ws2_32)
endif()

if(MSVC)
    target_compile_options(VehicleSimulator PRIVATE /W4)
    target_compile_options(vsim_core        PRIVATE /W4)
else()
    target_compile_options(VehicleSimulator PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(vsim_core        PRIVATE -Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------------------
# Runtime assets.
#
# The executable resolves `config/` and `assets/` relative to the current
# working directory first, then relative to the executable.  We copy them next
# to the binary so `./build/bin/VehicleSimulator` works from anywhere.
# --------------------------------------------------------------------------
add_custom_command(TARGET VehicleSimulator POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/config $<TARGET_FILE_DIR:VehicleSimulator>/config
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/assets $<TARGET_FILE_DIR:VehicleSimulator>/assets
    COMMAND ${CMAKE_COMMAND} -E make_directory
            $<TARGET_FILE_DIR:VehicleSimulator>/logs
    COMMENT "Staging config/ and assets/ next to the executable"
)

message(STATUS "")
message(STATUS "VehicleSimulator ${PROJECT_VERSION}")
message(STATUS "  build type : ${CMAKE_BUILD_TYPE}")
message(STATUS "  GLM        : ${VS_GLM_SOURCE}")
message(STATUS "  yaml-cpp   : ${VS_YAMLCPP_SOURCE}")
message(STATUS "  GLFW       : ${VS_GLFW_SOURCE}")
message(STATUS "  ImGui      : fetched (${VS_IMGUI_TAG})")
message(STATUS "")
