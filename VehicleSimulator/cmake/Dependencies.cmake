# ---------------------------------------------------------------------------
# Dependency resolution.
#
# Policy: prefer system packages (fast, offline); fall back to FetchContent.
# ImGui has no meaningful system package story, so it is always fetched and
# compiled into a small static library here.
# ---------------------------------------------------------------------------
include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

set(VS_IMGUI_TAG "v1.92.9b")
set(VS_GLFW_TAG  "3.4")
set(VS_GLM_TAG   "1.0.1")
set(VS_YAML_TAG  "0.8.0")

# --- OpenGL ----------------------------------------------------------------
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)

# --- GLM -------------------------------------------------------------------
set(VS_GLM_SOURCE "fetched")
if(VS_USE_SYSTEM_GLM)
    find_package(glm QUIET)
    if(glm_FOUND)
        set(VS_GLM_SOURCE "system")
    endif()
endif()
if(NOT VS_GLM_SOURCE STREQUAL "system")
    FetchContent_Declare(glm
        GIT_REPOSITORY https://github.com/g-truc/glm.git
        GIT_TAG        ${VS_GLM_TAG}
        GIT_SHALLOW    TRUE)
    FetchContent_MakeAvailable(glm)
endif()

# --- yaml-cpp --------------------------------------------------------------
set(VS_YAMLCPP_SOURCE "fetched")
if(VS_USE_SYSTEM_YAMLCPP)
    find_package(yaml-cpp QUIET)
    if(yaml-cpp_FOUND)
        set(VS_YAMLCPP_SOURCE "system")
    endif()
endif()
if(NOT VS_YAMLCPP_SOURCE STREQUAL "system")
    set(YAML_CPP_BUILD_TESTS   OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_TOOLS   OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG        ${VS_YAML_TAG}
        GIT_SHALLOW    TRUE)
    FetchContent_MakeAvailable(yaml-cpp)
endif()

# Older yaml-cpp packages export the bare `yaml-cpp` target rather than the
# namespaced one. Normalise so the rest of the build can use one name.
if(NOT TARGET yaml-cpp::yaml-cpp)
    if(TARGET yaml-cpp)
        add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
    else()
        message(FATAL_ERROR "yaml-cpp was found but exports no usable target")
    endif()
endif()

# --- GLFW ------------------------------------------------------------------
set(VS_GLFW_SOURCE "fetched")
if(VS_USE_SYSTEM_GLFW)
    find_package(glfw3 QUIET)
    if(glfw3_FOUND)
        set(VS_GLFW_SOURCE "system")
    endif()
endif()
if(NOT VS_GLFW_SOURCE STREQUAL "system")
    set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG        ${VS_GLFW_TAG}
        GIT_SHALLOW    TRUE)
    FetchContent_MakeAvailable(glfw)
endif()

# --- cgltf + stb_image ------------------------------------------------------
#
# glTF/GLB model import (src/rendering/GltfModel.h) needs a parser and a PNG
# decoder. Both are single public-domain/MIT headers with no build system of
# their own, so there is nothing to configure - just fetch the two files and
# compile one small translation unit (rendering/GltfLoaderImpl.cpp) that
# defines CGLTF_IMPLEMENTATION / STB_IMAGE_IMPLEMENTATION and includes them.
# This is the same "vendor exactly what is used" policy as GLLoader.h.
set(VS_CGLTF_TAG     "v1.14")
set(VS_STB_IMAGE_TAG "master")   # stb has no tags; master is stable in practice

set(VS_VENDOR_HEADER_DIR "${CMAKE_BINARY_DIR}/_deps/vendor_headers")
file(MAKE_DIRECTORY ${VS_VENDOR_HEADER_DIR})

function(vs_fetch_single_header name url dest)
    if(NOT EXISTS ${dest})
        message(STATUS "Fetching ${name} header...")
        file(DOWNLOAD ${url} ${dest} STATUS VS_DL_STATUS)
        list(GET VS_DL_STATUS 0 VS_DL_CODE)
        if(NOT VS_DL_CODE EQUAL 0)
            file(REMOVE ${dest})
            message(FATAL_ERROR "Failed to download ${name} from ${url}: ${VS_DL_STATUS}")
        endif()
    endif()
endfunction()

vs_fetch_single_header(cgltf
    "https://raw.githubusercontent.com/jkuhlmann/cgltf/${VS_CGLTF_TAG}/cgltf.h"
    "${VS_VENDOR_HEADER_DIR}/cgltf.h")
vs_fetch_single_header(stb_image
    "https://raw.githubusercontent.com/nothings/stb/${VS_STB_IMAGE_TAG}/stb_image.h"
    "${VS_VENDOR_HEADER_DIR}/stb_image.h")

add_library(vendor_gltf INTERFACE)
target_include_directories(vendor_gltf INTERFACE ${VS_VENDOR_HEADER_DIR})

# --- miniaudio (sound output) ------------------------------------------------
# Single header, no system dependency: talks to PipeWire/PulseAudio/ALSA on
# Linux, CoreAudio, WASAPI - whichever exists, loaded at runtime.
set(VS_MINIAUDIO_TAG "0.11.22")
vs_fetch_single_header(miniaudio
    "https://raw.githubusercontent.com/mackron/miniaudio/${VS_MINIAUDIO_TAG}/miniaudio.h"
    "${VS_VENDOR_HEADER_DIR}/miniaudio.h")
add_library(vendor_miniaudio INTERFACE)
target_include_directories(vendor_miniaudio SYSTEM INTERFACE ${VS_VENDOR_HEADER_DIR})

# --- Dear ImGui ------------------------------------------------------------
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        ${VS_IMGUI_TAG}
    GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(imgui)

add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends)
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)
# NOTE: do NOT define IMGUI_IMPL_OPENGL_LOADER_CUSTOM here. The backend tests
# whether that macro is DEFINED, not its value, so even `=0` would suppress
# ImGui's own bundled loader and leave imgui_impl_opengl3.cpp with no GL
# symbols at all. Letting it use its internal loader is exactly what we want:
# it lives in its own translation unit and never collides with the minimal
# loader in src/rendering/GLLoader.h.
