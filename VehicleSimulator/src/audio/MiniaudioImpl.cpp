// The one translation unit that compiles miniaudio's implementation - same
// pattern as rendering/GltfLoaderImpl.cpp for cgltf/stb_image. Built with
// warnings off (see CMakeLists.txt): it is third-party code.
#define MINIAUDIO_IMPLEMENTATION
#define MA_NO_DECODING
#define MA_NO_ENCODING
#define MA_NO_RESOURCE_MANAGER
#define MA_NO_NODE_GRAPH
#define MA_NO_ENGINE
#define MA_NO_GENERATION
#include <miniaudio.h>
