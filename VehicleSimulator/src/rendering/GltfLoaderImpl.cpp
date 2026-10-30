//===========================================================================
//  GltfLoaderImpl.cpp  --  the ONE translation unit that compiles the two
//  vendored single-header libraries backing GltfModel.
//
//  cgltf (parsing) and stb_image (PNG and JPEG decode) are implementation-in-header
//  libraries: including them normally gives declarations only, and exactly
//  one translation unit in the whole program must additionally define
//  CGLTF_IMPLEMENTATION / STB_IMAGE_IMPLEMENTATION before including them,
//  which compiles the function bodies. This file exists to be that one place
//  - every other file includes the headers declaration-only.
//===========================================================================

#define CGLTF_IMPLEMENTATION
#include <cgltf.h>

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_ONLY_JPEG   // the venue's surface textures - rendering/Texture.h
#define STBI_NO_STDIO
#include <stb_image.h>
