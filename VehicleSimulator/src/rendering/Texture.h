#pragma once
//===========================================================================
//  Texture.h  --  a single 2D RGBA texture.
//
//  Texture exists for imported models (src/rendering/GltfModel.h) and the
//  venue pipeline's noise. TextureArray, below, holds the venue's surface
//  materials.
//===========================================================================

#include <cstdint>
#include <string>
#include <vector>

#include "rendering/GLLoader.h"

namespace vsim {

class Texture {
public:
    Texture() = default;
    ~Texture();

    Texture(const Texture&) = delete;
    Texture& operator=(const Texture&) = delete;
    Texture(Texture&& other) noexcept;
    Texture& operator=(Texture&& other) noexcept;

    /// Upload RGBA8 pixel data, tightly packed, top-to-bottom or
    /// bottom-to-top - the caller decides by how it wrote `pixels`; this
    /// class does not flip anything.
    void upload(const std::uint8_t* pixels, int width, int height);

    void bind(unsigned int unit = 0) const;

    bool valid() const { return m_id != 0; }
    gl::GLuint id() const { return m_id; }

private:
    void destroy();

    gl::GLuint m_id = 0;
};

/// A stack of same-sized RGB images sampled by layer (sampler2DArray): the
/// venue's surface materials, one layer each (rendering/Venue.h). One
/// sampler serves every material, so a batched mesh holding six of them is
/// still one draw call and the shader needs no sampler per material.
class TextureArray {
public:
    TextureArray() = default;
    ~TextureArray();
    TextureArray(const TextureArray&) = delete;
    TextureArray& operator=(const TextureArray&) = delete;

    /// Load one image file (JPEG or PNG) per layer. Every layer is `size`
    /// square; an image of another size is rejected rather than resampled.
    /// A file that is missing or unreadable does not fail the load: its
    /// layer is filled with `fallback` (r, g, b), chosen by the caller to
    /// mean "no detail" to the shader, and the venue draws without that
    /// material's texture. Returns how many layers came from a file.
    /// `anisotropy` is the maximum anisotropic filtering ratio (1 = off).
    /// `compress` asks the driver to store the array block-compressed
    /// (S3TC/DXT1, a sixth of the memory): on an integrated GPU a texture
    /// fetch is limited by memory traffic, and this is what makes ten
    /// megapixel layers affordable. Falls back to uncompressed if the
    /// driver refuses.
    int load(const std::vector<std::string>& paths, int size, const std::uint8_t fallback[3],
             int anisotropy = 1, bool compress = false);

    void bind(unsigned int unit) const;
    bool valid() const { return m_id != 0; }

private:
    gl::GLuint m_id = 0;
};

} // namespace vsim
