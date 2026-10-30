#include "rendering/Texture.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <utility>

#define STBI_ONLY_PNG
#define STBI_ONLY_JPEG
#define STBI_NO_STDIO
#include <stb_image.h>

namespace vsim {

Texture::~Texture() { destroy(); }

Texture::Texture(Texture&& o) noexcept : m_id(o.m_id) { o.m_id = 0; }

Texture& Texture::operator=(Texture&& o) noexcept {
    if (this != &o) {
        destroy();
        m_id = o.m_id;
        o.m_id = 0;
    }
    return *this;
}

void Texture::destroy() {
    if (m_id) { gl::DeleteTextures(1, &m_id); m_id = 0; }
}

void Texture::upload(const std::uint8_t* pixels, int width, int height) {
    if (!pixels || width <= 0 || height <= 0) return;

    if (!m_id) gl::GenTextures(1, &m_id);

    gl::BindTexture(gl::TEXTURE_2D, m_id);

    // glTF PNGs are not guaranteed 4-byte row alignment (width * 4 is, but
    // callers may hand in RGB data too eventually); 1 is always safe.
    gl::PixelStorei(gl::UNPACK_ALIGNMENT, 1);

    gl::TexImage2D(gl::TEXTURE_2D, 0, static_cast<gl::GLint>(gl::RGBA),
                   width, height, 0, gl::RGBA, gl::UNSIGNED_BYTE_, pixels);

    gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, static_cast<gl::GLint>(gl::LINEAR_MIPMAP_LINEAR));
    gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, static_cast<gl::GLint>(gl::LINEAR));
    gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_S, static_cast<gl::GLint>(gl::REPEAT));
    gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_T, static_cast<gl::GLint>(gl::REPEAT));
    gl::GenerateMipmap(gl::TEXTURE_2D);

    gl::BindTexture(gl::TEXTURE_2D, 0);
}

void Texture::bind(unsigned int unit) const {
    gl::ActiveTexture(gl::TEXTURE0 + unit);
    gl::BindTexture(gl::TEXTURE_2D, m_id);
}

//---------------------------------------------------------------------------
//  TextureArray
//---------------------------------------------------------------------------

TextureArray::~TextureArray() {
    if (m_id) gl::DeleteTextures(1, &m_id);
}

int TextureArray::load(const std::vector<std::string>& paths, int size,
                       const std::uint8_t fallback[3], int anisotropy, bool compress) {
    if (paths.empty() || size <= 0) return 0;
    if (!m_id) gl::GenTextures(1, &m_id);

    const int layers = static_cast<int>(paths.size());
    gl::BindTexture(gl::TEXTURE_2D_ARRAY, m_id);
    gl::PixelStorei(gl::UNPACK_ALIGNMENT, 1);
    (void)gl::GetError();
    if (compress) {
        gl::TexImage3D(gl::TEXTURE_2D_ARRAY, 0, static_cast<gl::GLint>(gl::COMPRESSED_RGB_S3TC_DXT1),
                       size, size, layers, 0, gl::RGB, gl::UNSIGNED_BYTE_, nullptr);
        if (gl::GetError() != 0) compress = false;
    }
    if (!compress) {
        gl::TexImage3D(gl::TEXTURE_2D_ARRAY, 0, static_cast<gl::GLint>(gl::RGB), size, size, layers,
                       0, gl::RGB, gl::UNSIGNED_BYTE_, nullptr);
    }

    std::vector<std::uint8_t> flat(static_cast<std::size_t>(size) * size * 3);
    for (std::size_t i = 0; i < flat.size(); i += 3) {
        flat[i] = fallback[0]; flat[i + 1] = fallback[1]; flat[i + 2] = fallback[2];
    }

    int loaded = 0;
    for (int layer = 0; layer < layers; ++layer) {
        const std::uint8_t* pixels = flat.data();
        unsigned char* decoded = nullptr;

        // stb_image is built without stdio (GltfLoaderImpl.cpp), so the file
        // is read here and decoded from memory.
        std::ifstream in(paths[static_cast<std::size_t>(layer)], std::ios::binary);
        if (in) {
            const std::vector<char> bytes((std::istreambuf_iterator<char>(in)),
                                          std::istreambuf_iterator<char>());
            int w = 0, h = 0, channels = 0;
            decoded = stbi_load_from_memory(reinterpret_cast<const unsigned char*>(bytes.data()),
                                            static_cast<int>(bytes.size()), &w, &h, &channels, 3);
            if (decoded && w == size && h == size) {
                pixels = decoded;
                ++loaded;
            } else {
                std::cerr << "[texture] '" << paths[static_cast<std::size_t>(layer)] << "': "
                          << (decoded ? "wrong size" : stbi_failure_reason()) << "\n";
            }
        } else {
            std::cerr << "[texture] missing '" << paths[static_cast<std::size_t>(layer)] << "'\n";
        }

        gl::TexSubImage3D(gl::TEXTURE_2D_ARRAY, 0, 0, 0, layer, size, size, 1,
                          gl::RGB, gl::UNSIGNED_BYTE_, pixels);
        if (decoded) stbi_image_free(decoded);
    }

    gl::TexParameteri(gl::TEXTURE_2D_ARRAY, gl::TEXTURE_MIN_FILTER, static_cast<gl::GLint>(gl::LINEAR_MIPMAP_LINEAR));
    gl::TexParameteri(gl::TEXTURE_2D_ARRAY, gl::TEXTURE_MAG_FILTER, static_cast<gl::GLint>(gl::LINEAR));
    gl::TexParameteri(gl::TEXTURE_2D_ARRAY, gl::TEXTURE_WRAP_S, static_cast<gl::GLint>(gl::REPEAT));
    gl::TexParameteri(gl::TEXTURE_2D_ARRAY, gl::TEXTURE_WRAP_T, static_cast<gl::GLint>(gl::REPEAT));
    // A road is looked at almost edge-on, which is the case trilinear
    // filtering handles worst: without anisotropic filtering the asphalt
    // turns to grey mush a few car lengths ahead. Not core in 3.3 but
    // supported everywhere; where it is not, the call is a harmless error.
    gl::TexParameteri(gl::TEXTURE_2D_ARRAY, gl::TEXTURE_MAX_ANISOTROPY, std::max(anisotropy, 1));
    (void)gl::GetError();
    gl::GenerateMipmap(gl::TEXTURE_2D_ARRAY);
    gl::BindTexture(gl::TEXTURE_2D_ARRAY, 0);
    return loaded;
}

void TextureArray::bind(unsigned int unit) const {
    gl::ActiveTexture(gl::TEXTURE0 + unit);
    gl::BindTexture(gl::TEXTURE_2D_ARRAY, m_id);
}

} // namespace vsim
