#pragma once
//===========================================================================
//  Mesh.h  --  indexed vertex buffer wrapper.
//===========================================================================

#include <vector>

#include <glm/glm.hpp>

#include "rendering/GLLoader.h"

namespace vsim {

/// Vertex layout shared by every mesh in the project.
///
/// `uv` exists for imported models (src/rendering/GltfModel.h) that carry a
/// base-colour texture; every procedurally generated mesh leaves it at
/// (0,0) and is drawn with `uUseTexture = 0`, so it is never sampled and has
/// no effect on the flat-lit primitives that make up the rest of the scene.
struct Vertex {
    glm::vec3 position{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    glm::vec3 color{1.0f};
    glm::vec2 uv{0.0f};

    /// Surface material, for the venue pipeline (rendering/Venue.h):
    /// x = material id (see venuemat::), y = a per-material parameter such
    /// as how much rubber is laid on a piece of asphalt. Everything that
    /// predates that pipeline leaves this at (0, 0), which is "plain vertex
    /// colour, gloss from the uGloss uniform" - exactly what it always was.
    glm::vec2 material{0.0f};
};

/// Per-instance data for Mesh::drawInstanced(): a model matrix and a colour
/// multiplied into the mesh's vertex colours - except for vertices whose
/// material parameter (Vertex::material.y) is 1, which keep their own colour
/// (a spectator's face, a parked car's windows). No instanced mesh uses a
/// material that reads that parameter for anything else.
struct InstanceData {
    glm::mat4 transform{1.0f};
    glm::vec4 tint{1.0f};
};

class Mesh {
public:
    Mesh() = default;
    ~Mesh();

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&& other) noexcept;
    Mesh& operator=(Mesh&& other) noexcept;

    /// Upload geometry. `primitive` is gl::TRIANGLES or gl::LINES.
    void upload(const std::vector<Vertex>& vertices,
                const std::vector<gl::GLuint>& indices,
                gl::GLenum primitive = gl::TRIANGLES);

    /// Upload non-indexed geometry.
    void upload(const std::vector<Vertex>& vertices,
                gl::GLenum primitive = gl::TRIANGLES);

    void draw() const;

    /// Draw `count` copies, each placed by one InstanceData starting at
    /// `first` in `instances` (an array buffer of InstanceData). OpenGL 3.3
    /// has no base-instance draw, so the instance attribute pointers are
    /// re-aimed at the requested offset instead; that is four cheap calls.
    void drawInstanced(gl::GLuint instanceBuffer, std::size_t first, std::size_t count) const;

    bool valid() const { return m_vao != 0; }
    std::size_t indexCount() const { return m_indexCount; }
    std::size_t vertexCount() const { return m_vertexCount; }

private:
    void destroy();
    void ensureBuffers();

    gl::GLuint m_vao = 0;
    gl::GLuint m_vbo = 0;
    gl::GLuint m_ebo = 0;

    std::size_t m_vertexCount = 0;
    std::size_t m_indexCount  = 0;
    gl::GLenum  m_primitive   = gl::TRIANGLES;
};

/// A mesh whose contents change every frame (debug lines, trajectory trail).
///
/// Kept separate from Mesh because the usage hint and the re-upload path are
/// genuinely different: a static mesh is uploaded once with STATIC_DRAW, this
/// one is orphaned and refilled with STREAM_DRAW every frame.
class DynamicMesh {
public:
    DynamicMesh() = default;
    ~DynamicMesh();

    DynamicMesh(const DynamicMesh&) = delete;
    DynamicMesh& operator=(const DynamicMesh&) = delete;

    void setVertices(const std::vector<Vertex>& vertices, gl::GLenum primitive);
    void draw() const;
    bool valid() const { return m_vao != 0; }

private:
    void ensureBuffers();

    gl::GLuint m_vao = 0;
    gl::GLuint m_vbo = 0;
    std::size_t m_vertexCount = 0;
    std::size_t m_capacity    = 0;
    gl::GLenum m_primitive = gl::LINES;
};

} // namespace vsim
