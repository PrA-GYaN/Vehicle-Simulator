#include "rendering/Mesh.h"

#include <utility>

namespace vsim {
namespace {

/// Describe the Vertex layout to the currently bound VAO.
/// One place, so adding an attribute cannot be forgotten in half the meshes.
void configureAttributes() {
    const gl::GLsizei stride = sizeof(Vertex);

    gl::VertexAttribPointer(0, 3, gl::FLOAT_, gl::FALSE_, stride,
                            reinterpret_cast<const void*>(offsetof(Vertex, position)));
    gl::EnableVertexAttribArray(0);

    gl::VertexAttribPointer(1, 3, gl::FLOAT_, gl::FALSE_, stride,
                            reinterpret_cast<const void*>(offsetof(Vertex, normal)));
    gl::EnableVertexAttribArray(1);

    gl::VertexAttribPointer(2, 3, gl::FLOAT_, gl::FALSE_, stride,
                            reinterpret_cast<const void*>(offsetof(Vertex, color)));
    gl::EnableVertexAttribArray(2);

    gl::VertexAttribPointer(3, 2, gl::FLOAT_, gl::FALSE_, stride,
                            reinterpret_cast<const void*>(offsetof(Vertex, uv)));
    gl::EnableVertexAttribArray(3);

    gl::VertexAttribPointer(4, 2, gl::FLOAT_, gl::FALSE_, stride,
                            reinterpret_cast<const void*>(offsetof(Vertex, material)));
    gl::EnableVertexAttribArray(4);
}

} // namespace

//---------------------------------------------------------------------------
//  Mesh
//---------------------------------------------------------------------------

Mesh::~Mesh() { destroy(); }

Mesh::Mesh(Mesh&& o) noexcept
    : m_vao(o.m_vao), m_vbo(o.m_vbo), m_ebo(o.m_ebo),
      m_vertexCount(o.m_vertexCount), m_indexCount(o.m_indexCount),
      m_primitive(o.m_primitive) {
    o.m_vao = o.m_vbo = o.m_ebo = 0;
    o.m_vertexCount = o.m_indexCount = 0;
}

Mesh& Mesh::operator=(Mesh&& o) noexcept {
    if (this != &o) {
        destroy();
        m_vao = o.m_vao; m_vbo = o.m_vbo; m_ebo = o.m_ebo;
        m_vertexCount = o.m_vertexCount; m_indexCount = o.m_indexCount;
        m_primitive = o.m_primitive;
        o.m_vao = o.m_vbo = o.m_ebo = 0;
        o.m_vertexCount = o.m_indexCount = 0;
    }
    return *this;
}

void Mesh::destroy() {
    if (m_ebo) { gl::DeleteBuffers(1, &m_ebo); m_ebo = 0; }
    if (m_vbo) { gl::DeleteBuffers(1, &m_vbo); m_vbo = 0; }
    if (m_vao) { gl::DeleteVertexArrays(1, &m_vao); m_vao = 0; }
    m_vertexCount = m_indexCount = 0;
}

void Mesh::ensureBuffers() {
    if (!m_vao) gl::GenVertexArrays(1, &m_vao);
    if (!m_vbo) gl::GenBuffers(1, &m_vbo);
}

void Mesh::upload(const std::vector<Vertex>& vertices,
                  const std::vector<gl::GLuint>& indices,
                  gl::GLenum primitive) {
    if (vertices.empty()) { destroy(); return; }

    ensureBuffers();
    m_primitive   = primitive;
    m_vertexCount = vertices.size();
    m_indexCount  = indices.size();

    gl::BindVertexArray(m_vao);

    gl::BindBuffer(gl::ARRAY_BUFFER, m_vbo);
    gl::BufferData(gl::ARRAY_BUFFER,
                   static_cast<gl::GLsizeiptr>(vertices.size() * sizeof(Vertex)),
                   vertices.data(), gl::STATIC_DRAW);

    if (!indices.empty()) {
        if (!m_ebo) gl::GenBuffers(1, &m_ebo);
        gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, m_ebo);
        gl::BufferData(gl::ELEMENT_ARRAY_BUFFER,
                       static_cast<gl::GLsizeiptr>(indices.size() * sizeof(gl::GLuint)),
                       indices.data(), gl::STATIC_DRAW);
    }

    configureAttributes();
    gl::BindVertexArray(0);
}

void Mesh::upload(const std::vector<Vertex>& vertices, gl::GLenum primitive) {
    upload(vertices, {}, primitive);
}

void Mesh::draw() const {
    if (!m_vao) return;
    gl::BindVertexArray(m_vao);
    if (m_indexCount > 0) {
        gl::DrawElements(m_primitive, static_cast<gl::GLsizei>(m_indexCount),
                         gl::UNSIGNED_INT, nullptr);
    } else {
        gl::DrawArrays(m_primitive, 0, static_cast<gl::GLsizei>(m_vertexCount));
    }
    gl::BindVertexArray(0);
}

void Mesh::drawInstanced(gl::GLuint instanceBuffer, std::size_t first,
                         std::size_t count) const {
    if (!m_vao || !instanceBuffer || count == 0 || m_indexCount == 0) return;

    gl::BindVertexArray(m_vao);
    gl::BindBuffer(gl::ARRAY_BUFFER, instanceBuffer);

    // Locations 5..8 are the four columns of the matrix, 9 the tint.
    const gl::GLsizei stride = sizeof(InstanceData);
    const std::size_t base = first * sizeof(InstanceData);
    for (gl::GLuint c = 0; c < 5; ++c) {
        gl::VertexAttribPointer(5 + c, 4, gl::FLOAT_, gl::FALSE_, stride,
                                reinterpret_cast<const void*>(base + c * 4 * sizeof(float)));
        gl::EnableVertexAttribArray(5 + c);
        gl::VertexAttribDivisor(5 + c, 1);
    }

    gl::DrawElementsInstanced(m_primitive, static_cast<gl::GLsizei>(m_indexCount),
                              gl::UNSIGNED_INT, nullptr, static_cast<gl::GLsizei>(count));
    gl::BindVertexArray(0);
}

//---------------------------------------------------------------------------
//  DynamicMesh
//---------------------------------------------------------------------------

DynamicMesh::~DynamicMesh() {
    if (m_vbo) gl::DeleteBuffers(1, &m_vbo);
    if (m_vao) gl::DeleteVertexArrays(1, &m_vao);
}

void DynamicMesh::ensureBuffers() {
    if (!m_vao) gl::GenVertexArrays(1, &m_vao);
    if (!m_vbo) gl::GenBuffers(1, &m_vbo);
}

void DynamicMesh::setVertices(const std::vector<Vertex>& vertices,
                              gl::GLenum primitive) {
    m_primitive   = primitive;
    m_vertexCount = vertices.size();
    if (vertices.empty()) return;

    ensureBuffers();
    gl::BindVertexArray(m_vao);
    gl::BindBuffer(gl::ARRAY_BUFFER, m_vbo);

    const std::size_t bytes = vertices.size() * sizeof(Vertex);

    if (vertices.size() > m_capacity) {
        // Grow with headroom so a slowly-lengthening trail does not
        // reallocate every single frame.
        m_capacity = vertices.size() * 2;
        gl::BufferData(gl::ARRAY_BUFFER,
                       static_cast<gl::GLsizeiptr>(m_capacity * sizeof(Vertex)),
                       nullptr, gl::STREAM_DRAW);
    }

    gl::BufferSubData(gl::ARRAY_BUFFER, 0,
                      static_cast<gl::GLsizeiptr>(bytes), vertices.data());

    configureAttributes();
    gl::BindVertexArray(0);
}

void DynamicMesh::draw() const {
    if (!m_vao || m_vertexCount == 0) return;
    gl::BindVertexArray(m_vao);
    gl::DrawArrays(m_primitive, 0, static_cast<gl::GLsizei>(m_vertexCount));
    gl::BindVertexArray(0);
}

} // namespace vsim
