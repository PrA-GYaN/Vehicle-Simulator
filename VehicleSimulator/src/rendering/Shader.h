#pragma once
//===========================================================================
//  Shader.h  --  GLSL program wrapper.
//
//  Shaders are loaded from assets/shaders/. Keeping them as files rather than
//  string literals means they can be edited and reloaded without rebuilding,
//  which matters a lot when tuning the look of the debug overlays.
//===========================================================================

#include <string>
#include <unordered_map>

#include <glm/glm.hpp>

#include "rendering/GLLoader.h"

namespace vsim {

class Shader {
public:
    Shader() = default;
    ~Shader();

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    Shader(Shader&& other) noexcept;
    Shader& operator=(Shader&& other) noexcept;

    /// Compile and link from source strings. Returns false and logs on error.
    bool compile(const std::string& vertexSource,
                 const std::string& fragmentSource,
                 const std::string& debugName = "shader");

    /// Compile from files under assets/shaders/.
    bool loadFromFiles(const std::string& vertexPath,
                       const std::string& fragmentPath);

    void use() const;
    bool valid() const { return m_program != 0; }
    gl::GLuint id() const { return m_program; }

    //-- uniforms -------------------------------------------------------
    //
    // Locations are cached: glGetUniformLocation is a string lookup inside the
    // driver and calling it per-uniform per-draw is a measurable cost.
    void setMat4(const std::string& name, const glm::mat4& m) const;
    void setVec3(const std::string& name, const glm::vec3& v) const;
    void setVec4(const std::string& name, const glm::vec4& v) const;
    void setFloat(const std::string& name, float v) const;
    void setInt(const std::string& name, int v) const;

private:
    gl::GLint uniformLocation(const std::string& name) const;
    void destroy();

    gl::GLuint m_program = 0;
    std::string m_name;
    mutable std::unordered_map<std::string, gl::GLint> m_uniformCache;
};

} // namespace vsim
