#include "rendering/Shader.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <utility>
#include <vector>

#include <glm/gtc/type_ptr.hpp>

#include "config/Config.h"

namespace vsim {
namespace {

/// Compile one stage, reporting the driver's log verbatim on failure.
gl::GLuint compileStage(gl::GLenum type, const std::string& src,
                        const std::string& debugName) {
    gl::GLuint shader = gl::CreateShader(type);
    const char* srcPtr = src.c_str();
    gl::ShaderSource(shader, 1, &srcPtr, nullptr);
    gl::CompileShader(shader);

    gl::GLint ok = 0;
    gl::GetShaderiv(shader, gl::COMPILE_STATUS, &ok);
    if (!ok) {
        gl::GLint logLen = 0;
        gl::GetShaderiv(shader, gl::INFO_LOG_LENGTH, &logLen);
        std::vector<char> log(static_cast<std::size_t>(logLen > 1 ? logLen : 1));
        gl::GetShaderInfoLog(shader, logLen, nullptr, log.data());

        std::cerr << "[shader] " << debugName << " ("
                  << (type == gl::VERTEX_SHADER ? "vertex" : "fragment")
                  << ") failed to compile:\n" << log.data() << "\n";

        gl::DeleteShader(shader);
        return 0;
    }
    return shader;
}

} // namespace

Shader::~Shader() { destroy(); }

Shader::Shader(Shader&& other) noexcept
    : m_program(other.m_program),
      m_name(std::move(other.m_name)),
      m_uniformCache(std::move(other.m_uniformCache)) {
    other.m_program = 0;
}

Shader& Shader::operator=(Shader&& other) noexcept {
    if (this != &other) {
        destroy();
        m_program = other.m_program;
        m_name = std::move(other.m_name);
        m_uniformCache = std::move(other.m_uniformCache);
        other.m_program = 0;
    }
    return *this;
}

void Shader::destroy() {
    if (m_program) {
        gl::DeleteProgram(m_program);
        m_program = 0;
    }
    m_uniformCache.clear();
}

bool Shader::compile(const std::string& vertexSource,
                     const std::string& fragmentSource,
                     const std::string& debugName) {
    destroy();
    m_name = debugName;

    gl::GLuint vs = compileStage(gl::VERTEX_SHADER, vertexSource, debugName);
    if (!vs) return false;

    gl::GLuint fs = compileStage(gl::FRAGMENT_SHADER, fragmentSource, debugName);
    if (!fs) { gl::DeleteShader(vs); return false; }

    gl::GLuint prog = gl::CreateProgram();
    gl::AttachShader(prog, vs);
    gl::AttachShader(prog, fs);
    gl::LinkProgram(prog);

    // The shader objects are reference-counted by the program once attached,
    // so they can be released immediately after linking.
    gl::DeleteShader(vs);
    gl::DeleteShader(fs);

    gl::GLint ok = 0;
    gl::GetProgramiv(prog, gl::LINK_STATUS, &ok);
    if (!ok) {
        gl::GLint logLen = 0;
        gl::GetProgramiv(prog, gl::INFO_LOG_LENGTH, &logLen);
        std::vector<char> log(static_cast<std::size_t>(logLen > 1 ? logLen : 1));
        gl::GetProgramInfoLog(prog, logLen, nullptr, log.data());
        std::cerr << "[shader] " << debugName << " failed to link:\n"
                  << log.data() << "\n";
        gl::DeleteProgram(prog);
        return false;
    }

    m_program = prog;
    return true;
}

bool Shader::loadFromFiles(const std::string& vertexPath,
                           const std::string& fragmentPath) {
    auto readFile = [](const std::string& path, std::string& out) -> bool {
        const std::string resolved = resolveResourcePath(path);
        std::ifstream in(resolved);
        if (!in) {
            std::cerr << "[shader] cannot open '" << resolved << "'\n";
            return false;
        }
        std::ostringstream ss;
        ss << in.rdbuf();
        out = ss.str();
        return true;
    };

    std::string vsrc, fsrc;
    if (!readFile(vertexPath, vsrc) || !readFile(fragmentPath, fsrc)) return false;

    return compile(vsrc, fsrc, vertexPath);
}

void Shader::use() const {
    if (m_program) gl::UseProgram(m_program);
}

gl::GLint Shader::uniformLocation(const std::string& name) const {
    auto it = m_uniformCache.find(name);
    if (it != m_uniformCache.end()) return it->second;

    const gl::GLint loc = gl::GetUniformLocation(m_program, name.c_str());
    // A location of -1 is cached too. An unused uniform gets optimised away by
    // the driver, and re-querying it every frame would be pure waste.
    m_uniformCache.emplace(name, loc);
    return loc;
}

void Shader::setMat4(const std::string& n, const glm::mat4& m) const {
    const gl::GLint l = uniformLocation(n);
    if (l >= 0) gl::UniformMatrix4fv(l, 1, gl::FALSE_, glm::value_ptr(m));
}
void Shader::setVec3(const std::string& n, const glm::vec3& v) const {
    const gl::GLint l = uniformLocation(n);
    if (l >= 0) gl::Uniform3fv(l, 1, glm::value_ptr(v));
}
void Shader::setVec4(const std::string& n, const glm::vec4& v) const {
    const gl::GLint l = uniformLocation(n);
    if (l >= 0) gl::Uniform4fv(l, 1, glm::value_ptr(v));
}
void Shader::setFloat(const std::string& n, float v) const {
    const gl::GLint l = uniformLocation(n);
    if (l >= 0) gl::Uniform1f(l, v);
}
void Shader::setInt(const std::string& n, int v) const {
    const gl::GLint l = uniformLocation(n);
    if (l >= 0) gl::Uniform1i(l, v);
}

} // namespace vsim
