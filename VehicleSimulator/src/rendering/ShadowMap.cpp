#include "rendering/ShadowMap.h"

#include <cmath>
#include <iostream>

#include <glm/gtc/matrix_transform.hpp>

namespace vsim {
namespace {

/// Depth-only program. Kept as source rather than a file because it has no
/// tuning knobs at all - there is nothing to edit and reload, unlike the
/// lighting shader.
const char* kDepthVertex = R"(#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 5) in mat4 aInstance;   // Mesh::drawInstanced only

uniform mat4 uLightSpace;
uniform mat4 uModel;
uniform int  uInstanced;

void main() {
    mat4 model = (uInstanced != 0) ? uModel * aInstance : uModel;
    gl_Position = uLightSpace * model * vec4(aPosition, 1.0);
}
)";

const char* kDepthFragment = R"(#version 330 core
void main() { }
)";

} // namespace

ShadowMap::~ShadowMap() { destroy(); }

void ShadowMap::destroy() {
    if (m_depthTexture) { gl::DeleteTextures(1, &m_depthTexture); m_depthTexture = 0; }
    if (m_fbo)          { gl::DeleteFramebuffers(1, &m_fbo);      m_fbo = 0; }
}

bool ShadowMap::initialize(int resolution) {
    destroy();
    m_resolution = resolution;

    if (!m_depthShader.compile(kDepthVertex, kDepthFragment, "shadow_depth")) {
        std::cerr << "[shadow] depth program failed to compile\n";
        return false;
    }

    gl::GenTextures(1, &m_depthTexture);
    gl::BindTexture(gl::TEXTURE_2D, m_depthTexture);
    gl::TexImage2D(gl::TEXTURE_2D, 0, static_cast<gl::GLint>(gl::DEPTH_COMPONENT24),
                   resolution, resolution, 0, gl::DEPTH_COMPONENT, gl::FLOAT_, nullptr);
    gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, static_cast<gl::GLint>(gl::LINEAR));
    gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, static_cast<gl::GLint>(gl::LINEAR));

    // Everything outside the map is LIT, not shadowed. With CLAMP_TO_EDGE a
    // query past the edge would inherit whatever depth sits on the border and
    // smear the last shadow across the rest of the world; a border of 1.0 is
    // "nothing was in front of the light here".
    gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_S, static_cast<gl::GLint>(gl::CLAMP_TO_BORDER));
    gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_T, static_cast<gl::GLint>(gl::CLAMP_TO_BORDER));
    const gl::GLfloat border[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    gl::TexParameterfv(gl::TEXTURE_2D, gl::TEXTURE_BORDER_COLOR, border);

    gl::GenFramebuffers(1, &m_fbo);
    gl::BindFramebuffer(gl::FRAMEBUFFER, m_fbo);
    gl::FramebufferTexture2D(gl::FRAMEBUFFER, gl::DEPTH_ATTACHMENT, gl::TEXTURE_2D,
                             m_depthTexture, 0);
    // Depth only: a framebuffer with no colour attachment is incomplete
    // unless both buffers are explicitly set to NONE.
    gl::DrawBuffer(gl::NONE_);
    gl::ReadBuffer(gl::NONE_);

    const gl::GLenum status = gl::CheckFramebufferStatus(gl::FRAMEBUFFER);
    gl::BindFramebuffer(gl::FRAMEBUFFER, 0);

    if (status != gl::FRAMEBUFFER_COMPLETE) {
        std::cerr << "[shadow] framebuffer incomplete (0x" << std::hex << status
                  << std::dec << "); drawing without shadows\n";
        destroy();
        return false;
    }
    return true;
}

void ShadowMap::update(const glm::vec3& centre, const glm::vec3& lightDirection,
                       float radius) {
    const glm::vec3 dir = glm::normalize(lightDirection);

    // Depth range is generous in both directions: geometry BEHIND the centre
    // relative to the light (a grandstand, a gantry) still has to be in the
    // map, or it stops casting.
    const float depth = radius * 4.0f;

    const glm::vec3 eye = centre - dir * depth * 0.5f;
    const glm::vec3 up  = (std::abs(dir.y) > 0.95f) ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);

    glm::mat4 view = glm::lookAt(eye, centre, up);
    const glm::mat4 proj = glm::ortho(-radius, radius, -radius, radius, 0.0f, depth);

    // Texel snapping. The centre is quantised in LIGHT SPACE to a whole
    // number of shadow texels, so the map's sample grid stays locked to the
    // world as the car moves and shadow edges stop crawling.
    const float texelWorld = 2.0f * radius / static_cast<float>(m_resolution);
    const glm::vec4 centreLS = view * glm::vec4(centre, 1.0f);
    const float snapX = std::round(centreLS.x / texelWorld) * texelWorld - centreLS.x;
    const float snapY = std::round(centreLS.y / texelWorld) * texelWorld - centreLS.y;
    view = glm::translate(glm::mat4(1.0f), glm::vec3(snapX, snapY, 0.0f)) * view;

    m_lightSpace = proj * view;
}

void ShadowMap::beginDepthPass() {
    gl::BindFramebuffer(gl::FRAMEBUFFER, m_fbo);
    gl::Viewport(0, 0, m_resolution, m_resolution);
    gl::Clear(gl::DEPTH_BUFFER_BIT);

    // Front-face culling during the depth pass pushes the recorded depth to
    // the BACK of each occluder, which removes surface acne on everything
    // closed without needing a large constant bias. Flat geometry (the road,
    // the paint) has no back face and still relies on the slope-scaled bias
    // in the lighting shader.
    gl::Enable(gl::CULL_FACE);
    gl::CullFace(gl::FRONT);

    m_depthShader.use();
    m_depthShader.setMat4("uLightSpace", m_lightSpace);
}

void ShadowMap::endDepthPass(int viewportWidth, int viewportHeight) {
    gl::BindFramebuffer(gl::FRAMEBUFFER, 0);
    gl::Viewport(0, 0, viewportWidth, viewportHeight);
    gl::CullFace(gl::BACK);
}

void ShadowMap::setHardwareCompare(bool enabled) {
    if (!m_depthTexture) return;
    gl::BindTexture(gl::TEXTURE_2D, m_depthTexture);
    gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_COMPARE_MODE,
                      static_cast<gl::GLint>(enabled ? gl::COMPARE_REF_TO_TEXTURE : gl::NONE_));
    gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_COMPARE_FUNC, static_cast<gl::GLint>(gl::LEQUAL));
    gl::BindTexture(gl::TEXTURE_2D, 0);
}

void ShadowMap::bindTexture(int unit) const {
    gl::ActiveTexture(gl::TEXTURE0 + static_cast<gl::GLenum>(unit));
    gl::BindTexture(gl::TEXTURE_2D, m_depthTexture);
    gl::ActiveTexture(gl::TEXTURE0);
}

} // namespace vsim
