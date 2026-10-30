#pragma once
//===========================================================================
//  ShadowMap.h  --  one directional-light shadow map.
//
//  WHY A VEHICLE DYNAMICS TOOL HAS SHADOWS
//  ---------------------------------------
//  Not for looks alone. Without a contact shadow there is no visual cue for
//  where the car IS relative to the ground: a car in the air and a car on the
//  road render identically under a directional light with no occlusion, which
//  matters here because the renderer now actually shows terrain-driven heave,
//  pitch and roll (see Renderer::drawVehicle). The shadow under the car is
//  what makes suspension travel and a wheel lifting off legible.
//
//  HOW IT WORKS
//  ------------
//  One 2048^2 depth texture, rendered from the light's direction with an
//  orthographic projection FITTED AROUND THE VEHICLE rather than around the
//  whole circuit. A 5 km track covered by one shadow map would give each
//  texel about 2.5 m of world - useless. Following the car keeps texels at a
//  few centimetres where they are actually looked at, at the cost of no
//  shadows in the far distance, which nobody will look for.
//
//  The projection is SNAPPED TO THE TEXEL GRID each frame. Without that, a
//  shadow map that moves with the camera swims: the texel boundaries slide
//  under static geometry and every shadow edge crawls. Snapping costs two
//  rounding operations and removes the effect completely.
//===========================================================================

#include <glm/glm.hpp>

#include "rendering/GLLoader.h"
#include "rendering/Shader.h"

namespace vsim {

class ShadowMap {
public:
    ~ShadowMap();

    ShadowMap(const ShadowMap&) = delete;
    ShadowMap& operator=(const ShadowMap&) = delete;
    ShadowMap() = default;

    /// Allocate the depth texture, framebuffer and depth-only program.
    /// Returns false (and leaves valid() false) if any of that fails - the
    /// renderer then draws unshadowed rather than not at all.
    bool initialize(int resolution = 2048);

    bool valid() const { return m_fbo != 0 && m_depthShader.valid(); }

    /// Compute the light-space matrix for this frame. `centre` is the point
    /// the map is fitted around (the vehicle), `radius` the half-size of the
    /// covered box in metres.
    void update(const glm::vec3& centre, const glm::vec3& lightDirection, float radius);

    /// Bind the depth framebuffer and prepare for the depth-only pass. The
    /// caller then draws the scene with depthShader().
    void beginDepthPass();

    /// Restore the default framebuffer and the given viewport.
    void endDepthPass(int viewportWidth, int viewportHeight);

    /// Bind the depth texture to a texture unit for the lighting pass.
    void bindTexture(int unit) const;

    /// Switch the depth texture between plain sampling (read the stored
    /// depth through a sampler2D - what the original lighting path does) and
    /// hardware comparison (a sampler2DShadow returns the bilinearly
    /// filtered result of the depth test - what the venue path uses, because
    /// one such tap is worth four plain ones). A texture is one or the
    /// other, so the renderer sets this when the pipeline changes.
    void setHardwareCompare(bool enabled);

    const Shader& depthShader() const { return m_depthShader; }
    Shader&       depthShader()       { return m_depthShader; }

    const glm::mat4& lightSpaceMatrix() const { return m_lightSpace; }
    int resolution() const { return m_resolution; }

private:
    void destroy();

    gl::GLuint m_fbo = 0;
    gl::GLuint m_depthTexture = 0;
    int m_resolution = 0;

    Shader m_depthShader;
    glm::mat4 m_lightSpace{1.0f};
};

} // namespace vsim
