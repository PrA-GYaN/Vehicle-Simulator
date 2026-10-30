#pragma once
//===========================================================================
//  GltfModel.h  --  an imported glTF/GLB vehicle model.
//
//  WHAT THIS IS FOR
//  -----------------
//  MeshFactory generates every OTHER mesh in the project procedurally, on
//  purpose (see its header comment): the shapes are simple, and driving them
//  from the vehicle parameters keeps the visual honest. A real car model is
//  the one exception that is worth an importer - nobody is going to
//  procedurally generate a Mercedes-AMG GT3 body shell.
//
//  This loader is deliberately narrow. It does not attempt general-purpose
//  glTF support:
//
//    * ONE static mesh group for everything that is not a wheel, baked into
//      vehicle-local space at load time (position/normal only - no runtime
//      skeleton, no animation).
//    * FOUR wheel mesh groups, one per WheelIndex, each baked relative to
//      that wheel's OWN hub, ready to be re-positioned every frame by
//      exactly the same transform chain Renderer already applies to the
//      procedural wheel mesh (see Renderer::drawVehicle). The wheel's
//      PHYSICAL position still comes from the vehicle's own track/wheelbase
//      parameters, never from the model file - a wheel node here contributes
//      its shape and nothing else.
//    * Base-colour texturing only. No normal maps, no metallic/roughness,
//      no PBR. KB-grade honesty applies here too: this is a vehicle DYNAMICS
//      tool, and the shading exists so the geometry reads correctly on
//      screen, not to reproduce the source asset's material response.
//    * alphaMode BLEND materials (glass) are drawn transparent, last and
//      unsorted - see GltfPrimitive::transparent. MASK is treated as
//      opaque.
//
//  COORDINATE REMAPPING
//  ---------------------
//  glTF has its own axis convention, chosen by whatever tool exported the
//  asset - it is not the project's vehicle-local frame (+X forward, +Y up,
//  +Z right, origin at the REAR AXLE - see core/Coordinates.h). `ImportAxes`
//  says which glTF axis is "forward" and which is "up"; the right axis is
//  DERIVED as forward x up, which is the same right-handed relationship the
//  project's own frame uses, so an orthonormal rotation always comes out of
//  just those two choices.
//
//  The translation half of the remap is not authored at all: it is measured
//  from the model's own wheel hub nodes. Whatever point sits at the midpoint
//  of the two REAR wheel hubs becomes vehicle-local (0, hubY, 0) before the Y
//  component is discarded (ground is wherever the model's own geometry rests
//  at hub-height minus wheel radius; forcing it to 0 is what "origin at the
//  rear axle, ground plane" means). This is measurement, not configuration -
//  the four wheel hub node NAMES are the only thing that has to be told to
//  the loader, because names are not discoverable from geometry.
//===========================================================================

#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <glm/glm.hpp>

#include "core/WheelState.h"
#include "rendering/Mesh.h"
#include "rendering/Texture.h"

namespace vsim {

/// Which glTF axes map to the project's vehicle-local forward and up. The
/// right axis is derived as forward x up. Accepted strings: "+x","-x","+y",
/// "-y","+z","-z" (case-insensitive).
struct ImportAxes {
    std::string forward = "+z";
    std::string up      = "+y";
};

/// Node names identifying each wheel's hub assembly. Every node at or below
/// the named node in the scene graph is treated as belonging to that wheel
/// and is excluded from the static (chassis) mesh group.
struct WheelNodeNames {
    std::string frontLeft;
    std::string frontRight;
    std::string rearLeft;
    std::string rearRight;

    bool empty() const {
        return frontLeft.empty() && frontRight.empty() &&
               rearLeft.empty()  && rearRight.empty();
    }
};

/// One drawable piece: a baked mesh plus the material that colours it.
struct GltfPrimitive {
    Mesh mesh;
    std::shared_ptr<Texture> baseColorTexture;  ///< null = untextured
    glm::vec3 tint{1.0f};                       ///< material baseColorFactor
    /// glTF alphaMode BLEND: drawn after everything opaque, blended, with
    /// depth writes off - enough for a car's glass, which is a handful of
    /// panels that rarely overlap each other from any one viewpoint.
    bool  transparent = false;
    float alpha = 1.0f;                          ///< baseColorFactor.a
    bool  textureAlpha = false;                  ///< opacity also from the texture's alpha
};

class GltfModel {
public:
    /// Load and bake a .glb file. Returns false (and logs why) on any
    /// failure - the caller's fallback is always the procedural body, so a
    /// bad path or a malformed file is recoverable, never fatal.
    bool load(const std::string& glbPath, const ImportAxes& axes,
              const WheelNodeNames& wheelNodes);

    bool loaded() const { return m_loaded; }

    const std::vector<GltfPrimitive>& staticPrimitives() const { return m_static; }
    const std::vector<GltfPrimitive>& wheelPrimitives(int wheel) const {
        return m_wheels[static_cast<std::size_t>(wheel)];
    }

    /// True if this wheel index actually had a matching node - i.e. whether
    /// wheelPrimitives(wheel) is this model's own geometry or will be empty
    /// (in which case the caller should fall back to the procedural wheel).
    bool hasWheel(int wheel) const { return m_hasWheel[static_cast<std::size_t>(wheel)]; }

    /// Height of this wheel's hub node above the REAR hubs' mean height, in
    /// the model's own geometry [m]. Zero for the rear wheels and for any
    /// model whose four hubs sit level. Non-zero for a car on staggered tyre
    /// sizes (a smaller front tyre puts the front hub lower): the physics has
    /// one wheel radius, so the renderer adds this to the physics hub height
    /// for the model's own wheels only - measured from the file, like the
    /// rear-axle origin, never authored.
    float hubHeightOffset(int wheel) const { return m_hubHeightOffset[static_cast<std::size_t>(wheel)]; }

    /// Axis-aligned bounds of the STATIC geometry only, in whatever frame it
    /// was baked into (vehicle-local for a car, world for a scenery model
    /// loaded with empty WheelNodeNames - see importTransform in the .cpp).
    /// Zero if load() found no static geometry.
    glm::vec3 boundsMin() const { return m_boundsMin; }
    glm::vec3 boundsMax() const { return m_boundsMax; }

    /// Raw positions of every static-geometry vertex, in the same frame as
    /// boundsMin()/boundsMax(). Exists so a scenery model's ground shape can
    /// be handed to environment::TerrainHeightField::build() without
    /// re-parsing the glb - GltfModel already has the baked positions in
    /// hand while uploading them to the GPU. Empty for a model with no
    /// static geometry (e.g. wheels only, which does not happen in
    /// practice).
    const std::vector<glm::vec3>& staticPoints() const { return m_staticPoints; }

private:
    std::vector<GltfPrimitive> m_static;
    std::array<std::vector<GltfPrimitive>, WheelCount> m_wheels;
    std::array<bool, WheelCount> m_hasWheel{};
    std::array<float, WheelCount> m_hubHeightOffset{};
    bool m_loaded = false;
    // scratch for load(): the material alpha of the primitive being baked
    float m_lastAlpha = 1.0f;
    bool  m_lastBlend = false;
    bool  m_lastTextureAlpha = false;
    std::unordered_map<const void*, bool> m_translucentImages;

    std::vector<glm::vec3> m_staticPoints;
    glm::vec3 m_boundsMin{0.0f};
    glm::vec3 m_boundsMax{0.0f};
};

} // namespace vsim
