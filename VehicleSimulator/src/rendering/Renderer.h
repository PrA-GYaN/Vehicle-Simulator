#pragma once
//===========================================================================
//  Renderer.h  --  draws the simulation. Reads state, never writes it.
//
//  Every method takes `const Simulation&` / `const Vehicle&`. That const is
//  the architectural boundary: the renderer is a pure function of simulation
//  state, so removing it entirely would leave the physics untouched.
//===========================================================================

#include <array>
#include <memory>
#include <string>

#include <glm/glm.hpp>

#include "config/Config.h"
#include "environment/Track.h"
#include "environment/TerrainHeightField.h"
#include "rendering/Camera.h"
#include "rendering/CarMesh.h"
#include "rendering/DebugDraw.h"
#include "rendering/GltfModel.h"
#include "rendering/Mesh.h"
#include "rendering/Shader.h"
#include "rendering/ShadowMap.h"
#include "rendering/Texture.h"
#include "rendering/TrackMesh.h"
#include "rendering/Venue.h"

namespace vsim {

class Simulation;
class Vehicle;

/// Colours and lighting, exposed so the UI can tweak them live.
struct RenderStyle {
    glm::vec3 skyColor    {0.53f, 0.68f, 0.82f};
    glm::vec3 groundColor {0.27f, 0.36f, 0.19f};
    glm::vec3 bodyColor   {0.15f, 0.42f, 0.78f};
    glm::vec3 accentColor {0.93f, 0.94f, 0.96f};
    glm::vec3 glassColor  {0.13f, 0.17f, 0.22f};

    glm::vec3 lightDirection{-0.42f, -0.72f, -0.36f};
    glm::vec3 lightColor  {1.00f, 0.97f, 0.90f};
    float ambient = 0.55f;

    /// Exponential-squared fog toward the sky colour. At 0.0011 the far side
    /// of a 5 km circuit is fully faded while the car's own surroundings are
    /// untouched - it is a depth cue, not weather.
    float fogDensity = 0.0011f;

    /// Shadow map coverage, in metres either side of the vehicle. Larger
    /// covers more scenery, smaller puts more texels on the car.
    float shadowRadius = 42.0f;
    bool  shadows = true;

    /// Racing surface, paint, kerbs and runoff - everything you drive on or
    /// off. Off leaves the bare ground plane, which is what the pre-model
    /// debug view looked like.
    bool showRoad = true;

    /// Barriers, gantry, pit complex, grandstands, boards and trees. Separate
    /// from showRoad because it is the half of the circuit that exists purely
    /// to make the scene readable, and the half worth switching off when
    /// looking at the car.
    bool showScenery = true;

    bool wireframe = false;

    //-- venue pipeline only (rendering/Venue.h) -------------------------------
    //
    // A venue circuit is lit in linear light and tone mapped, so it needs
    // its own numbers: the sun is several times brighter than the sky, and
    // `ambient` above (tuned for the original gamma-space look) would wash
    // it out. Everything else - light direction and colour, shadows on/off,
    // the show* switches - is shared.
    float venueSunIntensity = 3.3f;
    float venueAmbient      = 1.15f;
    float venueExposure     = 0.92f;
    float venueFogDensity   = 0.00034f;   ///< plain exponential, per metre
    float venueShadowRadius = 36.0f;      ///< near map; tighter than shadowRadius
    float venueFarShadowRadius = 230.0f;  ///< far map: buildings, stands, trees
};

/// What the last frame cost, for the performance readout.
struct FrameStats {
    int drawCalls = 0;
    long long triangles = 0;
};

class Renderer {
public:
    /// Create GL resources. Requires a current GL context.
    bool initialize(const Track& track);

    /// Rebuild the vehicle meshes after the vehicle parameters change, so the
    /// drawn car always matches the configured dimensions.
    void rebuildVehicleMeshes(const VehicleParameters& params);

    /// Rebuild environment meshes for a new track.
    void rebuildTrackMeshes(const Track& track);

    /// Load (or clear, for an empty path) the environment 3D model - a real
    /// terrain scan drawn AND driven on, instead of the procedural
    /// ground/road (see config::EnvironmentModelConfig). Also builds the
    /// TerrainHeightField the physics queries (terrainField()) and resizes
    /// the debug grid to the model's own footprint. Returns true if a model
    /// was requested and loaded successfully.
    bool loadEnvironmentModel(const EnvironmentModelConfig& cfg);

    /// Build (or clear) terrainField() from a GENERATED track's own
    /// centreline, for a track like createNordschleife() that carries real
    /// elevation but - unlike "nurburgring_scan" - has no imported scan to
    /// build a TerrainHeightField from. Samples the road surface across its
    /// own width (not just the centreline) at each resampled point, so the
    /// field has real coverage under the wheels, not just down the racing
    /// line. A no-op that leaves terrainField() null for every flat track
    /// (track.hasElevation() false) - which is every preset except this one
    /// today - so calling it unconditionally on every track load is safe.
    ///
    /// Shares m_terrain with loadEnvironmentModel(): the two are mutually
    /// exclusive (a track is either the imported scan or a generated one),
    /// and the caller (Application::applyEnvironmentModel()) always calls
    /// exactly one of them per track load, so whichever ran last is correct.
    void buildTrackTerrain(const Track& track);

    /// True when an environment model is active - drawEnvironment() then
    /// draws it INSTEAD of the procedural ground/road.
    bool hasEnvironmentModel() const {
        return m_environmentModel && m_environmentModel->loaded();
    }

    /// Raw static-geometry points of the active environment model, in world
    /// space - the same source used to build terrainField(). Exists so a
    /// spawn-point heuristic can search actual scanned geometry directly
    /// (see Application::applyEnvironmentModel()) rather than only the
    /// binned heightfield, which is mostly EMPTY over most of the model's
    /// bounding box for a asset like a track scan. Empty when there is no
    /// environment model.
    const std::vector<glm::vec3>& environmentModelPoints() const {
        static const std::vector<glm::vec3> empty;
        return m_environmentModel ? m_environmentModel->staticPoints() : empty;
    }

    /// Ground height field for the active track, or null when the current
    /// track is flat (physics then sees ground height 0 everywhere - see
    /// FourWheelModel). Built by whichever of loadEnvironmentModel() /
    /// buildTrackTerrain() last ran for the current track. Non-owning; the
    /// caller (Application) hands this to Vehicle::setTerrain() - see
    /// IVehicleDynamics::update().
    const TerrainHeightField* terrainField() const {
        return m_terrain.valid() ? &m_terrain : nullptr;
    }

    void setViewport(int width, int height);

    /// True when the active track is drawn as a venue (rendering/Venue.h)
    /// rather than by the generic TrackMesh generator.
    bool venueActive() const { return m_venueActive; }
    const Venue& venue() const { return m_venue; }

    /// Venue draw calls and triangles submitted by the last render(), all
    /// passes included. Zero on a track that is not a venue.
    FrameStats lastFrameStats() const { return m_frameStats; }

    /// Write the back buffer to a binary PPM. Call after render() and
    /// before the buffers are swapped. Returns false if the file could not
    /// be written.
    bool saveScreenshot(const std::string& path) const;

    /// Draw one frame.
    void render(const Simulation& sim, const Camera& camera,
                const DebugDrawOptions& debugOptions);

    RenderStyle&       style()       { return m_style; }
    const RenderStyle& style() const { return m_style; }

    DebugDraw& debug() { return m_debug; }

    int viewportWidth()  const { return m_width; }
    int viewportHeight() const { return m_height; }
    float aspect() const {
        return (m_height > 0) ? static_cast<float>(m_width) / static_cast<float>(m_height)
                              : 16.0f / 9.0f;
    }

private:
    /// The scene, drawn identically by the shadow pass and the lighting pass.
    /// `m_activeShader` is whichever program is bound, so neither pass needs
    /// its own copy of the traversal - and geometry cannot end up in one pass
    /// but not the other.
    void drawEnvironment();
    /// Catch fencing and anything else blended. A separate call because its
    /// place in the frame depends on the camera - see render().
    void drawEnvironmentTransparent();
    void drawVenue();
    void drawSky();
    void buildNoiseTexture();
    void loadMaterialTextures();
    void drawVehicle(const Vehicle& vehicle);
    void drawDebugOverlays(const Simulation& sim, const DebugDrawOptions& opt);

    /// Fill the shadow map from the light's point of view, around the vehicle.
    void renderShadowPass(const Vehicle& vehicle);
    /// Venue only: the wide map. Static scenery only, and refreshed every
    /// few frames rather than every frame - nothing in it moves.
    void renderFarShadowPass(const Vehicle& vehicle);

    const Shader& shader() const { return *m_activeShader; }

    /// Shared by drawVehicle() and drawEnvironment(): draw one GltfModel
    /// primitive group under `transform`, texture-modulated if it has a
    /// base-colour texture, tinted-only otherwise.
    /// Which primitives of a group to draw: glass (GltfPrimitive::
    /// transparent) goes in its own, final pass.
    enum class PrimitivePass { Opaque, Transparent, All };
    void drawPrimitiveGroup(const std::vector<GltfPrimitive>& prims, const glm::mat4& transform,
                            PrimitivePass pass = PrimitivePass::All);

    Shader m_shader;

    /// Directional-light shadow map, fitted around the vehicle each frame.
    /// See ShadowMap.h. Left invalid on a driver that cannot give us a depth
    /// framebuffer, in which case everything still draws, unshadowed.
    ShadowMap m_shadow;

    /// Program currently bound: the lighting shader, or the shadow map's
    /// depth-only one during the shadow pass.
    const Shader* m_activeShader = nullptr;

    /// True while filling the shadow map. Flat geometry (ground, paint,
    /// markings) is skipped then: it casts nothing, and a coplanar caster is
    /// the classic source of shadow acne on the surface it sits on.
    bool m_depthPass = false;

    /// Circuit, generated from the track centreline - see rendering/TrackMesh.h.
    Mesh m_ground;
    Mesh m_grid;
    Mesh m_road;
    Mesh m_markings;
    Mesh m_kerbs;
    Mesh m_verge;
    Mesh m_barriers;
    Mesh m_scenery;

    /// The venue, when the active track has one - see rendering/Venue.h. The
    /// six generic meshes above are left empty while it is active.
    Venue  m_venue;
    bool   m_venueActive = false;
    Shader m_skyShader;
    Mesh   m_skyTriangle;
    Texture m_noise;
    /// Surface materials, one layer each - see loadMaterialTextures().
    TextureArray m_matAlbedo;
    TextureArray m_matDetail;
    ShadowMap m_shadowFar;
    int    m_farShadowAge = 1 << 20;      ///< frames since the far map was drawn
    glm::vec3 m_farShadowCentre{1e9f};

    /// View of the pass being drawn, for venue culling and level of detail.
    glm::mat4 m_passViewProj{1.0f};
    glm::vec3 m_cameraEye{0.0f};
    FrameStats m_frameStats;

    /// Vehicle, generated from the vehicle parameters - see rendering/CarMesh.h.
    /// The wheels come in mirrored pairs (outboard face out on both sides) and
    /// the brakes are separate because they steer without spinning.
    Mesh m_body;
    Mesh m_glass;
    Mesh m_wheelLeft;
    Mesh m_wheelRight;
    Mesh m_brakeLeft;
    Mesh m_brakeRight;
    Mesh m_comMarker;

    /// Imported vehicle model. Null whenever the active vehicle has no
    /// `model.path` configured, or its load failed - both cases fall back to
    /// the procedural m_body/m_cabin/m_wheel above. See GltfModel.h.
    std::unique_ptr<GltfModel> m_vehicleModel;
    std::string m_loadedModelPath;

    /// Environment (scenery/terrain) model - see loadEnvironmentModel(). Null
    /// whenever no EnvironmentModelConfig.path is configured, mirroring
    /// m_vehicleModel above.
    std::unique_ptr<GltfModel> m_environmentModel;
    std::string m_loadedEnvironmentModelPath;
    TerrainHeightField m_terrain;

    DebugDraw m_debug;
    RenderStyle m_style;

    int m_width = 1600;
    int m_height = 900;

    /// Cached so rebuildVehicleMeshes only runs when geometry actually changes.
    VehicleParameters m_cachedVehicleParams;
    bool m_vehicleMeshesBuilt = false;

    /// Body colour at the last rebuild. The generated model bakes colour into
    /// its vertices, so a change in the UI colour picker has to regenerate the
    /// mesh rather than just set a uniform.
    glm::vec3 m_cachedBodyColor{-1.0f};
};

} // namespace vsim
