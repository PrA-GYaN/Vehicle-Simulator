#include "rendering/Renderer.h"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>

#include "config/Config.h"
#include "core/Coordinates.h"
#include "rendering/ApexVenue.h"
#include "rendering/MeshFactory.h"
#include "simulation/Simulation.h"
#include "vehicle/Vehicle.h"

namespace vsim {

//---------------------------------------------------------------------------
//  Setup
//---------------------------------------------------------------------------

bool Renderer::initialize(const Track& track) {
    if (!m_shader.loadFromFiles("assets/shaders/basic.vert",
                                "assets/shaders/basic.frag")) {
        std::cerr << "[renderer] shader load failed\n";
        return false;
    }
    m_activeShader = &m_shader;

    // Shadows are a nice-to-have, not a requirement: a driver that cannot
    // give us a depth framebuffer gets the same scene without them.
    if (!m_shadow.initialize(2048)) {
        std::cerr << "[renderer] continuing without shadows\n";
    }

    // The venue pipeline's extras. None of them is required: without the
    // sky shader a venue is drawn against the clear colour, and without the
    // far shadow map distant scenery is simply unshadowed.
    if (!m_skyShader.loadFromFiles("assets/shaders/sky.vert", "assets/shaders/sky.frag")) {
        std::cerr << "[renderer] sky shader failed to load; venue circuits get a flat sky\n";
    }
    {
        std::vector<Vertex> tri(3);
        tri[0].position = {-1.0f, -1.0f, 0.0f};
        tri[1].position = { 3.0f, -1.0f, 0.0f};
        tri[2].position = {-1.0f,  3.0f, 0.0f};
        m_skyTriangle.upload(tri);
    }
    buildNoiseTexture();
    loadMaterialTextures();
    if (!m_shadowFar.initialize(2048)) {
        std::cerr << "[renderer] no far shadow map\n";
    }

    rebuildTrackMeshes(track);

    meshfactory::sphere(0.16f, 1, debugcolor::kCentreOfMass).uploadTo(m_comMarker);

    gl::Enable(gl::DEPTH_TEST);
    gl::DepthFunc(gl::LESS);
    gl::Enable(gl::CULL_FACE);
    gl::CullFace(gl::BACK);
    gl::FrontFace(gl::CCW);
    gl::Enable(gl::BLEND);
    gl::BlendFunc(gl::SRC_ALPHA, gl::ONE_MINUS_SRC_ALPHA);
    gl::Enable(gl::MULTISAMPLE);

    return true;
}

void Renderer::rebuildTrackMeshes(const Track& track) {
    const float extent = static_cast<float>(track.groundExtent());

    meshfactory::groundPlane(extent, m_style.groundColor).uploadTo(m_ground);

    meshfactory::grid(extent, 10.0f,
                      m_style.groundColor * 1.18f,
                      m_style.groundColor * 1.45f, 10)
        .uploadTo(m_grid, gl::LINES);

    //-----------------------------------------------------------------------
    //  The circuit itself: asphalt, paint, kerbs, runoff, barriers and
    //  trackside structures, all generated from this track's own centreline.
    //  See rendering/TrackMesh.h.
    //
    //  A track with an authored venue (rendering/ApexVenue.h) is drawn from
    //  that instead, through the venue pipeline, and the generic meshes are
    //  left empty.
    //-----------------------------------------------------------------------
    m_venueActive = isApexVenueTrack(track);
    m_farShadowAge = 1 << 20;
    // The two lighting paths read the shadow maps differently - see
    // ShadowMap::setHardwareCompare().
    m_shadow.setHardwareCompare(m_venueActive);
    m_shadowFar.setHardwareCompare(true);
    if (m_venueActive) {
        VenueData venue = buildApexVenue(track);
        m_venue.upload(std::move(venue));
        std::cout << "[renderer] venue: " << m_venue.chunkCount() << " chunks, "
                  << m_venue.totalInstances() << " instances, "
                  << m_venue.totalTriangles() << " triangles in total\n";
        const MeshData none;
        none.uploadTo(m_road);
        none.uploadTo(m_markings);
        none.uploadTo(m_kerbs);
        none.uploadTo(m_verge);
        none.uploadTo(m_barriers);
        none.uploadTo(m_scenery);
        return;
    }
    m_venue.clear();

    TrackPalette palette;
    palette.grass = m_style.groundColor * 0.92f;

    const TrackMeshSet circuit = buildTrackMeshes(track, palette);
    circuit.asphalt .uploadTo(m_road);
    circuit.markings.uploadTo(m_markings);
    circuit.kerbs   .uploadTo(m_kerbs);
    circuit.verge   .uploadTo(m_verge);
    circuit.barriers.uploadTo(m_barriers);
    circuit.scenery .uploadTo(m_scenery);
}

void Renderer::rebuildVehicleMeshes(const VehicleParameters& p) {
    //-----------------------------------------------------------------------
    //  Vehicle geometry is generated in the VEHICLE LOCAL frame:
    //      +X forward, +Y up, +Z right,  origin at the REAR AXLE.
    //  See core/Coordinates.h and rendering/CarMesh.h - every dimension of
    //  the drawn car comes from `p`, so the model cannot disagree with the
    //  configuration the physics is running.
    //-----------------------------------------------------------------------
    CarPalette palette;
    palette.body   = m_style.bodyColor;
    palette.accent = m_style.accentColor;
    palette.glass  = m_style.glassColor;

    const CarMeshSet car =
        buildCarMeshes(p, carBodyStyleFromString(p.model.bodyStyle), palette);

    car.body      .uploadTo(m_body);
    car.glass     .uploadTo(m_glass);
    car.wheelLeft .uploadTo(m_wheelLeft);
    car.wheelRight.uploadTo(m_wheelRight);
    car.brakeLeft .uploadTo(m_brakeLeft);
    car.brakeRight.uploadTo(m_brakeRight);

    m_cachedBodyColor = m_style.bodyColor;

    //-----------------------------------------------------------------------
    //  Imported model (optional, and no longer the better-looking option).
    //  The generated car above is still built unconditionally - it is the
    //  fallback whenever there is no model, the path fails to load, or a
    //  specific wheel node is missing.
    //-----------------------------------------------------------------------
    if (p.model.hasModel() && p.model.path != m_loadedModelPath) {
        auto model = std::make_unique<GltfModel>();

        ImportAxes axes;
        axes.forward = p.model.forwardAxis;
        axes.up      = p.model.upAxis;

        WheelNodeNames wheelNodes;
        wheelNodes.frontLeft  = p.model.frontLeftNode;
        wheelNodes.frontRight = p.model.frontRightNode;
        wheelNodes.rearLeft   = p.model.rearLeftNode;
        wheelNodes.rearRight  = p.model.rearRightNode;

        const std::string resolved = resolveResourcePath(p.model.path);
        if (model->load(resolved, axes, wheelNodes)) {
            m_vehicleModel = std::move(model);
        } else {
            std::cerr << "[renderer] falling back to the procedural body for '"
                      << p.model.path << "'\n";
            m_vehicleModel.reset();
        }
        m_loadedModelPath = p.model.path;
    } else if (!p.model.hasModel()) {
        m_vehicleModel.reset();
        m_loadedModelPath.clear();
    }

    m_cachedVehicleParams = p;
    m_vehicleMeshesBuilt = true;
}

bool Renderer::loadEnvironmentModel(const EnvironmentModelConfig& cfg) {
    if (!cfg.hasModel()) {
        m_environmentModel.reset();
        m_loadedEnvironmentModelPath.clear();
        m_terrain = TerrainHeightField{};
        return false;
    }
    if (m_environmentModel && cfg.path == m_loadedEnvironmentModelPath) {
        return m_environmentModel->loaded();
    }

    auto model = std::make_unique<GltfModel>();

    ImportAxes axes;
    axes.forward = cfg.forwardAxis;
    axes.up      = cfg.upAxis;

    // No wheel nodes - a scenery/terrain model is pure static geometry.
    // GltfModel already supports this (see its header on WheelNodeNames).
    const WheelNodeNames noWheels;

    const std::string resolved = resolveResourcePath(cfg.path);
    const bool ok = model->load(resolved, axes, noWheels);
    m_loadedEnvironmentModelPath = cfg.path;

    if (!ok) {
        std::cerr << "[renderer] environment model '" << cfg.path << "' failed to load; "
                  << "falling back to the procedural ground/track\n";
        m_environmentModel.reset();
        m_terrain = TerrainHeightField{};
        return false;
    }

    m_environmentModel = std::move(model);
    m_terrain.build(m_environmentModel->staticPoints());

    // Resize the debug grid to the model's own footprint - see Renderer.h.
    const glm::vec3 lo = m_environmentModel->boundsMin();
    const glm::vec3 hi = m_environmentModel->boundsMax();
    const float extent = std::max(200.0f, 0.75f * std::max(hi.x - lo.x, hi.z - lo.z));
    meshfactory::grid(extent, 10.0f,
                      m_style.groundColor * 1.18f,
                      m_style.groundColor * 1.45f, 10)
        .uploadTo(m_grid, gl::LINES);

    return true;
}

void Renderer::buildTrackTerrain(const Track& track) {
    if (!track.hasElevation()) {
        m_terrain = TerrainHeightField{};
        return;
    }

    // Sample the road surface across its own width, not just the
    // centreline: a wheel can be up to halfWidth off the racing line, and
    // TerrainHeightField bins by MAX height per cell (see its header), so a
    // cell that only ever saw the centreline would still be right under the
    // car but could miss a camber/edge point a wheel actually reaches.
    constexpr int kLateralSamples = 5;
    const double halfWidth = track.width() * 0.5;
    const auto& centreline = track.centreline();

    std::vector<glm::vec3> points;
    points.reserve(centreline.size() * kLateralSamples);

    for (const TrackPoint& p : centreline) {
        const double yaw = coords::yawFromDirection(p.tangent);
        const glm::dvec3 right = coords::right(yaw);
        for (int i = 0; i < kLateralSamples; ++i) {
            const double t = -1.0 + 2.0 * static_cast<double>(i) /
                                    static_cast<double>(kLateralSamples - 1);
            points.push_back(glm::vec3(p.position + right * (t * halfWidth)));
        }
    }

    // 3 m cells: smaller than the default 4 m (built for a scan whose
    // vertices are far denser) but still coarser than the 1.5 m centreline
    // spacing, which is what actually bounds the resolution this field can
    // usefully offer.
    m_terrain.build(points, 3.0);
}

void Renderer::setViewport(int width, int height) {
    m_width  = (width  > 0) ? width  : 1;
    m_height = (height > 0) ? height : 1;
    gl::Viewport(0, 0, m_width, m_height);
}

//---------------------------------------------------------------------------
//  Frame
//---------------------------------------------------------------------------

void Renderer::render(const Simulation& sim, const Camera& camera,
                      const DebugDrawOptions& debugOptions) {
    const Vehicle& vehicle = sim.vehicle();

    // Keep the drawn car in sync with the configured car. Comparing a couple
    // of scalars per frame is far cheaper than rebuilding meshes blindly.
    if (!m_vehicleMeshesBuilt ||
        m_cachedBodyColor                 != m_style.bodyColor ||
        m_cachedVehicleParams.length      != vehicle.parameters().length ||
        m_cachedVehicleParams.width       != vehicle.parameters().width  ||
        m_cachedVehicleParams.height      != vehicle.parameters().height ||
        m_cachedVehicleParams.wheelbase   != vehicle.parameters().wheelbase ||
        m_cachedVehicleParams.wheelRadius != vehicle.parameters().wheelRadius ||
        m_cachedVehicleParams.wheelWidth  != vehicle.parameters().wheelWidth ||
        m_cachedVehicleParams.model.path  != vehicle.parameters().model.path) {
        rebuildVehicleMeshes(vehicle.parameters());
    }

    //-----------------------------------------------------------------------
    //  PASS 1 - shadow map, from the light, fitted around the vehicle.
    //-----------------------------------------------------------------------
    m_venue.resetStats();
    m_cameraEye = camera.position();

    const bool shadows = m_style.shadows && m_shadow.valid();
    const bool farShadows = shadows && m_venueActive && m_shadowFar.valid();
    if (farShadows) renderFarShadowPass(vehicle);
    if (shadows) renderShadowPass(vehicle);

    //-----------------------------------------------------------------------
    //  PASS 2 - the lit scene.
    //-----------------------------------------------------------------------
    gl::ClearColor(m_style.skyColor.r, m_style.skyColor.g, m_style.skyColor.b, 1.0f);
    gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);

    gl::PolygonMode(gl::FRONT_AND_BACK, m_style.wireframe ? gl::LINE : gl::FILL);

    const glm::mat4 view = camera.viewMatrix();
    const glm::mat4 projection = camera.projectionMatrix(aspect());
    m_passViewProj = projection * view;

    m_activeShader = &m_shader;
    m_shader.use();
    m_shader.setMat4("uView", view);
    m_shader.setMat4("uProjection", projection);
    m_shader.setVec3("uLightDirection", glm::normalize(m_style.lightDirection));
    m_shader.setVec3("uLightColor", m_style.lightColor);
    m_shader.setVec3("uViewPosition", camera.position());
    m_shader.setFloat("uAmbient", m_venueActive ? m_style.venueAmbient : m_style.ambient);
    m_shader.setInt("uInstanced", 0);

    // Which of the fragment shader's two paths this frame takes.
    m_shader.setInt("uPipeline", m_venueActive ? 1 : 0);
    // Samplers of different kinds may not share a texture unit, even ones a
    // frame never reads, so the comparison samplers always get their own.
    m_shader.setInt("uShadowFarCmp", 3);
    m_shader.setInt("uShadowNearCmp", 4);
    m_shader.setInt("uNoise", 2);
    m_shader.setInt("uMatAlbedo", 5);
    m_shader.setInt("uMatDetail", 6);
    if (m_venueActive) {
        m_shader.setFloat("uSunIntensity", m_style.venueSunIntensity);
        m_shader.setFloat("uExposure", m_style.venueExposure);
        m_noise.bind(2);
        m_shader.setInt("uNoise", 2);
        m_shadowFar.bindTexture(3);
        m_shader.setInt("uShadowFarCmp", 3);
        m_shadow.bindTexture(4);
        m_shader.setInt("uShadowNearCmp", 4);
        m_shader.setMat4("uLightSpaceFar", m_shadowFar.lightSpaceMatrix());
        m_matAlbedo.bind(5);
        m_matDetail.bind(6);
        m_shader.setInt("uShadowFarEnabled", farShadows ? 1 : 0);
        gl::ActiveTexture(gl::TEXTURE0);
    }
    m_shader.setInt("uUnlit", 0);
    m_shader.setInt("uUseTexture", 0);

    // Hemispheric ambient. The ground term is the scene's own ground colour
    // dimmed - light bounced off grass is green and weak, and using the
    // actual ground colour keeps the two consistent when the UI changes it.
    m_shader.setVec3("uSkyColor", m_style.skyColor);
    m_shader.setVec3("uGroundColor", m_style.groundColor * 0.55f);
    m_shader.setVec3("uFogColor", m_style.skyColor);
    m_shader.setFloat("uFogDensity", m_venueActive ? m_style.venueFogDensity : m_style.fogDensity);
    m_shader.setFloat("uGloss", 0.0f);

    if (shadows) {
        m_shadow.bindTexture(1);
        m_shader.setInt("uShadowMap", 1);
        m_shader.setMat4("uLightSpace", m_shadow.lightSpaceMatrix());
        m_shader.setInt("uShadowEnabled", 1);
    } else {
        m_shader.setInt("uShadowEnabled", 0);
    }

    drawEnvironment();
    if (camera.vehicleNearPass()) {
        // Body-mounted camera (Camera.h, "NEAR PLANE"): the car is drawn on
        // a cleared depth buffer with its own close-range projection, so the
        // dashboard a few centimetres from the lens is not clipped by the
        // world's 0.5 m near plane. Catch fencing has to go down before that
        // clear, or it would be drawn across the inside of the cockpit.
        drawEnvironmentTransparent();
        gl::Clear(gl::DEPTH_BUFFER_BIT);
        m_shader.setMat4("uProjection", camera.nearProjectionMatrix(aspect()));
        drawVehicle(vehicle);
        m_shader.setMat4("uProjection", camera.projectionMatrix(aspect()));
    } else {
        // Otherwise the car goes first, so a fence between the camera and
        // the car (a trackside view) is blended over it, not hidden by it.
        if (camera.drawsVehicle()) drawVehicle(vehicle);
        drawEnvironmentTransparent();
    }

    m_frameStats.drawCalls = m_venue.stats().drawCalls;
    m_frameStats.triangles = m_venue.stats().triangles;

    // Debug lines last, with the polygon mode restored so wireframe mode does
    // not turn the overlay into a mess.
    gl::PolygonMode(gl::FRONT_AND_BACK, gl::FILL);
    m_debug.beginFrame();
    drawDebugOverlays(sim, debugOptions);
    m_debug.flush(m_shader);
}

void Renderer::renderShadowPass(const Vehicle& vehicle) {
    // Centre the map slightly AHEAD of the car: at speed the useful shadows
    // are the ones the car is driving into, and the region behind it is
    // leaving the screen anyway.
    const VehicleState& s = vehicle.state();
    const glm::dvec3 ahead = s.position + s.forward() * (m_style.shadowRadius * 0.28);

    m_shadow.update(glm::vec3(ahead), glm::normalize(m_style.lightDirection),
                    m_venueActive ? m_style.venueShadowRadius : m_style.shadowRadius);
    m_passViewProj = m_shadow.lightSpaceMatrix();

    m_depthPass = true;
    m_activeShader = &m_shadow.depthShader();
    m_shadow.beginDepthPass();

    // Wireframe is a debug view of the geometry, not of the lighting; the
    // shadow pass always runs filled so the map stays correct.
    gl::PolygonMode(gl::FRONT_AND_BACK, gl::FILL);
    m_activeShader->setInt("uInstanced", 0);

    drawEnvironment();
    drawVehicle(vehicle);

    m_shadow.endDepthPass(m_width, m_height);
    m_depthPass = false;
    m_activeShader = &m_shader;
}

void Renderer::renderFarShadowPass(const Vehicle& vehicle) {
    // Nothing in this map moves, so it only needs redrawing when the car
    // has travelled far enough for the map's coverage to matter, or every
    // so often regardless. The matrix the lit pass samples with is the one
    // the map was last drawn with, so a stale map is still a correct one.
    const VehicleState& s = vehicle.state();
    const glm::vec3 centre = glm::vec3(s.position + s.forward() * (m_style.venueFarShadowRadius * 0.35));
    ++m_farShadowAge;
    const float moved = glm::length(centre - m_farShadowCentre);
    if (m_farShadowAge < 240 && moved < m_style.venueFarShadowRadius * 0.12f) return;
    if (m_farShadowAge < 6) return;
    m_farShadowAge = 0;
    m_farShadowCentre = centre;

    m_shadowFar.update(centre, glm::normalize(m_style.lightDirection),
                       m_style.venueFarShadowRadius);
    m_passViewProj = m_shadowFar.lightSpaceMatrix();

    m_depthPass = true;
    m_activeShader = &m_shadowFar.depthShader();
    m_shadowFar.beginDepthPass();
    gl::PolygonMode(gl::FRONT_AND_BACK, gl::FILL);
    m_activeShader->setInt("uInstanced", 0);

    drawEnvironment();

    m_shadowFar.endDepthPass(m_width, m_height);
    m_depthPass = false;
    m_activeShader = &m_shader;
}

void Renderer::drawPrimitiveGroup(const std::vector<GltfPrimitive>& prims,
                                  const glm::mat4& transform, PrimitivePass pass) {
    shader().setMat4("uModel", transform);
    for (const GltfPrimitive& gp : prims) {
        if (pass == PrimitivePass::Opaque && gp.transparent) continue;
        if (pass == PrimitivePass::Transparent && !gp.transparent) continue;
        if (!m_depthPass && gp.baseColorTexture && gp.baseColorTexture->valid()) {
            gp.baseColorTexture->bind(0);
            shader().setInt("uTexture", 0);
            shader().setInt("uUseTexture", 1);
        } else {
            shader().setInt("uUseTexture", 0);
        }
        gp.mesh.draw();
    }
}

void Renderer::drawEnvironment() {
    if (hasEnvironmentModel()) {
        // Same "doubleSided, no culling assumption" reasoning as the
        // imported vehicle model - see drawVehicle().
        gl::Disable(gl::CULL_FACE);
        shader().setFloat("uGloss", 0.10f);
        drawPrimitiveGroup(m_environmentModel->staticPrimitives(), glm::mat4(1.0f));
        shader().setInt("uUseTexture", 0);
        gl::Enable(gl::CULL_FACE);
        return;
    }

    shader().setMat4("uModel", glm::mat4(1.0f));

    if (m_venueActive) {
        drawVenue();
        return;
    }

    // Flat surfaces cast nothing and would only invite acne on themselves -
    // see m_depthPass in Renderer.h. Kerbs, barriers and structures stand off
    // the ground and are drawn in both passes.
    if (!m_depthPass) {
        //-------------------------------------------------------------------
        //  THE FLAT LAYERS, AND WHY THEY NEED POLYGON OFFSET
        //
        //  Ground, grass, asphalt and paint are four surfaces within about a
        //  centimetre of each other, stretched over kilometres. Separating
        //  them by height alone (which is what TrackMesh's kGrassY/kAsphaltY/
        //  kPaintY do) works near the camera and FAILS at distance: the depth
        //  buffer is non-linear, so a few hundred metres away one depth unit
        //  is worth far more than the centimetre that separates the layers,
        //  and the road starts losing to the grass beside it.
        //
        //  glPolygonOffset works in depth-buffer units instead of world
        //  units, so the ordering holds at every distance. Positive pushes
        //  away from the camera - the ground plane is furthest back, the
        //  paint is furthest forward.
        //-------------------------------------------------------------------
        gl::Enable(gl::POLYGON_OFFSET_FILL);

        shader().setFloat("uGloss", 0.0f);
        gl::PolygonOffset(8.0f, 8.0f);
        m_ground.draw();

        if (m_style.showRoad) {
            gl::PolygonOffset(5.0f, 5.0f);
            m_verge.draw();

            shader().setFloat("uGloss", 0.14f);   // asphalt has a slight sheen
            gl::PolygonOffset(2.0f, 2.0f);
            m_road.draw();

            gl::PolygonOffset(-2.0f, -2.0f);
            m_markings.draw();
        }

        gl::PolygonOffset(0.0f, 0.0f);
        gl::Disable(gl::POLYGON_OFFSET_FILL);
    }

    if (m_style.showRoad) {
        shader().setFloat("uGloss", 0.06f);
        m_kerbs.draw();
    }
    if (m_style.showScenery) {
        shader().setFloat("uGloss", 0.30f);       // painted steel
        m_barriers.draw();
        shader().setFloat("uGloss", 0.05f);
        m_scenery.draw();
    }
    shader().setFloat("uGloss", 0.0f);
}

void Renderer::drawVenue() {
    VenueView view;
    view.viewProj  = m_passViewProj;
    view.eye       = m_cameraEye;
    view.shader    = m_activeShader;
    view.depthPass = m_depthPass;

    if (m_depthPass) {
        m_venue.drawLayer(VenueLayer::Solid, view);
        m_venue.drawInstances(view);
        return;
    }

    //-----------------------------------------------------------------------
    //  FRONT TO BACK.
    //
    //  A venue frame is limited by pixels shaded, not triangles drawn: every
    //  fragment samples a noise texture several times and a shadow map four.
    //  So the frame is drawn nearest-first and the depth test discards what
    //  is hidden before it is shaded - the road (which fills the bottom of
    //  every driving view), then everything standing on the ground, then the
    //  ground itself, and the sky last of all, only where nothing else is.
    //
    //  The flat layers are still separated by polygon offset, for the reason
    //  given in drawEnvironment(); the offset is applied to the depth a
    //  fragment is tested and written with, so the layering holds whatever
    //  order the layers are drawn in.
    //-----------------------------------------------------------------------
    shader().setInt("uWorldSpace", 1);    // venue chunks are authored in world space
    gl::Enable(gl::POLYGON_OFFSET_FILL);

    if (m_style.showRoad) {
        shader().setFloat("uGloss", 0.0f);
        gl::PolygonOffset(2.0f, 2.0f);
        m_venue.drawLayer(VenueLayer::Asphalt, view);
        gl::PolygonOffset(0.0f, 0.0f);
        m_venue.drawLayer(VenueLayer::Kerbs, view);
    }
    if (m_style.showScenery) {
        gl::PolygonOffset(0.0f, 0.0f);
        shader().setFloat("uGloss", 0.05f);
        m_venue.drawLayer(VenueLayer::Solid, view);
        shader().setFloat("uGloss", 0.30f);
        shader().setInt("uWorldSpace", 0);
        // Foliage cards are cut out by alpha-to-coverage: with 4x MSAA the
        // edge of every leaf is anti-aliased, and nothing needs sorting.
        gl::Enable(gl::SAMPLE_ALPHA_TO_COVERAGE);
        m_venue.drawInstances(view);
        gl::Disable(gl::SAMPLE_ALPHA_TO_COVERAGE);
        shader().setInt("uWorldSpace", 1);
    }

    shader().setFloat("uGloss", 0.0f);
    if (m_style.showRoad) {
        gl::PolygonOffset(5.0f, 5.0f);
        m_venue.drawLayer(VenueLayer::Verge, view);
    }
    gl::PolygonOffset(8.0f, 8.0f);
    m_venue.drawLayer(VenueLayer::Terrain, view);
    gl::PolygonOffset(0.0f, 0.0f);

    drawSky();

    if (m_style.showRoad) {
        // Paint is blended (worn lines, skid marks) and so comes after every
        // opaque surface it can lie on. It does not write depth: one mark
        // must not cut a hole in the next.
        gl::PolygonOffset(-2.0f, -2.0f);
        gl::DepthMask(false);
        m_venue.drawLayer(VenueLayer::Paint, view);
        gl::DepthMask(true);
        gl::PolygonOffset(0.0f, 0.0f);
    }
    gl::Disable(gl::POLYGON_OFFSET_FILL);
    shader().setInt("uWorldSpace", 0);
}

void Renderer::drawSky() {
    if (!m_skyShader.valid()) return;

    // At the far plane, passing only where the depth buffer is still clear.
    gl::PolygonMode(gl::FRONT_AND_BACK, gl::FILL);
    gl::DepthFunc(gl::LEQUAL);
    gl::DepthMask(false);
    m_skyShader.use();
    m_skyShader.setMat4("uInvViewProj", glm::inverse(m_passViewProj));
    m_skyShader.setVec3("uLightDirection", glm::normalize(m_style.lightDirection));
    m_skyShader.setVec3("uLightColor", m_style.lightColor);
    m_skyShader.setFloat("uSunIntensity", m_style.venueSunIntensity);
    m_skyShader.setFloat("uExposure", m_style.venueExposure);
    m_noise.bind(2);
    m_skyShader.setInt("uNoise", 2);
    m_skyTriangle.draw();
    gl::ActiveTexture(gl::TEXTURE0);
    gl::DepthMask(true);
    gl::DepthFunc(gl::LESS);
    gl::PolygonMode(gl::FRONT_AND_BACK, m_style.wireframe ? gl::LINE : gl::FILL);
    m_shader.use();
}

void Renderer::drawEnvironmentTransparent() {
    if (!m_venueActive || m_depthPass || !m_style.showScenery) return;

    VenueView view;
    view.viewProj = m_passViewProj;
    view.eye      = m_cameraEye;
    view.shader   = m_activeShader;

    shader().setMat4("uModel", glm::mat4(1.0f));
    shader().setInt("uUseTexture", 0);
    // A fence is seen from both sides and through itself.
    gl::Disable(gl::CULL_FACE);
    gl::DepthMask(false);
    m_venue.drawLayer(VenueLayer::Transparent, view);
    gl::DepthMask(true);
    gl::Enable(gl::CULL_FACE);
}

//---------------------------------------------------------------------------
//  The venue pipeline's one texture: 256^2 of tiling noise, generated here
//  rather than shipped. Each channel is a different kind:
//
//    R  fine grain        aggregate in asphalt, blades in grass
//    G  soft blotches     patchiness a few metres across
//    B  cloudy fbm        large-scale variation, and the clouds themselves
//    A  streaks           long in v, fine in u: rubber, scuffs, skid marks
//
//  All four tile, because they are built on lattices that wrap.
//---------------------------------------------------------------------------

void Renderer::buildNoiseTexture() {
    constexpr int kSize = 256;

    auto lattice = [](int x, int y, int seed) {
        unsigned int h = static_cast<unsigned int>(x) * 374761393u +
                         static_cast<unsigned int>(y) * 668265263u +
                         static_cast<unsigned int>(seed) * 2246822519u;
        h = (h ^ (h >> 13)) * 1274126177u;
        return static_cast<float>((h ^ (h >> 16)) & 0xFFFFu) / 65535.0f;
    };
    // Value noise with `cx` by `cy` cells across the texture, wrapping.
    auto value = [&](float u, float v, int cx, int cy, int seed) {
        const float fx = u * static_cast<float>(cx), fy = v * static_cast<float>(cy);
        const int x0 = static_cast<int>(std::floor(fx)), y0 = static_cast<int>(std::floor(fy));
        float tx = fx - static_cast<float>(x0), ty = fy - static_cast<float>(y0);
        tx = tx * tx * (3.0f - 2.0f * tx);
        ty = ty * ty * (3.0f - 2.0f * ty);
        auto at = [&](int x, int y) { return lattice(((x % cx) + cx) % cx, ((y % cy) + cy) % cy, seed); };
        const float a = at(x0, y0) + (at(x0 + 1, y0) - at(x0, y0)) * tx;
        const float b = at(x0, y0 + 1) + (at(x0 + 1, y0 + 1) - at(x0, y0 + 1)) * tx;
        return a + (b - a) * ty;
    };

    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(kSize) * kSize * 4);
    for (int y = 0; y < kSize; ++y) {
        for (int x = 0; x < kSize; ++x) {
            const float u = (static_cast<float>(x) + 0.5f) / kSize;
            const float v = (static_cast<float>(y) + 0.5f) / kSize;

            const float fine = 0.55f * lattice(x, y, 1) + 0.45f * value(u, v, 96, 96, 2);
            const float blot = 0.65f * value(u, v, 12, 12, 3) + 0.35f * value(u, v, 31, 31, 4);
            const float fbm  = 0.50f * value(u, v, 5, 5, 5) + 0.27f * value(u, v, 11, 11, 6) +
                               0.15f * value(u, v, 23, 23, 7) + 0.08f * value(u, v, 47, 47, 8);
            const float streak = 0.70f * value(u, v, 80, 5, 9) + 0.30f * value(u, v, 37, 9, 10);

            auto put = [](float f) {
                return static_cast<std::uint8_t>(std::min(std::max(f, 0.0f), 1.0f) * 255.0f + 0.5f);
            };
            std::uint8_t* p = &pixels[(static_cast<std::size_t>(y) * kSize + x) * 4];
            p[0] = put(fine);
            p[1] = put(blot);
            p[2] = put(fbm);
            p[3] = put(streak);
        }
    }
    m_noise.upload(pixels.data(), kSize, kSize);
}

//---------------------------------------------------------------------------
//  The venue's surface materials: photographed textures (CC0, from
//  ambientCG - see assets/textures/venue/SOURCES.md), baked by
//  tools/bake_venue_textures.py into a colour and a detail image each.
//  The layer order here is the order the shader's material table assumes.
//
//  A missing file is not an error. Its layer falls back to "no detail" -
//  mid-grey colour, a flat normal, medium roughness - and that material is
//  drawn in its plain vertex colour, as the venue was before it had textures.
//---------------------------------------------------------------------------

void Renderer::loadMaterialTextures() {
    static const char* const kLayers[] = {
        "00_asphalt", "01_grass", "02_gravel", "03_concrete", "04_plaster",
        "05_galvanised", "06_rubber", "07_roofing", "08_bark", "09_leaves",
    };
    std::vector<std::string> albedo, detail;
    for (const char* name : kLayers) {
        const std::string base = std::string("assets/textures/venue/") + name;
        albedo.push_back(resolveResourcePath(base + "_albedo.jpg"));
        detail.push_back(resolveResourcePath(base + "_detail.jpg"));
    }
    const std::uint8_t grey[3] = {128, 128, 128};
    const std::uint8_t flat[3] = {128, 128, 200};   // flat normal; roughness (or opacity) high
    // Anisotropic filtering is what keeps the road sharp into the distance,
    // and on an integrated GPU it is also the most expensive thing a texture
    // fetch can do. Colour gets a modest ratio; the normal map, whose detail
    // is lost to distance fog and mip averaging anyway, gets less.
    const int a = m_matAlbedo.load(albedo, 1024, grey, 4, true);
    const int d = m_matDetail.load(detail, 1024, flat, 2, true);
    std::cout << "[renderer] venue materials: " << a << " colour and " << d
              << " detail textures of " << albedo.size() << "\n";
    gl::ActiveTexture(gl::TEXTURE0);
}

bool Renderer::saveScreenshot(const std::string& path) const {
    std::vector<std::uint8_t> rgb(static_cast<std::size_t>(m_width) * m_height * 3);
    gl::PixelStorei(gl::PACK_ALIGNMENT, 1);
    gl::ReadPixels(0, 0, m_width, m_height, gl::RGB, gl::UNSIGNED_BYTE_, rgb.data());

    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    out << "P6\n" << m_width << " " << m_height << "\n255\n";
    // GL rows run bottom to top; a PPM's run top to bottom.
    for (int y = m_height - 1; y >= 0; --y) {
        out.write(reinterpret_cast<const char*>(&rgb[static_cast<std::size_t>(y) * m_width * 3]),
                  static_cast<std::streamsize>(m_width) * 3);
    }
    return static_cast<bool>(out);
}

void Renderer::drawVehicle(const Vehicle& vehicle) {
    const VehicleState& s = vehicle.state();

    //-----------------------------------------------------------------------
    //  Vehicle model matrix.
    //
    //  coords::modelMatrixYPR() is the ONLY place a yaw/pitch/roll angle
    //  becomes a rotation matrix. The renderer does not reimplement the
    //  convention. Pitch and the terrain component of roll come from the
    //  per-wheel terrain quarter car (Vehicle::terrainPitch/terrainRoll,
    //  zero on flat ground/without wheel data); heave likewise. The
    //  existing quasi-static cornering roll (s.rollAngle) is superposed on
    //  top - this is also the first time that angle is actually rendered,
    //  rather than computed and discarded (previously a documented known
    //  limitation).
    //-----------------------------------------------------------------------
    const glm::dvec3 heavePos = s.position + coords::kWorldUp * vehicle.terrainHeave();
    const double pitch = vehicle.terrainPitch();
    const double roll  = vehicle.terrainRoll() + s.rollAngle;
    const glm::dmat4 vehicleModel = coords::modelMatrixYPR(heavePos, s.yaw, pitch, roll);
    const bool useModel = (m_vehicleModel && m_vehicleModel->loaded());

    // See the fragment shader: `base = vColor; if (uUseTexture) base *=
    // texture(...)`, and every baked vertex already carries the material's
    // baseColorFactor as its vColor, so the untextured case needs nothing
    // further - drawPrimitiveGroup() (shared with drawEnvironment()) handles
    // both.
    if (useModel) {
        // glTF materials here are all doubleSided (KB... no, just how the
        // asset is authored) - the exported car has no back-face culling
        // assumption baked in, unlike the generated body.
        gl::Disable(gl::CULL_FACE);
        shader().setFloat("uGloss", 0.55f);
        drawPrimitiveGroup(m_vehicleModel->staticPrimitives(), glm::mat4(vehicleModel),
                           PrimitivePass::Opaque);
    } else {
        shader().setMat4("uModel", glm::mat4(vehicleModel));
        shader().setFloat("uGloss", 0.88f);   // car paint
        m_body.draw();
        shader().setFloat("uGloss", 1.00f);   // glass
        m_glass.draw();
    }

    //-----------------------------------------------------------------------
    //  Wheels.
    //
    //  Each wheel is placed by:
    //      vehicle transform
    //        * translate to the hub position (vehicle local frame)
    //        * steer   about local +Y   (steered wheels only)
    //        * spin    about local +Z   (the wheel's own axle)
    //
    //  identically whether the mesh is the procedural cylinder or an
    //  imported wheel group - a model wheel is baked relative to its own
    //  hub for exactly this reason (see GltfModel.h). The steer rotation is
    //  negated for the same reason as the vehicle yaw: a right-handed
    //  rotation about +Y by -angle turns +X toward +Z, which is our
    //  "positive = right" convention.
    //-----------------------------------------------------------------------
    const auto& poses = vehicle.wheelPoses();
    for (int i = 0; i < WheelCount; ++i) {
        const WheelPose& w = poses[static_cast<std::size_t>(i)];
        const bool left = (i == FrontLeft || i == RearLeft);

        const bool modelWheel = useModel && m_vehicleModel->hasWheel(i);

        // A model on staggered tyres has its front hubs at a different
        // height from its rear ones; the physics has one radius. Offset the
        // model's own wheels by its measured hub height (see
        // GltfModel::hubHeightOffset) so they sit in their arches.
        glm::dvec3 hubPos = w.localPosition;
        if (modelWheel) hubPos.y += m_vehicleModel->hubHeightOffset(i);

        glm::dmat4 hub = vehicleModel;
        hub = glm::translate(hub, hubPos);
        if (w.steered) hub = glm::rotate(hub, -w.steerAngle, coords::kWorldUp);

        const glm::dmat4 m = glm::rotate(hub, -w.spinAngle, glm::dvec3(0.0, 0.0, 1.0));

        if (modelWheel) {
            drawPrimitiveGroup(m_vehicleModel->wheelPrimitives(i), glm::mat4(m),
                               PrimitivePass::Opaque);
        } else {
            shader().setInt("uUseTexture", 0);

            // The wheel spins; the brake behind it does not. It is bolted to
            // the upright, so it follows the steering and nothing else -
            // drawing the disc and caliper inside the spinning mesh would put
            // a rotating caliper on the car, which is the most obvious tell
            // there is that a wheel is faked.
            shader().setFloat("uGloss", 0.45f);   // machined metal
            shader().setMat4("uModel", glm::mat4(hub));
            (left ? m_brakeLeft : m_brakeRight).draw();

            shader().setFloat("uGloss", 0.22f);   // rubber and rim
            shader().setMat4("uModel", glm::mat4(m));
            (left ? m_wheelLeft : m_wheelRight).draw();
        }
    }

    // Glass last: blended over everything opaque, including the car's own
    // interior, with depth writes off so panes do not hide each other. Not
    // in the shadow pass - glass barely shadows, and the depth shader has
    // no alpha.
    if (useModel && !m_depthPass) {
        gl::DepthMask(false);
        shader().setFloat("uGloss", 1.0f);
        for (const GltfPrimitive& gp : m_vehicleModel->staticPrimitives()) {
            if (!gp.transparent) continue;
            shader().setFloat("uAlpha", gp.alpha);
            shader().setMat4("uModel", glm::mat4(vehicleModel));
            if (gp.baseColorTexture && gp.baseColorTexture->valid()) {
                gp.baseColorTexture->bind(0);
                shader().setInt("uTexture", 0);
                shader().setInt("uUseTexture", 1);
            } else {
                shader().setInt("uUseTexture", 0);
            }
            shader().setInt("uTextureAlpha", gp.textureAlpha ? 1 : 0);
            gp.mesh.draw();
        }
        shader().setFloat("uAlpha", 1.0f);
        shader().setInt("uTextureAlpha", 0);
        gl::DepthMask(true);
    }

    shader().setFloat("uGloss", 0.0f);
    if (useModel) {
        shader().setInt("uUseTexture", 0);
        gl::Enable(gl::CULL_FACE);
    }
}

//---------------------------------------------------------------------------
//  Debug overlays
//---------------------------------------------------------------------------

void Renderer::drawDebugOverlays(const Simulation& sim, const DebugDrawOptions& opt) {
    const Vehicle& vehicle = sim.vehicle();
    const VehicleState& s  = vehicle.state();
    const VehicleParameters& p = vehicle.parameters();

    if (opt.grid) {
        m_shader.setMat4("uModel", glm::mat4(1.0f));
        m_shader.setInt("uUnlit", 1);
        m_grid.draw();
        m_shader.setInt("uUnlit", 0);
    }

    // --- world frame at the origin ---
    if (opt.worldFrame) {
        m_debug.axes({0.0, 0.05, 0.0},
                     {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 5.0);
    }

    // --- vehicle body frame at the CG ---
    if (opt.vehicleFrame) {
        const glm::dvec3 origin = vehicle.cgPosition();
        m_debug.axes(origin, s.forward(), coords::kWorldUp, s.right(), 2.2);
    }

    // --- centre of mass ---
    if (opt.centreOfMass) {
        const glm::dvec3 cg = vehicle.cgPosition();
        m_debug.cross(cg, 0.35, debugcolor::kCentreOfMass);

        m_shader.setMat4("uModel",
                         glm::mat4(glm::translate(glm::dmat4(1.0), cg)));
        m_shader.setInt("uUnlit", 1);
        m_comMarker.draw();
        m_shader.setInt("uUnlit", 0);

        // Drop line to the ground, so CG HEIGHT is readable on screen. It is
        // unused by the kinematic model but is a first-class parameter for
        // every model that follows, so it is worth showing.
        m_debug.line(cg, {cg.x, 0.0, cg.z}, debugcolor::kCentreOfMass * 0.6f);
    }

    const glm::dvec3 anchor = vehicle.cgPosition();

    // --- heading (unit forward), fixed length ---
    if (opt.headingVector) {
        m_debug.arrow(anchor, s.forward(), 4.0, debugcolor::kHeading);
    }

    // --- velocity vector, length proportional to speed ---
    if (opt.velocityVector && std::abs(s.velocity) > 0.05) {
        m_debug.arrow(anchor, s.velocityVector(),
                      std::abs(s.velocity) * opt.vectorScale,
                      debugcolor::kVelocity);
    }

    // --- steering direction, drawn at the front axle ---
    if (opt.steeringVector) {
        const glm::dvec3 frontAxle = vehicle.frontAxlePosition()
                                   + coords::kWorldUp * 0.35;
        // Steer angle is relative to the vehicle, so add it to the yaw to get
        // the world direction the front wheels point.
        const glm::dvec3 dir = coords::heading(s.yaw + s.steeringAngle);
        m_debug.arrow(frontAxle, dir, 3.0, debugcolor::kSteering);
    }

    // --- wheel contact points ---
    if (opt.wheelContacts) {
        for (const WheelPose& w : vehicle.wheelPoses()) {
            // Project the hub down to the ground: the contact patch is
            // directly below the hub centre for an unsuspended, upright wheel.
            const glm::dvec3 hubWorld =
                s.position + coords::localToWorld(w.localPosition, s.yaw);
            const glm::dvec3 contact{hubWorld.x, 0.02, hubWorld.z};

            m_debug.cross(contact, 0.22, debugcolor::kContact);
            m_debug.line(hubWorld, contact, debugcolor::kContact * 0.55f);
        }
    }

    // --- trajectory trail ---
    if (opt.trajectory) {
        const auto& traj = sim.trajectory();
        for (std::size_t i = 0; i + 1 < traj.size(); ++i) {
            m_debug.line({traj[i].x, 0.03, traj[i].z},
                         {traj[i + 1].x, 0.03, traj[i + 1].z},
                         debugcolor::kTrajectory);
        }
    }

    // --- instantaneous turning circle ---
    if (opt.turningRadius) {
        const double R = vehicle.turningRadius();
        if (std::isfinite(R) && std::abs(R) < 400.0) {
            //  The turn centre lies on the REAR AXLE line, perpendicular to
            //  the heading, at the signed distance R. Positive R is a right
            //  turn, and +Z (vehicle right) is where the centre goes.
            const glm::dvec3 centre = s.position + s.right() * R;

            m_debug.circle({centre.x, 0.04, centre.z}, std::abs(R),
                           debugcolor::kTurnCircle, 96);
            m_debug.line({s.position.x, 0.04, s.position.z},
                         {centre.x, 0.04, centre.z},
                         debugcolor::kTurnCircle * 0.7f);
            m_debug.cross({centre.x, 0.04, centre.z}, 0.6, debugcolor::kTurnCircle);
        }
    }

    // --- wheelbase reference line, front axle to rear axle ---
    if (opt.vehicleFrame) {
        const glm::dvec3 rear  = s.position + coords::kWorldUp * 0.05;
        const glm::dvec3 front = vehicle.frontAxlePosition() + coords::kWorldUp * 0.05;
        m_debug.line(rear, front, glm::vec3(0.9f, 0.9f, 0.2f));
        m_debug.cross(rear,  0.18, glm::vec3(0.9f, 0.9f, 0.2f));
        m_debug.cross(front, 0.18, glm::vec3(0.9f, 0.9f, 0.2f));
    }

    (void)p;  // reserved for future overlays that need parameters
}

} // namespace vsim
