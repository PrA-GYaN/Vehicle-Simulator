#include "rendering/GltfModel.h"

#include <cctype>
#include <functional>
#include <iostream>
#include <unordered_map>

#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <cgltf.h>
#include <stb_image.h>

namespace vsim {
namespace {

//---------------------------------------------------------------------------
//  Axis parsing - see ImportAxes in the header.
//---------------------------------------------------------------------------
bool parseAxis(const std::string& s, glm::vec3& out) {
    if (s.size() != 2) return false;
    const char sign = s[0];
    const char axis = static_cast<char>(std::tolower(static_cast<unsigned char>(s[1])));
    float m = 0.0f;
    if (sign == '+') m = 1.0f;
    else if (sign == '-') m = -1.0f;
    else return false;

    if (axis == 'x') out = {m, 0.0f, 0.0f};
    else if (axis == 'y') out = {0.0f, m, 0.0f};
    else if (axis == 'z') out = {0.0f, 0.0f, m};
    else return false;
    return true;
}

/// Rotation whose ROWS are (forward, up, right) expressed in glTF space, so
/// that `R * v_gltf` gives vehicle-local (x=forward, y=up, z=right). See the
/// header comment on ImportAxes for why `right` needs no configuration.
glm::mat3 importRotation(const ImportAxes& axes, bool& ok) {
    glm::vec3 fwd, up;
    ok = parseAxis(axes.forward, fwd) && parseAxis(axes.up, up);
    if (ok) {
        const glm::vec3 right = glm::cross(fwd, up);
        ok = glm::length(right) > 0.5f;   // degenerate if forward == +-up
        if (ok) {
            glm::mat3 R(0.0f);
            R[0][0] = fwd.x;   R[1][0] = fwd.y;   R[2][0] = fwd.z;
            R[0][1] = up.x;    R[1][1] = up.y;    R[2][1] = up.z;
            R[0][2] = right.x; R[1][2] = right.y; R[2][2] = right.z;
            return R;
        }
    }
    std::cerr << "[gltf] invalid or degenerate forward/up axes ('" << axes.forward
              << "', '" << axes.up << "'); using identity\n";
    return glm::mat3(1.0f);
}

glm::mat4 worldMatrix(cgltf_node* node) {
    float m[16];
    cgltf_node_transform_world(node, m);
    return glm::make_mat4(m);
}

cgltf_attribute* findAttribute(const cgltf_primitive* prim, cgltf_attribute_type type) {
    for (cgltf_size i = 0; i < prim->attributes_count; ++i) {
        if (prim->attributes[i].type == type && prim->attributes[i].index == 0) {
            return &prim->attributes[i];
        }
    }
    return nullptr;
}

cgltf_node* findNodeByName(cgltf_node* node, const std::string& name) {
    if (name.empty()) return nullptr;
    if (node->name && name == node->name) return node;
    for (cgltf_size i = 0; i < node->children_count; ++i) {
        if (cgltf_node* found = findNodeByName(node->children[i], name)) return found;
    }
    return nullptr;
}

/// Bakes one primitive's vertex data through `transform`, appending to a
/// GltfPrimitive whose Mesh is then uploaded. Returns false (no GL upload)
/// if the primitive has no position data or is not triangles.
bool bakePrimitive(const cgltf_primitive& prim, const glm::mat4& transform,
                   const glm::vec3& tint, std::vector<Vertex>& outVerts,
                   std::vector<gl::GLuint>& outIndices) {
    if (prim.type != cgltf_primitive_type_triangles) return false;

    cgltf_attribute* posAttr = findAttribute(&prim, cgltf_attribute_type_position);
    if (!posAttr) return false;
    cgltf_attribute* nrmAttr = findAttribute(&prim, cgltf_attribute_type_normal);
    cgltf_attribute* uvAttr  = findAttribute(&prim, cgltf_attribute_type_texcoord);

    const glm::mat3 normalMat = glm::inverseTranspose(glm::mat3(transform));

    const cgltf_size vcount = posAttr->data->count;
    const gl::GLuint base = static_cast<gl::GLuint>(outVerts.size());
    outVerts.reserve(outVerts.size() + vcount);

    for (cgltf_size i = 0; i < vcount; ++i) {
        float p[3] = {0, 0, 0};
        cgltf_accessor_read_float(posAttr->data, i, p, 3);
        float n[3] = {0, 1, 0};
        if (nrmAttr) cgltf_accessor_read_float(nrmAttr->data, i, n, 3);
        float uv[2] = {0, 0};
        if (uvAttr) cgltf_accessor_read_float(uvAttr->data, i, uv, 2);

        const glm::vec4 wp = transform * glm::vec4(p[0], p[1], p[2], 1.0f);
        glm::vec3 wn = normalMat * glm::vec3(n[0], n[1], n[2]);
        if (glm::length(wn) > 1e-8f) wn = glm::normalize(wn);

        Vertex v;
        v.position = glm::vec3(wp);
        v.normal   = wn;
        v.color    = tint;
        v.uv       = {uv[0], uv[1]};
        outVerts.push_back(v);
    }

    if (prim.indices) {
        const cgltf_size icount = prim.indices->count;
        outIndices.reserve(outIndices.size() + icount);
        for (cgltf_size i = 0; i < icount; ++i) {
            outIndices.push_back(base + static_cast<gl::GLuint>(cgltf_accessor_read_index(prim.indices, i)));
        }
    } else {
        // Non-indexed primitive: every source has had indices so far, but
        // handle it rather than silently dropping the geometry.
        outIndices.reserve(outIndices.size() + vcount);
        for (cgltf_size i = 0; i < vcount; ++i) outIndices.push_back(base + static_cast<gl::GLuint>(i));
    }
    return true;
}

} // namespace

//===========================================================================
//  Loader
//===========================================================================

bool GltfModel::load(const std::string& glbPath, const ImportAxes& axes,
                     const WheelNodeNames& wheelNodes) {
    m_static.clear();
    m_translucentImages.clear();
    for (auto& w : m_wheels) w.clear();
    m_hasWheel.fill(false);
    m_hubHeightOffset.fill(0.0f);
    m_staticPoints.clear();
    m_boundsMin = m_boundsMax = glm::vec3(0.0f);
    m_loaded = false;

    cgltf_options options{};
    cgltf_data* data = nullptr;

    cgltf_result result = cgltf_parse_file(&options, glbPath.c_str(), &data);
    if (result != cgltf_result_success) {
        std::cerr << "[gltf] failed to parse '" << glbPath << "' (code " << result << ")\n";
        return false;
    }
    result = cgltf_load_buffers(&options, data, glbPath.c_str());
    if (result != cgltf_result_success) {
        std::cerr << "[gltf] failed to load buffers for '" << glbPath << "' (code " << result << ")\n";
        cgltf_free(data);
        return false;
    }
    if (data->scenes_count == 0 || data->scene == nullptr) {
        std::cerr << "[gltf] '" << glbPath << "' has no default scene\n";
        cgltf_free(data);
        return false;
    }

    bool axesOk = false;
    const glm::mat3 R = importRotation(axes, axesOk);

    //-----------------------------------------------------------------------
    //  Locate the four wheel hub nodes (by name, anywhere in the scene) and
    //  measure the rear-axle offset. See the header comment on GltfModel for
    //  why this is measured rather than authored.
    //-----------------------------------------------------------------------
    std::array<cgltf_node*, WheelCount> wheelRoot{nullptr, nullptr, nullptr, nullptr};
    const std::array<const std::string*, WheelCount> wheelNames{
        &wheelNodes.frontLeft, &wheelNodes.frontRight,
        &wheelNodes.rearLeft,  &wheelNodes.rearRight};

    for (cgltf_size s = 0; s < data->scene->nodes_count; ++s) {
        for (int w = 0; w < WheelCount; ++w) {
            if (!wheelRoot[static_cast<std::size_t>(w)] && !wheelNames[static_cast<std::size_t>(w)]->empty()) {
                wheelRoot[static_cast<std::size_t>(w)] =
                    findNodeByName(data->scene->nodes[s], *wheelNames[static_cast<std::size_t>(w)]);
            }
        }
    }
    for (int w = 0; w < WheelCount; ++w) {
        m_hasWheel[static_cast<std::size_t>(w)] = (wheelRoot[static_cast<std::size_t>(w)] != nullptr);
        if (!m_hasWheel[static_cast<std::size_t>(w)] && !wheelNames[static_cast<std::size_t>(w)]->empty()) {
            std::cerr << "[gltf] wheel node '" << *wheelNames[static_cast<std::size_t>(w)]
                      << "' not found in '" << glbPath << "'\n";
        }
    }

    glm::vec3 offsetLocal(0.0f);
    if (wheelRoot[RearLeft] && wheelRoot[RearRight]) {
        const glm::vec3 rl(worldMatrix(wheelRoot[RearLeft])[3]);
        const glm::vec3 rr(worldMatrix(wheelRoot[RearRight])[3]);
        const glm::vec3 rearMid = 0.5f * (rl + rr);
        offsetLocal = R * rearMid;
        offsetLocal.y = 0.0f;   // ground is measured, not derived - see header

        for (int w = 0; w < WheelCount; ++w) {
            if (!wheelRoot[static_cast<std::size_t>(w)]) continue;
            const glm::vec3 hub = R * glm::vec3(worldMatrix(wheelRoot[static_cast<std::size_t>(w)])[3]);
            m_hubHeightOffset[static_cast<std::size_t>(w)] = hub.y - (R * rearMid).y;
        }
    } else if (!wheelNodes.rearLeft.empty() || !wheelNodes.rearRight.empty()) {
        // Only a real problem if the caller actually asked for wheels - a
        // scenery/terrain model (empty WheelNodeNames, see the header) is
        // expected to have none, and keeping the file's own origin is
        // exactly right for it.
        std::cerr << "[gltf] rear wheel hubs not found in '" << glbPath
                  << "'; vehicle-local origin will be the file's own origin\n";
    }

    const glm::mat4 importTransform =
        glm::translate(glm::mat4(1.0f), -offsetLocal) * glm::mat4(R);

    //-----------------------------------------------------------------------
    //  Texture cache, keyed by cgltf_image* so the (few) unique images are
    //  decoded once no matter how many primitives share a material.
    //-----------------------------------------------------------------------
    std::unordered_map<const cgltf_image*, std::shared_ptr<Texture>> textureCache;

    auto loadTexture = [&](const cgltf_image* img) -> std::shared_ptr<Texture> {
        if (!img) return nullptr;
        auto it = textureCache.find(img);
        if (it != textureCache.end()) return it->second;

        std::shared_ptr<Texture> tex;
        if (img->buffer_view) {
            const std::uint8_t* bytes = cgltf_buffer_view_data(img->buffer_view);
            const int size = static_cast<int>(img->buffer_view->size);
            int w = 0, h = 0, channels = 0;
            unsigned char* pixels = stbi_load_from_memory(bytes, size, &w, &h, &channels, 4);
            if (pixels) {
                // Does this image actually see through anywhere? A BLEND
                // material whose texture is opaque everywhere (the GT3's
                // body) must stay in the opaque pass; only one with real
                // translucent texels (its glass) goes to the glass pass.
                std::size_t translucent = 0;
                const std::size_t count = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
                for (std::size_t i = 0; i < count; ++i) {
                    if (pixels[i * 4 + 3] < 242) ++translucent;
                }
                m_translucentImages[img] = count > 0 && translucent * 100 > count;
                tex = std::make_shared<Texture>();
                tex->upload(pixels, w, h);
                stbi_image_free(pixels);
            } else {
                std::cerr << "[gltf] failed to decode embedded image '"
                          << (img->name ? img->name : "?") << "': " << stbi_failure_reason() << "\n";
            }
        }
        textureCache.emplace(img, tex);
        return tex;
    };

    auto materialOf = [&](const cgltf_primitive& prim, glm::vec3& tint) -> std::shared_ptr<Texture> {
        tint = glm::vec3(1.0f);
        m_lastAlpha = 1.0f;
        m_lastBlend = prim.material && prim.material->alpha_mode == cgltf_alpha_mode_blend;
        m_lastTextureAlpha = false;
        if (m_lastBlend && prim.material->has_pbr_metallic_roughness) {
            m_lastAlpha = prim.material->pbr_metallic_roughness.base_color_factor[3];
        }
        if (!prim.material || !prim.material->has_pbr_metallic_roughness) return nullptr;
        const cgltf_pbr_metallic_roughness& pbr = prim.material->pbr_metallic_roughness;
        tint = {pbr.base_color_factor[0], pbr.base_color_factor[1], pbr.base_color_factor[2]};
        if (pbr.base_color_texture.texture) {
            const cgltf_image* image = pbr.base_color_texture.texture->image;
            auto tex = loadTexture(image);
            auto it = m_translucentImages.find(image);
            m_lastTextureAlpha = m_lastBlend && it != m_translucentImages.end() && it->second;
            return tex;
        }
        return nullptr;
    };

    //-----------------------------------------------------------------------
    //  Bake. Two passes over the same recursive shape: the static pass
    //  bakes into vehicle-local space via `importTransform`; the per-wheel
    //  pass bakes into that wheel's own hub-relative space (no translation,
    //  rotation only) so the caller can place it with the physics-derived
    //  wheel pose at runtime, exactly like the procedural wheel mesh.
    //-----------------------------------------------------------------------
    std::size_t staticPrimCount = 0, wheelPrimCount = 0;

    // Bake every primitive of ONE node's mesh (no recursion) through
    // `nodeTransform`, appending the result to `out`. Shared by the static
    // and per-wheel traversals below - they differ only in which transform
    // and which node set they visit, not in how a single node is baked.
    // `points`, when non-null, receives every baked vertex position - used
    // only for the static pass, to build a TerrainHeightField from later
    // without re-parsing the glb (see the header comment on staticPoints()).
    auto bakeNode = [&](cgltf_node* node, const glm::mat4& nodeTransform,
                        std::vector<GltfPrimitive>& out,
                        std::vector<glm::vec3>* points) {
        if (!node->mesh) return;
        for (cgltf_size p = 0; p < node->mesh->primitives_count; ++p) {
            const cgltf_primitive& prim = node->mesh->primitives[p];
            glm::vec3 tint;
            std::shared_ptr<Texture> tex = materialOf(prim, tint);

            std::vector<Vertex> verts;
            std::vector<gl::GLuint> indices;
            if (!bakePrimitive(prim, nodeTransform, tint, verts, indices)) continue;

            if (points) {
                points->reserve(points->size() + verts.size());
                for (const Vertex& v : verts) points->push_back(v.position);
            }

            GltfPrimitive gp;
            gp.baseColorTexture = std::move(tex);
            gp.tint = tint;
            gp.transparent = m_lastBlend && (m_lastAlpha < 0.999f || m_lastTextureAlpha);
            gp.textureAlpha = m_lastTextureAlpha;
            gp.alpha = m_lastAlpha;
            gp.mesh.upload(verts, indices, gl::TRIANGLES);
            out.push_back(std::move(gp));
        }
    };

    // Per-wheel: every node in the subtree rooted at that wheel's hub node,
    // baked relative to the hub (rotation only - see the header comment).
    std::function<void(cgltf_node*, const glm::mat4&, std::vector<GltfPrimitive>&)> bakeSubtree =
        [&](cgltf_node* node, const glm::mat4& relativeTransform, std::vector<GltfPrimitive>& out) {
            bakeNode(node, relativeTransform * worldMatrix(node), out, nullptr);
            for (cgltf_size i = 0; i < node->children_count; ++i) {
                bakeSubtree(node->children[i], relativeTransform, out);
            }
        };

    // Static geometry: everything reachable from the scene roots, except the
    // subtrees rooted at the four wheel hub nodes.
    std::function<void(cgltf_node*)> visitStatic = [&](cgltf_node* node) {
        for (int w = 0; w < WheelCount; ++w) {
            if (node == wheelRoot[static_cast<std::size_t>(w)]) {
                const glm::mat4 hubInverse = glm::inverse(worldMatrix(node));
                const glm::mat4 wheelRelative = glm::mat4(R) * hubInverse;
                bakeSubtree(node, wheelRelative, m_wheels[static_cast<std::size_t>(w)]);
                wheelPrimCount += m_wheels[static_cast<std::size_t>(w)].size();
                return;   // do not also descend as static geometry
            }
        }
        bakeNode(node, importTransform * worldMatrix(node), m_static, &m_staticPoints);
        staticPrimCount = m_static.size();
        for (cgltf_size i = 0; i < node->children_count; ++i) visitStatic(node->children[i]);
    };

    for (cgltf_size s = 0; s < data->scene->nodes_count; ++s) {
        visitStatic(data->scene->nodes[s]);
    }

    if (!m_staticPoints.empty()) {
        m_boundsMin = m_boundsMax = m_staticPoints.front();
        for (const glm::vec3& p : m_staticPoints) {
            m_boundsMin = glm::min(m_boundsMin, p);
            m_boundsMax = glm::max(m_boundsMax, p);
        }
    }

    std::cout << "[gltf] loaded '" << glbPath << "': " << staticPrimCount
              << " static primitive(s), " << wheelPrimCount << " wheel primitive(s), "
              << textureCache.size() << " texture(s)\n";

    cgltf_free(data);
    m_loaded = !m_static.empty();
    return m_loaded;
}

} // namespace vsim
