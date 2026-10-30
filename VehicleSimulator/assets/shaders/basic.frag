#version 330 core

// ---------------------------------------------------------------------------
//  Lighting.
//
//  Still deliberately cheap - one directional light, one shadow map, no
//  post-processing - but no longer flat. The three things that were missing
//  and that a vehicle dynamics tool actually needs:
//
//    * HEMISPHERIC AMBIENT. A constant ambient term lights the underside of
//      the car exactly as brightly as the roof, which destroys every shape
//      cue on a curved body. Sky above and bounced ground below is one extra
//      mix() and is what makes a lofted panel read as curved.
//
//    * SHADOWS. Without a contact shadow there is no way to see that a wheel
//      has lifted or that the body has heaved - see ShadowMap.h.
//
//    * FOG. A circuit is kilometres long on a flat plane; without any depth
//      cue the far side of the track reads at the same distance as the near
//      side.
// ---------------------------------------------------------------------------

in vec3 vNormal;
in vec3 vColor;
in vec3 vWorldPos;
in vec2 vUV;
in vec4 vLightSpacePos;
flat in float vMaterialId;
in float vMaterialParam;

uniform vec3  uLightDirection;   // direction the light travels (normalised)
uniform vec3  uLightColor;
uniform vec3  uViewPosition;
uniform float uAmbient;
uniform vec3  uSkyColor;         // hemispheric ambient, from above
uniform vec3  uGroundColor;      // hemispheric ambient, bounced from below
uniform int   uUnlit;            // 1 = emit vertex colour unchanged

// Surface response, set per draw call: 0 = matte (asphalt, grass, tyres),
// 1 = gloss (car paint, glass). Everything between is valid.
uniform float uGloss;
// Opacity. 1 for everything except imported glass (GltfModel, alphaMode
// BLEND), which the renderer draws last with depth writes off.
uniform float uAlpha = 1.0;
// 1 in the glass pass only: take opacity from the texture's alpha too
// (models that keep their glass transparency in the texture, e.g. the GT3).
uniform int uTextureAlpha = 0;

// Fog. Exponential-squared, so it stays clear near the car and closes in at
// the far plane rather than washing out the whole scene uniformly.
uniform vec3  uFogColor;
uniform float uFogDensity;

uniform sampler2D uShadowMap;
uniform int       uShadowEnabled;

// Imported-model support (src/rendering/GltfModel.h). Every generated mesh in
// the project draws with uUseTexture = 0, in which case uTexture is never
// sampled - this is strictly additive to the vertex-colour path above.
uniform sampler2D uTexture;
uniform int       uUseTexture;

// ---------------------------------------------------------------------------
//  The venue pipeline (uPipeline = 1).
//
//  Everything above this line is the original look and is what every track
//  except a "venue" circuit still renders with, unchanged. A venue circuit
//  (rendering/Venue.h) switches the whole frame - car included - to the path
//  in venueMain() below, which differs in four ways:
//
//    * LINEAR LIGHT. Colours are authored as display values; they are
//      converted to linear, lit with a sun several times brighter than the
//      sky, tone mapped (ACES fit) and encoded back. That is what gives
//      sunlit and shaded surfaces a believable ratio without clipping.
//    * MATERIALS. Each vertex carries a material id (aMaterial), so one
//      batched mesh can hold asphalt, paint, steel and glass and still have
//      each respond differently. Surface detail comes from one small tiling
//      noise texture sampled in world space, so it cannot stretch.
//    * TWO SHADOW MAPS. The original tight map around the car, plus a wide
//      one for buildings, stands and trees further out.
//    * SKY. Fog and reflections use the same analytic sky the sky pass draws.
// ---------------------------------------------------------------------------
uniform int       uPipeline;
uniform sampler2D uNoise;          // R fine grain, G blotch, B cloud fbm, A streak
uniform sampler2DShadow uShadowNearCmp;   // the near map, read by comparison
uniform sampler2DShadow uShadowFarCmp;
uniform mat4      uLightSpace;     // near map
uniform mat4      uLightSpaceFar;
uniform int       uShadowFarEnabled;
uniform float     uSunIntensity;
uniform float     uExposure;

// Surface materials: photographed textures, one array layer per material
// (tools/bake_venue_textures.py). uMatAlbedo is colour normalised to
// mid-grey; uMatDetail is normal.xy in RG and roughness in B.
uniform sampler2DArray uMatAlbedo;
uniform sampler2DArray uMatDetail;

out vec4 FragColor;

vec3 toLinear(vec3 c) { return pow(max(c, vec3(0.0)), vec3(2.2)); }

// Analytic clear-day sky, linear. Shared (as text) with sky.frag.
vec3 skyColor(vec3 d)
{
    float t = pow(clamp(d.y, 0.0, 1.0), 0.42);
    vec3 c = mix(vec3(0.56, 0.72, 0.94), vec3(0.07, 0.21, 0.60), t);
    float s = max(dot(d, -uLightDirection), 0.0);
    c += vec3(1.0, 0.80, 0.52) * 0.28 * pow(s, 6.0) * (1.0 - 0.6 * t);
    if (d.y < 0.0) c = mix(c, vec3(0.25, 0.30, 0.24), clamp(-d.y * 5.0, 0.0, 1.0));
    return c;
}

vec3 toneMap(vec3 x)
{
    x *= uExposure;
    x = (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);
    return pow(clamp(x, 0.0, 1.0), vec3(1.0 / 2.2));
}

// Four hardware-filtered comparison taps: each is already a 2x2 bilinear
// percentage-closer lookup, so this is a smooth 3x3-texel kernel for the
// price of four fetches.
float shadowTaps(sampler2DShadow map, vec3 p, float bias, float spread)
{
    vec2 texel = spread / vec2(textureSize(map, 0));
    vec3 q = vec3(p.xy, p.z - bias);
    float lit = texture(map, q + vec3(-0.5, -0.5, 0.0) * vec3(texel, 0.0))
              + texture(map, q + vec3( 0.5, -0.5, 0.0) * vec3(texel, 0.0))
              + texture(map, q + vec3(-0.5,  0.5, 0.0) * vec3(texel, 0.0))
              + texture(map, q + vec3( 0.5,  0.5, 0.0) * vec3(texel, 0.0));
    return lit * 0.25;
}

// Near map where it has coverage, far map beyond, lit past both. The receiver
// is pushed along its normal by about a texel instead of using a depth bias
// alone, which is what lets one bias serve walls and ground alike.
float venueShadow(vec3 N)
{
    if (uShadowEnabled == 0) return 1.0;

    float lit = 1.0;
    float wNear = 0.0;

    vec4 ln = uLightSpace * vec4(vWorldPos + N * 0.05, 1.0);
    vec3 pn = ln.xyz * 0.5 + 0.5;
    float edgeN = max(abs(pn.x - 0.5), abs(pn.y - 0.5)) * 2.0;
    if (edgeN < 1.0 && pn.z < 1.0) wNear = 1.0 - smoothstep(0.86, 1.0, edgeN);

    if (wNear < 1.0 && uShadowFarEnabled != 0) {
        vec4 lf = uLightSpaceFar * vec4(vWorldPos + N * 0.40, 1.0);
        vec3 pf = lf.xyz * 0.5 + 0.5;
        float edgeF = max(abs(pf.x - 0.5), abs(pf.y - 0.5)) * 2.0;
        if (edgeF < 1.0 && pf.z < 1.0) {
            float wFar = 1.0 - smoothstep(0.80, 1.0, edgeF);
            lit = mix(1.0, shadowTaps(uShadowFarCmp, pf, 0.0012, 1.0), wFar);
        }
    }
    if (wNear > 0.0) lit = mix(lit, shadowTaps(uShadowNearCmp, pn, 0.0010, 2.0), wNear);
    return lit;
}

float hash21(vec2 p)
{
    p = fract(p * vec2(123.34, 456.21));
    p += dot(p, p + 45.32);
    return fract(p.x * p.y);
}

// ---------------------------------------------------------------------------
//  Texturing without texture coordinates.
//
//  Venue geometry carries no material UVs. A surface is textured by
//  projecting its WORLD position along whichever axis its normal is closest
//  to - straight down for the ground, sideways for a wall - which cannot
//  stretch, needs nothing baked into the mesh, and keeps the texel size the
//  same on everything in the scene. The same choice of axis gives the two
//  directions the normal map's x and y push the normal in.
//
//  surface() returns the colour multiplier (about 1 on average: see the
//  normalisation in the bake tool) and bends N by the normal map.
// ---------------------------------------------------------------------------
vec3 surface(float layer, float metres, float bump, inout vec3 N, out float texRough)
{
    vec3 a = abs(N);
    vec2 uv;
    vec3 T, B;
    if (a.y >= a.x && a.y >= a.z) { uv = vWorldPos.xz; T = vec3(1.0, 0.0, 0.0); B = vec3(0.0, 0.0, 1.0); }
    else if (a.x >= a.z)          { uv = vWorldPos.zy; T = vec3(0.0, 0.0, 1.0); B = vec3(0.0, 1.0, 0.0); }
    else                          { uv = vWorldPos.xy; T = vec3(1.0, 0.0, 0.0); B = vec3(0.0, 1.0, 0.0); }
    uv /= metres;

    vec3 d = texture(uMatDetail, vec3(uv, layer)).rgb;
    vec2 n = (d.xy * 2.0 - 1.0) * bump;
    N = normalize(N + T * n.x + B * n.y);
    texRough = d.b;
    return texture(uMatAlbedo, vec3(uv, layer)).rgb * 2.0;
}

// The same texture again, several times larger and offset: multiplied in, it
// breaks up the repeat that any tiling texture shows across a field or a
// straight seen from a height.
float macro(float layer, float metres)
{
    return texture(uMatAlbedo, vec3(vWorldPos.xz / metres + 0.37, layer)).g * 2.0;
}

void venueMain()
{
    vec3 base = vColor;
    float alpha = uAlpha;
    if (uUseTexture != 0) {
        vec4 t = texture(uTexture, vUV);
        base *= t.rgb;
        if (uTextureAlpha != 0) alpha *= t.a;
    }
    if (uUnlit != 0) { FragColor = vec4(base, 1.0); return; }

    int mat = int(vMaterialId + 0.5);

    // Back faces are lit from behind - except foliage cards, whose normals
    // point outward from the middle of the crown whichever way the card
    // happens to face, so that a tree shades as one rounded mass.
    vec3 N = normalize(vNormal);
    if (!gl_FrontFacing && mat != 17) N = -N;
    vec3 L = normalize(-uLightDirection);
    vec3 toEye = uViewPosition - vWorldPos;
    float dist = length(toEye);
    vec3 V = toEye / max(dist, 1e-4);

    vec2 w = vWorldPos.xz;

    vec3 albedo = toLinear(base);
    float rough = mix(0.90, 0.22, uGloss);
    float f0    = 0.04;
    float metal = 0.0;
    float env   = uGloss * uGloss;     // strength of the sky reflection
    float wrap  = 0.0;
    vec3  emission = vec3(0.0);
    bool  transmit = false;
    float tr = 1.0;                    // roughness read from the material texture

    if (mat == 1) {                    // ---- asphalt ----
        vec3 t = surface(0.0, 3.1, 0.9, N, tr);
        float med = texture(uNoise, w * 0.061 + 0.37).g;
        float big = texture(uNoise, w.yx * 0.0093 + 0.11).b;
        albedo = toLinear(base * t) * (0.86 + 0.28 * med) * (0.84 + 0.32 * big);
        // Rubber: laid in streaks along the direction of travel, so it is
        // sampled in track coordinates (uv = lateral, arc length) and
        // stretched hard along the lap. Rubber also fills the texture of
        // the surface in: darker, smoother, and shinier.
        float s1 = texture(uNoise, vec2(vUV.x * 0.45, vUV.y * 0.020)).a;
        float s2 = texture(uNoise, vec2(vUV.x * 1.70 + 0.5, vUV.y * 0.006)).a;
        float r = clamp(vMaterialParam * (0.40 + 2.2 * s1 * s2), 0.0, 1.0);
        albedo = mix(albedo, albedo * 0.30, r);
        rough = mix(mix(0.80, 0.98, tr), 0.58, r);
        env = 0.0;
    } else if (mat == 2) {             // ---- grass ----
        vec3 t = surface(1.0, 2.3, 1.0, N, tr);
        float big = texture(uNoise, w * 0.0049 + 0.63).b;
        albedo = toLinear(base * t) * (0.55 + 0.45 * macro(1.0, 17.0));
        albedo = mix(albedo, albedo * vec3(1.35, 1.12, 0.60), smoothstep(0.46, 0.80, big) * 0.50);
        if (vMaterialParam > 0.5) albedo *= (fract(vUV.x / 5.0) < 0.5) ? 0.93 : 1.07;   // mowing
        rough = 1.0; f0 = 0.02; wrap = 0.25; env = 0.0;
    } else if (mat == 3) {             // ---- gravel ----
        vec3 t = surface(2.0, 1.9, 1.3, N, tr);
        albedo = toLinear(base * t) * (0.45 + 0.25 * macro(2.0, 11.0));
        rough = 1.0; env = 0.0;
    } else if (mat == 4) {             // ---- concrete ----
        vec3 t = surface(3.0, 3.4, 0.8, N, tr);
        albedo = toLinear(base * mix(vec3(1.0), t, 0.55));
        rough = mix(0.70, 0.95, tr); env = 0.0;
    } else if (mat == 5) {             // ---- galvanised steel ----
        vec3 t = surface(5.0, 1.7, 0.5, N, tr);
        albedo = toLinear(base * t);
        rough = mix(0.30, 0.60, tr); metal = 0.55; f0 = 0.5; env = 0.55;
    } else if (mat == 6) {             // ---- architectural glass ----
        rough = 0.06; f0 = 0.08; env = 1.0;
    } else if (mat == 7) {             // ---- road paint ----
        // Paint follows the asphalt it is on: same grain, and it wears off
        // the high spots of the aggregate first.
        vec3 t = surface(0.0, 3.1, 0.55, N, tr);
        float med = texture(uNoise, w * 0.17 + 0.7).g;
        albedo = toLinear(base * mix(vec3(1.0), t, 0.30));
        albedo = mix(albedo, vec3(0.05), smoothstep(0.60, 0.95, med) * 0.30);
        rough = 0.62; env = 0.0;
    } else if (mat == 8) {             // ---- kerb paint ----
        vec3 t = surface(3.0, 2.1, 0.5, N, tr);
        float scuff = texture(uNoise, vec2(vUV.x * 2.3, vUV.y * 0.03)).a;
        albedo = toLinear(base * mix(vec3(1.0), t, 0.45));
        albedo = mix(albedo, albedo * 0.45, smoothstep(0.45, 0.9, scuff) * 0.45);
        rough = 0.55; env = 0.0;
    } else if (mat == 9) {             // ---- tyre rubber ----
        vec3 t = surface(6.0, 0.9, 0.8, N, tr);
        albedo = toLinear(base * t);
        rough = mix(0.60, 0.90, tr); env = 0.0;
    } else if (mat == 10) {            // ---- foliage, solid (distant trees) ----
        float keep = 1.0;
        vec3 flat_ = N;
        vec3 t = surface(9.0, 3.3, 0.0, flat_, keep);
        albedo = toLinear(base * mix(vec3(1.0), t, 0.75));
        rough = 1.0; f0 = 0.02; wrap = 0.45; env = 0.0; transmit = true;
    } else if (mat == 11) {            // ---- rubber decal (skid marks) ----
        float s1 = texture(uNoise, vec2(vUV.x * 3.1, vUV.y * 0.05)).a;
        alpha *= clamp(vMaterialParam * (0.35 + 1.3 * s1), 0.0, 1.0);
        rough = 0.6; env = 0.0;
    } else if (mat == 12) {            // ---- catch-fence mesh ----
        // Diamond mesh, 50 mm pitch. Drawn analytically and faded to its
        // own average coverage as it gets too fine to resolve, so a fence
        // 200 m away is a grey veil rather than a moire pattern.
        vec2 g = vec2(vUV.x + vUV.y, vUV.x - vUV.y) * 14.0;
        vec2 f = abs(fract(g) - 0.5);
        float fw = max(fwidth(g.x), fwidth(g.y));
        float line = smoothstep(0.42 - fw, 0.42 + fw, max(f.x, f.y));
        float cover = mix(line, 0.30, clamp(fw * 2.2, 0.0, 1.0));
        alpha *= cover * 0.92;
        rough = 0.5; metal = 0.4; f0 = 0.4; env = 0.2;
    } else if (mat == 13) {            // ---- crowd / seating ----
        // uv counts seats (x) and rows (y). A hashed share of the seats is
        // occupied and takes a hashed clothing colour; the rest show the
        // seat colour the vertex carries. vMaterialParam is the occupancy.
        vec2 cell = floor(vUV);
        float h1 = hash21(cell);
        float h2 = hash21(cell + 17.3);
        float h3 = hash21(cell + 41.7);
        vec3 cloth = mix(vec3(0.02 + 0.5 * h2, 0.03 + 0.4 * h3, 0.05 + 0.5 * fract(h2 * 7.0)),
                         vec3(0.55, 0.55, 0.55), step(0.7, h3) * 0.8);
        float occupied = step(h1, vMaterialParam);
        vec2 q = fract(vUV) - 0.5;
        float blob = 1.0 - smoothstep(0.30, 0.42, max(abs(q.x), abs(q.y)));
        float fwc = clamp(max(fwidth(vUV.x), fwidth(vUV.y)) * 1.5, 0.0, 1.0);
        blob = mix(blob, 0.62, fwc);
        albedo = mix(albedo, cloth, occupied * blob);
        rough = 0.9; env = 0.0;
    } else if (mat == 14) {            // ---- lit panel (screens, lamps) ----
        emission = albedo * vMaterialParam;
        albedo *= 0.2; rough = 0.3; env = 0.3;
    } else if (mat == 15) {            // ---- painted panel / cladding ----
        vec3 t = surface(4.0, 4.5, 0.45, N, tr);
        albedo = toLinear(base * mix(vec3(1.0), t, 0.6));
        rough = mix(0.35, 0.60, tr); env = 0.08;
    } else if (mat == 16) {            // ---- water ----
        vec2 rp = texture(uNoise, w * 0.05).rg + texture(uNoise, w.yx * 0.13 + 0.3).gb - 1.0;
        N = normalize(N + vec3(rp.x, 0.0, rp.y) * 0.10);
        rough = 0.05; f0 = 0.03; env = 1.0;
    } else if (mat == 17) {            // ---- foliage card ----
        // A cut-out: the opacity is in the detail layer's blue channel.
        // Sharpened to about a pixel wide and resolved by alpha-to-coverage,
        // which gives the leaves anti-aliased edges with no sorting.
        float o = texture(uMatDetail, vec3(vUV, 9.0)).b;
        //
        // There is deliberately no `discard` here. A shader that can discard
        // loses the early depth test for EVERY fragment it shades, and this
        // one shades the whole scene; with coverage doing the cutting, a
        // fully transparent fragment covers no samples and writes nothing.
        o = clamp((o - 0.45) / max(fwidth(o), 0.02) + 0.5, 0.0, 1.0);
        alpha *= o;
        albedo = toLinear(base * texture(uMatAlbedo, vec3(vUV, 9.0)).rgb * 2.0);
        rough = 0.85; f0 = 0.03; wrap = 0.55; env = 0.0; transmit = true;
    } else if (mat == 18) {            // ---- bark ----
        vec3 t = surface(8.0, 1.3, 1.1, N, tr);
        albedo = toLinear(base * t);
        rough = 0.95; env = 0.0;
    } else if (mat == 19) {            // ---- profiled steel roofing ----
        vec3 t = surface(7.0, 2.2, 1.0, N, tr);
        albedo = toLinear(base * mix(vec3(1.0), t, 0.5));
        rough = mix(0.30, 0.55, tr); f0 = 0.08; env = 0.12;
    }

    float shadow = venueShadow(N);

    vec3 sun = uLightColor * uSunIntensity;
    float ndl = dot(N, L);
    float diffuse = clamp((ndl + wrap) / (1.0 + wrap), 0.0, 1.0);

    vec3 skyAmb = vec3(0.42, 0.53, 0.74) * uAmbient;
    vec3 gndAmb = toLinear(uGroundColor) * 1.6 * uAmbient + vec3(0.02);
    vec3 ambient = mix(gndAmb, skyAmb, N.y * 0.5 + 0.5);

    vec3 H = normalize(L + V);
    float a2 = max(rough * rough * rough * rough, 0.0005);
    float power = clamp(2.0 / a2 - 2.0, 1.0, 4096.0);
    float fresH = f0 + (1.0 - f0) * pow(1.0 - max(dot(V, H), 0.0), 5.0);
    vec3 specTint = mix(vec3(1.0), albedo, metal);
    vec3 spec = sun * specTint * fresH * ((power + 2.0) / 8.0) *
                pow(max(dot(N, H), 0.0), power) * max(ndl, 0.0) * shadow;

    vec3 color = albedo * (1.0 - metal) * (ambient + sun * diffuse * shadow) + spec + emission;
    if (transmit) color += albedo * sun * max(-ndl, 0.0) * 0.30 * shadow;

    if (env > 0.0) {
        float ndv = max(dot(N, V), 0.0);
        float fresV = f0 + (1.0 - f0) * pow(1.0 - ndv, 5.0);
        vec3 R = reflect(-V, N);
        color += skyColor(R) * specTint * fresV * env * mix(0.55, 1.0, shadow);
    }

    // Aerial perspective toward the horizon colour in the view direction.
    float fog = 1.0 - exp(-dist * uFogDensity);
    vec3 fogDir = normalize(vec3(-V.x, 0.04, -V.z));
    color = mix(color, skyColor(fogDir), clamp(fog, 0.0, 1.0));

    FragColor = vec4(toneMap(color), alpha);
}

// Percentage-closer filtering over a 3x3 neighbourhood. Returns the fraction
// of the light that reaches this fragment, 0 (fully shadowed) to 1.
float shadowFactor(vec3 N, vec3 L)
{
    if (uShadowEnabled == 0) return 1.0;

    vec3 proj = vLightSpacePos.xyz / vLightSpacePos.w;
    proj = proj * 0.5 + 0.5;

    // Past the far plane of the light's frustum there is no information;
    // treat it as lit rather than guessing.
    if (proj.z > 1.0) return 1.0;

    // Slope-scaled bias: a surface seen edge-on from the light spans many
    // times more depth per texel than one facing it, and a single constant
    // bias cannot serve both without either acne or visible peter-panning.
    float cosTheta = clamp(dot(N, L), 0.0, 1.0);
    float bias = clamp(0.0015 * tan(acos(cosTheta)), 0.0005, 0.006);

    vec2 texel = 1.0 / vec2(textureSize(uShadowMap, 0));
    float lit = 0.0;
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            float depth = texture(uShadowMap, proj.xy + vec2(x, y) * texel).r;
            lit += (proj.z - bias > depth) ? 0.0 : 1.0;
        }
    }
    return lit / 9.0;
}

void main()
{
    if (uPipeline != 0) { venueMain(); return; }

    vec3 base = vColor;
    float alpha = uAlpha;
    if (uUseTexture != 0) {
        vec4 t = texture(uTexture, vUV);
        base *= t.rgb;
        if (uTextureAlpha != 0) alpha *= t.a;
    }

    if (uUnlit != 0) {
        FragColor = vec4(base, 1.0);
        return;
    }

    vec3 N = normalize(vNormal);
    vec3 L = normalize(-uLightDirection);
    vec3 V = normalize(uViewPosition - vWorldPos);

    // Hemispheric ambient: sky from above, bounced ground light from below.
    float hemi = N.y * 0.5 + 0.5;
    vec3 ambient = mix(uGroundColor, uSkyColor, hemi) * uAmbient;

    float shadow = shadowFactor(N, L);
    float diffuse = max(dot(N, L), 0.0) * shadow;

    // Blinn-Phong. The exponent and strength both ride uGloss so one uniform
    // covers asphalt through to car paint.
    vec3 H = normalize(L + V);
    float shininess = mix(16.0, 96.0, uGloss);
    float specStrength = mix(0.03, 0.55, uGloss);
    float spec = pow(max(dot(N, H), 0.0), shininess) * specStrength * shadow;

    // Weak Fresnel rim, which is what stops a dark car from becoming a
    // silhouette with no edge against a dark road.
    float fresnel = pow(1.0 - max(dot(N, V), 0.0), 5.0) * mix(0.02, 0.18, uGloss);

    vec3 color = base * (ambient + uLightColor * diffuse)
               + uLightColor * spec
               + uSkyColor * fresnel;

    float dist = length(uViewPosition - vWorldPos);
    float fog = 1.0 - exp(-pow(dist * uFogDensity, 2.0));
    color = mix(color, uFogColor, clamp(fog, 0.0, 1.0));

    FragColor = vec4(color, alpha);
}
