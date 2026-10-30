#version 330 core

// ---------------------------------------------------------------------------
//  Basic vertex shader.
//
//  One shader serves everything: environment, vehicle and debug lines. The
//  `uUnlit` flag in the fragment shader switches between lit geometry and
//  pure vertex colour (used by the line overlays).
// ---------------------------------------------------------------------------

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;
layout(location = 3) in vec2 aUV;
layout(location = 4) in vec2 aMaterial;      // x = material id, y = parameter

// Instancing (Mesh::drawInstanced): only read when uInstanced != 0. The
// attributes are simply not enabled on any other mesh.
layout(location = 5) in mat4 aInstance;
layout(location = 9) in vec4 aInstanceTint;
uniform int uInstanced;
uniform int uWorldSpace;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
uniform mat4 uLightSpace;   // world -> shadow map clip space

out vec3 vNormal;
out vec3 vColor;
out vec3 vWorldPos;
out vec2 vUV;
out vec4 vLightSpacePos;
flat out float vMaterialId;
out float vMaterialParam;

void main()
{
    vec4 worldPos;
    if (uInstanced != 0) {
        // Instances are placed with rotation and near-uniform scale only, so
        // the upper 3x3 is a good enough normal matrix - and it avoids a
        // per-vertex matrix inverse on the scene's densest geometry.
        mat4 model = uModel * aInstance;
        worldPos = model * vec4(aPosition, 1.0);
        vNormal = mat3(model) * aNormal;
        vColor = aColor * mix(aInstanceTint.rgb, vec3(1.0), step(0.5, aMaterial.y));
    } else if (uWorldSpace != 0) {
        // Venue scenery is authored in world space: no model matrix at all.
        worldPos = vec4(aPosition, 1.0);
        vNormal = aNormal;
        vColor = aColor;
    } else {
        worldPos = uModel * vec4(aPosition, 1.0);

        // Normal matrix. Using mat3(uModel) directly is only correct for
        // uniform scaling; the inverse-transpose is correct in general.
        vNormal = mat3(transpose(inverse(uModel))) * aNormal;
        vColor = aColor;
    }
    vWorldPos = worldPos.xyz;
    vMaterialId = aMaterial.x;
    vMaterialParam = aMaterial.y;
    vUV = aUV;
    vLightSpacePos = uLightSpace * worldPos;

    gl_Position = uProjection * uView * worldPos;
}
