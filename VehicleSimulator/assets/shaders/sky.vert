#version 330 core

// ---------------------------------------------------------------------------
//  Sky pass (venue pipeline only): one triangle covering the screen. The
//  view ray for each corner is reconstructed here and interpolated.
// ---------------------------------------------------------------------------

layout(location = 0) in vec3 aPosition;   // already in clip space

uniform mat4 uInvViewProj;

out vec3 vDir;

void main()
{
    vec4 nearP = uInvViewProj * vec4(aPosition.xy, -1.0, 1.0);
    vec4 farP  = uInvViewProj * vec4(aPosition.xy,  1.0, 1.0);
    vDir = farP.xyz / farP.w - nearP.xyz / nearP.w;
    gl_Position = vec4(aPosition.xy, 1.0, 1.0);   // on the far plane
}
