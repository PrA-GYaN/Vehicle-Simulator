#version 330 core

// ---------------------------------------------------------------------------
//  Clear-day sky: gradient, sun, and a thin layer of fair-weather cloud.
//  skyColor() and toneMap() are the same functions basic.frag fogs and
//  reflects with - keep the two copies identical, or distant geometry will
//  fade to a colour that is not the one behind it.
// ---------------------------------------------------------------------------

in vec3 vDir;

uniform vec3  uLightDirection;
uniform vec3  uLightColor;
uniform float uSunIntensity;
uniform float uExposure;
uniform sampler2D uNoise;

out vec4 FragColor;

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

void main()
{
    vec3 d = normalize(vDir);
    vec3 c = skyColor(d);

    // Cloud layer: the noise texture's fbm channel projected onto a plane
    // overhead, thresholded high so most of the sky stays clear.
    if (d.y > 0.01) {
        vec2 uv = d.xz / (d.y + 0.10) * 0.085;
        float n = texture(uNoise, uv).b * 0.62 + texture(uNoise, uv * 2.9 + 0.31).b * 0.38;
        float cover = smoothstep(0.56, 0.80, n) * smoothstep(0.01, 0.16, d.y);
        float shade = texture(uNoise, uv * 1.7 + 0.05).g;
        vec3 cloud = mix(vec3(0.78, 0.82, 0.90), vec3(1.25, 1.22, 1.16), shade);
        c = mix(c, cloud, cover * 0.85);
    }

    float s = max(dot(d, -uLightDirection), 0.0);
    c += uLightColor * uSunIntensity * (smoothstep(0.99985, 0.99995, s) * 6.0 + pow(s, 900.0) * 0.8);

    FragColor = vec4(toneMap(c), 1.0);
}
