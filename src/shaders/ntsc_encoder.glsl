// NTSC shader, adapted from https://github.com/allkern/iris (MIT License)
// Copyright (c) 2025 Allkern/Lisandro Alarcon
#version 410 core

in vec2 TexCoord;
out vec4 FragColor;

uniform sampler2D uVramTex;
uniform vec2 uDisplayOffset;
uniform vec2 uDisplaySize;
uniform vec2 uDisplayRangeY;
uniform vec2 uVramSize;
uniform vec2 uResolution;
uniform int uFrame;
uniform float uNoise;

#define PI 3.14159265358979323846

float hash13(vec3 p) {
    p = fract(p * .1031);
    p += dot(p, p.zyx + 31.32);
    return fract((p.x + p.y) * p.z);
}

const mat3 rgb_to_yiq = mat3(
    0.299,  0.596,  0.211,
    0.587, -0.274, -0.523,
    0.114, -0.322,  0.312
);

void main() {
    float displayY = TexCoord.y * uDisplaySize.y;

    float y1 = uDisplayRangeY.x;
    float y2 = uDisplayRangeY.y;
    float scanlines = (y2 > y1) ? (y2 - y1) : uDisplaySize.y;
    float scale = uDisplaySize.y / 240.0;
    float visibleHeight = scanlines * scale;

    float topMargin = (uDisplaySize.y - visibleHeight) / 2.0;
    float bottomMargin = topMargin + visibleHeight;

    vec3 rgb;
    if (y2 > y1 && (displayY < topMargin || displayY >= bottomMargin)) {
        rgb = vec3(0.0);
    } else {
        float vramLine = displayY - topMargin;
        vec2 displayUV = vec2(
            uDisplayOffset.x + TexCoord.x * uDisplaySize.x,
            uDisplayOffset.y + vramLine
        );
        rgb = texture(uVramTex, displayUV / uVramSize).rgb;
    }

    vec2 uv = TexCoord * uResolution;
    float fc = 128.0;

    vec3 yiq = rgb_to_yiq * rgb;
    float f = fc * uv.x + (PI / 2.0) * uv.y + PI * float((uFrame >> 1) & 1);
    float noise = (hash13(vec3(uv.xy, float(uFrame))) * 2.0 - 1.0) * uNoise;
    float c = yiq.x + yiq.y * cos(f) + yiq.z * sin(f) + noise;

    FragColor = vec4(c, 0.0, 0.0, 1.0);
}
