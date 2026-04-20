// NTSC shader, adapted from https://github.com/allkern/iris (MIT License)
// Copyright (c) 2025 Allkern/Lisandro Alarcon
#version 410 core

in vec2 TexCoord;
out vec4 FragColor;

uniform sampler2D uCompositeTex;
uniform vec2 uResolution;
uniform int uFrame;

#define PI 3.14159265358979323846
#define CHROMA_SATURATION_FACTOR 2.0

float blackman(float n, float N) {
    return 0.42 - 0.5 * cos((2.0 * PI * n) / (N - 1.0)) +
           0.08 * cos((4.0 * PI * n) / (N - 1.0));
}

float sinc(float x) {
    if (x == 0.0) return 1.0;
    float pix = PI * x;
    return sin(pix) / pix;
}

float sinc_f(float cutoff, float n, float N) {
    float cut2 = cutoff * 2.0;
    return cut2 * sinc(cut2 * (n - ((N - 1.0) / 2.0))) * 48.0;
}

float lowpass(float cutoff, float n, float N) {
    return (sinc_f(cutoff, n, N) * 2.0 - 2.0) * blackman(n, N);
}

float highpass(float cutoff, float n, float N) {
    return -(sinc_f(cutoff, n, N) * blackman(n, N));
}

const mat3 yiq_to_rgb = mat3(
    1.000,  1.000,  1.000,
    0.956, -0.272, -1.106,
    0.621, -0.647,  1.703
);

// FBO origin is bottom-left, uv has y=0 at screen top, so flip when sampling.
vec2 sampleUV(vec2 pixel) {
    return vec2(pixel.x / uResolution.x, 1.0 - pixel.y / uResolution.y);
}

void main() {
    vec2 uv = TexCoord * uResolution;
    float fc = 128.0;

    vec3 yiq = vec3(0.0);

    for (int d = 0; d < 51; d++) {
        vec2 p = vec2(uv.x + float(d) - 26.0, uv.y);
        float s = texture(uCompositeTex, sampleUV(p)).r;
        yiq.x += s * lowpass(1.0 / 6.0, float(d), 51.0);
    }
    yiq.x /= 51.0;

    for (int d = 0; d < 51; d++) {
        vec2 p = vec2(uv.x + float(d) - 26.0, uv.y);
        float s = texture(uCompositeTex, sampleUV(p)).r;
        float t = fc * (uv.x + float(d) - 26.0) + ((PI / 2.0) * uv.y) + (PI * float((uFrame >> 1) & 1));
        float filt = -highpass(0.25, float(d), 51.0);
        yiq.yz += s * filt * vec2(cos(t), sin(t));
    }

    yiq.yz /= 51.0;
    yiq.yz *= CHROMA_SATURATION_FACTOR;

    FragColor = vec4(yiq_to_rgb * yiq, 1.0);
}
