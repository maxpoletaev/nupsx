#version 410 core

in vec4 vColor;
in vec2 vUv;
flat in uvec4 vTex;
flat in uvec2 vFlags; // tex_blend, dither
out vec4 FragColor;

uniform sampler2D uVram;
uniform int uUpscale;
uniform bool uTextured;
uniform int uDepth;
uniform int uMaskSelect;
uniform uvec2 uTexWinMask;
uniform uvec2 uTexWinOffset;

const int DEPTH_4BIT = 0;
const int DEPTH_8BIT = 1;
const int DEPTH_15BIT = 2;

const int dithering_table[16] = int[16](
    -4, 0, -3, 1,
    2, -2, 3, -1,
    -3, 1, -4, 0,
    3, -1, 2, -2
);

uint readVram(uint x, uint y) {
    vec4 c = texelFetch(uVram, ivec2(x & 1023u, y & 511u) * uUpscale, 0);
    uvec4 v = uvec4(c * vec4(31.0, 31.0, 31.0, 1.0) + 0.5);
    return v.r | (v.g << 5u) | (v.b << 10u) | (v.a << 15u);
}

ivec3 unpackRGB5(uint hw) {
    return ivec3(hw & 31u, (hw >> 5u) & 31u, (hw >> 10u) & 31u);
}

bool maskBit(uint hw) {
    return (hw >> 15u) != 0u;
}

uvec2 applyTextureWindow(uvec2 uv) {
    return (uv & ~uTexWinMask) | (uTexWinOffset & uTexWinMask);
}

uint sampleTexture(uvec2 uv) {
    uvec2 page = vTex.xy;
    uvec2 clut = vTex.zw;

    switch (uDepth) {
        case DEPTH_4BIT: {
            uint hw = readVram(page.x + uv.x / 4u, page.y + uv.y);
            uint idx = (hw >> ((uv.x % 4u) * 4u)) & 0xfu;
            return readVram(clut.x + idx, clut.y);
        }
        case DEPTH_8BIT: {
            uint hw = readVram(page.x + uv.x / 2u, page.y + uv.y);
            uint idx = (hw >> ((uv.x % 2u) * 8u)) & 0xffu;
            return readVram(clut.x + idx, clut.y);
        }
        default:
            return readVram(page.x + uv.x, page.y + uv.y);
    }
}

ivec3 vertexColor() {
    return ivec3(vColor.rgb * 255.0 + 0.5);
}

ivec3 applyDithering(ivec3 color) {
    ivec2 p = ivec2(gl_FragCoord.xy) / uUpscale;
    int v = dithering_table[(p.x & 3) * 4 + (p.y & 3)];
    return clamp(color + v, 0, 255);
}

ivec3 toRGB5(ivec3 color) {
    return color >> 3;
}

ivec3 applyBlending(ivec3 texel, ivec3 color) {
    return min(texel * color / 128, 31);
}

void writePixel(ivec3 rgb5, bool mask_bit) {
    if (uMaskSelect >= 0 && int(mask_bit) != uMaskSelect) discard;
    FragColor = vec4(vec3(rgb5) / 31.0, float(mask_bit));
}

void setPixelFlat() {
    ivec3 color = vertexColor();
    if (vFlags.y != 0u) color = applyDithering(color);
    writePixel(toRGB5(color), false);
}

void setPixelTextured() {
    ivec3 color = vertexColor();
    if (vFlags.y != 0u) color = applyDithering(color);

    uvec2 uv = applyTextureWindow(uvec2(floor(vUv)) & 0xffu);
    uint texel = sampleTexture(uv);
    if (texel == 0u) discard;

    ivec3 front = unpackRGB5(texel);
    if (vFlags.x != 0u) front = applyBlending(front, color);
    writePixel(front, maskBit(texel));
}

void main() {
    if (uTextured) {
        setPixelTextured();
    } else {
        setPixelFlat();
    }
}
