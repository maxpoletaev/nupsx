#version 410 core

layout (location = 0) in vec2 aPos;
layout (location = 1) in vec2 aUv;
layout (location = 2) in vec4 aColor;
layout (location = 3) in uvec4 aTex;
layout (location = 4) in uvec2 aFlags;

out vec4 vColor;
out vec2 vUv;
flat out uvec4 vTex;
flat out uvec2 vFlags;

void main() {
    vColor = aColor;
    vUv = aUv;
    vTex = aTex;
    vFlags = aFlags;
    gl_Position = vec4(aPos.x / 512.0 - 1.0, aPos.y / 256.0 - 1.0, 0.0, 1.0);
}
