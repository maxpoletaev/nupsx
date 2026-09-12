#version 410 core

layout (location = 0) in vec2 aPos;
layout (location = 1) in vec4 aColor;

out vec4 vColor;

void main() {
    vColor = aColor;
    gl_Position = vec4(aPos.x / 512.0 - 1.0, aPos.y / 256.0 - 1.0, 0.0, 1.0);
}
