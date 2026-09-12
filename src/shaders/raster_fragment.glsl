#version 410 core

in vec4 vColor;
out vec4 FragColor;

void main() {
    ivec3 c = ivec3(vColor.rgb * 255.0 + 0.5) >> 3;
    FragColor = vec4(vec3(c) / 31.0, 0.0);
}
