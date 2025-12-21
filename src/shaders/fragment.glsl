#version 410 core

in vec2 TexCoord;
out vec4 FragColor;

uniform sampler2D texture1;
uniform vec2 uDisplayOffset;  // display area start position in VRAM
uniform vec2 uDisplaySize;    // display area size
uniform vec2 uVramSize;       // full vram size (1024x512)

void main() {
    vec2 displayUV = uDisplayOffset + TexCoord * uDisplaySize; // map textcoord [0,1] to display area
    vec2 vramUV = displayUV / uVramSize; // normalize vram coords to [0,1]
    FragColor = texture(texture1, vramUV);
}
