#version 410 core

in vec2 TexCoord;
out vec4 FragColor;

uniform sampler2D texture1;
uniform vec2 uDisplayOffset; // GP1(05h) display area start (already in texture-pixel units)
uniform vec2 uDisplaySize; // display resolution from GP1(08h) and video mode
uniform int uVideoMode; // GP1(08h).3: 0=NTSC, 1=PAL
uniform vec2 uDisplayRangeY; // GP1(07h) raw Y1, Y2 scanline values
uniform vec2 uVramSize; // full vram size in texture pixels (1024x512 or 682x512)

void main() {
    // Convert normalized texture coordinate [0..1] to a pixel row on the output window.
    float displayY = TexCoord.y * uDisplaySize.y;

    // Vertical display range (GP1(07h)):
    // The register controls which TV scanlines (Y1..Y2) are actually visible.
    // Y1/Y2 are single-field scanline units. Interlaced output doubles the
    // display target height, so scale the visible field range to output pixels.
    float y1 = uDisplayRangeY.x;
    float y2 = uDisplayRangeY.y;
    float scanlines = (y2 > y1) ? (y2 - y1) : uDisplaySize.y;
    float fieldHeight = (uVideoMode == 1) ? 288.0 : 240.0;
    float scale = uDisplaySize.y / fieldHeight;
    float visibleHeight = scanlines * scale;

    // Black bar boundaries: center the visible region within the output window.
    float topMargin = (uDisplaySize.y - visibleHeight) / 2.0;
    float bottomMargin = topMargin + visibleHeight;

    // Pixels outside the visible scanline range are black (overscan / blanking area).
    if (y2 > y1 && (displayY < topMargin || displayY >= bottomMargin)) {
        FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }

    // Map the visible pixel back to a VRAM coordinate:
    // vramLine: which line within the visible region this pixel corresponds to
    // (0 = first visible scanline). We add the VRAM display offset (GP1(05h))
    // so we read from the correct location in the VRAM.
    float vramLine = displayY - topMargin;
    vec2 displayUV = vec2(
        uDisplayOffset.x + TexCoord.x * uDisplaySize.x,
        uDisplayOffset.y + vramLine
    );

    ivec2 texel = ivec2(displayUV);
    FragColor = texelFetch(texture1, texel, 0);
}
