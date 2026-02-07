#version 410 core

in vec2 TexCoord;
out vec4 FragColor;

uniform sampler2D texture1;
uniform vec2 uDisplayOffset;     // GP1(05h) display area start in VRAM
uniform vec2 uDisplaySize;       // display resolution from GP1(08h)
uniform vec2 uDisplayRangeY;     // GP1(07h) raw Y1, Y2 scanline values
uniform vec2 uVramSize;          // full vram size (1024x512)

void main() {
    // Convert normalized texture coordinate [0..1] to a pixel row on the output window.
    float displayY = TexCoord.y * uDisplaySize.y;

    // --- Vertical display range (GP1(07h)) ---
    // The register controls which TV scanlines (Y1..Y2) are actually visible.
    // Most games use something like Y1=16, Y2=256 (240 visible lines). The value
    // is in scanline units, while the display size in pixels can be different.
    // We need to:
    //   1. Figure out how many scanlines are visible: (Y2 - Y1).
    //   2. Scale that to our output resolution. For 480i the display is 480px 
    //      tall but Y1/Y2 are still in 240-line units, so we scale by (displayHeight / 240).
    //   3. Center the visible region vertically. Not sure what happens in reality but without
    //      the image is anchored to the top and there's a big black bar at the bottom.
    float y1 = uDisplayRangeY.x;
    float y2 = uDisplayRangeY.y;
    float scale = uDisplaySize.y / 240.0;
    float visibleHeight = (y2 - y1) * scale;

    // Black bar boundaries: center the visible region within the output window.
    float topMargin = (uDisplaySize.y - visibleHeight) / 2.0;
    float bottomMargin = topMargin + visibleHeight;

    // Pixels outside the visible scanline range are black (overscan / blanking area).
    if (y2 > y1 && (displayY < topMargin || displayY >= bottomMargin)) {
        FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }

    // --- Map the visible pixel back to a VRAM coordinate ---
    // vramLine: which line within the visible region this pixel corresponds to
    // (0 = first visible scanline). We add the VRAM display offset (GP1(05h))
    // so we read from the correct location in the VRAM.
    float vramLine = displayY - topMargin;
    vec2 displayUV = vec2(
        uDisplayOffset.x + TexCoord.x * uDisplaySize.x,
        uDisplayOffset.y + vramLine
    );

    // Normalize to [0..1] UV range for the full VRAM texture and sample.
    vec2 vramUV = displayUV / uVramSize;
    FragColor = texture(texture1, vramUV);
}
