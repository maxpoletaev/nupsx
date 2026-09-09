#!/usr/bin/env bash

src="$1"
out="${2:-AppIcon.icns}"
set="AppIcon.iconset"

mkdir -p "$set"

sips -z 16 16 "$src" --out "$set/icon_16x16.png"
sips -z 32 32 "$src" --out "$set/icon_16x16@2x.png"
sips -z 32 32 "$src" --out "$set/icon_32x32.png"
sips -z 64 64 "$src" --out "$set/icon_32x32@2x.png"
sips -z 128 128 "$src" --out "$set/icon_128x128.png"
sips -z 256 256 "$src" --out "$set/icon_128x128@2x.png"
sips -z 256 256 "$src" --out "$set/icon_256x256.png"
sips -z 512 512 "$src" --out "$set/icon_256x256@2x.png"
sips -z 512 512 "$src" --out "$set/icon_512x512.png"
sips -z 1024 1024 "$src" --out "$set/icon_512x512@2x.png"

iconutil -c icns "$set" -o "$out"
rm -rf "$set"
