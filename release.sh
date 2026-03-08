#!/usr/bin/env bash
set -euo pipefail

TARGETS=(
    "aarch64-macos"
    "aarch64-linux-gnu"
    "x86_64-macos"
    "x86_64-linux-gnu"
    "x86_64-windows"
)

mkdir -p dist
DIST_DIR=`cd dist && pwd`

for target in "${TARGETS[@]}"; do
    if [ -d "zig-out/$target" ]; then
        echo "skipping $target, output directory already exists"
        continue
    fi
    echo "Building for $target..."
    zig build -Dtarget="$target" --prefix "zig-out/$target" --release=safe --prefix-exe-dir .
    (cd "zig-out/$target" && zip -r "$DIST_DIR/nupsx-$target.zip" .)
done
