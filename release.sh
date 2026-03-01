#!/usr/bin/env bash
set -euo pipefail

VERSION="$1"
if [ -z "$VERSION" ]; then
  echo "Usage: $0 <version>"
  exit 1
fi

TARGETS=(
    "aarch64-macos"
    "aarch64-linux-gnu"
    "x86_64-macos"
    "x86_64-linux-gnu"
    "x86_64-windows"
)

mkdir -p dist

for target in "${TARGETS[@]}"; do
    if [ -d "zig-out/$target" ]; then
        echo "skipping $target, output directory already exists"
        continue
    fi
    echo "Building for $target..."
    zig build -Dtarget="$target" --prefix "zig-out/$target" --release=safe --prefix-exe-dir .
    zip -r "dist/nupsx-${VERSION}-${target}.zip" "zig-out/${target}/"
done
