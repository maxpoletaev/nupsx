#!/usr/bin/env bash
set -euo pipefail

if ! command -v lipo >/dev/null 2>&1; then
    echo "error: lipo command not found" >&2
    exit 1
fi

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
VERSION="${VERSION:-${1:-0.0.0}}"
VERSION="${VERSION#v}"

# Temporary workdir for artifacts
WORK_DIR=$(mktemp -d "${TMPDIR:-/tmp}/nupsx-universal.XXXXXX")
trap 'rm -rf "$WORK_DIR"' EXIT

# Build the arm and Intel executables
echo "Building aarch64 executable..."
(cd "$ROOT_DIR" && zig build \
    -Dtarget=aarch64-macos \
    --release=safe \
    --prefix "$WORK_DIR/aarch64" \
    --prefix-exe-dir .)

echo "Building Intel executable..."
(cd "$ROOT_DIR" && zig build \
    -Dtarget=x86_64-macos \
    --release=safe \
    --prefix "$WORK_DIR/x86_64" \
    --prefix-exe-dir .)

echo "Creating universal executable..."
lipo -create \
    "$WORK_DIR/aarch64/nupsx" \
    "$WORK_DIR/x86_64/nupsx" \
    -output "$WORK_DIR/nupsx"

# Bundle the universal executable and create the DMG
BINARY_PATH="$WORK_DIR/nupsx" VERSION="$VERSION" "$ROOT_DIR/macos/make-app.sh"
