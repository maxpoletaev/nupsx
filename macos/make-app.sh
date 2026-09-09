#!/usr/bin/env bash
set -euo pipefail

if ! command -v hdiutil >/dev/null 2>&1; then
    echo "error: hdiutil command not found" >&2
    exit 1
fi

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
BINARY_PATH="${BINARY_PATH:-$ROOT_DIR/zig-out/bin/nupsx}"
APP_DIR="$ROOT_DIR/macos/nuPSX.app"
DMG_PATH="$ROOT_DIR/macos/nuPSX.dmg"
VERSION="${VERSION:-${1:-0.0.0}}"
VERSION="${VERSION#v}"

if [ ! -f "$BINARY_PATH" ]; then
    echo "error: executable not found: $BINARY_PATH" >&2
    echo "build it first with: zig build --release=safe" >&2
    exit 1
fi

echo "Creating app bundle: $APP_DIR"

rm -rf "$APP_DIR"
mkdir -p "$APP_DIR/Contents/MacOS" "$APP_DIR/Contents/Resources"
cp "$BINARY_PATH" "$APP_DIR/Contents/MacOS/nupsx"
cp "$ROOT_DIR/macos/nupsx.icns" "$APP_DIR/Contents/Resources/nupsx.icns"
sed "s/@VERSION@/$VERSION/g" "$ROOT_DIR/macos/Info.plist" > "$APP_DIR/Contents/Info.plist"

echo "Creating DMG: $DMG_PATH"

STAGING_DIR=$(mktemp -d "${TMPDIR:-/tmp}/nupsx-dmg.XXXXXX")
trap 'rm -rf "$STAGING_DIR"' EXIT

cp -R "$APP_DIR" "$STAGING_DIR/"
ln -s /Applications "$STAGING_DIR/Applications"

hdiutil create \
    -volname "nuPSX" \
    -srcfolder "$STAGING_DIR" \
    -ov \
    -format UDZO \
    "$DMG_PATH"

echo "Done"
