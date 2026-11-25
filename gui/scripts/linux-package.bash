#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${CONTROLLER_BIN_DIR:-$APP_DIR/build-out}"
ARCH="$(uname -m)"
case "$ARCH" in
  x86_64) EB_ARCH=x64 ;;
  aarch64) EB_ARCH=arm64 ;;
  *) echo "[ERROR] 지원 안 하는 아키텍처: $ARCH"; exit 1 ;;
esac
ASSET="rbq-controller-$ARCH.AppImage"
SKIP_WEB=false
[ "${1:-}" = "--skip-web" ] && SKIP_WEB=true

cd "$APP_DIR"
VER="${RBQ_VERSION:-$(git describe --tags --always 2>/dev/null || echo 0.0.0-dev)}"
VER="${VER#v}"
[[ "$VER" =~ ^[0-9]+\.[0-9]+\.[0-9]+ ]] || VER="0.0.0-$VER"
if ! $SKIP_WEB; then
  RBQ_VERSION="$VER" npm run web:export -- --clear
fi
[ -f dist/index.html ] || { echo "[ERROR] dist/ 없음 — --skip-web 없이 실행"; exit 1; }

CHANNEL="${RBQ_CHANNEL:-$([ "${RBQ_NIGHTLY:-}" = 1 ] && echo nightly || echo '')}"
PKG=rbq-desktop NAME=RBQ
if [ "$CHANNEL" = nightly ]; then PKG=rbq-desktop-nightly; NAME="RBQ Nightly"; fi

cd electron
npm ci --no-audit --no-fund
rm -rf ../build-out/linux
npx electron-builder --linux AppImage "--$EB_ARCH" --publish never \
  -c.directories.output=../build-out/linux -c.afterPack=./appimage-wrap.js \
  -c.extraMetadata.version="$VER" -c.extraMetadata.name="$PKG" -c.productName="$NAME"
mkdir -p "$OUT"
cp ../build-out/linux/rbq-controller.AppImage "$OUT/$ASSET"
echo "[linux] $OUT/$ASSET ($(du -h "$OUT/$ASSET" | cut -f1), $NAME $VER)"
