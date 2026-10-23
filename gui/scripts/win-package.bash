#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${CONTROLLER_BIN_DIR:-$APP_DIR/build-out}"
ASSET=rbq-controller-x64-setup.exe
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
ID=com.rainbowrobotics.rbq.desktop PKG=rbq NAME=RBQ
if [ "$CHANNEL" = nightly ]; then ID="$ID.nightly"; PKG=rbq-nightly; NAME="RBQ Nightly"; fi

cd electron
npm ci --no-audit --no-fund
rm -rf ../build-out/win
npx electron-builder --win nsis --x64 --publish never \
  -c.extraMetadata.version="$VER" -c.extraMetadata.name="$PKG" -c.appId="$ID" -c.productName="$NAME"
mkdir -p "$OUT"
cp ../build-out/win/"$ASSET" "$OUT/$ASSET"
echo "[win] $OUT/$ASSET ($(du -h "$OUT/$ASSET" | cut -f1), $NAME $VER)"
