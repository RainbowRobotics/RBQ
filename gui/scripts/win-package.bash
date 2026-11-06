#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${CONTROLLER_BIN_DIR:-$APP_DIR/build-out}"
ASSET=rbq-controller-x64-setup.exe
SKIP_WEB=false STEAM=false
for a in "$@"; do
  case "$a" in
    --skip-web) SKIP_WEB=true ;;
    --steam) STEAM=true ;;
    *) echo "[ERROR] 모르는 인자: $a"; exit 1 ;;
  esac
done
$STEAM && export RBQ_CHANNEL=release
$STEAM && export EXPO_PUBLIC_STEAM=1

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
npx electron-builder --win "$($STEAM && echo dir || echo nsis)" --x64 --publish never \
  -c.extraMetadata.version="$VER" -c.extraMetadata.name="$PKG" -c.appId="$ID" -c.productName="$NAME"
mkdir -p "$OUT"
if $STEAM; then
  rm -rf "$OUT/steam-win"
  cp -a ../build-out/win/win-unpacked "$OUT/steam-win"
  echo "[win] $OUT/steam-win ($(du -sh "$OUT/steam-win" | cut -f1), $NAME $VER)"
  exit 0
fi
cp ../build-out/win/"$ASSET" "$OUT/$ASSET"
echo "[win] $OUT/$ASSET ($(du -h "$OUT/$ASSET" | cut -f1), $NAME $VER)"
