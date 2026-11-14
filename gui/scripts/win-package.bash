#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${CONTROLLER_BIN_DIR:-$APP_DIR/build-out}"
ASSET=rbq-controller-x64-setup.exe
SKIP_WEB=false STEAM=false STORE=false
for a in "$@"; do
  case "$a" in
    --skip-web) SKIP_WEB=true ;;
    --steam) STEAM=true ;;
    --msstore) STORE=true ;;
    *) echo "[ERROR] 모르는 인자: $a"; exit 1 ;;
  esac
done
{ $STEAM || $STORE; } && export RBQ_CHANNEL=release
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
TARGET=nsis; $STEAM && TARGET=dir; $STORE && TARGET=appx
STORE_ARGS=()
$STORE && STORE_ARGS=(-c.directories.buildResources=store -c.appx.identityName=RainbowRobotics.RBQ
  -c.appx.publisher=CN=0B8D41BC-AB20-41EF-8F5F-FC784252966E -c.appx.publisherDisplayName="Rainbow Robotics"
  -c.appx.applicationId=RBQ -c.appx.displayName=RBQ -c.appx.artifactName='rbq-controller-x64.${ext}')
$STORE && export CSC_IDENTITY_AUTO_DISCOVERY=false && unset CSC_LINK CSC_KEY_PASSWORD
npx electron-builder --win "$TARGET" --x64 --publish never \
  -c.extraMetadata.version="$VER" -c.extraMetadata.name="$PKG" -c.appId="$ID" -c.productName="$NAME" "${STORE_ARGS[@]}"
mkdir -p "$OUT"
if $STORE; then
  cp ../build-out/win/rbq-controller-x64.appx "$OUT/"
  echo "[win] $OUT/rbq-controller-x64.appx ($(du -h "$OUT/rbq-controller-x64.appx" | cut -f1), $NAME $VER)"
  exit 0
fi
if $STEAM; then
  rm -rf "$OUT/steam-win"
  cp -a ../build-out/win/win-unpacked "$OUT/steam-win"
  echo "[win] $OUT/steam-win ($(du -sh "$OUT/steam-win" | cut -f1), $NAME $VER)"
  exit 0
fi
cp ../build-out/win/"$ASSET" "$OUT/$ASSET"
echo "[win] $OUT/$ASSET ($(du -h "$OUT/$ASSET" | cut -f1), $NAME $VER)"
