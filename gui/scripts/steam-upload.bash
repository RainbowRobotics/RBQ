#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE="${CONTROLLER_BIN_DIR:-$APP_DIR/build-out}"
WORK="$BASE/steampipe"
: "${STEAM_APP_ID:?Steamworks App ID 필요}" "${STEAM_USER:?빌드 계정 필요}"
LINUX="${STEAM_DEPOT_LINUX:-}" WIN="${STEAM_DEPOT_WIN:-}"
[ -n "$LINUX$WIN" ] || { echo "[ERROR] STEAM_DEPOT_LINUX / STEAM_DEPOT_WIN 중 하나는 필요"; exit 1; }
BRANCH=""; PREVIEW=0
while [ $# -gt 0 ]; do
  case "$1" in
    --branch) BRANCH="$2"; shift 2 ;;
    --preview) PREVIEW=1; shift ;;
    *) echo "[ERROR] 모르는 인자: $1"; exit 1 ;;
  esac
done
if [ -n "$LINUX" ]; then
  [ -x "$BASE/steam/rbq.sh" ] && [ -x "$BASE/steam/rbq" ] || { echo "[ERROR] $BASE/steam 없음/실행권한 없음 — steam-package.bash 먼저"; exit 1; }
fi
if [ -n "$WIN" ]; then
  [ -f "$BASE/steam-win/rbq.exe" ] || { echo "[ERROR] $BASE/steam-win/rbq.exe 없음 — 윈도우 PC 에서 win-package.bash --steam 후 가져올 것"; exit 1; }
fi
command -v steamcmd >/dev/null || { echo "[ERROR] steamcmd 없음 — https://developer.valvesoftware.com/wiki/SteamCMD"; exit 1; }

VERSION="$(cd "$APP_DIR" && git describe --tags --always 2>/dev/null || echo dev)"
mkdir -p "$WORK/output"
DEPOTS=""
depot() {
  cat > "$WORK/depot_build_$1.vdf" <<EOF
"DepotBuild"
{
  "DepotID" "$1"
  "ContentRoot" "$2"
  "FileMapping" { "LocalPath" "*" "DepotPath" "." "Recursive" "1" }
}
EOF
  DEPOTS="$DEPOTS \"$1\" \"$WORK/depot_build_$1.vdf\""
}
[ -n "$LINUX" ] && depot "$LINUX" "$BASE/steam"
[ -n "$WIN" ] && depot "$WIN" "$BASE/steam-win"
cat > "$WORK/app_build_$STEAM_APP_ID.vdf" <<EOF
"AppBuild"
{
  "AppID" "$STEAM_APP_ID"
  "Desc" "$VERSION"
  "Preview" "$PREVIEW"
  "SetLive" "$BRANCH"
  "BuildOutput" "$WORK/output"
  "Depots" {$DEPOTS }
}
EOF
echo "[steampipe] $VERSION → app $STEAM_APP_ID depot linux=${LINUX:--} win=${WIN:--}${BRANCH:+ (live: $BRANCH)}$([ $PREVIEW = 1 ] && echo ' [preview]')"
exec steamcmd +login "$STEAM_USER" +run_app_build "$WORK/app_build_$STEAM_APP_ID.vdf" +quit
