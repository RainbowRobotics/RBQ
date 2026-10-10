#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"

FORCE_WEB=false; PROFILE=debug; RUN=true
while [ $# -gt 0 ]; do
    case "$1" in
        --web)     FORCE_WEB=true; shift ;;
        --release) PROFILE=release; shift ;;
        --no-run)  RUN=false; shift ;;
        -h|--help) printf '%s\n' 'Usage: bash dev.bash [--web] [--release] [--no-run]' \
            '  (no flag)  re-export the web bundle if sources changed, build the desktop shell (debug) and run it' \
            '  --web      force re-export of the web bundle' '  --release  release profile (slow)' '  --no-run   build only'; exit 0 ;;
        *) echo "알 수 없는 인자: $1"; exit 1 ;;
    esac
done

command -v node >/dev/null 2>&1 || { echo "[ERROR] node 없음 — 먼저 'bash setup.bash' 실행 (최초 1회)"; exit 1; }
[ -f "$HOME/.cargo/env" ] && . "$HOME/.cargo/env"
command -v cargo >/dev/null || { echo "[ERROR] cargo 없음 — bash setup.bash 또는 도커 빌드(scripts/docker/run.bash) 사용"; exit 1; }

pkill -x app 2>/dev/null || true
pkill -f '[r]bq-proxy' 2>/dev/null || true
pkill -f '[r]bq-web-proxy' 2>/dev/null || true
if command -v fuser >/dev/null; then
    fuser -k 8090/tcp 2>/dev/null || true
elif command -v ss >/dev/null; then
    for p in $(ss -ltnpH 2>/dev/null | grep ':8090' | grep -oP 'pid=\K[0-9]+' | sort -u); do kill -9 "$p" 2>/dev/null || true; done
elif command -v lsof >/dev/null; then
    for p in $(lsof -tiTCP:8090 -sTCP:LISTEN 2>/dev/null); do kill -9 "$p" 2>/dev/null || true; done
fi
sleep 0.7
if command -v ss >/dev/null && ss -ltn 2>/dev/null | grep -q ':8090'; then
    echo "[dev] ⚠ 8090을 아직 누가 점유 중 — 앱은 다음 빈 포트(8091…)로 뜬다. 남의 프록시를 보고 있다면 이걸 먼저 정리:"
    ss -ltnp 2>/dev/null | grep ':8090' || true
fi

STAMP=node_modules/.rbq-stamp
WANT="$( { cat package-lock.json 2>/dev/null; node -v; } | md5sum | cut -d' ' -f1)"
if [ ! -f "$STAMP" ] || [ "$(cat "$STAMP" 2>/dev/null)" != "$WANT" ]; then
    echo "[dev] npm ci (의존성 변경 감지)"
    npm ci
    echo "$WANT" > "$STAMP"
fi

if $FORCE_WEB || [ ! -f dist/index.html ]; then
    echo "[dev] 웹 빌드 (expo export)"
    npx expo export --platform web
elif [ -n "$(find src assets app.json package.json metro.config.js -newer dist/index.html -print -quit 2>/dev/null)" ]; then
    echo "[dev] 소스가 dist보다 새로움 — 웹 재빌드 (expo export)"
    npx expo export --platform web
fi
PROXY_BIN_LATEST="$(ls -t src-tauri/binaries/rbq-proxy-* 2>/dev/null | head -1)"
if [ -z "$PROXY_BIN_LATEST" ]; then
    echo "[dev] 프록시 사이드카 스테이징 (최초 1회)"
    npm run proxy:bundle
elif [ -n "$(find tools scripts/bundle-proxy.mjs -newer "$PROXY_BIN_LATEST" -print -quit 2>/dev/null)" ]; then
    echo "[dev] 프록시 소스가 사이드카보다 새로움 — 재번들"
    npm run proxy:bundle
fi

echo "[dev] cargo 빌드 ($PROFILE)"
if [ "$PROFILE" = release ]; then
    (cd src-tauri && cargo build --release)
    BIN=src-tauri/target/release/app
else
    (cd src-tauri && cargo build)
    BIN=src-tauri/target/debug/app
fi
[ -x "$BIN" ] || { echo "[ERROR] 바이너리 없음: $BIN"; exit 1; }

$RUN || { echo "✅ 빌드 완료(실행 생략): $BIN"; exit 0; }

TDIR="$(dirname "$BIN")"
for f in src-tauri/binaries/rbq-proxy-*; do [ -f "$f" ] && cp -u "$f" "$TDIR/$(basename "$f")"; done
[ -f src-tauri/binaries/ffmpeg ] && cp -u src-tauri/binaries/ffmpeg "$TDIR/ffmpeg"
[ -d src-tauri/binaries/wrtc-modules ] && cp -ru src-tauri/binaries/wrtc-modules "$TDIR/" 2>/dev/null || true

rm -rf "$HOME/.local/share/com.rainbowrobotics.rbq/WebKitCache" \
       "$HOME/.local/share/com.rainbowrobotics.rbq/CacheStorage" 2>/dev/null || true

export WEBKIT_DISABLE_DMABUF_RENDERER="${WEBKIT_DISABLE_DMABUF_RENDERER:-1}"
export GDK_BACKEND="${GDK_BACKEND:-x11}"

echo "[dev] 실행: $BIN"
exec "$BIN"
