#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"

FORCE_WEB=false; RUN=true
while [ $# -gt 0 ]; do
    case "$1" in
        --web)    FORCE_WEB=true; shift ;;
        --no-run) RUN=false; shift ;;
        -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
        *) echo "알 수 없는 인자: $1"; exit 1 ;;
    esac
done

command -v node >/dev/null 2>&1 || { echo "[ERROR] node 없음 — 먼저 'bash setup.bash' 실행 (최초 1회)"; exit 1; }

STAMP=node_modules/.rbq-stamp
WANT="$( { cat package-lock.json 2>/dev/null; node -v; } | md5sum | cut -d' ' -f1)"
if [ "$(cat "$STAMP" 2>/dev/null)" != "$WANT" ]; then
    echo "[dev] npm ci (의존성 변경 감지)"
    npm ci
    echo "$WANT" > "$STAMP"
fi
[ -d electron/node_modules/electron ] || (cd electron && npm ci --no-audit --no-fund)

if $FORCE_WEB || [ ! -f dist/index.html ]; then
    echo "[dev] 웹 빌드"
    npm run web:export
elif [ -n "$(find src assets app.json package.json metro.config.js -newer dist/index.html -print -quit 2>/dev/null)" ]; then
    echo "[dev] 소스가 dist 보다 새로움 — 웹 재빌드"
    npm run web:export
fi

$RUN || { echo "✅ 준비 완료(실행 생략)"; exit 0; }
echo "[dev] 실행: electron"
cd electron && exec npx electron .
