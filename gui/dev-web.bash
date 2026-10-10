#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

command -v node >/dev/null 2>&1 || { echo "[ERROR] node 없음 — 먼저 'bash setup.bash' 실행 (최초 1회)" >&2; exit 1; }

SKIP_BUILD=0
if [ "${1:-}" = "-s" ] || [ "${1:-}" = "--serve-only" ]; then SKIP_BUILD=1; shift; fi
IP="${1:-127.0.0.1}"

if [ "$SKIP_BUILD" = 0 ]; then
  echo "[dev-web] 웹 빌드 (expo export --platform web)..."
  npx expo export --platform web
fi
[ -d dist ] || { echo "FAIL: dist 없음 — 먼저 빌드(-s 없이 실행)" >&2; exit 1; }

fuser -k 8090/tcp 2>/dev/null || true
sleep 1

echo "[dev-web] 프록시 기동 — robot=$IP  →  브라우저: http://localhost:8090 (LAN: http://$(hostname -I 2>/dev/null | awk '{print $1}'):8090)"
( sleep 1; xdg-open http://localhost:8090 >/dev/null 2>&1 || true ) &
export RBQ_DOWNLOAD_BASE="${RBQ_DOWNLOAD_BASE:-$(grep -hs '^EXPO_PUBLIC_DOWNLOAD_BASE=' .env.local .env | head -1 | cut -d= -f2-)}"
export RBQ_REVIEW="${RBQ_REVIEW:-$(grep -hs '^EXPO_PUBLIC_REVIEW_URL=' .env.local .env | head -1 | cut -d= -f2-)}"
exec node tools/rbq-web-proxy.js --robot "$IP" --dist dist --host 0.0.0.0
