#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${CONTROLLER_BIN_DIR:-$APP_DIR/build-out}/steam"
SKIP_WEB=false
[ "${1:-}" = "--skip-web" ] && SKIP_WEB=true

cd "$APP_DIR"
export RBQ_CHANNEL="${RBQ_CHANNEL:-release}"
if ! $SKIP_WEB; then
  RBQ_VERSION="${RBQ_VERSION:-$(git describe --tags --always 2>/dev/null || echo 0.0.0-dev)}" npm run web:export -- --clear
fi
[ -f dist/index.html ] || { echo "[ERROR] dist/ 없음 — --skip-web 없이 실행"; exit 1; }

(cd electron && npm ci --no-audit --no-fund)
ELECTRON_DIST="electron/node_modules/electron/dist"
[ -x "$ELECTRON_DIST/electron" ] || (cd electron && node node_modules/electron/install.js)

rm -rf "$OUT"
mkdir -p "$OUT/resources/app"
cp -a "$ELECTRON_DIST"/. "$OUT/"
rm -f "$OUT/chrome-sandbox" "$OUT/resources/default_app.asar"
mv "$OUT/electron" "$OUT/rbq"

APP="$OUT/resources/app"
cp electron/main.js electron/preload.js electron/wifi.js electron/package.json electron/package-lock.json "$APP/"
(cd "$APP" && npm ci --omit=dev --no-audit --no-fund)
rm -f "$APP/package-lock.json"
mkdir -p "$APP/tools"
cp tools/rbq-web-proxy.js "$APP/tools/"
cp -a dist "$APP/dist"

cat > "$OUT/rbq.sh" <<'EOF'
#!/bin/sh
here="$(cd "$(dirname "$0")" && pwd)"
# Steam sets GTK_IM_MODULE=Steam for games, so Chromium pops the Steam on-screen keyboard on every input focus,
# on top of the app's own keyboard (src/lib/osk). It also steals focus and never returns it to the field.
# Turn the input method off so only the app keyboard is used.
unset GTK_IM_MODULE QT_IM_MODULE
exec "$here/rbq" --no-sandbox --ozone-platform=x11 "$@"
EOF
chmod +x "$OUT/rbq.sh" "$OUT/rbq" "$OUT/chrome_crashpad_handler"

echo "[steam] $OUT ($(du -sh "$OUT" | cut -f1))"
