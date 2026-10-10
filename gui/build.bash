#!/usr/bin/env bash

set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${CONTROLLER_BIN_DIR:-$APP_DIR/build-out}"
HOST_OS="$(uname -s)"

sha256_16() { { command -v sha256sum >/dev/null && sha256sum "$1" || shasum -a 256 "$1"; } | cut -c1-16; }
java_major() { java -version 2>&1 | head -1 | sed -E 's/.*version "([0-9]+).*/\1/' | grep -E '^[0-9]+$' || echo 0; }

BUILD_WEB=false
BUILD_APK=false
BUILD_DESKTOP=false
BUILD_IOS=false
CLEAN_NATIVE=false

if [ $# -eq 0 ]; then
    BUILD_WEB=true
    case "$HOST_OS" in
        Linux)  BUILD_APK=true; BUILD_DESKTOP=true ;;
        Darwin) BUILD_APK=true; BUILD_DESKTOP=true ;;
        MINGW*|MSYS*|CYGWIN*) BUILD_APK=true; BUILD_DESKTOP=true ;;
        *) echo "[WARN] 미확인 OS($HOST_OS) — web만 빌드" ;;
    esac
fi
while [[ $# -gt 0 ]]; do
    case "$1" in
        --web)     BUILD_WEB=true;     shift ;;
        --apk)     BUILD_APK=true;     shift ;;
        --linux|--macos|--desktop) BUILD_DESKTOP=true; shift ;;
        --ios)     BUILD_IOS=true;     shift ;;
        --all)     BUILD_WEB=true; BUILD_APK=true; BUILD_DESKTOP=true; shift ;;
        --clean-native) CLEAN_NATIVE=true; shift ;;
        *) echo "[ERROR] Unknown argument: $1"; exit 1 ;;
    esac
done

need() {
    command -v "$1" >/dev/null 2>&1 || {
        echo "[ERROR] '$1' 없음 — 레포 루트에서 'bash gui/setup.bash' 실행 필요 ($2)"
        exit 1
    }
}

need node "웹/APK/데스크탑 공통"
cd "$APP_DIR"

LOCK_DIR="$APP_DIR/.build.lock"
if ! mkdir "$LOCK_DIR" 2>/dev/null; then
    echo "[ERROR] 다른 빌드가 진행 중 ($LOCK_DIR 존재) — 끝나길 기다리거나, 죽은 빌드면 'rmdir $LOCK_DIR' 후 재시도"
    exit 1
fi
trap 'rmdir "$LOCK_DIR" 2>/dev/null' EXIT

APP_VERSION="${RBQ_VERSION:-$(git -C "$APP_DIR" describe --tags --dirty --always 2>/dev/null || true)}"
APP_VERSION="${APP_VERSION#v}"
[[ "$APP_VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+ ]] || APP_VERSION="0.0.0-dev"
export RBQ_VERSION="$APP_VERSION"
echo "[INFO] 버전: $APP_VERSION"

VER_STAMP="$APP_DIR/.expo/rbq-version-stamp"
if [ "$(cat "$VER_STAMP" 2>/dev/null)" != "$APP_VERSION" ]; then
    echo "[INFO] 버전 변경 감지 — metro 캐시 무효화 (${TMPDIR:-/tmp}/metro-cache)"
    rm -rf "${TMPDIR:-/tmp}/metro-cache"
    mkdir -p "$APP_DIR/.expo" && echo "$APP_VERSION" > "$VER_STAMP"
fi
mkdir -p "$OUT_DIR"

PREBUILD_STAMP="$APP_DIR cfg:$(cat app.json app.config.js 2>/dev/null | { command -v sha256sum >/dev/null && sha256sum || shasum -a 256; } | cut -c1-16) ch:${RBQ_NIGHTLY:-0}/${RBQ_PLAY:-0}/${RBQ_CHANNEL:-none}"

sync_version_name() {
    local f="$1"
    [ -f "$f" ] || return 0
    grep -q "versionName \"${APP_VERSION}\"" "$f" && return 0
    sed -i.bak -E "s/(versionName )\"[^\"]*\"/\1\"${APP_VERSION}\"/" "$f" && rm -f "$f.bak"
    echo "[INFO] versionName → ${APP_VERSION} (prebuild 재생성 없이 제자리 갱신)"
}

STAMP_FILE="node_modules/.rbq-stamp"
WANT_STAMP="node$(node -v | cut -d. -f1)-lock$(sha256_16 package-lock.json)"
if [ "$(cat "$STAMP_FILE" 2>/dev/null)" != "$WANT_STAMP" ]; then
    echo "[INFO] 의존성 설치 (npm ci) — 최초이거나 lock/node 버전 변경"
    npm ci
    echo "$WANT_STAMP" > "$STAMP_FILE"
fi

if $CLEAN_NATIVE; then
    echo "[INFO] 네이티브 캐시 정리 (.cxx + node_modules 내 android/build + gradle clean)..."
    [ -d node_modules ] && find node_modules -maxdepth 3 -type d -name .cxx -path '*/android/*' -exec rm -rf {} + 2>/dev/null || true
    [ -d node_modules ] && find node_modules -maxdepth 3 -type d -name build -path '*/android/*' -exec rm -rf {} + 2>/dev/null || true
    rm -rf android/app/.cxx android/.cxx
    [ -d android ] && (cd android && ./gradlew clean --console=plain)
    echo "✅ 네이티브 캐시 정리 완료"
fi

node tools/i18n-check.js || { echo "[ERROR] i18n 누락 — 위 목록을 채우고 다시 빌드할 것"; exit 1; }
envval() { local v="${!1:-}"; [ -n "$v" ] || v=$(grep -hs "^$1=" .env.local .env | head -1 | cut -d= -f2-); printf %s "$v"; }

for V in EXPO_PUBLIC_SUPABASE_URL EXPO_PUBLIC_SUPABASE_ANON_KEY; do
    if [ -z "${!V:-}" ] && ! grep -qs "^$V=." .env .env.local; then
        echo "⚠️  $V 없음(env·.env·.env.local 모두) — 원격 로그인·로그 업로드 없는 빌드가 된다"
    fi
done

if $BUILD_WEB; then
    echo "[INFO] 웹 빌드 (expo export)..."
    npm run mujoco:sync
    npx expo export --platform web
    rm -rf "$OUT_DIR/web"
    cp -r dist "$OUT_DIR/web"

    case "$HOST_OS" in Darwin) PROXY_PAT="*apple-darwin*" ;; *) PROXY_PAT="*linux*" ;; esac
    find_proxy() { PROXY_BIN=""; for f in src-tauri/binaries/rbq-proxy-$PROXY_PAT src-tauri/binaries/rbq-proxy-*; do [ -f "$f" ] && PROXY_BIN="$f" && break; done; return 0; }
    find_proxy
    if [ -z "$PROXY_BIN" ] && command -v rustc >/dev/null 2>&1; then
        npm run proxy:bundle
        find_proxy
    fi
    if [ -n "$PROXY_BIN" ]; then
        cp "$PROXY_BIN" "$OUT_DIR/rbq-proxy"
        cat > "$OUT_DIR/run-web.sh" <<'RUNWEB'
#!/usr/bin/env bash
# RBQ Controller web: ./run-web.sh [robot IP] [vision IP]
# Default 127.0.0.1 = local simulator. Open http://<this PC>:8090 (served to the LAN).
# Uses port 8090 like the desktop app — run one at a time.
cd "$(dirname "$0")"
# The packaged proxy resolves relative paths inside its own snapshot, so --dist must be absolute
RUNWEB
        printf ': "${RBQ_DOWNLOAD_BASE:=%s}" "${RBQ_REVIEW:=%s}"\nexport RBQ_DOWNLOAD_BASE RBQ_REVIEW\n' \
            "$(envval EXPO_PUBLIC_DOWNLOAD_BASE)" "$(envval EXPO_PUBLIC_REVIEW_URL)" >> "$OUT_DIR/run-web.sh"
        cat >> "$OUT_DIR/run-web.sh" <<'RUNWEB'
exec ./rbq-proxy --robot "${1:-127.0.0.1}" ${2:+--robot-vision "$2"} --port 8090 --dist "$(pwd)/web" --host 0.0.0.0
RUNWEB
        chmod +x "$OUT_DIR/run-web.sh"
    else
        echo "[WARN] 프록시 실행파일 없음(rustc 필요) — 웹 실행은 'node tools/rbq-web-proxy.js' 사용"
    fi

    cat > "$OUT_DIR/README.txt" <<'ARTREADME'
RBQ Controller build output

- rbq-controller-<version>.apk     Android package
- RBQ_*.AppImage                   Ubuntu executable (no install: chmod +x, then run)
- RBQ_*.deb                        Ubuntu package: sudo apt install ./RBQ_*.deb
- web/ + run-web.sh + rbq-proxy    Web version: ./run-web.sh [robot IP]  ->  browser http://<this PC>:8090
                                   (no IP = local simulator; Node not required)

Note: the desktop app and run-web.sh both use port 8090 — run one at a time.
run-web.sh serves the whole LAN (phones/tablets on the same network can open it).
ARTREADME
    echo "✅ 웹: $OUT_DIR/web/ (+run-web.sh, README.txt)"
fi

if $BUILD_APK; then
    need java "APK 빌드"
    JAVA_MAJOR="$(java_major)"
    if [ "${JAVA_MAJOR:-0}" -lt 17 ]; then
        JDK17="$(ls -d /usr/lib/jvm/java-1[7-9]-openjdk* /usr/lib/jvm/java-2[0-9]-openjdk* 2>/dev/null | head -1 || true)"
        if [ -z "$JDK17" ] && [ "$HOST_OS" = Darwin ] && [ -x /usr/libexec/java_home ]; then
            JDK17="$(/usr/libexec/java_home -v 17+ 2>/dev/null || true)"
        fi
        if [ -n "$JDK17" ]; then
            export JAVA_HOME="$JDK17"
            export PATH="$JAVA_HOME/bin:$PATH"
            echo "[INFO] 기본 java가 ${JAVA_MAJOR}이라 JAVA_HOME을 17+로 지정: $JDK17"
        else
            echo "[ERROR] gradle은 JVM 17+ 필요 (현재 ${JAVA_MAJOR}) — 'bash gui/setup.bash' 실행"
            exit 1
        fi
    fi
    if [ -z "${ANDROID_HOME:-}" ]; then
        for cand in /opt/android-sdk "$APP_DIR/../3rdparty/android-sdk" "$HOME/android-sdk" \
                    "$HOME/Library/Android/sdk" "$HOME/Android/Sdk" \
                    "${LOCALAPPDATA:-}/Android/Sdk" "$HOME/AppData/Local/Android/Sdk"; do
            [ "$cand" = "/Android/Sdk" ] && continue
            if [ -d "$cand" ]; then export ANDROID_HOME="$cand"; break; fi
        done
    fi
    [ -n "${ANDROID_HOME:-}" ] || { echo "[ERROR] Android SDK 없음 (ANDROID_HOME)"; exit 1; }

    if [ -d android ] && [ "$(cat android/.rbq-stamp 2>/dev/null)" != "$PREBUILD_STAMP" ]; then
        echo "[INFO] android/가 다른 경로/설정에서 생성됨 — 재생성 (경로 전환 또는 app.json 변경)"
        rm -rf android
    fi
    if [ ! -d android ]; then
        echo "[INFO] android/ 없음 — expo prebuild 실행..."
        npx expo prebuild --platform android --no-install
        echo "$PREBUILD_STAMP" > android/.rbq-stamp
    fi
    sync_version_name android/app/build.gradle
    echo "[INFO] 시뮬 워커 자산 동기화 → android_asset/mujoco"
    npm run mujoco:sync
    rm -rf android/app/src/main/assets/mujoco
    mkdir -p android/app/src/main/assets
    cp -r public/mujoco android/app/src/main/assets/mujoco
    echo "[INFO] APK 빌드 (gradle assembleRelease)..."
    (cd android && ./gradlew assembleRelease --console=plain)
    cp android/app/build/outputs/apk/release/app-release.apk \
       "$OUT_DIR/rbq-controller-${APP_VERSION}.apk"
    echo "✅ APK: $OUT_DIR/rbq-controller-${APP_VERSION}.apk"
fi

if $BUILD_DESKTOP; then
    [ -f "$HOME/.cargo/env" ] && . "$HOME/.cargo/env"
    need cargo "데스크탑(Tauri) 빌드"
    case "$(uname -s)" in
        Linux)  TAURI_BUNDLES="deb,appimage" ;;
        Darwin) TAURI_BUNDLES="app,dmg" ;;
        MINGW*|MSYS*|CYGWIN*) TAURI_BUNDLES="nsis" ;;
        *) echo "[WARN] 미지원 OS — 데스크탑 빌드 건너뜀"; TAURI_BUNDLES="" ;;
    esac
    if [ -n "$TAURI_BUNDLES" ]; then
        if [ -x /opt/linuxdeploy/linuxdeploy-x86_64.AppImage ]; then
            TAURI_CACHE="${XDG_CACHE_HOME:-$HOME/.cache}/tauri"
            mkdir -p "$TAURI_CACHE"
            cp -f /opt/linuxdeploy/linuxdeploy-x86_64.AppImage "$TAURI_CACHE/"
            cp -f /opt/linuxdeploy/linuxdeploy-plugin-appimage-x86_64.AppImage \
                  "$TAURI_CACHE/linuxdeploy-plugin-appimage.AppImage"
            cp -f /opt/linuxdeploy/linuxdeploy-plugin-gtk.sh \
                  /opt/linuxdeploy/linuxdeploy-plugin-gstreamer.sh "$TAURI_CACHE/"
            chmod +x "$TAURI_CACHE"/linuxdeploy-*.AppImage "$TAURI_CACHE"/linuxdeploy-plugin-*.sh 2>/dev/null || true
            echo "[INFO] linuxdeploy 고정본 사용(본체+플러그인 3종): $TAURI_CACHE"
        fi
        echo "[INFO] 데스크탑 빌드 (tauri, bundles: $TAURI_BUNDLES)..."
        CI=true npm run tauri build -- --bundles "$TAURI_BUNDLES" --config "{\"version\":\"$APP_VERSION\"}"
        if [[ "$TAURI_BUNDLES" == *appimage* ]]; then
            bash scripts/fix-appimage-proxy.sh
        fi
        find src-tauri/target/release/bundle -maxdepth 2 -type f -name "*${APP_VERSION}*" \
            \( -name '*.deb' -o -name '*.AppImage' -o -name '*.exe' -o -name '*.dmg' \) \
            -exec cp {} "$OUT_DIR/" \;
        echo "✅ 데스크탑: $OUT_DIR/ (${TAURI_BUNDLES})"
    fi
fi

if $BUILD_IOS; then
    if [ "$HOST_OS" != Darwin ]; then
        echo "[ERROR] --ios는 macOS + Xcode에서만 가능"; exit 1
    fi
    need pod "iOS 빌드 (CocoaPods)"
    need xcodebuild "iOS 빌드 (Xcode)"
    if [ -d ios ] && [ "$(cat ios/.rbq-stamp 2>/dev/null)" != "$PREBUILD_STAMP" ]; then
        echo "[INFO] ios/가 다른 경로/설정에서 생성됨 — 재생성"
        rm -rf ios
    fi
    if [ ! -d ios ]; then
        echo "[INFO] ios/ 없음 — expo prebuild 실행..."
        npx expo prebuild --platform ios --no-install
        echo "$PREBUILD_STAMP" > ios/.rbq-stamp
    fi
    echo "[INFO] 시뮬 워커 자산 동기화 → ios/RBQ/mujoco"
    npm run mujoco:sync
    IOS_APP_DIR=$(ls -d ios/*/ 2>/dev/null | grep -vE "Pods|build|\.xcodeproj|\.xcworkspace" | head -1)
    if [ -n "$IOS_APP_DIR" ]; then
        rm -rf "${IOS_APP_DIR}mujoco" && cp -r public/mujoco "${IOS_APP_DIR}mujoco"
    fi
    (cd ios && pod install)
    WS="$(ls -d ios/*.xcworkspace | head -1)"
    SCHEME="$(basename "$WS" .xcworkspace)"
    echo "[INFO] iOS 시뮬레이터 빌드 (xcodebuild, scheme: $SCHEME)..."
    xcodebuild -workspace "$WS" -scheme "$SCHEME" -configuration Release \
        -sdk iphonesimulator -derivedDataPath ios/build \
        CODE_SIGNING_ALLOWED=NO build | tail -5
    APP_BUNDLE="$(ls -d ios/build/Build/Products/Release-iphonesimulator/*.app | head -1)"
    (cd "$(dirname "$APP_BUNDLE")" && zip -qry "$OUT_DIR/rbq-controller-${APP_VERSION}-ios-sim.zip" "$(basename "$APP_BUNDLE")")
    echo "✅ iOS(시뮬레이터): $OUT_DIR/rbq-controller-${APP_VERSION}-ios-sim.zip"
fi

echo ""
echo "빌드 완료 — 산출물 ($OUT_DIR):"
ls -lh "$OUT_DIR" | tail -n +2
[ -f "$OUT_DIR/README.txt" ] && echo "설치/실행 방법: $OUT_DIR/README.txt" || true
