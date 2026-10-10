#!/usr/bin/env bash
set -e
APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST_OS="$(uname -s)"
CMDLINE_VER="13114758"

install_android_sdk() {
    local os_tag="$1"
    local sdk="${ANDROID_HOME:-${ANDROID_SDK_ROOT:-$APP_DIR/../3rdparty/android-sdk}}"
    [ -d "$sdk/platforms" ] && { echo "[setup] Android SDK 이미 있음 → $sdk"; ANDROID_HOME="$sdk"; return; }
    echo "[setup] Android SDK 설치 → $sdk"
    local zip; zip="$(mktemp -u).zip"
    curl -fsSL -o "$zip" \
        "https://dl.google.com/android/repository/commandlinetools-${os_tag}-${CMDLINE_VER}_latest.zip"
    mkdir -p "$sdk/cmdline-tools"
    if command -v unzip >/dev/null 2>&1; then
        unzip -q "$zip" -d "$sdk/cmdline-tools"
    else
        powershell -NoProfile -Command "Expand-Archive -Force '$zip' '$sdk/cmdline-tools'"
    fi
    [ -d "$sdk/cmdline-tools/latest" ] || mv "$sdk/cmdline-tools/cmdline-tools" "$sdk/cmdline-tools/latest"
    rm -f "$zip"
    local sdkm="$sdk/cmdline-tools/latest/bin/sdkmanager"
    [ "$os_tag" = win ] && sdkm="$sdkm.bat"
    yes | "$sdkm" --sdk_root="$sdk" --licenses >/dev/null
    "$sdkm" --sdk_root="$sdk" \
        "platform-tools" \
        "platforms;android-34" "platforms;android-35" "platforms;android-36" \
        "build-tools;35.0.0" "build-tools;36.0.0" \
        "cmake;3.22.1" "ndk;27.1.12297006"
    ANDROID_HOME="$sdk"
}

setup_linux() {
    sudo apt-get update -qq
    sudo apt-get install -y curl unzip ca-certificates build-essential pkg-config libssl-dev python3
    if ! command -v node >/dev/null 2>&1 || [ "$(node -v | cut -c2-3)" -lt 20 ]; then
        echo "[setup] Node.js 20 설치..."
        curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
        sudo apt-get install -y nodejs
    fi
    local jm; jm="$(java -version 2>&1 | head -1 | grep -oP 'version "\K[0-9]+' || echo 0)"
    [ "${jm:-0}" -lt 17 ] && { echo "[setup] OpenJDK 17 설치..."; sudo apt-get install -y openjdk-17-jdk-headless; }
    sudo apt-get install -y libwebkit2gtk-4.1-dev libgtk-3-dev librsvg2-dev \
        libayatana-appindicator3-dev patchelf file
    sudo apt-get install -y gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-pipewire gstreamer1.0-plugins-bad libopenh264-6
    command -v cargo >/dev/null 2>&1 || {
        echo "[setup] Rust 설치 (rustup)..."; curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    }
    install_android_sdk linux
}

setup_macos() {
    command -v brew >/dev/null 2>&1 || {
        echo "[setup] Homebrew 설치..."; NONINTERACTIVE=1 /bin/bash -c \
            "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
        eval "$(/opt/homebrew/bin/brew shellenv)"
    }
    command -v node >/dev/null 2>&1 || brew install node@20
    if [ ! -x /opt/homebrew/opt/openjdk@17/bin/java ]; then brew install openjdk@17; fi
    if [ ! -e /Library/Java/JavaVirtualMachines/openjdk-17.jdk ]; then
        sudo ln -sfn /opt/homebrew/opt/openjdk@17/libexec/openjdk.jdk \
            /Library/Java/JavaVirtualMachines/openjdk-17.jdk
    fi
    command -v cargo >/dev/null 2>&1 || curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    command -v pod  >/dev/null 2>&1 || brew install cocoapods
    install_android_sdk mac
    if ! xcode-select -p >/dev/null 2>&1 || [ ! -d /Applications/Xcode.app ]; then
        echo "[setup] ⚠ iOS 빌드하려면 App Store에서 Xcode 설치 후:"
        echo "        sudo xcode-select -s /Applications/Xcode.app/Contents/Developer"
        echo "        sudo xcodebuild -license accept"
    fi
}

setup_windows() {
    echo "[setup] ⚠ Windows 셋업은 미검증 — 실패 시 각 도구 수동 설치. Git Bash/MSYS2에서 실행 가정."
    command -v winget >/dev/null 2>&1 || { echo "[ERROR] winget 필요(App Installer)"; exit 1; }
    command -v node >/dev/null 2>&1 || winget install -e --id OpenJS.NodeJS.LTS --accept-source-agreements
    command -v java >/dev/null 2>&1 || winget install -e --id Microsoft.OpenJDK.17
    command -v cargo >/dev/null 2>&1 || winget install -e --id Rustlang.Rustup
    echo "[setup] Tauri Windows 빌드엔 'Microsoft.VisualStudio.2022.BuildTools'(C++)가 필요합니다(수동 권장)."
    install_android_sdk win
}

case "$HOST_OS" in
    Linux)                setup_linux ;;
    Darwin)               setup_macos ;;
    MINGW*|MSYS*|CYGWIN*) setup_windows ;;
    *) echo "[ERROR] 미지원 OS: $HOST_OS"; exit 1 ;;
esac

echo "✅ controller 툴체인 준비 완료 ($HOST_OS, ANDROID_HOME=${ANDROID_HOME:-미설정})"
