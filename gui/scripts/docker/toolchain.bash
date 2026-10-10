#!/usr/bin/env bash
set -e
export DEBIAN_FRONTEND=noninteractive

apt-get update

apt-get install -y build-essential pkg-config libssl-dev python3

curl -fsSL https://deb.nodesource.com/setup_20.x | bash -
apt-get install -y nodejs unzip

apt-get install -y openjdk-17-jdk-headless

apt-get install -y gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-pipewire gstreamer1.0-plugins-bad libopenh264-6
apt-get install -y libwebkit2gtk-4.1-dev libgtk-3-dev librsvg2-dev \
    libayatana-appindicator3-dev patchelf file desktop-file-utils

LD_TAG="1-alpha-20251107-1"
LD_SHA="c20cd71e3a4e3b80c3483cef793cda3f4e990aca14014d23c544ca3ce1270b4d"
LD_DIR=/opt/linuxdeploy
mkdir -p "$LD_DIR"
curl -fsSL -o "$LD_DIR/linuxdeploy-x86_64.AppImage" \
    "https://github.com/linuxdeploy/linuxdeploy/releases/download/${LD_TAG}/linuxdeploy-x86_64.AppImage"
echo "${LD_SHA}  ${LD_DIR}/linuxdeploy-x86_64.AppImage" | sha256sum -c - \
    || { echo "FAIL: linuxdeploy 해시 불일치 — 공급망 확인 필요" >&2; exit 1; }
chmod +x "$LD_DIR/linuxdeploy-x86_64.AppImage"

LD_PA_TAG="1-alpha-20250213-1"
LD_PA_SHA="992d502a248e14ab185448ddf6f6e7d25558cb84d4623c354c3af350c25fccb3"
LD_GTK_REF="7a3fbc31a9e5075073ff8790f26effbac5f84453"
LD_GTK_SHA="b0f4cbc684a0103a9651f0955b635eaea0096b3a66c0f5a2c2aa337960375171"
LD_GST_REF="2a2e67491c32995a3f279ad0ecbe77abd512b42a"
LD_GST_SHA="c107b49d84edbffc6ab226ed1007e0626a4f7aa2c3a36b7782bef62351d49e94"

curl -fsSL -o "$LD_DIR/linuxdeploy-plugin-appimage-x86_64.AppImage" \
    "https://github.com/linuxdeploy/linuxdeploy-plugin-appimage/releases/download/${LD_PA_TAG}/linuxdeploy-plugin-appimage-x86_64.AppImage"
curl -fsSL -o "$LD_DIR/linuxdeploy-plugin-gtk.sh" \
    "https://raw.githubusercontent.com/linuxdeploy/linuxdeploy-plugin-gtk/${LD_GTK_REF}/linuxdeploy-plugin-gtk.sh"
curl -fsSL -o "$LD_DIR/linuxdeploy-plugin-gstreamer.sh" \
    "https://raw.githubusercontent.com/linuxdeploy/linuxdeploy-plugin-gstreamer/${LD_GST_REF}/linuxdeploy-plugin-gstreamer.sh"
printf '%s  %s\n%s  %s\n%s  %s\n' \
    "$LD_PA_SHA"  "$LD_DIR/linuxdeploy-plugin-appimage-x86_64.AppImage" \
    "$LD_GTK_SHA" "$LD_DIR/linuxdeploy-plugin-gtk.sh" \
    "$LD_GST_SHA" "$LD_DIR/linuxdeploy-plugin-gstreamer.sh" | sha256sum -c - \
    || { echo "FAIL: linuxdeploy 플러그인 해시 불일치 — 공급망 확인 필요" >&2; exit 1; }
chmod +x "$LD_DIR/linuxdeploy-plugin-appimage-x86_64.AppImage" "$LD_DIR"/linuxdeploy-plugin-*.sh

export RUSTUP_HOME=/usr/local/rustup
export CARGO_HOME=/usr/local/cargo
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --no-modify-path
chmod -R a+rX /usr/local/rustup /usr/local/cargo

SDK=/opt/android-sdk
CMDLINE_VER="13114758"
curl -fsSL -o /tmp/cmdline-tools.zip \
    "https://dl.google.com/android/repository/commandlinetools-linux-${CMDLINE_VER}_latest.zip"
mkdir -p "$SDK/cmdline-tools"
unzip -q /tmp/cmdline-tools.zip -d "$SDK/cmdline-tools"
mv "$SDK/cmdline-tools/cmdline-tools" "$SDK/cmdline-tools/latest"
rm -f /tmp/cmdline-tools.zip
yes | "$SDK/cmdline-tools/latest/bin/sdkmanager" --sdk_root="$SDK" --licenses >/dev/null
"$SDK/cmdline-tools/latest/bin/sdkmanager" --sdk_root="$SDK" \
    "platform-tools" \
    "platforms;android-34" "platforms;android-35" "platforms;android-36" \
    "build-tools;35.0.0" "build-tools;36.0.0" \
    "cmake;3.22.1" \
    "ndk;27.1.12297006"
chmod -R a+rX "$SDK"

echo "✅ controller 도커 툴체인 설치 완료"
