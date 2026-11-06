#!/usr/bin/env bash
set -e
export DEBIAN_FRONTEND=noninteractive

apt-get update

apt-get install -y build-essential pkg-config libssl-dev python3

curl -fsSL https://deb.nodesource.com/setup_20.x | bash -
apt-get install -y nodejs unzip

apt-get install -y openjdk-17-jdk-headless


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
