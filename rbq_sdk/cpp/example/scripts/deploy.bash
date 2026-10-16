#!/usr/bin/env bash
# Sends a built bin-<arch>/ to a user PC, into ~/rbq_ws.
# The architecture is taken from the target itself (ssh uname -m), so a
# cross-built bin-aarch64/ goes to an arm64 PC and never to an x86-64 one.
set -e

REMOTE_DEVICE=""
SOURCE_DIR="$PWD"

print_help() {
    echo "Usage: bash scripts/deploy.bash --device USER@IP [OPTIONS]"
    echo "Options:"
    echo "  --help             Display this help message and exit."
    echo "  --device [USER@IP] Target PC (required)."
    echo ""
    echo "Uses key-based ssh; set SSHPASS=<password> for password login."
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --help)   print_help; exit 0 ;;
        --device) REMOTE_DEVICE="${2:?--device requires USER@IP}"; shift 2 ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

if [[ -z "$REMOTE_DEVICE" ]]; then
    echo "[ERROR] --device USER@IP is required."
    print_help
    exit 1
fi
# Key-based ssh by default; set SSHPASS=<password> to use a password instead.
SSH="ssh"
if [[ -n "${SSHPASS:-}" ]] && command -v sshpass >/dev/null; then
    SSH="sshpass -e ssh"
fi

echo "[INFO] Asking $REMOTE_DEVICE for its architecture..."
# The pipe would swallow ssh's exit status, so judge by what came back.
TARGET_ARCH=$($SSH "$REMOTE_DEVICE" 'uname -m' | tr -d '\r') || true
if [[ -z "$TARGET_ARCH" ]]; then
    echo "[ERROR] could not determine $REMOTE_DEVICE's architecture (ssh uname -m failed)."
    exit 1
fi
BIN_DIR="$SOURCE_DIR/bin-$TARGET_ARCH"
if [[ ! -d "$BIN_DIR" ]]; then
    echo "[ERROR] $BIN_DIR not found. Build it first:"
    echo "        bash scripts/docker/run.bash$([[ "$TARGET_ARCH" == aarch64 ]] && echo ' --arch arm64')"
    exit 1
fi
echo "[INFO] $REMOTE_DEVICE is $TARGET_ARCH -> $BIN_DIR"

# The libraries sit next to the binaries ($ORIGIN rpath), so every deploy carries them.
LIBS=("$BIN_DIR"/lib*.so*)

send() {  # send <remote dir> <file...>
    local dest="$1"; shift
    echo "[INFO] $# file(s) -> $REMOTE_DEVICE:$dest/"
    rsync -az --progress -e "$SSH" --rsync-path="mkdir -p $dest && rsync" "$@" "$REMOTE_DEVICE:$dest/"
}

RBQ=("$BIN_DIR"/rbq_*)
if [[ ! -e "${RBQ[0]}" ]]; then
    echo "[ERROR] no rbq_* binaries in $BIN_DIR - build it first (bash scripts/docker/run.bash)."
    exit 1
fi
send '~/rbq_ws' "${RBQ[@]}" "${LIBS[@]}"
echo "✅ Deployed to ~/rbq_ws."
