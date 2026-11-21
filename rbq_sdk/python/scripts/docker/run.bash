#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"                                   # rbq_sdk/python

RAW_BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
SANITIZED_BRANCH_NAME=$(echo "$RAW_BRANCH_NAME" | sed 's#[/_]#-#g' | tr '[:upper:]' '[:lower:]')
DOCKER_NAME="rbq-sdk-py-${SANITIZED_BRANCH_NAME}"

if docker info &>/dev/null; then
    DOCKER="docker"
else
    DOCKER="sudo docker"
fi

print_help() {
    echo "Usage: bash scripts/docker/run.bash [OPTIONS]"
    echo ""
    echo "pip-install the rbq_sdk_py package in ubuntu:22.04 (Python 3.10) and"
    echo "ubuntu:24.04 (Python 3.12) containers, then import it and start DDS."
    echo "A failed install/import on either fails this script."
    echo ""
    echo "Options:"
    echo "  --help        Display this help message and exit."
    echo "  --no-cache    Build the Docker image without cache."
}

NO_CACHE_ARG=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --help) print_help; exit 0 ;;
        --no-cache) NO_CACHE_ARG=(--no-cache); shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

# Both LTS releases customers run: 22.04 = Python 3.10, 24.04 = Python 3.12.
STATUS=0
for UBUNTU in 22.04 24.04; do
    echo "Building rbq_sdk/python Docker image (ubuntu:$UBUNTU)..."
    if $DOCKER build "${NO_CACHE_ARG[@]}" --build-arg UBUNTU_VERSION="$UBUNTU" --file "$SCRIPT_DIR/Dockerfile" \
        --network host -t "$DOCKER_NAME-$UBUNTU" "$PROJECT_DIR"; then
        echo "✅ rbq_sdk/python ubuntu:$UBUNTU install + import succeeded."
    else
        STATUS=1
        echo "❌ rbq_sdk/python ubuntu:$UBUNTU failed."
    fi
done
exit $STATUS
