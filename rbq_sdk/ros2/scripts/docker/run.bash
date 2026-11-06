#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

RAW_BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
SANITIZED_BRANCH_NAME=$(echo "$RAW_BRANCH_NAME" | sed 's#[/_]#-#g' | tr '[:upper:]' '[:lower:]')
DOCKER_NAME="rbq-ros2-${SANITIZED_BRANCH_NAME}"

if docker info &>/dev/null; then
    DOCKER="docker"
else
    DOCKER="sudo docker"
fi

print_help() {
    echo "Usage: bash scripts/docker/run.bash [OPTIONS]"
    echo ""
    echo "Build the rbq_sdk/ros2 colcon workspace inside a ros:humble container"
    echo "(rosdep install + colcon build). A build failure fails this script."
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

echo "Building rbq_sdk/ros2 (colcon) Docker image..."
$DOCKER build "${NO_CACHE_ARG[@]}" --file "$SCRIPT_DIR/Dockerfile" --network host -t "$DOCKER_NAME" "$PROJECT_DIR"
echo "✅ ros2 colcon build succeeded."
