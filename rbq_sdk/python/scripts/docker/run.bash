#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"                                   # rbq_sdk/python
DDS_SCRIPT="$(cd "$SCRIPT_DIR/../../../cpp/rbq_sdk_cpp/scripts" && pwd)/build_dds.bash"

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
    echo "Build + install the rbq_sdk_py package inside an ubuntu:22.04 container"
    echo "(builds CycloneDDS, pip-installs the SDK, then import-tests it)."
    echo "A failed build/install/import fails this script."
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

echo "Building rbq_sdk/python Docker image..."
# build_dds.bash lives in the cpp SDK; stage it into the build context.
mkdir -p "$PROJECT_DIR/temp" && cp "$DDS_SCRIPT" "$PROJECT_DIR/temp/build_dds.bash"
set +e
$DOCKER build "${NO_CACHE_ARG[@]}" --file "$SCRIPT_DIR/Dockerfile" --network host -t "$DOCKER_NAME" "$PROJECT_DIR"
STATUS=$?
set -e
rm -rf "$PROJECT_DIR/temp"
if [[ $STATUS -eq 0 ]]; then
    echo "✅ rbq_sdk/python build + import succeeded."
else
    echo "❌ rbq_sdk/python build failed (exit $STATUS)."
fi
exit $STATUS
