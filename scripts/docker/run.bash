#!/usr/bin/env bash
set -e

# Exit if executed with sudo
if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
fi

# Dynamically set Docker image name based on Git branch
RAW_BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
SANITIZED_BRANCH_NAME=$(echo "$RAW_BRANCH_NAME" | sed 's#[/_]#-#g' | tr '[:upper:]' '[:lower:]')
IMAGE_NAME="rbq-public-${SANITIZED_BRANCH_NAME}"
DOCKER_DIR=".docker"
BIN_DIR="bin"
LIB_DIR="lib"
NO_CACHE=false
NO_CHECK=false

# Cleanup function
cleanup_on_failure() {
    if [[ $? -ne 0 ]]; then
        echo "Build failed. Cleaning up $DOCKER_DIR..."
        sudo rm -rf "$DOCKER_DIR"
    fi
}
trap cleanup_on_failure EXIT

# Parse command-line arguments
for arg in "$@"; do
    case $arg in
        --help) print_help; exit 0 ;;
        --no-cache) NO_CACHE=true; shift ;;
        --no-check) NO_CHECK=true; shift ;;
        *)
            echo "Unknown argument: $arg"
            exit 1
            ;;
    esac
done

# If --no-cache is set, remove DOCKER_DIR to ensure a clean build
if [[ "$NO_CACHE" == "true" ]]; then
    echo "No-cache option enabled. Removing $DOCKER_DIR..."
    sudo rm -rf "$DOCKER_DIR"
fi

# Build Docker image
if [[ "$NO_CHECK" == "false" ]]; then
    sudo snap install docker
    echo "Docker container build starting..."
    sudo docker build --file ./scripts/docker/Dockerfile -t $IMAGE_NAME .
fi

# Run the Docker container
sudo docker run \
    --rm \
    --cap-add SYS_ADMIN \
    --device /dev/fuse \
    --security-opt apparmor:unconfined \
    -v ${PWD}/src:/workspace/src \
    -v ${PWD}/$DOCKER_DIR/build:/workspace/build \
    -v ${PWD}/$DOCKER_DIR/$BIN_DIR:/workspace/$BIN_DIR \
    -v ${PWD}/$DOCKER_DIR/$LIB_DIR:/workspace/$LIB_DIR \
    -v ${PWD}/CMakeLists.txt:/workspace/CMakeLists.txt \
    $IMAGE_NAME bash -c "bash scripts/__build__.bash --examples"

# If we reached here, container ran successfully — disable trap
trap - EXIT

echo "Docker container executed successfully."

sudo chown -R $(logname):$(logname) $DOCKER_DIR

echo "Copying binaries $DOCKER_DIR/bin to $BIN_DIR ..."
cp -r $DOCKER_DIR/$BIN_DIR/* $BIN_DIR/
echo "Copy operation completed successfully."

