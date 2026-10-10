#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
RES_DIR=""
if [ -d "$PROJECT_DIR/../resources" ]; then
    RES_DIR="$(cd "$PROJECT_DIR/../resources" && pwd)"
else
    echo "[gui-docker] ⚠resources/ 없음 — 시뮬 자산 생성(npm run mujoco:sync)이 실패해 빌드가 멈춥니다." >&2
fi

RAW_BRANCH_NAME=$(git -C "$PROJECT_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
SANITIZED_BRANCH_NAME=$(echo "$RAW_BRANCH_NAME" | sed 's#[/_]#-#g' | tr '[:upper:]' '[:lower:]')
IMAGE_NAME="rbq-gui-${SANITIZED_BRANCH_NAME}"

if docker info &>/dev/null; then DOCKER="docker"; else DOCKER="sudo docker"; fi

NO_CACHE_ARG=()
BUILD_ARGS=()
DO_SETUP=false
DO_BUILD=false
DO_CHECK=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --setup) DO_SETUP=true; shift ;;
        --build) DO_BUILD=true; shift ;;
        --check) DO_CHECK=true; shift ;;
        --no-cache) NO_CACHE_ARG=(--no-cache); shift ;;
        --web|--apk|--linux|--desktop|--all) BUILD_ARGS+=("$1"); shift ;;
        --help) cat <<'USAGE'
Usage: bash scripts/docker/run.bash [stage] [targets...] [--no-cache]
  stage (default: --setup then --build)
    --setup   build the Docker image only
    --build   build inside the container (runs --setup first if the image is missing)
    --check   npm ci + tsc --noEmit + vitest run inside the container
  targets: --web --apk --linux --desktop --all (default: everything for Linux)
  output:  $CONTROLLER_BIN_DIR (default gui/build-out/)
USAGE
            exit 0 ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
done
if ! $DO_SETUP && ! $DO_BUILD && ! $DO_CHECK; then DO_SETUP=true; DO_BUILD=true; fi

PLATFORM_ARG=(--platform linux/amd64)

if { $DO_BUILD || $DO_CHECK; } && ! $DO_SETUP && ! $DOCKER image inspect "$IMAGE_NAME" &>/dev/null; then
    echo "[gui-docker] 이미지($IMAGE_NAME) 없음 — --setup 자동 선행"
    DO_SETUP=true
fi

if $DO_SETUP; then
    echo "[gui-docker] 이미지 빌드: $IMAGE_NAME"
    $DOCKER build "${NO_CACHE_ARG[@]}" "${PLATFORM_ARG[@]}" --file "$SCRIPT_DIR/Dockerfile" --network host -t "$IMAGE_NAME" "$PROJECT_DIR"
fi

run_in_image() {
    mkdir -p "$PROJECT_DIR/.docker/home"
    [ -n "${CONTROLLER_BIN_DIR:-}" ] && mkdir -p "$CONTROLLER_BIN_DIR"
    $DOCKER run --rm \
        "${PLATFORM_ARG[@]}" \
        --user "$(id -u):$(id -g)" \
        --network host \
        -e RBQ_VERSION \
        -e RBQ_CHANNEL \
        -e GRADLE_OPTS="-Dorg.gradle.jvmargs=-Xmx3g -Dorg.gradle.workers.max=2" \
        -v "$PROJECT_DIR":/workspace/gui \
        -v "$PROJECT_DIR/.docker":/workspace/cache \
        ${RES_DIR:+-v "$RES_DIR":/workspace/resources:ro} \
        ${CONTROLLER_BIN_DIR:+-e CONTROLLER_BIN_DIR=/workspace/bin-out} \
        ${CONTROLLER_BIN_DIR:+-v "$CONTROLLER_BIN_DIR":/workspace/bin-out} \
        "$IMAGE_NAME" bash -c "cd /workspace/gui && $1"
}
if $DO_CHECK; then
    echo "[gui-docker] check: npm ci + tsc + vitest"
    run_in_image "npm ci --no-audit --no-fund && npx tsc --noEmit && npx vitest run"
    echo "✅ gui check 통과"
fi
if $DO_BUILD; then
    RBQ_VERSION="${RBQ_VERSION:-$(git -C "$PROJECT_DIR" describe --tags --dirty --always 2>/dev/null || true)}"
    export RBQ_VERSION

    echo "[gui-docker] 컨테이너 빌드: build.bash ${BUILD_ARGS[*]:-(무인자 — 컨테이너 OS 자동)}"
    run_in_image "bash build.bash ${BUILD_ARGS[*]:-}"

    echo "✅ gui 도커 빌드 완료 — 산출물: ${CONTROLLER_BIN_DIR:-$PROJECT_DIR/build-out}/"
elif ! $DO_CHECK; then
    echo "✅ gui 도커 이미지 준비 완료: $IMAGE_NAME (빌드는 --build)"
fi
