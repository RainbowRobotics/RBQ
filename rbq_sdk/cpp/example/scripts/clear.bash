#!/bin/bash

TARGETS=(
    bin
    build
    .docker
)

rm -rf "${TARGETS[@]}" 2>/dev/null || sudo rm -rf "${TARGETS[@]}"
