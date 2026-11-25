#!/bin/bash

TARGETS=(
    bin*        # bin-x86_64, bin-aarch64 (and bin/ from the older layout)
    build
    .docker
)

rm -rf "${TARGETS[@]}" 2>/dev/null || sudo rm -rf "${TARGETS[@]}"
