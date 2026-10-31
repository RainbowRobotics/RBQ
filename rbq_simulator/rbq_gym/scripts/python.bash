#!/bin/bash

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPTS_DIR/activate.bash"

echo -ne "\033]0;python $*\007"

echo "Running python script: $*"
eval "python $*"
