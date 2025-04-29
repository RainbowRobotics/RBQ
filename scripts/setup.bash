#!/usr/bin/env bash
set -e

# Exit if executed with sudo
if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
fi

sudo bash scripts/install/apt.bash
#sudo locale-gen en_US.UTF-8 && sudo dpkg-reconfigure locales

# cmake
bash scripts/install/cmake.bash

# Eigen
bash scripts/install/eigen.bash

# JSON
bash scripts/install/json.bash

# Qt
bash scripts/install/download_qt.bash
bash scripts/install/qt.bash

# choreonoid
bash scripts/install/choreonoid.bash

