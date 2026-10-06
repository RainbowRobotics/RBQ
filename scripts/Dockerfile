FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# What the released binaries link against and do not carry in bin-<arch>/ themselves.
# Measured with ldd over every binary in a release tree on a bare 22.04 image (2026-09-29, 96 libraries).
# libusb-1.0-0-dev: a vendored library links the unversioned libusb-1.0.so, which only -dev ships.
# Commands the apps run rather than link (QProcess/popen in src/):
#   iputils-ping  Streamer checks the motion/vision PCs with `ping` (src/Streamer/Network.cpp)
#   ffmpeg        Streamer pushes the simulated cameras to mediamtx through ffmpeg - without it
#                 there is no video and no error either
#   lsb-release   Network reports the OS version with lsb_release
# nmcli is left out: it manages the robot's Wi-Fi, which a simulation has none of.
# libgl1-mesa-dri: Mesa's GL drivers are dlopen'ed (software GL, /dev/dri), so ldd never lists them;
# today they arrive as a dependency - named here so that cannot silently change.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libasound2 \
    libavcodec58 \
    libavutil56 \
    libboost-timer1.74.0 \
    libcurl4 \
    libdrm2 \
    libdw1 \
    libegl1 \
    libfmt8 \
    libfontconfig1 \
    libfreetype6 \
    libgbm1 \
    libgeotiff5 \
    libgl1 \
    libglfw3 \
    libglib2.0-0 \
    libgomp1 \
    libgstreamer-gl1.0-0 \
    libgstreamer-plugins-base1.0-0 \
    libgstreamer-plugins-good1.0-0 \
    libgstreamer1.0-0 \
    libice6 \
    libicu70 \
    libjsoncpp25 \
    libopencv-calib3d4.5d \
    libopencv-contrib4.5d \
    libopencv-core4.5d \
    libopencv-dnn4.5d \
    libopencv-highgui4.5d \
    libopencv-imgcodecs4.5d \
    libopencv-imgproc4.5d \
    libopencv-videoio4.5d \
    libopus0 \
    libpcl-common1.12 \
    libpcl-io1.12 \
    libpcl-visualization1.12 \
    libproj22 \
    libpulse0 \
    libqt5core5a \
    libqt5gui5 \
    libqt5network5 \
    libqt5widgets5 \
    libsdl2-2.0-0 \
    libsm6 \
    libspdlog1 \
    libswscale5 \
    libtbb12 \
    libtbbmalloc2 \
    libunwind8 \
    libusb-1.0-0 \
    libvtk9.1 \
    libvtk9.1-qt \
    libwayland-client0 \
    libwayland-cursor0 \
    libwayland-egl1 \
    libwebp7 \
    libwebpdemux2 \
    libwebpmux3 \
    libx11-6 \
    libx11-xcb1 \
    libxcb-glx0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-render0 \
    libxcb-shape0 \
    libxcb-shm0 \
    libxcb-sync1 \
    libxcb-xfixes0 \
    libxcb-xinerama0 \
    libxcb-xinput0 \
    libxcb-xkb1 \
    libxcb1 \
    libxcomposite1 \
    libxkbcommon-x11-0 \
    libxkbcommon0 \
    libxml2 \
    libusb-1.0-0-dev \
    iputils-ping \
    libgl1-mesa-dri \
    ffmpeg \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /rbq_ws
