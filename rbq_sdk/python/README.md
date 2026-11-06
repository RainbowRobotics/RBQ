# rbq_sdk_python
Standalone Python interface for rbq_sdk

# Installation
## Dependencies
- Python 3.10 - 3.13 (Ubuntu 22.04 and 24.04)
- cyclonedds == 11.0.1 (installed by pip; on Linux arm64 pip builds it from source,
  which needs the CycloneDDS C library — `scripts/build_dds.bash` below)
- numpy
- opencv-python

### Installing from PyPI
```bash
pip install rbq_sdk
```

### Installing from source
Enter the `rbq_sdk/cpp/rbq_sdk_cpp` directory, and then install cyclonedds and rbq_sdk:
```bash
sudo bash scripts/build_dds.bash
sudo bash scripts/build.bash
```
Enter the `rbq_sdk/python` directory, and then install the Python SDK.
```bash
sudo apt install python3-pip
pip3 install -e .
```

# Usage
The Python interface mirrors the rbq_sdk interface — robot status and control via
request/response or topic subscription/publishing.

> Example programs live in the source repository under `rbq_sdk/python/example/`.
> They are **not** included in the pip-installed package; clone the repo to run them.
> Replace `<iface>` below with the network interface connected to the robot (e.g. `eth0`).

## High-level control (`example/sport/`)
Publish `HighLevelCommand` (gait + velocity) to drive the robot:
```bash
python3 example/sport/rbq_high_level.py <iface>
```
Other sport examples: `rbq_low_level.py`, `rbq_sport_client.py`, `rbq_sport_client_joy.py`,
`rbq_wheel.py` (wheel robot, RBQ-W).

## Front camera (`example/front_camera/`)
Show the front camera with OpenCV (needs a graphical session; press ESC to exit). The examples subscribe to
the robot's camera topic `rt/rbq/vision/sensor_4/rgb/compressed`, so the robot's vision must be running:
```bash
python3 example/front_camera/camera_opencv.py <iface> [topic]
```
Also available: `capture_image.py` (saves one frame to `img.jpg`). Pass another `.../compressed` topic to
pick another camera.
