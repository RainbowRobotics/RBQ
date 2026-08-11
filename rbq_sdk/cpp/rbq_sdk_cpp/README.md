# rbq_sdk

## Building

CycloneDDS is bundled in `3rdparty/` (no system install required).

**First time** (build CycloneDDS into `3rdparty/`):

```bash
bash scripts/build_dds.bash
```

**Then** build and install:

```bash
cmake -B build
cmake --build build
sudo cmake --install build
```

Or from a build directory: `cmake .. && cmake --build . && sudo cmake --install .`

With `-DBUILD_SHARED_LIBS=OFF` (default), the build produces a **fat static library** (`librbq_sdk.a`) that includes CycloneDDS and CycloneDDS-CXX, so applications only need to link `rbq_sdk::rbq_sdk`. With `-DBUILD_SHARED_LIBS=ON` you get a shared library and must have CycloneDDS available at link time.

Optional: `scripts/build.bash` runs the DDS bootstrap and the above steps (with Ninja and install). To have CMake run the DDS bootstrap automatically when `3rdparty/` is missing, configure with `-DRBQ_SDK_BOOTSTRAP_DDS=ON`. To use a system or custom CycloneDDS instead of the bundle, use `-DRBQ_SDK_USE_BUNDLED_DDS=OFF` and set `CMAKE_PREFIX_PATH` to your CycloneDDS-CXX install.

