# rbq_sdk

## Installing (Ubuntu, prebuilt)

The packaged SDK is published to the Launchpad PPA **`ppa:rainbow-robotics/rbq-sdk`** — this is
the only PPA address; if a machine has any other `rainbow-robotics*` PPA entry for rbq-sdk in
`/etc/apt/sources.list.d/`, remove it and add this one.

```bash
sudo add-apt-repository -y ppa:rainbow-robotics/rbq-sdk
sudo apt-get update && sudo apt-get install -y rbq-sdk
```

## Building

CycloneDDS is not bundled — CMake finds CycloneDDS and CycloneDDS-CXX installed on the system.

**First time** (download, build and install CycloneDDS 0.10.2 system-wide):

```bash
sudo bash scripts/build_dds.bash
```

**Then** build and install:

```bash
cmake -B build
cmake --build build
sudo cmake --install build
```

Or from a build directory: `cmake .. && cmake --build . && sudo cmake --install .`

The build produces a shared library (`librbq_sdk.so`); applications link `rbq_sdk::rbq_sdk` and need
CycloneDDS installed at run time.

Optional: `sudo bash scripts/build.bash` runs the configure, build and install steps above (with Ninja). It
does not install CycloneDDS — run `build_dds.bash` first.

