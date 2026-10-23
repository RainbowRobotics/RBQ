# rbq_examples

Sample apps that consume `rbq_sdk` (CycloneDDS pub/sub) from outside the robot.

## Build

```bash
bash scripts/docker/run.bash                # host native   -> bin-x86_64/
bash scripts/docker/run.bash --arch arm64   # aarch64 user PC (e.g. Jetson), via binfmt/QEMU -> bin-aarch64/
```

Both architectures are built the same way, and `bin-<arch>/` is self-contained: the libraries sit next to
the binaries. A foreign architecture is emulated, so its first build is slow (CycloneDDS and the SDK are
built from source in the image).

The humanoid example lives in its own tree, `rb_sdk/cpp/example`, and builds against `rb_sdk`.

## Deploy

`scripts/deploy.bash` sends a built `bin-<arch>/` to `~/rbq_ws` on a user PC, with the libraries it
needs. It asks the target for its own `uname -m` and picks the matching `bin-<arch>/`, so an arm64 PC
never receives x86-64 binaries.

```bash
bash scripts/deploy.bash --device rbq@192.168.0.10
```

It uses key-based ssh; `SSHPASS=<password> bash scripts/deploy.bash …` logs in with a password instead.

`rbh_low_level` takes its model and policy from `default_config.yml` next to itself, so no paths are
needed. Edit that file to switch robot or policy, or point `--config <file>` at another one.

## Run

The examples use `rbq_sdk::Publisher` / `rbq_sdk::Subscriber`, which share a
`DomainParticipant` through `rbq_sdk::ChannelFactory`. No network interface is
passed per channel.

- **Localhost (sim or same PC as the robot process):** nothing to configure.
  `ChannelFactory` lazy-inits pinned to `lo` (loopback), so two hosts on the
  same LAN running sims won't interfere with each other.

- **Cross-host (separate PC talking to the robot):** call `Init` once in
  `main()` before any `Publisher` / `Subscriber` is constructed, passing the
  NIC you want pinned:

  ```cpp
  #include <rbq_sdk/dds/ChannelFactory.hpp>

  int main() {
      rbq_sdk::ChannelFactory::Instance().Init(/*domainId=*/0, /*iface=*/"eth0");
      // ... Publisher / Subscriber construction follows
  }
  ```

  `Init` writes a CycloneDDS XML config under `$TMPDIR` (or `/tmp`) and sets
  `CYCLONEDDS_URI` if it isn't already set.

## Files

- `src/rbq_low_level.cpp` — low-level: RL control, direct joint pub/sub.
- `src/rbq_high_level.cpp` — high-level: `HighLevelCommand_` wrapper, gait IDs.
