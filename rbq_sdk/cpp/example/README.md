# rbq_examples

Sample apps that consume `rbq_sdk` (CycloneDDS pub/sub) from outside the robot.

## Build

```bash
bash scripts/docker/run.bash
```

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
