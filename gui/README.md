# RBQ Controller

The operator app for the Rainbow Robotics **RBQ** quadruped — driving, cameras, 3D pose, maintenance,
logs and firmware status — built once with React Native + Expo and shipped to Android, iOS, the web,
Linux / Windows / macOS desktops (Tauri) and the Steam Deck.

This folder is the open-source release of the app that ships with the robot. The robot's onboard
software and Rainbow Robotics' cloud services are not part of it; the app talks to the robot directly
over your network.

- License: Apache-2.0 (see `LICENSE`, third-party notices in `NOTICE`). The "Rainbow Robotics" and
  "RBQ" names and logos are trademarks and are not licensed for use in your own distributions.
- **Pull requests are not accepted.** This tree is regenerated from the internal repository on every
  release. Bug reports and questions are welcome as GitHub Issues.
- Security issues: see `SECURITY.md` — please do not open a public issue.

## Quick start

Optional: `cp .env.example .env` and fill what you need — every value is optional and an empty one
hides the feature that needs it.

Everything builds in Docker, so the only host requirement is Docker:

```bash
bash scripts/docker/run.bash                 # image + web, Android APK and Linux AppImage/deb → build-out/
bash scripts/docker/run.bash --build --web   # just the web build
```

Or install the toolchain on the host (Node, JDK 17, Rust, Android SDK, webkit2gtk — detected per OS):

```bash
bash setup.bash
bash build.bash            # default targets for this OS
bash dev.bash --web        # fast desktop dev loop
npm run web                # Metro + browser
```

iOS and macOS builds need a Mac with Xcode; Windows installers need a Windows host.

### Windows

- **Web and Android builds**: install Docker Desktop (WSL 2 backend) and run `bash scripts/docker/run.bash`
  from Git Bash or a WSL shell — the build runs in a Linux container, exactly as on Linux.
- **Windows desktop app (`.exe`)**: Docker can't produce it. Build on Windows itself from Git Bash
  (`bash setup.bash`, then `bash build.bash`). The Windows path of `setup.bash` is not yet verified by us —
  report problems as an Issue.
- **Simulator**: the robot daemons and MuJoCo simulator in this repository are Ubuntu 22.04 binaries. On Windows,
  run them inside WSL 2 (Ubuntu); connecting the Windows app to a simulator inside WSL has not been verified yet.
  Without a robot you can always use the web build's demo mode (`?demo=1`).

## Connecting to a robot

1. Join the robot's Wi-Fi (or a network the robot is on).
2. Add the robot in the app. The robot is found at its network address and identified by its serial.
3. The robot asks for its **API password** the first time; the app stores it per robot on this device
   (in the app's ordinary storage, not a keychain).

### No robot? Use the simulator

The repository root contains the MuJoCo simulator and the robot daemons. From the root:

```bash
bash scripts/sim.bash
```

Then add a robot at `127.0.0.1`. The web build can also run without any robot in demo mode —
open it with `?demo=1`.

## Making it your own

| What | Where |
|---|---|
| App name, bundle id, URL scheme, icons, splash | `app.json` (`name`, `ios.bundleIdentifier`, `android.package`, `scheme`, `icon`) |
| Nightly variant id / icon | `app.config.js` |
| Desktop name, id, icons | `src-tauri/tauri.conf.json` (`productName`, `identifier`, `bundle.icon`) |
| Data folder of the desktop dev loop | `dev.bash` (hard-codes the desktop `identifier` path — change it with the id) |
| Robot defaults, servers, access codes | `.env` (see `.env.example`) |

**Set your own access codes.** Levels 2/3 unlock developer and maintenance screens. Without
`EXPO_PUBLIC_LV2_SHA256` / `EXPO_PUBLIC_LV3_SHA256` in `.env` the code is `0000` for both (it opens level 3),
so anyone with the app can open them. Put the SHA-256 of your codes there (`echo -n '<code>' | sha256sum`).

`app.json` enables push notifications (`aps-environment`); remove `expo-notifications` from it if your
signing profile has no push capability.

## Not included

- Remote (LTE / relay) connections, accounts, push notifications and log upload use Rainbow Robotics'
  platform servers and are hidden in this build. The relevant settings are environment variables
  (`EXPO_PUBLIC_*`, see `.env.example`) if you run your own.
- Some product-specific features are not included.
- `src/rb/` (the Rainbow Robotics design system) ships compiled, with type declarations.

## For developers and AI coding agents

Read `AGENTS.md` first — structure, conventions and the traps that are not visible from the types.
