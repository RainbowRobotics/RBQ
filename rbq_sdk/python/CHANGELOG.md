# Changelog

All notable changes to `rbq_sdk_py` are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
Version numbers track the framework release (`src/rcl/version.txt`), not an
independent SemVer sequence for the SDK.

The SDK is published to PyPI as part of the framework's public release
(`scripts/release.bash`, Step 9), at the same version as the framework
(`src/rcl/version.txt`). PyPI is immutable: every release needs a new version.

## [Unreleased]

### Changed

- PyPI distribution renamed `rbq_sdk_py` → `rbq_sdk` (`pip install rbq_sdk`).
  The import name is unchanged (`import rbq_sdk_py`).
- PyPI publish moved from a standalone GitHub Actions job (OIDC Trusted
  Publishing) into `rbq_sdk/python/scripts/release.bash`, invoked by
  `scripts/release.bash` Step 9 alongside the C++ PPA publish; uploads now use
  a stored PyPI API token (`PYPI_TOKEN`).

## [1.0.1] - 2026-06-26

### Added

- PEP 517/621 packaging with the `hatchling` backend.
- Version single-sourced from `rbq_sdk_py-v*` git tags via `hatch-vcs`.
- Automated release to PyPI through GitHub Actions + Trusted Publishing (OIDC).
