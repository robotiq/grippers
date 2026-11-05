# Robotiq Grippers C++ SDK has moved

**This repository is now the [`grippers/`](https://github.com/robotiq/robotiq-sdk/tree/main/grippers) folder of [robotiq/robotiq-sdk](https://github.com/robotiq/robotiq-sdk)**, the public repository for every Robotiq SDK. The full history came along, and all new work, releases and issues happen there.

This repository no longer changes.

## What still works

- **Pinned submodules and tags** of this repository (v1.0.0 to v1.3.0) stay fetchable here.
- **Links to past issues and pull requests** keep working; the commit messages in robotiq-sdk link back to them.

## Moving to robotiq-sdk

- **Docs:** [grippers/README.md](https://github.com/robotiq/robotiq-sdk/blob/main/grippers/README.md) and [grippers/docs/](https://github.com/robotiq/robotiq-sdk/tree/main/grippers/docs).
- **As a submodule:** `git submodule add https://github.com/robotiq/robotiq-sdk third_party/robotiq-sdk`, pin a release with `git -C third_party/robotiq-sdk checkout v3.0.0`, then `add_subdirectory(third_party/robotiq-sdk/grippers/sdk_cpp)`.
- **Versions:** robotiq-sdk has one version for every SDK, starting at 3.0.0. The grippers code in 3.0.0 is the same as in 1.3.0, with no API change. The CMake package keeps `SameMajorVersion`, so only `find_package(grippers 1)` becomes `find_package(grippers 3)`.
- **Issues and feature requests:** open them in [robotiq/robotiq-sdk](https://github.com/robotiq/robotiq-sdk/issues).
