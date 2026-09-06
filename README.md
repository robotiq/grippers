<!-- docs-site:exclude -->
# Robotiq Grippers C++ SDK

📖 Full documentation: https://robotiq.github.io/docs/drivers/2F%20hande/SDK/C++/
<!-- /docs-site:exclude -->

A C++ SDK for controlling Robotiq adaptive grippers (2F-85 / 2F-140 / Hand-E)
over their Modbus RTU serial link.
Cross-platform: Linux, Windows, macOS — and freestanding/RTOS targets such
as STM32 microcontrollers.

<!-- docs-site:exclude -->

## Documentation

- [Environment setup](docs/01-environment-setup.md)
- [Quick start](docs/02-quick-start.md)
- [How it works](docs/03-how-it-works.md)
- [Robust example walkthrough](docs/04-robust-example-walkthrough.md)
- [Embedded / bare-metal builds](docs/05-embedded-builds.md).

<!-- /docs-site:exclude -->

## Feature requests and bug reports
Submit here:
[Robotiq/grippers/issues](https://github.com/Robotiq/grippers/issues)

## Versioning

[Semantic versioning](https://semver.org) from 1.0.0 on: patch releases fix
bugs, minor releases add API, and a breaking change to the documented API takes
a major release. The documented API is what this README and the public headers
describe — `Gripper`, the command/status blocks and the register map,
`ConnectionConfig`, `Serial`, `Platform`, `Logger`, the `toString()` free
functions, and `detail::GripperModbusClient` for the no-thread path. The
text `toString()` renders is for people, not parsers: its layout may change
in any release. Anything under
`Robotiq/detail/` that is not described here is internal and may change in any
release.

## License

BSD-3-Clause. Robotiq develops and maintains this SDK; parts of it started from
PickNik Robotics'
[ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper)
driver (BSD-3-Clause), whose original copyright notices are preserved in the
affected files, with the full history preserved in git.
