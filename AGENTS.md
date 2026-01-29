# AGENTS.md

This repository is the `src/` folder of a ROS 2 `colcon` workspace.

- Git repo root: `/home/michael/arctos_ws/src`
- Colcon workspace root (build/test from here): `/home/michael/arctos_ws`

Packages (build types)
- `arctos_gui` (ament_python, PyQt5 + rclpy)
- `arctos_motor_driver` (ament_cmake, C++17 + rosidl services)
- `arctos_hardware_interface` (ament_cmake, C++17 ROS2 Control plugin)
- `arctos_controller` (ament_cmake, C++17 ros2_control controller plugin)
- `mks_motor_driver` (ament_cmake, C++17 low-level SocketCAN driver)
- `arctos_description`, `arctos_bringup`, `arctos_moveit_config` (ament_cmake, mostly config/launch)

Other top-level dirs
- `scripts/`: local Python tools (CAN + STM32 manual testing)
- `stm32_scripts/`: STM32-side notes/examples (not part of colcon build)
- `docs/`: architecture notes + reference PDFs

No Cursor rules found (`.cursor/rules/`, `.cursorrules`).
No Copilot instructions found (`.github/copilot-instructions.md`).

## Build

Environment setup
- Source ROS 2 first (example): `source /opt/ros/$ROS_DISTRO/setup.bash`
- From `/home/michael/arctos_ws`, source overlay after building: `source install/setup.bash`

Common builds (run from `/home/michael/arctos_ws`)
- Build everything: `colcon build --symlink-install`
- Build a single package: `colcon build --packages-select arctos_gui --symlink-install`
- Build a package + deps: `colcon build --packages-up-to arctos_bringup --symlink-install`
- Debuggable C++ build: `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`
- Export `compile_commands.json` (clangd/IDE):
  `colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`
- Rebuild only one package after interface changes:
  `colcon build --packages-select arctos_motor_driver --symlink-install --cmake-clean-cache`

Clean builds (when things get weird)
- Prefer `--cmake-clean-cache` on the affected package first.
- Full wipe (slow; rebuild everything):
  `rm -rf /home/michael/arctos_ws/build /home/michael/arctos_ws/install /home/michael/arctos_ws/log`

Common runs (after `source install/setup.bash`)
- GUI: `ros2 run arctos_gui arctos_gui`
- Bringup: `ros2 launch arctos_bringup arctos_control.launch.py` (if present in `arctos_bringup/launch/`)

Dependencies (optional)
- Install ROS deps: `rosdep install --from-paths src -i -y --rosdistro $ROS_DISTRO`

Gotchas
- `colcon build/test` must run from the workspace root (`/home/michael/arctos_ws`), not this git root.
- After building, either `source install/setup.bash` or open a new shell that has it sourced.
- If `ros2` can't find packages, confirm `echo $AMENT_PREFIX_PATH` includes `/home/michael/arctos_ws/install`.

## Test (colcon)

Run tests (run from `/home/michael/arctos_ws`)
- All packages: `colcon test`
- One package: `colcon test --packages-select arctos_gui`
- Show failures: `colcon test-result --verbose`
- Stream test output (helpful in CI): `colcon test --event-handlers console_direct+`

Run a single test (recommended patterns)
- Single pytest test function:
  `colcon test --packages-select arctos_gui --pytest-args "test/test_flake8.py::test_flake8"`
- Single pytest file:
  `colcon test --packages-select arctos_gui --pytest-args "test/test_pep257.py"`
- Single CTest/GTest by name regex (when present):
  `colcon test --packages-select arctos_controller --ctest-args -R <regex> --output-on-failure`

Direct CTest (when you are already in the build tree)
- `cd /home/michael/arctos_ws/build/<pkg> && ctest -R <regex> --output-on-failure`

Notes
- `arctos_gui/test/` contains ROS2 linter tests (ament_flake8 / ament_pep257).
- Some CMake packages declare test deps in `package.xml`, but may not currently register tests in
  `CMakeLists.txt` under `if(BUILD_TESTING)`.
  If you add tests, use the usual ament pattern:
  `find_package(ament_lint_auto REQUIRED)` + `ament_lint_auto_find_test_dependencies()`.

## Lint

Preferred (because it matches ROS2 CI expectations)
- Python flake8: `colcon test --packages-select arctos_gui --pytest-args "test/test_flake8.py::test_flake8"`
- Python docstrings (pep257): `colcon test --packages-select arctos_gui --pytest-args "test/test_pep257.py::test_pep257"`

C++ lint (only if/when enabled in a package's `CMakeLists.txt`)
- Typical target: `colcon test --packages-select <pkg> --ctest-args -R lint --output-on-failure`

Direct execution (useful for quick iteration)
- `pytest -q arctos_gui/test/test_flake8.py::test_flake8`
- `pytest -q arctos_gui/test/test_pep257.py::test_pep257`

## Code Style (follow existing code; avoid drive-by reformatting)

General
- Match the surrounding file/package style (this repo currently mixes brace/indent conventions).
- Keep changes focused; do not reformat unrelated lines just because they are nearby.
- Prefer explicit, readable code over cleverness; this is a robotics/control codebase.

Python (mainly `arctos_gui/` and `scripts/`)
- Formatting: 4 spaces; keep lines reasonably short; wrap long Qt stylesheet strings carefully.
- Docstrings: PEP257 style (module/class/function docstrings; first line is a short summary).
- Imports: standard library, third-party, then local; avoid unused imports (flake8).
- Imports (Qt): prefer importing large widgets/modules at module scope; use local imports only to break cycles.
- Typing: add type hints for public functions and cross-module APIs; use `Optional[T]` instead of `T | None` if you need older-Python compatibility.
- Naming: modules `snake_case.py`; classes `CamelCase`; functions/methods `snake_case`; constants `UPPER_SNAKE_CASE`; private members/methods prefixed with `_`.
- Error handling: catch specific exceptions where possible; log errors (ROS logger or GUI log) and keep the UI responsive.
- Qt/ROS: avoid blocking the UI thread with socket/ROS work; use `QTimer`, signals/slots, or a worker thread.
- ROS nodes: ensure clean shutdown (`rclpy.shutdown()`), and tear down sockets/threads in `closeEvent` or destructors.
- Logging: prefer `node.get_logger().info/warn/error(...)` (or a shared GUI log widget) over `print()`.

C++ (C++17; `arctos_*` + `mks_motor_driver/`)
- Standard: assume C++17 everywhere (many targets set `cxx_std_17`).
- Headers: follow local convention (`#pragma once` in `mks_motor_driver/`, include guards elsewhere).
- Includes: prefer order `#include <...>` then `#include "..."`; keep includes minimal.
- Headers: avoid `using namespace ...` in headers; prefer forward declarations when possible.
- Naming: follow the existing package API (some public APIs use `camelCase`, others `snake_case`).
- Types: prefer `std::size_t` for indices, fixed-width integers for protocol/transport (`uint8_t`, `int32_t`, ...), and `const` correctness.
- Error handling: prefer `std::optional`/status returns at boundaries that talk to hardware/transport; avoid throwing from real-time paths.
- Logging: use `RCLCPP_*` macros when a logger/node is available; avoid high-rate logging in read/write loops.
- Concurrency: guard shared state with `std::mutex` + `std::lock_guard`; ensure threads are stopped/joined in destructors.
- Warnings: `arctos_controller` uses strict warnings (`-Werror=conversion`, etc.); write explicit casts and avoid implicit narrowing.
- ROS2 control loop: avoid heap allocations, unbounded logging, and long blocking calls inside `read()` / `write()` / `update()`.
- Prefer RAII: no raw owning pointers; cleanly close sockets/threads in destructors.

ROS 2 specifics
- Prefer `colcon build --symlink-install` for iteration (Python edits picked up without reinstall).
- For interface changes (`arctos_motor_driver/srv/*.srv`), rebuild the package (and anything that depends on it).
- Plugin packages (`arctos_hardware_interface`, `arctos_controller`) must keep plugin class names and plugin XML in sync.
- Plugin XML references:
  - `arctos_hardware_interface/arctos_hardware_interface.xml`: `arctos_hardware_interface/STM32StepperInterface`
  - `arctos_controller/arctos_controller.xml`: `arctos_controller/ArctosSegmentController`

ament / CMake patterns
- Public headers live in `include/<pkg>/...` and must be installed via `install(DIRECTORY include/ ...)`.
- For new libs/executables: add `ament_target_dependencies(...)` and `install(TARGETS ...)`.
- For new ROS interfaces: add `rosidl_generate_interfaces(...)` and remember to depend on `rosidl_default_runtime`.

Hardware-facing code
- Be conservative: do not change protocol constants/byte layouts without updating docs and the STM32 side.
- Keep timeouts bounded and reconnection logic rate-limited; avoid blocking calls in hot control loops.
- Default interfaces seen in this repo: SocketCAN `can0`, STM32 endpoint `192.168.178.159:8888`.
