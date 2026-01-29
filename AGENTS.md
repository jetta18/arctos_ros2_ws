# AGENTS.md

This repository is the `src/` folder of a ROS 2 `colcon` workspace.

- Git repo root: `/home/michael/arctos_ws/src`
- Colcon workspace root (build/test from here): `/home/michael/arctos_ws`

Packages in this repo (build types)
- `arctos_gui` (ament_python, PyQt5 + rclpy)
- `arctos_controller` (ament_cmake, C++17 ros2_control controller plugin)
- `arctos_hardware_interface` (ament_cmake, C++17 ros2_control hardware plugin)
- `mks_motor_driver` (ament_cmake, C++17 low-level SocketCAN driver)
- `arctos_description`, `arctos_bringup`, `arctos_moveit_config` (ament_cmake; URDF/config/launch; MoveIt config is mostly auto-generated)

Other top-level dirs
- `scripts/`: local Python tools (CAN + STM32 manual testing)
- `stm32_scripts/`: STM32-side notes/examples (not part of colcon build)
- `docs/`: architecture notes + reference PDFs

Cursor/Copilot rules
- None found (`.cursor/rules/`, `.cursorrules`, `.github/copilot-instructions.md`).

## Build

Environment setup
- Source ROS 2 first (example): `source /opt/ros/$ROS_DISTRO/setup.bash`
- From `/home/michael/arctos_ws`, source overlay after building: `source install/setup.bash`

Common builds (run from `/home/michael/arctos_ws`)
- Build everything: `colcon build --symlink-install`
- Build a single package: `colcon build --packages-select arctos_gui --symlink-install`
- Build a package + deps: `colcon build --packages-up-to arctos_bringup --symlink-install`
- Debuggable C++ build: `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`
- Export `compile_commands.json` (clangd/IDE): `colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

Clean builds (when things get weird)
- Prefer `--cmake-clean-cache` first: `colcon build --packages-select <pkg> --symlink-install --cmake-clean-cache`
- Full wipe (slow; rebuild everything): `rm -rf /home/michael/arctos_ws/build /home/michael/arctos_ws/install /home/michael/arctos_ws/log`

Common runs (after `source install/setup.bash`)
- GUI: `ros2 run arctos_gui arctos_gui`
- Bringup: `ros2 launch arctos_bringup arctos_bringup.launch.py` (or `demo.launch.py`)

Dependencies (optional)
- Install ROS deps: `rosdep install --from-paths src -i -y --rosdistro $ROS_DISTRO`

Gotchas
- `colcon build/test` must run from the workspace root (`/home/michael/arctos_ws`), not this git root.
- If `ros2` can't find packages, confirm `echo $AMENT_PREFIX_PATH` includes `/home/michael/arctos_ws/install`.
- `arctos_gui` talks to MKS servos via direct CAN (`python-can` + `mks_servo_can`); ensure those Python deps are installed.

## Test (colcon)

Run tests (run from `/home/michael/arctos_ws`)
- All packages: `colcon test --event-handlers console_direct+`
- One package: `colcon test --packages-select arctos_gui`
- Show failures: `colcon test-result --verbose`

Run a single test (recommended patterns)
- Single pytest test function: `colcon test --packages-select arctos_gui --pytest-args "test/test_flake8.py::test_flake8"`
- Single pytest file: `colcon test --packages-select arctos_gui --pytest-args "test/test_pep257.py"`
- Single CTest/GTest by name regex: `colcon test --packages-select arctos_controller --ctest-args -R <regex> --output-on-failure`
- Direct CTest (from build tree): `cd /home/michael/arctos_ws/build/<pkg> && ctest -R <regex> --output-on-failure`

Notes
- `arctos_gui/test/` contains ROS 2 linter tests (ament_flake8 / ament_pep257 / ament_copyright).
- `arctos_controller/test/` contains gtest sources, but `arctos_controller/CMakeLists.txt` currently does not register tests under `if(BUILD_TESTING)`.
- If you add tests/lints, use standard ament patterns: `ament_add_gtest(...)` and/or `find_package(ament_lint_auto REQUIRED)` + `ament_lint_auto_find_test_dependencies()`.

## Lint

Preferred (matches ROS 2 CI expectations)
- Python flake8: `colcon test --packages-select arctos_gui --pytest-args "test/test_flake8.py::test_flake8"`
- Python docstrings (pep257): `colcon test --packages-select arctos_gui --pytest-args "test/test_pep257.py::test_pep257"`

Direct execution (useful for quick iteration)
- `pytest -q arctos_gui/test/test_flake8.py::test_flake8`
- `pytest -q arctos_gui/test/test_pep257.py::test_pep257`

## Code Style (keep diffs focused; prioritize clean, testable code)

Clean code is code the whole team can read and safely evolve (readable, changeable, extensible, maintainable).

Clean Code policy (apply everywhere)
- General: follow standard conventions; KISS (reduce complexity); fix root causes; boy scout rule (small local cleanups, avoid churn).
- Design: keep configurable data high-level (YAML/params); avoid over-configurability; use dependency injection; apply Law of Demeter.
- Branching: prefer polymorphism/strategies/tables to long if/else or switch/case; keep nesting shallow.
- Threading: keep multithreading boundaries explicit and isolated; never touch Qt widgets off the UI thread.
- Readability: be consistent; use explanatory variables; encapsulate boundary conditions and unit conversions.
- Types/units: prefer dedicated value objects/types over primitives; make units explicit (`*_rad`, `*_ms`, `*_hz`) and validate at boundaries.
- Naming: descriptive + unambiguous; searchable/pronounceable; avoid encodings; replace magic numbers with named constants.
- Functions: small, do one thing, descriptive; prefer fewer args; no flag args; minimize side effects.
- Comments: explain intent/constraints/warnings; avoid redundancy and noise; never comment-out code.
- Structure: separate concepts vertically; keep related code close; keep lines short; whitespace for grouping; no alignment tricks.
- Tests: readable, fast, independent, repeatable; one assert (or one concept) per test; hardware-free by default.
- Smells to avoid: rigidity, fragility, immobility, needless complexity, needless repetition, opacity.

Python (`arctos_gui/`, `scripts/`)
- Formatting: 4 spaces; PEP8-ish line lengths; wrap long Qt stylesheet strings carefully.
- Imports: standard library, third-party, then local; one import per line; no wildcard imports.
- Docstrings: PEP257 (module/class/function docstrings; first line is a short summary).
- Typing: add type hints for public APIs and cross-module calls; prefer `Optional[T]` for compatibility.
- Qt/ROS: never block the UI thread; use `QTimer`, signals/slots, or a worker thread for I/O.
- Error handling/logging: catch specific exceptions; use ROS logger or GUI log; avoid `print()`.
- Imports (Qt): prefer module-scope imports; use local imports only to break import cycles.
- Prefer `pathlib.Path` for filesystem paths and explicit encodings for file I/O.
- Use `dataclasses` for simple state/config objects instead of nested dicts.
- Threads: never touch Qt widgets off the UI thread; stop/join workers in `closeEvent`.

C++ (C++17; `arctos_*`, `mks_motor_driver/`)
- Match local formatting (this repo mixes brace/indent styles); keep changes minimal.
- Headers: follow the package's existing pattern (`#pragma once` vs include guards); avoid `using namespace` in headers.
- Includes: prefer `#include <...>` then `#include "..."`; forward declare when it reduces dependencies.
- Types: use `std::size_t` for indices, fixed-width ints for transport/protocol, and explicit casts to satisfy `-Werror=conversion`.
- Error handling: avoid exceptions in control loops and transport paths; return status/`std::optional`, log once per event, keep timeouts bounded.
- Concurrency/RT: avoid heap allocations and high-rate logging in `read()`/`write()`/`update()`; guard shared state and join threads in destructors.
- Time: prefer `std::chrono` for timeouts/durations; avoid raw doubles unless the rest of the API is seconds.
- Constants: prefer `constexpr` and `enum class`; avoid magic numbers.
- Logging: throttle recurring warnings (`RCLCPP_*_THROTTLE`) and keep logs out of hot paths.
- Avoid dynamic allocation in control loops; pre-allocate and reuse buffers where practical.
- Prefer `std::array` for fixed-size (6-axis) vectors when it simplifies ownership/perf.
- Include order: own header, then standard, then ROS/third-party, then local headers.

ROS 2 / ament specifics
- Prefer `colcon build --symlink-install` for iteration (Python edits picked up without reinstall).
- Declare parameters with defaults early, validate them on configure, and keep parameter reads out of hot loops.
- Create publishers/subscribers/timers once (init/configure), not inside high-rate update paths.
- Plugin packages must keep plugin XML and class names in sync:
  - `arctos_hardware_interface/arctos_hardware_interface.xml`: `arctos_hardware_interface/STM32StepperInterface`
  - `arctos_controller/arctos_controller.xml`: `arctos_controller/ArctosSegmentController`
- Public headers live in `include/<pkg>/...` and must be installed via `install(DIRECTORY include/ ...)`.
- For new libs/executables: add `ament_target_dependencies(...)` and `install(TARGETS ...)`.
- Launch files: use `get_package_share_directory` and launch arguments; avoid hard-coded absolute paths.
- Tests: register them under `if(BUILD_TESTING)` and keep them hardware-independent by default.

Config/URDF/launch
- YAML: 2-space indent, no tabs, avoid duplicate keys; keep units explicit.
- Xacro/URDF: prefer properties/macros; keep frame/joint naming consistent with controllers.
- `arctos_moveit_config/` is mostly generated; avoid manual edits unless you know what is overwritten.

Agent hygiene
- Keep changes small and focused; avoid drive-by refactors and churn.
- If you change behavior, update docs/launch/config and add a test when practical.
- Run at least a package-level build/test for touched packages when possible.

Hardware-facing code
- Do not change protocol constants/byte layouts without updating docs and the STM32 side.
- Keep reconnection logic rate-limited; avoid blocking calls in hot control loops.
- Default interfaces seen in this repo: SocketCAN `can0`, STM32 endpoint `192.168.178.159:8888`.
