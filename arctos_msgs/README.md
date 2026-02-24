# arctos_msgs

Custom message and service definitions for the Arctos robot arm.

This is a standalone message-only package with no runtime dependencies beyond `std_msgs` and `builtin_interfaces`. All Arctos-specific ROS2 interfaces live here so other packages can depend on `arctos_msgs` without pulling in the full bridge or hardware interface.

## Messages

| Message | Description |
|---|---|
| `ArctosState` | Comprehensive robot state (positions, velocities, system state, trajectory info, endstops, servo, homing, diagnostics) |
| `ArctosEndstops` | Detailed endstop states with bitmask, trigger counts, and triggered names |
| `ArctosError` | Error report with error code constants and message string |

## Services

| Service | Description |
|---|---|
| `Ping` | Check STM32 connectivity, returns round-trip time |
| `Stop` | Emergency stop — ramp all axes to zero velocity |
| `SetServo` | Set gripper servo pulse width and move duration |
| `HomeAxis` | Start or stop homing for a single axis |

## Usage

```python
from arctos_msgs.msg import ArctosState, ArctosEndstops, ArctosError
from arctos_msgs.srv import Ping, SetServo, Stop, HomeAxis
```

```cpp
#include "arctos_msgs/msg/arctos_state.hpp"
#include "arctos_msgs/srv/ping.hpp"
```

## Build

```bash
colcon build --packages-select arctos_msgs
```
