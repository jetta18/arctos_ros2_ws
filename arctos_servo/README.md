# arctos_servo

MoveIt Servo API for the Arctos robot. Provides real-time **joint jog** and
**cartesian jog** control through a single, thread-safe Python class
(`ServoClient`) that can be used from any context — GUI, script, or ROS node.

## Prerequisites

The bringup must be running:

```bash
ros2 launch arctos_bringup arctos_bringup.launch.py
```

The MoveIt Servo node must be running (the launch file handles controller
switching automatically):

```bash
ros2 launch arctos_servo servo_teleop.launch.py
```

## Python API

### Quick Start

```python
from arctos_servo import ServoClient

client = ServoClient()
client.connect()

# Joint jog — move X_joint at half speed
client.joint_jog("X_joint", 0.5)

# Cartesian jog — move in +X direction
client.cartesian_jog(linear=(0.5, 0.0, 0.0))

# Stop all motion
client.stop()

# Query state
positions = client.get_joint_positions()   # {"X_joint": 0.12, ...}
ee_pose = client.get_ee_pose()             # {"x": 0.3, "y": 0.0, ..., "rz": 0.1}

client.disconnect()
```

### ServoClient

```python
ServoClient(node_name="arctos_servo_client", auto_switch_controllers=True)
```

| Parameter                | Default                    | Description                                                    |
|--------------------------|----------------------------|----------------------------------------------------------------|
| `node_name`              | `"arctos_servo_client"`    | ROS node name for the internal node                            |
| `auto_switch_controllers`| `True`                     | Automatically load/switch to `arctos_servo_controller` on connect and switch back on disconnect |

#### Lifecycle

| Method                        | Description                                                                 |
|-------------------------------|-----------------------------------------------------------------------------|
| `connect(timeout_sec=10.0)`   | Start ROS node, switch controllers, start MoveIt Servo. Returns `bool`.     |
| `disconnect()`                | Stop servo, switch back to trajectory controller, clean up.                 |
| `is_connected()`              | Returns `True` if connected and servo is running.                           |

#### Joint Jog

| Method                                          | Description                                                |
|-------------------------------------------------|------------------------------------------------------------|
| `joint_jog(joint_name, velocity)`               | Jog a joint by name. Velocity is unitless `[-1.0, 1.0]`.  |
| `joint_jog_by_index(joint_index, velocity)`     | Jog a joint by index (0–5). Same velocity range.           |
| `multi_joint_jog(joint_names, velocities)`      | Jog multiple joints simultaneously.                        |

Joint names: `X_joint`, `Y_joint`, `Z_joint`, `A_joint`, `B_joint`, `C_joint`

#### Cartesian Jog

| Method                                                  | Description                                          |
|---------------------------------------------------------|------------------------------------------------------|
| `cartesian_jog(linear=(0,0,0), angular=(0,0,0), frame_id="base_link")` | Publish a twist command. All values unitless `[-1.0, 1.0]`. |

- `linear`: `(x, y, z)` — translation velocities
- `angular`: `(rx, ry, rz)` — rotation velocities
- `frame_id`: reference frame (default: `base_link`)

#### Stop

| Method   | Description                                              |
|----------|----------------------------------------------------------|
| `stop()` | Send zero velocity on both joint and cartesian channels. |

#### State Queries

| Method                              | Returns                                                    |
|-------------------------------------|------------------------------------------------------------|
| `get_joint_positions()`             | `dict[str, float]` — joint positions in radians            |
| `get_joint_position(joint_name)`    | `float` — single joint position in radians                 |
| `get_ee_pose()`                     | `dict` with keys `x, y, z` (m) and `rx, ry, rz` (rad)    |

### Constants

```python
from arctos_servo.constants import (
    JOINT_NAMES,          # ["X_joint", "Y_joint", ..., "C_joint"]
    NUM_JOINTS,           # 6
    JOINT_JOG_TOPIC,      # "/servo_node/delta_joint_cmds"
    TWIST_TOPIC,          # "/servo_node/delta_twist_cmds"
    PUBLISH_RATE_HZ,      # 30.0
    BASE_FRAME,           # "base_link"
    EE_FRAME,             # "Link_6_1"
    SERVO_CONTROLLER,     # "arctos_servo_controller"
    TRAJECTORY_CONTROLLER, # "arctos_controller"
)
```

## Keyboard Teleop

A CLI tool that uses `ServoClient` with both joint and cartesian modes:

```bash
ros2 run arctos_servo keyboard_teleop
```

| Key       | Joint Mode              | Cartesian Mode        |
|-----------|-------------------------|-----------------------|
| `1`–`6`  | Select joint            | —                     |
| `W` / `S` | Jog joint + / −        | +X / −X               |
| `A` / `D` | —                      | +Y / −Y               |
| `Q` / `E` | —                      | +Z / −Z               |
| `I` / `K` | —                      | +RX / −RX             |
| `J` / `L` | —                      | +RY / −RY             |
| `U` / `O` | —                      | +RZ / −RZ             |
| `Tab`     | Switch to cartesian    | Switch to joint       |
| `Space`   | Stop all motion        | Stop all motion       |
| `Esc`     | Quit                   | Quit                  |

## GUI Integration Example

```python
from arctos_servo import ServoClient

class MyWidget:
    def __init__(self):
        self._servo = ServoClient()

    def on_connect_clicked(self):
        self._servo.connect()

    def on_jog_button_pressed(self, joint_name, direction):
        self._servo.joint_jog(joint_name, direction)  # +1.0 or -1.0

    def on_jog_button_released(self):
        self._servo.stop()

    def on_cartesian_button_pressed(self, axis, direction):
        linear = [0.0, 0.0, 0.0]
        angular = [0.0, 0.0, 0.0]
        axes = {"x": 0, "y": 1, "z": 2}
        rot_axes = {"rx": 0, "ry": 1, "rz": 2}
        if axis in axes:
            linear[axes[axis]] = direction
        elif axis in rot_axes:
            angular[rot_axes[axis]] = direction
        self._servo.cartesian_jog(
            linear=tuple(linear), angular=tuple(angular),
        )

    def on_disconnect_clicked(self):
        self._servo.disconnect()
```

## Package Structure

```
arctos_servo/
├── arctos_servo/
│   ├── __init__.py          # Re-exports ServoClient, JOINT_NAMES, NUM_JOINTS
│   ├── constants.py         # Shared constants (topics, frames, joint names)
│   ├── servo_client.py      # ServoClient — the core API
│   └── keyboard_teleop.py   # CLI tool using ServoClient
├── config/
│   └── arctos_servo_config.yaml   # MoveIt Servo parameters
├── launch/
│   └── servo_teleop.launch.py     # Loads controller + starts servo node
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```
