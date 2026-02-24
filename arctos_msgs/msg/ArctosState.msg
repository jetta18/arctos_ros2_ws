# Comprehensive robot state, published by the arctos_bridge node.
# Maps directly to the STM32 state_broadcast_t packet.

std_msgs/Header header

# Motion state
float64[] positions          # Joint positions in radians
float64[] velocities         # Joint velocities in rad/s
float64[] positions_steps    # Joint positions in raw steps (from STM32)

# System state (matches arctos_system_state_t enum)
uint8 STATE_IDLE=0
uint8 STATE_MOVING=1
uint8 STATE_TRAJ_LOADING=2
uint8 STATE_TRAJ_RUNNING=3
uint8 STATE_TRAJ_COMPLETE=4
uint8 STATE_ERROR=5
uint8 STATE_STOPPING=6
uint8 system_state

# Trajectory info
uint32 trajectory_id
uint16 traj_points_loaded
uint16 traj_current_segment
uint16 traj_total_segments

# Endstop summary (detailed info in ArctosEndstops)
uint16 endstop_states        # Bitmask: bit = axis*2 + direction

# Servo
uint16 servo_pulse_us

# Homing
bool homing_active
uint8 homing_axis
uint8 homing_state
uint8 is_homed_bitmask          # bit[i] = axis i has been homed to position

# Diagnostics
uint32 stm32_uptime_ms
uint16 broadcast_seq
