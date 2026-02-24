# Detailed endstop state for all axes.
# Published by the arctos_bridge node.

std_msgs/Header header

# Endstop state bitmask: bit index = axis * 2 + direction
# Direction: 0 = MIN, 1 = MAX
# Axes: 0=X, 1=Y, 2=Z, 3=A, 4=B, 5=C
uint16 endstop_states

# Per-endstop trigger counts (12 entries: 6 axes × 2 directions)
uint32[] trigger_counts

# Convenience: list of currently triggered endstop names
string[] triggered_names
