"""Shared constants for the arctos_servo package."""

JOINT_NAMES = [
    'X_joint',
    'Y_joint',
    'Z_joint',
    'A_joint',
    'B_joint',
    'C_joint',
]

NUM_JOINTS = len(JOINT_NAMES)

JOINT_JOG_TOPIC = '/servo_node/delta_joint_cmds'
TWIST_TOPIC = '/servo_node/delta_twist_cmds'
JOINT_STATES_TOPIC = '/joint_states'
SERVO_STATUS_TOPIC = '/servo_node/status'

START_SERVO_SERVICE = '/servo_node/start_servo'
STOP_SERVO_SERVICE = '/servo_node/stop_servo'

PUBLISH_RATE_HZ = 30.0

BASE_FRAME = 'base_link'
EE_FRAME = 'Link_6_1'

PLANNING_GROUP = 'arctos_arm'

TRAJECTORY_CONTROLLER = 'arctos_controller'
SERVO_CONTROLLER = 'arctos_servo_controller'
