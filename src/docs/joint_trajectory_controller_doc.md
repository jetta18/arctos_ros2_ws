joint_trajectory_controller
Controller for executing joint-space trajectories on a group of joints. The controller interpolates in time between the points so that their distance can be arbitrary. Even trajectories with only one point are accepted. Trajectories are specified as a set of waypoints to be reached at specific time instants, which the controller attempts to execute as well as the mechanism allows. Waypoints consist of positions, and optionally velocities and accelerations.

Parts of this documentation were originally published in the ROS 1 wiki under the CC BY 3.0 license. Citations are given in the respective section, but were adapted for the ROS 2 implementation. 1

Hardware interface types
Currently, joints with hardware interface types position, velocity, acceleration, and effort (defined here) are supported in the following combinations as command interfaces:

position

position, velocity

position, velocity, acceleration

velocity

effort

This means that the joints can have one or more command interfaces, where the following control laws are applied at the same time:

For command interfaces position, the desired positions are simply forwarded to the joints,

For command interfaces acceleration, desired accelerations are simply forwarded to the joints.

For velocity (effort) command interfaces, the position+velocity trajectory following error is mapped to velocity (effort) commands through a PID loop if it is configured (Details about parameters).

This leads to the following allowed combinations of command and state interfaces:

With command interface position, there are no restrictions for state interfaces.

With command interface velocity:

if command interface velocity is the only one, state interfaces must include position, velocity .

With command interface effort, state interfaces must include position, velocity.

With command interface acceleration, state interfaces must include position, velocity.

Further restrictions of state interfaces exist:

velocity state interface cannot be used if position interface is missing.

acceleration state interface cannot be used if position and velocity interfaces are not present.”

Example controller configurations can be found below.

Other features
Realtime-safe implementation.

Proper handling of wrapping (continuous) joints.

Robust to system clock changes: Discontinuous system clock changes do not cause discontinuities in the execution of already queued trajectory segments.

Using Joint Trajectory Controller(s)
The controller expects at least position feedback from the hardware. Joint velocities and accelerations are optional. Currently the controller does not internally integrate velocity from acceleration and position from velocity. Therefore if the hardware provides only acceleration or velocity states they have to be integrated in the hardware-interface implementation of velocity and position to use these controllers.

The generic version of Joint Trajectory controller is implemented in this package. A yaml file for using it could be:

controller_manager:
  ros__parameters:
    joint_trajectory_controller:
    type: "joint_trajectory_controller/JointTrajectoryController"

joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    state_publish_rate: 50.0
    action_monitor_rate: 20.0

    allow_partial_joints_goal: false
    open_loop_control: true
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0
      joint1:
        trajectory: 0.05
        goal: 0.03
Preemption policy 1
Only one action goal can be active at any moment, or none if the topic interface is used. Path and goal tolerances are checked only for the trajectory segments of the active goal.

When an active action goal is preempted by another command coming from the action interface, the goal is canceled and the client is notified. The trajectory is replaced in a defined way, see trajectory replacement.

Sending an empty trajectory message from the topic interface (not the action interface) will override the current action goal and not abort the action.

Description of controller’s interfaces
References
(the controller is not yet implemented as chainable controller)

States
The state interfaces are defined with joints and state_interfaces parameters as follows: <joint>/<state_interface>.

Legal combinations of state interfaces are given in section Hardware Interface Types.

Commands
There are two mechanisms for sending trajectories to the controller:

via action, see actions

via topic, see subscriber

Both use the trajectory_msgs/msg/JointTrajectory message to specify trajectories, and require specifying values for all the controller joints (as opposed to only a subset) if allow_partial_joints_goal is not set to True. For further information on the message format, see trajectory representation.

Actions 1
<controller_name>/follow_joint_trajectory [control_msgs::action::FollowJointTrajectory]
Action server for commanding the controller

The primary way to send trajectories is through the action interface, and should be favored when execution monitoring is desired.

Action goals allow to specify not only the trajectory to execute, but also (optionally) path and goal tolerances. For details, see the JointTolerance message:

The tolerances specify the amount the position, velocity, and
accelerations can vary from the setpoints.  For example, in the case
of trajectory control, when the actual position varies beyond
(desired position + position tolerance), the trajectory goal may
abort.

There are two special values for tolerances:
  * 0 - The tolerance is unspecified and will remain at whatever the default is
  * -1 - The tolerance is "erased".  If there was a default, the joint will be
        allowed to move without restriction.
When no tolerances are specified, the defaults given in the parameter interface are used (see Details about parameters). If tolerances are violated during trajectory execution, the action goal is aborted, the client is notified, and the current position is held.

The action server returns success to the client and continues with the last commanded point after the target is reached within the specified tolerances.

Subscriber 1
<controller_name>/joint_trajectory [trajectory_msgs::msg::JointTrajectory]
Topic for commanding the controller

The topic interface is a fire-and-forget alternative. Use this interface if you don’t care about execution monitoring. The goal tolerance specification is not used in this case, as there is no mechanism to notify the sender about tolerance violations. If state tolerances are violated, the trajectory is aborted and the current position is held. Note that although some degree of monitoring is available through the ~/query_state service and ~/state topic it is much more cumbersome to realize than with the action interface.

Publishers
<controller_name>/controller_state [control_msgs::msg::JointTrajectoryControllerState]
Topic publishing internal states with the update-rate of the controller manager

Services
<controller_name>/query_state [control_msgs::srv::QueryTrajectoryState]
Query controller state at any future time

Trajectory Representation
Trajectories are represented internally with trajectory_msgs/msg/JointTrajectory data structure.

Currently, two interpolation methods are implemented: none and spline. By default, a spline interpolator is provided, but it’s possible to support other representations.

Warning

The user has to ensure that the correct inputs are provided for the trajectory, which are needed by the controller’s setup of command interfaces and PID configuration. There is no sanity check and missing fields in the sampled trajectory might cause segmentation faults.

Interpolation Method none
It returns the initial point until the time for the first trajectory data point is reached. Then, it simply takes the next given datapoint.

Warning

It does not deduce (integrate) trajectory from derivatives, nor does it calculate derivatives. I.e., one has to provide position and its derivatives as needed.

Interpolation Method spline
The spline interpolator uses the following interpolation strategies depending on the waypoint specification:

Linear:

Used, if only position is specified.

Returns position and velocity

Guarantees continuity at the position level.

Discouraged because it yields trajectories with discontinuous velocities at the waypoints.

Cubic:

Used, if position and velocity are specified.

Returns position, velocity, and acceleration.

Guarantees continuity at the velocity level.

Quintic:

Used, if position, velocity and acceleration are specified

Returns position, velocity, and acceleration.

Guarantees continuity at the acceleration level.

Trajectories with velocity fields only, velocity and acceleration only, or acceleration fields only can be processed and are accepted, if allow_integration_in_goal_trajectories is true. Position (and velocity) is then integrated from velocity (or acceleration, respectively) by Heun’s method.

Visualized Examples
To visualize the difference of the different interpolation methods and their inputs, different trajectories defined at a 0.5s grid and are sampled at a rate of 10ms.

Sampled trajectory with linear spline if position is given only:

Sampled trajectory with splines if position is given only
Sampled trajectory with cubic splines if velocity is given only (no deduction for interpolation method none):

Sampled trajectory with splines if velocity is given only
Sampled trajectory if position and velocity is given:

Note

If the same integration method was used (Trajectory class uses Heun’s method), then the spline method this gives identical results as above where velocity only was given as input.

Sampled trajectory if position and velocity is given
Sampled trajectory with quintic splines if acceleration is given only (no deduction for interpolation method none):

Sampled trajectory with splines if acceleration is given only
Sampled trajectory if position, velocity, and acceleration points are given:

Note

If the same integration method was used (Trajectory class uses Heun’s method), then the spline method this gives identical results as above where acceleration only was given as input.

Sampled trajectory with splines if position, velocity, and acceleration is given
Sampled trajectory if the same position, velocity, and acceleration points as above are given, but with a nonzero initial point:

Sampled trajectory with splines if position, velocity, and acceleration is given with nonzero initial point
Sampled trajectory if the same position, velocity, and acceleration points as above are given but with the first point starting at t=0:

Note

If the first point is starting at t=0, there is no interpolation from the initial point to the trajectory.

Sampled trajectory with splines if position, velocity, and acceleration is given with nonzero initial point and first point starting at ``t=0``
Sampled trajectory with splines if inconsistent position, velocity, and acceleration points are given:

Note

Interpolation method none only gives the next input points, while the spline interpolation method shows high overshoot to match the given trajectory points.

Sampled trajectory with splines if inconsistent position, velocity, and acceleration is given
Trajectory Replacement
Parts of this documentation were originally published in the ROS 1 wiki under the CC BY 3.0 license. 1

Joint trajectory messages allow to specify the time at which a new trajectory should start executing by means of the header timestamp, where zero time (the default) means “start now”.

The arrival of a new trajectory command does not necessarily mean that the controller will completely discard the currently running trajectory and substitute it with the new one. Rather, the controller will take the useful parts of both and combine them appropriately, yielding a smarter trajectory replacement strategy.

The steps followed by the controller for trajectory replacement are as follows:

Get useful parts of the new trajectory: Preserve all waypoints whose time to be reached is in the future, and discard those with times in the past. If there are no useful parts (ie. all waypoints are in the past) the new trajectory is rejected and the current one continues execution without changes.

Get useful parts of the current trajectory: Preserve the current trajectory up to the start time of the new trajectory, discard the later parts.

Combine the useful parts of the current and new trajectories.

The following examples describe this behavior in detail.

The first example shows a joint which is in hold position mode (flat grey line labeled pos hold in the figure below). A new trajectory (shown in red) arrives at the current time (now), which contains three waypoints and a start time in the future (traj start). The time at which waypoints should be reached (time_from_start member of trajectory_msgs/JointTrajectoryPoint) is relative to the trajectory start time.

The controller splices the current hold trajectory at time traj start and appends the three waypoints. Notice that between now and traj start the previous position hold is still maintained, as the new trajectory is not supposed to start yet. After the last waypoint is reached, its position is held until new commands arrive.

Receiving a new trajectory.

The controller guarantees that the transition between the current and new trajectories will be smooth. Longer times to reach the first waypoint mean slower transitions.

The next examples discuss the effect of sending the same trajectory to the controller with different start times. The scenario is that of a controller executing the trajectory from the previous example (shown in red), and receiving a new command (shown in green) with a trajectory start time set to either zero (start now), a future time, or a time in the past.

Trajectory start time in the future.

Zero trajectory start time (start now).

Of special interest is the last example, where the new trajectory start time and first waypoint are in the past (before now). In this case, the first waypoint is discarded and only the second one is realized.

Trajectory start time in the past.

Details about parameters
This controller uses the generate_parameter_library to handle its parameters. The parameter definition file located in the src folder contains descriptions for all the parameters used by the controller.

List of parameters
joints (string_array)
Joint names to control and listen to

Read only: True

Default: {}

Constraints:

contains no duplicates

command_joints (string_array)
Joint names to control. This parameters is used if JTC is used in a controller chain where command and state interfaces don’t have same names.

Read only: True

Default: {}

Constraints:

contains no duplicates

command_interfaces (string_array)
Command interfaces provided by the hardware interface for all joints

Read only: True

Default: {}

Constraints:

contains no duplicates

every element is one of the list [‘position’, ‘velocity’, ‘acceleration’, ‘effort’]

Custom validator: joint_trajectory_controller::command_interface_type_combinations

state_interfaces (string_array)
State interfaces provided by the hardware for all joints.

Read only: True

Default: {}

Constraints:

contains no duplicates

every element is one of the list [‘position’, ‘velocity’, ‘acceleration’]

Custom validator: joint_trajectory_controller::state_interface_type_combinations

allow_partial_joints_goal (bool)
Allow joint goals defining trajectory for only some joints.

Default: false

open_loop_control (bool)
Use controller in open-loop control mode

The controller ignores the states provided by hardware interface but using last commands as states for starting the trajectory interpolation.

It deactivates the feedback control, see the gains structure.

This is useful if hardware states are not following commands, i.e., an offset between those (typical for hydraulic manipulators).

If this flag is set, the controller tries to read the values from the command interfaces on activation. If they have real numeric values, those will be used instead of state interfaces. Therefore it is important set command interfaces to NaN (i.e., std::numeric_limits<double>::quiet_NaN()) or state values when the hardware is started.

Read only: True

Default: false

allow_integration_in_goal_trajectories (bool)
Allow integration in goal trajectories to accept goals without position or velocity specified

Default: false

state_publish_rate (double)
Rate controller state is published

Default: 50.0

Constraints:

greater than or equal to 0.1

set_last_command_interface_value_as_state_on_activation (bool)
When set to true, the last command interface value is used as both the current state and the last commanded state upon activation. When set to false, the current state is used for both.

Default: true

action_monitor_rate (double)
Rate to monitor status changes when the controller is executing action (control_msgs::action::FollowJointTrajectory)

Read only: True

Default: 20.0

Constraints:

greater than or equal to 0.1

interpolation_method (string)
The type of interpolation to use, if any

Read only: True

Default: “splines”

Constraints:

one of the specified values: [‘splines’, ‘none’]

allow_nonzero_velocity_at_trajectory_end (bool)
If false, the last velocity point has to be zero or the goal will be rejected

Default: true

cmd_timeout (double)
Timeout after which the input command is considered stale. Timeout is counted from the end of the trajectory (the last point). cmd_timeout must be greater than constraints.goal_time, otherwise ignored. If zero, timeout is deactivated

Default: 0.0

constraints
Default values for tolerances if no explicit values are set in the JointTrajectory message.

constraints.stopped_velocity_tolerance (double)
Velocity tolerance for at the end of the trajectory that indicates that controlled system is stopped.

Default: 0.01

constraints.goal_time (double)
Time tolerance for achieving trajectory goal before or after commanded time. If set to zero, the controller will wait a potentially infinite amount of time.

Default: 0.0

Constraints:

greater than or equal to 0.0

constraints.<joints>.trajectory (double)
Per-joint trajectory offset tolerance during movement.

Default: 0.0

constraints.<joints>.goal (double)
Per-joint trajectory offset tolerance at the goal position.

Default: 0.0

gains
Only relevant, if open_loop_control is not set.

If velocity is the only command interface for all joints or an effort command interface is configured, PID controllers are used for every joint. This structure contains the controller gains for every joint with the control law

with the desired velocity 
, the measured velocity 
, the position error 
 (definition see angle_wraparound below), the controller period 
, and the velocity or effort manipulated variable (control variable) 
, respectively.

gains.<joints>.p (double)
Proportional gain 
 for PID

Default: 0.0

gains.<joints>.i (double)
Integral gain 
 for PID

Default: 0.0

gains.<joints>.d (double)
Derivative gain 
 for PID

Default: 0.0

gains.<joints>.i_clamp (double)
Integral clamp. Symmetrical in both positive and negative direction.

Default: 0.0

gains.<joints>.ff_velocity_scale (double)
Feed-forward scaling 
 of velocity

Default: 0.0

gains.<joints>.normalize_error (bool)
(Deprecated) Use position error normalization to -pi to pi.

Default: false

gains.<joints>.angle_wraparound (bool)
For joints that wrap around (without end stop, ie. are continuous), where the shortest rotation to the target position is the desired motion. If true, the position error 
 is normalized between 
. Otherwise 
 is used, with the desired position 
 and the measured position 
 from the state interface.

Default: false

An example parameter file
joint_trajectory_controller:
  ros__parameters:
    action_monitor_rate: 20.0
    allow_integration_in_goal_trajectories: false
    allow_nonzero_velocity_at_trajectory_end: true
    allow_partial_joints_goal: false
    cmd_timeout: 0.0
    command_interfaces: '{}'
    command_joints: '{}'
    constraints:
      <joints>:
        goal: 0.0
        trajectory: 0.0
      goal_time: 0.0
      stopped_velocity_tolerance: 0.01
    gains:
      <joints>:
        angle_wraparound: false
        d: 0.0
        ff_velocity_scale: 0.0
        i: 0.0
        i_clamp: 0.0
        normalize_error: false
        p: 0.0
    interpolation_method: splines
    joints: '{}'
    open_loop_control: false
    set_last_command_interface_value_as_state_on_activation: true
    state_interfaces: '{}'
    state_publish_rate: 50.0