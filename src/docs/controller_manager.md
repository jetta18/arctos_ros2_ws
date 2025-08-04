Controller Manager
Controller Manager is the main component in the ros2_control framework. It manages lifecycle of controllers, access to the hardware interfaces and offers services to the ROS-world.

Determinism
For best performance when controlling hardware you want the controller manager to have as little jitter as possible in the main control loop. The normal linux kernel is optimized for computational throughput and therefore is not well suited for hardware control. The two easiest kernel options are the Real-time Ubuntu 22.04 LTS Beta or linux-image-rt-amd64 on Debian Bullseye.

If you have a realtime kernel installed, the main thread of Controller Manager attempts to configure SCHED_FIFO with a priority of 50. By default, the user does not have permission to set such a high priority. To give the user such permissions, add a group named realtime and add the user controlling your robot to this group:

sudo addgroup realtime
sudo usermod -a -G realtime $(whoami)
Afterwards, add the following limits to the realtime group in /etc/security/limits.conf:

@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock unlimited
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock unlimited
The limits will be applied after you log out and in again.

Subscribers
~/robot_description [std_msgs::msg::String]
String with the URDF xml, e.g., from robot_state_publisher.

Note

Typically one would remap the topic to /robot_description, which is the default setup with robot_state_publisher. An example for a python launchfile is

control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[robot_controllers],
    output="both",
    remappings=[
        ("~/robot_description", "/robot_description"),
    ],
)
Parameters
hardware_components_initial_state
Map of parameters for controlled lifecycle management of hardware components. The names of the components are defined as attribute of <ros2_control>-tag in robot_description. Hardware components found in robot_description, but without explicit state definition will be immediately activated. Detailed explanation of each parameter is given below. The full structure of the map is given in the following example:

hardware_components_initial_state:
  unconfigured:
    - "arm1"
    - "arm2"
  inactive:
    - "base3"
hardware_components_initial_state.unconfigured (optional; list<string>; default: empty)
Defines which hardware components will be only loaded immediately when controller manager is started.

hardware_components_initial_state.inactive (optional; list<string>; default: empty)
Defines which hardware components will be configured immediately when controller manager is started.

Note

Passing the robot description parameter directly to the control_manager node is deprecated. Use ~/robot_description topic from robot_state_publisher instead.

robot_description (optional; string; deprecated)
String with the URDF string as robot description. This is usually result of the parsed description files by xacro command.

update_rate (mandatory; double)
The frequency of controller manager’s real-time update loop. This loop reads states from hardware, updates controller and writes commands to hardware.

<controller_name>.type
Name of a plugin exported using pluginlib for a controller. This is a class from which controller’s instance with name “controller_name” is created.

Helper scripts
There are two scripts to interact with controller manager from launch files:

spawner - loads, configures and start a controller on startup.

unspawner - stops and unloads a controller.

hardware_spawner - activates and configures a hardware component.

spawner
ros2 run controller_manager spawner -h
usage: spawner [-h] [-c CONTROLLER_MANAGER] [-p PARAM_FILE] [-n NAMESPACE] [--load-only] [--stopped] [--inactive] [-t CONTROLLER_TYPE] [-u]
              [--controller-manager-timeout CONTROLLER_MANAGER_TIMEOUT] [--switch-timeout SWITCH_TIMEOUT]
              [--service-call-timeout SERVICE_CALL_TIMEOUT] [--activate-as-group]
              controller_names [controller_names ...]

positional arguments:
  controller_names      List of controllers

options:
  -h, --help            show this help message and exit
  -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                        Name of the controller manager ROS node
  -p PARAM_FILE, --param-file PARAM_FILE
                        Controller param file to be loaded into controller node before configure. Pass multiple times to load different files for different controllers or to override the parameters of the same controller.
  -n NAMESPACE, --namespace NAMESPACE
                        Namespace for the controller
  --load-only           Only load the controller and leave unconfigured.
  --stopped             Load and configure the controller, however do not activate them
  --inactive            Load and configure the controller, however do not activate them
  -t CONTROLLER_TYPE, --controller-type CONTROLLER_TYPE
                        If not provided it should exist in the controller manager namespace
  -u, --unload-on-kill  Wait until this application is interrupted and unload controller
  --controller-manager-timeout CONTROLLER_MANAGER_TIMEOUT
                        Time to wait for the controller manager service to be available
  --service-call-timeout SERVICE_CALL_TIMEOUT
                        Time to wait for the service response from the controller manager
  --switch-timeout SWITCH_TIMEOUT
                        Time to wait for a successful state switch of controllers. Useful when switching cannot be performed immediately, e.g.,
                        paused simulations at startup
  --activate-as-group   Activates all the parsed controllers list together instead of one by one. Useful for activating all chainable controllers
                        altogether
The parsed controller config file can follow the same conventions as the typical ROS 2 parameter file format. Now, the spawner can handle config files with wildcard entries and also the controller name in the absolute namespace. See the following examples on the config files:

/**:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController

    command_interfaces:
      - position
      .....

position_trajectory_controller_joint1:
  ros__parameters:
    joints:
      - joint1

position_trajectory_controller_joint2:
  ros__parameters:
    joints:
      - joint2
/**/position_trajectory_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
      - joint1
      - joint2

    command_interfaces:
      - position
      .....
/position_trajectory_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
      - joint1
      - joint2

    command_interfaces:
      - position
      .....
position_trajectory_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
      - joint1
      - joint2

    command_interfaces:
      - position
      .....
/rrbot_1/position_trajectory_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
      - joint1
      - joint2

    command_interfaces:
      - position
      .....
unspawner
ros2 run controller_manager unspawner -h
usage: unspawner [-h] [-c CONTROLLER_MANAGER] [--switch-timeout SWITCH_TIMEOUT] controller_names [controller_names ...]

positional arguments:
  controller_names      Name of the controller

options:
  -h, --help            show this help message and exit
  -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                        Name of the controller manager ROS node
  --switch-timeout SWITCH_TIMEOUT
                        Time to wait for a successful state switch of controllers. Useful if controllers cannot be switched immediately, e.g., paused
                        simulations at startup
hardware_spawner
ros2 run controller_manager hardware_spawner -h
usage: hardware_spawner [-h] [-c CONTROLLER_MANAGER] [--controller-manager-timeout CONTROLLER_MANAGER_TIMEOUT]
                        (--activate | --configure)
                        hardware_component_names [hardware_component_names ...]

positional arguments:
  hardware_component_names
                        The name of the hardware components which should be activated.

options:
  -h, --help            show this help message and exit
  -c CONTROLLER_MANAGER, --controller-manager CONTROLLER_MANAGER
                        Name of the controller manager ROS node
  --controller-manager-timeout CONTROLLER_MANAGER_TIMEOUT
                        Time to wait for the controller manager
  --activate            Activates the given components. Note: Components are by default configured before activated.
  --configure           Configures the given components.
rqt_controller_manager
A GUI tool to interact with the controller manager services to be able to switch the lifecycle states of the controllers as well as the hardware components.

../../../../_images/rqt_controller_manager.png
It can be launched independently using the following command or as rqt plugin.

 ros2 run rqt_controller_manager rqt_controller_manager

* Double-click on a controller or hardware component to show the additional info.
* Right-click on a controller or hardware component to show a context menu with options for lifecycle management.
Using the Controller Manager in a Process
The ControllerManager may also be instantiated in a process as a class, but proper care must be taken when doing so. The reason for this is because the ControllerManager class inherits from rclcpp::Node.

If there is more than one Node in the process, global node name remap rules can forcibly change the ControllerManager's node name as well, leading to duplicate node names. This occurs whether the Nodes are siblings or exist in a hierarchy.

../../../../_images/global_general_remap.png
The workaround for this is to specify another node name remap rule in the NodeOptions passed to the ControllerManager node (causing it to ignore the global rule), or ensure that any remap rules are targeted to specific nodes.

../../../../_images/global_specific_remap.png
auto options = controller_manager::get_cm_node_options();
  options.arguments({
    "--ros-args",
    "--remap", "_target_node_name:__node:=dst_node_name",
    "--log-level", "info"});

  auto cm = std::make_shared<controller_manager::ControllerManager>(
    executor, "_target_node_name", "some_optional_namespace", options);
Launching controller_manager with ros2_control_node
The controller_manager can be launched with the ros2_control_node executable. The following example shows how to launch the controller_manager with the ros2_control_node executable:

control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[robot_controllers],
    output="both",
)
The ros2_control_node executable uses the following parameters from the controller_manager node:

lock_memory (optional; bool; default: false for a non-realtime kernel, true for a realtime kernel)
Locks the memory of the controller_manager node at startup to physical RAM in order to avoid page faults and to prevent the node from being swapped out to disk. Find more information about the setup for memory locking in the following link : How to set ulimit values The following command can be used to set the memory locking limit temporarily : ulimit -l unlimited.

cpu_affinity (optional; int; default: -1)
Sets the CPU affinity of the controller_manager node to the specified CPU core. The value of -1 means that the CPU affinity is not set.

thread_priority (optional; int; default: 50)
Sets the thread priority of the controller_manager node to the specified value. The value must be between 0 and 99.

use_sim_time (optional; bool; default: false)
Enables the use of simulation time in the controller_manager node.

Concepts
Restarting all controllers
The simplest way to restart all controllers is by using switch_controllers services or CLI and adding all controllers to start and stop lists. Note that not all controllers have to be restarted, e.g., broadcasters.

Restarting hardware
If hardware gets restarted then you should go through its lifecycle again. This can be simply achieved by returning ERROR from write and read methods of interface implementation. NOT IMPLEMENTED YET - PLEASE STOP/RESTART ALL CONTROLLERS MANUALLY FOR NOW The controller manager detects that and stops all the controllers that are commanding that hardware and restarts broadcasters that are listening to its states.

Controller Chaining / Cascade Control
This document proposes a minimal-viable-implementation of serial controller chaining as described in Chaining Controllers design document. Cascade control is a specific type of controller chaining.

Scope of the Document and Background Knowledge
This approach focuses only on serial chaining of controllers and tries to reuse existing mechanisms for it. It focuses on inputs and outputs of a controller and their management in the controller manager. The concept of controller groups will be introduced only for clarity reasons, and its only meaning is that controllers in that group can be updated in arbitrary order. This doesn’t mean that the controller groups as described in the controller chaining document will not be introduced and used in the future. Nevertheless, the author is convinced that this would add only unnecessary complexity at this stage, although in the long term they could provide clearer structure and interfaces.

Motivation, Purpose and Use
To describe the intent of this document, lets focus on the simple yet sufficient example Example 2 from ‘controllers_chaining’ design docs:

Example2
In this example, we want to chain ‘position_tracking’ controller with ‘diff_drive_controller’ and two PID controllers. Let’s now imagine a use-case where we don’t only want to run all those controllers as a group, but also flexibly add preceding steps. This means the following:

When a robot is started, we want to check if motor velocity control is working properly and therefore only PID controllers are activated. At this stage we can control the input of PID controller also externally using topics. However, these controllers also provide virtual interfaces, so we can chain them.

Then “diff_drive_controller” is activated and attaches itself to the virtual input interfaces of PID controllers. PID controllers also get informed that they are working in chained mode and therefore disable their external interface through subscriber. Now we check if kinematics of differential robot is running properly.

After that, “position_tracking” can be activated and attached to “diff_drive_controller” that disables its external interfaces.

If any of the controllers is deactivated, also all preceding controllers are deactivated.

Note

Controllers that expose the reference interfaces are switched to chained mode only when their reference interfaces are used by other controllers. When their reference interfaces are not used by other controllers, they are switched back to get references from the subscriber. However, the controllers that expose the state interfaces are not triggered to chained mode when their state interfaces are used by other controllers.

Note

This document uses terms preceding and following controller. These terms refer to such ordering of controllers that controller A precedes controller B if A claims (connects its output to) B’s reference interfaces (inputs). In the example diagram at the beginning of this section, ‘diff_drive_controller’ precedes ‘pid left wheel’ and ‘pid right wheel’. Consequently, ‘pid left wheel’ and ‘pid right wheel’ are controllers following after ‘diff_drive_controller’.

Implementation
A Controller Base-Class: ChainableController
A ChainableController extends ControllerInterface class with virtual InterfaceConfiguration input_interface_configuration() const = 0 method. This method should implement for each controller that can be preceded by another controller exporting all the input interfaces. For simplicity reasons, it is assumed for now that controller’s all input interfaces are used. Therefore, do not try to implement any exclusive combinations of input interfaces, but rather write multiple controllers if you need exclusivity.

The ChainableController base class implements void set_chained_mode(bool activate) that sets an internal flag that a controller is used by another controller (in chained mode) and calls virtual void on_set_chained_mode(bool activate) = 0 that implements controller’s specific actions when chained modes is activated or deactivated, e.g., deactivating subscribers.

As an example, PID controllers export one virtual interface pid_reference and stop their subscriber <controller_name>/pid_reference when used in chained mode. ‘diff_drive_controller’ controller exports list of virtual interfaces <controller_name>/v_x, <controller_name>/v_y, and <controller_name>/w_z, and stops subscribers from topics <controller_name>/cmd_vel and <controller_name>/cmd_vel_unstamped. Its publishers can continue running.

Inner Resource Management
After configuring a chainable controller, controller manager calls input_interface_configuration method and takes ownership over controller’s input interfaces. This is the same process as done by ResourceManager and hardware interfaces. Controller manager maintains “claimed” status of interface in a vector (the same as done in ResourceManager).

Activation and Deactivation Chained Controllers
Chained controllers must be activated and deactivated together or in the proper order. This means you must first activate all following controllers to have the preceding one activated. For the deactivation there is the inverse rule - all preceding controllers have to be deactivated before the following controller is deactivated. One can also think of it as an actual chain, you can not add a chain link or break the chain in the middle.

Debugging outputs
Flag unavailable if the reference interface does not provide much information about anything at the moment. So don’t get confused by it. The reason we have it are internal implementation reasons irrelevant for the usage.

Closing remarks
Maybe addition of the new controller’s type ChainableController is not necessary. It would also be feasible to add an implementation of input_interface_configuration() method into ControllerInterface class with default result interface_configuration_type::NONE.

Joint Kinematics for ros2_control
This page should give an overview of the joint kinematics in the context of ros2_control. It is intended to give a brief introduction to the topic and to explain the current implementation in ros2_control.

Nomenclature
Degrees of Freedom (DoF)
From wikipedia:

In physics, the degrees of freedom (DoF) of a mechanical system is the number of independent parameters that define its configuration or state.

Joint
A joint is a connection between two links. In the ROS ecosystem, three types are more typical: Revolute (A hinge joint with position limits), Continuous (A continuous hinge without any position limits) or Prismatic (A sliding joint that moves along the axis).

In general, a joint can be actuated or non-actuated, also called passive. Passive joints are joints that do not have their own actuation mechanism but instead allow movement by external forces or by being passively moved by other joints. A passive joint can have a DoF of one, such as a pendulum, or it can be part of a parallel kinematic mechanism with zero DoF.

Serial Kinematics
Serial kinematics refers to the arrangement of joints in a robotic manipulator where each joint is independent of the others, and the number of joints is equal to the DoF of the kinematic chain.

A typical example is an industrial robot with six revolute joints, having 6-DoF. Each joint can be actuated independently, and the end-effector can be moved to any position and orientation in the workspace.

Kinematic Loops
On the other hand, kinematic loops, also known as closed-loop mechanisms, involve several joints that are connected in a kinematic chain and being actuated together. This means that the joints are coupled and cannot be moved independently: In general, the number of DoFs is smaller than the number of joints. This structure is typical for parallel kinematic mechanisms, where the end-effector is connected to the base by several kinematic chains.

An example is the four-bar linkage, which consists of four links and four joints. It can have one or two actuators and consequently one or two DoFs, despite having four joints. Furthermore, we can say that we have one (two) actuated joint and three (two) passive joints, which must satisfy the kinematic constraints of the mechanism.

URDF
URDF is the default format to describe robot kinematics in ROS. However, only serial kinematic chains are supported, except for the so-called mimic joints. See the URDF specification for more details.

Mimic joints can be defined in the following way in the URDF

<joint name="right_finger_joint" type="prismatic">
  <axis xyz="0 1 0"/>
  <origin xyz="0.0 -0.48 1" rpy="0.0 0.0 0.0"/>
  <parent link="base"/>
  <child link="finger_right"/>
  <limit effort="1000.0" lower="0" upper="0.38" velocity="10"/>
</joint>
<joint name="left_finger_joint" type="prismatic">
  <mimic joint="right_finger_joint" multiplier="1" offset="0"/>
  <axis xyz="0 1 0"/>
  <origin xyz="0.0 0.48 1" rpy="0.0 0.0 3.1415926535"/>
  <parent link="base"/>
  <child link="finger_left"/>
  <limit effort="1000.0" lower="0" upper="0.38" velocity="10"/>
</joint>
Mimic joints are an abstraction of the real world. For example, they can be used to describe

simple closed-loop kinematics with linear dependencies of the joint positions and velocities

links connected with belts, like belt and pulley systems or telescope arms

a simplified model of passive joints, e.g. a pendulum at the end-effector always pointing downwards

abstract complex groups of actuated joints, where several joints are directly controlled by low-level control loops and move synchronously. Without giving a real-world example, this could be several motors with their individual power electronics but commanded with the same setpoint.

Mimic joints defined in the URDF are parsed from the resource manager and stored in a class variable of type HardwareInfo, which can be accessed by the hardware components. The mimic joints must not have command interfaces but can have state interfaces.

<ros2_control>
  <joint name="right_finger_joint">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  <joint name="left_finger_joint">
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
From the officially released packages, the following packages are already using this information:

mock_components (generic system)

gazebo_ros2_control

ign_ros2_control

As the URDF specifies only the kinematics, the mimic tag has to be independent of the hardware interface type used in ros2_control. This means that we interpret this info in the following way:

position = multiplier * other_joint_position + offset

velocity = multiplier * other_joint_velocity

If someone wants to deactivate the mimic joint behavior for whatever reason without changing the URDF, it can be done by setting the attribute mimic=false of the joint tag in the <ros2_control> section of the XML.

<joint name="left_finger_joint" mimic="false">
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>
</joint>
Transmission Interface
Mechanical transmissions transform effort/flow variables such that their product (power) remains constant. Effort variables for linear and rotational domains are force and torque; while the flow variables are respectively linear velocity and angular velocity.

In robotics it is customary to place transmissions between actuators and joints. This interface adheres to this naming to identify the input and output spaces of the transformation. The provided interfaces allow bidirectional mappings between actuator and joint spaces for effort, velocity and position. Position is not a power variable, but the mappings can be implemented using the velocity map plus an integration constant representing the offset between actuator and joint zeros.

The transmission_interface provides a base class and some implementations for plugins, which can be integrated and loaded by custom hardware components. They are not automatically loaded by any hardware component or the gazebo plugins, each hardware component is responsible for loading the appropriate transmission interface to map the actuator readings to joint readings.

Currently the following implementations are available:

SimpleTransmission: A simple transmission with a constant reduction ratio and no additional dynamics.

DifferentialTransmission: A differential transmission with two actuators and two joints.

FourBarLinkageTransmission: A four-bar-linkage transmission with two actuators and two joints.

For more information, see example_8 or the transmission_interface documentation.

Simulating Closed-Loop Kinematic Chains
Depending on the simulation plugin, different approaches can be used to simulate closed-loop kinematic chains. The following list gives an overview of the available simulation plugins and their capabilities:

gazebo_ros2_control:
mimic joints

closed-loop kinematics are supported with <gazebo> tags in the URDF, see, e.g., here.

gz_ros2_control:
mimic joints

closed-loop kinematics are not directly supported yet, but can be implemented by using a DetachableJoint via custom plugins. Follow this issue for updates on this topic.

