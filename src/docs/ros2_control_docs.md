Writing a Hardware Component
In ros2_control hardware system components are libraries, dynamically loaded by the controller manager using the pluginlib interface. The following is a step-by-step guide to create source files, basic tests, and compile rules for a new hardware interface.

Preparing package

If the package for the hardware interface does not exist, then create it first. The package should have ament_cmake as a build type. The easiest way is to search online for the most recent manual. A helpful command to support this process is ros2 pkg create. Use the --help flag for more information on how to use it. There is also an option to create library source files and compile rules to help you in the following steps.

Preparing source files

After creating the package, you should have at least CMakeLists.txt and package.xml files in it. Create also include/<PACKAGE_NAME>/ and src folders if they do not already exist. In include/<PACKAGE_NAME>/ folder add <robot_hardware_interface_name>.hpp and <robot_hardware_interface_name>.cpp in the src folder. Optionally add visibility_control.h with the definition of export rules for Windows. You can copy this file from an existing controller package and change the name prefix to the <PACKAGE_NAME>.

Adding declarations into header file (.hpp)

Take care that you use header guards. ROS2-style is using #ifndef and #define preprocessor directives. (For more information on this, a search engine is your friend :) ).

Include "hardware_interface/$interface_type$_interface.hpp" and visibility_control.h if you are using one. $interface_type$ can be Actuator, Sensor or System depending on the type of hardware you are using. for more details about each type check Hardware Components description.

Define a unique namespace for your hardware_interface. This is usually the package name written in snake_case.

Define the class of the hardware_interface, extending $InterfaceType$Interface, e.g., .. code:: c++ class HardwareInterfaceName : public hardware_interface::$InterfaceType$Interface

5. Add a constructor without parameters and the following public methods implementing LifecycleNodeInterface: on_configure, on_cleanup, on_shutdown, on_activate, on_deactivate, on_error; and overriding $InterfaceType$Interface definition: on_init, export_state_interfaces, export_command_interfaces, prepare_command_mode_switch (optional), perform_command_mode_switch (optional), read, write. For further explanation of hardware-lifecycle check the pull request and for exact definitions of methods check the "hardware_interface/$interface_type$_interface.hpp" header or doxygen documentation for Actuator, Sensor or System.

Adding definitions into source file (.cpp)

Include the header file of your hardware interface and add a namespace definition to simplify further development.

Implement on_init method. Here, you should initialize all member variables and process the parameters from the info argument. In the first line usually the parents on_init is called to process standard values, like name. This is done using: hardware_interface::(Actuator|Sensor|System)Interface::on_init(info). If all required parameters are set and valid and everything works fine return CallbackReturn::SUCCESS or return CallbackReturn::ERROR otherwise.

Write the on_configure method where you usually setup the communication to the hardware and set everything up so that the hardware can be activated.

Implement on_cleanup method, which does the opposite of on_configure.

Implement export_state_interfaces and export_command_interfaces methods where interfaces that hardware offers are defined. For the Sensor-type hardware interface there is no export_command_interfaces method. As a reminder, the full interface names have structure <joint_name>/<interface_type>.

(optional) For Actuator and System types of hardware interface implement prepare_command_mode_switch and perform_command_mode_switch if your hardware accepts multiple control modes.

Implement the on_activate method where hardware “power” is enabled.

Implement the on_deactivate method, which does the opposite of on_activate.

Implement on_shutdown method where hardware is shutdown gracefully.

Implement on_error method where different errors from all states are handled.

Implement the read method getting the states from the hardware and storing them to internal variables defined in export_state_interfaces.

Implement write method that commands the hardware based on the values stored in internal variables defined in export_command_interfaces.

IMPORTANT: At the end of your file after the namespace is closed, add the PLUGINLIB_EXPORT_CLASS macro.

For this you will need to include the "pluginlib/class_list_macros.hpp" header. As first parameters you should provide exact hardware interface class, e.g., <my_hardware_interface_package>::<RobotHardwareInterfaceName>, and as second the base class, i.e., hardware_interface::(Actuator|Sensor|System)Interface.

Writing export definition for pluginlib

Create the <my_hardware_interface_package>.xml file in the package and add a definition of the library and hardware interface’s class which has to be visible for the pluginlib. The easiest way to do that is to check definition for mock components in the hardware_interface mock_components section.

Usually, the plugin name is defined by the package (namespace) and the class name, e.g., <my_hardware_interface_package>/<RobotHardwareInterfaceName>. This name defines the hardware interface’s type when the resource manager searches for it. The other two parameters have to correspond to the definition done in the macro at the bottom of the <robot_hardware_interface_name>.cpp file.

Writing a simple test to check if the controller can be found and loaded

Create the folder test in your package, if it does not exist already, and add a file named test_load_<robot_hardware_interface_name>.cpp.

You can copy the load_generic_system_2dof content defined in the test_generic_system.cpp package.

Change the name of the copied test and in the last line, where hardware interface type is specified put the name defined in <my_hardware_interface_package>.xml file, e.g., <my_hardware_interface_package>/<RobotHardwareInterfaceName>.

Add compile directives into ``CMakeLists.txt`` file

Under the line find_package(ament_cmake REQUIRED) add further dependencies. Those are at least: hardware_interface, pluginlib, rclcpp and rclcpp_lifecycle.

Add a compile directive for a shared library providing the <robot_hardware_interface_name>.cpp file as the source.

Add targeted include directories for the library. This is usually only include.

Add ament dependencies needed by the library. You should add at least those listed under 1.

Export for pluginlib description file using the following command: .. code:: cmake

pluginlib_export_plugin_description_file(hardware_interface <my_hardware_interface_package>.xml)

Add install directives for targets and include directory.

In the test section add the following dependencies: ament_cmake_gmock, hardware_interface.

Add compile definitions for the tests using the ament_add_gmock directive. For details, see how it is done for mock hardware in the ros2_control package.

(optional) Add your hardware interface`s library into ament_export_libraries before ament_package().

Add dependencies into ``package.xml`` file

Add at least the following packages into <depend> tag: hardware_interface, pluginlib, rclcpp, and rclcpp_lifecycle.

Add at least the following package into <test_depend> tag: ament_add_gmock and hardware_interface.

Compiling and testing the hardware component

Now everything is ready to compile the hardware component using the colcon build <my_hardware_interface_package> command. Remember to go into the root of your workspace before executing this command.

If compilation was successful, source the setup.bash file from the install folder and execute colcon test <my_hardware_interface_package> to check if the new controller can be found through pluginlib library and be loaded by the controller manager.

That’s it! Enjoy writing great controllers!


Different update rates for Hardware Components
In the following sections you can find some advice which will help you to implement Hardware Components with update rates different from the main control loop.

By counting loops
Current implementation of ros2_control main node has one update rate that controls the rate of the read(…) and write(…) calls in hardware_interface(s). To achieve different update rates for your hardware component you can use the following steps:

Add parameters of main control loop update rate and desired update rate for your hardware component

<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="system_interface" params="name main_loop_update_rate desired_hw_update_rate">

    <ros2_control name="${name}" type="system">
      <hardware>
          <plugin>my_system_interface/MySystemHardware</plugin>
          <param name="main_loop_update_rate">${main_loop_update_rate}</param>
          <param name="desired_hw_update_rate">${desired_hw_update_rate}</param>
      </hardware>
      ...
    </ros2_control>

  </xacro:macro>

</robot>
In you on_init() specific implementation fetch the desired parameters

namespace my_system_interface
{
hardware_interface::CallbackReturn MySystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  //   declaration in *.hpp file --> unsigned int main_loop_update_rate_, desired_hw_update_rate_ = 100 ;
  main_loop_update_rate_ = stoi(info_.hardware_parameters["main_loop_update_rate"]);
  desired_hw_update_rate_ = stoi(info_.hardware_parameters["desired_hw_update_rate"]);

  ...
}
...
} // my_system_interface
In your on_activate specific implementation reset internal loop counter

hardware_interface::CallbackReturn MySystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    //   declaration in *.hpp file --> unsigned int update_loop_counter_ ;
    update_loop_counter_ = 0;
    ...
}
In your read(const rclcpp::Time & time, const rclcpp::Duration & period) and/or write(const rclcpp::Time & time, const rclcpp::Duration & period) specific implementations decide if you should interfere with your hardware

hardware_interface::return_type MySystemHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{

  bool hardware_go = main_loop_update_rate_ == 0  ||
                    desired_hw_update_rate_ == 0 ||
                    ((update_loop_counter_ % desired_hw_update_rate_) == 0);

  if (hardware_go){
    // hardware comms and operations
    ...
  }
  ...

  // update counter
  ++update_loop_counter_;
  update_loop_counter_ %= main_loop_update_rate_;
}
By measuring elapsed time
Another way to decide if hardware communication should be executed in the read(const rclcpp::Time & time, const rclcpp::Duration & period) and/or write(const rclcpp::Time & time, const rclcpp::Duration & period) implementations is to measure elapsed time since last pass:

In your on_activate specific implementation reset the flags that indicate that this is the first pass of the read and write methods

hardware_interface::CallbackReturn MySystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    //   declaration in *.hpp file --> bool first_read_pass_, first_write_pass_ = true ;
    first_read_pass_ = first_write_pass_ = true;
    ...
}
In your read(const rclcpp::Time & time, const rclcpp::Duration & period) and/or write(const rclcpp::Time & time, const rclcpp::Duration & period) specific implementations decide if you should interfere with your hardware

hardware_interface::return_type MySystemHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (first_read_pass_ || (time - last_read_time_ ) > desired_hw_update_period_)
    {
      first_read_pass_ = false;
      //   declaration in *.hpp file --> rclcpp::Time last_read_time_ ;
      last_read_time_ = time;
      // hardware comms and operations
      ...
    }
    ...
}

hardware_interface::return_type MySystemHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (first_write_pass_ || (time - last_write_time_ ) > desired_hw_update_period_)
    {
      first_write_pass_ = false;
      //   declaration in *.hpp file --> rclcpp::Time last_write_time_ ;
      last_write_time_ = time;
      // hardware comms and operations
      ...
    }
    ...
}
Note

The approach to get the desired update period value from the URDF and assign it to the variable desired_hw_update_period_ is the same as in the previous section (step 1 and step 2) but with a different parameter name.

Debugging
All controllers and hardware components are plugins loaded into the controller_manager. Therefore, the debugger must be attached to the controller_manager. If multiple controller_manager instances are running on your robot or machine, you need to attach the debugger to the controller_manager associated with the hardware component or controller you want to debug.

How-To
Install xterm, gdb and gdbserver on your system

sudo apt install xterm gdb gdbserver
Make sure you run a “debug” or “release with debug information” build: This is done by passing --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo to colcon build. Remember that in release builds some breakpoints might not behave as you expect as the the corresponding line might have been optimized by the compiler. For such cases, a full Debug build (--cmake-args -DCMAKE_BUILD_TYPE=Debug) is recommended.

Adapt the launch file to run the controller manager with the debugger attached:

Version A: Run it directly with the gdb CLI:

Add prefix=['xterm -e gdb -ex run --args'] to the controller_manager node entry in your launch file. Due to how ros2launch works we need to run the specific node in a separate terminal instance.

Version B: Run it with gdbserver:

Add prefix=['gdbserver localhost:3000'] to the controller_manager node entry in your launch file. Afterwards, you can either attach a gdb CLI instance or any IDE of your choice to that gdbserver instance. Ensure you start your debugger from a terminal where you have sourced your workspace to properly resolve all paths.

Example launch file entry:

# Obtain the controller config file for the ros2 control node
controller_config_file = get_package_file("<package name>", "config/controllers.yaml")

controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[controller_config_file],
    output="both",
    emulate_tty=True,
    remappings=[
        ("~/robot_description", "/robot_description")
    ],
    prefix=['xterm -e gdb -ex run --args']  # or prefix=['gdbserver localhost:3000']
)

ld.add_action(controller_manager)
Additional notes
Debugging plugins

You can only set breakpoints in plugins after the plugin has been loaded. In the ros2_control context this means after the controller / hardware component has been loaded:

Debug builds

It’s often practical to include debug information only for the specific package you want to debug. colcon build --packages-select [package_name] --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo or colcon build --packages-select [package_name] --cmake-args -DCMAKE_BUILD_TYPE=Debug

Realtime

Warning

The update/on_activate/on_deactivate method of a controller and the read/write/on_activate/perform_command_mode_switch methods of a hardware component all run in the context of the realtime update loop. Setting breakpoints there can and will cause issues that might even break your hardware in the worst case.

From experience, it might be better to use meaningful logs for the real-time context (with caution) or to add additional debug state interfaces (or publishers in the case of a controller).

However, running the controller_manager and your plugin with gdb can still be very useful for debugging errors such as segfaults, as you can gather a full backtrace.

