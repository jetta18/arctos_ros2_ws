ROS 2 nodes are the main participants on ROS 2 ecosystem. They will communicate between each other using publishers, subscriptions, services, etc. Further information about ROS 2 nodes can be found here

    Initialization
        Cleaning Up
    Lifecycle
        Initialization
        Callbacks
        Running
        Cleaning Up
        Limitations

Initialization

    Create a node with default configuration:

    // Initialize micro-ROS allocator
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // Initialize support object
    rclc_support_t support;
    rcl_ret_t rc = rclc_support_init(&support, argc, argv, &allocator);

    // Create node object
    rcl_node_t node;
    const char * node_name = "test_node";

    // Node namespace (Can remain empty "")
    const char * namespace = "test_namespace";

    // Init default node
    rc = rclc_node_init_default(&node, node_name, namespace, &support);
    if (rc != RCL_RET_OK) {
      ... // Handle error
      return -1;
    }

    Create a node with custom options:

    The configuration of the node will also be applied to its future elements (Publishers, subscribers, services, …). The node options are configured on the rclc_support_t object with a custom API:

    // Initialize micro-ROS allocator
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // Initialize and modify options (Set DOMAIN ID to 10)
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 10);

    // Initialize rclc support object with custom options
    rclc_support_t support;
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    // Create node object
    rcl_node_t node;
    const char * node_name = "test_node";

    // Node namespace (Can remain empty "")
    const char * namespace = "test_namespace";

    // Init node with configured support object
    rclc_node_init_default(&node, node_name, namespace, &support);

    if (rc != RCL_RET_OK) {
      ... // Handle error
      return -1;
    }

Cleaning Up

To destroy a initialized node all entities owned by the node (Publishers, subscribers, services, …) have to be destroyed before the node itself:

// Destroy created entities (Example)
rcl_publisher_fini(&publisher, &node);
...

// Destroy the node
rcl_node_fini(&node);

This will delete the node from ROS2 graph, including any generated infrastructure on the agent (if possible) and used memory on the client.
Lifecycle

The rclc lifecycle package provides convenience functions in C to bundle an rcl node with the ROS 2 Node Lifecycle state machine, similar to the rclcpp Lifecycle Node for C++. Further information about ROS 2 node lifecycle can be found here

An usage example is given in the rclc_examples package.
Initialization

Creation of a lifecycle node as a bundle of an rcl node and the rcl lifecycle state machine. Assuming an already initialized node and executor:

// Create rcl state machine
rcl_lifecycle_state_machine_t state_machine =
rcl_lifecycle_get_zero_initialized_state_machine();

// Create the lifecycle node
rclc_lifecycle_node_t my_lifecycle_node;
rcl_ret_t rc = rclc_make_node_a_lifecycle_node(
  &my_lifecycle_node,
  &node,
  &state_machine,
  &allocator);

// Register lifecycle services on the allocator
rclc_lifecycle_add_get_state_service(&lifecycle_node, &executor);
rclc_lifecycle_add_get_available_states_service(&lifecycle_node, &executor);
rclc_lifecycle_add_change_state_service(&lifecycle_node, &executor);

Note: Executor needsto be equipped with 1 handle per node and per service
Callbacks

Optional callbacks are supported to act on lifecycle state changes. Example:

rcl_ret_t my_on_configure() {
  printf("  >>> my_lifecycle_node: on_configure() callback called.\n");
  return RCL_RET_OK;
}

To add them to the lifecycle node:

// Register lifecycle service callbacks
rclc_lifecycle_register_on_configure(&lifecycle_node, &my_on_configure);
rclc_lifecycle_register_on_activate(&lifecycle_node, &my_on_activate);
rclc_lifecycle_register_on_deactivate(&lifecycle_node, &my_on_deactivate);
rclc_lifecycle_register_on_cleanup(&lifecycle_node, &my_on_cleanup);

Running

To change states of the lifecycle node:

bool publish_transition = true;
rc += rclc_lifecycle_change_state(
  &my_lifecycle_node,
  lifecycle_msgs__msg__Transition__TRANSITION_CONFIGURE,
  publish_transition);

rc += rclc_lifecycle_change_state(
  &my_lifecycle_node,
  lifecycle_msgs__msg__Transition__TRANSITION_ACTIVATE,
  publish_transition);

Except for error processing transitions, transitions are usually triggered from outside, e.g., by ROS 2 services.
Cleaning Up

To clean everything up, simply do

rc += rcl_lifecycle_node_fini(&my_lifecycle_node, &allocator);

Limitations

Lifecycle services cannot yet be called via ros2 lifecycle client (ros2 lifecycle set /node ...). Instead use the ros2 service CLI, (Example: ros2 service call /node/change_state lifecycle_msgs/ChangeState "{transition: {id: 1, label: configure}}").




Publishers and subscribers

ROS 2 publishers and subscribers are the basic communication mechanism between nodes using topics. Further information about ROS 2 publish–subscribe pattern can be found here.

Ready to use code related to this concepts can be found in micro-ROS-demos/rclc/int32_publisher and micro-ROS-demos/rclc/int32_subscriber folders. Fragments of code from this examples are used on this tutorial.

    Publisher
        Initialization
        Publish a message
    Subscription
        Initialization
        Callbacks
    Message initialization
    Cleaning Up

Publisher
Initialization

Starting from a code where RCL is initialized and a micro-ROS node is created, there are three ways to initialize a publisher depending on the desired quality-of-service configuration:

    Reliable (default):

    // Publisher object
    rcl_publisher_t publisher;
    const char * topic_name = "test_topic";

    // Get message type support
    const rosidl_message_type_support_t * type_support =
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

    // Creates a reliable rcl publisher
    rcl_ret_t rc = rclc_publisher_init_default(
      &publisher, &node,
      type_support, topic_name);

    if (RCL_RET_OK != rc) {
      ...  // Handle error
      return -1;
    }

    Best effort:

    // Publisher object
    rcl_publisher_t publisher;
    const char * topic_name = "test_topic";

    // Get message type support
    const rosidl_message_type_support_t * type_support =
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

    // Creates a best effort rcl publisher
    rcl_ret_t rc = rclc_publisher_init_best_effort(
      &publisher, &node,
      type_support, topic_name);

    if (RCL_RET_OK != rc) {
      ...  // Handle error
      return -1;
    }

    Custom QoS:

    // Publisher object
    rcl_publisher_t publisher;
    const char * topic_name = "test_topic";

    // Get message type support
    const rosidl_message_type_support_t * type_support =
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

    // Set publisher QoS
    const rmw_qos_profile_t * qos_profile = &rmw_qos_profile_default;

    // Creates a rcl publisher with customized quality-of-service options
    rcl_ret_t rc = rclc_publisher_init(
      &publisher, &node,
      type_support, topic_name, qos_profile);

    if (RCL_RET_OK != rc) {
      ...  // Handle error
      return -1;
    }

    For a detail on the available QoS options and the advantages and disadvantages between reliable and best effort modes, check the QoS tutorial.

Publish a message

To publish messages to the topic:

// Int32 message object
std_msgs__msg__Int32 msg;

// Set message value
msg.data = 0;

// Publish message
rcl_ret_t rc = rcl_publish(&publisher, &msg, NULL);

if (rc != RCL_RET_OK) {
  ...  // Handle error
  return -1;
}

For periodic publications, rcl_publish can be placed inside a timer callback. Check the Executor and timers section for details.

Note: rcl_publish is thread safe and can be called from multiple threads.
Subscription
Initialization

The suscriptor initialization is almost identical to the publisher one:

    Reliable (default):

    // Subscription object
    rcl_subscription_t subscriber;
    const char * topic_name = "test_topic";

    // Get message type support
    const rosidl_message_type_support_t * type_support =
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

    // Initialize a reliable subscriber
    rcl_ret_t rc = rclc_subscription_init_default(
      &subscriber, &node,
      type_support, topic_name);

    if (RCL_RET_OK != rc) {
      ...  // Handle error
      return -1;
    }

    Best effort:

    // Subscription object
    rcl_subscription_t subscriber;
    const char * topic_name = "test_topic";

    // Get message type support
    const rosidl_message_type_support_t * type_support =
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

    // Initialize best effort subscriber
    rcl_ret_t rc = rclc_subscription_init_best_effort(
      &subscriber, &node,
      type_support, topic_name);

    if (RCL_RET_OK != rc) {
      ...  // Handle error
      return -1;
    }

    Custom QoS:

    // Subscription object
    rcl_subscription_t subscriber;
    const char * topic_name = "test_topic";

    // Get message type support
    const rosidl_message_type_support_t * type_support =
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

    // Set client QoS
    const rmw_qos_profile_t * qos_profile = &rmw_qos_profile_default;

    // Initialize a subscriber with customized quality-of-service options
    rcl_ret_t rc = rclc_subscription_init(
      &subscriber, &node,
      type_support, topic_name, qos_profile);

    if (RCL_RET_OK != rc) {
      ...  // Handle error
      return -1;
    }

For a detail on the available QoS options and the advantages and disadvantages between reliable and best effort modes, check the QoS tutorial.
Callbacks

The executor is responsible to call the configured callback when a message is published. The function will have the message as its only argument, containing the values sent by the publisher:

// Function prototype:
void (* rclc_subscription_callback_t)(const void *);

// Implementation example:
void subscription_callback(const void * msgin)
{
  // Cast received message to used type
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

  // Process message
  printf("Received: %d\n", msg->data);
}

Once the subscriber and the executor are initialized, the subscriber callback must be added to the executor to receive incoming publications once its spinning:

// Message object to receive publisher data
std_msgs__msg__Int32 msg;

// Add subscription to the executor
rcl_ret_t rc = rclc_executor_add_subscription(
  &executor, &subscriber, &msg,
  &subscription_callback, ON_NEW_DATA);

if (RCL_RET_OK != rc) {
  ...  // Handle error
  return -1;
}

// Spin executor to receive messages
rclc_executor_spin(&executor);

Message initialization

Before publishing or receiving a message, it may be necessary to initialize its memory for types with strings or sequences. Check the Handling messages memory in micro-ROS section for details.
Cleaning Up

After finishing the publisher/subscriber, the node will no longer be advertising that it is publishing/listening on the topic. To destroy an initialized publisher or subscriber:

// Destroy publisher
rcl_publisher_fini(&publisher, &node);

// Destroy subscriber
rcl_subscription_fini(&subscriber, &node);

This will delete any automatically created infrastructure on the agent (if possible) and deallocate used memory on the client side.



Services

ROS 2 services are another communication mechanism between nodes. Services implement a client-server paradigm based on ROS 2 messages and types. Further information about ROS 2 services can be found here

Ready to use code related to this concepts can be found in micro-ROS-demos/rclc/addtwoints_server and micro-ROS-demos/rclc/addtwoints_client folders. Fragments of code from this examples are used on this tutorial.

    Service server
        Initialization
        Callback
    Service Client
        Initialization
        Callback
        Send a request
    Message initialization
    Cleaning Up

Service server
Initialization

Starting from a code where RCL is initialized and a micro-ROS node is created, there are three ways to initialize a service server:

    Reliable (default):

    // Service server object
    rcl_service_t service;
    const char * service_name = "/addtwoints";

    // Get message type support
    const rosidl_message_type_support_t * type_support =
      ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

    // Initialize server with default configuration
    rcl_ret_t rc = rclc_service_init_default(
      &service, &node,
      type_support, service_name);

    if (rc != RCL_RET_OK) {
      ...  // Handle error
      return -1;
    }

    Best effort:

    // Service server object
    rcl_service_t service;
    const char * service_name = "/addtwoints";

    // Get message type support
    const rosidl_message_type_support_t * type_support =
      ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

    // Initialize server with default configuration
    rcl_ret_t rc = rclc_service_init_best_effort(
      &service, &node,
      type_support, service_name);

    if (rc != RCL_RET_OK) {
      ...  // Handle error
      return -1;
    }

    Custom QoS:

    // Service server object
    rcl_service_t service;
    const char * service_name = "/addtwoints";

    // Get message type support
    const rosidl_message_type_support_t * type_support =
      ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

    // Set service QoS
    const rmw_qos_profile_t * qos_profile = &rmw_qos_profile_services_default;

    // Initialize server with customized quality-of-service options
    rcl_ret_t rc = rclc_service_init(
      &service, &node, type_support,
      service_name, qos_profile);

    if (rc != RCL_RET_OK) {
      ...  // Handle error
      return -1;
    }

For a detail on the available QoS options and the advantages and disadvantages between reliable and best effort modes, check the QoS tutorial.
Callback

Once a request arrives, the executor will call the configured callback with the request and response messages as arguments. The request message contains the values sent by the client, the response_msg should be modified here as it will be delivered after the callback returns.

Using AddTwoInts.srv type definition as an example:

int64 a
int64 b
---
int64 sum

The client request message will contain two integers a and b, and expects the sum of them as a response:

// Function prototype:
void (* rclc_service_callback_t)(const void *, void *);

// Implementation example:
void service_callback(const void * request_msg, void * response_msg){
  // Cast messages to expected types
  example_interfaces__srv__AddTwoInts_Request * req_in =
    (example_interfaces__srv__AddTwoInts_Request *) request_msg;
  example_interfaces__srv__AddTwoInts_Response * res_in =
    (example_interfaces__srv__AddTwoInts_Response *) response_msg;

  // Handle request message and set the response message values
  printf("Client requested sum of %d and %d.\n", (int) req_in->a, (int) req_in->b);
  res_in->sum = req_in->a + req_in->b;
}

Note that it is necessary to cast each message to the expected type

Once the service and the executor are initialized, the service callback must be added to the executor in order to process incoming requests once the executor is spinning:

// Service message objects
example_interfaces__srv__AddTwoInts_Response response_msg;
example_interfaces__srv__AddTwoInts_Request request_msg;

// Add server callback to the executor
rc = rclc_executor_add_service(
  &executor, &service, &request_msg,
  &response_msg, service_callback);

if (rc != RCL_RET_OK) {
  ...  // Handle error
  return -1;
}

// Spin executor to receive requests
rclc_executor_spin(&executor);

Service Client
Initialization

The service client initialization is almost identical to the server one:

    Reliable (default):

    // Service client object
    rcl_client_t client;
    const char * service_name = "/addtwoints";

    // Get message type support
    const rosidl_message_type_support_t * type_support =
      ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

    // Initialize client with default configuration
    rcl_ret_t rc = rclc_client_init_default(
      &client, &node,
      type_support, service_name);

    if (rc != RCL_RET_OK) {
      ...  // Handle error
      return -1;
    }

    Best effort:

    // Service client object
    rcl_client_t client;
    const char * service_name = "/addtwoints";

    // Get message type support
    const rosidl_message_type_support_t * type_support =
      ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

    // Initialize client with default configuration
    rcl_ret_t rc = rclc_client_init_best_effort(
      &client, &node,
      type_support, service_name);

    if (rc != RCL_RET_OK) {
      ...  // Handle error
      return -1;
    }

    Custom QoS:

    // Service client object
    rcl_client_t client;
    const char * service_name = "/addtwoints";

    // Get message type support
    const rosidl_message_type_support_t * type_support =
      ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

    // Set client QoS
    const rmw_qos_profile_t * qos_profile = &rmw_qos_profile_services_default;

    // Initialize server with customized quality-of-service options
    rcl_ret_t rc = rclc_client_init(
      &client, &node, type_support,
      service_name, qos_profile);

    if (rc != RCL_RET_OK) {
      ...  // Handle error
      return -1;
    }

Callback

The executor is responsible to call the configured callback when the service response arrives. The function will have the response message as its only argument, containing the values sent by the server.

It is necessary to cast the response message to the expected type. Example:

// Function prototype:
void (* rclc_client_callback_t)(const void *);

// Implementation example:
void client_callback(const void * response_msg){
  // Cast response message to expected type
  example_interfaces__srv__AddTwoInts_Response * msgin =
    (example_interfaces__srv__AddTwoInts_Response * ) response_msg;

  // Handle response message
  printf("Received service response %ld + %ld = %ld\n", req.a, req.b, msgin->sum);
}

Once the client and the executor are initialized, the client callback must be added to the executor in order to receive the service response once the executor is spinning:

// Response message object
example_interfaces__srv__AddTwoInts_Response res;

// Add client callback to the executor
rcl_ret_t rc = rclc_executor_add_client(&executor, &client, &res, client_callback);

if (rc != RCL_RET_OK) {
  ...  // Handle error
  return -1;
}

// Spin executor to receive requests
rclc_executor_spin(&executor);

Send a request

Once the service client and server are configured, the service client can perform a request and spin the executor to get the reply. Following the example on AddTwoInts.srv:

// Request message object (Must match initialized client type support)
example_interfaces__srv__AddTwoInts_Request request_msg;

// Initialize request message memory and set its values
example_interfaces__srv__AddTwoInts_Request__init(&request_msg);
request_msg.a = 24;
request_msg.b = 42;

// Sequence number of the request (Populated in rcl_send_request)
int64_t sequence_number;

// Send request
rcl_send_request(&client, &request_msg, &sequence_number);

// Spin the executor to get the response
rclc_executor_spin(&executor);

Message initialization

Before sending or receiving a message, it may be necessary to initialize its memory for types with strings or sequences. Check the Handling messages memory in micro-ROS section for details.
Cleaning Up

To destroy an initialized service or client:

// Destroy service server and client
rcl_service_fini(&service, &node);
rcl_client_fini(&client, &node);

This will delete any automatically created infrastructure on the agent (if possible) and deallocate used memory on the client side.


Parameter server

ROS 2 parameters allow the user to create variables on a node and manipulate/read them with different ROS 2 commands. Further information about ROS 2 parameters can be found here

Ready to use code examples related to this tutorial can be found in rclc/rclc_examples/src/example_parameter_server.c.

    Initialization
    Memory requirements
    Add a parameter
    Delete a parameter
    Parameters description
    Callback
    Cleaning up

Initialization

    Default initialization:

      // Parameter server object
      rclc_parameter_server_t param_server;

      // Initialize parameter server with default configuration
      rcl_ret_t rc = rclc_parameter_server_init_default(&param_server, &node);

      if (RCL_RET_OK != rc) {
          ... // Handle error
          return -1;
      }

    Initialization with custom options:

    The following options can be configured:
        notify_changed_over_dds: Publish parameter events to other ROS 2 nodes as well.
        max_params: Maximum number of parameters allowed on the rclc_parameter_server_t object.
        allow_undeclared_parameters: Allows creation of parameters from external parameter clients. A new parameter will be created if a set operation is requested on a non-existing parameter.

        low_mem_mode: Reduces the memory used by the parameter server, functionality constrains are applied.

        // Parameter server object
        rclc_parameter_server_t param_server;

        // Initialize parameter server options
        const rclc_parameter_options_t options = {
            .notify_changed_over_dds = true,
            .max_params = 4,
            .allow_undeclared_parameters = true,
            .low_mem_mode = false; };

        // Initialize parameter server with configured options
        rcl_ret_t rc = rclc_parameter_server_init_with_option(&param_server, &node, &options);

        if (RCL_RET_OK != rc) {
          ... // Handle error
          return -1;
        }

    Low memory mode:

    This mode ports the parameter functionality to memory constrained devices. The following constrains are applied:
        Request size limited to one parameter on Set, Get, Get types and Describe services.
        List parameter request has no prefixes enabled nor depth.
        Parameter description strings not allowed, rclc_add_parameter_description is disabled.

    Memory benchmark results on STM32F4 for 7 parameters with RCLC_PARAMETER_MAX_STRING_LENGTH = 50 and notify_changed_over_dds = true:
        Full mode: 11736 B
        Low memory mode: 4160 B

Memory requirements

The parameter server uses five services and an optional publisher. These need to be taken into account on the rmw-microxrcedds package memory configuration:

# colcon.meta example with memory requirements to use a parameter server
{
    "names": {
        "rmw_microxrcedds": {
            "cmake-args": [
                "-DRMW_UXRCE_MAX_NODES=1",
                "-DRMW_UXRCE_MAX_PUBLISHERS=1",
                "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=0",
                "-DRMW_UXRCE_MAX_SERVICES=5",
                "-DRMW_UXRCE_MAX_CLIENTS=0"
            ]
        }
    }
}

At runtime, the variable RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER defines the necessary number of handles required by a parameter server for the rclc Executor:

// Executor init example with the minimum RCLC executor handles required
rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
rc = rclc_executor_init(
    &executor, &support.context,
    RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER, &allocator);

Humble: the variable is RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER has been renamed to RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES.
Add a parameter

The micro-ROS parameter server supports the following parameter types:

    Boolean:

      const char * parameter_name = "parameter_bool";
      bool param_value = true;

      // Add parameter to the server
      rcl_ret_t rc = rclc_add_parameter(&param_server, parameter_name, RCLC_PARAMETER_BOOL);

      // Set parameter value (Triggers `on_parameter_changed` callback, if defined)
      rc = rclc_parameter_set_bool(&param_server, parameter_name, param_value);

      // Get parameter value and store it in "param_value"
      rc = rclc_parameter_get_bool(&param_server, "param1", &param_value);

      if (RCL_RET_OK != rc) {
          ... // Handle error
          return -1;
      }

    Integer:

      const char * parameter_name = "parameter_int";
      int param_value = 100;

      // Add parameter to the server
      rcl_ret_t rc = rclc_add_parameter(&param_server, parameter_name, RCLC_PARAMETER_INT);

      // Set parameter value
      rc = rclc_parameter_set_int(&param_server, parameter_name, param_value);

      // Get parameter value on param_value
      rc = rclc_parameter_get_int(&param_server, parameter_name, &param_value);

    Double:

      const char * parameter_name = "parameter_double";
      double param_value = 0.15;

      // Add parameter to the server
      rcl_ret_t rc = rclc_add_parameter(&param_server, parameter_name, RCLC_PARAMETER_DOUBLE);

      // Set parameter value
      rc = rclc_parameter_set_double(&param_server, parameter_name, param_value);

      // Get parameter value on param_value
      rc = rclc_parameter_get_double(&param_server, parameter_name, &param_value);

Parameters can also be created by external clients if the allow_undeclared_parameters flag is set. The client just needs to set a value on a non-existing parameter. Then this parameter will be created if the server has still capacity and the callback allows the operation.

Max name size is controlled by the compile-time option RCLC_PARAMETER_MAX_STRING_LENGTH, default value is 50.
Delete a parameter

Parameters can be deleted by both, the parameter server and external clients:

rclc_delete_parameter(&param_server, "param2");

For external delete requests, the server callback will be executed, allowing the node to reject the operation.
Parameters description

    Parameter description Adds a description of a parameter and its constrains, which will be returned on a describe parameter requests:

      rclc_add_parameter_description(&param_server, "param2", "Second parameter", "Only even numbers");

    The maximum string size is controlled by the compilation time option RCLC_PARAMETER_MAX_STRING_LENGTH, default value is 50.
    Parameter constraints Informative numeric constraints can be added to int and double parameters, returning this values on describe parameter requests:
        from_value: Start value for valid values, inclusive.
        to_value: End value for valid values, inclusive.

        step: Size of valid steps between the from and to bound.

          int64_t int_from = 0;
          int64_t int_to = 20;
          uint64_t int_step = 2;
          rclc_add_parameter_constraint_integer(&param_server, "param2", int_from, int_to, int_step);

          double double_from = -0.5;
          double double_to = 0.5;
          double double_step = 0.01;
          rclc_add_parameter_constraint_double(&param_server, "param3", double_from, double_to, double_step);

    This constrains will not be applied by the parameter server, leaving values filtering to the user callback.
    Read-only parameters: The new API offers a read-only flag. This flag blocks parameter changes from external clients, but allows changes on the server side:

      bool read_only = true;
      rclc_set_parameter_read_only(&param_server, "param3", read_only);

Callback

When adding the parameter server to the executor, a callback could to be configured. This callback would then be executed on the following events:

    Parameter value change: Internal and external parameter set on existing parameters.
    Parameter creation: External parameter set on unexisting parameter if allow_undeclared_parameters is set.
    Parameter delete: External parameter delete on existing parameter.
    The user can allow or reject this operations using the bool return value.

Callback parameters:

    old_param: Parameter actual value, NULL for new parameter request.
    new_param: Parameter new value, NULL for parameter removal request.
    context: User context, configured on rclc_executor_add_parameter_server_with_context.

bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context)
{
  (void) context;

  if (old_param == NULL && new_param == NULL) {
    printf("Callback error, both parameters are NULL\n");
    return false;
  }

  if (old_param == NULL) {
    printf("Creating new parameter %s\n", new_param->name.data);
  } else if (new_param == NULL) {
    printf("Deleting parameter %s\n", old_param->name.data);
  } else {
    printf("Parameter %s modified.", old_param->name.data);
    switch (old_param->value.type) {
      case RCLC_PARAMETER_BOOL:
        printf(
          " Old value: %d, New value: %d (bool)", old_param->value.bool_value,
          new_param->value.bool_value);
        break;
      case RCLC_PARAMETER_INT:
        printf(
          " Old value: %ld, New value: %ld (int)", old_param->value.integer_value,
          new_param->value.integer_value);
        break;
      case RCLC_PARAMETER_DOUBLE:
        printf(
          " Old value: %f, New value: %f (double)", old_param->value.double_value,
          new_param->value.double_value);
        break;
      default:
        break;
    }
    printf("\n");
  }

  return true;
}

Parameters modifications are disabled while the callback on_parameter_changed is executed, causing the following methods to return RCLC_PARAMETER_DISABLED_ON_CALLBACK if they are invoked:

    rclc_add_parameter
    rclc_delete_parameter
    rclc_parameter_set_bool
    rclc_parameter_set_int
    rclc_parameter_set_double
    rclc_set_parameter_read_only
    rclc_add_parameter_constraint_double
    rclc_add_parameter_constraint_integer

Once the parameter server and the executor are initialized, the parameter server must be added to the executor in order to accept parameter commands from ROS 2:

// Add parameter server to the executor including defined callback
rc = rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed);

Note that this callback is optional as its just an event information for the user. To use the parameter server without a callback:

// Add parameter server to the executor without a callback
rc = rclc_executor_add_parameter_server(&executor, &param_server, NULL);

Configuration of the callback context:

// Add parameter server to the executor including defined callback and a context
rc = rclc_executor_add_parameter_server_with_context(&executor, &param_server, on_parameter_changed, &context);

Cleaning up

To destroy an initialized parameter server:

// Delete parameter server
rclc_parameter_server_fini(&param_server, &node);

This will delete any automatically created infrastructure on the agent (if possible) and deallocate used memory on the parameter server side.


Executor and timers

    Timers
        Initialization
        Callback
        Cleaning Up
    Executor
        Example 1: ‘Hello World’
        Example 2: Triggered execution

Timers

Timers can be created and added to the executor, which will call the timer callback periodically once it is spinning. They are usually used to handle periodic publications or events.
Initialization

// Timer period on nanoseconds
const unsigned int timer_period = RCL_MS_TO_NS(1000);

// Create and initialize timer object
rcl_timer_t timer;
rcl_ret_t rc = rclc_timer_init_default(&timer, &support, timer_period, timer_callback);

// Add to the executor
rc = rclc_executor_add_timer(&executor, &timer);

if (rc != RCL_RET_OK) {
  ... // Handle error
  return -1;
}

Callback

The callback gives a pointer to the associated timer and the time elapsed since the previous call or since the timer was created if it is the first call to the callback.

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	printf("Last callback time: %ld\n", last_call_time);

	if (timer != NULL) {
		// Perform actions
		...
	}
}

During the callback the timer can be canceled or have its period and/or callback modified using the passed pointer. Check rcl/timer.h for details.
Cleaning Up

To destroy an initialized timer:

// Destroy timer
rcl_timer_fini(&timer);

This will deallocate used memory and make the timer invalid
Executor

The rclc Executor provides a C API to manage the execution of subscription and timer callbacks, similar to the rclcpp Executor for C++. The rclc Executor is optimized for resource-constrained devices and provides additional features that allow the manual implementation of deterministic schedules with bounded end-to-end latencies.

In this section we provide two examples:

    Example 1: Hello-World example consisting of one executor and one publisher, timer and subscription.
    Example 2: Triggered execution example, demonstrating the capability of synchronizing the execution of callbacks based on the availability of new messages

Further information about the rclc Executor and its API can be found rclc repository, including further examples for using the rclc Executor in mobile robotics scenarios and real-time embedded applications.
Example 1: ‘Hello World’

To start with, we provide a very simple example for an rclc Executor with one timer and one subscription, so to say, a ‘Hello world’ example. It consists of a publisher, sending a ‘hello world’ message to a subscriber, which then prints out the received message on the console.

First, you include some header files, in particular the rclc/rclc.h and rclc/executor.h.

#include <stdio.h>
#include <std_msgs/msg/string.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

We define a publisher and two strings, which will be used later.

rcl_publisher_t my_pub;
std_msgs__msg__String pub_msg;
std_msgs__msg__String sub_msg;

The subscription callback casts the message parameter msgin to an equivalent type of std_msgs::msg::String in C and prints out the received message.

void my_subscriber_callback(const void * msgin)
{
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  if (msg == NULL) {
    printf("Callback: msg NULL\n");
  } else {
    printf("Callback: I heard: %s\n", msg->data.data);
  }
}

The timer callback publishes the message pub_msg with the publisher my_pub, which is initialized later in main().

void my_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  rcl_ret_t rc;
  UNUSED(last_call_time);
  if (timer != NULL) {
    rc = rcl_publish(&my_pub, &pub_msg, NULL);
    if (rc == RCL_RET_OK) {
      printf("Published message %s\n", pub_msg.data.data);
    } else {
      printf("Error in timer_callback: Message %s could not be published\n", pub_msg.data.data);
    }
  } else {
    printf("Error in timer_callback: timer parameter is NULL\n");
  }
}

After defining the callback functions, we present now the main() function. First, some initialization is necessary to create later rcl objects. That is an allocator for dynamic memory allocation, and a support object, which contains some rcl-objects simplifying the initialization of an rcl-node, an rcl-subscription, an rcl-timer and an rclc-executor.

int main(int argc, const char * argv[])
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_ret_t rc;

  // create init_options
  rc = rclc_support_init(&support, argc, argv, &allocator);
  if (rc != RCL_RET_OK) {
    printf("Error rclc_support_init.\n");
    return -1;
  }

Next, you define a ROS 2 node my_node and initialize it with rclc_executor_init_default():

  // create rcl_node
  rcl_node_t my_node;
  rc = rclc_node_init_default(&my_node, "node_0", "executor_examples", &support);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_node_init_default\n");
    return -1;
  }

You can create a publisher to publish topic ‘topic_0’ with type std_msg::msg::String with the following code:

const char * topic_name = "topic_0";
const rosidl_message_type_support_t * my_type_support =
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);

rc = rclc_publisher_init_default(&my_pub, &my_node, my_type_support, topic_name);
if (RCL_RET_OK != rc) {
  printf("Error in rclc_publisher_init_default %s.\n", topic_name);
  return -1;
}

Note, that variable my_pub was defined globally, so it can be used by the timer callback.

You can create a timer my_timer with a period of one second, which executes the callback my_timer_callback like this:

  rcl_timer_t my_timer;
  const unsigned int timer_timeout = 1000; // in ms
  rc = rclc_timer_init_default(&my_timer, &support, RCL_MS_TO_NS(timer_timeout), my_timer_callback);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_timer_init_default.\n");
    return -1;
  } else {
    printf("Created timer with timeout %d ms.\n", timer_timeout);
  }

The string Hello World! can be assigned directly to the message of the publisher pub_msg.data. First the publisher message is initialized with std_msgs__msg__String__init. Then you need to allocate memory for pub_msg.data.data, set the maximum capacity pub_msg.data.capacity and set the length of the message pub_msg.data.size accordingly. You can assign the content of the message with snprintf of pub_msg.data.data.

  // assign message to publisher
  std_msgs__msg__String__init(&pub_msg);
  const unsigned int PUB_MSG_CAPACITY = 20;
  pub_msg.data.data = malloc(PUB_MSG_CAPACITY);
  pub_msg.data.capacity = PUB_MSG_CAPACITY;
  snprintf(pub_msg.data.data, pub_msg.data.capacity, "Hello World!");
  pub_msg.data.size = strlen(pub_msg.data.data);

A subscription my_subcan be defined like this:

  rcl_subscription_t my_sub = rcl_get_zero_initialized_subscription();
  rc = rclc_subscription_init_default(&my_sub, &my_node, my_type_support, topic_name);
  if (rc != RCL_RET_OK) {
    printf("Failed to create subscriber %s.\n", topic_name);
    return -1;
  } else {
    printf("Created subscriber %s:\n", topic_name);
  }

The global message for this subscription sub_msg needs to be initialized with:

  std_msgs__msg__String__init(&sub_msg);

Now, all preliminary steps are done, and you can define and initialized the rclc executor with:

  rclc_executor_t executor;
  executor = rclc_executor_get_zero_initialized_executor();

In the next step, executor is initialized with the ROS 2 context, the number of communication objects num_handles and an allocator. The number of communication objects defines the total number of timers and subscriptions, the executor shall manage. In this example, the executor will be setup with one timer and one subscription.

  // total number of handles = #subscriptions + #timers
  unsigned int num_handles = 1 + 1;
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);

Now, you can add a subscription with the function rclc_c_executor_add_subscription with the previously defined subscription my_sub, its message sub_msgand its callback my_subscriber_callback:

rc = rclc_executor_add_subscription(&executor, &my_sub, &sub_msg, &my_subscriber_callback, ON_NEW_DATA);
if (rc != RCL_RET_OK) {
  printf("Error in rclc_executor_add_subscription. \n");
}

The option ON_NEW_DATA selects the execution semantics of the spin-method. In this example, the callback of the subscription my_subis only called if new data is available.

Note: Another execution semantics is ALWAYS, which means, that the subscription callback is always executed when the spin-method of the executor is called. This option might be useful in cases in which the callback shall be executed at a fixed rate irrespective of new data is available or not. If you choose this option, then the callback will be executed with message argument NULL if no new data is available. Therefore you need to make sure, that your callback also accepts NULL as message argument.

Likewise, you can add the timer my_timer with the function rclc_c_executor_add_timer:

rclc_executor_add_timer(&executor, &my_timer);
if (rc != RCL_RET_OK) {
  printf("Error in rclc_executor_add_timer.\n");
}

A key feature of the rclc Executor is that the order of these rclc-executor-add-*-functions matters. The order in which these functions are called defines the static processing order of the callbacks when the spin-function of the executor is running.

In this example, the timer was added to the executor before the subscription. Therefore, if the timer is ready and also a new message for the subscription is available, then the timer is executed first and after it the subscription. Such a behavior cannot be defined currently with the rclcpp Executor and is useful to implement a deterministic execution semantics.

Finally, you can run the executor with rclc_executor_spin():

  rclc_executor_spin(&executor);

This function runs forever without coming back. In this example, however, we want to publish the message only ten times. Therefore we are using the spin-method rclc_executor_spin_some, which spins only once and returns. The wait timeout for checking for new messages at the DDS-queue or waiting timers to get ready is configured to be one second.

for (unsigned int i = 0; i < 10; i++) {
  // timeout specified in nanoseconds (here 1s)
  rclc_executor_spin_some(&executor, 1000 * (1000 * 1000));
}

At the end, you need to free dynamically allocated memory:

  // clean up
  rc = rclc_executor_fini(&executor);
  rc += rcl_publisher_fini(&my_pub, &my_node);
  rc += rcl_timer_fini(&my_timer);
  rc += rcl_subscription_fini(&my_sub, &my_node);
  rc += rcl_node_fini(&my_node);
  rc += rclc_support_fini(&support);
  std_msgs__msg__String__fini(&pub_msg);
  std_msgs__msg__String__fini(&sub_msg);

  if (rc != RCL_RET_OK) {
    printf("Error while cleaning up!\n");
    return -1;
  }
return 0;
} // main

This completes the example. The source code can be found in the package rclc-examples rclc-examples/example_executor_only_rcl.c.
Example 2: Triggered execution

In robotic applications often multiple sensors are used to improve localization precision. These sensors can have different frequencies, for example, a high frequency IMU sensor and a low frequency laser scanner. One way is to trigger execution upon arrival of a laser scan and only then evaluate the most recent data from the aggregated IMU data.

This example demonstrates the additional feature of the rclc executor to trigger the execution of callbacks based on the availability of input data.

We setup one executor with two publishers, one with 100ms and one with 1000ms period. Then we setup one executor for two subscriptions. Their callbacks shall both be executed if the message of the publisher with the lower frequency arrives.

The output of this code example will look like this:

Created timer 'my_string_timer' with timeout 100 ms.
Created 'my_int_timer' with timeout 1000 ms.
Created subscriber topic_0:
Created subscriber topic_1:
Executor_pub: number of DDS handles: 2
Executor_sub: number of DDS handles: 2
Published: Hello World! 0
Published: Hello World! 1
Published: Hello World! 2
Published: Hello World! 3
Published: Hello World! 4
Published: Hello World! 5
Published: Hello World! 6
Published: Hello World! 7
Published: Hello World! 8
Published: Hello World! 9
Published: 0
Callback 1: Hello World! 9  <---
Callback 2: 0               <---
Published: Hello World! 10
Published: Hello World! 11
Published: Hello World! 12
Published: Hello World! 13
Published: Hello World! 14
Published: Hello World! 15
Published: Hello World! 16
Published: Hello World! 17
Published: Hello World! 18
Published: Hello World! 19
Published: 1
Callback 1: Hello World! 19 <---
Callback 2: 1               <---

This output shows, that the callbacks are executed, only if both message have received new data. In that case, the latest data of high-frequency topic is used.

You learn in this tutorial

    how to use pre-defined trigger conditions
    how to write custom-defined trigger conditions
    how to run multiple executors
    how to setup quality-of-service parameters for a subscription

We start with the necessary includes for string and int messages, <std_msgs/msg/string.h> and std_msgs/msg/int32.h respectivly. Then the necessary includes follow for the rclc convenience functions rclc.h and the the rclc executor executor.h:

#include <stdio.h>
#include <unistd.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>

#include <rclc/executor.h>
#include <rclc/rclc.h>

Then, global variables for the publishers and subscriptions as well as their messages are defined, which are initialized in the main() function and used in the corresponding callbacks:

rcl_publisher_t my_pub;
rcl_publisher_t my_int_pub;
std_msgs__msg__String sub_msg;
std_msgs__msg__Int32 pub_int_msg;
int pub_int_value;
std_msgs__msg__Int32 sub_int_msg;
int pub_string_value;

For the custom-defined trigger conditions, the type pub_trigger_object_t and the type sub_trigger_object_t are defined.

typedef struct
{
  rcl_timer_t * timer1;
  rcl_timer_t * timer2;
} pub_trigger_object_t;

typedef struct
{
  rcl_subscription_t * sub1;
  rcl_subscription_t * sub2;
} sub_trigger_object_t;

The executor for the publishers shall publish when any of corresponding timers for the publishers is ready. That is the or-logic. You could also use the predefined rclc_executor_trigger_any trigger condition, but this example shows how you can write your own trigger conditions.

In principle, the condition gets a list of handles, the length of this list, and the pre-defined condition type. In this case, we expect pub_trigger_object_t. First, the parameter obj is cased to this type (comm_obj). Then, each element of the handle list is checked for new data (or a timer is ready) by evaluating the field handles[i].data_available and its handle pointer is compared to the pointer of the communicatoin object. If at least one timer is ready, then the trigger condition returns true.

bool pub_trigger(rclc_executor_handle_t * handles, unsigned int size, void * obj)
{
  if (handles == NULL) {
    printf("Error in pub_trigger: 'handles' is a NULL pointer\n");
    return false;
  }
  if (obj == NULL) {
    printf("Error in pub_trigger: 'obj' is a NULL pointer\n");
    return false;
  }
  pub_trigger_object_t * comm_obj = (pub_trigger_object_t *) obj;
  bool timer1 = false;
  bool timer2 = false;
  //printf("pub_trigger ready set: ");
  for (unsigned int i = 0; i < size; i++) {
    if (handles[i].data_available) {
      void * handle_ptr = rclc_executor_handle_get_ptr(&handles[i]);
      if (handle_ptr == comm_obj->timer1) {
        timer1 = true;
      }
      if (handle_ptr == comm_obj->timer2) {
        timer2 = true;
      }
    }
  }
  return (timer1 || timer2);
}

The trigger condition for the subscription sub_triggershall implement an AND-logic. That is, only if both subscriptions have received a new message, then the executor shall start processing the callbacks.

The implementation is analogous to pub_trigger. The only difference is, that this trigger returns true, if both subscriptions have been found in the handle list. This is implemented in the condition sub1 && sub2 of the last if-statement.

bool sub_trigger(rclc_executor_handle_t * handles, unsigned int size, void * obj)
{
  if (handles == NULL) {
    printf("Error in sub_trigger: 'handles' is a NULL pointer\n");
    return false;
  }
  if (obj == NULL) {
    printf("Error in sub_trigger: 'obj' is a NULL pointer\n");
    return false;
  }
  sub_trigger_object_t * comm_obj = (sub_trigger_object_t *) obj;
  bool sub1 = false;
  bool sub2 = false;
  //printf("sub_trigger ready set: ");
  for (unsigned int i = 0; i < size; i++) {
    if (handles[i].data_available == true) {
      void * handle_ptr = rclc_executor_handle_get_ptr(&handles[i]);

      if (handle_ptr == comm_obj->sub1) {
        sub1 = true;
      }
      if (handle_ptr == comm_obj->sub2) {
        sub2 = true;
      }
    }
  }
  return (sub1 && sub2);
}

Like in the Hello-World example, the subscription callbacks just prints out the received message.

The my_string_subscriber callback prints out the string of the message msg->data.data:

void my_string_subscriber_callback(const void * msgin)
{
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  if (msg == NULL) {
    printf("my_string_subscriber_callback: msgin is NULL\n");
  } else {
    printf("Callback 1: %s\n", msg->data.data);
  }
}

The integer callback prints out the received integer msg->data:

void my_int_subscriber_callback(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg == NULL) {
    printf("my_int_subscriber_callback: msgin is NULL\n");
  } else {
    printf("Callback 2: %d\n", msg->data);
  }
}

To publish messages with different frequencies, we setup two timers. One timer to publish a string message, the my_timer_string_callback and one timer to publish the integer, the my_timer_int_callback.

In the my_timer_string_callback, the message pub_msg is created and filled with the string Hello World plus an integer, which is incremented by one, each time the timer callback is called. The the message is published with rcl_publish()

The macro UNUSED is a workaround for the linter warning, that the second parameter last_call_time is not used.

#define UNUSED(x) (void)x;

void my_timer_string_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  rcl_ret_t rc;
  UNUSED(last_call_time);
  if (timer != NULL) {
    //printf("Timer: time since last call %d\n", (int) last_call_time);

    std_msgs__msg__String pub_msg;
    std_msgs__msg__String__init(&pub_msg);
    const unsigned int PUB_MSG_CAPACITY = 20;
    pub_msg.data.data = malloc(PUB_MSG_CAPACITY);
    pub_msg.data.capacity = PUB_MSG_CAPACITY;
    snprintf(pub_msg.data.data, pub_msg.data.capacity, "Hello World!%d", pub_string_value++);
    pub_msg.data.size = strlen(pub_msg.data.data);

    rc = rcl_publish(&my_pub, &pub_msg, NULL);
    if (rc == RCL_RET_OK) {
      printf("Published: %s\n", pub_msg.data.data);
    } else {
      printf("Error in my_timer_string_callback: publishing message %s\n", pub_msg.data.data);
    }
    std_msgs__msg__String__fini(&pub_msg);
  } else {
    printf("Error in my_timer_string_callback: timer parameter is NULL\n");
  }
}

Likewise, the my_timer_int_callback increments the integer value pub_int_value in every call and assigns it to the message field pub_int_msg.data. Then the message is published with rcl_publish()

void my_timer_int_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  rcl_ret_t rc;
  UNUSED(last_call_time);
  if (timer != NULL) {
    //printf("Timer: time since last call %d\n", (int) last_call_time);
    pub_int_msg.data = pub_int_value++;
    rc = rcl_publish(&my_int_pub, &pub_int_msg, NULL);
    if (rc == RCL_RET_OK) {
      printf("Published: %d\n", pub_int_msg.data);
    } else {
      printf("Error in my_timer_int_callback: publishing message %d\n", pub_int_msg.data);
    }
  } else {
    printf("Error in my_timer_int_callback: timer parameter is NULL\n");
  }
}

Now were are all set for the main() function:

int main(int argc, const char * argv[])
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_ret_t rc;

  // create init_options
  rc = rclc_support_init(&support, argc, argv, &allocator);
  if (rc != RCL_RET_OK) {
    printf("Error rclc_support_init.\n");
    return -1;
  }

First rcl is initialized with the rclc_support_init using the default allocator. The rclc-support objects are saved in support. Next, a node my_node with the name node_0 and namespace executor_examples is created with:

// create rcl_node
  rcl_node_t my_node;
  rc = rclc_node_init_default(&my_node, "node_0", "executor_examples", &support);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_node_init_default\n");
    return -1;
  }

A publisher my_string_pub, which publishes a string message and its corresponding timer my_string_timer with a 100ms period is created like this:

// create a publisher 1
// - topic name: 'topic_0'
// - message type: std_msg::msg::String
const char * topic_name = "topic_0";
const rosidl_message_type_support_t * my_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);

rc = rclc_publisher_init_default(&my_string_pub, &my_node, my_type_support, topic_name);
if (RCL_RET_OK != rc) {
  printf("Error in rclc_publisher_init_default %s.\n", topic_name);
  return -1;
}

// create timer 1
// - publishes 'my_string_pub' every 'timer_timeout' ms
rcl_timer_t my_string_timer;
const unsigned int timer_timeout = 100;
rc = rclc_timer_init_default(&my_string_timer, &support, RCL_MS_TO_NS(timer_timeout), my_timer_string_callback);
if (rc != RCL_RET_OK) {
  printf("Error in rclc_timer_init_default.\n");
  return -1;
} else {
  printf("Created timer 'my_string_timer' with timeout %d ms.\n", timer_timeout);
}

Note that the previously defined my_timer_string_callback is connected to this timer. Likewise, a second publisher my_int_pub, which publishes an int message and its corresponding timer my_int_timer` with 1000ms period, is created like this:

// create publisher 2
  // - topic name: 'topic_1'
  // - message type: std_msg::msg::Int
  const char * topic_name_1 = "topic_1";
  const rosidl_message_type_support_t * my_int_type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  rc = rclc_publisher_init_default(&my_int_pub, &my_node, my_int_type_support, topic_name_1);
  if (RCL_RET_OK != rc) {
    printf("Error in rclc_publisher_init_default %s.\n", topic_name_1);
    return -1;
  }

  // create timer 2
  // - publishes 'my_int_pub' every 'timer_int_timeout' ms
  rcl_timer_t my_int_timer;
  const unsigned int timer_int_timeout = 10 * timer_timeout;
  rc = rclc_timer_init_default(&my_int_timer, &support, RCL_MS_TO_NS(timer_int_timeout), my_timer_int_callback);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_timer_init_default.\n");
    return -1;
  } else {
    printf("Created timer with timeout %d ms.\n", timer_int_timeout);
  }

Note that the my_timer_int_callback is connected to the my_int_timer. The data variables used for the publisher messages in the timer callbacks need to be initialized first:

std_msgs__msg__Int32__init(&int_pub_msg);
int_pub_value = 0;
string_pub_value = 0;

The first subscription my_string_sub is created with the function rcl_subscription_init because we change the quality-of-service parameter to ‘last-is-best’. That is, a new message will overwrite the older message if it has not been processed by the subscription. Also the message string_sub_msg needs to be initialized.

// create subscription 1
  rcl_subscription_t my_string_sub = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t my_subscription_options = rcl_subscription_get_default_options();
  my_subscription_options.qos.depth = 0; // qos: last is best = register semantics
  rc = rcl_subscription_init(&my_string_sub, &my_node, my_type_support, topic_name, &my_subscription_options);

  if (rc != RCL_RET_OK) {
    printf("Failed to create subscriber %s.\n", topic_name);
    return -1;
  } else {
    printf("Created subscriber %s:\n", topic_name);
  }
  // initialize subscription message
  std_msgs__msg__String__init(&string_sub_msg);

The second subscription my_int_sub is created with the rclc convenience function rclc_subscription_default and the message int_sub_msg is properly initialized.

// create subscription 2
  rcl_subscription_t my_int_sub = rcl_get_zero_initialized_subscription();
  rc = rclc_subscription_init_default(&my_int_sub, &my_node, my_int_type_support, topic_name_1);
  if (rc != RCL_RET_OK) {
    printf("Failed to create subscriber %s.\n", topic_name_1);
    return -1;
  } else {
    printf("Created subscriber %s:\n", topic_name_1);
  }
  // initialize subscription message
  std_msgs__msg__Int32__init(&int_sub_msg);

In this example, we are using two executors, one to schedule the publishers, and one to schedule the subscriptions:

rclc_executor_t executor_pub;
rclc_executor_t executor_sub;

The executor executor_pub is first created with rclc_executor_get_zero_initialized_executor() and has two handles (aka 2 timers).

// Executor for publishing messages
  unsigned int num_handles_pub = 2;
  printf("Executor_pub: number of DDS handles: %u\n", num_handles_pub);
  executor_pub = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor_pub, &support.context, num_handles_pub, &allocator);

  rc = rclc_executor_add_timer(&executor_pub, &my_string_timer);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_executor_add_timer 'my_string_timer'.\n");
  }

  rc = rclc_executor_add_timer(&executor_pub, &my_int_timer);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_executor_add_timer 'my_int_timer'.\n");
  }

Both timers are added to the exececutor with the function rclc_executor_add_timer:

rc = rclc_executor_add_timer(&executor_pub, &my_string_timer);
if (rc != RCL_RET_OK) {
  printf("Error in rclc_executor_add_timer 'my_string_timer'.\n");
}

rc = rclc_executor_add_timer(&executor_pub, &my_int_timer);
if (rc != RCL_RET_OK) {
  printf("Error in rclc_executor_add_timer 'my_int_timer'.\n");
}

Also the second publisher has two handles, the two subscriptions:

unsigned int num_handles_sub = 2;
printf("Executor_sub: number of DDS handles: %u\n", num_handles_sub);
executor_sub = rclc_executor_get_zero_initialized_executor();
rclc_executor_init(&executor_sub, &support.context, num_handles_sub, &allocator);

Which are added with the function rclc_executor_add_subscription:

// add subscription to executor
rc = rclc_executor_add_subscription(
  &executor_sub, &my_string_sub, &string_sub_msg,
  &my_string_subscriber_callback,
  ON_NEW_DATA);
if (rc != RCL_RET_OK) {
  printf("Error in rclc_executor_add_subscription 'my_string_sub'. \n");
}

// add int subscription to executor
rc = rclc_executor_add_subscription(
  &executor_sub, &my_int_sub, &int_sub_msg,
  &my_int_subscriber_callback,
  ON_NEW_DATA);
if (rc != RCL_RET_OK) {
  printf("Error in rclc_executor_add_subscription 'my_int_sub'. \n");
}

The trigger condition of the executor, which publishes messages, shall execute when any timer is ready. This can be configured with the function rclc_executor_set_trigger and the parameter rclc_executor_trigger_any. While the executor for the subscriptions shall only execute if both messages have arrived. Therefore the trigger parameter rclc_executor_trigger_any can be used:

rc = rclc_executor_set_trigger(&executor_pub, rclc_executor_trigger_any, NULL);
rc = rclc_executor_set_trigger(&executor_sub, rclc_executor_trigger_all, NULL);

Finally, the executors spin-some functions can be started. The sleep-time between the executors is intended for communication time for DDS.

for (unsigned int i = 0; i < 100; i++) {
  // timeout specified in ns                 (here: 1s)
  rclc_executor_spin_some(&executor_pub, 1000 * (1000 * 1000));
  usleep(1000); // 1ms
  rclc_executor_spin_some(&executor_sub, 1000 * (1000 * 1000));
}

This example is concluded with the clean-up code:

// clean up
rc = rclc_executor_fini(&executor_pub);
rc += rclc_executor_fini(&executor_sub);
rc += rcl_publisher_fini(&my_string_pub, &my_node);
rc += rcl_publisher_fini(&my_int_pub, &my_node);
rc += rcl_timer_fini(&my_string_timer);
rc += rcl_timer_fini(&my_int_timer);
rc += rcl_subscription_fini(&my_string_sub, &my_node);
rc += rcl_subscription_fini(&my_int_sub, &my_node);
rc += rcl_node_fini(&my_node);
rc += rclc_support_fini(&support);

std_msgs__msg__Int32__fini(&int_pub_msg);
std_msgs__msg__String__fini(&string_sub_msg);
std_msgs__msg__Int32__fini(&int_sub_msg);

if (rc != RCL_RET_OK) {
  printf("Error while cleaning up!\n");
  return -1;
}
return 0;
}

In case the default trigger conditions are not sufficient, then the user can define custom logic conditions. The source code of the custom-programmed trigger condition has already been presented. The following code will setup the executor accordingly:

 pub_trigger_object_t comm_obj_pub;
 comm_obj_pub.timer1 = &my_string_timer;
 comm_obj_pub.timer2 = &my_int_timer;

 sub_trigger_object_t comm_obj_sub;
 comm_obj_sub.sub1 = &my_string_sub;
 comm_obj_sub.sub2 = &my_int_sub;

 rc = rclc_executor_set_trigger(&executor_pub, pub_trigger, &comm_obj_pub);
 rc = rclc_executor_set_trigger(&executor_sub, sub_trigger, &comm_obj_sub);

The custom structs pub_trigger_object_t are used to save the pointer of the handles. The timers my_string_timer and my_int_timer for the publishing executor; and, likewise, the subscriptions my_string_sub and my_int_sub for the subscribing executor. The configuration is done also with the rclc_executor_set_trigger by passing the trigger function and the trigger object, e.g. pub_trigger and comm_obj_pub for the executor_pub, respectivly.

The complete source code of this example can be found in the file rclc-examples/example_executor_trigger.c.


Quality of service

    Reliable QoS
    Best Effort
    Custom QoS configuration

Reliable QoS

Reliable communication implies a confirmation for each message sent. This mode can detect errors in the communication process at the cost of increasing the message latency and the resources usage.

This message confirmation proccess can increase blocking time on rcl_publish or executor spin calls as reliable publishers, services and clients will wait for acknowledgement for each sent message. rmw-microxrcedds offers an API to individually configure the acknowledgement timeout on them:

  // Confirmation timeout in milliseconds
  int ack_timeout = 1000;

  // Set reliable publisher timeout
  rc = rmw_uros_set_publisher_session_timeout(&publisher, ack_timeout);

  // Set reliable service server timeout
  rc = rmw_uros_set_service_session_timeout(&service, ack_timeout);

  // Set reliable service client timeout
  rc = rmw_uros_set_client_session_timeout(&client, ack_timeout);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }

The default value for all publishers is configured at compilation time by the cmake variable RMW_UXRCE_PUBLISH_RELIABLE_TIMEOUT.
Best Effort

In best effort mode no acknowledgement is needed, the messages sent are expected to be received. This method improves publication throughput and reduces resources usage but is vulnerable to communication errors.
Custom QoS configuration

The user can customize their own QoS using the available rmw_qos_profile_t struct:

/// ROS MiddleWare quality of service profile.
typedef struct RMW_PUBLIC_TYPE rmw_qos_profile_t
{
  enum rmw_qos_history_policy_t history;
  /// Size of the message queue.
  size_t depth;
  /// Reliabiilty QoS policy setting
  enum rmw_qos_reliability_policy_t reliability;
  /// Durability QoS policy setting
  enum rmw_qos_durability_policy_t durability;
  /// The period at which messages are expected to be sent/received
  struct rmw_time_t deadline;
  /// The age at which messages are considered expired and no longer valid
  struct rmw_time_t lifespan;
  /// Liveliness QoS policy setting
  enum rmw_qos_liveliness_policy_t liveliness;
  /// The time within which the RMW node or publisher must show that it is alive
  struct rmw_time_t liveliness_lease_duration;

  /// If true, any ROS specific namespacing conventions will be circumvented.
  bool avoid_ros_namespace_conventions;
} rmw_qos_profile_t;



micro-ROS utilities

    Allocators
        Custom allocator
    Time sync
    Ping agent
    Continous serialization

Allocators

The allocator object wraps the dynamic memory allocation and deallocating methods used in micro-ROS

  // Get micro-ROS default allocator
  rcl_allocator_t allocator = rcl_get_default_allocator();

The default allocator wraps the following methods:

  - allocate = wraps malloc()
  - deallocate = wraps free()
  - reallocate = wraps realloc()
  - zero_allocate = wraps calloc()
  - state = `NULL`

Custom allocator

Working in embedded systems, the user might need to modify this default functions with its own memory allocation methods. To archieve this, the user can modify the default allocator with its own methods:

// Get empty allocator
rcl_allocator_t custom_allocator = rcutils_get_zero_initialized_allocator();

// Set custom allocation methods
custom_allocator.allocate = microros_allocate;
custom_allocator.deallocate = microros_deallocate;
custom_allocator.reallocate = microros_reallocate;
custom_allocator.zero_allocate =  microros_zero_allocate;

// Set custom allocator as default
if (!rcutils_set_default_allocator(&custom_allocator)) {
    ... // Handle error
    return -1;
}

Custom methods prototypes and examples:

    allocate:

    Allocates memory given a size, an error should be indicated by returning NULL:

    // Function prototype:
    void * (*allocate)(size_t size, void * state);

    // Implementation example:
    void * microros_allocate(size_t size, void * state){
      (void) state;
      void * ptr = malloc(size);
      return ptr;
    }

    deallocate

    Deallocate previously allocated memory, mimicking free():

    // Function prototype:
    void (* deallocate)(void * pointer, void * state);

    // Implementation example:
    void microros_deallocate(void * pointer, void * state){
      (void) state;
      free(pointer);
    }

    reallocate:

    Reallocate memory if possible, otherwise it deallocates and allocates:

    // Function prototype:
    void * (*reallocate)(void * pointer, size_t size, void * state);

    // Implementation example:
    void * microros_reallocate(void * pointer, size_t size, void * state){
      (void) state;
      void * ptr = realloc(pointer, size);
      return ptr;
    }

    zero_allocate:

    Allocate memory with all elements set to zero, given a number of elements and their size. An error should be indicated by returning NULL:

    // Function prototype:
    void * (*zero_allocate)(size_t number_of_elements, size_t size_of_element, void * state);

    // Implementation example:
    void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state){
      (void) state;
      void * ptr = malloc(number_of_elements * size_of_element);
      memset(ptr, 0, number_of_elements * size_of_element);
      return ptr;
    }

    Note: the state input argument is espected to be unused

Time sync

micro-ROS Clients can synchronize their epoch time with the connected Agent, this can be very useful when working in embedded environments that do not provide any time synchronization mechanism. This utility is based on the NTP protocol, taking into account delays caused by the transport layer. An usage example can be found on micro-ROS-demos/rclc/epoch_synchronization.

// Sync timeout
const int timeout_ms = 1000;

// Synchronize time with the agent
rmw_uros_sync_session(timeout_ms);

if (rmw_uros_epoch_synchronized())
{
    // Get time in milliseconds or nanoseconds
    int64_t time_ms = rmw_uros_epoch_millis();
    int64_t time_ns = rmw_uros_epoch_nanos();
}

Ping agent

The Client can test the connection with the Agent with the ping utility. This functionality can be used even when the micro-ROS context has not yet been initialized, which is useful to test the connection before trying to connect to the Agent. An example can be found on micro-ROS-demos/rclc/ping_uros_agent.

// Timeout for each attempt
const int timeout_ms = 1000;

// Number of attemps
const uint8_t attemps = 5;

// Ping the agent
rmw_ret_t ping_result = rmw_uros_ping_agent(timeout_ms, attempts);

if (RMW_RET_OK == ping_result)
{
    // micro-ROS Agent is reachable
    ...
}
else
{
    // micro-ROS Agent is not available
    ...
}

Note: rmw_uros_ping_agent is thread safe.
Continous serialization

This utility allows the client to serialize and send data up to a customized size. The user can set the topic lenght and then serialize the data within the publish process. An example can be found on micro-ROS-demos/rclc/ping_uros_agent, where fragments from an image are requested and serialized on the spot.

The user needs to define two callbacks, then set them on the rmw. It is recommended to clean the callbacks after the publication, to avoid interferences with other topics on the same process:

// Set serialization callbacks
rmw_uros_set_continous_serialization_callbacks(size_cb, serialization_cb);

// Publish message
rcl_publish(...);

// Clean callbacks
rmw_uros_set_continous_serialization_callbacks(NULL, NULL);

    Size callback:

This callback will pass a pointer with the calculated message size. The user is responsible of increase this size to the expected value:

// Function prototype:
void (* rmw_uros_continous_serialization_size)(uint32_t * topic_length);

// Implementation example:
void size_cb(uint32_t * topic_length){
    // Increase message size
    *topic_length += ucdr_alignment(*topic_length, sizeof(uint32_t)) + sizeof(uint32_t);
    *topic_length += IMAGE_BYTES;
}

    Serialize callback:

This callback gives the user the message buffer to be completed. The user is responsible of serialize the data up to the lenght established on the size callback:

// Function prototype:
void (* rmw_uros_continous_serialization)(ucdrBuffer * ucdr);

// Implementation example:
void serialization_cb(ucdrBuffer * ucdr){
    size_t len = 0;
    micro_ros_fragment_t fragment;

    // Serialize array size
    ucdr_serialize_uint32_t(ucdr, IMAGE_BYTES);

    while(len < IMAGE_BYTES){
      // Wait for new image "fragment"
      ...

      // Serialize data fragment
      ucdr_serialize_array_uint8_t(ucdr, fragment.data, fragment.len);
      len += fragment.len;
    }
}

Note: When the callback ends, the message will be published.
