#ifndef ARCTOS_MOTOR_DRIVER__SOCKET_CAN_BRIDGE_HPP_
#define ARCTOS_MOTOR_DRIVER__SOCKET_CAN_BRIDGE_HPP_

#include <string>
#include <functional>
#include <thread>
#include <atomic>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arctos_motor_driver
{

class SocketCANBridge
{
public:
    using ReceiveCallback = std::function<void(const can_msgs::msg::Frame&)>;

    explicit SocketCANBridge(rclcpp::Node::SharedPtr node);
    ~SocketCANBridge();

    bool open(const std::string& interface_name);
    void close();
    bool send(const can_msgs::msg::Frame& frame);
    void register_receive_callback(const ReceiveCallback& callback);
    bool is_open() const;

private:
    void receive_thread_func();

    rclcpp::Node::SharedPtr node_;
    int socket_fd_ = -1;
    std::thread receive_thread_;
    std::atomic<bool> is_running_;
    ReceiveCallback receive_callback_;
};

}

#endif // ARCTOS_MOTOR_DRIVER__SOCKET_CAN_BRIDGE_HPP_
