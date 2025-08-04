#include "arctos_motor_driver/socket_can_bridge.hpp"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace arctos_motor_driver
{

SocketCANBridge::SocketCANBridge(rclcpp::Node::SharedPtr node)
    : node_(node), is_running_(false) {}

SocketCANBridge::~SocketCANBridge()
{
    close();
}

bool SocketCANBridge::open(const std::string& interface_name)
{
    if (is_open())
    {
        RCLCPP_WARN(node_->get_logger(), "CAN socket is already open.");
        return true;
    }

    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create CAN socket: %s", strerror(errno));
        return false;
    }

    ifreq ifr;
    strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get CAN interface index for '%s': %s", interface_name.c_str(), strerror(errno));
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to bind CAN socket: %s", strerror(errno));
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    is_running_ = true;
    receive_thread_ = std::thread(&SocketCANBridge::receive_thread_func, this);

    RCLCPP_INFO(node_->get_logger(), "Successfully opened CAN interface '%s'", interface_name.c_str());
    return true;
}

void SocketCANBridge::close()
{
    if (!is_running_)
    {
        return;
    }

    is_running_ = false;

    if (socket_fd_ >= 0)
    {
        // Shutting down the socket interrupts the blocking read
        shutdown(socket_fd_, SHUT_RDWR);
    }

    if (receive_thread_.joinable())
    {
        receive_thread_.join();
    }

    if (socket_fd_ >= 0)
    {
        ::close(socket_fd_);
        socket_fd_ = -1;
        RCLCPP_INFO(node_->get_logger(), "CAN socket closed.");
    }
}

bool SocketCANBridge::send(const can_msgs::msg::Frame& frame)
{
    if (!is_open())
    {
        RCLCPP_ERROR(node_->get_logger(), "CAN socket is not open, cannot send frame.");
        return false;
    }

    can_frame can_frame_to_send;
    can_frame_to_send.can_id = frame.id;
    can_frame_to_send.can_dlc = frame.dlc;
    std::copy(frame.data.begin(), frame.data.end(), can_frame_to_send.data);

    if (write(socket_fd_, &can_frame_to_send, sizeof(can_frame)) != sizeof(can_frame))
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to send CAN frame: %s", strerror(errno));
        return false;
    }

    return true;
}

void SocketCANBridge::register_receive_callback(const ReceiveCallback& callback)
{
    receive_callback_ = callback;
}

bool SocketCANBridge::is_open() const
{
    return socket_fd_ >= 0;
}

void SocketCANBridge::receive_thread_func()
{
    while (is_running_)
    {
        can_frame received_frame;
        int nbytes = read(socket_fd_, &received_frame, sizeof(can_frame));

        if (!is_running_) break;

        if (nbytes < 0)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
            {
                RCLCPP_ERROR(node_->get_logger(), "CAN socket read error: %s", strerror(errno));
            }
            continue;
        }

        if (nbytes < (int)sizeof(can_frame))
        {
            RCLCPP_WARN(node_->get_logger(), "Incomplete CAN frame received.");
            continue;
        }

        if (receive_callback_)
        {
            can_msgs::msg::Frame frame_to_publish;
            frame_to_publish.id = received_frame.can_id;
            frame_to_publish.dlc = received_frame.can_dlc;
            std::copy(std::begin(received_frame.data), std::end(received_frame.data), frame_to_publish.data.begin());
            receive_callback_(frame_to_publish);
        }
    }
}

}
