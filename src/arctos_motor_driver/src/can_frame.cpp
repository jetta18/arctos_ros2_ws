#include "arctos_motor_driver/can_frame.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

namespace arctos_motor_driver {

// CANFrame Implementation

CANFrame::CANFrame() : id(0), dlc(0), data{} {}

CANFrame::CANFrame(uint32_t id, uint8_t dlc, const std::vector<uint8_t>& data_bytes) 
    : id(id), dlc(dlc), data{} {
    // Copy data and ensure we don't exceed 8 bytes
    size_t copy_size = std::min(static_cast<size_t>(dlc), std::min(data_bytes.size(), size_t(8)));
    std::copy_n(data_bytes.begin(), copy_size, data.begin());
}

uint8_t CANFrame::calculateCRC() const {
    uint32_t sum = id;
    // Calculate CRC on data bytes EXCLUDING the last byte (which is the CRC itself)
    for (size_t i = 0; i < static_cast<size_t>(dlc - 1) && i < 7; ++i) {
        sum += data[i];
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

bool CANFrame::isValid() const {
    // Check ID range (1-0x7FF, 0 is broadcast)
    if (id == 0 || id > 0x7FF) {
        return false;
    }
    
    // Check DLC
    if (dlc > 8 || dlc == 0) {
        return false;
    }
    
    // For MKS frames, the last byte should be the CRC
    // Calculate expected CRC on all bytes except the last one
    if (dlc > 1) {
        uint8_t expected_crc = calculateCRC();
        uint8_t actual_crc = data[dlc - 1];
        return expected_crc == actual_crc;
    }
    
    // Single byte frames are considered valid (no CRC to check)
    return true;
}

uint8_t CANFrame::getCommand() const {
    return (dlc > 0) ? data[0] : 0;
}

can_msgs::msg::Frame CANFrame::toRosMessage() const {
    can_msgs::msg::Frame msg;
    msg.id = id;
    msg.is_rtr = false;
    msg.is_extended = false;
    msg.is_error = false;
    msg.dlc = dlc;
    
    // Copy data to ROS message (always 8 bytes)
    std::copy(data.begin(), data.end(), msg.data.begin());
    
    return msg;
}

CANFrame CANFrame::fromRosMessage(const can_msgs::msg::Frame::SharedPtr& msg) {
    return fromRosMessage(*msg);
}

CANFrame CANFrame::fromRosMessage(const can_msgs::msg::Frame& msg) {
    CANFrame frame;
    frame.id = msg.id;
    frame.dlc = msg.dlc;
    
    // Copy data from ROS message
    std::copy(msg.data.begin(), msg.data.end(), frame.data.begin());
    
    return frame;
}

std::string CANFrame::toString() const {
    std::ostringstream oss;
    oss << "CANFrame[ID:0x" << std::hex << std::uppercase << id 
        << " DLC:" << std::dec << static_cast<int>(dlc) << " DATA:";
    
    for (size_t i = 0; i < dlc && i < 8; ++i) {
        oss << " 0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(2) 
            << static_cast<int>(data[i]);
    }
    
    oss << " CMD:0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(2) 
        << static_cast<int>(getCommand()) << "]";
    
    return oss.str();
}

// CANCommand Implementation

CANCommand::CANCommand() : cmd(0) {}

CANCommand::CANCommand(uint8_t cmd) : cmd(cmd) {}

CANCommand::CANCommand(uint8_t cmd, const std::vector<uint8_t>& payload) 
    : cmd(cmd), payload(payload) {}

CANFrame CANCommand::toFrame(uint32_t motor_id) const {
    std::vector<uint8_t> frame_data;
    frame_data.reserve(payload.size() + 2); // cmd + payload + crc
    
    // Add command byte
    frame_data.push_back(cmd);
    
    // Add payload
    frame_data.insert(frame_data.end(), payload.begin(), payload.end());
    
    // Calculate and add CRC
    uint32_t sum = motor_id;
    for (uint8_t byte : frame_data) {
        sum += byte;
    }
    uint8_t crc = static_cast<uint8_t>(sum & 0xFF);
    frame_data.push_back(crc);
    
    return CANFrame(motor_id, static_cast<uint8_t>(frame_data.size()), frame_data);
}

uint8_t CANCommand::getFrameSize() const {
    return static_cast<uint8_t>(1 + payload.size() + 1); // cmd + payload + crc
}

std::string CANCommand::toString() const {
    std::ostringstream oss;
    oss << "CANCommand[CMD:0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(2) 
        << static_cast<int>(cmd) << " PAYLOAD_SIZE:" << std::dec << payload.size() << "]";
    return oss.str();
}

} // namespace arctos_motor_driver
