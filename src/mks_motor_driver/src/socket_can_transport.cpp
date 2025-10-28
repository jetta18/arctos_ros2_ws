#include "mks_motor_driver/can_transport.hpp"

#include <stdexcept>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>

namespace mks {

SocketCanTransport::SocketCanTransport() : sock_(-1) {}
SocketCanTransport::~SocketCanTransport() { close(); }

bool SocketCanTransport::open(const std::string& ifname) {
  if (sock_ != -1) close();
  sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_ < 0) return false;

  // Bind to interface
  ifreq ifr{};
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname.c_str());
  if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
    ::close(sock_); sock_ = -1; return false;
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  // Non-blocking to allow timed reads using poll
  int flags = ::fcntl(sock_, F_GETFL, 0);
  if (flags >= 0) ::fcntl(sock_, F_SETFL, flags | O_NONBLOCK);

  if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(sock_); sock_ = -1; return false;
  }

  return true;
}

void SocketCanTransport::close() {
  if (sock_ != -1) {
    ::close(sock_);
    sock_ = -1;
  }
}

bool SocketCanTransport::write(const CanFrame& frame) {
  if (sock_ < 0) return false;
  can_frame f{};
  f.can_id = frame.id & CAN_SFF_MASK;
  f.can_dlc = frame.dlc > 8 ? 8 : frame.dlc;
  std::memcpy(f.data, frame.data, f.can_dlc);
  const ssize_t n = ::send(sock_, &f, sizeof(f), 0);
  return n == sizeof(f);
}

std::optional<CanFrame> SocketCanTransport::read(int timeout_ms) {
  if (sock_ < 0) return std::nullopt;
  pollfd p{};
  p.fd = sock_;
  p.events = POLLIN;
  const int r = ::poll(&p, 1, timeout_ms);
  if (r <= 0) return std::nullopt; // timeout or error
  can_frame f{};
  const ssize_t n = ::recv(sock_, &f, sizeof(f), 0);
  if (n != sizeof(f)) return std::nullopt;
  CanFrame out{};
  out.id = f.can_id & CAN_SFF_MASK;
  out.dlc = f.can_dlc;
  std::memcpy(out.data, f.data, out.dlc);
  return out;
}

} // namespace mks
