#pragma once

#include <cstdint>
#include <vector>
#include <optional>
#include "mks_motor_driver/can_transport.hpp"

namespace mks {

// Compute 8-bit checksum defined by manual:
// CRC = (CAN_ID + byte1 + ... + byteN) & 0xFF
inline uint8_t checksum8(uint16_t can_id, const uint8_t* data, size_t len) {
  uint32_t sum = (can_id & 0x7FF);
  for (size_t i = 0; i < len; ++i) sum += data[i];
  return static_cast<uint8_t>(sum & 0xFF);
}

inline uint8_t checksum8(uint16_t can_id, const std::vector<uint8_t>& bytes) {
  return checksum8(can_id, bytes.data(), bytes.size());
}

// Build a frame with payload (code + data) and append CRC as last byte
inline CanFrame build_frame(uint16_t can_id, const std::vector<uint8_t>& payload_no_crc) {
  CanFrame f{};
  f.id = can_id & 0x7FF; // standard 11-bit
  const uint8_t crc = checksum8(can_id, payload_no_crc);
  f.dlc = static_cast<uint8_t>(payload_no_crc.size() + 1);
  for (size_t i = 0; i < payload_no_crc.size() && i < 8; ++i) f.data[i] = payload_no_crc[i];
  if (f.dlc <= 8) {
    f.data[f.dlc - 1] = crc;
  }
  return f;
}

// Verify CRC of an incoming frame and return the payload without CRC if valid
inline std::optional<std::vector<uint8_t>> parse_frame(uint16_t can_id, const CanFrame& f) {
  if (f.dlc == 0 || f.dlc > 8) return std::nullopt;
  const uint8_t crc = f.data[f.dlc - 1];
  std::vector<uint8_t> bytes(f.data, f.data + (f.dlc - 1));
  const uint8_t calc = checksum8(can_id, bytes);
  if (calc != crc) return std::nullopt;
  return bytes;
}

// Helper to pack speed/dir as defined in manual (F6/FD use: dir in b7 of byte2, speed 12-bit into lower 4 bits of byte2 + all of byte3)
inline void pack_dir_speed(uint16_t speed, bool cw, uint8_t& b2, uint8_t& b3) {
  // Bit7 of byte2 indicates direction; per existing driver usage: 0=CW, 1=CCW
  const uint16_t sp = static_cast<uint16_t>(speed & 0x0FFF);
  const uint8_t dir_bit = cw ? 0x00 : 0x80;
  b2 = static_cast<uint8_t>(dir_bit | ((sp >> 8) & 0x0F));
  b3 = static_cast<uint8_t>(sp & 0xFF);
}

// Pack signed 24-bit integer, big-endian
inline void pack_i24(int32_t v, uint8_t& b5, uint8_t& b6, uint8_t& b7) {
  int32_t x = v;
  if (x > 0x7FFFFF) x = 0x7FFFFF;
  if (x < -0x800000) x = -0x800000;
  uint32_t ux = static_cast<uint32_t>(x & 0xFFFFFF);
  b5 = static_cast<uint8_t>((ux >> 16) & 0xFF);
  b6 = static_cast<uint8_t>((ux >> 8) & 0xFF);
  b7 = static_cast<uint8_t>(ux & 0xFF);
}

// Pack unsigned 24-bit integer, big-endian
inline void pack_u24(uint32_t v, uint8_t& b5, uint8_t& b6, uint8_t& b7) {
  uint32_t x = v & 0xFFFFFF;
  b5 = static_cast<uint8_t>((x >> 16) & 0xFF);
  b6 = static_cast<uint8_t>((x >> 8) & 0xFF);
  b7 = static_cast<uint8_t>(x & 0xFF);
}

// Unpack signed 48-bit big-endian to int64
inline int64_t unpack_i48_be(const uint8_t* p) {
  int64_t v = 0;
  for (int i = 0; i < 6; ++i) v = (v << 8) | p[i];
  // sign extend
  if (p[0] & 0x80) v |= (~0ULL) << 48;
  return v;
}

// Unpack signed 24-bit big-endian to int32
inline int32_t unpack_i24_be(const uint8_t* p) {
  int32_t v = (static_cast<int32_t>(p[0]) << 16) | (static_cast<int32_t>(p[1]) << 8) | static_cast<int32_t>(p[2]);
  if (p[0] & 0x80) v |= 0xFF000000; // sign extend
  return v;
}

// Unpack unsigned 24-bit big-endian to uint32
inline uint32_t unpack_u24_be(const uint8_t* p) {
  return (static_cast<uint32_t>(p[0]) << 16) | (static_cast<uint32_t>(p[1]) << 8) | static_cast<uint32_t>(p[2]);
}

} // namespace mks
