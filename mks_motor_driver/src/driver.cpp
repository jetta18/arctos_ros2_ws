#include "mks_motor_driver/driver.hpp"

#include <cstring>

namespace mks {

static inline uint16_t be16(uint16_t v) { return (uint16_t)((v >> 8) | (v << 8)); }

std::optional<std::vector<uint8_t>> MksDriver::request(uint8_t code, const std::vector<uint8_t>& args, int expect_payload_min) {
  if (!transport_) return std::nullopt;
  std::vector<uint8_t> payload;
  payload.reserve(1 + args.size());
  payload.push_back(code);
  payload.insert(payload.end(), args.begin(), args.end());
  CanFrame tx = build_frame(can_id_, payload);
  if (!transport_->write(tx)) return std::nullopt;

  auto rx = transport_->read(timeout_ms_);
  if (!rx) return std::nullopt;
  auto parsed = parse_frame(can_id_, *rx);
  if (!parsed) return std::nullopt;
  if (parsed->empty() || (*parsed)[0] != code) return std::nullopt;
  if (expect_payload_min >= 0 && static_cast<int>(parsed->size()) < expect_payload_min) return std::nullopt;
  return parsed;
}

std::optional<std::vector<uint8_t>> MksDriver::simple_request(uint8_t code) {
  return request(code, {}, 2);
}

std::optional<EncoderCarryValue> MksDriver::read_encoder_carry() {
  auto resp = request(0x30, {}, 7); // command + 4(carry) + 2(value)
  if (!resp) return std::nullopt;
  EncoderCarryValue out{};
  out.carry = (int32_t)(((*resp)[1] << 24) | ((*resp)[2] << 16) | ((*resp)[3] << 8) | (*resp)[4]);
  out.value = (uint16_t)(((*resp)[5] << 8) | (*resp)[6]);
  return out;
}

std::optional<int64_t> MksDriver::read_encoder_addition() {
  auto resp = request(0x31, {}, 7);
  if (!resp) return std::nullopt;
  return unpack_i48_be(&(*resp)[1]);
}

std::optional<int16_t> MksDriver::read_speed_rpm() {
  auto resp = request(0x32, {}, 3);
  if (!resp) return std::nullopt;
  int16_t v = (int16_t)(((*resp)[1] << 8) | (*resp)[2]);
  return v;
}

std::optional<int32_t> MksDriver::read_pulses() {
  auto resp = request(0x33, {}, 5);
  if (!resp) return std::nullopt;
  int32_t v = (int32_t)(((*resp)[1] << 24) | ((*resp)[2] << 16) | ((*resp)[3] << 8) | (*resp)[4]);
  return v;
}

std::optional<IoStatus> MksDriver::read_io_status() {
  auto resp = request(0x34, {}, 2);
  if (!resp) return std::nullopt;
  uint8_t s = (*resp)[1];
  IoStatus st{};
  st.in1 = (s & 0x01) != 0;
  st.in2 = (s & 0x02) != 0;
  st.out1 = (s & 0x04) != 0;
  st.out2 = (s & 0x08) != 0;
  return st;
}

std::optional<int32_t> MksDriver::read_angle_error() {
  auto resp = request(0x39, {}, 5);
  if (!resp) return std::nullopt;
  int32_t v = (int32_t)(((*resp)[1] << 24) | ((*resp)[2] << 16) | ((*resp)[3] << 8) | (*resp)[4]);
  return v;
}

std::optional<uint8_t> MksDriver::read_enable_status() {
  auto resp = request(0x3A, {}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1];
}

std::optional<uint8_t> MksDriver::read_zero_status() {
  auto resp = request(0x3B, {}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1];
}

std::optional<uint8_t> MksDriver::release_protection() {
  auto resp = request(0x3D, {}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1];
}

std::optional<uint8_t> MksDriver::read_protection_state() {
  auto resp = request(0x3E, {}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1];
}

std::optional<Result> MksDriver::calibrate() {
  auto resp = request(0x80, {0x00}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::set_mode(WorkMode mode) {
  auto resp = request(0x82, {static_cast<uint8_t>(mode)}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::set_work_current_ma(uint16_t ma) {
  uint8_t hi = (ma >> 8) & 0xFF;
  uint8_t lo = ma & 0xFF;
  auto resp = request(0x83, {hi, lo}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::set_hold_current_percent(uint8_t p10) {
  auto resp = request(0x9B, {static_cast<uint8_t>(p10 & 0x0F)}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::set_microstep(uint8_t ms) {
  auto resp = request(0x84, {ms}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::set_en_active(uint8_t enable_mode) {
  auto resp = request(0x85, {static_cast<uint8_t>(enable_mode & 0x03)}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::set_dir_polarity(Direction d) {
  auto resp = request(0x86, {static_cast<uint8_t>(d == Direction::CCW ? 1 : 0)}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::set_auto_screen(bool en) {
  auto resp = request(0x87, {static_cast<uint8_t>(en ? 1 : 0)}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::set_protect(bool en) {
  auto resp = request(0x88, {static_cast<uint8_t>(en ? 1 : 0)}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::set_mplyer(bool en) {
  auto resp = request(0x89, {static_cast<uint8_t>(en ? 1 : 0)}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::set_can_rate(uint8_t rate) {
  auto resp = request(0x8A, {static_cast<uint8_t>(rate & 0x03)}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::set_can_id_cmd(uint16_t id) {
  uint16_t x = id & 0x7FF;
  uint8_t hi = (x >> 8) & 0xFF;
  uint8_t lo = x & 0xFF;
  auto resp = request(0x8B, {hi, lo}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::set_respond_active(bool respon, bool active) {
  auto resp = request(0x8C, {static_cast<uint8_t>(respon ? 1 : 0), static_cast<uint8_t>(active ? 1 : 0)}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::set_key_lock(bool en) {
  auto resp = request(0x8F, {static_cast<uint8_t>(en ? 1 : 0)}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::set_group_id(uint16_t id) {
  uint16_t x = id & 0x7FF;
  uint8_t hi = (x >> 8) & 0xFF;
  uint8_t lo = x & 0xFF;
  auto resp = request(0x8D, {hi, lo}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::set_home_params(uint8_t trig_level, Direction dir, uint16_t rpm, bool end_limit) {
  uint8_t dirb = (dir == Direction::CCW) ? 1 : 0;
  uint8_t hi = (rpm >> 8) & 0xFF;
  uint8_t lo = rpm & 0xFF;
  uint8_t en = end_limit ? 1 : 0;
  const uint8_t trig = trig_level == 0 ? 0u : 1u;
  std::vector<uint8_t> payload{trig, dirb, hi, lo, en};
  auto resp = request(0x90, payload, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::go_home() {
  auto resp = simple_request(0x91);
  if (!resp) return std::nullopt;
  // status: 0 fail, 1 start, 2 success — we treat non-zero as success of command
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::set_current_axis_zero() {
  auto resp = simple_request(0x92);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::set_limit_remap(bool en) {
  auto resp = request(0x9E, {static_cast<uint8_t>(en ? 1 : 0)}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::restore_defaults() {
  auto resp = request(0x3F, {}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<MotorStatus> MksDriver::query_status() {
  auto resp = request(0xF1, {}, 2);
  if (!resp) return std::nullopt;
  return static_cast<MotorStatus>((*resp)[1]);
}

std::optional<Result> MksDriver::enable_driver(bool en) {
  auto resp = request(0xF3, {static_cast<uint8_t>(en ? 1 : 0)}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::emergency_stop() {
  auto resp = request(0xF7, {}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<Result> MksDriver::run_speed(Direction dir, const SpeedAcc& sa) {
  uint8_t b2 = 0, b3 = 0;
  pack_dir_speed(sa.speed, dir == Direction::CW, b2, b3);
  auto resp = request(0xF6, {b2, b3, sa.acc}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

std::optional<uint8_t> MksDriver::stop_speed(uint8_t acc) {
  auto resp = request(0xF6, {0x00, 0x00, acc}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1];
}

std::optional<uint8_t> MksDriver::move_relative_pulses(Direction dir, const SpeedAcc& sa, uint32_t pulses) {
  uint8_t b2 = 0, b3 = 0;
  pack_dir_speed(sa.speed, dir == Direction::CW, b2, b3);
  uint8_t b5, b6, b7;
  pack_u24(pulses, b5, b6, b7);
  auto resp = request(0xFD, {b2, b3, sa.acc, b5, b6, b7}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1];
}

std::optional<uint8_t> MksDriver::stop_relative_pulses(uint8_t acc) {
  auto resp = request(0xFD, {0x00, 0x00, acc, 0x00, 0x00, 0x00}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1];
}

std::optional<uint8_t> MksDriver::move_absolute_pulses(const SpeedAcc& sa, int32_t abs_pulses) {
  uint8_t sh = (sa.speed >> 8) & 0xFF;
  uint8_t sl = sa.speed & 0xFF;
  uint8_t b5, b6, b7;
  pack_i24(abs_pulses, b5, b6, b7);
  auto resp = request(0xFE, {sh, sl, sa.acc, b5, b6, b7}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1];
}

std::optional<uint8_t> MksDriver::stop_absolute_pulses(uint8_t acc) {
  auto resp = request(0xFE, {0x00, 0x00, acc, 0x00, 0x00, 0x00}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1];
}

std::optional<uint8_t> MksDriver::move_relative_axis(const SpeedAcc& sa, int32_t rel_axis) {
  uint8_t sh = (sa.speed >> 8) & 0xFF;
  uint8_t sl = sa.speed & 0xFF;
  uint8_t b5, b6, b7;
  pack_i24(rel_axis, b5, b6, b7);
  auto resp = request(0xF4, {sh, sl, sa.acc, b5, b6, b7}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1];
}

std::optional<uint8_t> MksDriver::stop_relative_axis(uint8_t acc) {
  auto resp = request(0xF4, {0x00, 0x00, acc, 0x00, 0x00, 0x00}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1];
}

std::optional<uint8_t> MksDriver::move_absolute_axis(const SpeedAcc& sa, int32_t abs_axis) {
  uint8_t sh = (sa.speed >> 8) & 0xFF;
  uint8_t sl = sa.speed & 0xFF;
  uint8_t b5, b6, b7;
  pack_i24(abs_axis, b5, b6, b7);
  auto resp = request(0xF5, {sh, sl, sa.acc, b5, b6, b7}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1];
}

std::optional<uint8_t> MksDriver::stop_absolute_axis(uint8_t acc) {
  auto resp = request(0xF5, {0x00, 0x00, acc, 0x00, 0x00, 0x00}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1];
}

std::optional<Result> MksDriver::save_speed_mode_params(bool save) {
  uint8_t state = save ? 0xC8 : 0xCA;
  auto resp = request(0xFF, {state}, 2);
  if (!resp) return std::nullopt;
  return (*resp)[1] ? Result::Success : Result::Fail;
}

} // namespace mks
