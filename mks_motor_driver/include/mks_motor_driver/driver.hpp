#pragma once

#include <cstdint>
#include <optional>
#include <vector>
#include <string>
#include <memory>
#include "mks_motor_driver/types.hpp"
#include "mks_motor_driver/can_transport.hpp"
#include "mks_motor_driver/protocol.hpp"

namespace mks {

struct EncoderCarryValue {
  int32_t carry;      // signed carry
  uint16_t value;     // 0..0x3FFF
};

struct IoStatus {
  bool in1 = false;
  bool in2 = false;
  bool out1 = false;
  bool out2 = false;
};

class MksDriver {
public:
  explicit MksDriver(std::shared_ptr<ICanTransport> transport, uint16_t can_id = 0x01)
    : transport_(std::move(transport)), can_id_(can_id) {}

  bool open(const std::string& ifname) { return transport_ ? transport_->open(ifname) : false; }
  void close() { if (transport_) transport_->close(); }

  void set_can_id(uint16_t id) { can_id_ = id & 0x7FF; }
  uint16_t can_id() const { return can_id_; }

  // --------- Query commands ---------
  std::optional<EncoderCarryValue> read_encoder_carry();      // code 0x30
  std::optional<int64_t> read_encoder_addition();             // code 0x31
  std::optional<int16_t> read_speed_rpm();                    // code 0x32
  std::optional<int32_t> read_pulses();                       // code 0x33
  std::optional<IoStatus> read_io_status();                   // code 0x34
  std::optional<int32_t> read_angle_error();                  // code 0x39
  std::optional<uint8_t> read_enable_status();                // code 0x3A (0/1)
  std::optional<uint8_t> read_zero_status();                  // code 0x3B (0..2)
  std::optional<uint8_t> release_protection();                // code 0x3D return status
  std::optional<uint8_t> read_protection_state();             // code 0x3E (0/1)

  // --------- Setup commands ---------
  std::optional<Result> calibrate();                          // 0x80
  std::optional<Result> set_mode(WorkMode mode);              // 0x82
  std::optional<Result> set_work_current_ma(uint16_t ma);     // 0x83
  std::optional<Result> set_hold_current_percent(uint8_t p10);// 0x9B (0..8 => 10..90%)
  std::optional<Result> set_microstep(uint8_t ms);            // 0x84 (1..256, generic 0..255)
  std::optional<Result> set_en_active(uint8_t enable_mode);   // 0x85 (0=L,1=H,2=Hold)
  std::optional<Result> set_dir_polarity(Direction d);        // 0x86 (0=CW,1=CCW)
  std::optional<Result> set_auto_screen(bool en);             // 0x87
  std::optional<Result> set_protect(bool en);                 // 0x88
  std::optional<Result> set_mplyer(bool en);                  // 0x89
  std::optional<Result> set_can_rate(uint8_t rate);           // 0x8A (0=125K..3=1M)
  std::optional<Result> set_can_id_cmd(uint16_t id);          // 0x8B write to device
  std::optional<Result> set_respond_active(bool respon, bool active); // 0x8C
  std::optional<Result> set_key_lock(bool en);                // 0x8F
  std::optional<Result> set_group_id(uint16_t id);            // 0x8D
  std::optional<Result> set_home_params(uint8_t trig_level, Direction dir, uint16_t rpm, bool end_limit);
  std::optional<Result> go_home();                            // 0x91
  std::optional<Result> set_current_axis_zero();              // 0x92
  std::optional<Result> set_limit_remap(bool en);             // 0x9E
  std::optional<Result> restore_defaults();                   // 0x3F

  // --------- Runtime control ---------
  std::optional<MotorStatus> query_status();                  // 0xF1
  std::optional<Result> enable_driver(bool en);               // 0xF3
  std::optional<Result> emergency_stop();                     // 0xF7

  // Speed mode (F6)
  std::optional<Result> run_speed(Direction dir, const SpeedAcc& sa);
  std::optional<uint8_t> stop_speed(uint8_t acc);             // returns status (0..2)

  // Position mode1: relative motion by pulses (FD)
  std::optional<uint8_t> move_relative_pulses(Direction dir, const SpeedAcc& sa, uint32_t pulses);
  std::optional<uint8_t> stop_relative_pulses(uint8_t acc);

  // Position mode2: absolute motion by pulses (FE)
  std::optional<uint8_t> move_absolute_pulses(const SpeedAcc& sa, int32_t abs_pulses);
  std::optional<uint8_t> stop_absolute_pulses(uint8_t acc);

  // Position mode3: relative motion by axis (F4)
  std::optional<uint8_t> move_relative_axis(const SpeedAcc& sa, int32_t rel_axis);
  std::optional<uint8_t> stop_relative_axis(uint8_t acc);

  // Position mode4: absolute motion by axis (F5)
  std::optional<uint8_t> move_absolute_axis(const SpeedAcc& sa, int32_t abs_axis);
  std::optional<uint8_t> stop_absolute_axis(uint8_t acc);

  // Persistence for speed mode: FF C8(save)/CA(clean)
  std::optional<Result> save_speed_mode_params(bool save);    // true=C8, false=CA

  // Set default request timeout in milliseconds for response commands
  void set_timeout_ms(int ms) { timeout_ms_ = ms; }

private:
  std::optional<std::vector<uint8_t>> request(uint8_t code, const std::vector<uint8_t>& args, int expect_dlc_min = 3);
  std::optional<std::vector<uint8_t>> simple_request(uint8_t code);

  std::shared_ptr<ICanTransport> transport_;
  uint16_t can_id_;
  int timeout_ms_ = 100; // default timeout
};

} // namespace mks
