// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
//
// Packing / unpacking of the widely used "MIT mini-cheetah" CAN protocol for
// BLDC actuator drivers (CubeMars AK series, MyActuator, SteadyWin, ...).
// Ranges are per motor type and configured in the hardware parameters.

#ifndef HYPERDOG_BLDC_CONTROL__MIT_CAN_PROTOCOL_HPP_
#define HYPERDOG_BLDC_CONTROL__MIT_CAN_PROTOCOL_HPP_

#include <algorithm>
#include <array>
#include <cstdint>

namespace hyperdog_bldc_control
{

struct MitRanges
{
  double p_max{12.5};   // [rad]
  double v_max{50.0};   // [rad/s]
  double kp_max{500.0};
  double kd_max{5.0};
  double t_max{25.0};   // [Nm]
};

struct MitFeedback
{
  uint8_t id{0};
  double position{0.0};
  double velocity{0.0};
  double torque{0.0};
  int temperature{0};
  int error{0};
};

inline uint32_t mit_float_to_uint(double x, double lo, double hi, int bits)
{
  x = std::clamp(x, lo, hi);
  const double span = hi - lo;
  return static_cast<uint32_t>((x - lo) * static_cast<double>((1u << bits) - 1) / span);
}

inline double mit_uint_to_float(uint32_t x, double lo, double hi, int bits)
{
  const double span = hi - lo;
  return static_cast<double>(x) * span / static_cast<double>((1u << bits) - 1) + lo;
}

/// 8 byte MIT-mode command frame: position 16 bit, velocity 12, kp 12, kd 12, torque 12.
inline std::array<uint8_t, 8> mit_pack_command(
  const MitRanges & r, double p, double v, double kp, double kd, double t)
{
  const uint32_t p_i = mit_float_to_uint(p, -r.p_max, r.p_max, 16);
  const uint32_t v_i = mit_float_to_uint(v, -r.v_max, r.v_max, 12);
  const uint32_t kp_i = mit_float_to_uint(kp, 0.0, r.kp_max, 12);
  const uint32_t kd_i = mit_float_to_uint(kd, 0.0, r.kd_max, 12);
  const uint32_t t_i = mit_float_to_uint(t, -r.t_max, r.t_max, 12);
  std::array<uint8_t, 8> d{};
  d[0] = static_cast<uint8_t>(p_i >> 8);
  d[1] = static_cast<uint8_t>(p_i & 0xFF);
  d[2] = static_cast<uint8_t>(v_i >> 4);
  d[3] = static_cast<uint8_t>(((v_i & 0xF) << 4) | (kp_i >> 8));
  d[4] = static_cast<uint8_t>(kp_i & 0xFF);
  d[5] = static_cast<uint8_t>(kd_i >> 4);
  d[6] = static_cast<uint8_t>(((kd_i & 0xF) << 4) | (t_i >> 8));
  d[7] = static_cast<uint8_t>(t_i & 0xFF);
  return d;
}

/// Reply frame: id, position 16 bit, velocity 12, torque 12, [temperature, error].
inline MitFeedback mit_unpack_reply(const MitRanges & r, const uint8_t * d, uint8_t len)
{
  MitFeedback fb;
  fb.id = d[0];
  const uint32_t p_i = (static_cast<uint32_t>(d[1]) << 8) | d[2];
  const uint32_t v_i = (static_cast<uint32_t>(d[3]) << 4) | (d[4] >> 4);
  const uint32_t t_i = (static_cast<uint32_t>(d[4] & 0xF) << 8) | d[5];
  fb.position = mit_uint_to_float(p_i, -r.p_max, r.p_max, 16);
  fb.velocity = mit_uint_to_float(v_i, -r.v_max, r.v_max, 12);
  fb.torque = mit_uint_to_float(t_i, -r.t_max, r.t_max, 12);
  if (len >= 8) {
    fb.temperature = static_cast<int>(d[6]) - 40;
    fb.error = d[7];
  }
  return fb;
}

inline std::array<uint8_t, 8> mit_enter_motor_mode() {return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};}
inline std::array<uint8_t, 8> mit_exit_motor_mode() {return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};}
inline std::array<uint8_t, 8> mit_set_zero() {return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};}

}  // namespace hyperdog_bldc_control

#endif  // HYPERDOG_BLDC_CONTROL__MIT_CAN_PROTOCOL_HPP_
