// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
//
// Electro-mechanical model of a geared BLDC (PMSM) actuator driven by a FOC
// motor driver. Used by the BLDC impedance controller to turn a requested
// joint torque into the torque a real actuator would actually deliver:
//
//   * q-axis current from torque:   i = tau / (Kt * N * eta)
//   * current limit (with thermal derating of the winding)
//   * voltage limit / back-EMF:      |R i + Ke w_rotor| <= V_available
//     -> realistic torque-speed envelope
//   * current loop dynamics (first order, configurable bandwidth)
//   * gearbox efficiency, coulomb + viscous friction
//   * lumped thermal model of the winding (copper losses)
//
// The model is header only and has no ROS dependency, so it can be unit tested
// and reused by the real-hardware interface.

#ifndef HYPERDOG_BLDC_CONTROL__BLDC_MOTOR_MODEL_HPP_
#define HYPERDOG_BLDC_CONTROL__BLDC_MOTOR_MODEL_HPP_

#include <algorithm>
#include <cmath>
#include <string>

namespace hyperdog_bldc_control
{

struct BldcMotorParams
{
  std::string name{"default"};
  // electrical
  double kv_rpm_per_volt{100.0};     // speed constant (rotor side); used if torque_constant <= 0
  double torque_constant{0.123};     // Kt [Nm/A] rotor side (q-axis)
  double phase_resistance{0.17};     // [Ohm]
  double phase_inductance{5.7e-5};   // [H] (informational, electrical time constant)
  int pole_pairs{14};
  double bus_voltage{48.0};          // [V]
  double voltage_utilization{0.577};  // usable bus fraction for the q-axis (SVPWM ~ 1/sqrt(3))
  double max_current{25.0};          // [A] peak q-axis current
  double rated_current{8.0};         // [A] continuous current
  double current_loop_bandwidth{1000.0};  // [Hz]
  // mechanical
  double gear_ratio{10.0};
  double gearbox_efficiency{0.9};
  double rotor_inertia{6.0e-5};      // [kg m^2] rotor side (reflected = J * N^2)
  double coulomb_friction{0.05};     // [Nm] output side
  double viscous_friction{0.01};     // [Nm s/rad] output side
  double max_velocity{0.0};          // [rad/s] output; <= 0 -> derived from back-EMF
  // thermal
  double thermal_resistance{2.5};    // [K/W] winding -> ambient
  double thermal_capacitance{50.0};  // [J/K]
  double ambient_temperature{25.0};  // [degC]
  double derate_start_temperature{90.0};  // [degC] current derating starts
  double max_winding_temperature{120.0};  // [degC] current limited to zero
  double mass{0.52};                 // [kg] (informational, used by the URDF)

  double kt() const
  {
    if (torque_constant > 0.0) {return torque_constant;}
    return 60.0 / (2.0 * M_PI * std::max(kv_rpm_per_volt, 1e-6));
  }
  double ke() const {return kt();}   // q-axis back-EMF constant [V s/rad] (SI units: Ke == Kt)
  double available_voltage() const {return bus_voltage * voltage_utilization;}
  double peak_torque() const {return kt() * gear_ratio * gearbox_efficiency * max_current;}
  double rated_torque() const {return kt() * gear_ratio * gearbox_efficiency * rated_current;}
  double no_load_speed() const
  {
    if (max_velocity > 0.0) {return max_velocity;}
    return available_voltage() / ke() / gear_ratio;
  }
};

struct BldcMotorOutput
{
  double torque{0.0};           // delivered output torque [Nm]
  double current{0.0};          // q-axis current [A]
  double electrical_power{0.0};  // [W]
  double temperature{25.0};     // [degC]
  bool saturated{false};
};

class BldcMotorModel
{
public:
  BldcMotorModel() {reset();}
  explicit BldcMotorModel(const BldcMotorParams & p)
  : params_(p) {reset();}

  void set_params(const BldcMotorParams & p) {params_ = p; reset();}
  const BldcMotorParams & params() const {return params_;}

  void reset()
  {
    current_ = 0.0;
    temperature_ = params_.ambient_temperature;
  }

  /// Current limit after thermal derating.
  double current_limit() const
  {
    const auto & p = params_;
    if (temperature_ <= p.derate_start_temperature) {return p.max_current;}
    const double span = std::max(p.max_winding_temperature - p.derate_start_temperature, 1e-3);
    const double s = std::clamp(1.0 - (temperature_ - p.derate_start_temperature) / span, 0.0, 1.0);
    return p.max_current * s;
  }

  /// Advance the model by dt with a requested output torque at joint speed `velocity`.
  /// When `simulate_dynamics` is false only the static limits are applied (real hardware:
  /// the driver closes the current loop).
  BldcMotorOutput update(
    double torque_request, double velocity, double dt,
    bool simulate_dynamics = true)
  {
    const auto & p = params_;
    BldcMotorOutput out;
    const double n = p.gear_ratio;
    const double kt = p.kt();
    const double w_rotor = velocity * n;
    // Torque -> current. The gearbox loses efficiency in both directions.
    const double eta = std::clamp(p.gearbox_efficiency, 0.05, 1.0);
    double i_cmd = torque_request / (kt * n * eta);

    // current limit (thermal derating)
    const double i_lim = current_limit();
    double i_sat = std::clamp(i_cmd, -i_lim, i_lim);
    // voltage limit: R i + Ke w within +-V
    const double v = p.available_voltage();
    const double r = std::max(p.phase_resistance, 1e-6);
    const double i_v_hi = (v - p.ke() * w_rotor) / r;
    const double i_v_lo = (-v - p.ke() * w_rotor) / r;
    i_sat = std::clamp(i_sat, std::min(i_v_lo, i_v_hi), std::max(i_v_lo, i_v_hi));
    out.saturated = std::abs(i_sat - i_cmd) > 1e-6;

    if (simulate_dynamics && dt > 0.0) {
      const double a = 1.0 - std::exp(-2.0 * M_PI * p.current_loop_bandwidth * dt);
      current_ += a * (i_sat - current_);
    } else {
      current_ = i_sat;
    }

    double tau = current_ * kt * n * eta;
    if (simulate_dynamics) {
      // friction of the gearbox / bearings (smooth coulomb model)
      tau -= p.coulomb_friction * std::tanh(velocity / 0.05) + p.viscous_friction * velocity;
    }
    // copper losses and thermal model (3 phase, amplitude invariant -> 1.5 R i^2)
    const double p_cu = 1.5 * r * current_ * current_;
    if (dt > 0.0) {
      const double dT = (p_cu - (temperature_ - p.ambient_temperature) / p.thermal_resistance) /
        std::max(p.thermal_capacitance, 1e-6);
      temperature_ += dT * dt;
    }
    out.torque = tau;
    out.current = current_;
    out.electrical_power = 1.5 * p.ke() * current_ * w_rotor + p_cu;
    out.temperature = temperature_;
    return out;
  }

  double temperature() const {return temperature_;}

private:
  BldcMotorParams params_;
  double current_{0.0};
  double temperature_{25.0};
};

}  // namespace hyperdog_bldc_control

#endif  // HYPERDOG_BLDC_CONTROL__BLDC_MOTOR_MODEL_HPP_
