// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi

#include "hyperdog_locomotion/locomotion_controller.hpp"

#include <algorithm>
#include <cmath>

#include "hyperdog_locomotion/planning/foothold_planner.hpp"

namespace hyperdog_locomotion
{

const char * to_string(Mode m)
{
  switch (m) {
    case Mode::PASSIVE: return "PASSIVE";
    case Mode::STAND_UP: return "STAND_UP";
    case Mode::BALANCE: return "BALANCE";
    case Mode::LOCOMOTION: return "LOCOMOTION";
    case Mode::SIT: return "SIT";
  }
  return "UNKNOWN";
}

namespace
{
Vec12 tile(const Vec3 & v)
{
  Vec12 o;
  for (int i = 0; i < 4; ++i) {
    o.segment<3>(3 * i) = v;
  }
  return o;
}

constexpr double kCrouchHeight = 0.12;        // [m] intermediate pose of the stand-up motion
constexpr double kSitHeight = 0.11;           // [m] final pose of the sit-down motion
constexpr double kMaxBodyRpyCommand = 0.4;    // [rad]
constexpr double kMaxTerrainTilt = 0.35;      // [rad]
constexpr double kMaxPositionError = 0.08;    // [m] max distance of the position reference
constexpr double kMaxYawError = 0.3;          // [rad]
constexpr double kCenteringSpeed = 0.2;       // [m/s] CoM re-centering over the feet while standing
constexpr double kHeightRate = 0.1;           // [m/s] body height command slew rate
constexpr double kFootholdFreeze = 0.85;      // swing progress after which the foothold is frozen
}  // namespace

LocomotionController::LocomotionController(const ControllerConfig & cfg)
: cfg_(cfg),
  dt_(1.0 / cfg.control_rate),
  kin_(cfg.geometry),
  inertia_(cfg.body.inertia.asDiagonal()),
  qp_(cfg.balance.qp, cfg.balance.limits, cfg.body.mass, inertia_),
  mpc_(cfg.balance.mpc, cfg.balance.limits, cfg.body.mass, inertia_),
  legs_(cfg.legs, kin_),
  mahony_(cfg.estimation.mahony_kp, cfg.estimation.mahony_ki),
  kf_(cfg.estimation.kalman, 1.0 / cfg.control_rate),
  contacts_(cfg.estimation.contact),
  ground_(cfg.estimation.ground_plane_filter),
  gait_(cfg.gaits),
  disturbance_(cfg.recovery),
  height_ref_(cfg.locomotion.body_height),
  step_height_(cfg.locomotion.step_height)
{
  mpc_decimation_ = std::max(1, static_cast<int>(std::lround(cfg.balance.mpc.update_period / dt_)));
}

std::vector<std::pair<double, std::string>> LocomotionController::take_events()
{
  auto e = std::move(events_);
  events_.clear();
  return e;
}

// ===================================================================== step
MotorCommand LocomotionController::step(const SensorData & s)
{
  time_ += dt_;
  ++tick_;
  mode_time_ += dt_;
  update_attitude(s);
  feet_body_ = kin_.forward_all(s.q);
  handle_mode(s);
  MotorCommand out;
  switch (mode_) {
    case Mode::PASSIVE:
      out.q = s.q;
      out.kd = tile(cfg_.posture.passive_kd);
      break;
    case Mode::STAND_UP:
      stand_up(s, out);
      break;
    case Mode::SIT:
      sit(s, out);
      break;
    case Mode::BALANCE:
    case Mode::LOCOMOTION:
      locomotion(s, out);
      break;
  }
  const Vec12 lim = tile(cfg_.safety.max_joint_torque);
  out.tau = out.tau.cwiseMax(-lim).cwiseMin(lim);
  return out;
}

// =============================================================== estimation
void LocomotionController::update_attitude(const SensorData & s)
{
  Eigen::Quaterniond q;
  if (cfg_.estimation.attitude_source == "mahony" || !s.imu_orientation) {
    q = mahony_.update(s.gyro, s.accel, dt_);
  } else {
    q = *s.imu_orientation;
  }
  R_ = q.normalized().toRotationMatrix();
  rpy_ = rot_to_rpy(R_);
  omega_body_ = s.gyro;
}

std::array<double, 4> LocomotionController::foot_forces_from_torque(const SensorData & s) const
{
  std::array<double, 4> fz{0, 0, 0, 0};
  const auto J = kin_.jacobians(s.q);
  for (int i = 0; i < 4; ++i) {
    // tau = J^T F_env  ->  F_env = J^-T tau ; ground reaction = -F_env
    const Vec3 f_body = J[i].transpose().fullPivLu().solve(Vec3(s.tau.segment<3>(3 * i)));
    fz[i] = -(R_ * f_body).z();
  }
  return fz;
}

void LocomotionController::update_contacts(const SensorData & s)
{
  const bool use_torque = cfg_.estimation.contact.source == "torque";
  std::array<double, 4> fz{};
  if (use_torque) {fz = foot_forces_from_torque(s);}
  contacts_.update(
    gait_.contact(), gait_.progress(), s.foot_contact ? &*s.foot_contact : nullptr,
    use_torque ? &fz : nullptr);
}

void LocomotionController::estimate_state(const SensorData & s)
{
  update_contacts(s);
  const auto trust = contacts_.trust(gait_.contact(), gait_.progress());
  const auto J = kin_.jacobians(s.q);
  Mat43 feet_vel;
  std::array<double, 4> ground_z;
  for (int i = 0; i < 4; ++i) {
    feet_vel.row(i) = (J[i] * s.dq.segment<3>(3 * i)).transpose();
    const Vec3 f = kf_.foot(i);
    ground_z[i] = ground_.height(f.x(), f.y()) + cfg_.geometry.foot_radius;
  }
  kf_.update(R_, omega_body_, s.accel, feet_body_, feet_vel, trust, ground_z);
  const Vec3 p = kf_.position();
  for (int i = 0; i < 4; ++i) {
    feet_world_.row(i) = (p + R_ * feet_body_.row(i).transpose()).transpose();
  }
  Bool4 use;
  for (int i = 0; i < 4; ++i) {
    use[i] = contacts_.contact()[i] && trust[i] > 0.5;
  }
  ground_.update(feet_world_, use, cfg_.geometry.foot_radius);
}

Eigen::Vector2d LocomotionController::terrain_rp() const
{
  if (!cfg_.locomotion.terrain_adaptation) {return Eigen::Vector2d::Zero();}
  return ground_.slope_rp(rpy_.z()).cwiseMax(-kMaxTerrainTilt).cwiseMin(kMaxTerrainTilt);
}

// ==================================================== finite state machine
void LocomotionController::set_mode(Mode m)
{
  if (m != mode_) {
    events_.emplace_back(time_, std::string(to_string(mode_)) + " -> " + to_string(m));
    mode_ = m;
    mode_time_ = 0.0;
  }
}

void LocomotionController::handle_mode(const SensorData & s)
{
  const Mode req = cmd_.mode;
  const bool active = mode_ == Mode::BALANCE || mode_ == Mode::LOCOMOTION;
  if (cfg_.safety.fall_protection && active &&
    (std::abs(rpy_.x()) > cfg_.safety.fall_angle || std::abs(rpy_.y()) > cfg_.safety.fall_angle))
  {
    events_.emplace_back(time_, "fall detected -> damping");
    set_mode(Mode::PASSIVE);
    cmd_.mode = Mode::PASSIVE;
    return;
  }
  const bool wants_up = req == Mode::BALANCE || req == Mode::LOCOMOTION;
  const bool is_down = mode_ == Mode::PASSIVE || mode_ == Mode::SIT;
  if (req == Mode::PASSIVE) {
    set_mode(Mode::PASSIVE);
  } else if (wants_up && is_down) {
    start_q_ = s.q;
    set_mode(Mode::STAND_UP);
  } else if (req == Mode::SIT && active) {
    start_q_ = s.q;
    set_mode(Mode::SIT);
  } else if (active) {
    set_mode(req == Mode::LOCOMOTION ? Mode::LOCOMOTION : Mode::BALANCE);
  }
}

// ========================================================== posture motions
Vec12 LocomotionController::joint_pose(double height) const
{
  Mat43 feet;
  for (int i = 0; i < 4; ++i) {
    feet.row(i) = kin_.leg(i).nominal_foot(height).transpose();
  }
  Vec12 q;
  kin_.inverse_all(feet, q);
  return q;
}

Vec12 LocomotionController::gravity_feedforward(const SensorData & s) const
{
  Vec12 tau = Vec12::Zero();
  const auto J = kin_.jacobians(s.q);
  const Vec3 f_world(0.0, 0.0, cfg_.body.mass * kGravity / 4.0);
  for (int i = 0; i < 4; ++i) {
    tau.segment<3>(3 * i) = J[i].transpose() * (R_.transpose() * -f_world);
  }
  return tau;
}

void LocomotionController::stand_up(const SensorData & s, MotorCommand & out)
{
  const auto & lc = cfg_.locomotion;
  const double T = lc.stand_up_time;
  const double t = mode_time_;
  const Vec12 crouch = joint_pose(kCrouchHeight);
  const double h = cmd_.body_height > 0.0 ?
    std::clamp(cmd_.body_height, lc.body_height_min, lc.body_height_max) : height_ref_;
  const Vec12 stand = joint_pose(h);
  // phase 1 (40 %): fold the legs under the body, phase 2: push up to the standing height
  double ff = 1.0;
  if (t < 0.4 * T) {
    const double a = smoothstep_cos(t / (0.4 * T));
    out.q = (1 - a) * start_q_ + a * crouch;
    ff = a;
  } else {
    const double a = smoothstep_cos((t - 0.4 * T) / (0.6 * T));
    out.q = (1 - a) * crouch + a * stand;
  }
  out.kp = tile(cfg_.posture.stand_up_kp);
  out.kd = tile(cfg_.posture.stand_up_kd);
  out.tau = ff * gravity_feedforward(s);
  if (t >= T) {
    height_ref_ = h;
    init_balance(s);
    set_mode(cmd_.mode == Mode::LOCOMOTION ? Mode::LOCOMOTION : Mode::BALANCE);
  }
}

void LocomotionController::sit(const SensorData & s, MotorCommand & out)
{
  const double T = cfg_.locomotion.sit_down_time;
  const double a = smoothstep_cos(mode_time_ / T);
  out.q = (1 - a) * start_q_ + a * joint_pose(kSitHeight);
  out.kp = tile(cfg_.posture.stand_up_kp);
  out.kd = tile(cfg_.posture.stand_up_kd);
  out.tau = (1 - a) * gravity_feedforward(s);
  if (mode_time_ > T + 0.3) {
    set_mode(Mode::PASSIVE);
    cmd_.mode = Mode::PASSIVE;
  }
}

void LocomotionController::init_balance(const SensorData & s)
{
  const Mat43 fb = kin_.forward_all(s.q);
  double h = 0.0;
  for (int i = 0; i < 4; ++i) {
    h -= (R_ * fb.row(i).transpose()).z() / 4.0;
  }
  h += cfg_.geometry.foot_radius;
  const Vec3 p0(0.0, 0.0, h);
  Mat43 fw;
  for (int i = 0; i < 4; ++i) {
    fw.row(i) = (p0 + R_ * fb.row(i).transpose()).transpose();
  }
  kf_.reset(p0, fw);
  ground_ = GroundPlaneEstimator(cfg_.estimation.ground_plane_filter);
  ground_.reset(fw, cfg_.geometry.foot_radius);
  feet_world_ = fw;
  p_ref_ = p0;
  yaw_ref_ = rpy_.z();
  anchor_ = fw;
  foothold_ = fw;
  liftoff_ = fw;
  gait_ = GaitScheduler(cfg_.gaits);
  prev_sched_ = {true, true, true, true};
  v_des_.setZero();
  for (int i = 0; i < 4; ++i) {
    f_des_.segment<3>(3 * i) = Vec3(0, 0, cfg_.body.mass * kGravity / 4.0);
  }
  qp_.reset();
  mpc_.reset();
  disturbance_.reset();
  idle_timer_ = 0.0;
}

// =============================================================== locomotion
void LocomotionController::locomotion(const SensorData & s, MotorCommand & out)
{
  const Vec3 target = update_velocity_command();
  estimate_state(s);
  v_des_world_ = rot_z(rpy_.z()) * Vec3(v_des_.x(), v_des_.y(), 0.0);

  select_gait(target);
  gait_.step(dt_);
  update_contacts(s);
  const Bool4 stance = update_leg_phases();
  const Vec3 rpy_ref = update_body_reference();
  update_footholds(stance);
  compute_ground_forces(stance, rpy_ref);

  LegControlInput in;
  in.R = R_;
  in.p = kf_.position();
  in.v = kf_.velocity();
  in.omega_body = omega_body_;
  in.q = s.q;
  in.dq = s.dq;
  in.feet_body = feet_body_;
  in.stance = stance;
  in.late = contacts_.late();
  in.anchor = anchor_;
  in.liftoff = liftoff_;
  in.foothold = foothold_;
  in.progress = gait_.progress();
  in.swing_time = gait_.current().swing_time();
  in.step_height = step_height_;
  in.forces = f_des_;
  for (int i = 0; i < 4; ++i) {
    if (!stance[i]) {in.forces.segment<3>(3 * i).setZero();}
  }
  foot_target_ = legs_.compute(in, out);
  stance_ = stance;
}

Vec3 LocomotionController::update_velocity_command()
{
  const auto & lc = cfg_.locomotion;
  Vec3 target(cmd_.vx, cmd_.vy, cmd_.wz);
  if (mode_ != Mode::LOCOMOTION) {target.setZero();}
  target = target.cwiseMax(-lc.max_velocity).cwiseMin(lc.max_velocity);
  const Vec3 step = lc.max_acceleration * dt_;
  v_des_ += (target - v_des_).cwiseMax(-step).cwiseMin(step);
  if (cmd_.body_height > 0.0) {
    const double h = std::clamp(cmd_.body_height, lc.body_height_min, lc.body_height_max);
    height_ref_ += std::clamp(h - height_ref_, -kHeightRate * dt_, kHeightRate * dt_);
  }
  if (cmd_.step_height > 0.0) {step_height_ = std::clamp(cmd_.step_height, 0.02, 0.12);}
  return target;
}

void LocomotionController::select_gait(const Vec3 & target)
{
  const bool moving = target.cwiseAbs().maxCoeff() > 1e-3 || v_des_.cwiseAbs().maxCoeff() > 0.02;
  idle_timer_ = (mode_ == Mode::LOCOMOTION && moving) ? 0.0 : idle_timer_ + dt_;
  const std::string gait_name = gait_.has(cmd_.gait) ? cmd_.gait : "trot";
  const bool want_walk = gait_name != "stand" && mode_ == Mode::LOCOMOTION &&
    (moving || idle_timer_ < cfg_.locomotion.idle_time_to_stand);

  // push recovery: step automatically when the standing robot is disturbed
  const Vec3 p = kf_.position();
  const Eigen::Vector2d trp = terrain_rp();
  const double tilt = std::max(std::abs(rpy_.x() - trp.x()), std::abs(rpy_.y() - trp.y()));
  const auto event = disturbance_.update(
    !want_walk, p, kf_.velocity(), p.z() - ground_.height(p.x(), p.y()), feet_world_, tilt, dt_);
  if (event == DisturbanceMonitor::Event::DETECTED) {
    events_.emplace_back(time_, "disturbance detected -> auto stepping");
  } else if (event == DisturbanceMonitor::Event::REJECTED) {
    events_.emplace_back(time_, "disturbance rejected -> stand");
  }

  if (want_walk) {
    gait_.request(gait_name);
  } else if (disturbance_.recovering()) {
    const auto & g = disturbance_.params().gait;
    gait_.request(gait_.has(g) ? g : "trot");
  } else {
    gait_.request("stand");
  }
}

Bool4 LocomotionController::update_leg_phases()
{
  const Bool4 sched = gait_.contact();
  const Bool4 early = contacts_.early();
  Bool4 stance;
  for (int i = 0; i < 4; ++i) {
    if (prev_sched_[i] && !sched[i]) {liftoff_.row(i) = feet_world_.row(i);}
    if ((!prev_sched_[i] && sched[i]) || (early[i] && !stance_[i])) {
      anchor_.row(i) = feet_world_.row(i);
    }
    stance[i] = sched[i] || early[i];
  }
  prev_sched_ = sched;
  return stance;
}

Vec3 LocomotionController::update_body_reference()
{
  const Vec3 p = kf_.position();
  const double yaw = rpy_.z();
  const bool recovering = disturbance_.recovering();
  Vec3 body_rpy_cmd = Vec3::Zero();
  if (gait_.is_standing() && !recovering) {
    // keep the CoM above the centre of the support polygon
    const Eigen::Vector2d center = feet_world_.leftCols<2>().colwise().mean().transpose();
    const double st = kCenteringSpeed * dt_;
    p_ref_.head<2>() += (center - p_ref_.head<2>()).cwiseMax(-st).cwiseMin(st);
    if (mode_ == Mode::BALANCE) {
      body_rpy_cmd = cmd_.body_rpy.cwiseMax(-kMaxBodyRpyCommand).cwiseMin(kMaxBodyRpyCommand);
    }
  } else {
    p_ref_.head<2>() += v_des_world_.head<2>() * dt_;
    if (recovering) {
      // do not fight the push with the position loop, damp the velocity instead
      p_ref_.head<2>() = p.head<2>() +
        (p_ref_.head<2>() - p.head<2>()).cwiseMax(-0.03).cwiseMin(0.03);
    }
  }
  const Eigen::Vector2d err = p_ref_.head<2>() - p.head<2>();
  if (err.norm() > kMaxPositionError) {
    p_ref_.head<2>() = p.head<2>() + err * kMaxPositionError / err.norm();
  }
  p_ref_.z() = ground_.height(p.x(), p.y()) + height_ref_;
  yaw_ref_ = wrap_angle(yaw_ref_ + v_des_.z() * dt_);
  const double yerr = wrap_angle(yaw_ref_ - yaw);
  if (std::abs(yerr) > kMaxYawError) {
    yaw_ref_ = wrap_angle(yaw + std::copysign(kMaxYawError, yerr));
  }
  const Eigen::Vector2d trp = terrain_rp();
  return Vec3(
    trp.x() + body_rpy_cmd.x(), trp.y() + body_rpy_cmd.y(),
    wrap_angle(yaw_ref_ + body_rpy_cmd.z()));
}

void LocomotionController::update_footholds(const Bool4 & stance)
{
  const Vec3 p = kf_.position();
  const Vec3 v = kf_.velocity();
  const auto & progress = gait_.progress();
  const double t_stance = gait_.current().stance_time();
  for (int i = 0; i < 4; ++i) {
    if (stance[i]) {
      foothold_.row(i) = anchor_.row(i);
    } else if (progress[i] < kFootholdFreeze) {
      Vec3 nominal = kin_.leg(i).nominal_foot(0.0);
      nominal.z() = 0.0;
      Vec3 f = plan_foothold(
        cfg_.foothold, nominal, p, rpy_.z(), v, v_des_world_, v_des_.z(),
        gait_.swing_remaining(i), t_stance, height_ref_);
      f.z() = ground_.height(f.x(), f.y()) + cfg_.geometry.foot_radius;
      foothold_.row(i) = f.transpose();
    }
  }
}

void LocomotionController::compute_ground_forces(const Bool4 & stance, const Vec3 & rpy_ref)
{
  const double wz_des = v_des_.z();
  const Vec3 normal = cfg_.locomotion.terrain_adaptation ? ground_.normal() : Vec3::UnitZ();
  BodyState bs;
  bs.R = R_;
  bs.rpy = rpy_;
  bs.p = kf_.position();
  bs.v = kf_.velocity();
  bs.omega_world = R_ * omega_body_;
  if (cfg_.balance.controller == "mpc" && !gait_.is_standing()) {
    if (tick_ % mpc_decimation_ == 0 || active_ctrl_ != "mpc") {
      const int N = mpc_.params().horizon;
      const double dtm = mpc_.params().dt;
      std::vector<Bool4> table{stance};
      const auto pred = gait_.contact_table(N - 1, dtm);
      table.insert(table.end(), pred.begin(), pred.end());
      const double yaw = rpy_.z();
      Eigen::MatrixXd ref(N, 12);
      for (int k = 0; k < N; ++k) {
        const double tk = k * dtm;
        const double yaw_k = yaw + wrap_angle(yaw_ref_ + wz_des * tk - yaw);
        ref.row(k).segment<3>(0) = Vec3(rpy_ref.x(), rpy_ref.y(), yaw_k).transpose();
        ref.row(k).segment<3>(3) = (p_ref_ + v_des_world_ * tk).transpose();
        ref.row(k).segment<3>(6) = Vec3(0.0, 0.0, wz_des).transpose();
        ref.row(k).segment<3>(9) = v_des_world_.transpose();
      }
      Mat43 feet;
      for (int i = 0; i < 4; ++i) {
        feet.row(i) = stance[i] ? feet_world_.row(i) : foothold_.row(i);
      }
      f_des_ = mpc_.compute(bs, ref, feet, table, normal);
      solve_time_ = mpc_.solve_time();
    }
    active_ctrl_ = "mpc";
  } else {
    f_des_ = qp_.compute(
      bs, rpy_to_rot(rpy_ref), p_ref_, v_des_world_, Vec3(0.0, 0.0, wz_des),
      feet_world_, stance, normal);
    solve_time_ = qp_.solve_time();
    active_ctrl_ = "qp";
  }
}

// ============================================================== diagnostics
Diagnostics LocomotionController::diagnostics() const
{
  Diagnostics d;
  d.mode = mode_;
  d.gait = gait_.current().name;
  d.controller = active_ctrl_;
  d.contact = stance_;
  d.scheduled_contact = gait_.contact();
  d.phase = gait_.leg_phase();
  d.foot_force = f_des_;
  d.rpy = rpy_;
  d.position = kf_.position();
  d.velocity = kf_.velocity();
  d.omega = omega_body_;
  d.height = kf_.position().z() - ground_.height(kf_.position().x(), kf_.position().y());
  d.recovering = disturbance_.recovering();
  d.solve_time_ms = 1e3 * solve_time_;
  d.feet_world = feet_world_;
  d.foot_target = foot_target_;
  return d;
}

}  // namespace hyperdog_locomotion
