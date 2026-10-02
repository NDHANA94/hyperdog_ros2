// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi

#include "hyperdog_locomotion/locomotion_controller.hpp"

#include <cmath>

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
  for (int i = 0; i < 4; ++i) {o.segment<3>(3 * i) = v;}
  return o;
}
}  // namespace

LocomotionController::LocomotionController(const ControllerConfig & cfg)
: cfg_(cfg),
  dt_(1.0 / cfg.control_rate),
  kin_(cfg.geometry),
  inertia_(cfg.body_inertia.asDiagonal()),
  qp_(cfg.qp, cfg.contact_limits, cfg.mass, inertia_),
  mpc_(cfg.mpc, cfg.contact_limits, cfg.mass, inertia_),
  mahony_(cfg.mahony_kp, cfg.mahony_ki),
  kf_(cfg.kalman, 1.0 / cfg.control_rate),
  contacts_(cfg.contact),
  ground_(cfg.ground_plane_filter),
  gait_(cfg.gaits),
  height_ref_(cfg.body_height),
  step_height_(cfg.step_height)
{
  mpc_decimation_ = std::max(1, static_cast<int>(std::lround(cfg.mpc.update_period / dt_)));
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
      out.kd = tile(cfg_.passive_kd);
      break;
    case Mode::STAND_UP:
      stand_up(s, out);
      break;
    case Mode::SIT:
      sit(s, out);
      break;
    default:
      locomotion(s, out);
  }
  const Vec12 lim = tile(cfg_.max_joint_torque);
  out.tau = out.tau.cwiseMax(-lim).cwiseMin(lim);
  return out;
}

// =============================================================== estimation
void LocomotionController::update_attitude(const SensorData & s)
{
  Eigen::Quaterniond q;
  if (cfg_.attitude_source == "mahony" || !s.imu_orientation) {
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

void LocomotionController::estimate(const SensorData & s)
{
  const Bool4 sched = gait_.contact();
  const auto progress = gait_.progress();
  std::array<double, 4> fz{};
  const bool use_torque = cfg_.contact.source == "torque";
  if (use_torque) {fz = foot_forces_from_torque(s);}
  contacts_.update(sched, progress, s.foot_contact ? &*s.foot_contact : nullptr,
    use_torque ? &fz : nullptr);
  const auto trust = contacts_.trust(sched, progress);
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
  for (int i = 0; i < 4; ++i) {use[i] = contacts_.contact()[i] && trust[i] > 0.5;}
  ground_.update(feet_world_, use, cfg_.geometry.foot_radius);
}

// ================================================================ modes
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
  if (cfg_.fall_protection && (mode_ == Mode::BALANCE || mode_ == Mode::LOCOMOTION) &&
    (std::abs(rpy_.x()) > cfg_.fall_angle || std::abs(rpy_.y()) > cfg_.fall_angle))
  {
    events_.emplace_back(time_, "fall detected -> damping");
    set_mode(Mode::PASSIVE);
    cmd_.mode = Mode::PASSIVE;
    return;
  }
  if (req == Mode::PASSIVE) {
    set_mode(Mode::PASSIVE);
  } else if ((req == Mode::BALANCE || req == Mode::LOCOMOTION) &&
    (mode_ == Mode::PASSIVE || mode_ == Mode::SIT))
  {
    start_q_ = s.q;
    set_mode(Mode::STAND_UP);
  } else if (req == Mode::SIT && (mode_ == Mode::BALANCE || mode_ == Mode::LOCOMOTION)) {
    start_q_ = s.q;
    set_mode(Mode::SIT);
  } else if (mode_ == Mode::BALANCE || mode_ == Mode::LOCOMOTION) {
    set_mode(req == Mode::LOCOMOTION ? Mode::LOCOMOTION : Mode::BALANCE);
  }
}

Vec12 LocomotionController::joint_pose(double height) const
{
  Mat43 feet;
  for (int i = 0; i < 4; ++i) {feet.row(i) = kin_.leg(i).nominal_foot(height).transpose();}
  Vec12 q;
  kin_.inverse_all(feet, q);
  return q;
}

Vec12 LocomotionController::gravity_feedforward(const SensorData & s) const
{
  Vec12 tau = Vec12::Zero();
  const auto J = kin_.jacobians(s.q);
  const Vec3 f_world(0.0, 0.0, cfg_.mass * kGravity / 4.0);
  for (int i = 0; i < 4; ++i) {
    tau.segment<3>(3 * i) = J[i].transpose() * (R_.transpose() * -f_world);
  }
  return tau;
}

void LocomotionController::stand_up(const SensorData & s, MotorCommand & out)
{
  const double T = cfg_.stand_up_time;
  const double t = mode_time_;
  const Vec12 crouch = joint_pose(0.12);
  const double h = cmd_.body_height > 0.0 ?
    std::clamp(cmd_.body_height, cfg_.body_height_min, cfg_.body_height_max) : height_ref_;
  const Vec12 stand = joint_pose(h);
  double ff = 1.0;
  if (t < 0.4 * T) {
    const double a = smoothstep_cos(t / (0.4 * T));
    out.q = (1 - a) * start_q_ + a * crouch;
    ff = a;
  } else {
    const double a = smoothstep_cos((t - 0.4 * T) / (0.6 * T));
    out.q = (1 - a) * crouch + a * stand;
  }
  out.kp = tile(cfg_.stand_up_kp);
  out.kd = tile(cfg_.stand_up_kd);
  out.tau = ff * gravity_feedforward(s);
  if (t >= T) {
    height_ref_ = h;
    init_balance(s);
    set_mode(cmd_.mode == Mode::LOCOMOTION ? Mode::LOCOMOTION : Mode::BALANCE);
  }
}

void LocomotionController::sit(const SensorData & s, MotorCommand & out)
{
  const double a = smoothstep_cos(mode_time_ / cfg_.sit_down_time);
  out.q = (1 - a) * start_q_ + a * joint_pose(0.11);
  out.kp = tile(cfg_.stand_up_kp);
  out.kd = tile(cfg_.stand_up_kd);
  out.tau = (1 - a) * gravity_feedforward(s);
  if (mode_time_ > cfg_.sit_down_time + 0.3) {
    set_mode(Mode::PASSIVE);
    cmd_.mode = Mode::PASSIVE;
  }
}

void LocomotionController::init_balance(const SensorData & s)
{
  const Mat43 fb = kin_.forward_all(s.q);
  double h = 0.0;
  for (int i = 0; i < 4; ++i) {h -= (R_ * fb.row(i).transpose()).z() / 4.0;}
  h += cfg_.geometry.foot_radius;
  const Vec3 p0(0.0, 0.0, h);
  Mat43 fw;
  for (int i = 0; i < 4; ++i) {fw.row(i) = (p0 + R_ * fb.row(i).transpose()).transpose();}
  kf_.reset(p0, fw);
  ground_ = GroundPlaneEstimator(cfg_.ground_plane_filter);
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
  for (int i = 0; i < 4; ++i) {f_des_.segment<3>(3 * i) = Vec3(0, 0, cfg_.mass * kGravity / 4.0);}
  qp_.reset();
  mpc_.reset();
  recovering_ = false;
  idle_timer_ = 0.0;
}

// ============================================================ locomotion
Vec3 LocomotionController::update_velocity_command()
{
  Vec3 target(cmd_.vx, cmd_.vy, cmd_.wz);
  if (mode_ != Mode::LOCOMOTION) {target.setZero();}
  target = target.cwiseMax(-cfg_.max_velocity).cwiseMin(cfg_.max_velocity);
  const Vec3 step = cfg_.max_acceleration * dt_;
  v_des_ += (target - v_des_).cwiseMax(-step).cwiseMin(step);
  if (cmd_.body_height > 0.0) {
    const double h = std::clamp(cmd_.body_height, cfg_.body_height_min, cfg_.body_height_max);
    height_ref_ += std::clamp(h - height_ref_, -0.1 * dt_, 0.1 * dt_);
  }
  if (cmd_.step_height > 0.0) {step_height_ = std::clamp(cmd_.step_height, 0.02, 0.12);}
  return target;
}

Eigen::Vector2d LocomotionController::terrain_rp() const
{
  if (!cfg_.terrain_adaptation) {return Eigen::Vector2d::Zero();}
  return ground_.slope_rp(rpy_.z()).cwiseMax(-0.35).cwiseMin(0.35);
}

void LocomotionController::select_gait(const Vec3 & target)
{
  const bool moving = target.cwiseAbs().maxCoeff() > 1e-3 || v_des_.cwiseAbs().maxCoeff() > 0.02;
  if (mode_ == Mode::LOCOMOTION && moving) {
    idle_timer_ = 0.0;
  } else {
    idle_timer_ += dt_;
  }
  std::string gait_name = gait_.has(cmd_.gait) ? cmd_.gait : "trot";
  bool want_walk = mode_ == Mode::LOCOMOTION && (moving || idle_timer_ < cfg_.idle_time_to_stand);
  if (gait_name == "stand") {want_walk = false;}

  // ---- disturbance detection: capture point leaving the support polygon,
  //      large body velocity or tilt -> step automatically (push recovery)
  const Vec3 p = kf_.position();
  const Vec3 v = kf_.velocity();
  if (cfg_.recovery_enabled && !want_walk) {
    const double h = std::max(p.z() - ground_.height(p.x(), p.y()), 0.1);
    const Eigen::Vector2d cp = p.head<2>() + v.head<2>() * std::sqrt(h / kGravity);
    const Eigen::Vector2d lo = feet_world_.leftCols<2>().colwise().minCoeff().transpose().array() +
      cfg_.recovery_capture_margin;
    const Eigen::Vector2d hi = feet_world_.leftCols<2>().colwise().maxCoeff().transpose().array() -
      cfg_.recovery_capture_margin;
    const bool outside = (cp.array() < lo.array()).any() || (cp.array() > hi.array()).any();
    const Eigen::Vector2d trp = terrain_rp();
    const double tilt = std::max(std::abs(rpy_.x() - trp.x()), std::abs(rpy_.y() - trp.y()));
    const double speed = v.head<2>().norm();
    if (!recovering_ && (outside || speed > cfg_.recovery_velocity_threshold ||
      tilt > cfg_.recovery_tilt_threshold))
    {
      recovering_ = true;
      settle_timer_ = 0.0;
      events_.emplace_back(time_, "disturbance detected -> auto stepping");
    }
    if (recovering_) {
      if (speed < cfg_.recovery_settle_velocity && tilt < 0.5 * cfg_.recovery_tilt_threshold && !outside) {
        settle_timer_ += dt_;
      } else {
        settle_timer_ = 0.0;
      }
      if (settle_timer_ > cfg_.recovery_settle_time) {
        recovering_ = false;
        events_.emplace_back(time_, "disturbance rejected -> stand");
      }
    }
  } else {
    recovering_ = false;
  }

  if (want_walk) {
    gait_.request(gait_name);
  } else if (recovering_) {
    gait_.request(gait_.has(cfg_.recovery_gait) ? cfg_.recovery_gait : "trot");
  } else {
    gait_.request("stand");
  }
}

void LocomotionController::locomotion(const SensorData & s, MotorCommand & out)
{
  const Vec3 target = update_velocity_command();
  estimate(s);
  const Vec3 p = kf_.position();
  const Vec3 v = kf_.velocity();
  const double yaw = rpy_.z();
  const Vec3 v_des_w = rot_z(yaw) * Vec3(v_des_.x(), v_des_.y(), 0.0);
  const double wz_des = v_des_.z();

  select_gait(target);
  gait_.step(dt_);
  const Bool4 sched = gait_.contact();
  const auto progress = gait_.progress();
  std::array<double, 4> fz{};
  const bool use_torque = cfg_.contact.source == "torque";
  if (use_torque) {fz = foot_forces_from_torque(s);}
  contacts_.update(sched, progress, s.foot_contact ? &*s.foot_contact : nullptr,
    use_torque ? &fz : nullptr);
  const Bool4 early = contacts_.early();
  const Bool4 late = contacts_.late();

  // ---- leg phase transitions
  Bool4 stance;
  for (int i = 0; i < 4; ++i) {
    if (prev_sched_[i] && !sched[i]) {liftoff_.row(i) = feet_world_.row(i);}
    if ((!prev_sched_[i] && sched[i]) || (early[i] && !stance_[i])) {anchor_.row(i) = feet_world_.row(i);}
    stance[i] = sched[i] || early[i];
  }
  prev_sched_ = sched;

  // ---- body reference
  const double ground_z = ground_.height(p.x(), p.y());
  const Eigen::Vector2d trp = terrain_rp();
  Vec3 body_rpy_cmd = Vec3::Zero();
  if (gait_.is_standing() && !recovering_ && mode_ == Mode::BALANCE) {
    // keep the CoM above the centre of the support polygon
    const Eigen::Vector2d center = feet_world_.leftCols<2>().colwise().mean().transpose();
    const double st = 0.2 * dt_;
    p_ref_.head<2>() += (center - p_ref_.head<2>()).cwiseMax(-st).cwiseMin(st);
    body_rpy_cmd = cmd_.body_rpy.cwiseMax(-0.4).cwiseMin(0.4);
  } else if (gait_.is_standing() && !recovering_) {
    const Eigen::Vector2d center = feet_world_.leftCols<2>().colwise().mean().transpose();
    const double st = 0.2 * dt_;
    p_ref_.head<2>() += (center - p_ref_.head<2>()).cwiseMax(-st).cwiseMin(st);
  } else {
    p_ref_.head<2>() += v_des_w.head<2>() * dt_;
    if (recovering_) {
      // do not fight the push with the position loop, damp the velocity instead
      p_ref_.head<2>() = p.head<2>() + (p_ref_.head<2>() - p.head<2>()).cwiseMax(-0.03).cwiseMin(0.03);
    }
  }
  const Eigen::Vector2d err = p_ref_.head<2>() - p.head<2>();
  if (err.norm() > 0.08) {p_ref_.head<2>() = p.head<2>() + err * 0.08 / err.norm();}
  p_ref_.z() = ground_z + height_ref_;
  yaw_ref_ = wrap_angle(yaw_ref_ + wz_des * dt_);
  const double yerr = wrap_angle(yaw_ref_ - yaw);
  if (std::abs(yerr) > 0.3) {yaw_ref_ = wrap_angle(yaw + std::copysign(0.3, yerr));}
  const Vec3 rpy_ref(trp.x() + body_rpy_cmd.x(), trp.y() + body_rpy_cmd.y(),
    wrap_angle(yaw_ref_ + body_rpy_cmd.z()));
  const Mat3 R_ref = rpy_to_rot(rpy_ref);
  const Vec3 omega_w = R_ * omega_body_;

  // ---- footholds
  const double t_stance = gait_.current().stance_time();
  const double t_swing = gait_.current().swing_time();
  for (int i = 0; i < 4; ++i) {
    if (!stance[i]) {
      if (progress[i] < 0.85) {
        Vec3 nominal = kin_.leg(i).nominal_foot(0.0);
        nominal.z() = 0.0;
        Vec3 f = plan_foothold(cfg_.foothold, nominal, p, yaw, v, v_des_w, wz_des,
            gait_.swing_remaining(i), t_stance, height_ref_);
        f.z() = ground_.height(f.x(), f.y()) + cfg_.geometry.foot_radius;
        foothold_.row(i) = f.transpose();
      }
    } else {
      foothold_.row(i) = anchor_.row(i);
    }
  }

  // ---- ground reaction forces
  const Vec3 normal = cfg_.terrain_adaptation ? ground_.normal() : Vec3::UnitZ();
  BodyState bs;
  bs.R = R_;
  bs.rpy = rpy_;
  bs.p = p;
  bs.v = v;
  bs.omega_world = omega_w;
  if (cfg_.balance_controller == "mpc" && !gait_.is_standing()) {
    if (tick_ % mpc_decimation_ == 0 || active_ctrl_ != "mpc") {
      const int N = mpc_.params().horizon;
      const double dtm = mpc_.params().dt;
      std::vector<Bool4> table{stance};
      const auto pred = gait_.contact_table(N - 1, dtm);
      table.insert(table.end(), pred.begin(), pred.end());
      Eigen::MatrixXd ref(N, 12);
      for (int k = 0; k < N; ++k) {
        const double tk = k * dtm;
        const double yaw_k = yaw + wrap_angle(yaw_ref_ + wz_des * tk - yaw);
        ref.row(k).segment<3>(0) = Vec3(rpy_ref.x(), rpy_ref.y(), yaw_k).transpose();
        ref.row(k).segment<3>(3) = (p_ref_ + Vec3(v_des_w.x(), v_des_w.y(), 0.0) * tk).transpose();
        ref.row(k).segment<3>(6) = Vec3(0.0, 0.0, wz_des).transpose();
        ref.row(k).segment<3>(9) = v_des_w.transpose();
      }
      Mat43 feet_mpc;
      for (int i = 0; i < 4; ++i) {feet_mpc.row(i) = stance[i] ? feet_world_.row(i) : foothold_.row(i);}
      f_des_ = mpc_.compute(bs, ref, feet_mpc, table, normal);
      solve_time_ = mpc_.solve_time();
    }
    active_ctrl_ = "mpc";
  } else {
    f_des_ = qp_.compute(bs, R_ref, p_ref_, Vec3(v_des_w.x(), v_des_w.y(), 0.0),
        Vec3(0.0, 0.0, wz_des), feet_world_, stance, normal);
    solve_time_ = qp_.solve_time();
    active_ctrl_ = "qp";
  }
  Vec12 f = f_des_;
  for (int i = 0; i < 4; ++i) {
    if (!stance[i]) {f.segment<3>(3 * i).setZero();}
  }

  // ---- joint level commands
  const auto J = kin_.jacobians(s.q);
  const Mat3 RT = R_.transpose();
  const Vec3 g_comp = RT * Vec3(0.0, 0.0, cfg_.leg_gravity_compensation_mass * kGravity);
  for (int i = 0; i < 4; ++i) {
    Vec3 p_b, v_b, tau, kp, kd;
    if (stance[i]) {
      Vec3 target_w = anchor_.row(i).transpose();
      if (late[i]) {
        // expected touchdown did not happen yet: keep reaching down
        target_w.z() -= 0.02;
        kp = cfg_.swing_kp;
        kd = cfg_.swing_kd;
        tau = J[i].transpose() * g_comp;
      } else {
        kp = cfg_.stance_kp;
        kd = cfg_.stance_kd;
        tau = J[i].transpose() * (RT * -Vec3(f.segment<3>(3 * i)));
      }
      foot_target_.row(i) = target_w.transpose();
      p_b = RT * (target_w - p);
      v_b = -RT * v - omega_body_.cross(p_b);
    } else {
      const SwingSample sw = swing_trajectory(liftoff_.row(i).transpose(), foothold_.row(i).transpose(),
          step_height_, progress[i], t_swing, cfg_.foothold.touchdown_depth);
      foot_target_.row(i) = sw.pos.transpose();
      p_b = RT * (sw.pos - p);
      v_b = RT * (sw.vel - v) - omega_body_.cross(p_b);
      kp = cfg_.swing_kp;
      kd = cfg_.swing_kd;
      const Vec3 v_foot = J[i] * s.dq.segment<3>(3 * i);
      Vec3 f_c = cfg_.swing_cartesian_kp.cwiseProduct(p_b - feet_body_.row(i).transpose()) +
        cfg_.swing_cartesian_kd.cwiseProduct(v_b - v_foot);
      f_c += g_comp + cfg_.leg_gravity_compensation_mass * (RT * sw.acc);
      tau = J[i].transpose() * f_c;
    }
    Vec3 q_des;
    kin_.leg(i).inverse(p_b, q_des);
    Vec3 dq_des = J[i].fullPivLu().solve(v_b);
    if (!dq_des.allFinite()) {dq_des.setZero();}
    out.q.segment<3>(3 * i) = q_des;
    out.dq.segment<3>(3 * i) = dq_des.cwiseMax(-20.0).cwiseMin(20.0);
    out.kp.segment<3>(3 * i) = kp;
    out.kd.segment<3>(3 * i) = kd;
    out.tau.segment<3>(3 * i) = tau;
  }
  stance_ = stance;
}

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
  d.recovering = recovering_;
  d.solve_time_ms = 1e3 * solve_time_;
  d.feet_world = feet_world_;
  d.foot_target = foot_target_;
  return d;
}

}  // namespace hyperdog_locomotion
