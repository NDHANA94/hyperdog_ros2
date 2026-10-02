// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
//
// Leg kinematics consistent with hyperdog_description/urdf:
//   hip  (ab/ad) axis +x (left) / -x (right) at the body corner
//   uleg (thigh) axis -y, offset laterally by abad_length
//   lleg (knee)  axis +y, upper_length behind the thigh joint (at q = 0)
//   foot sphere centre lower_length along +x of the knee frame
// Leg order everywhere: FR, FL, BR, BL. Joint order per leg: hip, uleg, lleg.

#ifndef HYPERDOG_LOCOMOTION__KINEMATICS__LEG_KINEMATICS_HPP_
#define HYPERDOG_LOCOMOTION__KINEMATICS__LEG_KINEMATICS_HPP_

#include <array>
#include <string>
#include <vector>

#include "hyperdog_locomotion/common/math.hpp"

namespace hyperdog_locomotion
{

constexpr std::array<const char *, 4> kLegNames{"FR", "FL", "BR", "BL"};

std::vector<std::string> joint_names();

struct RobotGeometry
{
  double hip_x{0.175};
  double hip_y{0.066};
  double abad_length{0.104};
  double upper_length{0.15};
  double lower_length{0.14};
  double foot_radius{0.02};
  Vec3 q_min{-1.0, -1.2217, 0.45};
  Vec3 q_max{1.0, 3.14, 2.3562};
};

class LegKinematics
{
public:
  LegKinematics() = default;
  LegKinematics(int index, const RobotGeometry & g);

  Vec3 forward(const Vec3 & q) const;
  Mat3 jacobian(const Vec3 & q) const;
  /// Analytic IK; unreachable targets are projected onto the workspace. Returns reachability.
  bool inverse(const Vec3 & p_foot, Vec3 & q) const;
  /// Foot right below the thigh joint for a base height (body frame).
  Vec3 nominal_foot(double height) const;

  double side() const {return side_;}
  const Vec3 & hip_offset() const {return hip_;}

private:
  void frames(const Vec3 & q, std::array<Vec3, 3> & p, std::array<Vec3, 3> & axes, Vec3 & foot) const;

  double side_{1.0};
  Vec3 hip_{Vec3::Zero()};
  double l1_{0.0}, l2_{0.0}, l3_{0.0};
  Vec3 q_min_, q_max_;
  std::array<Vec3, 3> axes_;
};

class RobotKinematics
{
public:
  explicit RobotKinematics(const RobotGeometry & g = RobotGeometry());
  Mat43 forward_all(const Vec12 & q) const;
  std::array<Mat3, 4> jacobians(const Vec12 & q) const;
  bool inverse_all(const Mat43 & feet, Vec12 & q) const;
  const LegKinematics & leg(int i) const {return legs_[i];}
  const RobotGeometry & geometry() const {return geom_;}

private:
  RobotGeometry geom_;
  std::array<LegKinematics, 4> legs_;
};

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__KINEMATICS__LEG_KINEMATICS_HPP_
