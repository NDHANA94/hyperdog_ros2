// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
//
// Minimum-jerk swing foot trajectories.

#ifndef HYPERDOG_LOCOMOTION__PLANNING__SWING_TRAJECTORY_HPP_
#define HYPERDOG_LOCOMOTION__PLANNING__SWING_TRAJECTORY_HPP_

#include "hyperdog_locomotion/common/math.hpp"

namespace hyperdog_locomotion
{

struct SwingSample
{
  Vec3 pos{Vec3::Zero()};
  Vec3 vel{Vec3::Zero()};
  Vec3 acc{Vec3::Zero()};
};

/// Min-jerk swing from p0 to pf with an apex `height` above the higher end point.
/// The trajectory ends `touchdown_depth` below pf to guarantee ground contact.
SwingSample swing_trajectory(
  const Vec3 & p0, const Vec3 & pf, double height, double s, double duration,
  double touchdown_depth);

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__PLANNING__SWING_TRAJECTORY_HPP_
