// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
//
// Least squares plane z = a + b x + c y through the stance feet. Used for
// terrain adaptation (body attitude, friction cone normal, foothold height).

#ifndef HYPERDOG_LOCOMOTION__ESTIMATION__GROUND_PLANE_ESTIMATOR_HPP_
#define HYPERDOG_LOCOMOTION__ESTIMATION__GROUND_PLANE_ESTIMATOR_HPP_

#include "hyperdog_locomotion/common/math.hpp"

namespace hyperdog_locomotion
{

class GroundPlaneEstimator
{
public:
  explicit GroundPlaneEstimator(double alpha = 0.02)
  : alpha_(alpha) {}

  void reset(const Mat43 & feet_world, double foot_radius);
  void update(const Mat43 & feet_world, const Bool4 & use, double foot_radius);
  double height(double x, double y) const {return coef_[0] + coef_[1] * x + coef_[2] * y;}
  Vec3 normal() const {return Vec3(-coef_[1], -coef_[2], 1.0).normalized();}
  /// Body roll / pitch that align the body with the plane for a heading yaw.
  Eigen::Vector2d slope_rp(double yaw) const;

private:
  double alpha_;
  Vec3 coef_{Vec3::Zero()};
  Mat43 hist_{Mat43::Zero()};
};

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__ESTIMATION__GROUND_PLANE_ESTIMATOR_HPP_
