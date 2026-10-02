// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi
//
// Scripted validation scenarios: a scenario is a sequence of steps, each with a
// velocity command, a gait, optional external pushes and the checks to apply.

#ifndef SCENARIOS_HPP_
#define SCENARIOS_HPP_

#include <string>
#include <vector>

namespace hyperdog_gazebo
{

struct Push
{
  double start{0.0};      // [s] after the step began
  double duration{0.0};   // [s]
  double fx{0.0}, fy{0.0};
};

struct Step
{
  std::string name;
  double duration{1.0};
  double vx{0.0}, vy{0.0}, wz{0.0};
  std::string gait{"trot"};
  std::vector<Push> pushes;
  // metric window (seconds from the step start) for velocity tracking; < 0 disables
  double track_from{-1.0};
  double vx_tol{0.12}, vy_tol{0.1}, wz_tol{0.25};
  bool expect_rest_at_end{false};
};

/// Scenario names: "full", "push", "walk", "stress", "terrain" (world:=terrain).
/// Returns an empty list for unknown names.
std::vector<Step> build_scenario(const std::string & name);

}  // namespace hyperdog_gazebo

#endif  // SCENARIOS_HPP_
