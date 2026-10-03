// Copyright 2024 W.M. Nipun Dhananjaya Weerakkodi
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Scripted validation scenarios: a scenario is a sequence of steps, each with a
// velocity command, a gait, optional external pushes and the checks to apply.

#ifndef SCENARIOS_HPP_
#define SCENARIOS_HPP_

#include <cmath>
#include <string>
#include <vector>

namespace hyperdog_gazebo
{

struct Push
{
  double start{0.0};      // [s] after the step began
  double duration{0.0};   // [s]
  double fx{0.0}, fy{0.0};
  double tx{0.0};         // [Nm] roll torque
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
  // a fall is provoked in this step: falling is not a failure, the step passes when the
  // robot is standing upright again at its end (self-righting)
  bool allow_fall{false};
  // act like an operator keeping the robot on the line y = 0, heading 0 (the courses of the
  // terrain worlds): lateral and yaw corrections are added to the command
  bool hold_line{false};
  // at the step start, drop the robot from `place_height` with this roll angle [rad]
  // at its current x, y (Gazebo set_pose); NaN: do not move the robot
  double place_roll{std::nan("")};
  double place_height{0.2};
};

/// Scenario names: "full", "push", "walk", "stress", "speed", "robust", "fall", "fall_back"
/// (flat world) and "terrain", "rough", "stairs", "slippery" (use the world of the same name).
/// Returns an empty list for unknown names.
std::vector<Step> build_scenario(const std::string & name);

}  // namespace hyperdog_gazebo

#endif  // SCENARIOS_HPP_
