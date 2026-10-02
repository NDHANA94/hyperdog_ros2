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

#include "scenarios.hpp"

#include <string>
#include <vector>

namespace hyperdog_gazebo
{

namespace
{
/// Step without velocity command: the robot stands (and auto-steps on pushes).
Step stand(const std::string & name, double duration)
{
  Step s;
  s.name = name;
  s.duration = duration;
  s.gait = "trot";
  return s;
}
}  // namespace

std::vector<Step> build_scenario(const std::string & name)
{
  std::vector<Step> steps;

  if (name == "full" || name == "push") {
    Step s = stand("stand still", 3.0);
    s.expect_rest_at_end = true;
    steps.push_back(s);
    s = stand("push while standing: lateral 40 N x 0.2 s", 5.0);
    s.pushes.push_back({0.5, 0.2, 0.0, 40.0});
    s.expect_rest_at_end = true;
    steps.push_back(s);
    s = stand("push while standing: frontal 50 N x 0.2 s", 5.0);
    s.pushes.push_back({0.5, 0.2, -50.0, 0.0});
    s.expect_rest_at_end = true;
    steps.push_back(s);
  }
  if (name == "full" || name == "walk") {
    Step s;
    s.name = "trot forward 0.4 m/s";
    s.duration = 6.0; s.vx = 0.4; s.track_from = 2.0;
    steps.push_back(s);
    if (name == "full") {
      s = Step();
      s.name = "push while trotting: lateral 40 N x 0.2 s";
      s.duration = 4.0; s.vx = 0.4;
      s.pushes.push_back({1.0, 0.2, 0.0, -40.0});
      steps.push_back(s);
    }
    s = Step();
    s.name = "trot sideways 0.2 m/s";
    s.duration = 5.0; s.vy = 0.2; s.track_from = 2.0;
    steps.push_back(s);
    s = Step();
    s.name = "turn in place 0.8 rad/s";
    s.duration = 5.0; s.wz = 0.8; s.track_from = 2.0;
    steps.push_back(s);
    s = Step();
    s.name = "walk gait forward 0.2 m/s";
    s.duration = 7.0; s.vx = 0.2; s.gait = "walk"; s.track_from = 3.0;
    steps.push_back(s);
    s = Step();
    s.name = "trot backward -0.3 m/s";
    s.duration = 5.0; s.vx = -0.3; s.track_from = 2.0;
    steps.push_back(s);
    s = stand("stop and stand", 4.0);
    s.expect_rest_at_end = true;
    steps.push_back(s);
  }
  if (name == "stress") {
    Step s = stand("push while standing: lateral 70 N x 0.2 s", 5.0);
    s.pushes.push_back({0.5, 0.2, 0.0, 70.0});
    s.expect_rest_at_end = true;
    steps.push_back(s);
    s = stand("push while standing: diagonal 80 N x 0.2 s", 5.0);
    s.pushes.push_back({0.5, 0.2, -56.0, -56.0});
    s.expect_rest_at_end = true;
    steps.push_back(s);
    s = Step();
    s.name = "fast trot 0.6 m/s";
    s.duration = 6.0; s.vx = 0.6; s.track_from = 3.0; s.vx_tol = 0.15;
    steps.push_back(s);
    s = Step();
    s.name = "push while fast trotting: lateral 50 N x 0.2 s";
    s.duration = 4.0; s.vx = 0.6;
    s.pushes.push_back({1.0, 0.2, 0.0, 50.0});
    steps.push_back(s);
    s = Step();
    s.name = "trot + turn 0.4 m/s, 0.6 rad/s";
    s.duration = 6.0; s.vx = 0.4; s.wz = 0.6; s.track_from = 2.0;
    steps.push_back(s);
    s = stand("stop and stand", 4.0);
    s.expect_rest_at_end = true;
    steps.push_back(s);
  }
  if (name == "speed") {
    // velocity ramp to find the speed envelope
    for (double v : {0.3, 0.4, 0.5, 0.6, 0.7}) {
      Step s;
      s.name = "trot " + std::to_string(v).substr(0, 3) + " m/s";
      s.duration = 5.0;
      s.vx = v;
      s.track_from = 2.0;
      s.vx_tol = 0.15;
      steps.push_back(s);
    }
    Step s = stand("stop and stand", 4.0);
    s.expect_rest_at_end = true;
    steps.push_back(s);
  }
  if (name == "terrain") {
    // terrain.sdf: 8 deg ramp up (x = 2 .. 4), plateau, 8 deg ramp down
    Step s;
    s.name = "trot over 8 deg ramp, plateau and ramp down at 0.3 m/s";
    s.duration = 32.0; s.vx = 0.3; s.track_from = 2.0; s.vx_tol = 0.15;
    steps.push_back(s);
    s = stand("stop and stand on flat ground", 4.0);
    s.expect_rest_at_end = true;
    steps.push_back(s);
  }
  return steps;
}

}  // namespace hyperdog_gazebo
