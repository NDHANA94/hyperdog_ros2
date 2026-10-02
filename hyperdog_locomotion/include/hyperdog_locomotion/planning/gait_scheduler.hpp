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
// Phase based gait scheduler. Each gait: period, duty factor and per-leg phase
// offset (FR, FL, BR, BL). Gait changes go through a "stopping" stage where
// legs on the ground stay down and swinging legs finish their step, so a leg
// is never dropped mid-swing.

#ifndef HYPERDOG_LOCOMOTION__PLANNING__GAIT_SCHEDULER_HPP_
#define HYPERDOG_LOCOMOTION__PLANNING__GAIT_SCHEDULER_HPP_

#include <array>
#include <map>
#include <string>
#include <vector>

#include "hyperdog_locomotion/common/math.hpp"

namespace hyperdog_locomotion
{

struct Gait
{
  std::string name{"stand"};
  double period{0.5};
  double duty{1.0};
  std::array<double, 4> offsets{0.0, 0.0, 0.0, 0.0};
  double stance_time() const {return period * duty;}
  double swing_time() const {return period * (1.0 - duty);}
  bool is_stand() const {return duty >= 1.0;}
};

std::map<std::string, Gait> default_gaits();

class GaitScheduler
{
public:
  explicit GaitScheduler(const std::map<std::string, Gait> & gaits = default_gaits());

  bool has(const std::string & name) const {return gaits_.count(name) > 0;}
  void request(const std::string & name);
  void step(double dt);
  void reset();

  /// predicted contacts for steps k = 1..horizon of length dt
  std::vector<Bool4> contact_table(int horizon, double dt) const;
  double swing_remaining(int leg) const;

  const Gait & current() const {return current_;}
  bool is_standing() const {return current_.is_stand() && !stopping_;}
  const Bool4 & contact() const {return contact_;}
  const std::array<double, 4> & progress() const {return progress_;}
  const std::array<double, 4> & leg_phase() const {return leg_phase_;}

private:
  void start(const Gait & g);
  void update_legs();

  std::map<std::string, Gait> gaits_;
  Gait current_, requested_;
  double phase_{0.0};
  bool stopping_{false};
  Bool4 latched_{false, false, false, false};
  Bool4 contact_{true, true, true, true};
  std::array<double, 4> progress_{0, 0, 0, 0};
  std::array<double, 4> leg_phase_{0, 0, 0, 0};
};

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__PLANNING__GAIT_SCHEDULER_HPP_
