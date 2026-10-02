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
// Fuses the gait schedule with foot contact sensors or torque based foot force
// estimates. Detects early touchdown (contact during late swing) and late
// touchdown (no contact at the start of the scheduled stance).

#ifndef HYPERDOG_LOCOMOTION__ESTIMATION__CONTACT_ESTIMATOR_HPP_
#define HYPERDOG_LOCOMOTION__ESTIMATION__CONTACT_ESTIMATOR_HPP_

#include <array>
#include <string>

#include "hyperdog_locomotion/common/math.hpp"

namespace hyperdog_locomotion
{

struct ContactParams
{
  std::string source{"sensor"};          // sensor | torque | schedule
  double force_threshold{12.0};          // [N] for source = torque
  double early_contact_min_progress{0.6};
  double late_contact_max_progress{0.4};
  double stance_trust_ramp{0.15};
};

class ContactEstimator
{
public:
  explicit ContactEstimator(const ContactParams & p = ContactParams())
  : p_(p) {}

  /// sensor: measured contacts or nullptr; foot_fz: estimated normal forces or nullptr
  void update(
    const Bool4 & scheduled, const std::array<double, 4> & progress, const Bool4 * sensor,
    const std::array<double, 4> * foot_fz);
  /// Confidence in [0, 1] that each foot is a static stance foot (Kalman filter weighting).
  std::array<double, 4> trust(
    const Bool4 & scheduled,
    const std::array<double, 4> & progress) const;

  const Bool4 & contact() const {return contact_;}
  const Bool4 & early() const {return early_;}
  const Bool4 & late() const {return late_;}
  const ContactParams & params() const {return p_;}

private:
  ContactParams p_;
  Bool4 contact_{true, true, true, true};
  Bool4 early_{false, false, false, false};
  Bool4 late_{false, false, false, false};
};

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__ESTIMATION__CONTACT_ESTIMATOR_HPP_
