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

#include "hyperdog_locomotion/estimation/contact_estimator.hpp"

#include <algorithm>

namespace hyperdog_locomotion
{

void ContactEstimator::update(
  const Bool4 & scheduled, const std::array<double, 4> & progress, const Bool4 * sensor,
  const std::array<double, 4> * foot_fz)
{
  Bool4 measured = scheduled;
  if (p_.source == "sensor" && sensor) {
    measured = *sensor;
  } else if (p_.source == "torque" && foot_fz) {
    for (int i = 0; i < 4; ++i) {
      measured[i] = (*foot_fz)[i] > p_.force_threshold;
    }
  }
  for (int i = 0; i < 4; ++i) {
    early_[i] = false;
    late_[i] = false;
    if (scheduled[i]) {
      late_[i] = !measured[i] && progress[i] <= p_.late_contact_max_progress;
      contact_[i] = !late_[i];
    } else {
      early_[i] = measured[i] && progress[i] > p_.early_contact_min_progress;
      contact_[i] = early_[i];
    }
  }
}

std::array<double, 4> ContactEstimator::trust(
  const Bool4 & scheduled, const std::array<double, 4> & progress) const
{
  std::array<double, 4> t{0, 0, 0, 0};
  for (int i = 0; i < 4; ++i) {
    if (contact_[i] && scheduled[i]) {
      const double r = p_.stance_trust_ramp;
      t[i] = r > 0.0 ?
        std::max(0.05, std::min({1.0, progress[i] / r, (1.0 - progress[i]) / r + 0.2})) : 1.0;
    } else if (contact_[i]) {
      t[i] = 0.3;
    }
  }
  return t;
}

}  // namespace hyperdog_locomotion
