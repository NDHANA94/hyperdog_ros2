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
#include <gtest/gtest.h>

#include <cmath>

#include "hyperdog_bldc_control/bldc_motor_model.hpp"
#include "hyperdog_bldc_control/mit_can_protocol.hpp"

using hyperdog_bldc_control::BldcMotorModel;
using hyperdog_bldc_control::BldcMotorParams;

TEST(BldcMotorModel, TracksSmallTorqueAtStandstill)
{
  BldcMotorParams p;
  p.coulomb_friction = 0.0;
  p.viscous_friction = 0.0;
  BldcMotorModel m(p);
  hyperdog_bldc_control::BldcMotorOutput out;
  for (int i = 0; i < 20; ++i) {
    out = m.update(5.0, 0.0, 0.001);
  }
  EXPECT_NEAR(out.torque, 5.0, 1e-3);
  EXPECT_FALSE(out.saturated);
}

TEST(BldcMotorModel, CurrentLimitCapsTorque)
{
  BldcMotorParams p;
  p.coulomb_friction = 0.0;
  p.viscous_friction = 0.0;
  BldcMotorModel m(p);
  hyperdog_bldc_control::BldcMotorOutput out;
  for (int i = 0; i < 20; ++i) {
    out = m.update(1000.0, 0.0, 0.001);
  }
  EXPECT_NEAR(out.torque, p.peak_torque(), 1e-3);
  EXPECT_TRUE(out.saturated);
}

TEST(BldcMotorModel, BackEmfLimitsTorqueAtSpeed)
{
  BldcMotorParams p;
  p.coulomb_friction = 0.0;
  p.viscous_friction = 0.0;
  BldcMotorModel m(p);
  // close to no-load speed only a small torque is available in the motoring direction
  const double w = 0.95 * p.no_load_speed();
  hyperdog_bldc_control::BldcMotorOutput out;
  for (int i = 0; i < 20; ++i) {
    out = m.update(p.peak_torque(), w, 0.001);
  }
  EXPECT_LT(out.torque, 0.5 * p.peak_torque());
  // braking torque is still fully available
  m.reset();
  for (int i = 0; i < 20; ++i) {
    out = m.update(-p.peak_torque(), w, 0.001);
  }
  EXPECT_NEAR(out.torque, -p.peak_torque(), 1e-3);
}

TEST(BldcMotorModel, ThermalDeratingReducesCurrent)
{
  BldcMotorParams p;
  p.thermal_capacitance = 1.0;   // heat up quickly
  BldcMotorModel m(p);
  for (int i = 0; i < 20000; ++i) {
    m.update(p.peak_torque(), 0.0, 0.001);
  }
  EXPECT_GT(m.temperature(), p.derate_start_temperature);
  EXPECT_LT(m.current_limit(), p.max_current);
}

TEST(MitProtocol, RoundTrip)
{
  hyperdog_bldc_control::MitRanges r;
  auto d = hyperdog_bldc_control::mit_pack_command(r, 1.234, -3.0, 20.0, 1.0, 4.5);
  // reply layout differs; re-encode position/velocity/torque as a reply frame
  uint8_t reply[8] = {7, d[0], d[1], d[2], static_cast<uint8_t>(d[3] & 0xF0), 0, 65, 0};
  const uint32_t t_i = hyperdog_bldc_control::mit_float_to_uint(4.5, -r.t_max, r.t_max, 12);
  reply[4] |= static_cast<uint8_t>(t_i >> 8);
  reply[5] = static_cast<uint8_t>(t_i & 0xFF);
  auto fb = hyperdog_bldc_control::mit_unpack_reply(r, reply, 8);
  EXPECT_EQ(fb.id, 7);
  EXPECT_NEAR(fb.position, 1.234, 1e-3);
  EXPECT_NEAR(fb.velocity, -3.0, 0.03);
  EXPECT_NEAR(fb.torque, 4.5, 0.02);
  EXPECT_EQ(fb.temperature, 25);
}
