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
#include <vector>

#include "hyperdog_perception/elevation_map.hpp"

using hyperdog_perception::ElevationMap;
using hyperdog_perception::ElevationMapParams;

namespace
{
// a step of height 0.05 m at x = 0.5 sampled every 5 mm
std::vector<Eigen::Vector3d> step_scene(double noise_sign = 0.0)
{
  std::vector<Eigen::Vector3d> pts;
  for (double x = -1.0; x < 1.0; x += 0.005) {
    for (double y = -1.0; y < 1.0; y += 0.005) {
      pts.emplace_back(x, y, (x >= 0.5 ? 0.05 : 0.0) + noise_sign * 0.001);
    }
  }
  return pts;
}
}  // namespace

TEST(ElevationMap, IntegratesAStep)
{
  ElevationMap m;
  m.integrate(step_scene());
  int ix, iy;
  ASSERT_TRUE(m.cell(0.3, 0.1, ix, iy));
  EXPECT_NEAR(m.at(ix, iy), 0.0, 1e-6);
  ASSERT_TRUE(m.cell(0.7, -0.2, ix, iy));
  EXPECT_NEAR(m.at(ix, iy), 0.05, 1e-6);
  // outside the 2.4 m map
  EXPECT_FALSE(m.cell(1.5, 0.0, ix, iy));
}

TEST(ElevationMap, FusionFiltersSmallChangesAndReplacesLargeOnes)
{
  ElevationMapParams p;
  p.fusion_alpha = 0.5;
  ElevationMap m(p);
  m.integrate({Eigen::Vector3d(0.1, 0.1, 0.0)});
  m.integrate({Eigen::Vector3d(0.1, 0.1, 0.01)});   // small: low-pass
  int ix, iy;
  ASSERT_TRUE(m.cell(0.1, 0.1, ix, iy));
  EXPECT_NEAR(m.at(ix, iy), 0.005, 1e-6);
  m.integrate({Eigen::Vector3d(0.1, 0.1, 0.2)});    // large: replace
  EXPECT_NEAR(m.at(ix, iy), 0.2, 1e-6);
  // cells without points keep their value
  m.integrate({Eigen::Vector3d(-0.3, 0.1, 0.0)});
  EXPECT_NEAR(m.at(ix, iy), 0.2, 1e-6);
}

TEST(ElevationMap, MovesWithTheRobotAndKeepsData)
{
  ElevationMap m;
  m.integrate({Eigen::Vector3d(0.5, 0.0, 0.07)});
  m.move_to(0.4, 0.0);   // the cell stays inside
  int ix, iy;
  ASSERT_TRUE(m.cell(0.5, 0.0, ix, iy));
  EXPECT_NEAR(m.at(ix, iy), 0.07, 1e-6);
  EXPECT_NEAR(m.origin_x(), 0.4 - 1.2, m.resolution() + 1e-9);
  m.move_to(3.0, 0.0);   // the cell has left the map
  EXPECT_FALSE(m.cell(0.5, 0.0, ix, iy));
  m.move_to(0.0, 0.0);   // and does not come back
  ASSERT_TRUE(m.cell(0.5, 0.0, ix, iy));
  EXPECT_TRUE(std::isnan(m.at(ix, iy)));
}

TEST(ElevationMap, InpaintsSmallHolesOnly)
{
  ElevationMap m;
  std::vector<Eigen::Vector3d> pts;
  for (int i = -20; i < 20; ++i) {
    for (int j = -20; j < 20; ++j) {
      const double x = 0.01 * i + 0.005, y = 0.01 * j + 0.005;
      if (x > 0.04 && x < 0.06 && y > 0.04 && y < 0.06) {continue;}  // one 2 cm cell empty
      pts.emplace_back(x, y, 0.03);
    }
  }
  m.integrate(pts);
  int ix, iy;
  ASSERT_TRUE(m.cell(0.05, 0.05, ix, iy));
  EXPECT_TRUE(std::isnan(m.at(ix, iy)));
  const auto filled = m.inpainted();
  EXPECT_NEAR(filled[static_cast<size_t>(iy * m.width() + ix)], 0.03, 1e-6);
  // far away from any data: stays unknown
  ASSERT_TRUE(m.cell(0.9, 0.9, ix, iy));
  EXPECT_TRUE(std::isnan(filled[static_cast<size_t>(iy * m.width() + ix)]));
}
