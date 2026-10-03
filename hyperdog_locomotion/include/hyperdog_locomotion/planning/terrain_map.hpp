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
// Terrain elevation map from perception (hyperdog_perception) and the foothold refinement
// that uses it: footholds move to the nearest cell that is not on a step edge, and their
// height comes from the map instead of the plane fitted through the stance feet.

#ifndef HYPERDOG_LOCOMOTION__PLANNING__TERRAIN_MAP_HPP_
#define HYPERDOG_LOCOMOTION__PLANNING__TERRAIN_MAP_HPP_

#include <vector>

#include "hyperdog_locomotion/common/math.hpp"

namespace hyperdog_locomotion
{

struct TerrainParams
{
  bool enabled{true};             // use the map when one is received
  double timeout{0.5};            // [s] maps older than this are ignored
  double search_radius{0.12};     // [m] how far a foothold may move
  double edge_radius{0.08};       // [m] neighbourhood checked for height changes
  double edge_threshold{0.02};    // [m] height range in that neighbourhood that marks an edge
  double support_radius{0.015};   // [m] the foot rests on the highest point within this radius
  bool terrain_swing{true};       // swing apex above the terrain along the path, step-over shape
  // registration of the map to the odometry from the feet: the shift that best explains the
  // terrain heights measured by the last stance feet
  bool register_with_feet{false};   // experimental
  double max_registration_shift{0.1};   // [m] in x / y
  double registration_step{0.01};       // [m] search resolution
  double max_registration_rate{0.02};   // [m] largest change per update (per step)
  // weight of the squared shift change [m] against the mean squared height residual [m^2]
  double registration_prior_weight{0.01};
};

class TerrainMap
{
public:
  /// data: heights, index iy * width + ix, NaN unknown; origin: corner of cell (0, 0).
  void set(
    double origin_x, double origin_y, double resolution, int width, int height,
    std::vector<float> data);
  bool empty() const {return data_.empty();}
  /// Shift between the odometry and the map: odometry point p is map point p + offset
  /// (x, y); map heights are lowered by offset.z.
  void set_offset(const Vec3 & offset) {offset_ = offset;}
  const Vec3 & offset() const {return offset_;}
  /// Estimates the offset that best explains the terrain surface points measured by the feet
  /// (odometry frame), starting from `prior`. Returns `prior` if there is not enough data.
  Vec3 estimate_offset(
    const std::vector<Vec3> & surface_points, const Vec3 & prior, const TerrainParams & prm) const;

  /// Highest known point within `radius` of (x, y); false if none is known.
  bool height(double x, double y, double radius, double & z) const;
  /// Highest known point within `radius` of the segment a-b (xy); false if none is known.
  bool max_along(const Vec3 & a, const Vec3 & b, double radius, double & z) const;
  /// Height range (max - min) of the known points within `radius`; 0 if none.
  double roughness(double x, double y, double radius) const;
  /// Moves p (xy) to the closest point within search_radius that is not on an edge and sets
  /// p.z to the terrain height there. Returns false (p unchanged) if the map does not cover p.
  bool refine_foothold(const TerrainParams & prm, Vec3 & p) const;

private:
  bool range(double x, double y, double radius, float & lo, float & hi) const;

  double origin_x_{0.0}, origin_y_{0.0}, res_{0.02};
  int width_{0}, height_{0};
  std::vector<float> data_;
  Vec3 offset_{Vec3::Zero()};
};

}  // namespace hyperdog_locomotion

#endif  // HYPERDOG_LOCOMOTION__PLANNING__TERRAIN_MAP_HPP_
