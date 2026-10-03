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
// Robot-centric 2.5D elevation map (no ROS dependency).
//
// A fixed size grid that follows the robot in whole cells, so a cell always covers the same
// patch of ground while it is in the map. Each scan is reduced to one measurement per cell
// (mean of the points in it) and fused: unknown cells take the measurement, cells that
// disagree by more than replace_threshold are replaced (terrain change, better view),
// otherwise the height is low-pass filtered.

#ifndef HYPERDOG_PERCEPTION__ELEVATION_MAP_HPP_
#define HYPERDOG_PERCEPTION__ELEVATION_MAP_HPP_

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace hyperdog_perception
{

struct ElevationMapParams
{
  double resolution{0.02};        // [m] cell size
  double length_x{2.4};           // [m] map size around the robot
  double length_y{2.4};
  double fusion_alpha{0.3};       // low-pass gain for repeated measurements
  double replace_threshold{0.04};  // [m] larger disagreements replace the cell
  int inpaint_radius{1};          // [cells] unknown cells are filled from known neighbours
};

class ElevationMap
{
public:
  explicit ElevationMap(const ElevationMapParams & p = ElevationMapParams());

  /// Moves the map so that (x, y) is near its centre (whole-cell shifts, data is kept).
  void move_to(double x, double y);
  /// Fuses one scan of points expressed in the map frame. Points outside are ignored.
  void integrate(const std::vector<Eigen::Vector3d> & points);
  void clear();

  /// Cell index of a position; false outside the map.
  bool cell(double x, double y, int & ix, int & iy) const;
  /// Height of a cell (NaN: unknown).
  float at(int ix, int iy) const {return data_[static_cast<size_t>(iy * width_ + ix)];}
  /// Copy with small holes filled from the mean of known neighbours (inpaint_radius).
  std::vector<float> inpainted() const;

  int width() const {return width_;}
  int height() const {return height_;}
  double resolution() const {return p_.resolution;}
  double origin_x() const {return origin_x_;}
  double origin_y() const {return origin_y_;}
  const std::vector<float> & data() const {return data_;}

private:
  ElevationMapParams p_;
  int width_, height_;
  double origin_x_{0.0}, origin_y_{0.0};
  int64_t cell_x0_{0}, cell_y0_{0};   // grid index of cell (0, 0)
  std::vector<float> data_;
  std::vector<double> sum_;
  std::vector<int> count_;
};

}  // namespace hyperdog_perception

#endif  // HYPERDOG_PERCEPTION__ELEVATION_MAP_HPP_
