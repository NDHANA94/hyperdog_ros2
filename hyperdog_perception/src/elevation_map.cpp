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

#include "hyperdog_perception/elevation_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hyperdog_perception
{

namespace
{
constexpr float kUnknown = std::numeric_limits<float>::quiet_NaN();
}

ElevationMap::ElevationMap(const ElevationMapParams & p)
: p_(p),
  width_(std::max(1, static_cast<int>(std::lround(p.length_x / p.resolution)))),
  height_(std::max(1, static_cast<int>(std::lround(p.length_y / p.resolution)))),
  data_(static_cast<size_t>(width_ * height_), kUnknown),
  sum_(data_.size(), 0.0),
  count_(data_.size(), 0)
{
  move_to(0.0, 0.0);
}

void ElevationMap::clear()
{
  std::fill(data_.begin(), data_.end(), kUnknown);
}

void ElevationMap::move_to(double x, double y)
{
  const int64_t nx0 = std::llround(std::floor(x / p_.resolution)) - width_ / 2;
  const int64_t ny0 = std::llround(std::floor(y / p_.resolution)) - height_ / 2;
  if (nx0 != cell_x0_ || ny0 != cell_y0_) {
    std::vector<float> moved(data_.size(), kUnknown);
    for (int iy = 0; iy < height_; ++iy) {
      const int64_t oy = ny0 + iy - cell_y0_;
      if (oy < 0 || oy >= height_) {continue;}
      for (int ix = 0; ix < width_; ++ix) {
        const int64_t ox = nx0 + ix - cell_x0_;
        if (ox < 0 || ox >= width_) {continue;}
        moved[static_cast<size_t>(iy * width_ + ix)] =
          data_[static_cast<size_t>(oy * width_ + ox)];
      }
    }
    data_.swap(moved);
    cell_x0_ = nx0;
    cell_y0_ = ny0;
  }
  origin_x_ = static_cast<double>(cell_x0_) * p_.resolution;
  origin_y_ = static_cast<double>(cell_y0_) * p_.resolution;
}

bool ElevationMap::cell(double x, double y, int & ix, int & iy) const
{
  ix = static_cast<int>(std::floor((x - origin_x_) / p_.resolution));
  iy = static_cast<int>(std::floor((y - origin_y_) / p_.resolution));
  return ix >= 0 && iy >= 0 && ix < width_ && iy < height_;
}

void ElevationMap::integrate(const std::vector<Eigen::Vector3d> & points)
{
  std::fill(sum_.begin(), sum_.end(), 0.0);
  std::fill(count_.begin(), count_.end(), 0);
  for (const auto & pt : points) {
    int ix, iy;
    if (!std::isfinite(pt.z()) || !cell(pt.x(), pt.y(), ix, iy)) {continue;}
    const size_t k = static_cast<size_t>(iy * width_ + ix);
    sum_[k] += pt.z();
    ++count_[k];
  }
  for (size_t k = 0; k < data_.size(); ++k) {
    if (count_[k] == 0) {continue;}
    const float m = static_cast<float>(sum_[k] / count_[k]);
    float & h = data_[k];
    if (std::isnan(h) || std::abs(m - h) > p_.replace_threshold) {
      h = m;
    } else {
      h += static_cast<float>(p_.fusion_alpha) * (m - h);
    }
  }
}

std::vector<float> ElevationMap::inpainted() const
{
  std::vector<float> out = data_;
  const int r = p_.inpaint_radius;
  if (r <= 0) {return out;}
  for (int iy = 0; iy < height_; ++iy) {
    for (int ix = 0; ix < width_; ++ix) {
      if (!std::isnan(at(ix, iy))) {continue;}
      double s = 0.0;
      int n = 0;
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          const int jx = ix + dx, jy = iy + dy;
          if (jx < 0 || jy < 0 || jx >= width_ || jy >= height_) {continue;}
          const float h = at(jx, jy);
          if (!std::isnan(h)) {
            s += h;
            ++n;
          }
        }
      }
      // fill only holes surrounded by data (at least half of the neighbourhood known)
      if (n * 2 >= (2 * r + 1) * (2 * r + 1)) {
        out[static_cast<size_t>(iy * width_ + ix)] = static_cast<float>(s / n);
      }
    }
  }
  return out;
}

}  // namespace hyperdog_perception
