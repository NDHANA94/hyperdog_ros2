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

#include "hyperdog_locomotion/planning/terrain_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace hyperdog_locomotion
{

void TerrainMap::set(
  double origin_x, double origin_y, double resolution, int width, int height,
  std::vector<float> data)
{
  if (width <= 0 || height <= 0 || resolution <= 0.0 ||
    data.size() != static_cast<size_t>(width) * static_cast<size_t>(height))
  {
    data_.clear();
    return;
  }
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  res_ = resolution;
  width_ = width;
  height_ = height;
  data_ = std::move(data);
}

bool TerrainMap::range(double x, double y, double radius, float & lo, float & hi) const
{
  lo = std::numeric_limits<float>::infinity();
  hi = -lo;
  if (data_.empty()) {return false;}
  x += offset_.x();
  y += offset_.y();
  const int r = static_cast<int>(std::ceil(radius / res_));
  const int cx = static_cast<int>(std::floor((x - origin_x_) / res_));
  const int cy = static_cast<int>(std::floor((y - origin_y_) / res_));
  const double r2 = (radius + 0.5 * res_) * (radius + 0.5 * res_);
  bool any = false;
  for (int iy = std::max(0, cy - r); iy <= std::min(height_ - 1, cy + r); ++iy) {
    for (int ix = std::max(0, cx - r); ix <= std::min(width_ - 1, cx + r); ++ix) {
      const double dx = origin_x_ + (ix + 0.5) * res_ - x;
      const double dy = origin_y_ + (iy + 0.5) * res_ - y;
      if (dx * dx + dy * dy > r2) {continue;}
      const float h = data_[static_cast<size_t>(iy * width_ + ix)];
      if (std::isnan(h)) {continue;}
      lo = std::min(lo, h);
      hi = std::max(hi, h);
      any = true;
    }
  }
  lo -= static_cast<float>(offset_.z());
  hi -= static_cast<float>(offset_.z());
  return any;
}

Vec3 TerrainMap::estimate_offset(
  const std::vector<Vec3> & points, const Vec3 & prior, const TerrainParams & prm) const
{
  if (data_.empty() || points.size() < 4) {return prior;}
  constexpr double kResidualCap = 0.03;   // [m] robust cost: larger residuals count as this
  const int n = static_cast<int>(std::round(prm.max_registration_shift / prm.registration_step));
  TerrainMap probe = *this;
  double best = std::numeric_limits<double>::infinity(), prior_cost = best;
  Vec3 best_offset = prior;
  std::vector<double> r;
  const int px = static_cast<int>(std::round(prior.x() / prm.registration_step));
  const int py = static_cast<int>(std::round(prior.y() / prm.registration_step));
  for (int iy = -n; iy <= n; ++iy) {
    for (int ix = -n; ix <= n; ++ix) {
      const bool at_prior = ix == px && iy == py;
      probe.offset_ = Vec3(ix * prm.registration_step, iy * prm.registration_step, 0.0);
      r.clear();
      for (const Vec3 & p : points) {
        double h;
        if (probe.height(p.x(), p.y(), 0.5 * res_, h)) {r.push_back(h - p.z());}
      }
      if (r.size() < 4 || r.size() * 4 < points.size() * 3) {continue;}
      // height offset: median residual
      std::vector<double> sorted = r;
      std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
      const double dz = sorted[sorted.size() / 2];
      double cost = 0.0;
      for (double e : r) {
        cost += std::min((e - dz) * (e - dz), kResidualCap * kResidualCap);
      }
      cost /= static_cast<double>(r.size());
      const Vec3 o(probe.offset_.x(), probe.offset_.y(), dz);
      if (at_prior) {prior_cost = cost;}
      cost += prm.registration_prior_weight * (o - prior).head<2>().squaredNorm();
      if (cost < best) {
        best = cost;
        best_offset = o;
      }
    }
  }
  if (!std::isfinite(best)) {return prior;}
  // change the shift only if it explains clearly more of the contacts (half a capped residual
  // less), and move at most max_registration_rate per update
  Vec3 out = best_offset;
  if (std::isfinite(prior_cost) &&
    prior_cost - best < 0.5 * kResidualCap * kResidualCap / static_cast<double>(points.size()))
  {
    out.head<2>() = prior.head<2>();
  }
  const Eigen::Vector2d step = out.head<2>() - prior.head<2>();
  if (step.norm() > prm.max_registration_rate) {
    out.head<2>() = prior.head<2>() + step * prm.max_registration_rate / step.norm();
  }
  return out;
}

bool TerrainMap::height(double x, double y, double radius, double & z) const
{
  float lo, hi;
  if (!range(x, y, radius, lo, hi)) {return false;}
  z = hi;
  return true;
}

bool TerrainMap::max_along(const Vec3 & a, const Vec3 & b, double radius, double & z) const
{
  const double len = (b - a).head<2>().norm();
  const int n = std::max(1, static_cast<int>(std::ceil(len / (0.5 * res_))));
  bool any = false;
  z = -std::numeric_limits<double>::infinity();
  for (int k = 0; k <= n; ++k) {
    const Vec3 p = a + (b - a) * (static_cast<double>(k) / n);
    double h;
    if (height(p.x(), p.y(), radius, h)) {
      z = std::max(z, h);
      any = true;
    }
  }
  return any;
}

double TerrainMap::roughness(double x, double y, double radius) const
{
  float lo, hi;
  return range(x, y, radius, lo, hi) ? static_cast<double>(hi - lo) : 0.0;
}

bool TerrainMap::refine_foothold(const TerrainParams & prm, Vec3 & p) const
{
  double z;
  if (!height(p.x(), p.y(), prm.support_radius, z)) {return false;}
  const int r = static_cast<int>(std::ceil(prm.search_radius / res_));
  double best_cost = std::numeric_limits<double>::infinity();
  Vec3 best = p;
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      const double x = p.x() + dx * res_, y = p.y() + dy * res_;
      const double d2 = (dx * dx + dy * dy) * res_ * res_;
      if (d2 > prm.search_radius * prm.search_radius || d2 >= best_cost) {continue;}
      double h;
      if (!height(x, y, prm.support_radius, h)) {continue;}
      if (roughness(x, y, prm.edge_radius) > prm.edge_threshold) {continue;}
      best_cost = d2;
      best = Vec3(x, y, h);
    }
  }
  if (std::isfinite(best_cost)) {
    p = best;
  } else {
    p.z() = z;   // everything around is an edge: keep the position, take the highest point
  }
  return true;
}

}  // namespace hyperdog_locomotion
