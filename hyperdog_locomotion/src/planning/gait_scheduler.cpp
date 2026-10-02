// MIT License - Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi

#include "hyperdog_locomotion/planning/gait_scheduler.hpp"

#include <cmath>
#include <stdexcept>

namespace hyperdog_locomotion
{

std::map<std::string, Gait> default_gaits()
{
  std::map<std::string, Gait> g;
  g["stand"] = Gait{"stand", 0.5, 1.0, {0.0, 0.0, 0.0, 0.0}};
  g["trot"] = Gait{"trot", 0.36, 0.55, {0.0, 0.5, 0.5, 0.0}};
  g["walk"] = Gait{"walk", 0.8, 0.75, {0.75, 0.25, 0.5, 0.0}};
  g["pace"] = Gait{"pace", 0.36, 0.55, {0.0, 0.5, 0.0, 0.5}};
  g["bound"] = Gait{"bound", 0.36, 0.5, {0.0, 0.0, 0.5, 0.5}};
  g["pronk"] = Gait{"pronk", 0.4, 0.5, {0.0, 0.0, 0.0, 0.0}};
  return g;
}

GaitScheduler::GaitScheduler(const std::map<std::string, Gait> & gaits)
: gaits_(gaits)
{
  if (!gaits_.count("stand")) {gaits_["stand"] = default_gaits()["stand"];}
  reset();
}

void GaitScheduler::reset()
{
  current_ = gaits_.at("stand");
  requested_ = current_;
  stopping_ = false;
  phase_ = 0.0;
  latched_ = {false, false, false, false};
  update_legs();
}

void GaitScheduler::request(const std::string & name)
{
  auto it = gaits_.find(name);
  if (it == gaits_.end()) {
    throw std::invalid_argument("unknown gait '" + name + "'");
  }
  requested_ = it->second;
  if (requested_.name != current_.name && !stopping_) {
    if (current_.is_stand()) {
      start(requested_);
    } else {
      stopping_ = true;
      latched_ = contact_;   // legs on the ground stay there
    }
  }
}

void GaitScheduler::step(double dt)
{
  if (current_.is_stand()) {
    phase_ = 0.0;
    update_legs();
    return;
  }
  const Bool4 prev = contact_;
  phase_ = std::fmod(phase_ + dt / current_.period, 1.0);
  update_legs();
  if (stopping_) {
    bool all = true;
    for (int i = 0; i < 4; ++i) {
      if (contact_[i] && !prev[i]) {latched_[i] = true;}   // touchdown
      if (latched_[i]) {contact_[i] = true;}
      all = all && latched_[i];
    }
    if (all) {
      stopping_ = false;
      start(requested_);
    }
  }
}

std::vector<Bool4> GaitScheduler::contact_table(int horizon, double dt) const
{
  std::vector<Bool4> table(horizon, Bool4{true, true, true, true});
  if (current_.is_stand()) {return table;}
  for (int k = 0; k < horizon; ++k) {
    for (int i = 0; i < 4; ++i) {
      const double ph = std::fmod(
        phase_ + (k + 1) * dt / current_.period + current_.offsets[i],
        1.0);
      table[k][i] = ph < current_.duty || (stopping_ && latched_[i]);
    }
  }
  return table;
}

double GaitScheduler::swing_remaining(int leg) const
{
  if (contact_[leg]) {return 0.0;}
  return (1.0 - progress_[leg]) * current_.swing_time();
}

void GaitScheduler::start(const Gait & g)
{
  current_ = g;
  phase_ = 0.0;
  latched_ = {false, false, false, false};
  update_legs();
}

void GaitScheduler::update_legs()
{
  if (current_.is_stand()) {
    contact_ = {true, true, true, true};
    progress_ = {0, 0, 0, 0};
    leg_phase_ = {0, 0, 0, 0};
    return;
  }
  for (int i = 0; i < 4; ++i) {
    leg_phase_[i] = std::fmod(phase_ + current_.offsets[i], 1.0);
    contact_[i] = leg_phase_[i] < current_.duty;
    progress_[i] = contact_[i] ? leg_phase_[i] / current_.duty :
      (leg_phase_[i] - current_.duty) / (1.0 - current_.duty);
  }
}

}  // namespace hyperdog_locomotion
