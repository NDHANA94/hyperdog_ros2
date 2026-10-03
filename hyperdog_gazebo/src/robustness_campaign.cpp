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
// Randomized robustness campaign (domain randomization of the simulation).
//
// For every run a model / actuator / sensor variation is sampled, the
// validation launch file is executed headless with it and the scenario result
// is collected. Varied: payload mass and position, foot friction, IMU noise,
// motor current limit, torque constant, friction and command latency.
//
//   ros2 run hyperdog_gazebo robustness_campaign --runs 20 --seed 1 --scenario robust
//     [--world flat] [--out hyperdog_campaign] [--min-pass-rate 0.9] [--timeout 240]
//
// Writes <out>/campaign.md (one row per run + pass rate) and the per-run reports.
// Exit code 0 when the pass rate reaches --min-pass-rate (default 1.0).

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace
{
struct Variation
{
  double payload_mass{0.0};
  double payload_x{0.0};
  double payload_y{0.0};
  double foot_mu{1.0};
  double imu_noise_scale{1.0};
  double max_current{25.0};
  double torque_constant{0.123};
  double coulomb_friction{0.05};
  double command_latency{0.0};
};

struct Options
{
  int runs{10};
  unsigned seed{1};
  std::string scenario{"robust"};
  std::string world{"flat"};
  std::string out{"hyperdog_campaign"};
  double min_pass_rate{1.0};
  int timeout_s{240};
};

Options parse(int argc, char ** argv)
{
  Options o;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string k = argv[i];
    const std::string v = argv[i + 1];
    if (k == "--runs") {
      o.runs = std::stoi(v);
    } else if (k == "--seed") {
      o.seed = static_cast<unsigned>(std::stoul(v));
    } else if (k == "--scenario") {
      o.scenario = v;
    } else if (k == "--world") {
      o.world = v;
    } else if (k == "--out") {
      o.out = v;
    } else if (k == "--min-pass-rate") {
      o.min_pass_rate = std::stod(v);
    } else if (k == "--timeout") {
      o.timeout_s = std::stoi(v);
    } else {
      std::cerr << "unknown option " << k << "\n";
      std::exit(2);
    }
  }
  return o;
}

Variation sample(std::mt19937 & rng)
{
  auto u = [&rng](double a, double b) {return std::uniform_real_distribution<double>(a, b)(rng);};
  Variation v;
  v.payload_mass = u(0.0, 1.5);
  v.payload_x = u(-0.05, 0.05);
  v.payload_y = u(-0.03, 0.03);
  v.foot_mu = u(0.5, 1.2);
  v.imu_noise_scale = u(1.0, 3.0);
  v.max_current = 25.0 * u(0.8, 1.0);
  v.torque_constant = 0.123 * u(0.9, 1.1);
  v.coulomb_friction = u(0.02, 0.1);
  v.command_latency = u(0.0, 0.006);
  return v;
}

std::string read_file(const std::string & path)
{
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

void cleanup_processes()
{
  // stray simulator processes of a timed out run would disturb the next one
  [[maybe_unused]] const int r = std::system(
    "pkill -9 -f 'gz sim' >/dev/null 2>&1; "
    "pkill -9 -f 'parameter_bridge|robot_state_publisher|locomotion_node|scenario_runner' "
    ">/dev/null 2>&1");
}
}  // namespace

int main(int argc, char ** argv)
{
  const Options opt = parse(argc, argv);
  namespace fs = std::filesystem;
  fs::create_directories(opt.out);
  const std::string out = fs::absolute(opt.out).string();
  std::mt19937 rng(opt.seed);

  std::ostringstream md;
  md << "# HyperDog robustness campaign\n\n"
     << "Scenario `" << opt.scenario << "` in world `" << opt.world << "`, " << opt.runs
     << " runs, seed " << opt.seed << ".\n\n"
     << "| run | payload [kg] @ (x, y) [m] | foot mu | IMU noise x | motor I_max [A] | Kt [Nm/A] "
     << "| friction [Nm] | latency [ms] | result |\n|---|---|---|---|---|---|---|---|---|\n";
  int passed = 0;
  for (int i = 0; i < opt.runs; ++i) {
    const Variation v = sample(rng);
    const std::string tag = out + "/run_" + std::to_string(i);
    {
      std::ofstream f(tag + "_actuators.yaml");
      f << "bldc_controller:\n  ros__parameters:\n"
        << "    command_latency: " << v.command_latency << "\n"
        << "    motors:\n      qdd_10_1:\n"
        << "        max_current: " << v.max_current << "\n"
        << "        torque_constant: " << v.torque_constant << "\n"
        << "        coulomb_friction: " << v.coulomb_friction << "\n";
    }
    std::ostringstream cmd;
    cmd << "timeout -s INT " << opt.timeout_s
        << " ros2 launch hyperdog_gazebo validate.launch.xml"
        << " scenario:=" << opt.scenario << " world:=" << opt.world
        << " report:=" << tag << ".md"
        << " payload_mass:=" << v.payload_mass << " payload_x:=" << v.payload_x
        << " payload_y:=" << v.payload_y << " foot_mu:=" << v.foot_mu
        << " imu_noise_scale:=" << v.imu_noise_scale
        << " extra_controller_params:=" << tag << "_actuators.yaml"
        << " > " << tag << ".log 2>&1";
    std::cout << "[" << i + 1 << "/" << opt.runs << "] " << std::flush;
    cleanup_processes();
    [[maybe_unused]] const int rc = std::system(cmd.str().c_str());
    cleanup_processes();
    const std::string report = read_file(tag + ".md");
    const bool ok = report.find("**Overall: PASS**") != std::string::npos;
    passed += ok ? 1 : 0;
    std::cout << (ok ? "PASS" : "FAIL") << std::endl;
    char row[512];
    std::snprintf(
      row, sizeof(row),
      "| %d | %.2f @ (%.3f, %.3f) | %.2f | %.1f | %.1f | %.3f | %.3f | %.1f | %s |\n",
      i, v.payload_mass, v.payload_x, v.payload_y, v.foot_mu, v.imu_noise_scale, v.max_current,
      v.torque_constant, v.coulomb_friction, 1e3 * v.command_latency,
      ok ? "PASS" : "**FAIL**");
    md << row;
  }
  const double rate = opt.runs > 0 ? static_cast<double>(passed) / opt.runs : 0.0;
  md << "\n**Pass rate: " << passed << " / " << opt.runs << " (" << static_cast<int>(100 * rate)
     << " %)**\n";
  std::ofstream(out + "/campaign.md") << md.str();
  std::cout << md.str();
  return rate + 1e-9 >= opt.min_pass_rate ? 0 : 1;
}
