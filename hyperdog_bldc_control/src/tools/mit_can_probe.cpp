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
// Bench tool for BLDC drivers speaking the MIT CAN protocol (no ROS needed).
//
//   mit_can_probe <can_if> scan [first_id] [last_id]   list responding motors
//   mit_can_probe <can_if> read <id> [seconds]         stream position / velocity / torque
//   mit_can_probe <can_if> zero <id> --yes             set the current position as zero
//   mit_can_probe <can_if> hold <id> <kp> <kd> [position] [seconds]
//                                                      impedance hold (kp <= 20, kd <= 2)
// Options (before the command): --pmax --vmax --kpmax --kdmax --tmax (MIT packing ranges).
//
// Every command that enables a motor disables it again on exit (also on Ctrl-C).

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "hyperdog_bldc_control/mit_can_protocol.hpp"

namespace hb = hyperdog_bldc_control;
using namespace std::chrono_literals;

namespace
{
volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) {g_stop = 1;}

class CanSocket
{
public:
  explicit CanSocket(const std::string & ifname)
  {
    fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (fd_ < 0) {
      std::perror("socket");
      return;
    }
    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
      std::fprintf(stderr, "CAN interface '%s' not found\n", ifname.c_str());
      close(fd_);
      fd_ = -1;
      return;
    }
    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      std::perror("bind");
      close(fd_);
      fd_ = -1;
    }
  }
  ~CanSocket()
  {
    if (fd_ >= 0) {close(fd_);}
  }
  bool ok() const {return fd_ >= 0;}

  bool send(int id, const std::array<uint8_t, 8> & data)
  {
    struct can_frame f {};
    f.can_id = static_cast<canid_t>(id);
    f.can_dlc = 8;
    std::memcpy(f.data, data.data(), 8);
    return ::write(fd_, &f, sizeof(f)) == static_cast<ssize_t>(sizeof(f));
  }

  /// Waits up to `timeout` for a reply of motor `id`.
  bool receive(
    int id, const hb::MitRanges & r, hb::MitFeedback & fb,
    std::chrono::milliseconds timeout)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    struct can_frame f {};
    while (std::chrono::steady_clock::now() < end) {
      if (::read(fd_, &f, sizeof(f)) == static_cast<ssize_t>(sizeof(f)) && f.can_dlc >= 6 &&
        f.data[0] == id)
      {
        fb = hb::mit_unpack_reply(r, f.data, f.can_dlc);
        return true;
      }
      std::this_thread::sleep_for(200us);
    }
    return false;
  }

private:
  int fd_{-1};
};

void usage()
{
  std::puts(
    "usage: mit_can_probe [--pmax P --vmax V --kpmax K --kdmax D --tmax T] <can_if> <command>\n"
    "  scan [first_id=1] [last_id=12]\n"
    "  read <id> [seconds=10]\n"
    "  zero <id> --yes\n"
    "  hold <id> <kp<=20> <kd<=2> [position=current] [seconds=5]");
}

void print(const hb::MitFeedback & fb)
{
  std::printf(
    "id %2d  pos %+8.4f rad  vel %+8.3f rad/s  torque %+7.3f Nm  temp %3d C  err %d\n",
    fb.id, fb.position, fb.velocity, fb.torque, fb.temperature, fb.error);
}
}  // namespace

int main(int argc, char ** argv)
{
  std::signal(SIGINT, on_signal);
  hb::MitRanges r;
  std::vector<std::string> args;
  const std::map<std::string, double *> range_options{
    {"--pmax", &r.p_max}, {"--vmax", &r.v_max}, {"--kpmax", &r.kp_max},
    {"--kdmax", &r.kd_max}, {"--tmax", &r.t_max}};
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const auto opt = range_options.find(a);
    if (opt != range_options.end() && i + 1 < argc) {
      *opt->second = std::stod(argv[++i]);
    } else {
      args.push_back(a);
    }
  }
  if (args.size() < 2) {
    usage();
    return 2;
  }
  CanSocket can(args[0]);
  if (!can.ok()) {return 1;}
  const std::string cmd = args[1];
  hb::MitFeedback fb;

  if (cmd == "scan") {
    const int first = args.size() > 2 ? std::stoi(args[2]) : 1;
    const int last = args.size() > 3 ? std::stoi(args[3]) : 12;
    int found = 0;
    for (int id = first; id <= last && !g_stop; ++id) {
      can.send(id, hb::mit_enter_motor_mode());
      can.send(id, hb::mit_pack_command(r, 0, 0, 0, 0, 0));
      if (can.receive(id, r, fb, 50ms)) {
        print(fb);
        ++found;
      } else {
        std::printf("id %2d  no reply\n", id);
      }
      can.send(id, hb::mit_exit_motor_mode());
    }
    std::printf("%d motor(s) found\n", found);
    return found > 0 ? 0 : 1;
  }
  if (args.size() < 3) {
    usage();
    return 2;
  }
  const int id = std::stoi(args[2]);
  if (cmd == "read") {
    const double seconds = args.size() > 3 ? std::stod(args[3]) : 10.0;
    can.send(id, hb::mit_enter_motor_mode());
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end && !g_stop) {
      // zero gains and zero torque: the motor is free to be turned by hand
      can.send(id, hb::mit_pack_command(r, 0, 0, 0, 0, 0));
      if (can.receive(id, r, fb, 20ms)) {print(fb);}
      std::this_thread::sleep_for(100ms);
    }
    can.send(id, hb::mit_exit_motor_mode());
    return 0;
  }
  if (cmd == "zero") {
    if (args.size() < 4 || args[3] != "--yes") {
      std::puts("zero changes the encoder offset of the driver: confirm with --yes");
      return 2;
    }
    can.send(id, hb::mit_set_zero());
    std::this_thread::sleep_for(50ms);
    can.send(id, hb::mit_enter_motor_mode());
    can.send(id, hb::mit_pack_command(r, 0, 0, 0, 0, 0));
    if (can.receive(id, r, fb, 50ms)) {print(fb);}
    can.send(id, hb::mit_exit_motor_mode());
    return 0;
  }
  if (cmd == "hold") {
    if (args.size() < 5) {
      usage();
      return 2;
    }
    const double kp = std::stod(args[3]);
    const double kd = std::stod(args[4]);
    if (kp < 0 || kp > 20 || kd < 0 || kd > 2) {
      std::puts("refusing: hold is limited to kp <= 20 Nm/rad and kd <= 2 Nm s/rad");
      return 2;
    }
    can.send(id, hb::mit_enter_motor_mode());
    can.send(id, hb::mit_pack_command(r, 0, 0, 0, 0, 0));
    if (!can.receive(id, r, fb, 50ms)) {
      std::printf("motor %d does not reply\n", id);
      can.send(id, hb::mit_exit_motor_mode());
      return 1;
    }
    const double target = args.size() > 5 ? std::stod(args[5]) : fb.position;
    const double seconds = args.size() > 6 ? std::stod(args[6]) : 5.0;
    std::printf(
      "holding motor %d at %.4f rad (kp %.1f, kd %.2f) for %.1f s\n", id, target, kp,
      kd, seconds);
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    int n = 0;
    while (std::chrono::steady_clock::now() < end && !g_stop) {
      can.send(id, hb::mit_pack_command(r, target, 0, kp, kd, 0));
      if (can.receive(id, r, fb, 5ms) && ++n % 50 == 0) {print(fb);}
      std::this_thread::sleep_for(2ms);
    }
    can.send(id, hb::mit_pack_command(r, 0, 0, 0, 0, 0));
    can.send(id, hb::mit_exit_motor_mode());
    return 0;
  }
  usage();
  return 2;
}
