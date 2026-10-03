# Hardware bring-up guide

This guide takes HyperDog from a box of BLDC actuators to a walking robot in stages. Do not skip
a stage: each one checks an assumption the next one relies on. Keep an emergency stop (a power
switch on the motor bus) within reach at all times.

> The `MitCanSystem` hardware interface and the tools below implement the widely used MIT
> mini-cheetah CAN protocol (CubeMars AK series and compatible drivers). They are tested against
> the protocol definition and in simulation, not yet on a physical robot. Report anything that
> behaves differently from this guide.

## 0. Preparation

- [ ] Fill in your actuators in `hyperdog_description/config/bldc_motors.yaml` from the
      datasheet: `torque_constant` (or `kv_rpm_per_volt`), `gear_ratio`, `max_current` (start
      **below** the datasheet peak, e.g. 60 %), `rated_current`, `bus_voltage`,
      `phase_resistance`, `mass`.
- [ ] Set the MIT packing ranges of your drivers (`p_max`, `v_max`, `kp_max`, `kd_max`, `t_max`)
      as joint `<param>`s in `hyperdog_description/urdf/ros2_control.xacro` if they differ from
      the defaults (12.5 rad, 50 rad/s, 500, 5, 25 Nm).
- [ ] Set every driver's CAN ID with the vendor tool: 1-12 in joint order
      FR hip, FR uleg, FR lleg, FL ..., BR ..., BL ...
- [ ] Bring the CAN bus up (1 Mbit/s, termination resistors at both ends):
      `sudo ip link set can0 up type can bitrate 1000000`
- [ ] Build the workspace and `source install/setup.bash`.

## 1. One motor on the bench (no leg attached)

```bash
ros2 run hyperdog_bldc_control mit_can_probe can0 scan            # all 12 IDs must reply
ros2 run hyperdog_bldc_control mit_can_probe can0 read 3 20       # turn the output by hand
```
- [ ] Every ID replies exactly once (no duplicates, no missing IDs).
- [ ] `read`: turning the output shaft changes the position smoothly and the velocity sign
      matches the direction. The reported torque is close to 0 when nothing is touching the shaft.
- [ ] Temperature reads plausibly (ambient).
- [ ] Low-gain hold: `mit_can_probe can0 hold 3 5 0.2` resists a hand push gently and returns
      to the hold position. Ctrl-C releases the motor.

## 2. Assembled leg, robot on a stand (feet free)

Mount the robot on a stand so that all feet hang in the air.

- [ ] **Zero offsets.** Put each leg into the reference pose with the URDF zero
      (hip: leg vertical in the frontal plane; uleg: thigh horizontal pointing backwards;
      lleg: shank folded back onto the thigh). Then `mit_can_probe can0 zero <id> --yes` per motor,
      or keep the drivers' zero and enter the measured angle as `offset` in `ros2_control.xacro`.
- [ ] **Directions.** Start the stack in passive mode:
      `ros2 launch hyperdog_bringup robot.launch.xml teleop:=false`
      (the locomotion controller waits, `auto_start` is false on the robot). Move each joint by
      hand and check `ros2 topic echo /joint_states`: positive motion must match the URDF axis
      (hip: abduction outwards for both sides; uleg: thigh swings forward/down; lleg: knee
      unfolds). Fix wrong ones with `direction: -1` in `ros2_control.xacro`.
- [ ] **Joint test, one joint at a time**, small amplitude first:
      ```bash
      ros2 run hyperdog_bldc_control joint_test --ros-args -p joint:=FR_lleg_joint \
        -p mode:=sine -p amplitude:=0.1 -p frequency:=0.5 -p kp:=10.0 -p kd:=0.3
      ```
      Check the printed tracking RMS (< 0.02 rad), the peak current (well below `max_current`)
      and that it never saturates. Increase to `amplitude:=0.3 frequency:=1.5 kp:=25`.
- [ ] **Torque sign.** `-p mode:=step -p torque:=1.0` must move the joint in the positive
      direction. If not, the torque direction of that driver is inverted relative to its encoder.
- [ ] Watch `/bldc_controller/motor_states`: no `saturated`, temperatures stable.

## 3. Whole robot in the air

- [ ] Verify the IMU: `ros2 topic echo /imu/data` shows gravity on +z when level and the
      roll/pitch signs follow the REP-103 convention (x forward, y left, z up). Either the IMU
      driver publishes an orientation (`estimation.attitude_source: imu_orientation`) or keep
      `mahony` (default on the robot).
- [ ] Stand-up motion in the air: send `MODE_STAND`
      (`ros2 topic pub --once /hyperdog/command hyperdog_msgs/msg/LocomotionCommand "{mode: 1}"`).
      The legs must move smoothly to the crouch and then the standing pose. Send
      `{mode: 0}` (passive) to stop.
- [ ] Contact detection from torques (`estimation.contact_source: torque`): with the robot held
      in the air, `/hyperdog/state` must report no contact; pushing a foot up by hand must
      report contact for that foot. Tune `estimation.contact_force_threshold` if needed.

## 4. On the ground

- [ ] Start with a tether or a person holding the robot. Lower `joint_gains.stand_up_kp` if the
      stand-up is harsh.
- [ ] Stand up and balance (`mode: 1`), push gently: the robot should resist and step when
      pushed harder (`disturbance_recovery`).
- [ ] Trot in place (`mode: 2`, zero velocity), then walk slowly with the gamepad. Increase the
      velocity limits (`locomotion.max_velocity`) only after the robot is stable at low speed.
- [ ] Log a session: `ros2 bag record /joint_states /imu/data /hyperdog/state /bldc_controller/motor_states`
      and compare the estimated velocity (`/odom`) with an external reference if available.

## Troubleshooting

| symptom | likely cause |
|---|---|
| a motor does not reply in `scan` | CAN ID, bitrate, termination, wiring |
| joint moves opposite to `/joint_states` sign | `direction` in `ros2_control.xacro` |
| joint test oscillates | `kd` too low or driver current loop gains; reduce `kp` |
| stand-up stops halfway, `saturated` true | `max_current` too low, or wrong gear ratio / torque constant |
| robot drifts while standing | IMU misaligned or biased; check `attitude_source` |
| feet never in contact (torque contact source) | `contact_force_threshold` too high, wrong torque sign |
