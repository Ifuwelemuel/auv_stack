# auv_stack

ROS 2 software stack for a low-cost autonomous underwater vehicle investigating
**DVL-free learned dead reckoning supervised only by sparse surface GPS fixes**.

MSc dissertation project, Sheffield Hallam University.
Supervisor: Konstantinos Domdouzis.

---

## Research context

An AUV cannot use GPS while submerged — electromagnetic signals attenuate
rapidly in water  so it must dead reckon between surfacings. Accuracy
conventionally depends on a Doppler Velocity Log (DVL), an acoustic instrument
that typically costs more than the rest of a low-cost vehicle combined.

Learned dead-reckoning methods promise to replace the DVL, but almost all are
**trained against DVL ground truth**. The instrument is removed at deployment
and retained at training time, so the method is cheap to run but not cheap to
produce.

This project investigates whether a learned dead-reckoning estimator can be
trained using **only sparse surface GPS fixes** as supervision, with no DVL
anywhere in the pipeline. Because GPS gives position at occasional instants
rather than velocity continuously, the loss is applied to the *displacement
accumulated between fixes*, which requires the dead-reckoning integration
itself to be differentiable.

---

## Hardware

| Subsystem | Component | Role |
|---|---|---|
| Companion computer | NVIDIA Jetson Orin Nano (Super) | All autonomy, control and estimation; ROS 2 host |
| Autopilot | Pixhawk 2.4.8 running ArduSub 4.7 | Calibrated sensor package only  **not** in the actuation path |
| Inertial / attitude | MPU6000 IMU, IST8310 magnetometer | Attitude, angular rate, heading |
| Depth | Blue Robotics Bar02 (10 m) | Depth measurement and control feedback |
| Position (supervision) | u-blox M8N on a surface float | Sparse surface fixes  the sole supervision signal |
| Propulsion | Blue Robotics T200 + ESC | Bidirectional thrust |
| Control surfaces | 2 × 25 kg-cm digital servos | Pitch and yaw fins |
| Ballast | DC motor via BTS7960 H-bridge | Variable buoyancy trim (Jetson GPIO) |
| Actuation interface | PCA9685 16-channel PWM (I²C) | Deterministic hardware PWM generation |
| Surface buoy | Raspberry Pi 4, Ubuntu Server 22.04 | GPS logging and network gateway |
| Power | 2 × 4S 5200 mAh LiPo | Propulsion and electronics on **separate** packs |

**Vehicle configuration:** ~1 m torpedo form, single stern thruster, two
independent control fins (pitch and yaw), syringe-based variable ballast. The
vehicle is non-holonomic — it cannot turn on the spot, and yaw authority is
proportional to forward speed.

---

## Architecture

```
                    ┌──────────────────────────┐
   surface float    │  Raspberry Pi (auv-buoy) │
                    │  GPS node → /buoy/fix    │
                    │  chrony time reference   │
                    └────────────┬─────────────┘
                       Ethernet tether (192.168.10.0/24)
                    ┌────────────┴─────────────┐
                    │  Jetson (jetson-desktop) │
                    │                          │
   Pixhawk ─UART──▶ │  mavros  (sensors only)  │
   (ArduSub)        │      ↓                   │
                    │  safety_supervisor       │  e-stop latch, watchdog
                    │      ↓                   │
                    │  actuator_mixer          │  priority arbitration
                    │      ↓                   │
                    │  pca9685_driver ─I²C──▶  │  PCA9685 → servos, ESC
                    └──────────────────────────┘
```

<img width="1385" height="1010" alt="Screenshot 2026-08-13 at 21 49 00" src="https://github.com/user-attachments/assets/05804e76-1392-4b8d-90ff-ef5098a1cd28" />


**Design principles enforced throughout:**

- **The mixer is the single command choke point.** Nothing else may publish
  actuator commands. Safety, teleoperation and autonomy all feed it and it
  arbitrates by priority with staleness rejection.
- **Safe state is fail-to-*surface*, not fail-to-off.** A de-powered submerged
  vehicle stays at depth. The safe command is thruster stopped, fins neutral,
  ballast driven to empty.
- **The estimator is interchangeable.** Any estimator publishing
  `nav_msgs/Odometry` on the standard interface can be substituted at runtime,
  so the learned model and its baselines are a controlled comparison.
- **Conventions follow REP-103** (SI units, right-handed FLU body frame) and
  **REP-105** (`map` → `odom` → `base_link`).

---

## Packages

| Package | Build type | Contents |
|---|---|---|
| `auv_interfaces` | ament_cmake | Message definitions. The system contract  depends on nothing, everything depends on it. |
| `auv_safety` | ament_python | `safety_supervisor`  latched e-stop, command watchdog, safe-state publisher |
| `auv_control` | ament_python | `actuator_mixer`, `teleop_node`, `pca9685_driver` |
| `auv_bringup` | ament_python | Launch files and configuration |
| `auv_buoy` | ament_python | `gps_node`  NMEA to `sensor_msgs/NavSatFix` with HDOP-derived covariance |

---

## Build

Requires **ROS 2 Humble** on **Ubuntu 22.04**.

```bash
mkdir -p ~/auv_ws/src && cd ~/auv_ws/src
git clone <this-repo> auv_stack
cd ~/auv_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

Python dependencies not resolvable by rosdep:

```bash
sudo pip3 install smbus2 pynmea2
```

**Note:** each terminal must `source install/setup.bash` separately.

---

## Running

### Bench

```bash
ros2 launch auv_bringup bench.launch.py
```

Brings up the safety supervisor, mixer, teleoperation and PCA9685 driver.
Add `use_local_joy:=false` when the gamepad is connected to the buoy Pi rather
than the vehicle.

**Before running with actuators connected: propeller off, vehicle secured,
physical isolation within reach.**

### Buoy

```bash
# on the Pi
ros2 run auv_buoy gps_node --ros-args --params-file \
  install/auv_buoy/share/auv_buoy/config/gps_params.yaml
```

Publishes `/buoy/fix` into the shared ROS graph.

---

## Safety architecture

Built and verified **before any actuator was driven**. Each mechanism is
tested individually and re-checked before every field session.

| Mechanism | Behaviour |
|---|---|
| **Latched e-stop** | Boots stopped; must be explicitly cleared. Published with transient-local QoS so a node starting later still receives the current state — start order is not a safety variable. |
| **Command watchdog** | Trips if the commanding authority stops sending heartbeats. Measures silence on the **local** clock, so remote clock skew cannot corrupt watchdog timing. |
| **Staleness rejection** | The mixer treats any command older than a timeout as absent, so a crashed node's last command cannot persist. |
| **Deadman gate** | Teleoperated motion requires a button held; releasing commands neutral. |
| **Dead-man's switch** | The heartbeat is emitted only on receipt of controller input, so losing the controller — not merely the software — stops the vehicle. |
| **Driver-level guard** | The actuator driver independently refuses to drive while e-stop is asserted, so a mixer defect alone cannot defeat the stop. |

### Acceptance tests

Repeated as a pre-launch checklist:

1. Vehicle initialises in the safe state
2. A latched stop reaches a subscriber that starts afterwards
3. A live teleoperation command is suppressed while stopped
4. Releasing the deadman returns actuators to neutral
5. Disconnecting the controller returns actuators to neutral within the timeout
6. `PCA9685 verified` appears in the driver log (configuration confirmed, not assumed)

---

## Calibration

**All actuator values are measured, never assumed, and are specific to the
installed hardware.** They must be re-measured if the PWM board is replaced.

Two findings drive this:

- **Servo ranges are non-standard** (ADR-013). Fin neutral and travel differ
  materially from the conventional 1500 µs centre; commanding the nominal range
  drives the servos into their mechanical stops.
- **PWM board oscillators deviate** (ADR-016). The PCA9685's oscillator differs
  from its nominal 25 MHz by a per-board amount, shifting every pulse width.
  The ESC neutral had to be measured rather than assumed  the nominal value
  read as a small throttle command, preventing arming and causing the motor to
  creep.

Calibration scripts are in `scripts/`. Procedure: find the value where the
actuator is silent and centred, step outward until strain is audible, back off,
and record. Values live in `auv_control/config/pca9685_params.yaml`.

---

## Network and time synchronisation

| Machine | Wired (tether) | Role |
|---|---|---|
| Jetson | `192.168.10.1` | Vehicle |
| Buoy Pi | `192.168.10.2` | Surface float, gateway, time reference |

`ROS_DOMAIN_ID=42` on all machines.

**Time synchronisation is an experimental control, not an implementation
detail** (ADR-018). The supervision target is a displacement measured on the
buoy compared against motion integrated on the vehicle; any inter-machine clock
offset appears directly as training error. Neither machine has a battery-backed
clock and there is no internet at the field site, so:

- chrony runs with the buoy Pi as the Jetson's **sole** reference
- the Pi is configured `local stratum 10`, so it continues serving time with no
  upstream source
- internet NTP is disabled on the Jetson so bench configuration matches field
  configuration

**Validated:** ROS 2 discovery and data delivery confirmed over the wired link
with the vehicle's WiFi disabled, and synchronisation confirmed to survive loss
of internet. Measured offset is microsecond-level against a 50 ms requirement.

Run `scripts/preflight_time.sh` before each recording session; the output is
stored alongside the bag so synchronisation quality is a recorded property of
the dataset.

---

## Data

Rosbags are treated as versioned datasets. **Binaries are excluded from git**;
`docs/datasets.md` records for each session its identifier, date, location,
conditions, vehicle configuration, synchronisation quality and duration.
Committed metadata, external binaries.

---

## Architecture decision records

`docs/adr/` records each significant decision with its context, the
alternatives considered, and the consequences. Notable entries:

| ADR | Decision |
|---|---|
| 002 | ArduSub retained as sensor hub; all control in ROS 2 (no ArduSub frame models a fin-steered vehicle) |
| 004 | Ballast on the Jetson via BTS7960, not the PWM board — a DC motor through an H-bridge needs direction and speed, not servo position |
| 007 | Split power domains: propulsion and electronics on separate packs, eliminating brownout by topology rather than mitigation |
| 010 | NVIDIA L4T packages held; never updated via apt |
| 011 | mavros command and override paths found non-functional on this autopilot combination |
| 012 | Actuation moved to the companion computer with a dedicated PWM generator |
| 013 | Measured, non-standard servo ranges, per axis |
| 016 | Measured ESC neutral; PWM board oscillator deviation |
| 018 | Time synchronisation architecture |

---

## Project phases

| Phase | Content | Status |
|---|---|---|
| 0 | Bench bring-up: software skeleton, sensor bridge, safety architecture, actuation, teleoperation | Complete |
| 1 | Vehicle model, coordinate-frame tree, lightweight dynamics simulation | In progress |
| 2 | Heading and depth control | Planned |
| 3 | Surface autonomy: guidance law and waypoint missions | Planned |
| 4 | Field data campaign: synchronised multi-machine dataset | Planned |
| 5 | Learned dead-reckoning model, training pipeline, evaluation harness | Planned |
| 6 | Onboard deployment, final trials, artefacts | Planned |

---

## Governance

The research involves no human participants, no personal data and no human
tissue, and was reviewed under the University's UREC 1 procedure. A health and
safety risk assessment covering water entry, lithium-polymer battery hazards,
the powered thruster, loss of the vehicle and lone working has been approved.
No field work commences until both are signed and the pre-launch checklist has
been completed for that session.

---

## Licence

MIT. See `LICENSE`.
