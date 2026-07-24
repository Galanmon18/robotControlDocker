# uma_fp_control

ROS Noetic package implementing hybrid position+force control strategies for autonomous laparoscopic soft-tissue suturing with a UR3e robot. Developed as part of a PhD thesis on cognitive architecture for Level 4 surgical autonomy at Universidad de Málaga (IBIMA).

---

## Package structure

```
uma_fp_control/
├── src/            Coordinators — orchestrate services into a complete application
├── services/       Control nodes — self-contained executables with ROS I/O
├── dependecies/    Libraries — reusable modules with no inter-dependency
├── mocks/          Simulators — virtual sensors and environment models
├── launch/         Launch files — configure and start nodes
└── config/         Calibration files — tool and inter-robot transforms
```

Each layer only depends on the layers below it. A `service` may use `dependecies`; a `dependency` knows nothing about ROS topics or services; a `mock` simulates a physical component without being one. This follows a Clean Architecture pattern adapted for ROS.

---

## Control strategies

Three interchangeable control executables share the same launch interface:

### `rotationMatrix`
Hybrid position + force controller with **geometrically imposed RCM** (Remote Center of Motion). Each control cycle, the desired TCP orientation is constructed so that the instrument Z-axis points from the estimated fulcrum toward the desired TTP position, forcing the instrument to pivot around the trocar. Abdominal forces are summed as a flee velocity on top of the position command. Tissue force feedback limits penetration via an integral setpoint modifier.

Key modules: `RCMGeometry`, `TipForceController`, `AbdomenForceController`.

### `jacobianThesis`
Architecturally equivalent to `rotationMatrix`. The abdominal force flee velocity was the primary development focus of this executable. Both share the same RCM geometric constraint.

Key modules: `RCMGeometry`, `TipForceController`, `AbdomenForceController`.

### `complianceThesis`
**Decoupled** hybrid controller where the RCM constraint is **not imposed geometrically**. The position loop drives the instrument linearly toward the goal. Abdominal forces generate desired tilt angles via the compliance law:

```
θ_x = F_x / (K_abd · (L − ρ))
θ_y = F_y / (K_abd · (L − ρ))
```

where `L` is the total instrument length and `ρ` is the external segment (trocar to flange, measured by `abdomenForceSensorMock`). The RCM constraint emerges from physical contact with the trocar rather than from a known fulcrum position. Robust to fulcrum estimation errors.

Key modules: `TipForceController`, `AbdomenComplianceController`.

---

## Quick start

**One robot (default parameters):**
```bash
roslaunch uma_fp_control one_fp_robot.launch
```

**One robot with explicit parameters:**
```bash
roslaunch uma_fp_control one_fp_robot.launch \
  prefix:=auto \
  control_type:=complianceThesis \
  p_estimado:=0.15 \
  tool_length:=0.34 \
  move_to_init:=false
```

**Two robots with different strategies:**
```bash
roslaunch uma_fp_control two_fp_robot.launch \
  prefix_1:=auto  control_type_1:=rotationMatrix   p_estimado_1:=0.15 \
  prefix_2:=darel control_type_2:=complianceThesis p_estimado_2:=0.10
```

**With real abdominal F/T sensor:**
```bash
roslaunch uma_fp_control one_fp_robot.launch \
  prefix:=auto \
  abdomen_source:=sensor \
  abdomen_sensor_ip:=192.168.1.1
```

**Custom tool geometry:**
```bash
roslaunch uma_fp_control one_fp_robot.launch \
  tool_config:=$(rospack find uma_fp_control)/config/tools/camera.yaml
```

---

## Launch parameters (`robot.launch`)

`robot.launch` is the base unit. It is never launched directly — it is included by `one_fp_robot.launch`, `two_fp_robot.launch`, etc.

### Identity

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `prefix` | `str` | — | Robot ROS namespace. All topics appear under `/<prefix>/`. Each robot in a multi-robot setup must have a unique prefix. **Required.** |
| `control_type` | `str` | `rotationMatrix` | Control executable: `rotationMatrix` \| `jacobianThesis` \| `complianceThesis` |

### Initial movement

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `move_to_init` | `bool` | `true` | Move to the home joint configuration before starting the control loop. Set `false` when the robot is already in position or during software-only development. |

### Tool geometry

Tool geometry is loaded from a YAML file at launch time. The file is processed and placed on the ROS param server under `/<prefix>/tool/`. No recompilation is needed to change or add tools.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tool_config` | `str` | `config/tools/tool_z_axis.yaml` | Path to tool YAML file defining TCP and DFP vectors. |
| `tool_length` | `double` (m) | `0.34` | Total instrument length flange-to-tip. Substituted into the YAML where `tcp[z] = 0`. |
| `p_estimado` | `double` (m) | `0.15` | Initial flange-to-trocar distance estimate. Substituted into the YAML where `dfp[z] = 0`. Affects `E_T_Fp` at startup. |

**Tool YAML format** (`config/tools/`):
```yaml
tcp: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # [x, y, z, rx, ry, rz] — z=0 → use tool_length
dfp: [0.0, 0.0, 0.0]                    # [x, y, z]              — z=0 → use p_estimado
length_axis: 2                           # axis index where tool_length/p_estimado apply
```

### PID gains

Cartesian position controller running at 125 Hz. Produces `[vx, vy, vz, wx, wy, wz]`.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `kp` | `double` | `1.0` | Proportional gain. Main tuning parameter. |
| `ki` | `double` | `0.0` | Integral gain. Use only if a persistent steady-state offset is observed. |
| `kd` | `double` | `0.0` | Derivative gain. Usually 0 for velocity-controlled robots. |
| `kf` | `double` | `0.175` | Feed-forward gain (reserved, not active). |

### F/T sensors

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sensor_ip` | `str` | `192.168.1.100` | IP of the **distal** F/T sensor at the instrument tip (ATI Hex-E, UDP). Used by `TipForceController`. |
| `abdomen_source` | `str` | `topic` | Abdominal force source: `sensor` \| `topic` \| `none` |
| `abdomen_sensor_ip` | `str` | `192.168.1.1` | IP of the **proximal** F/T sensor at the trocar. Only used when `abdomen_source=sensor`. |

**`abdomen_source` options:**

| Value | Behaviour | When to use |
|-------|-----------|-------------|
| `sensor` | Blocking UDP read from `abdomen_sensor_ip` each cycle (8 ms timeout). | Real experiments with sensor at trocar. |
| `topic` | Subscribes to `/abdomen_force [Fx, Fy, Fz, ρ]`. | Simulation with `abdomenForceSensorMock`. |
| `none` | `forceAbdomen = {0, 0, 0}`. No abdominal feedback. | Second robot without sensor, or baseline. |

### Initial joint configuration

Angles in **degrees**, converted to radians internally. Used when `move_to_init=true`.

| Parameter | Default | Joint |
|-----------|---------|-------|
| `base` | `90.0` | Joint 1 — base rotation |
| `shoulder` | `-90.0` | Joint 2 — shoulder |
| `elbow` | `90.0` | Joint 3 — elbow |
| `wrist1` | `-90.0` | Joint 4 |
| `wrist2` | `-90.0` | Joint 5 |
| `wrist3` | `0.0` | Joint 6 — tool rotation |

---

## Simulation (mocks)

For experiments without physical sensors, launch the mock nodes alongside the control node:

```bash
# Simulate abdominal forces (elastic model at trocar plane)
rosrun uma_fp_control abdomenForceSensorMock

# Simulate tissue contact forces (unilateral spring at z_surface)
rosrun uma_fp_control tissueForceSensorMock

# Timed goal sequence + fulcrum perturbations for reproducible experiments
rosrun uma_fp_control temporalMock
```

`abdomenForceSensorMock` supports configurable perturbations (sine, step, ramp) for testing disturbance rejection. See `mocks/abdomenForceSensorMock.cpp` for parameters.

---

## Topics

### Published (per robot, under `/<prefix>/`)

| Topic | Type | Description |
|-------|------|-------------|
| `/<prefix>/pose_topic` | `Float64MultiArray` | T_TTP: world→TTP, 4×4 column-major (16 elements) |
| `/<prefix>/effectorFinal_topic` | `Float64MultiArray` | T_E: world→effector, 4×4 column-major (16 elements) |
| `/<prefix>/velocity_topic` | `Float64MultiArray` | Commanded velocity `[vx, vy, vz, wx, wy, wz]` (m/s, rad/s) |
| `/<prefix>/fulcrum` | `Float64MultiArray` | Estimated fulcrum position `[x, y, z]` (m) |

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/<prefix>/coordinator/goal_position` | `geometry_msgs/Point` | Desired TTP position in world frame (m) |
| `/abdomen_force` | `Float64MultiArray` | `[Fx, Fy, Fz, ρ]` — forces (N) + external instrument length (m). When `abdomen_source=topic`. |
| `/tissue_force` | `Float64MultiArray` | `[Fx, Fy, Fz]` — tissue contact forces at instrument tip (N) |

---

## Dependencies

- ROS Noetic
- Eigen3
- Universal Robots ROS driver (`ur_robot_driver`)
- `urdf` (tool geometry parsing)
- ATI F/T sensor UDP driver (included in `dependecies/hex_ft_udp.hpp`)

---

## Package layout detail

```
dependecies/
├── abdomen_compliance_controller   Compliance law: F → θ_des → PI → [wx, wy]
├── abdomen_force_controller        Flee controller: F → velocity correction
├── tip_force_controller            Tissue force limiter: integral setpoint modifier
├── rcm_geometry                    Stateless RCM pose construction from fulcrum + goal
├── computeError                    Cartesian pose error (position + rotation)
├── computePID                      Cartesian PID controller
├── computeT                        Homogeneous transform utilities
├── fulcrumEstimation               Fulcrum position estimator (IN PROGRESS)
├── selectTool                      Tool geometry loader (YAML + legacy int)
├── ur_script                       UR robot command interface (speedl, movej, set_tcp)
└── hex_ft_udp                      ATI F/T sensor UDP client

services/
├── FPcontrol_RotationMatrix        rotationMatrix control node
├── FPcontrol_jacobian              jacobianThesis control node
├── FPcontrol_compilance            complianceThesis control node
└── ...                             Data logging, calibration, evaluation nodes

mocks/
├── abdomenForceSensorMock          Elastic trocar model + perturbation system
├── tissueForceSensorMock           Unilateral spring tissue contact model
└── temporalMock                    Timed goal + perturbation sequencer

src/
├── coordinatorOneRobot             Single-robot application coordinator
├── coordinatorTwoRobot_*           Two-robot coordinator variants
└── ...

config/
└── tools/
    ├── tool_z_axis.yaml            Instrument aligned with Z axis
    ├── tool_y_axis.yaml            Instrument aligned with Y axis
    └── camera.yaml                 Camera tool (no trocar pivot)
```