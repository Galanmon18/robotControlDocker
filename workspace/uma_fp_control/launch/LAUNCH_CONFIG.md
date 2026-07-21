# Launch Configuration Reference — `rotationMatrix` node

This document describes all configurable parameters for the `rotationMatrix` control node (`FPcontrol_RotationMatrix`), which implements the hybrid RCM position + force controller for a laparoscopic instrument.

---

## Robot identity

| Parameter | Type | Example | Description |
|-----------|------|---------|-------------|
| `prefix` | `str` | `"auto"` | ROS namespace prefix for this robot. All topics published by this node will be under `/<prefix>/`. Must match the namespace group in the launch file. When running two robots, each must have a unique prefix (e.g. `"auto"` and `"darel"`). |

---

## PID gains (Cartesian position controller)

The position controller runs at 125 Hz and takes the 6-DOF pose error between the desired and current TTP pose, producing a Cartesian velocity command.

| Parameter | Type | Example | Description |
|-----------|------|---------|-------------|
| `kp` | `double` | `1.0` | Proportional gain. Main tuning knob — increase for faster response, decrease if oscillation appears. |
| `ki` | `double` | `0.0` | Integral gain. Eliminates steady-state error. Start at 0; small values only (e.g. 0.01) if persistent offset is observed. |
| `kd` | `double` | `0.0` | Derivative gain. Adds damping. Usually left at 0 in velocity-controlled robots. |
| `kf` | `double` | `0.175` | Feed-forward gain. Currently read from launch but not actively used in the main control loop — reserved for future use. |

---

## Tool and trocar geometry

| Parameter | Type | Example | Description |
|-----------|------|---------|-------------|
| `tool_length` | `double` (m) | `0.34` | Total length of the laparoscopic instrument from flange to tip (metres). Used in the compliance law: `θ = F / (K · (L − ρ))` where `L` is this value. |
| `p_estimado` | `double` (m) | `0.2` | Initial estimate of the distance from the robot flange to the trocar entry point (metres). Used at startup to compute the initial fixed transform `E_T_Fp` (effector → fulcrum). If inaccurate, the RCM constraint will have an offset until the fulcrum estimator converges. |
| `type` | `int` | `1` | Tool geometry type identifier, passed to `selectTool`. Determines which fixed transforms (`E_T_TTP`, `E_T_Fp`, `TCP`) are loaded for this instrument. Check `selectTool.cpp` for the mapping. |

---

## Tissue F/T sensor (instrument tip)

| Parameter | Type | Example | Description |
|-----------|------|---------|-------------|
| `sensor_ip` | `str` | `"192.168.1.100"` | IP address of the distal F/T sensor (Hex-E or equivalent, ATI UDP protocol) measuring forces at the instrument tip against tissue. Used by the `TipForceController` to limit contact force. |

---

## Abdominal force source

Controls where the node reads the abdominal contact forces from. Only one source is active at a time.

| Parameter | Type | Options | Description |
|-----------|------|---------|-------------|
| `abdomen_source` | `str` | `"sensor"` / `"topic"` / `"none"` | Selects the abdominal force input source (see table below). |
| `abdomen_sensor_ip` | `str` | `"192.168.1.1"` | IP address of the proximal F/T sensor at the trocar interface. Only used when `abdomen_source = "sensor"`. |

### `abdomen_source` options

| Value | Behaviour | When to use |
|-------|-----------|-------------|
| `"sensor"` | Reads forces directly from the UDP F/T sensor at `abdomen_sensor_ip` each control cycle (blocking read, 8 ms timeout). | Real experiments with physical sensor connected. |
| `"topic"` | Subscribes to `/abdomen_force` topic `[Fx, Fy, Fz, ρ]`. | Simulation with `abdomenForceSensorMock`, or when sensor is on another machine. |
| `"none"` | `forceAbdomen` stays `{0, 0, 0}`. No abdominal feedback. | Second robot without sensor, or baseline experiments with no force feedback. |

---

## Initial joint configuration

Joint angles in **degrees** for the robot's starting position. The robot moves to this configuration during `initializeRobot()` before the control loop starts.

| Parameter | Type | Joint |
|-----------|------|-------|
| `base` | `double` (deg) | Joint 1 — base rotation |
| `shoulder` | `double` (deg) | Joint 2 — shoulder |
| `elbow` | `double` (deg) | Joint 3 — elbow |
| `wrist1` | `double` (deg) | Joint 4 — wrist 1 |
| `wrist2` | `double` (deg) | Joint 5 — wrist 2 |
| `wrist3` | `double` (deg) | Joint 6 — wrist 3 (tool rotation) |

Angles are converted to radians internally (`× π/180`).

---

## Minimal working example

```xml
<arg name="prefix_auto"  default="auto"/>
<arg name="kp"           default="1.0"/>
<arg name="tool_length"  default="0.34"/>
<arg name="base"         default="90.0"/>
<arg name="shoulder"     default="-90.0"/>
<arg name="elbow"        default="90.0"/>
<arg name="wrist1"       default="-90.0"/>
<arg name="wrist2"       default="-90.0"/>
<arg name="wrist3"       default="0.0"/>

<group ns="$(arg prefix_auto)">
  <remap from="/ur_hardware_interface/script_command"
         to="/$(arg prefix_auto)/ur_hardware_interface/script_command" />
  <node name="MyControl" pkg="uma_fp_control" type="rotationMatrix" output="screen">
    <param name="prefix"           value="$(arg prefix_auto)"  type="str"/>
    <param name="kp"               value="$(arg kp)"           type="double"/>
    <param name="ki"               value="0.0"                 type="double"/>
    <param name="kd"               value="0.0"                 type="double"/>
    <param name="kf"               value="0.175"               type="double"/>
    <param name="p_estimado"       value="0.2"                 type="double"/>
    <param name="tool_length"      value="$(arg tool_length)"  type="double"/>
    <param name="type"             value="1"                   type="int"/>
    <param name="sensor_ip"        value="192.168.1.100"       type="str"/>
    <param name="abdomen_source"   value="topic"               type="str"/>
    <param name="abdomen_sensor_ip" value="192.168.1.1"        type="str"/>
    <param name="base"             value="$(arg base)"         type="double"/>
    <param name="shoulder"         value="$(arg shoulder)"     type="double"/>
    <param name="elbow"            value="$(arg elbow)"        type="double"/>
    <param name="wrist1"           value="$(arg wrist1)"       type="double"/>
    <param name="wrist2"           value="$(arg wrist2)"       type="double"/>
    <param name="wrist3"           value="$(arg wrist3)"       type="double"/>
  </node>
</group>
```

---

## Topics published by this node

All topics are under the node namespace `/<prefix>/`.

| Topic | Type | Description |
|-------|------|-------------|
| `/<prefix>/pose_topic` | `Float64MultiArray` | T_TTP: world→TTP transform, 4×4 column-major (16 elements) |
| `/<prefix>/effectorFinal_topic` | `Float64MultiArray` | T_E: world→effector transform, 4×4 column-major (16 elements) |
| `/<prefix>/velocity_topic` | `Float64MultiArray` | Commanded Cartesian velocity `[vx, vy, vz, wx, wy, wz]` (m/s, rad/s) |
| `/<prefix>/fulcrum` | `Float64MultiArray` | Estimated fulcrum position `[x, y, z]` (m) |

## Topics subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/<prefix>/coordinator/goal_position` | `geometry_msgs/Point` | Desired TTP position in world frame (m) |
| `/abdomen_force` | `Float64MultiArray` | `[Fx, Fy, Fz, ρ]` — abdominal forces (N) + external instrument length (m). Only when `abdomen_source = "topic"`. |
| `/tissue_force` | `Float64MultiArray` | `[Fx, Fy, Fz]` — tissue contact forces at instrument tip (N) |
