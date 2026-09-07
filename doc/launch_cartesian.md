# Movensys 4-axis cartesian robot

WMX R2 owns the motion; MoveIt only plans. There is no `ros2_control_node`
anywhere in this path — `joint_trajectory_controller` below *is* the
`FollowJointTrajectory` action server that MoveIt executes against.

Root is required because the EtherCAT master runs in a real-time thread.

# For trajectory control
```
sudo --preserve-env=PATH \
     --preserve-env=AMENT_PREFIX_PATH \
     --preserve-env=COLCON_PREFIX_PATH \
     --preserve-env=PYTHONPATH \
     --preserve-env=LD_LIBRARY_PATH \
     --preserve-env=ROS_DISTRO \
     --preserve-env=ROS_VERSION \
     --preserve-env=ROS_PYTHON_VERSION \
     --preserve-env=ROS_DOMAIN_ID \
     --preserve-env=RMW_IMPLEMENTATION \
     bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && source $HOME/workspaces/movensys_ws/install/setup.bash && \
     ros2 launch wmx_r2_package wmx_r2_cartesian.launch.py use_sim_time:=false"
```

Set `use_sim_time:=true` for the HiL run (Isaac Sim publishes the clock).

Two bring-up options on `wmx_core_motion_node`:

| arg | default | what it does |
|---|---|---|
| `auto_servo_on` | `true` | clears alarms and enables the servos (WMX3 rejects motion in servo-off) |
| `auto_home` | `false` | homes with `HomeType CurrentPos` (no motion). Off, same as the manipulator launches — see below |

The cartesian drives are absolute encoders (`AbsoluteEncoderMode` 1 for axes
0..3 in `config/cartesian_wmx_parameters.xml`, as in the cr3a/cr5a files), so
they remember their position across a power cycle and nothing has to be homed
at bring-up.

One-time setup (and the procedure for a deliberate re-zero):

1. Park the machine at the model home pose — X/Y centred, Z at the bottom of
   its travel.
2. Home once. `HomeType CurrentPos` moves nothing; it labels that pose with each
   axis' `HomePosition` (`0.012` for axis 2, `0` for the rest) and makes WMX3
   compute `AbsoluteEncoderHomeOffset` for the axis:
   ```
   ros2 service call /wmx/axis/homing wmx_r2_message/srv/SetAxis \
     "{index: [0,1,2,3], data: [0,0,0,0]}"
   ```
3. Export the parameters from WOS / WMX Studio over
   `config/cartesian_wmx_parameters.xml`. wmx-r2 imports that file at start and
   never exports, so an offset that is not exported is gone at shutdown.

`AbsoluteEncoderHomeOffset` is not meant to be typed in by hand — WMX3 writes it
on homing. Do **not** pass `auto_home:=true` on these drives: it repeats step 2
at every start, overwriting the remembered position.

Why the position matters to MoveIt at all: `axis_z` travels [0.012, 0.090] m in
the URDF, so a raw reading of `0.000` is *outside* the joint limits and every
plan aborts with

```
[check_start_state_bounds]: Joint 'axis_z' from the starting state is outside
bounds by: [0 ] should be in the range [0.012 ], [0.09 ].
```

Homing must also be re-run after any gear ratio change — WMX3 recomputes the
zero position from it.

# Axis map

| URDF joint | WMX axis | travel (m)        |
|------------|----------|-------------------|
| `axis_x`   | 0        | [-0.115,  0.100]  |
| `axis_y`   | 1        | [-0.100,  0.100]  |
| `axis_z`   | 2        | [ 0.012,  0.090]  |
| `axis_r`   | 3        | continuous (rad)  |

`joint_trajectory_controller` resolves each column of an incoming trajectory
through `trajectory.joint_names`, using the `joint_name` / `joint_axes` pair in
`config/cartesian_config.yaml`. Do not remove `joint_name`: without it the
columns are taken positionally, and the orders do not agree — Pilz emits
`axis_x, axis_y, axis_z` while the robot model orders the joints
`axis_y, axis_x, axis_z`, so X and Y would be driven onto each other's axis with
nothing in the log to show for it.

The node also refuses a trajectory whose first point is more than
`start_tolerance` (0.01 m) away from where the axes actually are, instead of
letting WMX jump to it. That is the symptom a wrong mapping produces.

# Units

The ROS nodes do no unit conversion, so the WMX user unit has to *be* the URDF
unit: a metre for axes 0/1/2, a radian for axis 3. WMX converts user units to
servo pulses with the **gear ratio**

```
pulse = user unit * AxisGearRatioNumerator / AxisGearRatioDenominator
```

so that ratio is the number of pulses per metre. It is `1000000 / 1` in
`config/cartesian_wmx_parameters.xml` (1 pulse = 1 um). Left at the WMX default
of `1 / 1`, a 20 mm move is commanded as `0.02` **pulses**, the axis does not
move and the feedback stays at `0.000`.

(`AxisUnit` is unrelated — it rounds the *reported* position to a multiple of
itself and stays `0`, i.e. disabled.)

# Before running on real hardware

`config/cartesian_wmx_parameters.xml` is otherwise still a copy of
`default_wmx_parameters.xml`. Export the real one from WMX Studio; see the TODO
block at the top of that file for what has to be right. The gear ratio above is
a nominal 1 um/pulse placeholder — the real value is (encoder pulses per rev) /
(ball screw lead in metres), and getting it wrong makes the robot travel a
completely wrong distance.

# Isaac Sim (HiL)

The scene drives its articulation from a ROS2 Subscribe JointState node on
`/joint_command`, which is what `isaacsim_joint_topic` in
`config/cartesian_config.yaml` publishes to.

The scene must **not** publish `/joint_states`. The WMX encoders are the only
source of joint state; a second publisher makes MoveIt's start state flap
between the two sources, and planning then fails intermittently for no visible
reason. Delete the ROS2 Publish JointState node from the scene graph (or point
it at some other topic) and check with:

```
ros2 topic info /joint_states     # Publisher count must be 1
```
