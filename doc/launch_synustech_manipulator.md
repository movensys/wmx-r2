# Synustech manipulator (SIASUN DUCO GCR30-1100)

The Synustech robot is a mobile manipulator. WMX axes 0 and 1 are the drive
wheels of the base; the six arm joints are WMX axes **2 to 7**
(`joint_axes: [2, 3, 4, 5, 6, 7]`). The launch files below bring up the arm
only. The base axes are left servo-off.

`config/synustech_wmx_parameters.xml` is the vendor's WMX3 export of the whole
robot with the arm axes changed from degrees to radians. The DUCO joint drives
report output-side position, 524288 counts per joint revolution, so the gear
ratio is 524288 / 2π with no reducer term, unlike the Dobot files. Home offsets
are in pulses and unchanged from the export; the WMX zero is the DUCO
straight-up pose, which is also the URDF zero.

On the MoveIt side use `export MANIPULATOR_MODEL=synustech`. For the first
runs on hardware set `vel_scale` and `acc_scale` in
`config/synustech/moveit2_client.yaml` to 0.1.

# Simulation platform (no hardware)
When `/opt/wmx3/Module.ini` has the simu platform enabled, the virtual
encoders read zero pulses and the real file's home offsets would place the
joints at -5.68, 0.57, -6.83, -4.35, -4.37, 1.90 rad, outside the URDF
limits. Use the `_sim` parameter file, which zeroes the offsets on axes 2 to 7
and is otherwise identical:
```
ros2 launch wmx_r2_package wmx_r2_synustech_manipulator.launch.py use_sim_time:=false \
  wmx_param_file:=$HOME/workspaces/movensys_ws/install/wmx_r2_package/share/wmx_r2_package/config/synustech_wmx_parameters_sim.xml
```
Never load the `_sim` file on the real robot: the zero would then be wherever
the encoders happen to read, not the DUCO straight-up pose.

# Trajectory control
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
     ros2 launch wmx_r2_package wmx_r2_synustech_manipulator.launch.py use_sim_time:=false"
```


# ROS2 Control (joint state relay only)
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
     ros2 launch wmx_r2_control wmx_r2_control_synustech_manipulator.launch.py use_sim_time:=false"
```

# Shutdown
Ctrl-C the launch, then servo off the arm axes:
```
ros2 service call /wmx/axis/set_on wmx_r2_message/srv/SetAxis "{index: [2,3,4,5,6,7], data: [0,0,0,0,0,0]}"
```
Do not call `/wmx/axis/homing` on this arm. All six axes use absolute
encoders and homing would overwrite the zero with the current position.
