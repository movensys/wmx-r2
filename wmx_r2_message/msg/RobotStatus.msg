std_msgs/Header header
int32 robot_id
int32 motion_state         # kinematics MotionState
int32 motion_error_code    # kinematics MotionErrorCode
int32 motion_error_joint   # joint that raised the error, -1 if not applicable

RobotCartesianPose tool_pose_cmd
RobotCartesianPose tool_pose_fb

float64[] joint_pos_cmd      # deg for revolute joints, mm for prismatic
float64[] joint_pos_fb
float64[] joint_velocity_cmd
float64[] joint_velocity_fb
float64[] joint_torque_fb    # Nm, Articulate6Axis only
