cmake_minimum_required(VERSION 3.10)
project(wmx_r2_package)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-fsigned-char)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(wmx_r2_message REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(WMX3_SDK_PATH "/opt/wmx3" CACHE PATH "Path to WMX3 SDK installation")

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(joint_state_broadcaster src/joint_state_broadcaster.cpp)
ament_target_dependencies(joint_state_broadcaster rclcpp std_msgs sensor_msgs std_srvs wmx_r2_message)
target_include_directories(joint_state_broadcaster PRIVATE include ${WMX3_SDK_PATH}/include ${WMX3_SDK_PATH}/sample/Header)
target_link_libraries(joint_state_broadcaster coremotionapi ioapi wmx3api imdll)
target_link_directories(joint_state_broadcaster PRIVATE ${WMX3_SDK_PATH}/lib)
target_compile_definitions(joint_state_broadcaster PRIVATE WMX3_SDK_PATH="${WMX3_SDK_PATH}/")

add_executable(joint_trajectory_controller src/joint_trajectory_controller.cpp)
ament_target_dependencies(joint_trajectory_controller
  rclcpp rclcpp_action std_msgs std_srvs control_msgs trajectory_msgs wmx_r2_message)
target_include_directories(joint_trajectory_controller PRIVATE include ${WMX3_SDK_PATH}/include ${WMX3_SDK_PATH}/sample/Header)
target_link_libraries(joint_trajectory_controller coremotionapi advancedmotionapi ioapi wmx3api imdll)
target_link_directories(joint_trajectory_controller PRIVATE ${WMX3_SDK_PATH}/lib)
target_compile_definitions(joint_trajectory_controller PRIVATE WMX3_SDK_PATH="${WMX3_SDK_PATH}/")

add_executable(differential_drive_controller src/differential_drive_controller.cpp)
ament_target_dependencies(differential_drive_controller
  rclcpp std_msgs std_srvs geometry_msgs nav_msgs
  tf2 tf2_ros wmx_r2_message)
target_include_directories(differential_drive_controller PRIVATE include ${WMX3_SDK_PATH}/include ${WMX3_SDK_PATH}/sample/Header)
target_link_libraries(differential_drive_controller coremotionapi wmx3api imdll)
target_link_directories(differential_drive_controller PRIVATE ${WMX3_SDK_PATH}/lib)
target_compile_definitions(differential_drive_controller PRIVATE WMX3_SDK_PATH="${WMX3_SDK_PATH}/")

add_executable(gripper_controller src/gripper_controller.cpp)
ament_target_dependencies(gripper_controller rclcpp std_msgs sensor_msgs std_srvs wmx_r2_message)
target_include_directories(gripper_controller PRIVATE include ${WMX3_SDK_PATH}/include ${WMX3_SDK_PATH}/sample/Header)
target_link_libraries(gripper_controller coremotionapi ioapi wmx3api imdll)
target_link_directories(gripper_controller PRIVATE ${WMX3_SDK_PATH}/lib)
target_compile_definitions(gripper_controller PRIVATE WMX3_SDK_PATH="${WMX3_SDK_PATH}/")

add_executable(wmx_engine_node src/wmx_engine_node.cpp)
ament_target_dependencies(wmx_engine_node rclcpp std_msgs std_srvs wmx_r2_message)
target_include_directories(wmx_engine_node PRIVATE include ${WMX3_SDK_PATH}/include ${WMX3_SDK_PATH}/sample/Header)
target_link_libraries(wmx_engine_node ecapi wmx3api imdll)
target_link_directories(wmx_engine_node PRIVATE ${WMX3_SDK_PATH}/lib)
target_compile_definitions(wmx_engine_node PRIVATE WMX3_SDK_PATH="${WMX3_SDK_PATH}/")

add_executable(wmx_core_motion_node src/wmx_core_motion_node.cpp)
ament_target_dependencies(wmx_core_motion_node rclcpp std_msgs std_srvs wmx_r2_message)
target_include_directories(wmx_core_motion_node PRIVATE include ${WMX3_SDK_PATH}/include ${WMX3_SDK_PATH}/sample/Header)
target_link_libraries(wmx_core_motion_node coremotionapi wmx3api imdll)
target_link_directories(wmx_core_motion_node PRIVATE ${WMX3_SDK_PATH}/lib)
target_compile_definitions(wmx_core_motion_node PRIVATE WMX3_SDK_PATH="${WMX3_SDK_PATH}/")

# Keyboard jog teleop. Input node only: no WMX3 include or link.
add_executable(jog_keyboard_node src/jog_keyboard_node.cpp)
ament_target_dependencies(jog_keyboard_node rclcpp wmx_r2_message)

add_executable(wmx_io_node src/wmx_io_node.cpp)
ament_target_dependencies(wmx_io_node rclcpp std_msgs wmx_r2_message)
target_include_directories(wmx_io_node PRIVATE include ${WMX3_SDK_PATH}/include ${WMX3_SDK_PATH}/sample/Header)
target_link_libraries(wmx_io_node ioapi wmx3api imdll)
target_link_directories(wmx_io_node PRIVATE ${WMX3_SDK_PATH}/lib)
target_compile_definitions(wmx_io_node PRIVATE WMX3_SDK_PATH="${WMX3_SDK_PATH}/")

add_executable(wmx_ethercat_node src/wmx_ethercat_node.cpp)
ament_target_dependencies(wmx_ethercat_node rclcpp std_msgs wmx_r2_message)
target_include_directories(wmx_ethercat_node PRIVATE include ${WMX3_SDK_PATH}/include ${WMX3_SDK_PATH}/sample/Header)
target_link_libraries(wmx_ethercat_node ecapi wmx3api imdll)
target_link_directories(wmx_ethercat_node PRIVATE ${WMX3_SDK_PATH}/lib)
target_compile_definitions(wmx_ethercat_node PRIVATE WMX3_SDK_PATH="${WMX3_SDK_PATH}/")

install(TARGETS
  joint_state_broadcaster
  joint_trajectory_controller
  differential_drive_controller
  gripper_controller
  wmx_engine_node
  wmx_core_motion_node
  wmx_io_node
  wmx_ethercat_node
  jog_keyboard_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_pytest_test(test_message_interfaces test/test_message_interfaces.py
    TIMEOUT 30
  )
  add_launch_test(test/test_engine_lifecycle.py
    TIMEOUT 60
  )
  add_launch_test(test/test_core_motion_node.py
    TIMEOUT 90
  )
  add_launch_test(test/test_io_node.py
    TIMEOUT 60
  )
  add_launch_test(test/test_ethercat_node.py
    TIMEOUT 60
  )

  # C++ unit tests for the diff-drive logic (header-only, WMX/ROS-free) declared
  # in include/differential_drive_controller.hpp.
  find_package(ament_cmake_gtest REQUIRED)
  foreach(_t
      test_diff_drive_kinematics
      test_odometry_integrator
      test_odom_delta_accumulator
      test_accel_estimator)
    ament_add_gtest(${_t} test/${_t}.cpp)
    target_include_directories(${_t} PRIVATE include)
  endforeach()
endif()

ament_package()
