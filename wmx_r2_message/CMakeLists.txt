cmake_minimum_required(VERSION 3.10)
project(wmx_r2_message)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetEngine.srv"
  "srv/SetNodeState.srv"
  "srv/GetNodeStates.srv"
  "srv/SetAxes.srv"
  "srv/SetAxesGearRatio.srv"
  "srv/GetIoBit.srv"
  "srv/GetIoBytes.srv"
  "srv/SetIoBit.srv"
  "srv/SetIoBytes.srv"

  "srv/EcatGetNetworkState.srv"
  "srv/EcatRegisterRead.srv"
  "srv/EcatResetStatistics.srv"
  "srv/EcatScanNetwork.srv"
  "srv/EcatStartHotconnect.srv"

  "srv/LoadWmxParams.srv"
  "srv/GetWmxParams.srv"

  "msg/AxesVelocity.msg"
  "msg/AxesStatus.msg"
  "msg/AxesPose.msg"
  DEPENDENCIES std_msgs
)

install(DIRECTORY
  srv
  msg
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_interfaces test/test_interfaces.cpp)
  if(TARGET test_interfaces)
    rosidl_get_typesupport_target(cpp_typesupport_target
      ${PROJECT_NAME} rosidl_typesupport_cpp)
    target_link_libraries(test_interfaces "${cpp_typesupport_target}")
  endif()
endif()

ament_package()
