cmake_minimum_required(VERSION 3.8)
project(my_hexapod_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    
    "msg/WorldFeetPlace.msg"
    "msg/ArrayMessage.msg"
    "msg/LegPath.msg"
    "msg/PathVarNCmdVel.msg"
    "msg/PathVariables.msg"
    "msg/ThetaMessage.msg"
  # "msg/HardwareStatus.msg"
  # "msg/LEDStatus.msg"
  # "msg/HexapodLegTrajectory.msg"
  # "srv/ComputeRectangleArea.srv"
  # "srv/SetLedStatus.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)


ament_package()
