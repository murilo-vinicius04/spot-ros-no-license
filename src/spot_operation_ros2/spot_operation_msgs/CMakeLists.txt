cmake_minimum_required(VERSION 3.8)
project(spot_operation_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraspResult.msg"
  "msg/ObjectDetection.msg"
  "msg/GestureCommand.msg"
  "msg/OperationState.msg"
  "srv/ExecuteGrasp.srv"
  "srv/SetGraspStrategy.srv"
  "srv/GetOperationState.srv"
  "action/GraspObject.action"
  "action/ManipulateObject.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_package() 