cmake_minimum_required(VERSION 3.5)
project(uv_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(uv_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
 
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotDeviceManager.msg" # 机器人设备管理器
  "msg/RobotMotionController.msg" # 机器人运动控制器

  "msg/PidParams.msg"
  "msg/PidControllers.msg"
  "msg/PidControllersState.msg"

  "msg/ThrustCurve.msg"
  "msg/ThrustCurves.msg"

  "msg/RobotAxis.msg"

  "msg/ImuData.msg"
  "msg/MotorThrust.msg"
  "msg/ServoSet.msg"
  "msg/TargetPosDown.msg"
  "msg/CabinState.msg"
  "msg/PropellerThrust.msg"
  "msg/WorkState.msg"
  
  "msg/Yolov8.msg"
  "msg/TargetParams.msg"
  "msg/PixelAxis.msg"
  "msg/TargetAxis.msg"
  "msg/LedControllers.msg"
  "msg/MagnetController.msg"
  
  "srv/DetectRequest.srv"
  DEPENDENCIES  geometry_msgs sensor_msgs # 添加以上自定义依赖的ros包
)


ament_package()


