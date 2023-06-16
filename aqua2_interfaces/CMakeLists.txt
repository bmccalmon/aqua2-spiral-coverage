cmake_minimum_required(VERSION 3.5)
project(aqua2_interfaces)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED) 
find_package(geographic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StatusIndicator.msg"
  "msg/Command.msg"
  "msg/Health.msg"
  "msg/AutopilotCommand.msg"
  "msg/AutopilotTargets.msg"
  "msg/AutopilotOutput.msg"
  "msg/AutopilotPIDComponents.msg"
  "msg/PeriodicLegCommand.msg"
  "msg/LegState.msg"
  "msg/Waypoint.msg"
  "msg/RobodevelState.msg"
  "msg/BBox.msg"
  "msg/TagFeedback.msg"
  "srv/GetAutopilotState.srv"
  "srv/GetBool.srv"
  "srv/GetNamedBool.srv"
  "srv/GetNamedFloat.srv"
  "srv/GetNamedInt.srv"
  "srv/GetString.srv"
  "srv/MissionUpdate.srv"
  "srv/SetAutopilotMode.srv"
  "srv/SetBool.srv"
  "srv/SetFloat.srv"
  "srv/SetInt.srv"
  "srv/SetLegParams.srv"
  "srv/SetNamedFloat.srv"
  "srv/SetString.srv"
  DEPENDENCIES geographic_msgs std_msgs geometry_msgs
)

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

ament_package()
