cmake_minimum_required(VERSION 3.8)
project(sail_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/WingsailActuator.msg"
    "msg/WindData.msg"
    "msg/WingOrientation.msg"
    "msg/WingsailFeedback.msg"
    "msg/SailState.msg"
    "msg/AuxPropulsionFeedback.msg"
    "msg/PwmOutput.msg"
    "msg/AnalogInput.msg"
    "msg/PidTuning.msg"
    "msg/PidDebug.msg"
    "msg/SailPosition.msg"
    "msg/SailAngleType.msg"
    "msg/WindType.msg"
    "srv/GetPidTuning.srv"
    "srv/SetPidTuning.srv"
    "srv/SwitchPidDebug.srv"
    "action/DeployAuxPropulsion.action"
    "action/InitializeActuator.action"
    "action/DeploySail.action"
    DEPENDENCIES builtin_interfaces std_msgs)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
