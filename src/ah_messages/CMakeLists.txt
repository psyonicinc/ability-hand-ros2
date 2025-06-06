cmake_minimum_required(VERSION 3.16)
project(ah_messages)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

# === Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  builtin_interfaces
  std_msgs
)

# === Custom message, service, and action files ===
set(msg_files
    msg/Digits.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES ${dependencies}
)

# === Export ===
ament_export_dependencies(rosidl_default_runtime ${dependencies})

ament_package()
