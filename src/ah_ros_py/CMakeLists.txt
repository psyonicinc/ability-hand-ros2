cmake_minimum_required(VERSION 3.5)
project(ah_ros_py)

# Required for Python packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install entry points and any resources
install(
  DIRECTORY ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()