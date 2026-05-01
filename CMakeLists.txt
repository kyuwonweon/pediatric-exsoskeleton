cmake_minimum_required(VERSION 3.8)
project(micropython_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/multi_robot.py
  scripts/ROS2tcp.py
  scripts/ROS2udp.py
  scripts/tcp2ROS.py
  scripts/udp2ROS.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()