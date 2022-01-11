cmake_minimum_required(VERSION 3.5)
project(reachy_gazebo)

find_package(ament_cmake REQUIRED)

find_package(rclpy REQUIRED)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/set_params.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
