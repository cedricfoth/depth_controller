cmake_minimum_required(VERSION 3.5)
project(depth_control)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  nodes/depth_calculator.py
  nodes/depth_controller.py
  nodes/depth_setpoint.py
  nodes/pressure_noise_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
