cmake_minimum_required(VERSION 3.8)
project(ESP32_connection_pkg)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/esp32_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
