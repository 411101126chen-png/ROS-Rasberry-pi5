cmake_minimum_required(VERSION 3.8)
project(my_first_pkg)

# 設定 C++ 標準
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 建立 talker 這個可執行檔，來源是 src/talker.cpp
add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp)

# 安裝 talker，讓 ros2 run 可以找到
install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
