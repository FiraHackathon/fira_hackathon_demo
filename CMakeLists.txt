cmake_minimum_required(VERSION 3.5)
project(cyrille_contrib)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    rviz
    cfg_chal1
    cfg_chal2
    cfg_chal3
    cfg_demo
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
