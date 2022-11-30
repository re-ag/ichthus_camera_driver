
# Copyright 2022. Jaeun Kim (763k357@gmail.com) and Kanghee Kim(kim.kanghee@gmail.com) all rights reserved.
# added by ICHTHUS, Jaeeun Kim on 20221130

cmake_minimum_required(VERSION 3.8)
project(ichthus_camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(autoware_cmake REQUIRED)
find_package(OpenCV REQUIRED)

autoware_package()

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

ament_auto_add_library(ichthus_camera_driver SHARED
  src/ichthus_camera_driver_node.cpp
)

target_link_libraries(ichthus_camera_driver
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(ichthus_camera_driver
  PLUGIN "ichthus_camera_driver::IchthusCameraDriverNode"
  EXECUTABLE ichthus_camera_driver_node
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

ament_auto_package(INSTALL_TO_SHARE
launch
)

