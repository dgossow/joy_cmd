cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_make_distribution(0.1.0)

rosbuild_init()

find_package(Boost REQUIRED COMPONENTS system)

rosbuild_add_executable(joy_cmd
  src/joy_cmd_node.cpp
  src/joy_cmd.cpp
  )
target_link_libraries(joy_cmd
   ${Boost_LIBRARIES}
)

