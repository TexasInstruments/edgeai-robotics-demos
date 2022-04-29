cmake_minimum_required(VERSION 3.5)
project(vision_3d_perception)

find_package(catkin REQUIRED)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
