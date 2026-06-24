cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sim_worlds)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
