cmake_minimum_required(VERSION 3.10)
project(xgc2_gazebo_rendering VERSION 0.1.0 LANGUAGES CXX)

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED)

catkin_package()

add_library(xgc2_gazebo_rendering SHARED src/gazebo_rendering_plugin.cpp)
target_compile_features(xgc2_gazebo_rendering PRIVATE cxx_std_17)
target_compile_options(xgc2_gazebo_rendering PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(xgc2_gazebo_rendering PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(xgc2_gazebo_rendering PRIVATE ${GAZEBO_LIBRARIES})
install(TARGETS xgc2_gazebo_rendering LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
