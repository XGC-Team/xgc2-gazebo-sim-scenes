cmake_minimum_required(VERSION 3.10)
project(xgc2_gazebo_scene VERSION 0.1.0 LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
)
find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)

add_message_files(FILES
  ConvexPart.msg
  MotionSpec.msg
  ObstacleDefinition.msg
  ObstacleDefinitionArray.msg
  ObstacleState.msg
  ObstacleStateArray.msg
)
add_service_files(FILES
  ConfigureMotions.srv
  StopMotions.srv
)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES obstaclePathPlugin xgc2_gazebo_scene_motion xgc2_gazebo_scene_system
  CATKIN_DEPENDS geometry_msgs message_runtime roscpp std_msgs
)

add_library(xgc2_gazebo_scene_motion
  src/motion_controller.cpp
)
target_compile_features(xgc2_gazebo_scene_motion PUBLIC cxx_std_17)
target_compile_options(xgc2_gazebo_scene_motion PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(xgc2_gazebo_scene_motion
  PUBLIC include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(xgc2_gazebo_scene_motion PUBLIC ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(obstaclePathPlugin SHARED
  src/obstacle_path_plugin.cpp
)
target_compile_features(obstaclePathPlugin PUBLIC cxx_std_17)
target_include_directories(obstaclePathPlugin
  PRIVATE include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(obstaclePathPlugin PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(xgc2_gazebo_scene_system SHARED
  src/gazebo_scene_system_plugin.cpp
)
target_compile_features(xgc2_gazebo_scene_system PUBLIC cxx_std_17)
target_compile_options(xgc2_gazebo_scene_system PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(xgc2_gazebo_scene_system
  PRIVATE include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
)
add_dependencies(xgc2_gazebo_scene_system ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xgc2_gazebo_scene_system
  PRIVATE xgc2_gazebo_scene_motion ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
)

install(TARGETS obstaclePathPlugin xgc2_gazebo_scene_motion xgc2_gazebo_scene_system
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(xgc2_gazebo_scene_motion_test test/motion_controller_test.cpp)
  if(TARGET xgc2_gazebo_scene_motion_test)
    target_compile_features(xgc2_gazebo_scene_motion_test PUBLIC cxx_std_17)
    target_compile_options(xgc2_gazebo_scene_motion_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_include_directories(xgc2_gazebo_scene_motion_test PRIVATE include ${GAZEBO_INCLUDE_DIRS})
    target_link_libraries(xgc2_gazebo_scene_motion_test xgc2_gazebo_scene_motion)
  endif()
endif()
