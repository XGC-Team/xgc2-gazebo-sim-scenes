cmake_minimum_required(VERSION 3.10)
project(xgc2_gazebo_scene VERSION 0.1.0 LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  xgc2_geometry_msgs
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
  LIBRARIES
    obstaclePathPlugin
    xgc2_gazebo_scene_contact
    xgc2_gazebo_scene_geometry
    xgc2_gazebo_scene_motion
    xgc2_gazebo_scene_system
    xgc2_scene_model
    xgc2_scene_authoring_world
  CATKIN_DEPENDS geometry_msgs message_runtime roscpp std_msgs xgc2_geometry_msgs
)

add_library(xgc2_gazebo_scene_contact
  src/physical_contact_filter.cpp
)
target_compile_features(xgc2_gazebo_scene_contact PUBLIC cxx_std_17)
target_compile_options(xgc2_gazebo_scene_contact PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(xgc2_gazebo_scene_contact PUBLIC include)

add_library(xgc2_gazebo_scene_geometry
  src/convex_mesh_geometry.cpp
)
target_compile_features(xgc2_gazebo_scene_geometry PUBLIC cxx_std_17)
target_compile_options(xgc2_gazebo_scene_geometry PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(xgc2_gazebo_scene_geometry
  PUBLIC include ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(xgc2_gazebo_scene_geometry PUBLIC ${GAZEBO_LIBRARIES})

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
  PRIVATE
    xgc2_gazebo_scene_contact
    xgc2_gazebo_scene_geometry
    xgc2_gazebo_scene_motion
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)

add_library(xgc2_scene_model src/scene_model.cpp)
target_compile_features(xgc2_scene_model PUBLIC cxx_std_17)
target_compile_options(xgc2_scene_model PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(xgc2_scene_model PUBLIC include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
add_dependencies(xgc2_scene_model ${catkin_EXPORTED_TARGETS})
target_link_libraries(xgc2_scene_model PUBLIC xgc2_gazebo_scene_geometry ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(xgc2_scene_authoring_world SHARED src/scene_authoring_world_plugin.cpp)
target_compile_features(xgc2_scene_authoring_world PUBLIC cxx_std_17)
target_compile_options(xgc2_scene_authoring_world PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(xgc2_scene_authoring_world PRIVATE include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
add_dependencies(xgc2_scene_authoring_world ${catkin_EXPORTED_TARGETS})
target_link_libraries(xgc2_scene_authoring_world PRIVATE xgc2_scene_model ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS
  obstaclePathPlugin
  xgc2_gazebo_scene_contact
  xgc2_gazebo_scene_geometry
  xgc2_gazebo_scene_motion
  xgc2_gazebo_scene_system
  xgc2_scene_model
  xgc2_scene_authoring_world
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(xgc2_scene_authoring_test test/scene_authoring.test test/scene_authoring_test.cpp)
  if(TARGET xgc2_scene_authoring_test)
    target_compile_features(xgc2_scene_authoring_test PUBLIC cxx_std_17)
    target_compile_options(xgc2_scene_authoring_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_definitions(xgc2_scene_authoring_test
      PRIVATE XGC2_SCENE_AUTHORING_TEST_WORLD="${CMAKE_CURRENT_SOURCE_DIR}/test/scene_authoring.world")
    target_include_directories(xgc2_scene_authoring_test PRIVATE include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
    add_dependencies(xgc2_scene_authoring_test xgc2_scene_authoring_world ${catkin_EXPORTED_TARGETS})
    target_link_libraries(xgc2_scene_authoring_test xgc2_scene_model ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
  endif()
  catkin_install_python(PROGRAMS
    test/physical_contact_ground_truth_test.py
    test/scene_ground_truth_test.py
    test/scene_runtime_gazebo_integration.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  add_rostest(test/physical_contact_ground_truth.test)
  add_rostest(test/uav6_knot_ground_truth.test)
  add_rostest(test/ugv4_figure_eight_ground_truth.test)

  catkin_add_gtest(xgc2_gazebo_scene_contact_test test/physical_contact_filter_test.cpp)
  if(TARGET xgc2_gazebo_scene_contact_test)
    target_compile_features(xgc2_gazebo_scene_contact_test PUBLIC cxx_std_17)
    target_compile_options(xgc2_gazebo_scene_contact_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_include_directories(xgc2_gazebo_scene_contact_test PRIVATE include)
    target_link_libraries(xgc2_gazebo_scene_contact_test xgc2_gazebo_scene_contact)
  endif()

  catkin_add_gtest(xgc2_gazebo_scene_geometry_test test/convex_mesh_geometry_test.cpp)
  if(TARGET xgc2_gazebo_scene_geometry_test)
    target_compile_features(xgc2_gazebo_scene_geometry_test PUBLIC cxx_std_17)
    target_compile_options(xgc2_gazebo_scene_geometry_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_definitions(xgc2_gazebo_scene_geometry_test
      PRIVATE XGC2_SCENE_TEST_MODEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../gazebo_sim_worlds/models"
    )
    target_include_directories(xgc2_gazebo_scene_geometry_test PRIVATE include ${GAZEBO_INCLUDE_DIRS})
    target_link_libraries(xgc2_gazebo_scene_geometry_test xgc2_gazebo_scene_geometry)
  endif()

  catkin_add_gtest(xgc2_gazebo_scene_motion_test test/motion_controller_test.cpp)
  if(TARGET xgc2_gazebo_scene_motion_test)
    target_compile_features(xgc2_gazebo_scene_motion_test PUBLIC cxx_std_17)
    target_compile_options(xgc2_gazebo_scene_motion_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_include_directories(xgc2_gazebo_scene_motion_test PRIVATE include ${GAZEBO_INCLUDE_DIRS})
    target_link_libraries(xgc2_gazebo_scene_motion_test xgc2_gazebo_scene_motion)
  endif()
endif()
