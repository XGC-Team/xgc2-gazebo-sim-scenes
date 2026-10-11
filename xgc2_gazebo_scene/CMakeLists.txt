cmake_minimum_required(VERSION 3.16)
project(xgc2_gazebo_scene VERSION 0.1.0 LANGUAGES CXX)

# Clang does not enable the distribution compiler driver's as-needed default.
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--as-needed")

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  message_generation
  roscpp
  rosgraph_msgs
  std_msgs
  xgc2_geometry_msgs
)
find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(XgcXrpc 0.1.0 CONFIG REQUIRED COMPONENTS http)
find_package(XgcChassisHold CONFIG REQUIRED COMPONENTS core)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(JSONCPP REQUIRED IMPORTED_TARGET jsoncpp)

add_message_files(FILES
  ConvexPart.msg
  MotionSpec.msg
  ObstacleDefinition.msg
  ObstacleDefinitionArray.msg
  ObstacleState.msg
  ObstacleStateArray.msg
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
    xgc2_simulation_service
    xgc2_simulation_world
    xgc2_simulation_entity_ack
    xgc2_simulation_sensor_ack
    xgc2_simulation_ros_data
  CATKIN_DEPENDS gazebo_msgs geometry_msgs message_runtime roscpp rosgraph_msgs std_msgs xgc2_geometry_msgs
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
target_include_directories(xgc2_gazebo_scene_geometry PUBLIC include)
target_include_directories(xgc2_gazebo_scene_geometry SYSTEM PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(xgc2_gazebo_scene_geometry PUBLIC Eigen3::Eigen ${GAZEBO_LIBRARIES})

add_library(xgc2_gazebo_scene_motion
  src/motion_controller.cpp
)
target_compile_features(xgc2_gazebo_scene_motion PUBLIC cxx_std_17)
target_compile_options(xgc2_gazebo_scene_motion PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(xgc2_gazebo_scene_motion PUBLIC include)
target_include_directories(xgc2_gazebo_scene_motion SYSTEM PUBLIC ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
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
target_include_directories(xgc2_gazebo_scene_system PRIVATE include)
target_include_directories(xgc2_gazebo_scene_system SYSTEM PRIVATE ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
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
target_include_directories(xgc2_scene_model PUBLIC include)
target_include_directories(xgc2_scene_model SYSTEM PUBLIC ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
add_dependencies(xgc2_scene_model ${catkin_EXPORTED_TARGETS})
target_link_libraries(xgc2_scene_model PUBLIC xgc2_gazebo_scene_geometry ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(xgc2_simulation_service SHARED
  src/simulation_service.cpp
  src/chassis_hold.cpp
  src/chassis_command_queue.cpp
  src/native_scene_controller.cpp
  src/native_entity_motion.cpp
  src/native_sensor_controller.cpp
)
target_compile_features(xgc2_simulation_service PUBLIC cxx_std_20)
target_compile_options(xgc2_simulation_service PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(xgc2_simulation_service PUBLIC include)
target_include_directories(xgc2_simulation_service SYSTEM PUBLIC ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
add_dependencies(xgc2_simulation_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(xgc2_simulation_service PUBLIC xgc2_scene_model xgc2_gazebo_scene_motion
  PkgConfig::JSONCPP ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
  PRIVATE XgcChassisHold::core XgcXrpc::http)
foreach(plugin simulation_world simulation_entity_ack simulation_sensor_ack)
  add_library(xgc2_${plugin} SHARED src/${plugin}_plugin.cpp)
  target_compile_features(xgc2_${plugin} PUBLIC cxx_std_20)
  target_link_libraries(xgc2_${plugin} PRIVATE xgc2_simulation_service)
endforeach()
add_library(xgc2_simulation_ros_data SHARED src/simulation_ros_data_plugin.cpp)
target_compile_features(xgc2_simulation_ros_data PUBLIC cxx_std_20)
target_compile_options(xgc2_simulation_ros_data PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(xgc2_simulation_ros_data PRIVATE include)
target_include_directories(xgc2_simulation_ros_data SYSTEM PRIVATE ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
add_dependencies(xgc2_simulation_ros_data ${catkin_EXPORTED_TARGETS})
target_link_libraries(xgc2_simulation_ros_data PRIVATE Threads::Threads Eigen3::Eigen ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS
  obstaclePathPlugin
  xgc2_gazebo_scene_contact
  xgc2_gazebo_scene_geometry
  xgc2_gazebo_scene_motion
  xgc2_gazebo_scene_system
  xgc2_scene_model
  xgc2_simulation_service
  xgc2_simulation_world
  xgc2_simulation_entity_ack
  xgc2_simulation_sensor_ack
  xgc2_simulation_ros_data
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/spawn_robot_model scripts/simulation_world_prepare scripts/delete_robot_model
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
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

  catkin_add_gtest(xgc2_gazebo_scene_model_snapshot_test test/model_snapshot_test.cpp)
  if(TARGET xgc2_gazebo_scene_model_snapshot_test)
    target_compile_features(xgc2_gazebo_scene_model_snapshot_test PUBLIC cxx_std_17)
    target_compile_options(xgc2_gazebo_scene_model_snapshot_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_include_directories(xgc2_gazebo_scene_model_snapshot_test PRIVATE include)
  endif()

  catkin_add_gtest(xgc2_gazebo_scene_obstacle_messages_test test/obstacle_messages_test.cpp)
  if(TARGET xgc2_gazebo_scene_obstacle_messages_test)
    target_compile_features(xgc2_gazebo_scene_obstacle_messages_test PUBLIC cxx_std_17)
    target_compile_options(xgc2_gazebo_scene_obstacle_messages_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_include_directories(xgc2_gazebo_scene_obstacle_messages_test
      PRIVATE include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
    add_dependencies(xgc2_gazebo_scene_obstacle_messages_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(xgc2_gazebo_scene_obstacle_messages_test xgc2_gazebo_scene_motion)
  endif()

  catkin_add_gtest(xgc2_gazebo_scene_sdf_parser_test test/sdf_parser_test.cpp)
  if(TARGET xgc2_gazebo_scene_sdf_parser_test)
    target_compile_features(xgc2_gazebo_scene_sdf_parser_test PUBLIC cxx_std_17)
    target_compile_options(xgc2_gazebo_scene_sdf_parser_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_include_directories(xgc2_gazebo_scene_sdf_parser_test PRIVATE include ${GAZEBO_INCLUDE_DIRS})
    target_link_libraries(xgc2_gazebo_scene_sdf_parser_test ${GAZEBO_LIBRARIES})
  endif()

  catkin_add_gtest(xgc2_gazebo_scene_model_index_test test/model_index_test.cpp)
  if(TARGET xgc2_gazebo_scene_model_index_test)
    target_compile_features(xgc2_gazebo_scene_model_index_test PUBLIC cxx_std_17)
    target_compile_options(xgc2_gazebo_scene_model_index_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_include_directories(xgc2_gazebo_scene_model_index_test PRIVATE include)
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
