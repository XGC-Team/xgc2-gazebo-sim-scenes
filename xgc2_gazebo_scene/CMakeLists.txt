cmake_minimum_required(VERSION 3.16)
project(xgc2_gazebo_scene VERSION 0.1.0 LANGUAGES CXX)

# Clang does not enable the distribution compiler driver's as-needed default.
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--as-needed")

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  roscpp
  rosgraph_msgs
  std_msgs
  xgc2_geometry_msgs
)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Eigen3 REQUIRED)
find_package(xgc2_math CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(XgcXrpc 0.2.0 CONFIG REQUIRED COMPONENTS http)
find_package(XgcChassisHold 0.2.0 CONFIG REQUIRED COMPONENTS http)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(JSONCPP REQUIRED IMPORTED_TARGET jsoncpp)

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
    xgc2_simulation_ros_master
  CATKIN_DEPENDS gazebo_msgs geometry_msgs roscpp rosgraph_msgs std_msgs xgc2_geometry_msgs
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
add_dependencies(xgc2_gazebo_scene_system ${catkin_EXPORTED_TARGETS})
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
pkg_check_modules(PUGIXML REQUIRED IMPORTED_TARGET pugixml)
target_link_libraries(xgc2_scene_model PUBLIC xgc2_math::scene xgc2_gazebo_scene_geometry PkgConfig::PUGIXML ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(xgc2_simulation_ros_master SHARED src/ros_master_registration.cpp)
target_compile_features(xgc2_simulation_ros_master PUBLIC cxx_std_17)
target_compile_options(xgc2_simulation_ros_master PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(xgc2_simulation_ros_master PUBLIC include)
target_include_directories(xgc2_simulation_ros_master SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(xgc2_simulation_ros_master PRIVATE Threads::Threads ${catkin_LIBRARIES})
target_link_libraries(xgc2_gazebo_scene_system PRIVATE xgc2_simulation_ros_master)

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
  PkgConfig::JSONCPP nlohmann_json::nlohmann_json ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
  PRIVATE XgcChassisHold::http XgcXrpc::http xgc2_simulation_ros_master)
if(DEFINED ENV{XGC_GEOMETRY_MSGS_PREFIX} AND NOT "$ENV{XGC_GEOMETRY_MSGS_PREFIX}" STREQUAL "")
  foreach(xgc_geometry_target xgc2_scene_model xgc2_simulation_service)
    target_include_directories(${xgc_geometry_target} BEFORE PUBLIC "$ENV{XGC_GEOMETRY_MSGS_PREFIX}/include")
  endforeach()
endif()
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
target_link_libraries(xgc2_simulation_ros_data PRIVATE xgc2_simulation_ros_master Threads::Threads Eigen3::Eigen ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

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
  xgc2_simulation_ros_master
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY python/xgc2_gazebo_scene DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}
  PATTERN "__pycache__" EXCLUDE)
catkin_install_python(PROGRAMS scripts/spawn_robot_model
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  add_test(NAME product_robot_placement COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_robot_placement.py)
  find_package(rostest REQUIRED)
  # Give the native ACK fixtures the service's public compile requirements.
  foreach(fixture create_ack sensor_ack)
    add_library(xgc2_${fixture}_fixture SHARED test/native_simulation/${fixture}_fixture.cpp)
    target_link_libraries(xgc2_${fixture}_fixture PRIVATE xgc2_simulation_service)
  endforeach()
  catkin_install_python(PROGRAMS
    test/physical_contact_ground_truth_test.py
    test/scene_adapter_gazebo_integration.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  add_rostest(test/physical_contact_ground_truth.test)

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

  catkin_add_gtest(xgc2_gazebo_scene_sdf_parser_test test/sdf_parser_test.cpp)
  if(TARGET xgc2_gazebo_scene_sdf_parser_test)
    target_compile_features(xgc2_gazebo_scene_sdf_parser_test PUBLIC cxx_std_17)
    target_compile_options(xgc2_gazebo_scene_sdf_parser_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_include_directories(xgc2_gazebo_scene_sdf_parser_test PRIVATE include ${GAZEBO_INCLUDE_DIRS})
    target_link_libraries(xgc2_gazebo_scene_sdf_parser_test ${GAZEBO_LIBRARIES})
  endif()

  catkin_add_gtest(xgc2_gazebo_scene_native_logging_test test/native_runtime_logging_test.cpp)
  if(TARGET xgc2_gazebo_scene_native_logging_test)
    target_compile_features(xgc2_gazebo_scene_native_logging_test PUBLIC cxx_std_17)
    target_compile_options(xgc2_gazebo_scene_native_logging_test PRIVATE -Wall -Wextra -Wpedantic)
    target_include_directories(xgc2_gazebo_scene_native_logging_test PRIVATE include ${GAZEBO_INCLUDE_DIRS})
    target_link_libraries(xgc2_gazebo_scene_native_logging_test ${GAZEBO_LIBRARIES} Boost::filesystem Boost::system)
  endif()

  catkin_add_gtest(xgc2_gazebo_scene_model_index_test test/model_index_test.cpp)
  if(TARGET xgc2_gazebo_scene_model_index_test)
    target_compile_features(xgc2_gazebo_scene_model_index_test PUBLIC cxx_std_17)
    target_compile_options(xgc2_gazebo_scene_model_index_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_include_directories(xgc2_gazebo_scene_model_index_test PRIVATE include)
  endif()

  catkin_add_gtest(xgc2_gazebo_scene_resource_sdf_test test/resource_sdf_test.cpp)
  if(TARGET xgc2_gazebo_scene_resource_sdf_test)
    find_library(XGC2_RESOURCE_TEST_ODE ode REQUIRED)
    target_compile_features(xgc2_gazebo_scene_resource_sdf_test PUBLIC cxx_std_17)
    target_compile_options(xgc2_gazebo_scene_resource_sdf_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_link_libraries(xgc2_gazebo_scene_resource_sdf_test xgc2_simulation_service ${XGC2_RESOURCE_TEST_ODE} gtest_main)
  endif()

  catkin_add_gtest(xgc2_gazebo_scene_ground_slot_test test/ground_slot_test.cpp)
  if(TARGET xgc2_gazebo_scene_ground_slot_test)
    find_library(XGC2_GROUND_TEST_ODE ode REQUIRED)
    target_compile_features(xgc2_gazebo_scene_ground_slot_test PUBLIC cxx_std_17)
    target_compile_options(xgc2_gazebo_scene_ground_slot_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_link_libraries(xgc2_gazebo_scene_ground_slot_test xgc2_simulation_service ${XGC2_GROUND_TEST_ODE} gtest_main)
  endif()

  catkin_add_gtest(xgc2_gazebo_scene_ring_test test/ring_mesh_geometry_test.cpp)
  if(TARGET xgc2_gazebo_scene_ring_test)
    find_library(XGC2_RING_TEST_ODE ode REQUIRED)
    target_compile_features(xgc2_gazebo_scene_ring_test PUBLIC cxx_std_17)
    target_compile_options(xgc2_gazebo_scene_ring_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_link_libraries(xgc2_gazebo_scene_ring_test xgc2_scene_model ${XGC2_RING_TEST_ODE} gtest_main)
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
