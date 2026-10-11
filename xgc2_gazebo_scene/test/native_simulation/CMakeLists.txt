cmake_minimum_required(VERSION 3.16)
project(native_simulation_fixture LANGUAGES CXX)
find_package(XgcXrpc 0.2.0 CONFIG REQUIRED COMPONENTS http)
find_package(XgcChassisHold 0.2.0 CONFIG REQUIRED COMPONENTS http)
find_package(PkgConfig REQUIRED)
pkg_check_modules(Gazebo REQUIRED IMPORTED_TARGET gazebo)
get_target_property(gazebo_options PkgConfig::Gazebo INTERFACE_COMPILE_OPTIONS)
list(FILTER gazebo_options EXCLUDE REGEX "^-std=")
set_property(TARGET PkgConfig::Gazebo PROPERTY INTERFACE_COMPILE_OPTIONS "${gazebo_options}")
pkg_check_modules(JsonCpp REQUIRED IMPORTED_TARGET jsoncpp)
find_package(Eigen3 REQUIRED)
set(scene_dir "${CMAKE_CURRENT_SOURCE_DIR}/../..")
find_package(Boost REQUIRED COMPONENTS thread system filesystem)
set(GEOMETRY_MESSAGES_INCLUDE "" CACHE PATH "Generated product geometry messages")
if(NOT GEOMETRY_MESSAGES_INCLUDE)
  message(FATAL_ERROR "GEOMETRY_MESSAGES_INCLUDE is required")
endif()
add_library(xgc2_simulation_service SHARED
  "${scene_dir}/src/simulation_service.cpp"
  "${scene_dir}/src/chassis_hold.cpp"
  "${scene_dir}/src/chassis_command_queue.cpp"
  "${scene_dir}/src/native_sensor_controller.cpp"
  "${scene_dir}/src/native_scene_controller.cpp"
  "${scene_dir}/src/native_entity_motion.cpp"
  "${scene_dir}/src/motion_controller.cpp"
  "${scene_dir}/src/scene_model.cpp"
  "${scene_dir}/src/convex_mesh_geometry.cpp")
target_compile_features(xgc2_simulation_service PUBLIC cxx_std_20)
target_include_directories(xgc2_simulation_service PUBLIC "${scene_dir}/include")
target_include_directories(xgc2_simulation_service SYSTEM PUBLIC "${GEOMETRY_MESSAGES_INCLUDE}" "/opt/ros/noetic/include")
target_link_libraries(xgc2_simulation_service PUBLIC XgcChassisHold::http XgcXrpc::http PkgConfig::JsonCpp PkgConfig::Gazebo Eigen3::Eigen Boost::thread Boost::filesystem
  /opt/ros/noetic/lib/libroscpp.so /opt/ros/noetic/lib/librostime.so /opt/ros/noetic/lib/librosconsole.so)
foreach(plugin world entity_ack sensor_ack)
  add_library(xgc2_simulation_${plugin} SHARED "${scene_dir}/src/simulation_${plugin}_plugin.cpp")
  target_compile_features(xgc2_simulation_${plugin} PRIVATE cxx_std_20)
  target_link_libraries(xgc2_simulation_${plugin} PRIVATE xgc2_simulation_service)
endforeach()
add_library(xgc2_simulation_ros_data SHARED "${scene_dir}/src/simulation_ros_data_plugin.cpp")
set(GAZEBO_MESSAGES_INCLUDE "" CACHE PATH "Generated gazebo_msgs headers")
if(NOT GAZEBO_MESSAGES_INCLUDE)
  message(FATAL_ERROR "GAZEBO_MESSAGES_INCLUDE is required")
endif()
target_include_directories(xgc2_simulation_ros_data SYSTEM PRIVATE "${GAZEBO_MESSAGES_INCLUDE}")
target_compile_features(xgc2_simulation_ros_data PRIVATE cxx_std_20)
target_link_libraries(xgc2_simulation_ros_data PRIVATE xgc2_simulation_service)
add_library(xgc2_chassis_fixture SHARED chassis_fixture.cpp)
target_compile_features(xgc2_chassis_fixture PRIVATE cxx_std_20)
target_link_libraries(xgc2_chassis_fixture PRIVATE xgc2_simulation_service)
