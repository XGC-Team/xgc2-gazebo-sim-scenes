cmake_minimum_required(VERSION 3.16)
project(gazebo_sim_worlds)

find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/native_world_start scripts/native_obstacle.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_native_world_start.py)
  catkin_add_nosetests(
    test/verify_formation_scenes.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  catkin_add_nosetests(
    test/verify_calibration_targets.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()
