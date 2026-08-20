cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sim_worlds)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(
    test/verify_formation_scenes.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  catkin_add_nosetests(
    test/verify_calibration_targets.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()
