cmake_minimum_required(VERSION 3.0.2)
project(wholebodycontact_locomotion_planner_sample)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  wholebodycontact_locomotion_planner
  choreonoid_viewer
  choreonoid_qhull
  roslib
  )

find_package(choreonoid REQUIRED)

catkin_package(
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CHOREONOID_INCLUDE_DIRS}
)

add_library(CnoidWholeBodyContactLocomotionPlannerSamplePlugin SHARED
  src/CnoidWholeBodyContactLocomotionPlannerSamplePlugin.cpp
  src/samplerobot_common.cpp
  src/world_common.cpp
  src/sample0_display.cpp
  )
target_link_libraries(CnoidWholeBodyContactLocomotionPlannerSamplePlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_PLUGIN_LIBRARIES}
  )
set_target_properties(CnoidWholeBodyContactLocomotionPlannerSamplePlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
