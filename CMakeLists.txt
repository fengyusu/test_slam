cmake_minimum_required(VERSION 2.8.3)
project(test_slam)

set(CMAKE_CXX_STANDARD 14)
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf
)

set(Eigen3_DIR /usr/local/share/eigen3)
find_package( Eigen3 REQUIRED )
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )
find_package( Ceres REQUIRED )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES csm

  CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
  ${G2O_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${CSM_INCLUDE_DIR}
  /opt/ros/kinetic/include/csm
)

add_subdirectory(src/g2o_slam2d)


add_executable(test_slam_node
        src/main.cpp
        src/test_slam_node.cpp
        src/param_config.h
        src/scan_matcher.cpp src/scan_matcher.hpp
        src/laser_data_process.cpp src/laser_data_process.hpp
        src/pose_graph_interface.cpp src/pose_graph_interface.hpp
        src/common.hpp src/common.cpp
        src/pose_graph_custom.cpp src/pose_graph_custom.hpp
        src/pose_graph_g2o.cpp src/pose_graph_g2o.hpp
        src/pose_graph_ceres.cpp src/pose_graph_ceres.hpp)

add_dependencies(test_slam_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(test_slam_node
   ${catkin_LIBRARIES}
   /opt/ros/kinetic/lib/libcsm.so
   ${CSPARSE_LIBRARY}
   ${CERES_LIBRARIES}
   g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
   g2o_slam2d_library
 )

