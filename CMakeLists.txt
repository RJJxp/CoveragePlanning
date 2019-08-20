cmake_minimum_required(VERSION 2.8.3)
project(BowShapedPlanner)

add_compile_options(-std=c++11)

# set(Glog_DIR /usr/lib/cmake/ceres)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(Glog REQUIRED)

include_directories(
  include/
  /opt/ros/kinetic/include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  # ${GLOG_INCLUDE_DIR}
)

add_executable(planner
  src/main.cpp
  src/bow_shaped_planner.cpp
  src/decomposition.cpp
)
target_link_libraries(planner
  # glog
  ${OpenCV_LIBS}
)

