cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Coverage)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#common commands for building c++ executables and libraries
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_library(${PROJECT_NAME} src/probabilistic_grasp_planner.cpp
                                     src/recognition_probability_computer.cpp
                                     src/grasp_success_probability_computer.cpp
                                     src/grasp_retriever.cpp
                                     src/distribution_evaluator.cpp
                                     src/grasp_regression_evaluator.cpp)

#rosbuild_add_openmp_flags(${PROJECT_NAME})
rosbuild_add_executable(probabilistic_grasp_planner_node nodes/probabilistic_grasp_planner_node.cpp)
target_link_libraries(probabilistic_grasp_planner_node ${PROJECT_NAME})

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/test)

rosbuild_add_gtest(planner_tests test/distribution_tests.cpp)
target_link_libraries(planner_tests ${PROJECT_NAME})
