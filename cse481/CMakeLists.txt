cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_add_executable(cluster_extraction src/cluster_extraction.cpp)
#rosbuild_add_executable(foo src/foo.cpp)
rosbuild_add_library(object_detector src/object_detector.cpp src/table_cluster_detector.cpp)
rosbuild_add_library(rpc communication/Rpc.cpp communication/rpc_constants.cpp communication/rpc_types.cpp)
#rosbuild_add_executable(object_detector src/main.cpp src/object_detector.cpp src/table_object_detector.cpp)
include_directories(communication)
rosbuild_add_executable(server src/server.cpp src/marker_generator.cpp)
target_link_libraries(server rpc object_detector)

#rosbuild_link_boost(object_detector filesystem)
#rosbuild_add_executable (table_object_detector src/table_object_detector.cpp)
#target_link_libraries(example ${PROJECT_NAME})
