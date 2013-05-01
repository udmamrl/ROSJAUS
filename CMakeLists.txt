cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

link_directories("${PROJECT_SOURCE_DIR}/lib")
link_directories("${PROJECT_SOURCE_DIR}/include")
#rosbuild_add_executable( ROSJausDemo src/ROSJausDemo.cpp)
#target_link_libraries(   ROSJausDemo Junior rt)

rosbuild_add_executable( WaypointCOP src/WaypointCOP.cpp)
target_link_libraries(   WaypointCOP Junior rt)

rosbuild_add_executable( ROSJAUS src/ROSJAUS.cpp)
target_link_libraries(   ROSJAUS Junior rt)

#rosbuild_add_executable( sender src/sender.cpp)
#target_link_libraries(   sender Junior rt)

#rosbuild_add_executable( receiver src/receiver.cpp)
#target_link_libraries(   receiver Junior rt)

#rosbuild_add_executable( ROSJAUS_simple src/ROSJAUS_simple.cpp)
#target_link_libraries(   ROSJAUS_simple Junior rt)


