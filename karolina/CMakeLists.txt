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
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#target_link_libraries(gtk+-2.0)
#rosbuild_add_library(shadow src/libshadow/shadow_base.c src/libshadow/shadow_io.c src/libshadow/shadow_commands.c)
rosbuild_add_library(labjack ../labjack/src/liblabjack/u6.c ../labjack/include/labjackusb.h ../labjack/include/u6.h)


#rosbuild_add_executable(LJ_node src/LJpub/LJ_pub.cpp src/LJpub/LJ_node.cpp src/U6/u6.c )
#rosbuild_add_link_flags(LJ_node -llabjackusb)
#rosbuild_add_executable(shadow_PID src/shadow_node/shadow_PID.cpp)
#target_link_libraries(shadow_PID shadow)
#rosbuild_add_executable(shadow_PID_main src/shadow_node/shadow_PID_main.cpp src/shadow_node/shadow_PID.cpp)
#target_link_libraries(shadow_PID_main shadow)
#rosbuild_add_executable(shadow_node_main src/shadow_node/shadow_node_main.cpp src/shadow_node/shadow_node.cpp)
#target_link_libraries(shadow_node_main shadow)
#rosbuild_add_executable(labjack_node_main src/shadow_node/labjack_node_main.cpp src/shadow_node/labjack_node.cpp)

# Use both the local file and the file installed on the system
#target_link_libraries(labjack_node_main shadow labjack labjackusb) 

#rosbuild_add_executable(glove_finger src/shadow_node/glove_finger.cpp src/shadow_node/shadow_node.cpp src/shadow_node/shadow_PID.cpp)
#target_link_libraries(glove_finger shadow)


rosbuild_add_executable(controlMain_sofia src/valve_control/controlMain_sofia.cpp  src/valve_control/control_sofia.cpp) # src/shadow_node/labjack_node.cpp) # src/shadow_node/shadow_node.cpp)
target_link_libraries(controlMain_sofia labjackusb labjack)
#target_link_libraries(controlMain_sofia shadow labjack labjackusb)



#rosbuild_add_executable(finger_pushing_cal src/finger_pushing/finger_pushing_cal.cpp) # src/shadow_node/shadow_node.cpp)
#target_link_libraries(finger_pushing_cal shadow)


#rosbuild_add_executable(shadow_FC src/shadow_node/shadow_FC.cpp)
#rosbuild_add_link_flags(shadow_FC -llabjackusb)
#target_link_libraries(shadow_FC shadow)
