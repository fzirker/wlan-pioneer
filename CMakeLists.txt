cmake_minimum_required(VERSION 2.8.3)
project(wlan_pioneer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(FILES WlanSignalMsg.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} /usr/local/include/)
link_directories(${catkin_INCLUDE_DIRS} /usr/local/lib/)

add_executable(wlanSignal src/wlanSignal.cpp)
target_link_libraries(wlanSignal ${catkin_LIBRARIES} wifi-scan)
add_dependencies(wlanSignal wlan_pioneer_generate_messages_cpp)


# This setting disable new dtags for ld linker. So RUNPATH is
# replaced with RPATH in wlanSignal executable. We need RPATH
# for finding linked libraries at runtime when executed with
# sudo. WLAN scanning needs enhanced privileges. With RUNPATH
# linker disables LD_LIBRARY_PATH for security reasons if
# setuid/setgid is set. So this is a workaround.
# https://gitlab.kitware.com/cmake/community/wikis/doc/cmake/RPATH-handling
# https://answers.ros.org/question/165246/launch-node-with-root-permissions/
SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--disable-new-dtags")
