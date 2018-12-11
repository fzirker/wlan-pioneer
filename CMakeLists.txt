cmake_minimum_required(VERSION 2.8.3)
project(wlan_pioneer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  WlanSignalMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



add_executable(wlanSignal src/wlanSignal.cpp)
target_link_libraries(wlanSignal ${catkin_LIBRARIES})
add_dependencies(wlanSignal wlan_pioneer_generate_messages_cpp)