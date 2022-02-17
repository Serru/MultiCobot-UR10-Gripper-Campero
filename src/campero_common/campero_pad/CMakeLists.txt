cmake_minimum_required(VERSION 2.8.3)
project(campero_pad)

find_package(catkin REQUIRED COMPONENTS 
    message_generation
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs 
    diagnostic_updater
    )
    
add_service_files(DIRECTORY srv 
   FILES
   enable_disable_pad.srv
)

generate_messages(
   DEPENDENCIES
   robotnik_msgs sensor_msgs campero_pad
)

catkin_package(
  CATKIN_DEPENDS 
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs 
    message_runtime
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
)

add_executable(campero_pad src/campero_pad.cpp)

add_dependencies(campero_pad campero_pad_generate_messages_cpp)

target_link_libraries(campero_pad
   ${catkin_LIBRARIES}
)

install(
  TARGETS campero_pad
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
