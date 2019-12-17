cmake_minimum_required(VERSION 2.8.3)
project(web_backend)

## Compile as C++11, supported in ROS Kinetic and newer
SET(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -std=c++11 -w   -fpermissive -msse4.2")

find_package(catkin REQUIRED
  roscpp
  autodrive_msgs
  std_msgs
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
   CATKIN_DEPENDS message_runtime    
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})  

add_executable(web_backend 
               src/web_utils.cpp
               src/msg_callback.cpp
               src/point.cpp
               src/main.cpp
)

add_dependencies(web_backend ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(web_backend
  ${catkin_LIBRARIES}
  ${${PROJECT_NAME}_SOURCE_DIR}/third/libPocoUtil.a
  ${${PROJECT_NAME}_SOURCE_DIR}/third/libPocoNet.a
  ${${PROJECT_NAME}_SOURCE_DIR}/third/libPocoXML.a
  ${${PROJECT_NAME}_SOURCE_DIR}/third/libPocoJSON.a
  ${${PROJECT_NAME}_SOURCE_DIR}/third/libPocoFoundation.a
  pthread
)

