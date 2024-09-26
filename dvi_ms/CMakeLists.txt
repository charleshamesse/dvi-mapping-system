cmake_minimum_required(VERSION 2.8.3)
project(dvi_ms)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

add_compile_options(-std=c++14)
set( CMAKE_CXX_FLAGS "-std=c++14 -O3 -g" )


# add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("# processors:  ${N}")
  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message("# cores for MP:  4")
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message("# cores for MP:  ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  cv_bridge
  rospy
  std_msgs
  pcl_ros
  tf
  message_generation
  eigen_conversions
)

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(realsense2 REQUIRED)
# find_package(Ceres REQUIRED)
find_package(OpenCV)

message(Eigen: ${EIGEN3_INCLUDE_DIR})
message(PCL: ${PCL_INCLUDE_DIRS})

add_message_files(
  FILES
  Pose6D.msg
  States.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include
)


add_executable(dvi_ms 
  src/main.cpp  
  src/state.cpp 
  src/imu_processing.cpp 
  src/lidar_processing.cpp 
  src/vio_processing.cpp 
  src/common_lib.cpp 
  src/vilain_publisher.cpp 
  src/parameters.cpp 
  src/transformer.cpp
  src/degeneracy_handler.cpp
)

target_link_libraries(dvi_ms ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${realsense2_LIBRARY})


