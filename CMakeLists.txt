cmake_minimum_required(VERSION 2.8.3)
project(RTTGazeboClockPlugin)

# Inspired by Pouya's solution for https://github.com/corlab/rtt-gazebo-embedded fa8b9b6
if (CMAKE_SYSTEM_NAME MATCHES "Darwin")
    add_definitions(-DBOOST_DATE_TIME_POSIX_TIME_STD_CONFIG)
    # Apparently this is needed as well
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-c++11-narrowing")
endif()

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-typekit)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos-RTT. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()
message(STATUS "OROCOS-RTT_RTT-SCRIPTING_LIBRARY: ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY}")
#find_package(OROCOS-OCL REQUIRED COMPONENTS ocl-deployment ocl-taskbrowser ocl-logging)

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package(gazebo REQUIRED)
message(STATUS "GAZEBO version: ${GAZEBO_VERSION}") 
#message(STATUS "GAZEBO_INCLUDE_DIRS: ${GAZEBO_INCLUDE_DIRS}")
#message(STATUS "GAZEBO_LIBRARIES: ${GAZEBO_LIBRARIES}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


#set ( CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/eigen/  )
#find_package(Eigen REQUIRED)

find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})
include_directories(${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

find_package(Boost COMPONENTS thread REQUIRED)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  )

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
  )

orocos_use_package( ocl-taskbrowser REQUIRED )
orocos_use_package( ocl-deployment REQUIRED )
orocos_use_package( ocl-logging REQUIRED )

## Gazebo RTT plugin
add_definitions(-DRTT_COMPONENT)
add_definitions(${OROCOS-RTT_DEFINITIONS})
add_library(${PROJECT_NAME} SHARED
  src/rtt_clock.cpp
  src/rtt_clock.h
  src/rtt_clock_sim_clock_thread.cpp
  src/rtt_clock_sim_clock_thread.h
  src/rtt-gazebo-clock-plugin.cpp
  src/rtt-gazebo-clock-plugin.h)
#set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS -DRTT_STATIC)

target_link_libraries(${PROJECT_NAME}
  ${GAZEBO_LIBRARIES}
  ${OROCOS-RTT_LIBRARIES} 
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY}
  ${OROCOS-RTT_TYPEKIT_LIBRARIES}
  ${USE_OROCOS_LIBRARIES}
  ${Boost_LIBRARIES}
)

#set(CMAKE_BUILD_TYPE Debug)

orocos_generate_package()

install(TARGETS ${PROJECT_NAME}
     LIBRARY DESTINATION "lib/orocos/gnulinux/${PROJECT_NAME}" COMPONENT libraries)
