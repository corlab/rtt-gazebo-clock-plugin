cmake_minimum_required(VERSION 2.8.3)
project(rtt_gazebo_system)

# !!!!! export PKG_CONFIG_PATH=

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba rtt-typekit)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos-RTT. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()
message(STATUS "OROCOS-RTT_RTT-SCRIPTING_LIBRARY: ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY}")
#find_package(OROCOS-OCL REQUIRED COMPONENTS ocl-deployment ocl-taskbrowser ocl-logging)

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package(gazebo 6.4.0 REQUIRED)
message(STATUS "GAZEBO version: ${GAZEBO_VERSION}") 
#message(STATUS "GAZEBO_INCLUDE_DIRS: ${GAZEBO_INCLUDE_DIRS}")
#message(STATUS "GAZEBO_LIBRARIES: ${GAZEBO_LIBRARIES}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


set ( CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/eigen/  )
find_package(Eigen REQUIRED)

find_package(Boost COMPONENTS thread REQUIRED)

find_package(Protobuf REQUIRED)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${PROTUBUF_LIBRARY_DIRS}
  )

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIR}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
  )

orocos_use_package( ocl-taskbrowser REQUIRED )
orocos_use_package( ocl-deployment REQUIRED )
orocos_use_package( ocl-logging REQUIRED )
orocos_use_package( orocos-rtt-corba REQUIRED )
#orocos_use_package( rtt-typekit REQUIRED )

## Gazebo RTT plugin
add_definitions(-DRTT_COMPONENT)
add_definitions(${OROCOS-RTT_DEFINITIONS})
add_library(${PROJECT_NAME} SHARED
  src/rtt_clock.cpp
  src/rtt_clock.h
  src/rtt_clock_sim_clock_thread.cpp
  src/rtt_clock_sim_clock_thread.h
  src/rtt_system_plugin.cpp
  src/rtt_system_plugin.h)
#set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS -DRTT_STATIC)

target_link_libraries(${PROJECT_NAME}
  ${GAZEBO_LIBRARIES}
  ${OROCOS-RTT_LIBRARIES} 
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} 
  ${OROCOS-RTT_RTT-TRANSPORT-CORBA_LIBRARY}
  ${OROCOS-RTT_TYPEKIT_LIBRARIES}
  ${USE_OROCOS_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROTUBUF_LIBRARIES} 
)

#set(CMAKE_BUILD_TYPE Debug)

orocos_generate_package()
