cmake_minimum_required(VERSION 2.8.3)
project(a3_simulator)

# Importing the necessary external packages
find_package( Threads REQUIRED )
find_package( OpenCV REQUIRED )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

###################################################################################################################
#Please leave this section as it will attepmt to determine your version of Ubuntu and link to correct library

execute_process(COMMAND lsb_release -cs
    OUTPUT_VARIABLE RELEASE_CODENAME
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

message ( STATUS "System in use is ${RELEASE_CODENAME}" )

if("${RELEASE_CODENAME}" STREQUAL "xenial")
    message ( STATUS "Xenial 16.04 in use" )
elseif("${RELEASE_CODENAME}" STREQUAL "bionic")
    message ( STATUS "Bionic 18.04 in use " )
else("${RELEASE_CODENAME}" STREQUAL "xenial")
    message ( STATUS "Your vesrion on Ubuntu ${RELEASE_CODENAME} is not supported" )
endif("${RELEASE_CODENAME}" STREQUAL "xenial")

set(SIMULATOR_LIB_HOME ${CMAKE_SOURCE_DIR}/dep/${RELEASE_CODENAME})

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${SIMULATOR_LIB_HOME}
)

message ( STATUS "Looking for libray in ${SIMULATOR_LIB_HOME}" )
link_directories(${SIMULATOR_LIB_HOME})

####################################################################################################################


# Here we declare a C++ executable with only one cpp forming part of it, you will need to change to cater for more files.
add_executable(${PROJECT_NAME}_example main.cpp)

# Link the library
target_link_libraries(${PROJECT_NAME}_example ${OpenCV_LIBS} simulator)
