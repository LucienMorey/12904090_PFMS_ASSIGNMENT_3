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

#########################################################################
#########################################################################
# This section does not need to be edited in any way
# If you want to use the google test, simply copy this section below
# In your code and copy the CMakeLists.txt.in file which is in this directory
#
# The code will download and unpack googletest at configure time
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                 ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()
#########################################################################
#########################################################################


add_library(tracking pure_pursuit.cpp estimator.cpp data_updater.cpp planner.cpp time_planner.cpp controller.cpp)
set_target_properties(tracking PROPERTIES PUBLIC_HEADER "path_tracker.h; pure_pursuit.h; estimator.h; data_updater.h; planner.h;time_planner.h; controller.h")
target_link_libraries(tracking ${OpenCV_LIBS} simulator)

# Here we declare a C++ executable with only one cpp forming part of it, you will need to change to cater for more files.
add_executable(${PROJECT_NAME} main.cpp)

# Link the library
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} simulator tracking)

###################################
# Now the unit test bit!
# Add the unit test executable aagainst the source code of the test
add_executable(estimatorTests test/test_estimator.cpp)
# Linking to gtest
target_link_libraries(estimatorTests gtest gtest_main)
# Link the the library
target_link_libraries(estimatorTests ${OpenCV_LIBS} simulator tracking)