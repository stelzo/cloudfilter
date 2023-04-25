### A CMakeLists.txt file for using ros nodes in a catkin workspace.
### We just call the build script that builds the Rust binaries and copies them to the devel space.
###
### @author: Christopher Sieh <stelzo@steado.de>
### @date: 2023-04-25
### @license: MIT

cmake_minimum_required(VERSION 3.0)
project(cloudfilter)

####### Edit this list to include all your Rust binaries #######
set(BINARIES "${CMAKE_PROJECT_NAME}")
#######

find_package(catkin REQUIRED)
catkin_package()

# set CMAKE_BUILD_TYPE if not set
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE DEBUG)
endif()

add_custom_target(${CMAKE_PROJECT_NAME} ALL
    COMMAND ./build.sh ${CMAKE_BUILD_TYPE} ${BINARIES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)