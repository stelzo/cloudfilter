cmake_minimum_required(VERSION 3.0)
project(cloudfilter)

####### EDIT LIST TO INCLUDE YOUR BINARIES THAT SHOULD BE INSTALLED #######
set(BINARIES "${CMAKE_PROJECT_NAME}")

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