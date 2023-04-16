cmake_minimum_required(VERSION 3.0)
project(cloudfilter)

find_package(catkin REQUIRED)
catkin_package()

# build ros node with cargo and copy to build folder from catkin
# project
add_custom_target(cloudfilter ALL
    COMMAND cargo build --release
    COMMAND cp target/release/cloudfilter ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)