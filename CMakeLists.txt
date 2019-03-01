cmake_minimum_required(VERSION 3.5)

# set name of the project
project(assignment2)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
set(CMAKE_CXX_STANDARD 17)

set(DATA_FOLDER "\"${CMAKE_CURRENT_SOURCE_DIR}/data\"")

add_definitions(-DSOL_1_1)
add_definitions(-DSOL_1_2)
add_definitions(-DSOL_1_3)
add_definitions(-DSOL_1_4)
add_definitions(-DSOL_2)

# thirdparty code
add_subdirectory(ext)

# our code
add_subdirectory(src)
