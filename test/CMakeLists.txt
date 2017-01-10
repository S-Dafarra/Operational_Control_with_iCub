cmake_minimum_required(VERSION 2.8.9)
# set the project name
set(PROJECTNAME RollTest)
project(${PROJECTNAME})
# add the required cmake packages
find_package(RTF)
find_package(RTF COMPONENTS DLL)
find_package(YARP)
# add include directories
include_directories(${CMAKE_SOURCE_DIR}
                    ${RTF_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${YARP_HELPERS_INCLUDE_DIR})

# add the source codes to build the plugin library
add_library(${PROJECTNAME} MODULE roll_test.h
                                  roll_test.cpp)

# add required libraries 
target_link_libraries(${PROJECTNAME} ${RTF_LIBRARIES} ${YARP_LIBRARIES})

# set the installation options

install(DIRECTORY suits/
        DESTINATION ${CMAKE_BINARY_DIR}/test/suits)
install(DIRECTORY fixtures/
        DESTINATION ${CMAKE_BINARY_DIR}/test/fixtures)

