cmake_minimum_required(VERSION 2.8.9)
project(make-it-roll)

#message("YARP_LIBRARIES ${YARP_LIBRARIES}")
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
# add YARP include directories
include_directories(${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS})


add_executable(make-it-roll main.cpp)

target_link_libraries(make-it-roll ${YARP_LIBRARIES} ctrlLib)