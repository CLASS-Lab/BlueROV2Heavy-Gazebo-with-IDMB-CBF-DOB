cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(animated_fish1)

# Find packages

find_package(gazebo REQUIRED)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# include appropriate directories
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Create libraries and executables

add_library(animated_fish1 SHARED animated_fish1.cc)
target_link_libraries(animated_fish1 ${GAZEBO_LIBRARIES})

# add_executable(integrated_main integrated_main.cc)
# target_link_libraries(integrated_main ${GAZEBO_LIBRARIES} pthread)


# add_executable(independent_listener independent_listener.cc)
# target_link_libraries(independent_listener ${GAZEBO_LIBRARIES} pthread)
