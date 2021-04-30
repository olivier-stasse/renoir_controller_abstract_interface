cmake_minimum_required(VERSION 3.0.2)

SET(PROJECT_ORG olivier-stasse)
SET(PROJECT_DESCRIPTION renoir_controller_abstract_interface)
SET(PROJECT_NAME renoir_controller_abstract_interface)
SET(PROJECT_URL "https://gitlab.laas.fr/${PROJECT_ORG}/${PROJECT_NAME}")

# Project configuration
SET(CXX_DISABLE_WERROR True)
SET(PROJECT_USE_CMAKE_EXPORT TRUE)

# # JRL-cmakemodule setup
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/ros.cmake)


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_VERBOSE_MAKEFILE TRUE)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})
CHECK_MINIMAL_CXX_STANDARD(11 ENFORCE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES renoir_controller_abstract_interface
CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########


## Declare a C++ library
add_library(renoir_controller_example_interface SHARED
  tests/renoir_controller_example_interface.cpp
)

add_library(renoir_controller_abstract_interface INTERFACE
  )

## Specify additional locations of header files
## Your package locations should be listed before other locations

target_include_directories(renoir_controller_abstract_interface
  INTERFACE
  $<BUILD_INTERFACE:include/renoir_controller_abstract_interface>
  $<INSTALL_INTERFACE:include>
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(load_renoir_controller_node
  tests/load_renoir_controller_interface.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(load_renoir_controller_node
   ${catkin_LIBRARIES} renoir_controller_example_interface dl
)

#############
## Install ##
#############

install(TARGETS renoir_controller_abstract_interface
  EXPORT ${TARGETS_EXPORT_NAME}
  PUBLIC_HEADER
  )

install(FILES
  include/renoir_controller_abstract_interface/renoir_controller_abstract_interface.hh
  DESTINATION include/renoir_controller_abstract_interface
  )

install(TARGETS renoir_controller_example_interface
  DESTINATION lib
)

install(TARGETS load_renoir_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
)

