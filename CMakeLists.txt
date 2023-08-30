cmake_minimum_required(VERSION 2.8.3)
project(xsens_mti_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  diagnostic_updater
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xsens_mti_driver
  CATKIN_DEPENDS
  roscpp
  tf2
  tf2_ros
  sensor_msgs
  std_msgs
  geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  lib/xspublic
  ${catkin_INCLUDE_DIRS}
)

link_directories(lib/xspublic/xscontroller lib/xspublic/xscommon lib/xspublic/xstypes)
add_executable(
	xsens_mti_node
	src/main.cpp
	src/xdainterface.cpp
	src/xdacallback.cpp
)
target_link_libraries(
	xsens_mti_node
	xscontroller
	xscommon
	xstypes
	pthread
	dl
	${catkin_LIBRARIES}
)
add_dependencies(xsens_mti_node xspublic)

add_executable(
  list_filter_profiles
  src/list_filter_profiles.cpp
)
target_link_libraries(list_filter_profiles
  xscontroller
  xscommon
  xstypes
  dl
  ${catkin_LIBRARIES}
)
add_dependencies(list_filter_profiles xspublic)

add_executable(
  configure_outputs
  src/configure_outputs.cpp
  src/xdainterface.cpp
  src/xdacallback.cpp
)
target_link_libraries(configure_outputs
  xscontroller
  xscommon
  xstypes
  dl
  ${catkin_LIBRARIES}
)
add_dependencies(configure_outputs xspublic)

# Install targets
install(
  TARGETS
    xsens_mti_node
    configure_outputs
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install directories
install(
  DIRECTORY
    launch
    param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_custom_target(xspublic
  COMMAND +make -C ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic
)
