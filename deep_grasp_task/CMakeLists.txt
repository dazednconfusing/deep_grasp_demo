# C++ 20
add_compile_options(-std=c++14)
cmake_minimum_required(VERSION 3.0.2)
project(deep_grasp_task)



# Warnings
add_definitions(-W -Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith
  -Winit-self -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	actionlib
	moveit_core
	moveit_ros_planning_interface
	moveit_task_constructor_core
	moveit_task_constructor_msgs
  moveit_visual_tools
  roscpp
  pcl_ros
  pcl_conversions
  tf2_ros
  tf2
  tf2_geometry_msgs
	rosparam_shortcuts
  deep_grasp_msgs
)

###################################
## Catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
	moveit_task_constructor_msgs
  deep_grasp_msgs
  moveit_visual_tools
 	roscpp
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



# Declare a C++ executable
add_executable(salad_demo
 							 src/salad_demo.cpp
)


add_executable(cylinder_segment
 							 src/cylinder_segment.cpp
)

# # Specify libraries to link a library or executable target against
# target_link_libraries(salad_demo
#                       ${catkin_LIBRARIES}
# )

target_link_libraries(cylinder_segment
                      ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    salad_demo
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# # Mark roslaunch files for installation
# install(
#   DIRECTORY
#     launch
#   DESTINATION
#     ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
#
# # Mark config files for installation
# install(
#   DIRECTORY
#     config
#   DESTINATION
#     ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
