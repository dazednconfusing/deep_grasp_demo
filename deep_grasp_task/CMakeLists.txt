cmake_minimum_required(VERSION 3.5)
project(deep_grasp_task)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(dgl_ros_interfaces REQUIRED)
find_package(deep_grasp_msgs REQUIRED)

# ##################################
# # Catkin specific configuration ##
# ##################################

# ##########
# # Build ##
# ##########

# Specify additional locations of header files
include_directories(include)
add_executable(stack_blocks
  src/stack_blocks.cpp
)

add_executable(cylinder_segment
  src/cylinder_segment.cpp
)

add_library(deep_grasp_task SHARED
  src/deep_pick_place_task.cpp
)

add_library(action_client SHARED
  src/stages/deep_grasp_pose.cpp)

ament_target_dependencies(action_client
  moveit_task_constructor_core moveit_visual_tools rclcpp dgl_ros_interfaces
)

ament_target_dependencies(deep_grasp_task
  moveit_task_constructor_core moveit_visual_tools rclcpp dgl_ros_interfaces EIGEN3
)
target_link_libraries(deep_grasp_task action_client)

ament_target_dependencies(stack_blocks
  rclcpp
  moveit_task_constructor_core moveit_visual_tools deep_grasp_msgs EIGEN3)

target_link_libraries(stack_blocks deep_grasp_task action_client)

ament_target_dependencies(cylinder_segment
  rclcpp moveit_task_constructor_core
  deep_grasp_msgs pcl_ros pcl_conversions tf2_ros tf2_geometry_msgs)

# Your package locations should be listed before other locations

# Declare a C++ executable
# add_executable(salad_demo
# src/salad_demo.cpp
# src/salad_task.cpp
# )

# Declare a C++ executable

# # Specify libraries to link a library or executable target against

# ############
# # Install ##
# ############

# Mark executables and/or libraries for installation
install(
  TARGETS
  stack_blocks
  cylinder_segment
  deep_grasp_task
  action_client
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION share/${PROJECT_NAME}
)

# install(
# TARGETS
# stack_blocks
# cylinder_segment
# deep_grasp_task
# action_client
# EXPORT export_${PROJECT_NAME}
# LIBRARY DESTINATION share/${PROJECT_NAME}
# RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Install launch files
install(DIRECTORY
  launch
  urdf
  worlds
  config
  meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
