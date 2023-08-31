cmake_minimum_required(VERSION 3.1.3)
project(deep_grasp_msgs)
set(MSG_DEPS
  std_msgs
  actionlib_msgs
  geometry_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation ${MSG_DEPS})
add_action_files(DIRECTORY action FILES CylinderSegment.action SampleGraspPoses.action)
generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(
	CATKIN_DEPENDS
		message_runtime
		${MSG_DEPS}
)
