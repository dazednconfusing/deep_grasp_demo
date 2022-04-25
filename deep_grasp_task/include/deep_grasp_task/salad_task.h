/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2020 PickNik Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein, Boston Cleek
   Desc:   A demo to show MoveIt Task Constructor using a deep learning based
           grasp generator
*/

#pragma once

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/deep_grasp_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>
#include <moveit_task_constructor_msgs/SampleGraspPosesAction.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace deep_grasp_task
{
using namespace moveit::task_constructor;

class SaladTask
{
public:
  struct CollisionObjectParams
  {
    std::string name;
    std::vector<double> dimensions;
    geometry_msgs::Pose grasp_pose;
    geometry_msgs::Pose place_pose;
    geometry_msgs::Pose pre_pour_pose;
    geometry_msgs::Pose pour_pose;
  };

  SaladTask(const std::string& task_name, const ros::NodeHandle& nh, const std::vector<std::string> collision_objects);
  ~SaladTask() = default;

  void loadParameters();

  std::unique_ptr<SerialContainer>
  createCurrentState(const moveit::task_constructor::solvers::PipelinePlannerPtr sampling_planner);
  std::unique_ptr<SerialContainer>
  createPick(const std::string object, const moveit::task_constructor::TaskPtr tptr,
             const moveit::task_constructor::solvers::PipelinePlannerPtr sampling_planner,
             const moveit::task_constructor::solvers::CartesianPathPtr cartesian_planner,
             Stage* const current_state_ptr, Stage*& attach_obj_ptr);

  std::unique_ptr<SerialContainer>
  createPlace(const std::string object, const moveit::task_constructor::TaskPtr tptr,
              const moveit::task_constructor::solvers::PipelinePlannerPtr sampling_planner,
              const moveit::task_constructor::solvers::CartesianPathPtr cartesian_planner,
              Stage* const current_state_ptr, Stage* const attach_obj_ptr,
              const geometry_msgs::Vector3Stamped* const retreat_dir = nullptr, const float retreat_dist = 0);
  std::unique_ptr<SerialContainer>
  createPour(const std::string object, Stage*& attach_obj_ptr,
             const moveit::task_constructor::solvers::PipelinePlannerPtr sampling_planner,
             const moveit::task_constructor::solvers::CartesianPathPtr cartesian_planner);
  std::unique_ptr<SerialContainer> createPickScoopPlace(
      const std::string object, const std::string next_object, const moveit::task_constructor::TaskPtr tptr,
      const moveit::task_constructor::solvers::PipelinePlannerPtr sampling_planner,
      const moveit::task_constructor::solvers::CartesianPathPtr cartesian_planner, Stage*& current_state_ptr);

  std::unique_ptr<SerialContainer>
  createPickPourPlace(const std::string object,
                      const moveit::task_constructor::solvers::PipelinePlannerPtr sampling_planner,
                      const moveit::task_constructor::solvers::CartesianPathPtr cartesian_planner,
                      const geometry_msgs::Vector3Stamped* const reatreat_dir = nullptr, const float retreat_dist = 0);

  void init();

  bool plan();

  bool execute();

private:
  ros::NodeHandle nh_;

  std::string task_name_;
  moveit::task_constructor::TaskPtr task_;
  Stage* monitored_state_;

  // planning group properties
  std::string arm_group_name_;
  std::string eef_name_;
  std::string hand_group_name_;
  std::string hand_frame_;

  // object + surface
  std::vector<std::string> support_surfaces_;
  std::string object_reference_frame_;
  std::string surface_link_;
  std::map<std::string, CollisionObjectParams> collision_objects_;
  std::string world_frame_;

  // Predefined pose targets
  std::string hand_open_pose_;
  std::string hand_close_pose_;
  std::string arm_home_pose_;

  // Deep grasp properties
  std::string action_name_;

  // Execution
  actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> execute_;

  // Pick metrics
  Eigen::Isometry3d grasp_frame_transform_;
  double approach_object_min_dist_;
  double approach_object_max_dist_;
  double lift_object_min_dist_;
  double lift_object_max_dist_;

  // Place metrics

  geometry_msgs::Pose pour_pose_;
  geometry_msgs::Pose pre_pour_pose_;
  double place_surface_offset_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
};
}  // namespace deep_grasp_task
