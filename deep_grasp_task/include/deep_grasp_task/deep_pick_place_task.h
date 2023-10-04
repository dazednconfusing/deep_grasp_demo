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
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Eigen>


// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
namespace deep_grasp_task
{
using namespace moveit::task_constructor;

class DeepPickPlaceTask
{
public:
  DeepPickPlaceTask(const std::string& task_name, const rclcpp::Node::SharedPtr& nh);
  ~DeepPickPlaceTask() = default;

  void loadParameters();
  void loadParameters(const std::string& object);
  void loadParameters(const std::string& object, const std::string& allowed_collision);

  void init();

  bool plan();

  bool execute();

private:
  rclcpp::Node::SharedPtr nh_;

  std::string task_name_;
  moveit::task_constructor::TaskPtr task_;

  // planning group properties
  std::string arm_group_name_;
  std::string eef_name_;
  std::string hand_group_name_;
  std::string hand_frame_;

  // object + surface
  std::vector<std::string> support_surfaces_;
  std::string object_reference_frame_;
  std::string surface_link_;
  std::string object_name_;
  std::string world_frame_;
  std::vector<double> object_dimensions_;

  // Predefined pose targets
  std::string hand_open_pose_;
  std::string hand_close_pose_;
  std::string arm_home_pose_;

  // Deep grasp properties
  std::string action_name_;
  std::string allowed_collision_;
  std::string start_surface_;
  std::string end_surface_;


  // Pick metrics
  Eigen::Isometry3d grasp_frame_transform_;
  double approach_object_min_dist_;
  double approach_object_max_dist_;
  double lower_object_min_dist_;
  double lower_object_max_dist_;
  double lift_object_min_dist_;
  double lift_object_max_dist_;

  // Place metrics
  geometry_msgs::msg::Pose place_pose_;
  double place_surface_offset_;

  bool deep_grasps_;
};
}  // namespace deep_grasp_task