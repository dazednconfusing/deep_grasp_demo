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

#include <deep_grasp_task/deep_pick_place_task.h>
#include <deep_grasp_msgs/action/sample_grasp_poses.hpp>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <deep_grasp_task/stages/deep_grasp_pose.h>
#include <iostream>
#include <Eigen/Dense>



namespace deep_grasp_task
{
constexpr char LOGNAME[] = "pick_place_task";
DeepPickPlaceTask::DeepPickPlaceTask(const std::string& task_name,
  const rclcpp::Node::SharedPtr& nh)
  : nh_(nh), task_name_(task_name)
{
}
void DeepPickPlaceTask::loadParameters()
{
  loadParameters("object", "");
}
void DeepPickPlaceTask::loadParameters(const std::string& object)
{
  loadParameters(object, "");
}

geometry_msgs::msg::Pose poseFromXYZRPY(std::vector<double> pose)
{
  geometry_msgs::msg::Pose p;
  p.position.x = pose[0];
  p.position.y = pose[1];
  p.position.z = pose[2];
  Eigen::Quaterniond q = Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ());
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

Eigen::Isometry3d IsometryFromXYZRPY(std::vector<double> pose)
{
  Eigen::Translation3d translate(Eigen::Vector3d(pose[0], pose[1], pose[2]));
  Eigen::Quaterniond q = Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ());
  return translate * q;
}

void DeepPickPlaceTask::loadParameters(const std::string& object, const std::string& allowed_collision)
{
  /****************************************************
   *                                                  *
   *               Load Parameters                    *
   *                                                  *
   ***************************************************/
  RCLCPP_INFO(nh_->get_logger(), "Loading task parameters");
  // Planning group properties
  nh_->get_parameter("arm_group_name", arm_group_name_);
  nh_->get_parameter("hand_group_name", hand_group_name_);
  nh_->get_parameter("eef_name", eef_name_);
  nh_->get_parameter("hand_frame", hand_frame_);
  nh_->get_parameter("world_frame", world_frame_);
      std::vector<double> grasp_frame_transform;
  nh_->get_parameter( "grasp_frame_transform", grasp_frame_transform);
  grasp_frame_transform_ = IsometryFromXYZRPY(grasp_frame_transform);

  // Predefined pose targets
  nh_->get_parameter( "hand_open_pose", hand_open_pose_);
  nh_->get_parameter( "hand_close_pose", hand_close_pose_);
  nh_->get_parameter( "arm_home_pose", arm_home_pose_);

  // Target object
  nh_->get_parameter(object + "_name", object_name_);
  nh_->get_parameter(object + "_dimensions", object_dimensions_);
  nh_->get_parameter("object_reference_frame", object_reference_frame_);
  nh_->get_parameter("surface_link", surface_link_);
  nh_->get_parameter(object + "_start_surface", start_surface_);
  nh_->get_parameter(object + "_end_surface", end_surface_);
  support_surfaces_ = { surface_link_ };

  // Deep grasp properties
  nh_->get_parameter("action_name", action_name_);

  // Pick/Place metrics
  nh_->get_parameter("approach_object_min_dist", approach_object_min_dist_);
  nh_->get_parameter("approach_object_max_dist", approach_object_max_dist_);
  nh_->get_parameter("lower_object_min_dist", lower_object_min_dist_);
  nh_->get_parameter("lower_object_max_dist", lower_object_max_dist_);
  nh_->get_parameter("lift_object_min_dist", lift_object_min_dist_);
  nh_->get_parameter("lift_object_max_dist", lift_object_max_dist_);
  nh_->get_parameter("place_surface_offset", place_surface_offset_);
    std::vector<double> place_pose;
  nh_->get_parameter( object + "_place_pose", place_pose);
  place_pose_ = poseFromXYZRPY(place_pose);
  nh_->get_parameter("deep_grasps", deep_grasps_);

  allowed_collision_ = std::string(allowed_collision);
}


template <typename T>
std::unique_ptr<stages::ComputeIK> GenerateGraspWrapper(T&& grasp_generator,
  const std::string& hand_open_pose, const std::string& obj, const Eigen::Isometry3d& grasp_frame_transform,
  const std::string& hand_frame, Stage* curr_state) {

  grasp_generator->properties().configureInitFrom(Stage::PARENT);
  grasp_generator->properties().set("marker_ns", "grasp_pose");
  grasp_generator->setPreGraspPose(hand_open_pose);
  grasp_generator->setObject(obj);
  grasp_generator->setMonitoredStage(curr_state);  // Hook into current state

  // Compute IK
  auto wrapper = std::make_unique<stages::ComputeIK>(
    "grasp pose IK", std::move(grasp_generator));
  wrapper->setMaxIKSolutions(1);
  wrapper->setMinSolutionDistance(1.0);
  wrapper->setIKFrame(grasp_frame_transform, hand_frame);
  wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
  wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
  return wrapper;

}

void DeepPickPlaceTask::init()
{
  RCLCPP_INFO(nh_->get_logger(), "Initializing task pipeline");
  const std::string object = object_name_;
  RCLCPP_INFO_STREAM(nh_->get_logger(), "OBJECT NAME: " << object << ", ALLOWED COLLISION: " << allowed_collision_);

  // Reset ROS introspection before constructing the new object
  // TODO(henningkayser): verify this is a bug, fix if possible
  task_.reset();
  task_.reset(new moveit::task_constructor::Task());
  task_->introspection().reset();

  Task& t = *task_;
  t.stages()->setName(task_name_);
  t.loadRobotModel(nh_, "robot_description");

  // Sampling planner
  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(nh_);
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

  // Cartesian planner
  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.001);

  // Set task properties
  t.setProperty("group", arm_group_name_);
  t.setProperty("eef", eef_name_);
  t.setProperty("hand", hand_group_name_);
  t.setProperty("hand_grasping_frame", hand_frame_);
  t.setProperty("ik_frame", hand_frame_);

  // auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
  // {
  //   auto& state = scene->getCurrentStateNonConst();
  //   state.setToDefaultValues(state.getJointModelGroup(arm_group_name_), "Sleep");

    // auto fixed = std::make_unique<stages::FixedState>("initial state");
  //   fixed->setState(scene);
  //   t.add(std::move(fixed));
  // }



  /****************************************************
   *                                                  *
   *               Current State                      *
   *                                                  *
   ***************************************************/
  Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  {
    auto current = std::make_unique<SerialContainer>("current state wrapper");
    t.properties().exposeTo(current->properties(), { "eef", "hand", "group", "ik_frame" });
    current->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

    /****************************************************
---- *               Allow Collision (hand object)   *
  ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object) current state 1");
      stage->allowCollisions(
        object, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
        true);
      current->insert(std::move(stage));
    }

    auto current_state = std::make_unique<stages::CurrentState>("current state");

    // Verify that object is not attached
    // auto applicability_filter =
    //     std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
    // applicability_filter->setPredicate([object](const SolutionBase& s, std::string& comment) {
    //   if (s.start()->scene()->getCurrentState().hasAttachedBody(object))
    //   {
    //     comment = "object with id '" + object + "' is already attached and cannot be picked";
    //     return false;
    //   }
    //   return true;
    // });

    current->insert(std::move(current_state));

    /****************************************************
---- *               Allow Collision (hand object)   *
***************************************************/

    auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object) current state 2");
    stage->allowCollisions(
      object, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
      true);

    current->insert(std::move(stage));


    current_state_ptr = current.get();
    t.add(std::move(current));
  }


  /******************************************************
---- *          Forbid collision (hand, object)        *
*****************************************************/
  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)1");
    stage->allowCollisions(
      object_name_, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
      false);
    t.add(std::move(stage));
  }

  /****************************************************
   *                                                  *
   *               Open Hand                          *
   *                                                  *
   ***************************************************/
  {  // Open Hand
    auto stage = std::make_unique<stages::MoveTo>("open hand pick", sampling_planner);
    stage->setGroup(hand_group_name_);
    stage->setGoal(hand_open_pose_);
    t.add(std::move(stage));
  }


  /****************************************************
   *                                                  *
   *               Move to Pick                       *
   *                                                  *
   ***************************************************/
  {  // Move-to pre-grasp
    auto stage = std::make_unique<stages::Connect>(
      "move to pick", stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(Stage::PARENT);
    t.add(std::move(stage));
  }

  /****************************************************
   *                                                  *
   *               Pick Object                        *
   *                                                  *
   ***************************************************/
  Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator
  {
    auto grasp = std::make_unique<SerialContainer>("pick object");
    t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
    grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object) pick ");
      stage->allowCollisions(
        object, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
        false);
      grasp->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object) pick ");
      stage->allowCollisions(
        object, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
        true);
      grasp->insert(std::move(stage));
    }



    /****************************************************
  ---- *               Approach Object                    *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame_);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setMinMaxDistance(approach_object_min_dist_, approach_object_max_dist_);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame_;
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object) pick 1");
      stage->allowCollisions(
        object, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
        true);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  ---- *               Generate Grasp Pose                *


        ***************************************************/

    {
      if (deep_grasps_) {
        std::string stage_name = "generate DEEP grasp pose";
        auto dg = std::make_unique<stages::DeepGraspPose<deep_grasp_msgs::action::SampleGraspPoses >>(
          action_name_, stage_name, 0, 0);

        std::unique_ptr<stages::ComputeIK> wrapper = GenerateGraspWrapper(std::move(dg), hand_open_pose_, object, Eigen::Isometry3d::Identity(), hand_frame_, current_state_ptr);
        grasp->insert(std::move(wrapper));
      }
      else {
        auto gg = std::make_unique< stages::GenerateGraspPose>("generate grasp pose");
        gg->setAngleDelta(4 * M_PI / 16);
        std::unique_ptr<stages::ComputeIK> wrapper = GenerateGraspWrapper(std::move(gg), hand_open_pose_, object, grasp_frame_transform_, hand_frame_, current_state_ptr);
        grasp->insert(std::move(wrapper));
      }
      //   // stage.reset(std::reinterpret_cast<stages::GenerateGraspPose>(new stages::GenerateGraspPose));
      // }


    }

    /****************************************************
---- *               Allow Collision (hand object)   *
  ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object) pick 2");
      stage->allowCollisions(
        object, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
        true);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  ---- *               Close Hand                      *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
      // stage->properties().property("group").configureInitFrom(Stage::PARENT, hand_group_name_);
      stage->setGroup(hand_group_name_);
      stage->setGoal(hand_close_pose_);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  .... *               Attach Object                      *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object, hand_frame_);
      // attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    /****************************************************
  .... *               Allow collision (object support)   *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)1");
      stage->allowCollisions({ object }, { start_surface_, end_surface_ }, true);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    // {
    //   auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,support)");
    //   stage->allowCollisions(
    //     t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
    //     { start_surface_, end_surface_ }, true);
    //   attach_object_stage = stage.get();
    //   grasp->insert(std::move(stage));
    // }


    /****************************************************
  .... *               Lift object                        *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setMinMaxDistance(lift_object_min_dist_, lift_object_max_dist_);
      stage->setIKFrame(hand_frame_);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame_;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  .... *               Forbid collision (object support)  *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object, start surface)");
      stage->allowCollisions({ object }, { start_surface_ }, false);
      grasp->insert(std::move(stage));
    }

    // Add grasp container to task
    t.add(std::move(grasp));
  }


  /******************************************************
   *                                                    *
   *          Move to Place                             *
   *                                                    *
   *****************************************************/
  {
    auto stage = std::make_unique<stages::Connect>(
      "move to place", stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(Stage::PARENT);
    t.add(std::move(stage));
  }

  /******************************************************
   *                                                    *
   *          Place Object                              *
   *                                                    *
   *****************************************************/
  {
    auto place = std::make_unique<SerialContainer>("place object");
    t.properties().exposeTo(place->properties(), { "eef", "hand", "group" });
    place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group" });

    /******************************************************
  ---- *          Lower Object                              *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
      stage->properties().set("marker_ns", "lower_object");
      stage->properties().set("link", hand_frame_);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setMinMaxDistance(lower_object_min_dist_, lower_object_max_dist_);

      // Set downward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame_;
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    /******************************************************
  ---- *          Generate Place Pose                       *
     *****************************************************/
    {
      // Generate Place Pose
      auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(object);

      // Set target pose
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = object_reference_frame_;
      p.pose = place_pose_;
      p.pose.position.z += 0.5 * object_dimensions_.at(0) + place_surface_offset_;
      // ROS_WARN_STREAM_NAMED(nh_->get_logger(), " PARAM place pose: " << p.pose.position.x << "," << p.pose.position.y << "," << p.pose.position.z);
      stage->setPose(p);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage
      // stage->addSolutionCallback([](const SolutionBase& s) {
      //   planning_scene::PlanningSceneConstPtr scene = s.end()->scene();
      //   const moveit::core::RobotState& robot_state_ = scene->getCurrentState();

      //   std::vector<const moveit::core::AttachedBody*> attached_bodies;

      //   robot_state_.getAttachedBodies(attached_bodies);

      //   if (attached_bodies.size() > 0) {
      //     ROS_WARN_STREAM_NAMED(nh_->get_logger(), " num attached bodies: " << attached_bodies.size());
      //     const auto* body = attached_bodies[0];
      //     // current object_pose w.r.t. planning frame
      //     const Eigen::Isometry3d& orig_object_pose = body->getGlobalPose();
      //     const auto& pose_ = orig_object_pose.translation();
      //     ROS_WARN_STREAM_NAMED(nh_->get_logger(), body->getName() << " place pose: " << pose_.x() << "," << pose_.y() << "," << pose_.z());
      //   }
      //   });

      // Compute IK
      auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(1);
      wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
      wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
      // wrapper->addSolutionCallback([](const SolutionBase& s) {
      //   planning_scene::PlanningSceneConstPtr scene = s.end()->scene();
      //   const moveit::core::RobotState& robot_state_ = scene->getCurrentState();

      //   std::vector<const moveit::core::AttachedBody*> attached_bodies;

      //   robot_state_.getAttachedBodies(attached_bodies);

      //   if (attached_bodies.size() > 0) {
      //     ROS_WARN_STREAM_NAMED(nh_->get_logger(), " num attached bodies: " << attached_bodies.size());
      //     const auto* body = attached_bodies[0];
      //     // current object_pose w.r.t. planning frame
      //     const Eigen::Isometry3d& orig_object_pose = body->getGlobalPose();
      //     const auto& pose_ = orig_object_pose.translation();
      //     ROS_WARN_STREAM_NAMED(nh_->get_logger(), body->getName() << " IK place pose: " << pose_.x() << "," << pose_.y() << "," << pose_.z());
      //   }
      //   });

      place->insert(std::move(wrapper));
    }

    /******************************************************
  ---- *          Open Hand                              *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::MoveTo>("open hand place", sampling_planner);
      // stage->properties().property("group").configureInitFrom(Stage::PARENT, hand_group_name_);
      stage->setGroup(hand_group_name_);
      stage->setGoal(hand_open_pose_);
      place->insert(std::move(stage));
    }

    /******************************************************
  ---- *          Detach Object                             *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_name_, hand_frame_);
      place->insert(std::move(stage));
    }

    /******************************************************
  ---- *          Retreat Motion                            *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setMinMaxDistance(lift_object_min_dist_, lift_object_max_dist_);
      stage->setIKFrame(hand_frame_);
      stage->properties().set("marker_ns", "retreat");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame_;
      vec.vector.x = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    // Add place container to task
    t.add(std::move(place));
  }

  /******************************************************
---- *          Forbid collision (hand, object)        *
*****************************************************/
  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object) final");
    stage->allowCollisions(
      object_name_, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
      false);
    t.add(std::move(stage));
  }

  /******************************************************
   *                                                    *
   *          Move to Home                              *
   *                                                    *
   *****************************************************/
  {
    auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
    stage->properties().configureInitFrom(Stage::PARENT, { "group" });
    stage->setGoal(arm_home_pose_);
    stage->restrictDirection(stages::MoveTo::FORWARD);
    t.add(std::move(stage));
  }
}


bool DeepPickPlaceTask::plan()
{
  RCLCPP_INFO(nh_->get_logger(), "Start searching for task solutions");
  int max_solutions = nh_->get_parameter_or("max_solutions", 10);
   RCLCPP_INFO(nh_->get_logger(), "About to plan");
  try
  {
    task_->plan(max_solutions);
  }
  catch (InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(nh_->get_logger(), "Initialization failed: " << e);
    return false;
  }
  if (task_->numSolutions() == 0)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Planning failed");
    return false;
  }
  task_->introspection().publishSolution(*task_->solutions().front());
  RCLCPP_INFO(nh_->get_logger(), "SUCCESS!!!!! AYYYYYY");
  return true;
}

bool DeepPickPlaceTask::execute()
{
  // moveit_msgs::msg::MoveItErrorCodes execute_result;
  // execute_result = task_->execute(*task_->solutions().front());
  // // // If you want to inspect the goal message, use this instead:
  // // actionlib::SimpleActionClient<moveit_task_constructor_msgs::msg::ExecuteTaskSolutionAction>
  // // execute("execute_task_solution", true); execute.waitForServer();
  // // moveit_task_constructor_msgs::msg::ExecuteTaskSolutionGoal execute_goal;
  // // task_->solutions().front()->fillMessage(execute_goal.solution);
  // // execute.sendGoalAndWait(execute_goal);
  // // execute_result = execute.getResult()->error_code;

  // if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
  //   ROS_ERROR_STREAM("Task execution failed and returned: " << execute_result.val);
  // }
  RCLCPP_INFO(nh_->get_logger(), "Executing solution trajectory.");
  auto result = task_->execute(*task_->solutions().front());

  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    // task_->introspection().publishTaskState();
    RCLCPP_ERROR_STREAM(nh_->get_logger(), "Task execution failed and returned: " << result.val);
    return false;
  } else {
    RCLCPP_INFO(nh_->get_logger(), "Task execution completed.");
  }

  return true;
}
}  // namespace deep_grasp_task