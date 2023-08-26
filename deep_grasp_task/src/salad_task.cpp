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

#include <deep_grasp_task/salad_task.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace deep_grasp_task
{
namespace
{
static const double connectTimeout = 2.0;
static const double ikTimeout = 2.0;
static const double goalJointTolerence = 1e-5;
static const int maxIkSolutions = 5;
}  // namespace
constexpr char LOGNAME[] = "pick_place_task";
SaladTask::SaladTask(const std::string& task_name, const ros::NodeHandle& nh,
                     const std::vector<std::string> collision_objects)
  : nh_(nh), task_name_(task_name), execute_("execute_task_solution", true)
{
  for (auto obj : collision_objects)
  {
    collision_objects_.insert(std::pair<std::string, CollisionObjectParams>(obj, CollisionObjectParams()));
  }
}

size_t loadObjectParams(ros::NodeHandle& pnh, const std::string name,
                        std::map<std::string, SaladTask::CollisionObjectParams>& cobjects)
{
  size_t errors = 0;
  // Target object
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, name + "_name", cobjects[name].name);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, name + "_dimensions", cobjects[name].dimensions);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, name + "_grasp_pose", cobjects[name].grasp_pose);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, name + "_place_pose", cobjects[name].place_pose);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, name + "_pre_pour_pose", cobjects[name].pre_pour_pose);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, name + "_pour_pose", cobjects[name].pour_pose);
  return errors;
}
void SaladTask::loadParameters()
{
  /****************************************************
   *                                                  *
   *               Load Parameters                    *
   *                                                  *
   ***************************************************/
  ROS_INFO_NAMED(LOGNAME, "Loading task parameters");
  ros::NodeHandle pnh("~");

  // Planning group properties
  size_t errors = 0;
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm_group_name", arm_group_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_group_name", hand_group_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "eef_name", eef_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_frame", hand_frame_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "world_frame", world_frame_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "grasp_frame_transform", grasp_frame_transform_);

  // Predefined pose targets
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_open_pose", hand_open_pose_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_close_pose", hand_close_pose_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm_home_pose", arm_home_pose_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "surface_link", surface_link_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame_);
  support_surfaces_ = { surface_link_ };

  // Load object params
  for (auto cobj : collision_objects_)
  {
    errors += loadObjectParams(pnh, cobj.first, collision_objects_);
  }

  // Deep grasp properties
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "action_name", action_name_);

  // Pick/Place metrics
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_min_dist", approach_object_min_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_max_dist", approach_object_max_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_min_dist", lift_object_min_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_max_dist", lift_object_max_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "place_surface_offset", place_surface_offset_);
  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

std::unique_ptr<stages::ModifyPlanningScene>
createSetCollisions(const moveit::task_constructor::TaskPtr tptr, const std::string link,
                    const std::map<std::string, SaladTask::CollisionObjectParams>& cobjects, bool allow)

{
  auto all_cobjects = std::vector<std::string>();
  for (auto cobj : cobjects)
  {
    all_cobjects.push_back(cobj.first);
  }
  auto stage = std::make_unique<stages::ModifyPlanningScene>("set collisions (" + link + ")=" + std::to_string(allow));
  stage->allowCollisions(
      all_cobjects, tptr->getRobotModel()->getJointModelGroup(link)->getLinkModelNamesWithCollisionGeometry(), allow);

  return stage;
}

std::unique_ptr<SerialContainer>
SaladTask::createCurrentState(moveit::task_constructor::solvers::PipelinePlannerPtr sampling_planner)
{
  auto c = std::make_unique<SerialContainer>("Current State");
  task_->properties().exposeTo(c->properties(), { "eef", "hand", "group", "ik_frame" });
  c->properties().configureInitFrom(Stage::PARENT);
  auto current_state = std::make_unique<stages::CurrentState>("current state");
  c->insert(std::move(current_state));

  auto open_hand = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
  open_hand->properties().property("group").configureInitFrom(Stage::PARENT, "hand");
  open_hand->setGoal(hand_open_pose_);
  monitored_state_ = open_hand.get();
  c->insert(std::move(open_hand));

  return c;
}

std::unique_ptr<SerialContainer>
SaladTask::createPick(const std::string object, const moveit::task_constructor::TaskPtr tptr,
                      const moveit::task_constructor::solvers::PipelinePlannerPtr sampling_planner,
                      const moveit::task_constructor::solvers::CartesianPathPtr cartesian_planner,
                      Stage* const current_state_ptr, Stage*& attach_obj_ptr)
{
  moveit::task_constructor::Task& t = *tptr;
  auto c = std::make_unique<SerialContainer>("Pick Path");
  tptr->properties().exposeTo(c->properties(), { "eef", "hand", "group", "ik_frame" });
  c->properties().configureInitFrom(Stage::PARENT);

  {  // Move-to pre-grasp
    auto stage = std::make_unique<stages::Connect>(
        "move to pick", stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner },
                                                             { hand_group_name_, sampling_planner } });
    stage->setTimeout(connectTimeout);
    stage->properties().configureInitFrom(Stage::PARENT);
    c->insert(std::move(stage));
  }

  {  // Grasp
    auto grasp = std::make_unique<SerialContainer>("pick object");
    c->properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
    grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

    {  // Open Hand
      auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
      stage->properties().property("group").configureInitFrom(Stage::PARENT, "hand");
      stage->setGoal(hand_open_pose_);
      grasp->insert(std::move(stage));
    }

    {  // Allow Collisions

      auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision 2(hand,object)");
      stage->allowCollisions(
          object, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
          true);
      grasp->insert(std::move(stage));
    }

    {  // Approach
      auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame_);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setMinMaxDistance(approach_object_min_dist_, approach_object_max_dist_);

      // Set hand forward direction
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = hand_frame_;
      vec.vector.x = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    {  // Generate Grasp
      auto stage = std::make_unique<stages::GeneratePose>("generate grasp pose");
      stage->properties().configureInitFrom(Stage::PARENT);
      stage->setMonitoredStage(current_state_ptr);
      geometry_msgs::PoseStamped p;
      p.header.frame_id = object_reference_frame_;
      p.pose = collision_objects_[object].grasp_pose;
      stage->setPose(p);

      visual_tools_->publishAxis(p.pose);
      visual_tools_->trigger();

      // Compute IK
      auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(maxIkSolutions);
      wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
      wrapper->setTimeout(ikTimeout);
      wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    {  // Allow Collisions
      auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision 3 (hand,object)");
      stage->allowCollisions(
          object, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
          true);
      grasp->insert(std::move(stage));
    }

    {  // Close Hand
      auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
      stage->properties().property("group").configureInitFrom(Stage::PARENT, "hand");
      stage->setGoal(hand_close_pose_);
      grasp->insert(std::move(stage));
    }

    {  // Attach Object
      auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object, hand_frame_);

      // Set the ptr
      attach_obj_ptr = stage.get();
      grasp->insert(std::move(stage));
    }

    {  // Allow Support-Object Collision
      auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
      stage->allowCollisions({ object }, support_surfaces_, true);
      grasp->insert(std::move(stage));
    }
    c->insert(std::move(grasp));
  }

  {  // Lift object
    auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
    stage->properties().configureInitFrom(Stage::PARENT, { "group" });
    stage->setMinMaxDistance(lift_object_min_dist_, lift_object_max_dist_);
    stage->setIKFrame(hand_frame_);
    stage->properties().set("marker_ns", "lift_object");

    // Set upward direction
    geometry_msgs::Vector3Stamped vec;
    vec.header.frame_id = world_frame_;
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    c->insert(std::move(stage));
  }

  {  // Forbid collision
    auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
    stage->allowCollisions({ object }, support_surfaces_, false);
    c->insert(std::move(stage));
  }

  return c;
}

std::unique_ptr<SerialContainer>
SaladTask::createPlace(const std::string object, const moveit::task_constructor::TaskPtr tptr,
                       const moveit::task_constructor::solvers::PipelinePlannerPtr sampling_planner,
                       const moveit::task_constructor::solvers::CartesianPathPtr cartesian_planner,
                       Stage* const current_state_ptr, Stage* const attach_obj_ptr,
                       const geometry_msgs::Vector3Stamped* const retreat_dir, const float retreat_dist)
{
  moveit::task_constructor::Task& t = *tptr;
  auto c = std::make_unique<SerialContainer>("Cartesian Path");
  tptr->properties().exposeTo(c->properties(), { "eef", "hand", "group", "ik_frame" });
  c->properties().configureInitFrom(Stage::PARENT);

  {  // Place Object
    auto place = std::make_unique<SerialContainer>("place object");
    c->properties().exposeTo(place->properties(), { "eef", "hand", "group", "ik_frame" });
    place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

    {  // Lower Object
      auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
      stage->properties().set("marker_ns", "lower_object");
      stage->properties().set("link", hand_frame_);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setMinMaxDistance(lift_object_min_dist_, lift_object_max_dist_);

      // Set downward direction
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = world_frame_;
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    // {  // Allow Support-Object Collision
    //   auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
    //   stage->allowCollisions({ object }, support_surfaces_, true);
    //   place->insert(std::move(stage));
    // }
    {
      // Generate Place Pose
      auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
      geometry_msgs::PoseStamped pose_msg;
      stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(object);

      // Set target pose
      geometry_msgs::PoseStamped p;
      p.header.frame_id = object_reference_frame_;
      p.pose = collision_objects_[object].place_pose;
      p.pose.position.z += place_surface_offset_;
      // visual_tools_->publishAxis(p.pose);
      // visual_tools_->trigger();
      stage->setPose(p);
      stage->setMonitoredStage(attach_obj_ptr);  // Hook into attach_object_stage

      auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setTimeout(5.0);
      wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
      wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {  // Open Hand
      auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
      stage->properties().property("group").configureInitFrom(Stage::PARENT, "hand");
      stage->setGoal(hand_open_pose_);
      place->insert(std::move(stage));
    }
    {  // Detach
      auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object, hand_frame_);
      place->insert(std::move(stage));
    }

    {  // Forbid Collisions
      auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(
          object, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
          false);
      c->insert(std::move(stage));
    }

    {  // Retreat
      auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });

      if (retreat_dist == 0)
      {
        stage->setMinMaxDistance(approach_object_min_dist_, approach_object_max_dist_);
      }
      else
      {
        stage->setMinMaxDistance(retreat_dist, approach_object_max_dist_);
      }

      stage->setIKFrame(hand_frame_);
      stage->properties().set("marker_ns", "retreat");

      if (retreat_dir == nullptr)
      {
        geometry_msgs::Vector3Stamped vec;
        vec.header.frame_id = hand_frame_;
        vec.vector.x = -1.0;
        stage->setDirection(vec);
      }
      else
      {
        stage->setDirection(*retreat_dir);
      }
      monitored_state_ = stage.get();
      place->insert(std::move(stage));
    }
    c->insert(std::move(place));
  }

  return c;
}

std::unique_ptr<SerialContainer>
SaladTask::createPour(const std::string object, Stage*& attach_obj_ptr,
                      const moveit::task_constructor::solvers::PipelinePlannerPtr sampling_planner,
                      const moveit::task_constructor::solvers::CartesianPathPtr cartesian_planner)
{
  auto c = std::make_unique<SerialContainer>("Pour Path");
  task_->properties().exposeTo(c->properties(), { "eef", "hand", "group", "ik_frame" });
  c->properties().configureInitFrom(Stage::PARENT);
  {
    // Generate Pre pour Pose
    auto stage = std::make_unique<stages::GeneratePlacePose>("generate pre pour pose");
    geometry_msgs::PoseStamped pose_msg;
    stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
    stage->properties().set("marker_ns", "pre_pour_pose");
    stage->setObject(object);
    geometry_msgs::PoseStamped p;
    p.header.frame_id = object_reference_frame_;
    p.pose = collision_objects_[object].pre_pour_pose;
    visual_tools_->publishAxis(p.pose);
    visual_tools_->trigger();
    stage->setPose(p);
    stage->setMonitoredStage(attach_obj_ptr);  // Hook into attach_object_stage

    // Compute IK
    auto wrapper = std::make_unique<stages::ComputeIK>("pre pour pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(8.0);
    wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
    wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
    c->insert(std::move(wrapper));
  }
  {
    // Pour
    auto stage = std::make_unique<stages::Connect>(
        "pour", stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
    stage->setTimeout(0.5);
    stage->properties().configureInitFrom(Stage::PARENT);
    c->insert(std::move(stage));
  }
  {
    // Generate Pour Pose
    auto stage = std::make_unique<stages::GeneratePlacePose>("generate pour pose");
    geometry_msgs::PoseStamped pose_msg;
    stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
    stage->properties().set("marker_ns", "pour_pose");
    stage->setObject(object);
    // Set target pose
    geometry_msgs::PoseStamped p;
    p.header.frame_id = object_reference_frame_;
    p.pose = collision_objects_[object].pour_pose;
    stage->setPose(p);
    visual_tools_->publishAxis(p.pose);
    visual_tools_->trigger();
    stage->setMonitoredStage(attach_obj_ptr);  // Hook into attach_object_stage
    // Compute IK
    auto wrapper = std::make_unique<stages::ComputeIK>("pour pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(maxIkSolutions);
    wrapper->setTimeout(connectTimeout);
    wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
    wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
    c->insert(std::move(wrapper));
  }
  {
    // Return to pre pour
    auto stage = std::make_unique<stages::Connect>(
        "pour", stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
    stage->setTimeout(0.5);
    stage->properties().configureInitFrom(Stage::PARENT);
    c->insert(std::move(stage));
  }
  {
    // Generate Post pour Pose
    auto stage = std::make_unique<stages::GeneratePlacePose>("generate post pour pose");
    geometry_msgs::PoseStamped pose_msg;
    stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
    stage->properties().set("marker_ns", "pre_pour_pose");
    stage->setObject(object);
    geometry_msgs::PoseStamped p;
    p.header.frame_id = object_reference_frame_;
    p.pose = collision_objects_[object].pre_pour_pose;
    stage->setPose(p);
    stage->setMonitoredStage(attach_obj_ptr);  // Hook into attach_object_stage

    // Compute IK
    auto wrapper = std::make_unique<stages::ComputeIK>("pre pour pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(8.0);
    wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
    wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
    c->insert(std::move(wrapper));
  }
  return c;
}

std::unique_ptr<SerialContainer>
SaladTask::createPickPourPlace(const std::string object,
                               const moveit::task_constructor::solvers::PipelinePlannerPtr sampling_planner,
                               const moveit::task_constructor::solvers::CartesianPathPtr cartesian_planner,
                               const geometry_msgs::Vector3Stamped* const retreat_dir, const float retreat_dist)
{
  auto c = std::make_unique<SerialContainer>("PickScoopPlace Path");
  task_->properties().exposeTo(c->properties(), { "eef", "hand", "group", "ik_frame" });
  {
    Stage* attach_obj_ptr = nullptr;
    {  // Pick
      auto stage = createPick(object, task_, sampling_planner, cartesian_planner, monitored_state_, attach_obj_ptr);
      c->insert(std::move(stage));
    }
    {  // Connect to Pour
      auto stage = std::make_unique<stages::Connect>(
          "connect to pour", stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
      stage->setTimeout(connectTimeout);
      stage->properties().configureInitFrom(Stage::PARENT);
      c->insert(std::move(stage));
    }
    {  // Pour
      auto stage = createPour(object, attach_obj_ptr, sampling_planner, cartesian_planner);
      c->insert(std::move(stage));
    }
    {  // Connect to place
      auto stage = std::make_unique<stages::Connect>(
          "connect pick place", stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
      stage->setTimeout(connectTimeout);
      stage->properties().configureInitFrom(Stage::PARENT);
      c->insert(std::move(stage));
    }
    {  // Place
      auto place = SaladTask::createPlace(object, task_, sampling_planner, cartesian_planner, monitored_state_,
                                          attach_obj_ptr, retreat_dir, retreat_dist);
      c->insert(std::move(place));
    }
  }
  return c;
}

std::unique_ptr<SerialContainer> SaladTask::createPickScoopPlace(
    const std::string object, const std::string next_object, const moveit::task_constructor::TaskPtr tptr,
    const moveit::task_constructor::solvers::PipelinePlannerPtr sampling_planner,
    const moveit::task_constructor::solvers::CartesianPathPtr cartesian_planner, Stage*& current_state_ptr)
{
  auto c = std::make_unique<SerialContainer>("PickScoopPlace Path");
  tptr->properties().exposeTo(c->properties(), { "eef", "hand", "group", "ik_frame" });
  {
    Stage* attach_obj_ptr = nullptr;
    {  // Pick
      auto stage = createPick(object, task_, sampling_planner, cartesian_planner, current_state_ptr, attach_obj_ptr);
      c->insert(std::move(stage));
    }
    {  // Connect to place
      auto stage = std::make_unique<stages::Connect>(
          "connect pick place", stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
      stage->setTimeout(connectTimeout);
      stage->properties().configureInitFrom(Stage::PARENT);
      c->insert(std::move(stage));
    }
    {  // Place
      auto place =
          SaladTask::createPlace(object, task_, sampling_planner, cartesian_planner, current_state_ptr, attach_obj_ptr);
      c->insert(std::move(place));
    }
  }
  return c;
}

void SaladTask::init()
{
  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("world", "/move_group/display_grasp_markers"));
  ROS_INFO_NAMED(LOGNAME, "Initializing task pipeline");

  // Reset ROS introspection before constructing the new object
  // TODO(henningkayser): verify this is a bug, fix if possible
  task_.reset();
  task_.reset(new moveit::task_constructor::Task());
  task_->introspection().reset();
  Task& t = *task_;
  t.stages()->setName(task_name_);
  t.loadRobotModel();

  // Sampling planner
  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
  sampling_planner->setProperty("goal_joint_tolerance", goalJointTolerence);

  // Cartesian planner
  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScaling(1.0);
  cartesian_planner->setMaxAccelerationScaling(1.0);
  cartesian_planner->setStepSize(.001);

  // Set task properties
  t.setProperty("group", arm_group_name_);
  t.setProperty("eef", eef_name_);
  t.setProperty("hand", hand_group_name_);
  t.setProperty("hand_grasping_frame", hand_frame_);
  t.setProperty("ik_frame", hand_frame_);

  {  // Current State
    std::unique_ptr<SerialContainer> stage = SaladTask::createCurrentState(sampling_planner);
    t.add(std::move(stage));
  }
  if (collision_objects_.find("ladle1") != collision_objects_.end())
  {
    auto stage = SaladTask::createPickPourPlace("ladle1", sampling_planner, cartesian_planner);
    t.add(std::move(stage));
  }

  if (collision_objects_.find("ladle2") != collision_objects_.end())
  {
    auto stage = SaladTask::createPickPourPlace("ladle2", sampling_planner, cartesian_planner);
    t.add(std::move(stage));
  }

  if (collision_objects_.find("mustard") != collision_objects_.end())
  {
    geometry_msgs::Vector3Stamped vec;
    vec.header.frame_id = hand_frame_;
    vec.vector.x = -1.0;
    // auto stage = createPickPourPlace("mustard", sampling_planner, cartesian_planner, &vec, 0.03);
    auto stage = createPickPourPlace("mustard", sampling_planner, cartesian_planner);
    t.add(std::move(stage));
  }
  {  // Move to home
    auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
    stage->properties().configureInitFrom(Stage::PARENT, { "group" });
    stage->setGoal(arm_home_pose_);
    stage->restrictDirection(stages::MoveTo::FORWARD);
    t.add(std::move(stage));
  }
}

bool SaladTask::plan()
{
  ROS_INFO_NAMED(LOGNAME, "Start searching for task solutions");
  ros::NodeHandle pnh("~");
  int max_solutions = pnh.param<int>("max_solutions", 10);

  try
  {
    task_->plan(max_solutions);
  }
  catch (InitStageException& e)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Initialization failed: " << e);
    return false;
  }
  if (task_->numSolutions() == 0)
  {
    ROS_ERROR_NAMED(LOGNAME, "Planning failed");
    return false;
  }
  ROS_INFO_NAMED(LOGNAME, "SUCCESS!!!!! AYYYYYY");
  return true;
}

bool SaladTask::execute()
{
  ROS_INFO_NAMED(LOGNAME, "Executing solution trajectory");
  moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
  task_->solutions().front()->fillMessage(execute_goal.solution);
  execute_.sendGoal(execute_goal);
  execute_.waitForResult();
  moveit_msgs::MoveItErrorCodes execute_result = execute_.getResult()->error_code;

  if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Task execution failed and returned: " << execute_.getState().toString());
    return false;
  }

  return true;
}
}  // namespace deep_grasp_task
