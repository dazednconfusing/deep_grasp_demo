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
#include <rosparam_shortcuts/rosparam_shortcuts.h>


namespace deep_grasp_task
{
constexpr char LOGNAME[] = "pick_place_task";
DeepPickPlaceTask::DeepPickPlaceTask(const std::string& task_name,
  const ros::NodeHandle& nh)
  : nh_(nh), task_name_(task_name), execute_("execute_task_solution", true)
{
}
DeepPickPlaceTask::DeepPickPlaceTask(const std::string& task_name,
  const ros::NodeHandle& nh, const std::string& object)
  : nh_(nh), task_name_(task_name), execute_("execute_task_solution", true)
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
void DeepPickPlaceTask::loadParameters(const std::string& object, const std::string& allowed_collision)
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

  // Target object
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, object + "_name", object_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, object + "_dimensions", object_dimensions_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "surface_link", surface_link_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, object + "_start_surface", start_surface_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, object + "_end_surface", end_surface_);
  support_surfaces_ = { surface_link_ };

  // Deep grasp properties
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "action_name", action_name_);

  // Pick/Place metrics
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_object_min_dist", approach_object_min_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_object_max_dist", approach_object_max_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lower_object_min_dist", lower_object_min_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lower_object_max_dist", lower_object_max_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_object_min_dist", lift_object_min_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_object_max_dist", lift_object_max_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "place_surface_offset", place_surface_offset_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, object + "_place_pose", place_pose_);

  allowed_collision_ = std::string(allowed_collision);
  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

void DeepPickPlaceTask::init()
{
  ROS_INFO_NAMED(LOGNAME, "Initializing task pipeline");
  const std::string object = object_name_;
  ROS_INFO_STREAM_NAMED(LOGNAME, "OBJECT NAME: " << object << ", ALLOWED COLLISION: " << allowed_collision_);

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
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

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
      geometry_msgs::Vector3Stamped vec;
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
      auto stage = std::make_unique<stages::DeepGraspPose<moveit_task_constructor_msgs::SampleGraspPosesAction>>(
          action_name_, "generate grasp pose");
      stage->properties().configureInitFrom(Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose(hand_open_pose_);
      stage->setObject(object);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state

      // Compute IK
      auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
      wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));

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
      geometry_msgs::Vector3Stamped vec;
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
      geometry_msgs::Vector3Stamped vec;
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
      geometry_msgs::PoseStamped p;
      p.header.frame_id = object_reference_frame_;
      p.pose = place_pose_;
      p.pose.position.z += 0.5 * object_dimensions_.at(0) + place_surface_offset_;
      // ROS_WARN_STREAM_NAMED(LOGNAME, " PARAM place pose: " << p.pose.position.x << "," << p.pose.position.y << "," << p.pose.position.z);
      stage->setPose(p);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage
      // stage->addSolutionCallback([](const SolutionBase& s) {
      //   planning_scene::PlanningSceneConstPtr scene = s.end()->scene();
      //   const moveit::core::RobotState& robot_state_ = scene->getCurrentState();

      //   std::vector<const moveit::core::AttachedBody*> attached_bodies;

      //   robot_state_.getAttachedBodies(attached_bodies);

      //   if (attached_bodies.size() > 0) {
      //     ROS_WARN_STREAM_NAMED(LOGNAME, " num attached bodies: " << attached_bodies.size());
      //     const auto* body = attached_bodies[0];
      //     // current object_pose w.r.t. planning frame
      //     const Eigen::Isometry3d& orig_object_pose = body->getGlobalPose();
      //     const auto& pose_ = orig_object_pose.translation();
      //     ROS_WARN_STREAM_NAMED(LOGNAME, body->getName() << " place pose: " << pose_.x() << "," << pose_.y() << "," << pose_.z());
      //   }
      //   });

      // Compute IK
      auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
      wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
      // wrapper->addSolutionCallback([](const SolutionBase& s) {
      //   planning_scene::PlanningSceneConstPtr scene = s.end()->scene();
      //   const moveit::core::RobotState& robot_state_ = scene->getCurrentState();

      //   std::vector<const moveit::core::AttachedBody*> attached_bodies;

      //   robot_state_.getAttachedBodies(attached_bodies);

      //   if (attached_bodies.size() > 0) {
      //     ROS_WARN_STREAM_NAMED(LOGNAME, " num attached bodies: " << attached_bodies.size());
      //     const auto* body = attached_bodies[0];
      //     // current object_pose w.r.t. planning frame
      //     const Eigen::Isometry3d& orig_object_pose = body->getGlobalPose();
      //     const auto& pose_ = orig_object_pose.translation();
      //     ROS_WARN_STREAM_NAMED(LOGNAME, body->getName() << " IK place pose: " << pose_.x() << "," << pose_.y() << "," << pose_.z());
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
      geometry_msgs::Vector3Stamped vec;
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

bool DeepPickPlaceTask::execute()
{
  // moveit_msgs::MoveItErrorCodes execute_result;
  // execute_result = task_->execute(*task_->solutions().front());
  // // // If you want to inspect the goal message, use this instead:
  // // actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction>
  // // execute("execute_task_solution", true); execute.waitForServer();
  // // moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
  // // task_->solutions().front()->fillMessage(execute_goal.solution);
  // // execute.sendGoalAndWait(execute_goal);
  // // execute_result = execute.getResult()->error_code;

  // if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
  //   ROS_ERROR_STREAM("Task execution failed and returned: " << execute_result.val);
  // }
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