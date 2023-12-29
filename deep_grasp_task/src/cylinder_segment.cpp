#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <deep_grasp_msgs/action/cylinder_segment.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using GoalHandleSharedPtr =
    typename std::shared_ptr<rclcpp_action::ServerGoalHandle<deep_grasp_msgs::action::CylinderSegment>>;

class CylinderSegmentor
{
public:
  CylinderSegmentor(rclcpp::Node::SharedPtr action_node, rclcpp::Node::SharedPtr subscription_node);
  void loadParameters();
  void init();

  /** \brief Given the parameters of the cylinder add the cylinder to the planning scene. */
  void addCylinder();

  /** \brief Given the pointcloud containing just the cylinder,
      compute its center point and its height and store in cylinder_params.
      @param cloud - point cloud containing just the cylinder. */
  void extractLocatioaction_nodeheight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  /** \brief Given a pointcloud extract the ROI defined by the user.
      @param cloud - Pointcloud whose ROI needs to be extracted. */
  void passThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  /** \brief Given the pointcloud and pointer cloud_normals compute the point normals and store in cloud_normals.
      @param cloud - Pointcloud.
      @param cloud_normals - The point normals once computer will be stored in this. */
  void computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals);

  /** \brief Given the point normals and point indices, extract the normals for the indices.
      @param cloud_normals - Point normals.
      @param inliers_plane - Indices whose normals need to be extracted. */
  void extractNormals(const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                      const pcl::PointIndices::Ptr& inliers_plane);

  /** \brief Given the pointcloud and indices of the plane, remove the planar region from the pointcloud.
      @param cloud - Pointcloud.
      @param inliers_plane - Indices representing the plane. */
  void removePlaneSurface(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          const pcl::PointIndices::Ptr& inliers_plane);

  /** \brief Given the pointcloud, pointer to pcl::ModelCoefficients and point normals extract the cylinder from the
     pointcloud and store the cylinder parameters in coefficients_cylinder.
      @param cloud - Pointcloud whose plane is removed.
      @param coefficients_cylinder - Cylinder parameters used to define an infinite cylinder will be stored here.
      @param cloud_normals - Point normals corresponding to the plane on which cylinder is kept */
  void extractCylinder(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       const pcl::ModelCoefficients::Ptr& coefficients_cylinder,
                       const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals);

  rclcpp_action::CancelResponse handle_cancel(const GoalHandleSharedPtr goal_handle);
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const deep_grasp_msgs::action::CylinderSegment_Goal> goal);
  void handle_accepted(const GoalHandleSharedPtr& goal_handle);
  void cloudCB(const sensor_msgs::msg::PointCloud2& input);
  void execute(const GoalHandleSharedPtr& goal_handle);

private:
  rclcpp::Node::SharedPtr action_node_;
  rclcpp::Node::SharedPtr subscription_node_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  struct AddCylinderParams
  {
    /* Radius of the cylinder. */
    double radius;
    /* Direction vector along the z-axis of the cylinder. */
    double direction_vec[3];
    /* Center point of the cylinder. */
    double center_pt[3];
    /* Height of the cylinder. */
    double height;
  };
  AddCylinderParams cylinder_params_;
  bool goal_active_ = false;
  bool add_cylinder_ = false;
  bool new_cylinder_found_ = false;
  deep_grasp_msgs::action::CylinderSegment_Goal goal_;

  rclcpp_action::Server<deep_grasp_msgs::action::CylinderSegment>::SharedPtr server_;
  deep_grasp_msgs::action::CylinderSegment::Result::SharedPtr result_;
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  std::shared_ptr<tf2_ros::CreateTimerROS> timer_interface_;

};

CylinderSegmentor::CylinderSegmentor(rclcpp::Node::SharedPtr action_node, rclcpp::Node::SharedPtr subscription_node)
  : action_node_(action_node), subscription_node_(subscription_node)
{
  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(action_node_->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  timer_interface_ = std::make_shared<tf2_ros::CreateTimerROS>(action_node_->get_node_base_interface(),
                                                               action_node_->get_node_timers_interface());
  tfBuffer_->setCreateTimerInterface(timer_interface_);
  result_ = std::make_shared<deep_grasp_msgs::action::CylinderSegment::Result>();

  loadParameters();
  init();
}

void CylinderSegmentor::loadParameters()
{
  RCLCPP_INFO(action_node_->get_logger(), "Loading cylinder segment params");

  action_node_->get_parameter("goal_active", goal_active_);
  action_node_->get_parameter("add_cylinder", add_cylinder_);
}

void CylinderSegmentor::init()
{
  using namespace std::placeholders;
  server_ = rclcpp_action::create_server<deep_grasp_msgs::action::CylinderSegment>(
      action_node_, "cylinder_segment", std::bind(&CylinderSegmentor::handle_goal, this, _1, _2),
      std::bind(&CylinderSegmentor::handle_cancel, this, _1), std::bind(&CylinderSegmentor::handle_accepted, this, _1));

  subscription_ = subscription_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rgbd_camera/points", 10, std::bind(&CylinderSegmentor::cloudCB, this, _1));
}

rclcpp_action::CancelResponse CylinderSegmentor::handle_cancel(const GoalHandleSharedPtr goal_handle)
{
  RCLCPP_INFO(action_node_->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::GoalResponse CylinderSegmentor::handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const deep_grasp_msgs::action::CylinderSegment_Goal> goal)
{
  (void)uuid, (void)goal;
  RCLCPP_INFO(action_node_->get_logger(), "New goal accepted");
  goal_active_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void CylinderSegmentor::execute(const GoalHandleSharedPtr& goal_handle)
{
  rclcpp::Rate loop_rate(1);
  while (!new_cylinder_found_ && rclcpp::ok())
  {
    RCLCPP_INFO_ONCE(action_node_->get_logger(), "Waiting for new cylinder");
  }
  RCLCPP_INFO(action_node_->get_logger(),
              new_cylinder_found_ ? "New cylinder found2: true" : "new cylinder found2 false");
  goal_active_ = false;
  new_cylinder_found_ = false;

  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "camera_locobot_link";

  // Setting the position of cylinder.
  p.pose.position.x = cylinder_params_.center_pt[0];
  p.pose.position.y = cylinder_params_.center_pt[1];
  p.pose.position.z = cylinder_params_.center_pt[2];

  Eigen::Vector3d cylinder_z_direction(cylinder_params_.direction_vec[0], cylinder_params_.direction_vec[1],
                                       cylinder_params_.direction_vec[2]);
  Eigen::Vector3d origin_z_direction(0., 0., 1.);
  Eigen::Vector3d axis;
  axis = origin_z_direction.cross(cylinder_z_direction);
  axis.normalize();
  double angle = acos(cylinder_z_direction.dot(origin_z_direction));
  p.pose.orientation.x = axis.x() * sin(angle / 2);
  p.pose.orientation.y = axis.y() * sin(angle / 2);
  p.pose.orientation.z = axis.z() * sin(angle / 2);
  p.pose.orientation.w = cos(angle / 2);

  RCLCPP_INFO(action_node_->get_logger(), "com pose: ( %.2f %.2f %.2f)", p.pose.position.x, p.pose.position.y,
              p.pose.position.z);

  geometry_msgs::msg::TransformStamped tf_world_opt;
  try
  {
    tfBuffer_->waitForTransform(
        "world", "camera_locobot_link", action_node_->now(), rclcpp::Duration(5, 0),
        [&tf_world_opt](const tf2_ros::TransformStampedFuture& tf) { tf_world_opt = tf.get(); });

    tf2::doTransform(p, result_->com, tf_world_opt);
    RCLCPP_INFO(action_node_->get_logger(), "%s com pose: ( %.2f %.2f %.2f)", result_->com.header.frame_id.c_str(),
                result_->com.pose.position.x, result_->com.pose.position.y, result_->com.pose.position.z);
    result_->com.pose.position.x += 0.01 * (int)(result_->com.pose.position.x > 0);
    result_->com.pose.position.y += 0.005 * (int)(result_->com.pose.position.y > 0);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(action_node_->get_logger(), "%s", ex.what());
    RCLCPP_ERROR(action_node_->get_logger(), "TF FAILED!!\n");
    goal_handle->abort(result_);
  }
  if (rclcpp::ok())
  {
    goal_handle->succeed(result_);
  }
}
void CylinderSegmentor::handle_accepted(const GoalHandleSharedPtr& goal_handle)
{
  std::thread{ std::bind(&CylinderSegmentor::execute, this, std::placeholders::_1), goal_handle }.detach();
}

void CylinderSegmentor::cloudCB(const sensor_msgs::msg::PointCloud2& input)
{
  if (goal_active_)
  {
  

    RCLCPP_INFO(subscription_node_->get_logger(), "Goal active extracting cylinder");

    // This section uses a standard PCL-based processing pipeline to estimate a cylinder's pose in the point cloud.
    //
    // First, we convert from sensor_msgs to pcl::PointXYZ which is needed for most of the processing.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(input, *cloud);
    RCLCPP_WARN_STREAM(subscription_node_->get_logger(), "cloud1 : " << *cloud);
    // Use a passthrough filter to get the region of interest.
    // The filter removes points outside the specified range.
    // passThroughFilter(cloud);
      RCLCPP_WARN_STREAM(subscription_node_->get_logger(), "cloud2 : " << *cloud);
    // Compute point normals for later sample consensus step.
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    computeNormals(cloud, cloud_normals);
    // inliers_plane will hold the indices of the point cloud that correspond to a plane.
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

    // Detect and remove points on the (planar) surface on which the cylinder is resting.
    removePlaneSurface(cloud, inliers_plane);
      RCLCPP_WARN_STREAM(subscription_node_->get_logger(), "cloud3 : " << *cloud);
    // Remove surface points from normals as well
    extractNormals(cloud_normals, inliers_plane);
      RCLCPP_WARN_STREAM(subscription_node_->get_logger(), "cloud4 : " << *cloud);
    // ModelCoefficients will hold the parameters using which we can define a cylinder of infinite length.
    // It has a public attribute values of type std::vector<float> .
    // values[0-2] hold a point on the center line of the cylinder. 
    // values[3-5] hold direction vector of the z-axis.
    //  values[6] is the radius of the cylinder.
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    /* Extract the cylinder using SACSegmentation. */
    extractCylinder(cloud, coefficients_cylinder, cloud_normals);

    bool internal_found_new_cylinder = false;
    // END_SUB_TUTORIAL
    if (cloud->points.empty() || coefficients_cylinder->values.size() != 7)
    {
      RCLCPP_ERROR(subscription_node_->get_logger(), "Can't find the cylindrical component.");
      internal_found_new_cylinder = false;
      return;
    }
    else
    {
      internal_found_new_cylinder = true;
      RCLCPP_INFO(subscription_node_->get_logger(), "Detected Cylinder - Adding CollisionObject to PlanningScene");
    }

    /* Store the radius of the cylinder. */
    cylinder_params_.radius = coefficients_cylinder->values[6];
    /* Store direction vector of z-axis of cylinder. */
    cylinder_params_.direction_vec[0] = coefficients_cylinder->values[3];
    cylinder_params_.direction_vec[1] = coefficients_cylinder->values[4];
    cylinder_params_.direction_vec[2] = coefficients_cylinder->values[5];
    //
    // Compute the center point of the cylinder using standard geometry
    extractLocatioaction_nodeheight(cloud);
    new_cylinder_found_ = internal_found_new_cylinder;

    if (add_cylinder_)
    {
      addCylinder();
    }
  }
}

/** \brief Given the parameters of the cylinder add the cylinder to the planning scene. */
void CylinderSegmentor::addCylinder()
{

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "locobot_camera_depth_link";
  collision_object.id = "cylinder";

  // Define a cylinder which will be added to the world.
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  /* Setting height of cylinder. */
  primitive.dimensions[0] = cylinder_params_.height;
  /* Setting radius of cylinder. */
  primitive.dimensions[1] = cylinder_params_.radius;

  // Define a pose for the cylinder (specified relative to frame_id).
  geometry_msgs::msg::Pose cylinder_pose;
  /* Computing and setting quaternion from axis angle representation. */
  Eigen::Vector3d cylinder_z_direction(cylinder_params_.direction_vec[0], cylinder_params_.direction_vec[1],
                                       cylinder_params_.direction_vec[2]);
  Eigen::Vector3d origin_z_direction(0., 0., 1.);
  Eigen::Vector3d axis;
  axis = origin_z_direction.cross(cylinder_z_direction);
  axis.normalize();
  double angle = acos(cylinder_z_direction.dot(origin_z_direction));
  cylinder_pose.orientation.x = axis.x() * sin(angle / 2);
  cylinder_pose.orientation.y = axis.y() * sin(angle / 2);
  cylinder_pose.orientation.z = axis.z() * sin(angle / 2);
  cylinder_pose.orientation.w = cos(angle / 2);

  // Setting the position of cylinder.
  cylinder_pose.position.x = cylinder_params_.center_pt[0];
  cylinder_pose.position.y = cylinder_params_.center_pt[1];
  cylinder_pose.position.z = cylinder_params_.center_pt[2];

  // Add cylinder as collision object
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(cylinder_pose);
  collision_object.operation = collision_object.ADD;
  planning_scene_interface_.applyCollisionObject(collision_object);
  // END_SUB_TUTORIAL
}

/** \brief Given the pointcloud containing just the cylinder,
    compute its center point and its height and store in cylinder_params.
    @param cloud - point cloud containing just the cylinder. */
void CylinderSegmentor::extractLocatioaction_nodeheight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  double max_angle_y = -std::numeric_limits<double>::infinity();
  double min_angle_y = std::numeric_limits<double>::infinity();

  double lowest_point[3] = { 0.0, 0.0, 0.0 };
  double highest_point[3] = { 0.0, 0.0, 0.0 };
  // Consider a point inside the point cloud and imagine that point is formed on a XY plane where the perpendicular
  // distance from the plane to the camera is Z. 
  // The perpendicular drawn from the camera to the plane hits at center of the XY plane. 
  // We have the x and y coordinate of the point which is formed on the XY plane. 
  // X is the horizontal axis and Y is the vertical axis. 
  // C is the center of the plane which is Z meter away from the center of camera and A is any point on the plane.
  // 
  // Now we know Z is the perpendicular distance from the point to the camera. 
  // If you need to find the  actual distance d from the point to the camera, you should calculate the hypotenuse-
  //  hypot(point.z, point.x)
  // angle the point made horizontally-  atan2(point.z,point.x)
  // angle the point made Vertically-  atan2(point.z, point.y)
  // Loop over the entire pointcloud.
  for (auto const point : cloud->points)
  {
    const double angle = atan2(point.z, point.y);
    /* Find the coordinates of the highest point */
    if (angle < min_angle_y)
    {
      min_angle_y = angle;
      lowest_point[0] = point.x;
      lowest_point[1] = point.y;
      lowest_point[2] = point.z;
    }
    /* Find the coordinates of the lowest point */
    else if (angle > max_angle_y)
    {
      max_angle_y = angle;
      highest_point[0] = point.x;
      highest_point[1] = point.y;
      highest_point[2] = point.z;
    }
  }
  /* Store the center point of cylinder */
  cylinder_params_.center_pt[0] = (highest_point[0] + lowest_point[0]) / 2;
  cylinder_params_.center_pt[1] = (highest_point[1] + lowest_point[1]) / 2;
  cylinder_params_.center_pt[2] = (highest_point[2] + lowest_point[2]) / 2;
  /* Store the height of cylinder */
  cylinder_params_.height =
      sqrt(pow((lowest_point[0] - highest_point[0]), 2) + pow((lowest_point[1] - highest_point[1]), 2) +
           pow((lowest_point[2] - highest_point[2]), 2));
}

/** \brief Given a pointcloud extract the ROI defined by the user.
    @param cloud - Pointcloud whose ROI needs to be extracted. */
void CylinderSegmentor::passThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  geometry_msgs::msg::TransformStamped tf_camera_opt;
  geometry_msgs::msg::Point32 zmin, zmin_out;
  zmin.z = 0.3;
  tfBuffer_->waitForTransform(
    "locobot_camera_depth_link", "camera_locobot_link", action_node_->now(), rclcpp::Duration(5, 0),
    [&tf_camera_opt](const tf2_ros::TransformStampedFuture& tf) { tf_camera_opt = tf.get(); });
  tf2::doTransform(zmin, zmin_out, tf_camera_opt);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  for (auto& p : cloud->points) {
    RCLCPP_ERROR_ONCE(subscription_node_->get_logger(), "point: %f %f %f", p.x, p.y, p.z);
  }
  RCLCPP_ERROR_ONCE(subscription_node_->get_logger(), "zmin: %f", zmin_out.z);
  pass.setFilterFieldName("z");
  // min and max values in z axis to keep
  pass.setFilterLimits(0, 6);
  pass.filter(*cloud);
}

/** \brief Given the pointcloud and pointer cloud_normals compute the point normals and store in cloud_normals.
    @param cloud - Pointcloud.
    @param cloud_normals - The point normals once computer will be stored in this. */
void CylinderSegmentor::computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                       const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  // Set the number of k nearest neighbors to use for the feature estimation.
  ne.setKSearch(50);
  ne.compute(*cloud_normals);
}

/** \brief Given the point normals and point indices, extract the normals for the indices.
    @param cloud_normals - Point normals.
    @param inliers_plane - Indices whose normals need to be extracted. */
void CylinderSegmentor::extractNormals(const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                                       const pcl::PointIndices::Ptr& inliers_plane)
{
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  extract_normals.setNegative(true);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*cloud_normals);
}

/** \brief Given the pointcloud and indices of the plane, remove the planar region from the pointcloud.
    @param cloud - Pointcloud.
    @param inliers_plane - Indices representing the plane. */
void CylinderSegmentor::removePlaneSurface(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                           const pcl::PointIndices::Ptr& inliers_plane)
{
  // create a SAC segmenter without using normals
  pcl::SACSegmentation<pcl::PointXYZ> segmentor;
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_PLANE);
  segmentor.setMethodType(pcl::SAC_RANSAC);
  /* run at max 1000 iterations before giving up */
  segmentor.setMaxIterations(1000);
  /* tolerance for variation from model */
  segmentor.setDistanceThreshold(0.01);
  segmentor.setInputCloud(cloud);
  /* Create the segmentation object for the planar model and set all the parameters */
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  segmentor.segment(*inliers_plane, *coefficients_plane);
  /* Extract the planar inliers from the input cloud */
  pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
  extract_indices.setInputCloud(cloud);
  extract_indices.setIndices(inliers_plane);
  /* Remove the planar inliers, extract the rest */
  extract_indices.setNegative(true);
  extract_indices.filter(*cloud);
  
}

/** \brief Given the pointcloud, pointer to pcl::ModelCoefficients and point normals extract the cylinder from the
   pointcloud and store the cylinder parameters in coefficients_cylinder.
    @param cloud - Pointcloud whose plane is removed.
    @param coefficients_cylinder - Cylinder parameters used to define an infinite cylinder will be stored here.
    @param cloud_normals - Point normals corresponding to the plane on which cylinder is kept */
void CylinderSegmentor::extractCylinder(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                        const pcl::ModelCoefficients::Ptr& coefficients_cylinder,
                                        const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals)
{
  // Create the segmentation object for cylinder segmentation and set all the parameters
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> segmentor;
  pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_CYLINDER);
  segmentor.setMethodType(pcl::SAC_RANSAC);
  // Set the normal angular distance weight
  segmentor.setNormalDistanceWeight(0.1);
  // run at max 1000 iterations before giving up
  segmentor.setMaxIterations(1000);
  // tolerance for variation from model
  segmentor.setDistanceThreshold(0.008);
  // min max values of radius in meters to consider
  segmentor.setRadiusLimits(0.01, 0.1);
  segmentor.setInputCloud(cloud);
  segmentor.setInputNormals(cloud_normals);

  // Obtain the cylinder inliers and coefficients
  segmentor.segment(*inliers_cylinder, *coefficients_cylinder);

  // Extract the cylinder inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers_cylinder);
  extract.setNegative(false);
  extract.filter(*cloud);
}
// END_SUB_TUTORIALegment

int main(int argc, char** argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.allow_undeclared_parameters(true);
  auto node = rclcpp::Node::make_shared("cylinder_segment", node_options);
  auto subscription_node = rclcpp::Node::make_shared("cylinder_segment_subscription", node_options);
  // Start the segmentor
  std::thread action_thread([node] { rclcpp::spin(node); });
  std::thread subscription_thread([subscription_node] { rclcpp::spin(subscription_node); });
  CylinderSegmentor segmentor(node, subscription_node);
  action_thread.join();
  subscription_thread.join();
  rclcpp::shutdown();
  return 0;
}