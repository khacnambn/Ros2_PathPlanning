#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rrt_planner/rrt_planner.hpp"
#include "rrt_planner/rrt.hpp"

namespace rrt_planner
{

RRTPlanner::RRTPlanner()
: initialized_(false)
, tolerance_(0.0)
, is_outline_(false)
, factor_(0.5)
, is_expand_(false)
{
}

void RRTPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  frame_id_ = costmap_ros->getGlobalFrameID();
  costmap_ = costmap_ros->getCostmap();

  // Load parameters
  RCLCPP_INFO(node_->get_logger(), "Configuring %s of type RRTPlanner", name.c_str());

  // General planner params
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".default_tolerance", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name + ".default_tolerance", tolerance_);
  
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".outline_map", rclcpp::ParameterValue(false));
  node_->get_parameter(name + ".outline_map", is_outline_);
  
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".obstacle_factor", rclcpp::ParameterValue(0.5));
  node_->get_parameter(name + ".obstacle_factor", factor_);
  
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".expand_zone", rclcpp::ParameterValue(false));
  node_->get_parameter(name + ".expand_zone", is_expand_);

  int sample_points;
  double sample_max_d;
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".sample_points", rclcpp::ParameterValue(500));
  node_->get_parameter(name + ".sample_points", sample_points);
  
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".sample_max_d", rclcpp::ParameterValue(5.0));
  node_->get_parameter(name + ".sample_max_d", sample_max_d);

  g_planner_ = std::make_shared<rrt_planner::RRT>(costmap_, sample_points, sample_max_d);
  g_planner_->setFactor(factor_);

  plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("plan", 1);
  expand_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("tree", 1);

  // make_plan_srv_ = node_->create_service<nav_msgs::srv::GetPlan>(
  //   "make_plan", std::bind(&RRTPlanner::makePlanService, this,
  //                          std::placeholders::_1, std::placeholders::_2,
  //                          std::placeholders::_3));


  initialized_ = true;
}

void RRTPlanner::cleanup()
{ 
  RCLCPP_INFO(
    node_->get_logger(), "Cleaning up plugin %s of type RRTPlanner",
    name_.c_str());
  
  plan_pub_.reset();
  expand_pub_.reset();
  // make_plan_srv_.reset();
  g_planner_.reset();
  initialized_ = false;
}

void RRTPlanner::activate()
{ 
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type RRTPlanner",
    name_.c_str());

  // plan_pub_->on_activate();
  // expand_pub_->on_activate();
}

void RRTPlanner::deactivate()
{ 
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type RRTPlanner",
    name_.c_str());

  // plan_pub_->on_deactivate();
  // expand_pub_->on_deactivate();
}

nav_msgs::msg::Path RRTPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal)
{

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*g_planner_->getCostMap()->getMutex());

  nav_msgs::msg::Path plan;
  // plan.poses.resize(0);
  if (!initialized_) {
    RCLCPP_ERROR(rclcpp::get_logger("rrt_planner"), "RRTPlanner has not been initialized, unable to create plan");
    return plan;
  }

  if (goal.header.frame_id != frame_id_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("rrt_planner"),
      "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
      frame_id_.c_str(), goal.header.frame_id.c_str());
    return plan;
  }

  if (start.header.frame_id != frame_id_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("rrt_planner"),
      "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
      frame_id_.c_str(), start.header.frame_id.c_str());
    return plan;
  }

  plan.poses.clear();
  plan.header.stamp = node_->now();
  plan.header.frame_id = frame_id_;

  double wx = start.pose.position.x, wy = start.pose.position.y;
  unsigned int g_start_x, g_start_y, g_goal_x, g_goal_y;

  if (!g_planner_->world2Map(wx, wy, g_start_x, g_start_y)) {
    RCLCPP_WARN(
      rclcpp::get_logger("rrt_planner"),
      "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has "
      "been properly localized?");
    return plan;
  }

  wx = goal.pose.position.x;
  wy = goal.pose.position.y;
  if (!g_planner_->world2Map(wx, wy, g_goal_x, g_goal_y)) {
    RCLCPP_WARN(
      rclcpp::get_logger("rrt_planner"),
      "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
    return plan;
  }

  Node n_start(g_start_x, g_start_y, 0, 0, g_planner_->grid2Index(g_start_x, g_start_y), 0);
  Node n_goal(g_goal_x, g_goal_y, 0, 0, g_planner_->grid2Index(g_goal_x, g_goal_y), 0);

  g_planner_->getCostMap()->setCost(g_start_x, g_start_y, 0); // nav2_costmap_2d::FREE_SPACE = 0

  if (is_outline_) {
    g_planner_->outlineMap();
  }

  std::vector<Node> path;
  std::vector<Node> expand;
  bool path_found = g_planner_->plan(n_start, n_goal, path, expand);

  if (path_found) {
    if (_getPlanFromPath(path, plan)) {
      geometry_msgs::msg::PoseStamped goal_copy = goal;
      goal_copy.header.stamp = node_->now();
      plan.poses.push_back(goal_copy);
      history_plan_ = plan;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rrt_planner"), "Failed to get a plan from path when a legal path was found. This shouldn't happen.");
    }
  } else if (!history_plan_.poses.empty()) {
    plan = history_plan_;
    RCLCPP_WARN(rclcpp::get_logger("rrt_planner"), "Planning failed, but returning previously computed plan.");
  } else {
    RCLCPP_WARN(rclcpp::get_logger("rrt_planner"), "Planning failed. No valid path could be found.");
  }

  if (is_expand_) {
    _publishExpand(expand);
  }

  if (plan.poses.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("rrt_planner"), "Generated plan is empty.");
    // return plan;
  }

  publishPlan(plan);
  return plan;
}

void RRTPlanner::publishPlan(const nav_msgs::msg::Path& plan)
{
  if (!initialized_)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("rrt_planner"),
      "This planner has not been initialized yet. Please call initialize() before use.");
    return;
  }

  nav_msgs::msg::Path gui_plan;
  gui_plan.poses.resize(plan.poses.size());
  gui_plan.header.frame_id = frame_id_;
  gui_plan.header.stamp = node_->now();

  for (size_t i = 0; i < plan.poses.size(); ++i)
  {
    gui_plan.poses[i] = plan.poses[i];
  }

  plan_pub_->publish(gui_plan);
}


// bool RRTPlanner::makePlanService(
//     const std::shared_ptr<rmw_request_id_t> request_header,
//     const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
//     std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
// {
//   (void)request_header; // Unused parameter

//   nav_msgs::msg::Path srv_plan = createPlan(request->start, request->goal);
//   response->plan = srv_plan;
//   return true;
// }

bool RRTPlanner::_getPlanFromPath(std::vector<Node> path, nav_msgs::msg::Path& plan)
{
  if (!initialized_)
  {
    RCLCPP_ERROR(node_->get_logger(), "This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }

  plan.poses.clear();
  plan.header.stamp = node_->now();
  plan.header.frame_id = frame_id_;

  for (int i = path.size() - 1; i >= 0; i--)
  {
    double wx, wy;
    g_planner_->map2World((double)path[i].x(), (double)path[i].y(), wx, wy);

    // coding as message type
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = node_->now();
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    plan.poses.push_back(pose);
  }

  return !plan.poses.empty();
}

void RRTPlanner::_publishExpand(std::vector<Node>& expand)
{
  RCLCPP_DEBUG(node_->get_logger(), "Expand Zone Size:%ld", expand.size());

  // Initialize a Marker msg for a LINE_LIST
  visualization_msgs::msg::Marker tree_msg;

  tree_msg.header.frame_id = "map";
  tree_msg.id = 0;
  tree_msg.ns = "tree";
  tree_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
  tree_msg.action = visualization_msgs::msg::Marker::ADD;
  tree_msg.pose.orientation.w = 1.0;
  tree_msg.scale.x = 0.05;

  // Publish all edges
  for (const auto& node : expand) {
    if (node.pid() != 0) {
      _pubLine(tree_msg, expand_pub_, node.id(), node.pid());
    }
  }
}

void RRTPlanner::_pubLine(
  visualization_msgs::msg::Marker& line_msg,
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_pub,
  int id, int pid)
{
  // Update line_msg header
  line_msg.header.stamp = node_->now();

  // Build msg
  geometry_msgs::msg::Point p1, p2;
  std_msgs::msg::ColorRGBA c1, c2;
  int p1x, p1y, p2x, p2y;

  g_planner_->index2Grid(id, p1x, p1y);
  g_planner_->map2World(p1x, p1y, p1.x, p1.y);
  p1.z = 1.0;

  g_planner_->index2Grid(pid, p2x, p2y);
  g_planner_->map2World(p2x, p2y, p2.x, p2.y);
  p2.z = 1.0;

  c1.r = 0.43;
  c1.g = 0.54;
  c1.b = 0.24;
  c1.a = 0.5;

  c2.r = 0.43;
  c2.g = 0.54;
  c2.b = 0.24;
  c2.a = 0.5;

  line_msg.points.push_back(p1);
  line_msg.points.push_back(p2);
  line_msg.colors.push_back(c1);
  line_msg.colors.push_back(c2);

  // Publish line_msg
  line_pub->publish(line_msg);
}

}  // namespace rrt_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav2_core::GlobalPlanner)