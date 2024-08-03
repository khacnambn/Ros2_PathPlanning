/**
 * *********************************************************
 *
 * @file: graph_planner.cpp
 * @brief: Contains the graph planner ROS wrapper class
 * @author: Yang Haodong
 * @date: 2023-10-26
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <tf2/utils.h>

#include "jps_planner/jps_planner.hpp"
#include "jps_planner/jps.hpp"

namespace jps_planner
{
/**
 * @brief Construct a new Graph Planner object
 */
JPSPlanner::JPSPlanner()
:initialized_(false), g_planner_(nullptr)
{
}


/**
 * @brief Planner initialization
 * @param node     shared pointer to rclcpp::Node
 * @param name     planner name
 * @param tf       shared pointer to tf2_ros::Buffer
 * @param costmap_ros shared pointer to costmap_2d::Costmap2DROS
 */
void JPSPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("Unable to lock node");
  }
  
  tf_ = tf;
  frame_id_ = costmap_ros->getGlobalFrameID();
  costmap_ = costmap_ros->getCostmap();

  RCLCPP_INFO(node_->get_logger(), "Configuring %s of type JPSPlanner", name.c_str());

  // Load parameters
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

  g_planner_ = std::make_shared<jps_planner::JumpPointSearch>(costmap_);
  g_planner_->setFactor(factor_);


  // Publisher and service
  plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("plan", 1);
  expand_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("expand", 1);

  // make_plan_srv_ = node_->create_service<nav_msgs::srv::GetPlan>(
  //     "make_plan", std::bind(&JPSPlanner::makePlanService, this, std::placeholders::_1, std::placeholders::_2));

  initialized_ = true;
}

void JPSPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Cleaning up Graph Planner");
  
  plan_pub_.reset();
  expand_pub_.reset();
  // make_plan_srv_.reset();
  g_planner_.reset();
  initialized_ = false;
}

void JPSPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activating Graph Planner");

  // plan_pub_->on_activate();
  // expand_pub_->on_activate();
}

void JPSPlanner::deactivate()
{  
  RCLCPP_INFO(node_->get_logger(), "Deactivating Graph Planner");

  // plan_pub_->on_deactivate();
  // expand_pub_->on_deactivate();
}

/**
 * @brief Plan a path given start and goal in world map
 * @param start start in world map
 * @param goal  goal in world map
 * @param tolerance error tolerance
 * @param plan  plan
 * @return true if find a path successfully, else false
 */
nav_msgs::msg::Path JPSPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal)
{

  nav_msgs::msg::Path plan;
  plan.poses.resize(0);
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*g_planner_->getCostMap()->getMutex());

  if (!initialized_) {
    RCLCPP_ERROR(node_->get_logger(), "This planner has not been initialized yet, but it is being used");
    return plan;
  }

  if (goal.header.frame_id != frame_id_) {
    RCLCPP_ERROR(node_->get_logger(), "The goal pose must be in the %s frame, but is in the %s frame.",
                 frame_id_.c_str(), goal.header.frame_id.c_str());
    return plan;
  }

  if (start.header.frame_id != frame_id_) {
    RCLCPP_ERROR(node_->get_logger(), "The start pose must be in the %s frame, but is in the %s frame.",
                 frame_id_.c_str(), start.header.frame_id.c_str());
    return plan;
  }

  plan.poses.clear();
  plan.header.stamp = node_->now();
  plan.header.frame_id = frame_id_;

  double wx = start.pose.position.x, wy = start.pose.position.y;
  unsigned int g_start_x, g_start_y, g_goal_x, g_goal_y;
  if (!g_planner_->world2Map(wx, wy, g_start_x, g_start_y)) {
    RCLCPP_WARN(node_->get_logger(), "The start position is off the global costmap.");
    return plan;
  }
  wx = goal.pose.position.x; 
  wy = goal.pose.position.y;
  if (!g_planner_->world2Map(wx, wy, g_goal_x, g_goal_y)) {
    RCLCPP_WARN(node_->get_logger(), "The goal position is off the global costmap.");
    return plan;
  }

  Node start_node(g_start_x, g_start_y, 0, 0, g_planner_->grid2Index(g_start_x, g_start_y), 0);
  Node goal_node(g_goal_x, g_goal_y, 0, 0, g_planner_->grid2Index(g_goal_x, g_goal_y), 0);

  if (is_outline_) {
    g_planner_->outlineMap();
  }

  std::vector<Node> path;
  std::vector<Node> expand;
  bool path_found = g_planner_->plan(start_node, goal_node, path, expand);

  if (path_found) {
    if (_getPlanFromPath(path, plan)) {
      geometry_msgs::msg::PoseStamped goal_copy = goal;
      goal_copy.header.stamp = node_->now();
      plan.poses.push_back(goal_copy);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get a plan from path when a legal path was found.");
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get a path.");
  }

  if (is_expand_) {
    _publishExpand(expand);
  }

  publishPlan(plan);

  return plan;
}

/**
 * @brief publish planning path
 * @param path planning path
 */
void JPSPlanner::publishPlan(const nav_msgs::msg::Path& plan)
{

  if (!initialized_) {
    RCLCPP_ERROR(node_->get_logger(), "This planner has not been initialized yet, but it is being used");
    return;
  }

  nav_msgs::msg::Path gui_plan;
  gui_plan.poses.resize(plan.poses.size());
  gui_plan.header.frame_id = frame_id_;
  gui_plan.header.stamp = node_->now();
  for (unsigned int i = 0; i < plan.poses.size(); i++) {
    gui_plan.poses[i] = plan.poses[i];
  }

  plan_pub_->publish(gui_plan);
}
// /**
//  * @brief Regeister planning service
//  * @param req  request from client
//  * @param resp response from server
//  * @return true
//  */
// bool GraphPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
// {
//   makePlan(req.start, req.goal, resp.plan.poses);
//   resp.plan.header.stamp = ros::Time::now();
//   resp.plan.header.frame_id = frame_id_;

//   return true;
// }

/**
 * @brief publish expand zone
 * @param expand set of expand nodes
 */
void JPSPlanner::_publishExpand(std::vector<Node>& expand)
{

  RCLCPP_DEBUG(node_->get_logger(), "Expand Zone Size: %ld", expand.size());

  nav_msgs::msg::OccupancyGrid grid;
  float resolution = g_planner_->getCostMap()->getResolution();

  grid.header.frame_id = frame_id_;
  grid.header.stamp = node_->now();
  grid.info.resolution = resolution;
  grid.info.width = g_planner_->getCostMap()->getSizeInCellsX();
  grid.info.height = g_planner_->getCostMap()->getSizeInCellsY();

  double wx, wy;
  g_planner_->getCostMap()->mapToWorld(0, 0, wx, wy);
  grid.info.origin.position.x = wx - resolution / 2;
  grid.info.origin.position.y = wy - resolution / 2;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.resize(g_planner_->getMapSize());

  for (unsigned int i = 0; i < grid.data.size(); i++) {
    grid.data[i] = 0;
  }
  for (unsigned int i = 0; i < expand.size(); i++) {
    grid.data[expand[i].id()] = 50;
  }

  expand_pub_->publish(grid);
}

/**
 * @brief Calculate plan from planning path
 * @param path path generated by global planner
 * @param plan plan transfromed from path, i.e. [start, ..., goal]
 * @return bool true if successful, else false
 */
bool JPSPlanner::_getPlanFromPath(std::vector<Node>& path, nav_msgs::msg::Path& plan)
{

  if (!initialized_) {
    RCLCPP_ERROR(node_->get_logger(), "This planner has not been initialized yet, but it is being used");
    return false;
  }

  plan.poses.clear();
  plan.header.stamp = node_->now();
  plan.header.frame_id = frame_id_;

  for (int i = path.size() - 1; i >= 0; i--) {
    double wx, wy;
    g_planner_->map2World(path[i].x(), path[i].y(), wx, wy);

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
}  // namespace jps_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jps_planner::JPSPlanner, nav2_core::GlobalPlanner)