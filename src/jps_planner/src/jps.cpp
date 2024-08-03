/**
 * *********************************************************
 *
 * @file: jump_point_search.cpp
 * @brief: Contains the Jump Point Search(JPS) planner class
 * @author: Yang Haodong
 * @date: 2023-12-14
 * @version: 1.1
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "jps_planner/jps.hpp"

namespace jps_planner
{
/**
 * @brief Constructor
 * @param costmap   the environment for path planning
 */
JumpPointSearch::JumpPointSearch(nav2_costmap_2d::Costmap2D* costmap) : PlannerCore(costmap)
{
}

/**
 * @brief Jump Point Search(JPS) implementation
 * @param start          start node
 * @param goal           goal node
 * @param expand         containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool JumpPointSearch::plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
  // Copy start and goal
  start_ = start;
  goal_ = goal;

  // Clear vectors
  path.clear();
  expand.clear();

  // Log start and goal nodes
  RCLCPP_INFO(rclcpp::get_logger("jps_planner"), "Start node: (%d, %d), Goal node: (%d, %d)", start.x(), start.y(), goal.x(), goal.y());

  // Open list and closed list
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  std::unordered_map<int, Node> closed_list;

  // Add start node to open list
  open_list.push(start);

  // Check if the start node is correctly added to the open list
  if (open_list.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("jps_planner"), "Start node not added to the open list. Start node: (%d, %d)", start.x(), start.y());
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("jps_planner"), "Start node added to open list");

  // Get all possible motions
  std::vector<Node> motions = Node::getMotion();

  // Main loop
  while (!open_list.empty())
  {
    RCLCPP_INFO(rclcpp::get_logger("jps_planner"), "Open list not empty");

    // Pop current node from open list
    Node current = open_list.top();
    open_list.pop();

    RCLCPP_INFO(rclcpp::get_logger("jps_planner"), "Current node: (%d, %d)", current.x(), current.y());

    // Check if the current node is already in the closed list
    if (closed_list.find(current.id()) != closed_list.end())
      continue;

    // Add current node to closed list
    closed_list.insert(std::make_pair(current.id(), current));
    expand.push_back(current);

    // Check if the goal is found
    if (current == goal)
    {
      RCLCPP_INFO(rclcpp::get_logger("jps_planner"), "Goal found at node: (%d, %d)", current.x(), current.y());
      path = _convertClosedListToPath(closed_list, start, goal);
      return true;
    }

    // Explore neighbors of the current node
    for (const auto& motion : motions)
    {
      Node node_new = jump(current, motion);

      // Log the new node details
      RCLCPP_INFO(rclcpp::get_logger("jps_planner"), "Checking new node: (%d, %d)", node_new.x(), node_new.y());

      // Check if the new node is valid and not in the closed list
      if (node_new.id() == -1 || closed_list.find(node_new.id()) != closed_list.end())
        continue;

      node_new.set_pid(current.id());
      open_list.push(node_new);

      RCLCPP_INFO(rclcpp::get_logger("jps_planner"), "Added node (%d, %d) to open list with cost: %f", node_new.x(), node_new.y(), node_new.g());
    }
  }

  RCLCPP_WARN(rclcpp::get_logger("jps_planner"), "No path found from (%d, %d) to (%d, %d)", start.x(), start.y(), goal.x(), goal.y());
  return false;
}

/**
 * @brief Calculate jump node recursively
 * @param point  current node
 * @param motion the motion that current node executes
 * @return jump node
 */
Node JumpPointSearch::jump(const Node& point, const Node& motion)
{
  Node new_point = point + motion;
  new_point.set_id(grid2Index(new_point.x(), new_point.y()));
  new_point.set_pid(point.id());
  new_point.set_h(helper::dist(new_point, goal_));

  // RCLCPP_INFO(rclcpp::get_logger("jps_planner"), "Jumping from (%d, %d) to (%d, %d)", point.x(), point.y(), new_point.x(), new_point.y());

  // Check if the next node hits the boundary or an obstacle
  if (new_point.id() < 0 || new_point.id() >= map_size_ || costmap_->getCharMap()[new_point.id()] >= 254 * factor_)
  {
    // RCLCPP_INFO(rclcpp::get_logger("jps_planner"), "Node (%d, %d) hit boundary or obstacle", new_point.x(), new_point.y());
    return Node(-1, -1, -1, -1, -1, -1);
  }

  // Goal found
  if (new_point == goal_)
  {
    // RCLCPP_INFO(rclcpp::get_logger("jps_planner"), "Goal node reached: (%d, %d)", new_point.x(), new_point.y());
    return new_point;
  }

  // Diagonal motion
  if (motion.x() && motion.y())
  {
    Node x_dir = Node(motion.x(), 0, 1, 0, 0, 0);
    Node y_dir = Node(0, motion.y(), 1, 0, 0, 0);
    if (jump(new_point, x_dir).id() != -1 || jump(new_point, y_dir).id() != -1)
      return new_point;
  }

  // Check for forced neighbors
  if (detectForceNeighbor(new_point, motion))
  {
    // RCLCPP_INFO(rclcpp::get_logger("jps_planner"), "Forced neighbor detected at (%d, %d)", new_point.x(), new_point.y());
    return new_point;
  }
  else
  {
    return jump(new_point, motion);
  }
}


/**
 * @brief Detect whether current node has forced neighbors
 * @param point  current node
 * @param motion the motion that current node executes
 * @return true if current node has forced neighbor else false
 */
bool JumpPointSearch::detectForceNeighbor(const Node& point, const Node& motion)
{
  int x = point.x();
  int y = point.y();
  int x_dir = motion.x();
  int y_dir = motion.y();
  auto costs = costmap_->getCharMap();

  // horizontal
  if (x_dir && !y_dir)
  {
    if (costs[grid2Index(x, y + 1)] >= 254 * factor_ &&
        costs[grid2Index(x + x_dir, y + 1)] < 254 * factor_) //nav2_costmap_2d::LETHAL_OBSTACLE = 254
      return true;
    if (costs[grid2Index(x, y - 1)] >= 254 * factor_ &&
        costs[grid2Index(x + x_dir, y - 1)] < 254 * factor_) //nav2_costmap_2d::LETHAL_OBSTACLE = 254
      return true;
  }

  // vertical
  if (!x_dir && y_dir)
  {
    if (costs[grid2Index(x + 1, y)] >= 254 * factor_ &&
        costs[grid2Index(x + 1, y + y_dir)] < 254 * factor_) //nav2_costmap_2d::LETHAL_OBSTACLE = 254
      return true;
    if (costs[grid2Index(x - 1, y)] >= 254 * factor_ &&
        costs[grid2Index(x - 1, y + y_dir)] < 254 * factor_) //nav2_costmap_2d::LETHAL_OBSTACLE = 254
      return true;
  }

  // diagonal
  if (x_dir && y_dir)
  {
    if (costs[grid2Index(x - x_dir, y)] >= 254 * factor_ &&
        costs[grid2Index(x - x_dir, y + y_dir)] < 254 * factor_) //nav2_costmap_2d::LETHAL_OBSTACLE = 254
      return true;
    if (costs[grid2Index(x, y - y_dir)] >= 254 * factor_ &&
        costs[grid2Index(x + x_dir, y - y_dir)] < 254 * factor_) //nav2_costmap_2d::LETHAL_OBSTACLE = 254
      return true;
  }

  return false;
}

}  // namespace jps_planner