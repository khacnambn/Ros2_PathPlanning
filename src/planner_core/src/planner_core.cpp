/**
 * *********************************************************
 *
 * @file: global_planner.cpp
 * @brief: Contains the abstract global planner class
 * @author: Yang Haodong
 * @date: 2023-10-24
 * @version: 2.1
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "planner_core.hpp"

namespace planner_core
{
/**
 * @brief Set or reset obstacle factor
 * @param factor obstacle factor
 */
void PlannerCore::setFactor(float factor)
{
  factor_ = factor;
}

/**
 * @brief get the costmap
 * @return costmap costmap2d pointer
 */
nav2_costmap_2d::Costmap2D* PlannerCore::getCostMap() const
{
  return costmap_;
}

/**
 * @brief get the size of costmap
 * @return map_size the size of costmap
 */
int PlannerCore::getMapSize() const
{
  return map_size_;
}

/**
 * @brief Transform from grid map(x, y) to grid index(i)
 * @param x grid map x
 * @param y grid map y
 * @return index
 */
int PlannerCore::grid2Index(int x, int y)
{
  return x + static_cast<int>(costmap_->getSizeInCellsX() * y);
}

/**
 * @brief Transform from grid index(i) to grid map(x, y)
 * @param i grid index i
 * @param x grid map x
 * @param y grid map y
 */
void PlannerCore::index2Grid(int i, int& x, int& y)
{
  x = static_cast<int>(i % costmap_->getSizeInCellsX());
  y = static_cast<int>(i / costmap_->getSizeInCellsX());
}

/**
 * @brief Tranform from world map(x, y) to costmap(x, y)
 * @param mx costmap x
 * @param my costmap y
 * @param wx world map x
 * @param wy world map y
 * @return true if successfull, else false
 */
bool PlannerCore::world2Map(double wx, double wy, unsigned int& mx, unsigned int& my)
{
  return costmap_->worldToMap(wx, wy, mx, my);
}

/**
 * @brief Tranform from costmap(x, y) to world map(x, y)
 * @param mx costmap x
 * @param my costmap y
 * @param wx world map x
 * @param wy world map y
 */
void PlannerCore::map2World(unsigned int mx, unsigned int my, double& wx, double& wy)
{
  costmap_->mapToWorld(mx, my, wx, wy);
}

/**
 * @brief Inflate the boundary of costmap into obstacles to prevent cross planning
 */
void PlannerCore::outlineMap()
{
  auto nx = costmap_->getSizeInCellsX();
  auto ny = costmap_->getSizeInCellsY();
  auto pc = costmap_->getCharMap();
  for (unsigned int i = 0; i < nx; i++)
    *pc++ = 254; //nav2_costmap_2d::LETHAL_OBSTACLE = 254
  pc = costmap_->getCharMap() + (ny - 1) * nx;
  for (unsigned int i = 0; i < nx; i++)
    *pc++ = 254; // nav2_costmap_2d::LETHAL_OBSTACLE
  pc = costmap_->getCharMap();
  for (unsigned int i = 0; i < ny; i++, pc += nx)
    *pc = 254; // nav2_costmap_2d::LETHAL_OBSTACLE
  pc = costmap_->getCharMap() + nx - 1;
  for (unsigned int i = 0; i < ny; i++, pc += nx)
    *pc = 254; // nav2_costmap_2d::LETHAL_OBSTACLE
}

/**
 * @brief Convert closed list to path
 * @param closed_list closed list
 * @param start       start node
 * @param goal        goal node
 * @return vector containing path nodes
 */
std::vector<Node> PlannerCore::_convertClosedListToPath(std::unordered_map<int, Node>& closed_list, const Node& start,
                                                          const Node& goal)
{
  std::vector<Node> path;
  auto current = closed_list.find(goal.id());
  while (current->second != start)
  {
    path.emplace_back(current->second.x(), current->second.y());
    auto it = closed_list.find(current->second.pid());
    if (it != closed_list.end())
      current = it;
    else
      return {};
  }
  path.push_back(start);
  return path;
}

}  // namespace global_planner