/**
 * @file graph.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a graph
 * @version 0.1
 * @date 2021-07-11
 *
 * @copyright Copyright (c) 2021
 * 文件实现了图数据结构和A最短路径算法，是TARE规划器中路径规划的核心组件
 *
 */

#include <queue>  // 包含优先队列头文件，用于A*算法
#include <ros/ros.h>  // 包含ROS核心头文件
#include "utils/misc_utils.h"  // 包含工具函数头文件
#include "graph/graph.h"  // 包含图类头文件

namespace tare  // 定义TARE命名空间
{
// Graph类的构造函数，初始化指定节点数量的图
Graph::Graph(int node_number)
{
  connection_.resize(node_number);  // 调整连接关系数组大小
  distance_.resize(node_number);    // 调整距离数组大小
  positions_.resize(node_number);   // 调整位置数组大小
}

// 向图中添加新节点的函数
void Graph::AddNode(const Eigen::Vector3d& position)
{
  std::vector<int> connection;  // 创建新节点的连接关系向量
  connection_.push_back(connection);  // 将连接关系向量添加到图中
  std::vector<double> neighbor_distance;  // 创建新节点的邻居距离向量
  distance_.push_back(neighbor_distance);  // 将距离向量添加到图中
  positions_.push_back(position);  // 将节点位置添加到位置数组中
}

// 设置指定节点位置的函数
void Graph::SetNodePosition(int node_index, const Eigen::Vector3d& position)
{
  if (NodeIndexInRange(node_index))  // 如果节点索引在有效范围内
  {
    positions_[node_index] = position;  // 直接设置节点位置
  }
  else if (node_index == positions_.size())  // 如果节点索引等于当前节点数量
  {
    AddNode(position);  // 添加新节点
  }
  else  // 如果节点索引超出范围
  {
    ROS_ERROR_STREAM("Graph::SetNodePosition: node_index: " << node_index << " not in range [0, "  // 输出错误信息
                                                            << positions_.size() - 1 << "]");
  }
}

// 添加单向边的函数
void Graph::AddOneWayEdge(int from_node_index, int to_node_index, double distance)
{
  if (NodeIndexInRange(from_node_index) && NodeIndexInRange(to_node_index))  // 如果两个节点索引都在有效范围内
  {
    connection_[from_node_index].push_back(to_node_index);  // 添加从起始节点到目标节点的连接
    distance_[from_node_index].push_back(distance);  // 添加对应的距离值
  }
  else  // 如果节点索引超出范围
  {
    ROS_ERROR_STREAM("Graph::AddOneWayEdge: from_node_index: " << from_node_index << " to_node_index: " << to_node_index  // 输出错误信息
                                                               << " not in range [0, " << connection_.size() - 1
                                                               << "]");
  }
}

// 添加双向边的函数
void Graph::AddTwoWayEdge(int from_node_index, int to_node_index, double distance)
{
  AddOneWayEdge(from_node_index, to_node_index, distance);  // 添加从起始节点到目标节点的边
  AddOneWayEdge(to_node_index, from_node_index, distance);  // 添加从目标节点到起始节点的边
}

// 获取最短路径的函数
double Graph::GetShortestPath(int from_node_index, int to_node_index, bool get_path, nav_msgs::Path& shortest_path,
                              std::vector<int>& node_indices)
{
  node_indices.clear();  // 清空节点索引向量
  double path_length = AStarSearch(from_node_index, to_node_index, get_path, node_indices);  // 使用A*算法搜索最短路径
  if (get_path)  // 如果需要获取路径
  {
    shortest_path.poses.clear();  // 清空路径消息
    for (const auto& node_index : node_indices)  // 遍历路径中的所有节点索引
    {
      geometry_msgs::PoseStamped pose;  // 创建位姿消息
      pose.pose.position.x = positions_[node_index].x();  // 设置X坐标
      pose.pose.position.y = positions_[node_index].y();  // 设置Y坐标
      pose.pose.position.z = positions_[node_index].z();  // 设置Z坐标
      shortest_path.poses.push_back(pose);  // 将位姿添加到路径中
    }
  }
  return path_length;  // 返回路径长度
}

// A*搜索算法的实现
double Graph::AStarSearch(int from_node_index, int to_node_index, bool get_path, std::vector<int>& node_indices)
{
  MY_ASSERT(NodeIndexInRange(from_node_index));  // 断言起始节点索引有效
  MY_ASSERT(NodeIndexInRange(to_node_index));    // 断言目标节点索引有效

  double INF = 9999.0;  // 定义无穷大值
  typedef std::pair<double, int> iPair;  // 定义优先队列元素类型（距离，节点索引）
  double shortest_dist = 0;  // 初始化最短距离
  std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>> pq;  // 创建优先队列，按距离升序排列
  std::vector<double> g(connection_.size(), INF);  // g值数组，存储从起始点到各点的实际距离
  std::vector<double> f(connection_.size(), INF);  // f值数组，存储g值加启发式值
  std::vector<int> prev(connection_.size(), -1);   // 前驱节点数组，用于重建路径
  std::vector<bool> in_pg(connection_.size(), false);  // 标记节点是否在优先队列中

  g[from_node_index] = 0;  // 起始点到自身的距离为0
  f[from_node_index] = (positions_[from_node_index] - positions_[to_node_index]).norm();  // 计算起始点的f值（g值+启发式值）

  pq.push(std::make_pair(f[from_node_index], from_node_index));  // 将起始点加入优先队列
  in_pg[from_node_index] = true;  // 标记起始点在队列中

  while (!pq.empty())  // 当优先队列不为空时继续搜索
  {
    int u = pq.top().second;  // 取出f值最小的节点
    pq.pop();  // 从队列中移除该节点
    in_pg[u] = false;  // 标记该节点不在队列中
    if (u == to_node_index)  // 如果到达目标节点
    {
      shortest_dist = g[u];  // 记录最短距离
      break;  // 退出搜索
    }

    for (int i = 0; i < connection_[u].size(); i++)  // 遍历当前节点的所有邻居
    {
      int v = connection_[u][i];  // 获取邻居节点索引
      MY_ASSERT(misc_utils_ns::InRange<std::vector<int>>(connection_, v));  // 断言邻居节点索引有效
      double d = distance_[u][i];  // 获取到邻居节点的距离
      if (g[v] > g[u] + d)  // 如果通过当前节点到达邻居节点的距离更短
      {
        prev[v] = u;  // 更新邻居节点的前驱节点
        g[v] = g[u] + d;  // 更新邻居节点的g值
        f[v] = g[v] + (positions_[v] - positions_[to_node_index]).norm();  // 更新邻居节点的f值
        if (!in_pg[v])  // 如果邻居节点不在优先队列中
        {
          pq.push(std::make_pair(f[v], v));  // 将邻居节点加入优先队列
          in_pg[v] = true;  // 标记邻居节点在队列中
        }
      }
    }
  }

  if (get_path)  // 如果需要重建路径
  {
    node_indices.clear();  // 清空节点索引向量
    int u = to_node_index;  // 从目标节点开始
    if (prev[u] != -1 || u == from_node_index)  // 如果存在路径或目标节点就是起始节点
    {
      while (u != -1)  // 沿着前驱节点链回溯
      {
        node_indices.push_back(u);  // 将节点索引添加到路径中
        u = prev[u];  // 移动到前驱节点
      }
    }
  }
  std::reverse(node_indices.begin(), node_indices.end());  // 反转路径，使其从起始点到目标点

  return shortest_dist;  // 返回最短距离
}

}  // namespace tare  // 结束TARE命名空间
