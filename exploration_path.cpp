/**
 * @file exploration_path.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements the exploration path
 * @version 0.1
 * @date 2020-10-22
 *
 * @copyright Copyright (c) 2021
 *
 */
//探索路径（Exploration Path）的核心功能，用于管理机器人在探索过程中的路径规划。

#include "exploration_path/exploration_path.h"  // 包含探索路径头文件

namespace exploration_path_ns  // 定义探索路径命名空间
{
// Node类的默认构造函数
Node::Node()
  : type_(NodeType::LOCAL_VIA_POINT)  // 初始化节点类型为局部路径点
  , local_viewpoint_ind_(-1)          // 初始化局部视点索引为-1（无效值）
  , keypose_graph_node_ind_(-1)       // 初始化关键位姿图节点索引为-1（无效值）
  , global_subspace_index_(-1)        // 初始化全局子空间索引为-1（无效值）
  , position_(Eigen::Vector3d::Zero()) // 初始化位置为零向量
  , nonstop_(false)                   // 初始化非停止标志为false
{
}

// Node类的带位置参数的构造函数
Node::Node(Eigen::Vector3d position) : Node()  // 调用默认构造函数
{
  position_ = position;  // 设置节点位置
}

// Node类的带点和类型的构造函数
Node::Node(geometry_msgs::Point point, NodeType type) : Node(Eigen::Vector3d(point.x, point.y, point.z))  // 从ROS点消息构造Eigen向量并调用位置构造函数
{
  type_ = type;  // 设置节点类型
}

// 判断节点是否为局部节点的函数
bool Node::IsLocal()
{
  int node_type = static_cast<int>(type_);  // 将节点类型转换为整数
  return node_type % 2 == 0;  // 如果节点类型是偶数，则为局部节点
}

// 节点相等比较运算符重载
bool operator==(const Node& n1, const Node& n2)
{
  return ((n1.position_ - n2.position_).norm() < 0.2) && (n1.type_ == n2.type_);  // 位置距离小于0.2米且类型相同则认为相等
}

// 节点不等比较运算符重载
bool operator!=(const Node& n1, const Node& n2)
{
  return !(n1 == n2);  // 返回相等比较的否定
}

// 计算探索路径总长度的函数
double ExplorationPath::GetLength() const
{
  double length = 0.0;  // 初始化路径长度为0
  if (nodes_.size() < 2)  // 如果节点数量少于2个
  {
    return length;  // 返回0长度
  }
  for (int i = 1; i < nodes_.size(); i++)  // 从第二个节点开始遍历
  {
    length += (nodes_[i].position_ - nodes_[i - 1].position_).norm();  // 累加相邻节点间的欧几里得距离
  }
  return length;  // 返回总长度
}

// 向探索路径添加单个节点的函数
void ExplorationPath::Append(const Node& node)
{
  if (nodes_.empty() || nodes_.back() != node)  // 如果路径为空或最后一个节点与新节点不同
  {
    nodes_.push_back(node);  // 将新节点添加到路径末尾
  }
}

// 向探索路径添加另一个路径的函数
void ExplorationPath::Append(const ExplorationPath& path)
{
  for (int i = 0; i < path.nodes_.size(); i++)  // 遍历输入路径的所有节点
  {
    Append(path.nodes_[i]);  // 逐个添加节点（会自动去重）
  }
}

// 反转探索路径的函数
void ExplorationPath::Reverse()
{
  std::reverse(nodes_.begin(), nodes_.end());  // 使用STL算法反转节点顺序
}

// 将探索路径转换为ROS路径消息的函数
nav_msgs::Path ExplorationPath::GetPath() const
{
  nav_msgs::Path path;  // 创建ROS路径消息
  for (int i = 0; i < nodes_.size(); i++)  // 遍历所有节点
  {
    geometry_msgs::PoseStamped pose;  // 创建位姿消息
    pose.pose.position.x = nodes_[i].position_.x();  // 设置X坐标
    pose.pose.position.y = nodes_[i].position_.y();  // 设置Y坐标
    pose.pose.position.z = nodes_[i].position_.z();  // 设置Z坐标
    // pose.pose.orientation.w = static_cast<int>(nodes_[i].type_);  // 注释掉的代码：将节点类型存储在方向四元数的w分量中
    path.poses.push_back(pose);  // 将位姿添加到路径中
  }
  return path;  // 返回ROS路径消息
}

// 从ROS路径消息构造探索路径的函数
void ExplorationPath::FromPath(const nav_msgs::Path& path)
{
  nodes_.clear();  // 清空现有节点
  for (int i = 0; i < path.poses.size(); i++)  // 遍历路径中的所有位姿
  {
    exploration_path_ns::Node node;  // 创建新节点
    node.position_.x() = path.poses[i].pose.position.x;  // 设置X坐标
    node.position_.y() = path.poses[i].pose.position.y;  // 设置Y坐标
    node.position_.z() = path.poses[i].pose.position.z;  // 设置Z坐标
    node.type_ = static_cast<NodeType>(path.poses[i].pose.orientation.w);  // 从方向四元数的w分量恢复节点类型
    nodes_.push_back(node);  // 将节点添加到路径中
  }
}

// 获取用于可视化的点云数据的函数
void ExplorationPath::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud) const
{
  vis_cloud->clear();  // 清空点云
  for (int i = 0; i < nodes_.size(); i++)  // 遍历所有节点
  {
    pcl::PointXYZI point;  // 创建PCL点
    point.x = nodes_[i].position_.x();  // 设置X坐标
    point.y = nodes_[i].position_.y();  // 设置Y坐标
    point.z = nodes_[i].position_.z();  // 设置Z坐标
    point.intensity = static_cast<int>(nodes_[i].type_);  // 将节点类型作为强度值
    vis_cloud->points.push_back(point);  // 将点添加到点云中
  }
}

// 获取关键节点的点云数据的函数
void ExplorationPath::GetKeyPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud) const
{
  vis_cloud->clear();  // 清空点云
  for (int i = 0; i < nodes_.size(); i++)  // 遍历所有节点
  {
    // 只选择关键类型的节点进行可视化
    if (nodes_[i].type_ == exploration_path_ns::NodeType::ROBOT ||           // 机器人位置
        nodes_[i].type_ == exploration_path_ns::NodeType::LOOKAHEAD_POINT || // 前瞻点
        nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT || // 局部视点
        nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_START || // 局部路径起点
        nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_END ||   // 局部路径终点
        nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT)   // 全局视点
    {
      pcl::PointXYZI point;  // 创建PCL点
      point.x = nodes_[i].position_.x();  // 设置X坐标
      point.y = nodes_[i].position_.y();  // 设置Y坐标
      point.z = nodes_[i].position_.z();  // 设置Z坐标
      point.intensity = static_cast<int>(nodes_[i].type_);  // 将节点类型作为强度值
      vis_cloud->points.push_back(point);  // 将点添加到点云中
    }
  }
}

// 获取所有节点位置的函数
void ExplorationPath::GetNodePositions(std::vector<Eigen::Vector3d>& positions) const
{
  positions.clear();  // 清空位置向量
  for (int i = 0; i < nodes_.size(); i++)  // 遍历所有节点
  {
    positions.push_back(nodes_[i].position_);  // 将节点位置添加到向量中
  }
}

// 重置探索路径的函数
void ExplorationPath::Reset()
{
  nodes_.clear();  // 清空所有节点
}

}  // namespace exploration_path_ns  // 结束探索路径命名空间