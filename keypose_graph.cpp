//
// Created by caochao on 12/31/19.
//

/**
 * @file keypose_graph.cpp
 * @brief 关键姿态图实现文件
 * @author caochao
 * @date 2019-12-31
 * 
 * 该文件实现了关键姿态图(KeyposeGraph)的核心功能，包括：
 * - 关键姿态节点的创建和管理
 * - 节点间连接关系的建立和维护
 * - 路径规划和最短路径搜索
 * - 碰撞检测和连通性检查
 * - 可视化支持
 */

#include "../../include/keypose_graph/keypose_graph.h"  // 关键姿态图头文件
#include <viewpoint_manager/viewpoint_manager.h>        // 视点管理器头文件

namespace keypose_graph_ns
{
/**
 * @brief 关键姿态节点构造函数（坐标版本）
 * @param x X坐标
 * @param y Y坐标  
 * @param z Z坐标
 * @param node_ind 节点索引
 * @param keypose_id 关键姿态ID
 * @param is_keypose 是否为关键姿态节点
 */
KeyposeNode::KeyposeNode(double x, double y, double z, int node_ind, int keypose_id, bool is_keypose)
  : cell_ind_(0), node_ind_(node_ind), keypose_id_(keypose_id), is_keypose_(is_keypose), is_connected_(true)
{
  position_.x = x;                    // 设置节点X坐标
  position_.y = y;                    // 设置节点Y坐标
  position_.z = z;                    // 设置节点Z坐标

  offset_to_keypose_.x = 0.0;         // 初始化到关键姿态的X偏移量
  offset_to_keypose_.y = 0.0;         // 初始化到关键姿态的Y偏移量
  offset_to_keypose_.z = 0.0;         // 初始化到关键姿态的Z偏移量
}

/**
 * @brief 关键姿态节点构造函数（Point版本）
 * @param point 位置点
 * @param node_ind 节点索引
 * @param keypose_id 关键姿态ID
 * @param is_keypose 是否为关键姿态节点
 */
KeyposeNode::KeyposeNode(const geometry_msgs::Point& point, int node_ind, int keypose_id, bool is_keypose)
  : KeyposeNode(point.x, point.y, point.z, node_ind, keypose_id, is_keypose)  // 调用坐标版本构造函数
{
}

/**
 * @brief 关键姿态图构造函数
 * @param nh ROS节点句柄
 */
KeyposeGraph::KeyposeGraph(ros::NodeHandle& nh)
  : allow_vertical_edge_(false)                           // 默认不允许垂直边
  , current_keypose_id_(0)                               // 当前关键姿态ID初始化为0
  , kAddNodeMinDist(1.0)                                 // 添加节点的最小距离阈值
  , kAddEdgeCollisionCheckResolution(0.4)                // 边碰撞检查的分辨率
  , kAddEdgeCollisionCheckRadius(0.3)                    // 边碰撞检查的半径
  , kAddEdgeConnectDistThr(3.0)                          // 添加边的连接距离阈值
  , kAddEdgeToLastKeyposeDistThr(3.0)                    // 到最后一个关键姿态的距离阈值
  , kAddEdgeVerticalThreshold(1.0)                       // 垂直边阈值
  , kAddEdgeCollisionCheckPointNumThr(1)                 // 边碰撞检查点数量阈值
{
  ReadParameters(nh);                                    // 从ROS参数服务器读取参数
  // 初始化已连接节点的KD树
  kdtree_connected_nodes_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  // 初始化已连接节点的点云
  connected_nodes_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  // 初始化所有节点的KD树
  kdtree_nodes_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  // 初始化所有节点的点云
  nodes_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
}

/**
 * @brief 从ROS参数服务器读取关键姿态图参数
 * @param nh ROS节点句柄
 */
void KeyposeGraph::ReadParameters(ros::NodeHandle& nh)
{
  // 读取添加节点的最小距离阈值
  kAddNodeMinDist = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddNodeMinDist", 0.5);
  // 读取添加非关键姿态节点的最小距离阈值
  kAddNonKeyposeNodeMinDist = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddNonKeyposeNodeMinDist", 0.5);
  // 读取添加边的连接距离阈值
  kAddEdgeConnectDistThr = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeConnectDistThr", 0.5);
  // 读取到最后一个关键姿态的距离阈值
  kAddEdgeToLastKeyposeDistThr = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeToLastKeyposeDistThr", 0.5);
  // 读取垂直边阈值
  kAddEdgeVerticalThreshold = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeVerticalThreshold", 0.5);
  // 读取边碰撞检查的分辨率
  kAddEdgeCollisionCheckResolution =
      misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeCollisionCheckResolution", 0.5);
  // 读取边碰撞检查的半径
  kAddEdgeCollisionCheckRadius = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeCollisionCheckRadius", 0.5);
  // 读取边碰撞检查点数量阈值
  kAddEdgeCollisionCheckPointNumThr =
      misc_utils_ns::getParam<int>(nh, "keypose_graph/kAddEdgeCollisionCheckPointNumThr", 0.5);
}

/**
 * @brief 添加节点到关键姿态图
 * @param position 节点位置
 * @param node_ind 节点索引
 * @param keypose_id 关键姿态ID
 * @param is_keypose 是否为关键姿态节点
 */
void KeyposeGraph::AddNode(const geometry_msgs::Point& position, int node_ind, int keypose_id, bool is_keypose)
{
  KeyposeNode new_node(position, node_ind, keypose_id, is_keypose);  // 创建新节点
  nodes_.push_back(new_node);                                        // 将节点添加到节点列表
  std::vector<int> neighbors;                                        // 创建邻居列表
  graph_.push_back(neighbors);                                       // 将邻居列表添加到图中
  std::vector<double> neighbor_dist;                                 // 创建邻居距离列表
  dist_.push_back(neighbor_dist);                                    // 将距离列表添加到距离矩阵
}

/**
 * @brief 添加节点并建立与指定节点的连接
 * @param position 节点位置
 * @param node_ind 节点索引
 * @param keypose_id 关键姿态ID
 * @param is_keypose 是否为关键姿态节点
 * @param connected_node_ind 连接的节点索引
 * @param connected_node_dist 连接距离
 */
void KeyposeGraph::AddNodeAndEdge(const geometry_msgs::Point& position, int node_ind, int keypose_id, bool is_keypose,
                                  int connected_node_ind, double connected_node_dist)
{
  AddNode(position, node_ind, keypose_id, is_keypose);               // 先添加节点
  AddEdge(connected_node_ind, node_ind, connected_node_dist);        // 再添加边连接
}

/**
 * @brief 在两个节点之间添加边连接
 * @param from_node_ind 起始节点索引
 * @param to_node_ind 目标节点索引
 * @param dist 连接距离
 */
void KeyposeGraph::AddEdge(int from_node_ind, int to_node_ind, double dist)
{
  // 断言检查：确保节点索引在有效范围内
  MY_ASSERT(from_node_ind >= 0 && from_node_ind < graph_.size() && from_node_ind < dist_.size());
  MY_ASSERT(to_node_ind >= 0 && to_node_ind < graph_.size() && to_node_ind < dist_.size());

  // 在图中添加双向连接（无向图）
  graph_[from_node_ind].push_back(to_node_ind);          // 从起始节点到目标节点
  graph_[to_node_ind].push_back(from_node_ind);          // 从目标节点到起始节点

  // 在距离矩阵中记录连接距离
  dist_[from_node_ind].push_back(dist);                  // 记录起始节点的连接距离
  dist_[to_node_ind].push_back(dist);                    // 记录目标节点的连接距离
}

/**
 * @brief 检查指定位置是否已有节点
 * @param position 要检查的位置
 * @return 如果位置附近已有节点则返回true，否则返回false
 */
bool KeyposeGraph::HasNode(const Eigen::Vector3d& position)
{
  int closest_node_ind = -1;                              // 最近节点索引
  double min_dist = DBL_MAX;                              // 最小距离
  geometry_msgs::Point geo_position;                      // 转换为geometry_msgs::Point类型
  geo_position.x = position.x();                          // 设置X坐标
  geo_position.y = position.y();                          // 设置Y坐标
  geo_position.z = position.z();                          // 设置Z坐标
  
  // 获取最近节点的索引和距离
  GetClosestNodeIndAndDistance(geo_position, closest_node_ind, min_dist);
  
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())  // 检查索引是否有效
  {
    // 计算XY平面距离和Z轴距离
    double xy_dist = misc_utils_ns::PointXYDist<geometry_msgs::Point>(geo_position, nodes_[closest_node_ind].position_);
    double z_dist = std::abs(geo_position.z - nodes_[closest_node_ind].position_.z);
    
    // 如果XY距离小于阈值且Z距离小于1.0米，则认为已有节点
    if (xy_dist < kAddNonKeyposeNodeMinDist && z_dist < 1.0)
    {
      return true;
    }
  }
  return false;                                           // 没有找到合适的节点
}

/**
 * @brief 检查两个节点之间是否存在边连接
 * @param node_ind1 第一个节点索引
 * @param node_ind2 第二个节点索引
 * @return 如果存在边连接则返回true，否则返回false
 */
bool KeyposeGraph::HasEdgeBetween(int node_ind1, int node_ind2)
{
  // 检查两个节点索引是否都在有效范围内
  if (node_ind1 >= 0 && node_ind1 < nodes_.size() && node_ind2 >= 0 && node_ind2 < nodes_.size())
  {
    // 检查是否存在双向连接（无向图）
    if (std::find(graph_[node_ind1].begin(), graph_[node_ind1].end(), node_ind2) != graph_[node_ind1].end() ||
        std::find(graph_[node_ind2].begin(), graph_[node_ind2].end(), node_ind1) != graph_[node_ind2].end())
    {
      return true;                                        // 找到连接
    }
    else
    {
      return false;                                       // 没有连接
    }
  }
  else
  {
    return false;                                         // 节点索引无效
  }
}

/**
 * @brief 检查两个位置是否连通
 * @param from_position 起始位置
 * @param to_position 目标位置
 * @return 如果两个位置连通则返回true，否则返回false
 */
bool KeyposeGraph::IsConnected(const Eigen::Vector3d& from_position, const Eigen::Vector3d& to_position)
{
  // 转换起始位置为geometry_msgs::Point类型
  geometry_msgs::Point from_node_position;
  from_node_position.x = from_position.x();               // 设置起始位置X坐标
  from_node_position.y = from_position.y();               // 设置起始位置Y坐标
  from_node_position.z = from_position.z();               // 设置起始位置Z坐标
  int closest_from_node_ind = -1;                         // 最近起始节点索引
  double closest_from_node_dist = DBL_MAX;                // 到最近起始节点的距离
  GetClosestNodeIndAndDistance(from_node_position, closest_from_node_ind, closest_from_node_dist);

  // 转换目标位置为geometry_msgs::Point类型
  geometry_msgs::Point to_node_position;
  to_node_position.x = to_position.x();                   // 设置目标位置X坐标
  to_node_position.y = to_position.y();                   // 设置目标位置Y坐标
  to_node_position.z = to_position.z();                   // 设置目标位置Z坐标
  int closest_to_node_ind = -1;                           // 最近目标节点索引
  double closest_to_node_dist = DBL_MAX;                  // 到最近目标节点的距离
  GetClosestNodeIndAndDistance(to_node_position, closest_to_node_ind, closest_to_node_dist);

  // 检查连通性
  if (closest_from_node_ind != -1 && closest_from_node_ind == closest_to_node_ind)
  {
    return true;                                          // 两个位置对应同一个节点
  }
  else if (HasEdgeBetween(closest_from_node_ind, closest_to_node_ind))
  {
    return true;                                          // 两个节点之间有边连接
  }
  else
  {
    return false;                                         // 不连通
  }
}

/**
 * @brief 添加非关键姿态节点
 * @param new_node_position 新节点位置
 * @return 新节点的索引，如果位置太近已有节点则返回已有节点索引
 */
int KeyposeGraph::AddNonKeyposeNode(const geometry_msgs::Point& new_node_position)
{
  int new_node_index = -1;                                // 新节点索引
  int closest_node_ind = -1;                              // 最近节点索引
  double closest_node_dist = DBL_MAX;                     // 到最近节点的距离
  
  // 获取最近节点的索引和距离
  GetClosestNodeIndAndDistance(new_node_position, closest_node_ind, closest_node_dist);
  
  // 检查是否已有足够近的节点
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())
  {
    // 计算XY平面距离和Z轴距离
    double xy_dist =
        misc_utils_ns::PointXYDist<geometry_msgs::Point>(new_node_position, nodes_[closest_node_ind].position_);
    double z_dist = std::abs(new_node_position.z - nodes_[closest_node_ind].position_.z);
    
    // 如果距离太近，直接返回已有节点索引
    if (xy_dist < kAddNonKeyposeNodeMinDist && z_dist < 1.0)
    {
      return closest_node_ind;
    }
  }
  
  // 创建新节点
  new_node_index = nodes_.size();                         // 新节点索引为当前节点数量
  KeyposeNode new_node(new_node_position, new_node_index, current_keypose_id_, false);  // 创建非关键姿态节点
  new_node.SetCurrentKeyposePosition(current_keypose_position_);  // 设置当前关键姿态位置
  nodes_.push_back(new_node);                             // 将节点添加到节点列表
  
  // 为新节点创建邻居列表和距离列表
  std::vector<int> neighbors;                             // 邻居列表
  graph_.push_back(neighbors);                            // 添加到图中
  std::vector<double> neighbor_dist;                      // 距离列表
  dist_.push_back(neighbor_dist);                         // 添加到距离矩阵

  return new_node_index;                                  // 返回新节点索引
}

/**
 * @brief 将路径添加到关键姿态图中
 * @param path 要添加的路径
 */
void KeyposeGraph::AddPath(const nav_msgs::Path& path)
{
  // 检查路径是否至少包含2个点
  if (path.poses.size() < 2)
  {
    return;                                               // 路径太短，直接返回
  }
  
  int prev_node_index = -1;                               // 前一个节点索引
  
  // 遍历路径中的每个点
  for (int i = 0; i < path.poses.size(); i++)
  {
    // 为当前路径点添加非关键姿态节点
    int cur_node_index = AddNonKeyposeNode(path.poses[i].pose.position);
    
    // 如果不是第一个点，需要与前一个点建立连接
    if (i != 0)
    {
      // 检查前一个节点索引是否有效
      if (prev_node_index >= 0 && prev_node_index < nodes_.size())
      {
        // 检查是否已存在连接（避免重复）
        if (!HasEdgeBetween(prev_node_index, cur_node_index))
        {
          // 获取前一个节点的位置
          geometry_msgs::Point prev_node_position = nodes_[prev_node_index].position_;
          // 计算到前一个节点的距离
          double dist_to_prev = misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(
              prev_node_position, path.poses[i].pose.position);
          
          // 建立双向连接
          graph_[prev_node_index].push_back(cur_node_index);  // 从前一个节点到当前节点
          graph_[cur_node_index].push_back(prev_node_index);  // 从当前节点到前一个节点

          // 记录连接距离
          dist_[prev_node_index].push_back(dist_to_prev);     // 前一个节点的连接距离
          dist_[cur_node_index].push_back(dist_to_prev);      // 当前节点的连接距离
        }
      }
      else
      {
        // 前一个节点索引无效，输出错误信息并返回
        ROS_ERROR_STREAM("KeyposeGraph::AddPath: prev_node_index " << prev_node_index << " out of bound [0, "
                                                                   << nodes_.size() - 1 << "]");
        return;
      }
    }
    prev_node_index = cur_node_index;                      // 更新前一个节点索引
  }
  UpdateNodes();                                          // 更新节点数据结构
}

/**
 * @brief 获取关键姿态图的可视化标记
 * @param node_marker 节点标记（输出）
 * @param edge_marker 边标记（输出）
 */
void KeyposeGraph::GetMarker(visualization_msgs::Marker& node_marker, visualization_msgs::Marker& edge_marker)
{
  // 清空标记点
  node_marker.points.clear();                             // 清空节点标记点
  edge_marker.points.clear();                             // 清空边标记点

  // 添加所有节点位置到节点标记
  for (const auto& node : nodes_)
  {
    node_marker.points.push_back(node.position_);         // 添加节点位置
  }

  // 用于避免重复添加边的集合
  std::vector<std::pair<int, int>> added_edge;           // 已添加的边对
  
  // 遍历图中的所有节点
  for (int i = 0; i < graph_.size(); i++)
  {
    int start_ind = i;                                    // 起始节点索引
    // 遍历当前节点的所有邻居
    for (int j = 0; j < graph_[i].size(); j++)
    {
      int end_ind = graph_[i][j];                         // 目标节点索引
      // 检查是否已经添加过这条边（避免重复）
      if (std::find(added_edge.begin(), added_edge.end(), std::make_pair(start_ind, end_ind)) == added_edge.end())
      {
        // 获取起始和目标节点的位置
        geometry_msgs::Point start_node_position = nodes_[start_ind].position_;
        geometry_msgs::Point end_node_position = nodes_[end_ind].position_;
        // 添加边的两个端点
        edge_marker.points.push_back(start_node_position); // 添加起始点
        edge_marker.points.push_back(end_node_position);   // 添加目标点
        // 记录已添加的边
        added_edge.emplace_back(start_ind, end_ind);
      }
    }
  }
}

/**
 * @brief 获取连通节点的数量
 * @return 连通节点的数量
 */
int KeyposeGraph::GetConnectedNodeNum()
{
  int connected_node_num = 0;                              // 连通节点计数器
  // 遍历所有节点
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (nodes_[i].is_connected_)                           // 检查节点是否连通
    {
      connected_node_num++;                                // 连通节点数量加1
    }
  }
  return connected_node_num;                               // 返回连通节点总数
}

/**
 * @brief 获取关键姿态图的可视化点云
 * @param cloud 输出点云指针
 */
void KeyposeGraph::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  cloud->clear();                                          // 清空点云
  // 遍历所有节点
  for (const auto& node : nodes_)
  {
    pcl::PointXYZI point;                                  // 创建点云点
    point.x = node.position_.x;                           // 设置X坐标
    point.y = node.position_.y;                           // 设置Y坐标
    point.z = node.position_.z;                           // 设置Z坐标
    
    // 根据节点连通性设置强度值
    if (node.is_connected_)
    {
      point.intensity = 10;                               // 连通节点强度为10
    }
    else
    {
      point.intensity = -1;                               // 非连通节点强度为-1
    }
    cloud->points.push_back(point);                        // 将点添加到点云
  }
}

/**
 * @brief 获取与指定节点连通的所有节点索引（深度优先搜索）
 * @param query_ind 查询节点索引
 * @param connected_node_indices 输出连通节点索引列表
 * @param constraints 节点约束条件
 */
void KeyposeGraph::GetConnectedNodeIndices(int query_ind, std::vector<int>& connected_node_indices,
                                           std::vector<bool> constraints)
{
  // 检查约束条件大小是否与节点数量匹配
  if (nodes_.size() != constraints.size())
  {
    ROS_ERROR("KeyposeGraph::GetConnectedNodeIndices: constraints size not equal to node size");
    return;
  }
  // 检查查询节点索引是否在有效范围内
  if (query_ind < 0 || query_ind >= nodes_.size())
  {
    ROS_ERROR_STREAM("KeyposeGraph::GetConnectedNodeIndices: query_ind: " << query_ind << " out of range: [0, "
                                                                          << nodes_.size() << "]");
    return;
  }
  
  connected_node_indices.clear();                          // 清空连通节点索引列表
  std::vector<bool> visited(nodes_.size(), false);         // 访问标记数组
  std::stack<int> dfs_stack;                               // 深度优先搜索栈
  dfs_stack.push(query_ind);                               // 将查询节点压入栈
  
  // 深度优先搜索遍历连通节点
  while (!dfs_stack.empty())
  {
    int current_ind = dfs_stack.top();                     // 获取栈顶节点
    connected_node_indices.push_back(current_ind);         // 将当前节点添加到连通列表
    dfs_stack.pop();                                       // 弹出栈顶节点
    
    // 标记当前节点为已访问
    if (!visited[current_ind])
    {
      visited[current_ind] = true;
    }
    
    // 遍历当前节点的所有邻居
    for (int i = 0; i < graph_[current_ind].size(); i++)
    {
      int neighbor_ind = graph_[current_ind][i];           // 邻居节点索引
      // 如果邻居节点未访问且满足约束条件，则压入栈
      if (!visited[neighbor_ind] && constraints[neighbor_ind])
      {
        dfs_stack.push(neighbor_ind);
      }
    }
  }
}

/**
 * @brief 检查关键姿态图的局部碰撞
 * @param robot_position 机器人当前位置
 * @param viewpoint_manager 视点管理器指针
 */
void KeyposeGraph::CheckLocalCollision(const geometry_msgs::Point& robot_position,
                                       const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager)
{
  // 统计变量
  int in_local_planning_horizon_count = 0;                 // 在局部规划视野内的节点数量
  int collision_node_count = 0;                            // 碰撞节点数量
  int collision_edge_count = 0;                            // 碰撞边数量
  int in_viewpoint_range_count = 0;                        // 在视点范围内的节点数量
  
  // 获取视点分辨率并计算最大Z轴差异
  Eigen::Vector3d viewpoint_resolution = viewpoint_manager->GetResolution();
  double max_z_diff = std::max(viewpoint_resolution.x(), viewpoint_resolution.y()) * 2;
  
  // 遍历所有非关键姿态节点
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (nodes_[i].is_keypose_)                             // 跳过关键姿态节点
    {
      continue;
    }

    // 获取节点位置和对应的视点索引
    Eigen::Vector3d node_position =
        Eigen::Vector3d(nodes_[i].position_.x, nodes_[i].position_.y, nodes_[i].position_.z);
    int viewpoint_ind = viewpoint_manager->GetViewPointInd(node_position);
    bool node_in_collision = false;                        // 节点碰撞标志
    
    // 检查节点是否在视点范围内且高度差异在允许范围内
    if (viewpoint_manager->InRange(viewpoint_ind) &&
        std::abs(viewpoint_manager->GetViewPointHeight(viewpoint_ind) - node_position.z()) < max_z_diff)
    {
      in_local_planning_horizon_count++;                   // 在局部规划视野内
      in_viewpoint_range_count++;                          // 在视点范围内
      
      // 检查节点是否在碰撞中
      if (viewpoint_manager->ViewPointInCollision(viewpoint_ind))
      {
        node_in_collision = true;                          // 标记节点碰撞
        collision_node_count++;                            // 碰撞节点计数加1
        
        // 删除所有相关的边连接
        for (int j = 0; j < graph_[i].size(); j++)
        {
          int neighbor_ind = graph_[i][j];                 // 邻居节点索引
          // 从邻居节点中删除与当前节点的连接
          for (int k = 0; k < graph_[neighbor_ind].size(); k++)
          {
            if (graph_[neighbor_ind][k] == i)
            {
              graph_[neighbor_ind].erase(graph_[neighbor_ind].begin() + k);  // 删除邻居列表中的连接
              dist_[neighbor_ind].erase(dist_[neighbor_ind].begin() + k);    // 删除对应的距离
              k--;                                         // 调整索引
            }
          }
        }
        graph_[i].clear();                                 // 清空当前节点的所有连接
        dist_[i].clear();                                  // 清空当前节点的所有距离
      }
      else
      {
        // 节点不在碰撞中，检查边的碰撞
        Eigen::Vector3d viewpoint_resolution = viewpoint_manager->GetResolution();
        double collision_check_resolution = std::min(viewpoint_resolution.x(), viewpoint_resolution.y()) / 2;
        
        // 检查每条边的碰撞
        for (int j = 0; j < graph_[i].size(); j++)
        {
          int neighbor_ind = graph_[i][j];                 // 邻居节点索引
          Eigen::Vector3d start_position = node_position;  // 起始位置（当前节点）
          Eigen::Vector3d end_position = Eigen::Vector3d(  // 结束位置（邻居节点）
              nodes_[neighbor_ind].position_.x, nodes_[neighbor_ind].position_.y, nodes_[neighbor_ind].position_.z);
          
          // 在边上插值生成检查点
          std::vector<Eigen::Vector3d> interp_points;
          misc_utils_ns::LinInterpPoints(start_position, end_position, collision_check_resolution, interp_points);
          
          // 检查每个插值点是否在碰撞中
          for (const auto& collision_check_position : interp_points)
          {
            int viewpoint_ind = viewpoint_manager->GetViewPointInd(collision_check_position);
            if (viewpoint_manager->InRange(viewpoint_ind))
            {
              if (viewpoint_manager->ViewPointInCollision(viewpoint_ind))
              {
                geometry_msgs::Point viewpoint_position = viewpoint_manager->GetViewPointPosition(viewpoint_ind);
                
                // 删除邻居节点的边连接
                for (int k = 0; k < graph_[neighbor_ind].size(); k++)
                {
                  if (graph_[neighbor_ind][k] == i)
                  {
                    collision_edge_count++;                // 碰撞边计数加1
                    graph_[neighbor_ind].erase(graph_[neighbor_ind].begin() + k);  // 删除邻居列表中的连接
                    dist_[neighbor_ind].erase(dist_[neighbor_ind].begin() + k);    // 删除对应的距离
                    k--;                                   // 调整索引
                  }
                }
                // 删除当前节点的边连接
                graph_[i].erase(graph_[i].begin() + j);    // 删除当前节点的连接
                dist_[i].erase(dist_[i].begin() + j);      // 删除对应的距离
                j--;                                       // 调整索引
                break;                                     // 找到碰撞就跳出插值点循环
              }
            }
          }
        }
      }
    }
  }
}

/**
 * @brief 更新节点点云和KD树
 */
void KeyposeGraph::UpdateNodes()
{
  nodes_cloud_->clear();                                   // 清空节点点云
  // 遍历所有节点，构建点云
  for (int i = 0; i < nodes_.size(); i++)
  {
    pcl::PointXYZI point;                                  // 创建点云点
    point.x = nodes_[i].position_.x;                       // 设置X坐标
    point.y = nodes_[i].position_.y;                       // 设置Y坐标
    point.z = nodes_[i].position_.z;                       // 设置Z坐标
    point.intensity = i;                                   // 强度值设为节点索引
    nodes_cloud_->points.push_back(point);                 // 将点添加到点云
  }
  // 如果点云不为空，更新KD树
  if (!nodes_cloud_->points.empty())
  {
    kdtree_nodes_->setInputCloud(nodes_cloud_);            // 设置KD树输入点云
  }
}

/**
 * @brief 检查关键姿态图的连通性
 * @param robot_position 机器人当前位置
 */
void KeyposeGraph::CheckConnectivity(const geometry_msgs::Point& robot_position)
{
  if (nodes_.empty())                                      // 如果没有节点，直接返回
  {
    return;
  }
  UpdateNodes();                                          // 更新节点数据结构

  // 寻找第一个关键姿态节点作为连通性检查的起始点
  int first_keypose_node_ind = -1;                         // 第一个关键姿态节点索引
  bool found_connected = false;                            // 是否找到连通节点

  // 遍历所有节点，找到第一个关键姿态节点
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (nodes_[i].is_keypose_)
    {
      first_keypose_node_ind = i;                          // 记录第一个关键姿态节点索引
      break;
    }
  }

  // 从机器人位置开始检查连通性
  // 首先将所有节点标记为不连通
  for (int i = 0; i < nodes_.size(); i++)
  {
    nodes_[i].is_connected_ = false;                       // 标记所有节点为不连通
  }
  
  // 如果找到关键姿态节点，从关键姿态节点开始检查连通性
  if (first_keypose_node_ind >= 0 && first_keypose_node_ind < nodes_.size())
  {
    nodes_[first_keypose_node_ind].is_connected_ = true;   // 标记第一个关键姿态节点为连通
    connected_node_indices_.clear();                       // 清空连通节点索引列表
    std::vector<bool> constraint(nodes_.size(), true);     // 创建约束条件（所有节点都允许）
    GetConnectedNodeIndices(first_keypose_node_ind, connected_node_indices_, constraint);  // 获取连通节点
  }
  else
  {
    // 如果没有关键姿态节点，从最近的机器人节点开始检查
    int robot_node_ind = -1;                               // 机器人节点索引
    double robot_node_dist = DBL_MAX;                      // 到机器人节点的距离
    GetClosestNodeIndAndDistance(robot_position, robot_node_ind, robot_node_dist);
    
    if (robot_node_ind >= 0 && robot_node_ind < nodes_.size())
    {
      nodes_[robot_node_ind].is_connected_ = true;         // 标记机器人节点为连通
      connected_node_indices_.clear();                     // 清空连通节点索引列表
      std::vector<bool> constraint(nodes_.size(), true);   // 创建约束条件
      GetConnectedNodeIndices(robot_node_ind, connected_node_indices_, constraint);  // 获取连通节点
    }
    else
    {
      ROS_ERROR_STREAM("KeyposeGraph::CheckConnectivity: Cannot get closest robot node ind " << robot_node_ind);
    }
  }

  // 构建连通节点的点云
  connected_nodes_cloud_->clear();                         // 清空连通节点点云
  for (int i = 0; i < connected_node_indices_.size(); i++)
  {
    int node_ind = connected_node_indices_[i];             // 获取连通节点索引
    nodes_[node_ind].is_connected_ = true;                 // 标记节点为连通
    pcl::PointXYZI point;                                  // 创建点云点
    point.x = nodes_[node_ind].position_.x;               // 设置X坐标
    point.y = nodes_[node_ind].position_.y;               // 设置Y坐标
    point.z = nodes_[node_ind].position_.z;               // 设置Z坐标
    point.intensity = node_ind;                           // 强度值设为节点索引
    connected_nodes_cloud_->points.push_back(point);       // 将点添加到连通节点点云
  }
  
  // 如果连通节点点云不为空，更新连通节点的KD树
  if (!connected_nodes_cloud_->points.empty())
  {
    kdtree_connected_nodes_->setInputCloud(connected_nodes_cloud_);  // 设置连通节点KD树
  }
}

/**
 * @brief 添加关键姿态节点
 * @param keypose 关键姿态里程计信息
 * @param planning_env 规划环境
 * @return 新节点的索引，如果位置太近已有节点则返回已有节点索引
 */
int KeyposeGraph::AddKeyposeNode(const nav_msgs::Odometry& keypose, const planning_env_ns::PlanningEnv& planning_env)
{
  current_keypose_position_ = keypose.pose.pose.position;   // 更新当前关键姿态位置
  current_keypose_id_ = static_cast<int>(keypose.pose.covariance[0]);  // 更新当前关键姿态ID
  int new_node_ind = nodes_.size();                        // 新节点索引
  int keypose_node_count = 0;                              // 关键姿态节点计数
  
  // 统计现有的关键姿态节点数量
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (nodes_[i].is_keypose_)
    {
      keypose_node_count++;
    }
  }
  
  // 如果是第一个节点或没有关键姿态节点，直接添加
  if (nodes_.empty() || keypose_node_count == 0)
  {
    AddNode(current_keypose_position_, new_node_ind, current_keypose_id_, true);  // 添加关键姿态节点
    return new_node_ind;
  }
  else
  {
    // 寻找最近的节点和最后一个关键姿态节点
    double min_dist = DBL_MAX;                             // 最小距离
    int min_dist_ind = -1;                                 // 最近节点索引
    double last_keypose_dist = DBL_MAX;                    // 到最后一个关键姿态的距离
    int last_keypose_ind = -1;                             // 最后一个关键姿态节点索引
    int max_keypose_id = 0;                                // 最大关键姿态ID
    std::vector<int> in_range_node_indices;                // 在连接范围内的节点索引
    std::vector<double> in_range_node_dist;                // 对应的距离
    
    // 遍历所有节点，寻找合适的连接目标
    for (int i = 0; i < nodes_.size(); i++)
    {
      // 如果不允许垂直边，检查Z轴差异
      if (!allow_vertical_edge_)
      {
        if (std::abs(nodes_[i].position_.z - current_keypose_position_.z) > kAddEdgeVerticalThreshold)
        {
          continue;                                        // Z轴差异太大，跳过
        }
      }
      
      // 计算到当前节点的距离
      double dist = misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_,
                                                                                            current_keypose_position_);
      
      // 更新最近的关键姿态节点
      if (dist < min_dist && nodes_[i].is_keypose_)
      {
        min_dist = dist;
        min_dist_ind = i;
      }
      
      // 更新最后一个关键姿态节点（ID最大的）
      int keypose_id = nodes_[i].keypose_id_;
      if (keypose_id > max_keypose_id && nodes_[i].is_keypose_)
      {
        last_keypose_dist = dist;
        last_keypose_ind = i;
        max_keypose_id = keypose_id;
      }
      
      // 记录在连接范围内的节点
      if (dist < kAddEdgeConnectDistThr)
      {
        in_range_node_indices.push_back(i);
        in_range_node_dist.push_back(dist);
      }
    }
    // 如果最近的节点在有效范围内
    if (min_dist_ind >= 0 && min_dist_ind < nodes_.size())
    {
      if (min_dist > kAddNodeMinDist)                      // 如果距离足够远，需要添加新节点
      {
        // 如果最后一个关键姿态在范围内，优先连接到最后一个关键姿态
        if (last_keypose_dist < kAddEdgeToLastKeyposeDistThr && last_keypose_ind >= 0 &&
            last_keypose_ind < nodes_.size())
        {
          // 添加边连接到最后一个关键姿态节点
          AddNodeAndEdge(current_keypose_position_, new_node_ind, current_keypose_id_, true, last_keypose_ind,
                         last_keypose_dist);
        }
        else
        {
          // 否则连接到最近的节点
          AddNodeAndEdge(current_keypose_position_, new_node_ind, current_keypose_id_, true, min_dist_ind, min_dist);
        }
        
        // 检查其他在范围内的节点，尝试建立连接
        if (!in_range_node_indices.empty())
        {
          for (int idx = 0; idx < in_range_node_indices.size(); idx++)
          {
            int in_range_ind = in_range_node_indices[idx];
            if (in_range_ind >= 0 && in_range_ind < nodes_.size())
            {
              // 碰撞检查
              KeyposeNode neighbor_node = nodes_[in_range_ind];
              // 如果已经连接，跳过
              if (std::find(graph_[new_node_ind].begin(), graph_[new_node_ind].end(), in_range_ind) !=
                  graph_[new_node_ind].end())
                continue;
              
              double neighbor_node_dist = in_range_node_dist[idx];  // 到邻居节点的距离
              double diff_x = neighbor_node.position_.x - current_keypose_position_.x;  // X轴差异
              double diff_y = neighbor_node.position_.y - current_keypose_position_.y;  // Y轴差异
              double diff_z = neighbor_node.position_.z - current_keypose_position_.z;  // Z轴差异
              
              // 计算需要检查的碰撞点数量
              int check_point_num = static_cast<int>(neighbor_node_dist / kAddEdgeCollisionCheckResolution);
              bool in_collision = false;                   // 碰撞标志
              
              // 在边上插值检查碰撞
              for (int i = 0; i < check_point_num; i++)
              {
                // 计算检查点的坐标
                double check_point_x =
                    current_keypose_position_.x + kAddEdgeCollisionCheckResolution * i * diff_x / neighbor_node_dist;
                double check_point_y =
                    current_keypose_position_.y + kAddEdgeCollisionCheckResolution * i * diff_y / neighbor_node_dist;
                double check_point_z =
                    current_keypose_position_.z + kAddEdgeCollisionCheckResolution * i * diff_z / neighbor_node_dist;
                
                // 检查是否在碰撞中
                if (planning_env.InCollision(check_point_x, check_point_y, check_point_z))
                {
                  in_collision = true;                     // 标记为碰撞
                  break;                                   // 找到碰撞就跳出
                }
              }
              
              // 如果没有碰撞，添加边连接
              if (!in_collision)
              {
                AddEdge(new_node_ind, in_range_ind, neighbor_node_dist);
              }
            }
          }
        }
        return new_node_ind;                               // 返回新节点索引
      }
      else
      {
        return min_dist_ind;                               // 距离太近，返回最近节点索引
      }
    }
    else
    {
      ROS_ERROR_STREAM("KeyposeGraph::AddKeyposeNode: Nearest keypose ind out of range: " << min_dist_ind);
      return new_node_ind;                                 // 返回新节点索引
    }
  }
}

/**
 * @brief 检查指定位置是否可达（使用自定义距离阈值）
 * @param point 要检查的位置
 * @param dist_threshold 距离阈值
 * @return 如果位置可达则返回true，否则返回false
 */
bool KeyposeGraph::IsPositionReachable(const geometry_msgs::Point& point, double dist_threshold)
{
  int closest_node_ind = 0;                                // 最近连通节点索引
  double closest_node_dist = DBL_MAX;                      // 到最近连通节点的距离
  GetClosestConnectedNodeIndAndDistance(point, closest_node_ind, closest_node_dist);  // 获取最近连通节点
  
  // 检查是否在阈值范围内可达
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size() && closest_node_dist < dist_threshold)
  {
    return true;                                           // 位置可达
  }
  else
  {
    return false;                                          // 位置不可达
  }
}

/**
 * @brief 检查指定位置是否可达（使用默认距离阈值）
 * @param point 要检查的位置
 * @return 如果位置可达则返回true，否则返回false
 */
bool KeyposeGraph::IsPositionReachable(const geometry_msgs::Point& point)
{
  int closest_node_ind = 0;                                // 最近连通节点索引
  double closest_node_dist = DBL_MAX;                      // 到最近连通节点的距离
  GetClosestConnectedNodeIndAndDistance(point, closest_node_ind, closest_node_dist);  // 获取最近连通节点
  
  // 使用默认阈值检查是否可达
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size() && closest_node_dist < kAddNonKeyposeNodeMinDist)
  {
    return true;                                           // 位置可达
  }
  else
  {
    return false;                                          // 位置不可达
  }
}

/**
 * @brief 获取距离指定位置最近的节点索引
 * @param point 查询位置
 * @return 最近节点的索引
 */
int KeyposeGraph::GetClosestNodeInd(const geometry_msgs::Point& point)
{
  int node_ind = 0;                                        // 节点索引
  double min_dist = DBL_MAX;                               // 最小距离
  GetClosestNodeIndAndDistance(point, node_ind, min_dist); // 获取最近节点索引和距离
  return node_ind;                                         // 返回节点索引
}

/**
 * @brief 获取距离指定位置最近的节点索引和距离
 * @param point 查询位置
 * @param node_ind 输出最近节点索引
 * @param dist 输出到最近节点的距离
 */
void KeyposeGraph::GetClosestNodeIndAndDistance(const geometry_msgs::Point& point, int& node_ind, double& dist)
{
  node_ind = -1;                                           // 初始化节点索引为-1
  dist = DBL_MAX;                                          // 初始化距离为最大值
  
  // 如果点云为空，直接返回
  if (nodes_cloud_->points.empty())
  {
    node_ind = -1;
    dist = DBL_MAX;
    return;
  }
  
  // 创建搜索点
  pcl::PointXYZI search_point;
  search_point.x = point.x;                               // 设置搜索点X坐标
  search_point.y = point.y;                               // 设置搜索点Y坐标
  search_point.z = point.z;                               // 设置搜索点Z坐标
  
  // 使用KD树进行最近邻搜索
  std::vector<int> nearest_neighbor_node_indices(1);      // 最近邻节点索引
  std::vector<float> nearest_neighbor_squared_dist(1);    // 最近邻距离的平方
  kdtree_nodes_->nearestKSearch(search_point, 1, nearest_neighbor_node_indices, nearest_neighbor_squared_dist);
  
  // 检查搜索结果是否有效
  if (!nearest_neighbor_node_indices.empty() && nearest_neighbor_node_indices.front() >= 0 &&
      nearest_neighbor_node_indices.front() < nodes_cloud_->points.size())
  {
    // 从点云中获取节点索引和距离
    node_ind = static_cast<int>(nodes_cloud_->points[nearest_neighbor_node_indices.front()].intensity);
    dist = sqrt(nearest_neighbor_squared_dist.front());    // 计算实际距离
  }
  else
  {
    // KD树搜索失败，使用暴力搜索作为备选方案
    ROS_WARN_STREAM("KeyposeGraph::GetClosestNodeIndAndDistance: search for nearest neighbor failed with "
                    << nodes_cloud_->points.size() << " nodes.");
    if (!nearest_neighbor_node_indices.empty())
    {
      ROS_WARN_STREAM("Nearest neighbor node Ind: " << nearest_neighbor_node_indices.front());
    }
    
    // 遍历所有节点，找到最近的节点
    for (int i = 0; i < nodes_.size(); i++)
    {
      geometry_msgs::Point node_position = nodes_[i].position_;
      double dist_to_query =
          misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(point, node_position);
      if (dist_to_query < dist)
      {
        dist = dist_to_query;                              // 更新最小距离
        node_ind = i;                                      // 更新最近节点索引
      }
    }
  }
}

/**
 * @brief 获取距离指定位置最近的连通节点索引和距离
 * @param point 查询位置
 * @param node_ind 输出最近连通节点索引
 * @param dist 输出到最近连通节点的距离
 */
void KeyposeGraph::GetClosestConnectedNodeIndAndDistance(const geometry_msgs::Point& point, int& node_ind, double& dist)
{
  // 如果连通节点点云为空，直接返回
  if (connected_nodes_cloud_->points.empty())
  {
    node_ind = -1;
    dist = DBL_MAX;
    return;
  }
  
  // 创建搜索点
  pcl::PointXYZI search_point;
  search_point.x = point.x;                                // 设置搜索点X坐标
  search_point.y = point.y;                                // 设置搜索点Y坐标
  search_point.z = point.z;                                // 设置搜索点Z坐标
  
  // 使用连通节点KD树进行最近邻搜索
  std::vector<int> nearest_neighbor_node_indices(1);      // 最近邻节点索引
  std::vector<float> nearest_neighbor_squared_dist(1);    // 最近邻距离的平方
  kdtree_connected_nodes_->nearestKSearch(search_point, 1, nearest_neighbor_node_indices,
                                          nearest_neighbor_squared_dist);
  
  // 检查搜索结果是否有效
  if (!nearest_neighbor_node_indices.empty() && nearest_neighbor_node_indices.front() >= 0 &&
      nearest_neighbor_node_indices.front() < connected_nodes_cloud_->points.size())
  {
    // 从连通节点点云中获取节点索引和距离
    node_ind = static_cast<int>(connected_nodes_cloud_->points[nearest_neighbor_node_indices.front()].intensity);
    dist = sqrt(nearest_neighbor_squared_dist.front());    // 计算实际距离
  }
  else
  {
    // 搜索失败，输出警告信息
    ROS_WARN_STREAM("KeyposeGraph::GetClosestNodeInd: search for nearest neighbor failed with "
                    << connected_nodes_cloud_->points.size() << " connected nodes.");
    node_ind = -1;                                         // 设置无效索引
    dist = 0;                                              // 设置距离为0
  }
}

int KeyposeGraph::GetClosestKeyposeID(const geometry_msgs::Point& point)
{
  int closest_node_ind = GetClosestNodeInd(point);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())
  {
    return nodes_[closest_node_ind].keypose_id_;
  }
  else
  {
    return -1;
  }
}

geometry_msgs::Point KeyposeGraph::GetClosestNodePosition(const geometry_msgs::Point& point)
{
  int closest_node_ind = GetClosestNodeInd(point);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())
  {
    return nodes_[closest_node_ind].position_;
  }
  else
  {
    geometry_msgs::Point point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    return point;
  }
}

/**
 * @brief 获取带最大长度限制的最短路径
 * @param start_point 起始位置
 * @param target_point 目标位置
 * @param max_path_length 最大路径长度
 * @param get_path 是否获取路径
 * @param path 输出路径
 * @return 如果找到路径则返回true，否则返回false
 */
bool KeyposeGraph::GetShortestPathWithMaxLength(const geometry_msgs::Point& start_point,
                                                const geometry_msgs::Point& target_point, double max_path_length,
                                                bool get_path, nav_msgs::Path& path)
{
  // 如果节点数量少于2个，无法规划路径
  if (nodes_.size() < 2)
  {
    if (get_path)
    {
      // 创建简单的直线路径
      geometry_msgs::PoseStamped start_pose;
      start_pose.pose.position = start_point;
      geometry_msgs::PoseStamped target_pose;
      target_pose.pose.position = target_point;
      path.poses.push_back(start_pose);
      path.poses.push_back(target_pose);
    }
    return misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(start_point, target_point);
  }
  
  // 寻找最近的起始和目标节点
  int from_idx = 0;                                        // 起始节点索引
  int to_idx = 0;                                          // 目标节点索引
  double min_dist_to_start = DBL_MAX;                      // 到起始点的最小距离
  double min_dist_to_target = DBL_MAX;                     // 到目标点的最小距离
  
  // 遍历所有节点，找到最近的起始和目标节点
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (allow_vertical_edge_)                              // 如果允许垂直边
    {
      // 使用3D距离计算
      double dist_to_start =
          misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, start_point);
      double dist_to_target =
          misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, target_point);
      
      // 更新最近的起始节点
      if (dist_to_start < min_dist_to_start)
      {
        min_dist_to_start = dist_to_start;
        from_idx = i;
      }
      // 更新最近的目标节点
      if (dist_to_target < min_dist_to_target)
      {
        min_dist_to_target = dist_to_target;
        to_idx = i;
      }
    }
    else                                                  // 如果不允许垂直边
    {
      // 检查Z轴差异
      double z_diff_to_start = std::abs(nodes_[i].position_.z - start_point.z);
      double z_diff_to_target = std::abs(nodes_[i].position_.z - target_point.z);
      
      // 如果Z轴差异在允许范围内，使用XY平面距离
      if (z_diff_to_start < 1.5)
      {
        double xy_dist_to_start =
            misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, start_point);
        if (xy_dist_to_start < min_dist_to_start)
        {
          min_dist_to_start = xy_dist_to_start;
          from_idx = i;
        }
      }
      if (z_diff_to_target < 1.5)
      {
        double xy_dist_to_target =
            misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, target_point);
        if (xy_dist_to_target < min_dist_to_target)
        {
          min_dist_to_target = xy_dist_to_target;
          to_idx = i;
        }
      }
    }
  }

  // 构建节点位置列表
  std::vector<geometry_msgs::Point> node_positions;
  for (int i = 0; i < nodes_.size(); i++)
  {
    node_positions.push_back(nodes_[i].position_);
  }
  
  // 使用A*算法搜索带最大长度限制的最短路径
  std::vector<int> path_indices;                           // 路径节点索引
  double shortest_dist = DBL_MAX;                          // 最短距离
  bool found_path = misc_utils_ns::AStarSearchWithMaxPathLength(graph_, dist_, node_positions, from_idx, to_idx,
                                                                get_path, path_indices, shortest_dist, max_path_length);
  
  // 如果找到路径且需要获取路径，构建路径消息
  if (found_path && get_path)
  {
    path.poses.clear();                                    // 清空路径
    for (const auto& ind : path_indices)
    {
      geometry_msgs::PoseStamped pose;                     // 创建姿态消息
      pose.pose.position = nodes_[ind].position_;          // 设置位置
      pose.pose.orientation.w = nodes_[ind].keypose_id_;   // 设置关键姿态ID
      pose.pose.orientation.x = ind;                       // 设置节点索引
      path.poses.push_back(pose);                          // 添加到路径
    }
  }

  return found_path;                                       // 返回是否找到路径
}

double KeyposeGraph::GetShortestPath(const geometry_msgs::Point& start_point, const geometry_msgs::Point& target_point,
                                     bool get_path, nav_msgs::Path& path, bool use_connected_nodes)
{
  if (nodes_.size() < 2)
  {
    if (get_path)
    {
      geometry_msgs::PoseStamped start_pose;
      start_pose.pose.position = start_point;
      geometry_msgs::PoseStamped target_pose;
      target_pose.pose.position = target_point;
      path.poses.push_back(start_pose);
      path.poses.push_back(target_pose);
    }
    return misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(start_point, target_point);
  }
  int from_idx = 0;
  int to_idx = 0;
  double min_dist_to_start = DBL_MAX;
  double min_dist_to_target = DBL_MAX;
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (use_connected_nodes && !nodes_[i].is_connected_)
    {
      continue;
    }
    if (allow_vertical_edge_)
    {
      double dist_to_start =
          misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, start_point);
      double dist_to_target =
          misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, target_point);
      if (dist_to_start < min_dist_to_start)
      {
        min_dist_to_start = dist_to_start;
        from_idx = i;
      }
      if (dist_to_target < min_dist_to_target)
      {
        min_dist_to_target = dist_to_target;
        to_idx = i;
      }
    }
    else
    {
      double z_diff_to_start = std::abs(nodes_[i].position_.z - start_point.z);
      double z_diff_to_target = std::abs(nodes_[i].position_.z - target_point.z);
      // TODO: parameterize this
      if (z_diff_to_start < 1.5)
      {
        double xy_dist_to_start =
            misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, start_point);
        if (xy_dist_to_start < min_dist_to_start)
        {
          min_dist_to_start = xy_dist_to_start;
          from_idx = i;
        }
      }
      if (z_diff_to_target < 1.5)
      {
        double xy_dist_to_target =
            misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, target_point);
        if (xy_dist_to_target < min_dist_to_target)
        {
          min_dist_to_target = xy_dist_to_target;
          to_idx = i;
        }
      }
    }
  }

  std::vector<geometry_msgs::Point> node_positions;
  for (int i = 0; i < nodes_.size(); i++)
  {
    node_positions.push_back(nodes_[i].position_);
  }
  std::vector<int> path_indices;
  double shortest_dist =
      misc_utils_ns::AStarSearch(graph_, dist_, node_positions, from_idx, to_idx, get_path, path_indices);
  if (get_path)
  {
    path.poses.clear();
    for (const auto& ind : path_indices)
    {
      geometry_msgs::PoseStamped pose;
      pose.pose.position = nodes_[ind].position_;
      pose.pose.orientation.w = nodes_[ind].keypose_id_;
      pose.pose.orientation.x = ind;
      path.poses.push_back(pose);
    }
  }

  return shortest_dist;
}

geometry_msgs::Point KeyposeGraph::GetFirstKeyposePosition()
{
  geometry_msgs::Point point;
  point.x = 0;
  point.y = 0;
  point.z = 0;
  for (const auto& node : nodes_)
  {
    if (node.IsKeypose())
    {
      point = node.position_;
      break;
    }
  }
  return point;
}

geometry_msgs::Point KeyposeGraph::GetKeyposePosition(int keypose_id)
{
  geometry_msgs::Point point;
  point.x = 0;
  point.y = 0;
  point.z = 0;
  for (const auto& node : nodes_)
  {
    if (node.keypose_id_ == keypose_id)
    {
      point = node.position_;
      break;
    }
  }
  return point;
}

void KeyposeGraph::GetKeyposePositions(std::vector<Eigen::Vector3d>& positions)
{
  positions.clear();
  for (const auto& node : nodes_)
  {
    if (node.IsKeypose())
    {
      Eigen::Vector3d position(node.position_.x, node.position_.y, node.position_.z);
      positions.push_back(position);
    }
  }
}

geometry_msgs::Point KeyposeGraph::GetNodePosition(int node_ind)
{
  geometry_msgs::Point node_position;
  node_position.x = 0;
  node_position.y = 0;
  node_position.z = 0;
  if (node_ind >= 0 && node_ind < nodes_.size())
  {
    node_position = nodes_[node_ind].position_;
  }
  else
  {
    ROS_WARN_STREAM("KeyposeGraph::GetNodePosition: node_ind " << node_ind << " out of bound [0, " << nodes_.size() - 1
                                                               << "]");
  }
  return node_position;
}

}  // namespace keypose_graph_ns
