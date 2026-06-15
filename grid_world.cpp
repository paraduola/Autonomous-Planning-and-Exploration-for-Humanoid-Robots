/**
 * @file grid_world.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a grid world
 * @version 0.1
 * @date 2019-11-06
 *
 * @copyright Copyright (c) 2021
 *
 */

// 包含必要的头文件
#include "../../include/grid_world/grid_world.h"  // 包含网格世界的头文件
#include <map>                                    // 包含map容器的头文件
#include <algorithm>                              // 包含算法函数的头文件
#include <utils/misc_utils.h>                     // 包含工具函数的头文件
#include <viewpoint_manager/viewpoint_manager.h>  // 包含视点管理器的头文件

// 定义网格世界命名空间
namespace grid_world_ns
{

// Cell类的构造函数，接收x, y, z坐标参数
Cell::Cell(double x, double y, double z)
  : in_horizon_(false)                           // 初始化：不在视野范围内
  , robot_position_set_(false)                   // 初始化：机器人位置未设置
  , visit_count_(0)                              // 初始化：访问次数为0
  , keypose_id_(0)                               // 初始化：关键姿态ID为0
  , path_added_to_keypose_graph_(false)          // 初始化：路径未添加到关键姿态图
  , roadmap_connection_point_set_(false)         // 初始化：路线图连接点未设置
  , viewpoint_position_(Eigen::Vector3d(x, y, z)) // 初始化：视点位置为给定坐标
  , roadmap_connection_point_(Eigen::Vector3d(x, y, z)) // 初始化：路线图连接点为给定坐标
{
  // 设置单元格中心点坐标
  center_.x = x;
  center_.y = y;
  center_.z = z;

  // 初始化机器人位置为原点
  robot_position_.x = 0;
  robot_position_.y = 0;
  robot_position_.z = 0;
  
  // 设置单元格状态为未见过
  status_ = CellStatus::UNSEEN;
}

// Cell类的构造函数，接收geometry_msgs::Point类型的中心点参数
Cell::Cell(const geometry_msgs::Point& center) : Cell(center.x, center.y, center.z)
{
}

// 重置单元格的所有状态和属性
void Cell::Reset()
{
  status_ = CellStatus::UNSEEN;                  // 重置状态为未见过
  robot_position_.x = 0;                         // 重置机器人位置x坐标
  robot_position_.y = 0;                         // 重置机器人位置y坐标
  robot_position_.z = 0;                         // 重置机器人位置z坐标
  visit_count_ = 0;                              // 重置访问次数为0
  viewpoint_indices_.clear();                    // 清空视点索引列表
  connected_cell_indices_.clear();               // 清空连接的单元格索引列表
  keypose_graph_node_indices_.clear();           // 清空关键姿态图节点索引列表
}

// 检查指定单元格索引是否与当前单元格连接
bool Cell::IsCellConnected(int cell_ind)
{
  // 在连接的单元格索引列表中查找指定的索引
  if (std::find(connected_cell_indices_.begin(), connected_cell_indices_.end(), cell_ind) !=
      connected_cell_indices_.end())
  {
    return true;                                 // 如果找到，返回true表示已连接
  }
  else
  {
    return false;                                // 如果未找到，返回false表示未连接
  }
}

// GridWorld类的构造函数，接收ROS节点句柄参数
GridWorld::GridWorld(ros::NodeHandle& nh) : initialized_(false), use_keypose_graph_(false)
{
  ReadParameters(nh);                            // 读取ROS参数
  
  // 初始化机器人位置为原点
  robot_position_.x = 0.0;
  robot_position_.y = 0.0;
  robot_position_.z = 0.0;

  // 初始化网格世界原点为原点
  origin_.x = 0.0;
  origin_.y = 0.0;
  origin_.z = 0.0;

  // 创建网格大小向量（行数、列数、层数）
  Eigen::Vector3i grid_size(kRowNum, kColNum, kLevelNum);
  // 创建网格原点向量
  Eigen::Vector3d grid_origin(0.0, 0.0, 0.0);
  // 创建网格分辨率向量（单元格大小、单元格大小、单元格高度）
  Eigen::Vector3d grid_resolution(kCellSize, kCellSize, kCellHeight);
  
  Cell cell_tmp;                                 // 创建临时单元格对象
  // 创建网格对象，使用智能指针管理
  subspaces_ = std::make_unique<grid_ns::Grid<Cell>>(grid_size, cell_tmp, grid_origin, grid_resolution);
  
  // 遍历所有单元格，初始化为默认的Cell对象
  for (int i = 0; i < subspaces_->GetCellNumber(); ++i)
  {
    subspaces_->GetCell(i) = grid_world_ns::Cell();
  }

  // 初始化回家位置为原点
  home_position_.x() = 0.0;
  home_position_.y() = 0.0;
  home_position_.z() = 0.0;

  // 初始化当前关键姿态图节点位置为原点
  cur_keypose_graph_node_position_.x = 0.0;
  cur_keypose_graph_node_position_.y = 0.0;
  cur_keypose_graph_node_position_.z = 0.0;

  set_home_ = false;                             // 初始化：未设置回家位置
  return_home_ = false;                          // 初始化：不需要回家

  cur_robot_cell_ind_ = -1;                      // 初始化：当前机器人单元格索引为-1
  prev_robot_cell_ind_ = -1;                     // 初始化：前一个机器人单元格索引为-1
}

// GridWorld类的构造函数，接收网格参数
GridWorld::GridWorld(int row_num, int col_num, int level_num, double cell_size, double cell_height, int nearby_grid_num)
  : kRowNum(row_num)                             // 初始化行数
  , kColNum(col_num)                             // 初始化列数
  , kLevelNum(level_num)                         // 初始化层数
  , kCellSize(cell_size)                         // 初始化单元格大小
  , kCellHeight(cell_height)                     // 初始化单元格高度
  , KNearbyGridNum(nearby_grid_num)              // 初始化附近网格数量
  , kMinAddPointNumSmall(60)                     // 初始化小阈值最小添加点数
  , kMinAddPointNumBig(100)                      // 初始化大阈值最小添加点数
  , kMinAddFrontierPointNum(30)                  // 初始化前沿点最小添加点数
  , kCellExploringToCoveredThr(1)               // 初始化探索到覆盖的阈值
  , kCellCoveredToExploringThr(10)               // 初始化覆盖到探索的阈值
  , kCellExploringToAlmostCoveredThr(10)         // 初始化探索到几乎覆盖的阈值
  , kCellAlmostCoveredToExploringThr(20)         // 初始化几乎覆盖到探索的阈值
  , kCellUnknownToExploringThr(1)                // 初始化未知到探索的阈值
  , cur_keypose_id_(0)                           // 初始化当前关键姿态ID为0
  , cur_keypose_graph_node_ind_(0)               // 初始化当前关键姿态图节点索引为0
  , cur_robot_cell_ind_(-1)                      // 初始化当前机器人单元格索引为-1
  , prev_robot_cell_ind_(-1)                     // 初始化前一个机器人单元格索引为-1
  , cur_keypose_(0, 0, 0)                        // 初始化当前关键姿态为原点
  , initialized_(false)                           // 初始化：未初始化
  , use_keypose_graph_(false)                    // 初始化：不使用关键姿态图
{
  // 初始化机器人位置为原点
  robot_position_.x = 0.0;
  robot_position_.y = 0.0;
  robot_position_.z = 0.0;

  // 初始化网格世界原点为原点
  origin_.x = 0.0;
  origin_.y = 0.0;
  origin_.z = 0.0;

  // 创建网格大小向量
  Eigen::Vector3i grid_size(kRowNum, kColNum, kLevelNum);
  // 创建网格原点向量
  Eigen::Vector3d grid_origin(0.0, 0.0, 0.0);
  // 创建网格分辨率向量
  Eigen::Vector3d grid_resolution(kCellSize, kCellSize, kCellHeight);
  
  Cell cell_tmp;                                 // 创建临时单元格对象
  // 创建网格对象
  subspaces_ = std::make_unique<grid_ns::Grid<Cell>>(grid_size, cell_tmp, grid_origin, grid_resolution);
  
  // 遍历所有单元格，初始化为默认的Cell对象
  for (int i = 0; i < subspaces_->GetCellNumber(); ++i)
  {
    subspaces_->GetCell(i) = grid_world_ns::Cell();
  }

  // 初始化回家位置为原点
  home_position_.x() = 0.0;
  home_position_.y() = 0.0;
  home_position_.z() = 0.0;

  // 初始化当前关键姿态图节点位置为原点
  cur_keypose_graph_node_position_.x = 0.0;
  cur_keypose_graph_node_position_.y = 0.0;
  cur_keypose_graph_node_position_.z = 0.0;

  set_home_ = false;                             // 初始化：未设置回家位置
  return_home_ = false;                          // 初始化：不需要回家
}

// 从ROS参数服务器读取配置参数
void GridWorld::ReadParameters(ros::NodeHandle& nh)
{
  // 读取网格世界X方向数量，默认值为121
  kRowNum = misc_utils_ns::getParam<int>(nh, "kGridWorldXNum", 121);
  // 读取网格世界Y方向数量，默认值为121
  kColNum = misc_utils_ns::getParam<int>(nh, "kGridWorldYNum", 121);
  // 读取网格世界Z方向数量，默认值为121
  kLevelNum = misc_utils_ns::getParam<int>(nh, "kGridWorldZNum", 121);
  
  // 读取视点管理器X方向数量，默认值为40
  int viewpoint_number = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/number_x", 40);
  // 读取视点管理器X方向分辨率，默认值为1.0
  double viewpoint_resolution = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_x", 1.0);
  
  // 计算单元格大小：视点数量 * 视点分辨率 / 5
  kCellSize = viewpoint_number * viewpoint_resolution / 5;
  // 读取网格世界单元格高度，默认值为8.0
  kCellHeight = misc_utils_ns::getParam<double>(nh, "kGridWorldCellHeight", 8.0);
  // 读取网格世界附近网格数量，默认值为5
  KNearbyGridNum = misc_utils_ns::getParam<int>(nh, "kGridWorldNearbyGridNum", 5);
  
  // 读取小阈值最小添加点数，默认值为60
  kMinAddPointNumSmall = misc_utils_ns::getParam<int>(nh, "kMinAddPointNumSmall", 60);
  // 读取大阈值最小添加点数，默认值为100
  kMinAddPointNumBig = misc_utils_ns::getParam<int>(nh, "kMinAddPointNumBig", 100);
  // 读取前沿点最小添加点数，默认值为30
  kMinAddFrontierPointNum = misc_utils_ns::getParam<int>(nh, "kMinAddFrontierPointNum", 30);
  
  // 读取探索到覆盖的阈值，默认值为1
  kCellExploringToCoveredThr = misc_utils_ns::getParam<int>(nh, "kCellExploringToCoveredThr", 1);
  // 读取覆盖到探索的阈值，默认值为10
  kCellCoveredToExploringThr = misc_utils_ns::getParam<int>(nh, "kCellCoveredToExploringThr", 10);
  // 读取探索到几乎覆盖的阈值，默认值为10
  kCellExploringToAlmostCoveredThr = misc_utils_ns::getParam<int>(nh, "kCellExploringToAlmostCoveredThr", 10);
  // 读取几乎覆盖到探索的阈值，默认值为20
  kCellAlmostCoveredToExploringThr = misc_utils_ns::getParam<int>(nh, "kCellAlmostCoveredToExploringThr", 20);
  // 读取未知到探索的阈值，默认值为1
  kCellUnknownToExploringThr = misc_utils_ns::getParam<int>(nh, "kCellUnknownToExploringThr", 1);
}

// 更新邻居单元格，基于机器人位置
void GridWorld::UpdateNeighborCells(const geometry_msgs::Point& robot_position)
{
  // 如果网格世界还未初始化
  if (!initialized_)
  {
    initialized_ = true;                         // 标记为已初始化
    
    // 计算网格世界原点：机器人位置减去网格尺寸的一半
    origin_.x = robot_position.x - (kCellSize * kRowNum) / 2;
    origin_.y = robot_position.y - (kCellSize * kColNum) / 2;
    origin_.z = robot_position.z - (kCellHeight * kLevelNum) / 2;
    
    // 设置网格的原点
    subspaces_->SetOrigin(Eigen::Vector3d(origin_.x, origin_.y, origin_.z));
    
    // 更新所有单元格的中心位置
    for (int i = 0; i < kRowNum; i++)           // 遍历行
    {
      for (int j = 0; j < kColNum; j++)         // 遍历列
      {
        for (int k = 0; k < kLevelNum; k++)     // 遍历层
        {
          // 获取子空间中心位置
          Eigen::Vector3d subspace_center_position = subspaces_->Sub2Pos(i, j, k);
          // 转换为geometry_msgs::Point类型
          geometry_msgs::Point subspace_center_geo_position;
          subspace_center_geo_position.x = subspace_center_position.x();
          subspace_center_geo_position.y = subspace_center_position.y();
          subspace_center_geo_position.z = subspace_center_position.z();
          
          // 设置单元格位置
          subspaces_->GetCell(i, j, k).SetPosition(subspace_center_geo_position);
          // 设置单元格的路线图连接点
          subspaces_->GetCell(i, j, k).SetRoadmapConnectionPoint(subspace_center_position);
        }
      }
    }
  }

  // 获取邻居单元格
  std::vector<int> prev_neighbor_cell_indices = neighbor_cell_indices_; // 保存之前的邻居单元格索引
  neighbor_cell_indices_.clear();                // 清空当前邻居单元格索引列表
  
  int N = KNearbyGridNum / 2;                   // 计算邻居范围的一半
  int M = 1;                                     // 垂直方向的邻居范围设为1
  
  // 获取机器人位置附近的邻居单元格索引
  GetNeighborCellIndices(robot_position, Eigen::Vector3i(N, N, M), neighbor_cell_indices_);

  // 遍历新的邻居单元格索引
  for (const auto& cell_ind : neighbor_cell_indices_)
  {
    // 检查是否是新的邻居单元格（之前不在邻居列表中）
    if (std::find(prev_neighbor_cell_indices.begin(), prev_neighbor_cell_indices.end(), cell_ind) ==
        prev_neighbor_cell_indices.end())
    {
      // 为新邻居单元格增加访问计数
      // subspaces_->GetCell(cell_ind).AddVisitCount();
      subspaces_->GetCell(cell_ind).AddVisitCount();
    }
  }
}

// 更新机器人位置
void GridWorld::UpdateRobotPosition(const geometry_msgs::Point& robot_position)
{
  robot_position_ = robot_position;              // 更新机器人位置
  
  // 获取机器人所在的单元格索引
  int robot_cell_ind = GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z);
  
  // 如果机器人移动到新的单元格
  if (cur_robot_cell_ind_ != robot_cell_ind)
  {
    prev_robot_cell_ind_ = cur_robot_cell_ind_; // 保存前一个单元格索引
    cur_robot_cell_ind_ = robot_cell_ind;       // 更新当前单元格索引
  }
}

// 更新单元格的关键姿态图节点
void GridWorld::UpdateCellKeyposeGraphNodes(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  // 获取关键姿态图中已连接的节点索引
  std::vector<int> keypose_graph_connected_node_indices = keypose_graph->GetConnectedGraphNodeIndices();

  // 清空所有探索状态单元格的图节点索引
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (subspaces_->GetCell(i).GetStatus() == CellStatus::EXPLORING)
    {
      subspaces_->GetCell(i).ClearGraphNodeIndices();
    }
  }
  
  // 遍历所有已连接的图节点
  for (const auto& node_ind : keypose_graph_connected_node_indices)
  {
    // 获取节点位置
    geometry_msgs::Point node_position = keypose_graph->GetNodePosition(node_ind);
    // 获取节点所在的单元格索引
    int cell_ind = GetCellInd(node_position.x, node_position.y, node_position.z);
    
    // 检查单元格索引是否在有效范围内
    if (subspaces_->InRange(cell_ind))
    {
      // 如果单元格状态为探索中，添加图节点索引
      if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING)
      {
        subspaces_->GetCell(cell_ind).AddGraphNode(node_ind);
      }
    }
  }
}

// 检查两个单元格是否为邻居（相邻）
bool GridWorld::AreNeighbors(int cell_ind1, int cell_ind2)
{
  // 获取两个单元格的坐标
  Eigen::Vector3i cell_sub1 = subspaces_->Ind2Sub(cell_ind1);
  Eigen::Vector3i cell_sub2 = subspaces_->Ind2Sub(cell_ind2);
  
  // 计算坐标差
  Eigen::Vector3i diff = cell_sub1 - cell_sub2;
  
  // 如果曼哈顿距离为1，说明是邻居
  if (std::abs(diff.x()) + std::abs(diff.y()) + std::abs(diff.z()) == 1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// 根据3D坐标获取单元格索引
int GridWorld::GetCellInd(double qx, double qy, double qz)
{
  // 将3D坐标转换为网格坐标
  Eigen::Vector3i sub = subspaces_->Pos2Sub(qx, qy, qz);
  
  // 检查坐标是否在有效范围内
  if (subspaces_->InRange(sub))
  {
    return subspaces_->Sub2Ind(sub);             // 返回单元格索引
  }
  else
  {
    return -1;                                   // 超出范围返回-1
  }
}

// 根据3D坐标获取单元格的行、列、层索引
void GridWorld::GetCellSub(int& row_idx, int& col_idx, int& level_idx, double qx, double qy, double qz)
{
  // 将3D坐标转换为网格坐标
  Eigen::Vector3i sub = subspaces_->Pos2Sub(qx, qy, qz);
  
  // 检查并设置行索引
  row_idx = (sub.x() >= 0 && sub.x() < kRowNum) ? sub.x() : -1;
  // 检查并设置列索引
  col_idx = (sub.y() >= 0 && sub.y() < kColNum) ? sub.y() : -1;
  // 检查并设置层索引
  level_idx = (sub.z() >= 0 && sub.z() < kLevelNum) ? sub.z() : -1;
}

// 根据3D点获取单元格坐标
Eigen::Vector3i GridWorld::GetCellSub(const Eigen::Vector3d& point)
{
  return subspaces_->Pos2Sub(point);             // 返回网格坐标
}

// 获取可视化标记
void GridWorld::GetMarker(visualization_msgs::Marker& marker)
{
  marker.points.clear();                         // 清空标记点列表
  marker.colors.clear();                         // 清空标记颜色列表
  
  // 设置标记的缩放
  marker.scale.x = kCellSize;                    // X方向缩放为单元格大小
  marker.scale.y = kCellSize;                    // Y方向缩放为单元格大小
  marker.scale.z = kCellHeight;                  // Z方向缩放为单元格高度

  // 初始化各种状态的单元格计数
  int exploring_count = 0;                       // 探索中的单元格计数
  int covered_count = 0;                         // 已覆盖的单元格计数
  int unseen_count = 0;                          // 未见过的单元格计数

  // 遍历所有单元格
  for (int i = 0; i < kRowNum; i++)             // 遍历行
  {
    for (int j = 0; j < kColNum; j++)           // 遍历列
    {
      for (int k = 0; k < kLevelNum; k++)       // 遍历层
      {
        // 获取单元格索引
        int cell_ind = subspaces_->Sub2Ind(i, j, k);
        // 获取单元格中心位置
        geometry_msgs::Point cell_center = subspaces_->GetCell(cell_ind).GetPosition();
        
        std_msgs::ColorRGBA color;               // 创建颜色对象
        bool add_marker = false;                 // 是否添加标记的标志
        
        // 根据单元格状态设置颜色
        if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN)
        {
          color.r = 0.0;                         // 未见过：蓝色
          color.g = 0.0;
          color.b = 1.0;
          color.a = 0.1;
          unseen_count++;                        // 增加未见过计数
        }
        else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED)
        {
          color.r = 1.0;                         // 已覆盖：黄色
          color.g = 1.0;
          color.b = 0.0;
          color.a = 0.1;
          covered_count++;                       // 增加已覆盖计数
        }
        else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING)
        {
          color.r = 0.0;                         // 探索中：绿色
          color.g = 1.0;
          color.b = 0.0;
          color.a = 0.1;
          exploring_count++;                     // 增加探索中计数
          add_marker = true;                     // 标记为需要添加标记
        }
        else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::NOGO)
        {
          color.r = 1.0;                         // 禁止区域：红色
          color.g = 0.0;
          color.b = 0.0;
          color.a = 0.1;
        }
        else
        {
          color.r = 0.8;                         // 其他状态：灰色
          color.g = 0.8;
          color.b = 0.8;
          color.a = 0.1;
        }
        
        // 如果需要添加标记
        if (add_marker)
        {
          marker.colors.push_back(color);        // 添加颜色到标记
          marker.points.push_back(cell_center);  // 添加点到标记
        }
      }
    }
  }
  //        // Color neighbor cells differently
  //        for(const auto & ind : neighbor_cell_indices_){
  //            if(cells_[ind].GetStatus() == CellStatus::UNSEEN) continue;
  //            marker.colors[ind].a = 0.8;
  //        }
}

// 获取可视化点云
void GridWorld::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud)
{
  vis_cloud->points.clear();                     // 清空可视化点云
  
  // 遍历所有单元格
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    // 获取单元格状态
    CellStatus cell_status = subspaces_->GetCell(i).GetStatus();
    
    // 如果单元格有连接的单元格
    if (!subspaces_->GetCell(i).GetConnectedCellIndices().empty())
    {
      pcl::PointXYZI point;                      // 创建PCL点对象
      // 获取单元格的路线图连接点
      Eigen::Vector3d position = subspaces_->GetCell(i).GetRoadmapConnectionPoint();
      
      // 设置点的坐标
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      point.intensity = i;                       // 设置强度为单元格索引
      
      // 将点添加到可视化点云
      vis_cloud->points.push_back(point);
    }
  }
}

// 向指定单元格添加视点索引
void GridWorld::AddViewPointToCell(int cell_ind, int viewpoint_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));     // 断言：单元格索引在有效范围内
  subspaces_->GetCell(cell_ind).AddViewPoint(viewpoint_ind); // 添加视点索引
}

// 向指定单元格添加图节点索引
void GridWorld::AddGraphNodeToCell(int cell_ind, int node_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));     // 断言：单元格索引在有效范围内
  subspaces_->GetCell(cell_ind).AddGraphNode(node_ind); // 添加图节点索引
}

// 清空指定单元格的视点索引
void GridWorld::ClearCellViewPointIndices(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));     // 断言：单元格索引在有效范围内
  subspaces_->GetCell(cell_ind).ClearViewPointIndices(); // 清空视点索引
}

// 获取指定单元格的视点索引列表
std::vector<int> GridWorld::GetCellViewPointIndices(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));     // 断言：单元格索引在有效范围内
  return subspaces_->GetCell(cell_ind).GetViewPointIndices(); // 返回视点索引列表
}

// 获取指定中心单元格周围的邻居单元格索引
void GridWorld::GetNeighborCellIndices(const Eigen::Vector3i& center_cell_sub, const Eigen::Vector3i& neighbor_range,
                                       std::vector<int>& neighbor_indices)
{
  int row_idx = 0;                               // 行索引
  int col_idx = 0;                               // 列索引
  int level_idx = 0;                             // 层索引
  
  // 遍历邻居范围
  for (int i = -neighbor_range.x(); i <= neighbor_range.x(); i++)     // X方向邻居
  {
    for (int j = -neighbor_range.y(); j <= neighbor_range.y(); j++)   // Y方向邻居
    {
      row_idx = center_cell_sub.x() + i;         // 计算行索引
      col_idx = center_cell_sub.y() + j;         // 计算列索引
      
      for (int k = -neighbor_range.z(); k <= neighbor_range.z(); k++) // Z方向邻居
      {
        level_idx = center_cell_sub.z() + k;     // 计算层索引
        
        // 创建邻居坐标
        Eigen::Vector3i sub(row_idx, col_idx, level_idx);
        
        // 检查坐标是否在边界内
        // if (SubInBound(row_idx, col_idx, level_idx))
        if (subspaces_->InRange(sub))
        {
          // 将邻居单元格索引添加到列表中
          // int ind = sub2ind(row_idx, col_idx, level_idx);
          int ind = subspaces_->Sub2Ind(sub);
          neighbor_cell_indices_.push_back(ind);
        }
      }
    }
  }
}

// 根据位置获取邻居单元格索引
void GridWorld::GetNeighborCellIndices(const geometry_msgs::Point& position, const Eigen::Vector3i& neighbor_range,
                                       std::vector<int>& neighbor_indices)
{
  // 获取位置所在的单元格坐标
  Eigen::Vector3i center_cell_sub = GetCellSub(Eigen::Vector3d(position.x, position.y, position.z));

  // 调用重载函数获取邻居单元格索引
  GetNeighborCellIndices(center_cell_sub, neighbor_range, neighbor_indices);
}

// 获取所有探索状态的单元格索引
void GridWorld::GetExploringCellIndices(std::vector<int>& exploring_cell_indices)
{
  exploring_cell_indices.clear();                // 清空探索单元格索引列表
  
  // 遍历所有单元格
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    // 如果单元格状态为探索中，添加到列表
    if (subspaces_->GetCell(i).GetStatus() == CellStatus::EXPLORING)
    {
      exploring_cell_indices.push_back(i);
    }
  }
}

// 获取指定单元格的状态
CellStatus GridWorld::GetCellStatus(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));     // 断言：单元格索引在有效范围内
  return subspaces_->GetCell(cell_ind).GetStatus(); // 返回单元格状态
}

// 设置指定单元格的状态
void GridWorld::SetCellStatus(int cell_ind, grid_world_ns::CellStatus status)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));     // 断言：单元格索引在有效范围内
  subspaces_->GetCell(cell_ind).SetStatus(status); // 设置单元格状态
}

// 获取指定单元格的位置
geometry_msgs::Point GridWorld::GetCellPosition(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));     // 断言：单元格索引在有效范围内
  return subspaces_->GetCell(cell_ind).GetPosition(); // 返回单元格位置
}

// 设置指定单元格的机器人位置
void GridWorld::SetCellRobotPosition(int cell_ind, const geometry_msgs::Point& robot_position)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));     // 断言：单元格索引在有效范围内
  subspaces_->GetCell(cell_ind).SetRobotPosition(robot_position); // 设置机器人位置
}

// 获取指定单元格的机器人位置
geometry_msgs::Point GridWorld::GetCellRobotPosition(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));     // 断言：单元格索引在有效范围内
  return subspaces_->GetCell(cell_ind).GetRobotPosition(); // 返回机器人位置
}

// 增加指定单元格的访问计数
void GridWorld::CellAddVisitCount(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));     // 断言：单元格索引在有效范围内
  subspaces_->GetCell(cell_ind).AddVisitCount(); // 增加访问计数
}

// 获取指定单元格的访问计数
int GridWorld::GetCellVisitCount(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));     // 断言：单元格索引在有效范围内
  return subspaces_->GetCell(cell_ind).GetVisitCount(); // 返回访问计数
}

// 检查指定单元格的机器人位置是否已设置
bool GridWorld::IsRobotPositionSet(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));     // 断言：单元格索引在有效范围内
  return subspaces_->GetCell(cell_ind).IsRobotPositionSet(); // 返回机器人位置是否已设置
}

// 重置所有单元格
void GridWorld::Reset()
{
  // 遍历所有单元格，调用重置函数
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    subspaces_->GetCell(i).Reset();
  }
}

// 获取指定状态的单元格数量
int GridWorld::GetCellStatusCount(grid_world_ns::CellStatus status)
{
  int count = 0;                                 // 初始化计数器
  
  // 遍历所有单元格
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    // 如果单元格状态匹配，增加计数
    if (subspaces_->GetCell(i).GetStatus() == status)
    {
      count++;
    }
  }
  return count;                                  // 返回计数结果
}

// 更新单元格状态，基于视点管理器的信息
void GridWorld::UpdateCellStatus(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager)
{
  // 初始化各种状态的单元格计数
  int exploring_count = 0;                       // 探索中的单元格计数
  int unseen_count = 0;                          // 未见过的单元格计数
  int covered_count = 0;                         // 已覆盖的单元格计数
  
  // 统计各种状态的单元格数量
  for (int i = 0; i < subspaces_->GetCellNumber(); ++i)
  {
    if (subspaces_->GetCell(i).GetStatus() == CellStatus::EXPLORING)
    {
      exploring_count++;                         // 增加探索中计数
    }
    else if (subspaces_->GetCell(i).GetStatus() == CellStatus::UNSEEN)
    {
      unseen_count++;                            // 增加未见过计数
    }
    else if (subspaces_->GetCell(i).GetStatus() == CellStatus::COVERED)
    {
      covered_count++;                           // 增加已覆盖计数
    }
  }

  // 清空邻居单元格的视点索引，为重新分配做准备
  for (const auto& cell_ind : neighbor_cell_indices_)
  {
    subspaces_->GetCell(cell_ind).ClearViewPointIndices();
  }
  
  // 遍历所有候选视点索引，将它们分配到对应的单元格中
  for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
  {
    // 获取视点位置
    geometry_msgs::Point viewpoint_position = viewpoint_manager->GetViewPointPosition(viewpoint_ind);
    // 将视点位置转换为网格坐标
    Eigen::Vector3i sub =
        subspaces_->Pos2Sub(Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z));
    
    // 检查坐标是否在有效范围内
    if (subspaces_->InRange(sub))
    {
      // 获取视点所在的单元格索引
      int cell_ind = subspaces_->Sub2Ind(sub);
      // 向单元格添加视点索引
      AddViewPointToCell(cell_ind, viewpoint_ind);
      // 设置视点的单元格索引，建立双向关联
      viewpoint_manager->SetViewPointCellInd(viewpoint_ind, cell_ind);
    }
    else
    {
      // 输出错误信息：子空间坐标超出边界
      ROS_ERROR_STREAM("subspace sub out of bound: " << sub.transpose());
    }
  }

  // 遍历所有邻居单元格，更新它们的状态
  for (const auto& cell_ind : neighbor_cell_indices_)
  {
    // 如果单元格被其他机器人覆盖，跳过处理
    if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED_BY_OTHERS)
    {
      continue;
    }
    
    // 初始化各种计数器和标志，用于状态转换判断
    int candidate_count = 0;                     // 候选视点数量
    int selected_viewpoint_count = 0;            // 已选择的视点数量
    int above_big_threshold_count = 0;           // 超过大阈值的数量
    int above_small_threshold_count = 0;         // 超过小阈值的数量
    int above_frontier_threshold_count = 0;      // 超过前沿阈值的数量
    int highest_score_viewpoint_ind = -1;        // 最高分数的视点索引
    int highest_score = -1;                      // 最高分数
    
    // 遍历单元格的所有视点索引，统计各种指标
    for (const auto& viewpoint_ind : subspaces_->GetCell(cell_ind).GetViewPointIndices())
    {
      MY_ASSERT(viewpoint_manager->IsViewPointCandidate(viewpoint_ind)); // 断言：视点是候选视点
      candidate_count++;                         // 增加候选视点计数
      
      // 如果视点已被选择，增加计数
      if (viewpoint_manager->ViewPointSelected(viewpoint_ind))
      {
        selected_viewpoint_count++;
      }
      
      // 如果视点已被访问，跳过处理
      if (viewpoint_manager->ViewPointVisited(viewpoint_ind))
      {
        continue;
      }
      
      // 获取视点的覆盖点数和前沿点数，用于评分
      int score = viewpoint_manager->GetViewPointCoveredPointNum(viewpoint_ind);
      int frontier_score = viewpoint_manager->GetViewPointCoveredFrontierPointNum(viewpoint_ind);
      
      // 更新最高分数和对应视点索引，用于后续决策
      if (score > highest_score)
      {
        highest_score = score;
        highest_score_viewpoint_ind = viewpoint_ind;
      }
      
      // 统计超过各种阈值的数量，用于状态转换判断
      if (score > kMinAddPointNumSmall)
      {
        above_small_threshold_count++;           // 增加超过小阈值的计数
      }
      if (score > kMinAddPointNumBig)
      {
        above_big_threshold_count++;             // 增加超过大阈值的计数
      }
      if (frontier_score > kMinAddFrontierPointNum)
      {
        above_frontier_threshold_count++;        // 增加超过前沿阈值的计数
      }
    }
    
    // 探索状态到覆盖状态的转换条件：前沿点和小阈值计数都很少，且没有选中的视点
    if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING &&
        above_frontier_threshold_count < kCellExploringToCoveredThr &&
        above_small_threshold_count < kCellExploringToCoveredThr && selected_viewpoint_count == 0 &&
        candidate_count > 0)
    {
      subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED); // 设置为已覆盖状态
    }
    // 覆盖状态到探索状态的转换条件：大阈值或前沿阈值计数达到要求
    else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED &&
             (above_big_threshold_count >= kCellCoveredToExploringThr ||
              above_frontier_threshold_count >= kCellCoveredToExploringThr))
    {
      subspaces_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING); // 设置为探索状态
      almost_covered_cell_indices_.push_back(cell_ind); // 添加到几乎覆盖的单元格列表
    }
    // 探索状态到几乎覆盖状态的转换条件：没有选中的视点但有候选视点
    else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING && selected_viewpoint_count == 0 &&
             candidate_count > 0)
    {
      almost_covered_cell_indices_.push_back(cell_ind); // 添加到几乎覆盖的单元格列表
    }
    // 如果有选中的视点，设置为探索状态
    else if (subspaces_->GetCell(cell_ind).GetStatus() != CellStatus::COVERED && selected_viewpoint_count > 0)
    {
      subspaces_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING); // 设置为探索状态
      // 从几乎覆盖列表中移除，因为现在有选中的视点
      almost_covered_cell_indices_.erase(
          std::remove(almost_covered_cell_indices_.begin(), almost_covered_cell_indices_.end(), cell_ind),
          almost_covered_cell_indices_.end());
    }
    // 如果单元格没有候选视点，需要特殊处理
    else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING && candidate_count == 0)
    {
      // 第一次访问的情况：访问次数为1且没有图节点
      if (subspaces_->GetCell(cell_ind).GetVisitCount() == 1 &&
          subspaces_->GetCell(cell_ind).GetGraphNodeIndices().empty())
      {
        subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED); // 设置为已覆盖状态
      }
      else
      {
        // 获取单元格位置
        geometry_msgs::Point cell_position = subspaces_->GetCell(cell_ind).GetPosition();
        // 计算到机器人的XY平面距离
        double xy_dist_to_robot =
            misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(cell_position, robot_position_);
        // 计算到机器人的Z轴距离
        double z_dist_to_robot = std::abs(cell_position.z - robot_position_.z);
        
        // 如果距离机器人很近（XY距离小于单元格大小，Z距离小于单元格高度的80%），设置为已覆盖状态
        if (xy_dist_to_robot < kCellSize && z_dist_to_robot < kCellHeight * 0.8)
        {
          subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
        }
      }
    }

    // 如果单元格状态为探索中且有候选视点，更新相关信息
    if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING && candidate_count > 0)
    {
      // 设置机器人位置，用于后续路径规划
      subspaces_->GetCell(cell_ind).SetRobotPosition(robot_position_);
      // 设置关键姿态ID，用于跟踪机器人状态
      subspaces_->GetCell(cell_ind).SetKeyposeID(cur_keypose_id_);
    }
  }
  
  // 处理几乎覆盖的单元格：检查是否应该转换为已覆盖状态
  for (const auto& cell_ind : almost_covered_cell_indices_)
  {
    // 如果单元格不在邻居列表中，说明机器人已经离开该区域，设置为已覆盖状态
    if (std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), cell_ind) ==
        neighbor_cell_indices_.end())
    {
      subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED); // 设置为已覆盖状态
      // 从几乎覆盖列表中移除，因为已经转换为已覆盖状态
      almost_covered_cell_indices_.erase(
          std::remove(almost_covered_cell_indices_.begin(), almost_covered_cell_indices_.end(), cell_ind),
          almost_covered_cell_indices_.end());
    }
  }
}

// 解决全局旅行商问题（TSP），生成探索路径
exploration_path_ns::ExplorationPath GridWorld::SolveGlobalTSP(
    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
    std::vector<int>& ordered_cell_indices, const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  /****** 获取与机器人位置关联的关键姿态图节点 *****/
  double min_dist_to_robot = DBL_MAX;                                    // 初始化到机器人的最小距离为最大值
  geometry_msgs::Point global_path_robot_position = robot_position_;     // 初始化全局路径机器人位置为当前机器人位置
  Eigen::Vector3d eigen_robot_position(robot_position_.x, robot_position_.y, robot_position_.z); // 转换为Eigen向量
  
  // 获取最近的已连接节点
  int closest_node_ind = 0;                                              // 最近节点索引
  double closest_node_dist = DBL_MAX;                                    // 最近节点距离
  keypose_graph->GetClosestConnectedNodeIndAndDistance(robot_position_, closest_node_ind, closest_node_dist);
  
  // 如果最近节点距离小于单元格大小的一半且在有效范围内，使用该节点位置
  if (closest_node_dist < kCellSize / 2 && closest_node_ind >= 0 && closest_node_ind < keypose_graph->GetNodeNum())
  {
    global_path_robot_position = keypose_graph->GetNodePosition(closest_node_ind);
  }
  // 如果当前关键姿态图节点索引有效，使用该节点位置
  else if (cur_keypose_graph_node_ind_ >= 0 && cur_keypose_graph_node_ind_ < keypose_graph->GetNodeNum())
  {
    // ROS_WARN("GridWorld::SolveGlobalTSP: using nearest keypose node for robot position");
    global_path_robot_position = keypose_graph->GetNodePosition(cur_keypose_graph_node_ind_);
  }
  // 否则，使用邻居单元格的路线图连接点
  else
  {
    // ROS_WARN("GridWorld::SolveGlobalTSP: using neighbor cell roadmap connection points for robot position");
    for (int i = 0; i < neighbor_cell_indices_.size(); i++)
    {
      int cell_ind = neighbor_cell_indices_[i];
      // 如果单元格的路线图连接点已设置
      if (subspaces_->GetCell(cell_ind).IsRoadmapConnectionPointSet())
      {
        Eigen::Vector3d roadmap_connection_point = subspaces_->GetCell(cell_ind).GetRoadmapConnectionPoint();
        // 如果连接点在局部规划视野内
        if (viewpoint_manager->InLocalPlanningHorizon(roadmap_connection_point))
        {
          // 计算到机器人的距离
          double dist_to_robot = (roadmap_connection_point - eigen_robot_position).norm();
          // 如果距离更近，更新最小距离和位置
          if (dist_to_robot < min_dist_to_robot)
          {
            min_dist_to_robot = dist_to_robot;
            global_path_robot_position.x = roadmap_connection_point.x();
            global_path_robot_position.y = roadmap_connection_point.y();
            global_path_robot_position.z = roadmap_connection_point.z();
          }
        }
      }
    }
  }

  /****** 获取所有已连接的探索单元格 *****/
  exploration_path_ns::ExplorationPath global_path;                      // 创建全局探索路径对象
  std::vector<geometry_msgs::Point> exploring_cell_positions;           // 探索单元格位置列表
  std::vector<int> exploring_cell_indices;                              // 探索单元格索引列表
  
  // 遍历所有单元格，找到探索状态的单元格
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (subspaces_->GetCell(i).GetStatus() == CellStatus::EXPLORING)
    {
      // 如果单元格不在邻居列表中，或者单元格没有视点但访问次数大于1
      if (std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), i) == neighbor_cell_indices_.end() ||
          (subspaces_->GetCell(i).GetViewPointIndices().empty() && subspaces_->GetCell(i).GetVisitCount() > 1))
      {
        // 如果不使用关键姿态图或图为空，使用直线连接
        if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeNum() == 0)
        {
          // 使用直线连接
          exploring_cell_positions.push_back(GetCellPosition(i));
          exploring_cell_indices.push_back(i);
        }
        else
        {
          // 使用关键姿态图连接
          Eigen::Vector3d connection_point = subspaces_->GetCell(i).GetRoadmapConnectionPoint();
          geometry_msgs::Point connection_point_geo;
          connection_point_geo.x = connection_point.x();
          connection_point_geo.y = connection_point.y();
          connection_point_geo.z = connection_point.z();

          bool reachable = false;
          // 检查连接点是否可达
          if (keypose_graph->IsPositionReachable(connection_point_geo))
          {
            reachable = true;
          }
          else
          {
            // 检查单元格内所有关键姿态图节点，看是否有已连接的节点
            double min_dist = DBL_MAX;
            double min_dist_node_ind = -1;
            for (const auto& node_ind : subspaces_->GetCell(i).GetGraphNodeIndices())
            {
              geometry_msgs::Point node_position = keypose_graph->GetNodePosition(node_ind);
              double dist = misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(
                  node_position, connection_point_geo);
              if (dist < min_dist)
              {
                min_dist = dist;
                min_dist_node_ind = node_ind;
              }
            }
            // 如果找到最近的节点，使用该节点位置
            if (min_dist_node_ind >= 0 && min_dist_node_ind < keypose_graph->GetNodeNum())
            {
              reachable = true;
              connection_point_geo = keypose_graph->GetNodePosition(min_dist_node_ind);
            }
          }
          // 如果可达，添加到探索单元格列表
          if (reachable)
          {
            exploring_cell_positions.push_back(connection_point_geo);
            exploring_cell_indices.push_back(i);
          }
        }
      }
    }
  }

  /****** 返回起点（回家） ******/
  // 如果没有探索单元格，设置回家标志并生成回家路径
  if (exploring_cell_indices.empty())
  {
    return_home_ = true;                           // 设置回家标志

    geometry_msgs::Point home_position;            // 回家位置

    nav_msgs::Path return_home_path;               // 回家路径
    // 如果不使用关键姿态图或图为空，使用直线路径
    if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeNum() == 0)
    {
      geometry_msgs::PoseStamped robot_pose;       // 机器人姿态
      robot_pose.pose.position = robot_position_;

      geometry_msgs::PoseStamped home_pose;        // 回家姿态
      home_pose.pose.position = home_position;
      return_home_path.poses.push_back(robot_pose); // 添加机器人位置
      return_home_path.poses.push_back(home_pose);  // 添加回家位置
    }
    else
    {
      // 使用关键姿态图生成回家路径
      home_position = keypose_graph->GetFirstKeyposePosition(); // 获取第一个关键姿态位置作为回家位置
      keypose_graph->GetShortestPath(global_path_robot_position, home_position, true, return_home_path, false);
      if (return_home_path.poses.size() >= 2)
      {
        global_path.FromPath(return_home_path);    // 从路径创建全局路径
        global_path.nodes_.front().type_ = exploration_path_ns::NodeType::ROBOT; // 设置第一个节点为机器人类型

        // 设置中间节点为全局途经点类型
        for (int i = 1; i < global_path.nodes_.size() - 1; i++)
        {
          global_path.nodes_[i].type_ = exploration_path_ns::NodeType::GLOBAL_VIA_POINT;
        }
        global_path.nodes_.back().type_ = exploration_path_ns::NodeType::HOME; // 设置最后一个节点为回家类型
        // 使其成为循环路径：从倒数第二个节点开始反向添加
        for (int i = global_path.nodes_.size() - 2; i >= 0; i--)
        {
          global_path.Append(global_path.nodes_[i]);
        }
      }
      else
      {
        // ROS_ERROR("Cannot find path home");
        // TODO: find a path
      }
    }
    return global_path;                            // 返回回家路径
  }

  return_home_ = false;                            // 重置回家标志

  // 将当前机器人位置添加到列表末尾
  exploring_cell_positions.push_back(global_path_robot_position);
  exploring_cell_indices.push_back(-1);            // -1表示机器人位置

  /******* 构建距离矩阵 *****/
  // 创建距离矩阵，大小为探索单元格位置数量的平方
  std::vector<std::vector<int>> distance_matrix(exploring_cell_positions.size(),
                                                std::vector<int>(exploring_cell_positions.size(), 0));
  
  // 计算下三角矩阵的距离
  for (int i = 0; i < exploring_cell_positions.size(); i++)
  {
    for (int j = 0; j < i; j++)
    {
      // 如果不使用关键姿态图或图为空，使用直线距离
      if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeNum() == 0)
      {
        // 使用直线连接，距离乘以10转换为整数
        distance_matrix[i][j] =
            static_cast<int>(10 * misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(
                                      exploring_cell_positions[i], exploring_cell_positions[j]));
      }
      else
      {
        // 使用关键姿态图计算最短路径距离
        nav_msgs::Path path_tmp;
        distance_matrix[i][j] =
            static_cast<int>(10 * keypose_graph->GetShortestPath(exploring_cell_positions[i],
                                                                 exploring_cell_positions[j], false, path_tmp, false));
      }
    }
  }

  // 复制下三角矩阵到上三角矩阵，使矩阵对称
  for (int i = 0; i < exploring_cell_positions.size(); i++)
  {
    for (int j = i + 1; j < exploring_cell_positions.size(); j++)
    {
      distance_matrix[i][j] = distance_matrix[j][i];
    }
  }

  /****** 解决旅行商问题（TSP） ******/
  tsp_solver_ns::DataModel data_model;             // 创建TSP数据模型
  data_model.distance_matrix = distance_matrix;    // 设置距离矩阵
  data_model.depot = exploring_cell_positions.size() - 1; // 设置起始点（机器人位置）

  tsp_solver_ns::TSPSolver tsp_solver(data_model); // 创建TSP求解器
  tsp_solver.Solve();                              // 求解TSP
  std::vector<int> node_index;                     // 存储求解结果（节点访问顺序）
  tsp_solver.getSolutionNodeIndex(node_index, false); // 获取解决方案的节点索引

  ordered_cell_indices.clear();                    // 清空有序单元格索引列表

  // 在末尾添加第一个节点，使其成为循环路径
  if (!node_index.empty())
  {
    node_index.push_back(node_index[0]);
  }

  // 如果不使用关键姿态图或图为空，使用直线连接构建路径
  if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeNum() == 0)
  {
    // 遍历所有节点索引，创建全局视点节点
    for (int i = 0; i < node_index.size(); i++)
    {
      int cell_ind = node_index[i];                // 获取单元格索引
      geometry_msgs::PoseStamped pose;             // 创建姿态对象
      pose.pose.position = exploring_cell_positions[cell_ind]; // 设置位置
      // 创建全局视点节点
      exploration_path_ns::Node node(exploring_cell_positions[cell_ind],
                                     exploration_path_ns::NodeType::GLOBAL_VIEWPOINT);
      node.global_subspace_index_ = exploring_cell_indices[cell_ind]; // 设置全局子空间索引
      global_path.Append(node);                    // 添加到全局路径
      ordered_cell_indices.push_back(exploring_cell_indices[cell_ind]); // 添加到有序单元格索引
    }
  }
  else
  {
    // 使用关键姿态图构建路径
    geometry_msgs::Point cur_position;             // 当前位置
    geometry_msgs::Point next_position;            // 下一个位置
    int cur_keypose_id;                            // 当前关键姿态ID
    int next_keypose_id;                           // 下一个关键姿态ID
    int cur_ind;                                   // 当前索引
    int next_ind;                                  // 下一个索引

    // 遍历所有节点对，构建路径
    for (int i = 0; i < node_index.size() - 1; i++)
    {
      cur_ind = node_index[i];                     // 当前节点索引
      next_ind = node_index[i + 1];                // 下一个节点索引
      cur_position = exploring_cell_positions[cur_ind]; // 当前位置
      next_position = exploring_cell_positions[next_ind]; // 下一个位置

      nav_msgs::Path keypose_path;                 // 关键姿态路径
      keypose_graph->GetShortestPath(cur_position, next_position, true, keypose_path, false); // 获取最短路径

      // 创建探索路径节点
      exploration_path_ns::Node node(Eigen::Vector3d(cur_position.x, cur_position.y, cur_position.z));
      if (i == 0)
      {
        node.type_ = exploration_path_ns::NodeType::ROBOT; // 第一个节点为机器人类型
      }
      else
      {
        node.type_ = exploration_path_ns::NodeType::GLOBAL_VIEWPOINT; // 其他节点为全局视点类型
      }
      node.global_subspace_index_ = exploring_cell_indices[cur_ind]; // 设置全局子空间索引
      global_path.Append(node);                    // 添加到全局路径

      ordered_cell_indices.push_back(exploring_cell_indices[cur_ind]); // 添加到有序单元格索引

      // 填充中间路径
      if (keypose_path.poses.size() >= 2)
      {
        for (int j = 1; j < keypose_path.poses.size() - 1; j++)
        {
          geometry_msgs::Point node_position;      // 节点位置
          node_position = keypose_path.poses[j].pose.position; // 获取位置
          // 创建全局途经点节点
          exploration_path_ns::Node keypose_node(node_position, exploration_path_ns::NodeType::GLOBAL_VIA_POINT);
          keypose_node.keypose_graph_node_ind_ = static_cast<int>(keypose_path.poses[i].pose.orientation.x); // 设置关键姿态图节点索引
          global_path.Append(keypose_node);        // 添加到全局路径
        }
      }
    }
    // 在末尾添加机器人节点，形成循环
    if (!global_path.nodes_.empty())
    {
      global_path.Append(global_path.nodes_[0]);
    }
  }

  // 调试输出：打印路径顺序
  // std::cout << "path order: ";
  // for (int i = 0; i < ordered_cell_indices.size(); i++)
  // {
  //   std::cout << ordered_cell_indices[i] << " -> ";
  // }
  // std::cout << std::endl;

  return global_path;                              // 返回全局探索路径
}

// 获取指定单元格的关键姿态ID
int GridWorld::GetCellKeyposeID(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));       // 断言：单元格索引在有效范围内
  return subspaces_->GetCell(cell_ind).GetKeyposeID(); // 返回关键姿态ID
}

// 获取单元格视点位置列表
void GridWorld::GetCellViewPointPositions(std::vector<Eigen::Vector3d>& viewpoint_positions)
{
  viewpoint_positions.clear();                     // 清空视点位置列表
  
  // 遍历所有单元格
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    // 如果单元格状态不是探索中，跳过
    if (subspaces_->GetCell(i).GetStatus() != grid_world_ns::CellStatus::EXPLORING)
    {
      continue;
    }
    // 如果单元格不在邻居列表中，添加到视点位置列表
    if (std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), i) == neighbor_cell_indices_.end())
    {
      viewpoint_positions.push_back(subspaces_->GetCell(i).GetViewPointPosition());
    }
  }
}

// 在单元格之间添加路径连接
void GridWorld::AddPathsInBetweenCells(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                                       const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  // 确定每个单元格的连接点
  for (int i = 0; i < neighbor_cell_indices_.size(); i++)
  {
    int cell_ind = neighbor_cell_indices_[i];
    // 如果单元格的路线图连接点已设置
    if (subspaces_->GetCell(cell_ind).IsRoadmapConnectionPointSet())
    {
      // 如果连接点在局部规划视野内且不碰撞，继续下一个单元格
      if (viewpoint_manager->InLocalPlanningHorizon(subspaces_->GetCell(cell_ind).GetRoadmapConnectionPoint()) &&
          !viewpoint_manager->InCollision(subspaces_->GetCell(cell_ind).GetRoadmapConnectionPoint()))
      {
        continue;
      }
      else
      {
        // 否则清空连接的单元格索引
        subspaces_->GetCell(cell_ind).ClearConnectedCellIndices();
      }
    }

    // 获取候选视点索引列表
    std::vector<int> candidate_viewpoint_indices = subspaces_->GetCell(cell_ind).GetViewPointIndices();
    if (!candidate_viewpoint_indices.empty())
    {
      // 找到距离单元格中心最近的视点作为连接点
      double min_dist = DBL_MAX;
      double min_dist_viewpoint_ind = candidate_viewpoint_indices.front();
      for (const auto& viewpoint_ind : candidate_viewpoint_indices)
      {
        geometry_msgs::Point viewpoint_position = viewpoint_manager->GetViewPointPosition(viewpoint_ind);
        double dist_to_cell_center = misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(
            viewpoint_position, subspaces_->GetCell(cell_ind).GetPosition());
        if (dist_to_cell_center < min_dist)
        {
          min_dist = dist_to_cell_center;
          min_dist_viewpoint_ind = viewpoint_ind;
        }
      }
      // 设置路线图连接点为最近的视点位置
      geometry_msgs::Point min_dist_viewpoint_position =
          viewpoint_manager->GetViewPointPosition(min_dist_viewpoint_ind);
      subspaces_->GetCell(cell_ind).SetRoadmapConnectionPoint(
          Eigen::Vector3d(min_dist_viewpoint_position.x, min_dist_viewpoint_position.y, min_dist_viewpoint_position.z));
      subspaces_->GetCell(cell_ind).SetRoadmapConnectionPointSet(true);
    }
  }

  // 遍历所有邻居单元格，建立单元格间的连接
  for (int i = 0; i < neighbor_cell_indices_.size(); i++)
  {
    int from_cell_ind = neighbor_cell_indices_[i];  // 起始单元格索引
    int viewpoint_num = subspaces_->GetCell(from_cell_ind).GetViewPointIndices().size(); // 视点数量
    if (viewpoint_num == 0)
    {
      continue;                                     // 如果没有视点，跳过
    }
    // 获取起始单元格已连接的单元格索引
    std::vector<int> from_cell_connected_cell_indices = subspaces_->GetCell(from_cell_ind).GetConnectedCellIndices();
    // 获取起始单元格的路线图连接位置
    Eigen::Vector3d from_cell_roadmap_connection_position =
        subspaces_->GetCell(from_cell_ind).GetRoadmapConnectionPoint();
    // 如果连接位置不在局部规划视野内，跳过
    if (!viewpoint_manager->InLocalPlanningHorizon(from_cell_roadmap_connection_position))
    {
      continue;
    }
    // 获取起始单元格的网格坐标
    Eigen::Vector3i from_cell_sub = subspaces_->Ind2Sub(from_cell_ind);
    std::vector<int> nearby_cell_indices;           // 附近单元格索引列表
    
    // 遍历所有相邻的单元格（曼哈顿距离为1）
    for (int x = -1; x <= 1; x++)
    {
      for (int y = -1; y <= 1; y++)
      {
        for (int z = -1; z <= 1; z++)
        {
          if (std::abs(x) + std::abs(y) + std::abs(z) == 1) // 只考虑直接相邻的单元格
          {
            Eigen::Vector3i neighbor_sub = from_cell_sub + Eigen::Vector3i(x, y, z); // 计算邻居坐标
            if (subspaces_->InRange(neighbor_sub))   // 检查坐标是否在有效范围内
            {
              int neighbor_ind = subspaces_->Sub2Ind(neighbor_sub); // 转换为索引
              nearby_cell_indices.push_back(neighbor_ind); // 添加到附近单元格列表
            }
          }
        }
      }
    }

    // 遍历所有附近单元格，尝试建立连接
    for (int j = 0; j < nearby_cell_indices.size(); j++)
    {
      int to_cell_ind = nearby_cell_indices[j];     // 目标单元格索引
      // 调试检查：确保两个单元格确实是邻居
      if (!AreNeighbors(from_cell_ind, to_cell_ind))
      {
        ROS_ERROR_STREAM("Cell " << from_cell_ind << " and " << to_cell_ind << " are not neighbors");
      }
      // 如果目标单元格没有视点，跳过
      if (subspaces_->GetCell(to_cell_ind).GetViewPointIndices().empty())
      {
        continue;
      }
      // 获取目标单元格已连接的单元格索引
      std::vector<int> to_cell_connected_cell_indices = subspaces_->GetCell(to_cell_ind).GetConnectedCellIndices();
      // 获取目标单元格的路线图连接位置
      Eigen::Vector3d to_cell_roadmap_connection_position =
          subspaces_->GetCell(to_cell_ind).GetRoadmapConnectionPoint();
      // 如果连接位置不在局部规划视野内，跳过
      if (!viewpoint_manager->InLocalPlanningHorizon(to_cell_roadmap_connection_position))
      {
        continue;
      }

      // 检查是否在关键姿态图中已有直接连接
      bool connected_in_keypose_graph = HasDirectKeyposeGraphConnection(
          keypose_graph, from_cell_roadmap_connection_position, to_cell_roadmap_connection_position);

      // 检查前向和后向连接状态
      bool forward_connected =
          std::find(from_cell_connected_cell_indices.begin(), from_cell_connected_cell_indices.end(), to_cell_ind) !=
          from_cell_connected_cell_indices.end();
      bool backward_connected = std::find(to_cell_connected_cell_indices.begin(), to_cell_connected_cell_indices.end(),
                                          from_cell_ind) != to_cell_connected_cell_indices.end();

      // 如果已有连接，跳过
      if (connected_in_keypose_graph)
      {
        continue;
      }

      // 获取两个连接点之间的最短路径
      nav_msgs::Path path_in_between = viewpoint_manager->GetViewPointShortestPath(
          from_cell_roadmap_connection_position, to_cell_roadmap_connection_position);

      // 如果路径有效，添加到关键姿态图
      if (PathValid(path_in_between, from_cell_ind, to_cell_ind))
      {
        path_in_between = misc_utils_ns::SimplifyPath(path_in_between); // 简化路径
        for (auto& pose : path_in_between.poses)
        {
          pose.pose.orientation.w = -1;             // 设置特殊标记
        }
        // 添加路径到关键姿态图
        keypose_graph->AddPath(path_in_between);
        // 验证连接是否成功建立
        bool connected = HasDirectKeyposeGraphConnection(keypose_graph, from_cell_roadmap_connection_position,
                                                         to_cell_roadmap_connection_position);
        if (!connected)
        {
          // 如果连接失败，重置两个单元格的路线图连接点
          subspaces_->GetCell(from_cell_ind).SetRoadmapConnectionPointSet(false);
          subspaces_->GetCell(to_cell_ind).SetRoadmapConnectionPointSet(false);
          subspaces_->GetCell(from_cell_ind).ClearConnectedCellIndices();
          subspaces_->GetCell(to_cell_ind).ClearConnectedCellIndices();
          continue;
        }
        else
        {
          // 连接成功，建立双向连接关系
          subspaces_->GetCell(from_cell_ind).AddConnectedCell(to_cell_ind);
          subspaces_->GetCell(to_cell_ind).AddConnectedCell(from_cell_ind);
        }
      }
    }
  }
}

// 检查路径是否有效（路径上的所有点都在指定的两个单元格内）
bool GridWorld::PathValid(const nav_msgs::Path& path, int from_cell_ind, int to_cell_ind)
{
  if (path.poses.size() >= 2)                       // 路径至少需要2个点
  {
    // 检查路径上的每个点
    for (const auto& pose : path.poses)
    {
      // 获取点所在的单元格索引
      int cell_ind = GetCellInd(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
      // 如果点不在指定的两个单元格内，路径无效
      if (cell_ind != from_cell_ind && cell_ind != to_cell_ind)
      {
        return false;
      }
    }
    return true;                                    // 所有点都在指定单元格内，路径有效
  }
  else
  {
    return false;                                   // 路径点数不足，无效
  }
}

// 检查两个位置之间是否有直接的关键姿态图连接
bool GridWorld::HasDirectKeyposeGraphConnection(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                                                const Eigen::Vector3d& start_position,
                                                const Eigen::Vector3d& goal_position)
{
  // 如果起始位置或目标位置不在关键姿态图中，返回false
  if (!keypose_graph->HasNode(start_position) || !keypose_graph->HasNode(goal_position))
  {
    return false;
  }

  // 搜索连接起始位置和目标位置的路径，使用最大路径长度约束
  geometry_msgs::Point geo_start_position;          // 转换为geometry_msgs::Point类型
  geo_start_position.x = start_position.x();
  geo_start_position.y = start_position.y();
  geo_start_position.z = start_position.z();

  geometry_msgs::Point geo_goal_position;           // 转换为geometry_msgs::Point类型
  geo_goal_position.x = goal_position.x();
  geo_goal_position.y = goal_position.y();
  geo_goal_position.z = goal_position.z();

  double max_path_length = kCellSize * 2;           // 最大路径长度为单元格大小的2倍
  nav_msgs::Path path;                              // 路径对象
  bool found_path =
      keypose_graph->GetShortestPathWithMaxLength(geo_start_position, geo_goal_position, max_path_length, false, path);
  return found_path;                                // 返回是否找到路径
}

}  // namespace grid_world_ns