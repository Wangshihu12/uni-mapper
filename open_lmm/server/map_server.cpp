
#include "map_server.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Symbol.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <open_lmm/common/pointcloud_utils.hpp>
#include <open_lmm/utils/config.hpp>

namespace open_lmm {

MapServer::MapServer() { parseConfig(); }

MapServer::~MapServer() {}

/**
 * [功能描述]：解析配置文件，初始化地图服务器的各项参数和设置
 * 从全局配置中读取数据目录、输出目录、智能体数量等信息，并创建必要的目录结构
 * @return 无返回值
 */
void MapServer::parseConfig() {
  // 获取根数据目录路径，这是所有智能体数据的父目录
  const fs::path root_data_dir = fs::path(GlobalConfig::get_root_data_dir());
  
  // 遍历所有子目录，为每个智能体构建完整的数据路径
  for (const std::string& sub_dir : GlobalConfig::get_sub_dir_list()) {
    // 将根目录与子目录拼接，形成每个智能体的完整数据路径
    data_dir_list_.push_back(fs::path(root_data_dir / sub_dir));
  }

  // 根据数据目录列表的大小确定智能体数量
  agent_num_ = data_dir_list_.size();

  // TODO(gil) : here? - 开发者待办事项，可能需要在此处添加额外配置
  // 获取输出保存目录路径，用于存储处理结果和优化后的数据
  output_save_dir_ = GlobalConfig::get_save_dir_path();
  // 创建输出目录，如果目录不存在则递归创建所有必要的父目录
  fs::create_directories(output_save_dir_);

  // 加载地图服务器的专用配置文件
  config_map_server_ =
      Config(GlobalConfig::get_global_config_path("config_map_server"));
  
  // 从配置文件中读取地图更新器使能标志，默认为true
  // 参数格式：section="map_server", key="enable_map_updater", 默认值=true
  enable_map_updater_ =
      config_map_server_->param<bool>("map_server", "enable_map_updater", true);
}

// TODO(gil) : refactor process - 开发者待办事项：重构处理流程
// TODO(gil): Refactor to remove centralized shared_data_ structure - 开发者待办事项：重构以移除集中式共享数据结构
/**
 * [功能描述]：执行SLAM系统的核心处理流程，包括地图对齐、地图更新和结果保存
 * 该函数协调多个智能体的数据处理，进行地图对齐优化，并根据配置决定是否进行地图更新
 * @return 无返回值
 */
void MapServer::process() {
  // 第一阶段：地图对齐处理
  // 遍历所有智能体，为每个智能体创建地图对齐器并进行数据处理
  for (int i = 0; i < agent_num_; i++) {
    // 为每个智能体分配唯一的机器人ID，从'A'开始递增（'A', 'B', 'C', ...）
    const char robot_id = 'A' + i;
    // 创建地图对齐器实例，传入机器人ID和共享数据指针
    MapAlginer map_aligner(robot_id, shared_data_);
    // 处理当前智能体的数据目录，执行地图对齐算法
    map_aligner.process(data_dir_list_[i]);
  }

  // 第二阶段：地图更新处理（可选）
  // 如果启用了地图更新器，则进行地图更新和静态地图生成
  if (enable_map_updater_) {
    // 再次遍历所有智能体，进行地图更新处理
    for (int i = 0; i < agent_num_; i++) {
      // 获取当前智能体的机器人ID
      const char robot_id = 'A' + i;
      // 创建地图更新器实例，传入机器人ID和共享数据指针
      MapUpdater map_updater(robot_id, shared_data_);
      // 处理当前智能体数据，生成静态地图（去除动态物体后的地图）
      auto static_map = map_updater.process(data_dir_list_[i]);
      
      // 构建输出文件路径
      fs::path output_save_dir_path(output_save_dir_);
      // 为每个智能体生成独立的全局地图文件名，格式：global_map_A.pcd
      fs::path output_map_file =
          output_save_dir_path /
          ("global_map_" + std::string{robot_id} + ".pcd");
      
      // 对静态地图进行下采样和范围过滤，减少数据量并提高处理效率
      // 参数：点云、体素大小(0.2m)、最小范围(0)、最大范围(0)、是否使用范围过滤(false)
      auto ds_static_map =
          downsampleWithRangeFilter(static_map, 0.2, 0, 0, false);
      
      // 以二进制压缩格式保存下采样后的静态地图到PCD文件
      pcl::io::savePCDFileBinaryCompressed(output_map_file, *ds_static_map);
    }
  }

  // 第三阶段：保存优化结果
  std::cout << "SAVING OPTIMIZED POSES & MAPS" << std::endl;
  // 保存所有智能体优化后的位姿数据
  saveOptimizedPoses(output_save_dir_);

  // 如果未启用地图更新器，则使用传统方式保存优化后的地图
  if (!enable_map_updater_) {
    saveOptimizedMap(output_save_dir_);
  }

  // 处理完成提示
  std::cout << "ALL PROCESSES DONE" << std::endl;
}

void MapServer::saveOptimizedPoses(const std::string& output_save_dir) {
  for (const auto& optimized_poses : shared_data_->db_optimized_poses) {
    fs::path output_save_dir_path(output_save_dir);
    const char agent_id = optimized_poses.first;
    fs::path output_pose_file =
        output_save_dir_path /
        ("optimized_poses_" + std::string{agent_id} + ".txt");
    std::ofstream file(output_pose_file);

    for (const auto& pose : optimized_poses.second) {
      int scan_idx = pose.first;
      Eigen::Matrix4d pose_matrix = pose.second.matrix();
      Eigen::Vector3d translation = pose_matrix.block<3, 1>(0, 3);
      Eigen::Quaterniond quaternion(pose_matrix.block<3, 3>(0, 0));
      // TODO(gil) : add save options (pose format, extension, delimiter)
      file << scan_idx << "," << translation.x() << "," << translation.y()
           << "," << translation.z() << "," << quaternion.x() << ","
           << quaternion.y() << "," << quaternion.z() << "," << quaternion.w()
           << "\n";
    }
    file.close();
  }
}

void MapServer::saveOptimizedMap(const std::string& output_save_dir) {
  for (const auto& optimized_poses : shared_data_->db_optimized_poses) {
    fs::path output_save_dir_path(output_save_dir_);
    const char agent_id = optimized_poses.first;

    pcl::PointCloud<pcl::PointXYZI>::Ptr optimized_map(
        new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto& pose : optimized_poses.second) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan(
          new pcl::PointCloud<pcl::PointXYZI>);
      int scan_idx = pose.first;
      Eigen::Matrix4d pose_matrix = pose.second.matrix();
      pcl::transformPointCloud(*shared_data_->db_scans[agent_id].at(scan_idx),
                               *transformed_scan, pose_matrix);
      *optimized_map += *transformed_scan;
    }

    fs::path output_map_file =
        output_save_dir_path / ("global_map_" + std::string{agent_id} + ".pcd");
    pcl::io::savePCDFileBinaryCompressed(output_map_file, *optimized_map);
  }
}

}  // namespace open_lmm