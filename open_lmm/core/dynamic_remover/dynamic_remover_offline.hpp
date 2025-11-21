#pragma once
#include <tqdmcpp/tqdmcpp.hpp>


#include <open_lmm/core/dynamic_remover/remover_factory/offline/interface_offline_plugin.hpp>

#include "dynamic_remover_base.hpp"

namespace open_lmm {

struct OfflineParams {
 public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit OfflineParams();
  ~OfflineParams() = default;

 public:
  std::string dynamic_remover_type;
  std::string model;
};

/**
 * [功能描述]：离线动态物体移除器类，用于在离线场景中处理点云数据，移除动态物体。
 * 继承自 DynamicRemoverBase 基类，实现离线批处理模式的动态物体过滤功能。
 */
class DynamicRemoverOffline : public DynamicRemoverBase {
 public:
  /**
   * [功能描述]：构造函数，使用离线参数初始化动态物体移除器。
   * @param params：离线模式参数配置，包含模型路径、处理参数等。
   */
  DynamicRemoverOffline(const OfflineParams& params);
  
  /**
   * [功能描述]：析构函数，使用默认实现。
   */
  ~DynamicRemoverOffline() override = default;
  
  /**
   * [功能描述]：处理点云数据，移除动态物体后返回静态地图点云。
   * @param scans：输入的点云扫描序列，每个元素是一帧带强度信息的点云数据。
   * @param optimized_poses：优化后的位姿序列，每个元素包含帧 ID 和对应的 3D 变换矩阵（包含旋转和平移）。
   * @return 返回处理后的静态点云地图，已移除动态物体。
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr process(
      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans,
      std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses) override;
  
  /**
   * [功能描述]：生成原始地图点云，将所有扫描帧按优化位姿拼接成完整地图。
   * @param scans：输入的点云扫描序列，每个元素是一帧带强度信息的点云数据。
   * @param optimized_poses：优化后的位姿序列，每个元素包含帧 ID 和对应的 3D 变换矩阵。
   * @return 返回拼接后的原始地图点云（未进行动态物体移除）。
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr genRawMap(
      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans,
      std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses);

 private:
  OfflineParams params_;  // 离线模式参数配置
  std::shared_ptr<IOfflineRemoverPlugin> offline_model_;  // 离线动态物体移除插件实例

  /**
   * [功能描述]：从动态链接库加载动态物体移除模块。
   * @param so_name：动态链接库文件名（.so 文件路径）。
   * @return 返回加载成功的动态物体移除插件实例。
   */
  static std::shared_ptr<IOfflineRemoverPlugin> loadModule(
      const std::string& so_name);
};

}  // namespace open_lmm
