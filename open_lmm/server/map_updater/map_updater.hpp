#pragma once

#include <filesystem>
#include <memory>
#include <optional>

// #include <Eigen/Core>
#include <Eigen/Geometry>
#include <open_lmm/common/shared_data.hpp>
#include <open_lmm/core/data_loader/data_loader_base.hpp>
#include <open_lmm/core/dynamic_remover/dynamic_remover_base.hpp>
#include <open_lmm/utils/config.hpp>

// 文件系统命名空间别名，简化std::filesystem的使用
namespace fs = std::filesystem;
// open_lmm项目的主要命名空间
namespace open_lmm {

/**
 * [功能描述]：地图更新器类，负责处理单个智能体的数据并生成静态地图
 * 该类集成了数据加载器和动态物体移除器，用于从原始点云数据中生成去除动态物体的静态地图
 */
class MapUpdater {
 public:
  /**
   * [功能描述]：构造函数，初始化地图更新器
   * @param agent_id 智能体标识符，用于区分不同的智能体（如'A', 'B', 'C'等）
   * @param shared_data 共享数据库的智能指针，存储所有智能体的共享数据
   * @return 无返回值
   */
  explicit MapUpdater(const char agent_id,
                      const std::shared_ptr<SharedDatabase>& shared_data);
  
  /**
   * [功能描述]：析构函数，清理资源
   * @return 无返回值
   */
  ~MapUpdater();
  
  /**
   * [功能描述]：处理指定数据目录，生成静态地图
   * @param data_dir_path 数据目录路径，包含该智能体的点云数据文件
   * @return 返回处理后的静态点云地图的智能指针，类型为pcl::PointCloud<pcl::PointXYZI>::Ptr
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr process(fs::path data_dir_path);
  
  /**
   * [功能描述]：解析配置文件，加载数据加载器和动态移除器的配置参数
   * @return 无返回值
   */
  void parseConfig();

 private:
  // 智能体标识符，用于区分不同的智能体，在系统中唯一标识当前处理的智能体
  const char agent_id_;
  // 服务器数据库的智能指针，存储所有智能体的共享数据（点云、位姿等）
  std::shared_ptr<SharedDatabase> server_db_;
  // 数据加载器的唯一指针，负责从文件中加载点云数据
  std::unique_ptr<DataLoaderBase> data_loader_;
  // 动态物体移除器的智能指针，负责从点云中移除动态物体，生成静态地图
  std::shared_ptr<DynamicRemoverBase> dynamic_remover_;

  // 数据加载器的配置对象，包含数据加载相关的参数设置
  std::optional<Config> config_data_loader_;
  // 动态移除器的配置对象，包含动态物体检测和移除的参数设置
  std::optional<Config> config_dynamic_remover_;
};

}  // namespace open_lmm