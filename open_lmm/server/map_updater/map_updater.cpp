#include "map_updater.hpp"

namespace open_lmm {
MapUpdater::MapUpdater(const char agent_id,
                       const std::shared_ptr<SharedDatabase>& shared_data)
    : agent_id_(agent_id), server_db_(shared_data) {
  parseConfig();
  data_loader_ = DataLoaderBase::createInstance(config_data_loader_.value());
  dynamic_remover_ =
      DynamicRemoverBase::createInstance(config_dynamic_remover_.value());
  // TODO(gil) : add other modules (e.g. change detector)
}

void MapUpdater::parseConfig() {
  config_data_loader_ =
      Config(GlobalConfig::get_global_config_path("config_data_loader"));
  config_dynamic_remover_ =
      Config(GlobalConfig::get_global_config_path("config_dynamic_remover"));
  // TODO(gil) : add other modules (e.g. change detector)
}

MapUpdater::~MapUpdater() {}

/**
 * [功能描述]：地图更新器的主要处理函数，负责加载原始扫描数据、获取优化后的位姿，并通过动态物体移除生成静态地图。
 * @param data_dir_path：数据目录路径，包含需要处理的扫描数据文件。
 * @return 返回处理后的静态地图点云，移除了动态物体的干扰。
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr MapUpdater::process(
    fs::path data_dir_path) {
  // 输出处理开始的分隔线和标题信息，使用粗体格式显示
  std::cout << "\n"
            << "\033[1m"
            << "============================================================"
            << std::endl;
  // 显示当前处理的数据目录路径
  std::cout << "\033[1m" << "Map Updater : " << data_dir_path << std::endl;
  // 输出分隔线，标记处理开始
  std::cout << "------------------------------------------------------------"
            << std::endl;

  // 使用数据加载器从指定目录加载原始扫描数据
  // 返回包含所有原始点云扫描的向量
  auto raw_scans = data_loader_->loadRawScanData(data_dir_path);
  
  // 从服务器数据库获取当前智能体的优化后位姿对
  // 每个位姿对包含索引和对应的优化位姿
  auto optimized_poses_pair = server_db_->db_optimized_poses[agent_id_];
  
  // 使用动态物体移除器处理原始扫描和优化位姿
  // 移除动态物体（如行人、车辆等），生成静态地图
  auto static_map = dynamic_remover_->process(raw_scans, optimized_poses_pair);

  // TODO(gil) : add other modules (e.g. change detector)
  // 预留接口：未来可以添加其他模块，如变化检测器等

  // 输出处理完成的分隔线，使用粗体格式显示
  std::cout << "\033[1m"
            << "============================================================"
            << std::endl;

  // 返回处理后的静态地图点云
  return static_map;
}

}  // namespace open_lmm