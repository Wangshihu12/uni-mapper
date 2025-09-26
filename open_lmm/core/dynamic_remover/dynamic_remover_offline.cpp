#include "dynamic_remover_offline.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

namespace open_lmm {

OfflineParams::OfflineParams() {
  Config config =
      Config(GlobalConfig::get_global_config_path("config_dynamic_remover"));
  dynamic_remover_type =
      config.param<std::string>("dynamic_remover", "dynamic_remover_type", "");
  model = config.param<std::string>("dynamic_remover", "model", "");
}

DynamicRemoverOffline::DynamicRemoverOffline(const OfflineParams& params)
    : params_(params) {
  std::string so_model_name = "libcreate_" + params_.model + ".so";
  offline_model_ = loadModule(so_model_name);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr DynamicRemoverOffline::genRawMap(
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans,
    std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_map =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(
          new pcl::PointCloud<pcl::PointXYZI>());
  for (int i = 0; i < scans.size(); i++) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan =
        pcl::PointCloud<pcl::PointXYZI>::Ptr(
            new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*scans[i], *transformed_scan,
                             optimized_poses[i].second.matrix());
    *raw_map += *transformed_scan;
  }

  return raw_map;
}

/**
 * [功能描述]：离线动态物体移除器的主要处理函数，通过分析多帧点云数据来识别和移除动态物体，生成静态地图。
 * @param scans：扫描数据向量，包含多个时间步的点云数据。
 * @param optimized_poses：优化后的位姿对向量，每个元素包含索引和对应的位姿。
 * @return 返回移除动态物体后的静态地图点云。
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr DynamicRemoverOffline::process(
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans,
    std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses) {
  // 生成原始地图：将所有扫描数据变换到全局坐标系并合并
  // 这个原始地图包含了所有静态和动态物体
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_map =
      genRawMap(scans, optimized_poses);

  // 将原始地图设置为离线模型的输入，用于后续的动态物体检测
  offline_model_->setRawMap(raw_map);

  // 创建进度条，显示动态物体移除的处理进度
  auto T = tq::tqdm(scans);
  T.set_prefix("Dynamic Remover");
  int idx = 0;
  
  // 遍历每个扫描数据，逐个处理动态物体移除
  for (auto scan : T) {
    // 创建变换后的扫描点云对象
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan =
        pcl::PointCloud<pcl::PointXYZI>::Ptr(
            new pcl::PointCloud<pcl::PointXYZI>());
    
    // 将当前扫描从局部坐标系变换到全局坐标系
    // 使用对应的优化位姿矩阵进行变换
    pcl::transformPointCloud(*scan, *transformed_scan,
                             optimized_poses[idx].second.matrix());
    
    // 运行离线动态物体移除模型
    // 输入变换后的扫描和对应的位姿，用于分析动态物体
    offline_model_->run(transformed_scan, optimized_poses[idx].second);
    idx++;
  }
  // 完成进度条显示
  T.finish();

  // 从离线模型中获取处理后的静态地图
  // 该地图已经移除了所有动态物体（如行人、车辆等）
  pcl::PointCloud<pcl::PointXYZI>::Ptr static_map =
      offline_model_->getStaticMap();

  // 返回移除动态物体后的静态地图
  return static_map;
}

std::shared_ptr<IOfflineRemoverPlugin> DynamicRemoverOffline::loadModule(
    const std::string& so_name) {
  return load_module_from_so<IOfflineRemoverPlugin>(
      so_name, "create_dynamic_remover_module");
}

}  // namespace open_lmm