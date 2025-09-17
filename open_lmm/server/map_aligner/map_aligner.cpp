#include "map_aligner.hpp"

namespace open_lmm {
MapAlginer::MapAlginer(const char agent_id,
                       const std::shared_ptr<SharedDatabase>& shared_data)
    : agent_id_(agent_id), server_db_(shared_data) {
  parseConfig();
  data_loader_ = DataLoaderBase::createInstance(config_data_loader_.value());
  loop_detector_ =
      LoopDetectorBase::createInstance(config_loop_detector_.value());
  backend_optimizer_ =
      BackendOptimizerBase::createInstance(config_backend_optimizer_.value());
}

void MapAlginer::parseConfig() {
  config_data_loader_ =
      Config(GlobalConfig::get_global_config_path("config_data_loader"));
  config_loop_detector_ =
      Config(GlobalConfig::get_global_config_path("config_loop_detector"));
  config_backend_optimizer_ =
      Config(GlobalConfig::get_global_config_path("config_backend_optimizer"));
}

MapAlginer::~MapAlginer() {}

// std::tuple<PoseVec, ScanVec, ScanVec> MapAlginer::runDataLoader(
//     fs::path data_dir_path) {
//   return data_loader_->process(server_db_, agent_id_, data_dir_path);
//   //   PoseVec raw_poses = data_loader_->loadPoseData(data_dir_path);
//   //   ScanVec raw_scans = data_loader_->loadRawScanData(data_dir_path);
//   //   ScanVec filtered_scans =
//   //   data_loader_->loadFilteredScanData(data_dir_path); return {raw_poses,
//   //   raw_scans, filtered_scans};
//   // server_db_->db_odom_poses[agent_id_] = poses;
//   // server_db_->db_scans[agent_id_] = filtered_scans;
// }

/**
 * [功能描述]：执行地图对齐器的核心处理流程，实现完整的SLAM算法
 * 该函数协调数据加载、回环检测和后端优化三个主要步骤，生成优化后的位姿
 * @param data_dir_path 数据目录路径，包含该智能体的传感器数据文件
 * @return 无返回值
 */
void MapAlginer::process(fs::path data_dir_path) {
  // 注释掉的旧代码：使用runDataLoader函数加载数据
  // auto [poses, raw_scans, filtered_scans] = runDataLoader(data_dir_path);
  //////////////////////////////////////////////////////////////////////

  // TODO(gil): Refactor needed — remove dependency on centralized `server_db_`
  //  aim for a more functional design
  // 开发者待办事项：重构代码以移除对集中式server_db_的依赖，采用更函数式的设计
  
  // 输出处理开始的分隔线和标题，使用ANSI转义序列设置粗体格式
  std::cout << "\n"
            << "\033[1m"
            << "============================================================"
            << std::endl;
  std::cout << "\033[1m" << "Map Aligner : " << data_dir_path << std::endl;
  std::cout << "------------------------------------------------------------"
            << std::endl;

  // 第一步：数据加载处理
  // 使用结构化绑定从数据加载器获取位姿、原始扫描和过滤扫描数据
  // 数据加载器负责从文件中读取传感器数据并进行预处理
  auto [poses, raw_scans, filtered_scans] =
      data_loader_->process(server_db_, agent_id_, data_dir_path);

  // 第二步：回环检测处理
  // 使用结构化绑定从回环检测器获取内部回环和跨智能体回环
  // 回环检测器分析扫描数据，识别机器人是否回到之前访问过的位置
  auto [intra_loops, inter_loops] =
      loop_detector_->process(server_db_, agent_id_, filtered_scans);

  // 第三步：后端优化处理
  // 使用后端优化器对位姿进行全局优化，利用回环约束提高位姿精度
  // 输入参数：共享数据库、智能体ID、初始位姿、内部回环、跨智能体回环
  auto optimized_poses = backend_optimizer_->process(
      server_db_, agent_id_, poses, intra_loops, inter_loops);

  // 输出处理完成的分隔线，使用ANSI转义序列设置粗体格式
  std::cout << "\033[1m"
            << "============================================================"
            << std::endl;
}

}  // namespace open_lmm