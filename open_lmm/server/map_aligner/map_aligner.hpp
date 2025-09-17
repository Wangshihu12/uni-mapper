#pragma once

#include <filesystem>
#include <memory>
#include <optional>

// #include <Eigen/Core>
#include <Eigen/Geometry>
#include <open_lmm/common/shared_data.hpp>
#include <open_lmm/core/backend_optimizer/backend_optimizer_base.hpp>
#include <open_lmm/core/data_loader/data_loader_base.hpp>
#include <open_lmm/core/dynamic_remover/dynamic_remover_base.hpp>
#include <open_lmm/core/loop_detector/loop_detector_base.hpp>
#include <open_lmm/utils/config.hpp>

// 文件系统命名空间别名，简化std::filesystem的使用
namespace fs = std::filesystem;
// open_lmm项目的主要命名空间
namespace open_lmm {

/**
 * [功能描述]：地图对齐器类，负责单个智能体的SLAM处理流程
 * 该类集成了数据加载、回环检测、后端优化和动态物体移除等核心SLAM组件
 * 实现从原始传感器数据到优化位姿和地图的完整处理流程
 */
class MapAlginer {
 public:
  /**
   * [功能描述]：构造函数，初始化地图对齐器
   * @param agent_id 智能体标识符，用于区分不同的智能体（如'A', 'B', 'C'等）
   * @param shared_data 共享数据库的智能指针，存储所有智能体的共享数据
   * @return 无返回值
   */
  explicit MapAlginer(const char agent_id,
                      const std::shared_ptr<SharedDatabase>& shared_data);
  
  /**
   * [功能描述]：析构函数，清理资源
   * @return 无返回值
   */
  ~MapAlginer();
  
  /**
   * [功能描述]：处理指定数据目录，执行完整的SLAM流程
   * @param data_dir_path 数据目录路径，包含该智能体的传感器数据文件
   * @return 无返回值
   */
  void process(fs::path data_dir_path);
  
  /**
   * [功能描述]：解析配置文件，加载各个组件的配置参数
   * @return 无返回值
   */
  void parseConfig();
  
  /**
   * [功能描述]：运行数据加载器，从指定目录加载传感器数据
   * @param data_dir_path 数据目录路径，包含传感器数据文件
   * @return 返回三元组，包含位姿向量、原始扫描向量和预处理扫描向量
   *         std::tuple<PoseVec, ScanVec, ScanVec> - (位姿, 原始扫描, 预处理扫描)
   */
  std::tuple<PoseVec, ScanVec, ScanVec> runDataLoader(fs::path data_dir_path);

 private:
  // 智能体标识符，用于区分不同的智能体，在系统中唯一标识当前处理的智能体
  const char agent_id_;
  // 服务器数据库的智能指针，存储所有智能体的共享数据（点云、位姿等）
  std::shared_ptr<SharedDatabase> server_db_;
  // 数据加载器的唯一指针，负责从文件中加载传感器数据（点云、位姿等）
  std::unique_ptr<DataLoaderBase> data_loader_;
  // 回环检测器的唯一指针，负责检测机器人是否回到之前访问过的位置
  std::unique_ptr<LoopDetectorBase> loop_detector_;
  // 后端优化器的唯一指针，负责全局位姿优化和图优化
  std::unique_ptr<BackendOptimizerBase> backend_optimizer_;
  // 动态物体移除器的智能指针，负责从点云中移除动态物体
  std::shared_ptr<DynamicRemoverBase> dynamic_remover_;

  // 数据加载器的配置对象，包含数据加载相关的参数设置
  std::optional<Config> config_data_loader_;
  // 回环检测器的配置对象，包含回环检测算法的参数设置
  std::optional<Config> config_loop_detector_;
  // 后端优化器的配置对象，包含图优化算法的参数设置
  std::optional<Config> config_backend_optimizer_;
  // 动态移除器的配置对象，包含动态物体检测和移除的参数设置
  std::optional<Config> config_dynamic_remover_;
};

}  // namespace open_lmm