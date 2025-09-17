#pragma once

#include <filesystem>
#include <open_lmm/common/shared_data.hpp>
#include <open_lmm/server/map_aligner/map_aligner.hpp>
#include <open_lmm/server/map_updater/map_updater.hpp>
#include <vector>

// 文件系统命名空间别名，简化std::filesystem的使用
namespace fs = std::filesystem;
// open_lmm项目的主要命名空间
namespace open_lmm {

/**
 * [功能描述]：地图服务器类，负责管理多智能体SLAM系统的地图构建和优化
 * 该类是整个SLAM系统的核心组件，协调多个智能体的数据融合和地图更新
 */
class MapServer {
 public:
  /**
   * [功能描述]：构造函数，初始化地图服务器
   * @return 无返回值
   */
  MapServer();
  
  /**
   * [功能描述]：析构函数，清理资源
   * @return 无返回值
   */
  ~MapServer();
  
  /**
   * [功能描述]：解析配置文件，加载系统参数和设置
   * @return 无返回值
   */
  void parseConfig();
  
  /**
   * [功能描述]：主处理函数，执行SLAM算法的核心流程
   * @return 无返回值
   */
  void process();
  
  /**
   * [功能描述]：为指定智能体分配内存空间
   * @param agent_id 智能体标识符，用于区分不同的智能体
   * @return 无返回值
   */
  void allocateAgentMemory(const char agent_id);
  
  /**
   * [功能描述]：保存优化后的位姿数据到指定目录
   * @param save_dir 保存目录路径，位姿文件将存储在此目录下
   * @return 无返回值
   */
  void saveOptimizedPoses(const std::string& save_dir);
  
  /**
   * [功能描述]：保存优化后的地图数据到指定目录
   * @param save_dir 保存目录路径，地图文件将存储在此目录下
   * @return 无返回值
   */
  void saveOptimizedMap(const std::string& save_dir);

 private:
  // 共享数据库智能指针，存储所有智能体的共享数据（点云、位姿等）
  std::shared_ptr<SharedDatabase> shared_data_ =
      std::make_shared<SharedDatabase>();
  // 智能体数量，表示系统中参与SLAM的智能体总数
  int agent_num_;
  // 数据目录列表，存储每个智能体的数据文件路径
  std::vector<fs::path> data_dir_list_;
  // 输出保存目录，用于存储处理结果和优化后的数据
  std::string output_save_dir_;
  // 地图服务器配置对象，包含系统运行参数和设置
  std::optional<Config> config_map_server_;
  // 地图更新器使能标志，控制是否启用地图更新功能
  bool enable_map_updater_;
};

}  // namespace open_lmm