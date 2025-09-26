#include "backend_optimizer_incremental.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <open_lmm/common/registration.hpp>

#include "BetweenFactorWithAnchoring.h"

namespace open_lmm {

BackendOptimizerIncremental::BackendOptimizerIncremental(Config config) {
  parseConfig(config);
  initNoise();
}

BackendOptimizerIncremental::~BackendOptimizerIncremental() {}

void BackendOptimizerIncremental::parseConfig(Config config) {
  param_.relinearize_threshold =
      config.param<double>("backend_optimizer", "relinearizeThreshold", 0.1);
  param_.relinearize_skip =
      config.param<int>("backend_optimizer", "relinearizeSkip", 1);
}

/**
 * [功能描述]：增量式后端优化器的主要处理函数，构建因子图并进行位姿优化。
 * @param shared_data：共享数据库的智能指针引用，存储因子图和优化后的位姿。
 * @param agent_id：当前智能体ID，用于标识不同的智能体。
 * @param poses：当前智能体的里程计位姿向量。
 * @param intra_loops：智能体内循环闭合对向量。
 * @param inter_loops：智能体间循环闭合对向量。
 * @return 返回优化后的位姿向量，每个元素包含索引和对应的位姿。
 */
std::vector<std::pair<int, Eigen::Isometry3d>>
BackendOptimizerIncremental::process(
    std::shared_ptr<SharedDatabase>& shared_data, const char agent_id,
    PoseVec poses, LoopPairVec intra_loops, LoopPairVec inter_loops) {
  // 创建锚点节点，使用单位矩阵作为初始位姿
  gtsam::Pose3 anchor_node = gtsam::Pose3(Eigen::Matrix4d::Identity());

  //! 1. 初始化因子图（锚点 + 先验因子 + 里程计因子）
  // 创建锚点符号，用于建立全局坐标系参考
  gtsam::Symbol anchor_symbol(agent_id, ANCHOR_IDX);
  if (agent_id == 'A') {
    // 智能体A使用严格的先验噪声模型，作为全局坐标系基准
    shared_data->graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        anchor_symbol, anchor_node, prior_noise_));
  } else {
    // 其他智能体使用宽松的噪声模型，允许更大的不确定性
    shared_data->graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        anchor_symbol, anchor_node, large_noise_));
  }
  // 将锚点位姿插入到因子图的值中
  shared_data->values.insert(anchor_symbol, anchor_node);

  // TODO(gil) : refactor
  //! 2. 添加里程计约束
  for (size_t i = 0; i < poses.size(); i++) {
    // 创建当前节点和前一节点的符号
    gtsam::Symbol node_current(agent_id, i);
    gtsam::Symbol node_prev(agent_id, i - 1);
    if (i == 0) {
      // 第一个位姿节点的处理
      shared_data->values.insert(node_current, gtsam::Pose3(poses[i].matrix()));
      if (agent_id == 'A') {
        // 智能体A的第一个位姿使用严格的先验约束
        shared_data->graph.add(gtsam::PriorFactor<gtsam::Pose3>(
            node_current, gtsam::Pose3(poses[i].matrix()), prior_noise_));
      } else {
        // 其他智能体的第一个位姿使用宽松的先验约束
        shared_data->graph.add(gtsam::PriorFactor<gtsam::Pose3>(
            node_current, gtsam::Pose3(poses[i].matrix()), large_noise_));
      }

    } else {
      // 后续位姿节点的处理
      shared_data->values.insert(node_current, gtsam::Pose3(poses[i].matrix()));
      // 计算相对位姿变换：从前一帧到当前帧的变换
      Eigen::Isometry3d relative_pose = poses[i - 1].inverse() * poses[i];
      // 添加里程计约束因子，连接相邻的两个位姿节点
      shared_data->graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
          node_prev, node_current, gtsam::Pose3(relative_pose.matrix()),
          odometry_noise_));
    }
  }

  //! 3. 添加智能体内循环闭合约束
  auto T1 = tq::tqdm(intra_loops);
  T1.set_prefix("Intra Backend Optimizer");
  for (auto loop : T1) {
    // 跳过距离太近的循环闭合（小于30帧），避免冗余约束
    if (loop.from.second - loop.to.second < 30) continue;
    // 创建循环闭合中起始和终止节点的符号
    gtsam::Symbol node_from(loop.from.first, loop.from.second);
    gtsam::Symbol node_to(loop.to.first, loop.to.second);
    // 通过点云配准精化循环闭合的相对位姿
    std::optional<Eigen::Isometry3d> refined_pose =
        registerPointCloud(*shared_data, loop, 3);
    if (refined_pose) {
      // 如果配准成功，添加精化后的循环闭合约束
      Eigen::Matrix4d refined_pose_mat = refined_pose.value().matrix();
      shared_data->graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
          node_from, node_to, gtsam::Pose3(refined_pose_mat),
          robust_loop_noise_));
    }
  }
  T1.finish();

  //! 4. 添加智能体间循环闭合约束
  if (agent_id != 'A') {
    auto T2 = tq::tqdm(inter_loops);
    T2.set_prefix("Inter Backend Optimizer");
    for (auto loop : T2) {
      // 创建跨智能体循环闭合中起始和终止节点的符号
      gtsam::Symbol node_from(loop.from.first, loop.from.second);
      gtsam::Symbol node_to(loop.to.first, loop.to.second);
      // 通过点云配准精化跨智能体循环闭合的相对位姿
      std::optional<Eigen::Isometry3d> refined_pose =
          registerPointCloud(*shared_data, loop, 3);
      if (refined_pose) {
        Eigen::Matrix4d refined_pose_mat = refined_pose.value().matrix();
        // TODO(gil) : use BetweenFactorWithAnchoring?
        gtsam::Symbol anchor_symbol_to(loop.to.first, ANCHOR_IDX);
        // 添加跨智能体循环闭合约束
        shared_data->graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            node_from, node_to, gtsam::Pose3(refined_pose_mat),
            robust_loop_noise_));
        // 注释掉的锚点因子实现（备用方案）
        // shared_data->graph.add(gtsam::BetweenFactorWithAnchoring<gtsam::Pose3>(
        //   node_to, node_from,
        //   anchor_symbol_to, anchor_symbol,
        //   gtsam::Pose3(refined_pose_mat), robust_loop_noise_)
        // );
      }
    }
    T2.finish();
  }

  //! 5. 配置并执行ISAM2增量优化
  gtsam::ISAM2Params isam_param;
  // 设置重新线性化阈值，控制何时重新线性化因子图
  isam_param.relinearizeThreshold = param_.relinearize_threshold;
  // 设置重新线性化跳过参数，控制重新线性化的频率
  isam_param.relinearizeSkip = param_.relinearize_skip;
  gtsam::ISAM2 isam_(isam_param);
  // 更新因子图和初始值
  isam_.update(shared_data->graph, shared_data->values);
  // 多次更新以确保收敛
  isam_.update();
  isam_.update();
  isam_.update();
  isam_.update();
  isam_.update();
  // 计算最优估计结果
  shared_data->values = isam_.calculateBestEstimate();

  //! 6. 提取优化后的位姿并更新数据库
  for (const auto& key_value : shared_data->values) {
    gtsam::Key key = key_value.key;
    gtsam::Symbol symbol(key);
    gtsam::Symbol anchor_symbol(symbol.chr(), ANCHOR_IDX);
    // TODO(gil) : use anchor factor?
    // 获取对应智能体的锚点位姿
    Eigen::Matrix4d anchor_pose =
        shared_data->values.at<gtsam::Pose3>(anchor_symbol).matrix();
    // 获取当前节点的位姿
    Eigen::Matrix4d pose = shared_data->values.at<gtsam::Pose3>(key).matrix();
    // 计算全局位姿：锚点位姿 × 相对位姿
    Eigen::Isometry3d global_pose(anchor_pose * pose);
    // TODO(gil) : must refactored
    // 跳过锚点节点，只处理普通位姿节点
    if (symbol.index() != ANCHOR_IDX) {
      if (symbol.chr() == agent_id) {
        // 当前智能体的位姿：添加到向量末尾
        shared_data->db_optimized_poses[symbol.chr()].push_back(
            std::make_pair(symbol.index(), global_pose));
        // 同时更新KdTree中用于快速搜索的位姿点
        shared_data->db_kdtree_poses[symbol.chr()].push_back(pcl::PointXYZ(
            global_pose.translation().x(), global_pose.translation().y(),
            global_pose.translation().z()));
      } else {
        // 其他智能体的位姿：更新对应索引位置
        shared_data->db_optimized_poses[symbol.chr()].at(symbol.index()) =
            (std::make_pair(symbol.index(), global_pose));
        // 更新KdTree中对应索引的位姿点
        shared_data->db_kdtree_poses[symbol.chr()].at(symbol.index()) =
            pcl::PointXYZ(global_pose.translation().x(),
                          global_pose.translation().y(),
                          global_pose.translation().z());
      }
    }
  }
  // 获取当前智能体优化后的位姿
  auto optimized_poses = shared_data->db_optimized_poses[agent_id];

  // 返回优化后的位姿向量
  return optimized_poses;
}

// TODO(gil) : add noise parameter config and parsing function
void BackendOptimizerIncremental::initNoise() {
  prior_noise_ = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12)
          .finished());

  odometry_noise_ = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

  gtsam::Vector robust_noise_vec(6);
  robust_noise_vec << 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1;
  robust_loop_noise_ = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(1),
      gtsam::noiseModel::Diagonal::Variances(robust_noise_vec));

  large_noise_ = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << M_PI * M_PI, M_PI * M_PI, M_PI * M_PI, 1e8, 1e8, 1e8)
          .finished());
}

}  // namespace open_lmm