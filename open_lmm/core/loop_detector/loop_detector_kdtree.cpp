#pragma once

#include "loop_detector_kdtree.hpp"

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <open_lmm/common/pointcloud_utils.hpp>

namespace open_lmm {

KdtreeParams::KdtreeParams() {
  Config config =
      Config(GlobalConfig::get_global_config_path("config_loop_detector"));
  num_candidates = config.param<int>("database", "num_candidates", 5);
  distance_threshold =
      config.param<double>("database", "distance_threshold", 0.13);
  kdtree_rebuild_threshold =
      config.param<int>("database", "rebuild_threshold", 50);
  model = config.param<std::string>("loop_detector", "model", "");
}

LoopDetectorKdtree::LoopDetectorKdtree(const KdtreeParams& params)
    : params_(params) {
  std::string so_model_name = "libcreate_" + params_.model + ".so";
  model_descriptor_ = loadModule(so_model_name);
  database_ = DatabaseKdtree();
}

LoopPair LoopDetectorKdtree::createLoopPair(
    char agent_id, size_t current_idx,
    const LoopCandidateInfo& candidate_info) {
  LoopPair loop;
  auto [db_id, key, init_rel_pose] = candidate_info;
  loop.to = std::make_pair(db_id, key);
  loop.from = std::make_pair(agent_id, current_idx);
  loop.init_rel_pose = init_rel_pose;
  return loop;
}

/**
 * [功能描述]：检测智能体内循环闭合，通过比较当前扫描与历史扫描的相似性来发现回环。
 * @param scans：当前智能体的扫描数据向量，包含点云数据。
 * @param agent_id：智能体ID，用于标识当前处理的智能体。
 * @return 返回检测到的智能体内循环对向量。
 */
std::vector<LoopPair> LoopDetectorKdtree::detectIntraLoops(const ScanVec& scans,
                                                           char agent_id) {
  // 存储检测到的智能体内循环对的向量
  std::vector<LoopPair> intra_loop_pairs;
  // 获取总扫描数量
  int total_scans = scans.size();
  // 创建进度条显示，用于显示智能体内循环检测的进度
  auto T = tq::trange(0, total_scans);
  T.set_prefix("Intra Loop Detector");
  
  // 遍历所有扫描数据，逐个检测循环闭合
  for (auto idx : T) {
  // for (size_t idx = 0; idx < scans.size(); ++idx) {
    // 获取当前索引对应的扫描数据
    auto scan = scans[idx];
    // 使用描述符模型为当前扫描生成描述符
    auto descriptor = model_descriptor_->makeDescriptor(scan);
    
    // 在数据库中查询与当前描述符相似的候选循环
    std::optional<LoopCandidateInfo> intra_loop_candidates =
        database_->query(descriptor);

    // 如果找到循环候选，则创建循环对
    if (intra_loop_candidates != std::nullopt) {
      // 将找到的循环候选添加到循环对向量中
      intra_loop_pairs.push_back(
          createLoopPair(agent_id, idx, intra_loop_candidates.value()));
    }

    // 将当前扫描的描述符添加到数据库中，用于后续的循环检测
    database_->insert(agent_id, idx, descriptor);
  }
  // 完成进度条显示
  T.finish();
  // 返回检测到的智能体内循环对
  return intra_loop_pairs;
}

/**
 * [功能描述]：检测智能体间循环闭合，通过比较当前智能体的扫描与其他智能体历史扫描的相似性来发现跨智能体回环。
 * @param scans：当前智能体的扫描数据向量，包含点云数据。
 * @param shared_data：共享数据库的智能指针引用，包含所有智能体的描述符数据。
 * @param agent_id：当前智能体ID，用于标识当前处理的智能体。
 * @return 返回检测到的智能体间循环对向量。
 */
std::vector<LoopPair> LoopDetectorKdtree::detectInterLoops(
    const ScanVec& scans, std::shared_ptr<SharedDatabase>& shared_data,
    char agent_id) {
  // 存储检测到的智能体间循环对的向量
  std::vector<LoopPair> inter_loop_pairs;

  // 智能体A不进行智能体间循环检测，因为它是第一个智能体，没有其他智能体的历史数据
  if (agent_id == 'A') {
    return inter_loop_pairs;
  }

  // 获取总扫描数量
  int total_scans = scans.size();
  // 创建进度条显示，用于显示智能体间循环检测的进度
  auto T = tq::trange(0, total_scans);
  T.set_prefix("Inter Loop Detector");
  
  // 遍历当前智能体的所有扫描数据，逐个检测与其他智能体的循环闭合
  for (auto idx : T) {
  // for (size_t idx = 0; idx < scans.size(); ++idx) {
    // 获取当前索引对应的扫描数据
    auto scan = scans[idx];
    // 使用描述符模型为当前扫描生成描述符
    auto descriptor = model_descriptor_->makeDescriptor(scan);

    // 在共享数据库中查询与当前描述符相似的候选循环
    // 共享数据库包含所有已处理智能体的描述符数据
    std::optional<LoopCandidateInfo> inter_loop_candidates =
        shared_data->total_db_descriptors.query(descriptor);

    // 如果找到跨智能体的循环候选，则创建循环对
    if (inter_loop_candidates != std::nullopt) {
      // 将找到的跨智能体循环候选添加到循环对向量中
      inter_loop_pairs.push_back(
          createLoopPair(agent_id, idx, inter_loop_candidates.value()));
    }
  }
  // 完成进度条显示
  T.finish();
  // 返回检测到的智能体间循环对
  return inter_loop_pairs;
}

std::vector<LoopPair> LoopDetectorKdtree::findLoopPairsFromKdTree(
    std::shared_ptr<SharedDatabase>& shared_data,
    const std::vector<Eigen::Isometry3f>& transformed_poses, char agent_id,
    float distance_threshold) {
  std::vector<LoopPair> loop_pairs;

  for (auto& pose_kdtree : shared_data->db_kdtree_poses) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = pose_kdtree.second;
    kdtree.setInputCloud(cloud);

    Eigen::Vector3f prev_pose = transformed_poses[0].translation();

    for (size_t idx = 0; idx < transformed_poses.size(); idx++) {
      auto pose = transformed_poses[idx];

      Eigen::Vector3f curr_pose = pose.translation();
      float distance = (curr_pose - prev_pose).norm();
      // TODO(gil) : hardcoded distance
      if (distance < 10.0F) {
        continue;
      } else {
        prev_pose = curr_pose;
      }

      std::vector<int> pointIdxNKNSearch(1);
      std::vector<float> pointNKNSquaredDistance(1);
      pcl::PointXYZ src_point(pose.translation().x(), pose.translation().y(),
                              pose.translation().z());
      kdtree.nearestKSearch(src_point, 1, pointIdxNKNSearch,
                            pointNKNSquaredDistance);

      if (pointIdxNKNSearch[0] != -1 &&
          std::sqrt(pointNKNSquaredDistance[0]) < distance_threshold) {
        LoopPair inter_loop;
        auto db_id = pose_kdtree.first;
        auto key = pointIdxNKNSearch[0];
        auto init_rel_pose = pose.cast<double>().inverse() *
                             shared_data->db_odom_poses[db_id][key];

        inter_loop.to = std::make_pair(db_id, key);
        inter_loop.from = std::make_pair(agent_id, idx);
        inter_loop.init_rel_pose = init_rel_pose;
        loop_pairs.push_back(inter_loop);
      }
    }
  }

  return loop_pairs;
}

/**
 * [功能描述]：使用KissMatcher进行地图匹配，检测额外的循环闭合并合并地图数据。
 * @param shared_data：共享数据库的智能指针引用，包含合并地图和原始地图数据。
 * @param agent_id：当前智能体ID，用于标识当前处理的智能体。
 * @return 返回通过地图匹配发现的额外循环对向量。
 */
std::vector<LoopPair> LoopDetectorKdtree::detectKissMatcherLoops(
    std::shared_ptr<SharedDatabase>& shared_data, char agent_id) {
  // 存储通过地图匹配发现的额外循环对
  std::vector<LoopPair> additional_loops;
  // 地图匹配的阈值参数：2.0米，用于判断地图是否匹配成功
  constexpr float kMapMatchingThreshold = 2.0f;
  // KdTree搜索的距离阈值：10.0米，用于寻找近距离的位姿对
  constexpr float kDistanceThreshold = 10.0f;

  // 智能体A是第一个智能体，直接将其原始地图设置为合并地图
  if (agent_id == 'A') {
    shared_data->db_merged_map = shared_data->db_original_maps[agent_id];
    return additional_loops;
  }

  // 存储相对位姿变换矩阵，用于将当前智能体的地图对齐到合并地图
  Eigen::Matrix4f relative_map_pose;
  // 使用KissMatcher尝试匹配合并地图和当前智能体的原始地图
  if (!TryKissMatcher(shared_data->db_merged_map,
                      shared_data->db_original_maps[agent_id],
                      kMapMatchingThreshold, false, relative_map_pose)) {
    // 如果地图匹配失败，返回空的循环对向量
    return additional_loops;  // If map matching fails, return empty vector
  }

  // 使用相对位姿变换矩阵将当前智能体的里程计位姿变换到合并地图坐标系
  auto transformed_poses = transformEigenPoses(
      shared_data->db_odom_poses[agent_id], relative_map_pose);

  // 使用KdTree搜索在变换后的位姿中寻找额外的循环对
  additional_loops = findLoopPairsFromKdTree(shared_data, transformed_poses,
                                             agent_id, kDistanceThreshold);

  // 使用相对位姿变换矩阵将当前智能体的原始地图点变换到合并地图坐标系
  auto transformed_map_points = transformEigenPoints(
      shared_data->db_original_maps[agent_id], relative_map_pose);

  // 将变换后的地图点添加到合并地图中，实现地图数据的融合
  shared_data->db_merged_map.insert(shared_data->db_merged_map.end(),
                                    transformed_map_points.begin(),
                                    transformed_map_points.end());

  // 返回通过地图匹配发现的额外循环对
  return additional_loops;
}

/**
 * [功能描述]：处理循环检测的主要函数，检测智能体内的循环闭合和智能体间的循环闭合。
 * @param shared_data：共享数据库的智能指针引用，存储全局描述符和地图数据。
 * @param agent_id：当前处理的智能体ID，用于标识不同的智能体。
 * @param scans：当前智能体的扫描数据向量，包含点云数据。
 * @return 返回包含智能体内循环对向量和智能体间循环对向量的元组。
 */
std::tuple<LoopPairVec, LoopPairVec> LoopDetectorKdtree::process(
    std::shared_ptr<SharedDatabase>& shared_data, const char agent_id,
    ScanVec scans) {
  // 设置数据库的智能体ID，用于标识当前处理的智能体
  database_->setAgentId(agent_id);
  
  // 检测智能体内循环闭合：在同一智能体的轨迹中寻找回环
  std::vector<LoopPair> intra_loop_pairs = detectIntraLoops(scans, agent_id);
  
  // 检测智能体间循环闭合：在不同智能体的轨迹中寻找相似位置
  std::vector<LoopPair> inter_loop_pairs =
      detectInterLoops(scans, shared_data, agent_id);
      
  // 通过地图变换和合并获取额外的循环闭合候选
  // 使用KissMatcher进行地图匹配，找到额外的回环关系
  auto additional_loops = detectKissMatcherLoops(shared_data, agent_id);
  
  // 将地图匹配发现的额外循环闭合添加到智能体间循环对中
  inter_loop_pairs.insert(inter_loop_pairs.end(), additional_loops.begin(),
                          additional_loops.end());

  // 根据智能体ID处理描述符数据库的合并
  if (agent_id != 'A') {
    // 对于非A智能体，将当前智能体的描述符合并到全局数据库中
    shared_data->total_db_descriptors.merge(database_.value());
  } else {
    // 对于A智能体（第一个智能体），直接移动数据库内容到全局数据库
    shared_data->total_db_descriptors = std::move(database_.value());
  }

  // 返回智能体内循环对和智能体间循环对的元组
  return {intra_loop_pairs, inter_loop_pairs};
}

std::shared_ptr<IDescriptorKdtree> LoopDetectorKdtree::loadModule(
    const std::string& so_name) {
  return load_module_from_so<IDescriptorKdtree>(
      so_name, "create_descriptor_kdtree_module");
}

}  // namespace open_lmm