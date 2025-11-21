#include "erasor.hpp"

#include <pcl/filters/voxel_grid.h>

#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/util/downsampling_tbb.hpp>

// TODO(gil) : remove define
#define NUM_PTS_LARGE_ENOUGH 200000
#define NUM_PTS_LARGE_ENOUGH_FOR_MAP 20000000

ErasorServer::ErasorServer(const common::Config& params) : cfg_(params) {
  initialize();
  erasor_core_.setConfig(cfg_);
}

// ErasorServer::~ErasorServer() {}

void ErasorServer::initialize() {
  map_static_estimate_.reset(new pcl::PointCloud<PointT>());
  map_egocentric_complement_.reset(new pcl::PointCloud<PointT>());
  map_staticAdynamic.reset(new pcl::PointCloud<PointT>());
  map_filtered_.reset(new pcl::PointCloud<PointT>());
  map_arranged_global_.reset(new pcl::PointCloud<PointT>());
  map_arranged_complement_.reset(new pcl::PointCloud<PointT>());
}

// TODO(gil) : remove
void VoxelPointCloud(const pcl::PointCloud<PointT>::Ptr& cloud,
                     pcl::PointCloud<PointT>::Ptr& cloud_voxelized,
                     const double voxel_size) {
  if (voxel_size <= 0.001) {
    *cloud_voxelized = *cloud;
    return;
  }
  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel_grid.filter(*cloud_voxelized);
}

/**
 * [功能描述]：设置并预处理原始地图点云数据，为后续的动态物体移除做准备。
 * @param raw_map：原始地图点云指针，包含所有扫描帧拼接后的完整地图数据（带有 XYZI 信息）。
 * @return 无返回值。
 */
void ErasorServer::setRawMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& raw_map) {
  // 重置并创建新的整理后地图点云对象
  map_arranged_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  // TODO(gil) : use small gicp
  //  VoxelPointCloud(raw_map, map_arranged_, cfg_.map_voxel_size_);
  
  // 使用 small_gicp 库进行体素网格降采样，减少点云数据量以加速处理
  // 通过 TBB 并行加速，根据配置的体素大小对原始地图进行下采样
  map_arranged_ =
      small_gicp::voxelgrid_sampling_tbb(*raw_map, cfg_.map_voxel_size_);
  
  // 记录降采样后的点云数量，用于后续处理和统计
  num_pcs_init_ = map_arranged_->points.size();
  
  // 如果配置为大规模场景模式
  if (cfg_.is_large_scale_) {
    // 预分配足够的内存空间以存储大规模地图点云（2000万个点）
    map_arranged_global_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
    // 将降采样后的地图拷贝到全局地图中
    *map_arranged_global_ = *map_arranged_;
  }
}

/**
 * [功能描述]：ERASOR 算法的核心运行函数，处理单帧扫描以识别和移除动态物体。
 * @param scan：当前帧的扫描点云数据（已变换到世界坐标系）。
 * @param optimized_pose：当前帧优化后的位姿（3D 变换矩阵，包含旋转和平移）。
 * @return 无返回值，处理结果更新到内部成员变量中。
 */
void ErasorServer::run(pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
                       Eigen::Isometry3d& optimized_pose) {
  // 扫描帧计数器递增
  scan_num_++;
  
  // 根据配置的间隔进行处理，不是每一帧都执行动态物体移除
  // 例如：如果 removal_interval_ = 5，则每隔 5 帧处理一次
  if (scan_num_ % cfg_.removal_interval_ != 0) {
    return;
  }

  // 创建降采样后的点云对象
  pcl::PointCloud<PointT>::Ptr filter_pc(new pcl::PointCloud<PointT>());
  // TODO(gil) : use small gicp
  //  VoxelPointCloud(scan, filter_pc, cfg_.query_voxel_size_);
  
  // 对当前扫描进行体素网格降采样，减少查询点云的数据量
  filter_pc = small_gicp::voxelgrid_sampling_tbb(*scan, cfg_.query_voxel_size_);
  
  // 从优化位姿中提取当前扫描的 3D 位置坐标
  float x_curr = optimized_pose.translation().x();
  float y_curr = optimized_pose.translation().y();
  float z_curr = optimized_pose.translation().z();

  // 如果是大规模场景模式，重新分配子地图区域
  // 只处理当前位置附近的局部地图，而不是整个全局地图
  if (cfg_.is_large_scale_) {
    reassign_submap(x_curr, y_curr);
  }

  // 步骤1：提取感兴趣体积区域（Volume of Interest, VoI）
  // 从当前扫描和地图中提取以当前位置为中心的局部区域
  // query_voi_ 存储当前扫描的 VoI，map_voi_ 存储地图的 VoI
  fetch_VoI(
      x_curr, y_curr,
      *filter_pc);  // query_voi_ and map_voi_ are ready in the same world frame

  // 设置 ERASOR 核心算法的中心点（当前扫描位置）
  erasor_core_.setCenter(x_curr, y_curr, z_curr);
  
  // 将提取的 VoI 输入到 ERASOR 核心算法
  erasor_core_.set_inputs(*map_voi_, *query_voi_);
  
  // 步骤2：比较两个 VoI 区域，识别动态物体并恢复地面点
  // 通过比较地图和当前扫描的高度差异来检测动态物体
  erasor_core_.compare_vois_and_revert_ground_w_block();
  
  // 步骤3：获取静态点估计结果
  // map_static_estimate_：估计的静态点云
  // map_staticAdynamic：静态和动态点云的混合
  // map_egocentric_complement_：自我中心补偿点云（处理遮挡区域）
  erasor_core_.get_static_estimate(*map_static_estimate_, *map_staticAdynamic,
                                   *map_egocentric_complement_);

  // 更新整理后的地图：合并静态估计点 + 外围区域点 + 自我中心补偿点
  // 这就是移除动态物体后的最终静态地图
  *map_arranged_ =
      *map_static_estimate_ + *map_outskirts_ + *map_egocentric_complement_;
}

/**
 * [功能描述]：提取感兴趣体积区域（Volume of Interest, VoI）。
 * 从查询点云和地图点云中提取以指定位置为中心、在最大范围内的局部区域点云。
 * VoI 用于局部化动态物体检测，只比较当前位置附近的区域，提高计算效率。
 * @param x_criterion：VoI 中心的 X 坐标（通常是当前扫描位置）。
 * @param y_criterion：VoI 中心的 Y 坐标（通常是当前扫描位置）。
 * @param query_pcd：查询点云（当前扫描帧的点云数据）。
 * @return 无返回值，结果存储在成员变量 query_voi_、map_voi_ 和 map_outskirts_ 中。
 */
void ErasorServer::fetch_VoI(double x_criterion, double y_criterion,
                             pcl::PointCloud<PointT>& query_pcd) {
  // 重置并初始化三个点云对象
  query_voi_.reset(new pcl::PointCloud<PointT>());      // 查询点云的 VoI
  map_voi_.reset(new pcl::PointCloud<PointT>());        // 地图点云的 VoI
  map_outskirts_.reset(new pcl::PointCloud<PointT>());  // 地图的外围区域（VoI 之外）

  // 使用朴素模式（naive mode）进行 VoI 提取
  if (cfg_.mode == "naive") {
    // 预先计算最大范围的平方，避免在循环中重复计算和开平方操作
    double max_dist_square = pow(cfg_.max_range_, 2);
    
    // 步骤1：从查询点云中提取 VoI
    // 遍历当前扫描的所有点
    for (auto const& pt : query_pcd.points) {
      // 计算点到 VoI 中心的欧氏距离平方（只考虑 XY 平面，忽略 Z 轴）
      double dist_square =
          pow(pt.x - x_criterion, 2) + pow(pt.y - y_criterion, 2);
      
      // 如果点在最大范围内，将其加入查询 VoI
      if (dist_square < max_dist_square) {
        query_voi_->points.emplace_back(pt);
      }
    }

    // 步骤2：从地图点云中提取 VoI 和外围区域
    // 遍历整理后的地图的所有点
    for (auto& pt : map_arranged_->points) {
      // 计算点到 VoI 中心的欧氏距离平方
      double dist_square =
          pow(pt.x - x_criterion, 2) + pow(pt.y - y_criterion, 2);
      
      // 如果点在最大范围内，将其加入地图 VoI
      if (dist_square < max_dist_square) {
        map_voi_->points.emplace_back(pt);
      } else {
        // 否则将点加入外围区域
        // 如果配置了替换强度值，将外围点的强度设为 0（便于可视化区分）
        if (cfg_.replace_intensity) pt.intensity = 0;
        map_outskirts_->points.emplace_back(pt);
      }
    }
  }
}

/**
 * [功能描述]：重新分配子地图区域，用于大规模场景的动态地图管理。
 * 根据当前位置动态调整处理的子地图范围，避免一次性处理整个大规模地图。
 * @param pose_x：当前扫描位置的 X 坐标。
 * @param pose_y：当前扫描位置的 Y 坐标。
 * @return 无返回值，更新内部的子地图成员变量。
 */
void ErasorServer::reassign_submap(double pose_x, double pose_y) {
  // 首次初始化子地图
  if (is_submap_not_initialized_) {
    // 从全局地图中提取以当前位置为中心的子地图区域
    // map_arranged_：当前子地图（活动区域）
    // map_arranged_complement_：子地图补集（外围区域）
    set_submap(*map_arranged_global_, *map_arranged_, *map_arranged_complement_,
               pose_x, pose_y, submap_size_);
    
    // 记录子地图中心位置
    submap_center_x_ = pose_x;
    submap_center_y_ = pose_y;
    
    // 标记子地图已初始化
    is_submap_not_initialized_ = false;

  } else {
    // 子地图已初始化，检查当前位置是否偏离子地图中心太远
    
    // 计算当前位置与子地图中心的距离差
    double diff_x = abs(submap_center_x_ - pose_x);
    double diff_y = abs(submap_center_y_ - pose_y);
    
    // 计算子地图的半径大小（使用 static 避免重复计算）
    static double half_size = submap_size_ / 2.0;
    
    // 如果当前位置在 X 或 Y 方向上超出了子地图半径范围
    // 需要重新分配子地图，将子地图中心移动到新位置
    if ((diff_x > half_size) || (diff_y > half_size)) {
      // 重新分配子地图
      
      // 重置全局地图，准备合并当前处理结果
      map_arranged_global_.reset(new pcl::PointCloud<pcl::PointXYZI>());
      map_arranged_global_->reserve(num_pcs_init_);
      
      // 将当前子地图和补集合并回全局地图
      // 这样可以保留之前处理的动态物体移除结果
      *map_arranged_global_ = *map_arranged_ + *map_arranged_complement_;

      // 基于新的位置重新划分子地图区域
      set_submap(*map_arranged_global_, *map_arranged_,
                 *map_arranged_complement_, pose_x, pose_y, submap_size_);
      
      // 更新子地图中心位置到当前位置
      submap_center_x_ = pose_x;
      submap_center_y_ = pose_y;
    }
  }
}
/**
 * [功能描述]：从全局地图中提取以指定位置为中心的子地图区域。
 * 根据给定的中心坐标和子地图大小，将全局地图点云划分为子地图和补集两部分。
 * @param map_global：输入的全局地图点云（常量引用）。
 * @param submap：输出的子地图点云，包含中心区域内的所有点。
 * @param submap_complement：输出的子地图补集点云，包含中心区域外的所有点。
 * @param x：子地图中心的 X 坐标。
 * @param y：子地图中心的 Y 坐标。
 * @param submap_size：子地图的尺寸（从中心到边界的距离，形成正方形区域）。
 * @return 无返回值，结果通过引用参数返回。
 */
void ErasorServer::set_submap(
    const pcl::PointCloud<pcl::PointXYZI>& map_global,
    pcl::PointCloud<pcl::PointXYZI>& submap,
    pcl::PointCloud<pcl::PointXYZI>& submap_complement, double x, double y,
    double submap_size) {
  // 清空并预分配子地图内存空间
  submap.clear();
  submap.reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);  // 预分配 2000 万个点的空间
  
  // 清空并预分配补集地图内存空间
  submap_complement.clear();
  submap_complement.reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);

  // 遍历全局地图中的每个点，根据位置划分到子地图或补集中
  for (const auto pt : map_global.points) {
    // 计算当前点与子地图中心的距离（X 和 Y 方向）
    double diff_x = fabs(x - pt.x);
    double diff_y = fabs(y - pt.y);
    
    // 判断点是否在子地图的正方形区域内
    // 如果点在 X 和 Y 方向上都距离中心小于 submap_size，则属于子地图
    if ((diff_x < submap_size) && (diff_y < submap_size)) {
      submap.points.emplace_back(pt);  // 将点添加到子地图
    } else {
      // 否则将点添加到补集中（外围区域）
      submap_complement.points.emplace_back(pt);
    }
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr ErasorServer::getStaticMap() {
  pcl::PointCloud<PointT>::Ptr ptr_src(new pcl::PointCloud<PointT>);
  ptr_src->reserve(num_pcs_init_);

  if (cfg_.is_large_scale_) {
    *ptr_src = *map_arranged_ + *map_arranged_complement_;
  } else {
    *ptr_src = *map_arranged_;
  }
  // save map_static_estimate_
  // if (ptr_src->size() == 0) {
  //   return;
  // }
  if (map_staticAdynamic->size() > 0 && cfg_.replace_intensity) {
    *ptr_src += *map_staticAdynamic;
  }
  // pcl::io::savePCDFileBinary("/home/gil/erasor_output.pcd", *ptr_src);
  return ptr_src;
}