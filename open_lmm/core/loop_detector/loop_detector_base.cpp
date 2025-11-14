#include "loop_detector_base.hpp"
// #include "loop_detector_sc.hpp"

#include "loop_detector_kdtree.hpp"

namespace open_lmm {

std::unique_ptr<LoopDetectorBase> LoopDetectorBase::createInstance(
    Config config) {
  std::string loop_detector_type =
      config.param<std::string>("loop_detector", "loop_detector_type", "");
  if (loop_detector_type == "kdtree") {
    return std::make_unique<LoopDetectorKdtree>(KdtreeParams());
  } else if (loop_detector_type == "hashmap") {
    throw std::runtime_error(
        "[loop_detector_base.cpp] Hashmap loop detector is not implemented ");
  } else {
    throw std::invalid_argument(
        "[loop_detector_base.cpp] Invalid loop detector type: " +
        loop_detector_type);
  }
};

/**
 * [功能描述]：尝试使用KISS-Matcher算法进行点云配准，计算源点云到目标点云的变换矩阵。
 * @param tgt_map_vec：目标地图点云向量，包含目标点云的所有3D点坐标。
 * @param src_map_vec：源地图点云向量，包含待配准点云的所有3D点坐标。
 * @param leaf_size：体素叶子大小，用于点云下采样的体素网格尺寸。
 * @param use_quatro：是否使用Quatro算法进行旋转估计的标志。
 * @param output：输出参数，存储计算得到的4x4变换矩阵（旋转+平移）。
 * @return 返回配准是否成功：true表示成功，false表示失败。
 */
bool LoopDetectorBase::TryKissMatcher(
    const std::vector<Eigen::Vector3f> tgt_map_vec,
    const std::vector<Eigen::Vector3f> src_map_vec, const float leaf_size,
    const bool use_quatro, Eigen::Matrix4f& output) {
  // 创建KISS-Matcher配置对象，传入叶子大小参数
  kiss_matcher::KISSMatcherConfig config =
      kiss_matcher::KISSMatcherConfig(leaf_size);
  // 设置是否使用Quatro算法进行旋转估计
  config.use_quatro_ = use_quatro;
  // 使用配置创建KISS-Matcher匹配器实例
  kiss_matcher::KISSMatcher matcher(config);

  // 执行点云配准估计，计算从源点云到目标点云的变换
  const auto solution = matcher.estimate(src_map_vec, tgt_map_vec);

  // 初始化一个4x4单位矩阵用于存储变换结果
  Eigen::Matrix4f solution_eigen = Eigen::Matrix4f::Identity();
  // 将旋转矩阵（3x3）填充到变换矩阵的左上角
  solution_eigen.block<3, 3>(0, 0) = solution.rotation.cast<float>();
  // 将平移向量（3x1）填充到变换矩阵的右上角
  solution_eigen.topRightCorner(3, 1) = solution.translation.cast<float>();

  // 获取旋转估计阶段的内点数量
  size_t num_rot_inliers = matcher.getNumRotationInliers();
  // 获取最终配准结果的内点数量
  size_t num_final_inliers = matcher.getNumFinalInliers();

  // 设置内点数量阈值，用于判断配准是否成功
  size_t thres_num_inliers = 5;
  // 判断最终内点数量是否小于阈值
  if (num_final_inliers < thres_num_inliers) {
    // 内点数量不足，配准可能失败，输出黄色警告信息
    std::cout << "\033[1;33m=> KISS-MATCHER might have failed :(\033[0m\n";
    // 输出单位矩阵表示无有效变换
    output = Eigen::Matrix4f::Identity();
    return false;  // 返回失败
  } else {
    // 内点数量充足，配准可能成功，输出绿色成功信息
    std::cout << "\033[1;32m=> KISS-MATCHER likely succeeded XD\033[0m\n";
    // 输出计算得到的变换矩阵
    output = solution_eigen;
    return true;  // 返回成功
  }
}



}  // namespace open_lmm