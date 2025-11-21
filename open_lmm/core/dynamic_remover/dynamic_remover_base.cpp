#include "dynamic_remover_base.hpp"

#include "dynamic_remover_online.hpp"
#include "dynamic_remover_offline.hpp"

namespace open_lmm {

/**
 * [功能描述]：工厂方法，根据配置参数创建动态物体移除器的实例。
 * @param config：配置对象，包含动态物体移除器的类型和模型参数。
 * @return 返回指向 DynamicRemoverBase 的智能指针，根据配置创建 offline 或 online 类型的动态物体移除器实例。
 * @throws std::invalid_argument 当配置的动态物体移除器类型无效时抛出异常。
 */
std::shared_ptr<DynamicRemoverBase> DynamicRemoverBase::createInstance(
    Config config) {
  // 从配置文件中读取动态物体移除器的类型（offline 或 online）
  std::string dynamic_remover_type =
      config.param<std::string>("dynamic_remover", "dynamic_remover_type", "");
  // 从配置文件中读取动态物体移除器使用的模型名称
  std::string dynamic_remover_model =
      config.param<std::string>("dynamic_remover", "model", "");
  
  // 根据类型创建对应的动态物体移除器实例
  if (dynamic_remover_type == "offline") {
    // 创建离线模式的动态物体移除器（适用于离线处理场景）
    auto dynamic_remover =
        std::make_shared<DynamicRemoverOffline>(OfflineParams());
    return dynamic_remover;
  } else if (dynamic_remover_type == "online") {
    // 创建在线模式的动态物体移除器（适用于实时处理场景）
    auto dynamic_remover =
        std::make_shared<DynamicRemoverOnline>(OnlineParams());
    return dynamic_remover;
  } else {
    // 类型无效，抛出异常并提示错误信息
    throw std::invalid_argument(
        "[dynamic_remover_base.cpp] Invalid dynamic remover type: " +
        dynamic_remover_type);
  }
};

}  // namespace open_lmm