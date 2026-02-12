//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/WbcBase.h"

namespace legged {

// 加权WBC（基于加权任务的分层控制）类，继承自WBC基类
class WeightedWbc : public WbcBase {
 public:
  using WbcBase::WbcBase;  // 继承基类构造函数

  // 更新函数：根据期望状态、输入、测量状态、运动模式和周期计算控制输出
  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                  scalar_t period) override;

  // 从配置文件加载任务参数设置
  void loadTasksSetting(const std::string& taskFile, bool verbose) override;

 protected:
  // 构建约束任务（如接触力约束、关节限位等）
  virtual Task formulateConstraints();
  // 构建加权任务（如摆动腿轨迹跟踪、基座加速度控制等），根据期望状态和输入计算
  virtual Task formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);

 private:
  // 任务权重参数：摆动腿跟踪权重、基座加速度权重、接触力权重
  scalar_t weightSwingLeg_, weightBaseAccel_, weightContactForce_;
};

}  // namespace legged