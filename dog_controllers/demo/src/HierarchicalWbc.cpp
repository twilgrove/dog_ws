//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/HierarchicalWbc.h"

#include "legged_wbc/HoQp.h"

namespace legged {
vector_t HierarchicalWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                                 scalar_t period) {
  // 调用基类更新函数，初始化WBC所需数据
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

  // 第一优先级任务：浮基座动力学方程、扭矩限制、摩擦锥约束、无接触运动约束
  Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
  // 第二优先级任务：基座加速度跟踪、摆动腿轨迹跟踪
  Task task1 = formulateBaseAccelTask(stateDesired, inputDesired, period) + formulateSwingLegTask();
  // 第三优先级任务：接触力跟踪
  Task task2 = formulateContactForceTask(inputDesired);
  // 构建分层二次规划问题，task0优先级最高，task2最低
  HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));

  // 返回分层QP求解得到的最优解（如关节扭矩、接触力等）
  return hoQp.getSolutions();
}

}  // namespace legged