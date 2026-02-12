//
// Created by qiayuan on 22-12-23.
//
#pragma once

#include "legged_wbc/WbcBase.h"

namespace legged {

// 层次化全身控制（Hierarchical Whole Body Control）类
// 继承自WbcBase基类，实现层次化优化策略的全身控制算法
class HierarchicalWbc : public WbcBase {
 public:
  using WbcBase::WbcBase;  // 继承基类构造函数

  // 核心更新函数：根据期望状态、输入和实际测量状态计算控制输出
  // 参数说明：
  // stateDesired: 期望状态向量（如关节位置、速度等）
  // inputDesired: 期望输入向量（如期望力矩或加速度）
  // rbdStateMeasured: 基于刚体动力学的实际测量状态
  // mode: 当前运动模式标识（如站立、行走等）
  // period: 控制周期时间（单位：秒）
  // 返回值：计算得到的控制输出向量（如关节力矩）
  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                  scalar_t period) override;
};

}  // namespace legged