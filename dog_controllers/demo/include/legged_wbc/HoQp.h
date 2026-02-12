//
// Created by qiayuan on 2022/6/28.
//
//
// Ref: https://github.com/bernhardpg/quadruped_locomotion
//

#pragma once

#include "legged_wbc/Task.h"

#include <memory>

namespace legged {
// 分层优化二次规划（Hierarchical Optimization Quadratic Program）
class HoQp {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using HoQpPtr = std::shared_ptr<HoQp>;

  // 构造函数：接受一个任务，高层问题为空
  explicit HoQp(const Task& task) : HoQp(task, nullptr){};

  // 构造函数：接受任务和高层问题指针
  HoQp(Task task, HoQpPtr higherProblem);

  // 获取堆叠的零空间矩阵Z
  matrix_t getStackedZMatrix() const { return stackedZ_; }

  // 获取堆叠的任务
  Task getStackedTasks() const { return stackedTasks_; }

  // 获取堆叠的松弛变量解
  vector_t getStackedSlackSolutions() const { return stackedSlackVars_; }

  // 获取最终解：x = xPrev + stackedZPrev * decisionVarsSolutions
  vector_t getSolutions() const {
    vector_t x = xPrev_ + stackedZPrev_ * decisionVarsSolutions_;
    return x;
  }

  // 获取带松弛变量的变量数量
  size_t getSlackedNumVars() const { return stackedTasks_.d_.rows(); }

 private:
  // 初始化变量
  void initVars();
  // 构建优化问题
  void formulateProblem();
  // 求解优化问题
  void solveProblem();

  // 构建Hessian矩阵H（二次项系数）
  void buildHMatrix();
  // 构建梯度向量c（一次项系数）
  void buildCVector();
  // 构建不等式约束矩阵D
  void buildDMatrix();
  // 构建不等式约束向量f
  void buildFVector();

  // 构建零空间矩阵Z
  void buildZMatrix();
  // 堆叠松弛变量解
  void stackSlackSolutions();

  // 当前任务、前一层堆叠任务、当前堆叠任务
  Task task_, stackedTasksPrev_, stackedTasks_;
  // 高层问题指针
  HoQpPtr higherProblem_;

  // 标志：是否有等式约束、不等式约束
  bool hasEqConstraints_{}, hasIneqConstraints_{};
  // 松弛变量数量、决策变量数量
  size_t numSlackVars_{}, numDecisionVars_{};
  // 前一层堆叠的零空间矩阵、当前堆叠的零空间矩阵
  matrix_t stackedZPrev_, stackedZ_;
  // 前一层堆叠的松弛变量解、前一层的最优解
  vector_t stackedSlackSolutionsPrev_, xPrev_;
  // 前一层松弛变量数量
  size_t numPrevSlackVars_{};

  // 优化问题矩阵：Hessian矩阵H、不等式约束矩阵D
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> h_, d_;
  // 优化问题向量：梯度向量c、不等式约束向量f
  vector_t c_, f_;
  // 堆叠的松弛变量、松弛变量解、决策变量解
  vector_t stackedSlackVars_, slackVarsSolutions_, decisionVarsSolutions_;

  // 方便计算的矩阵：单位矩阵、零矩阵
  matrix_t eyeNvNv_;
  matrix_t zeroNvNx_;
};

}  // namespace legged