//
// Created by qiayuan on 2022/6/28.
//
// 参考: https://github.com/bernhardpg/quadruped_locomotion
//

#include "legged_wbc/HoQp.h"

#include <qpOASES.hpp>
#include <utility>

namespace legged {

// 构造函数：初始化任务和更高优先级问题，执行初始化、问题构建和求解
HoQp::HoQp(Task task, HoQp::HoQpPtr higherProblem) : task_(std::move(task)), higherProblem_(std::move(higherProblem)) {
  initVars();          // 初始化变量
  formulateProblem();  // 构建优化问题
  solveProblem();      // 求解QP问题
  // 为下一优先级问题准备
  buildZMatrix();      // 构建零空间投影矩阵
  stackSlackSolutions(); // 堆叠松弛变量解
}

// 初始化变量：设置任务变量、前序任务变量和便利矩阵
void HoQp::initVars() {
  // 任务变量
  numSlackVars_ = task_.d_.rows();           // 松弛变量数量（不等式约束行数）
  hasEqConstraints_ = task_.a_.rows() > 0;   // 是否存在等式约束
  hasIneqConstraints_ = numSlackVars_ > 0;   // 是否存在不等式约束

  // 前序任务变量（来自更高优先级问题）
  if (higherProblem_ != nullptr) {
    stackedZPrev_ = higherProblem_->getStackedZMatrix();           // 前序零空间投影矩阵
    stackedTasksPrev_ = higherProblem_->getStackedTasks();         // 前序任务堆叠
    stackedSlackSolutionsPrev_ = higherProblem_->getStackedSlackSolutions(); // 前序松弛变量解
    xPrev_ = higherProblem_->getSolutions();                       // 前序决策变量解
    numPrevSlackVars_ = higherProblem_->getSlackedNumVars();       // 前序松弛变量数量

    numDecisionVars_ = stackedZPrev_.cols();  // 决策变量维度
  } else {
    // 若无更高优先级问题，初始化默认值
    numDecisionVars_ = std::max(task_.a_.cols(), task_.d_.cols()); // 决策变量数取A或D矩阵列数较大者

    stackedTasksPrev_ = Task(numDecisionVars_);                    // 空任务
    stackedZPrev_ = matrix_t::Identity(numDecisionVars_, numDecisionVars_); // 单位矩阵
    stackedSlackSolutionsPrev_ = Eigen::VectorXd::Zero(0);         // 空松弛解
    xPrev_ = Eigen::VectorXd::Zero(numDecisionVars_);              // 零决策变量
    numPrevSlackVars_ = 0;                                         // 无前序松弛变量
  }

  stackedTasks_ = task_ + stackedTasksPrev_;  // 堆叠当前任务与前序任务

  // 初始化便利矩阵
  eyeNvNv_ = matrix_t::Identity(numSlackVars_, numSlackVars_);  // 松弛变量单位矩阵
  zeroNvNx_ = matrix_t::Zero(numSlackVars_, numDecisionVars_);  // 零矩阵
}

// 构建优化问题：组装QP问题的各个组件
void HoQp::formulateProblem() {
  buildHMatrix();  // 构建Hessian矩阵
  buildCVector();  // 构建梯度向量
  buildDMatrix();  // 构建不等式约束矩阵
  buildFVector();  // 构建不等式约束向量
}

// 构建Hessian矩阵：用于QP目标函数 1/2*x^T*H*x + c^T*x
void HoQp::buildHMatrix() {
  matrix_t zTaTaz(numDecisionVars_, numDecisionVars_);

  if (hasEqConstraints_) {
    // 计算 A*Z_prev 并确保 A^T*A 特征值非负（处理数值问题）
    matrix_t aCurrZPrev = task_.a_ * stackedZPrev_;
    zTaTaz = aCurrZPrev.transpose() * aCurrZPrev + 1e-12 * matrix_t::Identity(numDecisionVars_, numDecisionVars_);
    // 这种拆分乘法的方式比直接乘4个矩阵快约两倍
  } else {
    zTaTaz.setZero();  // 无等式约束时为零矩阵
  }

  // 构建块对角Hessian矩阵：左上为等式约束项，右下为松弛变量惩罚项
  h_ = (matrix_t(numDecisionVars_ + numSlackVars_, numDecisionVars_ + numSlackVars_)  // clang-format off
            << zTaTaz, zeroNvNx_.transpose(),
                zeroNvNx_, eyeNvNv_)  // clang-format on
           .finished();
}

// 构建梯度向量c：QP目标函数的线性项
void HoQp::buildCVector() {
  vector_t c = vector_t::Zero(numDecisionVars_ + numSlackVars_);
  vector_t zeroVec = vector_t::Zero(numSlackVars_);  // 松弛变量部分梯度为零

  vector_t temp(numDecisionVars_);
  if (hasEqConstraints_) {
    // 计算 A^T*(A*x_prev - b)
    temp = (task_.a_ * stackedZPrev_).transpose() * (task_.a_ * xPrev_ - task_.b_);
  } else {
    temp.setZero();  // 无等式约束时为零
  }

  // 组合决策变量和松弛变量的梯度
  c_ = (vector_t(numDecisionVars_ + numSlackVars_) << temp, zeroVec).finished();
}

// 构建不等式约束矩阵D：满足 D*x <= f
void HoQp::buildDMatrix() {
  matrix_t stackedZero = matrix_t::Zero(numPrevSlackVars_, numSlackVars_);  // 前序松弛变量与当前松弛变量的零关系

  matrix_t dCurrZ;
  if (hasIneqConstraints_) {
    dCurrZ = task_.d_ * stackedZPrev_;  // 当前不等式约束在前序零空间上的投影
  } else {
    dCurrZ = matrix_t::Zero(0, numDecisionVars_);  // 无不等式约束时为空矩阵
  }

  // 构建三部分不等式约束：
  // 1. 松弛变量非负约束：-s <= 0
  // 2. 前序任务不等式约束：D_prev*Z_prev*x + 0*s <= f_prev - D_prev*x_prev + s_prev
  // 3. 当前任务不等式约束：D*Z_prev*x - s <= f - D*x_prev
  // 注意：与论文顺序相反，但与算法其余部分保持一致
  d_ = (matrix_t(2 * numSlackVars_ + numPrevSlackVars_, numDecisionVars_ + numSlackVars_)  // clang-format off
            << zeroNvNx_, -eyeNvNv_,
                stackedTasksPrev_.d_ * stackedZPrev_, stackedZero,
                dCurrZ, -eyeNvNv_)  // clang-format on
           .finished();
}

// 构建不等式约束向量f
void HoQp::buildFVector() {
  vector_t zeroVec = vector_t::Zero(numSlackVars_);  // 松弛变量非负约束的右端项

  vector_t fMinusDXPrev;
  if (hasIneqConstraints_) {
    fMinusDXPrev = task_.f_ - task_.d_ * xPrev_;  // 当前任务不等式约束调整项
  } else {
    fMinusDXPrev = vector_t::Zero(0);  // 无不等式约束时为空
  }

  // 对应D矩阵的三部分右端项：
  // 1. 零向量（松弛变量非负）
  // 2. 前序任务不等式约束调整项（包含前序松弛解）
  // 3. 当前任务不等式约束调整项
  f_ = (vector_t(2 * numSlackVars_ + numPrevSlackVars_) << zeroVec,
        stackedTasksPrev_.f_ - stackedTasksPrev_.d_ * xPrev_ + stackedSlackSolutionsPrev_, fMinusDXPrev)
           .finished();
}

// 构建零空间投影矩阵Z：用于将解投影到当前任务等式约束的零空间
void HoQp::buildZMatrix() {
  if (hasEqConstraints_) {
    assert((task_.a_.cols() > 0));
    // 计算 A*Z_prev 的零空间基，实现层级投影
    stackedZ_ = stackedZPrev_ * (task_.a_ * stackedZPrev_).fullPivLu().kernel();
  } else {
    // 无等式约束时保持前序零空间
    stackedZ_ = stackedZPrev_;
  }
}

// 求解QP问题：使用qpOASES库
void HoQp::solveProblem() {
  // 创建QP问题：变量数 = 决策变量数 + 松弛变量数，约束数 = f向量长度
  auto qpProblem = qpOASES::QProblem(numDecisionVars_ + numSlackVars_, f_.size());
  qpOASES::Options options;
  options.setToMPC();  // 设置为MPC优化选项
  options.printLevel = qpOASES::PL_LOW;  // 低输出级别
  qpProblem.setOptions(options);
  int nWsr = 20;  // 最大工作集更新次数

  // 初始化并求解QP问题
  qpProblem.init(h_.data(), c_.data(), d_.data(), nullptr, nullptr, nullptr, f_.data(), nWsr);
  vector_t qpSol(numDecisionVars_ + numSlackVars_);

  qpProblem.getPrimalSolution(qpSol.data());  // 获取原始解

  // 分离决策变量和松弛变量解
  decisionVarsSolutions_ = qpSol.head(numDecisionVars_);
  slackVarsSolutions_ = qpSol.tail(numSlackVars_);
}

// 堆叠松弛变量解：将当前松弛解与前序松弛解连接
void HoQp::stackSlackSolutions() {
  if (higherProblem_ != nullptr) {
    // 连接更高优先级问题的松弛解和当前松弛解
    stackedSlackVars_ = Task::concatenateVectors(higherProblem_->getStackedSlackSolutions(), slackVarsSolutions_);
  } else {
    // 最高优先级问题只有当前松弛解
    stackedSlackVars_ = slackVarsSolutions_;
  }
}

}  // namespace legged