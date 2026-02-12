//
// Created by qiayuan on 2022/6/28.
//
//
// Ref: https://github.com/bernhardpg/quadruped_locomotion
//

#pragma once

#include <ocs2_core/Types.h>

#include <utility>

namespace legged
{
  using namespace ocs2;

  class Task
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Task() = default;
    ~Task() = default;
    // 构造函数，初始化任务矩阵和向量
    // a: 等式约束矩阵，b: 等式约束向量
    // d: 不等式约束矩阵，f: 不等式约束向量
    Task(matrix_t a, vector_t b, matrix_t d, vector_t f) : a_(std::move(a)), d_(std::move(d)), b_(std::move(b)), f_(std::move(f)) {}

    // 显式构造函数，创建零矩阵/向量的空任务
    explicit Task(size_t numDecisionVars)
        : Task(matrix_t::Zero(0, numDecisionVars), vector_t::Zero(0), matrix_t::Zero(0, numDecisionVars), vector_t::Zero(0)) {}

    // 重载+运算符，合并两个任务（垂直堆叠约束）
    Task operator+(const Task &rhs) const
    {
      return {concatenateMatrices(a_, rhs.a_), concatenateVectors(b_, rhs.b_), concatenateMatrices(d_, rhs.d_),
              concatenateVectors(f_, rhs.f_)};
    }

    // 重载*运算符，缩放任务（乘以标量）
    Task operator*(scalar_t rhs) const
    { // clang-format off
    return {a_.cols() > 0 ? rhs * a_ : a_,      // 如果矩阵非空则缩放，否则保持原样
            b_.cols() > 0 ? rhs * b_ : b_,
            d_.cols() > 0 ? rhs * d_ : d_,
            f_.cols() > 0 ? rhs * f_ : f_}; // clang-format on
    }

    // 任务数据成员：等式约束A*x=b，不等式约束D*x<=f
    matrix_t a_, d_;
    vector_t b_, f_;

    // 静态方法：垂直堆叠两个矩阵（用于合并约束）
    static matrix_t concatenateMatrices(matrix_t m1, matrix_t m2)
    {
      if (m1.cols() <= 0)
      { // 如果m1为空，返回m2
        return m2;
      }
      else if (m2.cols() <= 0)
      { // 如果m2为空，返回m1
        return m1;
      }
      assert(m1.cols() == m2.cols());                 // 确保列数相同
      matrix_t res(m1.rows() + m2.rows(), m1.cols()); // 创建结果矩阵
      res << m1, m2;                                  // 垂直堆叠
      return res;
    }

    // 静态方法：垂直堆叠两个向量
    static vector_t concatenateVectors(const vector_t &v1, const vector_t &v2)
    {
      if (v1.cols() <= 0)
      { // 如果v1为空，返回v2
        return v2;
      }
      else if (v2.cols() <= 0)
      { // 如果v2为空，返回v1
        return v1;
      }
      assert(v1.cols() == v2.cols());      // 确保列数相同
      vector_t res(v1.rows() + v2.rows()); // 创建结果向量
      res << v1, v2;                       // 垂直堆叠
      return res;
    }
  };

} // namespace legged