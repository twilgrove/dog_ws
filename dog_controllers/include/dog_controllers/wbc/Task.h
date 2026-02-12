#pragma once

#include <ocs2_core/Types.h>

namespace dog_controllers
{

  using namespace ocs2;

  class Task
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Task() = default;

    Task(matrix_t a, vector_t b, matrix_t d, vector_t f)
        : a_(std::move(a)), d_(std::move(d)), b_(std::move(b)), f_(std::move(f)) {}

    explicit Task(size_t numDecisionVars)
        : a_(0, numDecisionVars), d_(0, numDecisionVars), b_(0), f_(0) {}

    ~Task() = default;

    Task operator+(const Task &rhs) const
    {
      return {concatenateMatrices(a_, rhs.a_), concatenateVectors(b_, rhs.b_),
              concatenateMatrices(d_, rhs.d_), concatenateVectors(f_, rhs.f_)};
    }

    Task operator*(scalar_t weight) const
    {
      return {a_.size() > 0 ? (weight * a_).eval() : a_,
              b_.size() > 0 ? (weight * b_).eval() : b_,
              d_.size() > 0 ? (weight * d_).eval() : d_,
              f_.size() > 0 ? (weight * f_).eval() : f_};
    }

    matrix_t a_, d_;
    vector_t b_, f_;

    static matrix_t concatenateMatrices(const matrix_t &m1, const matrix_t &m2)
    {
      if (m1.rows() == 0)
        return m2;
      if (m2.rows() == 0)
        return m1;

      assert(m1.cols() == m2.cols() && "Matrices must have the same number of columns to concatenate.");

      matrix_t res(m1.rows() + m2.rows(), m1.cols());
      res << m1, m2;
      return res;
    }

    static vector_t concatenateVectors(const vector_t &v1, const vector_t &v2)
    {
      if (v1.size() == 0)
        return v2;
      if (v2.size() == 0)
        return v1;

      vector_t res(v1.size() + v2.size());
      res << v1, v2;
      return res;
    }
  };

}