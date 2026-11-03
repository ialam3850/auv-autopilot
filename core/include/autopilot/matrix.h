#pragma once

#include <array>
#include <cstddef>

namespace ap {

// Tiny fixed-size matrix: just what a 4-state EKF needs. No heap, no
// exceptions, so the same code runs on the STM32 and on a laptop.
template <std::size_t R, std::size_t C>
struct Mat {
  std::array<float, R * C> v{};

  constexpr float& operator()(std::size_t r, std::size_t c) { return v[r * C + c]; }
  constexpr float operator()(std::size_t r, std::size_t c) const { return v[r * C + c]; }

  static constexpr Mat identity() {
    static_assert(R == C);
    Mat m;
    for (std::size_t i = 0; i < R; ++i) m(i, i) = 1.0f;
    return m;
  }

  constexpr Mat<C, R> transpose() const {
    Mat<C, R> t;
    for (std::size_t r = 0; r < R; ++r)
      for (std::size_t c = 0; c < C; ++c) t(c, r) = (*this)(r, c);
    return t;
  }
};

template <std::size_t R, std::size_t K, std::size_t C>
constexpr Mat<R, C> operator*(const Mat<R, K>& a, const Mat<K, C>& b) {
  Mat<R, C> out;
  for (std::size_t r = 0; r < R; ++r)
    for (std::size_t c = 0; c < C; ++c) {
      float s = 0.0f;
      for (std::size_t k = 0; k < K; ++k) s += a(r, k) * b(k, c);
      out(r, c) = s;
    }
  return out;
}

template <std::size_t R, std::size_t C>
constexpr Mat<R, C> operator+(Mat<R, C> a, const Mat<R, C>& b) {
  for (std::size_t i = 0; i < R * C; ++i) a.v[i] += b.v[i];
  return a;
}

template <std::size_t R, std::size_t C>
constexpr Mat<R, C> operator-(Mat<R, C> a, const Mat<R, C>& b) {
  for (std::size_t i = 0; i < R * C; ++i) a.v[i] -= b.v[i];
  return a;
}

// Inverse of a 3x3 via the adjugate. Returns false if (nearly) singular.
bool invert3(const Mat<3, 3>& m, Mat<3, 3>& out);

}  // namespace ap
