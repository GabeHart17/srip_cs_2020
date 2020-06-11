#ifndef POINT_HEADER
#define POINT_HEADER

#include <array>


template<size_t DIMS>
using Point = std::array<double, DIMS>;

template<size_t DIMS>
double dist_axis(Point<DIMS> p0, Point<DIMS> p1, size_t axis) {
  return std::abs(p0[axis] - p1[axis]);
}

template<size_t DIMS>
double dist(Point<DIMS> p0, Point<DIMS> p1) {
  double res = 0;
  for (size_t i = 0; i < DIMS; i++) {
    res = hypot(res, dist_axis(p0, p1, i));
  }
  return res;
}

#endif
