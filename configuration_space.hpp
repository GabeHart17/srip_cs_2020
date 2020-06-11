#ifndef CONFIGURATION_SPACE_HEADER
#define CONFIGURATION_SPACE_HEADER

#include <utility>
#include "kd_tree.hpp"

template <size_t DIMS>
class ConfigurationSpace {
public:
  ConfigurationSpace();
  ~ConfigurationSpace() {}
  bool is_free(Point<DIMS>) const;  // true if point is in free space
  bool is_unobstructed(Point<DIMS>, Point<DIMS>) const;  // true if line of sight between points
  Point<DIMS> random() const;  // random point in unobstructed space
};

#endif
