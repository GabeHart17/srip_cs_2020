#ifndef CONFIGURATION_SPACE_HEADER
#define CONFIGURATION_SPACE_HEADER

#include <utility>
#include "kd_tree.hpp"

template <size_t DIMS>
class ConfigurationSpace {
public:
  ConfigurationSpace();
  ~ConfigurationSpace() {}
  bool is_free(Point<DIMS>) const;
  bool is_obstructed(Point<DIMS>, Point<DIMS>) const;
  Point<DIMS> random() const;
};

#endif
