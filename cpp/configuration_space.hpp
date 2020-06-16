#ifndef CONFIGURATION_SPACE_HEADER
#define CONFIGURATION_SPACE_HEADER

#include <utility>
#include "kd_tree.hpp"

template <size_t DIMS>
class ConfigurationSpace {
public:
  ConfigurationSpace() {}
  virtual ~ConfigurationSpace() {}
  virtual bool is_free(const Point<DIMS>&) const = 0;  // true if point is in free space
  virtual bool is_unobstructed(const Point<DIMS>&, const Point<DIMS>&) const = 0;  // true if line of sight between points
  virtual Point<DIMS> random() = 0;  // random point in unobstructed space
  virtual double lebesgue() const = 0; // the lebesgue measure of the unobstructed space
};

#endif
