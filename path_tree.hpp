#ifndef PATH_TREE_HEADER
#define PATH_TREE_HEADER

#include <vector>
#include <array>
#include <cmath>
#include "point.hpp"


template<size_t DIMS>
struct PathTree {
  PathTree<DIMS>* parent;
  std::vector<PathTree<DIMS>* > children;
  Point<DIMS> point;
  double cost;
  PathTree(Point<DIMS> p): point(p), cost(INFINITY), parent(nullptr) {}
  ~PathTree() {
    for (PathTree<DIMS>* i : children) {
      delete i;
    }
  }
};

#endif
