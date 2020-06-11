#ifndef KD_TREE_HEADER
#define KD_TREE_HEADER

#include <vector>
#include <array>
#include <cmath>
#include "path_tree.hpp"


template<size_t DIMS>
class KDTree {
private:
  struct KDNode {
    PathTree<DIMS>& pt;
    KDNode* smaller;
    KDNode* greater;
    KDNode* parent;
    KDNode(const PathTree<DIMS>& n, KDNode* p): pt(n), parent(p), smaller(nullptr), greater(nullptr) {}
    ~KDNode() {
      delete smaller;
      delete greater;
    }
  };

  KDNode* root_;

  KDNode* nearest_helper_(const Point<DIMS>& p, KDNode* r, size_t axis) const {
    double d_r = dist(p, r->pt.point);
    double d_s = 0;
    double d_g = 0;
    unsigned char children = 0; // 1s bit is presence of smaller child, 2s is greater
    if (r->smaller != nullptr) {
      d_s = dist(p, r->smaller->pt.point);
      children += 1;
    }
    if (r->greater != nullptr) {
      d_g = dist(p, r->greater->pt.point);
      children += 2;
    }
    if (!children) {
      return r;
    }
    bool side = ((children == 3) && (d_s < d_g)) || (children == 2);  // true if closer to smaller, false if closer to greater
    KDNode* side_res = nearest_helper_(p, side ? r->smaller : r-> greater, (axis + 1) % DIMS);
    double side_dist = dist(p, side_res->pt.point);
    if (side_dist <= dist_axis(p, r->pt.point, axis)) {
      return side_res;
    } else if (children < 3) {
      return side_dist < d_r ? side_res : r;
    } else {
      KDNode* other_res = nearest_helper_(p, side ? r-> greater : r->smaller, (axis + 1) % DIMS);
      double other_dist = dist(p, other_res->pt.point);
      unsigned char best = (side_dist < d_r && side_dist < other_dist) +
                           2 * (other_dist < d_r && other_dist < side_dist);  // 0 for local root, 1 for side, 2 for other
      switch (best) {
        case 0:
        return r;
        case 1:
        return side_res;
        case 2:
        return other_res;
      }
    }
  }

  void near_helper_(const Point<DIMS>& p, double radius, std::vector<PathTree<DIMS>& >& v, KDNode* r, size_t axis) const {
    bool continue_s = false;
    bool continue_g = false;
    if (dist(p, r->pt.point) < radius) {
      v.push_back(r->pt);
    }
    if (dist_axis(p, r->pt.point, axis) < radius) {
      continue_s = true;
      continue_g = true;
    } else if (p[axis] > r->pt.point[axis]) {
      continue_g = true;
    } else {
      continue_s = true;
    }
    if (continue_g && r->greater != nullptr) {
      near_helper_(p, radius, v, r->greater, (axis + 1) % DIMS);
    }
    if (continue_s && r->smaller != nullptr) {
      near_helper_(p, radius, v, r->smaller, (axis + 1) % DIMS);
    }
  }

public:
  KDTree() {
    root_ = nullptr;
  }

  KDTree(PathTree<DIMS>& root) {
    root_ = new KDNode(root, nullptr);
  }

  ~KDTree() {
    delete root_;
  }


  void insert(const PathTree<DIMS>& p) {
    KDNode* parent = root_;
    size_t axis = 0;
    while (true) {
      KDNode* branch = (parent->pt.point)[axis] < p.point[axis] ? parent->greater : parent->smaller;
      if (branch == nullptr) {
        ((parent->pt.point)[axis] < p.point[axis] ? parent->greater : parent->smaller) =
          new KDNode(p, parent);
        return;
      } else {
        parent = branch;
        axis++;
        axis %= DIMS;
      }
    }
  }

  PathTree<DIMS>& nearest(const Point<DIMS>& p) const {
    return nearest_helper_(p, root_, 0)->pt;
  }

  void remove(const Point<DIMS>& p) {
    KDNode* near = nearest_helper_(p, root_, 0);
    KDNode* parent = near->parent;
    bool smaller = parent->smaller == near;
    delete near;
    *(smaller ? parent->smaller : parent->larger) = nullptr;
  }

  // returns vector where the first element is the start and the last is the goal
  // is working, but is not needed
  // std::vector<Point<DIMS> > pathToNearest(Point<DIMS> p) const {
  //   std::vector<Point<DIMS> > res;
  //   KDNode* n = nearest(p);
  //   while (n != nullptr) {
  //     res.push_back(n->pt.point);
  //     n = n->parent;
  //   }
  //   std::vector<Point<DIMS> > r_res;
  //   for (typename std::vector<Point<DIMS> >::reverse_iterator it = res.rbegin(); it != res.rend(); it++) {
  //     r_res.push_back(*it);
  //   }
  //   return r_res;
  // }

  // all nodes within radius r of p
  std::vector<PathTree<DIMS>& > near(const Point<DIMS>& p, double r) const {
    std::vector<Point<DIMS> > res;
    return near_helper_(p, r, res, root_, 0);
  }
};


#endif
