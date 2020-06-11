#include "kd_tree.hpp"


int main(int argc, char const *argv[]) {
  Point<3> kp = {1, 2, 3};
  PathTree<3> pt(kp);
  KDTree<3> kdt(pt);
  Point<3> kp2 = {2, 2, 3};
  PathTree<3> pt2(kp2);
  kdt.insert(pt2);
  Point<3> kp3 = {0.5, 1, 3};
  PathTree<3> pt3(kp3);
  kdt.insert(pt3);
  Point<3> test = {0.6, 1.1, 3};
  Point<3> n = kdt.nearest(test).point;
  return 0;
}
