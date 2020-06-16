#include "kd_tree.hpp"
#include "rectangle_configuration_space.hpp"
#include <vector>


int main(int argc, char const *argv[]) {
  // Point<3> kp = {1, 2, 3};
  // PathTree<3> pt(kp);
  // KDTree<3> kdt(pt);
  // Point<3> kp2 = {2, 2, 3};
  // PathTree<3> pt2(kp2);
  // kdt.insert(pt2);
  // Point<3> kp3 = {0.5, 1, 3};
  // PathTree<3> pt3(kp3);
  // kdt.insert(pt3);
  // Point<3> test = {0.6, 1.1, 3};
  // Point<3> n = kdt.nearest(test).point;

  Point<2> s = {0, 0};
  Point<2> g = {8, 16};
  Rectangle rect = {s, g};
  RectangleConfigurationSpace space(rect, std::vector<Rectangle>());
  Point<2> root = {4, 4};
  PathTree<2> pt(root);
  KDTree<2> kdt(&pt);
  for (int i = 0; i < 100; i++) {
    Point<2> p = space.random();
    PathTree<2>* new_pt = new PathTree<2>(p);
    kdt.insert(new_pt);
  }
  Point<2> test = {4, 8};
  Point<2> res = kdt.nearest(test).point;
  return 0;
}
