#include <iostream>
#include <vector>
#include "rrt_star.hpp"
#include "rectangle_configuration_space.hpp"


int main(int argc, char const *argv[]) {
  Point<2> s = {0, 0};
  Point<2> g = {8, 16};
  RectangleConfigurationSpace space(Rectangle(s, g), std::vector<Rectangle>());
  RRTStarPlanner<2> planner(&space);
  Point<2> start = {4, 1};
  Point<2> goal = {4, 12};
  double eta = 0.1;
  double radius = 0.5;
  std::vector<Point<2> > path = planner.path(start, goal, 1000, radius, eta);
  for (Point<2> p : path) {
    std::cout << p[0] << ", " << p[1] << '\n';
  }
  return 0;
}
