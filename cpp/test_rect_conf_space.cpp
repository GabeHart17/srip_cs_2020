#include <vector>
#include "rectangle_configuration_space.hpp"
#include "point.hpp"


int main(int argc, char const *argv[]) {
  Point<2> p0 = {0, 0};
  Point<2> p1 = {1, 2};
  Rectangle r0(p0, p1);
  RectangleConfigurationSpace rcs(r0, std::vector<Rectangle>());
  return 0;
}
