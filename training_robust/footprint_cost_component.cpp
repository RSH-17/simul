// Offline calls to the installed Nav2 checker, never a ROS node or policy input.
#include <iostream>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>

int main() {
  unsigned width, height, points, queries;
  double resolution, ox, oy, padding;
  if (!(std::cin >> width >> height >> resolution >> ox >> oy)) return 2;
  nav2_costmap_2d::Costmap2D map(width, height, resolution, ox, oy, 0);
  for (unsigned y = 0; y < height; ++y) {
    for (unsigned x = 0; x < width; ++x) {
      unsigned cost;
      if (!(std::cin >> cost) || cost > 255) return 3;
      map.setCost(x, y, cost);
    }
  }
  if (!(std::cin >> points >> padding) || points < 3) return 4;
  nav2_costmap_2d::Footprint footprint(points);
  for (auto & point : footprint) if (!(std::cin >> point.x >> point.y)) return 5;
  nav2_costmap_2d::padFootprint(footprint, padding);
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(&map);
  if (!(std::cin >> queries)) return 6;
  for (unsigned i = 0; i < queries; ++i) {
    double x, y, yaw;
    if (!(std::cin >> x >> y >> yaw)) return 7;
    std::cout << checker.footprintCostAtPose(x, y, yaw, footprint) << '\n';
  }
}
