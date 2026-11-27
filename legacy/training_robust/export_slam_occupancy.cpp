// Read-only occupancy reconstruction from saved SLAM Toolbox sensor data.
// No ROS graph, policy commands, or evaluator ground truth are accessed.
#include <karto_sdk/Karto.h>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>

int main(int argc, char **argv) {
  if (argc != 6) {
    std::cerr << "Usage: export_slam_occupancy INPUT.data RESOLUTION MIN_PASS_THROUGH OCCUPANCY_THRESHOLD OUTPUT.json\n";
    return 2;
  }
  try {
    const double resolution = std::stod(argv[2]), threshold = std::stod(argv[4]);
    const int passes = std::stoi(argv[3]);
    if (!std::isfinite(resolution) || resolution <= 0 || resolution > 1 ||
        passes < 0 || !std::isfinite(threshold) || threshold < 0 || threshold > 1)
      throw std::runtime_error("Invalid rasterization parameters");
    karto::Dataset dataset;
    dataset.LoadFromFile(argv[1]);
    for (auto *object : dataset.GetLasers()) {
      auto *sensor = dynamic_cast<karto::Sensor *>(object);
      if (sensor) karto::SensorManager::GetInstance()->RegisterSensor(sensor);
    }
    karto::LocalizedRangeScanVector scans;
    for (const auto &entry : dataset.GetData()) {
      auto *scan = dynamic_cast<karto::LocalizedRangeScan *>(entry.second);
      if (scan) scans.push_back(scan);
    }
    std::unique_ptr<karto::OccupancyGrid> grid(
        karto::OccupancyGrid::CreateFromScans(scans, resolution, passes, threshold));
    if (!grid) throw std::runtime_error("No localized range scans");
    std::ofstream output(argv[5]);
    if (!output) throw std::runtime_error("Cannot create output");
    const auto &origin = grid->GetCoordinateConverter()->GetOffset();
    output << std::setprecision(17)
           << "{\"resolution\":" << resolution << ",\"origin\":[" << origin.GetX()
           << ',' << origin.GetY() << "],\"width\":" << grid->GetWidth()
           << ",\"height\":" << grid->GetHeight() << ",\"scans\":" << scans.size()
           << ",\"min_pass_through\":" << passes << ",\"occupancy_threshold\":" << threshold
           << ",\"data\":[";
    bool first = true;
    for (int y = 0; y < grid->GetHeight(); ++y) {
      for (int x = 0; x < grid->GetWidth(); ++x) {
        auto value = grid->GetValue(karto::Vector2<kt_int32s>(x, y));
        if (!first) output << ',';
        first = false;
        output << (value == karto::GridStates_Free ? 0 : value == karto::GridStates_Occupied ? 100 : -1);
      }
    }
    output << "]}\n";
    if (!output) throw std::runtime_error("Output write failed");
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
