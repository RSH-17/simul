// Read stored sensor readings without changing the serialized SLAM graph.
// These are SLAM input scans, not a reconstruction of every detector scan.
#include <karto_sdk/Karto.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "Usage: export_slam_scan_readings INPUT.data NEW_OUTPUT.csv\n";
    return 2;
  }
  try {
    if (std::filesystem::exists(argv[2])) {
      throw std::runtime_error("Preserve the existing export");
    }
    karto::Dataset dataset;
    dataset.LoadFromFile(argv[1]);
    for (auto *object : dataset.GetLasers()) {
      auto *sensor = dynamic_cast<karto::Sensor *>(object);
      if (sensor) karto::SensorManager::GetInstance()->RegisterSensor(sensor);
    }
    std::ofstream output(argv[2]);
    if (!output) throw std::runtime_error("Cannot create output CSV");
    output << std::setprecision(17);
    size_t count = 0, beams = 0;
    for (const auto &entry : dataset.GetData()) {
      const auto *scan = dynamic_cast<const karto::LocalizedRangeScan *>(entry.second);
      if (!scan) continue;
      auto *laser = scan->GetLaserRangeFinder();
      const auto ranges = scan->GetRangeReadingsVector();
      if (!count) {
        beams = ranges.size();
        output << "node_id,stamp,range_min,range_max,angle_min,angle_increment,offset_x,offset_y,offset_yaw";
        for (size_t i = 0; i < beams; ++i) output << ",r" << i;
        output << '\n';
      }
      if (ranges.size() != beams || beams == 0) {
        throw std::runtime_error("Unsupported variable or empty beam count");
      }
      const auto offset = laser->GetOffsetPose();
      output << scan->GetUniqueId() << ',' << scan->GetTime() << ','
             << laser->GetMinimumRange() << ',' << laser->GetMaximumRange() << ','
             << laser->GetMinimumAngle() << ',' << laser->GetAngularResolution() << ','
             << offset.GetX() << ',' << offset.GetY() << ',' << offset.GetHeading();
      for (const auto range : ranges) output << ',' << range;
      output << '\n';
      ++count;
    }
    if (!count) throw std::runtime_error("Dataset contains no localized range scans");
    output.close();
    if (!output) throw std::runtime_error("Failed to finish export");
    std::cerr << "Exported " << count << " stored scans with " << beams << " beams\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
