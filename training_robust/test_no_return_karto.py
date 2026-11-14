"""Exercise installed Karto to prevent fake obstacle endpoints after conversion."""
from pathlib import Path
import subprocess

import pytest


def test_substituted_ray_clears_without_hit_or_matching_feature(tmp_path):
    if not Path('/opt/ros/jazzy/include/karto_sdk/Karto.h').exists():
        pytest.skip('Requires installed Jazzy Karto library')
    source = tmp_path/'probe.cpp'
    source.write_text(r'''
#include <karto_sdk/Karto.h>
#include <iostream>
#include <memory>
#include <limits>
int main(int argc, char** argv) {
  double threshold=std::stod(argv[1]);
  auto* laser=karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom,karto::Name("probe"));
  laser->SetMinimumRange(.164); laser->SetMaximumRange(12.);
  laser->SetRangeThreshold(threshold);
  laser->SetMinimumAngle(-3.141592653589793); laser->SetMaximumAngle(3.141592653589793);
  laser->SetAngularResolution(3.141592653589793/4.);
  karto::SensorManager::GetInstance()->RegisterSensor(laser);
  karto::LocalizedRangeScanVector scans;
  std::vector<std::unique_ptr<karto::LocalizedRangeScan>> owned;
  for(int i=0;i<5;++i) {
    // Two real hits bound the map; no-return beams must not add more hits.
    std::vector<double> ranges(9,argc>2 ? std::numeric_limits<double>::infinity() : 11.95);
    ranges[1]=10.; ranges[5]=10.;
    auto s=std::make_unique<karto::LocalizedRangeScan>(laser->GetName(),ranges);
    s->SetOdometricPose(karto::Pose2(0.,0.,0.)); s->SetCorrectedPose(karto::Pose2(0.,0.,0.));
    scans.push_back(s.get()); owned.push_back(std::move(s));
  }
  std::unique_ptr<karto::OccupancyGrid> grid(karto::OccupancyGrid::CreateFromScans(scans,.05,2,.1));
  int free=0,occupied=0;
  for(int y=0;y<grid->GetHeight();++y) for(int x=0;x<grid->GetWidth();++x) {
    auto v=grid->GetValue(karto::Vector2<kt_int32s>(x,y));
    free += v==karto::GridStates_Free; occupied += v==karto::GridStates_Occupied;
  }
  std::cout<<"RESULT "<<free<<" "<<occupied<<" "<<scans[0]->GetPointReadings(true).size()<<std::endl;
}
''')
    binary = tmp_path/'probe'
    subprocess.run(['g++','-std=c++17','-O1','-I/opt/ros/jazzy/include','-I/usr/include/eigen3',str(source),
        '-L/opt/ros/jazzy/lib','-Wl,-rpath,/opt/ros/jazzy/lib','-lkartoSlamToolbox',
        '-lboost_serialization','-lboost_thread','-lpthread','-o',str(binary)],check=True,capture_output=True)
    def run(threshold, raw=False):
        output=subprocess.check_output([str(binary),str(threshold)]+(['raw'] if raw else []),text=True)
        line=next(s for s in output.splitlines() if s.startswith('RESULT '))
        return tuple(map(int,line.split()[1:]))
    raw_free,raw_occupied,raw_features=run(11.9,raw=True)
    free,occupied,features=run(11.9)
    assert free>raw_free and occupied==raw_occupied>0 and features==raw_features==2
    _,occupied,features=run(12.)
    assert occupied>2 and features>2  # would create a false range-limit wall
