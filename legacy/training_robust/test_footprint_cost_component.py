import os
from pathlib import Path
import shlex
import subprocess

import numpy as np

from audit_navigation_costs import costs


def test_installed_cpp_checker_keeps_real_obstacles_and_padding(tmp_path):
    source=Path(__file__).with_name('footprint_cost_component.cpp').resolve()
    (tmp_path/'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.16)\n'
        'project(footprint_component_test)\nfind_package(ament_cmake REQUIRED)\n'
        'find_package(nav2_costmap_2d REQUIRED)\n'
        f'add_executable(checker "{source}")\nament_target_dependencies(checker nav2_costmap_2d)\n')
    command='source /opt/ros/jazzy/setup.bash\ncmake -S '+shlex.quote(str(tmp_path))+' -B '+shlex.quote(str(tmp_path/'build'))
    command+='\ncmake --build '+shlex.quote(str(tmp_path/'build'))+' -j 1'
    subprocess.run(['/bin/bash','-eo','pipefail','-c',command],check=True,capture_output=True,text=True)
    binary=tmp_path/'build/checker';grid=np.zeros((400,400),dtype=np.uint8)
    footprint=[[.189,0.],[.134,-.134],[0.,-.189],[-.134,-.134],[-.189,0.],[-.134,.134],[0.,.189],[.134,.134]]
    poses=[[2.,2.,0.],[2.,2.,1.3]]
    assert np.all(costs(binary,grid,.01,[0.,0.,0.],footprint,.01,poses)==0)
    grid[:,219]=254
    assert costs(binary,grid,.01,[0.,0.,0.],footprint,0.,poses[:1])[0]==0
    assert costs(binary,grid,.01,[0.,0.,0.],footprint,.01,poses[:1])[0]==254
