from glob import glob
from setuptools import setup

package_name = 'turtlebot4_payload_sim'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/'+package_name]),
                  ('share/'+package_name, ['package.xml']),
                  ('share/'+package_name+'/config', glob('config/*')),
                  ('share/'+package_name+'/launch', glob('launch/*')),
                  ('share/'+package_name+'/worlds', glob('worlds/*'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='simul', maintainer_email='peter010905@users.noreply.github.com',
      description='Parametric TurtleBot4 mounted payload simulator',
      license='Apache-2.0',
      entry_points={'console_scripts': [
          'payload_model = turtlebot4_payload_sim.model:main',
          'payload_scan_probe = turtlebot4_payload_sim.scan_probe:main',
          'payload_nav_probe = turtlebot4_payload_sim.nav_probe:main',
          'payload_fixture_filter = turtlebot4_payload_sim.fixture_filter:main',
          'payload_filter_probe = turtlebot4_payload_sim.filter_probe:main',
          'payload_sweep = turtlebot4_payload_sim.sweep:main',
          'payload_motion_probe = turtlebot4_payload_sim.motion_probe:main',
          'payload_analyze_sweep = turtlebot4_payload_sim.analyze_sweep:main',
          'payload_analyze = turtlebot4_payload_sim.analyze:main']})
