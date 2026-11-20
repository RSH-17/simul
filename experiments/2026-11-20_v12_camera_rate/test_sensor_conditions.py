"""Verify simulated camera changes preserve IMU/LiDAR and explicit FPS."""
from xml.dom import minidom
import pytest
from osp_mission.sensor_model import configure_sensors


@pytest.mark.parametrize('width',[250,320,400,480,640])
@pytest.mark.parametrize('fps',[5,10,15])
def test_camera_condition_preserves_other_sensor_rates(monkeypatch,width,fps):
    monkeypatch.setenv('CAMERA_BENCH_WIDTH',str(width));monkeypatch.setenv('CAMERA_BENCH_HEIGHT',str(width))
    monkeypatch.setenv('CAMERA_BENCH_FPS',str(fps))
    doc=minidom.parseString('''<robot><sensor type="rgbd_camera"><update_rate>30</update_rate>
        <image><width>320</width><height>240</height></image></sensor>
        <sensor type="imu"><update_rate>200</update_rate></sensor>
        <sensor type="gpu_lidar"><update_rate>10</update_rate></sensor></robot>''')
    configure_sensors(doc,640)
    rgb,imu,lidar=doc.getElementsByTagName('sensor')
    value=lambda node,tag:float(node.getElementsByTagName(tag)[0].firstChild.data)
    assert value(rgb,'width')==value(rgb,'height')==width and value(rgb,'update_rate')==fps
    assert value(imu,'update_rate')==100 and value(lidar,'update_rate')==10
