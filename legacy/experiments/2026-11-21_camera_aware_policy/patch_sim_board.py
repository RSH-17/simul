"""Install a display-only schema fix; never restart ROS, Gazebo or the board."""
import argparse
from pathlib import Path
import shutil

OLD_PY="""    boxes = [dict(name=b.get('name', ''), x=b['x'], y=b['y'], sx=b['sx'], sy=b['sy'], yaw=b.get('yaw', 0.))
             for b in list(scene.get('partitions', []))+list(scene.get('obstacles', []))]"""
NEW_PY="""    from camera_scene_shapes import obstacle_shapes
    boxes = obstacle_shapes(list(scene.get('partitions', []))+list(scene.get('obstacles', [])))"""
OLD_JS="""      ctx.fillRect(-b.sx * v.s / 2, -b.sy * v.s / 2, b.sx * v.s, b.sy * v.s);"""
NEW_JS="""      if (b.radius != null) {
        ctx.beginPath(); ctx.arc(0, 0, b.radius * v.s, 0, 2 * Math.PI); ctx.fill();
      } else {
        ctx.fillRect(-b.sx * v.s / 2, -b.sy * v.s / 2, b.sx * v.s, b.sy * v.s);
      }"""


def install(directory):
    changes=[]
    for path,old,new in ((directory/'sim_board.py',OLD_PY,NEW_PY),
                         (directory/'sim_web/app.js',OLD_JS,NEW_JS)):
        text=path.read_text()
        if new in text:continue
        assert text.count(old)==1, f'Unrecognized board source: {path}'
        changes.append((path,text.replace(old,new)))
    for path,text in changes:
        backup=path.with_name(path.name+'.before_camera_shapes')
        if not backup.exists():shutil.copy2(path,backup)
        path.write_text(text)
    shutil.copy2(Path(__file__).with_name('camera_scene_shapes.py'),directory/'camera_scene_shapes.py')
    print('Display schema installed. No processes restarted.')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path)
    install(parser.parse_args().directory)
