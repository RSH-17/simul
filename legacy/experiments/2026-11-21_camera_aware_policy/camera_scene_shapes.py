"""Display-only normalization of box and cylinder scenario geometry."""
import math


def obstacle_shapes(obstacles):
    result=[]
    for item in obstacles:
        try:
            row=dict(name=item.get('name',''),x=float(item['x']),y=float(item['y']),yaw=float(item.get('yaw',0.)))
            if item.get('type')=='cylinder':
                radius=float(item['radius'])
                row.update(radius=radius,sx=2*radius,sy=2*radius)
            else:
                row.update(sx=float(item['sx']),sy=float(item['sy']))
            if not all(math.isfinite(row[k]) for k in ('x','y','yaw','sx','sy')):continue
            if min(row['sx'],row['sy'])<=0:continue
        except (TypeError,ValueError,KeyError):
            continue
        result.append(row)
    return result
