"""Optional, reproducible degradation of the RGB frames forwarded to the detector.

Set TEAM_SIM_CAMERA_NOISE to a comma list, e.g. "sigma=6,blur=1.0,drop=0.05,seed=1":
  sigma  additive Gaussian pixel noise (0-255 scale)
  blur   Gaussian blur sigma in pixels (motion/defocus stand-in)
  drop   probability of withholding a frame (USB / DDS loss stand-in)
  seed   RNG seed so a run can be repeated
The simulator camera is otherwise noise free. This is a separate stress
condition: compare only runs recorded with the same setting.
"""
import os

import numpy as np

KEYS = {'sigma': float, 'blur': float, 'drop': float, 'seed': int}


def parse(text):
    settings = dict(sigma=0., blur=0., drop=0., seed=0)
    for item in filter(None, (part.strip() for part in (text or '').split(','))):
        key, _, value = item.partition('=')
        key = key.strip()
        if key not in KEYS or not value:
            raise ValueError(f'TEAM_SIM_CAMERA_NOISE: unknown or empty entry {item!r}')
        settings[key] = KEYS[key](value)
    if settings['sigma'] < 0 or settings['blur'] < 0 or not 0 <= settings['drop'] < 1:
        raise ValueError('TEAM_SIM_CAMERA_NOISE: sigma/blur must be >= 0 and 0 <= drop < 1')
    return settings


class CameraNoise:
    def __init__(self, settings):
        self.settings = settings
        self.enabled = any(settings[k] > 0 for k in ('sigma', 'blur', 'drop'))
        self.rng = np.random.default_rng(settings['seed'])
        self.forwarded = self.dropped = 0

    @classmethod
    def from_environment(cls, environment=os.environ):
        return cls(parse(environment.get('TEAM_SIM_CAMERA_NOISE', '')))

    def apply(self, msg):
        """Return the frame to forward, or None when it is dropped."""
        if not self.enabled:
            self.forwarded += 1
            return msg
        if self.settings['drop'] and self.rng.random() < self.settings['drop']:
            self.dropped += 1
            return None
        import cv2
        from cv_bridge import CvBridge
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, 'bgr8').astype(np.float32)
        if self.settings['blur'] > 0:
            image = cv2.GaussianBlur(image, (0, 0), self.settings['blur'])
        if self.settings['sigma'] > 0:
            image += self.rng.normal(0., self.settings['sigma'], image.shape).astype(np.float32)
        out = bridge.cv2_to_imgmsg(np.clip(image, 0, 255).astype(np.uint8), 'bgr8')
        out.header = msg.header
        self.forwarded += 1
        return out

    def report(self):
        return dict(self.settings, enabled=self.enabled, forwarded=self.forwarded, dropped=self.dropped)
