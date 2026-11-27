"""Reuse the exam launch, exposing real detector pixels from the common start."""
import importlib.util
import os
from pathlib import Path


def generate_launch_description():
    path = Path(os.environ['SURVEY_EXPERIMENTS'])/'2026-11-21_camera_aware_policy/camera.launch.py'
    spec = importlib.util.spec_from_file_location('frozen_exam_camera_launch', path)
    native = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(native)
    original_include = native.include

    def include(path, **arguments):
        if path.name == 'robot.launch.py':
            arguments['rgb_topic'] = '/exam/detector_rgb'
        return original_include(path, **arguments)

    native.include = include
    return native.generate_launch_description()
