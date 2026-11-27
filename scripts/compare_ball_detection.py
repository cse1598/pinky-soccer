#!/usr/bin/env python3
"""Compare frozen/new pixels and unchanged projection on identical field images."""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml
from pinky_mujoco.description import expand_urdf
from pinky_soccer_bringup.camera_mount import apply_mount, fixed_transforms, OPTICAL
from pinky_soccer_world_model.ball_detector import load_camera_yaml
from pinky_soccer_world_model.ball_offset import apply_ball_offset, load_ball_offset
from pinky_soccer_world_model.ball_vision import project_ball
from pinky_soccer_world_model.geometry import intrinsics_from_hfov
from pinky_soccer_world_model.robust_ball import BallImageDetector

ROOT = Path(__file__).resolve().parents[1]
VALIDATION = ROOT/'docs/validation/ball_hls_20261127'


def baseline(name, filename):
    spec = importlib.util.spec_from_file_location('pinky_soccer_world_model.'+name, VALIDATION/filename)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def cases():
    entries = []
    for name, center, radius in [
            ('full_small', (320, 350), 10), ('full', (320, 350), 35),
            ('left_cut', (5, 300), 35), ('center_outside', (-5, 300), 35),
            ('bottom_cut', (320, 475), 35), ('roi_cut', (320, 140), 35),
            ('short_arc', (-25, 300), 35), ('occluded', (320, 350), 35),
            ('shadow', (320, 350), 35), ('glare', (320, 350), 35)]:
        image = np.zeros((480, 640, 3), np.uint8)
        cv2.circle(image, center, radius, (4, 57, 230), -1)
        if name == 'occluded':
            cv2.rectangle(image, (330, 300), (380, 400), (40, 50, 60), -1)
        if name == 'shadow':
            image[:, :325] = (image[:, :325].astype(float)*.3).astype(np.uint8)
        if name == 'glare':
            cv2.circle(image, (320, 340), 20, (255, 255, 255), -1)
        entries.append((name, image, (*center, radius), 'synthetic'))
    for engine in ('mujoco', 'gazebo'):
        path = ROOT/'docs/validation/sim_colors_20261127'/f'{engine}_blue.png'
        if path.exists():
            entries.append((engine+'_camera', cv2.imread(str(path)), None, 'simulation'))
    return entries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--real-profile', default='')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    old = baseline('before_hls_ball', 'baseline_robust_ball.py')
    projection = baseline('before_hls_projection', 'baseline_ball_vision.py')
    urdf = expand_urdf(package='pinky_soccer_description', filename='urdf/soccer_robot.urdf.xacro')
    transforms = fixed_transforms(ET.fromstring(urdf), 'base_footprint')
    cam = transforms['front_camera_link']
    standard_cam = (cam[:3, :3]@OPTICAL, cam[:3, 3])
    inverse = np.linalg.inv(transforms['base_link'])
    output = inverse[:3, :3], inverse[:3, 3]
    standard_k, standard_d = intrinsics_from_hfov(640, 480, math.degrees(1.1519)), np.zeros(5)
    real_k, real_d, real_cam, offset = standard_k, standard_d, standard_cam, np.zeros(2)
    if args.real_profile:
        real_k, real_d, _ = load_camera_yaml(args.real_profile)
        data = yaml.safe_load(Path(args.real_profile).read_text())
        if data.get('camera_mount'):
            real_cam = apply_mount(standard_cam, data['camera_mount'])
        offset, _ = load_ball_offset(args.real_profile)
    results, panels, differences = [], [], []
    for name, image, truth, environment in cases():
        k, d, cam_rt = ((real_k, real_d, real_cam) if environment == 'synthetic' else
                        (standard_k, standard_d, standard_cam))
        row = dict(case=name, pixel_truth=truth, environment=environment)
        pair = []
        for label, detector in [('before', old.BallImageDetector([((3, 160, 70), (19, 255, 255))])),
                                ('after', BallImageDetector([]))]:
            times = []
            for i in range(8):
                started = time.perf_counter()
                observation = detector.detect(image, 1+i/15, (name, i))
                times.append(1000*(time.perf_counter()-started))
            blob = observation.blob
            point = project_ball(blob, image.shape, k, d, cam_rt, output, .02135, 2.5)
            frozen = projection.project_ball(blob, image.shape, k, d, cam_rt, output, .02135, 2.5)
            if point is not None:
                differences.append(float(np.max(np.abs(point-frozen))))
            corrected = apply_ball_offset(point, offset if environment == 'synthetic' else [0., 0.])
            values = dict(blob=blob, point=point.tolist() if point is not None else None,
                corrected_point=corrected.tolist() if corrected is not None else None,
                acquisition_ms=times[0], mean_tracking_ms=float(np.mean(times[1:])),
                p95_tracking_ms=float(np.percentile(times[1:], 95)), max_tracking_ms=max(times[1:]),
                bearing_valid=getattr(observation, 'bearing_valid', blob is not None))
            if truth and blob:
                values.update(center_error_px=math.dist(blob[:2], truth[:2]), radius_bias_px=blob[2]-truth[2])
            row[label] = values
            preview = image.copy()
            if blob:
                cv2.circle(preview, tuple(round(v) for v in blob[:2]), round(blob[2]), (0, 255, 0), 2)
            cv2.putText(preview, f'{name} {label}: '+(f'r={blob[2]:.2f}' if blob else 'no metric circle'),
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 2)
            pair.append(cv2.resize(preview, (480, 360)))
        panels.append(np.hstack(pair))
        results.append(row)
    payload = dict(same_input_projection_max_abs_difference_m=max(differences, default=0.),
        physical_radius_m=.02135, image_radius_unit='pixels; ignored by project_ball',
        real_offset_xy_m=offset.tolist(), cases=results)
    (args.output/'comparison.json').write_text(json.dumps(payload, indent=2)+'\n')
    cv2.imwrite(str(args.output/'comparison.png'), np.vstack(panels))
    print(json.dumps(payload, indent=2))


if __name__ == '__main__':
    main()
