"""Shared gameplay/review projection and drawing; detection lives in geometry.py."""

import math

import cv2

from .geometry import intersect_plane_z, pixel_ray


def project_ball(blob, shape, k, d, camera_rt, output_rt, radius, max_range, rotated=False):
    """Use the gameplay outline-centre ray and ball plane; return None when invalid."""
    if blob is None or k is None or camera_rt is None or output_rt is None:
        return None
    u, v = blob[:2]
    if rotated:
        h, w = shape[:2]
        u, v = w - 1 - u, h - 1 - v
    rotation, origin = camera_rt
    hit = intersect_plane_z(origin, rotation @ pixel_ray(u, v, k, d), radius)
    if hit is None or math.hypot(hit[0], hit[1]) > max_range:
        return None
    rotation, translation = output_rt
    return rotation @ hit + translation


def annotate_ball(frame, blob, calibration='calibrated', observation=None):
    """Draw the same raw pixel candidate for gameplay debug and calibration review."""
    preview = frame.copy()
    status = 'NO BALL'
    if blob is not None:
        u, v, radius, _ = blob
        center = (round(u), round(v))
        if observation is not None and observation.partial:
            for angle in range(0, 360, 24):
                cv2.ellipse(preview, center, (max(1, round(radius)),)*2,
                            0, angle, angle+14, (0, 210, 255), 2)
        else:
            cv2.circle(preview, center, max(1, round(radius)), (0, 255, 0), 2)
        cv2.drawMarker(preview, center, (0, 255, 0), cv2.MARKER_CROSS, 12, 1)
        status = f'BALL pixel=({u:.0f}, {v:.0f}) radius={radius:.1f}px'
        if observation is not None:
            status += f' score={observation.confidence:.2f}'
            if observation.partial:
                status += ' PARTIAL'
    cv2.putText(preview, status, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .5,
                (0, 255, 0), 1, cv2.LINE_AA)
    if calibration != 'calibrated':
        text = ('APPROXIMATE: assumed HFOV / zero distortion / mounting TF'
                if calibration == 'approximate' else
                'UNCALIBRATED: pixels only, no metric position')
        cv2.putText(preview, text, (8, 44), cv2.FONT_HERSHEY_SIMPLEX, .45,
                    (0, 200, 255), 1, cv2.LINE_AA)
    return preview
