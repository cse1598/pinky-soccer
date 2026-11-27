"""Common HLS/arc detector; pixel observations retain full-image coordinates."""
from dataclasses import dataclass
import math

import numpy as np

ROBUST_DEFAULTS = {'min_arc_fraction': .42, 'min_seed_fraction': .10,
                   'min_explained_fraction': .42, 'min_detection_confidence': .48,
                   'visual_memory_s': .7, 'detection_top_fraction': 130/480,
                   'min_bearing_arc_fraction': .16, 'max_radius_uncertainty': .08,
                   'min_contiguous_arc_fraction': .25}


@dataclass
class BallObservation:
    """A current observation; absent/unreliable circles never become metric points."""

    blob: tuple | None
    mask: np.ndarray
    confidence: float = 0.
    arc_fraction: float = 0.
    color_fraction: float = 0.
    partial: bool = False
    candidates: int = 0
    roi_top: int = 0
    bearing_uv: tuple | None = None
    bearing_valid: bool = False
    distance_valid: bool = False
    fit_error_px: float | None = None
    radius_uncertainty_px: float | None = None


def circle_fit(points):
    """Least-squares circle from visible boundary inliers, with degeneracy rejection."""
    if len(points) < 8:
        return None
    mean = points.mean(axis=0)
    xy = points-mean
    a = np.column_stack((2*xy, np.ones(len(xy))))
    if np.linalg.cond(a) > 1e4:
        return None
    x, y, c = np.linalg.lstsq(a, np.sum(xy*xy, axis=1), rcond=None)[0]
    radius2 = c+x*x+y*y
    if radius2 <= 0:
        return None
    return float(x+mean[0]), float(y+mean[1]), float(math.sqrt(radius2))



def observe_ball(frame, ranges, min_area=12, min_circularity=.72,
                 shape=None, options=None, previous=None, hls_ranges=None):
    """Fit within the lower ROI and restore u/v/r in original image pixels.

    ranges/min_circularity remain accepted for old callers; HLS uses its explicit
    configuration. No radius-to-distance formula is introduced here.
    """
    from .hls_ball import HLS_RANGES, region
    cfg = {**ROBUST_DEFAULTS, **(options or {})}
    fraction = cfg['detection_top_fraction']
    if not math.isfinite(fraction) or not 0 <= fraction < 1:
        raise ValueError('detection_top_fraction must be between 0 (inclusive) and 1')
    height, width = frame.shape[:2]
    top = min(height-1, math.ceil(height*fraction))
    if min(height-top, width) < 8:
        return BallObservation(None, np.zeros((height, width), np.uint8), roi_top=top)
    prior = ((previous[0], previous[1]-top, *previous[2:])
             if previous is not None and previous[1] >= top else None)
    result = region(frame[top:], HLS_RANGES if hls_ranges is None else hls_ranges,
                    min_area, shape or {}, cfg, prior)
    mask = np.zeros((height, width), np.uint8)
    mask[top:] = result.mask
    result.mask, result.roi_top = mask, top
    if result.blob is not None:
        u, v, radius, area = result.blob
        result.blob = (u, v+top, radius, area)
    if result.bearing_uv is not None:
        result.bearing_uv = (result.bearing_uv[0], result.bearing_uv[1]+top)
    return result


class BallImageDetector:
    """Retain one pixel target briefly, but never manufacture a current observation."""

    def __init__(self, ranges, min_area=12, min_circularity=.72, shape=None, options=None, hls_ranges=None):
        self.ranges, self.min_area, self.min_circularity = ranges, min_area, min_circularity
        self.shape, self.options = shape, {**ROBUST_DEFAULTS, **(options or {})}
        for key, value in self.options.items():
            if key == 'detection_top_fraction':
                if not math.isfinite(value) or not 0 <= value < 1:
                    raise ValueError(f'Invalid robust detector option: {key}')
                continue
            if not math.isfinite(value) or value <= 0 or (key != 'visual_memory_s' and value > 1):
                raise ValueError(f'Invalid robust detector option: {key}')
        from .hls_ball import HLS_RANGES
        self.hls_ranges = HLS_RANGES if hls_ranges is None else hls_ranges
        for low, high in self.hls_ranges:
            if (len(low) != 3 or len(high) != 3 or
                    any(not 0 <= a <= b <= limit for a, b, limit in zip(low, high, (180, 255, 255)))):
                raise ValueError('Invalid HLS ball bounds')
        self.reset()

    def reset(self):
        self.previous = self.last_observed = self.last_time = self.frame_key = self.result = None
        self.frame_shape = None

    def detect(self, frame, timestamp, frame_key=None):
        if self.frame_shape is not None and self.frame_shape != frame.shape:
            self.reset()
        if self.last_time is not None and timestamp < self.last_time:
            self.reset()
        self.frame_shape = frame.shape
        if frame_key is not None and frame_key == self.frame_key and self.result is not None:
            return self.result
        if (self.last_observed is not None
                and timestamp-self.last_observed > self.options['visual_memory_s']):
            self.previous = None
        self.result = observe_ball(frame, self.ranges, self.min_area, self.min_circularity,
                                   self.shape, self.options, self.previous, self.hls_ranges)
        self.frame_key, self.last_time = frame_key, timestamp
        if self.result.blob is not None:
            self.previous, self.last_observed = self.result.blob, timestamp
        return self.result
