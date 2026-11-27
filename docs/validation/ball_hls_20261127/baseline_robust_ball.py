"""Single orange-ball observation from colour evidence and visible circular arcs.

Colour seeds suggest identity, not a connected object silhouette. Image edges
separate a ball from a similarly coloured table. Missing colour under glare and
missing arcs under occlusion are allowed; a circle without evidence is not.
"""

from dataclasses import dataclass
import math

import cv2
import numpy as np

from .geometry import _compact_blob, color_mask

ROBUST_DEFAULTS = {'min_arc_fraction': .42, 'min_seed_fraction': .10,
                   'min_explained_fraction': .42, 'min_detection_confidence': .48,
                   'visual_memory_s': .7, 'detection_top_fraction': 1/3}


@dataclass
class BallObservation:
    """One current-frame observation, not a prediction through complete invisibility."""

    blob: tuple | None
    mask: np.ndarray
    confidence: float = 0.
    arc_fraction: float = 0.
    color_fraction: float = 0.
    partial: bool = False
    candidates: int = 0
    roi_top: int = 0


def illumination_evidence(frame, ranges):
    """Combine raw and bounded neutral-patch white balance, preserving dark colour."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    neutral = (hsv[:, :, 1] < 80) & (hsv[:, :, 2] > 120)
    gain = np.ones(3)
    if np.count_nonzero(neutral) > max(64, frame.shape[0]*frame.shape[1]*.005):
        white = np.mean(frame[neutral], axis=0)
        gain = np.clip(np.mean(white)/np.maximum(white, 1), .75, 1.3)
    balanced = np.clip(frame.astype(np.float32)*gain, 0, 255).astype(np.uint8)
    seed_ranges, weak_ranges = [], []
    for low, high in ranges:
        # Hue stays user-configurable; relax only saturation/value for shadows.
        seed_ranges.append(((low[0], min(low[1], 95), min(low[2], 25)), tuple(high)))
        weak_ranges.append(((max(0, low[0]-4), 35, 15),
                            (min(179, high[0]+4), high[1], high[2])))
    seed = color_mask(frame, seed_ranges) | color_mask(balanced, seed_ranges)
    weak = color_mask(frame, weak_ranges) | color_mask(balanced, weak_ranges)
    glare = (hsv[:, :, 1] < 85) & (hsv[:, :, 2] > 115)
    explained = (weak > 0) | glare
    return seed, explained


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


def edge_support(circle, gradients):
    """Find observed edge points with a radial normal, in 96 angular sectors."""
    u, v, radius = circle
    angles = np.linspace(0, 2*math.pi, 96, endpoint=False)
    c, s = np.cos(angles), np.sin(angles)
    step = max(1., radius*.035)
    radii = radius+np.arange(-2, 3)[:, None]*step
    xx = np.rint(u+radii*c).astype(int)
    yy = np.rint(v+radii*s).astype(int)
    height, width = gradients[0][0].shape
    valid = (xx >= 1) & (xx < width-1) & (yy >= 1) & (yy < height-1)
    x, y = np.clip(xx, 0, width-1), np.clip(yy, 0, height-1)
    scores = np.zeros(xx.shape)
    for gx, gy in gradients:
        dx, dy = gx[y, x], gy[y, x]
        magnitude = np.hypot(dx, dy)
        alignment = np.abs(dx*c+dy*s)/(magnitude+1e-6)
        votes = np.where(valid & (alignment > .90) & (magnitude > 25), magnitude, 0.)
        scores = np.maximum(scores, votes)
    best = scores.argmax(axis=0)
    supported = scores[best, np.arange(96)] > 0
    points = np.column_stack((xx[best, np.arange(96)][supported],
                              yy[best, np.arange(96)][supported])).astype(float)
    return float(supported.mean()), points, supported


def circle_fractions(circle, seed, explained):
    """Measure evidence inside a proposed circle, including partly visible circles."""
    u, v, radius = circle
    height, width = seed.shape
    x0, y0 = max(0, int(u-radius)), max(0, int(v-radius))
    x1, y1 = min(width, math.ceil(u+radius+1)), min(height, math.ceil(v+radius+1))
    if x1 <= x0 or y1 <= y0:
        return 0., 0.
    yy, xx = np.ogrid[y0:y1, x0:x1]
    inside = (xx-u)**2+(yy-v)**2 <= (radius*.93)**2
    count = max(1, np.count_nonzero(inside))
    return (float(np.count_nonzero(seed[y0:y1, x0:x1][inside])/count),
            float(np.count_nonzero(explained[y0:y1, x0:x1][inside])/count))


def observe_ball(frame, ranges, min_area=12, min_circularity=.72,
                 shape=None, options=None, previous=None):
    """Search the field ROI and restore full-image coordinates for projection."""
    fraction = (options or {}).get('detection_top_fraction',
                                   ROBUST_DEFAULTS['detection_top_fraction'])
    if not math.isfinite(fraction) or not 0 <= fraction < 1:
        raise ValueError('detection_top_fraction must be between 0 (inclusive) and 1')
    height, width = frame.shape[:2]
    top = min(height-1, math.ceil(height*fraction))
    if min(height-top, width) < 8:
        return BallObservation(None, np.zeros((height, width), np.uint8), roi_top=top)
    prior = previous
    if prior is not None:
        prior = ((prior[0], prior[1]-top, *prior[2:])
                 if prior[1] >= top else None)
    result = _observe_ball_region(frame[top:], ranges, min_area, min_circularity,
                                  shape, options, prior)
    mask = np.zeros((height, width), np.uint8)
    mask[top:] = result.mask
    result.mask, result.roi_top = mask, top
    if result.blob is not None:
        u, v, radius, area = result.blob
        result.blob = (u, v+top, radius, area) if v >= 0 else None
    return result


def _observe_ball_region(frame, ranges, min_area=12, min_circularity=.72,
                         shape=None, options=None, previous=None):
    """Generate circle hypotheses, refit visible arcs, then return at most one."""
    cfg = {**ROBUST_DEFAULTS, **(options or {})}
    height, width = frame.shape[:2]
    scale = min(1., 480/max(height, width))
    image = cv2.resize(frame, (round(width*scale), round(height*scale))) if scale < 1 else frame
    seed, explained = illumination_evidence(image, ranges)
    result = BallObservation(None, cv2.resize(seed, (width, height)) if scale < 1 else seed)
    if np.count_nonzero(seed) < min_area*scale*scale:
        return result
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    light = cv2.createCLAHE(clipLimit=2., tileGridSize=(8, 8)).apply(lab[:, :, 0])
    rgb = image.astype(np.float32)
    saturation = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)[:, :, 1]
    red_excess = (rgb[:, :, 2]-rgb[:, :, 1])/(rgb.sum(axis=2)+1)
    chroma = np.clip(red_excess*765, 0, 255)
    planes = [cv2.GaussianBlur(light, (5, 5), 1),
              cv2.GaussianBlur(chroma.astype(np.uint8), (5, 5), 1)]
    gradients = [(cv2.Sobel(p, cv2.CV_32F, 1, 0), cv2.Sobel(p, cv2.CV_32F, 0, 1))
                 for p in planes]
    proposals = []
    compact = _compact_blob(image, ranges, min_area*scale*scale,
                            min_circularity, **(shape or {}))
    if compact is not None:
        proposals.append(compact[:3])
    fast_previous = False
    if previous is not None:
        prior = tuple(v*scale for v in previous[:3])
        coverage, _, _ = edge_support(prior, gradients)
        fraction, explanation = circle_fractions(prior, seed, explained)
        fast_previous = coverage >= .65 and fraction >= .15 and explanation >= .5
        proposals.append(prior)
    for plane in ([] if fast_previous else planes):
        x0 = y0 = 0
        crop = plane
        min_radius, max_radius = 4, max(5, min(image.shape[:2])//3)
        if previous is not None:
            u, v, r = (value*scale for value in previous[:3])
            margin = r*2.5+16
            x0, y0 = max(0, int(u-margin)), max(0, int(v-margin))
            x1 = min(plane.shape[1], math.ceil(u+margin))
            y1 = min(plane.shape[0], math.ceil(v+margin))
            crop = plane[y0:y1, x0:x1]
            min_radius, max_radius = max(3, int(r*.5)), max(5, int(r*1.7))
        for method, threshold in [(cv2.HOUGH_GRADIENT, 20), (cv2.HOUGH_GRADIENT_ALT, .65)]:
            circles = cv2.HoughCircles(
                crop, method, dp=1.5, minDist=16, param1=100, param2=threshold,
                minRadius=min_radius, maxRadius=max_radius)
            if circles is not None:
                proposals.extend((float(row[0]+x0), float(row[1]+y0), float(row[2]))
                                 for row in circles[0, :24])
    accepted = []
    for proposed in proposals:
        seed_fraction, explained_fraction = circle_fractions(proposed, seed, explained)
        if (seed_fraction < cfg['min_seed_fraction']
                or explained_fraction < cfg['min_explained_fraction']):
            continue
        coverage, points, _ = edge_support(proposed, gradients)
        if coverage < cfg['min_arc_fraction']:
            continue
        # Robust trimming removes finger/straight-edge points before circle fitting.
        fitted = proposed
        for _ in range(2):
            distances = np.linalg.norm(points-np.array(fitted[:2]), axis=1)
            inliers = points[np.abs(distances-fitted[2]) < max(1.8, fitted[2]*.055)]
            update = circle_fit(inliers)
            if update is None:
                break
            if (np.linalg.norm(np.array(update[:2])-proposed[:2]) > proposed[2]*.25
                    or not .75 < update[2]/proposed[2] < 1.25):
                break
            fitted = update
        coverage, _, supported = edge_support(fitted, gradients)
        fraction, explained_fraction = circle_fractions(fitted, seed, explained)
        if (coverage < cfg['min_arc_fraction'] or fraction < cfg['min_seed_fraction']
                or explained_fraction < cfg['min_explained_fraction']):
            continue
        # Circular texture in skin/wood must not count as an orange object edge.
        angles = np.linspace(0, 2*math.pi, 96, endpoint=False)
        samples = []
        for factor in [.8, 1.2]:
            xx = np.rint(fitted[0]+factor*fitted[2]*np.cos(angles)).astype(int)
            yy = np.rint(fitted[1]+factor*fitted[2]*np.sin(angles)).astype(int)
            valid = ((xx >= 0) & (xx < seed.shape[1])
                     & (yy >= 0) & (yy < seed.shape[0]))
            samples.append((red_excess[np.clip(yy, 0, seed.shape[0]-1),
                                       np.clip(xx, 0, seed.shape[1]-1)], valid))
        contrast = (samples[0][0]-samples[1][0] > .035)
        contrast &= samples[0][1] & samples[1][1] & supported
        if contrast.mean() < .25:
            continue
        if coverage < .75:
            angles = np.linspace(0, 2*math.pi, 96, endpoint=False)
            xx = np.rint(fitted[0]+1.3*fitted[2]*np.cos(angles)).astype(int)
            yy = np.rint(fitted[1]+1.3*fitted[2]*np.sin(angles)).astype(int)
            valid = ((xx >= 0) & (xx < seed.shape[1]) & (yy >= 0) & (yy < seed.shape[0])
                     & ~supported)
            if valid.any() and np.mean(seed[yy[valid], xx[valid]] > 0) > .6:
                continue  # a coloured shape continuing beyond the proposed circle
        yy, xx = np.ogrid[:seed.shape[0], :seed.shape[1]]
        interior = (xx-fitted[0])**2+(yy-fitted[1])**2 < (.8*fitted[2])**2
        colour_values = red_excess[interior]
        strength = float(np.percentile(colour_values, 70))
        coloured = saturation[interior & (seed > 0)]
        # Glare/occluders are excluded from this identity check. Allow uniformly
        # washed-out orange, but reject mixed weak skin/wood texture.
        if (len(coloured) == 0
                or (np.mean(coloured >= 160) < .6 and np.std(coloured) > 25)):
            continue
        vividness = float(np.clip((strength-.06)/.24, 0, 1))
        score = (.30*coverage+.15*min(1., fraction/.7)+.10*explained_fraction
                 + .15*min(1., contrast.mean()/.65)+.30*vividness)
        if score < cfg['min_detection_confidence']:
            continue
        blob = (fitted[0]/scale, fitted[1]/scale, fitted[2]/scale,
                math.pi*(fitted[2]/scale)**2*fraction)
        if blob[3] < min_area:
            continue
        continuity = 0.
        if previous is not None:
            shift = math.hypot(blob[0]-previous[0], blob[1]-previous[1])
            if shift < max(20., previous[2]*2) and .5 < blob[2]/previous[2] < 2:
                continuity = .12*(1-shift/max(20., previous[2]*2))
        accepted.append((score+continuity, score, coverage, fraction, blob))
    result.candidates = len(accepted)
    if previous is not None:
        accepted = [row for row in accepted
                    if math.hypot(row[4][0]-previous[0], row[4][1]-previous[1])
                    < max(20., previous[2]*2) and .5 < row[4][2]/previous[2] < 2]
    if accepted:
        (_, result.confidence, result.arc_fraction,
         result.color_fraction, result.blob) = max(accepted)
        result.partial = result.arc_fraction < .8 or result.color_fraction < .6
    return result


class BallImageDetector:
    """Retain one pixel target briefly, but never manufacture a current observation."""

    def __init__(self, ranges, min_area=12, min_circularity=.72, shape=None, options=None):
        self.ranges, self.min_area, self.min_circularity = ranges, min_area, min_circularity
        self.shape, self.options = shape, {**ROBUST_DEFAULTS, **(options or {})}
        for key, value in self.options.items():
            if key == 'detection_top_fraction':
                if not math.isfinite(value) or not 0 <= value < 1:
                    raise ValueError(f'Invalid robust detector option: {key}')
                continue
            if not math.isfinite(value) or value <= 0 or (key != 'visual_memory_s' and value > 1):
                raise ValueError(f'Invalid robust detector option: {key}')
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
                                   self.shape, self.options, self.previous)
        self.frame_key, self.last_time = frame_key, timestamp
        if self.result.blob is not None:
            self.previous, self.last_observed = self.result.blob, timestamp
        return self.result
