"""Bounded mask-contour arc consensus; no Hough, image gradients or white balance."""
import math

import cv2
import numpy as np

HLS_RANGES = [((3, 15, 100), (19, 220, 255)), ((3, 65, 60), (19, 160, 99))]


def triple_circle(points):
    """Analytic proposal only; accepted arcs are refitted by the existing LS code."""
    a, b, c = points
    x, y = b-a, c-a
    determinant = 2*(x[0]*y[1]-x[1]*y[0])
    if abs(determinant) < 2.:
        return None
    xx, yy = float(x@x), float(y@y)
    center = a+np.array([xx*y[1]-yy*x[1], x[0]*yy-y[0]*xx])/determinant
    return (*center, float(np.linalg.norm(center-a)))


def region(frame, ranges, min_area, shape, cfg, previous):
    from .robust_ball import BallObservation, circle_fit

    h, w = frame.shape[:2]
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    mask = np.zeros((h, w), np.uint8)
    for lo, hi in ranges:
        mask |= cv2.inRange(hls, np.asarray(lo, np.uint8), np.asarray(hi, np.uint8))
    result = BallObservation(None, mask)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    accepted, directional = [], []
    rng = np.random.default_rng(17)
    for contour in sorted(contours, key=cv2.contourArea, reverse=True)[:8]:
        if cv2.contourArea(contour) < min_area or len(contour) < 16:
            continue
        raw = contour[:, 0].astype(float)
        # Image/ROI cut edges are not observations of the spherical outline.
        tangent = np.roll(raw, -3, axis=0)-np.roll(raw, 3, axis=0)
        tangent /= np.maximum(1e-9, np.linalg.norm(tangent, axis=1))[:, None]
        keep = ((raw[:, 0] > 1) & (raw[:, 0] < w-2) &
                (raw[:, 1] > 1) & (raw[:, 1] < h-2))
        points, tangents = raw[keep], tangent[keep]
        if len(points) < 12:
            continue
        stride = max(1, math.ceil(len(points)/240))
        points, tangents = points[::stride], tangents[::stride]
        initial_fit = circle_fit(points)

        def proposals():
            if initial_fit is not None:
                yield initial_fit
            if previous is not None:
                yield previous[:3]
            for _ in range(48):
                item = triple_circle(points[rng.choice(len(points), 3, replace=False)])
                if item is not None:
                    yield item

        for proposal in proposals():
            u, v, radius = proposal
            if not 3 <= radius <= 2*max(h, w) or not (-radius < u < w+radius and -radius < v < h+radius):
                continue
            fitted = proposal
            for _ in range(2):
                delta = points-np.asarray(fitted[:2])
                distance = np.linalg.norm(delta, axis=1)
                radial = delta/np.maximum(1e-9, distance)[:, None]
                inliers = ((np.abs(distance-fitted[2]) < max(1.3, .04*fitted[2])) &
                           (np.abs(np.sum(radial*tangents, axis=1)) < .25))
                update = circle_fit(points[inliers])
                if update is None:
                    break
                fitted = update
            u, v, radius = fitted
            if not 3 <= radius <= 2*max(h, w):
                continue
            delta = points-np.array([u, v])
            distance = np.linalg.norm(delta, axis=1)
            radial = delta/np.maximum(1e-9, distance)[:, None]
            inliers = ((np.abs(distance-radius) < max(1.3, .04*radius)) &
                       (np.abs(np.sum(radial*tangents, axis=1)) < .25))
            if inliers.sum() < 10:
                continue
            angles = np.arctan2(delta[inliers, 1], delta[inliers, 0])
            sectors = np.unique(np.floor((angles+math.pi)*36/(2*math.pi)).astype(int) % 36)
            coverage = len(sectors)/36
            if coverage < cfg['min_bearing_arc_fraction']:
                continue
            bits = np.zeros(36, bool)
            bits[sectors] = True
            # Small raster circles can skip one 10-degree bin. Bridge only
            # isolated gaps for continuity, retaining the raw measured coverage.
            continuity_bits = bits | (np.roll(bits, 1) & np.roll(bits, -1))
            longest = run = 0
            for bit in np.r_[continuity_bits, continuity_bits]:
                run = run+1 if bit else 0
                longest = max(longest, min(36, run))
            contiguous = longest/36
            if contiguous < cfg['min_bearing_arc_fraction']:
                continue
            error = float(np.sqrt(np.mean((distance[inliers]-radius)**2)))
            x0, x1 = max(0, math.floor(u-radius)), min(w, math.ceil(u+radius+1))
            y0, y1 = max(0, math.floor(v-radius)), min(h, math.ceil(v+radius+1))
            if x1 <= x0 or y1 <= y0:
                continue
            yy, xx = np.ogrid[y0:y1, x0:x1]
            inside = (xx-u)**2+(yy-v)**2 < (.9*radius)**2
            fill = float(np.mean(mask[y0:y1, x0:x1][inside] > 0)) if inside.any() else 0.
            if fill < max(.12, shape.get('min_color_fraction', .55)*coverage):
                continue
            j = np.c_[radial[inliers], np.ones(inliers.sum())]
            covariance = np.linalg.pinv(j.T@j)*max(.5, error)**2
            uncertainty = float(math.sqrt(max(0., covariance[2, 2])))
            score = (.45*coverage+.25*min(1., fill/max(.1, coverage))+
                     .2*float(inliers.mean())+.1*max(0., 1-error/max(1., radius*.05)))
            observed = points[inliers].mean(axis=0)
            directional.append((score, tuple(observed), coverage, error, uncertainty))
            if (coverage < cfg['min_arc_fraction'] or contiguous < cfg['min_contiguous_arc_fraction']
                    or uncertainty/radius > cfg['max_radius_uncertainty']
                    or error/radius > .05 or score < cfg['min_detection_confidence']):
                continue
            blob = (float(u), float(v), float(radius), math.pi*radius*radius*fill)
            if blob[3] < min_area:
                continue
            if previous is not None and (math.hypot(u-previous[0], v-previous[1]) >= max(20., 2*previous[2])
                                         or not .5 < radius/previous[2] < 2):
                continue
            continuity = .12 if previous is not None else 0.
            accepted.append((score+continuity, score, coverage, fill, blob, error, uncertainty))
            if inliers.mean() > .9 and error < max(.5, .02*radius):
                break  # well-explained clipped arcs also need no random recovery
    result.candidates = len(accepted)
    if accepted:
        _, result.confidence, result.arc_fraction, result.color_fraction, result.blob, \
            result.fit_error_px, result.radius_uncertainty_px = max(accepted)
        result.partial = result.arc_fraction < .8 or result.color_fraction < .6
        result.bearing_uv = result.blob[:2]
        result.bearing_valid = result.distance_valid = True
    elif directional:
        result.confidence, result.bearing_uv, result.arc_fraction, result.fit_error_px, \
            result.radius_uncertainty_px = max(directional)
        result.partial = result.bearing_valid = True
    return result
