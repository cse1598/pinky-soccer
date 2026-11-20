"""Ball search grid: where could the ball be, given where the camera has looked.

Each cell holds a presence belief in [0, 1]. Cells inside the camera field of view
without a detection are lowered; all cells relax back towards the prior over time,
so areas not seen for a while become interesting again. A detection sets a peak.
Same idea as field-coverage grids in RoboCup teams (refs/BHumanCodeRelease
Modeling/FieldCoverage); no code copied. Occlusion by opponents is TODO.
"""

import math

import numpy as np


class SearchGrid:
    """Belief grid over a pinky_soccer_field Rect (row 0 = lowest y)."""

    def __init__(self, rect, resolution=0.05, prior=0.5, relax_tau=8.0,
                 miss_factor=0.3, hit_sigma=0.1):
        self.rect = rect
        self.res = resolution
        self.nx = max(1, math.ceil(rect.width / resolution))
        self.ny = max(1, math.ceil(rect.height / resolution))
        self.origin = (rect.xmin, rect.ymin)
        xs = rect.xmin + (np.arange(self.nx) + 0.5) * resolution
        ys = rect.ymin + (np.arange(self.ny) + 0.5) * resolution
        self.cx, self.cy = np.meshgrid(xs, ys)
        self.prior = prior
        self.relax_tau = relax_tau        # [s] time to forget a "not here" observation
        self.miss_factor = miss_factor    # belief multiplier per second of looking, no ball
        self.hit_sigma = hit_sigma        # [m] spread of a detection peak
        self.p = np.full((self.ny, self.nx), prior)

    def relax(self, dt):
        """Move every cell back towards the prior."""
        if dt <= 0:
            return
        a = 1.0 - math.exp(-dt / self.relax_tau)
        self.p += (self.prior - self.p) * a

    def fov_mask(self, x, y, heading, hfov, min_range, max_range):
        """Return a boolean mask of cells the camera can see from pose (x, y, heading)."""
        dx, dy = self.cx - x, self.cy - y
        dist = np.hypot(dx, dy)
        bearing = np.arctan2(dy, dx) - heading
        bearing = np.arctan2(np.sin(bearing), np.cos(bearing))
        return (np.abs(bearing) <= hfov / 2) & (dist >= min_range) & (dist <= max_range)

    def observe(self, mask, dt, ball_xy=None):
        """Apply one camera cycle: lower seen cells, raise around a detection."""
        if dt > 0 and mask.any():
            self.p[mask] *= self.miss_factor ** dt
        if ball_xy is not None:
            d2 = (self.cx - ball_xy[0]) ** 2 + (self.cy - ball_xy[1]) ** 2
            peak = np.exp(-0.5 * d2 / self.hit_sigma ** 2)
            self.p = np.maximum(self.p * (1.0 - peak), peak)
        np.clip(self.p, 0.0, 1.0, out=self.p)

    def as_int8(self):
        """Return belief scaled to 0..100 (OccupancyGrid convention)."""
        return np.rint(self.p * 100).astype(np.int8)

    def best_cell(self):
        """Return the (x, y) centre of the most likely cell."""
        j, i = np.unravel_index(int(np.argmax(self.p)), self.p.shape)
        return float(self.cx[j, i]), float(self.cy[j, i])
