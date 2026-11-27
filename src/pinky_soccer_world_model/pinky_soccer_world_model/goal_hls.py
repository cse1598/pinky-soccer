"""Initialization-only pixel counts; no shape operations or production ball changes."""
import cv2
import numpy as np


class HlsGoalDetector:
    def __init__(self, yellow_low=(24, 85, 180), yellow_high=(35, 180, 255),
                 blue_low=(96, 30, 135), blue_high=(115, 130, 255),
                 minimum=500, ratio=3., confirmations=3, crop_top=130):
        self.bounds = {'yellow': (yellow_low, yellow_high), 'blue': (blue_low, blue_high)}
        for low, high in self.bounds.values():
            if (len(low) != 3 or len(high) != 3 or
                    any(not 0 <= a <= b <= limit for a, b, limit in zip(low, high, (180, 255, 255)))):
                raise ValueError('Invalid OpenCV HLS bounds')
        if minimum < 1 or ratio <= 1 or confirmations < 1 or crop_top < 0:
            raise ValueError('Invalid HLS decision parameters')
        self.minimum, self.ratio = int(minimum), float(ratio)
        self.confirmations, self.crop_top = int(confirmations), int(crop_top)
        self.reset()

    def reset(self):
        self.previous, self.streak = None, 0

    def detect(self, frame, debug=False):
        if frame.ndim != 3 or frame.shape[2] != 3 or frame.shape[0] <= self.crop_top:
            self.reset()
            return {'color': None, 'counts': {'yellow': 0, 'blue': 0}, 'streak': 0}
        # Exactly one conversion per eligible frame, restricted before processing.
        hls = cv2.cvtColor(frame[self.crop_top:], cv2.COLOR_BGR2HLS)
        masks = {c: cv2.inRange(hls, np.asarray(lo, np.uint8), np.asarray(hi, np.uint8))
                 for c, (lo, hi) in self.bounds.items()}
        counts = {c: cv2.countNonZero(m) for c, m in masks.items()}
        winner = max(counts, key=counts.get)
        loser = 'blue' if winner == 'yellow' else 'yellow'
        if counts[winner] < self.minimum or counts[winner] < self.ratio*max(1, counts[loser]):
            winner = None
        self.streak = self.streak+1 if winner and winner == self.previous else int(winner is not None)
        self.previous = winner
        result = {'color': winner if self.streak >= self.confirmations else None,
                  'counts': counts, 'streak': self.streak}
        if debug:
            result['masks'] = masks
        return result
