import math
from dataclasses import dataclass

import config

NEVER_SEEN = -1e9

VISION_FIELDS = ["frame", "t_mono", "cv_ms", "fps", "seen", "centre_x",
                 "centre_y", "bearing_deg", "elevation_deg", "span_px",
                 "range_m", "corner_count", "corners_seen", "mask_px",
                 "box_x", "box_y", "box_w", "box_h",
                 "exposure_us", "gain", "lux"]


@dataclass
class Detection:
    stamp: float = NEVER_SEEN
    bearing_deg: float = 0.0
    elevation_deg: float = 0.0
    range_m: float = config.MAX_TRACK_RANGE_M
    span_px: float = 0.0
    corner_count: int = 0

    def age(self, now):
        return now - self.stamp


class FrameGeometry:
    def __init__(self, frame_size=config.FRAME_SIZE):
        self.width, self.height = frame_size
        self.focal_px = ((self.width / 2.0) /
                         math.tan(math.radians(config.HORIZONTAL_FOV_DEG / 2.0)))
        self.range_constant = self.focal_px * config.CAGE_WIDTH_M
        self.half_width_tangent = math.tan(
            math.radians(config.HORIZONTAL_FOV_DEG / 2.0))
        self.half_height_tangent = math.tan(
            math.radians(config.VERTICAL_FOV_DEG / 2.0))

    def bearing_deg(self, x):
        return self._angle(x, self.width, self.half_width_tangent)

    def elevation_deg(self, y):
        return self._angle(y, self.height, self.half_height_tangent)

    def range_m(self, span_px):
        return min(config.MAX_TRACK_RANGE_M,
                   self.range_constant / max(span_px, 1e-6))

    def detection_from(self, cluster, stamp):
        return Detection(
            stamp=stamp,
            bearing_deg=self.bearing_deg(cluster.x),
            elevation_deg=self.elevation_deg(cluster.y),
            range_m=self.range_m(cluster.span_px),
            span_px=cluster.span_px,
            corner_count=cluster.corner_count)

    def _angle(self, pixel, extent, half_tangent):
        offset = (pixel - extent / 2.0) / (extent / 2.0)
        return math.degrees(math.atan(offset * half_tangent))


def vision_row(frame_index, cluster, detection, mask_pixels, t_mono,
               cv_seconds, fps, metadata=None):
    metadata = metadata or {}
    seen = cluster is not None
    return {
        "frame": frame_index,
        "t_mono": round(t_mono, 4),
        "cv_ms": round(cv_seconds * 1000.0, 1),
        "fps": round(fps, 2),
        "seen": int(seen),
        "centre_x": round(cluster.x, 1) if seen else "",
        "centre_y": round(cluster.y, 1) if seen else "",
        "bearing_deg": round(detection.bearing_deg, 2) if seen else "",
        "elevation_deg": round(detection.elevation_deg, 2) if seen else "",
        "span_px": round(cluster.span_px, 1) if seen else "",
        "range_m": round(detection.range_m, 2) if seen else "",
        "corner_count": cluster.corner_count if seen else 0,
        "corners_seen": cluster.corners_seen if seen else 0,
        "mask_px": int(mask_pixels),
        "box_x": cluster.box[0] if seen else "",
        "box_y": cluster.box[1] if seen else "",
        "box_w": cluster.box[2] if seen else "",
        "box_h": cluster.box[3] if seen else "",
        "exposure_us": metadata.get("ExposureTime", ""),
        "gain": round(metadata.get("AnalogueGain", 0.0), 2),
        "lux": round(metadata.get("Lux", 0.0), 1),
    }
