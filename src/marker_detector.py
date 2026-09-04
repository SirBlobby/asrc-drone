from dataclasses import dataclass

import cv2
import numpy as np

import config


@dataclass
class Cluster:
    x: float
    y: float
    span_px: float
    corner_count: int
    corners_seen: int
    box: tuple


class MarkerDetector:
    def __init__(self):
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (config.OPEN_KERNEL_PX, config.OPEN_KERNEL_PX))
        self.mask = None
        self.mask_pixels = 0

    def find(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.mask = self._marker_mask(hsv)
        points, areas, radii, boxes = self._corners(self.mask)
        return self._densest_cluster(points, areas, radii, boxes)

    def _marker_mask(self, hsv):
        low_saturation = config.MARKER_SATURATION_MIN
        low_value = config.MARKER_VALUE_MIN
        upper_red = cv2.inRange(
            hsv,
            np.array((config.MARKER_HUE_WRAP_LOW, low_saturation, low_value),
                     dtype=np.uint8),
            np.array((179, 255, 255), dtype=np.uint8))
        lower_red = cv2.inRange(
            hsv,
            np.array((0, low_saturation, low_value), dtype=np.uint8),
            np.array((config.MARKER_HUE_WRAP_HIGH, 255, 255), dtype=np.uint8))
        mask = cv2.bitwise_or(upper_red, lower_red)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

    def _corners(self, mask):
        count, _, blobs, centroids = cv2.connectedComponentsWithStats(mask, 8)
        blobs = blobs[1:]
        centroids = centroids[1:]

        areas = blobs[:, cv2.CC_STAT_AREA].astype(np.float32)
        self.mask_pixels = int(areas.sum())
        widths = np.maximum(blobs[:, cv2.CC_STAT_WIDTH], 1)
        heights = np.maximum(blobs[:, cv2.CC_STAT_HEIGHT], 1)
        fill = areas / (widths * heights)
        aspect = np.maximum(widths, heights) / np.minimum(widths, heights)

        keep = ((areas >= config.MIN_CORNER_AREA_PX) &
                (areas <= config.MAX_CORNER_AREA_PX) &
                (fill >= config.MIN_CORNER_FILL) &
                (aspect <= config.MAX_CORNER_ASPECT))
        kept = np.nonzero(keep)[0]
        kept = kept[np.argsort(-areas[kept])][:config.MAX_CORNERS]

        points = centroids[kept].astype(np.float32)
        areas = areas[kept]
        radii = np.sqrt(areas / np.pi).astype(np.float32)
        boxes = blobs[kept][:, :4]
        return points, areas, radii, boxes

    def _densest_cluster(self, points, areas, radii, boxes):
        if len(points) < config.MIN_CLUSTER_CORNERS:
            return None
        distances = _pairwise(points)
        adjacency = _adjacency(radii, distances)
        members = _best_group(_groups(adjacency), areas)
        return _cluster_from(members, points, areas, distances, boxes,
                             len(points))


def _pairwise(points):
    delta = points[:, None, :] - points[None, :, :]
    return np.sqrt((delta * delta).sum(axis=2))


def _link_reach(distances):
    total = len(distances)
    spaced = distances + np.eye(total, dtype=np.float32) * 1e6
    nearest = float(np.median(spaced.min(axis=1)))
    return max(config.CLUSTER_LINK_MIN_PX,
               config.CLUSTER_LINK_SCALE * nearest)


def _adjacency(radii, distances):
    smaller = np.minimum(radii[:, None], radii[None, :])
    larger = np.maximum(radii[:, None], radii[None, :])
    comparable = larger <= config.MAX_CORNER_RADIUS_RATIO * np.maximum(
        smaller, 1e-6)
    return (distances <= _link_reach(distances)) & comparable


def _groups(adjacency):
    unvisited = set(range(len(adjacency)))
    groups = []
    while unvisited:
        stack = [unvisited.pop()]
        members = []
        while stack:
            node = stack.pop()
            members.append(node)
            neighbours = set(np.nonzero(adjacency[node])[0].tolist()) & unvisited
            unvisited -= neighbours
            stack.extend(neighbours)
        groups.append(members)
    return groups


def _best_group(groups, areas):
    usable = [g for g in groups if len(g) >= config.MIN_CLUSTER_CORNERS]
    return max(usable, key=lambda g: float(areas[g].sum()), default=None)


def _cluster_from(members, points, areas, distances, boxes, corners_seen):
    if members is None:
        return None
    index = np.array(members)
    weights = areas[index]
    centre = (points[index] * weights[:, None]).sum(axis=0) / weights.sum()
    left = int(boxes[index][:, 0].min())
    top = int(boxes[index][:, 1].min())
    right = int((boxes[index][:, 0] + boxes[index][:, 2]).max())
    bottom = int((boxes[index][:, 1] + boxes[index][:, 3]).max())
    return Cluster(
        x=float(centre[0]), y=float(centre[1]),
        span_px=float(distances[np.ix_(index, index)].max()),
        corner_count=len(members), corners_seen=corners_seen,
        box=(left, top, right - left, bottom - top))
