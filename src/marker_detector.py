import math
from dataclasses import dataclass

import cv2
import numpy as np

import config

FAR_AWAY = 10 ** 9


@dataclass
class Corner:
    x: float
    y: float
    area: int
    radius: float
    box: tuple
    in_cluster: bool = False


@dataclass
class Cluster:
    x: float
    y: float
    span_px: float
    corner_count: int
    corners_seen: int
    box: tuple
    relaxed: bool = False


class MarkerDetector:
    def __init__(self):
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (config.OPEN_KERNEL_PX, config.OPEN_KERNEL_PX))
        self.mask_pixels = 0
        self.corners = []
        self.last = None
        self.frames_since_seen = FAR_AWAY

    def find(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blobs, centroids, seeded = self._scan(hsv)

        cluster = self._search(blobs, centroids, seeded)
        if cluster is None and self.frames_since_seen <= config.FALLBACK_FRAMES:
            loose = np.zeros(len(seeded), dtype=bool)
            loose[1:] = True
            cluster = self._search(blobs, centroids, loose)
            if cluster is not None:
                cluster.relaxed = True

        self._remember(cluster)
        return cluster

    def _scan(self, hsv):
        loose = self._window(hsv, config.MARKER_SATURATION_MIN,
                             config.MARKER_VALUE_MIN)
        loose = cv2.morphologyEx(loose, cv2.MORPH_OPEN, self.kernel)
        count, labels, blobs, centroids = cv2.connectedComponentsWithStats(
            loose, 8)

        seeded = np.zeros(count, dtype=bool)
        if count > 1:
            core = self._window(hsv, config.MARKER_CORE_SATURATION,
                                config.MARKER_CORE_VALUE)
            seeded[labels[core > 0]] = True
            seeded[0] = False

        self.mask_pixels = int(blobs[seeded, cv2.CC_STAT_AREA].sum())
        return blobs, centroids, seeded

    def _window(self, hsv, saturation, value):
        above = cv2.inRange(
            hsv,
            np.array((config.MARKER_HUE_WRAP_LOW, saturation, value),
                     dtype=np.uint8),
            np.array((179, 255, 255), dtype=np.uint8))
        below = cv2.inRange(
            hsv,
            np.array((0, saturation, value), dtype=np.uint8),
            np.array((config.MARKER_HUE_WRAP_HIGH, 255, 255), dtype=np.uint8))
        return cv2.bitwise_or(above, below)

    def _search(self, blobs, centroids, chosen):
        self.corners = self._corners(blobs, centroids, chosen)
        if len(self.corners) < config.MIN_CLUSTER_CORNERS:
            return None

        points = np.array([(c.x, c.y) for c in self.corners], dtype=np.float32)
        radii = np.array([c.radius for c in self.corners], dtype=np.float32)
        distances = _pairwise(points)
        groups = _groups(_adjacency(radii, distances))

        members = self._best_group(groups)
        if members is None:
            return None
        for index in members:
            self.corners[index].in_cluster = True
        return self._cluster_from(members, distances)

    def _corners(self, blobs, centroids, chosen):
        index = np.nonzero(chosen)[0]
        if index.size == 0:
            return []

        areas = blobs[index, cv2.CC_STAT_AREA].astype(np.float32)
        widths = np.maximum(blobs[index, cv2.CC_STAT_WIDTH], 1)
        heights = np.maximum(blobs[index, cv2.CC_STAT_HEIGHT], 1)
        fill = areas / (widths * heights)
        aspect = np.maximum(widths, heights) / np.minimum(widths, heights)

        testable = areas >= config.SHAPE_TEST_MIN_AREA_PX
        shaped = ~testable | ((fill >= config.MIN_CORNER_FILL) &
                              (aspect <= config.MAX_CORNER_ASPECT))
        keep = ((areas >= config.MIN_CORNER_AREA_PX) &
                (areas <= config.MAX_CORNER_AREA_PX) & shaped)

        kept = index[keep]
        kept = kept[np.argsort(-blobs[kept, cv2.CC_STAT_AREA])]
        kept = kept[:config.MAX_CORNERS]
        return _drop_undersized([
            Corner(x=float(centroids[i][0]), y=float(centroids[i][1]),
                   area=int(blobs[i, cv2.CC_STAT_AREA]),
                   radius=math.sqrt(blobs[i, cv2.CC_STAT_AREA] / math.pi),
                   box=tuple(int(v) for v in blobs[i, :4]))
            for i in kept])

    def _best_group(self, groups):
        usable = [g for g in groups if len(g) >= config.MIN_CLUSTER_CORNERS]
        return max(usable, key=self._rank, default=None)

    def _rank(self, members):
        area = sum(self.corners[i].area for i in members)
        return area * self._continuity(members)

    def _continuity(self, members):
        if (self.last is None or
                self.frames_since_seen > config.CONTINUITY_HOLD_FRAMES):
            return 1.0
        last_x, last_y, last_span = self.last
        x = sum(self.corners[i].x for i in members) / len(members)
        y = sum(self.corners[i].y for i in members) / len(members)
        reach = config.CONTINUITY_REACH * max(last_span, 20.0)
        near = math.hypot(x - last_x, y - last_y) <= reach
        return config.CONTINUITY_BONUS if near else 1.0

    def _cluster_from(self, members, distances):
        chosen = [self.corners[i] for i in members]
        weights = sum(c.area for c in chosen)
        index = np.array(members)
        return Cluster(
            x=sum(c.x * c.area for c in chosen) / weights,
            y=sum(c.y * c.area for c in chosen) / weights,
            span_px=float(distances[np.ix_(index, index)].max()),
            corner_count=len(chosen),
            corners_seen=len(self.corners),
            box=_bounding_box(chosen))

    def _remember(self, cluster):
        if cluster is None:
            self.frames_since_seen += 1
            if self.frames_since_seen > config.CONTINUITY_HOLD_FRAMES:
                self.last = None
            return
        self.last = (cluster.x, cluster.y, cluster.span_px)
        self.frames_since_seen = 0


def _drop_undersized(corners):
    if config.MIN_AREA_FRACTION <= 0 or len(corners) < 3:
        return corners
    areas = sorted(c.area for c in corners)
    floor = areas[len(areas) // 2] * config.MIN_AREA_FRACTION
    kept = [c for c in corners if c.area >= floor]
    return kept if len(kept) >= config.MIN_CLUSTER_CORNERS else corners


def _bounding_box(corners):
    left = min(c.box[0] for c in corners)
    top = min(c.box[1] for c in corners)
    right = max(c.box[0] + c.box[2] for c in corners)
    bottom = max(c.box[1] + c.box[3] for c in corners)
    return (left, top, right - left, bottom - top)


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
