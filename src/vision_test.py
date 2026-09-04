import argparse
import csv
import os
import time

import cv2

import annotate
import config
from detection import (CORNER_FIELDS, Detection, FrameGeometry,
                       VISION_FIELDS, corner_rows, vision_row)
from flight_log import FlightLog
from marker_detector import MarkerDetector

IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp")


class DetectionSummary:
    def __init__(self):
        self.frames = 0
        self.hits = 0
        self.spans = []
        self.ranges = []
        self.corners = []
        self.mask_pixels = []
        self.cv_milliseconds = []
        self.relaxed = 0
        self.gaps = []
        self.open_gap = 0

    @classmethod
    def from_csv(cls, path):
        summary = cls()
        if not os.path.exists(path):
            return summary
        with open(path) as handle:
            for row in csv.DictReader(handle):
                summary._add_row(row)
        return summary

    def _add_row(self, row):
        self.frames += 1
        self.mask_pixels.append(float(row["mask_px"] or 0))
        self.cv_milliseconds.append(float(row["cv_ms"] or 0))
        seen = row["seen"] == "1"
        self._track_gap(seen)
        if not seen:
            return
        self.hits += 1
        self.relaxed += int(row.get("relaxed") == "1")
        self.spans.append(float(row["span_px"]))
        self.ranges.append(float(row["range_m"]))
        self.corners.append(float(row["corner_count"]))

    def _track_gap(self, seen):
        if not seen:
            self.open_gap += 1
            return
        if self.open_gap:
            self.gaps.append(self.open_gap)
        self.open_gap = 0

    def _close_gap(self):
        if self.open_gap:
            self.gaps.append(self.open_gap)
            self.open_gap = 0

    def add(self, cluster, detection, mask_pixels, cv_seconds):
        self.frames += 1
        self.mask_pixels.append(mask_pixels)
        self.cv_milliseconds.append(cv_seconds * 1000.0)
        self._track_gap(cluster is not None)
        if cluster is None:
            return
        self.hits += 1
        self.relaxed += int(cluster.relaxed)
        self.spans.append(cluster.span_px)
        self.ranges.append(detection.range_m)
        self.corners.append(cluster.corner_count)

    def report(self, log):
        if self.frames == 0:
            log.event("summary", "no frames processed")
            return
        self._close_gap()
        rate = 100.0 * self.hits / self.frames
        log.event("summary", f"{self.hits} of {self.frames} frames detected "
                             f"a target ({rate:.1f} percent)")
        for name, values, unit in (("cv time", self.cv_milliseconds, "ms"),
                                   ("mask", self.mask_pixels, "px"),
                                   ("span", self.spans, "px"),
                                   ("range", self.ranges, "m"),
                                   ("corners", self.corners, "")):
            log.event("summary", f"{name:8} {_spread(values, unit)}")
        log.event("summary", f"relaxed  {self.relaxed} frames held the target "
                             f"only through the fallback pass")
        log.event("summary", f"dropouts {self._dropouts()}")
        for line in self._verdicts(rate):
            log.event("summary", line)

    def longest_gap_s(self):
        return max(self.gaps, default=0) / float(config.FRAME_RATE)

    def _dropouts(self):
        if not self.gaps:
            return "none, the target was held on every frame"
        longest = max(self.gaps)
        return (f"{len(self.gaps)} {_plural('run', len(self.gaps))} of missed "
                f"frames, longest {longest} "
                f"{_plural('frame', longest)} "
                f"({longest / float(config.FRAME_RATE):.2f} s), "
                f"median {_median(self.gaps)}")

    def _verdicts(self, rate):
        lines = []
        if rate < 50.0:
            lines.append("detection is unreliable, widen the colour window "
                         "or check lighting before flying")
        if self.corners and _median(self.corners) < config.TRUSTED_CORNER_COUNT:
            lines.append(f"the median cluster holds fewer than "
                         f"{config.TRUSTED_CORNER_COUNT} corners, so "
                         f"visibility will stay below 1.0 in flight")
        if self.longest_gap_s() >= config.DETECTION_HOLD_S:
            lines.append(f"a {self.longest_gap_s():.2f} s dropout reaches "
                         f"DETECTION_HOLD_S ({config.DETECTION_HOLD_S:.2f} s), "
                         f"so the drone would have fallen all the way back "
                         f"to searching")
        return lines or ["detection looks healthy"]


def _spread(values, unit):
    if not values:
        return "no samples"
    ordered = sorted(values)
    return (f"min {ordered[0]:.1f}{unit} "
            f"median {ordered[len(ordered) // 2]:.1f}{unit} "
            f"max {ordered[-1]:.1f}{unit}")


def _median(values):
    return sorted(values)[len(values) // 2]


def _plural(word, count):
    return word if count == 1 else f"{word}s"


class Snapshots:
    def __init__(self, log, every):
        self.every = every
        self.directory = os.path.join(log.directory, "snaps")
        self.saved = 0
        if every > 0:
            os.makedirs(self.directory, exist_ok=True)

    def maybe_save(self, frame, cluster, detection, frame_index, mask_pixels,
                   corners=()):
        wanted = frame_index == 1 or frame_index % self.every == 0
        if (self.every <= 0 or not wanted
                or self.saved >= config.SNAPSHOT_LIMIT):
            return
        image = annotate.draw_cluster(frame.copy(), cluster, detection,
                                      frame_index, mask_pixels, corners)
        cv2.imwrite(os.path.join(self.directory, f"{frame_index:05d}.jpg"),
                    image, [cv2.IMWRITE_JPEG_QUALITY, 85])
        self.saved += 1


def run_file(log, source, snapshot_every):
    detector = MarkerDetector()
    geometry = FrameGeometry()
    summary = DetectionSummary()
    snapshots = Snapshots(log, snapshot_every)
    rows = log.csv("vision", VISION_FIELDS)
    corner_rows_csv = log.csv("corners", CORNER_FIELDS)

    log.event("check", f"replaying {source}")
    for frame_index, frame in enumerate(_frames(source), start=1):
        frame = cv2.resize(frame, config.FRAME_SIZE)
        started = time.monotonic()
        cluster = detector.find(frame)
        cv_seconds = time.monotonic() - started
        detection = (geometry.detection_from(cluster, started) if cluster
                     else Detection())

        rows.write(**vision_row(frame_index, cluster, detection,
                                detector.mask_pixels, started, cv_seconds,
                                config.FRAME_RATE))
        for corner in corner_rows(frame_index, detector.corners):
            corner_rows_csv.write(**corner)
        summary.add(cluster, detection, detector.mask_pixels, cv_seconds)
        snapshots.maybe_save(frame, cluster, detection, frame_index,
                             detector.mask_pixels, detector.corners)
        log.event("frame", annotate.caption(frame_index, cluster, detection) +
                  f"  mask {detector.mask_pixels} px  "
                  f"cv {cv_seconds * 1000.0:.1f} ms")

    summary.report(log)
    log.event("check", f"{snapshots.saved} snapshots in snaps/")


def _frames(source):
    if source.lower().endswith(IMAGE_SUFFIXES):
        image = cv2.imread(source)
        if image is None:
            raise SystemExit(f"cannot read {source}")
        yield image
        return

    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        raise SystemExit(f"cannot open {source}")
    while True:
        ok, frame = capture.read()
        if not ok:
            capture.release()
            return
        yield frame


def run_live(log, seconds):
    from camera_controller import CameraController

    camera = CameraController(log)
    try:
        camera.start()
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            now = time.monotonic()
            detection = camera.latest()
            log.event("check",
                      f"age {min(detection.age(now), 999.0):5.2f} s "
                      f"range {detection.range_m:5.2f} m "
                      f"bearing {detection.bearing_deg:+6.1f} deg "
                      f"span {detection.span_px:5.1f} px "
                      f"corners {detection.corner_count} "
                      f"mask {camera.detector.mask_pixels} px "
                      f"cam {camera.fps:.1f} fps")
            time.sleep(config.STATUS_INTERVAL_S)
    except KeyboardInterrupt:
        log.event("check", "interrupted by operator")
    finally:
        camera.stop()
        summary = DetectionSummary.from_csv(
            os.path.join(log.directory, "vision.csv"))
        summary.report(log)
        annotate.build(log, camera.convert_video(),
                       camera.recording_started_at)


def main():
    parser = argparse.ArgumentParser(
        description="Test the marker detection without flying the drone.")
    parser.add_argument("--source", metavar="FILE",
                        help="image or video to replay instead of the camera")
    parser.add_argument("--seconds", type=float, default=30.0,
                        help="how long to run the live camera")
    parser.add_argument("--snapshot-every", type=int, default=10,
                        metavar="N",
                        help="save an annotated frame every N frames "
                             "when replaying a file, 0 to disable")
    arguments = parser.parse_args()

    log = FlightLog(root=config.LOG_ROOT, tag="vision")
    log.event("check", f"session {log.directory}")
    log.event("check", f"marker {config.MARKER_HEX} hue "
                       f"{config.MARKER_HUE_WRAP_LOW}-"
                       f"{config.MARKER_HUE_WRAP_HIGH} "
                       f"saturation min {config.MARKER_SATURATION_MIN} "
                       f"value min {config.MARKER_VALUE_MIN}")
    try:
        if arguments.source:
            run_file(log, arguments.source, arguments.snapshot_every)
        else:
            run_live(log, arguments.seconds)
    finally:
        log.event("check", f"session saved to {log.directory}")
        log.event("check", f"copy it with: {log.download_command()}")
        log.close()


if __name__ == "__main__":
    main()
