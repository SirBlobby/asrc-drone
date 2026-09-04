import threading
import time

import config
from detection import Detection, FrameGeometry, VISION_FIELDS, vision_row
from marker_detector import MarkerDetector
from video_recorder import VideoRecorder


class CameraController:
    def __init__(self, log):
        self.log = log
        self.detector = MarkerDetector()
        self.geometry = FrameGeometry()

        self.camera = None
        self.recorder = None
        self.recording_started_at = 0.0
        self.thread = None
        self.running = False
        self.lock = threading.Lock()
        self.detection = Detection()
        self.frames = 0
        self.detected_frames = 0
        self.fps = 0.0
        self.had_target = False
        self.csv = None

    def start(self):
        from picamera2 import Picamera2

        self.csv = self.log.csv("vision", VISION_FIELDS)
        frame_duration = int(1e6 / config.FRAME_RATE)
        streams = {"main": {"size": config.FRAME_SIZE, "format": "RGB888"}}
        if config.RECORD_VIDEO:
            streams["lores"] = {"size": config.FRAME_SIZE, "format": "YUV420"}

        self.camera = Picamera2()
        self.camera.configure(self.camera.create_preview_configuration(
            **streams,
            controls={"FrameDurationLimits": (frame_duration,
                                              frame_duration)}))
        self.camera.start()
        time.sleep(1.0)

        self.log.event("camera", f"{self.geometry.width}x"
                                 f"{self.geometry.height} at "
                                 f"{config.FRAME_RATE} fps, focal "
                                 f"{self.geometry.focal_px:.0f} px, marker "
                                 f"{config.MARKER_HEX} hue "
                                 f"{config.MARKER_HUE_WRAP_LOW}-"
                                 f"{config.MARKER_HUE_WRAP_HIGH}")

        if config.RECORD_VIDEO:
            self.recorder = VideoRecorder(self.camera, self.log)
            self.recorder.start()
            self.recording_started_at = time.monotonic()

        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
            self.thread = None
        if self.recorder:
            self.recorder.stop()
        if self.camera:
            self.camera.stop()
            self.camera = None
        if self.csv:
            self.csv.flush()
        self.log.event("camera", f"{self.detected_frames} of {self.frames} "
                                 f"frames held a target")

    def convert_video(self):
        return self.recorder.convert() if self.recorder else None

    def latest(self):
        with self.lock:
            return self.detection

    def _loop(self):
        previous = time.monotonic()
        while self.running:
            try:
                frame, metadata = self._grab()
            except Exception as error:
                self.log.event("camera", f"capture failed: {error}")
                time.sleep(0.1)
                continue

            now = time.monotonic()
            cluster = self.detector.find(frame)
            cv_seconds = time.monotonic() - now
            self._publish(cluster, now)

            elapsed = now - previous
            previous = now
            self.fps = 0.9 * self.fps + 0.1 / max(elapsed, 1e-3)
            self.frames += 1
            self.detected_frames += int(cluster is not None)
            self._announce(cluster)
            self.csv.write(**vision_row(
                self.frames, cluster, self.latest(),
                self.detector.mask_pixels, now, cv_seconds, self.fps,
                metadata))

    def _grab(self):
        request = self.camera.capture_request()
        try:
            return request.make_array("main").copy(), request.get_metadata()
        finally:
            request.release()

    def _publish(self, cluster, now):
        if cluster is None:
            return
        with self.lock:
            self.detection = self.geometry.detection_from(cluster, now)

    def _announce(self, cluster):
        has_target = cluster is not None
        if has_target == self.had_target:
            return
        self.had_target = has_target
        detection = self.latest()
        if has_target:
            self.log.event("camera",
                           f"target acquired at frame {self.frames}, "
                           f"{detection.range_m:.2f} m, bearing "
                           f"{detection.bearing_deg:+.1f} deg, "
                           f"{cluster.corner_count} corners")
        else:
            self.log.event("camera", f"target lost at frame {self.frames}")
