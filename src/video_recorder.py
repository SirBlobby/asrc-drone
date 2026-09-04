import os
import shutil
import subprocess

import config


class VideoRecorder:
    def __init__(self, camera, log, stream="lores"):
        self.camera = camera
        self.log = log
        self.stream = stream
        self.encoder = None
        self.raw_path = os.path.join(log.directory, "video.h264")
        self.mp4_path = os.path.join(log.directory, "video.mp4")

    def start(self):
        from picamera2.encoders import H264Encoder
        from picamera2.outputs import FileOutput

        try:
            self.encoder = H264Encoder(bitrate=config.VIDEO_BITRATE)
            self.camera.start_encoder(
                self.encoder, FileOutput(self.raw_path),
                pts=os.path.join(self.log.directory, "video.pts"),
                name=self.stream)
            self.log.event("video", f"recording {self.stream} to "
                                    f"video.h264 at "
                                    f"{config.VIDEO_BITRATE // 1000} kbps")
        except Exception as error:
            self.encoder = None
            self.log.event("video", f"recording unavailable: {error}")

    def stop(self):
        if self.encoder is None:
            return
        try:
            self.camera.stop_encoder()
            size_mb = os.path.getsize(self.raw_path) / 1e6
            self.log.event("video", f"recording stopped, {size_mb:.1f} MB")
        except Exception as error:
            self.log.event("video", f"stop failed: {error}")
        self.encoder = None

    def convert(self):
        if not os.path.exists(self.raw_path):
            return None
        if shutil.which("ffmpeg") is None:
            self.log.event("video", "ffmpeg not installed, keeping video.h264")
            return None

        command = ["ffmpeg", "-y", "-loglevel", "error",
                   "-r", str(config.FRAME_RATE), "-i", self.raw_path,
                   "-c", "copy", self.mp4_path]
        try:
            subprocess.run(command, check=True)
        except subprocess.CalledProcessError as error:
            self.log.event("video", f"conversion failed: {error}")
            return None

        self.log.event("video", f"wrote {os.path.basename(self.mp4_path)}")
        if config.DELETE_RAW_VIDEO:
            os.remove(self.raw_path)
        return self.mp4_path
