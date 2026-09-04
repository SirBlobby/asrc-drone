import time

import annotate
import config
import mission
from camera_controller import CameraController
from drone_controller import DroneController
from flight_log import FlightLog
from pid import PID

BEHAVIOUR_FIELDS = ["t_mono", "elapsed", "visibility", "target_range_m",
                    "range_m", "range_error_m", "bearing_deg", "corner_count",
                    "detection_age_s", "forward_track", "forward_cmd",
                    "speed_limit", "yaw_track", "yaw_cmd", "altitude_agl",
                    "heading_deg", "camera_fps"]


class ClumpController:
    def __init__(self, log):
        self.log = log
        self.schedule = mission.RangeSchedule()
        self.bearing_pid = PID(config.YAW_GAINS, config.YAW_RATE_LIMIT_DPS)
        self.range_pid = PID(config.RANGE_GAINS,
                             config.FORWARD_SPEED_LIMIT_M_S)
        self.announced_range = None

    def step(self, detection, elapsed, now, dt):
        target_range_m = self.schedule.target_range(elapsed)
        self._announce_phase(target_range_m, elapsed)

        seen = mission.visibility(detection, now)
        range_error_m = detection.range_m - target_range_m

        yaw_track = self.bearing_pid.update(detection.bearing_deg, dt, seen)
        forward_track = self.range_pid.update(range_error_m, dt, seen)

        yaw_cmd = mission.blend(seen, yaw_track,
                                mission.search_yaw_rate(detection))
        forward_cmd = mission.blend(seen, forward_track,
                                    config.SEARCH_FORWARD_SPEED_M_S)
        speed_limit = mission.approach_speed_limit(detection.range_m)
        forward_cmd = min(forward_cmd, speed_limit)

        return {
            "visibility": round(seen, 3),
            "target_range_m": target_range_m,
            "range_m": round(detection.range_m, 2),
            "range_error_m": round(range_error_m, 2),
            "bearing_deg": round(detection.bearing_deg, 2),
            "corner_count": detection.corner_count,
            "detection_age_s": round(min(detection.age(now), 999.0), 2),
            "forward_track": round(forward_track, 3),
            "forward_cmd": round(forward_cmd, 3),
            "speed_limit": round(speed_limit, 3),
            "yaw_track": round(yaw_track, 2),
            "yaw_cmd": round(yaw_cmd, 2),
        }

    def _announce_phase(self, target_range_m, elapsed):
        if target_range_m == self.announced_range:
            return
        self.announced_range = target_range_m
        self.log.event("mission", f"t={elapsed:.1f}s target range is now "
                                  f"{target_range_m:.2f} m")


def status_line(command, drone, camera):
    return (f"see {command['visibility']:.2f} "
            f"range {command['range_m']:.2f}/{command['target_range_m']:.2f} m "
            f"bearing {command['bearing_deg']:+.1f} deg "
            f"corners {command['corner_count']} "
            f"fwd {command['forward_cmd']:+.2f} m/s "
            f"yaw {command['yaw_cmd']:+.1f} deg/s "
            f"alt {drone.altitude_agl():.2f} m "
            f"hdg {drone.yaw_deg():.0f} deg "
            f"cam {camera.fps:.1f} fps")


def fly(log, camera, drone):
    controller = ClumpController(log)
    rows = log.csv("behaviour", BEHAVIOUR_FIELDS)
    interval = 1.0 / config.CONTROL_RATE_HZ

    log.event("mission", f"clump to {config.CLUMP_RANGE_M:.1f} m then declump "
                         f"to {config.DECLUMP_RANGE_M:.1f} m over "
                         f"{controller.schedule.duration:.0f} s, collision "
                         f"floor {config.MINIMUM_RANGE_M:.2f} m")

    started = time.monotonic()
    next_tick = started
    next_status = started
    while True:
        now = time.monotonic()
        elapsed = now - started
        if controller.schedule.finished(elapsed):
            log.event("mission", "schedule complete")
            break
        if elapsed > config.MISSION_TIMEOUT_S:
            log.event("mission", "timeout reached")
            break

        command = controller.step(camera.latest(), elapsed, now, interval)
        drone.move_body(command["forward_cmd"], 0.0, command["yaw_cmd"])

        rows.write(t_mono=round(now, 4), elapsed=round(elapsed, 2),
                   altitude_agl=round(drone.altitude_agl(), 2),
                   heading_deg=round(drone.yaw_deg(), 1),
                   camera_fps=round(camera.fps, 1), **command)

        if now >= next_status:
            log.event("flight", status_line(command, drone, camera))
            next_status = now + config.STATUS_INTERVAL_S

        next_tick += interval
        time.sleep(max(0.0, next_tick - time.monotonic()))


def finish(log, camera):
    video_path = camera.convert_video()
    annotate.build(log, video_path, camera.recording_started_at)
    log.event("session", f"saved to {log.directory}")
    log.event("session", f"copy it with: {log.download_command()}")
    log.close()


def main():
    log = FlightLog(root=config.LOG_ROOT, tag="clump")
    camera = CameraController(log)
    drone = DroneController(log)
    log.event("mission", f"session {log.directory}")
    try:
        drone.connect()
        camera.start()
        if drone.takeoff(config.FLIGHT_ALTITUDE_M):
            fly(log, camera, drone)
    except KeyboardInterrupt:
        log.event("mission", "interrupted by operator")
    except Exception as error:
        log.event("mission", f"failed: {type(error).__name__}: {error}")
        raise
    finally:
        drone.land()
        time.sleep(1.0)
        camera.stop()
        drone.shutdown()
        finish(log, camera)


if __name__ == "__main__":
    main()
