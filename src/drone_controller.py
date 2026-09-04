import math
import threading
import time

from pymavlink import mavutil

import config
from pid import clamp

PX4_MAIN_MODES = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO",
                  5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED"}
OFFBOARD_MODE = 6
AUTO_MODE = 4
LAND_SUBMODE = 6

RECEIVED_TYPES = ["LOCAL_POSITION_NED", "ATTITUDE", "HEARTBEAT", "STATUSTEXT",
                  "BATTERY_STATUS"]

LINK_FIELDS = ["t_mono", "mode", "armed", "setpoint_hz", "heartbeat_gap_s",
               "position_age_s", "local_x", "local_y", "local_z",
               "altitude_agl", "heading_deg", "battery_v", "battery_pct",
               "setpoint_kind"]

POSITION_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

VELOCITY_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE)


class DroneController:
    def __init__(self, log=None, connection_string=config.CONNECTION_STRING):
        self.log = log
        self.connection_string = connection_string
        self.link = None
        self.lock = threading.Lock()
        self.thread = None
        self.running = False

        self.setpoint_is_velocity = False
        self.position_setpoint = (0.0, 0.0, 0.0, 0.0)
        self.velocity_setpoint = (0.0, 0.0, 0.0)
        self.velocity_stamp = 0.0
        self.held_z = 0.0

        self.local_position = None
        self.attitude = None
        self.home = None
        self.mode = None
        self.armed = False
        self.battery = None
        self.csv = None
        self.setpoints_sent = 0
        self.last_heartbeat_at = 0.0
        self.last_position_at = 0.0

    def connect(self):
        self.link = mavutil.mavlink_connection(self.connection_string)
        self._wait_for_autopilot()
        self._request_streams()

        self.local_position = self.link.recv_match(
            type="LOCAL_POSITION_NED", blocking=True, timeout=5)
        self.attitude = self.link.recv_match(
            type="ATTITUDE", blocking=True, timeout=5)
        if self.local_position is None or self.attitude is None:
            raise RuntimeError("no local position or attitude, EKF not ready")

        self.home = (self.local_position.x, self.local_position.y,
                     self.local_position.z, self.attitude.yaw)
        self._event(f"home x={self.home[0]:.2f} y={self.home[1]:.2f} "
                    f"z={self.home[2]:.2f} "
                    f"yaw={math.degrees(self.home[3]):.0f} deg")

        self.csv = self.log.csv("link", LINK_FIELDS) if self.log else None
        self.running = True
        self.thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.thread.start()

    def takeoff(self, altitude_m):
        home_x, home_y, home_z, home_yaw = self.home
        target_z = home_z - altitude_m

        self._set_position(home_x, home_y, home_z, home_yaw)
        time.sleep(1.0)
        if not self._set_mode(OFFBOARD_MODE):
            self._event("offboard refused, aborting takeoff")
            return False
        if not self._arm():
            self._event("arm refused, aborting takeoff")
            return False

        self._set_position(home_x, home_y, target_z, home_yaw)
        deadline = time.monotonic() + config.TAKEOFF_TIMEOUT_S
        while time.monotonic() < deadline:
            if abs(self.local_position.z - target_z) < config.TAKEOFF_TOLERANCE_M:
                self._event(f"reached {altitude_m:.2f} m")
                return True
            time.sleep(0.1)
        self._event("takeoff timed out, landing")
        self.land()
        return False

    def move_body(self, forward_m_s, right_m_s, yaw_rate_dps):
        with self.lock:
            self.velocity_setpoint = (
                clamp(forward_m_s, config.LATERAL_SPEED_LIMIT_M_S),
                clamp(right_m_s, config.LATERAL_SPEED_LIMIT_M_S),
                math.radians(clamp(yaw_rate_dps, config.YAW_RATE_LIMIT_DPS)))
            self.velocity_stamp = time.monotonic()
            self.setpoint_is_velocity = True

    def hold(self):
        position, attitude = self.local_position, self.attitude
        if position is None or attitude is None:
            return
        self._set_position(position.x, position.y, self.held_z, attitude.yaw)

    def land(self):
        with self.lock:
            self.setpoint_is_velocity = False
        self._set_mode(AUTO_MODE, LAND_SUBMODE, confirm=False)
        self._event("landing")

    def shutdown(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
            self.thread = None

    def altitude_agl(self):
        if self.local_position is None or self.home is None:
            return 0.0
        return -(self.local_position.z - self.home[2])

    def yaw_deg(self):
        return math.degrees(self.attitude.yaw) if self.attitude else 0.0

    def _wait_for_autopilot(self, timeout_s=15.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._send_heartbeat()
            beat = self.link.recv_match(type="HEARTBEAT", blocking=True,
                                        timeout=1)
            if (beat and beat.get_srcComponent() == 1 and
                    beat.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID):
                self.link.target_system = beat.get_srcSystem()
                self.link.target_component = beat.get_srcComponent()
                self._event(f"autopilot on system {self.link.target_system}")
                return
        raise RuntimeError("no autopilot heartbeat")

    def _request_streams(self):
        wanted = {30: 20, 32: 20, 1: 2, 147: 1, 253: 2}
        for message_id, rate_hz in wanted.items():
            self._command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                          message_id, int(1e6 / rate_hz))

    def _set_position(self, x, y, z, yaw):
        with self.lock:
            self.position_setpoint = (x, y, z, yaw)
            self.held_z = z
            self.setpoint_is_velocity = False

    def _stream_loop(self):
        interval = 1.0 / config.SETPOINT_RATE_HZ
        next_tick = time.monotonic()
        last_heartbeat = 0.0
        last_sample = time.monotonic()
        while self.running:
            try:
                self._drain()
                now = time.monotonic()
                self._send_setpoint(now)
                self.setpoints_sent += 1
                if now - last_heartbeat >= 1.0:
                    self._send_heartbeat()
                    last_heartbeat = now
                if now - last_sample >= config.LINK_SAMPLE_INTERVAL_S:
                    self._sample_link(now, now - last_sample)
                    last_sample = now
            except Exception as error:
                self._event(f"stream error: {error}")
                time.sleep(0.05)

            next_tick += interval
            time.sleep(max(0.0, next_tick - time.monotonic()))

    def _send_setpoint(self, now):
        with self.lock:
            expired = (self.setpoint_is_velocity and
                       now - self.velocity_stamp > config.COMMAND_TIMEOUT_S)
            use_velocity = self.setpoint_is_velocity and not expired

        if expired:
            self._event("velocity command expired, holding position")
            self.hold()

        if use_velocity:
            self._send_velocity()
        else:
            self._send_position()

    def _send_position(self):
        with self.lock:
            x, y, z, yaw = self.position_setpoint
        self.link.mav.set_position_target_local_ned_send(
            0, self.link.target_system, self.link.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, POSITION_MASK,
            x, y, z, 0, 0, 0, 0, 0, 0, yaw, 0)

    def _send_velocity(self):
        with self.lock:
            forward, right, yaw_rate = self.velocity_setpoint
            held_z = self.held_z
        current_z = self.local_position.z if self.local_position else held_z
        down = clamp(config.ALTITUDE_GAIN * (held_z - current_z),
                     config.CLIMB_SPEED_LIMIT_M_S)
        self.link.mav.set_position_target_local_ned_send(
            0, self.link.target_system, self.link.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED, VELOCITY_MASK,
            0, 0, 0, forward, right, down, 0, 0, 0, 0, yaw_rate)

    def _drain(self):
        while True:
            message = self.link.recv_match(type=RECEIVED_TYPES, blocking=False)
            if message is None:
                return
            kind = message.get_type()
            if kind == "LOCAL_POSITION_NED":
                self.local_position = message
                self.last_position_at = time.monotonic()
            elif kind == "BATTERY_STATUS":
                self.battery = message
            elif kind == "ATTITUDE":
                self.attitude = message
            elif kind == "HEARTBEAT":
                self._on_heartbeat(message)
            elif kind == "STATUSTEXT":
                self._event(_text_of(message), source="px4")

    def _on_heartbeat(self, message):
        if message.get_srcSystem() != self.link.target_system:
            return
        self.last_heartbeat_at = time.monotonic()
        armed = bool(message.base_mode &
                     mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        main = (message.custom_mode >> 16) & 0xFF
        mode = PX4_MAIN_MODES.get(main, f"main{main}")
        if mode != self.mode:
            self._event(f"mode {self.mode} -> {mode}")
            self.mode = mode
        if armed != self.armed:
            self._event("armed" if armed else "disarmed")
            self.armed = armed

    def _sample_link(self, now, window_s):
        if self.csv is None:
            return
        position = self.local_position
        with self.lock:
            kind = "velocity" if self.setpoint_is_velocity else "position"
        self.csv.write(
            t_mono=round(now, 3), mode=self.mode, armed=int(self.armed),
            setpoint_hz=round(self.setpoints_sent / window_s, 1),
            heartbeat_gap_s=round(now - self.last_heartbeat_at, 2),
            position_age_s=round(now - self.last_position_at, 2),
            local_x=round(position.x, 2) if position else "",
            local_y=round(position.y, 2) if position else "",
            local_z=round(position.z, 2) if position else "",
            altitude_agl=round(self.altitude_agl(), 2),
            heading_deg=round(self.yaw_deg(), 1),
            battery_v=_battery_volts(self.battery),
            battery_pct=getattr(self.battery, "battery_remaining", ""),
            setpoint_kind=kind)
        self.setpoints_sent = 0

    def _send_heartbeat(self):
        self.link.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def _command(self, command, *params):
        padded = list(params) + [0] * (7 - len(params))
        self.link.mav.command_long_send(
            self.link.target_system, self.link.target_component,
            command, 0, *padded)

    def _set_mode(self, main_mode, sub_mode=0, confirm=True, timeout_s=4.0):
        self._command(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                      mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                      main_mode, sub_mode)
        if not confirm:
            return True
        return self._wait_until(
            lambda: self.mode == PX4_MAIN_MODES.get(main_mode), timeout_s)

    def _arm(self, timeout_s=4.0):
        self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        return self._wait_until(lambda: self.armed, timeout_s)

    def _wait_until(self, condition, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if condition():
                return True
            time.sleep(0.1)
        return False

    def _event(self, message, source="drone"):
        if self.log:
            self.log.event(source, message)
        else:
            print(f"[{source}] {message}")


def _battery_volts(battery):
    voltages = getattr(battery, "voltages", None)
    if not voltages or voltages[0] in (0, 65535):
        return ""
    return round(voltages[0] / 1000.0, 2)


def _text_of(message):
    text = getattr(message, "text", "")
    if isinstance(text, bytes):
        text = text.decode("utf-8", "ignore")
    return text.strip("\x00").strip()
