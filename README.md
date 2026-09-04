# Horus Drones

Clump and declump behaviour for caged FPV drones. A drone looks for the
`#c74026` markers on another drone's cage with the Pi camera, closes on it,
holds formation, then separates again. A drone never knows where the others
are: everything it does comes from what the camera can see right now.

Target hardware is a Raspberry Pi Zero 2W talking to a PX4 flight controller
over MAVLink.

## How it works

There is no state machine. Two PID loops run every control tick and their
output is blended against a search command by a single continuous weight.

- **Bearing PID** drives yaw rate from the horizontal angle to the marker
  cluster centroid.
- **Range PID** drives forward speed from the error between the measured
  range and the range the mission schedule currently asks for.
- **Visibility** is a number between 0 and 1 built from how old the last
  detection is and how many corners it held. At 1 the drone follows the PID
  output, at 0 it follows the search command, and in between it mixes the two.

Clumping and declumping are the same controller. Only the range setpoint
changes: `MISSION_PHASES` in `src/config.py` holds it at `CLUMP_RANGE_M`, then
at `DECLUMP_RANGE_M`, and the range PID reverses on its own.

Forward speed is also capped by a braking limit computed from the distance
left to the collision floor, so the drone can always stop before the cages
touch.

## Layout

| file | what it does |
|---|---|
| `src/config.py` | every tunable value in the project |
| `src/pid.py` | PID with output clamping and weighted integration |
| `src/marker_detector.py` | `#c74026` mask, corner blobs, cluster, centroid |
| `src/detection.py` | pixels to bearing, elevation and range |
| `src/camera_controller.py` | Picamera2 capture thread |
| `src/video_recorder.py` | H264 recording and mp4 conversion |
| `src/annotate.py` | draws detections onto the recording after landing |
| `src/drone_controller.py` | MAVLink offboard setpoints, takeoff, landing |
| `src/mission.py` | range schedule, visibility, safe approach speed |
| `src/flight_log.py` | session directory, event log, CSV writers |
| `src/clump_declump.py` | the flight loop |
| `src/vision_test.py` | detection on the bench or on a recording, no flying |

## Install

Picamera2 comes from the Raspberry Pi OS packages, not from pip. On 32-bit
Pi OS there is no OpenCV wheel either, so take that from apt as well.

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-libcamera python3-opencv \
                    python3-numpy ffmpeg
```

Use `--system-site-packages` so the virtualenv can still see those, then add
the rest.

```bash
git clone <this-repo> asrc-drone
cd asrc-drone
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install pymavlink
```

On a 64-bit machine everything is available from pip instead.

```bash
pip install -r requirements.txt
```

`mavp2p` must be running and forwarding the flight controller to the address
in `CONNECTION_STRING`.

## Run

Check the detection on the bench first. `vision_test.py` never connects to
the flight controller and never arms anything, so it is safe to run with the
props on and the drone on the desk.

```bash
source .venv/bin/activate
python3 src/vision_test.py --seconds 30
```

It writes the same `vision.csv`, recording and `annotated.mp4` as a flight,
prints a line a second while it runs, and ends with a summary:

```
   31.02 [summary] 268 of 291 frames detected a target (92.1 percent)
   31.02 [summary] cv time  min 4.1ms median 5.8ms max 11.9ms
   31.02 [summary] mask     min 0.0px median 412.0px max 980.0px
   31.02 [summary] span     min 38.2px median 61.4px max 96.0px
   31.02 [summary] range    min 3.54m median 5.53m max 8.89m
   31.02 [summary] corners  min 2.0 median 4.0 max 6.0
   31.02 [summary] detection looks healthy
```

The same script replays a photo or a video instead of the live camera, so
detection can be checked on a laptop against footage from an earlier run. It
saves annotated frames into `snaps/` so you can see exactly what was matched.

```bash
python3 src/vision_test.py --source drone.jpg
python3 src/vision_test.py --source logs/clump_20260904_150612/video.mp4
python3 src/vision_test.py --source clip.mp4 --snapshot-every 1
```

Fly the clump and declump mission.

```bash
source .venv/bin/activate
python3 src/clump_declump.py
```

Keep the transmitter kill switch in hand for every powered run.

## Session logs

Every run writes a timestamped directory under `logs/`.

```
logs/clump_20260904_150612/
  events.log     human readable timeline of everything the drone did
  behaviour.csv  one row per control tick, 10 Hz
  vision.csv     one row per camera frame
  link.csv       MAVLink health, mode, battery, once a second
  video.h264     raw camera recording
  video.mp4      the same recording, converted after landing
  video.pts      frame timestamps from the camera
  annotated.mp4  the recording with the detection drawn on every frame
```

`annotated.mp4` marks the cluster bounding box, the centroid the controller
steered to, the frame centre, and a caption with range, bearing, corner count
and pixel span. Frames are matched to `vision.csv` through `video.pts`, so the
overlay stays aligned even when the detector drops behind the camera.

The last two lines of every run are the session path and a ready to paste
command for pulling it onto your laptop:

```
  74.918 [session] saved to logs/clump_20260904_150612
  74.919 [session] copy it with: scp -r pi@horus1.local:/home/pi/asrc-drone/logs/clump_20260904_150612 ~/Downloads
```

Run that from your laptop, not from the Pi. Set `SCP_HOST` in
`src/config.py` if the Pi's hostname does not resolve, and `SCP_DESTINATION`
for somewhere other than `~/Downloads`.

`events.log` is also printed live to the terminal. It carries mode and arm
changes, PX4 status text, target acquired and lost transitions, mission phase
changes, and a one line flight summary every second:

```
   12.40 [flight] see 0.85 range 3.42/2.00 m bearing +2.4 deg corners 4 fwd +0.28 m/s yaw +2.6 deg/s alt 3.01 m hdg 214 deg cam 9.8 fps
```

The mp4 conversion runs after landing and needs `ffmpeg`. Without it the raw
`video.h264` is kept and can be converted later:

```bash
ffmpeg -r 10 -i logs/<session>/video.h264 -c copy logs/<session>/video.mp4
```

Set `DELETE_RAW_VIDEO = True` in `src/config.py` to drop the h264 file once
the mp4 exists, and `ANNOTATE_VIDEO = False` to skip the annotation pass,
which is the slowest part of shutdown on a Zero 2W.

## Tuning

Everything lives in `src/config.py`.

**Marker colour.** `#c74026` sits at hue 5 in OpenCV, so the window wraps
through red: `MARKER_HUE_WRAP_LOW` to 179 and 0 to `MARKER_HUE_WRAP_HIGH`.
Lower `MARKER_SATURATION_MIN` if the markers wash out in bright sun, raise it
if the mask picks up skin, brick or dirt. Run `vision_test.py` after every
change and watch the `mask` and detection rate lines in its summary.

**Blob sizes.** `MIN_CORNER_AREA_PX` and `MAX_CORNER_AREA_PX` are in pixels at
`FRAME_SIZE`. Changing the resolution means refitting both, along with
`CLUSTER_LINK_MIN_PX`. Everything else in the clustering is relative and
carries across unchanged.

**Range.** Range comes from the pixel span of the corner cluster:
`range = focal_px * CAGE_WIDTH_M / span_px`. `HORIZONTAL_FOV_DEG` has to match
the lens actually fitted, or every range will be wrong by a constant factor.

**Control.** `YAW_GAINS` and `RANGE_GAINS` are `(kp, ki, kd)`. Raise
`DETECTION_HOLD_S` to coast further through dropped frames, lower it to fall
back to searching sooner.

## Performance

The Pi Zero 2W is the constraint. `FRAME_SIZE` is 320x240 because every stage
before clustering is linear in pixels, and the detector only ever labels the
frame once, filters blobs with vectorized numpy, and builds one pairwise
distance matrix. Watch `cv_ms` and `fps` in `vision.csv`: if `fps` falls far
below `FRAME_RATE` the control loop is acting on stale detections and
`DETECTION_HOLD_S` will start cutting visibility.
