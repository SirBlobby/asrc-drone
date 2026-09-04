# Horus Drones

Clumping behaviour for caged FPV drones. A drone looks for the `#c74026`
markers on another drone's cage with the Pi camera, closes on it, and holds
formation. A drone never knows where the others are: everything it does comes
from what the camera can see right now.

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

```
visibility  = freshness(detection age) * strength(corner count)
yaw rate    = visibility * bearing PID + (1 - visibility) * search yaw rate
forward     = visibility * range PID   + (1 - visibility) * search creep
forward     = min(forward, braking limit from the collision floor)
```

The range setpoint comes from `MISSION_PHASES` in `src/config.py`, which holds
it at `CLUMP_RANGE_M` for `CLUMP_HOLD_S` and then lands. The schedule is a list
of `(duration_s, target_range_m)` pairs rather than a branch, so adding a phase
is a data change: a second entry at a longer range makes the drone separate
again, and the same range PID reverses on its own to fly it.

The integral term is scaled by visibility, so a drone that is searching never
winds up.

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
| `src/clump.py` | the flight loop |
| `src/vision_test.py` | detection on the bench or on a recording, no flying |

## Install

Picamera2 comes from the Raspberry Pi OS packages, not from pip, so the
virtualenv has to be able to see the system packages.

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-libcamera ffmpeg libopenblas0

git clone <this-repo> asrc-drone
cd asrc-drone
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`libopenblas0` is there because pip installs OpenCV as a wheel that links
against OpenBLAS at runtime without bringing it along. Raspberry Pi OS serves
those wheels from piwheels on both 32-bit and 64-bit, so this applies either
way. Without it the import fails:

```
ImportError: libopenblas.so.0: cannot open shared object file
```

On older releases the package is `libopenblas0-pthread` or `libopenblas-base`.

### Taking OpenCV from apt instead

The Debian build is compiled for the Pi and pulls its own dependencies, so
nothing can be missing at runtime. It is the safer option if pip keeps
fighting you.

```bash
sudo apt install -y python3-opencv python3-numpy
pip install pymavlink
```

Skip `requirements.txt` in that case, or pip will shadow the apt build with a
wheel inside the virtualenv.

`mavp2p` must be running and forwarding the flight controller to the address
in `CONNECTION_STRING`.

## Run

Run both scripts from the repository root. Sessions are written to `logs/`
relative to the working directory.

### Test the detection, no flying

`vision_test.py` never opens the MAVLink connection and never arms anything,
so it is safe to run with the drone on the desk.

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
   31.02 [summary] relaxed  6 frames held the target only through the fallback pass
   31.02 [summary] dropouts 9 runs of missed frames, longest 3 frames (0.30 s), median 1
   31.02 [summary] detection looks healthy
```

The `dropouts` line is the one to watch. It counts runs of consecutive missed
frames and reports the longest. If that longest run reaches
`DETECTION_HOLD_S`, visibility fell to zero and the drone would have gone all
the way back to searching, and the summary says so outright. Short runs are
harmless: visibility decays smoothly, so a frame or two missed only softens
the tracking, it does not flip any switch.

The remaining verdict lines flag detection under 50 percent, and clusters
whose median size is under `TRUSTED_CORNER_COUNT`, which caps visibility below
1.0 and leaves the drone permanently part way into its search behaviour.

The same script replays a photo or a video instead of the live camera, so
detection can be checked on a laptop against footage from an earlier run.
Frames are resized to `FRAME_SIZE` first, so the pixel thresholds mean the
same thing they do in flight. Replay writes `vision.csv` and annotated stills
into `snaps/`; it does not produce a video.

```bash
python3 src/vision_test.py --source drone.jpg
python3 src/vision_test.py --source logs/clump_20260904_150612/video.mp4
python3 src/vision_test.py --source clip.mp4 --snapshot-every 1
```

`--snapshot-every N` saves every Nth frame and always saves the first, capped
at `SNAPSHOT_LIMIT`. Pass 0 to disable.

### Fly the mission

```bash
source .venv/bin/activate
python3 src/clump.py
```

Keep the transmitter kill switch in hand for every powered run.

By default the drone climbs to 3 m, scans until it finds another drone, closes
to `CLUMP_RANGE_M` and holds there for `CLUMP_HOLD_S`, then lands. The whole
run is capped at `MISSION_TIMEOUT_S`, 150 s, whether or not the schedule ever
completes.

Both entry points run a preflight geometry check first and log anything that
does not add up, so a target range the hardware cannot reach shows up on the
ground rather than as a drone that hovers and never closes:

```
    0.02 [preflight] WARNING 0.20 m target is inside the 0.49 m collision floor, so the braking cap will stop the drone short of it forever
    0.02 [preflight] WARNING 0.20 m target needs a 1699 px span in a 320 px frame, which cannot be measured. The closest measurable range at 24.3 deg is 1.33 m; 0.20 m needs a lens of at least 110 deg
    0.02 [preflight] WARNING 0.20 m is closer than two 0.46 m cages can physically be. They touch at 0.46 m centre to centre, so this asks them to overlap by 26 cm. Ranges are centre to centre, not the air gap between the cages
```

`CLUMP_RANGE_M` is set to 0.20 m deliberately, as a request to close as far as
the hardware allows. Ranges are centre to centre, and two 0.46 m cages touch
at 0.4572 m, so 0.20 m is not reachable. The collision floor at 0.4872 m is
what actually stops the drone, holding the cages about 3 cm apart. The three
warnings above are expected on every run at this setting.

The schedule clock starts on the first detection, not at takeoff. Searching is
untimed, so a drone that takes a minute to find its partner still gets its
full clump phase. `behaviour.csv` logs both clocks: `elapsed` since takeoff and
`schedule_t` since acquisition.

## Session logs

Every run writes a timestamped directory under `logs/`. A flight is tagged
`clump`, a bench detection test is tagged `vision`.

```
logs/clump_20260904_150612/
  events.log     human readable timeline of everything the drone did
  behaviour.csv  one row per control tick, 10 Hz
  vision.csv     one row per camera frame
  corners.csv    every accepted corner blob, one row per corner per frame
  link.csv       MAVLink health, mode, battery, once a second
  video.h264     raw camera recording
  video.mp4      the same recording, converted after landing
  video.pts      frame timestamps from the camera
  annotated.mp4  the recording with the detection drawn on every frame

logs/vision_20260904_144210/
  events.log     per frame detection lines and the closing summary
  vision.csv     as above
  corners.csv    as above
  snaps/         annotated stills, when replaying a file
```

`events.log` is also printed live to the terminal. It carries mode and arm
changes, PX4 status text, target acquired and lost transitions, mission phase
changes, and a one line flight summary every second:

```
   12.40 [flight] t0009.2 see 0.85 range 3.42/2.00 m bearing +2.4 deg corners 4 fwd +0.28 m/s yaw +2.6 deg/s alt 3.01 m hdg 214 deg cam 9.8 fps
```

The last two lines of every run are the session path and a ready to paste
command for pulling it onto your laptop:

```
  74.918 [session] saved to logs/clump_20260904_150612
  74.919 [session] copy it with: scp -r pi@horus1.local:/home/pi/asrc-drone/logs/clump_20260904_150612 ~/Downloads
```

Run that from your laptop, not from the Pi. Set `SCP_HOST` in
`src/config.py` if the Pi's hostname does not resolve, and `SCP_DESTINATION`
for somewhere other than `~/Downloads`.

### behaviour.csv

One row per control tick. This is the controller's own view of the world.

| column | meaning |
|---|---|
| `t_mono`, `elapsed` | monotonic clock, and seconds since takeoff |
| `schedule_t` | seconds since the first detection, what the schedule reads |
| `visibility` | the blend weight, 0 searching to 1 fully tracking |
| `target_range_m` | what the schedule is asking for right now |
| `range_m`, `range_error_m` | measured range, and measured minus target |
| `bearing_deg` | horizontal angle to the centroid, positive to the right |
| `corner_count`, `detection_age_s` | size and age of the detection in use |
| `forward_track`, `yaw_track` | raw PID output before blending |
| `forward_cmd`, `yaw_cmd` | what was actually sent to the flight controller |
| `speed_limit` | braking cap from the collision floor |
| `altitude_agl`, `heading_deg` | where the drone was |
| `camera_fps` | detection loop rate at that moment |

`forward_cmd` differing from `forward_track` means the drone was part way into
its search behaviour, or the braking limit was binding.

### vision.csv

One row per camera frame, written whether or not anything was found.

| column | meaning |
|---|---|
| `frame`, `t_mono` | frame number and capture time |
| `cv_ms`, `fps` | detector cost and the rate it is achieving |
| `seen` | 1 if a cluster was accepted |
| `mask_px` | pixels passing the colour window, before clustering |
| `corners_seen`, `corner_count` | corners in the frame, and in the chosen cluster |
| `centre_x`, `centre_y` | the centroid |
| `box_x`, `box_y`, `box_w`, `box_h` | cluster bounding box |
| `span_px`, `range_m` | widest corner separation, and the range it implies |
| `bearing_deg`, `elevation_deg` | angles to the centroid |
| `exposure_us`, `gain`, `lux` | what the camera thought of the light |

`mask_px` high with `seen` at 0 means the colour window is matching something
that is not a cage. Both near 0 means the window is too tight, or the markers
are out of frame.

### corners.csv

One row per corner blob per frame: `frame`, `x`, `y`, `area`, `radius`,
`in_cluster`. This is what the annotator draws, and it is the file to read
when a cluster looks wrong. Corners with `in_cluster` at 0 were accepted as
blobs but did not join the winning group, which usually means the link reach
was too tight or a second drone was in frame.

### link.csv

MAVLink health, once a second: `mode`, `armed`, `setpoint_hz`,
`heartbeat_gap_s`, `position_age_s`, `local_x`, `local_y`, `local_z`,
`altitude_agl`, `heading_deg`, `battery_v`, `battery_pct`, `setpoint_kind`.

`setpoint_hz` well under `SETPOINT_RATE_HZ`, or `heartbeat_gap_s` climbing
past a second, means PX4 will drop out of offboard. Check those first when a
flight ends earlier than the schedule.

### Video

`annotated.mp4` shows the whole detection, not just its result:

- every accepted corner blob gets a circle, green when it made the cluster and
  dim red when it was found but grouped out
- the cluster gets a green box, which is the other drone
- the centroid the controller actually steered to is a cross inside the box
- the frame centre is a grey cross, so bearing error is the gap between them
- a caption carries range, bearing, cluster size out of corners seen, and span

On a frame with no detection the last known box is redrawn in orange, labelled
`coasting on last box`, so a brief dropout reads as a held track rather than
the box vanishing. The label says `searching` only once the track is genuinely
gone. `ANNOTATE_SCALE` upscales the video before drawing, so the overlay is
legible at 320x240.

Frames are matched to `vision.csv` through `video.pts`, so the overlay stays
aligned even when the detector drops behind the camera.

The mp4 conversion runs after landing and needs `ffmpeg`. Without it the raw
`video.h264` is kept and can be converted later:

```bash
ffmpeg -r 10 -i logs/<session>/video.h264 -c copy logs/<session>/video.mp4
```

Set `DELETE_RAW_VIDEO = True` in `src/config.py` to drop the h264 file once
the mp4 exists, `ANNOTATE_VIDEO = False` to skip the annotation pass, which is
the slowest part of shutdown on a Zero 2W, and `RECORD_VIDEO = False` to turn
recording off entirely.

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
Confirm it against a tape measure with `vision_test.py` before trusting any
range.

The lens sets how close the drones can formate. A cage cannot be measured once
it overflows the frame, so the closest measurable range is
`range_constant / (USABLE_FRAME_FRACTION * frame width)`. At 24.3 degrees and
320 px that is 1.33 m, and the span at each range is:

| range | span | of a 320 px frame |
|---|---|---|
| 2.00 m | 170 px | 53 percent |
| 1.33 m | 256 px | 80 percent, the practical limit |
| 1.00 m | 340 px | overflows |
| 0.49 m | 697 px | overflows, the collision floor |
| 0.20 m | 1699 px | overflows by more than 5x |

Below the limit the corners clip on the frame edge, the span stops growing,
and the range estimate saturates and then reads backwards as corners leave the
frame. Widening the lens is the only fix. A 47 degree lens reaches 0.66 m, and
60 degrees reaches the 0.49 m cage contact floor. That trades against
acquisition, since a wider lens puts fewer pixels on a distant cage, so the
drone will not see its partner from as far away.

**This limit also caps the collision floor.** The braking cap is computed from
the measured range, and below 1.33 m that measurement saturates. The drone can
be at 0.8 m while the camera still reports 1.33 m, and the cap will happily
allow 0.66 m/s into a cage it cannot see closing. Until the lens is wide
enough to measure the whole approach, the floor is advisory and the kill
switch is the real protection.

**Control.** `YAW_GAINS` and `RANGE_GAINS` are `(kp, ki, kd)`. Raise
`DETECTION_HOLD_S` to coast further through dropped frames, lower it to fall
back to searching sooner. `TRUSTED_CORNER_COUNT` is the cluster size at which
the drone commits fully to tracking.

**Holding the track.** Four settings fight frame to frame dropouts, in the
order they act:

- `SHAPE_TEST_MIN_AREA_PX` exempts small blobs from the fill and aspect tests.
  A corner only a few pixels across has meaningless shape statistics, and
  testing it anyway is the single most likely cause of a target that flickers
  at range.
- `MARKER_CORE_SATURATION` and `MARKER_CORE_VALUE` are a strict inner colour
  window. A blob is kept if any of its pixels clear the strict window, but its
  full extent comes from the loose one, so dim edges stay attached while pale
  look-alikes are still rejected.
- `CONTINUITY_BONUS` weights clusters near the previous frame's cluster, so
  the choice does not hop between candidates. `CONTINUITY_REACH` is how far
  counts as near, in multiples of the last span, and `CONTINUITY_HOLD_FRAMES`
  is how long the memory survives without a detection.
- `FALLBACK_FRAMES` lets the detector retry with the strict colour seeding
  dropped, for that many frames after a good detection. `vision.csv` marks
  those frames with `relaxed` at 1, and the run summary counts them.

**Search.** With nothing in view the drone scans in steps rather than turning
continuously. It yaws `SEARCH_STEP_DEG` at `SEARCH_YAW_RATE_DPS`, then holds
still for `SEARCH_DWELL_S` before the next step. At the defaults that is a 20
degree step taking half a second, a one second pause, and 27 seconds for a
full revolution.

The pause is the point. The camera sees 24 degrees, so a 20 degree step
overlaps slightly and nothing falls between steps, and each dwell gives the
detector about ten motion free frames to work with. Turning continuously
smears the markers across the frame and is why a spinning drone detects worse
than a stationary one.

It steps toward the side the target was last seen on, so a target that leaves
the right edge of the frame is searched for to the right.

`SEARCH_FORWARD_SPEED_M_S` defaults to 0, so the drone holds station while it
scans. Setting it above 0 makes the drone creep forward while searching, which
sounds helpful but means two drones that cannot see each other drive apart in
whatever direction they happen to be facing.

**Mission and safety.** `MISSION_PHASES` is a tuple of
`(duration_s, target_range_m)` pairs and can hold as many phases as you want.
Those durations are measured from the first detection, so lengthening the
search does not eat into the clump phase. It currently holds one phase, the
clump. Appending `(20.0, 4.0)` would make the drone separate to 4 m for 20 s
afterwards, with no code change.

`MINIMUM_RANGE_M` is derived, not set directly: it is two cage radii plus
`SAFETY_GAP_M`, currently 0.4572 plus 0.03 for a 0.4872 m floor. The approach
speed is capped by how much room is left before that floor, so a target range
at or inside the floor means the drone closes to the floor and holds there for
the rest of the phase, which is the intent at the current 0.20 m setting.
`BRAKING_DECEL_M_S2` and `CONTROL_LAG_S` set how early the cap bites.


## Performance

The Pi Zero 2W is the constraint. `FRAME_SIZE` is 320x240 because every stage
before clustering is linear in pixels, and the detector only ever labels the
frame once, filters blobs with vectorized numpy, and builds one pairwise
distance matrix. Watch `cv_ms` and `fps` in `vision.csv`: if `fps` falls far
below `FRAME_RATE` the control loop is acting on stale detections and
`DETECTION_HOLD_S` will start cutting visibility.
