import csv
import os
from collections import defaultdict

import cv2

import config

MEMBER_COLOUR = (0, 255, 0)
REJECTED_COLOUR = (90, 90, 200)
BOX_COLOUR = (0, 255, 0)
COASTED_COLOUR = (0, 140, 200)
CENTROID_COLOUR = (0, 200, 255)
CROSSHAIR_COLOUR = (110, 110, 110)
TEXT_COLOUR = (0, 255, 255)
FONT = cv2.FONT_HERSHEY_SIMPLEX


def caption(frame_index, cluster, detection):
    if cluster is None:
        return f"f{frame_index} no target"
    return (f"f{frame_index} {detection.range_m:.2f} m "
            f"{detection.bearing_deg:+.1f} deg "
            f"{cluster.corner_count}/{cluster.corners_seen} corners "
            f"span {cluster.span_px:.0f} px")


def draw(frame, box, centre, corners, text, note=None, held=False):
    scale = config.ANNOTATE_SCALE
    if scale != 1:
        frame = cv2.resize(frame, None, fx=scale, fy=scale,
                           interpolation=cv2.INTER_NEAREST)
    height, width = frame.shape[:2]

    cv2.drawMarker(frame, (width // 2, height // 2), CROSSHAIR_COLOUR,
                   cv2.MARKER_CROSS, 16, 1)
    for x, y, radius, in_cluster in corners:
        cv2.circle(frame, (int(x * scale), int(y * scale)),
                   max(4, int(radius * scale) + 3),
                   MEMBER_COLOUR if in_cluster else REJECTED_COLOUR, 1)

    if box is not None:
        left, top, box_width, box_height = (int(v * scale) for v in box)
        colour = COASTED_COLOUR if held else BOX_COLOUR
        cv2.rectangle(frame, (left, top), (left + box_width, top + box_height),
                      colour, 2)
        if centre is not None:
            cv2.drawMarker(frame, (int(centre[0] * scale),
                                   int(centre[1] * scale)),
                           CENTROID_COLOUR, cv2.MARKER_TILTED_CROSS, 14, 2)

    cv2.putText(frame, text, (6, 18), FONT, 0.45, TEXT_COLOUR, 1)
    if note:
        cv2.putText(frame, note, (6, height - 8), FONT, 0.45, TEXT_COLOUR, 1)
    return frame


def draw_cluster(frame, cluster, detection, frame_index, mask_pixels,
                 corners=()):
    drawn = [(c.x, c.y, c.radius, c.in_cluster) for c in corners]
    box = cluster.box if cluster else None
    centre = (cluster.x, cluster.y) if cluster else None
    return draw(frame, box, centre, drawn,
                caption(frame_index, cluster, detection),
                f"mask {mask_pixels} px  corners {len(drawn)}")


def build(log, video_path, recording_started_at=0.0):
    if not config.ANNOTATE_VIDEO or not video_path:
        return None
    rows = _load_vision(os.path.join(log.directory, "vision.csv"))
    if not rows:
        log.event("annotate", "no vision rows, skipping")
        return None

    capture = cv2.VideoCapture(video_path)
    if not capture.isOpened():
        log.event("annotate", f"cannot open {os.path.basename(video_path)}")
        return None

    corners = _load_corners(os.path.join(log.directory, "corners.csv"))
    scale = config.ANNOTATE_SCALE
    size = (int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)) * scale,
            int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)) * scale)
    output_path = os.path.join(log.directory, "annotated.mp4")
    writer = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc(*"mp4v"),
                             config.FRAME_RATE, size)

    timeline = _timeline(log.directory, rows, recording_started_at)
    written, held = _draw_all(capture, writer, rows, corners, timeline)
    capture.release()
    writer.release()

    source = "frame timestamps" if timeline else "frame order"
    log.event("annotate", f"wrote annotated.mp4, {written} frames "
                          f"aligned by {source}, {held} coasted")
    return output_path


def _draw_all(capture, writer, rows, corners, timeline):
    written = 0
    held = 0
    last_hit = None
    while True:
        ok, frame = capture.read()
        if not ok:
            return written, held
        row = rows[_row_for(written, rows, timeline)]
        last_hit = row if row["seen"] == "1" else last_hit
        held += int(row["seen"] != "1" and last_hit is not None)
        writer.write(_draw_row(frame, row, last_hit, corners))
        written += 1


def _row_for(frame_index, rows, timeline):
    if timeline and frame_index < len(timeline):
        return timeline[frame_index]
    return min(frame_index, len(rows) - 1)


def _timeline(directory, rows, recording_started_at):
    stamps = _load_timestamps(os.path.join(directory, "video.pts"))
    if not stamps or recording_started_at <= 0.0:
        return None

    times = [float(row["t_mono"]) for row in rows]
    timeline = []
    cursor = 0
    for milliseconds in stamps:
        frame_time = recording_started_at + milliseconds / 1000.0
        while (cursor + 1 < len(times) and
               abs(times[cursor + 1] - frame_time) <=
               abs(times[cursor] - frame_time)):
            cursor += 1
        timeline.append(cursor)
    return timeline


def _draw_row(frame, row, last_hit, corners):
    seen = row["seen"] == "1"
    shown = row if seen else last_hit
    box = _box_of(shown)
    centre = _centre_of(shown) if seen else None
    note = _note(row, seen, shown)
    return draw(frame, box, centre, corners.get(row["frame"], []),
                _row_caption(row), note, held=not seen)


def _note(row, seen, shown):
    if seen:
        return (f"mask {row['mask_px']} px  "
                f"corners {row['corners_seen']}"
                f"{'  relaxed' if row.get('relaxed') == '1' else ''}")
    if shown is None:
        return f"searching  mask {row['mask_px']} px"
    return f"coasting on last box  mask {row['mask_px']} px"


def _box_of(row):
    if row is None or row["seen"] != "1":
        return None
    return (float(row["box_x"]), float(row["box_y"]),
            float(row["box_w"]), float(row["box_h"]))


def _centre_of(row):
    return (float(row["centre_x"]), float(row["centre_y"]))


def _row_caption(row):
    if row["seen"] != "1":
        return f"f{row['frame']} no target"
    return (f"f{row['frame']} {row['range_m']} m {row['bearing_deg']} deg "
            f"{row['corner_count']}/{row['corners_seen']} corners "
            f"span {row['span_px']} px")


def _load_vision(path):
    if not os.path.exists(path):
        return []
    with open(path) as handle:
        return list(csv.DictReader(handle))


def _load_corners(path):
    found = defaultdict(list)
    if not os.path.exists(path):
        return found
    with open(path) as handle:
        for row in csv.DictReader(handle):
            found[row["frame"]].append((float(row["x"]), float(row["y"]),
                                        float(row["radius"]),
                                        row["in_cluster"] == "1"))
    return found


def _load_timestamps(path):
    if not os.path.exists(path):
        return []
    stamps = []
    with open(path) as handle:
        for line in handle:
            try:
                stamps.append(float(line.strip()))
            except ValueError:
                continue
    return stamps
