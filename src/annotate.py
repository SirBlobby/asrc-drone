import csv
import os

import cv2

import config

BOX_COLOUR = (0, 255, 0)
CENTROID_COLOUR = (0, 200, 255)
CROSSHAIR_COLOUR = (120, 120, 120)
TEXT_COLOUR = (0, 255, 255)
FONT = cv2.FONT_HERSHEY_SIMPLEX


def caption(frame_index, cluster, detection):
    if cluster is None:
        return f"f{frame_index} no target"
    return (f"f{frame_index} {detection.range_m:.2f} m "
            f"{detection.bearing_deg:+.1f} deg "
            f"{cluster.corner_count}/{cluster.corners_seen} corners "
            f"span {cluster.span_px:.0f} px")


def draw(frame, box, centre, text, mask_pixels=None):
    height, width = frame.shape[:2]
    cv2.drawMarker(frame, (width // 2, height // 2), CROSSHAIR_COLOUR,
                   cv2.MARKER_CROSS, 14, 1)
    cv2.putText(frame, text, (6, 16), FONT, 0.4, TEXT_COLOUR, 1)
    if mask_pixels is not None:
        cv2.putText(frame, f"mask {mask_pixels} px", (6, height - 8), FONT,
                    0.4, TEXT_COLOUR, 1)

    if box is None:
        return frame
    left, top, box_width, box_height = box
    cv2.rectangle(frame, (left, top), (left + box_width, top + box_height),
                  BOX_COLOUR, 1)
    cv2.drawMarker(frame, centre, CENTROID_COLOUR, cv2.MARKER_TILTED_CROSS,
                   12, 2)
    return frame


def draw_cluster(frame, cluster, detection, frame_index, mask_pixels):
    box = cluster.box if cluster else None
    centre = (int(cluster.x), int(cluster.y)) if cluster else None
    return draw(frame, box, centre,
                caption(frame_index, cluster, detection), mask_pixels)


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

    output_path = os.path.join(log.directory, "annotated.mp4")
    size = (int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    writer = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc(*"mp4v"),
                             config.FRAME_RATE, size)

    timeline = _timeline(log.directory, rows, recording_started_at)
    written = _draw_all(capture, writer, rows, timeline)
    capture.release()
    writer.release()

    source = "frame timestamps" if timeline else "frame order"
    log.event("annotate", f"wrote annotated.mp4, {written} frames "
                          f"aligned by {source}")
    return output_path


def _draw_all(capture, writer, rows, timeline):
    written = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            return written
        _draw_row(frame, rows[_row_for(written, rows, timeline)])
        writer.write(frame)
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


def _draw_row(frame, row):
    seen = row["seen"] == "1"
    box = (int(float(row["box_x"])), int(float(row["box_y"])),
           int(float(row["box_w"])), int(float(row["box_h"]))) if seen else None
    centre = (int(float(row["centre_x"])),
              int(float(row["centre_y"]))) if seen else None
    return draw(frame, box, centre, _row_caption(row), row["mask_px"])


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
