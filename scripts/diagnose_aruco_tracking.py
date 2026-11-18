"""실행 중인 서버를 읽어 정지/이동 구간의 검출 누락과 위치 만료를 구분한다."""

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import statistics
import time
from urllib.request import urlopen


def distribution(values):
    values = sorted(v for v in values if isinstance(v, (int, float)))
    if not values:
        return None
    return {"count": len(values), "median": statistics.median(values),
            "p95": values[min(len(values) - 1, int((len(values) - 1) * .95))], "max": max(values)}


def summarize(samples):
    # GET 폴링 횟수를 카메라 프레임 수로 오인하지 않는다.
    frames = {}
    for sample in samples:
        pose = sample.get("pose", {})
        if pose.get("received_monotonic_s") is not None:
            key = (json.dumps(pose.get("camera_source"), sort_keys=True), pose["received_monotonic_s"], pose.get("frame_id"))
            frames.setdefault(key, pose)
    frames = list(frames.values())
    statuses = Counter(p.get("detection_status", p.get("tracking_status")) for p in frames)
    arrival_runs = {}
    for sample in samples:
        arrival = sample.get("pose", {}).get("arrival_test", {})
        if arrival.get("started_at") and isinstance(arrival.get("hold_reset_counts"), dict):
            arrival_runs[arrival["started_at"]] = arrival["hold_reset_counts"]
    reset_counts = Counter()
    for counts in arrival_runs.values():
        reset_counts.update(counts)
    centers = [p['rover']['center_px'] for p in frames if p.get('rover')]
    # 이동 구간이라는 이름만으로 실물 이동을 검증했다고 보고하지 않는다.
    span = [max(p[i] for p in centers) - min(p[i] for p in centers) for i in (0, 1)] if centers else None
    intervals = [(b["received_monotonic_s"] - a["received_monotonic_s"]) * 1000
                 for a, b in zip(frames, frames[1:]) if a.get("camera_source") == b.get("camera_source")]
    return {"unique_frames": len(frames), "detection_status_counts": dict(statuses),
            "marker_center_span_px": span,
            "detection_ratio": statuses["detected"] / len(frames) if frames else None,
            "first_seen_tracking_status_counts": dict(Counter(p.get("tracking_status") for p in frames)),
            "poll_tracking_status_counts": dict(Counter(s.get("pose", {}).get("tracking_status", "request_error") for s in samples)),
            "server_receive_interval_ms": distribution(intervals),
            "processing_ms": distribution([p.get("processing_ms") for p in frames]),
            "first_seen_capture_age_ms": distribution([p.get("capture_age_ms") for p in frames]),
            "arrival_hold_reset_counts": dict(reset_counts),
            "arrival_poll_status_counts": dict(Counter(s.get("pose", {}).get("arrival_test", {}).get("status", "unavailable") for s in samples)),
            "arrival_reason_counts": dict(Counter(p.get("arrival_test", {}).get("reason", "none") for p in frames))}


def collect(base_url, duration, interval):
    samples = []
    started = time.monotonic()
    while time.monotonic() - started < duration:
        tick = time.monotonic()
        sample = {"elapsed_s": tick - started}
        try:
            with urlopen(base_url + "/api/rover-pose", timeout=2) as response:
                sample["pose"] = json.load(response)
        except (OSError, ValueError) as error:
            sample["error"] = str(error)
        samples.append(sample)
        time.sleep(max(0, interval - (time.monotonic() - tick)))
    return samples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--label", choices=("stationary", "moving", "arrival"), required=True)
    parser.add_argument("--duration", type=float, default=10)
    parser.add_argument("--interval", type=float, default=.05)
    parser.add_argument("--url", default="http://127.0.0.1:8001")
    parser.add_argument("--output-dir", type=Path, default=Path("data/rover_runs/aruco/diagnostics"))
    args = parser.parse_args()
    if not 1 <= args.duration <= 120 or not .02 <= args.interval <= 1:
        parser.error("수집 시간은 1~120초, 폴링 간격은 0.02~1초여야 합니다.")
    wall_started = datetime.now(timezone.utc).isoformat()
    samples = collect(args.url.rstrip("/"), args.duration, args.interval)
    report = {"label": args.label, "started_at": wall_started, "duration_s": args.duration,
              "note": "서버 수신 간격은 브라우저 응답 간격과 다르며, 촬영 나이는 GET 폴링 대기 시간을 포함한다.",
              "summary": summarize(samples), "samples": samples}
    args.output_dir.mkdir(parents=True, exist_ok=True)
    try:
        with urlopen(args.url.rstrip("/") + "/api/aruco-debug?miss=1", timeout=2) as response:
            diagnostic = json.load(response)
        with urlopen(args.url.rstrip("/") + diagnostic["frame_url"], timeout=2) as response:
            body, content_type = response.read(), response.headers.get_content_type()
        if hashlib.sha256(body).hexdigest() != diagnostic["image_sha256"]:
            raise ValueError("진단 JSON과 원본 영상의 해시가 다릅니다.")
        # 이전 시험의 누락을 이번 구간 누락으로 기록하지 않는다.
        captured = diagnostic.get("client_captured_at")
        if captured and datetime.fromisoformat(captured.replace("Z", "+00:00")) >= datetime.fromisoformat(wall_started):
            suffix = ".jpg" if content_type == "image/jpeg" else ".png"
            frame_path = args.output_dir / (args.label + ".miss" + suffix)
            frame_path.write_bytes(body)
            report["miss_frame"] = {**diagnostic, "saved_path": str(frame_path.resolve())}
    except (OSError, ValueError, KeyError) as error:
        report["miss_frame_error"] = str(error)
    path = args.output_dir / (args.label + ".json")
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report["summary"], ensure_ascii=False, indent=2))
    print(path.resolve())


if __name__ == "__main__":
    main()
