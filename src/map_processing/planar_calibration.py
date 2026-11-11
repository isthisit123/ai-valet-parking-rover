import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def quadrilateral(points, name):
    values = np.asarray(points, dtype=np.float32)
    if values.shape != (4, 2) or not np.isfinite(values).all():
        raise ValueError(f"{name}에는 유효한 [x, y] 좌표가 4개 필요합니다.")
    if not cv2.isContourConvex(values):
        raise ValueError(f"{name}의 네 점을 둘레 순서로 지정하세요.")
    return values


def project_points(points, matrix):
    values = np.asarray(points, dtype=np.float64)
    return cv2.perspectiveTransform(values.reshape(-1, 1, 2), matrix).reshape(-1, 2)


def load_world_to_image(path, *, map_id, image_size):
    report = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if report.get("map_id") != map_id:
        raise ValueError(f"지도 {map_id!r}와 Homography의 map_id가 다릅니다.")
    if report.get("validation", {}).get("passed") is not True:
        raise ValueError("검증을 통과한 Homography JSON이 필요합니다.")
    if report.get("source_frame", {}).get("size_px") != list(image_size):
        raise ValueError("Homography 사진과 현재 지도 이미지의 해상도가 다릅니다.")
    target_frame = report.get("target_frame", {})
    if (
        target_frame.get("coordinate_frame") != "world"
        or target_frame.get("unit") != "cm"
        or target_frame.get("origin") != "paper_top_left"
        or target_frame.get("axis_directions") != {"x": "right", "y": "down"}
    ):
        raise ValueError("전지 왼쪽 위 원점, x 오른쪽, y 아래 방향의 cm 좌표가 필요합니다.")
    paper_size = np.asarray(report.get("paper_size_cm"), dtype=float)
    if paper_size.shape != (2,) or not np.isfinite(paper_size).all() or (paper_size <= 0).any():
        raise ValueError("Homography JSON에 유효한 전지 크기(paper_size_cm)가 필요합니다.")
    matrix = np.asarray(report.get("image_to_world"), dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all() or np.linalg.matrix_rank(matrix) != 3:
        raise ValueError("Homography JSON에 역변환 가능한 3×3 image_to_world 행렬이 필요합니다.")
    return np.linalg.inv(matrix), paper_size


def fit_floor_transform(pixel, world_cm, paper_px):
    matrix, _ = cv2.findHomography(pixel, world_cm, method=0)
    if matrix is None or not np.isfinite(matrix).all() or np.linalg.matrix_rank(matrix) != 3:
        raise ValueError("이 기준점으로 유효한 변환을 계산할 수 없습니다.")
    # 종이 안에서 투영 분모가 0을 지나면 유효한 바닥 좌표를 얻을 수 없다.
    denominator = paper_px @ matrix[2, :2] + matrix[2, 2]
    if not (np.all(denominator > 0) or np.all(denominator < 0)):
        raise ValueError("이 기준점으로 전지 전체에 유효한 변환을 계산할 수 없습니다.")
    return matrix


def calibrate_floor(spec, image_size, *, max_error_cm):
    if not np.isfinite(max_error_cm) or max_error_cm <= 0:
        raise ValueError("허용 오차는 양수(cm)로 지정하세요.")
    size_cm = np.asarray(spec["paper_size_cm"], dtype=np.float32)
    if size_cm.shape != (2,) or not np.isfinite(size_cm).all() or (size_cm <= 0).any():
        raise ValueError("전지의 가로·세로 길이를 양수(cm)로 지정하세요.")
    width_cm, height_cm = map(float, size_cm)
    paper_world = np.float32([[0, 0], [width_cm, 0], [width_cm, height_cm], [0, height_cm]])

    controls = spec["control_points"]
    marker_px = quadrilateral([point["pixel"] for point in controls], "십자의 픽셀 좌표")
    marker_cm = quadrilateral([point["world_cm"] for point in controls], "십자의 실측 좌표")
    paper_px = quadrilateral(spec["paper_corners_px"], "전지 모서리의 픽셀 좌표")
    all_px = np.concatenate([paper_px, marker_px])
    if (all_px < 0).any() or (all_px >= image_size).any():
        raise ValueError("픽셀 좌표가 사진 범위를 벗어났습니다.")
    if (marker_cm < 0).any() or (marker_cm > size_cm).any():
        raise ValueError("십자의 실측 좌표가 전지 범위를 벗어났습니다.")
    if any(cv2.contourArea(quad, oriented=True) <= 0 for quad in (marker_px, marker_cm, paper_px)):
        raise ValueError("좌표를 왼쪽 위부터 시계 방향으로 지정하세요.")

    all_world = np.concatenate([paper_world, marker_cm])
    matrix = fit_floor_transform(all_px, all_world, paper_px)
    point_names = ["paper_top_left", "paper_top_right", "paper_bottom_right", "paper_bottom_left"]
    point_names.extend(point["id"] for point in controls)
    # 계산에 쓰지 않은 점의 오차를 보기 위해 여덟 점을 하나씩 제외한다.
    held_out_checks = []
    for index, name in enumerate(point_names):
        remaining = np.arange(len(all_px)) != index
        trial = fit_floor_transform(all_px[remaining], all_world[remaining], paper_px)
        estimated = project_points(all_px[index:index + 1], trial)[0]
        held_out_checks.append({
            "id": name,
            "pixel": all_px[index].tolist(),
            "expected_cm": all_world[index].tolist(),
            "estimated_cm": estimated.tolist(),
            "error_cm": float(np.linalg.norm(estimated - all_world[index])),
        })
    max_error = max(check["error_cm"] for check in held_out_checks)
    passed = max_error <= max_error_cm

    # 재측정이 필요한 십자를 찾을 수 있도록 모서리만 사용한 비교도 기록한다.
    paper_to_world = cv2.getPerspectiveTransform(paper_px, paper_world)
    estimated_markers_cm = project_points(marker_px, paper_to_world)
    return {
        "map_id": spec["map_id"],
        "source_frame": {"coordinate_frame": "camera_image", "size_px": list(image_size)},
        "target_frame": {
            "coordinate_frame": "world",
            "unit": "cm",
            "origin": "paper_top_left",
            "axis_directions": {"x": "right", "y": "down"},
        },
        "lens_distortion_corrected": False,
        "paper_size_cm": spec["paper_size_cm"],
        ("image_to_world" if passed else "candidate_image_to_world"): matrix.tolist(),
        "validation": {
            "passed": passed,
            "method": "leave_one_out",
            "tolerance_cm": max_error_cm,
            "max_held_out_error_cm": max_error,
            "held_out_checks": held_out_checks,
            "marker_checks_against_paper": [
                {**point, "estimated_cm": estimate.tolist(),
                 "error_cm": float(np.linalg.norm(estimate - measured))}
                for point, estimate, measured in zip(controls, estimated_markers_cm, marker_cm, strict=True)
            ],
        },
    }


def main():
    parser = argparse.ArgumentParser(description="전지 모서리와 십자로 바닥 픽셀을 cm로 변환하고 오차를 검사합니다.")
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--points", type=Path, required=True, help="기준점, 실측 좌표 및 전지 모서리 JSON")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-error-cm", type=float, required=True)
    args = parser.parse_args()
    if args.output.resolve() in (args.image.resolve(), args.points.resolve()):
        parser.error("결과는 입력과 다른 파일에 저장하세요.")
    source = args.image.read_bytes()
    image = cv2.imdecode(np.frombuffer(source, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        parser.error(f"사진을 읽을 수 없습니다. {args.image}")
    height, width = image.shape[:2]
    try:
        spec = json.loads(args.points.read_text(encoding="utf-8"))
        report = calibrate_floor(spec, (width, height), max_error_cm=args.max_error_cm)
    except (KeyError, TypeError, ValueError) as error:
        parser.error(str(error))
    report["source_frame"].update({"image": args.image.name, "sha256": hashlib.sha256(source).hexdigest()})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    error = report["validation"]["max_held_out_error_cm"]
    if not report["validation"]["passed"]:
        print(f"검증 실패. 최대 교차검증 오차 {error:.2f} cm. 후보 결과: {args.output}")
        return 1
    print(f"검증 통과. 최대 교차검증 오차 {error:.2f} cm. 결과: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
