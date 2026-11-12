"""지도 사진과 칸 번호의 버전을 점유·경로 단계에 전달한다."""

import json
from hashlib import sha256


MAP_ID_FIELDS = ("map_id", "id_assignment_revision", "map_image_sha256")
AXES = {"x": "right", "y": "down"}
WORLD_ORIGIN = "paper_top_left"


def map_identity(data):
    return {key: data[key] for key in MAP_ID_FIELDS if data.get(key) is not None}


def configured_map_identity(config):
    labels = json.loads(config.label_path.read_text(encoding="utf-8-sig"))
    digest = sha256(config.image_path.read_bytes()).hexdigest()
    if labels.get("image_sha256", digest) != digest:
        raise ValueError("라벨의 기준 사진과 현재 지도 이미지가 다릅니다.")
    return {
        **map_identity(labels),
        "map_id": config.map_id,
        "map_image_sha256": digest,
    }


def require_map_identity(expected, incoming, description):
    for field, value in map_identity(expected).items():
        if incoming.get(field) != value:
            raise ValueError(f"{description}의 {field}가 현재 지도와 다릅니다.")
