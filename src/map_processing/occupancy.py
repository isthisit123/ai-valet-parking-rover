"""주차칸 ID 기반 점유 스냅샷 검증. 카메라 좌표 매칭은 생산자가 담당한다."""

import json
import math
from datetime import datetime, timezone

from .map_identity import require_map_identity


DEFAULT_OCCUPANCY_TTL = 5.0
MIN_FREE_CONFIDENCE = 0.5


def load_occupancy(path):
    with open(path, encoding="utf-8-sig") as stream:
        snapshot = json.load(stream)
    if not isinstance(snapshot, dict):
        raise ValueError("점유 스냅샷은 JSON 객체여야 합니다.")
    return snapshot


def occupancy_states(map_data, snapshot, *, max_age_seconds=DEFAULT_OCCUPANCY_TTL, now=None):
    if not isinstance(snapshot, dict):
        raise ValueError("점유 스냅샷은 JSON 객체여야 합니다.")
    # 이 TTL은 초기 통합 기본값이며 실제 카메라 갱신 주기에 맞춰 조정한다.
    if not math.isfinite(max_age_seconds) or max_age_seconds <= 0:
        raise ValueError("점유 정보 TTL은 양의 유한한 초 단위 값이어야 합니다.")
    if not map_data.get("map_id"):
        raise ValueError("의미 지도에 map_id가 필요합니다.")
    require_map_identity(map_data, snapshot, "점유 스냅샷")
    if snapshot.get("schema_version") != "0.1-draft":
        raise ValueError("지원하는 점유 schema_version은 0.1-draft입니다.")
    try:
        observed = datetime.fromisoformat(snapshot["observed_at"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("observed_at은 시간대가 포함된 ISO 8601이어야 합니다.") from exc
    if observed.utcoffset() is None:
        raise ValueError("observed_at에 시간대가 필요합니다.")
    now = now or datetime.now(timezone.utc)
    if now.utcoffset() is None:
        raise ValueError("현재 시각에 시간대가 필요합니다.")
    age = (now - observed).total_seconds()
    fresh = 0 <= age <= max_age_seconds
    states = {
        node["region_id"]: "unknown" for node in map_data["nodes"]
        if node.get("role") in ("parking_normal", "parking_accessible")
    }
    if not isinstance(snapshot.get("spaces"), list):
        raise ValueError("점유 spaces는 배열이어야 합니다.")
    seen = set()
    for space in snapshot["spaces"]:
        if not isinstance(space, dict):
            raise ValueError("점유 spaces의 각 항목은 객체여야 합니다.")
        space_id = space.get("parking_space_id")
        if not isinstance(space_id, str) or space_id not in states or space_id in seen:
            raise ValueError(f"알 수 없거나 중복된 주차칸 ID: {space_id}")
        seen.add(space_id)
        status = space.get("status")
        if status not in ("free", "occupied", "unknown"):
            raise ValueError(f"잘못된 점유 상태: {status}")
        confidence = space.get("confidence")
        if confidence is not None and (
            isinstance(confidence, bool) or not isinstance(confidence, (int, float))
            or not math.isfinite(confidence) or not 0 <= confidence <= 1
        ):
            raise ValueError(f"잘못된 점유 신뢰도: {space_id}")
        # 차량 미검출만으로 free를 만들지 않는다. 생산자의 명시적 판정이 필요하다.
        # 0.5는 임시 소비 기준이며 차량 탐지 확률과 같은 의미가 아니다.
        if fresh and (status != "free" or confidence is not None and confidence >= MIN_FREE_CONFIDENCE):
            states[space_id] = status
    return states
