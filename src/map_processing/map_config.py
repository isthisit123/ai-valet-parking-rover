from argparse import ArgumentParser
from dataclasses import dataclass
from pathlib import Path
import re


BASE_DIR = Path(__file__).resolve().parents[2]
MAPS_DIR = BASE_DIR / "assets" / "maps"
DEFAULT_OUTPUT_ROOT = BASE_DIR / "data" / "maps"
TEST_MAP_ID = "parking_map_v1_test"
REAL_MAP_ID = "parking_map_real"
DEFAULT_MAP_ID = REAL_MAP_ID
OCR_LABEL_SCHEME = "A,B,P1-P6,EN,EX"
OCR_ID_REVISION = "ocr-p-labels-v1"

# HSV 범위는 하한과 상한 순서다. 시설 설정의 마지막 값은 최소 면적이다.
ROAD_HSV_RANGES = (
    ((0, 100, 70), (10, 255, 255)),
    ((170, 100, 70), (179, 255, 255)),
)
# 초록색 주차칸을 검출하고 같은 색의 작은 범례는 면적으로 제외한다.
NORMAL_REGION = ((45, 100, 100), (75, 255, 255), 1500)
# 실물 촬영에서는 음영으로 어두워진 초록색도 같은 주차칸 후보로 검출한다.
REAL_NORMAL_REGION = ((45, 100, 90), (75, 255, 255), 1500)
TEST_NORMAL_REGION = ((0, 0, 245), (179, 55, 255), 1000)
ACCESSIBLE_REGION = ((90, 70, 150), (135, 255, 255), 3000)
# 웹캠 사진의 파란색 주차칸은 디지털 지도보다 어둡게 찍힌다.
REAL_ACCESSIBLE_REGION = ((90, 70, 90), (135, 255, 255), 3000)
# 어두운 실물 촬영에서는 칸의 밝기 하한을 낮추고 검은 시설은 배경보다 어두운 픽셀로 분리한다.
LOW_LIGHT_NORMAL_REGION = ((45, 100, 60), (75, 255, 255), 1500)
LOW_LIGHT_ACCESSIBLE_REGION = ((90, 70, 65), (135, 255, 255), 3000)
LOW_LIGHT_BUILDING_REGION = ((0, 0, 0), (179, 255, 70), 1500)
# 실물 사진의 픽셀 면적과 형태 처리에 사용한 기준 해상도다.
REAL_REGION_REFERENCE_SIZE = (1280, 720)
BUILDING_REGION = ((0, 0, 0), (179, 150, 120), 1500)
GATE_REGION = ((0, 0, 150), (179, 85, 238), 1500)
REGION_CLOSE_KERNEL_SIZE = (3, 3)
# 실물 사진에서 입구에 가늘게 이어진 그림자를 끊는다.
REAL_DARK_OPEN_KERNEL_SIZE = (7, 7)
# 임시 완화: 그림자가 붙은 실물 입구 후보를 보존한다. 재검토 사항은 ROADMAP 참고.
REAL_DARK_MIN_FILL_RATIO = 0.5
# 회색 바닥과 검은 시설이 붙어 검출될 때만 적용하는 더 어두운 재검출 기준.
REAL_DARK_RETRY_VALUE_LIMITS = (110, 100, 90)
REGION_MAX_IMAGE_RATIO = 0.08
REGION_BORDER_MARGIN = 1
REGION_MIN_SIDE = 20
REGION_MIN_FILL_RATIO = 0.6
JUNCTION_KERNEL_SIZE = (31, 31)
ENDPOINT_RADIUS = 5
JUNCTION_RADIUS = 20


def normal_region_for_map(map_id):
    # 기존 테스트 지도에만 흰색 주차칸 설정을 적용한다.
    return TEST_NORMAL_REGION if map_id == TEST_MAP_ID else NORMAL_REGION


@dataclass(frozen=True)
class MapConfig:
    map_id: str
    image_path: Path
    label_path: Path
    output_dir: Path

    @property
    def graph_path(self):
        return self.output_dir / "graph.json"

    @property
    def semantic_path(self):
        return self.output_dir / "semantic.json"

    @property
    def preview_path(self):
        return self.output_dir / "graph_preview.png"

    @property
    def skeleton_path(self):
        return self.output_dir / "road_skeleton.png"


def available_map_ids():
    # 이미지와 대응 라벨 파일을 갖춘 지도 ID를 자동으로 찾는다.
    return sorted(
        path.name.removesuffix("_labels.json")
        for path in MAPS_DIR.glob("*_labels.json")
        if (MAPS_DIR / f"{path.name.removesuffix('_labels.json')}.png").is_file()
    )


def validate_map_id(map_id):
    if not isinstance(map_id, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", map_id):
        raise ValueError("map_id에는 영문자, 숫자, 밑줄, 하이픈만 사용할 수 있습니다.")
    return map_id


def map_paths(map_id, *, output_root=DEFAULT_OUTPUT_ROOT):
    return MapConfig(
        map_id=map_id,
        image_path=MAPS_DIR / f"{map_id}.png",
        label_path=MAPS_DIR / f"{map_id}_labels.json",
        output_dir=Path(output_root) / map_id,
    )


def get_map_config(map_id=DEFAULT_MAP_ID, *, output_root=DEFAULT_OUTPUT_ROOT):
    if map_id not in available_map_ids():
        raise ValueError(f"지도 {map_id!r}의 PNG 이미지와 _labels.json 파일이 필요합니다.")
    return map_paths(map_id, output_root=output_root)


# 라벨을 처음 생성하는 명령에서도 기본 경로를 가져올 수 있어야 한다.
DEFAULT_MAP = map_paths(DEFAULT_MAP_ID)


def create_map_parser(description, *, uses_output_dir=True):
    parser = ArgumentParser(description=description)
    parser.add_argument(
        "--map-id", choices=available_map_ids(), default=DEFAULT_MAP_ID,
        help=f"처리할 지도 ID (기본값 {DEFAULT_MAP_ID})",
    )
    if uses_output_dir:
        parser.add_argument(
            "--output-dir", type=Path, default=DEFAULT_OUTPUT_ROOT,
            help="지도 결과를 읽고 쓰는 기준 폴더 (기본값 data/maps)",
        )
    return parser


def parse_map_config(description, *, uses_output_dir=True):
    args = create_map_parser(description, uses_output_dir=uses_output_dir).parse_args()
    return get_map_config(
        args.map_id, output_root=getattr(args, "output_dir", DEFAULT_OUTPUT_ROOT)
    )
