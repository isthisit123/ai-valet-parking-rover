from argparse import ArgumentParser
from dataclasses import dataclass
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parents[2]
MAPS_DIR = BASE_DIR / "assets" / "maps"
DEFAULT_OUTPUT_ROOT = BASE_DIR / "data" / "graphs"
DEFAULT_MAP_ID = "parking_map_v1"
TEST_MAP_ID = "parking_map_v1_test"
REAL_MAP_ID = "parking_map_real"
LABEL_FILENAMES = {
    DEFAULT_MAP_ID: "parking_map_v1_labels.json",
    TEST_MAP_ID: "parking_map_v1_labels_test.json",
    REAL_MAP_ID: "parking_map_real_labels.json",
}

# HSV 범위는 하한과 상한 순서다. 시설 설정의 마지막 값은 최소 면적이다.
ROAD_HSV_RANGES = (
    ((0, 100, 70), (10, 255, 255)),
    ((170, 100, 70), (179, 255, 255)),
)
# 초록색 주차칸을 검출하고 같은 색의 작은 범례는 면적으로 제외한다.
NORMAL_REGION = ((45, 100, 100), (75, 255, 255), 1500)
TEST_NORMAL_REGION = ((0, 0, 245), (179, 55, 255), 1000)
ACCESSIBLE_REGION = ((90, 70, 150), (135, 255, 255), 3000)
# 웹캠 사진의 파란색 주차칸은 디지털 지도보다 어둡게 찍힌다.
REAL_ACCESSIBLE_REGION = ((90, 70, 90), (135, 255, 255), 3000)
BUILDING_REGION = ((0, 0, 0), (179, 150, 120), 1500)
GATE_REGION = ((0, 0, 150), (179, 85, 238), 1500)
REGION_CLOSE_KERNEL_SIZE = (3, 3)
REAL_DARK_OPEN_KERNEL_SIZE = (3, 3)
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
        return self.output_dir / f"{self.map_id}.json"

    @property
    def semantic_path(self):
        return self.output_dir / f"{self.map_id}_semantic.json"

    @property
    def preview_path(self):
        return self.output_dir / "graph_preview.png"

    @property
    def skeleton_path(self):
        return self.output_dir / "road_skeleton.png"


def get_map_config(map_id=DEFAULT_MAP_ID, *, output_root=DEFAULT_OUTPUT_ROOT):
    if map_id not in LABEL_FILENAMES:
        raise ValueError(f"등록되지 않은 지도 ID {map_id!r}입니다.")

    # 원본은 기존 경로를 사용하고, 다른 지도는 자기 폴더에 결과를 저장한다.
    output_dir = Path(output_root)
    if map_id != DEFAULT_MAP_ID:
        output_dir /= map_id
    return MapConfig(
        map_id=map_id,
        image_path=MAPS_DIR / f"{map_id}.png",
        label_path=MAPS_DIR / LABEL_FILENAMES[map_id],
        output_dir=output_dir,
    )


DEFAULT_MAP = get_map_config()


def create_map_parser(description, *, uses_output_dir=True):
    parser = ArgumentParser(description=description)
    parser.add_argument(
        "--map-id", choices=LABEL_FILENAMES, default=DEFAULT_MAP_ID,
        help=f"처리할 지도 ID (기본값 {DEFAULT_MAP_ID})",
    )
    if uses_output_dir:
        parser.add_argument(
            "--output-dir", type=Path, default=DEFAULT_OUTPUT_ROOT,
            help="지도 결과를 읽고 쓰는 기준 폴더 (기본값 data/graphs)",
        )
    return parser


def parse_map_config(description, *, uses_output_dir=True):
    args = create_map_parser(description, uses_output_dir=uses_output_dir).parse_args()
    return get_map_config(
        args.map_id, output_root=getattr(args, "output_dir", DEFAULT_OUTPUT_ROOT)
    )
