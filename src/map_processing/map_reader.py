
import cv2
import numpy as np
from pathlib import Path

from .map_config import DEFAULT_MAP, ROAD_HSV_RANGES, parse_map_config


def load_map_image(image_path):
    image = cv2.imdecode(np.frombuffer(Path(image_path).read_bytes(), np.uint8), cv2.IMREAD_COLOR)

    if image is None:
        raise FileNotFoundError(f"지도 이미지를 읽을 수 없습니다. {image_path}")
    return image


def detect_road_mask(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask1, mask2 = [cv2.inRange(hsv, lower, upper) for lower, upper in ROAD_HSV_RANGES]
    return cv2.bitwise_or(mask1, mask2)


def get_road_mask(image_path=DEFAULT_MAP.image_path):
    return detect_road_mask(load_map_image(image_path))


if __name__ == "__main__":
    config = parse_map_config("도로 마스크를 미리 봅니다.", uses_output_dir=False)
    road_mask = get_road_mask(config.image_path)

    cv2.imshow("Road Mask", road_mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
