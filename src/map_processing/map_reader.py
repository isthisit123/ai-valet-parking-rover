
from pathlib import Path
import cv2
import numpy as np

BASE_DIR = Path(__file__).resolve().parents[2]

IMAGE_PATH = BASE_DIR / "assets" / "maps" / "parking_map_v1.png"


def get_road_mask():
    image = cv2.imread(str(IMAGE_PATH))

    if image is None:
        raise FileNotFoundError(f"이미지를 찾을 수 없습니다: {IMAGE_PATH}")

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 100, 70])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 100, 70])
    upper_red2 = np.array([179, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    road_mask = cv2.bitwise_or(mask1, mask2)

    return road_mask


if __name__ == "__main__":
    road_mask = get_road_mask()

    cv2.imshow("Road Mask", road_mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
