
import cv2
import numpy as np
from skimage.morphology import skeletonize

from .map_reader import get_road_mask, BASE_DIR


def create_skeleton(road_mask):
    # 도로 마스크를 Boolean 배열로 변환
    binary = road_mask > 0

    # 도로를 1픽셀 두께의 중심선으로 변환
    skeleton = skeletonize(binary)

    # Boolean 배열을 OpenCV 이미지 형식으로 변환
    skeleton = skeleton.astype(np.uint8) * 255

    return skeleton

def remove_small_components(road_mask):
    # 연결된 흰색 영역을 각각 구분
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        road_mask, connectivity=8
    )

    # 도로가 검출되지 않았다면 오류 발생
    if num_labels <= 1:
        raise ValueError("No road detected")

    # 배경(0번)을 제외하고 가장 큰 영역 선택
    largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])

    # 가장 큰 영역만 남기기
    cleaned_mask = np.where(labels == largest_label, 255, 0).astype(np.uint8)

    return cleaned_mask

def detect_nodes(skeleton):
    # Skeleton을 0과 1로 변환
    binary = (skeleton > 0).astype(np.uint8)

    # 주변 8개 픽셀의 연결 개수를 계산
    kernel = np.ones((3, 3), dtype=np.uint8)
    kernel[1, 1] = 0

    neighbor_count = cv2.filter2D(
        binary,
        cv2.CV_16S,
        kernel,
        borderType=cv2.BORDER_CONSTANT
    )

    # 끝점과 교차점 후보 검출
    endpoints = ((binary == 1) & (neighbor_count == 1))
    junctions = ((binary == 1) & (neighbor_count >= 3))

    # 교차점 후보를 이진 마스크로 변환
    junction_mask = junctions.astype(np.uint8) * 255

    # 가까운 교차점 후보들을 하나의 그룹으로 병합
    junction_mask = cv2.dilate(
        junction_mask,
        np.ones((31, 31), dtype=np.uint8),
        iterations=1
    )

    # 교차점 그룹의 중심 좌표 계산
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        junction_mask,
        connectivity=8
    )

    junction_points = []

    for i in range(1, count):
        x, y = centroids[i]
        junction_points.append((int(round(x)), int(round(y))))

    # 끝점 좌표 추출
    ys, xs = np.where(endpoints)
    endpoint_points = list(zip(xs.tolist(), ys.tolist()))

    return endpoint_points, junction_points


if __name__ == "__main__":
    # 빨간색 도로가 추출된 마스크 불러오기
    road_mask = get_road_mask()

    # 범례 등 분리된 작은 영역 제거
    road_mask = remove_small_components(road_mask)

    # 세선화 수행
    skeleton = create_skeleton(road_mask)

    # Skeleton 이미지 저장 경로 설정
    output_path = BASE_DIR / "data" / "graphs" / "road_skeleton.png"
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # 원본 Skeleton 저장
    cv2.imwrite(str(output_path), skeleton)

    # 화면 표시용으로 Skeleton 선을 두껍게 만들기
    kernel = np.ones((5, 5), np.uint8)
    preview = cv2.dilate(skeleton, kernel, iterations=1)

    # 화면에 맞게 이미지 크기 조절
    height, width = preview.shape
    scale = min(1.0, 700 / width, 700 / height)

    preview = cv2.resize(
        preview,
        (int(width * scale), int(height * scale)),
        interpolation=cv2.INTER_NEAREST
    )

    # 세선화 결과 화면 출력
    cv2.imshow("Skeleton Preview", preview)

    # 도로 마스크와 Skeleton의 픽셀 수 비교
    print("Road Mask pixels:", cv2.countNonZero(road_mask))
    print("Skeleton pixels:", cv2.countNonZero(skeleton))
    print("Saved to:", output_path)

    # 끝점과 교차점 후보 검출
    endpoints, junctions = detect_nodes(skeleton)

    # 화면 표시용으로 Skeleton 선을 두껍게 만들기
    kernel = np.ones((5, 5), np.uint8)
    node_preview = cv2.dilate(skeleton, kernel, iterations=1)

    # 흑백 이미지를 컬러 이미지로 변환
    node_preview = cv2.cvtColor(node_preview, cv2.COLOR_GRAY2BGR)

    # 끝점은 빨간색으로 표시
    for x, y in endpoints:
        cv2.circle(node_preview, (x, y), 10, (0, 0, 255), -1)

    # 교차점은 초록색으로 표시
    for x, y in junctions:
        cv2.circle(node_preview, (x, y), 10, (0, 255, 0), -1)

    # 화면에 맞게 이미지 크기 축소
    height, width = node_preview.shape[:2]
    scale = min(1.0, 700 / width, 700 / height)

    node_preview = cv2.resize(
        node_preview,
        (int(width * scale), int(height * scale)),
        interpolation=cv2.INTER_NEAREST
    )

    # 노드 검출 결과 출력
    cv2.imshow("Node Detection", node_preview)

    print("Endpoint candidates:", len(endpoints))
    print("Junction candidates:", len(junctions))

    # 키 입력을 기다린 뒤 창 닫기
    cv2.waitKey(0)
    cv2.destroyAllWindows()
