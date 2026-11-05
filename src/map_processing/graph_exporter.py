
import json
import math

import cv2
import networkx as nx
import numpy as np

from .map_reader import BASE_DIR, get_road_mask
from .graph_builder import (
    remove_small_components,
    create_skeleton,
    detect_nodes,
)


def build_graph(skeleton, endpoints, junctions):
    # Skeleton의 모든 흰색 픽셀로 임시 그래프 생성
    pixel_graph = nx.Graph()

    ys, xs = np.where(skeleton > 0)
    pixels = set(zip(xs.tolist(), ys.tolist()))
    pixel_graph.add_nodes_from(pixels)

    # 서로 인접한 Skeleton 픽셀들을 연결
    for x, y in pixels:
        for dx, dy in [(1, 0), (0, 1), (1, 1), (-1, 1)]:
            next_pixel = (x + dx, y + dy)

            if next_pixel not in pixels:
                continue

            # 대각선에 불필요한 지름길이 생기는 것 방지
            if dx != 0 and dy != 0:
                if (x + dx, y) in pixels or (x, y + dy) in pixels:
                    continue

            pixel_graph.add_edge(
                (x, y),
                next_pixel,
                weight=math.hypot(dx, dy)
            )

    # 끝점과 교차점을 실제 그래프 노드로 등록
    points = [("endpoint", p) for p in endpoints]
    points += [("junction", p) for p in junctions]

    graph = nx.MultiGraph()

    for index, (kind, (x, y)) in enumerate(points):
        node_id = f"N{index}"

        graph.add_node(
            node_id,
            x=int(x),
            y=int(y),
            type=kind
        )

    # 같은 노드 주변의 픽셀을 하나의 영역으로 묶기
    region_of = {}

    for pixel in pixels:
        x, y = pixel
        candidates = []

        for index, (kind, (node_x, node_y)) in enumerate(points):
            radius = 5 if kind == "endpoint" else 20

            distance_sq = (
                (x - node_x) ** 2 +
                (y - node_y) ** 2
            )

            if distance_sq <= radius ** 2:
                candidates.append(
                    (distance_sq, f"N{index}")
                )

        if candidates:
            region_of[pixel] = min(candidates)[1]

    # 노드 영역을 제외하고 도로 구간만 남기기
    road_graph = pixel_graph.copy()
    road_graph.remove_nodes_from(region_of)

    # 각 도로 구간이 연결하는 두 노드 찾기
    for component in nx.connected_components(road_graph):
        touched = set()
        contact_pixels = {}

        for pixel in component:
            for neighbor in pixel_graph.neighbors(pixel):
                node_id = region_of.get(neighbor)

                if node_id is not None:
                    touched.add(node_id)
                    contact_pixels.setdefault(
                        node_id, []
                    ).append(pixel)

        if len(touched) != 2:
            raise ValueError(
                f"Road connection error: {sorted(touched)}"
            )

        a, b = sorted(touched)
        subgraph = road_graph.subgraph(component)

        # 두 노드에 가장 가까운 도로 픽셀 선택
        pa = (graph.nodes[a]["x"], graph.nodes[a]["y"])
        pb = (graph.nodes[b]["x"], graph.nodes[b]["y"])

        start = min(
            contact_pixels[a],
            key=lambda p: math.dist(p, pa)
        )

        end = min(
            contact_pixels[b],
            key=lambda p: math.dist(p, pb)
        )

        # 도로 중심선을 따라 경로 추적
        pixel_path = nx.shortest_path(
            subgraph, start, end, weight="weight"
        )

        path = [pa] + pixel_path + [pb]

        # 중심선을 따라 이동한 거리 계산
        length = sum(
            math.dist(p, q)
            for p, q in zip(path, path[1:])
        )

        # 최종 그래프에 간선 추가
        graph.add_edge(
            a,
            b,
            weight=round(length, 2),
            path=path
        )

    return graph


def save_graph(graph, skeleton):
    # 노드 데이터를 JSON 형식으로 변환
    nodes = [
        {
            "id": node_id,
            "type": data["type"],
            "x": data["x"],
            "y": data["y"]
        }
        for node_id, data in graph.nodes(data=True)
    ]

    # 간선 데이터를 JSON 형식으로 변환
    edges = []

    for a, b, data in graph.edges(data=True):
        path = data["path"]

        # 경로의 시작점이 from 노드와 일치하는지 확인
        start_pos = (
            graph.nodes[a]["x"],
            graph.nodes[a]["y"]
        )

        # 경로 방향이 반대라면 뒤집기
        if tuple(path[0]) != start_pos:
            path = path[::-1]

        edges.append({
            "from": a,
            "to": b,
            "weight": data["weight"],
            "path": [list(p) for p in path]
        })

    # JSON 저장
    output_dir = BASE_DIR / "data" / "graphs"
    output_dir.mkdir(parents=True, exist_ok=True)

    json_path = output_dir / "parking_map_v1.json"

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(
            {
                "unit": "pixel",
                "nodes": nodes,
                "edges": edges
            },
            f,
            indent=2,
            ensure_ascii=False
        )

    # 결과 시각화를 위한 이미지 생성
    preview = cv2.dilate(
        skeleton,
        np.ones((3, 3), np.uint8)
    )
    preview = cv2.cvtColor(
        preview, cv2.COLOR_GRAY2BGR
    )

    # 그래프 노드 표시
    for node_id, data in graph.nodes(data=True):
        x, y = data["x"], data["y"]

        if data["type"] == "endpoint":
            color = (0, 0, 255)
        else:
            color = (0, 255, 0)

        cv2.circle(preview, (x, y), 11, color, -1)

        cv2.putText(
            preview,
            node_id,
            (x + 14, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            2
        )

    # 그래프 시각화 이미지 저장
    preview_path = output_dir / "graph_preview.png"
    cv2.imwrite(str(preview_path), preview)

    return json_path, preview_path


if __name__ == "__main__":
    # 기존 지도 분석 함수 재사용
    road_mask = get_road_mask()
    road_mask = remove_small_components(road_mask)

    skeleton = create_skeleton(road_mask)
    endpoints, junctions = detect_nodes(skeleton)

    # 노드와 간선을 가진 그래프 생성
    graph = build_graph(skeleton, endpoints, junctions)

    # 결과 저장
    json_path, preview_path = save_graph(graph, skeleton)

    # 결과 확인
    print("Nodes:", graph.number_of_nodes())
    print("Edges:", graph.number_of_edges())
    print("Connected:", nx.is_connected(graph))
    print("JSON:", json_path)
    print("Preview:", preview_path)
