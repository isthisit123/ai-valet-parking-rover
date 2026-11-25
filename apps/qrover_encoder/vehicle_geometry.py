"""월드 cm의 차체 외곽·회전 궤적을 종이 경계와 시설 외곽에 대조한다."""
import math


def cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def on_segment(p, a, b):
    return abs(cross(a, b, p)) < 1e-8 and all(min(x, y) - 1e-8 <= v <= max(x, y) + 1e-8
                                            for v, x, y in zip(p, a, b))


def intersects(a, b, c, d):
    if any((on_segment(p, x, y)) for p, x, y in ((a, c, d), (b, c, d), (c, a, b), (d, a, b))):
        return True
    return cross(a, b, c) * cross(a, b, d) < 0 and cross(c, d, a) * cross(c, d, b) < 0


def inside(point, polygon):
    result = False
    x, y = point
    for a, b in zip(polygon, [*polygon[1:], polygon[0]]):
        if on_segment(point, a, b):
            return True
        if (a[1] > y) != (b[1] > y) and x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]:
            result = not result
    return result


def edges(polygon):
    return zip(polygon, [*polygon[1:], polygon[0]])


def overlaps(a, b):
    return (any(intersects(p, q, r, s) for p, q in edges(a) for r, s in edges(b))
            or inside(a[0], b) or inside(b[0], a))


class VehicleGeometry:
    def __init__(self, paper_size, obstacles=(), target=None, *, width=15, length=19, margin=0.5,
                 center_boundary=False):
        values = [*paper_size, width, length, margin]
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise ValueError('차체·지도 치수는 유한한 cm 숫자여야 합니다.')
        if min(*paper_size, width, length) <= 0 or margin < 0:
            raise ValueError('차체·지도 치수는 양수, 여유 폭은 0 이상이어야 합니다.')
        self.paper_size, self.obstacles, self.target = paper_size, list(obstacles), target
        self.half_length, self.half_width = length / 2 + margin, width / 2 + margin
        self.center_boundary = center_boundary
        self.edge_warning = None

    def rectangle(self, x, y, heading):
        angle = math.radians(heading)
        forward = math.cos(angle), math.sin(angle)
        right = -forward[1], forward[0]
        return [(x + a * forward[0] + b * right[0], y + a * forward[1] + b * right[1])
                for a, b in ((self.half_length, self.half_width), (self.half_length, -self.half_width),
                             (-self.half_length, -self.half_width), (-self.half_length, self.half_width))]

    def check(self, point, heading, *, parked=False):
        box = self.rectangle(*point, heading)
        xs, ys = zip(*box)
        width, height = self.paper_size
        excess = [('왼쪽', -min(xs)), ('오른쪽', max(xs) - width),
                  ('위쪽', -min(ys)), ('아래쪽', max(ys) - height)]
        outside = ', '.join(f'{side} {cm:.2f}cm' if cm >= 0.01 else f'{side} 0.01cm 미만'
                            for side, cm in excess if cm > 0)
        if outside:
            message = f'차체 여유 외곽이 전지 {outside} 초과 (중심 x={point[0]:.1f}, y={point[1]:.1f}cm)'
            if not self.center_boundary:
                raise ValueError(message)
            if not (0 <= point[0] <= width and 0 <= point[1] <= height):
                raise ValueError(f'로버 중심이 전지 밖입니다. {message}')
            if self.edge_warning is None:
                self.edge_warning = message
        for label, polygon in self.obstacles:
            if overlaps(box, polygon):
                raise ValueError(f'차체·회전 공간이 {label} 외곽과 겹칩니다.')
        if parked and self.target is not None and not inside(point, self.target):
            raise ValueError('로버 중심이 목표 주차칸 밖에 있습니다.')

    def pivot(self, point, start, delta):
        # 표본 사이 회전 궤적은 기본 0.5cm 여유 폭으로 함께 제한한다.
        count = max(1, math.ceil(abs(delta) / 2))
        for i in range(count + 1):
            self.check(point, start + delta * i / count)

    def line(self, start, end, heading):
        count = max(1, math.ceil(math.dist(start, end) / 0.25))
        for i in range(count + 1):
            point = [a + (b - a) * i / count for a, b in zip(start, end)]
            self.check(point, heading)

    def path(self, points, initial_heading, final_heading):
        heading = initial_heading
        for segment, (start, end) in enumerate(zip(points, points[1:]), start=1):
            desired = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
            delta = (desired - heading + 180) % 360 - 180
            try:
                self.pivot(start, heading, delta)
                self.line(start, end, desired)
            except ValueError as error:
                raise ValueError(f'계획 경로 {segment}번 구간 검사 중: {error}') from error
            heading = desired
        try:
            self.pivot(points[-1], heading, (final_heading - heading + 180) % 360 - 180)
            self.check(points[-1], final_heading, parked=True)
        except ValueError as error:
            raise ValueError(f'목표 칸 최종 자세 검사 중: {error}') from error

    def action(self, pose, action, cm_per_pulse, track_width):
        point = (pose['x_cm'], pose['y_cm'])
        heading = pose['heading_deg']
        mode, pulses = action
        if mode == 1:
            # 시작 펄스 위상과 제동 오차를 1펄스＋1cm 여유로 함께 검사한다.
            distance = (pulses + 1) * cm_per_pulse + 1.0
            angle = math.radians(heading)
            end = (point[0] + distance * math.cos(angle), point[1] + distance * math.sin(angle))
            self.line(point, end, heading)
        else:
            delta = math.degrees(2 * (pulses + 1) * cm_per_pulse / track_width) + 5
            self.pivot(point, heading, delta if mode == 2 else -delta)
