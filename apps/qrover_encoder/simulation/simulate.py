"""Deterministic 2-D differential-drive test of the packaged node follower.

The UNO Q dynamics and firmware control loop are modeled in Python. The model
reads the shipped sketch's tuning values. It does not emulate STM32 hardware.
"""
import argparse
from itertools import product
import json
import math
from pathlib import Path
import random
import re
from statistics import median
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from node_follower import CM_PER_PULSE, GOAL_CM, PathGuard, follow, load_route, plan, settled_pose, simplify_path, wrap
from receiver_source import read_receiver

SKETCH = read_receiver("sketch/sketch.ino")


def tuning(key):
    return float(re.search(rf"\b{key}\s*=\s*([0-9.]+)", SKETCH).group(1))


FORWARD_PWM, TURN_PWM, FINE_PWM, MIN_PWM, MAX_PWM, KP = map(tuning, (
    "FORWARD_PWM", "TURN_PWM", "FINE_TURN_PWM", "MIN_PWM", "MAX_PWM", "KP"))
DT = tuning("CONTROL_MS") / 1000
STALL_S = tuning("STALL_MS") / 1000
PHASES = list(product((0.02, 0.22, 0.42, 0.62, 0.82), repeat=2))


class SimulationFault(RuntimeError):
    pass


class RoverModel:
    def __init__(self, width=15, phase=(0.2, 0.6), *, x=20, y=30, heading=0,
                 kp=KP, drift=False, brake_tau=0.13, noise=0, seed=0,
                 fine_pwm=FINE_PWM, fault=None, record=False):
        self.width, self.kp, self.drift = width, kp, drift
        self.brake_tau, self.noise, self.fine_pwm = brake_tau, noise, fine_pwm
        self.rng = random.Random(seed)
        self.x, self.y, self.heading = x, y, heading
        self.wheel_distance = [p * CM_PER_PULSE for p in phase]
        self.speed = [0.0, 0.0]
        self.time = 0.0
        self.jobs = []
        self.fault = fault
        self.trace = [] if record else None
        self.sample()

    def sample(self):
        if self.trace is not None:
            self.trace.append([round(v, 3) for v in (self.time, self.x, self.y, self.heading)])

    def counts(self):
        return [int(value / CM_PER_PULSE) for value in self.wheel_distance]

    def advance(self, pwm, direction=(1, 1)):
        gain_left = 0.82 if self.drift and (8 <= self.x < 22 or 49 <= self.x < 68) else 1
        wheel_deltas = []
        for i, gain in enumerate((gain_left, 1)):
            target = direction[i] * max(0, pwm[i] - 19) * 0.42 * gain
            tau = 0.075 if pwm[i] else self.brake_tau
            self.speed[i] += (target - self.speed[i]) * (1 - math.exp(-DT / tau))
            delta = self.speed[i] * DT
            self.wheel_distance[i] += abs(delta)
            wheel_deltas.append(delta)
        left, right = wheel_deltas
        d_angle = (left - right) / self.width
        angle = math.radians(self.heading) + d_angle / 2
        distance = (left + right) / 2
        self.x += distance * math.cos(angle)
        self.y += distance * math.sin(angle)
        self.heading = wrap(self.heading + math.degrees(d_angle))
        self.time += DT
        if round(self.time / DT) % 5 == 0:
            self.sample()

    def pose(self):
        samples = [dict(x_cm=self.x + self.rng.uniform(-0.10, 0.10) * bool(self.noise),
                        y_cm=self.y + self.rng.uniform(-0.10, 0.10) * bool(self.noise),
                        heading_deg=wrap(self.heading + self.rng.uniform(-self.noise, self.noise)))
                   for _ in range(3)]
        return settled_pose(samples)

    def run_job(self, action, timed=False):
        mode, target = action
        self.jobs.append(action)
        direction = {1: (1, 1), 2: (1, -1), 3: (-1, 1)}[mode]
        initial = self.counts()
        previous = [0, 0]
        last_edge = [0.0, 0.0]
        stop_reason = None
        for tick in range(601):
            elapsed = tick * DT
            if self.fault and elapsed >= self.fault[1]:
                stop_reason = self.fault[0]
                break
            progress = [v - start for v, start in zip(self.counts(), initial)]
            if timed:
                if elapsed >= target * CM_PER_PULSE / ((FORWARD_PWM - 19) * 0.42):
                    break
                pwm = [FORWARD_PWM, FORWARD_PWM]
            else:
                for i in range(2):
                    if progress[i] != previous[i]:
                        last_edge[i] = elapsed
                previous = progress
                if any(v < target and elapsed - edge > STALL_S
                       for v, edge in zip(progress, last_edge)):
                    stop_reason = "MCU -9 (encoder stall)"
                    break
                if abs(progress[0] - progress[1]) > 3:
                    stop_reason = "MCU -10 (wheel mismatch)"
                    break
                if min(progress) >= target:
                    break
                base = FORWARD_PWM if mode == 1 and target - min(progress) > 2 else TURN_PWM
                limit = MAX_PWM
                if mode != 1 and target == 1:
                    base = limit = self.fine_pwm
                correction = self.kp * (progress[0] - progress[1])
                pwm = [0 if progress[i] >= target else min(limit, max(MIN_PWM, val))
                       for i, val in enumerate((base - correction, base + correction))]
            self.advance(pwm, direction)
        else:
            stop_reason = "MCU -6 (job timeout)"
        # The next plan follows post-brake settling plus three 10 Hz frames.
        for _ in range(round(0.60 / DT)):
            self.advance((0, 0))
        if stop_reason:
            raise SimulationFault(stop_reason)
        return self.pose()


def straight_cases():
    results = {}
    for name, timed, kp in (("timed", True, 0), ("encoder_no_p", False, 0), ("encoder_p", False, KP)):
        results[name] = {}
        for width in (12, 15):
            outcomes = []
            for phase in PHASES:
                model = RoverModel(width, phase, x=0, y=0, kp=kp, drift=True, record=True)
                for _ in range(100):
                    if model.x >= 100:
                        break
                    model.run_job((1, 5), timed)
                if model.x < 100:
                    raise SimulationFault("straight drive did not reach the 100 cm measurement line")
                samples = model.trace
                at_100 = next(point for point in samples if point[1] >= 100)
                # Absolute deviation at the same longitudinal coordinate.
                before = samples[samples.index(at_100) - 1]
                fraction = (100 - before[1]) / (at_100[1] - before[1])
                y = before[2] + fraction * (at_100[2] - before[2])
                heading = before[3] + fraction * wrap(at_100[3] - before[3])
                outcomes.append((abs(y), abs(heading)))
            results[name][str(width)] = {
                "runs": len(outcomes),
                "median_abs_drift_at_100cm": round(median(x for x, _ in outcomes), 3),
                "median_abs_heading_deg_at_100cm": round(median(y for _, y in outcomes), 3),
            }
    return results


def rotation_cases():
    report = {}
    for label, pwm, noise in (("low_pwm_exact_pose", FINE_PWM, 0),
                              ("regular_pwm_noisy_pose", TURN_PWM, 0.6),
                              ("low_pwm_noisy_pose", FINE_PWM, 0.6)):
        for width in (12, 15):
            settled_errors, reversals = [], []
            failures = []
            for i, (angle, phase) in enumerate(product((15.6, -15.6, 45, -90, 90, 175), PHASES)):
                model = RoverModel(width, phase, x=0, y=0, noise=noise, seed=i, fine_pwm=pwm)
                target = (100 * math.cos(math.radians(angle)), 100 * math.sin(math.radians(angle)))
                pose = model.pose()
                for turn_count in range(41):
                    action, _, _ = plan(pose, target, width)
                    if action[0] == 1:
                        true_target = math.degrees(math.atan2(target[1] - model.y, target[0] - model.x))
                        settled_errors.append(abs(wrap(true_target - model.heading)))
                        reversals.append(sum(a[0] != b[0] for a, b in zip(model.jobs, model.jobs[1:])))
                        break
                    if turn_count == 40:
                        failures.append({"target_deg": angle, "phase": phase})
                        break
                    pose = model.run_job(action)
            errors = settled_errors
            report[f"{label}_{width}cm"] = {
                "runs": len(errors) + len(failures), "reached": len(errors),
                "true_error_within_5deg": sum(error <= 5 for error in errors),
                "max_true_error_deg": round(max(errors), 3),
                "max_direction_reversals": max(reversals), "failures": failures,
            }
    return report


def route_cases():
    route = json.loads((ROOT / "nodes.example.json").read_text(encoding="utf-8"))
    nodes, _ = load_route(route)
    final_heading = 0  # Deliberately require a turn after the southbound approach.
    cases, traces = [], {}
    for width, phase in product((12, 15), PHASES):
        record = phase == PHASES[7]
        model = RoverModel(width, phase, drift=True, noise=0.6, seed=55, record=record)
        path = [(model.x, model.y), *nodes]
        targets = simplify_path(path)[1:]
        arrivals = []
        def log(line, **_):
            if line.endswith("job=None"):
                node = targets[len(arrivals)]
                arrivals.append(math.hypot(node[0] - model.x, node[1] - model.y))
        try:
            follow(path, final_heading, width, model.run_job, model.pose(), clock=lambda: model.time,
                   log=log, path_mode=True, check=PathGuard())
            angle = abs(wrap(model.heading - final_heading))
            cases.append((width, arrivals, angle))
        except (RuntimeError, TimeoutError) as error:
            cases.append((width, str(error), None))
        if record:
            traces[str(width)] = model.trace
    results = {}
    for width in (12, 15):
        completed = [(ds, angle) for w, ds, angle in cases if w == width and angle is not None]
        results[str(width)] = {
            "runs": len(PHASES), "completed": len(completed),
            "within_position_and_angle": sum(max(ds) <= GOAL_CM and angle <= 5 for ds, angle in completed),
            "max_node_error_cm": round(max(max(ds) for ds, _ in completed), 3) if completed else None,
            "max_final_angle_error_deg": round(max(angle for _, angle in completed), 3) if completed else None,
            "failures": [ds for w, ds, angle in cases if w == width and angle is None],
        }
    return results, {"nodes": nodes, "traces": traces, "final_heading_deg": final_heading}


def safety_cases():
    result = {}
    for name, trigger_after in (("MCU -4 (ToF obstacle)", 0.15),
                                ("MCU -5 (ToF stale)", 0.15),
                                ("MCU -3 (heartbeat expired)", 0.41),
                                ("ArUco lost", 0.25)):
        model = RoverModel(fault=(name, trigger_after))
        try:
            follow([(70, 30)], None, 15, model.run_job, model.pose(), clock=lambda: model.time, log=lambda *a, **k: None)
        except SimulationFault as error:
            result[name] = str(error) == name and model.jobs == [(1, 5)]
        else:
            result[name] = False
    return result


def validate_report(report):
    for width in ('12', '15'):
        routes = report['routes'][width]
        if (routes['runs'] <= 0 or routes['completed'] != routes['runs']
                or routes['within_position_and_angle'] != routes['runs']
                or routes['max_node_error_cm'] is None or not math.isfinite(routes['max_node_error_cm'])
                or routes['max_node_error_cm'] > GOAL_CM or routes['max_final_angle_error_deg'] is None
                or not math.isfinite(routes['max_final_angle_error_deg'])
                or routes['max_final_angle_error_deg'] > 5 or routes['failures']):
            raise AssertionError(f'{width}cm route acceptance failed: {routes}')
        rotation = report['rotation'][f'low_pwm_noisy_pose_{width}cm']
        if (rotation['runs'] <= 0 or rotation['true_error_within_5deg'] != rotation['runs']
                or not math.isfinite(rotation['max_true_error_deg'])
                or rotation['max_true_error_deg'] > 5 or rotation['failures']):
            raise AssertionError(f'{width}cm rotation acceptance failed')
    if not report['simulated_fault_injection'] or not all(report['simulated_fault_injection'].values()):
        raise AssertionError('a simulated fault did not stop the route')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    args.output.mkdir(exist_ok=True, parents=True)
    routes, replay = route_cases()
    report = {"straight": straight_cases(), "rotation": rotation_cases(),
              "routes": routes, "simulated_fault_injection": safety_cases()}
    (args.output / "results.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    (args.output / "replay_data.js").write_text(
        "const replayData = " + json.dumps(replay, separators=(",", ":")) + ";\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    validate_report(report)


if __name__ == "__main__":
    main()
