#!/usr/bin/env python3

import math
import unittest

import numpy as np

from vrpn_imu_cmd_calibration.analysis import analyze_samples, build_outputs, wrap


def synthetic_run(t_ci_yaw=math.radians(5.0)):
    phases = []
    cursor = 0.0

    def add(name, kind, duration, **values):
        nonlocal cursor
        phase = {"sequence": len(phases) + 1, "name": name, "kind": kind, "start": cursor, "end": cursor + duration}
        phase.update(values)
        phases.append(phase)
        cursor += duration

    add("rest_start", "rest", 20.0)
    for heading in ("a",):
        for repeat in range(2):
            add("%s_fwd_%d" % (heading, repeat + 1), "line", 4.0, direction=1.0)
            add("settle", "rest", 1.0)
            add("%s_rev_%d" % (heading, repeat + 1), "line", 4.0, direction=-1.0)
            add("settle", "rest", 1.0)
    for name, angle, rate in (
        ("probe_slow_ccw", 45.0, 0.22),
        ("probe_slow_cw", -45.0, -0.22),
        ("probe_fast_cw", -45.0, -0.40),
        ("probe_fast_ccw", 45.0, 0.40),
        ("turn_to_b", 60.0, 0.22),
    ):
        add(name, "rotate", abs(math.radians(angle) / rate), rate_rps=rate, target_angle_rad=math.radians(angle))
        add("settle_" + name, "rest", 1.0)
    for repeat in range(2):
        add("b_fwd_%d" % (repeat + 1), "line", 4.0, direction=1.0)
        add("settle", "rest", 1.0)
        add("b_rev_%d" % (repeat + 1), "line", 4.0, direction=-1.0)
        add("settle", "rest", 1.0)
    add("turn_home", "rotate", math.radians(60.0) / 0.22, rate_rps=-0.22, target_angle_rad=-math.radians(60.0))
    add("settle_turn_home", "rest", 1.0)
    add("rest_end", "rest", 20.0)

    dt = 0.01
    stamps = np.arange(0.0, cursor, dt)
    x = np.zeros_like(stamps)
    y = np.zeros_like(stamps)
    yaw = np.zeros_like(stamps)
    accel_c = np.zeros_like(stamps)
    omega = np.zeros_like(stamps)
    command_v = np.zeros_like(stamps)
    command_w = np.zeros_like(stamps)
    state_x = 12.5
    state_y = -7.3
    state_yaw = 0.25

    for index, stamp in enumerate(stamps):
        phase = next(item for item in phases if item["start"] <= stamp < item["end"])
        local = stamp - phase["start"]
        v = 0.0
        a = 0.0
        w = 0.0
        if phase["kind"] == "line":
            direction = float(phase["direction"])
            if local < 0.8:
                v = direction * 0.35 * local / 0.8
                a = direction * 0.35 / 0.8
            elif local > 3.2:
                v = direction * 0.35 * max(0.0, (4.0 - local) / 0.8)
                a = -direction * 0.35 / 0.8
            else:
                v = direction * 0.35
        elif phase["kind"] == "rotate":
            w = float(phase["target_angle_rad"]) / (phase["end"] - phase["start"])
        state_x += math.cos(state_yaw) * v * dt
        state_y += math.sin(state_yaw) * v * dt
        state_yaw += w * dt
        x[index], y[index], yaw[index] = state_x, state_y, state_yaw
        accel_c[index], omega[index] = a, w
        command_v[index], command_w[index] = v, w

    rng = np.random.default_rng(7)
    c, s = math.cos(-t_ci_yaw), math.sin(-t_ci_yaw)
    accel_i_x = c * accel_c + 0.02 + rng.normal(0.0, 0.025, stamps.size)
    accel_i_y = s * accel_c - 0.01 + rng.normal(0.0, 0.025, stamps.size)
    accel_i_z = 9.8066 + 0.03 + rng.normal(0.0, 0.03, stamps.size)
    gyro_scale = 1.02
    gyro_z = omega / gyro_scale + 0.006 + rng.normal(0.0, 0.003, stamps.size)
    imu = np.column_stack(
        (
            stamps,
            accel_i_x,
            accel_i_y,
            accel_i_z,
            rng.normal(0.0, 0.002, stamps.size),
            rng.normal(0.0, 0.002, stamps.size),
            gyro_z,
        )
    )

    pose_indices = np.arange(0, stamps.size, 3)
    pose_t = stamps[pose_indices]
    t_cm_yaw = math.radians(93.0)
    pose = np.column_stack(
        (
            pose_t,
            x[pose_indices] + rng.normal(0.0, 0.002, pose_t.size),
            y[pose_indices] + rng.normal(0.0, 0.002, pose_t.size),
            rng.normal(0.0, 0.001, pose_t.size),
            rng.normal(0.0, 0.0008, pose_t.size),
            rng.normal(0.0, 0.0008, pose_t.size),
            np.asarray([wrap(value + t_cm_yaw) for value in yaw[pose_indices]])
            + rng.normal(0.0, 0.0008, pose_t.size),
        )
    )
    cmd_indices = np.arange(0, stamps.size, 5)
    cmd = np.column_stack((stamps[cmd_indices], command_v[cmd_indices], command_w[cmd_indices]))
    return {"imu": imu, "pose": pose, "cmd": cmd, "phases": phases}, t_cm_yaw, gyro_scale


class AnalysisTest(unittest.TestCase):
    def test_priorities_one_through_six_pass(self):
        samples, expected_t_cm, expected_gyro_scale = synthetic_run()
        result = analyze_samples(samples)
        self.assertEqual(result["status"], "PASS", result["failed_checks"])
        self.assertAlmostEqual(wrap(result["t_cm"]["yaw_rad"] - expected_t_cm), 0.0, delta=math.radians(0.8))
        self.assertAlmostEqual(result["gyro_z"]["correction_scale"], expected_gyro_scale, delta=0.02)
        self.assertAlmostEqual(result["t_ci"]["yaw_deg"], 5.0, delta=2.0)
        self.assertTrue(result["stochastic"]["passed"])
        self.assertEqual(set(result["stochastic"]["selected"]), {
            "vrpn_position_noise_std",
            "vrpn_orientation_noise_std",
            "accel_noise_std",
            "gyro_noise_std",
            "gyro_bias_random_walk_std",
            "accel_bias_random_walk_std",
        })

    def test_large_t_ci_yaw_blocks_runtime_asset(self):
        samples, _expected_t_cm, _expected_scale = synthetic_run(math.radians(22.0))
        result = analyze_samples(samples)
        self.assertEqual(result["status"], "FAIL")
        self.assertFalse(result["checks"]["t_ci_installation_compatible"])
        asset, estimator, controller = build_outputs(result)
        self.assertFalse(asset["quality"]["runtime_yaml_ready"])
        self.assertFalse(estimator["extrinsic_verified"])
        self.assertEqual(controller["state_source"], "state_estimator")


if __name__ == "__main__":
    unittest.main()
