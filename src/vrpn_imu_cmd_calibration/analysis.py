"""Pure numerical analysis for the short UGV calibration run.

The ROS bag adapter lives in ``scripts/analyze_calibration_bag.py``. Keeping
this module ROS-free makes the estimators testable with synthetic arrays.
Times in the arrays are bag receive times from one recorder clock; the module
does not attempt to identify Motive transport delay.
"""

from __future__ import annotations

import copy
import math
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

import numpy as np


DEFAULT_PRIORS = {
    "accel_noise_std": 0.35,
    "gyro_noise_std": 0.03,
    "vrpn_position_noise_std": 0.01,
    "vrpn_orientation_noise_std": 0.01,
    "gyro_bias_random_walk_std": 1.0e-4,
    "accel_bias_random_walk_std": 1.0e-3,
}


def wrap(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def circular_mean(values: Sequence[float]) -> float:
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        raise ValueError("cannot take circular mean of an empty array")
    return math.atan2(float(np.sin(values).mean()), float(np.cos(values).mean()))


def circular_residuals(values: Sequence[float], center: float) -> np.ndarray:
    return np.asarray([wrap(float(value) - center) for value in values], dtype=float)


def robust_sigma(values: Sequence[float]) -> float:
    values = np.asarray(values, dtype=float)
    if values.size < 3:
        return float("nan")
    center = float(np.median(values))
    return 1.4826 * float(np.median(np.abs(values - center)))


def detrended_sigma(stamps: Sequence[float], values: Sequence[float]) -> float:
    stamps = np.asarray(stamps, dtype=float)
    values = np.asarray(values, dtype=float)
    if values.size < 5:
        return float("nan")
    centered_t = stamps - float(np.median(stamps))
    design = np.column_stack((centered_t, np.ones_like(centered_t)))
    slope, intercept = np.linalg.lstsq(design, values, rcond=None)[0]
    return robust_sigma(values - (slope * centered_t + intercept))


def sample_period(stamps: Sequence[float]) -> float:
    stamps = np.asarray(stamps, dtype=float)
    differences = np.diff(stamps)
    differences = differences[np.isfinite(differences) & (differences > 1.0e-5)]
    if differences.size < 3:
        raise ValueError("not enough increasing samples to determine sample period")
    return float(np.median(differences))


def _rows_between(array: np.ndarray, start: float, end: float) -> np.ndarray:
    return array[(array[:, 0] >= start) & (array[:, 0] <= end)]


def _edge_median(array: np.ndarray, start: float, end: float, at_start: bool) -> np.ndarray:
    duration = max(0.0, end - start)
    window = min(0.35, max(0.10, duration * 0.12))
    if at_start:
        rows = _rows_between(array, start, start + window)
    else:
        rows = _rows_between(array, end - window, end)
    if rows.shape[0] < 3:
        raise ValueError("too few samples near phase edge")
    return np.median(rows[:, 1:], axis=0)


def _line_heading_separation(headings: Sequence[float]) -> float:
    best = 0.0
    for index, first in enumerate(headings):
        for second in headings[index + 1 :]:
            difference = abs(wrap(first - second))
            difference = min(difference, abs(math.pi - difference))
            best = max(best, difference)
    return best


def _phases_of_kind(phases: Iterable[Mapping[str, Any]], kind: str) -> List[Mapping[str, Any]]:
    return [phase for phase in phases if phase.get("kind") == kind]


def _rest_blocks(phases: Iterable[Mapping[str, Any]]) -> List[Mapping[str, Any]]:
    return [
        phase
        for phase in phases
        if phase.get("kind") == "rest"
        and phase.get("name") in ("rest_start", "rest_end")
    ]


def _estimate_t_cm(
    pose: np.ndarray, phases: Sequence[Mapping[str, Any]], settings: Mapping[str, Any]
) -> Dict[str, Any]:
    candidates: List[float] = []
    body_headings: List[float] = []
    legs: List[Dict[str, float]] = []
    minimum_distance = float(settings.get("min_line_displacement_m", 0.65))

    for phase in _phases_of_kind(phases, "line"):
        start, end = float(phase["start"]), float(phase["end"])
        rows = _rows_between(pose, start, end)
        if rows.shape[0] < 8:
            continue
        p0 = _edge_median(pose[:, :4], start, end, True)[:2]
        p1 = _edge_median(pose[:, :4], start, end, False)[:2]
        displacement = p1 - p0
        distance = float(np.linalg.norm(displacement))
        if distance < minimum_distance:
            continue
        path_bearing = math.atan2(float(displacement[1]), float(displacement[0]))
        direction = 1.0 if float(phase.get("direction", 1.0)) >= 0.0 else -1.0
        body_heading = path_bearing if direction > 0.0 else wrap(path_bearing + math.pi)
        marker_yaw = circular_mean(rows[:, 6])
        candidate = wrap(marker_yaw - body_heading)
        candidates.append(candidate)
        body_headings.append(body_heading)
        legs.append(
            {
                "distance_m": distance,
                "body_heading_rad": body_heading,
                "marker_yaw_rad": marker_yaw,
                "t_cm_yaw_rad": candidate,
            }
        )

    if len(candidates) < 4:
        return {"passed": False, "reason": "fewer than four usable line legs", "legs": legs}
    estimate = circular_mean(candidates)
    residuals = np.abs(circular_residuals(candidates, estimate))
    spread = float(np.max(residuals))
    separation = _line_heading_separation(body_headings)
    passed = (
        separation >= math.radians(float(settings.get("min_heading_separation_deg", 20.0)))
        and spread <= math.radians(float(settings.get("max_t_cm_spread_deg", 3.0)))
    )
    return {
        "passed": passed,
        "yaw_rad": estimate,
        "yaw_deg": math.degrees(estimate),
        "max_residual_rad": spread,
        "max_residual_deg": math.degrees(spread),
        "heading_separation_rad": separation,
        "heading_separation_deg": math.degrees(separation),
        "legs": legs,
        "reason": "ok" if passed else "line-heading separation or candidate spread failed",
    }


def _static_imu_bias(imu: np.ndarray, rest: Sequence[Mapping[str, Any]]) -> np.ndarray:
    blocks = [_rows_between(imu, float(p["start"]), float(p["end"])) for p in rest]
    blocks = [block for block in blocks if block.shape[0] >= 20]
    if not blocks:
        raise ValueError("no usable long static IMU block")
    return np.median(np.vstack(blocks)[:, 1:7], axis=0)


def _estimate_t_ci(
    imu: np.ndarray,
    phases: Sequence[Mapping[str, Any]],
    static_bias: np.ndarray,
    settings: Mapping[str, Any],
) -> Dict[str, Any]:
    candidates: List[float] = []
    impulses: List[Dict[str, float]] = []
    minimum_impulse = float(settings.get("min_imu_impulse_mps", 0.05))
    for phase in _phases_of_kind(phases, "line"):
        start, end = float(phase["start"]), float(phase["end"])
        impulse_end = min(end, start + 0.90)
        rows = _rows_between(imu, start, impulse_end)
        if rows.shape[0] < 8:
            continue
        acceleration = rows[:, 1:3] - static_bias[:2]
        delta_v = np.trapz(acceleration, rows[:, 0], axis=0)
        direction = 1.0 if float(phase.get("direction", 1.0)) >= 0.0 else -1.0
        canonical = delta_v * direction
        magnitude = float(np.linalg.norm(canonical))
        if magnitude < minimum_impulse:
            continue
        # A physical +C.x acceleration has I-frame bearing -yaw(T_CI).
        candidate = wrap(-math.atan2(float(canonical[1]), float(canonical[0])))
        candidates.append(candidate)
        impulses.append(
            {
                "delta_v_i_x_mps": float(delta_v[0]),
                "delta_v_i_y_mps": float(delta_v[1]),
                "magnitude_mps": magnitude,
                "t_ci_yaw_rad": candidate,
            }
        )
    if len(candidates) < 4:
        return {
            "passed": False,
            "installation_compatible": False,
            "reason": "fewer than four usable IMU start impulses",
            "impulses": impulses,
        }
    estimate = circular_mean(candidates)
    spread = float(np.max(np.abs(circular_residuals(candidates, estimate))))
    statistical_pass = spread <= math.radians(float(settings.get("max_t_ci_spread_deg", 15.0)))
    installation_compatible = abs(estimate) <= math.radians(
        float(settings.get("max_abs_t_ci_yaw_deg", 10.0))
    )
    return {
        "passed": statistical_pass,
        "installation_compatible": installation_compatible,
        "yaw_rad": estimate,
        "yaw_deg": math.degrees(estimate),
        "max_residual_rad": spread,
        "max_residual_deg": math.degrees(spread),
        "impulses": impulses,
        "reason": "ok" if statistical_pass else "IMU impulse directions are inconsistent",
    }


def _estimate_gyro(
    imu: np.ndarray,
    pose: np.ndarray,
    phases: Sequence[Mapping[str, Any]],
    static_bias: np.ndarray,
    settings: Mapping[str, Any],
) -> Dict[str, Any]:
    candidates: List[float] = []
    turns: List[Dict[str, float]] = []
    turn_signs = set()
    rates = set()
    minimum_rotation = math.radians(float(settings.get("min_rotation_deg", 20.0)))
    bias_z = float(static_bias[5])

    for phase in _phases_of_kind(phases, "rotate"):
        start, end = float(phase["start"]), float(phase["end"])
        imu_rows = _rows_between(imu, start, end)
        pose_rows = _rows_between(pose, start, end)
        if imu_rows.shape[0] < 8 or pose_rows.shape[0] < 5:
            continue
        pose_yaw = np.unwrap(pose_rows[:, 6])
        gyro_time = imu_rows[:, 0]
        gyro_rate = imu_rows[:, 6] - bias_z
        increments = 0.5 * (gyro_rate[1:] + gyro_rate[:-1]) * np.diff(gyro_time)
        cumulative_gyro = np.concatenate(([0.0], np.cumsum(increments)))
        gyro_at_pose = np.interp(pose_rows[:, 0], gyro_time, cumulative_gyro)
        regression = np.column_stack((gyro_at_pose, np.ones_like(gyro_at_pose)))
        scale, _offset = np.linalg.lstsq(regression, pose_yaw, rcond=None)[0]
        pose_delta = float(pose_yaw[-1] - pose_yaw[0])
        gyro_integral = float(gyro_at_pose[-1] - gyro_at_pose[0])
        if abs(pose_delta) < minimum_rotation or abs(gyro_integral) < minimum_rotation * 0.5:
            continue
        scale = float(scale)
        candidates.append(scale)
        commanded_rate = float(phase.get("rate_rps", 0.0))
        turn_signs.add(1 if commanded_rate >= 0.0 else -1)
        rates.add(round(abs(commanded_rate), 2))
        turns.append(
            {
                "pose_delta_yaw_rad": pose_delta,
                "gyro_integral_rad": gyro_integral,
                "correction_scale": scale,
                "commanded_rate_rps": commanded_rate,
            }
        )
    if len(candidates) < 4:
        return {
            "passed": False,
            "reason": "fewer than four usable rotations",
            "bias_z_rad_s": bias_z,
            "turns": turns,
        }
    scale = float(np.median(candidates))
    spread = float(np.max(np.abs(np.asarray(candidates) - scale)))
    passed = (
        float(settings.get("min_gyro_scale", 0.85)) <= scale
        <= float(settings.get("max_gyro_scale", 1.15))
        and spread <= float(settings.get("max_gyro_scale_spread", 0.08))
        and turn_signs == {-1, 1}
        and len(rates) >= 2
    )
    return {
        "passed": passed,
        "bias_z_rad_s": bias_z,
        "correction_scale": scale,
        "raw_sign": 1 if scale >= 0.0 else -1,
        "max_scale_residual": spread,
        "turns": turns,
        "reason": "ok" if passed else "gyro sign, scale, rate coverage, or repeatability failed",
    }


def _block_sigmas(
    array: np.ndarray,
    blocks: Sequence[Mapping[str, Any]],
    columns: Sequence[int],
    angular: bool = False,
) -> List[List[float]]:
    output: List[List[float]] = []
    for phase in blocks:
        rows = _rows_between(array, float(phase["start"]), float(phase["end"]))
        if rows.shape[0] < 20:
            continue
        output.append(
            [
                detrended_sigma(
                    rows[:, 0], np.unwrap(rows[:, column]) if angular else rows[:, column]
                )
                for column in columns
            ]
        )
    return output


def _engineering_choice(raw: float, prior: float) -> float:
    """Choose a conservative nearby value from a tiny logarithmic grid."""
    target = max(0.0, float(raw)) * 2.0
    candidates = [prior * multiplier for multiplier in (0.5, 1.0, 2.0, 4.0)]
    for candidate in candidates:
        if candidate >= target:
            return float(candidate)
    return float(candidates[-1])


def _estimate_noise_and_walk(
    imu: np.ndarray,
    pose: np.ndarray,
    phases: Sequence[Mapping[str, Any]],
    priors: Mapping[str, float],
) -> Dict[str, Any]:
    rest = _rest_blocks(phases)
    if len(rest) < 2:
        return {"passed": False, "reason": "need rest_start and rest_end"}
    position_blocks = _block_sigmas(pose, rest, (1, 2, 3))
    orientation_blocks = _block_sigmas(pose, rest, (4, 5, 6), angular=True)
    accel_blocks = _block_sigmas(imu, rest, (1, 2, 3))
    gyro_blocks = _block_sigmas(imu, rest, (4, 5, 6))
    if not all((position_blocks, orientation_blocks, accel_blocks, gyro_blocks)):
        return {"passed": False, "reason": "static blocks contain too few samples"}

    static_imu = np.vstack(
        [_rows_between(imu, float(phase["start"]), float(phase["end"])) for phase in rest]
    )
    dt = sample_period(static_imu[:, 0])
    position_raw = float(np.nanmax(position_blocks))
    orientation_raw = float(np.nanmax(orientation_blocks))
    accel_sample_raw = float(np.nanmax(accel_blocks))
    gyro_sample_raw = float(np.nanmax(gyro_blocks))
    accel_density_raw = accel_sample_raw * math.sqrt(dt)
    gyro_density_raw = gyro_sample_raw * math.sqrt(dt)

    start_imu = _rows_between(imu, float(rest[0]["start"]), float(rest[0]["end"]))
    end_imu = _rows_between(imu, float(rest[-1]["start"]), float(rest[-1]["end"]))
    start_bias = np.median(start_imu[:, 1:7], axis=0)
    end_bias = np.median(end_imu[:, 1:7], axis=0)
    separation_s = max(1.0, float(np.median(end_imu[:, 0]) - np.median(start_imu[:, 0])))
    gyro_walk_raw = float(np.max(np.abs(end_bias[3:6] - start_bias[3:6]))) / math.sqrt(separation_s)
    accel_walk_raw = float(np.max(np.abs(end_bias[0:3] - start_bias[0:3]))) / math.sqrt(separation_s)

    raw = {
        "vrpn_position_noise_std": position_raw,
        "vrpn_orientation_noise_std": orientation_raw,
        "accel_noise_std": accel_density_raw,
        "gyro_noise_std": gyro_density_raw,
        "gyro_bias_random_walk_std": gyro_walk_raw,
        "accel_bias_random_walk_std": accel_walk_raw,
    }
    selected = {
        key: _engineering_choice(value, float(priors[key])) for key, value in raw.items()
    }
    return {
        "passed": all(np.isfinite(list(raw.values()))) and 0.001 <= dt <= 0.1,
        "reason": "ok",
        "sample_period_s": dt,
        "raw_short_run": raw,
        "selected": selected,
        "static_block_count": len(rest),
        "start_imu_median": [float(value) for value in start_bias],
        "end_imu_median": [float(value) for value in end_bias],
        "bias_block_separation_s": separation_s,
    }


def analyze_samples(
    samples: Mapping[str, Any],
    settings: Mapping[str, Any] | None = None,
    priors: Mapping[str, float] | None = None,
) -> Dict[str, Any]:
    """Estimate priorities 1--6 from bag-adapted arrays.

    ``imu`` columns: t, ax, ay, az, gx, gy, gz.
    ``pose`` columns: t, x, y, z, roll, pitch, yaw.
    ``cmd`` columns: t, linear.x, angular.z (retained as evidence).
    """
    settings = dict(settings or {})
    merged_priors = dict(DEFAULT_PRIORS)
    if priors:
        merged_priors.update(
            {
                key: float(value)
                for key, value in priors.items()
                if key in merged_priors and math.isfinite(float(value)) and float(value) > 0.0
            }
        )
    imu = np.asarray(samples["imu"], dtype=float)
    pose = np.asarray(samples["pose"], dtype=float)
    cmd = np.asarray(samples.get("cmd", []), dtype=float)
    phases = sorted(samples["phases"], key=lambda phase: float(phase["start"]))
    if imu.ndim != 2 or imu.shape[1] != 7 or imu.shape[0] < 100:
        raise ValueError("IMU samples must have shape Nx7 and contain at least 100 rows")
    if pose.ndim != 2 or pose.shape[1] != 7 or pose.shape[0] < 50:
        raise ValueError("pose samples must have shape Nx7 and contain at least 50 rows")
    if cmd.size and (cmd.ndim != 2 or cmd.shape[1] != 3):
        raise ValueError("cmd samples must have shape Nx3")
    if len(phases) < 8:
        raise ValueError("phase evidence is missing or incomplete")
    if not np.all(np.isfinite(imu)) or not np.all(np.isfinite(pose)):
        raise ValueError("non-finite sensor sample")

    rest = _rest_blocks(phases)
    static_bias = _static_imu_bias(imu, rest)
    t_cm = _estimate_t_cm(pose, phases, settings)
    t_ci = _estimate_t_ci(imu, phases, static_bias, settings)
    gyro = _estimate_gyro(imu, pose, phases, static_bias, settings)
    stochastic = _estimate_noise_and_walk(imu, pose, phases, merged_priors)

    checks = {
        "t_cm_yaw": bool(t_cm.get("passed")),
        "gyro_z": bool(gyro.get("passed")),
        "t_ci_yaw": bool(t_ci.get("passed")),
        "t_ci_installation_compatible": bool(t_ci.get("installation_compatible")),
        "short_run_noise_and_bias_walk": bool(stochastic.get("passed")),
    }
    reasons = [key for key, passed in checks.items() if not passed]
    return {
        "status": "PASS" if not reasons else "FAIL",
        "scope": "short-run engineering estimate; not a sensor datasheet",
        "checks": checks,
        "failed_checks": reasons,
        "t_cm": t_cm,
        "t_ci": t_ci,
        "gyro_z": gyro,
        "stochastic": stochastic,
        "sample_counts": {"imu": int(imu.shape[0]), "pose": int(pose.shape[0]), "cmd": int(cmd.shape[0]) if cmd.size else 0},
    }


def _native(value: Any) -> Any:
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return [_native(item) for item in value.tolist()]
    if isinstance(value, dict):
        return {str(key): _native(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_native(item) for item in value]
    return value


def build_outputs(
    result: Mapping[str, Any],
    base_estimator: Mapping[str, Any] | None = None,
    metadata: Mapping[str, Any] | None = None,
) -> Tuple[Dict[str, Any], Dict[str, Any], Dict[str, Any]]:
    """Build the evidence asset plus estimator/controller ROS YAML files."""
    runtime_ready = result.get("status") == "PASS"
    estimator = copy.deepcopy(dict(base_estimator or {}))
    old_rpy = list(estimator.get("imu_to_vrpn_marker_rpy", [0.0, 0.0, 0.0]))
    if len(old_rpy) != 3:
        old_rpy = [0.0, 0.0, 0.0]
    old_xyz = list(estimator.get("imu_to_vrpn_marker_xyz", [0.0, 0.0, 0.0]))
    if len(old_xyz) != 3:
        old_xyz = [0.0, 0.0, 0.0]
    t_cm_yaw = float(result.get("t_cm", {}).get("yaw_rad", old_rpy[2]))
    selected = result.get("stochastic", {}).get("selected", {})

    estimator.update(
        {
            "imu_to_vrpn_marker_xyz": [float(value) for value in old_xyz],
            "imu_to_vrpn_marker_rpy": [float(old_rpy[0]), float(old_rpy[1]), t_cm_yaw],
            "extrinsic_verified": bool(runtime_ready),
            "estimate_extrinsic": False,
            "imu_noise_std_is_density": True,
        }
    )
    for key in DEFAULT_PRIORS:
        if key in selected:
            estimator[key] = float(selected[key])

    controller = {
        "state_source": "state_estimator",
        "state_estimate_topic": "alg/state_estimator/state",
    }
    asset = {
        "schema_version": "xgc2.vrpn_imu_cmd_calibration/result/v1",
        "status": result.get("status", "FAIL"),
        "scope": result.get("scope"),
        "frames": {
            "C": "cmd_vel FLU control frame; estimator/controller output target",
            "I": "raw IMU sensor frame",
            "M": "VRPN marker rigid-body frame",
            "relation": "T_IM = T_CM compose T_CI",
        },
        "metadata": dict(metadata or {}),
        "quality": {
            "checks": result.get("checks", {}),
            "failed_checks": result.get("failed_checks", []),
            "runtime_yaml_ready": bool(runtime_ready),
        },
        "estimates": {
            "T_CM": {
                "xyz_m": [float(value) for value in old_xyz],
                "rpy_rad": [float(old_rpy[0]), float(old_rpy[1]), t_cm_yaw],
                "observable": [False, False, False, False, False, True],
                "details": result.get("t_cm", {}),
            },
            "T_CI": {
                "rpy_rad": [0.0, 0.0, result.get("t_ci", {}).get("yaw_rad")],
                "observable": [False, False, False, False, False, True],
                "runtime_note": "reported separately; current estimator has no T_CI input",
                "details": result.get("t_ci", {}),
            },
            "gyro_z": result.get("gyro_z", {}),
            "stochastic": result.get("stochastic", {}),
        },
        "integration": {
            "estimator_yaml": "estimator.yaml",
            "controller_yaml": "controller.yaml",
            "controller_state_frame": "C",
            "controller_direct_vrpn_bypass": False,
        },
        "sample_counts": result.get("sample_counts", {}),
    }
    return _native(asset), _native(estimator), _native(controller)
