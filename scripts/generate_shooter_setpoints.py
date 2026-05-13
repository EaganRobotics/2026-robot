#!/usr/bin/env python3
"""Generate shooter setpoints by simulating ball trajectories.

This script numerically simulates a 2D projectile and finds a shooter RPM for
each hood angle that reaches the requested target distance/height. It then
selects the lowest-RPM valid solution for each distance and prints:

1. A readable summary table
2. Ready-to-paste Java lines for ShooterDistanceTable

The defaults are intentionally easy to override because the exact robot
geometry, wheel slip, and game piece aerodynamics are robot-specific.
"""

from __future__ import annotations

import argparse
import csv
import html
import math
import sys
import tkinter as tk
from tkinter import ttk
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable

FEET_TO_METERS = 0.3048
RPM_TO_RAD_PER_SEC = 2.0 * math.pi / 60.0
REFERENCE_SETPOINTS = [
    (5.0, 2400.0, 0.0),
    (6.0, 2500.0, 9.0),
    (8.0, 2600.0, 11.0),
    (9.0, 2650.0, 12.5),
    (10.0, 2700.0, 13.0),
    (11.0, 2700.0, 20.0),
    (12.0, 2700.0, 23.0),
    (13.0, 2750.0, 25.0),
    (15.0, 2850.0, 27.0),
    (18.0, 3200.0, 27.0),
]


def hood_to_launch_deg(hood_deg: float, config: "Config") -> float:
    hood_span = config.hood_max_deg - config.hood_min_deg
    if abs(hood_span) < 1e-9:
        return config.launch_angle_at_zero_hood_deg
    alpha = (hood_deg - config.hood_min_deg) / hood_span
    return config.launch_angle_at_zero_hood_deg + alpha * (
        config.launch_angle_at_max_hood_deg - config.launch_angle_at_zero_hood_deg
    )


@dataclass(frozen=True)
class TrajectoryResult:
    y_at_target_m: float | None
    time_at_target_s: float | None
    apex_m: float


@dataclass(frozen=True)
class CalibrationResult:
    config: "Config"
    rms_error_m: float
    mean_abs_error_m: float
    max_abs_error_m: float
    rpm_rms_error: float
    rpm_mean_abs_error: float
    rpm_max_abs_error: float
    objective_score: float
    sample_count: int


@dataclass(frozen=True)
class Candidate:
    distance_ft: float
    hood_deg: float
    launch_deg: float
    shooter_rpm: float
    release_speed_mps: float
    flight_time_s: float
    apex_m: float
    vertical_error_m: float
    rank_for_distance: int = 1
    source: str = "generated"


@dataclass(frozen=True)
class Config:
    release_height_m: float
    target_height_m: float
    target_x_offset_m: float
    wheel_radius_m: float
    muzzle_velocity_scale: float
    launch_angle_at_zero_hood_deg: float
    launch_angle_at_max_hood_deg: float
    compression_m: float
    hood_min_deg: float
    hood_max_deg: float
    hood_step_deg: float
    rpm_min: float
    rpm_max: float
    solver_tolerance_m: float
    simulation_dt_s: float
    max_time_s: float
    gravity_mps2: float
    drag_coefficient: float
    air_density_kgpm3: float
    ball_diameter_m: float
    ball_mass_kg: float
    lift_coefficient: float
    feeder_rpm: float
    candidate_sort_mode: str
    generation_mode: str
    distances_ft: list[float]

    @property
    def drag_area_m2(self) -> float:
        return math.pi * (self.ball_diameter_m * 0.5) ** 2

    @property
    def drag_enabled(self) -> bool:
        return (
            self.drag_coefficient > 0.0
            and self.air_density_kgpm3 > 0.0
            and self.ball_diameter_m > 0.0
            and self.ball_mass_kg > 0.0
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Pre-calculate shooter setpoints by numerically simulating the "
            "ball trajectory for a list of distances."
        )
    )
    parser.add_argument(
        "--distances-ft",
        default="5,6,8,9,10,11,12,13,15,18",
        help=(
            "Comma-separated list of shot distances in feet. Ignored if "
            "--distance-start-ft, --distance-end-ft, and --distance-count are provided."
        ),
    )
    parser.add_argument(
        "--distance-start-ft",
        type=float,
        default=None,
        help="Optional start of an evenly-sampled distance range in feet.",
    )
    parser.add_argument(
        "--distance-end-ft",
        type=float,
        default=None,
        help="Optional end of an evenly-sampled distance range in feet.",
    )
    parser.add_argument(
        "--distance-count",
        type=int,
        default=None,
        help="Optional number of evenly-sampled distances between start and end, inclusive.",
    )
    parser.add_argument(
        "--release-height-m",
        type=float,
        default=0.381,
        help="Launch height above the carpet in meters.",
    )
    parser.add_argument(
        "--target-height-m",
        type=float,
        default=1.8288,
        help="Target height above the carpet in meters. For REBUILT, 72 in is the HUB front edge.",
    )
    parser.add_argument(
        "--target-x-offset-m",
        type=float,
        default=0.0,
        help=(
            "Horizontal offset applied to each requested distance before simulation. "
            "Use a negative value if your distance is measured to the HUB center but "
            "you want to target a point closer to the front edge of the opening."
        ),
    )
    parser.add_argument(
        "--wheel-radius-m",
        type=float,
        default=0.0508,
        help="Shooter wheel radius in meters.",
    )
    parser.add_argument(
        "--muzzle-velocity-scale",
        type=float,
        default=1.0,
        help=(
            "Multiplier from wheel surface speed to actual ball exit speed. "
            "Use this to model slip/compression losses."
        ),
    )
    parser.add_argument(
        "--launch-angle-at-zero-hood-deg",
        type=float,
        default=80.0,
        help="Actual launch angle in degrees above flat when hood command is 0 deg.",
    )
    parser.add_argument(
        "--launch-angle-at-max-hood-deg",
        type=float,
        default=60.0,
        help="Actual launch angle in degrees above flat when hood command is at hood max.",
    )
    parser.add_argument(
        "--compression-m",
        type=float,
        default=0.0254,
        help=(
            "Approximate game piece compression through the shooter in meters. "
            "Currently informational only, but worth recording with generated data."
        ),
    )
    parser.add_argument("--hood-min-deg", type=float, default=0.0)
    parser.add_argument("--hood-max-deg", type=float, default=27.5)
    parser.add_argument("--hood-step-deg", type=float, default=0.25)
    parser.add_argument("--rpm-min", type=float, default=1000.0)
    parser.add_argument("--rpm-max", type=float, default=5000.0)
    parser.add_argument(
        "--solver-tolerance-m",
        type=float,
        default=0.01,
        help="Allowed vertical miss at the target x-position.",
    )
    parser.add_argument(
        "--simulation-dt-s",
        type=float,
        default=0.002,
        help="Integration timestep in seconds.",
    )
    parser.add_argument(
        "--max-time-s",
        type=float,
        default=5.0,
        help="Safety cutoff for each trajectory simulation.",
    )
    parser.add_argument("--gravity-mps2", type=float, default=9.81)
    parser.add_argument(
        "--drag-coefficient",
        type=float,
        default=0.0,
        help="Quadratic drag coefficient. Leave at 0.0 to ignore drag.",
    )
    parser.add_argument("--air-density-kgpm3", type=float, default=1.225)
    parser.add_argument("--ball-diameter-m", type=float, default=0.24)
    parser.add_argument("--ball-mass-kg", type=float, default=0.27)
    parser.add_argument(
        "--lift-coefficient",
        type=float,
        default=0.0,
        help=(
            "Simple upward acceleration fraction of gravity used as a first-pass "
            "stand-in for backspin lift. 0.0 disables it."
        ),
    )
    parser.add_argument(
        "--feeder-rpm",
        type=float,
        default=3000.0,
        help="Feeder RPM to emit in the generated Java setpoints.",
    )
    parser.add_argument(
        "--solutions-per-distance",
        type=int,
        default=1,
        help=(
            "How many candidate trajectories to keep per distance. 1 gives direct table.put(...) "
            "output, larger values emit grouped alternatives."
        ),
    )
    parser.add_argument(
        "--candidate-sort-mode",
        choices=["balanced", "low-rpm", "low-hood"],
        default="balanced",
        help=(
            "How to rank candidate trajectories at each distance. "
            "'balanced' avoids overly tall arcs and biases toward the trusted shot family, "
            "'low-rpm' minimizes RPM, and 'low-hood' minimizes hood angle."
        ),
    )
    parser.add_argument(
        "--generation-mode",
        choices=["calibrated-physics", "physics", "reference"],
        default="calibrated-physics",
        help=(
            "How to generate candidate setpoints. 'calibrated-physics' fits the physics model "
            "to the confirmed map first, then solves trajectories from that fitted model. "
            "'physics' uses the raw trajectory solver directly. 'reference' uses the confirmed "
            "map as a fallback/interpolation tool only."
        ),
    )
    parser.add_argument(
        "--csv-out",
        type=Path,
        default=None,
        help="Optional path to write a CSV summary.",
    )
    parser.add_argument(
        "--java-out",
        type=Path,
        default=None,
        help="Optional path to write Java table.put(...) lines.",
    )
    parser.add_argument(
        "--visual-out",
        type=Path,
        default=None,
        help="Optional path to write a standalone HTML trajectory visualization.",
    )
    parser.add_argument(
        "--visualize-gui",
        action="store_true",
        help="Open an interactive Tkinter GUI to compare generated and reference trajectories.",
    )
    return parser.parse_args()


def parse_distances_feet(text: str) -> list[float]:
    values: list[float] = []
    for chunk in text.split(","):
        stripped = chunk.strip()
        if not stripped:
            continue
        values.append(float(stripped))
    if not values:
        raise ValueError("At least one distance is required.")
    return values


def build_distance_range(start_ft: float, end_ft: float, count: int) -> list[float]:
    if count < 1:
        raise ValueError("--distance-count must be at least 1.")
    if count == 1:
        return [start_ft]
    step_ft = (end_ft - start_ft) / (count - 1)
    return [start_ft + step_ft * index for index in range(count)]


def interpolate_reference_setpoint(distance_ft: float) -> tuple[float, float] | None:
    if not REFERENCE_SETPOINTS:
        return None

    if distance_ft <= REFERENCE_SETPOINTS[0][0]:
        _, rpm, hood = REFERENCE_SETPOINTS[0]
        return rpm, hood
    if distance_ft >= REFERENCE_SETPOINTS[-1][0]:
        _, rpm, hood = REFERENCE_SETPOINTS[-1]
        return rpm, hood

    for index in range(len(REFERENCE_SETPOINTS) - 1):
        left_distance, left_rpm, left_hood = REFERENCE_SETPOINTS[index]
        right_distance, right_rpm, right_hood = REFERENCE_SETPOINTS[index + 1]
        if left_distance <= distance_ft <= right_distance:
            span = right_distance - left_distance
            if span <= 1e-9:
                return left_rpm, left_hood
            alpha = (distance_ft - left_distance) / span
            rpm = left_rpm + alpha * (right_rpm - left_rpm)
            hood = left_hood + alpha * (right_hood - left_hood)
            return rpm, hood

    return None


def get_neighbor_reference_setpoints(
    distance_ft: float,
) -> tuple[tuple[float, float, float] | None, tuple[float, float, float] | None]:
    if not REFERENCE_SETPOINTS:
        return None, None

    if distance_ft <= REFERENCE_SETPOINTS[0][0]:
        return REFERENCE_SETPOINTS[0], REFERENCE_SETPOINTS[1] if len(REFERENCE_SETPOINTS) > 1 else None
    if distance_ft >= REFERENCE_SETPOINTS[-1][0]:
        return (
            REFERENCE_SETPOINTS[-2] if len(REFERENCE_SETPOINTS) > 1 else None,
            REFERENCE_SETPOINTS[-1],
        )

    for index in range(len(REFERENCE_SETPOINTS) - 1):
        left = REFERENCE_SETPOINTS[index]
        right = REFERENCE_SETPOINTS[index + 1]
        if left[0] <= distance_ft <= right[0]:
            return left, right

    return None, None


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def reference_guidance_penalty(candidate: Candidate) -> float:
    reference = interpolate_reference_setpoint(candidate.distance_ft)
    if reference is None:
        return 0.0

    reference_rpm, reference_hood = reference
    hood_delta = abs(candidate.hood_deg - reference_hood) / 5.0
    rpm_delta = abs(candidate.shooter_rpm - reference_rpm) / 350.0
    return hood_delta + 0.8 * rpm_delta


def rpm_to_exit_speed_mps(rpm: float, config: Config) -> float:
    return rpm * RPM_TO_RAD_PER_SEC * config.wheel_radius_m * config.muzzle_velocity_scale


def simulate_to_distance(
    distance_m: float, launch_speed_mps: float, hood_deg: float, config: Config
) -> TrajectoryResult:
    theta = math.radians(hood_to_launch_deg(hood_deg, config))
    x = 0.0
    y = config.release_height_m
    vx = launch_speed_mps * math.cos(theta)
    vy = launch_speed_mps * math.sin(theta)
    t = 0.0
    apex_m = y
    previous = (x, y, t)

    while t < config.max_time_s:
        speed = math.hypot(vx, vy)
        ax = 0.0
        ay = -config.gravity_mps2 * (1.0 - config.lift_coefficient)

        if config.drag_enabled and speed > 1e-9:
            drag_force = (
                0.5
                * config.air_density_kgpm3
                * config.drag_coefficient
                * config.drag_area_m2
                * speed
                * speed
            )
            drag_accel = drag_force / config.ball_mass_kg
            ax -= drag_accel * (vx / speed)
            ay -= drag_accel * (vy / speed)

        vx += ax * config.simulation_dt_s
        vy += ay * config.simulation_dt_s
        x += vx * config.simulation_dt_s
        y += vy * config.simulation_dt_s
        t += config.simulation_dt_s
        apex_m = max(apex_m, y)

        if x >= distance_m:
            prev_x, prev_y, prev_t = previous
            segment_dx = x - prev_x
            alpha = 0.0 if abs(segment_dx) < 1e-9 else (distance_m - prev_x) / segment_dx
            y_at_target = prev_y + alpha * (y - prev_y)
            t_at_target = prev_t + alpha * (t - prev_t)
            return TrajectoryResult(y_at_target, t_at_target, apex_m)

        if y < 0.0:
            return TrajectoryResult(None, None, apex_m)

        previous = (x, y, t)

    return TrajectoryResult(None, None, apex_m)


def sample_trajectory_points(
    distance_m: float, launch_speed_mps: float, hood_deg: float, config: Config
) -> list[tuple[float, float]]:
    theta = math.radians(hood_to_launch_deg(hood_deg, config))
    x = 0.0
    y = config.release_height_m
    vx = launch_speed_mps * math.cos(theta)
    vy = launch_speed_mps * math.sin(theta)
    t = 0.0
    points: list[tuple[float, float]] = [(x, y)]

    while t < config.max_time_s:
        speed = math.hypot(vx, vy)
        ax = 0.0
        ay = -config.gravity_mps2 * (1.0 - config.lift_coefficient)

        if config.drag_enabled and speed > 1e-9:
            drag_force = (
                0.5
                * config.air_density_kgpm3
                * config.drag_coefficient
                * config.drag_area_m2
                * speed
                * speed
            )
            drag_accel = drag_force / config.ball_mass_kg
            ax -= drag_accel * (vx / speed)
            ay -= drag_accel * (vy / speed)

        vx += ax * config.simulation_dt_s
        vy += ay * config.simulation_dt_s
        x += vx * config.simulation_dt_s
        y += vy * config.simulation_dt_s
        t += config.simulation_dt_s
        points.append((x, y))

        if x >= distance_m or y < 0.0:
            break

    return points


def vertical_error_m(
    distance_m: float, rpm: float, hood_deg: float, config: Config
) -> float | None:
    modeled_distance_m = max(0.0, distance_m + config.target_x_offset_m)
    result = simulate_to_distance(
        modeled_distance_m, rpm_to_exit_speed_mps(rpm, config), hood_deg, config
    )
    if result.y_at_target_m is None:
        return None
    return result.y_at_target_m - config.target_height_m


def compute_reference_heights(config: Config) -> list[float] | None:
    heights: list[float] = []
    for distance_ft, shooter_rpm, hood_deg in REFERENCE_SETPOINTS:
        modeled_distance_m = max(0.0, distance_ft * FEET_TO_METERS + config.target_x_offset_m)
        result = simulate_to_distance(
            modeled_distance_m, rpm_to_exit_speed_mps(shooter_rpm, config), hood_deg, config
        )
        if result.y_at_target_m is None:
            return None
        heights.append(result.y_at_target_m)
    return heights


def evaluate_calibration_metrics(config: Config) -> CalibrationResult:
    vertical_errors: list[float] = []
    rpm_errors: list[float] = []

    for distance_ft, shooter_rpm, hood_deg in REFERENCE_SETPOINTS:
        distance_m = distance_ft * FEET_TO_METERS
        predicted_rpm = solve_no_drag_rpm(distance_m, hood_deg, config)
        measured_vertical_error = vertical_error_m(distance_m, shooter_rpm, hood_deg, config)

        if predicted_rpm is None or measured_vertical_error is None:
            return CalibrationResult(
                config=config,
                rms_error_m=999.0,
                mean_abs_error_m=999.0,
                max_abs_error_m=999.0,
                rpm_rms_error=999.0,
                rpm_mean_abs_error=999.0,
                rpm_max_abs_error=999.0,
                objective_score=999.0,
                sample_count=0,
            )

        vertical_errors.append(measured_vertical_error)
        rpm_errors.append(predicted_rpm - shooter_rpm)

    if not vertical_errors:
        return CalibrationResult(
            config=config,
            rms_error_m=999.0,
            mean_abs_error_m=999.0,
            max_abs_error_m=999.0,
            rpm_rms_error=999.0,
            rpm_mean_abs_error=999.0,
            rpm_max_abs_error=999.0,
            objective_score=999.0,
            sample_count=0,
        )

    mean_abs_error = sum(abs(error) for error in vertical_errors) / len(vertical_errors)
    rms_error = math.sqrt(sum(error * error for error in vertical_errors) / len(vertical_errors))
    max_abs_error = max(abs(error) for error in vertical_errors)
    rpm_mean_abs_error = sum(abs(error) for error in rpm_errors) / len(rpm_errors)
    rpm_rms_error = math.sqrt(sum(error * error for error in rpm_errors) / len(rpm_errors))
    rpm_max_abs_error = max(abs(error) for error in rpm_errors)

    prior_penalty = (
        0.10 * abs(config.muzzle_velocity_scale - 1.0) / 0.25
        + 0.06 * abs(config.target_x_offset_m) / 0.20
        + 0.04 * config.lift_coefficient / 0.25
    )
    objective_score = (
        rpm_rms_error / 180.0
        + 0.55 * (rpm_max_abs_error / 260.0)
        + rms_error / 0.10
        + 0.35 * (max_abs_error / 0.18)
        + prior_penalty
    )

    return CalibrationResult(
        config=config,
        rms_error_m=rms_error,
        mean_abs_error_m=mean_abs_error,
        max_abs_error_m=max_abs_error,
        rpm_rms_error=rpm_rms_error,
        rpm_mean_abs_error=rpm_mean_abs_error,
        rpm_max_abs_error=rpm_max_abs_error,
        objective_score=objective_score,
        sample_count=len(vertical_errors),
    )


def refine_calibration_seed(config: Config) -> CalibrationResult:
    current = evaluate_calibration_metrics(config)
    step_zero = 8.0
    step_max = 6.0
    step_scale = 0.08
    step_offset = 0.05
    step_lift = 0.03
    step_target_height = 0.05

    for _ in range(6):
        improved = True
        while improved:
            improved = False
            candidate_configs: list[Config] = []

            for delta in (-step_zero, step_zero):
                candidate_configs.append(
                    replace(
                        current.config,
                        launch_angle_at_zero_hood_deg=clamp(
                            current.config.launch_angle_at_zero_hood_deg + delta, 25.0, 85.0
                        ),
                    )
                )
            for delta in (-step_max, step_max):
                candidate_configs.append(
                    replace(
                        current.config,
                        launch_angle_at_max_hood_deg=clamp(
                            current.config.launch_angle_at_max_hood_deg + delta, 15.0, 75.0
                        ),
                    )
                )
            for delta in (-step_scale, step_scale):
                candidate_configs.append(
                    replace(
                        current.config,
                        muzzle_velocity_scale=clamp(
                            current.config.muzzle_velocity_scale + delta, 0.45, 1.20
                        ),
                    )
                )
            for delta in (-step_offset, step_offset):
                candidate_configs.append(
                    replace(
                        current.config,
                        target_x_offset_m=clamp(
                            current.config.target_x_offset_m + delta, -0.35, 0.35
                        ),
                    )
                )
            for delta in (-step_lift, step_lift):
                candidate_configs.append(
                    replace(
                        current.config,
                        lift_coefficient=clamp(
                            current.config.lift_coefficient + delta, 0.0, 0.35
                        ),
                    )
                )
            for delta in (-step_target_height, step_target_height):
                candidate_configs.append(
                    replace(
                        current.config,
                        target_height_m=clamp(
                            current.config.target_height_m + delta, 1.8288, 2.25
                        ),
                    )
                )

            for candidate_config in candidate_configs:
                if candidate_config.launch_angle_at_zero_hood_deg <= candidate_config.launch_angle_at_max_hood_deg + 2.0:
                    continue
                candidate = evaluate_calibration_metrics(candidate_config)
                if candidate.objective_score + 1e-9 < current.objective_score:
                    current = candidate
                    improved = True

        step_zero *= 0.5
        step_max *= 0.5
        step_scale *= 0.5
        step_offset *= 0.5
        step_lift *= 0.5
        step_target_height *= 0.5

    return current


def calibrate_model(config: Config) -> CalibrationResult:
    seed_configs = [
        config,
        replace(
            config,
            launch_angle_at_zero_hood_deg=55.0,
            launch_angle_at_max_hood_deg=35.0,
            muzzle_velocity_scale=1.0,
            target_x_offset_m=0.0,
            target_height_m=1.90,
            lift_coefficient=0.0,
        ),
        replace(
            config,
            launch_angle_at_zero_hood_deg=48.0,
            launch_angle_at_max_hood_deg=30.0,
            muzzle_velocity_scale=0.95,
            target_x_offset_m=0.0,
            target_height_m=1.95,
            lift_coefficient=0.02,
        ),
        replace(
            config,
            launch_angle_at_zero_hood_deg=42.0,
            launch_angle_at_max_hood_deg=26.0,
            muzzle_velocity_scale=0.90,
            target_x_offset_m=0.03,
            target_height_m=2.00,
            lift_coefficient=0.04,
        ),
    ]

    best_result = evaluate_calibration_metrics(seed_configs[0])
    for seed_config in seed_configs:
        if seed_config.launch_angle_at_zero_hood_deg <= seed_config.launch_angle_at_max_hood_deg + 2.0:
            continue
        result = refine_calibration_seed(seed_config)
        if result.objective_score + 1e-9 < best_result.objective_score:
            best_result = result
    return best_result


def solve_no_drag_rpm(distance_m: float, hood_deg: float, config: Config) -> float | None:
    distance_m = max(0.0, distance_m + config.target_x_offset_m)
    launch_deg = hood_to_launch_deg(hood_deg, config)
    theta = math.radians(launch_deg)
    cos_theta = math.cos(theta)
    if abs(cos_theta) < 1e-9:
        return None

    delta_height_m = config.target_height_m - config.release_height_m
    denominator = 2.0 * cos_theta * cos_theta * (distance_m * math.tan(theta) - delta_height_m)
    if denominator <= 0.0:
        return None

    effective_gravity = config.gravity_mps2 * (1.0 - config.lift_coefficient)
    launch_speed_squared = effective_gravity * distance_m * distance_m / denominator
    if launch_speed_squared <= 0.0:
        return None

    launch_speed_mps = math.sqrt(launch_speed_squared)
    surface_speed_per_rpm = RPM_TO_RAD_PER_SEC * config.wheel_radius_m * config.muzzle_velocity_scale
    if surface_speed_per_rpm <= 1e-9:
        return None

    return launch_speed_mps / surface_speed_per_rpm


def solve_rpm_for_angle(distance_m: float, hood_deg: float, config: Config) -> Candidate | None:
    modeled_distance_m = max(0.0, distance_m + config.target_x_offset_m)

    if not config.drag_enabled:
        analytic_rpm = solve_no_drag_rpm(distance_m, hood_deg, config)
        if analytic_rpm is None or analytic_rpm < config.rpm_min or analytic_rpm > config.rpm_max:
            return None

        final_speed_mps = rpm_to_exit_speed_mps(analytic_rpm, config)
        result = simulate_to_distance(modeled_distance_m, final_speed_mps, hood_deg, config)
        if result.y_at_target_m is None or result.time_at_target_s is None:
            return None

        final_error = result.y_at_target_m - config.target_height_m
        if abs(final_error) > config.solver_tolerance_m:
            return None

        return Candidate(
            distance_ft=distance_m / FEET_TO_METERS,
            hood_deg=hood_deg,
            launch_deg=hood_to_launch_deg(hood_deg, config),
            shooter_rpm=analytic_rpm,
            release_speed_mps=final_speed_mps,
            flight_time_s=result.time_at_target_s,
            apex_m=result.apex_m,
            vertical_error_m=final_error,
        )

    low_rpm = config.rpm_min
    high_rpm = config.rpm_max
    low_error = vertical_error_m(distance_m, low_rpm, hood_deg, config)
    high_error = vertical_error_m(distance_m, high_rpm, hood_deg, config)

    low_value = -1e9 if low_error is None else low_error
    high_value = -1e9 if high_error is None else high_error

    if low_value > config.solver_tolerance_m:
        return None
    if high_value < -config.solver_tolerance_m:
        return None

    for _ in range(50):
        mid_rpm = 0.5 * (low_rpm + high_rpm)
        mid_error = vertical_error_m(distance_m, mid_rpm, hood_deg, config)
        mid_value = -1e9 if mid_error is None else mid_error

        if mid_value < 0.0:
            low_rpm = mid_rpm
        else:
            high_rpm = mid_rpm

    final_rpm = high_rpm
    final_speed_mps = rpm_to_exit_speed_mps(final_rpm, config)
    result = simulate_to_distance(modeled_distance_m, final_speed_mps, hood_deg, config)
    if result.y_at_target_m is None or result.time_at_target_s is None:
        return None

    final_error = result.y_at_target_m - config.target_height_m
    if abs(final_error) > config.solver_tolerance_m:
        return None

    return Candidate(
        distance_ft=distance_m / FEET_TO_METERS,
        hood_deg=hood_deg,
        launch_deg=hood_to_launch_deg(hood_deg, config),
        shooter_rpm=final_rpm,
        release_speed_mps=final_speed_mps,
        flight_time_s=result.time_at_target_s,
        apex_m=result.apex_m,
        vertical_error_m=final_error,
    )


def candidate_sort_key(candidate: Candidate) -> tuple[float, float, float, float]:
    return (
        candidate.shooter_rpm,
        candidate.hood_deg,
        candidate.flight_time_s,
        candidate.apex_m,
    )


def score_candidates_balanced(candidates: list[Candidate]) -> list[Candidate]:
    if len(candidates) <= 1:
        return candidates

    min_rpm = min(candidate.shooter_rpm for candidate in candidates)
    max_rpm = max(candidate.shooter_rpm for candidate in candidates)
    min_hood = min(candidate.hood_deg for candidate in candidates)
    max_hood = max(candidate.hood_deg for candidate in candidates)
    min_apex = min(candidate.apex_m for candidate in candidates)
    max_apex = max(candidate.apex_m for candidate in candidates)
    min_time = min(candidate.flight_time_s for candidate in candidates)
    max_time = max(candidate.flight_time_s for candidate in candidates)

    def normalize(value: float, low: float, high: float) -> float:
        if abs(high - low) < 1e-9:
            return 0.0
        return (value - low) / (high - low)

    return sorted(
        candidates,
        key=lambda candidate: (
            0.40 * reference_guidance_penalty(candidate)
            + 0.25 * normalize(candidate.hood_deg, min_hood, max_hood)
            + 0.20 * normalize(candidate.shooter_rpm, min_rpm, max_rpm)
            + 0.10 * normalize(candidate.apex_m, min_apex, max_apex)
            + 0.05 * normalize(candidate.flight_time_s, min_time, max_time),
            candidate.hood_deg,
            candidate.shooter_rpm,
            candidate.flight_time_s,
        ),
    )


def frange(start: float, stop: float, step: float) -> Iterable[float]:
    value = start
    while value <= stop + 1e-9:
        yield round(value, 10)
        value += step


def generate_candidates_for_distance(distance_ft: float, config: Config) -> list[Candidate]:
    distance_m = distance_ft * FEET_TO_METERS
    candidates: list[Candidate] = []

    for hood_deg in frange(config.hood_min_deg, config.hood_max_deg, config.hood_step_deg):
        candidate = solve_rpm_for_angle(distance_m, hood_deg, config)
        if candidate is not None:
            candidates.append(candidate)

    if config.candidate_sort_mode == "low-rpm":
        candidates.sort(key=candidate_sort_key)
    elif config.candidate_sort_mode == "low-hood":
        candidates.sort(
            key=lambda candidate: (
                candidate.hood_deg,
                candidate.shooter_rpm,
                candidate.flight_time_s,
                candidate.apex_m,
            )
        )
    else:
        candidates = score_candidates_balanced(candidates)

    return candidates


def make_reference_candidate(
    distance_ft: float,
    shooter_rpm: float,
    hood_deg: float,
    config: Config,
    rank_for_distance: int,
    source: str,
) -> Candidate:
    modeled_distance_m = max(0.0, distance_ft * FEET_TO_METERS + config.target_x_offset_m)
    release_speed_mps = rpm_to_exit_speed_mps(shooter_rpm, config)
    result = simulate_to_distance(modeled_distance_m, release_speed_mps, hood_deg, config)
    y_at_target = result.y_at_target_m if result.y_at_target_m is not None else float("nan")
    vertical_error = (
        y_at_target - config.target_height_m
        if result.y_at_target_m is not None
        else float("nan")
    )
    return Candidate(
        distance_ft=distance_ft,
        hood_deg=hood_deg,
        launch_deg=hood_to_launch_deg(hood_deg, config),
        shooter_rpm=shooter_rpm,
        release_speed_mps=release_speed_mps,
        flight_time_s=result.time_at_target_s if result.time_at_target_s is not None else float("nan"),
        apex_m=result.apex_m,
        vertical_error_m=vertical_error,
        rank_for_distance=rank_for_distance,
        source=source,
    )


def generate_reference_candidates_for_distance(
    distance_ft: float, config: Config, solutions_per_distance: int
) -> list[Candidate]:
    base = interpolate_reference_setpoint(distance_ft)
    if base is None:
        return []

    base_rpm, base_hood = base
    candidates: list[Candidate] = [
        make_reference_candidate(
            distance_ft, base_rpm, base_hood, config, 1, "reference-interp"
        )
    ]

    if solutions_per_distance <= 1:
        return candidates

    left, right = get_neighbor_reference_setpoints(distance_ft)
    option_rank = 2

    for neighbor in (left, right):
        if neighbor is None or option_rank > solutions_per_distance:
            continue
        neighbor_distance_ft, neighbor_rpm, neighbor_hood = neighbor
        if abs(neighbor_distance_ft - distance_ft) < 1e-9:
            continue
        candidates.append(
            make_reference_candidate(
                distance_ft,
                neighbor_rpm,
                neighbor_hood,
                config,
                option_rank,
                "reference-neighbor",
            )
        )
        option_rank += 1

    hood_offsets = (-0.25, 0.25, -0.5, 0.5)
    for hood_offset in hood_offsets:
        if option_rank > solutions_per_distance:
            break
        adjusted_hood = max(config.hood_min_deg, min(config.hood_max_deg, base_hood + hood_offset))
        if abs(adjusted_hood - base_hood) < 1e-9:
            continue
        adjusted_rpm = base_rpm + (hood_offset * -80.0)
        candidates.append(
            make_reference_candidate(
                distance_ft,
                adjusted_rpm,
                adjusted_hood,
                config,
                option_rank,
                "reference-variant",
            )
        )
        option_rank += 1

    return candidates[:solutions_per_distance]


def generate_setpoints(
    config: Config, solutions_per_distance: int
) -> tuple[list[Candidate], list[float]]:
    chosen_candidates: list[Candidate] = []
    missing_distances_ft: list[float] = []

    for distance_ft in config.distances_ft:
        if config.generation_mode == "reference":
            candidates = generate_reference_candidates_for_distance(
                distance_ft, config, solutions_per_distance
            )
        else:
            candidates = generate_candidates_for_distance(distance_ft, config)

        if not candidates:
            if config.generation_mode == "reference":
                missing_distances_ft.append(distance_ft)
                continue
            reference_setpoint = interpolate_reference_setpoint(distance_ft)
            if reference_setpoint is None:
                missing_distances_ft.append(distance_ft)
                continue
            shooter_rpm, hood_deg = reference_setpoint
            candidates = [
                make_reference_candidate(
                    distance_ft,
                    shooter_rpm,
                    hood_deg,
                    config,
                    1,
                    "reference-fallback",
                )
            ]

        for index, candidate in enumerate(candidates[:solutions_per_distance], start=1):
            chosen_candidates.append(
                Candidate(
                    distance_ft=candidate.distance_ft,
                    hood_deg=candidate.hood_deg,
                    launch_deg=candidate.launch_deg,
                    shooter_rpm=candidate.shooter_rpm,
                    release_speed_mps=candidate.release_speed_mps,
                    flight_time_s=candidate.flight_time_s,
                    apex_m=candidate.apex_m,
                    vertical_error_m=candidate.vertical_error_m,
                    rank_for_distance=index,
                    source=candidate.source,
                )
            )

    return chosen_candidates, missing_distances_ft


def format_number(value: float, digits: int = 2) -> str:
    text = f"{value:.{digits}f}"
    text = text.rstrip("0").rstrip(".")
    return text if text else "0"


def group_candidates_by_distance(candidates: list[Candidate]) -> list[tuple[float, list[Candidate]]]:
    grouped: dict[float, list[Candidate]] = {}
    ordered_distances: list[float] = []
    for candidate in candidates:
        key = round(candidate.distance_ft, 6)
        if key not in grouped:
            grouped[key] = []
            ordered_distances.append(key)
        grouped[key].append(candidate)
    return [(distance_ft, grouped[distance_ft]) for distance_ft in ordered_distances]


def build_calibration_lines(calibration: CalibrationResult | None) -> list[str]:
    if calibration is None:
        return []

    return [
        (
            "// calibratedPhysics: targetHeight="
            f"{format_number(calibration.config.target_height_m, 3)}m, targetXOffset="
            f"{format_number(calibration.config.target_x_offset_m, 3)}m, velocityScale="
            f"{format_number(calibration.config.muzzle_velocity_scale, 3)}, launchAtZeroHood="
            f"{format_number(calibration.config.launch_angle_at_zero_hood_deg, 2)}deg, launchAtMaxHood="
            f"{format_number(calibration.config.launch_angle_at_max_hood_deg, 2)}deg, liftCoefficient="
            f"{format_number(calibration.config.lift_coefficient, 3)}, dragCoefficient="
            f"{format_number(calibration.config.drag_coefficient, 3)}, rmsFitError="
            f"{format_number(calibration.rms_error_m, 3)}m, maxFitError="
            f"{format_number(calibration.max_abs_error_m, 3)}m, rpmRmsFitError="
            f"{format_number(calibration.rpm_rms_error, 1)}, rpmMaxFitError="
            f"{format_number(calibration.rpm_max_abs_error, 1)}"
        )
    ]


def build_java_lines(
    candidates: list[Candidate], config: Config, calibration: CalibrationResult | None = None
) -> list[str]:
    lines = [
        "// Generated by scripts/generate_shooter_setpoints.py",
        (
            "// releaseHeight="
            f"{format_number(config.release_height_m, 3)}m, targetHeight="
            f"{format_number(config.target_height_m, 3)}m, targetXOffset="
            f"{format_number(config.target_x_offset_m, 3)}m, wheelRadius="
            f"{format_number(config.wheel_radius_m, 4)}m, velocityScale="
            f"{format_number(config.muzzle_velocity_scale, 3)}, launchAtZeroHood="
            f"{format_number(config.launch_angle_at_zero_hood_deg, 2)}deg, launchAtMaxHood="
            f"{format_number(config.launch_angle_at_max_hood_deg, 2)}deg, compression="
            f"{format_number(config.compression_m, 3)}m, liftCoefficient="
            f"{format_number(config.lift_coefficient, 3)}, dragCoefficient="
            f"{format_number(config.drag_coefficient, 3)}"
        ),
    ]
    lines.extend(build_calibration_lines(calibration))
    for distance_ft, distance_candidates in group_candidates_by_distance(candidates):
        if len(distance_candidates) > 1:
            lines.append(f"// {format_number(distance_ft, 2)} ft options")
        for candidate in distance_candidates:
            prefix_parts: list[str] = []
            if len(distance_candidates) > 1:
                prefix_parts.append(f"option {candidate.rank_for_distance}")
            if candidate.source != "generated":
                prefix_parts.append(candidate.source)
            prefix = f"// {', '.join(prefix_parts)}: " if prefix_parts else ""
            lines.append(
                prefix
                + "table.put("
                f"{format_number(candidate.distance_ft, 2)}, "
                "new ShooterSetpoint("
                f"RPM.of({round(candidate.shooter_rpm)}), "
                f"RPM.of({round(config.feeder_rpm)}), "
                f"Degree.of({format_number(candidate.hood_deg, 2)})));"
            )
    return lines


def print_summary(candidates: list[Candidate]) -> None:
    print(
        "Distance(ft)  Option  Source             HoodCmd(deg)  Launch(deg)  ShooterRPM  ExitSpeed(m/s)  FlightTime(s)  Apex(m)"
    )
    for candidate in candidates:
        print(
            f"{candidate.distance_ft:11.2f}  "
            f"{candidate.rank_for_distance:6d}  "
            f"{candidate.source:17.17s}  "
            f"{candidate.hood_deg:12.2f}  "
            f"{candidate.launch_deg:11.2f}  "
            f"{candidate.shooter_rpm:10.1f}  "
            f"{candidate.release_speed_mps:14.3f}  "
            f"{candidate.flight_time_s:13.3f}  "
            f"{candidate.apex_m:7.3f}"
        )


def print_calibration_summary(calibration: CalibrationResult | None) -> None:
    if calibration is None:
        return

    print("Calibration Fit")
    print(
        "  "
        f"targetHeight={calibration.config.target_height_m:.3f}m, "
        f"targetXOffset={calibration.config.target_x_offset_m:.3f}m, "
        f"velocityScale={calibration.config.muzzle_velocity_scale:.3f}, "
        f"launchAtZeroHood={calibration.config.launch_angle_at_zero_hood_deg:.2f}deg, "
        f"launchAtMaxHood={calibration.config.launch_angle_at_max_hood_deg:.2f}deg, "
        f"liftCoefficient={calibration.config.lift_coefficient:.3f}, "
        f"dragCoefficient={calibration.config.drag_coefficient:.3f}"
    )
    print(
        "  "
        f"rmsFitError={calibration.rms_error_m:.3f}m, "
        f"meanAbsFitError={calibration.mean_abs_error_m:.3f}m, "
        f"maxFitError={calibration.max_abs_error_m:.3f}m, "
        f"rpmRmsFitError={calibration.rpm_rms_error:.1f}, "
        f"rpmMaxFitError={calibration.rpm_max_abs_error:.1f}, "
        f"samples={calibration.sample_count}"
    )


def write_csv(path: Path, candidates: list[Candidate]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "distance_ft",
                "option_index",
                "source",
                "hood_deg",
                "launch_deg",
                "shooter_rpm",
                "exit_speed_mps",
                "flight_time_s",
                "apex_m",
                "vertical_error_m",
            ]
        )
        for candidate in candidates:
            writer.writerow(
                [
                    f"{candidate.distance_ft:.3f}",
                    candidate.rank_for_distance,
                    candidate.source,
                    f"{candidate.hood_deg:.3f}",
                    f"{candidate.launch_deg:.3f}",
                    f"{candidate.shooter_rpm:.3f}",
                    f"{candidate.release_speed_mps:.5f}",
                    f"{candidate.flight_time_s:.5f}",
                    f"{candidate.apex_m:.5f}",
                    f"{candidate.vertical_error_m:.5f}",
                ]
            )


def write_lines(path: Path, lines: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_visualization_html(
    candidates: list[Candidate], config: Config, calibration: CalibrationResult | None = None
) -> str:
    grouped_candidates = group_candidates_by_distance(candidates)
    trajectory_entries: list[tuple[Candidate, list[tuple[float, float]]]] = []
    for candidate in candidates:
        modeled_distance_m = max(0.0, candidate.distance_ft * FEET_TO_METERS + config.target_x_offset_m)
        trajectory_entries.append(
            (
                candidate,
                sample_trajectory_points(
                    modeled_distance_m,
                    candidate.release_speed_mps,
                    candidate.hood_deg,
                    config,
                ),
            )
        )

    max_distance_m = max((points[-1][0] for _, points in trajectory_entries), default=1.0)
    max_height_m = max(
        (
            max(max(y for _, y in points), config.target_height_m, config.release_height_m)
            for _, points in trajectory_entries
        ),
        default=max(config.target_height_m, config.release_height_m, 1.0),
    )

    width = 1200.0
    height = 720.0
    padding_left = 72.0
    padding_right = 24.0
    padding_top = 24.0
    padding_bottom = 64.0
    plot_width = width - padding_left - padding_right
    plot_height = height - padding_top - padding_bottom
    x_scale = plot_width / max(max_distance_m, 0.1)
    y_scale = plot_height / max(max_height_m, 0.1)
    target_y_px = padding_top + plot_height - config.target_height_m * y_scale

    colors = [
        "#0b5fff",
        "#ff6b00",
        "#00a86b",
        "#cc0066",
        "#6b46c1",
        "#c2410c",
        "#0f766e",
        "#d97706",
    ]

    svg_lines: list[str] = [
        f'<svg viewBox="0 0 {width:.0f} {height:.0f}" width="100%" xmlns="http://www.w3.org/2000/svg">',
        '<rect x="0" y="0" width="100%" height="100%" fill="#ffffff"/>',
        f'<line x1="{padding_left:.1f}" y1="{padding_top + plot_height:.1f}" '
        f'x2="{padding_left + plot_width:.1f}" y2="{padding_top + plot_height:.1f}" stroke="#222" stroke-width="2"/>',
        f'<line x1="{padding_left:.1f}" y1="{padding_top:.1f}" '
        f'x2="{padding_left:.1f}" y2="{padding_top + plot_height:.1f}" stroke="#222" stroke-width="2"/>',
        f'<line x1="{padding_left:.1f}" y1="{target_y_px:.1f}" '
        f'x2="{padding_left + plot_width:.1f}" y2="{target_y_px:.1f}" stroke="#999" stroke-dasharray="8 6" stroke-width="1.5"/>',
        f'<text x="{padding_left + 8:.1f}" y="{target_y_px - 8:.1f}" fill="#444" font-size="14">Target height {config.target_height_m:.3f} m</text>',
    ]

    x_tick_count = min(10, max(2, len(grouped_candidates)))
    for index in range(x_tick_count + 1):
        x_m = max_distance_m * index / x_tick_count
        x_px = padding_left + x_m * x_scale
        svg_lines.append(
            f'<line x1="{x_px:.1f}" y1="{padding_top + plot_height:.1f}" '
            f'x2="{x_px:.1f}" y2="{padding_top + plot_height + 8:.1f}" stroke="#222" stroke-width="1"/>'
        )
        svg_lines.append(
            f'<text x="{x_px:.1f}" y="{padding_top + plot_height + 28:.1f}" text-anchor="middle" fill="#444" font-size="12">{x_m / FEET_TO_METERS:.1f} ft</text>'
        )

    for index in range(6):
        y_m = max_height_m * index / 5
        y_px = padding_top + plot_height - y_m * y_scale
        svg_lines.append(
            f'<line x1="{padding_left - 8:.1f}" y1="{y_px:.1f}" '
            f'x2="{padding_left:.1f}" y2="{y_px:.1f}" stroke="#222" stroke-width="1"/>'
        )
        svg_lines.append(
            f'<text x="{padding_left - 12:.1f}" y="{y_px + 4:.1f}" text-anchor="end" fill="#444" font-size="12">{y_m:.2f} m</text>'
        )
        if index > 0:
            svg_lines.append(
                f'<line x1="{padding_left:.1f}" y1="{y_px:.1f}" '
                f'x2="{padding_left + plot_width:.1f}" y2="{y_px:.1f}" stroke="#eeeeee" stroke-width="1"/>'
            )

    for index, (candidate, points) in enumerate(trajectory_entries):
        color = colors[index % len(colors)]
        polyline = " ".join(
            f"{padding_left + x * x_scale:.1f},{padding_top + plot_height - y * y_scale:.1f}"
            for x, y in points
        )
        svg_lines.append(
            f'<polyline fill="none" stroke="{color}" stroke-width="2.5" points="{polyline}"/>'
        )
        modeled_distance_m = max(0.0, candidate.distance_ft * FEET_TO_METERS + config.target_x_offset_m)
        target_x_px = padding_left + modeled_distance_m * x_scale
        svg_lines.append(
            f'<circle cx="{target_x_px:.1f}" cy="{target_y_px:.1f}" r="4" fill="{color}"/>'
        )

    svg_lines.append("</svg>")

    table_rows: list[str] = []
    for index, candidate in enumerate(candidates):
        color = colors[index % len(colors)]
        table_rows.append(
            "<tr>"
            f'<td><span style="display:inline-block;width:14px;height:14px;background:{color};border-radius:2px;"></span></td>'
            f"<td>{candidate.distance_ft:.2f}</td>"
            f"<td>{candidate.rank_for_distance}</td>"
            f"<td>{html.escape(candidate.source)}</td>"
            f"<td>{candidate.hood_deg:.2f}</td>"
            f"<td>{candidate.launch_deg:.2f}</td>"
            f"<td>{candidate.shooter_rpm:.1f}</td>"
            f"<td>{candidate.flight_time_s:.3f}</td>"
            f"<td>{candidate.apex_m:.3f}</td>"
            "</tr>"
        )

    assumptions = html.escape(
        " | ".join(
            [
                f"releaseHeight={config.release_height_m:.3f}m",
                f"targetHeight={config.target_height_m:.3f}m",
                f"targetXOffset={config.target_x_offset_m:.3f}m",
                f"wheelRadius={config.wheel_radius_m:.4f}m",
                f"velocityScale={config.muzzle_velocity_scale:.3f}",
                f"launchAtZeroHood={config.launch_angle_at_zero_hood_deg:.2f}deg",
                f"launchAtMaxHood={config.launch_angle_at_max_hood_deg:.2f}deg",
                f"liftCoefficient={config.lift_coefficient:.3f}",
                f"dragCoefficient={config.drag_coefficient:.3f}",
                f"generationMode={config.generation_mode}",
                (
                    "calibratedFit="
                    f"{calibration.rms_error_m:.3f}m rms / {calibration.max_abs_error_m:.3f}m max"
                    if calibration is not None
                    else "calibratedFit=n/a"
                ),
            ]
        )
    )

    return "\n".join(
        [
            "<!DOCTYPE html>",
            "<html lang=\"en\">",
            "<head>",
            "<meta charset=\"utf-8\" />",
            "<title>Shooter Setpoint Visualization</title>",
            "<style>",
            "body { font-family: Arial, sans-serif; margin: 24px; color: #1f2937; }",
            "h1, h2 { margin-bottom: 8px; }",
            ".note { color: #4b5563; margin-bottom: 18px; }",
            ".chart { border: 1px solid #d1d5db; border-radius: 12px; padding: 12px; background: #fafafa; }",
            "table { border-collapse: collapse; margin-top: 16px; width: 100%; }",
            "th, td { border: 1px solid #d1d5db; padding: 8px 10px; text-align: left; }",
            "th { background: #f3f4f6; }",
            "pre { background: #f3f4f6; border-radius: 8px; padding: 12px; overflow-x: auto; }",
            "</style>",
            "</head>",
            "<body>",
            "<h1>Shooter Trajectory Visualization</h1>",
            f"<p class=\"note\">{assumptions}</p>",
            "<div class=\"chart\">",
            *svg_lines,
            "</div>",
            "<h2>Candidate Setpoints</h2>",
            "<table>",
            "<thead><tr><th></th><th>Distance (ft)</th><th>Option</th><th>Source</th><th>Hood Cmd (deg)</th><th>Launch (deg)</th><th>Shooter RPM</th><th>Flight Time (s)</th><th>Apex (m)</th></tr></thead>",
            "<tbody>",
            *table_rows,
            "</tbody>",
            "</table>",
            "<h2>Java Lines</h2>",
            "<pre>",
            html.escape("\n".join(build_java_lines(candidates, config, calibration))),
            "</pre>",
            "</body>",
            "</html>",
        ]
    )


def build_reference_candidates(config: Config) -> list[Candidate]:
    references: list[Candidate] = []
    for distance_ft, shooter_rpm, hood_deg in REFERENCE_SETPOINTS:
        modeled_distance_m = max(0.0, distance_ft * FEET_TO_METERS + config.target_x_offset_m)
        release_speed_mps = rpm_to_exit_speed_mps(shooter_rpm, config)
        result = simulate_to_distance(modeled_distance_m, release_speed_mps, hood_deg, config)
        y_at_target = result.y_at_target_m if result.y_at_target_m is not None else float("nan")
        vertical_error = (
            y_at_target - config.target_height_m
            if result.y_at_target_m is not None
            else float("nan")
        )
        references.append(
            Candidate(
                distance_ft=distance_ft,
                hood_deg=hood_deg,
                launch_deg=hood_to_launch_deg(hood_deg, config),
                shooter_rpm=shooter_rpm,
                release_speed_mps=release_speed_mps,
                flight_time_s=result.time_at_target_s if result.time_at_target_s is not None else float("nan"),
                apex_m=result.apex_m,
                vertical_error_m=vertical_error,
                rank_for_distance=1,
                source="reference",
            )
        )
    return references


def launch_gui(
    candidates: list[Candidate], config: Config, calibration: CalibrationResult | None = None
) -> None:
    references = build_reference_candidates(config)
    root = tk.Tk()
    root.title("Shooter Setpoint Visualizer")
    root.geometry("1500x900")

    mainframe = ttk.Frame(root, padding=12)
    mainframe.pack(fill=tk.BOTH, expand=True)

    title = ttk.Label(
        mainframe,
        text="Shooter Trajectory Visualizer",
        font=("Segoe UI", 16, "bold"),
    )
    title.pack(anchor=tk.W)

    subtitle = ttk.Label(
        mainframe,
        text=(
            "Generated shots are solid lines. Confirmed reference shots are dashed lines. "
            "Use this to sanity-check the model before trusting any table output."
        ),
        wraplength=1200,
    )
    subtitle.pack(anchor=tk.W, pady=(4, 10))

    assumptions_text = (
        f"releaseHeight={config.release_height_m:.3f}m | "
        f"targetHeight={config.target_height_m:.3f}m | "
        f"targetXOffset={config.target_x_offset_m:.3f}m | "
        f"wheelRadius={config.wheel_radius_m:.4f}m | "
        f"velocityScale={config.muzzle_velocity_scale:.3f} | "
        f"launchAtZeroHood={config.launch_angle_at_zero_hood_deg:.2f}deg | "
        f"launchAtMaxHood={config.launch_angle_at_max_hood_deg:.2f}deg | "
        f"liftCoefficient={config.lift_coefficient:.3f} | "
        f"generationMode={config.generation_mode} | "
        f"sortMode={config.candidate_sort_mode}"
    )
    if calibration is not None:
        assumptions_text += (
            f" | fitRms={calibration.rms_error_m:.3f}m"
            f" | fitMax={calibration.max_abs_error_m:.3f}m"
        )
    ttk.Label(mainframe, text=assumptions_text, wraplength=1300).pack(anchor=tk.W, pady=(0, 10))

    top_pane = ttk.Panedwindow(mainframe, orient=tk.HORIZONTAL)
    top_pane.pack(fill=tk.BOTH, expand=True)

    left_panel = ttk.Frame(top_pane)
    right_panel = ttk.Frame(top_pane)
    top_pane.add(left_panel, weight=4)
    top_pane.add(right_panel, weight=3)

    canvas = tk.Canvas(left_panel, width=950, height=560, background="white", highlightthickness=1)
    canvas.pack(fill=tk.BOTH, expand=True)

    colors = [
        "#0b5fff",
        "#ff6b00",
        "#00a86b",
        "#cc0066",
        "#6b46c1",
        "#c2410c",
        "#0f766e",
        "#d97706",
    ]

    all_entries: list[tuple[str, Candidate, list[tuple[float, float]]]] = []
    for candidate in candidates:
        modeled_distance_m = max(0.0, candidate.distance_ft * FEET_TO_METERS + config.target_x_offset_m)
        all_entries.append(
            (
                "generated",
                candidate,
                sample_trajectory_points(
                    modeled_distance_m, candidate.release_speed_mps, candidate.hood_deg, config
                ),
            )
        )
    for reference in references:
        modeled_distance_m = max(0.0, reference.distance_ft * FEET_TO_METERS + config.target_x_offset_m)
        all_entries.append(
            (
                "reference",
                reference,
                sample_trajectory_points(
                    modeled_distance_m, reference.release_speed_mps, reference.hood_deg, config
                ),
            )
        )

    max_distance_m = max((points[-1][0] for _, _, points in all_entries), default=1.0)
    max_height_m = max(
        (
            max(max(y for _, y in points), config.target_height_m, config.release_height_m)
            for _, _, points in all_entries
        ),
        default=max(config.target_height_m, config.release_height_m, 1.0),
    )

    canvas_width = 930.0
    canvas_height = 540.0
    padding_left = 70.0
    padding_right = 24.0
    padding_top = 20.0
    padding_bottom = 60.0
    plot_width = canvas_width - padding_left - padding_right
    plot_height = canvas_height - padding_top - padding_bottom
    x_scale = plot_width / max(max_distance_m, 0.1)
    y_scale = plot_height / max(max_height_m, 0.1)

    def to_canvas(x_m: float, y_m: float) -> tuple[float, float]:
        return (
            padding_left + x_m * x_scale,
            padding_top + plot_height - y_m * y_scale,
        )

    target_y = to_canvas(0.0, config.target_height_m)[1]
    canvas.create_line(
        padding_left,
        padding_top + plot_height,
        padding_left + plot_width,
        padding_top + plot_height,
        width=2,
        fill="#222222",
    )
    canvas.create_line(
        padding_left,
        padding_top,
        padding_left,
        padding_top + plot_height,
        width=2,
        fill="#222222",
    )
    canvas.create_line(
        padding_left,
        target_y,
        padding_left + plot_width,
        target_y,
        width=2,
        fill="#999999",
        dash=(8, 6),
    )
    canvas.create_text(
        padding_left + 8,
        target_y - 10,
        anchor="w",
        text=f"Target height {config.target_height_m:.3f} m",
        fill="#444444",
        font=("Segoe UI", 10),
    )

    for index in range(6):
        y_m = max_height_m * index / 5
        _, y_px = to_canvas(0.0, y_m)
        if index > 0:
            canvas.create_line(
                padding_left,
                y_px,
                padding_left + plot_width,
                y_px,
                fill="#eeeeee",
            )
        canvas.create_line(padding_left - 6, y_px, padding_left, y_px, fill="#222222")
        canvas.create_text(
            padding_left - 10,
            y_px,
            anchor="e",
            text=f"{y_m:.2f} m",
            font=("Segoe UI", 9),
        )

    tick_count = min(10, max(2, len(config.distances_ft)))
    for index in range(tick_count + 1):
        x_m = max_distance_m * index / tick_count
        x_px, _ = to_canvas(x_m, 0.0)
        canvas.create_line(
            x_px,
            padding_top + plot_height,
            x_px,
            padding_top + plot_height + 6,
            fill="#222222",
        )
        canvas.create_text(
            x_px,
            padding_top + plot_height + 20,
            text=f"{x_m / FEET_TO_METERS:.1f} ft",
            font=("Segoe UI", 9),
        )

    for index, (kind, candidate, points) in enumerate(all_entries):
        color = colors[index % len(colors)] if kind == "generated" else "#555555"
        flat_points: list[float] = []
        for x_m, y_m in points:
            x_px, y_px = to_canvas(x_m, y_m)
            flat_points.extend([x_px, y_px])
        if len(flat_points) >= 4:
            canvas.create_line(
                *flat_points,
                fill=color,
                width=2.5 if kind == "generated" else 2.0,
                dash=() if kind == "generated" else (7, 5),
                smooth=False,
            )

    generated_label = ttk.Label(right_panel, text="Generated Candidates", font=("Segoe UI", 12, "bold"))
    generated_label.pack(anchor=tk.W)

    generated_tree = ttk.Treeview(
        right_panel,
        columns=("distance", "option", "source", "hood", "launch", "rpm", "time", "apex"),
        show="headings",
        height=10,
    )
    for column, width, title_text in [
        ("distance", 85, "Dist ft"),
        ("option", 60, "Opt"),
        ("source", 120, "Source"),
        ("hood", 85, "Hood"),
        ("launch", 85, "Launch"),
        ("rpm", 85, "RPM"),
        ("time", 85, "Time"),
        ("apex", 85, "Apex"),
    ]:
        generated_tree.heading(column, text=title_text)
        generated_tree.column(column, width=width, anchor=tk.CENTER)
    for candidate in candidates:
        generated_tree.insert(
            "",
            tk.END,
            values=(
                f"{candidate.distance_ft:.2f}",
                candidate.rank_for_distance,
                candidate.source,
                f"{candidate.hood_deg:.2f}",
                f"{candidate.launch_deg:.2f}",
                f"{candidate.shooter_rpm:.0f}",
                f"{candidate.flight_time_s:.3f}",
                f"{candidate.apex_m:.3f}",
            ),
        )
    generated_tree.pack(fill=tk.X, pady=(6, 12))

    reference_label = ttk.Label(right_panel, text="Confirmed Reference Map", font=("Segoe UI", 12, "bold"))
    reference_label.pack(anchor=tk.W)

    reference_tree = ttk.Treeview(
        right_panel,
        columns=("distance", "hood", "launch", "rpm", "time", "apex", "error"),
        show="headings",
        height=10,
    )
    for column, width, title_text in [
        ("distance", 85, "Dist ft"),
        ("hood", 85, "Hood"),
        ("launch", 85, "Launch"),
        ("rpm", 85, "RPM"),
        ("time", 85, "Time"),
        ("apex", 85, "Apex"),
        ("error", 95, "Err m"),
    ]:
        reference_tree.heading(column, text=title_text)
        reference_tree.column(column, width=width, anchor=tk.CENTER)
    for reference in references:
        reference_tree.insert(
            "",
            tk.END,
            values=(
                f"{reference.distance_ft:.2f}",
                f"{reference.hood_deg:.2f}",
                f"{reference.launch_deg:.2f}",
                f"{reference.shooter_rpm:.0f}",
                "n/a" if math.isnan(reference.flight_time_s) else f"{reference.flight_time_s:.3f}",
                f"{reference.apex_m:.3f}",
                "n/a" if math.isnan(reference.vertical_error_m) else f"{reference.vertical_error_m:.3f}",
            ),
        )
    reference_tree.pack(fill=tk.X, pady=(6, 12))

    java_label = ttk.Label(right_panel, text="Java Lines", font=("Segoe UI", 12, "bold"))
    java_label.pack(anchor=tk.W)
    java_text = tk.Text(right_panel, height=14, wrap="none")
    java_text.insert("1.0", "\n".join(build_java_lines(candidates, config, calibration)))
    java_text.configure(state="disabled")
    java_text.pack(fill=tk.BOTH, expand=True)

    root.mainloop()


def main() -> int:
    args = parse_args()
    if (
        args.distance_start_ft is not None
        or args.distance_end_ft is not None
        or args.distance_count is not None
    ):
        if (
            args.distance_start_ft is None
            or args.distance_end_ft is None
            or args.distance_count is None
        ):
            print(
                "error: --distance-start-ft, --distance-end-ft, and --distance-count must be provided together.",
                file=sys.stderr,
            )
            return 1
        distances_ft = build_distance_range(
            args.distance_start_ft, args.distance_end_ft, args.distance_count
        )
    else:
        distances_ft = parse_distances_feet(args.distances_ft)

    if args.solutions_per_distance < 1:
        print("error: --solutions-per-distance must be at least 1.", file=sys.stderr)
        return 1

    raw_config = Config(
        release_height_m=args.release_height_m,
        target_height_m=args.target_height_m,
        target_x_offset_m=args.target_x_offset_m,
        wheel_radius_m=args.wheel_radius_m,
        muzzle_velocity_scale=args.muzzle_velocity_scale,
        launch_angle_at_zero_hood_deg=args.launch_angle_at_zero_hood_deg,
        launch_angle_at_max_hood_deg=args.launch_angle_at_max_hood_deg,
        compression_m=args.compression_m,
        hood_min_deg=args.hood_min_deg,
        hood_max_deg=args.hood_max_deg,
        hood_step_deg=args.hood_step_deg,
        rpm_min=args.rpm_min,
        rpm_max=args.rpm_max,
        solver_tolerance_m=args.solver_tolerance_m,
        simulation_dt_s=args.simulation_dt_s,
        max_time_s=args.max_time_s,
        gravity_mps2=args.gravity_mps2,
        drag_coefficient=args.drag_coefficient,
        air_density_kgpm3=args.air_density_kgpm3,
        ball_diameter_m=args.ball_diameter_m,
        ball_mass_kg=args.ball_mass_kg,
        lift_coefficient=args.lift_coefficient,
        feeder_rpm=args.feeder_rpm,
        candidate_sort_mode=args.candidate_sort_mode,
        generation_mode=args.generation_mode,
        distances_ft=distances_ft,
    )

    calibration: CalibrationResult | None = None
    config = raw_config
    if raw_config.generation_mode == "calibrated-physics":
        calibration = calibrate_model(raw_config)
        config = calibration.config

    candidates, missing_distances_ft = generate_setpoints(config, args.solutions_per_distance)
    if not candidates:
        print(
            "error: no generated or fallback candidates were available for the requested distances.",
            file=sys.stderr,
        )
        return 1

    print_calibration_summary(calibration)
    if calibration is not None:
        print()
    print_summary(candidates)
    print()
    java_lines = build_java_lines(candidates, config, calibration)
    print("\n".join(java_lines))

    if missing_distances_ft:
        print(
            "\nwarning: no generated or fallback candidate was available for these distances: "
            + ", ".join(format_number(distance_ft, 2) for distance_ft in missing_distances_ft)
        )

    if args.visualize_gui:
        launch_gui(candidates, config, calibration)

    if args.csv_out is not None:
        try:
            write_csv(args.csv_out, candidates)
            print(f"\nWrote CSV to {args.csv_out}")
        except OSError as error:
            print(f"error: could not write CSV to {args.csv_out}: {error}", file=sys.stderr)
            return 1

    if args.java_out is not None:
        try:
            write_lines(args.java_out, java_lines)
            print(f"Wrote Java snippet to {args.java_out}")
        except OSError as error:
            print(f"error: could not write Java snippet to {args.java_out}: {error}", file=sys.stderr)
            return 1

    if args.visual_out is not None:
        try:
            args.visual_out.parent.mkdir(parents=True, exist_ok=True)
            args.visual_out.write_text(
                build_visualization_html(candidates, config, calibration), encoding="utf-8"
            )
            print(f"Wrote visualization to {args.visual_out}")
        except OSError as error:
            print(f"error: could not write visualization to {args.visual_out}: {error}", file=sys.stderr)
            return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#COMMAND TO RUN IS powershell -ExecutionPolicy Bypass -File .\scripts\generate_shooter_setpoints.ps1 --distance-start-ft 5 --distance-end-ft 18 --distance-count 10 --solutions-per-distance 3 --candidate-sort-mode balanced
