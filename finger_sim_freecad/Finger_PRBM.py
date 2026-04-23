from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from typing import Iterable

import matplotlib.pyplot as plt

BASE_BLOCK_LENGTH_MM = 20.0
RIGID_SEGMENT_LENGTH_MM = 25.0
COMPLIANT_SEGMENT_LENGTH_MM = 15.0
SEGMENT_HEIGHT_MM = 10.0
JOINT_THICKNESSES_MM = (3.0, 3.25, 3.25, 3.5)
N_JOINTS = len(JOINT_THICKNESSES_MM)

DEFAULT_CABLE_FORCE_N = 20.0
DEFAULT_MOMENT_ARM_MM = 8.0
DEFAULT_FORCE_MULTIPLIERS = (1.25, 2.5, 3.75, 5.0, 10.0, 20.0)
DEFAULT_INTERMEDIATE_STEPS = 0

NOMINAL_JOINT_THICKNESS_MM = 3.25
NOMINAL_JOINT_STIFFNESS_NMM_PER_RAD = 50.0


Point2D = tuple[float, float]
Segment2D = tuple[Point2D, Point2D]


@dataclass(frozen=True)
class FingerState:
    cable_force_n: float
    moment_arm_mm: float
    joint_moments_nmm: tuple[float, ...]
    joint_stiffnesses_nmm_per_rad: tuple[float, ...]
    joint_bends_deg: tuple[float, ...]
    segment_angles_deg: tuple[float, ...]
    joint_midpoints_mm: tuple[Point2D, ...]
    segment_centers_mm: tuple[Point2D, ...]
    rigid_segments_mm: tuple[Segment2D, ...]
    centerline_points_mm: tuple[Point2D, ...]
    tip_position_mm: Point2D
    tip_angle_deg: float


def joint_torsional_stiffnesses_nmm_per_rad(
    joint_thicknesses_mm: Iterable[float] = JOINT_THICKNESSES_MM,
) -> tuple[float, ...]:
    return tuple(
        NOMINAL_JOINT_STIFFNESS_NMM_PER_RAD * (float(joint_thickness_mm) / NOMINAL_JOINT_THICKNESS_MM) ** 3
        for joint_thickness_mm in joint_thicknesses_mm
    )


def joint_moments_from_cable_force(
    cable_force_n: float,
    *,
    moment_arm_mm: float = DEFAULT_MOMENT_ARM_MM,
    n_joints: int = N_JOINTS,
) -> tuple[float, ...]:
    return tuple(float(cable_force_n) * float(moment_arm_mm) for _ in range(n_joints))


def predicted_joint_bends_deg(
    cable_force_n: float,
    *,
    moment_arm_mm: float = DEFAULT_MOMENT_ARM_MM,
    joint_thicknesses_mm: Iterable[float] = JOINT_THICKNESSES_MM,
) -> tuple[float, ...]:
    stiffnesses = joint_torsional_stiffnesses_nmm_per_rad(joint_thicknesses_mm)
    moments = joint_moments_from_cable_force(
        cable_force_n,
        moment_arm_mm=moment_arm_mm,
        n_joints=len(stiffnesses),
    )
    return tuple(-math.degrees(moment_nmm / stiffness_nmm_per_rad) for moment_nmm, stiffness_nmm_per_rad in zip(moments, stiffnesses))


def build_target_cable_forces(
    base_cable_force_n: float,
    *,
    force_multipliers: tuple[float, ...] = DEFAULT_FORCE_MULTIPLIERS,
    intermediate_steps: int = DEFAULT_INTERMEDIATE_STEPS,
) -> list[float]:
    if intermediate_steps < 0:
        raise ValueError("Intermediate steps must be zero or greater.")
    targets = [0.0]
    anchor_multipliers = (0.0, *force_multipliers)
    for start_multiplier, end_multiplier in zip(anchor_multipliers, anchor_multipliers[1:]):
        step_count = intermediate_steps + 1
        for step_index in range(1, step_count + 1):
            multiplier = start_multiplier + (end_multiplier - start_multiplier) * (step_index / step_count)
            force_n = float(base_cable_force_n) * multiplier
            if not math.isclose(force_n, targets[-1], rel_tol=1e-9, abs_tol=1e-9):
                targets.append(force_n)
    return targets


def _advance(point_mm: Point2D, length_mm: float, geometry_angle_deg: float) -> Point2D:
    angle_rad = math.radians(geometry_angle_deg)
    return (
        point_mm[0] + float(length_mm) * math.cos(angle_rad),
        point_mm[1] + float(length_mm) * math.sin(angle_rad),
    )


def solve_prbm_state(
    cable_force_n: float,
    *,
    moment_arm_mm: float = DEFAULT_MOMENT_ARM_MM,
    joint_thicknesses_mm: Iterable[float] = JOINT_THICKNESSES_MM,
    base_block_length_mm: float = BASE_BLOCK_LENGTH_MM,
    compliant_segment_length_mm: float = COMPLIANT_SEGMENT_LENGTH_MM,
    rigid_segment_length_mm: float = RIGID_SEGMENT_LENGTH_MM,
) -> FingerState:
    stiffnesses = joint_torsional_stiffnesses_nmm_per_rad(joint_thicknesses_mm)
    joint_moments = joint_moments_from_cable_force(
        cable_force_n,
        moment_arm_mm=moment_arm_mm,
        n_joints=len(stiffnesses),
    )
    joint_bends_deg = tuple(
        -math.degrees(moment_nmm / stiffness_nmm_per_rad)
        for moment_nmm, stiffness_nmm_per_rad in zip(joint_moments, stiffnesses)
    )

    current_point = (float(base_block_length_mm), 0.0)
    centerline_points = [(0.0, 0.0), current_point]
    joint_midpoints: list[Point2D] = []
    segment_centers: list[Point2D] = []
    rigid_segments: list[Segment2D] = []
    segment_angles_deg: list[float] = []
    cumulative_reported_angle_deg = 0.0

    for joint_bend_deg in joint_bends_deg:
        joint_midpoint = _advance(
            current_point,
            0.5 * float(compliant_segment_length_mm),
            -(cumulative_reported_angle_deg + 0.5 * joint_bend_deg),
        )
        joint_midpoints.append(joint_midpoint)

        segment_start = _advance(
            current_point,
            float(compliant_segment_length_mm),
            -(cumulative_reported_angle_deg + 0.5 * joint_bend_deg),
        )
        centerline_points.append(segment_start)

        cumulative_reported_angle_deg += joint_bend_deg
        segment_angles_deg.append(cumulative_reported_angle_deg)

        segment_center = _advance(
            segment_start,
            0.5 * float(rigid_segment_length_mm),
            -cumulative_reported_angle_deg,
        )
        segment_end = _advance(
            segment_start,
            float(rigid_segment_length_mm),
            -cumulative_reported_angle_deg,
        )
        segment_centers.append(segment_center)
        rigid_segments.append((segment_start, segment_end))
        centerline_points.append(segment_end)
        current_point = segment_end

    return FingerState(
        cable_force_n=float(cable_force_n),
        moment_arm_mm=float(moment_arm_mm),
        joint_moments_nmm=joint_moments,
        joint_stiffnesses_nmm_per_rad=stiffnesses,
        joint_bends_deg=joint_bends_deg,
        segment_angles_deg=tuple(segment_angles_deg),
        joint_midpoints_mm=tuple(joint_midpoints),
        segment_centers_mm=tuple(segment_centers),
        rigid_segments_mm=tuple(rigid_segments),
        centerline_points_mm=tuple(centerline_points),
        tip_position_mm=current_point,
        tip_angle_deg=segment_angles_deg[-1] if segment_angles_deg else 0.0,
    )


def plot_finger_state(ax, state: FingerState, *, color: str, label: str, linestyle: str = "-", linewidth: float = 2.5) -> None:
    xs = [point[0] for point in state.centerline_points_mm]
    zs = [point[1] for point in state.centerline_points_mm]
    ax.plot(xs, zs, linestyle=linestyle, linewidth=linewidth, color=color, label=label)
    for segment_start, segment_end in state.rigid_segments_mm:
        ax.plot(
            [segment_start[0], segment_end[0]],
            [segment_start[1], segment_end[1]],
            color=color,
            linewidth=6.0,
            solid_capstyle="round",
            alpha=0.85,
        )
    ax.scatter(xs[1:], zs[1:], s=28, color=color, zorder=3)


def format_state_summary(state: FingerState) -> str:
    joint_text = ", ".join(f"j{i + 1}={bend_deg:.2f} deg" for i, bend_deg in enumerate(state.joint_bends_deg))
    return (
        f"Cable force: {state.cable_force_n:.3f} N\n"
        f"Moment arm: {state.moment_arm_mm:.3f} mm\n"
        f"Tip angle: {state.tip_angle_deg:.3f} deg\n"
        f"Joint bends: {joint_text}"
    )


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analytical PRBM model for the 4-joint compliant finger.")
    parser.add_argument("--cable-force-n", type=float, default=DEFAULT_CABLE_FORCE_N, help="Cable force in newtons.")
    parser.add_argument("--moment-arm-mm", type=float, default=DEFAULT_MOMENT_ARM_MM, help="Cable moment arm in millimeters.")
    parser.add_argument("--plot-schedule", action="store_true", help="Plot the full FreeCAD stage schedule instead of one state.")
    parser.add_argument("--no-show", action="store_true", help="Create the figure without showing it.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    fig, ax = plt.subplots(figsize=(9, 6), facecolor="white")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("z [mm]")
    ax.plot([0.0, BASE_BLOCK_LENGTH_MM], [0.0, 0.0], color="black", linewidth=7.0, solid_capstyle="round", label="Base")

    if args.plot_schedule:
        schedule = build_target_cable_forces(
            1.0,
            force_multipliers=DEFAULT_FORCE_MULTIPLIERS,
            intermediate_steps=DEFAULT_INTERMEDIATE_STEPS,
        )
        colors = plt.cm.cividis([index / max(1, len(schedule) - 1) for index in range(len(schedule))])
        for index, cable_force_n in enumerate(schedule):
            state = solve_prbm_state(cable_force_n, moment_arm_mm=args.moment_arm_mm)
            plot_finger_state(ax, state, color=colors[index], label=f"{cable_force_n:.2f} N")
    else:
        state = solve_prbm_state(args.cable_force_n, moment_arm_mm=args.moment_arm_mm)
        plot_finger_state(ax, state, color="#d97706", label="Analytical PRBM")
        ax.text(
            1.02,
            1.0,
            format_state_summary(state),
            transform=ax.transAxes,
            va="top",
            fontsize=10,
            bbox={"boxstyle": "round,pad=0.4", "facecolor": "#fff7ed", "edgecolor": "#c2410c"},
        )

    ax.legend(loc="upper left")
    ax.set_title("Finger PRBM Response")
    fig.tight_layout()
    if not args.no_show:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
