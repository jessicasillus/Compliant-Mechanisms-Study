from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
import tempfile
import tkinter as tk
from dataclasses import dataclass, field
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Optional

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
SIMULATION_DIR = REPO_ROOT / "finger_sim_freecad"
if str(SIMULATION_DIR) not in sys.path:
    sys.path.insert(0, str(SIMULATION_DIR))

from Finger_PRBM import (
    BASE_BLOCK_LENGTH_MM,
    COMPLIANT_SEGMENT_LENGTH_MM,
    DEFAULT_CABLE_FORCE_N,
    DEFAULT_FORCE_MULTIPLIERS,
    DEFAULT_INTERMEDIATE_STEPS,
    DEFAULT_MOMENT_ARM_MM,
    JOINT_THICKNESSES_MM,
    N_JOINTS,
    RIGID_SEGMENT_LENGTH_MM,
    _advance,
    build_target_cable_forces,
    solve_prbm_state,
)
from freecad_runtime import find_freecadcmd

FREECAD_SCRIPT_PATH = SIMULATION_DIR / "Finger_Sim_FreeCAD.py"
FREECAD_DOCUMENT_PATH = SIMULATION_DIR / "Finger_Sim_VM2_FEM.FCStd"
FEA_STAGE_CSV_PATH = SIMULATION_DIR / "Finger_Sim_VM2_FEA_stages.csv"
BASE_CABLE_FORCE_N = 1.0

# Default path for experimental joint-angle CSV (from Video_Analysis.py pipeline).
# Edit this to point at your actual output file, or use the "Load Experimental" button.
DEFAULT_EXPERIMENTAL_CSV_PATH = SCRIPT_DIR / "joint_angles.csv"
GRAPH_EXPERIMENTAL_SERIES_PATH = SCRIPT_DIR / "_graph_experimental_reference_2026-03-30.png"

BACKGROUND = "#f7f3eb"
PANEL = "#fffdf8"
EDGE = "#cfc4b5"
ANALYTICAL_COLOR = "#c26a00"
TWENTY_SPRING_COLOR = "#7b2d8b"
FEA_COLOR = "#1f6aa5"
EXPERIMENTAL_COLOR = "#2ca02c"
DISCREPANCY_COLOR = "#b42318"

Point2D = tuple[float, float]
Segment2D = tuple[Point2D, Point2D]

# ─────────────────────────────────────────────────────────────────────────────
#  Video-Analysis physics — used to convert motor Δ (deg) → cable tension (N)
#  Parameters match Video_Analysis.py exactly.
# ─────────────────────────────────────────────────────────────────────────────
_VA_lj        = 15e-3   # flexible joint length        [m]
_VA_h         = 6.5e-3  # tendon moment arm            [m]
_VA_bw        = 10e-3   # finger width                 [m]
_VA_JH        = np.array([2.50e-3, 3.25e-3, 3.25e-3, 3.50e-3])  # joint heights [m]
_VA_E_eff     = 2.5e6   # effective joint modulus      [Pa]
_VA_RP        = 0.015   # effective pulley radius      [m]
_VA_GAIN      = 1.00    # motor-to-cable-angle gain    [—]
_VA_C_TENDON  = 0.005   # tendon compliance            [m/N]
_VA_PRETENSION = 16.0   # resting pretension           [deg equivalent]

# Pre-compute joint stiffnesses from Video_Analysis model
_VA_Ij = (1.0 / 12.0) * _VA_bw * _VA_JH ** 3          # [m^4]
_VA_LK = _VA_E_eff * _VA_Ij / _VA_lj                   # [N·m/rad]

# Analytical closed-form for the uncoupled case (no gravity loads F1–F4=0):
#   each joint: th_i = Fc·h / lk_i
#   tendon constraint: h·Σth_i - RP·DPhi + c_t·Fc = 0
#   → Fc = RP·DPhi / (h²·Σ(1/lk_i) + c_t)
_VA_SUM_FLEX = float(np.sum(1.0 / _VA_LK))             # rad/(N·m)
_VA_DENOM    = _VA_h ** 2 * _VA_SUM_FLEX + _VA_C_TENDON  # m/N

# Experimental points digitized from the March 30, 2026 comparison figure.
# These correspond to the motor-angle increments used during recording:
# 0, 15, 30, 45, 60, 75, 90, 105, 120 degrees from open.
GRAPH_MOTOR_DELTAS_DEG = (0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 105.0, 120.0)
GRAPH_EXPERIMENTAL_JOINTS_DEG = (
    (0.0, 11.0, 22.0, 31.5, 52.0, 64.0, 77.0, 90.0, 106.0),
    (0.0, 8.5, 15.0, 21.0, 25.0, 31.0, 35.5, 38.5, 42.5),
    (0.0, 9.0, 14.0, 18.0, 27.0, 31.0, 31.0, 35.0, 38.0),
    (0.0, 6.0, 11.0, 14.0, 17.0, 24.0, 25.5, 29.0, 29.0),
)


def _va_dphi(delta_deg: float) -> float:
    """Motor delta (deg) → DeltaPhi (rad) after applying gain."""
    return math.radians(delta_deg) * _VA_GAIN


def va_cable_force_from_delta(delta_deg: float) -> float:
    """
    Cable tension (N) for a given motor delta increment (deg from open).
    Uses the Video_Analysis PRBM physics; returns the CHANGE from pretension
    so that delta_deg=0 → Fc_delta=0 N (consistent with PRBM cable_force=0).
    """
    dphi_pre  = _va_dphi(_VA_PRETENSION)
    dphi_tot  = dphi_pre + _va_dphi(delta_deg)
    fc_pre    = _VA_RP * dphi_pre  / _VA_DENOM
    fc_tot    = _VA_RP * dphi_tot  / _VA_DENOM
    return max(0.0, fc_tot - fc_pre)


def cable_length_mm_from_delta(delta_deg: float) -> float:
    return 1000.0 * _VA_RP * _va_dphi(delta_deg)


def delta_deg_from_cable_length(cable_length_mm: float) -> float:
    return math.degrees((float(cable_length_mm) / 1000.0) / _VA_RP)


def va_cable_force_from_length(cable_length_mm: float) -> float:
    return va_cable_force_from_delta(delta_deg_from_cable_length(cable_length_mm))


def va_joint_angles_from_delta(delta_deg: float) -> tuple[float, ...]:
    """
    PRBM joint angle changes (deg) relative to pretension for each of the
    4 joints, using Video_Analysis physics. Returns (th1, th2, th3, th4).
    Angles are positive for closing (to match the experimental CSV sign).
    """
    dphi_pre = _va_dphi(_VA_PRETENSION)
    dphi_tot = dphi_pre + _va_dphi(delta_deg)
    fc_pre   = _VA_RP * dphi_pre / _VA_DENOM
    fc_tot   = _VA_RP * dphi_tot / _VA_DENOM
    angles = tuple(
        math.degrees((fc_tot - fc_pre) * _VA_h / lk)
        for lk in _VA_LK
    )
    return angles


# ─────────────────────────────────────────────────────────────────────────────
#  Experimental data structures
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ExperimentalPoint:
    target_delta_deg: float
    cable_length_mm: float
    cable_force_n: float          # computed from Video_Analysis physics
    joint_deltas_deg: tuple[float, ...]  # (Δθ1, Δθ2, Δθ3, Δθ4) from video
    tip_angle_deg: float          # Σ joint deltas (same sign convention as PRBM)


@dataclass
class ExperimentalSeries:
    path: Path
    points: list[ExperimentalPoint] = field(default_factory=list)

    @property
    def cable_forces_n(self) -> np.ndarray:
        return np.array([p.cable_force_n for p in self.points])

    @property
    def tip_angles_deg(self) -> np.ndarray:
        return np.array([p.tip_angle_deg for p in self.points])


@dataclass(frozen=True)
class ModelFitStats:
    label: str
    mae_deg: float
    rmse_deg: float
    max_abs_err_deg: float


def load_experimental_csv(csv_path: Path) -> ExperimentalSeries:
    """
    Read a joint_angles.csv produced by Video_Analysis.py and return an
    ExperimentalSeries.  The joint angle delta columns are used directly;
    cable force is derived from the motor delta via Video_Analysis physics.
    """
    series = ExperimentalSeries(path=csv_path)
    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            delta_deg = float(row["target_delta"])
            cable_len = float(row["cable_length_mm"])
            fc        = va_cable_force_from_delta(delta_deg)

            # Read experimental delta angles (change from open position)
            joint_deltas: list[float] = []
            for i in range(1, 5):
                key = f"theta_{i}_exp_delta_deg"
                if key in row:
                    joint_deltas.append(float(row[key]))
            if not joint_deltas:
                continue

            # Tip angle = cumulative sum of joint bends.
            # Experimental deltas are negative for closing; keep that convention.
            tip = float(sum(joint_deltas))

            series.points.append(
                ExperimentalPoint(
                    target_delta_deg=delta_deg,
                    cable_length_mm=cable_len,
                    cable_force_n=fc,
                    joint_deltas_deg=tuple(joint_deltas),
                    tip_angle_deg=tip,
                )
            )
    series.points.sort(key=lambda p: (p.cable_force_n, p.target_delta_deg))
    return series


def graph_experimental_series() -> ExperimentalSeries:
    series = ExperimentalSeries(path=GRAPH_EXPERIMENTAL_SERIES_PATH)
    joint_rows = [tuple(float(v) for v in row) for row in GRAPH_EXPERIMENTAL_JOINTS_DEG]
    for index, delta_deg in enumerate(GRAPH_MOTOR_DELTAS_DEG):
        joint_deltas = tuple(-row[index] for row in joint_rows)
        cable_length_mm = cable_length_mm_from_delta(delta_deg)
        series.points.append(
            ExperimentalPoint(
                target_delta_deg=float(delta_deg),
                cable_length_mm=float(cable_length_mm),
                cable_force_n=va_cable_force_from_delta(delta_deg),
                joint_deltas_deg=joint_deltas,
                tip_angle_deg=float(sum(joint_deltas)),
            )
        )
    return series


# ─────────────────────────────────────────────────────────────────────────────
#  Display state and FEA surrogate
# ─────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class DisplayState:
    name: str
    cable_force_n: float
    tip_angle_deg: float
    joint_bends_deg: tuple[float, ...]
    segment_angles_deg: tuple[float, ...]
    segment_centers_mm: tuple[Point2D, ...]
    rigid_segments_mm: tuple[Segment2D, ...]
    centerline_points_mm: tuple[Point2D, ...]


@dataclass(frozen=True)
class ComparisonMetrics:
    tip_error_deg: float
    tip_error_pct: float
    joint_errors_deg: tuple[float, ...]
    rms_joint_error_deg: float
    max_joint_error_deg: float


def _segment_from_center(center_mm: Point2D, reported_angle_deg: float) -> Segment2D:
    geometry_angle_rad = math.radians(-reported_angle_deg)
    dx = 0.5 * RIGID_SEGMENT_LENGTH_MM * math.cos(geometry_angle_rad)
    dz = 0.5 * RIGID_SEGMENT_LENGTH_MM * math.sin(geometry_angle_rad)
    return ((center_mm[0] - dx, center_mm[1] - dz), (center_mm[0] + dx, center_mm[1] + dz))


def _display_state_from_values(
    name: str,
    cable_force_n: float,
    tip_angle_deg: float,
    joint_bends_deg: tuple[float, ...],
    segment_angles_deg: tuple[float, ...],
    segment_centers_mm: tuple[Point2D, ...],
) -> DisplayState:
    rigid_segments = tuple(
        _segment_from_center(center_mm, angle_deg)
        for center_mm, angle_deg in zip(segment_centers_mm, segment_angles_deg)
    )
    centerline_points = [(0.0, 0.0), (BASE_BLOCK_LENGTH_MM, 0.0)]
    for segment_start_mm, segment_end_mm in rigid_segments:
        centerline_points.extend([segment_start_mm, segment_end_mm])
    return DisplayState(
        name=name,
        cable_force_n=float(cable_force_n),
        tip_angle_deg=float(tip_angle_deg),
        joint_bends_deg=tuple(float(v) for v in joint_bends_deg),
        segment_angles_deg=tuple(float(v) for v in segment_angles_deg),
        segment_centers_mm=tuple((float(x), float(z)) for x, z in segment_centers_mm),
        rigid_segments_mm=rigid_segments,
        centerline_points_mm=tuple(centerline_points),
    )


def display_state_from_joint_bends(
    name: str,
    cable_force_n: float,
    joint_bends_deg: tuple[float, ...],
    *,
    base_block_length_mm: float = BASE_BLOCK_LENGTH_MM,
    compliant_segment_length_mm: float = COMPLIANT_SEGMENT_LENGTH_MM,
    rigid_segment_length_mm: float = RIGID_SEGMENT_LENGTH_MM,
) -> DisplayState:
    current_point: Point2D = (float(base_block_length_mm), 0.0)
    centerline_points: list[Point2D] = [(0.0, 0.0), current_point]
    segment_centers: list[Point2D] = []
    rigid_segments: list[Segment2D] = []
    segment_angles_deg: list[float] = []
    cumulative_angle_deg = 0.0

    for joint_bend_deg in joint_bends_deg:
        segment_start = _advance(
            current_point,
            float(compliant_segment_length_mm),
            -(cumulative_angle_deg + 0.5 * float(joint_bend_deg)),
        )
        centerline_points.append(segment_start)
        cumulative_angle_deg += float(joint_bend_deg)
        segment_angles_deg.append(cumulative_angle_deg)
        segment_center = _advance(
            segment_start,
            0.5 * float(rigid_segment_length_mm),
            -cumulative_angle_deg,
        )
        segment_end = _advance(
            segment_start,
            float(rigid_segment_length_mm),
            -cumulative_angle_deg,
        )
        segment_centers.append(segment_center)
        rigid_segments.append((segment_start, segment_end))
        centerline_points.append(segment_end)
        current_point = segment_end

    return DisplayState(
        name=name,
        cable_force_n=float(cable_force_n),
        tip_angle_deg=segment_angles_deg[-1] if segment_angles_deg else 0.0,
        joint_bends_deg=tuple(float(v) for v in joint_bends_deg),
        segment_angles_deg=tuple(segment_angles_deg),
        segment_centers_mm=tuple(segment_centers),
        rigid_segments_mm=tuple(rigid_segments),
        centerline_points_mm=tuple(centerline_points),
    )


class FEASurrogate:
    def __init__(self, csv_path: Path):
        self.csv_path = Path(csv_path)
        self.rows = self._load_rows(self.csv_path)
        self.force_points = np.array([row["cable_force_n"] for row in self.rows], dtype=float)
        self._monotonic_abs_cache: dict[str, np.ndarray] = {}
        if len(self.force_points) < 2:
            raise RuntimeError("The FEA stage CSV must contain at least two unique cable-force rows.")
        self.max_force_n = float(self.force_points[-1])

    @staticmethod
    def _load_rows(csv_path: Path) -> list[dict]:
        if not csv_path.is_file():
            raise FileNotFoundError(f"Missing FEA stage CSV: {csv_path}")
        with csv_path.open("r", newline="", encoding="utf-8") as f:
            raw_rows = list(csv.DictReader(f))
        if not raw_rows:
            raise RuntimeError(f"The FEA stage CSV is empty: {csv_path}")
        deduped: dict[float, dict] = {}
        for raw in raw_rows:
            row: dict = {"source": raw["source"]}
            for k, v in raw.items():
                if k != "source":
                    row[k] = float(v)
            deduped[round(float(row["cable_force_n"]), 6)] = row
        return [deduped[k] for k in sorted(deduped)]

    def interpolate_state(self, cable_force_n: float) -> DisplayState:
        cf = float(np.clip(cable_force_n, self.force_points[0], self.force_points[-1]))
        joint_bends_deg = tuple(
            -self._interp_abs(cf, f"joint{i}_bend_deg") for i in range(1, N_JOINTS + 1)
        )
        return display_state_from_joint_bends(
            name="FEA (CalculiX)",
            cable_force_n=cf,
            joint_bends_deg=joint_bends_deg,
        )

    def tip_angle_curve(self, forces: np.ndarray) -> np.ndarray:
        joint_curves = [
            np.interp(
                forces,
                self.force_points,
                self._monotonic_abs_values(f"joint{i}_bend_deg"),
            )
            for i in range(1, N_JOINTS + 1)
        ]
        return -np.sum(np.vstack(joint_curves), axis=0)

    def _interp(self, force: float, key: str) -> float:
        return float(np.interp(
            force,
            self.force_points,
            np.array([r[key] for r in self.rows], dtype=float),
        ))

    def _interp_abs(self, force: float, key: str) -> float:
        return float(np.interp(
            force,
            self.force_points,
            self._monotonic_abs_values(key),
        ))

    def _monotonic_abs_values(self, key: str) -> np.ndarray:
        values = self._monotonic_abs_cache.get(key)
        if values is None:
            raw = np.abs(np.array([r[key] for r in self.rows], dtype=float))
            values = np.maximum.accumulate(raw)
            self._monotonic_abs_cache[key] = values
        return values

    def prbm1_tip_curve(self, forces: np.ndarray) -> np.ndarray:
        """1-spring PRBM tip angles evaluated analytically at the requested forces."""
        return np.array([analytical_display_state(float(f)).tip_angle_deg for f in forces])

    def prbm20_tip_curve(self, forces: np.ndarray) -> np.ndarray:
        """20-spring PRBM tip angles evaluated analytically at the requested forces."""
        return np.array([solve_20spring_state(float(f)).tip_angle_deg for f in forces])


def compute_metrics(analytical: DisplayState, fea: DisplayState) -> ComparisonMetrics:
    joint_errors = tuple(a - b for a, b in zip(analytical.joint_bends_deg, fea.joint_bends_deg))
    tip_err = analytical.tip_angle_deg - fea.tip_angle_deg
    denom = max(abs(fea.tip_angle_deg), 1e-6)
    return ComparisonMetrics(
        tip_error_deg=tip_err,
        tip_error_pct=100.0 * tip_err / denom,
        joint_errors_deg=joint_errors,
        rms_joint_error_deg=math.sqrt(sum(e * e for e in joint_errors) / len(joint_errors)),
        max_joint_error_deg=max(abs(e) for e in joint_errors),
    )


def experimental_fit_stats(exp_series: ExperimentalSeries) -> tuple[ModelFitStats, ModelFitStats]:
    exp_forces = exp_series.cable_forces_n
    exp_tips = exp_series.tip_angles_deg
    one_spring = np.array(
        [analytical_display_state(force_n).tip_angle_deg for force_n in exp_forces],
        dtype=float,
    )
    twenty_spring = np.array(
        [solve_20spring_state(force_n).tip_angle_deg for force_n in exp_forces],
        dtype=float,
    )

    def _stats(label: str, predicted: np.ndarray) -> ModelFitStats:
        errors = predicted - exp_tips
        abs_errors = np.abs(errors)
        return ModelFitStats(
            label=label,
            mae_deg=float(np.mean(abs_errors)),
            rmse_deg=float(np.sqrt(np.mean(errors * errors))),
            max_abs_err_deg=float(np.max(abs_errors)),
        )

    return _stats("1-Spring PRBM", one_spring), _stats("20-Spring PRBM", twenty_spring)


# ─────────────────────────────────────────────────────────────────────────────
#  FEA CSV export — ONE simulation ramped through all target forces
# ─────────────────────────────────────────────────────────────────────────────

def export_fea_stage_csv(
    *,
    csv_path: Path = FEA_STAGE_CSV_PATH,
    fcstd_path: Path = FREECAD_DOCUMENT_PATH,
    peak_cable_force_n: float = build_target_cable_forces(
        BASE_CABLE_FORCE_N,
        force_multipliers=DEFAULT_FORCE_MULTIPLIERS,
        intermediate_steps=DEFAULT_INTERMEDIATE_STEPS,
    )[-1],
    moment_arm_mm: float = DEFAULT_MOMENT_ARM_MM,
    extra_target_forces_n: Optional[list[float]] = None,
) -> Path:
    """
    Run a SINGLE FreeCAD/CalculiX simulation ramped from 0 → peak force,
    saving intermediate results at each target force level.  This avoids the
    per-force instability that occurred when each level was a separate cold run.
    """
    freecadcmd = find_freecadcmd()
    if freecadcmd is None:
        raise RuntimeError("freecadcmd not found. Set FREECADCMD or install FreeCAD.")

    csv_path = Path(csv_path).resolve()
    working_dir = Path(tempfile.gettempdir()) / "finger_fea_combined_refresh"
    working_dir.mkdir(parents=True, exist_ok=True)
    case_fcstd   = working_dir / "Finger_Sim_Combined.FCStd"
    case_csv     = working_dir / "Finger_Sim_Combined_segments.csv"

    target_forces = build_target_cable_forces(
        BASE_CABLE_FORCE_N,
        force_multipliers=DEFAULT_FORCE_MULTIPLIERS,
        intermediate_steps=DEFAULT_INTERMEDIATE_STEPS,
    )
    if extra_target_forces_n:
        target_forces = sorted(set(target_forces) | set(extra_target_forces_n))

    positive_targets = [f for f in target_forces if f > 0.0]
    if not positive_targets:
        raise RuntimeError("No positive cable forces to simulate.")

    # Express each target as a multiplier of BASE_CABLE_FORCE_N
    multipliers = sorted({round(f / BASE_CABLE_FORCE_N, 9) for f in positive_targets})

    script_args = [
        str(FREECAD_SCRIPT_PATH.name),
        "--cable-force-n", f"{BASE_CABLE_FORCE_N:.12g}",
        "--force-multipliers", ",".join(f"{m:.9g}" for m in multipliers),
        "--intermediate-steps", "0",
        "--stages", str(max(6, len(multipliers))),
        "--no-stage-views",
        "--no-show",
        "--fcstd", str(case_fcstd),
        "--working-dir", str(working_dir),
        "--segment-csv", str(case_csv),
        "--moment-arm-mm", f"{moment_arm_mm:.12g}",
    ]
    code = (
        "import runpy, sys; "
        f"sys.argv={script_args!r}; "
        f"runpy.run_path({str(FREECAD_SCRIPT_PATH)!r}, run_name='__main__')"
    )
    completed = subprocess.run(
        [freecadcmd, "-c", code],
        cwd=str(SCRIPT_DIR),
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        msg = (completed.stderr or completed.stdout or "").strip()
        raise RuntimeError(f"FreeCAD simulation failed: {msg or completed.returncode}")

    if not case_csv.is_file():
        raise RuntimeError("FreeCAD simulation did not produce a segment CSV.")

    # Copy the combined CSV to the final destination
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    import shutil
    shutil.copy2(str(case_csv), str(csv_path))
    return csv_path


def ensure_fea_stage_csv(*, refresh: bool = False, extra_forces: Optional[list[float]] = None) -> Path:
    if refresh or not FEA_STAGE_CSV_PATH.is_file():
        export_fea_stage_csv(extra_target_forces_n=extra_forces)
    return FEA_STAGE_CSV_PATH


def analytical_display_state(cable_force_n: float) -> DisplayState:
    st = solve_prbm_state(cable_force_n, moment_arm_mm=DEFAULT_MOMENT_ARM_MM)
    return DisplayState(
        name="1-Spring PRBM",
        cable_force_n=st.cable_force_n,
        tip_angle_deg=st.tip_angle_deg,
        joint_bends_deg=st.joint_bends_deg,
        segment_angles_deg=st.segment_angles_deg,
        segment_centers_mm=st.segment_centers_mm,
        rigid_segments_mm=st.rigid_segments_mm,
        centerline_points_mm=st.centerline_points_mm,
    )


# ── 20-spring PRBM static analytical model (from Finger_Sim_20springs.py) ─────
# Each flexible gap is divided into N_SUB=20 sub-springs in series, which has
# the same total stiffness as a single spring.  The only physics difference from
# the 1-spring PRBM is the per-joint moment arm:
#   wrap_r_i = H_CABLE − T_GAP[i]/2   (≈ 4.25–4.5 mm vs the fixed 8 mm)
# Geometry also differs: base block is 25 mm (vs 20 mm in 1-spring PRBM).
_20S_H_CABLE_MM  = 6.0
_20S_T_GAP_MM    = (3.0, 3.25, 3.25, 3.5)
_20S_JS_BASE_NMM = 50.0      # N·mm/rad — matches NOMINAL_JOINT_STIFFNESS_NMM_PER_RAD
_20S_T_NOMINAL   = 3.25      # mm       — matches NOMINAL_JOINT_THICKNESS_MM
_20S_L_BASE_MM   = 25.0
_20S_L_LNK_MM    = 25.0
_20S_L_GAP_MM    = 15.0


def solve_20spring_state(cable_force_n: float) -> DisplayState:
    """Static equilibrium of the 20-sub-spring PRBM from Finger_Sim_20springs.py."""
    bends_deg: list[float] = []
    for t_mm in _20S_T_GAP_MM:
        wrap_r_mm = _20S_H_CABLE_MM - t_mm / 2.0
        js_gap = _20S_JS_BASE_NMM * (t_mm / _20S_T_NOMINAL) ** 3
        bends_deg.append(-math.degrees(cable_force_n * wrap_r_mm / js_gap))
    return display_state_from_joint_bends(
        name="20-Spring PRBM",
        cable_force_n=float(cable_force_n),
        joint_bends_deg=tuple(float(b) for b in bends_deg),
    )


def format_summary(force_n: float, fea_surrogate: FEASurrogate) -> str:
    a = analytical_display_state(force_n)
    b = fea_surrogate.interpolate_state(force_n)
    m = compute_metrics(a, b)
    return (
        f"Force: {force_n:.3f} N\n"
        f"Analytical tip angle: {a.tip_angle_deg:.3f} deg\n"
        f"FEA tip angle: {b.tip_angle_deg:.3f} deg\n"
        f"Tip discrepancy: {m.tip_error_deg:.3f} deg ({m.tip_error_pct:.2f}%)\n"
        f"RMS joint discrepancy: {m.rms_joint_error_deg:.3f} deg\n"
        f"Max joint discrepancy: {m.max_joint_error_deg:.3f} deg"
    )


# ─────────────────────────────────────────────────────────────────────────────
#  GUI application
# ─────────────────────────────────────────────────────────────────────────────

def experimental_display_state(point: ExperimentalPoint) -> DisplayState:
    return display_state_from_joint_bends(
        name="Experimental",
        cable_force_n=point.cable_force_n,
        joint_bends_deg=point.joint_deltas_deg,
    )


class FingerDiscrepancyApp:
    def __init__(self, root: tk.Tk,
                 fea_surrogate: Optional[FEASurrogate] = None,
                 exp_series: Optional[ExperimentalSeries] = None):
        self.root = root
        self.fea_surrogate = fea_surrogate
        self.exp_series: Optional[ExperimentalSeries] = exp_series

        # FEA stage buttons (only shown when FEA is available)
        self.stage_forces_n: tuple[float, ...] = (
            tuple(float(v) for v in fea_surrogate.force_points)
            if fea_surrogate is not None else ()
        )
        # Force-slider range: FEA range if available, else experimental or default
        if fea_surrogate is not None:
            self.fc_lo = float(self.stage_forces_n[0])
            self.fc_hi = float(self.stage_forces_n[-1])
        elif exp_series is not None and exp_series.points:
            self.fc_lo = 0.0
            self.fc_hi = float(exp_series.cable_forces_n.max()) * 1.1
        else:
            self.fc_lo = 0.0
            self.fc_hi = float(DEFAULT_CABLE_FORCE_N)

        self.force_var       = tk.DoubleVar(value=self.fc_hi)
        self.force_entry_var = tk.StringVar(value=f"{self.fc_hi:.3f}")
        self.snap_var        = tk.BooleanVar(value=False)
        self.show_exp_var    = tk.BooleanVar(value=bool(exp_series and exp_series.points))
        self.show_fea_var    = tk.BooleanVar(value=False)
        self.summary_var     = tk.StringVar()
        src = fea_surrogate.csv_path.name if fea_surrogate else "no FEA loaded"
        self.status_var = tk.StringVar(value=f"FEA source: {src}")

        self.root.title("Compliant Finger: Experimental vs 1-Spring vs 20-Spring PRBM")
        self.root.configure(bg=BACKGROUND)
        self._configure_style()
        self._build_layout()
        self.root.bind("<Configure>", self._on_window_resize)
        self._on_window_resize()
        self._update_visuals()

    def _configure_style(self) -> None:
        s = ttk.Style()
        if "clam" in s.theme_names():
            s.theme_use("clam")
        s.configure("TFrame", background=BACKGROUND)
        s.configure("Panel.TFrame", background=PANEL, relief="solid", borderwidth=1)
        s.configure("TLabel", background=BACKGROUND)
        s.configure("Panel.TLabel", background=PANEL)
        s.configure("TCheckbutton", background=BACKGROUND)
        s.configure("Accent.TButton", padding=(10, 6))

    def _build_layout(self) -> None:
        container = ttk.Frame(self.root)
        container.pack(fill="both", expand=True, padx=14, pady=14)

        # ── Controls ──────────────────────────────────────────────────────────
        ctrl = ttk.Frame(container, style="Panel.TFrame", padding=12)
        ctrl.pack(fill="x")

        ttk.Label(ctrl, text="Compliant Finger: Experimental vs 1-Spring vs 20-Spring PRBM",
                  style="Panel.TLabel", font=("Segoe UI Semibold", 14)
                  ).grid(row=0, column=0, columnspan=6, sticky="w")

        ttk.Label(ctrl, text="Compare the 1-spring PRBM, 20-spring PRBM, and optional FEA overlay.",
                  style="Panel.TLabel"
                  ).grid(row=1, column=0, columnspan=6, sticky="w", pady=(2, 10))

        ttk.Label(ctrl, text="Cable Force [N]", style="Panel.TLabel"
                  ).grid(row=2, column=0, sticky="w")
        ttk.Scale(ctrl, from_=self.fc_lo, to=self.fc_hi,
                  orient="horizontal", variable=self.force_var,
                  command=self._on_scale_move
                  ).grid(row=2, column=1, columnspan=2, sticky="ew", padx=(8, 12))

        force_entry = ttk.Entry(ctrl, width=10, textvariable=self.force_entry_var)
        force_entry.grid(row=2, column=3, sticky="w")
        force_entry.bind("<Return>", self._on_entry_commit)

        ttk.Button(ctrl, text="Apply", command=self._on_apply_clicked,
                   style="Accent.TButton").grid(row=2, column=4, sticky="w", padx=(8, 0))

        ttk.Button(ctrl, text="Load Experimental…",
                   command=self._on_load_experimental
                   ).grid(row=2, column=5, sticky="w", padx=(16, 0))

        # Stage buttons (only when FEA is available)
        sf = ttk.Frame(ctrl, style="Panel.TFrame")
        sf.grid(row=3, column=0, columnspan=6, sticky="w", pady=(10, 0))
        if self.stage_forces_n:
            ttk.Label(sf, text="FEA Stages", style="Panel.TLabel").pack(side="left", padx=(0, 8))
            for fn in self.stage_forces_n:
                ttk.Button(sf, text=f"{fn:.2f} N",
                           command=lambda f=fn: self._set_force(f)).pack(side="left", padx=(0, 4))

        ttk.Checkbutton(ctrl, text="Snap slider to nearest FEA stage",
                        variable=self.snap_var, command=self._update_visuals
                        ).grid(row=4, column=0, columnspan=3, sticky="w", pady=(10, 0))
        ttk.Checkbutton(ctrl, text="Show experimental data",
                        variable=self.show_exp_var, command=self._update_visuals
                        ).grid(row=4, column=3, columnspan=3, sticky="w", pady=(10, 0))
        if self.fea_surrogate is not None:
            ttk.Checkbutton(ctrl, text="Show FEA overlay",
                            variable=self.show_fea_var, command=self._update_visuals
                            ).grid(row=5, column=0, columnspan=3, sticky="w", pady=(6, 0))

        for c in range(6):
            ctrl.columnconfigure(c, weight=1 if c == 1 else 0)

        # ── Metrics panel ──────────────────────────────────────────────────────
        mp = ttk.Frame(container, style="Panel.TFrame", padding=12)
        mp.pack(fill="x", pady=(10, 10))
        self.summary_label = ttk.Label(mp, textvariable=self.summary_var,
                                       style="Panel.TLabel", justify="left",
                                       font=("Consolas", 10), wraplength=1100)
        self.summary_label.pack(anchor="w", fill="x")

        # ── Figure ────────────────────────────────────────────────────────────
        self.figure = Figure(figsize=(13.6, 10.4), dpi=100, facecolor=PANEL, constrained_layout=True)
        self.figure.set_constrained_layout_pads(
            w_pad=4.0 / 72.0,
            h_pad=6.0 / 72.0,
            wspace=0.10,
            hspace=0.10,
        )
        grid = self.figure.add_gridspec(3, 2, width_ratios=(1.30, 1.0))
        self.ax_shape       = self.figure.add_subplot(grid[:, 0])
        self.ax_response    = self.figure.add_subplot(grid[0, 1])
        self.ax_discrepancy = self.figure.add_subplot(grid[1, 1])
        self.ax_joint_err   = self.figure.add_subplot(grid[2, 1])
        self.canvas = FigureCanvasTkAgg(self.figure, master=container)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # ── Table ─────────────────────────────────────────────────────────────
        tp = ttk.Frame(container, style="Panel.TFrame", padding=12)
        tp.pack(fill="x", pady=(10, 6))
        ttk.Label(tp, text="Current Discrepancy Table",
                  style="Panel.TLabel", font=("Segoe UI Semibold", 11)).pack(anchor="w")

        cols = ("quantity", "prbm_1s", "prbm_20s", "experimental", "err_1s", "err_20s")
        self.table = ttk.Treeview(tp, columns=cols, show="headings", height=N_JOINTS + 1)
        self.table.heading("quantity",     text="Quantity")
        self.table.heading("prbm_1s",      text="1-Spring PRBM")
        self.table.heading("prbm_20s",     text="20-Spring PRBM")
        self.table.heading("experimental", text="Experimental")
        self.table.heading("err_1s",       text="1-Spring − Exp")
        self.table.heading("err_20s",      text="20-Spring − Exp")
        self.table.column("quantity", width=120, anchor="w")
        for c in ("prbm_1s", "prbm_20s", "experimental", "err_1s", "err_20s"):
            self.table.column(c, width=115, anchor="e")
        self.table.pack(fill="x")

        self.status_label = ttk.Label(container, textvariable=self.status_var,
                                      justify="left", wraplength=1100)
        self.status_label.pack(fill="x")

    # ── Control callbacks ──────────────────────────────────────────────────────

    def _on_window_resize(self, _event=None) -> None:
        width = max(self.root.winfo_width() - 120, 520)
        if hasattr(self, "summary_label"):
            self.summary_label.configure(wraplength=width)
        if hasattr(self, "status_label"):
            self.status_label.configure(wraplength=width)

    def _set_force(self, f: float) -> None:
        self.force_var.set(float(f))
        self.force_entry_var.set(f"{float(f):.3f}")
        self._update_visuals()

    def _on_scale_move(self, _: str) -> None:
        self.force_entry_var.set(f"{self.force_var.get():.3f}")
        self._update_visuals()

    def _on_apply_clicked(self) -> None:
        self._on_entry_commit()

    def _on_entry_commit(self, _event=None) -> None:
        try:
            v = float(self.force_entry_var.get())
        except ValueError:
            self.force_entry_var.set(f"{self.force_var.get():.3f}")
            return
        v = float(np.clip(v, self.fc_lo, self.fc_hi))
        self._set_force(v)

    def _on_load_experimental(self) -> None:
        path = filedialog.askopenfilename(
            title="Select joint_angles.csv from Video_Analysis",
            filetypes=[("CSV", "*.csv"), ("All", "*.*")],
        )
        if not path:
            return
        try:
            self.exp_series = load_experimental_csv(Path(path))
            self.show_exp_var.set(True)
            self.status_var.set(
                f"Experimental: {Path(path).name}  "
                f"({len(self.exp_series.points)} points, "
                f"Fc range {self.exp_series.cable_forces_n.min():.3f}–"
                f"{self.exp_series.cable_forces_n.max():.3f} N)"
            )
            self._update_visuals()
        except Exception as exc:
            messagebox.showerror("Experimental CSV Error", str(exc))

    def _current_force(self) -> float:
        f = float(np.clip(self.force_var.get(), self.fc_lo, self.fc_hi))
        if self.snap_var.get() and self.stage_forces_n:
            f = min(self.stage_forces_n, key=lambda s: abs(s - f))
            self.force_var.set(f)
            self.force_entry_var.set(f"{f:.3f}")
        return f

    # ── Closest experimental point to a given cable force ─────────────────────

    def _closest_exp_point(self, cable_force_n: float) -> Optional[ExperimentalPoint]:
        if not self.exp_series or not self.exp_series.points:
            return None
        return min(self.exp_series.points, key=lambda p: abs(p.cable_force_n - cable_force_n))

    # ── Shape plot ────────────────────────────────────────────────────────────

    def _plot_state(self, ax, state: DisplayState, *, color: str, label: str,
                    linestyle: str, linewidth: float) -> None:
        xs = [p[0] for p in state.centerline_points_mm]
        zs = [p[1] for p in state.centerline_points_mm]
        ax.plot(xs, zs, linestyle=linestyle, linewidth=linewidth,
                color=color, label=label, alpha=0.9)
        for seg_s, seg_e in state.rigid_segments_mm:
            ax.plot([seg_s[0], seg_e[0]], [seg_s[1], seg_e[1]],
                    color=color, linewidth=6.0, solid_capstyle="round", alpha=0.85)
        ax.scatter([c[0] for c in state.segment_centers_mm],
                   [c[1] for c in state.segment_centers_mm],
                   color=color, s=34, zorder=3)

    def _update_shape_plot(self, analytical: DisplayState, twenty_spring: DisplayState,
                           fea: Optional[DisplayState] = None) -> None:
        ax = self.ax_shape
        ax.clear()
        ax.set_facecolor(PANEL)
        ax.grid(True, alpha=0.22)
        ax.plot([0.0, BASE_BLOCK_LENGTH_MM], [0.0, 0.0],
                color="black", linewidth=7.0, solid_capstyle="round", label="Base")
        self._plot_state(ax, analytical, color=ANALYTICAL_COLOR,
                         label="1-Spring PRBM", linestyle="-", linewidth=2.8)
        self._plot_state(ax, twenty_spring, color=TWENTY_SPRING_COLOR,
                         label="20-Spring PRBM", linestyle="--", linewidth=2.6)
        if fea is not None and self.show_fea_var.get():
            self._plot_state(ax, fea, color=FEA_COLOR,
                             label="FEA (CalculiX)", linestyle=":", linewidth=2.2)

        if self.show_exp_var.get() and self.exp_series:
            ep = self._closest_exp_point(analytical.cable_force_n)
            if ep is not None:
                self._plot_state(ax, experimental_display_state(ep), color=EXPERIMENTAL_COLOR,
                                 label="Experimental", linestyle="-.", linewidth=2.4)
                delta_label = (
                    f"Exp Δ={ep.target_delta_deg:.0f}°  "
                    f"Fc≈{ep.cable_force_n:.3f} N  "
                    f"tip≈{ep.tip_angle_deg:.1f}°"
                )
                ax.annotate(delta_label, xy=(0.02, 0.04), xycoords="axes fraction",
                            fontsize=8, color=EXPERIMENTAL_COLOR,
                            bbox=dict(boxstyle="round,pad=0.3", facecolor=PANEL,
                                      edgecolor=EXPERIMENTAL_COLOR, alpha=0.85))

        ax.set_title("Finger Shape Overlay")
        ax.set_xlabel("x [mm]")
        ax.set_ylabel("z [mm]")
        ax.tick_params(axis="both", labelsize=8.5)
        ax.legend(loc="upper left", fontsize=7.4, framealpha=0.94,
                  borderpad=0.35, labelspacing=0.32, handlelength=2.0)
        ax.set_aspect("equal", adjustable="box")

    # ── Response plot ─────────────────────────────────────────────────────────

    def _update_response_plot(self, cable_force_n: float) -> None:
        ax = self.ax_response
        ax.clear()
        ax.set_facecolor(PANEL)

        fc_min = self.fc_lo
        fc_max = self.fc_hi
        if self.show_exp_var.get() and self.exp_series and self.exp_series.points:
            fc_max = max(fc_max, self.exp_series.cable_forces_n.max() * 1.05)

        dense = np.linspace(fc_min, fc_max, 300)
        analytical_angles = np.array(
            [analytical_display_state(f).tip_angle_deg for f in dense], dtype=float
        )
        twenty_spring_angles = np.array(
            [solve_20spring_state(f).tip_angle_deg for f in dense], dtype=float
        )

        ax.plot(dense, analytical_angles, color=ANALYTICAL_COLOR, linewidth=2.5,
                label="1-Spring PRBM")
        ax.plot(dense, twenty_spring_angles, color=TWENTY_SPRING_COLOR, linewidth=2.5,
                linestyle="--", label="20-Spring PRBM")
        ax.fill_between(dense, analytical_angles, twenty_spring_angles,
                        color="#d8c0e8", alpha=0.18)

        # Optional FEA overlay with step-aligned PRBM comparison markers
        if self.fea_surrogate is not None and self.show_fea_var.get():
            stages = np.array(self.stage_forces_n)
            fea_stage_tips  = self.fea_surrogate.tip_angle_curve(stages)
            prbm1_stage_tips  = self.fea_surrogate.prbm1_tip_curve(stages)
            prbm20_stage_tips = self.fea_surrogate.prbm20_tip_curve(stages)

            fea_angles = self.fea_surrogate.tip_angle_curve(
                np.clip(dense, stages[0], stages[-1])
            )
            ax.plot(dense, fea_angles, color=FEA_COLOR, linewidth=1.8,
                    linestyle=":", label="FEA (CalculiX)")
            ax.scatter(stages, fea_stage_tips,  color=FEA_COLOR,            s=55, zorder=5, label="_nolegend_")
            ax.scatter(stages, prbm1_stage_tips,  color=ANALYTICAL_COLOR,   s=44, zorder=5,
                       marker="^", alpha=0.85, label="_nolegend_")
            ax.scatter(stages, prbm20_stage_tips, color=TWENTY_SPRING_COLOR, s=44, zorder=5,
                       marker="s", alpha=0.85, label="_nolegend_")

        # Current-force markers
        ax.scatter([cable_force_n], [analytical_display_state(cable_force_n).tip_angle_deg],
                   color=ANALYTICAL_COLOR, s=60, zorder=5)
        ax.scatter([cable_force_n], [solve_20spring_state(cable_force_n).tip_angle_deg],
                   color=TWENTY_SPRING_COLOR, s=60, zorder=5)

        # Experimental data
        if self.show_exp_var.get() and self.exp_series and self.exp_series.points:
            ef = self.exp_series.cable_forces_n
            ea = self.exp_series.tip_angles_deg
            ax.scatter(
                ef,
                np.array([analytical_display_state(f).tip_angle_deg for f in ef], dtype=float),
                color=ANALYTICAL_COLOR,
                s=28,
                alpha=0.85,
                zorder=5,
            )
            ax.scatter(
                ef,
                np.array([solve_20spring_state(f).tip_angle_deg for f in ef], dtype=float),
                color=TWENTY_SPRING_COLOR,
                marker="s",
                s=26,
                alpha=0.85,
                zorder=5,
            )
            if self.fea_surrogate is not None and self.show_fea_var.get():
                ax.scatter(
                    ef,
                    self.fea_surrogate.tip_angle_curve(np.clip(ef, self.stage_forces_n[0], self.stage_forces_n[-1])),
                    color=FEA_COLOR,
                    marker="D",
                    s=24,
                    alpha=0.85,
                    zorder=5,
                )
            ax.scatter(ef, ea, color=EXPERIMENTAL_COLOR, edgecolors="white",
                       linewidths=0.7, s=42, zorder=6, label="Experimental data")
            ax.plot(ef, ea, color=EXPERIMENTAL_COLOR, linewidth=1.1,
                    linestyle=":", alpha=0.55, zorder=4)

        ax.set_title("Experiment vs PRBM Tip Angle", pad=10, fontsize=11)
        ax.set_xlabel("Cable force [N]")
        ax.set_ylabel("Tip angle [deg]")
        ax.grid(True, alpha=0.22)
        ax.tick_params(axis="both", labelsize=8.2)
        ax.margins(x=0.04)
        ax.legend(loc="lower left", fontsize=6.6, framealpha=0.94,
                  borderpad=0.35, labelspacing=0.30, handlelength=2.0)

    # ── Discrepancy plot ──────────────────────────────────────────────────────

    def _update_discrepancy_plot(self, cable_force_n: float) -> None:
        ax = self.ax_discrepancy
        ax.clear()
        ax.set_facecolor(PANEL)

        fc_min = self.fc_lo
        fc_max = self.fc_hi

        has_fea = self.fea_surrogate is not None
        has_exp = (self.show_exp_var.get()
                   and self.exp_series is not None
                   and len(self.exp_series.points) > 0)

        if has_exp:
            fc_max = max(fc_max, self.exp_series.cable_forces_n.max() * 1.05)  # type: ignore[union-attr]

        if has_fea:
            # Primary comparison: both PRBM models vs FEA at the exact solved steps
            stages      = np.array(self.stage_forces_n)
            fea_tips    = self.fea_surrogate.tip_angle_curve(stages)
            prbm1_tips  = self.fea_surrogate.prbm1_tip_curve(stages)
            prbm20_tips = self.fea_surrogate.prbm20_tip_curve(stages)

            err_1s  = np.abs(prbm1_tips  - fea_tips)
            err_20s = np.abs(prbm20_tips - fea_tips)

            ax.plot(stages, err_1s,  color=ANALYTICAL_COLOR,    marker="^", markersize=7,
                    linewidth=2.0, label="|1-Spring − FEA|")
            ax.plot(stages, err_20s, color=TWENTY_SPRING_COLOR, marker="s", markersize=7,
                    linewidth=2.0, linestyle="--", label="|20-Spring − FEA|")

            # Current-force markers (interpolated from stage values)
            cur_fea  = float(np.interp(cable_force_n, stages, fea_tips))
            cur_1s   = abs(analytical_display_state(cable_force_n).tip_angle_deg - cur_fea)
            cur_20s  = abs(solve_20spring_state(cable_force_n).tip_angle_deg     - cur_fea)
            ax.scatter([cable_force_n], [cur_1s],  color=ANALYTICAL_COLOR,    s=52, zorder=5)
            ax.scatter([cable_force_n], [cur_20s], color=TWENTY_SPRING_COLOR, s=52, zorder=5)

            # Background context: continuous |1-spring − 20-spring| difference
            dense = np.linspace(fc_min, fc_max, 300)
            a1s  = np.array([analytical_display_state(f).tip_angle_deg for f in dense], dtype=float)
            a20s = np.array([solve_20spring_state(f).tip_angle_deg     for f in dense], dtype=float)
            ax.plot(dense, np.abs(a1s - a20s), color=DISCREPANCY_COLOR, linewidth=1.0,
                    linestyle=":", alpha=0.4, label="|1-Spring − 20-Spring|")

            rmse_1s  = float(np.sqrt(np.mean(err_1s  ** 2)))
            rmse_20s = float(np.sqrt(np.mean(err_20s ** 2)))
            better   = "1-Spring" if rmse_1s <= rmse_20s else "20-Spring"
            ax.annotate(
                f"Closer to FEA: {better}\nRMSE {rmse_1s:.2f}° vs {rmse_20s:.2f}°",
                xy=(0.02, 0.96), xycoords="axes fraction", va="top", fontsize=8,
                bbox=dict(boxstyle="round,pad=0.3", facecolor=PANEL, edgecolor=EDGE, alpha=0.92),
            )
            title = "PRBM vs FEA Discrepancy (per step)"

        elif has_exp:
            # No FEA: compare PRBM models against experimental data
            exp_forces = self.exp_series.cable_forces_n  # type: ignore[union-attr]
            exp_tips   = self.exp_series.tip_angles_deg  # type: ignore[union-attr]

            prbm_at_exp = np.array(
                [analytical_display_state(f).tip_angle_deg for f in exp_forces], dtype=float
            )
            s20_at_exp = np.array(
                [solve_20spring_state(f).tip_angle_deg for f in exp_forces], dtype=float
            )
            err_1s  = np.abs(prbm_at_exp - exp_tips)
            err_20s = np.abs(s20_at_exp  - exp_tips)

            ax.plot(exp_forces, err_1s, color=ANALYTICAL_COLOR, linewidth=2.2,
                    marker="o", markersize=5, label="|1-Spring − Exp|")
            ax.plot(exp_forces, err_20s, color=TWENTY_SPRING_COLOR, linewidth=2.2,
                    linestyle="--", marker="s", markersize=5, label="|20-Spring − Exp|")

            cur_1s  = abs(analytical_display_state(cable_force_n).tip_angle_deg
                         - float(np.interp(cable_force_n, exp_forces, exp_tips)))
            cur_20s = abs(solve_20spring_state(cable_force_n).tip_angle_deg
                         - float(np.interp(cable_force_n, exp_forces, exp_tips)))
            ax.scatter([cable_force_n], [cur_1s],  color=ANALYTICAL_COLOR,    s=52, zorder=4)
            ax.scatter([cable_force_n], [cur_20s], color=TWENTY_SPRING_COLOR, s=52, zorder=4)

            fit_1s, fit_20s = experimental_fit_stats(self.exp_series)
            better = fit_1s if fit_1s.rmse_deg <= fit_20s.rmse_deg else fit_20s
            ax.annotate(
                f"Closer overall: {better.label}\n"
                f"RMSE {fit_1s.rmse_deg:.2f} deg vs {fit_20s.rmse_deg:.2f} deg",
                xy=(0.02, 0.96), xycoords="axes fraction", va="top", fontsize=8,
                bbox=dict(boxstyle="round,pad=0.3", facecolor=PANEL, edgecolor=EDGE, alpha=0.92),
            )
            title = "Absolute Error vs Experimental"

        else:
            # No FEA, no experimental: show continuous |1-spring − 20-spring|
            dense = np.linspace(fc_min, fc_max, 300)
            a1s  = np.array([analytical_display_state(f).tip_angle_deg for f in dense], dtype=float)
            a20s = np.array([solve_20spring_state(f).tip_angle_deg     for f in dense], dtype=float)
            diff = np.abs(a1s - a20s)
            ax.plot(dense, diff, color=DISCREPANCY_COLOR, linewidth=2.0,
                    label="|1-Spring − 20-Spring|")
            ax.fill_between(dense, 0.0, diff, color="#f5c8c4", alpha=0.35)
            cur_diff = abs(analytical_display_state(cable_force_n).tip_angle_deg
                          - solve_20spring_state(cable_force_n).tip_angle_deg)
            ax.scatter([cable_force_n], [cur_diff], color=DISCREPANCY_COLOR, s=52, zorder=3)
            title = "1-Spring vs 20-Spring Difference"

        ax.axhline(0, color="grey", linewidth=0.8, linestyle=":")
        ax.set_title(title, pad=10, fontsize=11)
        ax.set_xlabel("Cable force [N]")
        ax.set_ylabel("|Error| [deg]")
        ax.grid(True, alpha=0.22)
        ax.tick_params(axis="both", labelsize=8.2)
        ax.margins(x=0.04)
        ax.legend(loc="upper right", fontsize=6.7, framealpha=0.94,
                  borderpad=0.35, labelspacing=0.30, handlelength=2.0)

    # ── Table ─────────────────────────────────────────────────────────────────

    def _update_table(self, analytical: DisplayState, twenty_spring: DisplayState) -> None:
        for iid in self.table.get_children():
            self.table.delete(iid)

        ep = self._closest_exp_point(analytical.cable_force_n) if self.show_exp_var.get() else None

        def exp_tip_str() -> str:
            return f"{ep.tip_angle_deg:.2f} deg (Δ={ep.target_delta_deg:.0f}°)" if ep else "—"

        def err_str(model_val: float) -> str:
            return f"{model_val - ep.tip_angle_deg:.2f} deg" if ep else "—"

        self.table.insert("", "end", values=(
            "Tip angle",
            f"{analytical.tip_angle_deg:.2f} deg",
            f"{twenty_spring.tip_angle_deg:.2f} deg",
            exp_tip_str(),
            err_str(analytical.tip_angle_deg),
            err_str(twenty_spring.tip_angle_deg),
        ))

        for j, (a_bend, s20_bend) in enumerate(
            zip(analytical.joint_bends_deg, twenty_spring.joint_bends_deg), start=1
        ):
            exp_bend_str = "—"
            err_1s_str   = "—"
            err_20s_str  = "—"
            if ep is not None and j - 1 < len(ep.joint_deltas_deg):
                ed = ep.joint_deltas_deg[j - 1]
                exp_bend_str = f"{ed:.2f} deg"
                err_1s_str   = f"{a_bend   - ed:.2f} deg"
                err_20s_str  = f"{s20_bend - ed:.2f} deg"
            self.table.insert("", "end", values=(
                f"Joint {j}",
                f"{a_bend:.2f} deg",
                f"{s20_bend:.2f} deg",
                exp_bend_str,
                err_1s_str,
                err_20s_str,
            ))

    # ── Per-joint error plot ──────────────────────────────────────────────────

    def _update_joint_error_plot(self, cable_force_n: float) -> None:
        ax = self.ax_joint_err
        ax.clear()
        ax.set_facecolor(PANEL)

        a_state   = analytical_display_state(cable_force_n)
        s20_state = solve_20spring_state(cable_force_n)

        has_fea = self.fea_surrogate is not None
        has_exp = (self.show_exp_var.get()
                   and self.exp_series is not None
                   and len(self.exp_series.points) > 0)

        x = np.arange(N_JOINTS)
        bar_w = 0.24
        xlabels = [
            f"J{i}\n({t:.2f} mm)"
            for i, t in enumerate(JOINT_THICKNESSES_MM, start=1)
        ]

        if has_fea:
            fea_state = self.fea_surrogate.interpolate_state(cable_force_n)
            err_1s  = [abs(a - f) for a, f in zip(a_state.joint_bends_deg,   fea_state.joint_bends_deg)]
            err_20s = [abs(s - f) for s, f in zip(s20_state.joint_bends_deg, fea_state.joint_bends_deg)]
            ax.bar(x - bar_w / 2, err_1s,  bar_w, label="|1-Spring − FEA|",
                   color=ANALYTICAL_COLOR,    alpha=0.85)
            ax.bar(x + bar_w / 2, err_20s, bar_w, label="|20-Spring − FEA|",
                   color=TWENTY_SPRING_COLOR, alpha=0.85)
            title = f"Per-Joint |Error vs FEA|  (Fc={cable_force_n:.2f} N)"

        elif has_exp:
            ep = self._closest_exp_point(cable_force_n)
            if ep is not None and len(ep.joint_deltas_deg) >= N_JOINTS:
                err_1s  = [abs(a - e) for a, e in zip(a_state.joint_bends_deg,   ep.joint_deltas_deg)]
                err_20s = [abs(s - e) for s, e in zip(s20_state.joint_bends_deg, ep.joint_deltas_deg)]
                ax.bar(x - bar_w / 2, err_1s,  bar_w, label="|1-Spring − Exp|",
                       color=ANALYTICAL_COLOR,    alpha=0.85)
                ax.bar(x + bar_w / 2, err_20s, bar_w, label="|20-Spring − Exp|",
                       color=TWENTY_SPRING_COLOR, alpha=0.85)
            title = f"Per-Joint |Error vs Exp|  (Fc≈{cable_force_n:.2f} N)"

        else:
            diff = [abs(a - s) for a, s in zip(a_state.joint_bends_deg, s20_state.joint_bends_deg)]
            ax.bar(x, diff, bar_w * 1.5, label="|1-Spring − 20-Spring|",
                   color=DISCREPANCY_COLOR, alpha=0.85)
            title = f"Per-Joint |1-Spring − 20-Spring|  (Fc={cable_force_n:.2f} N)"

        ax.set_xticks(x)
        ax.set_xticklabels(xlabels, fontsize=7.5)
        ax.set_xlabel("Joint (flexure thickness)")
        ax.set_ylabel("|Error| [deg]")
        ax.set_title(title, fontsize=10, pad=10)
        ax.set_ylim(bottom=0)
        ax.tick_params(axis="y", labelsize=8.2)
        ax.legend(fontsize=6.7, framealpha=0.94, borderpad=0.35,
                  labelspacing=0.30, handlelength=2.0)
        ax.grid(True, alpha=0.22, axis="y")

    # ── Main update ───────────────────────────────────────────────────────────

    def _update_visuals(self) -> None:
        fc           = self._current_force()
        analytical   = analytical_display_state(fc)
        twenty_spring = solve_20spring_state(fc)
        fea          = (self.fea_surrogate.interpolate_state(fc)
                        if self.fea_surrogate is not None and self.show_fea_var.get() else None)

        tip_diff = twenty_spring.tip_angle_deg - analytical.tip_angle_deg
        summary_lines = [
            f"Current cable force: {fc:.3f} N",
            f"Tip angle:  1-Spring {analytical.tip_angle_deg:.2f}°  |  "
            f"20-Spring {twenty_spring.tip_angle_deg:.2f}°  "
            f"(Δ = {tip_diff:+.2f}°)",
        ]
        if fea is not None:
            m = compute_metrics(analytical, fea)
            summary_lines.append(
                f"FEA {fea.tip_angle_deg:.2f}°  "
                f"(PRBM−FEA {m.tip_error_deg:.2f}°, {m.tip_error_pct:.2f}%)"
            )
        if self.show_exp_var.get() and self.exp_series:
            ep = self._closest_exp_point(fc)
            if ep is not None:
                summary_lines.append(
                    f"Experimental (Δ={ep.target_delta_deg:.0f}°, "
                    f"Fc≈{ep.cable_force_n:.3f} N):  tip {ep.tip_angle_deg:.2f}°  |  "
                    f"1S err {analytical.tip_angle_deg - ep.tip_angle_deg:+.2f}°  "
                    f"20S err {twenty_spring.tip_angle_deg - ep.tip_angle_deg:+.2f}°"
                )
        if self.show_exp_var.get() and self.exp_series and self.exp_series.points:
            fit_1s, fit_20s = experimental_fit_stats(self.exp_series)
            better = fit_1s if fit_1s.rmse_deg <= fit_20s.rmse_deg else fit_20s
            summary_lines.append(
                f"Overall fit to experiment: {better.label} is closer "
                f"(RMSE {fit_1s.rmse_deg:.2f} deg vs {fit_20s.rmse_deg:.2f} deg, "
                f"MAE {fit_1s.mae_deg:.2f} deg vs {fit_20s.mae_deg:.2f} deg)"
            )
        self.summary_var.set("\n".join(summary_lines))

        self._update_shape_plot(analytical, twenty_spring, fea)
        self._update_response_plot(fc)
        self._update_discrepancy_plot(fc)
        self._update_joint_error_plot(fc)
        self._update_table(analytical, twenty_spring)
        self.canvas.draw_idle()


# ─────────────────────────────────────────────────────────────────────────────
#  CLI
# ─────────────────────────────────────────────────────────────────────────────

def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Analytical vs FEA vs Experimental comparison app.")
    p.add_argument("--refresh-fea", action="store_true",
                   help="Regenerate the FEA stage CSV from FreeCAD/CalculiX.")
    p.add_argument("--experimental-csv", type=Path, default=None,
                   help="Path to joint_angles.csv from Video_Analysis.")
    p.add_argument("--summary-force-n", type=float, default=None,
                   help="Print a comparison summary for this force and exit.")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])

    # FEA surrogate is optional; warn but continue without it
    fea_surrogate: Optional[FEASurrogate] = None
    try:
        csv_path = ensure_fea_stage_csv(refresh=args.refresh_fea)
        fea_surrogate = FEASurrogate(csv_path)
    except Exception as exc:
        if args.summary_force_n is not None:
            raise
        print(f"[WARN] FEA data not available ({exc}); continuing without FEA.", file=sys.stderr)

    if args.summary_force_n is not None:
        if fea_surrogate is None:
            print("[ERROR] --summary-force-n requires FEA data.", file=sys.stderr)
            return 1
        print(format_summary(args.summary_force_n, fea_surrogate))
        return 0

    # Use the digitized comparison figure by default so the repo works out of the box.
    exp_series: Optional[ExperimentalSeries] = graph_experimental_series()
    if args.experimental_csv is not None:
        try:
            exp_series = load_experimental_csv(args.experimental_csv)
        except Exception as exc:
            print(f"[WARN] Could not load experimental CSV: {exc}", file=sys.stderr)

    root = tk.Tk()
    app  = FingerDiscrepancyApp(root, fea_surrogate, exp_series)
    root.minsize(1280, 1100)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
