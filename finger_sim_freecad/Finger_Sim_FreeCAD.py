from __future__ import annotations

import argparse
import csv
import math
import re
import subprocess
import sys
import tempfile
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SCRIPT_PATH = Path(__file__).resolve()
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from freecad_runtime import launch_freecad_gui_document, relaunch_with_freecad

try:
    import FreeCAD
    import Fem
    import Mesh
    import ObjectsFem
    import Part
    from femmesh.gmshtools import GmshTools
    from femtools.ccxtools import CcxTools
except ModuleNotFoundError as exc:
    relaunch_with_freecad(
        script_path=SCRIPT_PATH,
        argv=sys.argv[1:],
        missing_module=exc.name or "FreeCAD FEM module",
    )

BASE_BLOCK_LENGTH_MM = 20.0
RIGID_SEGMENT_LENGTH_MM = 25.0
COMPLIANT_SEGMENT_LENGTH_MM = 15.0
SEGMENT_HEIGHT_MM = 10.0
SEGMENT_WIDTH_MM = 12.0
JOINT_THICKNESSES_MM = (3.0, 3.25, 3.25, 3.5)
N_JOINTS = len(JOINT_THICKNESSES_MM)

DEFAULT_CABLE_FORCE_N = 1.0
DEFAULT_MOMENT_ARM_MM = 8.0
DEFAULT_MESH_MAX_MM = 1.8
DEFAULT_MESH_MIN_MM = 0.55
DEFAULT_RESULT_STAGES = 6
DEFAULT_FORCE_MULTIPLIERS = (1.25, 2.5, 3.75, 5.0, 10.0, 20.0)
DEFAULT_INTERMEDIATE_STEPS = 0
DEFAULT_STAGE_OFFSET_MM = 20.0
SEGMENT_SAMPLE_SLAB_MM = 1.5
JOINT_MESH_PAD_MM = 1.5
JOINT_MESH_MIN_SCALE = 0.8
JOINT_MESH_MIN_FLOOR_MM = 0.4

CABLE_CENTER_HEIGHT_MM = 9.0

RIGID_LINK_YOUNGS_MODULUS_MPA = 200_000.0
RIGID_LINK_POISSON_RATIO = 0.35
RIGID_LINK_DENSITY_KG_M3 = 1200.0

TPU_HYPERELASTIC_C10_MPA = -0.233
TPU_HYPERELASTIC_C01_MPA = 2.562
TPU_HYPERELASTIC_C20_MPA = 0.116
TPU_HYPERELASTIC_C11_MPA = -0.561
TPU_HYPERELASTIC_C02_MPA = 0.900
TPU_HYPERELASTIC_D1_INV_MPA = 0.0
TPU_HYPERELASTIC_D2_INV_MPA = 0.0
TPU_DENSITY_KG_M3 = 1200.0

CCX_RIGID_ELSET_NAME = "PatchedRigidLinkMaterialSolid"
CCX_TPU_ELSET_NAME = "PatchedJointTPUMaterialSolid"
CCX_RIGID_MATERIAL_NAME = "RigidLinkMaterial"
CCX_TPU_MATERIAL_NAME = "JointTPUMaterial"

SURFACE_FACE_NODE_MAPS = {
    4: {1: [0, 1, 2], 2: [0, 3, 1], 3: [1, 3, 2], 4: [2, 3, 0]},
    5: {1: [0, 1, 2, 3], 2: [0, 4, 1], 3: [1, 4, 2], 4: [2, 4, 3], 5: [3, 4, 0]},
    6: {1: [0, 1, 2], 2: [3, 5, 4], 3: [0, 3, 4, 1], 4: [1, 4, 5, 2], 5: [0, 2, 5, 3]},
    8: {1: [0, 1, 2, 3], 2: [4, 7, 6, 5], 3: [0, 4, 5, 1], 4: [1, 5, 6, 2], 5: [2, 6, 7, 3], 6: [3, 7, 4, 0]},
    10: {1: [0, 1, 2], 2: [0, 3, 1], 3: [1, 3, 2], 4: [2, 3, 0]},
    13: {1: [0, 1, 2, 3], 2: [0, 4, 1], 3: [1, 4, 2], 4: [2, 4, 3], 5: [3, 4, 0]},
    15: {1: [0, 1, 2], 2: [3, 5, 4], 3: [0, 3, 4, 1], 4: [1, 4, 5, 2], 5: [0, 2, 5, 3]},
    20: {1: [0, 1, 2, 3], 2: [4, 7, 6, 5], 3: [0, 4, 5, 1], 4: [1, 5, 6, 2], 5: [2, 6, 7, 3], 6: [3, 7, 4, 0]},
}


def mm(value: float) -> str:
    return f"{value:.6g} mm"


def mpa(value: float) -> str:
    return f"{value:.6g} MPa"


def kg_m3(value: float) -> str:
    return f"{value:.6g} kg/m^3"


def vector(x: float, y: float, z: float) -> FreeCAD.Vector:
    return FreeCAD.Vector(float(x), float(y), float(z))


def midpoint(bounds: tuple[float, float]) -> float:
    return 0.5 * (bounds[0] + bounds[1])


def fuse_all(shapes: list[Part.Shape]) -> Part.Shape:
    fused = shapes[0]
    for shape in shapes[1:]:
        fused = fused.fuse(shape)
    if hasattr(fused, "removeSplitter"):
        fused = fused.removeSplitter()
    return fused


def add_box(x0: float, length: float, height: float) -> Part.Shape:
    return Part.makeBox(length, SEGMENT_WIDTH_MM, height, vector(x0, 0.0, 0.0))


def find_face_name(shape, *, center, direction=None, area=None, center_tol=0.2, area_tol=0.6) -> str:
    best_index = None
    best_score = float("inf")
    expected_direction = direction.normalize() if direction is not None else None
    for index, face in enumerate(shape.Faces, start=1):
        score = face.CenterOfMass.distanceToPoint(center)
        if score > center_tol:
            continue
        if area is not None and abs(face.Area - area) > area_tol:
            continue
        if expected_direction is not None:
            u0, u1, v0, v1 = face.ParameterRange
            normal = face.normalAt(midpoint((u0, u1)), midpoint((v0, v1))).normalize()
            if abs(normal.dot(expected_direction)) < 0.98:
                continue
        if score < best_score:
            best_index = index
            best_score = score
    if best_index is None:
        raise RuntimeError(
            f"Could not find face near ({center.x:.3f}, {center.y:.3f}, {center.z:.3f})"
        )
    return f"Face{best_index}"


def find_vertex_name(shape, *, center, center_tol=0.2) -> str:
    best_index = None
    best_score = float("inf")
    for index, vertex in enumerate(shape.Vertexes, start=1):
        score = vertex.Point.distanceToPoint(center)
        if score > center_tol:
            continue
        if score < best_score:
            best_index = index
            best_score = score
    if best_index is None:
        raise RuntimeError(
            f"Could not find vertex near ({center.x:.3f}, {center.y:.3f}, {center.z:.3f})"
        )
    return f"Vertex{best_index}"


def build_finger_shape():
    solids = [add_box(0.0, BASE_BLOCK_LENGTH_MM, SEGMENT_HEIGHT_MM)]
    joint_targets = []
    cursor_x = BASE_BLOCK_LENGTH_MM
    for joint_index, joint_thickness_mm in enumerate(JOINT_THICKNESSES_MM, start=1):
        joint_x0 = cursor_x
        joint_x1 = joint_x0 + COMPLIANT_SEGMENT_LENGTH_MM
        rigid_x0 = joint_x1
        rigid_x1 = rigid_x0 + RIGID_SEGMENT_LENGTH_MM
        solids.append(add_box(joint_x0, COMPLIANT_SEGMENT_LENGTH_MM, joint_thickness_mm))
        solids.append(add_box(rigid_x0, RIGID_SEGMENT_LENGTH_MM, SEGMENT_HEIGHT_MM))
        joint_targets.append(
            {
                "joint_index": joint_index,
                "joint_thickness_mm": joint_thickness_mm,
                "joint_x0": joint_x0,
                "joint_x1": joint_x1,
            }
        )
        cursor_x = rigid_x1
    shape = fuse_all(solids)
    if getattr(shape, "ShapeType", "") == "Compound" and len(getattr(shape, "Solids", [])) == 1:
        shape = shape.Solids[0]
    return shape, joint_targets, {
        "total_length_mm": cursor_x,
        "fixed_face_center": vector(0.0, 0.5 * SEGMENT_WIDTH_MM, 0.5 * SEGMENT_HEIGHT_MM),
        "fixed_face_area_mm2": SEGMENT_WIDTH_MM * SEGMENT_HEIGHT_MM,
    }


def create_geometry_document(doc):
    shape, joint_targets, metadata = build_finger_shape()
    finger_obj = doc.addObject("Part::Feature", "FingerSolid")
    finger_obj.Label = "CompliantFinger"
    finger_obj.Shape = shape
    axis_lines = []
    axis_origin_y_mm = SEGMENT_WIDTH_MM + 6.0
    axis_line_length_mm = 12.0
    for axis_index in range(1, N_JOINTS + 2):
        origin = vector(0.0, axis_origin_y_mm + 3.0 * (axis_index - 1), 0.0)
        axis_lines.append(
            Part.makeLine(
                origin,
                vector(
                    axis_line_length_mm,
                    origin.y,
                    origin.z,
                ),
            )
        )
    axis_obj = doc.addObject("Part::Feature", "LoadAxis")
    axis_obj.Shape = Part.Shape(axis_lines)
    if getattr(axis_obj, "ViewObject", None) is not None:
        axis_obj.ViewObject.Visibility = False
    doc.recompute()
    metadata["fixed_face_name"] = find_face_name(
        finger_obj.Shape,
        center=metadata["fixed_face_center"],
        direction=vector(-1.0, 0.0, 0.0),
        area=metadata["fixed_face_area_mm2"],
        center_tol=0.25,
        area_tol=2.0,
    )
    joint_loads = []
    for target in joint_targets:
        joint_thickness_mm = float(target["joint_thickness_mm"])
        joint_x0 = float(target["joint_x0"])
        joint_x1 = float(target["joint_x1"])
        shoulder_z_mm = joint_thickness_mm
        proximal_top_vertices = (
            find_vertex_name(
                finger_obj.Shape,
                center=vector(joint_x0, 0.0, SEGMENT_HEIGHT_MM),
                center_tol=0.25,
            ),
            find_vertex_name(
                finger_obj.Shape,
                center=vector(joint_x0, SEGMENT_WIDTH_MM, SEGMENT_HEIGHT_MM),
                center_tol=0.25,
            ),
        )
        proximal_bottom_vertices = (
            find_vertex_name(
                finger_obj.Shape,
                center=vector(joint_x0, 0.0, shoulder_z_mm),
                center_tol=0.25,
            ),
            find_vertex_name(
                finger_obj.Shape,
                center=vector(joint_x0, SEGMENT_WIDTH_MM, shoulder_z_mm),
                center_tol=0.25,
            ),
        )
        distal_top_vertices = (
            find_vertex_name(
                finger_obj.Shape,
                center=vector(joint_x1, 0.0, SEGMENT_HEIGHT_MM),
                center_tol=0.25,
            ),
            find_vertex_name(
                finger_obj.Shape,
                center=vector(joint_x1, SEGMENT_WIDTH_MM, SEGMENT_HEIGHT_MM),
                center_tol=0.25,
            ),
        )
        distal_bottom_vertices = (
            find_vertex_name(
                finger_obj.Shape,
                center=vector(joint_x1, 0.0, shoulder_z_mm),
                center_tol=0.25,
            ),
            find_vertex_name(
                finger_obj.Shape,
                center=vector(joint_x1, SEGMENT_WIDTH_MM, shoulder_z_mm),
                center_tol=0.25,
            ),
        )
        joint_loads.append(
            {
                "joint_index": int(target["joint_index"]),
                "proximal_top_vertices": proximal_top_vertices,
                "proximal_bottom_vertices": proximal_bottom_vertices,
                "distal_top_vertices": distal_top_vertices,
                "distal_bottom_vertices": distal_bottom_vertices,
                "couple_arm_mm": SEGMENT_HEIGHT_MM - shoulder_z_mm,
                "proximal_shoulder_face_name": find_face_name(
                    finger_obj.Shape,
                    center=vector(
                        joint_x0,
                        0.5 * SEGMENT_WIDTH_MM,
                        shoulder_z_mm + 0.5 * (SEGMENT_HEIGHT_MM - shoulder_z_mm),
                    ),
                    direction=vector(1.0, 0.0, 0.0),
                    area=(SEGMENT_HEIGHT_MM - shoulder_z_mm) * SEGMENT_WIDTH_MM,
                    center_tol=0.25,
                    area_tol=2.0,
                ),
                "distal_shoulder_face_name": find_face_name(
                    finger_obj.Shape,
                    center=vector(
                        joint_x1,
                        0.5 * SEGMENT_WIDTH_MM,
                        shoulder_z_mm + 0.5 * (SEGMENT_HEIGHT_MM - shoulder_z_mm),
                    ),
                    direction=vector(-1.0, 0.0, 0.0),
                    area=(SEGMENT_HEIGHT_MM - shoulder_z_mm) * SEGMENT_WIDTH_MM,
                    center_tol=0.25,
                    area_tol=2.0,
                ),
                "proximal_axis_edge_name": f"Edge{int(target['joint_index'])}",
                "distal_axis_edge_name": f"Edge{int(target['joint_index']) + 1}",
            }
        )
    return finger_obj, axis_obj, joint_loads, metadata


def scaled_joint_moment_arms_mm(nominal_joint1_arm_mm: float) -> tuple[float, ...]:
    base_moment_arms = tuple(
        CABLE_CENTER_HEIGHT_MM - 0.5 * joint_thickness_mm
        for joint_thickness_mm in JOINT_THICKNESSES_MM
    )
    if base_moment_arms[0] <= 0.0:
        raise RuntimeError(
            "Joint 1 has a non-positive tendon moment arm; check the cable center height and joint thickness."
        )
    arm_scale = nominal_joint1_arm_mm / base_moment_arms[0]
    return tuple(moment_arm_mm * arm_scale for moment_arm_mm in base_moment_arms)


def joint_moments_from_cable_force(cable_force_n: float, *, moment_arm_mm: float) -> tuple[float, ...]:
    return tuple(
        cable_force_n * local_moment_arm_mm
        for local_moment_arm_mm in scaled_joint_moment_arms_mm(moment_arm_mm)
    )


def add_closing_joint_moments(doc, analysis, geom_obj, axis_obj, joint_loads, joint_moments_nmm):
    # Apply each joint moment as a top/bottom force couple along the global tendon
    # pull direction. The FEA load definition stays purely geometric rather than
    # pre-rotating the force axes from a separate reduced-order kinematic model.
    for joint_load, joint_moment_nmm in zip(joint_loads, joint_moments_nmm):
        joint_index = int(joint_load["joint_index"])
        couple_arm_mm = float(joint_load["couple_arm_mm"])
        if couple_arm_mm <= 0.0:
            raise RuntimeError("Joint moment couple arm must be positive.")
        equivalent_force_n = abs(joint_moment_nmm) / couple_arm_mm
        closing = joint_moment_nmm >= 0.0
        load_specs = (
            ("ProximalTop", joint_load["proximal_top_vertices"], not closing, joint_load["proximal_axis_edge_name"]),
            ("ProximalBottom", joint_load["proximal_bottom_vertices"], closing, joint_load["proximal_axis_edge_name"]),
            ("DistalTop", joint_load["distal_top_vertices"], closing, joint_load["distal_axis_edge_name"]),
            ("DistalBottom", joint_load["distal_bottom_vertices"], not closing, joint_load["distal_axis_edge_name"]),
        )
        for suffix, references, reversed_flag, edge_name in load_specs:
            force_obj = ObjectsFem.makeConstraintForce(doc, f"MomentJoint{joint_index}{suffix}")
            force_obj.References = [(geom_obj, references)]
            force_obj.Force = f"{equivalent_force_n:.6g} N"
            force_obj.Direction = (axis_obj, [edge_name])
            force_obj.Reversed = reversed_flag
            analysis.addObject(force_obj)


def add_analysis(doc, geom_obj, axis_obj, joint_loads, metadata, *, joint_moments_nmm, mesh_max_mm, mesh_min_mm, nonlinear, result_stages):
    analysis = ObjectsFem.makeAnalysis(doc, "Analysis")
    solver_obj = ObjectsFem.makeSolverCalculiXCcxTools(doc, "CalculiXCcxTools")
    result_stage_count = max(1, result_stages)
    increment_size = 1.0 / result_stage_count
    if nonlinear:
        increment_size = min(increment_size, 0.05)
    solver_obj.WorkingDir = ""
    solver_obj.SplitInputWriter = False
    solver_obj.AnalysisType = "static"
    solver_obj.GeometricalNonlinearity = "nonlinear" if nonlinear else "linear"
    solver_obj.ThermoMechSteadyState = False
    solver_obj.MatrixSolverType = "default"
    solver_obj.IterationsControlParameterTimeUse = False
    solver_obj.AutomaticIncrementation = True
    solver_obj.IncrementsMaximum = max(1200 if nonlinear else 100, result_stage_count * (220 if nonlinear else 25))
    solver_obj.TimeInitialIncrement = increment_size
    solver_obj.TimePeriod = 1.0
    solver_obj.TimeMinimumIncrement = min(1e-5, increment_size)
    solver_obj.TimeMaximumIncrement = increment_size
    solver_obj.OutputFrequency = 1 if nonlinear else max(1, math.ceil(solver_obj.IncrementsMaximum / result_stage_count))
    analysis.addObject(solver_obj)
    material_obj = ObjectsFem.makeMaterialSolid(doc, "RigidLinkMaterial")
    material = material_obj.Material
    material["Name"] = CCX_RIGID_MATERIAL_NAME
    material["YoungsModulus"] = mpa(RIGID_LINK_YOUNGS_MODULUS_MPA)
    material["PoissonRatio"] = f"{RIGID_LINK_POISSON_RATIO:.6g}"
    material["Density"] = kg_m3(RIGID_LINK_DENSITY_KG_M3)
    material_obj.Material = material
    analysis.addObject(material_obj)
    fixed_constraint = ObjectsFem.makeConstraintFixed(doc, "FixedBase")
    fixed_constraint.References = [(geom_obj, metadata["fixed_face_name"])]
    analysis.addObject(fixed_constraint)
    add_closing_joint_moments(doc, analysis, geom_obj, axis_obj, joint_loads, joint_moments_nmm)
    mesh_obj = analysis.addObject(ObjectsFem.makeMeshGmsh(doc, "FEMMeshGmsh"))[0]
    mesh_obj.Shape = geom_obj
    if hasattr(mesh_obj, "CharacteristicLengthMax"):
        mesh_obj.CharacteristicLengthMax = mm(mesh_max_mm)
    if hasattr(mesh_obj, "CharacteristicLengthMin"):
        mesh_obj.CharacteristicLengthMin = mm(mesh_min_mm)
    if hasattr(mesh_obj, "ElementDimension"):
        mesh_obj.ElementDimension = "3D"
    if hasattr(mesh_obj, "SecondOrderLinear"):
        mesh_obj.SecondOrderLinear = False
    if hasattr(mesh_obj, "ElementOrder"):
        mesh_obj.ElementOrder = "2nd"
    doc.recompute()
    return analysis, solver_obj

def generate_mesh(mesh_obj, *, working_dir: Path):
    gmsh = GmshTools(mesh_obj)
    gmsh.load_properties()
    gmsh.update_mesh_data()
    gmsh.get_tmp_file_paths(param_working_dir=str(working_dir), create=True)
    gmsh.get_gmsh_command()
    gmsh.write_gmsh_input_files()
    add_joint_mesh_refinement_to_geo(Path(gmsh.temp_file_geo), mesh_obj)

    log_level = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/Fem/Gmsh").GetString(
        "LogVerbosity", "3"
    )
    completed = subprocess.run(
        [gmsh.gmsh_bin, "-v", log_level, "-", gmsh.temp_file_geo],
        cwd=str(working_dir),
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode != 0 or not Path(gmsh.temp_file_mesh).is_file():
        message = (completed.stderr or completed.stdout).strip()
        raise RuntimeError(f"Gmsh meshing failed: {message or completed.returncode}")

    mesh_obj.FemMesh = Fem.read(gmsh.temp_file_mesh)
    gmsh.rename_groups()


def append_geo_line(lines: list[str], line: str):
    lines.append(f"{line}\n")


def add_joint_mesh_refinement_to_geo(geo_path: Path, mesh_obj):
    if not geo_path.is_file():
        raise RuntimeError(f"Gmsh geometry file was not created: {geo_path}")

    mesh_max_mm = DEFAULT_MESH_MAX_MM
    if hasattr(mesh_obj, "CharacteristicLengthMax"):
        try:
            mesh_max_mm = float(FreeCAD.Units.Quantity(mesh_obj.CharacteristicLengthMax).Value)
        except Exception:
            mesh_max_mm = DEFAULT_MESH_MAX_MM

    mesh_min_mm = DEFAULT_MESH_MIN_MM
    if hasattr(mesh_obj, "CharacteristicLengthMin"):
        try:
            mesh_min_mm = float(FreeCAD.Units.Quantity(mesh_obj.CharacteristicLengthMin).Value)
        except Exception:
            mesh_min_mm = DEFAULT_MESH_MIN_MM

    joint_mesh_min_mm = min(mesh_min_mm, max(JOINT_MESH_MIN_FLOOR_MM, mesh_min_mm * JOINT_MESH_MIN_SCALE))
    y_min_mm = -JOINT_MESH_PAD_MM
    y_max_mm = SEGMENT_WIDTH_MM + JOINT_MESH_PAD_MM
    z_min_mm = -JOINT_MESH_PAD_MM
    z_max_mm = SEGMENT_HEIGHT_MM + JOINT_MESH_PAD_MM

    lines = [
        "\n// Local joint mesh refinement added by Finger_Sim_FreeCAD.py\n",
        f"Mesh.CharacteristicLengthMin = {mesh_min_mm:.6g};\n",
        f"Mesh.CharacteristicLengthMax = {mesh_max_mm:.6g};\n",
    ]
    joint_field_ids = []
    for field_id, (x0_mm, x1_mm) in enumerate(joint_x_ranges_mm(), start=1):
        joint_field_ids.append(str(field_id))
        append_geo_line(lines, f"Field[{field_id}] = Box;")
        append_geo_line(lines, f"Field[{field_id}].VIn = {joint_mesh_min_mm:.6g};")
        append_geo_line(lines, f"Field[{field_id}].VOut = {mesh_max_mm:.6g};")
        append_geo_line(lines, f"Field[{field_id}].XMin = {x0_mm - JOINT_MESH_PAD_MM:.6g};")
        append_geo_line(lines, f"Field[{field_id}].XMax = {x1_mm + JOINT_MESH_PAD_MM:.6g};")
        append_geo_line(lines, f"Field[{field_id}].YMin = {y_min_mm:.6g};")
        append_geo_line(lines, f"Field[{field_id}].YMax = {y_max_mm:.6g};")
        append_geo_line(lines, f"Field[{field_id}].ZMin = {z_min_mm:.6g};")
        append_geo_line(lines, f"Field[{field_id}].ZMax = {z_max_mm:.6g};")

    min_field_id = len(joint_field_ids) + 1
    append_geo_line(lines, f"Field[{min_field_id}] = Min;")
    append_geo_line(lines, f"Field[{min_field_id}].FieldsList = {{{', '.join(joint_field_ids)}}};")
    append_geo_line(lines, f"Background Field = {min_field_id};")

    with geo_path.open("a", encoding="utf-8") as handle:
        handle.writelines(lines)


def joint_x_ranges_mm() -> list[tuple[float, float]]:
    ranges = []
    cursor_x = BASE_BLOCK_LENGTH_MM
    for _ in JOINT_THICKNESSES_MM:
        x0 = cursor_x
        x1 = x0 + COMPLIANT_SEGMENT_LENGTH_MM
        ranges.append((x0, x1))
        cursor_x = x1 + RIGID_SEGMENT_LENGTH_MM
    return ranges


def element_centroid_x_mm(fem_mesh, element_id: int) -> float:
    node_ids = fem_mesh.getElementNodes(element_id)
    if not node_ids:
        raise RuntimeError(f"Element {element_id} has no nodes.")
    return sum(fem_mesh.getNodeById(node_id).x for node_id in node_ids) / float(len(node_ids))


def classify_volume_elements(fem_mesh):
    if fem_mesh.VolumeCount <= 0:
        raise RuntimeError("The finger mesh does not contain 3D volume elements.")
    rigid_elements = []
    joint_elements = []
    joint_ranges = joint_x_ranges_mm()
    for element_id in fem_mesh.Volumes:
        centroid_x = element_centroid_x_mm(fem_mesh, element_id)
        if any(x0 <= centroid_x <= x1 for x0, x1 in joint_ranges):
            joint_elements.append(int(element_id))
        else:
            rigid_elements.append(int(element_id))
    if not rigid_elements or not joint_elements:
        raise RuntimeError("Failed to split the mesh into rigid and compliant regions.")
    return rigid_elements, joint_elements


def format_ccx_elset(name: str, element_ids: list[int], *, line_width: int = 16) -> str:
    lines = [f"*ELSET,ELSET={name}"]
    for start in range(0, len(element_ids), line_width):
        lines.append(",".join(str(element_id) for element_id in element_ids[start:start + line_width]))
    return "\n".join(lines) + "\n"


def replace_between_markers(text: str, *, start_marker: str, end_marker: str, replacement: str) -> str:
    start = text.find(start_marker)
    if start < 0:
        raise RuntimeError(f"Could not find marker in CalculiX input: {start_marker}")
    end = text.find(end_marker, start)
    if end < 0:
        raise RuntimeError(f"Could not find marker in CalculiX input: {end_marker}")
    return text[:start] + replacement + text[end:]


def patch_calculix_input_materials(inp_path: Path, fem_mesh) -> dict[str, int]:
    rigid_elements, joint_elements = classify_volume_elements(fem_mesh)
    inp_text = inp_path.read_text(encoding="utf-8")
    elset_block = format_ccx_elset(CCX_RIGID_ELSET_NAME, rigid_elements) + format_ccx_elset(CCX_TPU_ELSET_NAME, joint_elements) + "\n"
    insert_marker = "\n***********************************************************\n** constraints fixed node sets\n"
    insert_at = inp_text.find(insert_marker)
    if insert_at < 0:
        raise RuntimeError("Could not find the constraint set section in the CalculiX input.")
    inp_text = inp_text[:insert_at] + elset_block + inp_text[insert_at:]

    material_block = f"""***********************************************************
** Materials
*MATERIAL, NAME={CCX_RIGID_MATERIAL_NAME}
*ELASTIC
{RIGID_LINK_YOUNGS_MODULUS_MPA:.13G},{RIGID_LINK_POISSON_RATIO:.13G}
*DENSITY
{FreeCAD.Units.Quantity(kg_m3(RIGID_LINK_DENSITY_KG_M3)).getValueAs('t/mm^3').Value:.13G}
*MATERIAL, NAME={CCX_TPU_MATERIAL_NAME}
*HYPERELASTIC,POLYNOMIAL,N=2
{TPU_HYPERELASTIC_C10_MPA:.13G},{TPU_HYPERELASTIC_C01_MPA:.13G},{TPU_HYPERELASTIC_D1_INV_MPA:.13G}
{TPU_HYPERELASTIC_C20_MPA:.13G},{TPU_HYPERELASTIC_C11_MPA:.13G},{TPU_HYPERELASTIC_C02_MPA:.13G},{TPU_HYPERELASTIC_D2_INV_MPA:.13G}
*DENSITY
{FreeCAD.Units.Quantity(kg_m3(TPU_DENSITY_KG_M3)).getValueAs('t/mm^3').Value:.13G}

"""
    inp_text = replace_between_markers(
        inp_text,
        start_marker="***********************************************************\n** Materials\n",
        end_marker="***********************************************************\n** Sections\n",
        replacement=material_block,
    )

    section_block = f"""***********************************************************
** Sections
*SOLID SECTION, ELSET={CCX_RIGID_ELSET_NAME}, MATERIAL={CCX_RIGID_MATERIAL_NAME}
*SOLID SECTION, ELSET={CCX_TPU_ELSET_NAME}, MATERIAL={CCX_TPU_MATERIAL_NAME}

"""
    inp_text = replace_between_markers(
        inp_text,
        start_marker="***********************************************************\n** Sections\n",
        end_marker="***********************************************************\n** At least one step is needed to run an CalculiX analysis of FreeCAD\n",
        replacement=section_block,
    )
    inp_path.write_text(inp_text, encoding="utf-8")
    return {"rigid_element_count": len(rigid_elements), "joint_element_count": len(joint_elements)}


def solve_with_patched_input(doc, solver_obj, *, working_dir: Path):
    fea = CcxTools(solver=solver_obj)
    fea.reset_mesh_purge_results_checked()
    fea.update_objects()
    fea.setup_working_dir(str(working_dir))
    fea.setup_ccx()
    message = fea.check_prerequisites()
    if message:
        raise RuntimeError(f"CalculiX prerequisites failed:\n{message}")
    fea.write_inp_file()
    inp_path = Path(fea.inp_file_name)
    if not inp_path.is_file():
        raise RuntimeError("CalculiX input file was not written.")
    mesh_obj = doc.getObject("FEMMeshGmsh")
    if mesh_obj is None or mesh_obj.FemMesh is None:
        raise RuntimeError("FEMMeshGmsh is missing its mesh data.")
    patch_stats = patch_calculix_input_materials(inp_path, mesh_obj.FemMesh)
    for suffix in (".dat", ".cvg", ".sta", ".frd", ".12d"):
        stale_output = inp_path.with_suffix(suffix)
        if stale_output.exists():
            stale_output.unlink()
    ret_code = fea.ccx_run()
    if ret_code != 0:
        raise RuntimeError(f"CalculiX finished with error code {ret_code}.")
    try:
        fea.load_results()
    except Exception as exc:
        existing_results = [obj for obj in doc.Objects if obj.isDerivedFrom("Fem::FemResultObject")]
        if not existing_results:
            raise
        patch_stats["result_import_warning"] = str(exc)
    return patch_stats

def segment_x_ranges_mm() -> list[tuple[float, float]]:
    ranges = []
    cursor_x = BASE_BLOCK_LENGTH_MM + COMPLIANT_SEGMENT_LENGTH_MM
    for _ in JOINT_THICKNESSES_MM:
        x0 = cursor_x
        x1 = x0 + RIGID_SEGMENT_LENGTH_MM
        ranges.append((x0, x1))
        cursor_x = x1 + COMPLIANT_SEGMENT_LENGTH_MM
    return ranges


def initial_segment_kinematics() -> dict[str, float]:
    values = {}
    for segment_index, (x0, x1) in enumerate(segment_x_ranges_mm(), start=1):
        values[f"seg{segment_index}_angle_deg"] = 0.0
        values[f"seg{segment_index}_center_x_mm"] = 0.5 * (x0 + x1)
        values[f"seg{segment_index}_center_y_mm"] = 0.0
        values[f"seg{segment_index}_center_z_mm"] = 0.5 * SEGMENT_HEIGHT_MM
        values[f"joint{segment_index}_bend_deg"] = 0.0
    values["tip_angle_deg"] = values[f"seg{N_JOINTS}_angle_deg"]
    return values


def parse_result_progress(result_name: str) -> float:
    match = re.match(r"^CCX_Time_(.+?)_Results(?:\d+)?$", result_name)
    return float(match.group(1).replace("_", ".")) if match is not None else 1.0


def sorted_result_objects(doc) -> list:
    return sorted(
        [obj for obj in doc.Objects if obj.isDerivedFrom("Fem::FemResultObject")],
        key=lambda obj: (parse_result_progress(obj.Name), obj.Name),
    )


def unique_result_objects(result_objects, *, progress_digits: int = 6) -> list:
    unique_by_progress = {}
    for result_obj in result_objects:
        unique_by_progress[round(parse_result_progress(result_obj.Name), progress_digits)] = result_obj
    return sorted(
        unique_by_progress.values(),
        key=lambda obj: (parse_result_progress(obj.Name), obj.Name),
    )


def remove_object_tree(doc, obj_name: str) -> None:
    obj = doc.getObject(obj_name)
    if obj is None:
        return
    members = list(getattr(obj, "Group", []))
    doc.removeObject(obj.Name)
    for member in members:
        if doc.getObject(member.Name) is not None:
            doc.removeObject(member.Name)


def surface_triangles_for_result(fem_mesh, result_obj, *, displacement_scale: float):
    face_code_dict = {}
    face_code_list = []
    shift_bits = 20
    element_ids = list(fem_mesh.Volumes) if fem_mesh.VolumeCount > 0 else list(fem_mesh.Faces)
    for element_id in element_ids:
        element_nodes = fem_mesh.getElementNodes(element_id)
        face_def = SURFACE_FACE_NODE_MAPS.get(len(element_nodes))
        if face_def is None:
            continue
        for node_indices in face_def.values():
            node_list = [element_nodes[node_index] for node_index in node_indices]
            code_list = sorted(node_list)
            face_code = 0
            shifter = 0
            for node_id in code_list:
                face_code += node_id << shifter
                shifter += shift_bits
            face_code_dict[face_code] = node_list
            face_code_list.append(face_code)
    face_code_list.sort()
    single_faces = []
    face_index = 0
    while face_index < len(face_code_list):
        if face_index < len(face_code_list) - 1 and face_code_list[face_index] == face_code_list[face_index + 1]:
            face_index += 2
        else:
            single_faces.append(face_code_list[face_index])
            face_index += 1

    displacements = {}
    if result_obj is not None:
        displacements = {int(node_id): result_obj.DisplacementVectors[index] for index, node_id in enumerate(result_obj.NodeNumbers)}

    def point_for_node(node_id: int):
        point = fem_mesh.getNodeById(node_id)
        displacement = displacements.get(node_id)
        return point if displacement is None else point + displacement * displacement_scale

    triangles = []
    for face_code in single_faces:
        face_nodes = face_code_dict[face_code]
        triangles.extend([point_for_node(face_nodes[0]), point_for_node(face_nodes[1]), point_for_node(face_nodes[2])])
        if len(face_nodes) == 4:
            triangles.extend([point_for_node(face_nodes[2]), point_for_node(face_nodes[3]), point_for_node(face_nodes[0])])
    return triangles


def add_stage_mesh_object(doc, group, *, name: str, label: str, fem_mesh, result_obj, displacement_scale: float, offset_y_mm: float):
    stage_mesh_obj = doc.addObject("Mesh::Feature", name)
    stage_mesh_obj.Label = label
    stage_mesh_obj.Mesh = Mesh.Mesh(surface_triangles_for_result(fem_mesh, result_obj, displacement_scale=displacement_scale))
    stage_mesh_obj.Placement.Base = vector(0.0, offset_y_mm, 0.0)
    group.addObject(stage_mesh_obj)


def select_stage_results(result_objects, *, peak_cable_force_n: float, target_cable_forces_n: list[float]):
    if not result_objects or peak_cable_force_n <= 0.0:
        return []
    selected = []
    used = set()
    for target_cable_force_n in target_cable_forces_n:
        if target_cable_force_n <= 0.0:
            continue
        target_load_factor = min(1.0, max(0.0, target_cable_force_n / peak_cable_force_n))
        ranked_candidates = sorted(
            result_objects,
            key=lambda obj: (
                abs(parse_result_progress(obj.Name) - target_load_factor),
                parse_result_progress(obj.Name),
                obj.Name,
            ),
        )
        best = next((candidate for candidate in ranked_candidates if candidate.Name not in used), None)
        if best is None:
            continue
        actual_cable_force_n = peak_cable_force_n * parse_result_progress(best.Name)
        selected.append(
            {
                "result_obj": best,
                "target_cable_force_n": target_cable_force_n,
                "actual_cable_force_n": actual_cable_force_n,
            }
        )
        used.add(best.Name)
    return selected


def retain_stage_results(doc, *, result_objects, peak_cable_force_n: float, target_cable_forces_n: list[float]) -> list:
    result_objects = unique_result_objects(result_objects)
    if not result_objects:
        return []
    selected_stage_results = select_stage_results(
        result_objects,
        peak_cable_force_n=peak_cable_force_n,
        target_cable_forces_n=target_cable_forces_n,
    )
    keep_names = {stage_info["result_obj"].Name for stage_info in selected_stage_results}
    keep_names.add(result_objects[-1].Name)
    for result_obj in result_objects:
        if result_obj.Name not in keep_names:
            remove_object_tree(doc, result_obj.Name)
    doc.recompute()
    return sorted_result_objects(doc)


def create_stage_visualizations(doc, *, result_objects, peak_cable_force_n: float, target_cable_forces_n: list[float], stage_offset_mm: float) -> int:
    mesh_obj = doc.getObject("FEMMeshGmsh")
    if mesh_obj is None or mesh_obj.FemMesh is None:
        raise RuntimeError("FEM mesh is missing; cannot create stage visualizations.")
    result_objects = unique_result_objects(result_objects)
    fem_mesh = mesh_obj.FemMesh
    remove_object_tree(doc, "BendingStageViews")
    group = doc.addObject("App::DocumentObjectGroup", "BendingStageViews")
    group.Label = "BendingStageViews"
    stage_count = 1
    add_stage_mesh_object(
        doc,
        group,
        name="StageMesh01",
        label="Stage 01 | Fc 0.000 N",
        fem_mesh=fem_mesh,
        result_obj=None,
        displacement_scale=1.0,
        offset_y_mm=0.0,
    )
    for stage_count, stage_info in enumerate(
        select_stage_results(
            result_objects,
            peak_cable_force_n=peak_cable_force_n,
            target_cable_forces_n=target_cable_forces_n,
        ),
        start=2,
    ):
        result_obj = stage_info["result_obj"]
        target_cable_force_n = float(stage_info["target_cable_force_n"])
        actual_cable_force_n = float(stage_info["actual_cable_force_n"])
        label = f"Stage {stage_count:02d} | Fc {target_cable_force_n:.3f} N"
        if not math.isclose(actual_cable_force_n, target_cable_force_n, rel_tol=1e-3, abs_tol=1e-3):
            label = f"{label} | solved {actual_cable_force_n:.3f} N"
        add_stage_mesh_object(
            doc,
            group,
            name=f"StageMesh{stage_count:02d}",
            label=label,
            fem_mesh=fem_mesh,
            result_obj=result_obj,
            displacement_scale=1.0,
            offset_y_mm=(stage_count - 1) * stage_offset_mm,
        )
    doc.recompute()
    return stage_count


def build_segment_node_sets(fem_mesh):
    node_sets = []
    for x0, x1 in segment_x_ranges_mm():
        segment_nodes = []
        proximal_nodes = []
        distal_nodes = []
        for node_id in fem_mesh.Nodes:
            point = fem_mesh.getNodeById(node_id)
            if not (0.0 <= point.y <= SEGMENT_WIDTH_MM and 0.0 <= point.z <= SEGMENT_HEIGHT_MM + 1e-6):
                continue
            if x0 <= point.x <= x1:
                segment_nodes.append(node_id)
                if point.x <= x0 + SEGMENT_SAMPLE_SLAB_MM:
                    proximal_nodes.append(node_id)
                if point.x >= x1 - SEGMENT_SAMPLE_SLAB_MM:
                    distal_nodes.append(node_id)
        if not segment_nodes or not proximal_nodes or not distal_nodes:
            raise RuntimeError(f"Could not identify enough mesh nodes for rigid segment x-range {x0:.3f}-{x1:.3f} mm.")
        node_sets.append({"segment_nodes": segment_nodes, "proximal_nodes": proximal_nodes, "distal_nodes": distal_nodes})
    return node_sets


def average_deformed_point(fem_mesh, displacements, node_ids):
    zero = vector(0.0, 0.0, 0.0)
    sx = sy = sz = 0.0
    for node_id in node_ids:
        point = fem_mesh.getNodeById(node_id)
        displacement = displacements.get(node_id, zero)
        sx += point.x + displacement.x
        sy += point.y + displacement.y
        sz += point.z + displacement.z
    count = float(len(node_ids))
    return sx / count, sy / count, sz / count


def segment_kinematics_for_result(fem_mesh, result_obj, node_sets):
    displacements = {int(node_id): result_obj.DisplacementVectors[index] for index, node_id in enumerate(result_obj.NodeNumbers)}
    values = {}
    previous_angle_deg = 0.0
    for segment_index, node_set in enumerate(node_sets, start=1):
        center = average_deformed_point(fem_mesh, displacements, node_set["segment_nodes"])
        proximal = average_deformed_point(fem_mesh, displacements, node_set["proximal_nodes"])
        distal = average_deformed_point(fem_mesh, displacements, node_set["distal_nodes"])
        angle_deg = math.degrees(math.atan2(proximal[2] - distal[2], distal[0] - proximal[0]))
        values[f"seg{segment_index}_angle_deg"] = angle_deg
        values[f"seg{segment_index}_center_x_mm"] = center[0]
        values[f"seg{segment_index}_center_y_mm"] = center[1] - 0.5 * SEGMENT_WIDTH_MM
        values[f"seg{segment_index}_center_z_mm"] = center[2]
        values[f"joint{segment_index}_bend_deg"] = angle_deg - previous_angle_deg
        previous_angle_deg = angle_deg
    values["tip_angle_deg"] = values[f"seg{N_JOINTS}_angle_deg"]
    return values


def flatten_segment_row(*, source: str, load_factor: float, peak_cable_force_n: float, moment_arm_mm: float, values):
    cable_force_n = peak_cable_force_n * load_factor
    joint_moments_nmm = joint_moments_from_cable_force(cable_force_n, moment_arm_mm=moment_arm_mm)
    local_moment_arms_mm = scaled_joint_moment_arms_mm(moment_arm_mm)
    row = {"source": source, "load_factor": load_factor, "cable_force_n": cable_force_n, "tip_angle_deg": values["tip_angle_deg"]}
    for joint_index, (moment_nmm, moment_arm_mm_local) in enumerate(zip(joint_moments_nmm, local_moment_arms_mm), start=1):
        row[f"joint{joint_index}_moment_nmm"] = moment_nmm
        row[f"joint{joint_index}_moment_arm_mm"] = moment_arm_mm_local
        row[f"joint{joint_index}_bend_deg"] = values[f"joint{joint_index}_bend_deg"]
    for segment_index in range(1, N_JOINTS + 1):
        row[f"seg{segment_index}_angle_deg"] = values[f"seg{segment_index}_angle_deg"]
        row[f"seg{segment_index}_center_x_mm"] = values[f"seg{segment_index}_center_x_mm"]
        row[f"seg{segment_index}_center_y_mm"] = values[f"seg{segment_index}_center_y_mm"]
        row[f"seg{segment_index}_center_z_mm"] = values[f"seg{segment_index}_center_z_mm"]
    return row


def segment_csv_fieldnames() -> list[str]:
    fields = ["source", "load_factor", "cable_force_n", "tip_angle_deg"]
    for joint_index in range(1, N_JOINTS + 1):
        fields.extend([
            f"joint{joint_index}_moment_nmm",
            f"joint{joint_index}_moment_arm_mm",
            f"joint{joint_index}_bend_deg",
        ])
    for segment_index in range(1, N_JOINTS + 1):
        fields.extend([
            f"seg{segment_index}_angle_deg",
            f"seg{segment_index}_center_x_mm",
            f"seg{segment_index}_center_y_mm",
            f"seg{segment_index}_center_z_mm",
        ])
    return fields


def write_segment_progress_csv(doc, *, peak_cable_force_n: float, moment_arm_mm: float, output_path: Path):
    mesh_obj = doc.getObject("FEMMeshGmsh")
    if mesh_obj is None or mesh_obj.FemMesh is None:
        raise RuntimeError("FEM mesh is missing from FEMMeshGmsh.")
    result_objects = unique_result_objects(sorted_result_objects(doc))
    if not result_objects:
        raise RuntimeError("No FemResultObject entries found in document.")
    node_sets = build_segment_node_sets(mesh_obj.FemMesh)
    rows = [
        flatten_segment_row(
            source="fea_initial",
            load_factor=0.0,
            peak_cable_force_n=peak_cable_force_n,
            moment_arm_mm=moment_arm_mm,
            values=initial_segment_kinematics(),
        )
    ]
    for result_obj in result_objects:
        rows.append(
            flatten_segment_row(
                source=result_obj.Name,
                load_factor=parse_result_progress(result_obj.Name),
                peak_cable_force_n=peak_cable_force_n,
                moment_arm_mm=moment_arm_mm,
                values=segment_kinematics_for_result(mesh_obj.FemMesh, result_obj, node_sets),
            )
        )
    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=segment_csv_fieldnames())
        writer.writeheader()
        writer.writerows(rows)
    return rows

def save_document_with_fallback(doc, desired_path: Path, working_dir: Path) -> Path:
    desired_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        doc.saveAs(str(desired_path))
        return desired_path
    except OSError:
        for fallback_path in (Path(tempfile.gettempdir()) / desired_path.name, working_dir / desired_path.name):
            try:
                fallback_path.parent.mkdir(parents=True, exist_ok=True)
                doc.saveAs(str(fallback_path))
                return fallback_path
            except OSError:
                continue
        raise


# ─────────────────────────────────────────────────────────────────────────────
#  Video-Analysis physics — cable tension from motor delta
#  (same constants as Video_Analysis.py and Finger_Discrepancy_App.py)
# ─────────────────────────────────────────────────────────────────────────────

import math as _math
import numpy as _np

_VA_lj        = 15e-3
_VA_h         = 6.5e-3
_VA_bw        = 10e-3
_VA_JH        = _np.array([2.50e-3, 3.25e-3, 3.25e-3, 3.50e-3])
_VA_E_eff     = 2.5e6
_VA_RP        = 0.015
_VA_GAIN      = 1.00
_VA_C_TENDON  = 0.005
_VA_PRETENSION = 16.0

_VA_Ij       = (1.0 / 12.0) * _VA_bw * _VA_JH ** 3
_VA_LK       = _VA_E_eff * _VA_Ij / _VA_lj
_VA_SUM_FLEX = float(_np.sum(1.0 / _VA_LK))
_VA_DENOM    = _VA_h ** 2 * _VA_SUM_FLEX + _VA_C_TENDON


def cable_force_from_motor_delta(delta_deg: float) -> float:
    """Return the cable tension increment (N) above pretension for motor delta (deg)."""
    dphi_pre = _math.radians(_VA_PRETENSION) * _VA_GAIN
    dphi_tot = dphi_pre + _math.radians(delta_deg) * _VA_GAIN
    return max(0.0, _VA_RP * (dphi_tot - dphi_pre) / _VA_DENOM)


def parse_motor_deltas(value: str) -> tuple[float, ...]:
    """Parse a comma-separated list of motor delta values in degrees."""
    try:
        deltas = sorted({float(t.strip()) for t in value.split(",") if t.strip()})
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"Invalid motor delta list: {value}") from exc
    if not deltas:
        raise argparse.ArgumentTypeError("Motor delta list must not be empty.")
    return tuple(deltas)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Create a FreeCAD FEM model for the 4-joint compliant finger.")
    parser.add_argument("--fcstd", default=str(SCRIPT_DIR / "Finger_Sim_VM2_FEM.FCStd"), help="Output FreeCAD document path.")
    parser.add_argument("--working-dir", default=str(SCRIPT_DIR / "Finger_Sim_VM2_FEM_run"), help="CalculiX working directory.")
    parser.add_argument("--cable-force-n", type=float, default=DEFAULT_CABLE_FORCE_N, help="Base cable force used to define the stage-force schedule.")
    parser.add_argument(
        "--force-multipliers",
        type=parse_force_multipliers,
        default=DEFAULT_FORCE_MULTIPLIERS,
        help="Comma-separated multipliers applied to --cable-force-n for the visible stage forces.",
    )
    parser.add_argument(
        "--intermediate-steps",
        type=int,
        default=DEFAULT_INTERMEDIATE_STEPS,
        help="Number of evenly spaced intermediate stage forces inserted between each multiplier anchor.",
    )
    parser.add_argument(
        "--motor-deltas",
        type=parse_motor_deltas,
        default=None,
        metavar="DEG",
        help=(
            "Comma-separated motor position deltas (deg from open, e.g. 15,30,45,60,75,90,105,120). "
            "Each delta is converted to a cable force via Video_Analysis PRBM physics and used as "
            "an additional FEA stage target.  When provided, these forces REPLACE the default "
            "--force-multipliers schedule so the simulation aligns with experimental conditions."
        ),
    )
    parser.add_argument("--moment-arm-mm", type=float, default=DEFAULT_MOMENT_ARM_MM, help="Cable moment arm used in Fc*h = k*theta.")
    parser.add_argument("--mesh-max-mm", type=float, default=DEFAULT_MESH_MAX_MM, help="Maximum tetra element size in millimeters.")
    parser.add_argument("--mesh-min-mm", type=float, default=DEFAULT_MESH_MIN_MM, help="Minimum tetra element size in millimeters.")
    parser.add_argument("--stages", type=int, default=DEFAULT_RESULT_STAGES, help="Minimum number of saved solver load stages.")
    parser.add_argument("--stage-offset-mm", type=float, default=DEFAULT_STAGE_OFFSET_MM, help="Y-offset between visible stage meshes.")
    parser.add_argument("--segment-csv", default=None, help="Optional CSV path for stage-by-stage kinematics.")
    parser.add_argument("--nonlinear", action="store_true", default=True, help="Enable geometrically nonlinear static analysis.")
    parser.add_argument("--linear", dest="nonlinear", action="store_false", help="Force geometrically linear static analysis.")
    parser.add_argument("--skip-solve", action="store_true", help="Create and mesh the model without running CalculiX.")
    parser.add_argument("--no-stage-views", action="store_true", help="Do not create side-by-side stage meshes in the FCStd file.")
    parser.add_argument("--no-show", action="store_true", help="Do not open the generated FCStd in the FreeCAD GUI after the run.")
    return parser.parse_args(argv)


def format_joint_values(values: tuple[float, ...], unit: str) -> str:
    return ", ".join(f"j{i + 1}={value:.3f} {unit}" for i, value in enumerate(values))


def parse_force_multipliers(value: str) -> tuple[float, ...]:
    try:
        multipliers = sorted({float(token.strip()) for token in value.split(",") if token.strip()})
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"Invalid force multiplier list: {value}") from exc
    if not multipliers or multipliers[0] <= 0.0:
        raise argparse.ArgumentTypeError("Force multipliers must be positive comma-separated numbers.")
    return tuple(multipliers)


def build_target_cable_forces(
    base_cable_force_n: float,
    *,
    force_multipliers: tuple[float, ...],
    intermediate_steps: int,
) -> list[float]:
    if intermediate_steps < 0:
        raise ValueError("Intermediate steps must be zero or greater.")
    targets = [0.0]
    anchor_multipliers = (0.0, *force_multipliers)
    for start_multiplier, end_multiplier in zip(anchor_multipliers, anchor_multipliers[1:]):
        step_count = intermediate_steps + 1
        for step_index in range(1, step_count + 1):
            multiplier = start_multiplier + (end_multiplier - start_multiplier) * (step_index / step_count)
            force_n = base_cable_force_n * multiplier
            if not math.isclose(force_n, targets[-1], rel_tol=1e-9, abs_tol=1e-9):
                targets.append(force_n)
    return targets


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    # When --motor-deltas is supplied, convert each delta to a cable force using
    # the Video_Analysis PRBM physics and use those as the stage-force schedule.
    # This aligns the FEA stages with the actual experimental operating points.
    if args.motor_deltas is not None:
        exp_forces = [cable_force_from_motor_delta(d) for d in args.motor_deltas]
        exp_forces = sorted(set(f for f in exp_forces if f > 1e-6))
        if not exp_forces:
            raise ValueError("All computed experimental cable forces are zero; check --motor-deltas values.")
        peak_exp = exp_forces[-1]
        # Re-express as multipliers of BASE (1 N) so build_target_cable_forces can consume them
        base_for_exp = peak_exp  # use peak as the base; multiplier=1 gives the peak
        multipliers_for_exp = tuple(sorted({f / base_for_exp for f in exp_forces}))
        print(f"[motor-deltas] Δ={list(args.motor_deltas)} deg → "
              f"Fc={[f'{f:.4f}' for f in exp_forces]} N (peak={peak_exp:.4f} N)")
        target_cable_forces_n = build_target_cable_forces(
            base_for_exp,
            force_multipliers=multipliers_for_exp,
            intermediate_steps=0,
        )
    else:
        target_cable_forces_n = build_target_cable_forces(
            args.cable_force_n,
            force_multipliers=args.force_multipliers,
            intermediate_steps=args.intermediate_steps,
        )

    solve_peak_cable_force_n = target_cable_forces_n[-1]
    peak_joint_moments_nmm = joint_moments_from_cable_force(solve_peak_cable_force_n, moment_arm_mm=args.moment_arm_mm)
    local_moment_arms_mm = scaled_joint_moment_arms_mm(args.moment_arm_mm)
    solver_stage_count = max(args.stages, len(target_cable_forces_n) - 1)

    output_path = Path(args.fcstd).resolve()
    working_dir = Path(args.working_dir).resolve()
    working_dir.mkdir(parents=True, exist_ok=True)

    doc_name = output_path.stem.replace(" ", "_")
    existing = None
    try:
        existing = FreeCAD.getDocument(doc_name)
    except NameError:
        existing = None
    if existing is not None:
        FreeCAD.closeDocument(doc_name)
    doc = FreeCAD.newDocument(doc_name)

    geom_obj, axis_obj, joint_loads, metadata = create_geometry_document(doc)
    _analysis, solver_obj = add_analysis(
        doc,
        geom_obj,
        axis_obj,
        joint_loads,
        metadata,
        joint_moments_nmm=peak_joint_moments_nmm,
        mesh_max_mm=args.mesh_max_mm,
        mesh_min_mm=args.mesh_min_mm,
        nonlinear=args.nonlinear,
        result_stages=solver_stage_count,
    )
    actual_output_path = save_document_with_fallback(doc, output_path, working_dir)

    try:
        mesh_obj = doc.getObject("FEMMeshGmsh")
        if mesh_obj is None:
            raise RuntimeError("Mesh object was not created.")
        generate_mesh(mesh_obj, working_dir=working_dir)
        doc.save()

        result_objects = []
        patch_stats = None
        segment_rows = []
        if not args.skip_solve:
            patch_stats = solve_with_patched_input(doc, solver_obj, working_dir=working_dir)
            doc.recompute()
            result_objects = retain_stage_results(
                doc,
                result_objects=sorted_result_objects(doc),
                peak_cable_force_n=solve_peak_cable_force_n,
                target_cable_forces_n=target_cable_forces_n,
            )
            doc.save()

        if args.segment_csv is not None:
            if args.skip_solve:
                raise RuntimeError("--segment-csv requires a solved model; omit --skip-solve.")
            segment_csv_path = Path(args.segment_csv).resolve()
            segment_rows = write_segment_progress_csv(
                doc,
                peak_cable_force_n=solve_peak_cable_force_n,
                moment_arm_mm=args.moment_arm_mm,
                output_path=segment_csv_path,
            )
            print(f"[OK] Segment CSV: {segment_csv_path}")

        stage_mesh_count = 0
        if result_objects and not args.no_stage_views:
            stage_mesh_count = create_stage_visualizations(
                doc,
                result_objects=result_objects,
                peak_cable_force_n=solve_peak_cable_force_n,
                target_cable_forces_n=target_cable_forces_n,
                stage_offset_mm=args.stage_offset_mm,
            )
            doc.save()

        print(f"[OK] Saved FreeCAD model: {actual_output_path}")
        print(f"[OK] Base cable force: {args.cable_force_n:.3f} N | solve peak: {solve_peak_cable_force_n:.3f} N | moment arm: {args.moment_arm_mm:.3f} mm")
        print(f"[OK] Stage cable forces: {', '.join(f'{force_n:.3f}' for force_n in target_cable_forces_n)} N")
        print(f"[OK] Local tendon moment arms: {format_joint_values(local_moment_arms_mm, 'mm')}")
        print(f"[OK] Peak joint moments: {format_joint_values(peak_joint_moments_nmm, 'N*mm')}")

        if args.skip_solve:
            print("[OK] Solve skipped.")
        elif patch_stats is not None:
            print(
                "[OK] Patched CalculiX regions: "
                f"{patch_stats['rigid_element_count']} rigid, {patch_stats['joint_element_count']} compliant"
            )
            if "result_import_warning" in patch_stats:
                print(f"[WARN] Result import warning: {patch_stats['result_import_warning']}")
        if result_objects:
            displacement = getattr(result_objects[-1], "DisplacementLengths", None)
            if displacement and len(displacement) > 0:
                print(f"[OK] Max displacement: {max(displacement):.6g} mm")
        if segment_rows:
            print(f"[OK] Final tip angle: {float(segment_rows[-1]['tip_angle_deg']):.3f} deg")
        if stage_mesh_count:
            print(f"[OK] Stage meshes created: {stage_mesh_count}")

        if not args.no_show:
            launch_freecad_gui_document(actual_output_path)
        return 0
    except Exception:
        fallback_output_path = locals().get("actual_output_path", output_path)
        if not args.no_show and fallback_output_path.exists():
            launch_freecad_gui_document(fallback_output_path)
        raise
    finally:
        FreeCAD.closeDocument(doc.Name)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
