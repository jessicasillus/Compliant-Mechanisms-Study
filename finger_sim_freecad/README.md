# Finger Modeling Source

This folder collects the core modeling code used across the compliant-finger study. The scripts here cover the reduced-order analytical model, the staged FreeCAD/CalculiX FEA workflow, the bundled FEA stage CSV, and the 20-spring distributed-compliance reference model.

## Files

- `Finger_Sim_FreeCAD.py`
  Builds the 3D finger geometry, creates the FreeCAD FEM study, refines the joint mesh, patches material regions in the CalculiX input deck, solves the staged nonlinear analysis, and exports stage-by-stage joint and segment data.

- `Finger_PRBM.py`
  Implements the one-spring pseudo-rigid-body model, including torsional-stiffness scaling with joint thickness, cable-force-to-moment conversion, centerline reconstruction, and plotting helpers.

- `Finger_Sim_20_Springs.py`
  Stores the distributed 20-spring MuJoCo reference model used to compare a smoother compliance distribution against the simpler analytical PRBM.

- `Finger_Sim_VM2_FEA_stages.csv`
  Bundled staged FEA results used by the desktop app and the GitHub Pages browser GUI.

- `freecad_runtime.py`
  Helper utilities for relaunching scripts inside the correct `freecadcmd` runtime when needed.

## Suggested Usage

Review the analytical model:

```powershell
python Finger_PRBM.py --plot-schedule
```

Inspect the FreeCAD driver options:

```powershell
python Finger_Sim_FreeCAD.py --help
```

Run the 20-spring MuJoCo reference model:

```powershell
python Finger_Sim_20_Springs.py
```

## Notes

- `Finger_Sim_FreeCAD.py` expects a working FreeCAD FEM + CalculiX installation.
- `Finger_Sim_20_Springs.py` is a MuJoCo-based comparison model rather than a FreeCAD solver script, but it is kept here so the analytical and simulation-side source files are archived together in one place.
- The large generated `.FCStd` document is intentionally not committed because the file size is too large for a normal GitHub upload.
