# Compliant Mechanisms Study

This repository packages the full ME 7751 Project 2 submission for the tendon-driven compliant finger study. It includes the final paper, the slideshow PDF, the browser-based GitHub Pages GUI, the desktop discrepancy app, the analytical PRBM code, the FreeCAD/CalculiX FEA driver, and the distributed 20-spring reference model.

## Deliverables

- [paper/project2.pdf](paper/project2.pdf): final report PDF
- [paper/project2_slides.pdf](paper/project2_slides.pdf): slideshow PDF uploaded with the report assets
- [index.html](index.html): GitHub Pages landing page and browser GUI host
- [site.js](site.js): browser-side PRBM, FEA interpolation, plotting, and table logic
- [site.css](site.css): GitHub Pages styling

## Core Source Files

- [finger_sim_freecad/Finger_Sim_FreeCAD.py](finger_sim_freecad/Finger_Sim_FreeCAD.py): builds the FreeCAD geometry, applies staged loading, patches CalculiX material regions, and exports the FEA stage CSV
- [finger_sim_freecad/Finger_PRBM.py](finger_sim_freecad/Finger_PRBM.py): analytical pseudo-rigid-body model, stiffness scaling, force schedule generation, and 2D centerline reconstruction
- [finger_sim_freecad/Finger_Sim_20_Springs.py](finger_sim_freecad/Finger_Sim_20_Springs.py): 20-spring MuJoCo reference model with tendon wrapping for distributed-compliance comparison
- [finger_discrepancy_app/Finger_Discrepancy_App.py](finger_discrepancy_app/Finger_Discrepancy_App.py): local Tkinter and Matplotlib comparison app
- [finger_sim_freecad/Finger_Sim_VM2_FEA_stages.csv](finger_sim_freecad/Finger_Sim_VM2_FEA_stages.csv): bundled FEA stage data reused by the desktop app and the browser GUI

## Browser GUI

Live site:

[GUI app](https://jessicasillus.github.io/Compliant-Mechanisms-Study/)

The GitHub Pages app is the lightweight presentation layer for the project. It compares the 1-spring PRBM, the 20-spring PRBM, and the bundled FEA stages on a shared cable-force axis. The current browser GUI is intentionally limited to the 0 to 5 N range because that is the cleanest validated comparison window from the staged FEA export, and all displayed bend values are shown as positive magnitudes for consistency.

## Repository Layout

- `paper/`: final report PDF and slideshow PDF
- `finger_discrepancy_app/`: local desktop GUI and Python requirements
- `finger_sim_freecad/`: analytical, MuJoCo-reference, and FreeCAD/CalculiX modeling code plus the exported stage CSV
- `index.html`, `site.css`, `site.js`: browser-based GitHub Pages interface

## Modeling Workflow

1. Use `Finger_PRBM.py` to evaluate the reduced-order joint stiffness model and reconstruct the finger centerline under cable force.
2. Use `Finger_Sim_FreeCAD.py` to build the 3D solid finger, assign material regions, mesh the geometry, solve staged nonlinear loading in CalculiX, and export the FEA stage CSV.
3. Use `Finger_Sim_20_Springs.py` as a higher-resolution distributed-compliance reference model for comparison to the simpler one-spring PRBM.
4. Use `Finger_Discrepancy_App.py` or the GitHub Pages GUI to compare PRBM and FEA responses at the tip and joint levels.

## Run Locally

Install the desktop app dependencies:

```powershell
python -m pip install -r finger_discrepancy_app/requirements.txt
```

Launch the desktop discrepancy app:

```powershell
python finger_discrepancy_app/Finger_Discrepancy_App.py
```

Run the analytical PRBM script directly:

```powershell
python finger_sim_freecad/Finger_PRBM.py --plot-schedule
```

Inspect the FreeCAD/CalculiX driver entry point:

```powershell
python finger_sim_freecad/Finger_Sim_FreeCAD.py --help
```

Run the 20-spring MuJoCo reference model if MuJoCo is installed:

```powershell
python finger_sim_freecad/Finger_Sim_20_Springs.py
```

## Notes

- The large `Finger_Sim_VM2_FEM.FCStd` file is not committed because it exceeds a practical normal GitHub upload size.
- The browser GUI uses the bundled staged CSV rather than rerunning FreeCAD in the browser.
- The published Pages site now links directly to the report, the slideshow, and the main source files so the repository works as a complete project archive.
