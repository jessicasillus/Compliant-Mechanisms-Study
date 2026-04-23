# Compliant Mechanisms Study

This repository packages the Project 2 paper, the desktop discrepancy app, the FreeCAD/CalculiX source, and a browser demo that can be published with GitHub Pages.

## Contents

- [paper/project2.pdf](paper/project2.pdf): final PDF for the project report
- [finger_discrepancy_app](finger_discrepancy_app): desktop comparison app
- [finger_sim_freecad](finger_sim_freecad): FreeCAD and CalculiX source plus the bundled FEA stage CSV
- [index.html](index.html): browser-friendly demo for GitHub Pages

## Browser Demo

Enable GitHub Pages from `Settings -> Pages` and publish from the `main` branch root.

After Pages is enabled, the site will be available at:

`https://jessicasillus.github.io/Compliant-Mechanisms-Study/`

The browser demo uses:

- the bundled `Finger_Sim_VM2_FEA_stages.csv`
- the same 1-spring and 20-spring PRBM equations used by the desktop app
- the digitized experimental comparison points already embedded in the app logic

## Run The Desktop App

1. Install Python 3.11+.
2. Install the Python dependencies:

```powershell
python -m pip install -r finger_discrepancy_app/requirements.txt
```

3. Launch the app:

```powershell
python finger_discrepancy_app/Finger_Discrepancy_App.py
```

Optional:

- Load a real `joint_angles.csv` from the app UI to replace the built-in digitized experimental series.
- Install FreeCAD 1.1 if you want the app to refresh FEA results locally through `freecadcmd`.

## Notes

- The current `Finger_Sim_VM2_FEM.FCStd` file is about 415 MB, so it is not committed here because it exceeds normal GitHub file limits.
- If you want the raw `.FCStd` model in the repository later, the clean options are Git LFS or attaching it to a GitHub release.
