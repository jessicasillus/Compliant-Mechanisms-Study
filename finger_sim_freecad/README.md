# Finger Sim FreeCAD

This folder contains the FreeCAD and CalculiX source used to generate the compliant-finger FEA stages.

## Files

- `Finger_Sim_FreeCAD.py`: main FreeCAD/CalculiX driver
- `Finger_PRBM.py`: analytical PRBM helpers shared by the desktop app
- `freecad_runtime.py`: relaunch helpers for `freecadcmd`
- `Finger_Sim_VM2_FEA_stages.csv`: bundled stage data used by the desktop app and the Pages demo

## Run Locally

If FreeCAD is installed and `freecadcmd` is available, the script can relaunch itself into the FreeCAD Python environment:

```powershell
python Finger_Sim_FreeCAD.py --help
```

## Large Model Note

The generated `Finger_Sim_VM2_FEM.FCStd` document is not committed here because the current file is too large for a normal GitHub commit.
