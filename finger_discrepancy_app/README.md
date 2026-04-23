# Finger Discrepancy App

Desktop Tkinter app for comparing:

- 1-spring PRBM
- 20-spring PRBM
- bundled FEA stage data
- experimental joint-angle data

## Run

```powershell
python -m pip install -r requirements.txt
python Finger_Discrepancy_App.py
```

## Optional Experimental Data

You can load a `joint_angles.csv` file from the UI or pass it on the command line:

```powershell
python Finger_Discrepancy_App.py --experimental-csv path\to\joint_angles.csv
```

If no CSV is provided, the app starts with the digitized experimental comparison series that matches the browser demo.
