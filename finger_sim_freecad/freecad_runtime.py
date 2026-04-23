from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Sequence


def _existing_path(candidates: Sequence[str | None]) -> str | None:
    for candidate in candidates:
        if candidate and Path(candidate).exists():
            return str(Path(candidate))
    return None


def find_freecadcmd() -> str | None:
    return _existing_path(
        [
            os.environ.get("FREECADCMD"),
            shutil.which("freecadcmd"),
            shutil.which("FreeCADCmd"),
            r"C:\Program Files\FreeCAD 1.1\bin\freecadcmd.exe",
            r"C:\Program Files\FreeCAD\bin\freecadcmd.exe",
        ]
    )


def find_freecad_gui() -> str | None:
    return _existing_path(
        [
            os.environ.get("FREECADGUI"),
            shutil.which("FreeCAD"),
            shutil.which("FreeCAD.exe"),
            r"C:\Program Files\FreeCAD 1.1\bin\FreeCAD.exe",
            r"C:\Program Files\FreeCAD\bin\FreeCAD.exe",
        ]
    )


def relaunch_with_freecad(
    *,
    script_path: Path,
    argv: Sequence[str],
    missing_module: str,
) -> None:
    freecadcmd = find_freecadcmd()
    if freecadcmd is None:
        raise SystemExit(
            "This script needs FreeCAD's Python environment, but `freecadcmd` could not be found.\n"
            f"Missing module: {missing_module}\n\n"
            "Install FreeCAD or set the `FREECADCMD` environment variable to the full path of `freecadcmd.exe`."
        )

    resolved_script_path = script_path.resolve()
    forwarded_argv = [str(resolved_script_path), *argv]
    code = (
        "import runpy, sys; "
        f"sys.path.insert(0, {str(resolved_script_path.parent)!r}); "
        f"sys.argv = {forwarded_argv!r}; "
        f"runpy.run_path({str(resolved_script_path)!r}, run_name='__main__')"
    )
    print(f"[INFO] Relaunching with FreeCAD: {freecadcmd}")
    completed = subprocess.run(
        [freecadcmd, "-c", code],
        cwd=os.getcwd(),
        check=False,
    )
    raise SystemExit(completed.returncode)


def launch_freecad_gui_document(fcstd_path: Path) -> bool:
    resolved_fcstd_path = fcstd_path.resolve()
    if not resolved_fcstd_path.exists():
        print(f"[WARN] FreeCAD document does not exist yet, so it could not be opened: {resolved_fcstd_path}")
        return False

    freecad_gui = find_freecad_gui()
    if freecad_gui is not None:
        try:
            subprocess.Popen([freecad_gui, str(resolved_fcstd_path)], cwd=str(resolved_fcstd_path.parent))
            print(f"[OK] Opened FreeCAD GUI: {resolved_fcstd_path}")
            return True
        except OSError as exc:
            print(f"[WARN] FreeCAD GUI launch failed via executable ({exc}). Trying Windows file association...")

    try:
        os.startfile(str(resolved_fcstd_path))
        print(f"[OK] Opened FreeCAD document via Windows association: {resolved_fcstd_path}")
        return True
    except OSError as exc:
        print(f"[WARN] Could not open FreeCAD visualization: {exc}")
        return False
