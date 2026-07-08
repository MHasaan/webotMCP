"""Locate the Webots installation on any machine (Windows / Linux / macOS).

Resolution order: WEBOTS_HOME env var -> Windows registry -> webots on PATH ->
well-known install locations.
"""

import os
import shutil
import sys
from pathlib import Path

_cached = None


def _candidates():
    if sys.platform == "win32":
        for base in (os.environ.get("ProgramFiles", r"C:\Program Files"),
                     os.environ.get("LOCALAPPDATA", ""),
                     r"C:\Program Files", r"D:\Webots", r"I:\Webots"):
            if base:
                yield Path(base) / "Webots" if not str(base).lower().endswith("webots") else Path(base)
    elif sys.platform == "darwin":
        yield Path("/Applications/Webots.app")
        yield Path.home() / "Applications" / "Webots.app"
    else:
        yield Path("/usr/local/webots")
        yield Path("/opt/webots")
        yield Path("/snap/webots/current/usr/share/webots")


def _from_registry():
    if sys.platform != "win32":
        return None
    try:
        import winreg
        for hive in (winreg.HKEY_CURRENT_USER, winreg.HKEY_LOCAL_MACHINE):
            try:
                with winreg.OpenKey(hive, r"SOFTWARE\Cyberbotics") as cb:
                    versions = []
                    i = 0
                    while True:
                        try:
                            versions.append(winreg.EnumKey(cb, i))
                            i += 1
                        except OSError:
                            break
                    for ver in sorted(versions, reverse=True):
                        try:
                            with winreg.OpenKey(cb, ver) as k:
                                home, _ = winreg.QueryValueEx(k, "webotsHome")
                                if home and Path(home).exists():
                                    return Path(home)
                        except OSError:
                            continue
            except OSError:
                continue
    except ImportError:
        pass
    return None


def _looks_like_home(p: Path) -> bool:
    if not p or not p.exists():
        return False
    if sys.platform == "darwin":
        return (p / "Contents" / "MacOS" / "webots").exists()
    return (p / "resources").exists() or (p / "lib" / "controller").exists()


def find_webots_home() -> Path:
    global _cached
    if _cached is not None:
        return _cached
    env = os.environ.get("WEBOTS_HOME")
    if env and _looks_like_home(Path(env)):
        _cached = Path(env)
        return _cached
    reg = _from_registry()
    if reg and _looks_like_home(reg):
        _cached = reg
        return _cached
    exe = shutil.which("webots")
    if exe:
        p = Path(exe).resolve()
        for parent in p.parents:
            if _looks_like_home(parent):
                _cached = parent
                return _cached
    for cand in _candidates():
        if _looks_like_home(cand):
            _cached = cand
            return _cached
    # last resort: return the conventional path with a helpful error deferred to use time
    _cached = Path(env or r"C:\Program Files\Webots" if sys.platform == "win32" else "/usr/local/webots")
    return _cached


def find_webots_executable(console: bool = True) -> Path:
    """Path to the webots launcher (console variant preferred for log capture)."""
    home = find_webots_home()
    if sys.platform == "win32":
        names = ["webots.exe", "webotsw.exe"] if console else ["webotsw.exe", "webots.exe"]
        for sub in ("msys64/mingw64/bin", "bin", "."):
            for name in names:
                p = home / sub / name
                if p.exists():
                    return p
    elif sys.platform == "darwin":
        p = home / "Contents" / "MacOS" / "webots"
        if p.exists():
            return p
    else:
        for p in (home / "webots", home / "bin" / "webots"):
            if p.exists():
                return p
        exe = shutil.which("webots")
        if exe:
            return Path(exe)
    raise FileNotFoundError(
        f"Webots executable not found (searched under {home}). "
        "Install Webots or set the WEBOTS_HOME environment variable.")
