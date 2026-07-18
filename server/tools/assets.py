"""Webots asset library, project scaffolding, and application preferences.

Makes the MCP work with the WHOLE Webots installation: every bundled robot/
object/appearance PROTO, sample worlds, new-project creation, and the
application's own settings (registry-backed preferences).
"""

import sys
try:
    import defusedxml.ElementTree as ET  # hardened parser if available
except ImportError:
    import xml.etree.ElementTree as ET  # noqa: S405 - parsing trusted local Webots file
from pathlib import Path
from typing import Optional

from webots_home import find_webots_home

MCP_ROOT = Path(__file__).resolve().parents[2]

_proto_index = None  # cached {name: {url, base_type, description, parameters}}


def _build_proto_index():
    """Index every official Webots PROTO from resources/proto-list.xml (~850 assets:
    robots, objects, appearances, devices...), plus any local .proto files."""
    global _proto_index
    if _proto_index is None:
        _proto_index = {}
        xml_path = find_webots_home() / "resources" / "proto-list.xml"
        if xml_path.exists():
            root = ET.parse(xml_path).getroot()
            for proto in root.iter("proto"):
                name = proto.findtext("name")
                if not name:
                    continue
                _proto_index[name] = {
                    "url": proto.findtext("url"),
                    "base_type": proto.findtext("base-type"),
                    "description": (proto.findtext("description") or "").replace("\\n", " ").strip(),
                    "parameters": (proto.findtext("parameters") or "").split("\\n"),
                    "license": proto.findtext("license"),
                }
        # local project PROTOs (this repo and any user protos folder)
        for base in (MCP_ROOT / "protos",):
            if base.exists():
                for p in base.glob("**/*.proto"):
                    _proto_index.setdefault(p.stem, {"url": str(p).replace("\\", "/"),
                                                     "base_type": None,
                                                     "description": "local PROTO file",
                                                     "parameters": [], "license": None})
    return _proto_index


def register(mcp, bridge):
    @mcp.tool()
    def search_protos(query: str, max_results: int = 20, include_details: bool = False) -> dict:
        """Search ALL PROTO assets bundled with Webots (hundreds of robots, objects,
        furniture, appearances, sensors...). E.g. 'nao', 'youbot', 'e-puck', 'table',
        'wall'. Returns names + paths; include_details adds description/parameters.
        Use add_proto_to_world to make a PROTO usable in the current world."""
        index = _build_proto_index()
        q = query.lower()
        hits = [(n, m) for n, m in sorted(index.items())
                if q in n.lower() or q in (m.get("description") or "").lower()]
        # name matches first
        hits.sort(key=lambda x: q not in x[0].lower())
        results = []
        for name, meta in hits[:max_results]:
            entry = {"name": name, "base_type": meta.get("base_type")}
            if include_details:
                entry["description"] = meta.get("description")
                entry["parameters"] = meta.get("parameters")
            results.append(entry)
        return {"query": query, "total_matches": len(hits), "results": results,
                "total_protos_indexed": len(index)}

    @mcp.tool()
    def get_proto_info(name: str) -> dict:
        """Get a PROTO's description, parameters, and base node type — check this
        before spawning it so you pass valid fields."""
        index = _build_proto_index()
        meta = index.get(name)
        if meta is None:
            close = [n for n in index if name.lower() in n.lower()][:10]
            raise ValueError(f"PROTO '{name}' not found. Close matches: {close}")
        return {"name": name, **meta}

    @mcp.tool()
    def add_proto_to_world(proto_name: str, world_path: Optional[str] = None,
                           reload_world: bool = False) -> dict:
        """Declare a PROTO (EXTERNPROTO) in a world file so it can be spawned with
        spawn_node, e.g. add_proto_to_world('E-puck') then spawn_node('E-puck { }').
        Defaults to the currently loaded world. Requires a world reload to take
        effect (reload_world=True does it via the bridge, resetting the simulation)."""
        if world_path is None:
            world_path = bridge.command("get_simulation_state")["world"]
        index = _build_proto_index()
        meta = index.get(proto_name)
        if meta is None:
            raise ValueError(f"PROTO '{proto_name}' not found; use search_protos first")
        wp = Path(world_path)
        text = wp.read_text(encoding="utf-8")
        line = f'EXTERNPROTO "{meta["url"]}"'
        if line in text or f"/{proto_name}.proto" in text:
            added = False
        else:
            lines = text.splitlines()
            insert_at = 1
            for i, l in enumerate(lines):
                if l.startswith("EXTERNPROTO"):
                    insert_at = i + 1
            lines.insert(insert_at, line)
            wp.write_text("\n".join(lines) + "\n", encoding="utf-8")
            added = True
        result = {"world": str(wp), "externproto": line, "added": added}
        if reload_world:
            bridge.command("reset_simulation", {"reload_world": True})
            result["reloaded"] = True
        else:
            result["note"] = "reload the world (reset_simulation reload_world=True) before spawning"
        return result

    @mcp.tool()
    def list_sample_worlds(query: str = "", max_results: int = 30) -> dict:
        """Search the sample worlds shipped with Webots (robot demos, environments).
        Open any of them with launch_webots or load_world — the MCP bridge can be
        auto-installed."""
        base = find_webots_home() / "projects"
        hits = [str(p) for p in base.glob("**/worlds/*.wbt")
                if query.lower() in p.stem.lower() and not p.stem.startswith(".")]
        return {"query": query, "total": len(hits), "worlds": hits[:max_results]}

    @mcp.tool()
    def create_project(directory: str, world_name: str = "world",
                       include_bridge: bool = True) -> dict:
        """Create a new Webots project: standard folder layout (worlds/, controllers/,
        protos/) with a minimal world (floor + lighting), optionally with the MCP
        bridge pre-installed. Returns the world path to open with launch_webots."""
        root = Path(directory)
        for sub in ("worlds", "controllers", "protos"):
            (root / sub).mkdir(parents=True, exist_ok=True)
        bridge_snippet = (
            '\nRobot {\n  translation 0 0 -10\n  name "mcp_bridge"\n'
            '  controller "mcp_bridge"\n  supervisor TRUE\n}\n') if include_bridge else "\n"
        world = f"""#VRML_SIM R2025a utf8
WorldInfo {{
  basicTimeStep 32
}}
Viewpoint {{
  position 2 -2 1.5
  orientation -0.35 0.35 0.87 1.7
}}
Background {{
  skyColor [
    0.55 0.75 0.95
  ]
}}
DirectionalLight {{
  direction -0.4 -0.5 -1
  intensity 2.5
  castShadows TRUE
}}
DEF FLOOR Solid {{
  translation 0 0 -0.05
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.75 0.75 0.72
        roughness 0.9
        metalness 0
      }}
      geometry Box {{
        size 10 10 0.1
      }}
    }}
  ]
  name "floor"
  boundingObject Box {{
    size 10 10 0.1
  }}
}}{bridge_snippet}"""
        world_path = root / "worlds" / f"{world_name}.wbt"
        world_path.write_text(world, encoding="utf-8")
        if include_bridge:
            import shutil
            for name in ("mcp_bridge", "mcp_robot"):
                dst = root / "controllers" / name
                dst.mkdir(exist_ok=True)
                shutil.copy2(MCP_ROOT / "controllers" / name / f"{name}.py", dst / f"{name}.py")
        return {"project": str(root), "world": str(world_path),
                "next_step": f"launch_webots(world_path='{world_path}')"}

    @mcp.tool()
    def create_controller(project_dir: str, name: str, code: str,
                          language: str = "python") -> dict:
        """Create a robot controller in a project's controllers/ folder. code is the
        full controller source. Assign it to a robot with
        set_node_field(robot, 'controller', name) and restart via
        execute_supervisor_code."""
        ext = {"python": ".py", "c": ".c", "cpp": ".cpp", "java": ".java"}.get(language)
        if ext is None:
            raise ValueError("language must be python, c, cpp, or java")
        ctrl_dir = Path(project_dir) / "controllers" / name
        ctrl_dir.mkdir(parents=True, exist_ok=True)
        path = ctrl_dir / f"{name}{ext}"
        path.write_text(code, encoding="utf-8")
        return {"created": str(path),
                "note": "non-python languages must be compiled before use"}

    # -- application preferences ------------------------------------------------
    # Windows: registry HKCU\SOFTWARE\Cyberbotics\Webots-<version>
    # Linux:   ~/.config/Cyberbotics/Webots-<version>.conf (INI)
    # macOS:   ~/Library/Preferences/com.cyberbotics.Webots-<version>.plist

    def _windows_pref_key():
        import winreg
        with winreg.OpenKey(winreg.HKEY_CURRENT_USER, r"SOFTWARE\Cyberbotics") as cb:
            versions = []
            i = 0
            while True:
                try:
                    versions.append(winreg.EnumKey(cb, i))
                    i += 1
                except OSError:
                    break
        if not versions:
            raise FileNotFoundError("no Webots preferences in registry")
        return r"SOFTWARE\Cyberbotics" + "\\" + sorted(versions)[-1]

    def _linux_pref_file():
        conf_dir = Path.home() / ".config" / "Cyberbotics"
        confs = sorted(conf_dir.glob("Webots-*.conf"))
        if not confs:
            raise FileNotFoundError(f"no Webots config found in {conf_dir}")
        return confs[-1]

    @mcp.tool()
    def get_webots_preferences() -> dict:
        """Read the Webots application preferences (startup mode, python command,
        rendering options, telemetry, etc.). Windows registry / Linux .conf backed."""
        try:
            if sys.platform == "win32":
                import winreg
                key_path = _windows_pref_key()
                prefs = {}

                def _read_values(path, prefix):
                    with winreg.OpenKey(winreg.HKEY_CURRENT_USER, path) as key:
                        n_sub, n_val, _ = winreg.QueryInfoKey(key)
                        for i in range(n_val):
                            name, value, _ = winreg.EnumValue(key, i)
                            prefs[f"{prefix}{name}"] = value
                        for j in range(n_sub):
                            sub = winreg.EnumKey(key, j)
                            _read_values(f"{path}\\{sub}", f"{prefix}{sub}/")

                # Qt stores each preference group (General, OpenGL, ...) as a subkey;
                # flatten to 'Group/name' to match set_webots_preference and Linux.
                _read_values(key_path, "")
                return {"source": f"HKCU\\{key_path}", "preferences": prefs}
            conf = _linux_pref_file()
            import configparser
            cp = configparser.ConfigParser()
            cp.read(conf)
            prefs = {f"{s}/{k}": v for s in cp.sections() for k, v in cp[s].items()}
            return {"source": str(conf), "preferences": prefs}
        except FileNotFoundError as exc:
            return {"error": str(exc),
                    "note": "run Webots at least once to create its preferences"}

    @mcp.tool()
    def set_webots_preference(name: str, value: str) -> dict:
        """Set a Webots application preference (e.g. 'General/startupMode'='Pause',
        'General/pythonCommand', 'General/disableSaveWarning'). Takes effect on the
        next Webots launch. Use get_webots_preferences to see valid names."""
        if sys.platform == "win32":
            import winreg
            key_path = _windows_pref_key()
            # 'Group/name' maps to the subkey 'Group' + value 'name' (Qt layout).
            group, sep, leaf = name.rpartition("/")
            full_path = f"{key_path}\\{group}" if sep else key_path
            with winreg.CreateKey(winreg.HKEY_CURRENT_USER, full_path) as key:
                winreg.SetValueEx(key, leaf, 0, winreg.REG_SZ, str(value))
        else:
            conf = _linux_pref_file()
            import configparser
            cp = configparser.ConfigParser()
            cp.read(conf)
            section, _, key = name.partition("/")
            if not cp.has_section(section):
                cp.add_section(section)
            cp[section][key] = str(value)
            with open(conf, "w", encoding="utf-8") as fh:
                cp.write(fh)
        return {"set": name, "value": value, "note": "applies on next Webots launch"}
