"""Visual perception and sensor tools."""

import base64
import io
from typing import Optional

from mcp.server.fastmcp import Image


def _to_image(b64_image: str, max_dim: int = 1024) -> Image:
    data = base64.b64decode(b64_image)
    fmt = "png" if data[:8] == b"\x89PNG\r\n\x1a\n" else "jpeg"
    try:
        from PIL import Image as PILImage
        img = PILImage.open(io.BytesIO(data))
        if max(img.size) > max_dim:
            img.thumbnail((max_dim, max_dim))
            buf = io.BytesIO()
            img.convert("RGB").save(buf, format="JPEG", quality=85)
            data = buf.getvalue()
            fmt = "jpeg"
    except ImportError:
        pass  # Pillow missing: return full-size image
    return Image(data=data, format=fmt)


def register(mcp, bridge):
    @mcp.tool()
    def get_viewport_screenshot(quality: int = 90, max_dim: int = 1024,
                                view_target: Optional[str] = None,
                                view_position: Optional[list] = None,
                                keep_viewpoint: bool = False) -> Image:
        """Take a screenshot of the Webots 3D view — use this to SEE the scene.
        Positioned capture: view_target = node name/DEF/id (or omit) to aim at;
        view_position = [x,y,z] camera location. With only view_target, a framing
        position is chosen automatically. The viewpoint is restored afterwards
        unless keep_viewpoint=True. Plain call captures the current view."""
        result = bridge.command("screenshot",
                                {"quality": quality, "view_target": view_target,
                                 "view_position": view_position,
                                 "keep_viewpoint": keep_viewpoint}, timeout=60.0)
        return _to_image(result["base64"], max_dim)

    @mcp.tool()
    def screenshot_multiview(target: Optional[str] = None, batch: str = "surround",
                             radius: Optional[float] = None, azimuths: int = 8,
                             elevations: Optional[list] = None, max_dim: int = 512,
                             quality: int = 80) -> list:
        """Capture the scene from MULTIPLE angles in one call — the way to verify 3D
        placement, occlusion and appearance from all sides. batch='surround' gives 6
        canonical views (4 sides, eye-level 3/4, top-down); batch='orbit' gives an
        azimuths x elevations grid (capped at 12 frames). target = node or omit for
        the whole scene; radius = camera distance (auto from scene bounds). Each
        image is preceded by its angle caption. Viewpoint is restored afterwards."""
        result = bridge.command("screenshot_batch",
                                {"target": target, "batch": batch, "radius": radius,
                                 "azimuths": azimuths, "elevations": elevations,
                                 "quality": quality}, timeout=300.0)
        out = [f"scene center {result['scene_center']}, camera radius "
               f"{round(result['capture_radius'], 2)}m:"]
        for s in result["screenshots"]:
            out.append(f"azimuth {s['azimuth']}°, elevation {s['elevation']}°:")
            out.append(_to_image(s["base64"], max_dim))
        return out

    @mcp.tool()
    def get_camera_image(robot: str, camera: Optional[str] = None, max_dim: int = 1024) -> Image:
        """Get an image from a robot's onboard camera (requires the mcp_robot agent —
        see attach_mcp_controller). Omit 'camera' to use the robot's first camera."""
        result = bridge.robot_command(robot, "get_camera_image", {"camera": camera})
        return _to_image(result["base64"], max_dim)

    @mcp.tool()
    def get_camera_recognition(robot: str, camera: Optional[str] = None) -> dict:
        """GROUND-TRUTH object detection from a robot camera: the simulator reports
        every recognizable object the camera sees — model name, node id, 3D position/
        orientation relative to the camera, physical size, pixel bounding box, and
        colors. No ML involved. Requires a Recognition node on the camera; if missing,
        call enable_camera_recognition first. Requires the mcp_robot agent."""
        return bridge.robot_command(robot, "get_recognition", {"camera": camera})

    @mcp.tool()
    def enable_camera_recognition(robot: str, camera: Optional[str] = None,
                                  segmentation: bool = False) -> dict:
        """Add (or configure) a Recognition node on a robot's camera via the
        supervisor, so get_camera_recognition / get_segmentation_image work.
        segmentation=True also enables per-object mask rendering. Note: objects are
        only recognizable if their Solid has a non-empty recognitionColors field."""
        return bridge.command("enable_camera_recognition",
                              {"robot": robot, "camera": camera,
                               "segmentation": segmentation})

    @mcp.tool()
    def get_segmentation_image(robot: str, camera: Optional[str] = None,
                               max_dim: int = 1024) -> Image:
        """Per-object segmentation mask from a robot camera's Recognition node —
        each recognized object rendered in its own flat color, background black.
        Requires enable_camera_recognition(segmentation=True) once beforehand."""
        result = bridge.robot_command(robot, "get_segmentation_image",
                                      {"camera": camera})
        return _to_image(result["base64"], max_dim)

    @mcp.tool()
    def get_depth_image(robot: str, rangefinder: Optional[str] = None,
                        max_dim: int = 1024) -> list:
        """Depth map from a robot's RangeFinder: grayscale depth image (near=dark)
        plus min/max/mean distance stats. Requires the mcp_robot agent."""
        r = bridge.robot_command(robot, "get_depth_image",
                                 {"rangefinder": rangefinder})
        caption = (f"rangefinder '{r['rangefinder']}' {r['width']}x{r['height']}, "
                   f"range {r['min_range']}-{r['max_range']}m, "
                   f"depth stats: {r['depth_stats_m']}")
        return [caption, _to_image(r["base64"], max_dim)]

    @mcp.tool()
    def get_radar_targets(robot: str, radar: Optional[str] = None) -> dict:
        """Read a robot's radar: detected targets with distance, azimuth, speed and
        received power. Requires the mcp_robot agent."""
        return bridge.robot_command(robot, "get_radar_targets", {"radar": radar})

    @mcp.tool()
    def get_recording_status() -> dict:
        """Check movie recording status (ready / failed) without stopping it."""
        return bridge.command("get_recording_status")

    @mcp.tool()
    def get_sensor_values(robot: str, sensor: Optional[str] = None) -> dict:
        """Read a robot's sensors (GPS, IMU, gyro, distance/light/touch sensors,
        position sensors, compass, accelerometer...). Omit 'sensor' to read all
        simple sensors at once. Requires the mcp_robot agent on that robot."""
        return bridge.robot_command(robot, "get_sensor_values", {"sensor": sensor})

    @mcp.tool()
    def get_lidar_summary(robot: str, lidar: Optional[str] = None, max_points: int = 72,
                          sectors: int = 12, include_point_cloud: bool = False) -> dict:
        """Get a robot's lidar scan: downsampled ranges PLUS an LLM-friendly polar
        occupancy summary (nearest obstacle per angular sector, bearings in degrees,
        null = clear). include_point_cloud=True adds downsampled 3D points
        (lidar frame)."""
        return bridge.robot_command(robot, "get_lidar_summary",
                                    {"lidar": lidar, "max_points": max_points,
                                     "sectors": sectors,
                                     "include_point_cloud": include_point_cloud})

    @mcp.tool()
    def export_screenshot(path: str, quality: int = 90) -> dict:
        """Save a full-quality screenshot of the 3D view to an image file on disk
        (PNG or JPG by extension) instead of returning it inline."""
        return bridge.command("export_screenshot", {"path": path, "quality": quality},
                              timeout=30.0)

    @mcp.tool()
    def start_movie_recording(path: str, width: int = 1280, height: int = 720,
                              quality: int = 90, acceleration: int = 1) -> dict:
        """Start recording the 3D view to an .mp4 file. The simulation must be
        running (realtime/fast) to capture frames. Stop with stop_movie_recording."""
        return bridge.command("start_movie", {"path": path, "width": width,
                                              "height": height, "quality": quality,
                                              "acceleration": acceleration})

    @mcp.tool()
    def stop_movie_recording() -> dict:
        """Stop the movie recording and wait for encoding to finish."""
        return bridge.command("stop_movie", {}, timeout=120.0)

    @mcp.tool()
    def start_animation_recording(path: str) -> dict:
        """Start recording an interactive HTML5 3D animation (.html) of the
        simulation — shareable, scrubbable playback in a browser."""
        return bridge.command("start_animation", {"path": path})

    @mcp.tool()
    def stop_animation_recording() -> dict:
        """Stop the HTML5 animation recording."""
        return bridge.command("stop_animation", {})

    @mcp.tool()
    def set_label(text: str, label_id: int = 0, x: float = 0.05, y: float = 0.05,
                  size: float = 0.08, color: str = "0xFFFFFF",
                  transparency: float = 0.0) -> dict:
        """Overlay a text label on the 3D view (debugging/annotation). Coordinates are
        0..1 relative to the viewport; color is hex like '0xFF0000'. Empty text with
        the same label_id removes the label."""
        return bridge.command("set_label", {"text": text, "label_id": label_id, "x": x,
                                            "y": y, "size": size, "color": color,
                                            "transparency": transparency})
