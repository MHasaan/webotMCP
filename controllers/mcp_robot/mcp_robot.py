"""MCP Robot Agent - generic controller for ANY Webots robot.

Assign this controller to a robot (manually or via the bridge's
attach_mcp_controller command). It:
  1. auto-discovers every device on the robot (motors, sensors, cameras, LEDs, ...)
  2. connects to the mcp_bridge agent port and registers under the robot's name
  3. executes commands proxied by the bridge, replying between simulation steps

No robot-specific assumptions: works with NAO, YouBot, e-puck, custom robots, etc.
"""

import base64
import io
import json
import os
import socket
import struct
import sys
import tempfile
import traceback

# Webots sets WEBOTS_HOME for controller processes it spawns.
for _home in filter(None, (os.environ.get("WEBOTS_HOME"),
                           r"C:\Program Files\Webots", "/usr/local/webots",
                           "/Applications/Webots.app")):
    _py_api = os.path.join(_home, "lib", "controller", "python")
    if os.path.isdir(_py_api):
        if _py_api not in sys.path:
            sys.path.insert(0, _py_api)
        break

from controller import Robot, Motion  # noqa: E402

AGENT_PORT = int(os.environ.get("WEBOTS_MCP_AGENT_PORT", "10023"))

# Webots device node-type name -> category
SENSOR_TYPES = {
    "Accelerometer", "Altimeter", "Camera", "Compass", "DistanceSensor", "GPS",
    "Gyro", "InertialUnit", "Lidar", "LightSensor", "PositionSensor", "Radar",
    "RangeFinder", "Receiver", "TouchSensor", "VacuumGripper",
}
ACTUATOR_TYPES = {
    "Brake", "Connector", "Display", "Emitter", "LED", "LinearMotor", "Motor",
    "Muscle", "Pen", "Propeller", "RotationalMotor", "Speaker", "Track",
}


def send_frame(sock, obj):
    data = json.dumps(obj).encode("utf-8")
    sock.sendall(struct.pack(">I", len(data)) + data)


def recv_frame_nonblocking(sock):
    """Return a frame if one is fully available, else None. Raises on disconnect."""
    sock.setblocking(False)
    try:
        header = sock.recv(4, socket.MSG_PEEK)
    except BlockingIOError:
        return None
    finally:
        sock.setblocking(True)
    if header == b"":
        raise ConnectionError("bridge closed connection")
    if len(header) < 4:
        return None
    (length,) = struct.unpack(">I", sock.recv(4))
    buf = b""
    while len(buf) < length:
        chunk = sock.recv(length - len(buf))
        if not chunk:
            raise ConnectionError("bridge closed connection mid-frame")
        buf += chunk
    return json.loads(buf.decode("utf-8"))


class Agent:
    def __init__(self):
        self.robot = Robot()
        self.name = self.robot.getName()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.devices = {}       # name -> device object
        self.device_types = {}  # name -> node type name
        self.enabled_sensors = set()
        self.current_motion = None
        self._discover_devices()
        self.sock = self._connect()

    def _discover_devices(self):
        for i in range(self.robot.getNumberOfDevices()):
            dev = self.robot.getDeviceByIndex(i)
            name = dev.getName()
            self.devices[name] = dev
            self.device_types[name] = type(dev).__name__
        print(f"[mcp_robot:{self.name}] discovered {len(self.devices)} devices", flush=True)

    def _connect(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.connect(("127.0.0.1", AGENT_PORT))
        send_frame(sock, {"register": self.name})
        # blocking read of ack
        header = b""
        while len(header) < 4:
            header += sock.recv(4 - len(header))
        (length,) = struct.unpack(">I", header)
        buf = b""
        while len(buf) < length:
            buf += sock.recv(length - len(buf))
        print(f"[mcp_robot:{self.name}] registered with bridge", flush=True)
        return sock

    # ------------------------------------------------------------------

    def run(self):
        while self.robot.step(self.timestep) != -1:
            while True:
                try:
                    req = recv_frame_nonblocking(self.sock)
                except ConnectionError:
                    print(f"[mcp_robot:{self.name}] bridge disconnected; retrying", flush=True)
                    try:
                        self.sock = self._connect()
                    except OSError:
                        pass
                    break
                if req is None:
                    break
                resp = {"id": req.get("id")}
                try:
                    resp["status"] = "ok"
                    resp["result"] = self.dispatch(req.get("action"), req.get("params") or {})
                except Exception as exc:  # noqa: BLE001
                    resp = {"id": req.get("id"), "status": "error",
                            "error": f"{type(exc).__name__}: {exc}",
                            "traceback": traceback.format_exc(limit=4)}
                send_frame(self.sock, resp)

    def dispatch(self, action, p):
        handler = getattr(self, "cmd_" + str(action), None)
        if handler is None:
            raise ValueError(f"unknown robot action '{action}'")
        return handler(p)

    def get_device(self, name):
        dev = self.devices.get(name)
        if dev is None:
            raise ValueError(f"no device '{name}'. Available: {sorted(self.devices)}")
        return dev

    def _ensure_enabled(self, dev):
        name = dev.getName()
        if name not in self.enabled_sensors and hasattr(dev, "enable"):
            dev.enable(self.timestep)
            self.enabled_sensors.add(name)
            # step once so the sensor has a valid reading
            self.robot.step(self.timestep)

    # ==================================================================
    # Commands
    # ==================================================================

    def cmd_ping(self, p):
        return {"pong": True, "robot": self.name}

    def cmd_get_devices(self, p):
        out = []
        for name, dev in sorted(self.devices.items()):
            dtype = self.device_types[name]
            info = {"name": name, "type": dtype}
            if dtype in ("Motor", "RotationalMotor", "LinearMotor") or hasattr(dev, "getMaxPosition"):
                try:
                    info["min_position"] = dev.getMinPosition()
                    info["max_position"] = dev.getMaxPosition()
                    info["max_velocity"] = dev.getMaxVelocity()
                    info["max_torque"] = dev.getMaxTorque()
                except Exception:  # noqa: BLE001
                    pass
            out.append(info)
        return {"robot": self.name, "devices": out}

    def cmd_set_motor(self, p):
        """Set position and/or velocity for one or many motors.

        params: {"motors": {"name": {"position": x, "velocity": v}}}
             or {"motor": "name", "position": x, "velocity": v}
        """
        motors = p.get("motors")
        if motors is None:
            motors = {p["motor"]: {k: p[k] for k in ("position", "velocity") if k in p}}
        applied = {}
        for name, target in motors.items():
            dev = self.get_device(name)
            if not hasattr(dev, "setPosition"):
                raise ValueError(f"'{name}' is not a motor ({self.device_types[name]})")
            if target.get("velocity") is not None and target.get("position") is None:
                # velocity-control mode
                dev.setPosition(float("inf"))
                dev.setVelocity(float(target["velocity"]))
            else:
                if target.get("position") is not None:
                    pos = float(target["position"])
                    lo, hi = dev.getMinPosition(), dev.getMaxPosition()
                    if lo != hi:  # equal bounds mean unlimited
                        pos = max(lo, min(hi, pos))
                    dev.setPosition(pos)
                if target.get("velocity") is not None:
                    dev.setVelocity(abs(float(target["velocity"])))
            applied[name] = target
        return {"applied": applied}

    def cmd_get_motor_state(self, p):
        name = p["motor"]
        dev = self.get_device(name)
        state = {"target_position": dev.getTargetPosition(), "velocity": dev.getVelocity()}
        # actual position needs the associated PositionSensor
        ps = getattr(dev, "getPositionSensor", lambda: None)()
        if ps is not None:
            self._ensure_enabled(ps)
            state["position"] = ps.getValue()
        for k, getter in (("acceleration", "getAcceleration"),
                          ("available_force", "getAvailableForce"),
                          ("available_torque", "getAvailableTorque"),
                          ("max_force", "getMaxForce"),
                          ("max_torque", "getMaxTorque")):
            try:
                state[k] = getattr(dev, getter)()
            except Exception:  # noqa: BLE001 - rotational vs linear motors differ
                pass
        if p.get("include_feedback"):
            for enable, get, k in (("enableTorqueFeedback", "getTorqueFeedback", "torque_feedback"),
                                   ("enableForceFeedback", "getForceFeedback", "force_feedback")):
                try:
                    getattr(dev, enable)(self.timestep)
                    self.robot.step(self.timestep)
                    state[k] = getattr(dev, get)()
                except Exception:  # noqa: BLE001
                    pass
        return state

    def cmd_configure_motor(self, p):
        """Advanced motor configuration: acceleration limit, available force/torque,
        PID gains, or direct force/torque actuation (bypasses position control)."""
        dev = self.get_device(p["motor"])
        applied = {}
        if p.get("acceleration") is not None:
            dev.setAcceleration(float(p["acceleration"]))
            applied["acceleration"] = p["acceleration"]
        if p.get("available_force") is not None:
            dev.setAvailableForce(float(p["available_force"]))
            applied["available_force"] = p["available_force"]
        if p.get("available_torque") is not None:
            dev.setAvailableTorque(float(p["available_torque"]))
            applied["available_torque"] = p["available_torque"]
        if p.get("pid") is not None:
            kp, ki, kd = [float(v) for v in p["pid"]]
            dev.setControlPID(kp, ki, kd)
            applied["pid"] = [kp, ki, kd]
        if p.get("force") is not None:
            dev.setForce(float(p["force"]))
            applied["force"] = p["force"]
        if p.get("torque") is not None:
            dev.setTorque(float(p["torque"]))
            applied["torque"] = p["torque"]
        if not applied:
            raise ValueError("nothing to configure: pass acceleration, "
                             "available_force, available_torque, pid, force or torque")
        return {"motor": p["motor"], "applied": applied}

    def cmd_export_urdf(self, p):
        """Export the robot's kinematic structure as URDF."""
        return {"robot": self.name, "urdf": self.robot.getUrdf(str(p.get("prefix", "")))}

    def cmd_set_led(self, p):
        name = p.get("led")
        value = p.get("value", 1)
        if isinstance(value, str):
            colors = {"red": 0xFF0000, "green": 0x00FF00, "blue": 0x0000FF,
                      "white": 0xFFFFFF, "off": 0}
            value = colors.get(value.lower())
            if value is None:
                value = int(str(p["value"]).lstrip("#"), 16)
        targets = [name] if name else [n for n, t in self.device_types.items() if t == "LED"]
        for n in targets:
            self.get_device(n).set(int(value))
        return {"leds_set": targets, "value": value}

    def cmd_get_camera_image(self, p):
        cameras = [n for n, t in self.device_types.items() if t == "Camera"]
        name = p.get("camera") or (cameras[0] if cameras else None)
        if not name:
            raise ValueError(f"robot has no camera. Devices: {sorted(self.devices)}")
        cam = self.get_device(name)
        self._ensure_enabled(cam)
        path = os.path.join(tempfile.gettempdir(), f"webots_mcp_cam_{self.name}_{name}.jpg".replace(" ", "_"))
        cam.saveImage(path, int(p.get("quality", 90)))
        with open(path, "rb") as fh:
            data = fh.read()
        os.remove(path)
        return {"camera": name, "width": cam.getWidth(), "height": cam.getHeight(),
                "format": "jpeg", "base64": base64.b64encode(data).decode("ascii")}

    def _first_device(self, dtype, given=None):
        if given:
            return given
        name = next((n for n, t in self.device_types.items() if t == dtype), None)
        if not name:
            raise ValueError(f"robot has no {dtype}. Devices: {sorted(self.devices)}")
        return name

    def cmd_get_recognition(self, p):
        """Ground-truth object detection from a camera's Recognition node."""
        name = self._first_device("Camera", p.get("camera"))
        cam = self.get_device(name)
        self._ensure_enabled(cam)
        if not cam.hasRecognition():
            raise ValueError(
                f"camera '{name}' has no Recognition node. Use the "
                "enable_camera_recognition tool (adds one via the supervisor), "
                "then retry.")
        if cam.getRecognitionSamplingPeriod() <= 0:
            cam.recognitionEnable(self.timestep)
            self.robot.step(self.timestep)
        objects = []
        for o in cam.getRecognitionObjects():
            objects.append({
                "id": o.getId(),
                "model": o.getModel(),
                "position_relative": [round(v, 4) for v in o.getPosition()],
                "orientation_relative": [round(v, 4) for v in o.getOrientation()],
                "size_m": [round(v, 4) for v in o.getSize()],
                "bbox_center_px": list(o.getPositionOnImage()),
                "bbox_size_px": list(o.getSizeOnImage()),
                "colors": [round(v, 3) for v in o.getColors()[:3 * o.getNumberOfColors()]],
            })
        return {"camera": name, "width": cam.getWidth(), "height": cam.getHeight(),
                "fov": cam.getFov(), "count": len(objects), "objects": objects}

    def cmd_get_segmentation_image(self, p):
        """Per-object color mask from the camera's Recognition segmentation."""
        name = self._first_device("Camera", p.get("camera"))
        cam = self.get_device(name)
        self._ensure_enabled(cam)
        if not cam.hasRecognition():
            raise ValueError(
                f"camera '{name}' has no Recognition node. Use "
                "enable_camera_recognition(segmentation=True) first.")
        if cam.getRecognitionSamplingPeriod() <= 0:
            cam.recognitionEnable(self.timestep)
        if not cam.hasRecognitionSegmentation():
            raise ValueError(
                f"Recognition node of '{name}' has segmentation disabled. Use "
                "enable_camera_recognition(segmentation=True) first.")
        if not cam.isRecognitionSegmentationEnabled():
            cam.enableRecognitionSegmentation()
            self.robot.step(self.timestep)
        path = os.path.join(tempfile.gettempdir(),
                            f"webots_mcp_seg_{self.name}_{name}.png".replace(" ", "_"))
        cam.saveRecognitionSegmentationImage(path, 100)
        with open(path, "rb") as fh:
            data = fh.read()
        os.remove(path)
        return {"camera": name, "format": "png",
                "base64": base64.b64encode(data).decode("ascii")}

    def cmd_get_depth_image(self, p):
        """Depth map from a RangeFinder: grayscale image + numeric stats."""
        name = self._first_device("RangeFinder", p.get("rangefinder"))
        rf = self.get_device(name)
        self._ensure_enabled(rf)
        path = os.path.join(tempfile.gettempdir(),
                            f"webots_mcp_depth_{self.name}_{name}.png".replace(" ", "_"))
        rf.saveImage(path, 100)
        with open(path, "rb") as fh:
            data = fh.read()
        os.remove(path)
        ranges = [v for v in rf.getRangeImage() if v == v]  # drop NaN
        finite = [v for v in ranges if v != float("inf")]
        stats = {"min": round(min(finite), 4), "max": round(max(finite), 4),
                 "mean": round(sum(finite) / len(finite), 4)} if finite else None
        return {"rangefinder": name, "width": rf.getWidth(), "height": rf.getHeight(),
                "fov": rf.getFov(), "min_range": rf.getMinRange(),
                "max_range": rf.getMaxRange(), "depth_stats_m": stats,
                "format": "png", "base64": base64.b64encode(data).decode("ascii")}

    def cmd_get_radar_targets(self, p):
        name = self._first_device("Radar", p.get("radar"))
        radar = self.get_device(name)
        self._ensure_enabled(radar)
        targets = [{"distance": round(t.distance, 4), "azimuth": round(t.azimuth, 4),
                    "speed": round(t.speed, 4),
                    "received_power": round(t.receiver_power, 4)}
                   for t in radar.getTargets()]
        return {"radar": name, "min_range": radar.getMinRange(),
                "max_range": radar.getMaxRange(),
                "horizontal_fov": radar.getHorizontalFov(),
                "count": len(targets), "targets": targets}

    def cmd_get_sensor_values(self, p):
        """Read one sensor or all readable sensors."""
        wanted = p.get("sensor")
        readings = {}
        for name, dev in self.devices.items():
            dtype = self.device_types[name]
            if wanted and name != wanted:
                continue
            if dtype not in SENSOR_TYPES or dtype in ("Camera", "RangeFinder", "Lidar", "Radar", "Receiver"):
                if not wanted:
                    continue
            try:
                self._ensure_enabled(dev)
                if hasattr(dev, "getValues"):
                    readings[name] = {"type": dtype, "values": list(dev.getValues())}
                elif hasattr(dev, "getValue"):
                    readings[name] = {"type": dtype, "value": dev.getValue()}
                elif hasattr(dev, "getRollPitchYaw"):
                    readings[name] = {"type": dtype, "roll_pitch_yaw": list(dev.getRollPitchYaw())}
                elif wanted:
                    raise ValueError(f"don't know how to read {dtype} '{name}'")
            except Exception as exc:  # noqa: BLE001
                readings[name] = {"type": dtype, "error": str(exc)}
        if wanted and not readings:
            raise ValueError(f"no sensor '{wanted}'. Devices: {sorted(self.devices)}")
        return {"robot": self.name, "sensors": readings}

    def cmd_get_lidar_summary(self, p):
        name = p.get("lidar") or next((n for n, t in self.device_types.items() if t == "Lidar"), None)
        if not name:
            raise ValueError("robot has no lidar")
        dev = self.get_device(name)
        self._ensure_enabled(dev)
        image = dev.getRangeImage()
        n = len(image)
        stride = max(1, n // int(p.get("max_points", 72)))
        out = {"lidar": name, "fov": dev.getFov(), "min_range": dev.getMinRange(),
               "max_range": dev.getMaxRange(), "resolution": dev.getHorizontalResolution(),
               "ranges_downsampled": [round(image[i], 3) for i in range(0, n, stride)]}
        # LLM-friendly polar occupancy: nearest obstacle per angular sector
        sectors = int(p.get("sectors", 12))
        if sectors > 0 and n:
            fov = dev.getFov()
            max_r = dev.getMaxRange()
            occ = []
            per = max(1, n // sectors)
            for s in range(0, n, per):
                chunk = [v for v in image[s:s + per] if v == v and v != float("inf")]
                nearest = min(chunk) if chunk else None
                a0 = -fov / 2 + fov * s / n
                a1 = -fov / 2 + fov * min(s + per, n) / n
                occ.append({"bearing_deg": [round(a0 * 57.2958, 1), round(a1 * 57.2958, 1)],
                            "nearest_m": round(nearest, 3) if nearest is not None
                            and nearest < max_r * 0.999 else None})
            out["occupancy"] = occ
            out["occupancy_note"] = ("nearest obstacle per sector, bearings relative "
                                     "to lidar axis (negative = left); null = clear")
        if p.get("include_point_cloud"):
            try:
                # getPointCloud() returns [] until tracking is enabled, then
                # all-zero points until a couple of steps have populated the
                # buffer; enable and step until the points are actually valid.
                dev.enablePointCloud()
                pts = dev.getPointCloud()

                def _valid(ps):
                    return any(q.x * q.x + q.y * q.y + q.z * q.z > 1e-9 for q in ps)

                tries = 0
                while (not pts or not _valid(pts)) and tries < 3:
                    self.robot.step(self.timestep)
                    pts = dev.getPointCloud()
                    tries += 1
                k = max(1, len(pts) // int(p.get("max_points", 72)))
                out["point_cloud"] = [[round(q.x, 3), round(q.y, 3), round(q.z, 3)]
                                      for q in pts[::k]
                                      if q.x == q.x and abs(q.x) != float("inf")]
            except Exception as exc:  # noqa: BLE001 - point cloud must be supported
                out["point_cloud_error"] = str(exc)
        return out

    def cmd_play_motion(self, p):
        path = p["motion_file"]
        if not os.path.isabs(path):
            # look relative to the controller dir and common motions folders
            candidates = [
                path,
                os.path.join(os.path.dirname(__file__), path),
                os.path.join(os.path.dirname(__file__), "..", "..", "motions", os.path.basename(path)),
            ]
            path = next((c for c in candidates if os.path.exists(c)), path)
        if not os.path.exists(path):
            raise ValueError(f"motion file not found: {p['motion_file']}")
        if self.current_motion:
            self.current_motion.stop()
        self.current_motion = Motion(path)
        if not self.current_motion.isValid():
            self.current_motion = None
            raise ValueError(f"invalid motion file: {path}")
        self.current_motion.setLoop(bool(p.get("loop", False)))
        self.current_motion.play()
        return {"playing": os.path.basename(path), "duration_ms": self.current_motion.getDuration()}

    def cmd_get_motion_state(self, p):
        if self.current_motion is None:
            return {"playing": False}
        return {"playing": not self.current_motion.isOver(),
                "time_ms": self.current_motion.getTime(),
                "duration_ms": self.current_motion.getDuration()}

    def cmd_stop_motion(self, p):
        if self.current_motion:
            self.current_motion.stop()
            self.current_motion = None
        return {"stopped": True}

    def cmd_send_message(self, p):
        """Broadcast a message via an Emitter device (inter-robot comms)."""
        name = self._first_device("Emitter", p.get("emitter"))
        em = self.get_device(name)
        if p.get("channel") is not None:
            em.setChannel(int(p["channel"]))
        em.send(str(p["message"]))
        return {"sent": True, "emitter": name, "channel": em.getChannel()}

    def cmd_get_messages(self, p):
        """Drain a Receiver device's queue (messages from other robots' Emitters)."""
        name = self._first_device("Receiver", p.get("receiver"))
        rc = self.get_device(name)
        self._ensure_enabled(rc)
        if p.get("channel") is not None:
            rc.setChannel(int(p["channel"]))
        messages = []
        while rc.getQueueLength() > 0 and len(messages) < int(p.get("max_messages", 50)):
            entry = {"data": rc.getString()}
            try:
                entry["signal_strength"] = round(rc.getSignalStrength(), 4)
                entry["direction"] = [round(v, 3) for v in rc.getEmitterDirection()]
            except Exception:  # noqa: BLE001
                pass
            messages.append(entry)
            rc.nextPacket()
        return {"receiver": name, "channel": rc.getChannel(),
                "count": len(messages), "messages": messages}

    def cmd_set_connector(self, p):
        """Lock/unlock a Connector (docking / magnetic gripping)."""
        name = self._first_device("Connector", p.get("connector"))
        con = self.get_device(name)
        con.enablePresence(self.timestep)
        self.robot.step(self.timestep)
        if p.get("lock"):
            con.lock()
        else:
            con.unlock()
        return {"connector": name, "locked": bool(con.isLocked()),
                "presence": con.getPresence(),
                "presence_note": "1 = compatible connector in range, 0 = none, -1 = n/a"}

    def cmd_vacuum_gripper(self, p):
        """Turn a VacuumGripper on/off (suction grasping)."""
        name = self._first_device("VacuumGripper", p.get("gripper"))
        vg = self.get_device(name)
        vg.enablePresence(self.timestep)
        if p.get("on"):
            vg.turnOn()
        else:
            vg.turnOff()
        self.robot.step(self.timestep)
        return {"gripper": name, "on": bool(vg.isOn()),
                "object_attached": bool(vg.getPresence())}

    def cmd_speak(self, p):
        """Text-to-speech through a Speaker device."""
        name = self._first_device("Speaker", p.get("speaker"))
        sp = self.get_device(name)
        if p.get("language"):
            sp.setLanguage(str(p["language"]))
        sp.speak(str(p["text"]), float(p.get("volume", 1.0)))
        return {"speaker": name, "speaking": str(p["text"])}

    def cmd_set_brake(self, p):
        """Set a Brake device's damping constant (N·m·s or N·s)."""
        name = self._first_device("Brake", p.get("brake"))
        self.get_device(name).setDampingConstant(float(p["damping"]))
        return {"brake": name, "damping": float(p["damping"])}

    def cmd_display_draw(self, p):
        """Draw on a Display device. commands = list of ops:
        {"op":"color","value":"0xFF0000"}, {"op":"clear"}, {"op":"text",...},
        {"op":"line"/"rect"/"fill_rect"/"oval"/"fill_oval"/"pixel", coords...}"""
        name = self._first_device("Display", p.get("display"))
        d = self.get_device(name)
        w, h = d.getWidth(), d.getHeight()
        for cmd in p.get("commands", []):
            op = cmd.get("op")
            if op == "color":
                d.setColor(int(str(cmd["value"]), 0))
            elif op == "alpha":
                d.setAlpha(float(cmd["value"]))
            elif op == "clear":
                prev = cmd.get("color", "0x000000")
                d.setColor(int(str(prev), 0))
                d.fillRectangle(0, 0, w, h)
            elif op == "text":
                d.drawText(str(cmd["text"]), int(cmd.get("x", 0)), int(cmd.get("y", 0)))
            elif op == "pixel":
                d.drawPixel(int(cmd["x"]), int(cmd["y"]))
            elif op == "line":
                d.drawLine(int(cmd["x1"]), int(cmd["y1"]), int(cmd["x2"]), int(cmd["y2"]))
            elif op == "rect":
                d.drawRectangle(int(cmd["x"]), int(cmd["y"]), int(cmd["w"]), int(cmd["h"]))
            elif op == "fill_rect":
                d.fillRectangle(int(cmd["x"]), int(cmd["y"]), int(cmd["w"]), int(cmd["h"]))
            elif op == "oval":
                d.drawOval(int(cmd["cx"]), int(cmd["cy"]), int(cmd["a"]), int(cmd["b"]))
            elif op == "fill_oval":
                d.fillOval(int(cmd["cx"]), int(cmd["cy"]), int(cmd["a"]), int(cmd["b"]))
            else:
                raise ValueError(f"unknown display op '{op}'")
        return {"display": name, "width": w, "height": h,
                "commands_applied": len(p.get("commands", []))}

    def cmd_get_battery(self, p):
        self.robot.batterySensorEnable(self.timestep)
        self.robot.step(self.timestep)
        value = self.robot.batterySensorGetValue()
        return {"robot": self.name, "battery": value,
                "note": "NaN/-1 means the robot has no battery field configured"}

    def cmd_get_custom_data(self, p):
        return {"robot": self.name, "custom_data": self.robot.getCustomData()}

    def cmd_set_custom_data(self, p):
        self.robot.setCustomData(str(p.get("data", "")))
        return {"robot": self.name, "custom_data": self.robot.getCustomData()}

    def cmd_execute_code(self, p):
        namespace = {"robot": self.robot, "agent": self, "devices": self.devices, "result": None}
        stdout = io.StringIO()
        old = sys.stdout
        sys.stdout = stdout
        try:
            exec(compile(p["code"], "<mcp>", "exec"), namespace)  # noqa: S102
        finally:
            sys.stdout = old
        result = namespace.get("result")
        try:
            json.dumps(result)
        except (TypeError, ValueError):
            result = repr(result)
        return {"result": result, "stdout": stdout.getvalue()}


if __name__ == "__main__":
    Agent().run()
