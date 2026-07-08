"""Generic robot control tools (work with any robot via the mcp_robot agent)."""

from typing import Optional


def register(mcp, bridge):
    @mcp.tool()
    def list_robots() -> dict:
        """List all robots in the world: name, model/type, current controller,
        position, and whether an MCP agent is attached (has_mcp_agent)."""
        return bridge.command("list_robots")

    @mcp.tool()
    def attach_mcp_controller(robot: str) -> dict:
        """Take control of a robot: swaps its controller to the generic 'mcp_robot'
        agent and restarts it. Afterwards you can use get_robot_devices, set_motor,
        get_camera_image, get_sensor_values, play_motion, execute_robot_code on it.
        NOTE: replaces the robot's own behavior controller (original name is returned
        so it can be restored with set_node_field)."""
        return bridge.command("attach_mcp_controller", {"robot": robot})

    @mcp.tool()
    def restart_controller(robot: str) -> dict:
        """Restart a robot's current controller (whatever it is). Use after changing
        the controller field with set_node_field, or to restore a robot's original
        behavior controller."""
        return bridge.command("restart_controller", {"robot": robot})

    @mcp.tool()
    def get_robot_devices(robot: str) -> dict:
        """List every device on a robot (motors with position/velocity limits,
        cameras, sensors, LEDs, grippers...). Requires the mcp_robot agent."""
        return bridge.robot_command(robot, "get_devices")

    @mcp.tool()
    def set_motor(robot: str, motor: Optional[str] = None,
                  position: Optional[float] = None, velocity: Optional[float] = None,
                  motors: Optional[dict] = None) -> dict:
        """Command robot motors. Single motor: pass motor + position (rad/m) and/or
        velocity. Velocity-only = continuous rotation mode (wheels). Multiple motors
        at once: motors={"left wheel motor": {"velocity": 3.0}, ...}.
        Positions are clamped to the motor's limits."""
        params = {"motors": motors} if motors else {"motor": motor, "position": position,
                                                    "velocity": velocity}
        return bridge.robot_command(robot, "set_motor", params)

    @mcp.tool()
    def get_motor_state(robot: str, motor: str, include_feedback: bool = False) -> dict:
        """Get a motor's target position, velocity, actual measured position (if it
        has a PositionSensor), acceleration and force/torque limits.
        include_feedback=True also measures actual applied force/torque."""
        return bridge.robot_command(robot, "get_motor_state",
                                    {"motor": motor,
                                     "include_feedback": include_feedback})

    @mcp.tool()
    def configure_motor(robot: str, motor: str, acceleration: Optional[float] = None,
                        available_force: Optional[float] = None,
                        available_torque: Optional[float] = None,
                        pid: Optional[list] = None, force: Optional[float] = None,
                        torque: Optional[float] = None) -> dict:
        """Advanced motor control: set acceleration limit, available force/torque,
        PID gains [kp, ki, kd], or apply a DIRECT force/torque (bypasses position
        control — the motor becomes force-actuated until set_motor is called again)."""
        return bridge.robot_command(robot, "configure_motor",
                                    {"motor": motor, "acceleration": acceleration,
                                     "available_force": available_force,
                                     "available_torque": available_torque,
                                     "pid": pid, "force": force, "torque": torque})

    @mcp.tool()
    def export_urdf(robot: str) -> dict:
        """Export the robot's kinematic model (links, joints) as URDF — useful
        context for planning motions of an unfamiliar robot."""
        return bridge.robot_command(robot, "export_urdf")

    @mcp.tool()
    def send_message(robot: str, message: str, channel: Optional[int] = None,
                     emitter: Optional[str] = None) -> dict:
        """Send a message from a robot's Emitter device (inter-robot radio).
        Other robots read it with get_messages. channel selects the radio channel."""
        return bridge.robot_command(robot, "send_message",
                                    {"message": message, "channel": channel,
                                     "emitter": emitter})

    @mcp.tool()
    def get_messages(robot: str, channel: Optional[int] = None,
                     receiver: Optional[str] = None, max_messages: int = 50) -> dict:
        """Read queued messages from a robot's Receiver device (with signal strength
        and direction to the sender when available)."""
        return bridge.robot_command(robot, "get_messages",
                                    {"channel": channel, "receiver": receiver,
                                     "max_messages": max_messages})

    @mcp.tool()
    def set_connector(robot: str, lock: bool, connector: Optional[str] = None) -> dict:
        """Lock/unlock a robot's Connector device (docking or magnetic gripping).
        Locking only attaches when a compatible connector is present (see 'presence')."""
        return bridge.robot_command(robot, "set_connector",
                                    {"lock": lock, "connector": connector})

    @mcp.tool()
    def vacuum_gripper(robot: str, on: bool, gripper: Optional[str] = None) -> dict:
        """Turn a robot's VacuumGripper on/off (suction grasping). Reports whether
        an object is attached."""
        return bridge.robot_command(robot, "vacuum_gripper",
                                    {"on": on, "gripper": gripper})

    @mcp.tool()
    def speak(robot: str, text: str, volume: float = 1.0,
              speaker: Optional[str] = None, language: Optional[str] = None) -> dict:
        """Make a robot talk via its Speaker device (text-to-speech)."""
        return bridge.robot_command(robot, "speak",
                                    {"text": text, "volume": volume,
                                     "speaker": speaker, "language": language})

    @mcp.tool()
    def set_brake(robot: str, damping: float, brake: Optional[str] = None) -> dict:
        """Set a Brake device's damping constant (higher = stronger braking)."""
        return bridge.robot_command(robot, "set_brake",
                                    {"damping": damping, "brake": brake})

    @mcp.tool()
    def display_draw(robot: str, commands: list, display: Optional[str] = None) -> dict:
        """Draw on a robot's Display device. commands = list of ops, e.g.
        [{"op":"clear"}, {"op":"color","value":"0x00FF00"},
        {"op":"text","text":"hi","x":4,"y":4}, {"op":"fill_rect","x":0,"y":20,"w":32,"h":8},
        {"op":"line","x1":0,"y1":0,"x2":63,"y2":63}]. Ops: color, alpha, clear, text,
        pixel, line, rect, fill_rect, oval, fill_oval."""
        return bridge.robot_command(robot, "display_draw",
                                    {"commands": commands, "display": display})

    @mcp.tool()
    def get_battery(robot: str) -> dict:
        """Read the robot's battery level (needs the robot's 'battery' field set)."""
        return bridge.robot_command(robot, "get_battery")

    @mcp.tool()
    def robot_custom_data(robot: str, data: Optional[str] = None) -> dict:
        """Get (data=None) or set the robot's customData field — a lightweight
        shared-string channel between the MCP and the robot's native controllers."""
        if data is None:
            return bridge.robot_command(robot, "get_custom_data")
        return bridge.robot_command(robot, "set_custom_data", {"data": data})

    @mcp.tool()
    def set_led(robot: str, led: Optional[str] = None, value: str = "on") -> dict:
        """Set robot LED(s). value: color name (red/green/blue/white/off), hex like
        '#FF8800', or an integer. Omit 'led' to set all LEDs."""
        return bridge.robot_command(robot, "set_led", {"led": led, "value": value})

    @mcp.tool()
    def play_motion(robot: str, motion_file: str, loop: bool = False) -> dict:
        """Play a Webots .motion file on the robot (absolute path, or filename found
        in the project's motions/ folder). Returns immediately with the duration;
        poll with get_motion_state or advance time with step_simulation."""
        return bridge.robot_command(robot, "play_motion",
                                    {"motion_file": motion_file, "loop": loop})

    @mcp.tool()
    def get_motion_state(robot: str) -> dict:
        """Check whether the robot's current motion is still playing."""
        return bridge.robot_command(robot, "get_motion_state")

    @mcp.tool()
    def stop_motion(robot: str) -> dict:
        """Stop the robot's currently playing motion."""
        return bridge.robot_command(robot, "stop_motion")
