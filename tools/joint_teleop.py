#!/usr/bin/env python3
"""Keyboard teleop that sends sensor_msgs/JointState commands.

Paste any joint names. Each joint is commanded by position, velocity or
effort. Made for Isaac Sim's "ROS2 Subscribe Joint State" ->
"Articulation Controller" graph, but works with anything that takes
JointState commands. Linux / macOS terminal only.

Examples (source ROS 2 first):
  joint_teleop.py --mode vel wheel_1_joint wheel_2_joint wheel_3_joint wheel_4_joint
  joint_teleop.py "wheel_1_joint:vel, wheel_2_joint:vel, rob_arm_1_joint, arm_rota_1_joint"
  joint_teleop.py --topic /isaac_joint_commands --state-topic /isaac_joint_states ...

Per joint: name[:mode], mode = pos | vel | eff (default --mode, pos).
Commas or spaces between joints both work.

- Nothing is sent before the first key press. Until then, position joints
  take their current value from --state-topic, so the robot does not jump.
- One message is sent per mode (only the position, velocity or effort
  array filled), at --rate Hz, repeating the last command.
- On quit, all velocity and effort joints are sent 0 once.
"""
import argparse
import os
import select
import sys
import termios
import threading
import tty

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState

MODES = {
    "p": "position", "pos": "position", "position": "position",
    "v": "velocity", "vel": "velocity", "velocity": "velocity",
    "e": "effort", "eff": "effort", "effort": "effort",
}
MODE_ORDER = ["position", "velocity", "effort"]
DEFAULT_STEP = {"position": 0.05, "velocity": 0.5, "effort": 0.5}

HELP = (
    "keys: up/down or 1-9 select   right/w/+ increase   left/s/- decrease\n"
    "      ] / [ step x2 / /2   0 zero joint   m change mode\n"
    "      a apply to all joints with the same mode   space stop all vel/eff   q quit"
)


def parse_joints(items, default_mode):
    joints = []
    for item in items:
        for token in item.replace(",", " ").split():
            name, _, mode = token.partition(":")
            full = MODES.get((mode or default_mode).lower())
            if full is None:
                raise SystemExit(f"Unknown mode '{mode}' for {name}: use pos, vel or eff")
            joints.append((name, full))
    if not joints:
        raise SystemExit("No joint names given")
    names = [n for n, _ in joints]
    dup = sorted({n for n in names if names.count(n) > 1})
    if dup:
        raise SystemExit(f"Duplicate joints: {', '.join(dup)}")
    return joints


class JointTeleop(Node):
    def __init__(self, joints, topic, state_topic, rate):
        super().__init__("joint_teleop")
        self.topic = topic
        self.rate = rate
        self.names = [n for n, _ in joints]
        self.modes = [m for _, m in joints]
        self.values = [0.0] * len(joints)
        self.steps = [DEFAULT_STEP[m] for m in self.modes]
        self.from_state = [False] * len(joints)
        self.selected = 0
        self.all_same_mode = False
        self.active = False  # nothing is sent before the first key
        self.lock = threading.Lock()
        self.pub = self.create_publisher(JointState, topic, 10)
        if state_topic:
            self.create_subscription(JointState, state_topic, self.on_state, 10)
        self.create_timer(1.0 / rate, self.publish)

    # --- ROS --------------------------------------------------------------
    def on_state(self, msg):
        """Before the first key: start position joints at their current value."""
        with self.lock:
            if self.active:
                return
            for name, pos in zip(msg.name, msg.position):
                if name in self.names:
                    i = self.names.index(name)
                    if self.modes[i] == "position":
                        self.values[i] = pos
                        self.from_state[i] = True

    def publish(self, only_stop=False):
        with self.lock:
            if not self.active:
                return
            stamp = self.get_clock().now().to_msg()
            for mode in MODE_ORDER:
                if only_stop and mode == "position":
                    continue
                idx = [i for i, m in enumerate(self.modes) if m == mode]
                if not idx:
                    continue
                msg = JointState()
                msg.header.stamp = stamp
                msg.name = [self.names[i] for i in idx]
                values = [0.0 if only_stop else self.values[i] for i in idx]
                setattr(msg, mode, values)
                self.pub.publish(msg)

    def stop(self):
        """Send 0 to every velocity and effort joint."""
        with self.lock:
            for i, m in enumerate(self.modes):
                if m != "position":
                    self.values[i] = 0.0
        self.publish(only_stop=True)

    # --- keys -------------------------------------------------------------
    def targets(self):
        if not self.all_same_mode:
            return [self.selected]
        mode = self.modes[self.selected]
        return [i for i, m in enumerate(self.modes) if m == mode]

    def on_key(self, key):  # not "handle": that name is taken by rclpy's Node
        """Apply one key. Returns False to quit."""
        if key in ("q", "\x03"):
            return False
        with self.lock:
            n = len(self.names)
            sel = self.selected
            if key in ("up", "k"):
                self.selected = (sel - 1) % n
            elif key in ("down", "j"):
                self.selected = (sel + 1) % n
            elif key.isdigit() and key != "0" and int(key) <= n:
                self.selected = int(key) - 1
            elif key in ("right", "w", "+", "="):
                for i in self.targets():
                    self.values[i] += self.steps[sel]
                self.active = True
            elif key in ("left", "s", "-", "_"):
                for i in self.targets():
                    self.values[i] -= self.steps[sel]
                self.active = True
            elif key == "]":
                for i in self.targets():
                    self.steps[i] *= 2.0
            elif key == "[":
                for i in self.targets():
                    self.steps[i] /= 2.0
            elif key == "0":
                for i in self.targets():
                    self.values[i] = 0.0
                self.active = True
            elif key == "m":
                mode = MODE_ORDER[(MODE_ORDER.index(self.modes[sel]) + 1) % 3]
                self.modes[sel] = mode
                self.values[sel] = 0.0
                self.steps[sel] = DEFAULT_STEP[mode]
                self.from_state[sel] = False
            elif key == "a":
                self.all_same_mode = not self.all_same_mode
            elif key == " ":
                for i, m in enumerate(self.modes):
                    if m != "position":
                        self.values[i] = 0.0
                self.active = True
        return True

    def screen(self):
        with self.lock:
            state = (f"sending at {self.rate:g} Hz" if self.active
                     else "waiting: nothing sent before the first key")
            lines = [f"joint_teleop -> {self.topic}   [{state}]",
                     f"apply to all joints with the same mode: {'ON' if self.all_same_mode else 'off'}",
                     "",
                     "     #  joint                          mode         value       step"]
            targets = set(self.targets())
            for i, name in enumerate(self.names):
                mark = ">" if i == self.selected else ("*" if i in targets else " ")
                note = "  (from state)" if self.from_state[i] and not self.active else ""
                lines.append(f"  {mark} {i + 1:2d}  {name:30s} {self.modes[i]:9s} "
                             f"{self.values[i]:10.4f} {self.steps[i]:10.4f}{note}")
            lines += ["", HELP]
        return "\n".join(lines)


def read_key(fd, timeout):
    if not select.select([fd], [], [], timeout)[0]:
        return None
    ch = os.read(fd, 1)
    if ch == b"\x1b":  # arrow keys: ESC [ A/B/C/D
        seq = b""
        while len(seq) < 2 and select.select([fd], [], [], 0.01)[0]:
            seq += os.read(fd, 1)
        return {b"[A": "up", b"[B": "down", b"[C": "right", b"[D": "left"}.get(seq, "esc")
    return ch.decode(errors="ignore")


def main():
    ap = argparse.ArgumentParser(
        description="Keyboard teleop that sends sensor_msgs/JointState commands.",
        epilog="joint: name[:pos|vel|eff]; commas or spaces between joints.")
    ap.add_argument("joints", nargs="+", help="joint names, e.g. wheel_1_joint:vel")
    ap.add_argument("--mode", default="pos", help="default mode: pos | vel | eff (default pos)")
    ap.add_argument("--topic", default="/joint_command", help="command topic (default /joint_command)")
    ap.add_argument("--state-topic", default="/joint_states",
                    help="read start positions from here; '' to disable (default /joint_states)")
    ap.add_argument("--rate", type=float, default=20.0, help="send rate in Hz (default 20)")
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

    joints = parse_joints(args.joints, args.mode)
    fd = sys.stdin.fileno()
    if not os.isatty(fd):
        raise SystemExit("joint_teleop needs a terminal (keyboard input)")

    rclpy.init(args=sys.argv)
    node = JointTeleop(joints, args.topic, args.state_topic, args.rate)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            sys.stdout.write("\x1b[H\x1b[J" + node.screen() + "\n")
            sys.stdout.flush()
            key = read_key(fd, 0.5)  # redraw at least twice a second
            if key is not None and not node.on_key(key):
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.stop()
        print("\nStopped: velocity and effort joints sent 0.")
        executor.shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
