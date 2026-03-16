#!/usr/bin/env python3

from __future__ import annotations

import math
import threading
import tkinter as tk
from dataclasses import dataclass

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu


@dataclass
class ImuSnapshot:
    stamp_sec: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0


class ImuAxisCheckNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_axis_check_gui")

        self.declare_parameter("imu_topic", "/handsfree/imu")
        self.declare_parameter("stale_timeout_sec", 1.0)
        self.declare_parameter("tilt_threshold_deg", 4.0)
        self.declare_parameter("turn_threshold_rad_s", 0.15)
        self.declare_parameter("refresh_ms", 80)
        self.declare_parameter("window_title", "HENES IMU Axis Check")

        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.stale_timeout_sec = float(self.get_parameter("stale_timeout_sec").value)
        self.tilt_threshold_deg = float(self.get_parameter("tilt_threshold_deg").value)
        self.turn_threshold_rad_s = float(self.get_parameter("turn_threshold_rad_s").value)
        self.refresh_ms = max(30, int(self.get_parameter("refresh_ms").value))
        self.window_title = str(self.get_parameter("window_title").value)

        self._snapshot = ImuSnapshot()
        self._lock = threading.Lock()

        self.create_subscription(Imu, self.imu_topic, self.imu_cb, 20)
        self.get_logger().info(f"IMU axis check GUI listening to {self.imu_topic}")

    def imu_cb(self, msg: Imu) -> None:
        roll, pitch, yaw = self.quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        with self._lock:
            self._snapshot = ImuSnapshot(
                stamp_sec=self.get_clock().now().nanoseconds * 1e-9,
                roll_deg=math.degrees(roll),
                pitch_deg=math.degrees(pitch),
                yaw_deg=math.degrees(yaw),
                gyro_x=float(msg.angular_velocity.x),
                gyro_y=float(msg.angular_velocity.y),
                gyro_z=float(msg.angular_velocity.z),
                accel_x=float(msg.linear_acceleration.x),
                accel_y=float(msg.linear_acceleration.y),
                accel_z=float(msg.linear_acceleration.z),
            )

    def get_snapshot(self) -> ImuSnapshot:
        with self._lock:
            return ImuSnapshot(**self._snapshot.__dict__)

    @staticmethod
    def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw


class ImuAxisWindow:
    BG = "#f2ecdf"
    PANEL = "#fffaf1"
    PANEL_ALT = "#ede3cf"
    TEXT = "#1e242b"
    MUTED = "#6e757d"
    GOOD = "#2c9961"
    WARN = "#cb7d00"
    BAD = "#be3b34"
    BLUE = "#1664c0"
    PURPLE = "#7d4bd6"

    def __init__(self, node: ImuAxisCheckNode) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title(node.window_title)
        self.root.geometry("1260x820")
        self.root.configure(bg=self.BG)
        self.root.protocol("WM_DELETE_WINDOW", self._close)

        self.canvas = tk.Canvas(
            self.root,
            width=1260,
            height=820,
            bg=self.BG,
            highlightthickness=0,
        )
        self.canvas.pack(fill="both", expand=True)

        self._closed = False

    def run(self) -> None:
        self._tick()
        self.root.mainloop()

    def _close(self) -> None:
        self._closed = True
        self.root.destroy()

    def _tick(self) -> None:
        if self._closed:
            return
        snapshot = self.node.get_snapshot()
        now_sec = self.node.get_clock().now().nanoseconds * 1e-9
        fresh = snapshot.stamp_sec > 0.0 and (now_sec - snapshot.stamp_sec) <= self.node.stale_timeout_sec
        self._draw(snapshot, fresh)
        self.root.after(self.node.refresh_ms, self._tick)

    def _draw(self, s: ImuSnapshot, fresh: bool) -> None:
        c = self.canvas
        c.delete("all")

        c.create_rectangle(0, 0, 1260, 820, fill=self.BG, outline="")
        c.create_rectangle(0, 0, 1260, 88, fill="#e4d7bb", outline="")
        c.create_text(
            38,
            34,
            text="HENES IMU Axis Check",
            anchor="w",
            font=("Helvetica", 24, "bold"),
            fill=self.TEXT,
        )
        c.create_text(
            40,
            64,
            text="Lift the car slightly and check which label lights up. If the opposite side lights, tell me that result.",
            anchor="w",
            font=("Helvetica", 11),
            fill=self.MUTED,
        )

        self._card(30, 112, 380, 300, "1. Left / Right Tilt")
        self._card(440, 112, 380, 300, "2. Front / Rear Tilt")
        self._card(850, 112, 380, 300, "3. Turn Direction")
        self._card(30, 442, 1200, 338, "How To Test")

        self._draw_roll_panel(30, 112, 380, 300, s, fresh)
        self._draw_pitch_panel(440, 112, 380, 300, s, fresh)
        self._draw_yaw_panel(850, 112, 380, 300, s, fresh)
        self._draw_help_panel(30, 442, 1200, 338, s, fresh)

    def _card(self, x: int, y: int, w: int, h: int, title: str) -> None:
        c = self.canvas
        c.create_rectangle(x, y, x + w, y + h, fill=self.PANEL, outline="#cfbea0", width=2)
        c.create_rectangle(x, y, x + w, y + 10, fill=self.PANEL_ALT, outline="")
        c.create_text(
            x + 22,
            y + 34,
            text=title,
            anchor="w",
            font=("Helvetica", 18, "bold"),
            fill=self.TEXT,
        )

    def _status_text(self, fresh: bool) -> tuple[str, str]:
        if fresh:
            return "LIVE", self.GOOD
        return "STALE", self.BAD

    def _roll_state(self, roll_deg: float) -> tuple[str, str]:
        if roll_deg > self.node.tilt_threshold_deg:
            return "LEFT UP", self.GOOD
        if roll_deg < -self.node.tilt_threshold_deg:
            return "RIGHT UP", self.BAD
        return "LEVEL", self.MUTED

    def _pitch_state(self, pitch_deg: float) -> tuple[str, str]:
        if pitch_deg > self.node.tilt_threshold_deg:
            return "FRONT UP", self.GOOD
        if pitch_deg < -self.node.tilt_threshold_deg:
            return "REAR UP", self.BAD
        return "LEVEL", self.MUTED

    def _turn_state(self, gyro_z: float) -> tuple[str, str]:
        if gyro_z > self.node.turn_threshold_rad_s:
            return "CCW / LEFT TURN", self.GOOD
        if gyro_z < -self.node.turn_threshold_rad_s:
            return "CW / RIGHT TURN", self.BAD
        return "STILL", self.MUTED

    def _pill(self, x: int, y: int, text: str, color: str) -> None:
        w = 12 * len(text) + 26
        self.canvas.create_rectangle(x, y, x + w, y + 34, fill=color, outline="")
        self.canvas.create_text(
            x + w / 2,
            y + 18,
            text=text,
            font=("Helvetica", 11, "bold"),
            fill="#ffffff" if color != self.MUTED else "#f8f8f8",
        )

    def _draw_roll_panel(self, x: int, y: int, w: int, h: int, s: ImuSnapshot, fresh: bool) -> None:
        state, color = self._roll_state(s.roll_deg)
        live_text, live_color = self._status_text(fresh)
        self._pill(x + w - 140, y + 18, live_text, live_color)

        cx = x + w / 2
        cy = y + 180
        body = [cx - 68, cy - 34, cx + 68, cy + 34]
        self.canvas.create_rectangle(*body, fill="#c6d8f5", outline=self.BLUE, width=3)
        self.canvas.create_rectangle(cx - 48, cy - 16, cx + 48, cy + 16, fill="#90b6ec", outline="")
        self.canvas.create_text(cx, cy - 58, text="FRONT", font=("Helvetica", 12, "bold"), fill=self.TEXT)
        self.canvas.create_text(cx - 118, cy, text="LEFT", font=("Helvetica", 12, "bold"), fill=self.TEXT)
        self.canvas.create_text(cx + 118, cy, text="RIGHT", font=("Helvetica", 12, "bold"), fill=self.TEXT)

        arrow = 54
        if state == "LEFT UP":
            self.canvas.create_line(cx - 118, cy + arrow / 2, cx - 118, cy - arrow / 2, arrow=tk.FIRST, width=8, fill=self.GOOD)
            self.canvas.create_line(cx + 118, cy - arrow / 2, cx + 118, cy + arrow / 2, arrow=tk.LAST, width=8, fill=self.BAD)
        elif state == "RIGHT UP":
            self.canvas.create_line(cx - 118, cy - arrow / 2, cx - 118, cy + arrow / 2, arrow=tk.LAST, width=8, fill=self.BAD)
            self.canvas.create_line(cx + 118, cy + arrow / 2, cx + 118, cy - arrow / 2, arrow=tk.FIRST, width=8, fill=self.GOOD)
        else:
            self.canvas.create_line(cx - 118, cy - 24, cx - 118, cy + 24, width=5, fill=self.MUTED)
            self.canvas.create_line(cx + 118, cy - 24, cx + 118, cy + 24, width=5, fill=self.MUTED)

        self.canvas.create_text(cx, y + 248, text=state, font=("Helvetica", 22, "bold"), fill=color)
        self.canvas.create_text(
            cx,
            y + 278,
            text=f"roll = {s.roll_deg:+.2f} deg",
            font=("Helvetica", 14),
            fill=self.TEXT,
        )

    def _draw_pitch_panel(self, x: int, y: int, w: int, h: int, s: ImuSnapshot, fresh: bool) -> None:
        state, color = self._pitch_state(s.pitch_deg)
        live_text, live_color = self._status_text(fresh)
        self._pill(x + w - 140, y + 18, live_text, live_color)

        cx = x + w / 2
        cy = y + 184
        car = [cx - 82, cy - 24, cx + 82, cy + 24]
        self.canvas.create_rectangle(*car, fill="#f8d7b8", outline=self.WARN, width=3)
        self.canvas.create_rectangle(cx - 30, cy - 44, cx + 42, cy - 24, fill="#f1b480", outline="")
        self.canvas.create_text(cx + 102, cy - 6, text="FRONT", font=("Helvetica", 12, "bold"), fill=self.TEXT)
        self.canvas.create_text(cx - 108, cy - 6, text="REAR", font=("Helvetica", 12, "bold"), fill=self.TEXT)

        if state == "FRONT UP":
            self.canvas.create_line(cx + 100, cy + 42, cx + 100, cy - 42, arrow=tk.FIRST, width=8, fill=self.GOOD)
            self.canvas.create_line(cx - 100, cy - 42, cx - 100, cy + 42, arrow=tk.LAST, width=8, fill=self.BAD)
        elif state == "REAR UP":
            self.canvas.create_line(cx + 100, cy - 42, cx + 100, cy + 42, arrow=tk.LAST, width=8, fill=self.BAD)
            self.canvas.create_line(cx - 100, cy + 42, cx - 100, cy - 42, arrow=tk.FIRST, width=8, fill=self.GOOD)
        else:
            self.canvas.create_line(cx + 100, cy - 24, cx + 100, cy + 24, width=5, fill=self.MUTED)
            self.canvas.create_line(cx - 100, cy - 24, cx - 100, cy + 24, width=5, fill=self.MUTED)

        self.canvas.create_text(cx, y + 248, text=state, font=("Helvetica", 22, "bold"), fill=color)
        self.canvas.create_text(
            cx,
            y + 278,
            text=f"pitch = {s.pitch_deg:+.2f} deg",
            font=("Helvetica", 14),
            fill=self.TEXT,
        )

    def _draw_yaw_panel(self, x: int, y: int, w: int, h: int, s: ImuSnapshot, fresh: bool) -> None:
        state, color = self._turn_state(s.gyro_z)
        live_text, live_color = self._status_text(fresh)
        self._pill(x + w - 140, y + 18, live_text, live_color)

        cx = x + w / 2
        cy = y + 172
        radius = 82
        self.canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, outline=self.PURPLE, width=3)
        self.canvas.create_text(cx, cy - radius - 22, text="TOP VIEW", font=("Helvetica", 12, "bold"), fill=self.TEXT)
        self.canvas.create_text(cx, cy + 2, text="NOSE", font=("Helvetica", 14, "bold"), fill=self.TEXT)
        self.canvas.create_line(cx, cy + 28, cx, cy - 48, arrow=tk.LAST, width=8, fill=self.PURPLE)

        if state == "CCW / LEFT TURN":
            self.canvas.create_arc(cx - 112, cy - 112, cx + 112, cy + 112, start=50, extent=250, style=tk.ARC, outline=self.GOOD, width=8)
            self.canvas.create_line(cx - 58, cy - 82, cx - 34, cy - 112, arrow=tk.LAST, width=8, fill=self.GOOD)
        elif state == "CW / RIGHT TURN":
            self.canvas.create_arc(cx - 112, cy - 112, cx + 112, cy + 112, start=240, extent=250, style=tk.ARC, outline=self.BAD, width=8)
            self.canvas.create_line(cx + 60, cy - 82, cx + 34, cy - 112, arrow=tk.LAST, width=8, fill=self.BAD)

        self.canvas.create_text(cx, y + 248, text=state, font=("Helvetica", 20, "bold"), fill=color)
        self.canvas.create_text(
            cx,
            y + 274,
            text=f"gyro z = {s.gyro_z:+.3f} rad/s",
            font=("Helvetica", 14),
            fill=self.TEXT,
        )
        self.canvas.create_text(
            cx,
            y + 298,
            text=f"yaw = {s.yaw_deg:+.2f} deg",
            font=("Helvetica", 13),
            fill=self.MUTED,
        )

    def _draw_help_panel(self, x: int, y: int, w: int, h: int, s: ImuSnapshot, fresh: bool) -> None:
        c = self.canvas
        lines = [
            "Step 1. Keep the car flat. Values should stay near LEVEL.",
            "Step 2. Lift the front slightly. If FRONT UP lights, front/back sign is correct.",
            "Step 3. Lift the left side slightly. If LEFT UP lights, left/right sign is correct.",
            "Step 4. Rotate the car counterclockwise when viewed from above. If CCW / LEFT TURN lights, yaw sign is correct.",
            "Step 5. If any opposite label lights, tell me exactly which one lit and I will patch the driver.",
        ]
        for idx, line in enumerate(lines):
            c.create_text(
                x + 26,
                y + 62 + idx * 34,
                text=line,
                anchor="w",
                font=("Helvetica", 15 if idx == 0 else 13),
                fill=self.TEXT if idx == 0 else self.MUTED,
            )

        c.create_rectangle(x + 24, y + 206, x + w - 24, y + h - 24, fill="#f7efdd", outline="#d3c09c", width=2)
        c.create_text(
            x + 46,
            y + 236,
            text="Current Raw Values",
            anchor="w",
            font=("Helvetica", 16, "bold"),
            fill=self.TEXT,
        )

        c.create_text(
            x + 48,
            y + 282,
            text=f"roll {s.roll_deg:+7.2f} deg    pitch {s.pitch_deg:+7.2f} deg    yaw {s.yaw_deg:+7.2f} deg",
            anchor="w",
            font=("Courier", 16, "bold"),
            fill=self.TEXT,
        )
        c.create_text(
            x + 48,
            y + 322,
            text=f"gyro x {s.gyro_x:+7.3f}    gyro y {s.gyro_y:+7.3f}    gyro z {s.gyro_z:+7.3f}   [rad/s]",
            anchor="w",
            font=("Courier", 15),
            fill=self.TEXT,
        )
        c.create_text(
            x + 48,
            y + 362,
            text=f"accel x {s.accel_x:+7.3f}   accel y {s.accel_y:+7.3f}   accel z {s.accel_z:+7.3f}   [m/s^2]",
            anchor="w",
            font=("Courier", 15),
            fill=self.TEXT,
        )
        c.create_text(
            x + 48,
            y + 404,
            text=f"topic: {self.node.imu_topic}    status: {'LIVE' if fresh else 'STALE / no data'}",
            anchor="w",
            font=("Helvetica", 13),
            fill=self.GOOD if fresh else self.BAD,
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ImuAxisCheckNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        ImuAxisWindow(node).run()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
