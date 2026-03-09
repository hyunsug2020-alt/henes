#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import time
from collections import deque
from typing import Dict, List, Tuple

import rclpy
from geometry_msgs.msg import Accel
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String
from tkinter import Tk, StringVar
from tkinter import scrolledtext, ttk

from bisa.msg import MPCPerformance


def _fmt_float(v: float, digits: int = 3) -> str:
    if not math.isfinite(v):
        return "-"
    return f"{v:.{digits}f}"


def _parse_gate_status(data: str) -> Dict[str, str]:
    parsed: Dict[str, str] = {}
    for token in data.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        parsed[key] = value
    return parsed


class RuntimeLogMonitor(Node):
    def __init__(self) -> None:
        super().__init__("cav_runtime_log_gui")

        self.declare_parameter("cav_ids", [1, 2, 3, 4])
        cav_ids_param = self.get_parameter("cav_ids").get_parameter_value().integer_array_value
        self.cav_ids: List[int] = [int(v) for v in cav_ids_param] if cav_ids_param else [1, 2, 3, 4]

        self._lock = threading.Lock()
        self._events: deque[str] = deque(maxlen=500)

        self.offline_status: str = "N/A"
        self.offline_status_time: float = 0.0

        self.state: Dict[int, Dict[str, object]] = {}
        for cav_id in self.cav_ids:
            self.state[cav_id] = {
                "mode": "N/A",
                "block": -1,
                "raw_v": float("nan"),
                "sim_v": float("nan"),
                "cap": float("nan"),
                "flags": "",
                "solver": "N/A",
                "updated": 0.0,
                "raw_zero_pass_latched": False,
                "sim_zero_pass_latched": False,
            }

        qos_status = QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE)
        qos_latched = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        qos_cmd = QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(String, "/offline_scheduler/status", self._offline_status_cb, qos_latched)

        self._subs = []
        for cav_id in self.cav_ids:
            id_str = f"{cav_id:02d}"
            gate_topic = f"/cav{id_str}/priority_gate_status"
            raw_topic = f"/cav{id_str}/accel_raw"
            out_topic = f"/sim/cav{id_str}/accel"
            cap_topic = f"/cav{id_str}/offline_speed_cap"
            perf_topic = f"/cav{id_str}/mpc_performance"

            self._subs.append(
                self.create_subscription(
                    String, gate_topic, lambda msg, cid=cav_id: self._gate_cb(cid, msg), qos_status
                )
            )
            self._subs.append(
                self.create_subscription(
                    Accel, raw_topic, lambda msg, cid=cav_id: self._raw_cb(cid, msg), qos_cmd
                )
            )
            self._subs.append(
                self.create_subscription(
                    Accel, out_topic, lambda msg, cid=cav_id: self._sim_cb(cid, msg), qos_cmd
                )
            )
            self._subs.append(
                self.create_subscription(
                    Float64, cap_topic, lambda msg, cid=cav_id: self._cap_cb(cid, msg), qos_status
                )
            )
            self._subs.append(
                self.create_subscription(
                    MPCPerformance, perf_topic, lambda msg, cid=cav_id: self._perf_cb(cid, msg), qos_status
                )
            )

            self.get_logger().info(
                f"CAV{cav_id:02d} topics: gate={gate_topic}, raw={raw_topic}, sim={out_topic}, "
                f"cap={cap_topic}, perf={perf_topic}"
            )

    def _push_event(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self._events.append(f"[{stamp}] {text}")

    def _offline_status_cb(self, msg: String) -> None:
        now = time.monotonic()
        with self._lock:
            if msg.data != self.offline_status:
                self._push_event(f"offline_status: {msg.data}")
            self.offline_status = msg.data
            self.offline_status_time = now

    def _gate_cb(self, cav_id: int, msg: String) -> None:
        kv = _parse_gate_status(msg.data)
        now = time.monotonic()
        with self._lock:
            s = self.state[cav_id]
            prev_mode = str(s["mode"])
            mode = kv.get("mode", prev_mode)
            block_text = kv.get("block", "-1")
            try:
                block = int(block_text)
            except Exception:
                block = -1

            s["mode"] = mode
            s["block"] = block
            flags = []
            if kv.get("geo", "0") == "1":
                flags.append("geo")
            if kv.get("res", "0") == "1":
                flags.append("res")
            if kv.get("occ", "0") == "1":
                flags.append("occ")
            if kv.get("near", "0") == "1":
                flags.append("near")
            if kv.get("imm", "0") == "1":
                flags.append("imm")
            if kv.get("ems", "0") == "1":
                flags.append("ems")
            if kv.get("tie", "0") == "1":
                flags.append("tie")
            if kv.get("rear_slow", "0") == "1":
                flags.append("rear_slow")
            if kv.get("rear_stop", "0") == "1":
                flags.append("rear_stop")
            if kv.get("offline_hold", "0") == "1":
                flags.append("offhold")
            s["flags"] = "|".join(flags) if flags else "-"
            s["updated"] = now

            if mode != prev_mode:
                self._push_event(
                    f"CAV{cav_id:02d} mode: {prev_mode} -> {mode} (block={block}, flags={s['flags']})"
                )

            self._check_pass_zero_locked(cav_id, s)

    def _raw_cb(self, cav_id: int, msg: Accel) -> None:
        now = time.monotonic()
        v = max(0.0, float(msg.linear.x))
        with self._lock:
            s = self.state[cav_id]
            s["raw_v"] = v
            s["updated"] = now
            self._check_pass_zero_locked(cav_id, s)

    def _sim_cb(self, cav_id: int, msg: Accel) -> None:
        now = time.monotonic()
        v = max(0.0, float(msg.linear.x))
        with self._lock:
            s = self.state[cav_id]
            s["sim_v"] = v
            s["updated"] = now
            self._check_pass_zero_locked(cav_id, s)

    def _cap_cb(self, cav_id: int, msg: Float64) -> None:
        now = time.monotonic()
        cap = max(0.0, float(msg.data))
        with self._lock:
            s = self.state[cav_id]
            prev_cap = float(s["cap"]) if isinstance(s["cap"], float) else float("nan")
            s["cap"] = cap
            s["updated"] = now
            if cap <= 1e-3 and (not math.isfinite(prev_cap) or prev_cap > 1e-3):
                self._push_event(f"CAV{cav_id:02d} offline_cap -> 0.000")

    def _perf_cb(self, cav_id: int, msg: MPCPerformance) -> None:
        now = time.monotonic()
        solver = msg.solver_status
        with self._lock:
            s = self.state[cav_id]
            prev_solver = str(s["solver"])
            s["solver"] = solver
            s["updated"] = now
            if solver != prev_solver:
                self._push_event(f"CAV{cav_id:02d} solver: {prev_solver} -> {solver}")

    def _check_pass_zero_locked(self, cav_id: int, s: Dict[str, object]) -> None:
        mode = str(s["mode"])
        raw_v = float(s["raw_v"]) if isinstance(s["raw_v"], float) else float("nan")
        sim_v = float(s["sim_v"]) if isinstance(s["sim_v"], float) else float("nan")

        if mode != "PASS":
            s["raw_zero_pass_latched"] = False
            s["sim_zero_pass_latched"] = False
            return

        raw_zero = math.isfinite(raw_v) and raw_v < 0.02
        raw_resume = math.isfinite(raw_v) and raw_v > 0.08
        if raw_zero and not bool(s["raw_zero_pass_latched"]):
            self._push_event(f"CAV{cav_id:02d} raw_v≈0 while mode=PASS")
            s["raw_zero_pass_latched"] = True
        elif raw_resume:
            s["raw_zero_pass_latched"] = False

        sim_zero = math.isfinite(sim_v) and sim_v < 0.02
        sim_resume = math.isfinite(sim_v) and sim_v > 0.08
        if sim_zero and not bool(s["sim_zero_pass_latched"]):
            self._push_event(f"CAV{cav_id:02d} sim_v≈0 while mode=PASS")
            s["sim_zero_pass_latched"] = True
        elif sim_resume:
            s["sim_zero_pass_latched"] = False

    def snapshot(self) -> Tuple[str, List[Tuple[int, str, int, float, float, float, str, str, float]], List[str]]:
        with self._lock:
            now = time.monotonic()
            offline_age = (now - self.offline_status_time) if self.offline_status_time > 0.0 else float("nan")
            offline_line = f"offline_status: {self.offline_status} (age={_fmt_float(offline_age, 1)}s)"

            rows: List[Tuple[int, str, int, float, float, float, str, str, float]] = []
            for cav_id in self.cav_ids:
                s = self.state[cav_id]
                updated = float(s["updated"]) if isinstance(s["updated"], float) else 0.0
                age = (now - updated) if updated > 0.0 else float("nan")
                rows.append(
                    (
                        cav_id,
                        str(s["mode"]),
                        int(s["block"]),
                        float(s["raw_v"]),
                        float(s["sim_v"]),
                        float(s["cap"]),
                        str(s["flags"]),
                        str(s["solver"]),
                        age,
                    )
                )

            events = list(self._events)
            self._events.clear()
            return offline_line, rows, events


class RuntimeLogWindow:
    def __init__(self, monitor: RuntimeLogMonitor):
        self.monitor = monitor

        self.root = Tk()
        self.root.title("CAV Runtime Log Monitor")
        self.root.geometry("1100x760")

        self.status_var = StringVar(value="offline_status: N/A")
        status_label = ttk.Label(self.root, textvariable=self.status_var, font=("TkDefaultFont", 11))
        status_label.pack(fill="x", padx=8, pady=(8, 4))

        columns = ("cav", "mode", "block", "raw_v", "sim_v", "cap", "flags", "solver", "age_s")
        self.table = ttk.Treeview(self.root, columns=columns, show="headings", height=8)
        for col in columns:
            self.table.heading(col, text=col)
        self.table.column("cav", width=70, anchor="center")
        self.table.column("mode", width=90, anchor="center")
        self.table.column("block", width=90, anchor="center")
        self.table.column("raw_v", width=120, anchor="center")
        self.table.column("sim_v", width=120, anchor="center")
        self.table.column("cap", width=120, anchor="center")
        self.table.column("flags", width=150, anchor="center")
        self.table.column("solver", width=180, anchor="center")
        self.table.column("age_s", width=100, anchor="center")
        self.table.pack(fill="x", padx=8, pady=4)

        for cav_id in self.monitor.cav_ids:
            self.table.insert(
                "",
                "end",
                iid=f"cav{cav_id:02d}",
                values=(cav_id, "-", -1, "-", "-", "-", "-", "-", "-"),
            )

        self.log = scrolledtext.ScrolledText(self.root, wrap="word", height=24, font=("TkFixedFont", 10))
        self.log.pack(fill="both", expand=True, padx=8, pady=(4, 8))
        self.log.configure(state="disabled")

        self.root.after(200, self._refresh)

    def _append_log_lines(self, lines: List[str]) -> None:
        if not lines:
            return
        self.log.configure(state="normal")
        for line in lines:
            self.log.insert("end", line + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _refresh(self) -> None:
        offline_line, rows, events = self.monitor.snapshot()
        self.status_var.set(offline_line)

        for cav_id, mode, block, raw_v, sim_v, cap, flags, solver, age in rows:
            self.table.item(
                f"cav{cav_id:02d}",
                values=(
                    cav_id,
                    mode,
                    block,
                    _fmt_float(raw_v, 3),
                    _fmt_float(sim_v, 3),
                    _fmt_float(cap, 3),
                    flags,
                    solver,
                    _fmt_float(age, 1),
                ),
            )

        self._append_log_lines(events)
        self.root.after(200, self._refresh)

    def run(self) -> None:
        self.root.mainloop()


def main(args=None) -> None:
    rclpy.init(args=args)
    monitor = RuntimeLogMonitor()

    spin_thread = threading.Thread(target=rclpy.spin, args=(monitor,), daemon=True)
    spin_thread.start()

    try:
        window = RuntimeLogWindow(monitor)
        window.run()
    finally:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
