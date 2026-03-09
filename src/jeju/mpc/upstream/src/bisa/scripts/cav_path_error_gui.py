#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


@dataclass
class CAVState:
    cav_id: int
    path_points_xy: Optional[np.ndarray] = None
    seg_start_xy: Optional[np.ndarray] = None
    seg_vec_xy: Optional[np.ndarray] = None
    seg_len_sq: Optional[np.ndarray] = None
    prev_pose_xy: Optional[np.ndarray] = None
    latest_pose_xy: Optional[np.ndarray] = None
    latest_error_m: Optional[float] = None
    latest_error_pct: Optional[float] = None
    trajectory_xy: List[Tuple[float, float]] = field(default_factory=list)
    overshoot_xy: List[Tuple[float, float]] = field(default_factory=list)
    error_pct_history: List[Tuple[float, float]] = field(default_factory=list)
    error_m_history: List[Tuple[float, float]] = field(default_factory=list)
    bucket_error_sum: float = 0.0
    bucket_samples: int = 0
    bucket_travel_m: float = 0.0
    gate_mode: str = "N/A"
    gate_block: int = -1
    gate_hit: int = -1
    gate_res: bool = False
    gate_occ: bool = False
    gate_near: bool = False
    gate_geo: bool = False
    gate_near_block: int = -1
    gate_imm: bool = False
    gate_imm_block: int = -1
    gate_imm_hit: int = -1
    gate_ems: bool = False
    gate_ems_block: int = -1
    gate_ems_d: float = -1.0
    gate_tie: bool = False
    gate_tie_block: int = -1
    gate_tie_d: float = -1.0
    gate_rear_slow: bool = False
    gate_rear_stop: bool = False
    gate_rear_block: int = -1
    gate_rear_d: float = -1.0
    gate_ego: bool = False
    gate_my_path_n: int = 0
    gate_other_pose_n: int = 0
    gate_other_path_n: int = 0
    gate_updated_time: float = 0.0
    path_ready: bool = False
    pose_ready: bool = False


class CAVPathErrorMonitor(Node):
    def __init__(self):
        super().__init__("cav_path_error_gui")

        self.declare_parameter("cav_ids", [1, 2, 3, 4])
        self.declare_parameter("focus_cav_id", 1)
        self.declare_parameter("global_path_topic_template", "/cav{cav_id:02d}/global_path")
        self.declare_parameter("pose_topic_template", "/CAV_{cav_id:02d}")
        self.declare_parameter("gate_status_topic_template", "/cav{cav_id:02d}/priority_gate_status")
        self.declare_parameter("update_period_sec", 0.5)
        self.declare_parameter("gui_frame_ms", 30)
        self.declare_parameter("min_travel_for_percent_m", 0.05)
        self.declare_parameter("max_display_percent", 300.0)
        self.declare_parameter("trajectory_max_points", 5000)
        self.declare_parameter("error_history_sec", 45.0)
        self.declare_parameter("overshoot_error_m_threshold", 0.08)
        self.declare_parameter("overshoot_max_points", 1500)

        cav_ids_param = self.get_parameter("cav_ids").get_parameter_value().integer_array_value
        cfg_cav_ids: List[int] = [int(v) for v in cav_ids_param] if cav_ids_param else [1, 2, 3, 4]
        focus_cav_id = int(self.get_parameter("focus_cav_id").value)
        self.focus_cav_id = focus_cav_id if focus_cav_id > 0 else 1
        self.cav_ids: List[int] = [self.focus_cav_id]
        self.path_topic_tmpl = self.get_parameter("global_path_topic_template").value
        self.pose_topic_tmpl = self.get_parameter("pose_topic_template").value
        self.gate_topic_tmpl = self.get_parameter("gate_status_topic_template").value
        self.update_period_sec = max(0.1, float(self.get_parameter("update_period_sec").value))
        self.gui_frame_ms = max(10, int(self.get_parameter("gui_frame_ms").value))
        self.min_travel_for_percent_m = max(
            0.01, float(self.get_parameter("min_travel_for_percent_m").value)
        )
        self.max_display_percent = max(20.0, float(self.get_parameter("max_display_percent").value))
        self.trajectory_max_points = max(500, int(self.get_parameter("trajectory_max_points").value))
        self.error_history_sec = max(5.0, float(self.get_parameter("error_history_sec").value))
        self.overshoot_error_m_threshold = max(
            0.01, float(self.get_parameter("overshoot_error_m_threshold").value)
        )
        self.overshoot_max_points = max(100, int(self.get_parameter("overshoot_max_points").value))

        self.start_time = time.monotonic()
        self._lock = threading.Lock()

        self.states: Dict[int, CAVState] = {cav_id: CAVState(cav_id=cav_id) for cav_id in self.cav_ids}

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        pose_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        gate_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)

        self._path_subs = []
        self._pose_subs = []
        self._gate_subs = []

        for cav_id in self.cav_ids:
            path_topic = self._format_topic(self.path_topic_tmpl, cav_id)
            pose_topic = self._format_topic(self.pose_topic_tmpl, cav_id)
            gate_topic = self._format_topic(self.gate_topic_tmpl, cav_id)

            self._path_subs.append(
                self.create_subscription(
                    Path,
                    path_topic,
                    lambda msg, cid=cav_id: self._path_callback(cid, msg),
                    path_qos,
                )
            )
            self._pose_subs.append(
                self.create_subscription(
                    PoseStamped,
                    pose_topic,
                    lambda msg, cid=cav_id: self._pose_callback(cid, msg),
                    pose_qos,
                )
            )
            self._gate_subs.append(
                self.create_subscription(
                    String,
                    gate_topic,
                    lambda msg, cid=cav_id: self._gate_callback(cid, msg),
                    gate_qos,
                )
            )

            self.get_logger().info(
                f"[CAV{cav_id:02d}] subscribe global_path={path_topic}, pose={pose_topic}, gate={gate_topic}"
            )

        self._timer = self.create_timer(self.update_period_sec, self._on_timer)
        self.get_logger().info(
            "Path error GUI ready: focus_cav=%d (cfg=%s), update_period=%.2fs, "
            "min_travel_for_percent=%.2fm, overshoot_threshold=%.3fm"
            % (
                self.focus_cav_id,
                cfg_cav_ids,
                self.update_period_sec,
                self.min_travel_for_percent_m,
                self.overshoot_error_m_threshold,
            )
        )

    @staticmethod
    def _format_topic(template: str, cav_id: int) -> str:
        try:
            return template.format(cav_id=cav_id)
        except Exception:
            return template

    def _path_callback(self, cav_id: int, msg: Path) -> None:
        pts = np.array(
            [[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses],
            dtype=np.float64,
        )

        with self._lock:
            state = self.states[cav_id]

            if pts.shape[0] < 2:
                state.path_points_xy = None
                state.seg_start_xy = None
                state.seg_vec_xy = None
                state.seg_len_sq = None
                state.path_ready = False
                return

            seg_start = pts[:-1]
            seg_vec = pts[1:] - pts[:-1]
            seg_len_sq = np.einsum("ij,ij->i", seg_vec, seg_vec)
            valid = seg_len_sq > 1e-12

            if not np.any(valid):
                state.path_points_xy = pts
                state.seg_start_xy = None
                state.seg_vec_xy = None
                state.seg_len_sq = None
                state.path_ready = False
                return

            state.path_points_xy = pts
            state.seg_start_xy = seg_start[valid]
            state.seg_vec_xy = seg_vec[valid]
            state.seg_len_sq = seg_len_sq[valid]
            state.path_ready = True

    def _pose_callback(self, cav_id: int, msg: PoseStamped) -> None:
        pose_xy = np.array([msg.pose.position.x, msg.pose.position.y], dtype=np.float64)
        with self._lock:
            state = self.states[cav_id]
            err_m_now: Optional[float] = None

            if state.prev_pose_xy is not None and state.path_ready:
                ds = float(np.linalg.norm(pose_xy - state.prev_pose_xy))
                if np.isfinite(ds) and ds >= 0.0:
                    state.bucket_travel_m += ds

            if state.path_ready:
                err_m = self._point_to_polyline_dist(pose_xy, state)
                if err_m is not None and np.isfinite(err_m):
                    err_m_now = float(err_m)
                    state.bucket_error_sum += float(err_m)
                    state.bucket_samples += 1

            state.prev_pose_xy = pose_xy
            state.latest_pose_xy = pose_xy
            state.pose_ready = True

            state.trajectory_xy.append((float(pose_xy[0]), float(pose_xy[1])))
            if len(state.trajectory_xy) > self.trajectory_max_points:
                del state.trajectory_xy[: len(state.trajectory_xy) - self.trajectory_max_points]

            if err_m_now is not None and err_m_now >= self.overshoot_error_m_threshold:
                state.overshoot_xy.append((float(pose_xy[0]), float(pose_xy[1])))
                if len(state.overshoot_xy) > self.overshoot_max_points:
                    del state.overshoot_xy[: len(state.overshoot_xy) - self.overshoot_max_points]

    def _gate_callback(self, cav_id: int, msg: String) -> None:
        kv = {}
        for token in msg.data.strip().split():
            if "=" not in token:
                continue
            k, v = token.split("=", 1)
            kv[k] = v

        with self._lock:
            state = self.states[cav_id]
            state.gate_mode = kv.get("mode", state.gate_mode)
            state.gate_block = int(kv.get("block", state.gate_block))
            state.gate_hit = int(kv.get("hit", state.gate_hit))
            state.gate_res = kv.get("res", "0") == "1"
            state.gate_occ = kv.get("occ", "0") == "1"
            state.gate_near = kv.get("near", "0") == "1"
            state.gate_geo = kv.get("geo", "0") == "1"
            state.gate_near_block = int(kv.get("near_block", state.gate_near_block))
            state.gate_imm = kv.get("imm", "0") == "1"
            state.gate_imm_block = int(kv.get("imm_block", state.gate_imm_block))
            state.gate_imm_hit = int(kv.get("imm_hit", state.gate_imm_hit))
            state.gate_ems = kv.get("ems", "0") == "1"
            state.gate_ems_block = int(kv.get("ems_block", state.gate_ems_block))
            state.gate_ems_d = float(kv.get("ems_d", state.gate_ems_d))
            state.gate_tie = kv.get("tie", "0") == "1"
            state.gate_tie_block = int(kv.get("tie_block", state.gate_tie_block))
            state.gate_tie_d = float(kv.get("tie_d", state.gate_tie_d))
            state.gate_rear_slow = kv.get("rear_slow", "0") == "1"
            state.gate_rear_stop = kv.get("rear_stop", "0") == "1"
            state.gate_rear_block = int(kv.get("rear_block", state.gate_rear_block))
            state.gate_rear_d = float(kv.get("rear_d", state.gate_rear_d))
            state.gate_ego = kv.get("ego", "0") == "1"
            state.gate_my_path_n = int(kv.get("my_path_n", state.gate_my_path_n))
            state.gate_other_pose_n = int(kv.get("other_pose_n", state.gate_other_pose_n))
            state.gate_other_path_n = int(kv.get("other_path_n", state.gate_other_path_n))
            state.gate_updated_time = time.monotonic()

    def _point_to_polyline_dist(self, point_xy: np.ndarray, state: CAVState) -> Optional[float]:
        if (
            state.seg_start_xy is None
            or state.seg_vec_xy is None
            or state.seg_len_sq is None
            or state.seg_start_xy.shape[0] == 0
        ):
            if state.path_points_xy is None or state.path_points_xy.shape[0] == 0:
                return None
            d = np.linalg.norm(state.path_points_xy - point_xy[None, :], axis=1)
            return float(np.min(d))

        diff = point_xy[None, :] - state.seg_start_xy
        t = np.einsum("ij,ij->i", diff, state.seg_vec_xy) / state.seg_len_sq
        t = np.clip(t, 0.0, 1.0)
        proj = state.seg_start_xy + state.seg_vec_xy * t[:, None]
        d = np.linalg.norm(proj - point_xy[None, :], axis=1)
        return float(np.min(d))

    def _on_timer(self) -> None:
        with self._lock:
            now_elapsed = time.monotonic() - self.start_time
            for cav_id, state in self.states.items():
                if state.latest_pose_xy is None or not state.path_ready:
                    state.latest_error_m = None
                    state.latest_error_pct = None
                    state.bucket_error_sum = 0.0
                    state.bucket_samples = 0
                    state.bucket_travel_m = 0.0
                    continue

                if state.bucket_samples <= 0:
                    state.latest_error_m = None
                    state.latest_error_pct = None
                    state.bucket_error_sum = 0.0
                    state.bucket_samples = 0
                    state.bucket_travel_m = 0.0
                    continue

                err_m = state.bucket_error_sum / float(state.bucket_samples)
                travel_m = max(state.bucket_travel_m, self.min_travel_for_percent_m)
                err_pct = (err_m / travel_m) * 100.0
                err_pct = max(0.0, min(err_pct, self.max_display_percent))

                state.latest_error_m = float(err_m)
                state.latest_error_pct = float(err_pct)
                state.error_m_history.append((now_elapsed, state.latest_error_m))
                state.error_pct_history.append((now_elapsed, state.latest_error_pct))

                cutoff = now_elapsed - self.error_history_sec
                while state.error_m_history and state.error_m_history[0][0] < cutoff:
                    state.error_m_history.pop(0)
                while state.error_pct_history and state.error_pct_history[0][0] < cutoff:
                    state.error_pct_history.pop(0)

                state.bucket_error_sum = 0.0
                state.bucket_samples = 0
                state.bucket_travel_m = 0.0

    def snapshot(self) -> Tuple[Dict[int, dict], float]:
        with self._lock:
            latest = {}
            for cav_id, state in self.states.items():
                latest[cav_id] = {
                    "error_m": state.latest_error_m,
                    "error_pct": state.latest_error_pct,
                    "path_ready": state.path_ready,
                    "pose_ready": state.pose_ready,
                    "path_xy": None if state.path_points_xy is None else state.path_points_xy.copy(),
                    "traj_xy": np.array(state.trajectory_xy, dtype=np.float64)
                    if state.trajectory_xy else np.empty((0, 2), dtype=np.float64),
                    "overshoot_xy": np.array(state.overshoot_xy, dtype=np.float64)
                    if state.overshoot_xy else np.empty((0, 2), dtype=np.float64),
                    "error_pct_hist": np.array(state.error_pct_history, dtype=np.float64)
                    if state.error_pct_history else np.empty((0, 2), dtype=np.float64),
                    "error_m_hist": np.array(state.error_m_history, dtype=np.float64)
                    if state.error_m_history else np.empty((0, 2), dtype=np.float64),
                    "gate_mode": state.gate_mode,
                    "gate_block": state.gate_block,
                    "gate_hit": state.gate_hit,
                    "gate_res": state.gate_res,
                    "gate_occ": state.gate_occ,
                    "gate_near": state.gate_near,
                    "gate_geo": state.gate_geo,
                    "gate_near_block": state.gate_near_block,
                    "gate_imm": state.gate_imm,
                    "gate_imm_block": state.gate_imm_block,
                    "gate_imm_hit": state.gate_imm_hit,
                    "gate_ems": state.gate_ems,
                    "gate_ems_block": state.gate_ems_block,
                    "gate_ems_d": state.gate_ems_d,
                    "gate_tie": state.gate_tie,
                    "gate_tie_block": state.gate_tie_block,
                    "gate_tie_d": state.gate_tie_d,
                    "gate_rear_slow": state.gate_rear_slow,
                    "gate_rear_stop": state.gate_rear_stop,
                    "gate_rear_block": state.gate_rear_block,
                    "gate_rear_d": state.gate_rear_d,
                    "gate_ego": state.gate_ego,
                    "gate_my_path_n": state.gate_my_path_n,
                    "gate_other_pose_n": state.gate_other_pose_n,
                    "gate_other_path_n": state.gate_other_path_n,
                    "gate_age_sec": time.monotonic() - state.gate_updated_time
                    if state.gate_updated_time > 0.0 else 1e9,
                }
            now_elapsed = time.monotonic() - self.start_time
        return latest, now_elapsed


class PathErrorGUI:
    def __init__(self, node: CAVPathErrorMonitor):
        self.node = node
        self.cav_id = node.cav_ids[0] if node.cav_ids else 1
        self.anim_interval_ms = node.gui_frame_ms

        self.fig = plt.figure(figsize=(14, 8), facecolor="#000000")
        gs = self.fig.add_gridspec(2, 1, height_ratios=[4.2, 1.8], hspace=0.15)
        self.ax_map = self.fig.add_subplot(gs[0, 0])
        self.ax_pct = self.fig.add_subplot(gs[1, 0])
        self.fig.suptitle(
            f"CAV {self.cav_id:02d} Overshoot Debug Monitor ({self.anim_interval_ms}ms frame)",
            color="white",
            fontsize=14,
        )

        self._style_axis(self.ax_map)
        self._style_axis(self.ax_pct)

        self.ax_map.set_title("Global Path vs Actual Trajectory", color="white", pad=8)
        self.ax_map.set_xlabel("X [m]")
        self.ax_map.set_ylabel("Y [m]")
        self.ax_map.set_aspect("equal", adjustable="box")

        self.global_line, = self.ax_map.plot(
            [], [], color="#00E5FF", linewidth=2.2, label="global path", alpha=0.95
        )
        self.traj_line, = self.ax_map.plot(
            [], [], color="#FFD740", linewidth=2.0, label="actual path", alpha=0.95
        )
        self.overshoot_scatter = self.ax_map.scatter(
            [], [], s=24, c="#FF5252", marker="x", linewidths=1.3, alpha=0.95, label="overshoot"
        )
        self.current_dot = self.ax_map.scatter(
            [], [], s=70, c="#FFFFFF", edgecolors="#000000", linewidths=0.8, zorder=5, label="current"
        )

        legend = self.ax_map.legend(loc="upper right", facecolor="#111111", edgecolor="#666666")
        for t in legend.get_texts():
            t.set_color("white")

        self.map_text = self.ax_map.text(
            0.01,
            0.99,
            "waiting for path/pose...",
            transform=self.ax_map.transAxes,
            ha="left",
            va="top",
            color="white",
            fontsize=11,
            family="monospace",
        )

        self.ax_pct.set_title("Tracking Error Percent", color="white", pad=6)
        self.ax_pct.set_xlabel("Time [s]")
        self.ax_pct.set_ylabel("Error [%]")
        self.ax_pct.set_xlim(0.0, self.node.error_history_sec)
        self.ax_pct.set_ylim(0.0, min(self.node.max_display_percent, 100.0))

        self.pct_line, = self.ax_pct.plot(
            [], [], color="#FFAB40", linewidth=2.2, alpha=0.98, label="error %"
        )
        self.pct_fill = None
        pct_legend = self.ax_pct.legend(loc="upper left", facecolor="#111111", edgecolor="#666666")
        for t in pct_legend.get_texts():
            t.set_color("white")

        self.pct_value_text = self.ax_pct.text(
            0.02,
            0.82,
            "Error: -- %",
            transform=self.ax_pct.transAxes,
            ha="left",
            va="top",
            color="white",
            fontsize=17,
            weight="bold",
            family="monospace",
        )
        self.pct_aux_text = self.ax_pct.text(
            0.02,
            0.55,
            "",
            transform=self.ax_pct.transAxes,
            ha="left",
            va="top",
            color="#BBBBBB",
            fontsize=11,
            family="monospace",
        )
        self.gate_text = self.ax_pct.text(
            0.98,
            0.82,
            "gate: waiting",
            transform=self.ax_pct.transAxes,
            ha="right",
            va="top",
            color="#9EE37D",
            fontsize=11,
            family="monospace",
        )

        self._anim = FuncAnimation(
            self.fig,
            self._refresh,
            interval=self.anim_interval_ms,
            blit=False,
        )

    @staticmethod
    def _style_axis(ax):
        ax.set_facecolor("#101010")
        for spine in ax.spines.values():
            spine.set_color("#808080")
        ax.tick_params(colors="white")
        ax.xaxis.label.set_color("white")
        ax.yaxis.label.set_color("white")
        ax.grid(True, color="#3A3A3A", alpha=0.40)

    @staticmethod
    def _gate_line(info: dict) -> str:
        gate_mode = str(info.get("gate_mode", "N/A"))
        gate_age = float(info.get("gate_age_sec", 1e9))
        if gate_age > 2.0:
            return "gate: stale"
        if gate_mode != "STOP":
            return f"gate: {gate_mode}"

        reasons: List[str] = []
        if info.get("gate_geo", False):
            reasons.append("geo")
        if info.get("gate_res", False):
            reasons.append("res")
        if info.get("gate_occ", False):
            reasons.append("occ")
        if info.get("gate_near", False):
            reasons.append(f"near{int(info.get('gate_near_block', -1))}")
        if info.get("gate_imm", False):
            reasons.append(
                f"imm{int(info.get('gate_imm_block', -1))}@{int(info.get('gate_imm_hit', -1))}"
            )
        if info.get("gate_tie", False):
            reasons.append(
                f"tie{int(info.get('gate_tie_block', -1))}@{float(info.get('gate_tie_d', -1.0)):.2f}m"
            )
        if info.get("gate_ems", False):
            reasons.append(
                f"ems{int(info.get('gate_ems_block', -1))}@{float(info.get('gate_ems_d', -1.0)):.2f}m"
            )
        if info.get("gate_rear_slow", False):
            reasons.append(
                f"rearS{int(info.get('gate_rear_block', -1))}@{float(info.get('gate_rear_d', -1.0)):.2f}m"
            )
        if info.get("gate_rear_stop", False):
            reasons.append(
                f"rearT{int(info.get('gate_rear_block', -1))}@{float(info.get('gate_rear_d', -1.0)):.2f}m"
            )
        if not reasons:
            reasons.append("unk")

        block = int(info.get("gate_block", -1))
        hit = int(info.get("gate_hit", -1))
        return f"gate: STOP b{block} h{hit} {'/'.join(reasons)}"

    def _fit_map_limits(self, path_xy: np.ndarray, traj_xy: np.ndarray) -> None:
        chunks = []
        if isinstance(path_xy, np.ndarray) and path_xy.size > 0:
            chunks.append(path_xy)
        if isinstance(traj_xy, np.ndarray) and traj_xy.size > 0:
            chunks.append(traj_xy)
        if not chunks:
            return

        pts = np.vstack(chunks)
        xmin = float(np.min(pts[:, 0]))
        xmax = float(np.max(pts[:, 0]))
        ymin = float(np.min(pts[:, 1]))
        ymax = float(np.max(pts[:, 1]))
        w = max(1.0, xmax - xmin)
        h = max(1.0, ymax - ymin)
        half = 0.5 * max(w, h) + 0.35
        cx = 0.5 * (xmin + xmax)
        cy = 0.5 * (ymin + ymax)
        self.ax_map.set_xlim(cx - half, cx + half)
        self.ax_map.set_ylim(cy - half, cy + half)

    def _refresh(self, _frame):
        latest, _ = self.node.snapshot()
        info = latest.get(self.cav_id, {})
        path_xy = info.get("path_xy")
        traj_xy = info.get("traj_xy")
        overshoot_xy = info.get("overshoot_xy")
        pct_hist = info.get("error_pct_hist")
        err_m = info.get("error_m")
        err_pct = info.get("error_pct")
        gate_line = self._gate_line(info)

        artists = []

        if isinstance(path_xy, np.ndarray) and path_xy.shape[0] > 0:
            self.global_line.set_data(path_xy[:, 0], path_xy[:, 1])
        else:
            self.global_line.set_data([], [])

        if isinstance(traj_xy, np.ndarray) and traj_xy.shape[0] > 0:
            self.traj_line.set_data(traj_xy[:, 0], traj_xy[:, 1])
            self.current_dot.set_offsets(traj_xy[-1:, :])
        else:
            self.traj_line.set_data([], [])
            self.current_dot.set_offsets(np.empty((0, 2), dtype=np.float64))

        if isinstance(overshoot_xy, np.ndarray) and overshoot_xy.shape[0] > 0:
            self.overshoot_scatter.set_offsets(overshoot_xy)
        else:
            self.overshoot_scatter.set_offsets(np.empty((0, 2), dtype=np.float64))

        self._fit_map_limits(path_xy, traj_xy)

        if self.pct_fill is not None:
            self.pct_fill.remove()
            self.pct_fill = None

        if isinstance(pct_hist, np.ndarray) and pct_hist.shape[0] > 0:
            t = pct_hist[:, 0]
            y = np.clip(pct_hist[:, 1], 0.0, self.node.max_display_percent)
            self.pct_line.set_data(t, y)
            xmax = float(t[-1]) + 0.5
            xmin = max(0.0, xmax - self.node.error_history_sec)
            self.ax_pct.set_xlim(xmin, max(xmin + 0.5, xmax))
            y_top = max(20.0, min(self.node.max_display_percent, float(np.max(y)) * 1.25))
            self.ax_pct.set_ylim(0.0, y_top)
            self.pct_fill = self.ax_pct.fill_between(t, 0.0, y, color="#FFAB40", alpha=0.22)
            artists.append(self.pct_fill)
        else:
            self.pct_line.set_data([], [])
            self.ax_pct.set_xlim(0.0, self.node.error_history_sec)
            self.ax_pct.set_ylim(0.0, min(self.node.max_display_percent, 100.0))

        over = (err_m is not None) and (float(err_m) >= self.node.overshoot_error_m_threshold)
        if err_pct is not None and err_m is not None:
            self.pct_value_text.set_text(f"Error: {float(err_pct):6.2f}%")
            self.pct_value_text.set_color("#FF5252" if over else "white")
            self.pct_aux_text.set_text(
                f"error_m={float(err_m):.3f} m   overshoot_thr={self.node.overshoot_error_m_threshold:.3f} m   "
                f"overshoot_pts={0 if not isinstance(overshoot_xy, np.ndarray) else overshoot_xy.shape[0]}"
            )
        else:
            self.pct_value_text.set_text("Error: -- %")
            self.pct_value_text.set_color("white")
            self.pct_aux_text.set_text(
                f"error_m=-- m   overshoot_thr={self.node.overshoot_error_m_threshold:.3f} m   "
                f"overshoot_pts={0 if not isinstance(overshoot_xy, np.ndarray) else overshoot_xy.shape[0]}"
            )

        path_n = 0 if not isinstance(path_xy, np.ndarray) else int(path_xy.shape[0])
        traj_n = 0 if not isinstance(traj_xy, np.ndarray) else int(traj_xy.shape[0])
        self.map_text.set_text(
            f"path_pts={path_n}  traj_pts={traj_n}\n"
            f"focus=CAV{self.cav_id:02d}  overshoot={'ON' if over else 'OFF'}"
        )
        self.gate_text.set_text(gate_line)

        artists.extend(
            [
                self.global_line,
                self.traj_line,
                self.overshoot_scatter,
                self.current_dot,
                self.map_text,
                self.pct_line,
                self.pct_value_text,
                self.pct_aux_text,
                self.gate_text,
            ]
        )
        return artists


def main(args=None):
    rclpy.init(args=args)
    node = CAVPathErrorMonitor()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    _gui = PathErrorGUI(node)

    try:
        plt.show()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
