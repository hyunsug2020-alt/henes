#!/usr/bin/env python3
"""
freeze_analyzer.py
------------------
MPC freeze event analyzer.
Loads a screen-recorded video, tracks the vehicle (yellow box),
measures deviation from the red reference path, estimates velocity,
and generates a multi-panel diagnostic plot.

Usage:
    python3 scripts/freeze_analyzer.py [video_path]

If video_path is omitted, the most recent file in ~/Videos/ is used.
"""

import argparse
import glob
import os
import sys

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
import numpy as np

# ── pixel → mm scale (vehicle ~0.3m wide, ~22px → 1px ≈ 1.4cm) ───────────
PX_TO_MM = 14.0  # mm per pixel


# ── detection helpers ──────────────────────────────────────────────────────

def detect_vehicle(hsv, prev_center=None):
    """Return (cx, cy) of yellow vehicle blob, or None."""
    lower = np.array([18, 100, 100])
    upper = np.array([35, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, np.ones((2, 2), np.uint8))

    num, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
    best, best_score = None, 0
    for i in range(1, num):
        x, y, w, h, area = stats[i]
        # skip known static panels
        if 30 < x < 70 and 270 < y < 330:
            continue
        aspect = max(w, h) / max(1, min(w, h))
        if aspect > 5 or area < 80 or area > 700:
            continue
        sol = area / (w * h) if w * h > 0 else 0
        if sol < 0.30:
            continue
        penalty = 0.0
        if prev_center is not None:
            dx = cents[i][0] - prev_center[0]
            dy = cents[i][1] - prev_center[1]
            penalty = np.sqrt(dx * dx + dy * dy) * 0.3
        score = area * sol - penalty
        if score > best_score:
            best_score = score
            best = (float(cents[i][0]), float(cents[i][1]))
    return best


def get_red_path_mask(hsv):
    """Return binary mask of the red reference path (dots removed)."""
    m1 = cv2.inRange(hsv, np.array([0,  120, 80]), np.array([8,  255, 255]))
    m2 = cv2.inRange(hsv, np.array([172, 120, 80]), np.array([180, 255, 255]))
    mask = cv2.bitwise_or(m1, m2)
    k = np.ones((5, 5), np.uint8)
    dots_expanded = cv2.dilate(cv2.erode(mask, k), k)
    return cv2.bitwise_and(mask, cv2.bitwise_not(dots_expanded))


def path_deviation(veh_center, path_mask):
    """Euclidean distance (px) from vehicle centre to nearest path pixel."""
    pts = np.column_stack(np.where(path_mask > 0))  # [row, col]
    if len(pts) < 3:
        return None
    dists = np.sqrt(
        (pts[:, 1] - veh_center[0]) ** 2 +
        (pts[:, 0] - veh_center[1]) ** 2
    )
    return float(np.min(dists))


# ── main analysis ──────────────────────────────────────────────────────────

def analyze_video(video_path: str, fps_extract: float = 10.0):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        sys.exit(f"Cannot open video: {video_path}")

    native_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    step = max(1, round(native_fps / fps_extract))

    times, xs, ys, devs, red_pixels = [], [], [], [], []
    frame_idx = 0
    prev_center = None
    first_frame_bgr = None

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if frame_idx % step == 0:
            t = frame_idx / native_fps
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            vc = detect_vehicle(hsv, prev_center)
            if vc is not None:
                prev_center = vc

            red_mask = get_red_path_mask(hsv)
            rp = int(np.sum(red_mask > 0))

            dev = None
            if vc is not None:
                dev = path_deviation(vc, red_mask)

            times.append(t)
            xs.append(vc[0] if vc else None)
            ys.append(vc[1] if vc else None)
            devs.append(dev)
            red_pixels.append(rp)

            if first_frame_bgr is None:
                first_frame_bgr = frame.copy()

        frame_idx += 1

    cap.release()

    times = np.array(times)
    xs_arr = np.array([v if v is not None else np.nan for v in xs])
    ys_arr = np.array([v if v is not None else np.nan for v in ys])
    devs_arr = np.array([v if v is not None else np.nan for v in devs])
    red_arr = np.array(red_pixels, dtype=float)

    return times, xs_arr, ys_arr, devs_arr, red_arr, first_frame_bgr


def detect_freeze(xs, ys, times, move_thresh_px=0.5, min_freeze_s=0.5):
    """
    Returns (freeze_start_t, freeze_pos) or (None, None).
    A freeze is defined as the vehicle not moving > move_thresh_px
    for at least min_freeze_s seconds.
    """
    vx = np.diff(xs)
    vy = np.diff(ys)
    speed = np.sqrt(vx**2 + vy**2)  # px per sample

    # sliding window
    dt = np.mean(np.diff(times))
    min_frames = max(1, int(min_freeze_s / dt))

    for i in range(len(speed) - min_frames + 1):
        window = speed[i:i + min_frames]
        if np.nanmax(window) <= move_thresh_px:
            return float(times[i + 1]), (float(xs[i + 1]), float(ys[i + 1]))
    return None, None


def classify_section(t_arr, x_arr, y_arr):
    """
    Label each timestep as 'straight' or 'corner' based on
    the angular rate of the heading direction.
    """
    vx = np.gradient(x_arr)
    vy = np.gradient(y_arr)
    heading = np.arctan2(vy, vx)
    omega = np.abs(np.gradient(np.unwrap(heading)))
    labels = np.where(omega > 0.08, "corner", "straight")
    return labels, omega


# ── plotting ───────────────────────────────────────────────────────────────

def make_plot(times, xs, ys, devs, red_pixels, first_frame,
              freeze_t, freeze_pos, video_path, out_path):

    labels, omega = classify_section(times, xs, ys)

    # estimated speed (px/s)
    speed_px = np.abs(np.gradient(xs, times)) + np.abs(np.gradient(ys, times))
    speed_mm_s = speed_px * PX_TO_MM

    fig = plt.figure(figsize=(16, 12))
    fig.patch.set_facecolor("#1a1a2e")
    gs = gridspec.GridSpec(3, 2, figure=fig,
                           left=0.07, right=0.97,
                           top=0.91, bottom=0.06,
                           hspace=0.45, wspace=0.35)

    title = (f"MPC Freeze Event Analyzer\n"
             f"{os.path.basename(video_path)}  |  "
             f"freeze detected @ t={freeze_t:.2f}s" if freeze_t else
             f"MPC Freeze Event Analyzer\n{os.path.basename(video_path)}  |  no freeze detected")
    fig.suptitle(title, color="white", fontsize=13, fontweight="bold")

    AXBG   = "#0d1117"
    GRID   = "#2d3748"
    C_DEV  = "#f97316"   # orange
    C_SPD  = "#38bdf8"   # blue
    C_RED  = "#ef4444"   # red
    C_OMEG = "#a78bfa"   # purple
    C_FREEZE = "#facc15" # yellow

    def style_ax(ax, ylabel="", xlabel="Time (s)"):
        ax.set_facecolor(AXBG)
        ax.tick_params(colors="white")
        ax.yaxis.label.set_color("white")
        ax.xaxis.label.set_color("white")
        ax.title.set_color("white")
        for sp in ax.spines.values():
            sp.set_edgecolor(GRID)
        ax.grid(True, color=GRID, linewidth=0.5, alpha=0.7)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.set_xlabel(xlabel, fontsize=9)

    def vline(ax):
        if freeze_t is not None:
            ax.axvline(freeze_t, color=C_FREEZE, lw=1.5, ls="--", alpha=0.9,
                       label=f"freeze t={freeze_t:.2f}s")

    # ── Panel 1: Path deviation ────────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, 0])
    style_ax(ax1, "Deviation (mm)")
    ax1.set_title("Path Deviation from Red Line")
    valid = ~np.isnan(devs)
    ax1.fill_between(times[valid], devs[valid] * PX_TO_MM,
                     alpha=0.25, color=C_DEV)
    ax1.plot(times[valid], devs[valid] * PX_TO_MM,
             color=C_DEV, lw=1.2, label="deviation mm")
    ax1.axhline(120, color="#fb923c", lw=1, ls="--",
                label="lateral_bound=120mm (new)")
    ax1.axhline(250, color="#fbbf24", lw=1, ls=":",
                label="lateral_bound=250mm (prev)")
    vline(ax1)
    ax1.set_ylim(0, max(300, np.nanmax(devs) * PX_TO_MM * 1.2))
    ax1.legend(fontsize=7, facecolor=AXBG, labelcolor="white", loc="upper right")

    # ── Panel 2: Estimated speed ───────────────────────────────────────────
    ax2 = fig.add_subplot(gs[0, 1])
    style_ax(ax2, "Speed (mm/s)")
    ax2.set_title("Estimated Vehicle Speed")
    ax2.fill_between(times, speed_mm_s, alpha=0.25, color=C_SPD)
    ax2.plot(times, speed_mm_s, color=C_SPD, lw=1.2, label="speed mm/s")
    vline(ax2)
    ax2.legend(fontsize=7, facecolor=AXBG, labelcolor="white")

    # ── Panel 3: Angular rate (corner detection) ───────────────────────────
    ax3 = fig.add_subplot(gs[1, 0])
    style_ax(ax3, "Angular rate (rad/sample)")
    ax3.set_title("Heading Angular Rate  (corner vs. straight)")
    ax3.plot(times, omega, color=C_OMEG, lw=1.0, label="angular rate")
    # shade corners
    in_corner = False
    for i, lbl in enumerate(labels):
        if lbl == "corner" and not in_corner:
            c_start = times[i]; in_corner = True
        elif lbl != "corner" and in_corner:
            ax3.axvspan(c_start, times[i], color=C_OMEG, alpha=0.10)
            in_corner = False
    if in_corner:
        ax3.axvspan(c_start, times[-1], color=C_OMEG, alpha=0.10)
    ax3.axhline(0.08, color=C_OMEG, lw=0.8, ls="--", alpha=0.6,
                label="corner threshold")
    vline(ax3)
    ax3.legend(fontsize=7, facecolor=AXBG, labelcolor="white")

    # ── Panel 4: Red path pixel count ─────────────────────────────────────
    ax4 = fig.add_subplot(gs[1, 1])
    style_ax(ax4, "Red path pixels")
    ax4.set_title("Red Path Visibility  (sudden drop = path lost)")
    ax4.fill_between(times, red_pixels, alpha=0.25, color=C_RED)
    ax4.plot(times, red_pixels, color=C_RED, lw=1.2, label="red path px")
    vline(ax4)
    ax4.legend(fontsize=7, facecolor=AXBG, labelcolor="white")

    # ── Panel 5: Vehicle XY trajectory ────────────────────────────────────
    ax5 = fig.add_subplot(gs[2, 0])
    style_ax(ax5, "Y pixel", "X pixel")
    ax5.set_title("Vehicle XY Trajectory  (color = time)")
    if first_frame is not None:
        h, w = first_frame.shape[:2]
        ax5.imshow(cv2.cvtColor(first_frame, cv2.COLOR_BGR2RGB),
                   alpha=0.4, extent=[0, w, h, 0])
    valid = ~np.isnan(xs)
    sc = ax5.scatter(xs[valid], ys[valid],
                     c=times[valid], cmap="plasma",
                     s=8, vmin=times[0], vmax=times[-1], zorder=3)
    plt.colorbar(sc, ax=ax5, label="Time (s)")
    if freeze_t is not None and freeze_pos is not None:
        ax5.scatter(*freeze_pos, s=120, c=C_FREEZE,
                    marker="X", zorder=5, label=f"freeze ({freeze_t:.2f}s)")
        ax5.legend(fontsize=7, facecolor=AXBG, labelcolor="white")
    ax5.set_aspect("equal")
    ax5.invert_yaxis()

    # ── Panel 6: Diagnosis text ────────────────────────────────────────────
    ax6 = fig.add_subplot(gs[2, 1])
    ax6.set_facecolor(AXBG)
    ax6.set_xticks([]); ax6.set_yticks([])
    for sp in ax6.spines.values():
        sp.set_edgecolor(GRID)

    if freeze_t is not None:
        pre  = devs[times <= freeze_t]
        post = devs[times >  freeze_t]
        dev_at_freeze = np.nanmean(pre[-3:]) * PX_TO_MM if len(pre) >= 3 else np.nan
        spd_at_freeze = speed_mm_s[np.argmin(np.abs(times - freeze_t))]

        # determine likely cause
        cause_lines = []
        if dev_at_freeze > 120:
            cause_lines.append("lateral_bound(120mm) exceeded")
            cause_lines.append("=> QP infeasible => solver fail => v=0")
        elif dev_at_freeze > 50:
            cause_lines.append("off_path_recovery triggered")
            cause_lines.append("=> speed limited to 0.10 m/s")
        else:
            cause_lines.append("within lateral bounds but stopped")
            cause_lines.append("=> oscillation_guard damping?")
            cause_lines.append("=> or path < 5 points at roundabout")

        diag = [
            "── FREEZE DIAGNOSIS ──",
            f"Freeze time       : {freeze_t:.2f} s",
            f"Freeze position   : ({freeze_pos[0]:.0f}, {freeze_pos[1]:.0f}) px",
            f"Dev at freeze     : {dev_at_freeze:.1f} mm",
            f"Speed at freeze   : {spd_at_freeze:.1f} mm/s",
            f"Heading rate      : {omega[np.argmin(np.abs(times-freeze_t))]:.3f} rad/sample",
            "",
            "── LIKELY CAUSE ──",
        ] + cause_lines + [
            "",
            "── SUGGESTED FIX ──",
            "lateral_bound: 0.12 -> 0.18 m",
            "oscillation_guard_cte_deadband:",
            "  0.02 -> 0.04 m",
            "Q_pos: 125 (keep)",
        ]
    else:
        diag = [
            "── RESULT ──",
            "No freeze event detected.",
            f"Mean deviation : {np.nanmean(devs)*PX_TO_MM:.1f} mm",
            f"Max deviation  : {np.nanmax(devs)*PX_TO_MM:.1f} mm",
            f"Min speed      : {np.nanmin(speed_mm_s):.1f} mm/s",
        ]

    ax6.set_title("Diagnostic Summary", color="white", fontsize=9, fontweight="bold")
    y_pos = 0.95
    for line in diag:
        color = C_FREEZE if "──" in line else (
                C_RED if "CAUSE" in line or "infeasible" in line or "fail" in line else
                "#86efac" if "FIX" in line else "white")
        fontweight = "bold" if "──" in line else "normal"
        ax6.text(0.04, y_pos, line, transform=ax6.transAxes,
                 fontsize=8, color=color, fontweight=fontweight,
                 fontfamily="monospace", va="top")
        y_pos -= 0.07

    plt.savefig(out_path, dpi=130, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.close()
    print(f"[freeze_analyzer] plot saved -> {out_path}")


# ── entry point ────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("video", nargs="?",
                        help="Path to screen-recorded video file")
    parser.add_argument("--fps", type=float, default=10.0,
                        help="Frame extraction rate (default 10 = 0.1s intervals)")
    parser.add_argument("--out", default=None,
                        help="Output PNG path (default: same dir as video)")
    args = parser.parse_args()

    # resolve video path
    if args.video:
        video_path = args.video
    else:
        pattern = os.path.expanduser("~/Videos/*.mp4")
        candidates = sorted(glob.glob(pattern), key=os.path.getmtime)
        if not candidates:
            sys.exit("No .mp4 files found in ~/Videos/")
        video_path = candidates[-1]
        print(f"[freeze_analyzer] using latest video: {video_path}")

    if not os.path.exists(video_path):
        sys.exit(f"File not found: {video_path}")

    out_path = args.out or os.path.splitext(video_path)[0] + "_freeze_analysis.png"

    print(f"[freeze_analyzer] analyzing {video_path} @ {args.fps} fps ...")
    times, xs, ys, devs, red_pixels, first_frame = analyze_video(video_path, args.fps)

    print(f"[freeze_analyzer] {len(times)} samples extracted")
    print(f"[freeze_analyzer] dev  mean={np.nanmean(devs)*PX_TO_MM:.1f}mm "
          f"max={np.nanmax(devs)*PX_TO_MM:.1f}mm")

    freeze_t, freeze_pos = detect_freeze(xs, ys, times)
    if freeze_t:
        print(f"[freeze_analyzer] FREEZE detected at t={freeze_t:.2f}s "
              f"pos=({freeze_pos[0]:.1f},{freeze_pos[1]:.1f})")
    else:
        print("[freeze_analyzer] no freeze detected")

    make_plot(times, xs, ys, devs, red_pixels, first_frame,
              freeze_t, freeze_pos, video_path, out_path)
    print(f"[freeze_analyzer] done -> {out_path}")


if __name__ == "__main__":
    main()
