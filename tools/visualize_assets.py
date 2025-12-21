#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ===== 强制无 GUI 后端（必须在 import pyplot 之前）=====
import matplotlib
matplotlib.use("Agg")

import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math

def world_to_cell(wx, wy, origin_x, origin_y, res):
    cx = int(math.floor((wx - origin_x) / res))
    cy = int(math.floor((wy - origin_y) / res))
    return cx, cy

def world_to_local(wx, wy, px, py, yaw):
    # local: x forward, y left
    dx = wx - px
    dy = wy - py
    c = math.cos(yaw)
    s = math.sin(yaw)
    lx =  c * dx + s * dy
    ly = -s * dx + c * dy
    return lx, ly

def raycast_grid_hit(occ, px, py, ang_world, origin_x, origin_y, res,
                     max_range=8.0, hit_occ_value=100, hit_unknown=True):
    """
    栅格 raycast (DDA)：
      - 命中 occupied(>=hit_occ_value) -> hit
      - 若 hit_unknown=True，进入 unknown(-1) -> hit（作为已知边界）
      - 否则，unknown 继续走直到 occupied 或 max_range
    返回：(hit_x_world, hit_y_world, range)
    """
    H, W = occ.shape
    dx = math.cos(ang_world)
    dy = math.sin(ang_world)
    eps = 1e-9

    # 起点格子
    cx, cy = world_to_cell(px, py, origin_x, origin_y, res)
    if cx < 0 or cx >= W or cy < 0 or cy >= H:
        # 起点不在地图内：直接 max_range 截断
        hx = px + max_range * dx
        hy = py + max_range * dy
        return hx, hy, max_range

    step_x = 1 if dx > 0 else -1
    step_y = 1 if dy > 0 else -1

    if abs(dx) < eps:
        tMaxX = float("inf")
        tDeltaX = float("inf")
    else:
        next_vert = origin_x + (cx + (1 if dx > 0 else 0)) * res
        tMaxX = (next_vert - px) / dx
        tDeltaX = res / abs(dx)

    if abs(dy) < eps:
        tMaxY = float("inf")
        tDeltaY = float("inf")
    else:
        next_horz = origin_y + (cy + (1 if dy > 0 else 0)) * res
        tMaxY = (next_horz - py) / dy
        tDeltaY = res / abs(dy)

    t = 0.0
    # 先检查起始 cell
    v0 = int(occ[cy, cx])
    if (hit_unknown and v0 == -1) or (v0 >= hit_occ_value):
        return px, py, 0.0

    while t <= max_range:
        # 前进到下一个格子边界
        if tMaxX < tMaxY:
            cx += step_x
            t = tMaxX
            tMaxX += tDeltaX
        else:
            cy += step_y
            t = tMaxY
            tMaxY += tDeltaY

        if t > max_range:
            break

        # 出图：截断到 max_range
        if cx < 0 or cx >= W or cy < 0 or cy >= H:
            hx = px + t * dx
            hy = py + t * dy
            return hx, hy, t

        v = int(occ[cy, cx])

        # ✅ unknown 命中（已知区域边界）
        if hit_unknown and v == -1:
            hx = px + t * dx
            hy = py + t * dy
            return hx, hy, t

        # ✅ occupied 命中墙
        if v >= hit_occ_value:
            hx = px + t * dx
            hy = py + t * dy
            return hx, hy, t

    # 未命中：max_range 截断
    hx = px + max_range * dx
    hy = py + max_range * dy
    return hx, hy, max_range

def save_and_close(path: Path):
    plt.tight_layout()
    plt.savefig(path, dpi=200)
    plt.close()

def main():
    ap = argparse.ArgumentParser(description="Offline visualization for grid_map + trajectory")
    ap.add_argument("--grid", default="assets/grid_map.npy")
    ap.add_argument("--traj", default="assets/trajectory.csv")
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument(
        "--scan_time",
        type=float,
        default=-1.0,
        help="Scan time in seconds since trajectory start; default uses mid time",
    )
    ap.add_argument("--out_dir", default="assets/vis")
    args = ap.parse_args()

    grid_path = Path(args.grid)
    traj_path = Path(args.traj)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # ---------- Load data ----------
    occ = np.load(grid_path)  # int16: -1 unknown, 0 free, 100 occupied
    H, W = occ.shape
    res = float(args.res)

    df = pd.read_csv(traj_path)
    if df.shape[0] < 2:
        raise RuntimeError("trajectory too short")

    t_ns = df["timestamp_ns"].to_numpy(dtype=np.int64)
    x = df["x_m"].to_numpy(dtype=np.float64)
    y = df["y_m"].to_numpy(dtype=np.float64)
    a = df["angle_rad"].to_numpy(dtype=np.float64)

    t0 = t_ns[0]
    t_sec = (t_ns - t0) / 1e9

    xmin, ymin = 0.0, 0.0
    xmax, ymax = W * res, H * res

    # ---------- 1) GridMap ----------
    disp = occ.astype(float)
    disp[disp < 0] = np.nan  # unknown

    plt.figure(figsize=(6, 6))
    plt.imshow(
        disp,
        origin="lower",
        extent=[xmin, xmax, ymin, ymax],
        aspect="equal",
    )
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title(f"GridMap (H={H}, W={W}, res={res}m)")
    save_and_close(out_dir / "01_grid_map.png")

    # ---------- 2) Trajectory overlay ----------
    plt.figure(figsize=(6, 6))
    plt.imshow(
        disp,
        origin="lower",
        extent=[xmin, xmax, ymin, ymax],
        aspect="equal",
    )
    plt.plot(x, y, linewidth=1.0)
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title(f"Trajectory overlay (samples={len(df)})")
    save_and_close(out_dir / "02_trajectory_overlay.png")

    # ---------- 3) Yaw over time ----------
    plt.figure(figsize=(8, 4))
    plt.plot(t_sec, a)
    plt.xlabel("time (s)")
    plt.ylabel("yaw (rad)")
    plt.title("Yaw over time")
    save_and_close(out_dir / "03_yaw_over_time.png")

    # ---------- 4) One LaserScan frame: grid raycast -> local pointcloud ----------
    if args.scan_time < 0:
        scan_t = 0.5 * float(t_sec[-1])
    else:
        scan_t = float(args.scan_time)
    scan_t = np.clip(scan_t, 0.0, float(t_sec[-1]))

    idx = int(np.argmin(np.abs(t_sec - scan_t)))
    px, py, pa = float(x[idx]), float(y[idx]), float(a[idx])

    origin_x, origin_y = 0.0, 0.0

    num_beams = 360
    angle_min = 0.0
    angle_inc = 2.0 * np.pi / num_beams

    local_x = np.empty(num_beams, dtype=np.float64)
    local_y = np.empty(num_beams, dtype=np.float64)

    ranges = np.empty(num_beams, dtype=np.float64)

    for i in range(num_beams):
        ang_world = pa + angle_min + i * angle_inc
        hx, hy, rr = raycast_grid_hit(
            occ, px, py, ang_world,
            origin_x, origin_y, res,
            max_range=8.0,
            hit_occ_value=100,
            hit_unknown=True  # ✅ unknown 作为已知边界命中
        )
        lx, ly = world_to_local(hx, hy, px, py, pa)
        local_x[i] = lx
        local_y[i] = ly
        ranges[i] = rr

    # 绘制 local 点云
    plt.figure(figsize=(6, 6))
    plt.scatter(local_x, local_y, s=0.125, c="red", marker="o", zorder=3, label="laser hits")

    # 机器人原点与朝向（可选）
    plt.plot(0.0, 0.0, "bo", markersize=5, zorder=5, label="robot")
    plt.arrow(0.0, 0.0, 0.5, 0.0, head_width=0.08, length_includes_head=True, zorder=5)

    plt.gca().set_aspect("equal", adjustable="box")
    plt.xlabel("x (m) [forward]")
    plt.ylabel("y (m) [left]")
    rmax = float(np.nanmax(ranges))
    lim = max(1.0, min(8.0, rmax + 0.5))
    plt.xlim(-lim, lim)
    plt.ylim(-lim, lim)

    plt.title(
        f"Laser point cloud (from GridMap raycast) @ t={scan_t:.2f}s\n"
        f"pose=({px:.2f},{py:.2f}), yaw={pa:.2f}"
    )

    save_and_close(out_dir / "04_laserscan.png")    

    # ---------- Summary ----------
    print("Visualization done. Files written to:")
    for name in [
        "01_grid_map.png",
        "02_trajectory_overlay.png",
        "03_yaw_over_time.png",
        "04_laserscan.png",
    ]:
        print(" ", out_dir / name)


if __name__ == "__main__":
    main()
