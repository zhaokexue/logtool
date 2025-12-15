import argparse
import numpy as np
from PIL import Image
import pandas as pd

def binary_dilate(m, iters=1):
    m = m.astype(np.uint8)
    for _ in range(iters):
        p = np.pad(m, 1, mode="edge")
        acc = np.zeros_like(m)
        for dy in (-1,0,1):
            for dx in (-1,0,1):
                acc = np.maximum(acc, p[1+dy:1+dy+m.shape[0], 1+dx:1+dx+m.shape[1]])
        m = acc
    return m.astype(bool)

def binary_erode(m, iters=1):
    m = m.astype(np.uint8)
    for _ in range(iters):
        p = np.pad(m, 1, mode="edge")
        acc = np.ones_like(m)
        for dy in (-1,0,1):
            for dx in (-1,0,1):
                acc = np.minimum(acc, p[1+dy:1+dy+m.shape[0], 1+dx:1+dx+m.shape[1]])
        m = acc
    return m.astype(bool)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--img", required=True)
    ap.add_argument("--out_map", required=True)
    ap.add_argument("--out_traj", required=True)

    ap.add_argument("--height_m", type=float, default=10.0)        # 固定 10m
    ap.add_argument("--res", type=float, default=0.05)             # 0.05m/cell
    ap.add_argument("--lane_step_m", type=float, default=0.30)     # 覆盖条带间距
    ap.add_argument("--pose_hz", type=float, default=20.0)         # Pose 输出频率
    ap.add_argument("--speed", type=float, default=0.25)           # m/s
    ap.add_argument("--t0_ns", type=int, default=1_000_000_000)    # 起始时间戳
    args = ap.parse_args()

    img = Image.open(args.img).convert("RGB")
    rgb = np.array(img)
    h, w = rgb.shape[:2]

    # 1) 粗分割：非黑色区域
    mask = (rgb.sum(axis=2) > 60)

    # 清理：闭运算填小洞、开运算去噪
    mask = binary_dilate(mask, iters=2)
    mask = binary_erode(mask, iters=2)

    # 2) 偏蓝像素：更稳地锁定“可通行蓝色区域”，避免文字/家具图标
    r = rgb[:, :, 0].astype(np.int16)
    g = rgb[:, :, 1].astype(np.int16)
    b = rgb[:, :, 2].astype(np.int16)
    blueish = (b - r > 10) & (b - g > 0)

    walkable = mask & blueish
    walkable = binary_dilate(walkable, iters=2)
    walkable = binary_erode(walkable, iters=2)

    # 3) Resize to occupancy grid
    H = int(round(args.height_m / args.res))
    W = int(round(H * (w / h)))

    walkable_small = Image.fromarray((walkable * 255).astype(np.uint8)).resize(
        (W, H), resample=Image.NEAREST
    )
    free = (np.array(walkable_small) > 0)

    # 4) Build occupancy: -1 unknown, 0 free, 100 occupied (boundary band)
    occ = np.full((H, W), -1, dtype=np.int16)
    occ[free] = 0

    edge = free & (~binary_erode(free, 1))
    edge = binary_dilate(edge, 1)
    occ[edge] = 100
    occ[free & (~edge)] = 0

    # 5) Serpentine coverage path on free space
    row_step = max(1, int(round(args.lane_step_m / args.res)))
    pts = []
    angs = []
    direction = 1

    for y in range(0, H, row_step):
        row = (occ[y, :] == 0)
        xs = np.where(row)[0]
        if xs.size == 0:
            continue

        # contiguous segments
        segs = []
        s = xs[0]
        p = xs[0]
        for x in xs[1:]:
            if x == p + 1:
                p = x
            else:
                segs.append((s, p))
                s = x
                p = x
        segs.append((s, p))

        ordered = segs if direction == 1 else segs[::-1]
        for (a, b) in ordered:
            if direction == 1:
                xs_path = range(a, b + 1)
                ang = 0.0
            else:
                xs_path = range(b, a - 1, -1)
                ang = np.pi

            for x in xs_path:
                wx = (x + 0.5) * args.res
                wy = (y + 0.5) * args.res
                pts.append((wx, wy))
                angs.append(ang)

        direction *= -1

    pts = np.array(pts, dtype=np.float32)
    angs = np.array(angs, dtype=np.float32)

    if len(pts) < 2:
        raise RuntimeError("trajectory too short: check segmentation thresholds or input image style")

    # 6) Resample at pose_hz with constant speed
    d = np.diff(pts, axis=0)
    ds = np.sqrt((d * d).sum(axis=1))
    s = np.concatenate([[0.0], np.cumsum(ds)])
    total_len = float(s[-1])

    total_time = total_len / args.speed
    t = np.arange(0.0, total_time, 1.0 / args.pose_hz)

    x = np.interp(t * args.speed, s, pts[:, 0])
    y = np.interp(t * args.speed, s, pts[:, 1])

    idx = np.searchsorted(s, t * args.speed, side="right") - 1
    idx = np.clip(idx, 0, len(angs) - 1)
    a = angs[idx]

    ts_ns = args.t0_ns + (t * 1e9).astype(np.int64)

    pd.DataFrame({
        "timestamp_ns": ts_ns,
        "x_m": x,
        "y_m": y,
        "angle_rad": a
    }).to_csv(args.out_traj, index=False)

    np.save(args.out_map, occ)
    print(f"Wrote grid: {args.out_map} shape={occ.shape} (H,W)")
    print(f"Wrote traj: {args.out_traj} samples={len(ts_ns)} duration~{total_time:.2f}s")

if __name__ == "__main__":
    main()

