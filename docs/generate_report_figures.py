#!/usr/bin/env python3
"""
生成 AAE5303 Trajectory Comparison 四张图（与 joyziyan678/AAE5303_assignment2_orbslam3_demo- 一致）：
1) 2D 轨迹 - 对齐前（匹配位姿）
2) 2D 轨迹 - Sim(3) 对齐后
3) ATE 误差分布直方图
4) ATE 沿轨迹变化

需要先运行: evo_ape tum ground_truth.txt CameraTrajectory.txt --align --correct_scale --t_max_diff 0.1 -va --save_results ate.zip
"""

from __future__ import annotations

import argparse
import zipfile
from typing import List, Tuple

import numpy as np


def load_tum_positions(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """Load TUM file, skip comments. Return (t, p) with p shape (N,3)."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 4:
                try:
                    t = float(parts[0])
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    rows.append([t, x, y, z])
                except ValueError:
                    pass
    if not rows:
        return np.array([]), np.array([]).reshape(0, 3)
    arr = np.array(rows)
    return arr[:, 0].astype(float), arr[:, 1:4].astype(float)


def associate_by_time(t_gt: np.ndarray, t_est: np.ndarray, t_max_diff_s: float) -> Tuple[np.ndarray, np.ndarray]:
    """Greedy two-pointer association. Returns (gt_indices, est_indices)."""
    gt_idx, est_idx = [], []
    i, j = 0, 0
    n, m = len(t_gt), len(t_est)
    while i < n and j < m:
        dt = t_est[j] - t_gt[i]
        if abs(dt) <= t_max_diff_s:
            gt_idx.append(i)
            est_idx.append(j)
            i += 1
            j += 1
            continue
        if dt < -t_max_diff_s:
            j += 1
        else:
            i += 1
    return np.array(gt_idx, dtype=int), np.array(est_idx, dtype=int)


def load_sim3_and_errors(evo_ape_zip_path: str) -> Tuple[np.ndarray, np.ndarray]:
    with zipfile.ZipFile(evo_ape_zip_path, "r") as zf:
        sim3 = np.load(zf.open("alignment_transformation_sim3.npy"), allow_pickle=False)
        err = np.load(zf.open("error_array.npy"), allow_pickle=False)
    return sim3.astype(float), err.astype(float)


def apply_sim3(sim3: np.ndarray, xyz: np.ndarray) -> np.ndarray:
    """Apply 4x4 Sim(3) to Nx3 points."""
    ones = np.ones((xyz.shape[0], 1), dtype=float)
    pts = np.hstack([xyz, ones])
    out = (sim3 @ pts.T).T
    return out[:, :3]


def main():
    parser = argparse.ArgumentParser(description="生成 Trajectory Comparison 四张图")
    parser.add_argument("--gt", default="ground_truth.txt", help="Ground truth TUM")
    parser.add_argument("--est", default="CameraTrajectory.txt", help="Estimated TUM")
    parser.add_argument("--evo-ape-zip", default="ate.zip", help="evo_ape --save_results 的 zip")
    parser.add_argument("--out", default="figures/trajectory_evaluation.png", help="输出 PNG")
    parser.add_argument("--t-max-diff", type=float, default=0.1)
    parser.add_argument("--title-suffix", default="(HKisland_GNSS03)")
    args = parser.parse_args()

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("请安装 matplotlib: pip3 install matplotlib", file=__import__("sys").stderr)
        return 1

    t_gt, p_gt = load_tum_positions(args.gt)
    t_est, p_est = load_tum_positions(args.est)
    if len(t_gt) < 5 or len(t_est) < 5:
        raise SystemExit("轨迹点数不足，无法绘图")

    gt_idx, est_idx = associate_by_time(t_gt, t_est, args.t_max_diff)
    if len(gt_idx) < 5:
        raise SystemExit(f"匹配位姿过少 ({len(gt_idx)})，请检查时间戳或使用插值轨迹")

    gt_m = p_gt[gt_idx]
    est_m = p_est[est_idx]

    sim3, ate_errors = load_sim3_and_errors(args.evo_ape_zip)
    if len(ate_errors) != len(est_m):
        raise SystemExit(f"evo zip 中 error 数量 ({len(ate_errors)}) 与匹配位姿数 ({len(est_m)}) 不一致，请用同一轨迹重新运行 evo_ape --save_results")
    est_aligned = apply_sim3(sim3, est_m)

    fig, axes = plt.subplots(2, 2, figsize=(12, 12))

    # 1) Before alignment
    ax = axes[0, 0]
    ax.set_title(f"2D Trajectory - Before Alignment {args.title_suffix}")
    ax.plot(gt_m[:, 0], gt_m[:, 1], color="green", label="Ground Truth", linewidth=2)
    ax.plot(est_m[:, 0], est_m[:, 1], color="red", linestyle="--", label="VO (Unaligned)", linewidth=1.5)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    # 2) After Sim(3) alignment
    ax = axes[0, 1]
    ax.set_title(f"2D Trajectory - After Sim(3) Alignment {args.title_suffix}")
    ax.plot(gt_m[:, 0], gt_m[:, 1], color="green", label="Ground Truth", linewidth=2)
    ax.plot(est_aligned[:, 0], est_aligned[:, 1], color="blue", label="VO (Aligned)", linewidth=1.5)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    # 3) ATE histogram
    ax = axes[1, 0]
    ax.set_title("Absolute Trajectory Error Distribution")
    ax.hist(ate_errors, bins=40, color="#4C78A8", edgecolor="black", alpha=0.75)
    mean = float(np.mean(ate_errors))
    median = float(np.median(ate_errors))
    ax.axvline(mean, color="red", linestyle="--", linewidth=2, label=f"Mean: {mean:.2f} m")
    ax.axvline(median, color="orange", linestyle="--", linewidth=2, label=f"Median: {median:.2f} m")
    ax.set_xlabel("ATE [m]")
    ax.set_ylabel("Frequency")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    # 4) ATE along trajectory
    ax = axes[1, 1]
    ax.set_title("ATE Error Along Trajectory")
    x = np.arange(len(ate_errors))
    ax.plot(x, ate_errors, color="blue", linewidth=1)
    ax.fill_between(x, ate_errors, color="#72B7B2", alpha=0.25)
    ax.set_xlabel("Matched Pose Index")
    ax.set_ylabel("ATE [m]")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    out_path = args.out
    import os
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    fig.savefig(out_path, dpi=200)
    plt.close(fig)
    print(f"已保存: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
