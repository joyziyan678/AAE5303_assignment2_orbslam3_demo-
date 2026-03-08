#!/usr/bin/env python3
"""
AAE5303 作业评估脚本：单目 VO 轨迹精度
评估指标：ATE（Sim3 对齐）、RPE 漂移率（平移+旋转）、完整性（每帧轨迹，使用 CameraTrajectory.txt）
"""

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path


def count_tum_poses(path: str) -> int:
    """统计 TUM 格式轨迹文件中的位姿数量（跳过空行和 # 注释）。"""
    count = 0
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 8:
                try:
                    float(parts[0])
                    count += 1
                except ValueError:
                    pass
    return count


def run_evo_ape(gt: str, est: str, t_max_diff: float) -> tuple:
    """运行 evo_ape，返回 (matched_pairs, ate_rmse, ate_mean, ate_std)。"""
    cmd = [
        "evo_ape", "tum", gt, est,
        "--align", "--correct_scale",
        "--t_max_diff", str(t_max_diff),
        "-va",
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    out = result.stdout + result.stderr
    if result.returncode != 0 and "Degenerate" not in out:
        print(out, file=sys.stderr)
        raise RuntimeError(f"evo_ape failed: {result.returncode}")

    matched = None
    rmse = mean = std = None
    for line in out.splitlines():
        # "Compared 6 absolute pose pairs."
        m = re.search(r"Compared\s+(\d+)\s+absolute\s+pose\s+pairs", line, re.I)
        if m:
            matched = int(m.group(1))
        m = re.search(r"Found\s+(\d+)\s+of\s+max\.\s+\d+\s+possible\s+matching", line)
        if m and matched is None:
            matched = int(m.group(1))
        # Tab 分隔统计表: "      rmse	0.031947"
        if "\t" in line:
            parts = line.split("\t", 1)
            k, v = parts[0].strip().lower(), parts[1].strip()
            try:
                val = float(v)
                if "rmse" in k:
                    rmse = val
                elif "mean" in k:
                    mean = val
                elif "std" in k:
                    std = val
            except (ValueError, TypeError):
                pass
    if matched is None:
        m = re.search(r"Found\s+(\d+)\s+of\s+max\.\s+\d+\s+possible\s+matching", out)
        if m:
            matched = int(m.group(1))
    return (matched, rmse, mean, std)


def run_evo_rpe(gt: str, est: str, t_max_diff: float, delta: float, delta_unit: str, pose_relation: str) -> tuple:
    """运行 evo_rpe，返回 (matched_pairs, mean_error)。"""
    cmd = [
        "evo_rpe", "tum", gt, est,
        "--align", "--correct_scale",
        "--t_max_diff", str(t_max_diff),
        "--delta", str(delta), "--delta_unit", delta_unit,
        "--pose_relation", pose_relation,
        "-va",
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    out = result.stdout + result.stderr
    if result.returncode != 0 and "Degenerate" not in out:
        print(out, file=sys.stderr)
        raise RuntimeError(f"evo_rpe failed: {result.returncode}")

    matched = None
    mean_err = None
    for line in out.splitlines():
        m = re.search(r"Compared\s+(\d+)\s+relative", line, re.I)
        if m:
            matched = int(m.group(1))
        if m is None or not m:
            m = re.search(r"Found\s+(\d+)\s+of\s+max\.\s+\d+\s+possible", line)
            if m and matched is None:
                matched = int(m.group(1))
        if "\t" in line:
            parts = line.split("\t", 1)
            k, v = parts[0].strip().lower(), parts[1].strip()
            try:
                val = float(v)
                if "mean" in k:
                    mean_err = val
            except (ValueError, TypeError):
                pass
    if matched is None:
        m = re.search(r"Found\s+(\d+)\s+of\s+max\.\s+\d+\s+possible", out)
        if m:
            matched = int(m.group(1))
    return (matched, mean_err)


def main():
    parser = argparse.ArgumentParser(description="单目 VO 评估：ATE、RPE 漂移率、完整性（使用每帧轨迹 CameraTrajectory.txt）")
    parser.add_argument("--groundtruth", "-g", default="ground_truth.txt", help="真值轨迹 TUM 文件")
    parser.add_argument("--estimated", "-e", default="CameraTrajectory.txt", help="估计轨迹 TUM 文件（每帧）")
    parser.add_argument("--t-max-diff", type=float, default=0.1, help="时间关联阈值 (s)")
    parser.add_argument("--delta-m", type=float, default=10.0, help="RPE 距离间隔 (m)")
    parser.add_argument("--workdir", "-w", default=None, help="工作目录（默认当前目录）")
    parser.add_argument("--json-out", "-j", default=None, help="输出 JSON 结果路径")
    args = parser.parse_args()

    workdir = Path(args.workdir) if args.workdir else Path.cwd()
    gt = workdir / args.groundtruth
    est = workdir / args.estimated
    if not gt.exists():
        gt = Path(args.groundtruth)
    if not est.exists():
        est = Path(args.estimated)
    if not gt.exists() or not est.exists():
        print(f"ERROR: 文件不存在: gt={gt}, est={est}", file=sys.stderr)
        sys.exit(1)

    gt_poses = count_tum_poses(str(gt))
    est_poses = count_tum_poses(str(est))
    gt_str, est_str = str(gt), str(est)

    print("=" * 80)
    print("单目 VO 评估（每帧轨迹 CameraTrajectory）")
    print("=" * 80)
    print(f"真值: {gt_str} ({gt_poses} 位姿)")
    print(f"估计: {est_str} ({est_poses} 位姿)")
    print()

    # 1) ATE
    matched_ate, ate_rmse, ate_mean, ate_std = run_evo_ape(gt_str, est_str, args.t_max_diff)
    if matched_ate is None:
        matched_ate = 0
    completeness_pct = (matched_ate / gt_poses * 100.0) if gt_poses else 0.0

    # 2) RPE 平移
    _, rpe_trans_mean = run_evo_rpe(
        gt_str, est_str, args.t_max_diff,
        delta=args.delta_m, delta_unit="m", pose_relation="trans_part",
    )
    trans_drift_rate = (rpe_trans_mean / args.delta_m) if rpe_trans_mean is not None else None

    # 3) RPE 旋转
    _, rpe_rot_mean = run_evo_rpe(
        gt_str, est_str, args.t_max_diff,
        delta=args.delta_m, delta_unit="m", pose_relation="angle_deg",
    )
    rot_drift_rate = (rpe_rot_mean / args.delta_m * 100.0) if rpe_rot_mean is not None else None

    # 报告
    print("METRIC 1: ATE (Absolute Trajectory Error) — Sim(3) 对齐 + 尺度校正")
    print("-" * 60)
    print(f"  RMSE:   {ate_rmse:.4f} m" if ate_rmse is not None else "  RMSE:   N/A")
    print(f"  Mean:   {ate_mean:.4f} m" if ate_mean is not None else "  Mean:   N/A")
    print(f"  Std:    {ate_std:.4f} m" if ate_std is not None else "  Std:    N/A")
    print()

    print("METRIC 2: RPE 平移漂移率 (距离间隔 delta={} m)".format(args.delta_m))
    print("-" * 60)
    print(f"  {args.delta_m} m 内平移 RPE 均值: {rpe_trans_mean:.4f} m" if rpe_trans_mean is not None else "  N/A")
    print(f"  平移漂移率:           {trans_drift_rate:.4f} m/m" if trans_drift_rate is not None else "  N/A")
    print()

    print("METRIC 3: RPE 旋转漂移率 (距离间隔 delta={} m)".format(args.delta_m))
    print("-" * 60)
    print(f"  {args.delta_m} m 内旋转 RPE 均值: {rpe_rot_mean:.4f} deg" if rpe_rot_mean is not None else "  N/A")
    print(f"  旋转漂移率:           {rot_drift_rate:.4f} deg/100m" if rot_drift_rate is not None else "  N/A")
    print()

    print("METRIC 4: 完整性 (Completeness)")
    print("-" * 60)
    print(f"  匹配位姿: {matched_ate} / {gt_poses}")
    print(f"  完整性:   {completeness_pct:.2f}%")
    print("=" * 80)

    # JSON
    report = {
        "ground_truth_file": str(gt),
        "estimated_file": str(est),
        "gt_poses": gt_poses,
        "estimated_poses": est_poses,
        "matched_poses": matched_ate,
        "completeness_percent": round(completeness_pct, 2),
        "ate_rmse_m": round(ate_rmse, 4) if ate_rmse is not None else None,
        "ate_mean_m": round(ate_mean, 4) if ate_mean is not None else None,
        "ate_std_m": round(ate_std, 4) if ate_std is not None else None,
        "rpe_delta_m": args.delta_m,
        "rpe_trans_mean_m": round(rpe_trans_mean, 4) if rpe_trans_mean is not None else None,
        "rpe_trans_drift_rate_m_per_m": round(trans_drift_rate, 4) if trans_drift_rate is not None else None,
        "rpe_rot_mean_deg": round(rpe_rot_mean, 4) if rpe_rot_mean is not None else None,
        "rpe_rot_drift_rate_deg_per_100m": round(rot_drift_rate, 4) if rot_drift_rate is not None else None,
        "t_max_diff_s": args.t_max_diff,
    }
    if args.json_out:
        out_path = Path(args.json_out)
        if not out_path.is_absolute() and args.workdir:
            out_path = workdir / out_path
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        print(f"已保存 JSON: {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
