#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Plot EKF-estimated landmarks vs. ground-truth landmarks and save PNG + CSV.

Usage:
  python plot_ekf_vs_true.py --groundtruth TrueMap.txt --estimate ekf_map.txt \
      --out_png ekf_true_map_diff.png --out_csv ekf_true_errors.csv

Notes:
- Ground truth file is a JSON (e.g., TrueMap.txt).
- Estimate file is a Python dict string with fields "taglist" and "map"
  (same as your EKF saver输出): {"taglist":[...], "map":[[x1,x2,...],[y1,y2,...]]}
- Uses rigid (SE(2)) alignment via Umeyama (no scale).
- Uses matplotlib (no seaborn).
"""

import ast
import json
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ---------- Parsers & math (adapted from your reference code) ----------

def parse_groundtruth(fname: str) -> dict:
    """Read ground truth JSON and return {tag:int -> 2x1 ndarray [x;y]} for aruco* only."""
    with open(fname, 'r', encoding='utf-8') as f:
        gt_dict = json.load(f)
        aruco_dict = {}
        for key in gt_dict:
            if key.startswith("aruco"):
                aruco_num = int(key.strip('aruco')[:-2])  # "aruco10_0" -> 10
                aruco_dict[aruco_num] = np.reshape([gt_dict[key]["x"], gt_dict[key]["y"]], (2, 1))
    return aruco_dict


def parse_user_map(fname: str) -> dict:
    """Read EKF exported map (Python dict string) and return {tag:int -> 2x1 ndarray [x;y]}."""
    with open(fname, 'r', encoding='utf-8') as f:
        usr_dict = ast.literal_eval(f.read())
        aruco_dict = {}
        for (i, tag) in enumerate(usr_dict["taglist"]):
            aruco_dict[int(tag)] = np.reshape(
                [usr_dict["map"][0][i], usr_dict["map"][1][i]],
                (2, 1)
            )
    return aruco_dict


def match_aruco_points(aruco0: dict, aruco1: dict):
    """
    Return (keys, points0, points1)
    Where points0 = [aruco0[key] ...], points1 = [aruco1[key] ...], both 2xN
    Only keys present in both dicts are used.
    """
    points0 = []
    points1 = []
    keys = []
    for key in aruco0:
        if key not in aruco1:
            continue
        points0.append(aruco0[key])
        points1.append(aruco1[key])
        keys.append(key)
    if len(points0) == 0:
        return [], np.zeros((2, 0)), np.zeros((2, 0))
    return keys, np.hstack(points0), np.hstack(points1)


def solve_umeyama2d(points1, points2):
    """
    Solve optimal R, t such that: R * p1_i + t = p2_i
    Returns rotation angle theta and translation x (2x1).
    """
    assert points1.shape[0] == 2
    assert points1.shape == points2.shape

    num_points = points1.shape[1]
    mu1 = (1/num_points) * np.reshape(np.sum(points1, axis=1), (2, -1))
    mu2 = (1/num_points) * np.reshape(np.sum(points2, axis=1), (2, -1))
    Sig12 = (1/num_points) * (points2 - mu2) @ (points1 - mu1).T

    U, d, Vh = np.linalg.svd(Sig12)
    S = np.eye(2)
    if np.linalg.det(Sig12) < 0:
        S[-1, -1] = -1

    R = U @ S @ Vh
    theta = np.arctan2(R[1, 0], R[0, 0])
    x = mu2 - R @ mu1
    return theta, x


def apply_transform(theta, x, points):
    """Apply SE(2) transform to 2xN points."""
    assert points.shape[0] == 2
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s,  c]])
    return R @ points + x


def compute_rmse(points1, points2):
    """
    RMSE between two matched sets (both 2xN).
    Pads to 10 points using the highest residual (to match your original metric).
    Returns (rmse, residuals[2xN_after_pad]).
    """
    assert points1.shape[0] == 2
    assert points1.shape == points2.shape
    num_points = points1.shape[1]
    residual = (points1 - points2)

    # Pad to 10 with the highest residual (keeps your earlier grading convention)
    if num_points != 10 and num_points > 0:
        best = 0.0
        idx = 0
        for i in range(num_points):
            r2 = residual[0, i]**2 + residual[1, i]**2
            if r2 > best:
                best = r2
                idx = i
        highest = residual[:, idx:idx+1]
        for _ in range(10 - num_points):
            residual = np.concatenate((residual, highest), axis=1)

    unraveled_residual = residual.copy()
    residual_flat = residual.ravel()
    if residual_flat.size == 0:
        return np.nan, unraveled_residual
    MSE = (1.0/10) * np.sum(residual_flat**2)
    return float(np.sqrt(MSE)), unraveled_residual


# ---------- Plot & CSV ----------

def plot_and_save(common_tags, gt_vec, est_vec_aligned, out_png):
    """
    gt_vec: 2xN, est_vec_aligned: 2xN in the same (GT) frame.
    """
    x_gt = gt_vec[0, :]
    y_gt = gt_vec[1, :]
    x_est = est_vec_aligned[0, :]
    y_est = est_vec_aligned[1, :]

    plt.figure(figsize=(7, 7))
    plt.scatter(x_est, y_est, marker='o', label='EKF (aligned)')
    plt.scatter(x_gt, y_gt, marker='s', label='Ground Truth')

    for i, t in enumerate(common_tags):
        plt.plot([x_est[i], x_gt[i]], [y_est[i], y_gt[i]])
        plt.annotate(f"{t}", (x_gt[i], y_gt[i]), textcoords="offset points", xytext=(5, 5))

    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('Landmarks: EKF vs True (aligned via Umeyama SE(2))')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_png, dpi=150)
    print(f"Saved figure -> {out_png}")


def build_csv(common_tags, gt_vec, est_vec_raw, est_vec_aligned, out_csv):
    """
    Save per-tag data to CSV: raw est, aligned est, GT, errors (aligned).
    """
    N = len(common_tags)
    x_gt = gt_vec[0, :N]
    y_gt = gt_vec[1, :N]
    x_est_raw = est_vec_raw[0, :N]
    y_est_raw = est_vec_raw[1, :N]
    x_est = est_vec_aligned[0, :N]
    y_est = est_vec_aligned[1, :N]

    dx = x_est - x_gt
    dy = y_est - y_gt
    err = np.sqrt(dx**2 + dy**2)

    df = pd.DataFrame({
        "tag": common_tags,
        "x_est_raw": x_est_raw, "y_est_raw": y_est_raw,
        "x_est_aligned": x_est, "y_est_aligned": y_est,
        "x_true": x_gt, "y_true": y_gt,
        "dx": dx, "dy": dy, "err": err
    }).sort_values("tag").reset_index(drop=True)

    df.to_csv(out_csv, index=False)
    print(f"Saved errors CSV -> {out_csv}")


# ---------- Main ----------

def main():
    parser = argparse.ArgumentParser(description="Plot EKF map vs True map (auto-read & rigid alignment)")
    parser.add_argument("--groundtruth", required=True, help="Ground truth JSON file (e.g., TrueMap.txt)")
    parser.add_argument("--estimate", required=True, help="EKF map file (Python dict string with taglist/map)")
    parser.add_argument("--out_png", default="ekf_true_map_diff.png", help="Output PNG path")
    parser.add_argument("--out_csv", default="ekf_true_errors.csv", help="Output CSV path")
    args = parser.parse_args()

    # Parse
    gt_aruco = parse_groundtruth(args.groundtruth)
    us_aruco = parse_user_map(args.estimate)

    # Match & stack
    common_tags, us_vec, gt_vec = match_aruco_points(us_aruco, gt_aruco)
    if len(common_tags) == 0:
        print("No common aruco tags between estimate and ground truth.")
        return

    # RMSE before alignment
    rmse0, _ = compute_rmse(us_vec, gt_vec)
    print(f"RMSE before alignment: {rmse0}")

    # Solve rigid alignment (SE(2)): est -> GT
    if us_vec.shape[1] >= 2:
        theta, x = solve_umeyama2d(us_vec, gt_vec)
        us_vec_aligned = apply_transform(theta, x, us_vec)
        print("Optimal transform (est -> GT):")
        print(f"  Rotation angle (rad): {theta}")
        print(f"  Translation vector: ({x[0,0]}, {x[1,0]})")
    else:
        # Not enough points to estimate rotation; just translate to mean
        print("Not enough common points for full alignment; using identity transform.")
        theta = 0.0
        x = np.zeros((2, 1))
        us_vec_aligned = us_vec

    # RMSE after alignment
    rmse1, residuals = compute_rmse(us_vec_aligned, gt_vec)
    print(f"RMSE after alignment: {rmse1}")

    # Plot (aligned)
    plot_and_save(common_tags, gt_vec, us_vec_aligned, args.out_png)

    # CSV (raw + aligned + GT + errors)
    build_csv(common_tags, gt_vec, us_vec, us_vec_aligned, args.out_csv)

    # Also print arrays (兼容你原来的打印习惯)
    print()
    print("Pred Locations (raw, stacked 2xN):")
    print(np.array2string(us_vec, precision=4, separator=','))
    print("Aligned Pred Locations (stacked 2xN):")
    print(np.array2string(us_vec_aligned, precision=4, separator=','))
    print("Real Locations (stacked 2xN):")
    print(np.array2string(gt_vec, precision=4, separator=','))

    print("Marker errors (aligned residuals, padded if needed):")
    print(np.array2string(residuals, precision=4, separator=','))


if __name__ == "__main__":
    main()
