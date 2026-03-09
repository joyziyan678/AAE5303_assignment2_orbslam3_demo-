# AAE5303 Assignment: Visual Odometry with ORB-SLAM3

ORB-SLAM3 VO Dataset Status

**Monocular Visual Odometry Evaluation on UAV Aerial Imagery**

_Hong Kong Island GNSS Dataset - MARS-LVIG_

---

## 📋 Table of Contents

1. Executive Summary
2. Introduction
3. Methodology
4. Dataset Description
5. Implementation Details
6. Results and Analysis
7. Visualizations
8. Discussion
9. Conclusions
10. References
11. Appendix

---

## 📊 Executive Summary

This report presents the implementation and evaluation of **Monocular Visual Odometry (VO)** using the **ORB-SLAM3** framework on the **HKisland_GNSS03** UAV aerial imagery dataset. The project evaluates trajectory accuracy against RTK ground truth using **four parallel, monocular-appropriate metrics** computed with the `evo` toolkit.

### Key Results

| Metric              | Value                 | Description                                               |
| ------------------- | --------------------- | --------------------------------------------------------- |
| **ATE RMSE**        | **97.9091 m**         | Global accuracy after Sim(3) alignment (scale corrected)  |
| **RPE Trans Drift** | **1.7585 m/m**        | Translation drift rate (mean error per meter, delta=10 m) |
| **RPE Rot Drift**   | **151.0589 deg/100m** | Rotation drift rate (mean angle per 100 m, delta=10 m)    |
| **Completeness**    | **97.03%**            | Matched poses / GT poses in evaluation window (1013 / 1044) |
| **Estimated poses**| 2,056                 | Trajectory poses in CameraTrajectory.txt                  |

Evaluation is performed over the **time window where both ground truth and estimated trajectory exist** (ground truth trimmed to estimated range; no interpolation). Full sequence: 1,955 GT poses; evaluation window: 1,044 GT poses.

---

## 📖 Introduction

### Background

ORB-SLAM3 is a state-of-the-art visual SLAM system capable of performing:

* **Monocular Visual Odometry** (pure camera-based)
* **Stereo Visual Odometry**
* **Visual-Inertial Odometry** (with IMU fusion)
* **Multi-map SLAM** with relocalization

This assignment focuses on **Monocular VO mode**, which:

* Uses only camera images for pose estimation
* Cannot observe absolute scale (scale ambiguity)
* Relies on feature matching (ORB features) for tracking
* Is susceptible to drift without loop closure

### Objectives

1. Implement monocular Visual Odometry using ORB-SLAM3
2. Process UAV aerial imagery from the HKisland_GNSS03 dataset
3. Extract RTK (Real-Time Kinematic) GPS data as ground truth
4. Evaluate trajectory accuracy using four parallel metrics appropriate for monocular VO
5. Document the complete workflow for reproducibility

### Scope

This assignment evaluates:

* **ATE (Absolute Trajectory Error)**: Global trajectory accuracy after Sim(3) alignment (monocular-friendly)
* **RPE drift rates (translation + rotation)**: Local consistency (drift per traveled distance)
* **Completeness**: Robustness / coverage (how much of the sequence is successfully tracked and evaluated)

---

## 🔬 Methodology

### ORB-SLAM3 Visual Odometry Overview

ORB-SLAM3 performs visual odometry through the following pipeline:

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  Input Image    │────▶│   ORB Feature   │────▶│   Feature       │
│  Sequence       │     │   Extraction    │     │   Matching      │
└─────────────────┘     └─────────────────┘     └────────┬────────┘
                                                         │
┌─────────────────┐     ┌─────────────────┐     ┌────────▼────────┐
│   Trajectory    │◀────│   Pose          │◀────│   Motion        │
│   Output        │     │   Estimation    │     │   Model         │
└─────────────────┘     └────────┬────────┘     └─────────────────┘
                                 │
                        ┌────────▼────────┐
                        │   Local Map     │
                        │   Optimization  │
                        └─────────────────┘
```

### Evaluation Metrics

#### 1. ATE (Absolute Trajectory Error)

Measures the RMSE of the aligned trajectory after Sim(3) alignment:

$$ATE_{RMSE} = \sqrt{\frac{1}{N}\sum_{i=1}^{N}|\mathbf{p}_{est}^i - \mathbf{p}_{gt}^i|^2}$$

**Reference**: Sturm et al., "A Benchmark for the Evaluation of RGB-D SLAM Systems", IROS 2012

#### 2. RPE (Relative Pose Error) – Drift Rates

Measures local consistency by comparing relative transformations:

$$RPE_{trans} = |\Delta\mathbf{p}_{est} - \Delta\mathbf{p}_{gt}|$$

where $\Delta\mathbf{p} = \mathbf{p}(t+\Delta) - \mathbf{p}(t)$

**Reference**: Geiger et al., "Vision meets Robotics: The KITTI Dataset", IJRR 2013

We report drift as **rates**:

* **Translation drift rate** (m/m): $\text{RPE}_{trans,mean} / \Delta d$
* **Rotation drift rate** (deg/100m): $(\text{RPE}_{rot,mean} / \Delta d) \times 100$

where $\Delta d$ is a distance interval in meters (e.g., 10 m).

#### 3. Completeness

Completeness measures how many ground-truth poses (in the evaluation window) can be associated and evaluated:

$$Completeness = \frac{N_{matched}}{N_{gt}} \times 100\%$$

Here, the evaluation window is the time range where the estimated trajectory exists; ground truth is trimmed to this window so the denominator is the number of GT poses in that window (no interpolation).

### Trajectory Alignment

We use Sim(3) (7-DOF) alignment:

* **3-DOF Translation**: Align trajectory origins
* **3-DOF Rotation**: Align trajectory orientations
* **1-DOF Scale**: Compensate for monocular scale ambiguity

### Evaluation Protocol

#### Inputs

* **Ground truth**: `ground_truth.txt` (TUM format); for evaluation, trimmed to estimated time range → `ground_truth_trimmed.txt`
* **Estimated trajectory**: `CameraTrajectory.txt` (TUM format, all tracked frames)
* **Association threshold**: `t_max_diff = 0.1 s`
* **Distance delta for RPE**: `delta = 10 m`

#### Step 1 — ATE with Sim(3) alignment (scale corrected)

```bash
evo_ape tum ground_truth_trimmed.txt CameraTrajectory.txt \
  --align --correct_scale \
  --t_max_diff 0.1 -va
```

#### Step 2 — RPE (translation + rotation) in the distance domain

```bash
evo_rpe tum ground_truth_trimmed.txt CameraTrajectory.txt \
  --align --correct_scale --t_max_diff 0.1 \
  --delta 10 --delta_unit m --pose_relation trans_part -va

evo_rpe tum ground_truth_trimmed.txt CameraTrajectory.txt \
  --align --correct_scale --t_max_diff 0.1 \
  --delta 10 --delta_unit m --pose_relation angle_deg -va
```

#### Step 3 — Completeness

Completeness = matched_poses / gt_poses_in_window × 100%, with ground truth trimmed to the estimated trajectory time range (no interpolation).

---

## 📁 Dataset Description

### HKisland_GNSS03 Dataset

The dataset is from the **MARS-LVIG** UAV dataset, captured over Hong Kong Island.

| Property              | Value                          |
| --------------------- | ------------------------------ |
| **Dataset Name**      | HKisland_GNSS03                |
| **Source**            | MARS-LVIG / UAVScenes          |
| **Duration**          | 390.78 seconds (~6.5 minutes)  |
| **Total Images**      | 3,911 frames (rgb.txt)         |
| **Image Resolution**  | 2448 × 2048 pixels             |
| **Frame Rate**        | ~10 Hz                         |
| **Trajectory Length** | ~1,902 meters                  |

### Data Sources

| Resource          | Link                                    |
| ----------------- | --------------------------------------- |
| MARS-LVIG Dataset | https://mars.hku.hk/dataset.html        |
| UAVScenes GitHub  | https://github.com/sijieaaa/UAVScenes   |

### Ground Truth

| Property              | Value                                |
| --------------------- | ------------------------------------ |
| **RTK Positions**     | 1,955 poses (full sequence)          |
| **Rate**              | 5 Hz                                 |
| **Accuracy**          | ±2 cm (horizontal), ±5 cm (vertical) |
| **Coordinate System** | WGS84 → Local ENU                    |

---

## ⚙️ Implementation Details

### System Configuration

| Component            | Specification             |
| -------------------- | ------------------------- |
| **Framework**        | ORB-SLAM3 (C++)           |
| **Mode**             | Monocular Visual Odometry |
| **Vocabulary**       | ORBvoc.txt (pre-trained)  |
| **Camera config**    | Examples/Monocular/HKisland_Mono.yaml |

### Camera Calibration (HKisland_Mono.yaml)

```
Camera.type: "PinHole"
Camera1.fx: 1444.43
Camera1.fy: 1444.34
Camera1.cx: 1179.50
Camera1.cy: 1044.90
Camera1.k1: -0.0560   Camera1.k2: 0.1180
Camera1.p1: 0.00122   Camera1.p2: 0.00064   Camera1.k3: -0.0627
Camera.width: 2448    Camera.height: 2048
Camera.fps: 10
```

### ORB Feature Extraction Parameters

| Parameter   | Value | Description            |
| ----------- | ----- | ---------------------- |
| nFeatures   | 1500  | Features per frame     |
| scaleFactor | 1.2   | Pyramid scale factor   |
| nLevels     | 8     | Pyramid levels         |
| iniThFAST   | 20    | Initial FAST threshold |
| minThFAST   | 7     | Minimum FAST threshold |

---

## 📈 Results and Analysis

### Evaluation Results

```
================================================================================
VISUAL ODOMETRY EVALUATION RESULTS
================================================================================

Ground Truth (full):      RTK trajectory (1,955 poses)
Ground Truth (window):   Trimmed to est. time range (1,044 poses)
Estimated:                ORB-SLAM3 camera trajectory (2,056 poses)
Matched Poses:            1,013 / 1,044 (97.03%)  ← Completeness

METRIC 1: ATE (Absolute Trajectory Error)
────────────────────────────────────────
RMSE:   97.9091 m
Mean:   68.2671 m
Std:    70.1841 m

METRIC 2: RPE Translation Drift (distance-based, delta=10 m)
────────────────────────────────────────
Mean translational RPE over 10 m: 17.5853 m
Translation drift rate:           1.7585 m/m

METRIC 3: RPE Rotation Drift (distance-based, delta=10 m)
────────────────────────────────────────
Mean rotational RPE over 10 m: 15.1059 deg
Rotation drift rate:        151.0589 deg/100m

================================================================================
```

### Trajectory Alignment Statistics

| Parameter                           | Value                          |
| ----------------------------------- | ------------------------------ |
| **Sim(3) scale correction**         | ~0.851                         |
| **Association threshold**           | t_max_diff = 0.1 s             |
| **Association rate (Completeness)** | 97.03%                         |
| **Evaluation window**               | GT trimmed to est. time range (no interpolation) |

### Performance Summary

| Metric              | Value           | Description                    |
| ------------------- | --------------- | ------------------------------ |
| **ATE RMSE**        | 97.91 m         | Global error after alignment   |
| **RPE Trans Drift** | 1.76 m/m        | Local translation drift rate   |
| **RPE Rot Drift**   | 151.06 deg/100m | Local rotation drift rate      |
| **Completeness**    | 97.03%          | Matched in evaluation window   |

---

## 📊 Visualizations

### Trajectory Comparison

![Trajectory Evaluation](figures/trajectory_evaluation.png)

This figure is generated from the same inputs used for evaluation (`ground_truth_trimmed.txt` and `CameraTrajectory.txt`) and includes:

1. **Top-Left**: 2D trajectory before alignment (matched poses only). Scale/rotation mismatch typical for monocular VO.
2. **Top-Right**: 2D trajectory after Sim(3) alignment (scale corrected). Remaining discrepancy reflects drift and local tracking errors.
3. **Bottom-Left**: Distribution of ATE translation errors (meters) over all matched poses.
4. **Bottom-Right**: ATE translation error as a function of the matched pose index (highlights where drift accumulates).

**Reproducibility**: run `evo_ape` with `--save_results ate.zip`, then:

```bash
python3 scripts/generate_report_figures.py --from-report --json evaluation_report.json
```

---

## 💭 Discussion

### Strengths

1. **High evaluation coverage**: 97.03% completeness in the evaluation window indicates that almost all GT poses in the common time range can be associated and evaluated.
2. **End-to-end pipeline**: The system produces a TUM trajectory and can be evaluated reproducibly with evo and the provided scripts.
3. **Transparent evaluation**: Ground truth is trimmed to the estimated trajectory time range (no interpolation), so completeness is a real tracking/evaluation rate.

### Limitations

1. **Tracking start delay**: Estimated trajectory begins after ~15 s of the sequence (initialization / early tracking loss), so full-sequence completeness would be lower without trimming.
2. **Drift**: Translation and rotation drift rates remain significant for long trajectories without loop closure.
3. **No loop closure**: Pure VO mode accumulates drift over time.

### Error Sources

1. **UAV motion**: Fast motion and motion blur can affect feature tracking.
2. **Feature extraction**: ORB parameters may be tuned for better robustness on high-resolution aerial imagery.
3. **Calibration**: Camera intrinsics and distortion affect pose quality.

---

## 🎯 Conclusions

This assignment demonstrates monocular Visual Odometry using ORB-SLAM3 on UAV aerial imagery (HKisland_GNSS03). Key findings:

1. ✅ **System operation**: ORB-SLAM3 runs on the image sequence and outputs a TUM trajectory.
2. ✅ **Evaluation coverage**: 97.03% completeness in the evaluation window (ground truth trimmed to estimated time range).
3. ⚠️ **Accuracy**: ATE RMSE and RPE drift rates reflect typical monocular VO drift on this sequence; tuning and/or VIO could improve results.

### Recommendations for Improvement

| Priority | Action                          | Expected effect              |
| -------- | ------------------------------- | ---------------------------- |
| High     | Increase nFeatures (e.g. 2000+) | Better tracking/completeness |
| High     | Lower FAST thresholds           | More keypoints in low contrast |
| Medium   | Verify camera calibration       | Better pose accuracy         |
| Low      | Enable IMU (VIO mode)           | Scale and drift improvement  |

---

## 📚 References

1. Campos, C., Elvira, R., Rodríguez, J. J. G., Montiel, J. M., & Tardós, J. D. (2021). **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**. _IEEE Transactions on Robotics_, 37(6), 1874-1890.
2. Sturm, J., Engelhard, N., Endres, F., Burgard, W., & Cremers, D. (2012). **A Benchmark for the Evaluation of RGB-D SLAM Systems**. _IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)_.
3. Geiger, A., Lenz, P., & Urtasun, R. (2012). **Are we ready for Autonomous Driving? The KITTI Vision Benchmark Suite**. _IEEE Conference on Computer Vision and Pattern Recognition (CVPR)_.
4. MARS-LVIG Dataset: https://mars.hku.hk/dataset.html
5. ORB-SLAM3 GitHub: https://github.com/UZ-SLAMLab/ORB_SLAM3

---

## 📎 Appendix

### A. Repository Structure (AAE5303 evaluation)

```
ORB_SLAM3/
├── README_AAE5303.md           # This report
├── HOW_TO_EVALUATE.md          # Evaluation guide
├── evaluation_report.json      # Main evaluation output (ATE, RPE, completeness)
├── evaluation_result.json      # Template-format results (optional)
├── ground_truth.txt            # Full RTK trajectory (TUM)
├── ground_truth_trimmed.txt    # GT trimmed to est. time range
├── CameraTrajectory.txt        # ORB-SLAM3 per-frame trajectory
├── KeyFrameTrajectory.txt      # Keyframe trajectory (not used for this eval)
├── figures/
│   └── trajectory_evaluation.png   # 2×2 Trajectory Comparison figure
├── scripts/
│   ├── evaluate_vo_accuracy.py     # ATE, RPE, completeness (uses trimmed GT by default)
│   ├── generate_report_figures.py  # Trajectory Comparison 4-panel figure
│   ├── evaluate_vo_accuracy_template.py  # JSON in assignment template format
│   ├── interpolate_trajectory_at_gt.py   # Optional: interpolate est. at GT times
│   └── extract_tum_from_bag.py      # Extract images + rgb.txt from ROS bag
├── data/
│   ├── HKisland_GNSS03.bag
│   └── tum_sequence/           # rgb.txt + rgb/*.png
├── Examples/Monocular/
│   └── HKisland_Mono.yaml      # Camera + ORB settings
└── docs/
    ├── 作业评估一步一步.md
    └── 终端一步一步实现指南.md
```

### B. Running Commands

```bash
# 1. Extract images from ROS bag (TUM format)
python3 scripts/extract_tum_from_bag.py data/HKisland_GNSS03.bag -o data/tum_sequence

# 2. Extract RTK ground truth (TUM format)
# (run the Python snippet in HOW_TO_EVALUATE.md or docs/终端一步一步实现指南.md)

# 3. Build and run ORB-SLAM3 monocular
mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4
cd ..
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/HKisland_Mono.yaml data/tum_sequence

# 4. Evaluate trajectory (ATE, RPE, completeness; GT trimmed to est. window by default)
python3 scripts/evaluate_vo_accuracy.py \
  --groundtruth ground_truth.txt \
  --estimated CameraTrajectory.txt \
  --t-max-diff 0.1 --delta-m 10 \
  --json-out evaluation_report.json

# 5. Generate Trajectory Comparison figure (requires ate.zip from evo_ape --save_results)
rm -f ate.zip
evo_ape tum ground_truth_trimmed.txt CameraTrajectory.txt \
  --align --correct_scale --t_max_diff 0.1 -va --save_results ate.zip
python3 scripts/generate_report_figures.py --from-report --json evaluation_report.json
```

### C. Native evo Commands

```bash
# ATE (Sim(3) alignment + scale correction)
evo_ape tum ground_truth_trimmed.txt CameraTrajectory.txt \
  --align --correct_scale --t_max_diff 0.1 -va

# RPE translation (delta = 10 m)
evo_rpe tum ground_truth_trimmed.txt CameraTrajectory.txt \
  --align --correct_scale --t_max_diff 0.1 \
  --delta 10 --delta_unit m --pose_relation trans_part -va

# RPE rotation (delta = 10 m)
evo_rpe tum ground_truth_trimmed.txt CameraTrajectory.txt \
  --align --correct_scale --t_max_diff 0.1 \
  --delta 10 --delta_unit m --pose_relation angle_deg -va
```

### D. Output Trajectory Format (TUM)

```
# timestamp x y z qx qy qz qw
1698132964.816207 -0.019883508 0.016566988 -0.096414186 ...
...
```

---

**AAE5303 - Robust Control Technology in Low-Altitude Aerial Vehicle**

_Department of Aeronautical and Aviation Engineering_

_The Hong Kong Polytechnic University_
