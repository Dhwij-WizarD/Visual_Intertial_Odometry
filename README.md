# Visual-Inertial Odometry (VIO) — No ROS

This project implements a **minimal Visual Odometry (VO) pipeline with IMU support** using the KITTI raw dataset — built completely **without ROS**.

It demonstrates how camera images and inertial data can be processed from scratch to estimate motion.

---

## 🚀 Features

* Monocular Visual Odometry (VO)
* Feature tracking using Optical Flow (KLT)
* Pose estimation using Essential Matrix
* Ego-centric trajectory visualization
* IMU data loading (acceleration, gyro, velocity)
* Real-time velocity display (km/h)
* Feature visualization on image

---

## 🧠 Pipeline Overview

```
Image Sequence → Feature Detection → Feature Tracking → Essential Matrix → Pose Estimation → Trajectory
```

### Components:

* **Camera (image_00)** → Visual motion
* **IMU (OXTS)** → Velocity + future fusion
* **Calibration** → Camera intrinsics

---

## 📁 Project Structure

```
vio_no_ros/
│── data/
│   └── 2011_09_26_drive_0001_sync/
│
│── src/
│   ├── main.py
│   ├── dataset/
│   │   └── kitti_loader.py
│   ├── vo/
│   │   └── mono_vo.py
│   └── utils/
│       └── calib.py
```

---

## 📦 Requirements

Install dependencies:

```bash
pip install opencv-python numpy
```

---

## 📥 Dataset Setup

Download the **KITTI Raw Dataset** (not odometry dataset):

Place it like this:

```
data/
└── 2011_09_26_drive_0001_sync/
    ├── image_00/
    ├── oxts/
    ├── calib_cam_to_cam.txt
```

---

## ▶️ How to Run

```bash
cd src
python main.py
```

---

## 🖥️ Output

### Camera View

* Tracked features (dots)
* Vehicle speed (top-right, km/h)

### Trajectory View

* Green dots → past trajectory
* Red line → current heading
* Camera stays centered (ego-frame)

---

## ⚠️ Limitations

This is **pure monocular VO**, so:

* ❌ Scale is unknown
* ❌ Drift accumulates over time
* ❌ No global consistency

These are expected limitations.

---

## 🔥 Next Improvements

* IMU integration (true VIO)
* Scale recovery using acceleration
* Gyro-based rotation stabilization
* Sensor fusion (EKF / optimization)

---

## 🎯 Goal

This project is meant for:

* Learning VIO fundamentals
* Understanding VO geometry
* Building SLAM systems from scratch (no ROS abstraction)

---

## 👨‍💻 Notes

* KITTI images are already rectified → no distortion handling needed
* Velocity displayed is ground truth from IMU (`vf`)
* Works best with grayscale camera (`image_00`)

---

## 📌 Future Work

* Stereo VO (image_00 + image_01)
* Bundle adjustment
* Loop closure
* Full SLAM pipeline

---

## 🧭 Summary

You now have a working:

> **Minimal Visual Odometry system with IMU data pipeline (ROS-free)**

---
