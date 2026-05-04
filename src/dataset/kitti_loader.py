import os
import glob
import cv2
import numpy as np


class KITTIRawLoader:
    def __init__(self, drive_path, camera="image_00"):
        self.drive_path = drive_path
        self.camera = camera

        self.img_path = os.path.join(self.drive_path, f"{camera}/data")
        self.oxts_path = os.path.join(self.drive_path, "oxts/data")

        self.img_files = sorted(glob.glob(self.img_path + "/*.png"))
        self.oxts_files = sorted(glob.glob(self.oxts_path + "/*.txt"))

        ts_file = os.path.join(self.drive_path, f"{camera}/timestamps.txt")
        with open(ts_file, 'r') as f:
            self.timestamps = [line.strip() for line in f.readlines()]

        assert len(self.img_files) > 0, f"No images in {self.img_path}"
        assert len(self.img_files) == len(self.oxts_files), "Image-IMU mismatch!"
        assert len(self.img_files) == len(self.timestamps), "Timestamp mismatch!"

        print(f"[INFO] Camera: {camera}")
        print(f"[INFO] Loaded {len(self.img_files)} frames")

    def __len__(self):
        return len(self.img_files)

    def load_image(self, idx):
        return cv2.imread(self.img_files[idx], cv2.IMREAD_GRAYSCALE)

    def load_imu(self, idx):
        data = np.loadtxt(self.oxts_files[idx])

        return {
            "acc": data[11:14],
            "gyro": data[17:20],
            "roll": data[3],
            "pitch": data[4],
            "yaw": data[5],
            "vf": data[8],   # forward velocity (m/s)
        }

    def get(self, idx):
        return (
            self.timestamps[idx],
            self.load_image(idx),
            self.load_imu(idx),
        )