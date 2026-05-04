import cv2
import numpy as np


class MonoVO:
    def __init__(self, K):
        self.K = K

        self.prev_img = None
        self.prev_pts = None

        self.R = np.eye(3)
        self.t = np.zeros((3, 1))

    def detect_features(self, img):
        return cv2.goodFeaturesToTrack(
            img,
            maxCorners=2000,
            qualityLevel=0.01,
            minDistance=7
        )

    def process_first_frame(self, img):
        self.prev_img = img
        self.prev_pts = self.detect_features(img)

    def process_frame(self, img):
        # Track features
        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_img, img, self.prev_pts, None
        )

        # Filter valid points
        good_prev = self.prev_pts[status == 1]
        good_curr = curr_pts[status == 1]

        # Estimate Essential Matrix
        E, mask = cv2.findEssentialMat(
            good_curr,
            good_prev,
            self.K,
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.0
        )

        # Apply RANSAC mask
        good_prev = good_prev[mask.ravel() == 1]
        good_curr = good_curr[mask.ravel() == 1]

        # Recover pose
        _, R, t, _ = cv2.recoverPose(
            E,
            good_curr,
            good_prev,
            self.K
        )

        # Accumulate pose
        self.t += self.R @ t
        self.R = R @ self.R

        # Re-detect features if too few
        if len(good_curr) < 1000:
            self.prev_pts = self.detect_features(img)
        else:
            self.prev_pts = good_curr.reshape(-1, 1, 2)

        self.prev_img = img

        return self.R, self.t, good_curr