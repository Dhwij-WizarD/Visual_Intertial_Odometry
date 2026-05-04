import cv2
import numpy as np

from dataset.kitti_loader import KITTIRawLoader
from vo.mono_vo import MonoVO
from utils.calib import load_kitti_intrinsics


def main():
    dataset = KITTIRawLoader(
        "../data/2011_09_26_drive_0093_sync",
        camera="image_00"
    )

    calib_file = "../data/calib_cam_to_cam.txt"
    K = load_kitti_intrinsics(calib_file)

    vo = MonoVO(K)

    trajectory_points = []

    canvas_size = 600
    center = np.array([canvas_size // 2, canvas_size // 2])
    scale = 1  # increased scale

    for i in range(len(dataset)):
        _, img, imu = dataset.get(i)

        # Velocity (safe)
        vf = imu.get("vf", 0.0)
        speed_kmph = vf * 3.6

        if i == 0:
            vo.process_first_frame(img)
            continue

        R, t, pts = vo.process_frame(img)
        trajectory_points.append(t.copy())

        traj_img = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8)

        R_curr = vo.R
        t_curr = vo.t

        # Draw trajectory in ego-frame
        for p in trajectory_points:
            p_rel = R_curr.T @ (p - t_curr)

            x_rel = p_rel[0][0]
            z_rel = p_rel[2][0]

            draw_x = int(x_rel * scale) + center[0]
            draw_y = int(-z_rel * scale) + center[1]

            if 0 <= draw_x < canvas_size and 0 <= draw_y < canvas_size:
                cv2.circle(traj_img, (draw_x, draw_y), 1, (0, 255, 0), 2)

        # Heading direction
        forward = R_curr @ np.array([[0], [0], [1]])
        fx = int(forward[0][0] * 20 + center[0])
        fy = int(-forward[2][0] * 20 + center[1])

        cv2.line(traj_img, tuple(center), (fx, fy), (0, 0, 255), 2)

        # Draw velocity text
        text = f"{speed_kmph:.1f} km/h"

        (text_w, text_h), _ = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2
        )

        x = traj_img.shape[1] - text_w - 10
        y = 30

        cv2.putText(
            traj_img,
            text,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

        for pt in pts:
            x, y = int(pt[0]), int(pt[1])
            cv2.circle(img, (x, y), 1, (255, 0, 0), -1)

        # Show
        cv2.imshow("Camera", img)
        cv2.imshow("Trajectory (Ego Frame)", traj_img)

        key = cv2.waitKey(30)
        if key == 27:
            break

    print("Done. Press any key to exit...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()