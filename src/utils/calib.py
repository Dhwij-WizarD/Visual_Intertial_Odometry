import numpy as np


def load_kitti_intrinsics(calib_file, cam_id="P_rect_00"):
    with open(calib_file, 'r') as f:
        lines = f.readlines()

    for line in lines:
        if line.startswith(cam_id):
            parts = line.replace(":", "").split()[1:]
            values = list(map(float, parts))

            P = np.array(values).reshape(3, 4)
            K = P[:, :3]
            return K

    raise ValueError(f"{cam_id} not found in calibration file")