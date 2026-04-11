#!/usr/bin/env python3
import os

import numpy as np
from scipy.io import savemat
from scipy.ndimage import distance_transform_cdt


def build_line_gray_matrix(
    rows: int = 33,
    cols: int = 40,
    line_rows: int = 3,
    line_cols: int = 10,
    gray_levels: int = 15,
) -> np.ndarray:
    if rows <= 0 or cols <= 0:
        raise ValueError("rows/cols must be positive")
    if line_rows <= 0 or line_cols <= 0:
        raise ValueError("line_rows/line_cols must be positive")
    if line_rows > rows or line_cols > cols:
        raise ValueError("line bbox must fit inside the canvas")
    if gray_levels <= 0:
        raise ValueError("gray_levels must be positive")

    binary = np.ones((rows, cols), dtype=np.uint8)
    r0 = (rows - line_rows) // 2
    c0 = (cols - line_cols) // 2
    binary[r0:r0 + line_rows, c0:c0 + line_cols] = 0

    # Match the existing rectangle MAT style: 3x3-mask iterative grayscale
    # expansion, equivalent to chessboard distance normalized by h.
    dist = distance_transform_cdt(binary, metric="chessboard").astype(np.float64)
    gray = np.minimum(dist / float(gray_levels), 1.0)
    gray[binary == 0] = 0.0
    return gray


def main() -> None:
    out_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "shape_images"))
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "Image_line.mat")
    gray_mtr = build_line_gray_matrix()
    savemat(out_path, {"gray_mtr": gray_mtr})
    black_count = int(np.sum(np.abs(gray_mtr) <= 1e-9))
    print(f"wrote {out_path}")
    print(f"shape={gray_mtr.shape} black_count={black_count} gray_levels={len(np.unique(gray_mtr))}")


if __name__ == "__main__":
    main()
