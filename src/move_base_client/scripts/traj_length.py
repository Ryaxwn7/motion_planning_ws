#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path


def point_distance(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def read_points(csv_file):
    points = []
    with csv_file.open(newline="") as f:
        reader = csv.DictReader(f)
        required_fields = {"x", "y", "z"}
        if not reader.fieldnames or not required_fields.issubset(reader.fieldnames):
            raise ValueError(f"{csv_file} missing x/y/z columns")

        for row in reader:
            points.append((float(row["x"]), float(row["y"]), float(row["z"])))
    return points


def path_length(points):
    return sum(point_distance(points[i - 1], points[i]) for i in range(1, len(points)))


def iter_csv_files(path):
    if path.is_dir():
        return sorted(path.glob("*_traj.csv"))
    return [path]


def main():
    parser = argparse.ArgumentParser(description="Calculate total path length from saved VRPN trajectory CSV files.")
    parser.add_argument("path", help="A trajectory CSV file or a directory containing *_traj.csv files.")
    args = parser.parse_args()

    input_path = Path(args.path).expanduser()
    if not input_path.exists():
        raise SystemExit(f"not found: {input_path}")

    files = iter_csv_files(input_path)
    if not files:
        raise SystemExit(f"no *_traj.csv files found in {input_path}")

    total_length = 0.0
    for csv_file in files:
        points = read_points(csv_file)
        length = path_length(points)
        total_length += length
        print(f"{csv_file}: poses={len(points)} length={length:.6f}")

    print(f"total_length={total_length:.6f}")


if __name__ == "__main__":
    main()
