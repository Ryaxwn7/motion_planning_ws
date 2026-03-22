#!/usr/bin/env python3
import argparse
import math
import os
import random
import re
import sys
import time
from typing import Dict, List, Optional, Tuple

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

try:
    import tf2_ros
except Exception:
    tf2_ros = None

try:
    from dynamic_reconfigure.server import Server as DynamicReconfigureServer
    from sim_env.cfg import ShapeAssemblySwarmConfig
except Exception:
    DynamicReconfigureServer = None
    ShapeAssemblySwarmConfig = None

try:
    from formation_msgs.msg import RobotFormationStatus, ShapeTask
except Exception:
    RobotFormationStatus = None
    ShapeTask = None


def _sign(x: float) -> float:
    if x > 0:
        return 1.0
    if x < 0:
        return -1.0
    return 0.0


def _limit_angle(angle: float) -> float:
    if angle > math.pi:
        angle -= 2.0 * math.pi
    if angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _weight_function(x: float, r: float, s: float) -> float:
    if x < 2.0 * s:
        return 1.0
    if x > r:
        return 0.0
    return (1.0 + math.cos(math.pi * (x - 2.0 * s) / (r - 2.0 * s))) / 2.0


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class SimParam:
    def __init__(self) -> None:
        self.time = 20.0
        self.t = 0.05
        self.max_step = 0
        self.r_body = 0.2
        self.r_safe = 0.35
        self.r_avoid = 1.5
        self.r_sense = 2.5
        self.vel_max = 0.6
        self.swarm_size = 0
        self.leader_fraction = 0.2

        # negotiation params
        self.kappa_conse_pos = 1.6
        self.kappa_track_pos = 3.0
        self.kappa_conse_head = 1.6
        self.kappa_track_head = 3.0
        self.alpha = 0.8
        self.hvel_max = math.pi / 2.0

        # entering/exploration/interaction gains
        self.kappa_enter = 10.0
        self.kappa_explore_1 = 5.0
        self.kappa_explore_2 = 15.0
        self.kappa_avoid = 25.0
        self.kappa_hard_avoid = 25.0
        self.kappa_consensus = 1.0

        # command smoothing
        self.cmd_smooth_ratio = 0.1
        self.cmd_scale = 1.0
        self.shape_vel_max = 1.2


class RobotState:
    def __init__(self, swarm_size: int) -> None:
        self.pos_x = [0.0] * swarm_size
        self.pos_y = [0.0] * swarm_size
        self.vel_x = [0.0] * swarm_size
        self.vel_y = [0.0] * swarm_size
        self.yaw = [0.0] * swarm_size


class ReferState:
    def __init__(self, pos_x: float, pos_y: float) -> None:
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.head = 0.0
        self.hvel = 0.0


class ShapeState:
    def __init__(self, swarm_size: int) -> None:
        self.pos_x = [0.0] * swarm_size
        self.pos_y = [0.0] * swarm_size
        self.vel_x = [0.0] * swarm_size
        self.vel_y = [0.0] * swarm_size
        self.head = [0.0] * swarm_size
        self.hvel = [0.0] * swarm_size


class ShapeInfo:
    def __init__(self) -> None:
        self.rn = 0
        self.cn = 0
        self.cen_x = 0
        self.cen_y = 0
        self.black_num = 0
        self.gray_num = 0
        self.gray_scale = 1.0
        self.grid = 1.0


class ShapeMatrix:
    def __init__(self) -> None:
        self.base_x: List[List[float]] = []
        self.base_y: List[List[float]] = []
        self.value: List[List[float]] = []


class ShapeDyn:
    def __init__(self, swarm_size: int) -> None:
        self.shape_x: List[List[List[float]]] = [[] for _ in range(swarm_size)]
        self.shape_y: List[List[List[float]]] = [[] for _ in range(swarm_size)]
        self.value: List[List[float]] = []


class SwarmMetric:
    def __init__(self) -> None:
        self.cover_rate = 0.0
        self.enter_rate = 0.0
        self.inside_rate = 0.0
        self.dist_var = 0.0
        self.vel_align = 0.0
        self.neigh_mean = 0.0
        self.cmd_sat_rate = 0.0
        self.shape_center_err = 0.0
        self.min_pair_dist = 0.0
        self.collision_pairs = 0


class ControlStrategy:
    # Always publish shape-assembly command to robots.
    SHAPE_ONLY = "shape_only"
    # Keep move_base active first, then switch per robot to shape control.
    MOVE_BASE_THEN_SHAPE = "move_base_then_shape"


def _normalize_control_strategy(strategy: str) -> str:
    # Accept legacy/short aliases from launch or CLI.
    key = str(strategy).strip().lower()
    alias = {
        "shape": ControlStrategy.SHAPE_ONLY,
        "shape_only": ControlStrategy.SHAPE_ONLY,
        "move_base_then_shape": ControlStrategy.MOVE_BASE_THEN_SHAPE,
        "movebase_then_shape": ControlStrategy.MOVE_BASE_THEN_SHAPE,
        "hybrid": ControlStrategy.MOVE_BASE_THEN_SHAPE,
    }
    return alias.get(key, key)


class ShapeTargetMode:
    # Target shape follows each robot's negotiated local frame (legacy).
    NEGOTIATED = "negotiated"
    # Target shape is anchored to one shared reference center/heading.
    REFERENCE = "reference"


def _normalize_shape_target_mode(mode: str) -> str:
    key = str(mode).strip().lower()
    alias = {
        "negotiated": ShapeTargetMode.NEGOTIATED,
        "reference": ShapeTargetMode.REFERENCE,
        "fixed": ShapeTargetMode.REFERENCE,
    }
    return alias.get(key, key)


def _normalize_shape_type(shape_type: str) -> str:
    key = str(shape_type).strip().lower()
    alias = {
        "ring": "ring",
        "o": "letter_o",
        "letter": "letter_o",
        "letter_o": "letter_o",
        "r": "letter_r",
        "letter_r": "letter_r",
        "b": "letter_b",
        "letter_b": "letter_b",
        "snow": "snowflake",
        "snowflake": "snowflake",
        "geom_snow": "snowflake",
        "star": "starfish",
        "starfish": "starfish",
        "geom_starfish": "starfish",
        "rect": "rectangle",
        "rectangle": "rectangle",
        "triangle": "triangle",
        "sphere": "sphere",
        "sph": "sphere",
    }
    return alias.get(key, key)


def _get_default_shape_dir() -> str:
    return os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "../shape_images",
        )
    )


def _resolve_shape_mat_path(shape_type: str, shape_mat_path: str, shape_library_root: str) -> str:
    if shape_mat_path:
        expanded = os.path.expanduser(shape_mat_path)
        if os.path.isfile(expanded):
            return expanded
    shape_key = _normalize_shape_type(shape_type)
    file_map = {
        "letter_o": "Image_letter_O.mat",
        "letter_r": "Image_letter_R.mat",
        "letter_b": "Image_letter_B.mat",
        "starfish": "Image_geom_starfish.mat",
        "snowflake": "Image_geom_snow.mat",
        "rectangle": "Image_rectangle.mat",
        "triangle": "Image_triangle.mat",
        "sphere": "Image_sphere.mat"
    }
    file_name = file_map.get(shape_key, "")
    if not file_name:
        return ""
    root = shape_library_root if shape_library_root else _get_default_shape_dir()
    candidate = os.path.join(os.path.expanduser(root), file_name)
    if os.path.isfile(candidate):
        return candidate
    return ""


def _load_mat_shape_matrix(shape_mat_path: str) -> Tuple[Optional[List[List[float]]], str]:
    if not shape_mat_path:
        return None, "shape_mat_path is empty"
    try:
        import scipy.io  # type: ignore
    except Exception as exc:  # pragma: no cover - depends on host env
        return None, f"scipy.io unavailable: {exc}"
    try:
        data = scipy.io.loadmat(shape_mat_path)
    except Exception as exc:  # pragma: no cover - depends on runtime file
        return None, f"failed to load mat file {shape_mat_path}: {exc}"
    if "gray_mtr" not in data:
        return None, f"gray_mtr not found in {shape_mat_path}"

    gray_mtr = data["gray_mtr"]
    if len(gray_mtr.shape) != 2:
        return None, f"gray_mtr expected 2D but got shape={gray_mtr.shape}"

    matrix: List[List[float]] = []
    for row in gray_mtr.tolist():
        matrix.append([float(v) for v in row])
    return matrix, f"mat:{shape_mat_path}"


def load_shape_matrix(
    shape_source: str,
    shape_type: str,
    shape_resolution: int,
    ring_inner_ratio: float,
    ring_outer_ratio: float,
    gray_width: int,
    shape_mat_path: str = "",
    shape_library_root: str = "",
) -> Tuple[List[List[float]], str]:
    source = str(shape_source).strip().lower()
    shape_key = _normalize_shape_type(shape_type)

    if source == "mat" or shape_key != "ring":
        mat_path = _resolve_shape_mat_path(shape_key, shape_mat_path, shape_library_root)
        image_mtr, desc = _load_mat_shape_matrix(mat_path)
        if image_mtr is not None:
            return image_mtr, desc
        if source == "mat":
            raise RuntimeError(desc)
        # Keep compatibility: when analytic + non-ring type fails loading,
        # fall back to ring to avoid hard crash.
        return generate_ring_shape(shape_resolution, ring_inner_ratio, ring_outer_ratio, gray_width), f"fallback_ring({desc})"

    return generate_ring_shape(shape_resolution, ring_inner_ratio, ring_outer_ratio, gray_width), "analytic:ring"


def generate_ring_shape(resolution: int, inner_ratio: float, outer_ratio: float, gray_width: int) -> List[List[float]]:
    center = (resolution - 1) / 2.0
    inner_r = max(1.0, inner_ratio * (resolution / 2.0))
    outer_r = max(inner_r + 1.0, outer_ratio * (resolution / 2.0))
    gray_width = max(1, int(gray_width))
    value = [[1.0 for _ in range(resolution)] for _ in range(resolution)]
    for r in range(resolution):
        for c in range(resolution):
            dx = c - center
            dy = center - r
            dist = math.sqrt(dx * dx + dy * dy)
            if inner_r <= dist <= outer_r:
                value[r][c] = 0.0
            elif inner_r - gray_width <= dist < inner_r:
                value[r][c] = (inner_r - dist) / gray_width
            elif outer_r < dist <= outer_r + gray_width:
                value[r][c] = (dist - outer_r) / gray_width
            else:
                value[r][c] = 1.0
    return value


def init_form_shape(sim_param: SimParam, image_mtr: List[List[float]]) -> Tuple[ShapeMatrix, ShapeInfo]:
    info = ShapeInfo()
    info.rn = len(image_mtr)
    info.cn = len(image_mtr[0]) if info.rn > 0 else 0
    info.cen_x = int(math.ceil(info.cn / 2.0))
    info.cen_y = int(math.ceil(info.rn / 2.0))

    black_num = 0
    gray_num = 0
    for r in range(info.rn):
        for c in range(info.cn):
            v = image_mtr[r][c]
            if v == 0:
                black_num += 1
            if v < 1.0:
                gray_num += 1
    info.black_num = black_num
    info.gray_num = max(0, gray_num - black_num)

    if info.black_num <= 0:
        rospy.logwarn("Shape matrix has no black cells; using 1 to avoid division by zero.")
        info.black_num = 1

    temp_row = image_mtr[info.cen_y - 1]
    temp = [2.0 if v == 0 else v for v in temp_row]
    info.gray_scale = 1.0 / min(temp) if temp else 1.0

    info.grid = math.sqrt((math.pi / 4.0) * (sim_param.swarm_size / info.black_num)) * sim_param.r_avoid

    shape = ShapeMatrix()
    shape.value = image_mtr
    shape.base_x = [[0.0 for _ in range(info.cn)] for _ in range(info.rn)]
    shape.base_y = [[0.0 for _ in range(info.cn)] for _ in range(info.rn)]

    for r in range(info.rn):
        y = (info.rn - r - info.cen_y) * info.grid
        for c in range(info.cn):
            x = (c + 1 - info.cen_x) * info.grid
            shape.base_x[r][c] = x
            shape.base_y[r][c] = y

    return shape, info


def get_dyn_formation(sim_param: SimParam, shape_mtr: ShapeMatrix, shape_info: ShapeInfo, shape_state: ShapeState) -> ShapeDyn:
    dyn = ShapeDyn(sim_param.swarm_size)
    dyn.value = shape_mtr.value
    for i in range(sim_param.swarm_size):
        cos_h = math.cos(shape_state.head[i])
        sin_h = math.sin(shape_state.head[i])
        shape_x = []
        shape_y = []
        for r in range(shape_info.rn):
            row_x = []
            row_y = []
            for c in range(shape_info.cn):
                base_x = shape_mtr.base_x[r][c]
                base_y = shape_mtr.base_y[r][c]
                rx = base_x * cos_h - base_y * sin_h + shape_state.pos_x[i]
                ry = base_x * sin_h + base_y * cos_h + shape_state.pos_y[i]
                row_x.append(rx)
                row_y.append(ry)
            shape_x.append(row_x)
            shape_y.append(row_y)
        dyn.shape_x[i] = shape_x
        dyn.shape_y[i] = shape_y
    return dyn


def get_neighbor_set(sim_param: SimParam, robot_state: RobotState) -> Dict[str, List[List[float]]]:
    n = sim_param.swarm_size
    a_mtr = [[0 for _ in range(n)] for _ in range(n)]
    d_mtr = [[0.0 for _ in range(n)] for _ in range(n)]
    rx_mtr = [[0.0 for _ in range(n)] for _ in range(n)]
    ry_mtr = [[0.0 for _ in range(n)] for _ in range(n)]
    for i in range(n):
        xi = robot_state.pos_x[i]
        yi = robot_state.pos_y[i]
        for j in range(n):
            if i == j:
                continue
            dx = robot_state.pos_x[j] - xi
            dy = robot_state.pos_y[j] - yi
            dist = math.hypot(dx, dy)
            d_mtr[i][j] = dist
            if dist <= sim_param.r_sense:
                a_mtr[i][j] = 1
                rx_mtr[i][j] = dx
                ry_mtr[i][j] = dy
    return {"a_mtr": a_mtr, "d_mtr": d_mtr, "rx_mtr": rx_mtr, "ry_mtr": ry_mtr}


def negotiate_position(sim_param: SimParam, neigh: Dict[str, List[List[float]]], shape_state: ShapeState, refer_state: ReferState, inform_index: List[int]) -> ShapeState:
    n = sim_param.swarm_size
    a_mtr = neigh["a_mtr"]

    err_x = [0.0] * n
    err_y = [0.0] * n
    for i in range(n):
        sum_x = 0.0
        sum_y = 0.0
        neigh_num = 0
        for j in range(n):
            if a_mtr[i][j] == 0:
                continue
            sum_x += (shape_state.pos_x[j] - shape_state.pos_x[i])
            sum_y += (shape_state.pos_y[j] - shape_state.pos_y[i])
            neigh_num += 1
        if neigh_num == 0:
            neigh_num = 1
        err_x[i] = sum_x / neigh_num
        err_y[i] = sum_y / neigh_num

    cmd_cons_x = []
    cmd_cons_y = []
    for i in range(n):
        cmd_cons_x.append(sim_param.kappa_conse_pos * _sign(err_x[i]) * (abs(err_x[i]) ** sim_param.alpha))
        cmd_cons_y.append(sim_param.kappa_conse_pos * _sign(err_y[i]) * (abs(err_y[i]) ** sim_param.alpha))

    cmd_track_x = [sim_param.kappa_track_pos * (refer_state.pos_x - shape_state.pos_x[i]) + refer_state.vel_x for i in range(n)]
    cmd_track_y = [sim_param.kappa_track_pos * (refer_state.pos_y - shape_state.pos_y[i]) + refer_state.vel_y for i in range(n)]

    cmd_align_x = [0.0] * n
    cmd_align_y = [0.0] * n
    for i in range(n):
        sum_x = 0.0
        sum_y = 0.0
        neigh_num = 0
        for j in range(n):
            if a_mtr[i][j] == 0:
                continue
            sum_x += shape_state.vel_x[j]
            sum_y += shape_state.vel_y[j]
            neigh_num += 1
        if neigh_num == 0:
            neigh_num = 1
        cmd_align_x[i] = sum_x / neigh_num
        cmd_align_y[i] = sum_y / neigh_num

    inform_set = [0] * n
    for idx in inform_index:
        if 0 <= idx < n:
            inform_set[idx] = 1

    new_state = ShapeState(n)
    for i in range(n):
        cmd_x = -cmd_cons_x[i] + inform_set[i] * cmd_track_x[i] + (1 - inform_set[i]) * cmd_align_x[i]
        cmd_y = -cmd_cons_y[i] + inform_set[i] * cmd_track_y[i] + (1 - inform_set[i]) * cmd_align_y[i]
        if sim_param.shape_vel_max > 0:
            cmd_speed = math.hypot(cmd_x, cmd_y)
            if cmd_speed > sim_param.shape_vel_max and cmd_speed > 1e-9:
                scale = sim_param.shape_vel_max / cmd_speed
                cmd_x *= scale
                cmd_y *= scale
        new_state.pos_x[i] = shape_state.pos_x[i] + shape_state.vel_x[i] * sim_param.t
        new_state.pos_y[i] = shape_state.pos_y[i] + shape_state.vel_y[i] * sim_param.t
        new_state.vel_x[i] = cmd_x
        new_state.vel_y[i] = cmd_y
    new_state.head = list(shape_state.head)
    new_state.hvel = list(shape_state.hvel)
    return new_state


def negotiate_orientation(sim_param: SimParam, neigh: Dict[str, List[List[float]]], shape_state: ShapeState, refer_state: ReferState, inform_index: List[int]) -> ShapeState:
    n = sim_param.swarm_size
    a_mtr = neigh["a_mtr"]

    err = [0.0] * n
    for i in range(n):
        sum_h = 0.0
        neigh_num = 0
        for j in range(n):
            if a_mtr[i][j] == 0:
                continue
            sum_h += (shape_state.head[j] - shape_state.head[i])
            neigh_num += 1
        if neigh_num == 0:
            neigh_num = 1
        err[i] = sum_h / neigh_num

    cmd_cons = [sim_param.kappa_conse_head * _sign(err[i]) * (abs(err[i]) ** sim_param.alpha) for i in range(n)]
    cmd_track = [sim_param.kappa_track_head * (refer_state.head - shape_state.head[i]) + refer_state.hvel for i in range(n)]

    cmd_align = [0.0] * n
    for i in range(n):
        sum_h = 0.0
        neigh_num = 0
        for j in range(n):
            if a_mtr[i][j] == 0:
                continue
            sum_h += shape_state.hvel[j]
            neigh_num += 1
        if neigh_num == 0:
            neigh_num = 1
        cmd_align[i] = sum_h / neigh_num

    inform_set = [0] * n
    for idx in inform_index:
        if 0 <= idx < n:
            inform_set[idx] = 1

    new_state = ShapeState(n)
    new_state.pos_x = list(shape_state.pos_x)
    new_state.pos_y = list(shape_state.pos_y)
    new_state.vel_x = list(shape_state.vel_x)
    new_state.vel_y = list(shape_state.vel_y)

    new_head = [0.0] * n
    new_hvel = [0.0] * n
    for i in range(n):
        cmd = -cmd_cons[i] + inform_set[i] * cmd_track[i] + (1 - inform_set[i]) * cmd_align[i]
        if cmd > sim_param.hvel_max:
            cmd = sim_param.hvel_max
        if cmd < -sim_param.hvel_max:
            cmd = -sim_param.hvel_max
        new_head[i] = shape_state.head[i] + shape_state.hvel[i] * sim_param.t
        new_hvel[i] = cmd

    new_state.head = new_head
    new_state.hvel = new_hvel
    return new_state


def trans_goal_to_local(pos_x: float, pos_y: float, fpos_x: float, fpos_y: float, fhead: float, grid: float, cen_x: int, cen_y: int, rn: int, cn: int) -> Tuple[int, int, int]:
    head = math.atan2(pos_y - fpos_y, pos_x - fpos_x)
    azim = _limit_angle(head - fhead)
    dist = math.hypot(pos_x - fpos_x, pos_y - fpos_y)
    x_coor = int(round(dist * math.cos(azim) / grid))
    y_coor = int(round(dist * math.sin(azim) / grid))

    x_grid = x_coor + cen_x
    y_grid = rn - (y_coor + cen_y) + 1

    inside = 1 if (1 <= x_grid <= cn and 1 <= y_grid <= rn) else 0
    row = y_grid - 1
    col = x_grid - 1
    return row, col, inside


def get_gray_value(location: Tuple[int, int, int], shape_value: List[List[float]]) -> float:
    row, col, inside = location
    if inside == 0:
        return 1.0
    return shape_value[row][col]


def get_local_target_out(location: Tuple[int, int, int], pos_x: float, pos_y: float, shape_x: List[List[float]], shape_y: List[List[float]], shape_value: List[List[float]]) -> Tuple[float, float]:
    rn = len(shape_value)
    cn = len(shape_value[0]) if rn > 0 else 0
    min_dist = float("inf")
    goal_x = pos_x
    goal_y = pos_y

    for r in range(rn):
        for c in range(cn):
            if shape_value[r][c] >= 1.0:
                continue
            if location[2] and r == location[0] and c == location[1]:
                continue
            dx = shape_x[r][c] - pos_x
            dy = shape_y[r][c] - pos_y
            dist = math.hypot(dx, dy)
            if dist < min_dist:
                min_dist = dist
                goal_x = shape_x[r][c]
                goal_y = shape_y[r][c]
    return goal_x, goal_y


def get_local_target_in(location: Tuple[int, int, int], pos_x: float, pos_y: float, shape_x: List[List[float]], shape_y: List[List[float]], shape_value: List[List[float]], rn: int, cn: int, grid: float, range_cells: int) -> Tuple[float, float]:
    row, col, inside = location
    if inside == 0:
        return pos_x, pos_y
    gray_value = shape_value[row][col]
    if gray_value <= 0.0:
        return pos_x, pos_y

    min_r = max(0, row - range_cells)
    max_r = min(rn - 1, row + range_cells)
    min_c = max(0, col - range_cells)
    max_c = min(cn - 1, col + range_cells)

    min_dist = float("inf")
    goal_x = pos_x
    goal_y = pos_y
    for r in range(min_r, max_r + 1):
        for c in range(min_c, max_c + 1):
            if shape_value[r][c] >= gray_value:
                continue
            dx = shape_x[r][c] - pos_x
            dy = shape_y[r][c] - pos_y
            dist = math.hypot(dx, dy)
            if dist == 0:
                continue
            if dist < min_dist:
                min_dist = dist
                goal_x = shape_x[r][c]
                goal_y = shape_y[r][c]
    return goal_x, goal_y


def cal_entering_cmd(
    sim_param: SimParam,
    shape_dyn: ShapeDyn,
    shape_info: ShapeInfo,
    robot_state: RobotState,
    shape_state: ShapeState,
) -> Tuple[List[float], List[float], List[Tuple[int, int]], List[Tuple[float, float, float]]]:
    n = sim_param.swarm_size
    cmd_x = [0.0] * n
    cmd_y = [0.0] * n
    grid_set = [(0, 0)] * n
    enter_goals: List[Tuple[float, float, float]] = [(0.0, 0.0, 1.0)] * n

    for i in range(n):
        location = trans_goal_to_local(
            robot_state.pos_x[i],
            robot_state.pos_y[i],
            shape_state.pos_x[i],
            shape_state.pos_y[i],
            shape_state.head[i],
            shape_info.grid,
            shape_info.cen_x,
            shape_info.cen_y,
            shape_info.rn,
            shape_info.cn,
        )
        gray_color = get_gray_value(location, shape_dyn.value)
        shape_x = shape_dyn.shape_x[i]
        shape_y = shape_dyn.shape_y[i]
        if gray_color == 1.0:
            goal_x, goal_y = get_local_target_out(location, robot_state.pos_x[i], robot_state.pos_y[i], shape_x, shape_y, shape_dyn.value)
        else:
            delta = sim_param.vel_max * sim_param.t
            range_cells = int(math.ceil(delta / shape_info.grid)) + 3
            goal_x, goal_y = get_local_target_in(location, robot_state.pos_x[i], robot_state.pos_y[i], shape_x, shape_y, shape_dyn.value, shape_info.rn, shape_info.cn, shape_info.grid, range_cells)
        grid_set[i] = (location[0], location[1])
        enter_goals[i] = (goal_x, goal_y, gray_color)

        dx = goal_x - robot_state.pos_x[i]
        dy = goal_y - robot_state.pos_y[i]
        dist = math.hypot(dx, dy)
        if dist <= 0:
            dist = 1.0
        # MATLAB implementation scales entering force by gray value (0..1)
        # so entering force vanishes in black region and is weaker in gray.
        scale = gray_color
        cmd_x[i] = sim_param.kappa_enter * dx * scale / dist + shape_state.vel_x[i]
        cmd_y[i] = sim_param.kappa_enter * dy * scale / dist + shape_state.vel_y[i]
    return cmd_x, cmd_y, grid_set, enter_goals


def get_mean_point_fill(grid_pos: Tuple[int, int], pos_x: float, pos_y: float, shape_x: List[List[float]], shape_y: List[List[float]], shape_value: List[List[float]], rn: int, cn: int, grid: float, neigh_range: int) -> Tuple[float, float]:
    row, col = grid_pos
    min_r = max(0, row - neigh_range)
    max_r = min(rn - 1, row + neigh_range)
    min_c = max(0, col - neigh_range)
    max_c = min(cn - 1, col + neigh_range)

    sum_w = 0.0
    sum_x = 0.0
    sum_y = 0.0
    for r in range(min_r, max_r + 1):
        for c in range(min_c, max_c + 1):
            if shape_value[r][c] != 0:
                continue
            dx = shape_x[r][c] - pos_x
            dy = shape_y[r][c] - pos_y
            dist = math.hypot(dx, dy)
            w = _weight_function(dist, grid * neigh_range, 0.0)
            if w <= 0.0:
                continue
            sum_w += w
            sum_x += shape_x[r][c] * w
            sum_y += shape_y[r][c] * w
    if sum_w == 0.0:
        return pos_x, pos_y
    return sum_x / sum_w, sum_y / sum_w


def get_mean_point_expl(index: int, grid_pos: Tuple[int, int], pos_x: List[float], pos_y: List[float], shape_x: List[List[float]], shape_y: List[List[float]], shape_value: List[List[float]], rn: int, cn: int, cen_x: int, cen_y: int, grid: float, neigh_range: int, robot_range: int, a_row: List[int], fpos_x: float, fpos_y: float, fhead: float) -> Tuple[float, float]:
    valid_mtr = [[1 if shape_value[r][c] > 0 else 0 for c in range(cn)] for r in range(rn)]

    for j, connected in enumerate(a_row):
        if connected == 0:
            continue
        location = trans_goal_to_local(pos_x[j], pos_y[j], fpos_x, fpos_y, fhead, grid, cen_x, cen_y, rn, cn)
        if location[2] == 0:
            continue
        row, col, _ = location
        min_r = max(0, row - robot_range)
        max_r = min(rn - 1, row + robot_range)
        min_c = max(0, col - robot_range)
        max_c = min(cn - 1, col + robot_range)
        for r in range(min_r, max_r + 1):
            for c in range(min_c, max_c + 1):
                valid_mtr[r][c] = 1

    row, col = grid_pos
    min_r = max(0, row - neigh_range)
    max_r = min(rn - 1, row + neigh_range)
    min_c = max(0, col - neigh_range)
    max_c = min(cn - 1, col + neigh_range)

    sum_w = 0.0
    sum_x = 0.0
    sum_y = 0.0
    for r in range(min_r, max_r + 1):
        for c in range(min_c, max_c + 1):
            if valid_mtr[r][c] != 0:
                continue
            dx = shape_x[r][c] - pos_x[index]
            dy = shape_y[r][c] - pos_y[index]
            dist = math.hypot(dx, dy)
            w = _weight_function(dist, grid * neigh_range, 0.0)
            if w <= 0.0:
                continue
            sum_w += w
            sum_x += shape_x[r][c] * w
            sum_y += shape_y[r][c] * w
    if sum_w == 0.0:
        return pos_x[index], pos_y[index]
    return sum_x / sum_w, sum_y / sum_w


def cal_exploration_cmd(sim_param: SimParam, shape_dyn: ShapeDyn, shape_info: ShapeInfo, grid_set: List[Tuple[int, int]], neigh: Dict[str, List[List[float]]], robot_state: RobotState, shape_state: ShapeState) -> Tuple[List[float], List[float]]:
    n = sim_param.swarm_size
    cmd_x = [0.0] * n
    cmd_y = [0.0] * n
    robot_range = max(1, int(math.floor(max(sim_param.r_avoid / 2.0, sim_param.r_body * 2.0) / shape_info.grid)))
    neigh_range = int(math.ceil(sim_param.r_sense / shape_info.grid))

    for i in range(n):
        shape_x = shape_dyn.shape_x[i]
        shape_y = shape_dyn.shape_y[i]
        gf_x, gf_y = get_mean_point_fill(grid_set[i], robot_state.pos_x[i], robot_state.pos_y[i], shape_x, shape_y, shape_dyn.value, shape_info.rn, shape_info.cn, shape_info.grid, neigh_range)
        ge_x, ge_y = get_mean_point_expl(i, grid_set[i], robot_state.pos_x, robot_state.pos_y, shape_x, shape_y, shape_dyn.value, shape_info.rn, shape_info.cn, shape_info.cen_x, shape_info.cen_y, shape_info.grid, neigh_range, robot_range, neigh["a_mtr"][i], shape_state.pos_x[i], shape_state.pos_y[i], shape_state.head[i])
        cmd_x[i] = sim_param.kappa_explore_1 * (gf_x - robot_state.pos_x[i]) + sim_param.kappa_explore_2 * (ge_x - robot_state.pos_x[i])
        cmd_y[i] = sim_param.kappa_explore_1 * (gf_y - robot_state.pos_y[i]) + sim_param.kappa_explore_2 * (ge_y - robot_state.pos_y[i])
    return cmd_x, cmd_y


def cal_interaction_cmd(sim_param: SimParam, neigh: Dict[str, List[List[float]]], robot_state: RobotState) -> Tuple[List[float], List[float]]:
    n = sim_param.swarm_size
    a_mtr = neigh["a_mtr"]
    d_mtr = neigh["d_mtr"]
    rx_mtr = neigh["rx_mtr"]
    ry_mtr = neigh["ry_mtr"]

    cmd_avoid_x = [0.0] * n
    cmd_avoid_y = [0.0] * n
    cmd_hard_x = [0.0] * n
    cmd_hard_y = [0.0] * n
    hard_dist = max(sim_param.r_safe, sim_param.r_body * 2.0)
    for i in range(n):
        sum_x = 0.0
        sum_y = 0.0
        hard_x = 0.0
        hard_y = 0.0
        for j in range(n):
            if i == j:
                continue
            dis = d_mtr[i][j] + (1 - a_mtr[i][j]) * sim_param.r_avoid
            temp = sim_param.r_avoid - d_mtr[i][j]
            if temp <= 0:
                dis_vec = d_mtr[i][j]
            else:
                if dis <= 1e-9:
                    dis_vec = d_mtr[i][j]
                else:
                    spr = temp / dis
                    unit_x = -rx_mtr[i][j] / dis
                    unit_y = -ry_mtr[i][j] / dis
                    sum_x += spr * unit_x
                    sum_y += spr * unit_y
                    dis_vec = d_mtr[i][j]
            if dis_vec <= 1e-9 or dis_vec >= hard_dist:
                continue
            # Emergency short-range barrier term to avoid physical interpenetration.
            # It activates only when robots are closer than hard_dist.
            over = hard_dist - dis_vec
            unit_x = -rx_mtr[i][j] / dis_vec
            unit_y = -ry_mtr[i][j] / dis_vec
            gain = over / max(dis_vec, 1e-3)
            hard_x += gain * unit_x
            hard_y += gain * unit_y
        cmd_avoid_x[i] = sum_x
        cmd_avoid_y[i] = sum_y
        cmd_hard_x[i] = hard_x
        cmd_hard_y[i] = hard_y

    cmd_cons_x = [0.0] * n
    cmd_cons_y = [0.0] * n
    for i in range(n):
        sum_x = 0.0
        sum_y = 0.0
        neigh_num = 0
        for j in range(n):
            if a_mtr[i][j] == 0:
                continue
            sum_x += (robot_state.vel_x[i] - robot_state.vel_x[j])
            sum_y += (robot_state.vel_y[i] - robot_state.vel_y[j])
            neigh_num += 1
        if neigh_num == 0:
            neigh_num = 1
        cmd_cons_x[i] = sum_x / neigh_num
        cmd_cons_y[i] = sum_y / neigh_num

    cmd_x = [
        sim_param.kappa_avoid * cmd_avoid_x[i]
        + sim_param.kappa_hard_avoid * cmd_hard_x[i]
        - sim_param.kappa_consensus * cmd_cons_x[i]
        for i in range(n)
    ]
    cmd_y = [
        sim_param.kappa_avoid * cmd_avoid_y[i]
        + sim_param.kappa_hard_avoid * cmd_hard_y[i]
        - sim_param.kappa_consensus * cmd_cons_y[i]
        for i in range(n)
    ]
    return cmd_x, cmd_y


def enforce_safety_barrier(sim_param: SimParam, neigh: Dict[str, List[List[float]]], cmd_x: List[float], cmd_y: List[float]) -> Tuple[List[float], List[float]]:
    n = len(cmd_x)
    if n == 0:
        return cmd_x, cmd_y
    d_mtr = neigh.get("d_mtr", [])
    rx_mtr = neigh.get("rx_mtr", [])
    ry_mtr = neigh.get("ry_mtr", [])
    if not d_mtr or not rx_mtr or not ry_mtr:
        return cmd_x, cmd_y

    safe_dist = max(sim_param.r_safe, 2.0 * sim_param.r_body)
    hard_dist = max(1.2 * sim_param.r_body, 0.75 * safe_dist)
    if safe_dist <= 1e-6:
        return cmd_x, cmd_y

    for i in range(n):
        push_x = 0.0
        push_y = 0.0
        nearest = float("inf")
        for j in range(n):
            if i == j:
                continue
            dis = d_mtr[i][j]
            if dis < nearest:
                nearest = dis
            if dis <= 1e-9 or dis >= safe_dist:
                continue

            # Remove any velocity component that keeps moving toward neighbor j.
            to_x = rx_mtr[i][j] / dis
            to_y = ry_mtr[i][j] / dis
            inward = cmd_x[i] * to_x + cmd_y[i] * to_y
            if inward > 0.0:
                cmd_x[i] -= inward * to_x
                cmd_y[i] -= inward * to_y

            away_x = -to_x
            away_y = -to_y
            closeness = (safe_dist - dis) / safe_dist
            push_x += closeness * closeness * away_x
            push_y += closeness * closeness * away_y

        push_norm = math.hypot(push_x, push_y)
        if push_norm > 1e-9:
            cmd_x[i] += sim_param.kappa_hard_avoid * push_x
            cmd_y[i] += sim_param.kappa_hard_avoid * push_y

        if nearest < hard_dist and push_norm > 1e-9:
            # Hard emergency mode: prioritize separation direction.
            cmd_x[i] = sim_param.vel_max * push_x / push_norm
            cmd_y[i] = sim_param.vel_max * push_y / push_norm

    return cmd_x, cmd_y


def limit_speed(cmd_x: List[float], cmd_y: List[float], vel_max: float) -> Tuple[List[float], List[float]]:
    n = len(cmd_x)
    for i in range(n):
        speed = math.hypot(cmd_x[i], cmd_y[i])
        if speed > vel_max and speed > 1e-6:
            scale = vel_max / speed
            cmd_x[i] *= scale
            cmd_y[i] *= scale
    return cmd_x, cmd_y


def get_shape_center(shape_state: ShapeState) -> Tuple[float, float, float]:
    n = max(1, len(shape_state.pos_x))
    center_x = sum(shape_state.pos_x) / n
    center_y = sum(shape_state.pos_y) / n
    # Keep arithmetic mean to stay consistent with the MATLAB metric.
    center_h = sum(shape_state.head) / n
    return center_x, center_y, center_h


def compute_swarm_metric(
    sim_param: SimParam,
    shape_mtr: ShapeMatrix,
    shape_info: ShapeInfo,
    robot_state: RobotState,
    neigh: Dict[str, List[List[float]]],
    shape_state: ShapeState,
    refer_state: Optional[ReferState] = None,
    cmd_x: Optional[List[float]] = None,
    cmd_y: Optional[List[float]] = None,
) -> SwarmMetric:
    metric = SwarmMetric()
    n = max(1, sim_param.swarm_size)
    center_x, center_y, center_h = get_shape_center(shape_state)

    base_value = [[0 if shape_mtr.value[r][c] <= 0.0 else 1 for c in range(shape_info.cn)] for r in range(shape_info.rn)]
    cell_num = sum(1 for r in range(shape_info.rn) for c in range(shape_info.cn) if base_value[r][c] == 0)
    if cell_num <= 0:
        cell_num = 1
    cover_value = [row[:] for row in base_value]
    robot_range = int(math.floor(sim_param.r_avoid / max(shape_info.grid, 1e-6)))
    robot_range = max(1, robot_range)

    num_entered = 0
    num_inside = 0
    enter_threshold = 1.0 / max(shape_info.gray_scale, 1e-6)
    for i in range(sim_param.swarm_size):
        location = trans_goal_to_local(
            robot_state.pos_x[i],
            robot_state.pos_y[i],
            center_x,
            center_y,
            center_h,
            shape_info.grid,
            shape_info.cen_x,
            shape_info.cen_y,
            shape_info.rn,
            shape_info.cn,
        )
        if location[2]:
            row, col, _ = location
            min_r = max(0, row - robot_range)
            max_r = min(shape_info.rn - 1, row + robot_range)
            min_c = max(0, col - robot_range)
            max_c = min(shape_info.cn - 1, col + robot_range)
            for r in range(min_r, max_r + 1):
                for c in range(min_c, max_c + 1):
                    cover_value[r][c] = 1
        gray_value = get_gray_value(location, shape_mtr.value)
        if gray_value < 1.0:
            num_inside += 1
        if abs(gray_value) <= enter_threshold:
            num_entered += 1

    void_num = sum(1 for r in range(shape_info.rn) for c in range(shape_info.cn) if cover_value[r][c] == 0)
    metric.cover_rate = float(cell_num - void_num) / float(cell_num)
    metric.inside_rate = float(num_inside) / float(n)
    metric.enter_rate = float(num_entered) / float(n)

    a_mtr = neigh.get("a_mtr", [])
    d_mtr = neigh.get("d_mtr", [])
    min_dist_set: List[float] = []
    pair_min = float("inf")
    collision_pairs = 0
    collision_dist = max(2.0 * sim_param.r_body, 1e-6)
    if a_mtr and d_mtr:
        for i in range(sim_param.swarm_size):
            for j in range(i + 1, sim_param.swarm_size):
                dist_ij = d_mtr[i][j]
                if dist_ij < pair_min:
                    pair_min = dist_ij
                if dist_ij < collision_dist:
                    collision_pairs += 1
        for i in range(sim_param.swarm_size):
            best = sim_param.r_sense
            for j in range(sim_param.swarm_size):
                if i == j:
                    continue
                if a_mtr[i][j]:
                    best = min(best, d_mtr[i][j])
            min_dist_set.append(best)
    else:
        min_dist_set = [sim_param.r_sense] * sim_param.swarm_size
    if pair_min == float("inf"):
        pair_min = sim_param.r_sense
    mean_dist = sum(min_dist_set) / float(n)
    metric.dist_var = sum((v - mean_dist) ** 2 for v in min_dist_set)
    metric.min_pair_dist = pair_min
    metric.collision_pairs = collision_pairs

    vel_norm_sum = 0.0
    vel_sum_x = 0.0
    vel_sum_y = 0.0
    for i in range(sim_param.swarm_size):
        speed = math.hypot(robot_state.vel_x[i], robot_state.vel_y[i])
        vel_norm_sum += speed
        vel_sum_x += robot_state.vel_x[i]
        vel_sum_y += robot_state.vel_y[i]
    if vel_norm_sum <= 1e-9:
        vel_norm_sum = 1.0
    metric.vel_align = math.hypot(vel_sum_x, vel_sum_y) / vel_norm_sum

    if a_mtr:
        neigh_total = sum(sum(row) for row in a_mtr)
        metric.neigh_mean = float(neigh_total) / float(n)

    if cmd_x is not None and cmd_y is not None and cmd_x and cmd_y:
        sat = 0
        for i in range(min(len(cmd_x), len(cmd_y))):
            if math.hypot(cmd_x[i], cmd_y[i]) >= sim_param.vel_max * 0.99:
                sat += 1
        metric.cmd_sat_rate = float(sat) / float(max(1, min(len(cmd_x), len(cmd_y))))

    if refer_state is not None:
        metric.shape_center_err = math.hypot(center_x - refer_state.pos_x, center_y - refer_state.pos_y)
    return metric


def format_metric(metric: SwarmMetric) -> str:
    return (
        "cover={:.3f} inside={:.3f} enter={:.3f} dist_var={:.3f} vel_align={:.3f} "
        "neigh_mean={:.2f} sat={:.3f} center_err={:.3f} min_pair={:.3f} coll_pairs={}"
    ).format(
        metric.cover_rate,
        metric.inside_rate,
        metric.enter_rate,
        metric.dist_var,
        metric.vel_align,
        metric.neigh_mean,
        metric.cmd_sat_rate,
        metric.shape_center_err,
        metric.min_pair_dist,
        metric.collision_pairs,
    )


def _init_random_robot_state(sim_param: SimParam, center_x: float, center_y: float, spread: float) -> RobotState:
    state = RobotState(sim_param.swarm_size)
    for i in range(sim_param.swarm_size):
        state.pos_x[i] = center_x + random.uniform(-spread, spread)
        state.pos_y[i] = center_y + random.uniform(-spread, spread)
        state.vel_x[i] = 0.0
        state.vel_y[i] = 0.0
        state.yaw[i] = random.uniform(-math.pi, math.pi)
    return state


def run_offline_self_test(args: argparse.Namespace) -> int:
    random.seed(args.seed)

    sim_param = SimParam()
    sim_param.swarm_size = int(args.agents)
    sim_param.t = float(args.dt)
    sim_param.r_body = float(args.r_body)
    sim_param.r_safe = float(args.r_safe)
    sim_param.r_avoid = float(args.r_avoid)
    sim_param.r_sense = float(args.r_sense)
    sim_param.vel_max = float(args.vel_max)
    sim_param.shape_vel_max = float(args.shape_vel_max)
    sim_param.leader_fraction = float(args.leader_fraction)
    sim_param.kappa_conse_pos = float(args.kappa_conse_pos)
    sim_param.kappa_track_pos = float(args.kappa_track_pos)
    sim_param.kappa_conse_head = float(args.kappa_conse_head)
    sim_param.kappa_track_head = float(args.kappa_track_head)
    sim_param.alpha = float(args.alpha)
    sim_param.hvel_max = float(args.hvel_max)
    sim_param.kappa_enter = float(args.kappa_enter)
    sim_param.kappa_explore_1 = float(args.kappa_explore_1)
    sim_param.kappa_explore_2 = float(args.kappa_explore_2)
    sim_param.kappa_avoid = float(args.kappa_avoid)
    sim_param.kappa_hard_avoid = float(args.kappa_hard_avoid)
    sim_param.kappa_consensus = float(args.kappa_consensus)

    image_mtr, shape_desc = load_shape_matrix(
        shape_source=args.shape_source,
        shape_type=args.shape_type,
        shape_resolution=int(args.shape_resolution),
        ring_inner_ratio=float(args.ring_inner_ratio),
        ring_outer_ratio=float(args.ring_outer_ratio),
        gray_width=int(args.gray_width),
        shape_mat_path=args.shape_mat_path,
        shape_library_root=args.shape_library_root,
    )
    shape_mtr, shape_info = init_form_shape(sim_param, image_mtr)

    robot_state = _init_random_robot_state(sim_param, args.ref_x, args.ref_y, args.init_spread)
    refer_state = ReferState(args.ref_x, args.ref_y)
    refer_state.head = args.ref_head

    shape_state = ShapeState(sim_param.swarm_size)
    shape_state.pos_x = list(robot_state.pos_x)
    shape_state.pos_y = list(robot_state.pos_y)
    shape_state.vel_x = list(robot_state.vel_x)
    shape_state.vel_y = list(robot_state.vel_y)
    temp_head = [random.random() * 2.0 * math.pi for _ in range(sim_param.swarm_size)]
    aveg_head = sum(temp_head) / float(sim_param.swarm_size)
    shape_state.head = [h - aveg_head for h in temp_head]
    shape_state.hvel = [0.0 for _ in range(sim_param.swarm_size)]

    leader_num = int(math.ceil(sim_param.swarm_size * sim_param.leader_fraction))
    if leader_num > 0:
        inform_index = sorted(random.sample(range(sim_param.swarm_size), leader_num))
    else:
        inform_index = []

    print(
        "[self-test] agents={} leaders={} shape={} source={} dt={:.3f} steps={}".format(
            sim_param.swarm_size,
            inform_index,
            _normalize_shape_type(args.shape_type),
            shape_desc,
            sim_param.t,
            args.steps,
        )
    )

    metric = SwarmMetric()
    for step in range(args.steps):
        neigh = get_neighbor_set(sim_param, robot_state)
        shape_state = negotiate_position(sim_param, neigh, shape_state, refer_state, inform_index)
        shape_state = negotiate_orientation(sim_param, neigh, shape_state, refer_state, inform_index)
        shape_dyn = get_dyn_formation(sim_param, shape_mtr, shape_info, shape_state)

        cmd_enter_x, cmd_enter_y, grid_set, _enter_goals = cal_entering_cmd(
            sim_param,
            shape_dyn,
            shape_info,
            robot_state,
            shape_state,
        )
        cmd_explore_x, cmd_explore_y = cal_exploration_cmd(sim_param, shape_dyn, shape_info, grid_set, neigh, robot_state, shape_state)
        cmd_interact_x, cmd_interact_y = cal_interaction_cmd(sim_param, neigh, robot_state)
        cmd_x = [cmd_enter_x[i] + cmd_explore_x[i] + cmd_interact_x[i] for i in range(sim_param.swarm_size)]
        cmd_y = [cmd_enter_y[i] + cmd_explore_y[i] + cmd_interact_y[i] for i in range(sim_param.swarm_size)]
        cmd_x, cmd_y = enforce_safety_barrier(sim_param, neigh, cmd_x, cmd_y)
        cmd_x, cmd_y = limit_speed(cmd_x, cmd_y, sim_param.vel_max)

        # Match MATLAB update model for deterministic offline verification.
        ratio = args.motion_ratio
        for i in range(sim_param.swarm_size):
            robot_state.pos_x[i] += robot_state.vel_x[i] * sim_param.t
            robot_state.pos_y[i] += robot_state.vel_y[i] * sim_param.t
            robot_state.vel_x[i] = ratio * cmd_x[i] + (1.0 - ratio) * robot_state.vel_x[i]
            robot_state.vel_y[i] = ratio * cmd_y[i] + (1.0 - ratio) * robot_state.vel_y[i]
            robot_state.yaw[i] = math.atan2(robot_state.vel_y[i], robot_state.vel_x[i] + 1e-9)

        metric = compute_swarm_metric(sim_param, shape_mtr, shape_info, robot_state, neigh, shape_state, refer_state, cmd_x, cmd_y)
        if step == 0 or (step + 1) % args.log_every == 0 or step + 1 == args.steps:
            print("[self-test] step={} {}".format(step + 1, format_metric(metric)))

    passed = metric.inside_rate >= args.target_inside and metric.cover_rate >= args.target_cover
    if passed:
        print(
            "[self-test] RESULT=PASS target_inside={:.3f} target_cover={:.3f}".format(
                args.target_inside,
                args.target_cover,
            )
        )
        return 0
    print(
        "[self-test] RESULT=FAIL target_inside={:.3f} target_cover={:.3f} final=({})".format(
            args.target_inside,
            args.target_cover,
            format_metric(metric),
        )
    )
    return 2


def build_self_test_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Offline shape assembly self-test")
    parser.add_argument("--self-test", action="store_true", help="run offline simulation instead of ROS node")
    parser.add_argument("--agents", type=int, default=8)
    parser.add_argument("--steps", type=int, default=3000)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--init-spread", type=float, default=2.0)
    parser.add_argument("--motion-ratio", type=float, default=0.1, help="robot velocity update ratio")
    parser.add_argument("--shape-source", type=str, default="analytic")
    parser.add_argument("--shape-type", type=str, default="ring")
    parser.add_argument("--shape-resolution", type=int, default=80)
    parser.add_argument("--ring-inner-ratio", type=float, default=0.25)
    parser.add_argument("--ring-outer-ratio", type=float, default=0.4)
    parser.add_argument("--gray-width", type=int, default=4)
    parser.add_argument("--shape-mat-path", type=str, default="")
    parser.add_argument("--shape-library-root", type=str, default="")
    parser.add_argument("--ref-x", type=float, default=0.0)
    parser.add_argument("--ref-y", type=float, default=0.0)
    parser.add_argument("--ref-head", type=float, default=0.0)
    parser.add_argument("--log-every", type=int, default=250)
    parser.add_argument("--target-inside", type=float, default=0.8)
    parser.add_argument("--target-cover", type=float, default=0.8)

    parser.add_argument("--r-body", type=float, default=0.2)
    parser.add_argument("--r-safe", type=float, default=0.35)
    parser.add_argument("--r-avoid", type=float, default=1.0)
    parser.add_argument("--r-sense", type=float, default=4.5)
    parser.add_argument("--vel-max", type=float, default=0.6)
    parser.add_argument("--shape-vel-max", type=float, default=1.2)
    parser.add_argument("--leader-fraction", type=float, default=0.25)
    parser.add_argument("--kappa-conse-pos", type=float, default=1.6)
    parser.add_argument("--kappa-track-pos", type=float, default=3.0)
    parser.add_argument("--kappa-conse-head", type=float, default=1.6)
    parser.add_argument("--kappa-track-head", type=float, default=3.0)
    parser.add_argument("--alpha", type=float, default=0.8)
    parser.add_argument("--hvel-max", type=float, default=1.57079632679)
    parser.add_argument("--kappa-enter", type=float, default=10.0)
    parser.add_argument("--kappa-explore-1", type=float, default=3.0)
    parser.add_argument("--kappa-explore-2", type=float, default=10.0)
    parser.add_argument("--kappa-avoid", type=float, default=15.0)
    parser.add_argument("--kappa-hard-avoid", type=float, default=25.0)
    parser.add_argument("--kappa-consensus", type=float, default=1.0)
    return parser


class ShapeAssemblySwarm:
    def __init__(self) -> None:
        self.sim_param = SimParam()
        self.sim_param.t = rospy.get_param("~control_dt", rospy.get_param("~sim_dt", 0.05))
        control_hz = rospy.get_param("~control_hz", 0.0)
        if control_hz > 0:
            self.sim_param.t = 1.0 / float(control_hz)
        self.sim_param.r_body = rospy.get_param("~r_body", self.sim_param.r_body)
        self.sim_param.r_safe = rospy.get_param("~r_safe", self.sim_param.r_safe)
        self.sim_param.r_avoid = rospy.get_param("~r_avoid", self.sim_param.r_avoid)
        self.sim_param.r_sense = rospy.get_param("~r_sense", self.sim_param.r_sense)
        self.sim_param.vel_max = rospy.get_param("~vel_max", self.sim_param.vel_max)
        self.sim_param.leader_fraction = rospy.get_param("~leader_fraction", self.sim_param.leader_fraction)
        self.sim_param.kappa_enter = rospy.get_param("~kappa_enter", self.sim_param.kappa_enter)
        self.sim_param.kappa_explore_1 = rospy.get_param("~kappa_explore_1", self.sim_param.kappa_explore_1)
        self.sim_param.kappa_explore_2 = rospy.get_param("~kappa_explore_2", self.sim_param.kappa_explore_2)
        self.sim_param.kappa_avoid = rospy.get_param("~kappa_avoid", self.sim_param.kappa_avoid)
        self.sim_param.kappa_hard_avoid = rospy.get_param("~kappa_hard_avoid", self.sim_param.kappa_hard_avoid)
        self.sim_param.kappa_consensus = rospy.get_param("~kappa_consensus", self.sim_param.kappa_consensus)
        self.sim_param.kappa_conse_pos = rospy.get_param("~kappa_conse_pos", self.sim_param.kappa_conse_pos)
        self.sim_param.kappa_track_pos = rospy.get_param("~kappa_track_pos", self.sim_param.kappa_track_pos)
        self.sim_param.kappa_conse_head = rospy.get_param("~kappa_conse_head", self.sim_param.kappa_conse_head)
        self.sim_param.kappa_track_head = rospy.get_param("~kappa_track_head", self.sim_param.kappa_track_head)
        self.sim_param.alpha = rospy.get_param("~alpha", self.sim_param.alpha)
        self.sim_param.hvel_max = rospy.get_param("~hvel_max", self.sim_param.hvel_max)
        self.sim_param.cmd_smooth_ratio = rospy.get_param("~cmd_smooth_ratio", self.sim_param.cmd_smooth_ratio)
        self.sim_param.cmd_scale = rospy.get_param("~cmd_scale", self.sim_param.cmd_scale)
        self.sim_param.shape_vel_max = rospy.get_param("~shape_vel_max", max(self.sim_param.vel_max * 2.0, self.sim_param.shape_vel_max))

        self.enabled = rospy.get_param("~enabled", True)
        self.auto_detect = rospy.get_param("~auto_detect", True)
        self.namespace_prefix = rospy.get_param("~namespace_prefix", "robot")
        self.robot_namespace_prefix = str(
            rospy.get_param("~robot_namespace_prefix", self.namespace_prefix)
        ).strip("/")
        if self.robot_namespace_prefix:
            self.namespace_prefix = self.robot_namespace_prefix
        self.agent_number = int(rospy.get_param("~agent_number", 0))
        self.robot_namespaces = rospy.get_param("~robot_namespaces", [])
        self.robot_detect_topic_suffix = str(rospy.get_param("~robot_detect_topic_suffix", "/odom"))
        self.robot_odom_topic_suffix = str(
            rospy.get_param("~robot_odom_topic_suffix", self.robot_detect_topic_suffix)
        )
        self.robot_detect_timeout = max(0.0, float(rospy.get_param("~robot_detect_timeout", 8.0)))
        self.distributed_mode = bool(rospy.get_param("~distributed_mode", False))
        self.distributed_agent_namespace = str(
            rospy.get_param("~distributed_agent_namespace", rospy.get_namespace().strip("/"))
        ).strip("/")
        self.distributed_marker_owner = str(
            rospy.get_param("~distributed_marker_owner", "")
        ).strip("/")

        self.shape_source = rospy.get_param("~shape_source", "analytic")
        self.shape_type = rospy.get_param("~shape_type", "ring")
        self.shape_resolution = int(rospy.get_param("~shape_resolution", 80))
        self.ring_inner_ratio = float(rospy.get_param("~ring_inner_ratio", 0.35))
        self.ring_outer_ratio = float(rospy.get_param("~ring_outer_ratio", 0.6))
        self.gray_width = int(rospy.get_param("~gray_width", 4))
        self.shape_mat_path = rospy.get_param("~shape_mat_path", "")
        self.shape_library_root = os.path.expanduser(str(rospy.get_param("~shape_library_root", "")).strip() or _get_default_shape_dir())
        self.formation_task_topic = str(rospy.get_param("~formation_task_topic", "")).strip()
        self.robot_status_topic = str(rospy.get_param("~robot_status_topic", "shape_assembly/status")).strip()
        self.active_task_id = int(rospy.get_param("~initial_task_id", 0))
        self.pending_shape_reload = False

        self.reference_center = rospy.get_param("~reference_center", [])
        self.reference_heading = float(rospy.get_param("~reference_heading", 0.0))
        self.reference_center_topic = rospy.get_param("~reference_center_topic", "")
        self.reference_center_wait = bool(rospy.get_param("~reference_center_wait", False))
        self.reference_center_use_topic_heading = bool(rospy.get_param("~reference_center_use_topic_heading", False))
        self.init_center_mode = rospy.get_param("~init_center_mode", "reference")
        self.init_center_use_reference_only = rospy.get_param("~init_center_use_reference_only", False)
        self.random_seed = rospy.get_param("~random_seed", None)
        self.control_strategy = _normalize_control_strategy(rospy.get_param("~control_strategy", ControlStrategy.SHAPE_ONLY))
        if self.control_strategy not in (ControlStrategy.SHAPE_ONLY, ControlStrategy.MOVE_BASE_THEN_SHAPE):
            rospy.logwarn("Unknown control_strategy=%s, fallback to %s", self.control_strategy, ControlStrategy.SHAPE_ONLY)
            self.control_strategy = ControlStrategy.SHAPE_ONLY
        self.shape_target_mode = _normalize_shape_target_mode(rospy.get_param("~shape_target_mode", ShapeTargetMode.NEGOTIATED))
        if self.shape_target_mode not in (ShapeTargetMode.NEGOTIATED, ShapeTargetMode.REFERENCE):
            rospy.logwarn("Unknown shape_target_mode=%s, fallback to %s", self.shape_target_mode, ShapeTargetMode.NEGOTIATED)
            self.shape_target_mode = ShapeTargetMode.NEGOTIATED
        self.switch_gray_threshold = float(rospy.get_param("~switch_gray_threshold", 0.999))
        self.cancel_move_base_on_switch = bool(rospy.get_param("~cancel_move_base_on_switch", True))
        self.cancel_grace_period = max(0.0, float(rospy.get_param("~cancel_grace_period", 0.0)))
        self.stop_path_planning_on_shape_takeover = bool(
            rospy.get_param("~stop_path_planning_on_shape_takeover", False)
        )
        self.switch_use_reference_shape = bool(rospy.get_param("~switch_use_reference_shape", True))
        self.switch_reference_radius_enable = bool(rospy.get_param("~switch_reference_radius_enable", True))
        self.switch_reference_radius_margin = max(0.0, float(rospy.get_param("~switch_reference_radius_margin", 0.25)))
        self.switch_reference_radius_min = max(0.0, float(rospy.get_param("~switch_reference_radius_min", 0.0)))
        self.switch_reference_radius = 0.0
        self.use_local_costmap_avoid = bool(rospy.get_param("~use_local_costmap_avoid", True))
        self.local_costmap_topic_suffix = str(rospy.get_param("~local_costmap_topic_suffix", "move_base/local_costmap/costmap"))
        self.local_costmap_obstacle_threshold = int(rospy.get_param("~local_costmap_obstacle_threshold", 80))
        self.local_costmap_unknown_is_obstacle = bool(rospy.get_param("~local_costmap_unknown_is_obstacle", False))
        self.local_costmap_avoid_radius = float(rospy.get_param("~local_costmap_avoid_radius", max(self.sim_param.r_avoid, self.sim_param.r_safe)))
        self.local_costmap_avoid_gain = float(rospy.get_param("~local_costmap_avoid_gain", self.sim_param.kappa_avoid))
        self.local_costmap_hard_radius = float(rospy.get_param("~local_costmap_hard_radius", max(self.sim_param.r_safe, 2.0 * self.sim_param.r_body)))
        self.local_costmap_hard_gain = float(rospy.get_param("~local_costmap_hard_gain", self.sim_param.kappa_hard_avoid))
        self.local_costmap_stride = max(1, int(rospy.get_param("~local_costmap_stride", 1)))
        self.local_costmap_max_samples = max(1, int(rospy.get_param("~local_costmap_max_samples", 500)))
        self.local_costmap_timeout = max(0.0, float(rospy.get_param("~local_costmap_timeout", 1.0)))
        self.local_costmap_self_ignore_radius = max(0.0, float(rospy.get_param("~local_costmap_self_ignore_radius", self.sim_param.r_body * 1.2)))
        self.local_costmap_skip_on_frame_mismatch = bool(rospy.get_param("~local_costmap_skip_on_frame_mismatch", True))
        self.auto_shape_heading = bool(rospy.get_param("~auto_shape_heading", True))
        self.auto_shape_heading_map_topic = str(rospy.get_param("~auto_shape_heading_map_topic", "/map"))
        self.auto_shape_heading_angle_step_deg = max(1.0, float(rospy.get_param("~auto_shape_heading_angle_step_deg", 15.0)))
        self.auto_shape_heading_update_interval = max(0.05, float(rospy.get_param("~auto_shape_heading_update_interval", 0.5)))
        self.auto_shape_heading_obstacle_threshold = int(rospy.get_param("~auto_shape_heading_obstacle_threshold", 80))
        self.auto_shape_heading_unknown_is_obstacle = bool(rospy.get_param("~auto_shape_heading_unknown_is_obstacle", True))
        self.auto_shape_heading_shape_stride = max(1, int(rospy.get_param("~auto_shape_heading_shape_stride", 2)))
        self.auto_shape_heading_min_improve = max(0.0, float(rospy.get_param("~auto_shape_heading_min_improve", 0.01)))
        self.auto_shape_heading_yaw_bias = max(0.0, float(rospy.get_param("~auto_shape_heading_yaw_bias", 0.02)))
        self.auto_shape_heading_oob_is_obstacle = bool(rospy.get_param("~auto_shape_heading_oob_is_obstacle", True))

        if self.random_seed is not None:
            random.seed(int(self.random_seed))
        elif self.distributed_mode:
            random.seed(1)
            rospy.logwarn("ShapeAssembly: distributed_mode enabled without random_seed, fallback to deterministic seed=1.")

        self.publish_markers = rospy.get_param("~publish_markers", True)
        self.marker_topic = rospy.get_param("~marker_topic", "/shape_assembly/markers")
        self.marker_frame = rospy.get_param("~marker_frame", "map")
        self.shape_point_stride = max(1, int(rospy.get_param("~shape_point_stride", 1)))
        self.shape_point_size = float(rospy.get_param("~shape_point_size", 0.05))
        self.shape_black_threshold = float(rospy.get_param("~shape_black_threshold", 1e-6))
        self.shape_color_mode = rospy.get_param("~shape_color_mode", "gradient")
        self.shape_gradient_alpha = float(rospy.get_param("~shape_gradient_alpha", 0.9))
        self.shape_black_color = rospy.get_param("~shape_black_color", [1.0, 0.2, 0.2])
        self.shape_gray_color = rospy.get_param("~shape_gray_color", [0.2, 0.8, 0.2])
        self.shape_black_alpha = float(rospy.get_param("~shape_black_alpha", 0.9))
        self.shape_gray_alpha = float(rospy.get_param("~shape_gray_alpha", 0.6))
        self.show_comm_range = rospy.get_param("~show_comm_range", True)
        self.comm_range_radius = float(rospy.get_param("~comm_range_radius", 0.0))
        self.comm_range_line_width = float(rospy.get_param("~comm_range_line_width", 0.01))
        self.comm_range_color = rospy.get_param("~comm_range_color", [0.1, 0.7, 1.0])
        self.comm_range_alpha = float(rospy.get_param("~comm_range_alpha", 0.25))
        self.comm_range_segments = max(12, int(rospy.get_param("~comm_range_segments", 48)))
        self.show_occupancy = rospy.get_param("~show_occupancy", True)
        self.occupancy_radius = float(rospy.get_param("~occupancy_radius", 0.0))
        self.occupancy_height = float(rospy.get_param("~occupancy_height", 0.02))
        self.occupancy_color = rospy.get_param("~occupancy_color", [1.0, 0.6, 0.2])
        self.occupancy_alpha = float(rospy.get_param("~occupancy_alpha", 0.2))
        self.publish_links = rospy.get_param("~publish_links", True)
        self.link_color = rospy.get_param("~link_color", [0.2, 0.6, 1.0])
        self.link_alpha = float(rospy.get_param("~link_alpha", 0.5))
        self.link_width = float(rospy.get_param("~link_width", 0.02))
        self.link_max_pairs = int(rospy.get_param("~link_max_pairs", 0))
        self.show_enter_targets = rospy.get_param("~show_enter_targets", True)
        self.enter_target_size = float(rospy.get_param("~enter_target_size", max(0.03, self.shape_point_size * 1.4)))
        self.enter_target_line_width = float(rospy.get_param("~enter_target_line_width", self.link_width))
        self.enter_target_alpha = float(rospy.get_param("~enter_target_alpha", 0.85))
        self.debug_log = rospy.get_param("~debug_log", False)
        self.debug_log_hz = float(rospy.get_param("~debug_log_hz", 1.0))
        self.debug_log_robot_indices = rospy.get_param("~debug_log_robot_indices", [])
        self.debug_log_components = rospy.get_param("~debug_log_components", False)
        self.debug_log_metric = rospy.get_param("~debug_log_metric", True)
        self._last_debug_time = 0.0
        self.monitor_report_enabled = bool(rospy.get_param("~monitor_report_enabled", False))
        self.monitor_report_interval = float(rospy.get_param("~monitor_report_interval", 5.0))
        self._last_monitor_report_time = 0.0
        self._detect_start_time = 0.0
        self._detect_last_change_time = 0.0
        self._detect_best_ids: List[int] = []
        self.velocity_source = rospy.get_param("~velocity_source", "odom")
        self.cmd_in_map_frame = rospy.get_param("~cmd_in_map_frame", True)
        self.cmd_smooth_use_odom = rospy.get_param("~cmd_smooth_use_odom", True)
        self.shape_marker_mode = rospy.get_param("~shape_marker_mode", "reference")
        self.odom_twist_in_base = rospy.get_param("~odom_twist_in_base", False)
        self.velocity_scale = float(rospy.get_param("~velocity_scale", 1.0))
        self.arrow_shaft_diameter = float(rospy.get_param("~arrow_shaft_diameter", 0.04))
        self.arrow_head_diameter = float(rospy.get_param("~arrow_head_diameter", 0.08))
        self.arrow_head_length = float(rospy.get_param("~arrow_head_length", 0.12))
        self.show_speed_text = rospy.get_param("~show_speed_text", True)
        self.speed_text_z = float(rospy.get_param("~speed_text_z", 0.3))
        self.speed_text_size = float(rospy.get_param("~speed_text_size", 0.2))
        self.show_metric_text = rospy.get_param("~show_metric_text", True)
        self.metric_text_z = float(rospy.get_param("~metric_text_z", 0.8))
        self.metric_text_size = float(rospy.get_param("~metric_text_size", 0.16))
        self.metric_text_color = rospy.get_param("~metric_text_color", [1.0, 1.0, 1.0])
        self.metric_text_alpha = float(rospy.get_param("~metric_text_alpha", 0.95))
        self.converge_inside_rate = float(rospy.get_param("~converge_inside_rate", 0.80))
        self.converge_enter_rate = float(rospy.get_param("~converge_enter_rate", 0.10))
        self.converge_cover_rate = float(rospy.get_param("~converge_cover_rate", 0.80))
        self.converge_hold_time = float(rospy.get_param("~converge_hold_time", 3.0))
        self._converged_since = -1.0
        self._converged_reported = False

        self.odom_subs: List[rospy.Subscriber] = []
        self.odom_msgs: List[Optional[Odometry]] = []
        self.local_costmap_subs: List[rospy.Subscriber] = []
        self.local_costmap_msgs: List[Optional[OccupancyGrid]] = []
        self.local_costmap_topics: List[str] = []
        self.local_costmap_ready: List[bool] = []
        self.local_costmap_obs_cells: List[int] = []
        self.local_costmap_msg_age: List[float] = []
        self.local_costmap_frame: List[str] = []
        self.cmd_pubs: List[Optional[rospy.Publisher]] = []
        self.move_base_cancel_pubs: List[Optional[rospy.Publisher]] = []
        self.reference_center_sub: Optional[rospy.Subscriber] = None
        self.formation_task_sub: Optional[rospy.Subscriber] = None
        self.reference_center_received = False
        self.marker_pub: Optional[rospy.Publisher] = None
        self.robot_status_pub: Optional[rospy.Publisher] = None
        self.auto_shape_heading_map_sub: Optional[rospy.Subscriber] = None
        self.auto_shape_heading_map_msg: Optional[OccupancyGrid] = None
        self._shape_overlap_samples: List[Tuple[float, float, float]] = []
        self._last_auto_heading_time = 0.0
        self.ns_list: List[str] = []
        self.self_agent_index = -1
        self.marker_owner_active = True

        self.shape_mtr: Optional[ShapeMatrix] = None
        self.shape_info: Optional[ShapeInfo] = None
        self.shape_state: Optional[ShapeState] = None
        self.refer_state: Optional[ReferState] = None
        self.inform_index: List[int] = []
        self.prev_cmd_x: List[float] = []
        self.prev_cmd_y: List[float] = []
        self.shape_ctrl_active: List[bool] = []
        self.shape_ctrl_hold_until: List[float] = []
        self.switch_gray_values: List[float] = []
        self.marker_needs_clear = True
        self._shape_desc = ""
        self._last_stop_path_planning_state: Optional[bool] = None
        self._last_stop_path_planning_ids: List[int] = []
        self.tf_buffer = None
        self.tf_listener = None
        if self.use_local_costmap_avoid and tf2_ros is not None:
            try:
                self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(8.0))
                self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            except Exception as exc:
                rospy.logwarn("ShapeAssembly: failed to initialize tf2 for local costmap adaptation (%s).", str(exc))
                self.tf_buffer = None
                self.tf_listener = None

        if self.publish_markers:
            self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)
        if RobotFormationStatus is not None and self.robot_status_topic:
            self.robot_status_pub = rospy.Publisher(self.robot_status_topic, RobotFormationStatus, queue_size=5)

        if self.reference_center_topic:
            self.reference_center_sub = rospy.Subscriber(
                self.reference_center_topic,
                PoseStamped,
                self._reference_center_cb,
                queue_size=4,
            )
            rospy.loginfo(
                "ShapeAssembly: subscribing reference center topic=%s wait=%s use_heading=%s",
                self.reference_center_topic,
                str(self.reference_center_wait),
                str(self.reference_center_use_topic_heading),
            )
        if str(self.shape_source).strip().lower() == "mat" and not os.path.isdir(self.shape_library_root):
            rospy.logwarn("ShapeAssembly: local shape_library_root=%s missing. Each robot runtime must store local shape_images or override ~shape_library_root.", self.shape_library_root)
        if self.formation_task_topic:
            rospy.loginfo("ShapeAssembly: runtime shape selection follows host task topic=%s; local ~shape_type is startup fallback only.", self.formation_task_topic)

        if ShapeTask is not None and self.formation_task_topic:
            self.formation_task_sub = rospy.Subscriber(
                self.formation_task_topic,
                ShapeTask,
                self._formation_task_cb,
                queue_size=2,
            )
            rospy.loginfo("ShapeAssembly: subscribing formation task topic=%s", self.formation_task_topic)

        if self.auto_shape_heading and self.auto_shape_heading_map_topic:
            self.auto_shape_heading_map_sub = rospy.Subscriber(
                self.auto_shape_heading_map_topic,
                OccupancyGrid,
                self._auto_shape_heading_map_cb,
                queue_size=1,
            )
            rospy.loginfo(
                "ShapeAssembly: auto shape heading enabled topic=%s step=%.1fdeg interval=%.2fs mode=%s",
                self.auto_shape_heading_map_topic,
                self.auto_shape_heading_angle_step_deg,
                self.auto_shape_heading_update_interval,
                self.shape_target_mode,
            )

        self._update_takeover_state_param([])

        self._dyn_server = None
        self._dyn_ready = False
        self._setup_dynamic_reconfigure()

        self.timer = rospy.Timer(rospy.Duration(self.sim_param.t), self._on_timer)

    def _setup_dynamic_reconfigure(self) -> None:
        if DynamicReconfigureServer is None or ShapeAssemblySwarmConfig is None:
            rospy.logwarn("ShapeAssembly: dynamic_reconfigure unavailable, continue with static params.")
            return
        try:
            self._dyn_server = DynamicReconfigureServer(
                ShapeAssemblySwarmConfig,
                self._dynamic_reconfigure_cb,
            )
            rospy.loginfo("ShapeAssembly: dynamic_reconfigure ready.")
        except Exception as exc:
            self._dyn_server = None
            rospy.logwarn("ShapeAssembly: failed to start dynamic_reconfigure (%s).", str(exc))

    def _dynamic_reconfigure_cb(self, config, _level):
        self.enabled = bool(config.enabled)

        self.sim_param.vel_max = max(0.0, float(config.vel_max))
        self.sim_param.shape_vel_max = max(0.0, float(config.shape_vel_max))
        self.sim_param.hvel_max = max(0.0, float(config.hvel_max))
        self.sim_param.leader_fraction = max(0.0, min(1.0, float(config.leader_fraction)))
        self.sim_param.kappa_enter = max(0.0, float(config.kappa_enter))
        self.sim_param.kappa_explore_1 = max(0.0, float(config.kappa_explore_1))
        self.sim_param.kappa_explore_2 = max(0.0, float(config.kappa_explore_2))
        self.sim_param.kappa_avoid = max(0.0, float(config.kappa_avoid))
        self.sim_param.kappa_hard_avoid = max(0.0, float(config.kappa_hard_avoid))
        self.sim_param.kappa_consensus = max(0.0, float(config.kappa_consensus))
        self.sim_param.kappa_conse_pos = max(0.0, float(config.kappa_conse_pos))
        self.sim_param.kappa_track_pos = max(0.0, float(config.kappa_track_pos))
        self.sim_param.kappa_conse_head = max(0.0, float(config.kappa_conse_head))
        self.sim_param.kappa_track_head = max(0.0, float(config.kappa_track_head))
        self.sim_param.alpha = max(0.0, min(1.0, float(config.alpha)))
        self.sim_param.cmd_smooth_ratio = max(0.0, min(1.0, float(config.cmd_smooth_ratio)))
        self.sim_param.cmd_scale = max(0.0, float(config.cmd_scale))

        self.velocity_scale = max(0.0, float(config.velocity_scale))
        self.switch_gray_threshold = max(0.0, min(1.0, float(config.switch_gray_threshold)))
        self.switch_reference_radius_enable = bool(config.switch_reference_radius_enable)
        self.switch_reference_radius_margin = max(0.0, float(config.switch_reference_radius_margin))
        self.switch_reference_radius_min = max(0.0, float(config.switch_reference_radius_min))
        self.switch_reference_radius = self._compute_reference_switch_radius()

        self.local_costmap_obstacle_threshold = max(0, min(100, int(config.local_costmap_obstacle_threshold)))
        hard_radius = max(0.0, float(config.local_costmap_hard_radius))
        avoid_radius = max(hard_radius, float(config.local_costmap_avoid_radius))
        self.local_costmap_hard_radius = hard_radius
        self.local_costmap_avoid_radius = avoid_radius
        self.local_costmap_avoid_gain = max(0.0, float(config.local_costmap_avoid_gain))
        self.local_costmap_hard_gain = max(0.0, float(config.local_costmap_hard_gain))
        self.local_costmap_stride = max(1, int(config.local_costmap_stride))
        self.local_costmap_max_samples = max(1, int(config.local_costmap_max_samples))
        self.local_costmap_timeout = max(0.0, float(config.local_costmap_timeout))
        self.local_costmap_self_ignore_radius = max(0.0, float(config.local_costmap_self_ignore_radius))

        self.auto_shape_heading_angle_step_deg = max(1.0, float(config.auto_shape_heading_angle_step_deg))
        self.auto_shape_heading_update_interval = max(0.05, float(config.auto_shape_heading_update_interval))
        self.auto_shape_heading_obstacle_threshold = max(0, min(100, int(config.auto_shape_heading_obstacle_threshold)))
        self.auto_shape_heading_min_improve = max(0.0, float(config.auto_shape_heading_min_improve))
        self.auto_shape_heading_yaw_bias = max(0.0, float(config.auto_shape_heading_yaw_bias))

        self.converge_inside_rate = max(0.0, min(1.0, float(config.converge_inside_rate)))
        self.converge_enter_rate = max(0.0, min(1.0, float(config.converge_enter_rate)))
        self.converge_cover_rate = max(0.0, min(1.0, float(config.converge_cover_rate)))
        self.converge_hold_time = max(0.0, float(config.converge_hold_time))

        self.publish_markers = bool(config.publish_markers)
        if self.publish_markers and self.marker_pub is None:
            self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)
            self.marker_needs_clear = True
        elif not self.publish_markers:
            self.marker_needs_clear = True

        self.debug_log = bool(config.debug_log)
        self.debug_log_hz = max(0.1, float(config.debug_log_hz))
        self.debug_log_components = bool(config.debug_log_components)
        self.debug_log_metric = bool(config.debug_log_metric)

        if self._dyn_ready:
            rospy.loginfo(
                "ShapeAssembly[dyn]: enabled=%s vel_max=%.2f shape_vel_max=%.2f kappa_avoid=%.2f hard_avoid=%.2f",
                str(self.enabled),
                self.sim_param.vel_max,
                self.sim_param.shape_vel_max,
                self.sim_param.kappa_avoid,
                self.sim_param.kappa_hard_avoid,
            )
        else:
            self._dyn_ready = True
        return config

    def _reload_shape_model(self) -> None:
        if self.sim_param.swarm_size <= 0:
            self.pending_shape_reload = True
            return
        try:
            image_mtr, self._shape_desc = load_shape_matrix(
                shape_source=self.shape_source,
                shape_type=self.shape_type,
                shape_resolution=self.shape_resolution,
                ring_inner_ratio=self.ring_inner_ratio,
                ring_outer_ratio=self.ring_outer_ratio,
                gray_width=self.gray_width,
                shape_mat_path=self.shape_mat_path,
                shape_library_root=self.shape_library_root,
            )
        except Exception as exc:
            rospy.logwarn("Failed to load shape matrix: %s. Falling back to analytic ring.", exc)
            image_mtr = generate_ring_shape(self.shape_resolution, self.ring_inner_ratio, self.ring_outer_ratio, self.gray_width)
            self._shape_desc = "fallback:analytic:ring"
        self.shape_mtr, self.shape_info = init_form_shape(self.sim_param, image_mtr)
        self.switch_reference_radius = self._compute_reference_switch_radius()
        self._build_shape_overlap_samples()
        self.marker_needs_clear = True
        self.pending_shape_reload = False

    def _formation_task_cb(self, msg: "ShapeTask") -> None:
        changed_shape = False
        next_shape_type = str(msg.shape_type).strip()
        if next_shape_type and next_shape_type != self.shape_type:
            self.shape_type = next_shape_type
            changed_shape = True

        self.active_task_id = int(msg.task_id)
        self.reference_center_received = True
        self.reference_center = [float(msg.center.position.x), float(msg.center.position.y)]
        self.reference_heading = float(msg.shape_heading)

        if self.refer_state is not None:
            self.refer_state.pos_x = self.reference_center[0]
            self.refer_state.pos_y = self.reference_center[1]
            self.refer_state.head = self.reference_heading

        if changed_shape:
            self.pending_shape_reload = True
            self._reload_shape_model()

        self._converged_since = -1.0
        self._converged_reported = False
        self.marker_needs_clear = True
        rospy.loginfo(
            "ShapeAssembly: task=%d center=(%.2f, %.2f) shape=%s heading=%.1fdeg",
            self.active_task_id,
            self.reference_center[0],
            self.reference_center[1],
            self.shape_type,
            math.degrees(self.reference_heading),
        )

    def _detect_namespaces(self) -> List[str]:
        prefix = "/" + self.namespace_prefix
        now = time.time()
        if self._detect_start_time <= 0.0:
            self._detect_start_time = now
            self._detect_last_change_time = now
            self._detect_best_ids = []

        found_ids = set()
        for topic, _ttype in rospy.get_published_topics():
            if topic.rfind(prefix, 0) != 0:
                continue
            id_begin = len(prefix)
            id_end = id_begin
            while id_end < len(topic) and topic[id_end].isdigit():
                id_end += 1
            if id_end == id_begin or id_end >= len(topic) or topic[id_end] != "/":
                continue
            if self.robot_detect_topic_suffix:
                if topic[id_end:id_end + len(self.robot_detect_topic_suffix)] != self.robot_detect_topic_suffix:
                    continue
            try:
                robot_id = int(topic[id_begin:id_end])
            except Exception:
                continue
            if robot_id > 0:
                found_ids.add(robot_id)

        found_ids_list = sorted(found_ids)
        should_update_best = (
            bool(found_ids_list) and (
                len(found_ids_list) > len(self._detect_best_ids) or
                found_ids_list != self._detect_best_ids
            )
        )
        if should_update_best:
            self._detect_best_ids = found_ids_list
            self._detect_last_change_time = now

        timeout_sec = self.robot_detect_timeout
        settle_sec = min(0.6, timeout_sec) if timeout_sec > 0.0 else 0.0
        deadline_hit = timeout_sec <= 0.0 or (now - self._detect_start_time) >= timeout_sec
        settled = settle_sec <= 0.0 or (now - self._detect_last_change_time) >= settle_sec
        if not self._detect_best_ids:
            return []
        if not settled and not deadline_hit:
            return []

        ns_list = [f"{self.namespace_prefix}{robot_id}" for robot_id in self._detect_best_ids]
        rospy.set_param("~resolved_robot_ids", list(self._detect_best_ids))
        rospy.set_param("~resolved_agent_number", len(self._detect_best_ids))
        return ns_list

    def _setup_io(self, ns_list: List[str]) -> None:
        self.ns_list = ns_list
        self.self_agent_index = -1
        if self.distributed_mode:
            if not self.distributed_agent_namespace:
                rospy.logwarn("ShapeAssembly: distributed_mode=true but distributed_agent_namespace is empty.")
            else:
                try:
                    self.self_agent_index = ns_list.index(self.distributed_agent_namespace)
                except ValueError:
                    rospy.logwarn(
                        "ShapeAssembly: distributed agent namespace %s not found in ns_list=%s",
                        self.distributed_agent_namespace,
                        ns_list,
                    )
        self.odom_msgs = [None for _ in ns_list]
        self.local_costmap_msgs = [None for _ in ns_list]
        self.odom_subs = []
        self.local_costmap_subs = []
        self.local_costmap_topics = []
        self.local_costmap_ready = [False for _ in ns_list]
        self.local_costmap_obs_cells = [0 for _ in ns_list]
        self.local_costmap_msg_age = [float("inf") for _ in ns_list]
        self.local_costmap_frame = ["" for _ in ns_list]
        self.cmd_pubs = [None for _ in ns_list]
        self.move_base_cancel_pubs = [None for _ in ns_list]
        costmap_suffix = self.local_costmap_topic_suffix.lstrip("/")
        odom_suffix = self.robot_odom_topic_suffix.lstrip("/")
        for idx, ns in enumerate(ns_list):
            odom_topic = f"/{ns}/{odom_suffix}" if ns and odom_suffix else f"/{ns}" if ns else f"/{odom_suffix}" if odom_suffix else "/odom"
            cmd_topic = f"/{ns}/cmd_vel" if ns else "/cmd_vel"
            cancel_topic = f"/{ns}/move_base/cancel" if ns else "/move_base/cancel"
            costmap_topic = f"/{ns}/{costmap_suffix}" if ns else f"/{costmap_suffix}"
            self.local_costmap_topics.append(costmap_topic)
            self.odom_subs.append(rospy.Subscriber(odom_topic, Odometry, self._odom_cb, callback_args=idx, queue_size=10))
            if (not self.distributed_mode) or idx == self.self_agent_index:
                self.cmd_pubs[idx] = rospy.Publisher(cmd_topic, Twist, queue_size=10)
            if self.use_local_costmap_avoid and ((not self.distributed_mode) or idx == self.self_agent_index):
                self.local_costmap_subs.append(
                    rospy.Subscriber(costmap_topic, OccupancyGrid, self._local_costmap_cb, callback_args=idx, queue_size=2)
                )
            if self.control_strategy == ControlStrategy.MOVE_BASE_THEN_SHAPE and ((not self.distributed_mode) or idx == self.self_agent_index):
                self.move_base_cancel_pubs[idx] = rospy.Publisher(cancel_topic, GoalID, queue_size=4)
        # Initialize per-robot control state whenever I/O is rebuilt.
        self.shape_ctrl_active = []
        self.shape_ctrl_hold_until = [0.0 for _ in ns_list]
        self.switch_gray_values = []
        rospy.loginfo("ShapeAssembly: connected to %s", ", ".join([ns or "(no-ns)" for ns in ns_list]))
        if self.distributed_mode:
            owner_ns = self.distributed_marker_owner.strip("/") if self.distributed_marker_owner else ""
            if owner_ns:
                self.marker_owner_active = owner_ns in ("*", self.distributed_agent_namespace)
            else:
                self.marker_owner_active = self.self_agent_index == 0
            rospy.loginfo(
                "ShapeAssembly: distributed agent=%s self_idx=%d marker_owner=%s",
                self.distributed_agent_namespace if self.distributed_agent_namespace else "(unset)",
                self.self_agent_index,
                str(self.marker_owner_active),
            )
        else:
            self.marker_owner_active = True
        self.marker_needs_clear = True
        self._update_takeover_state_param(self.shape_ctrl_active)

    def _maybe_init(self) -> None:
        if self.shape_state is not None:
            return
        if any(msg is None for msg in self.odom_msgs):
            return

        n = len(self.ns_list)
        self.sim_param.swarm_size = n

        robot_state = self._get_robot_state()
        if robot_state is None:
            return

        if self.reference_center_topic and self.reference_center_wait and (not self.reference_center_received):
            rospy.logwarn_throttle(2.0, "ShapeAssembly: waiting for reference center topic %s", self.reference_center_topic)
            return

        if self.reference_center_received and self.reference_center and len(self.reference_center) >= 2:
            ref_x = float(self.reference_center[0])
            ref_y = float(self.reference_center[1])
        elif self.reference_center and len(self.reference_center) >= 2:
            ref_x = float(self.reference_center[0])
            ref_y = float(self.reference_center[1])
        else:
            ref_x = sum(robot_state.pos_x) / n
            ref_y = sum(robot_state.pos_y) / n

        self.refer_state = ReferState(ref_x, ref_y)
        self.refer_state.head = self.reference_heading
        if (
            self.control_strategy == ControlStrategy.MOVE_BASE_THEN_SHAPE
            and self.switch_use_reference_shape
            and (not self.reference_center or len(self.reference_center) < 2)
            and not self.reference_center_topic
        ):
            rospy.logwarn(
                "move_base_then_shape uses reference shape for switching but reference_center is unset; "
                "switch region will be centered at initial robot average."
            )

        init_mode = str(self.init_center_mode).strip().lower()
        if init_mode not in ("robot", "average", "reference"):
            init_mode = "reference"
        init_cx = ref_x
        init_cy = ref_y
        if init_mode == "average":
            init_cx = sum(robot_state.pos_x) / n
            init_cy = sum(robot_state.pos_y) / n
        elif init_mode == "reference":
            if (not self.reference_center or len(self.reference_center) < 2) and not self.init_center_use_reference_only:
                init_cx = sum(robot_state.pos_x) / n
                init_cy = sum(robot_state.pos_y) / n
        else:
            init_cx = None
            init_cy = None

        self.shape_state = ShapeState(n)
        if init_mode == "robot":
            self.shape_state.pos_x = list(robot_state.pos_x)
            self.shape_state.pos_y = list(robot_state.pos_y)
        else:
            self.shape_state.pos_x = [init_cx for _ in range(n)]
            self.shape_state.pos_y = [init_cy for _ in range(n)]
        self.shape_state.vel_x = list(robot_state.vel_x)
        self.shape_state.vel_y = list(robot_state.vel_y)
        temp_head = [random.random() * 2.0 * math.pi for _ in range(n)]
        aveg_head = sum(temp_head) / n
        if aveg_head > 2.0 * math.pi:
            aveg_head -= 2.0 * math.pi
        self.shape_state.head = [h - aveg_head for h in temp_head]
        self.shape_state.hvel = [0.0 for _ in range(n)]

        leader_num = int(math.ceil(n * self.sim_param.leader_fraction))
        if leader_num > 0:
            self.inform_index = sorted(random.sample(range(n), leader_num))
        else:
            self.inform_index = []

        self._reload_shape_model()

        self.prev_cmd_x = [0.0] * n
        self.prev_cmd_y = [0.0] * n
        if self.control_strategy == ControlStrategy.MOVE_BASE_THEN_SHAPE:
            self.shape_ctrl_active = [False] * n
        else:
            self.shape_ctrl_active = [True] * n
        self.shape_ctrl_hold_until = [0.0] * n
        self.switch_gray_values = [1.0] * n
        self.marker_needs_clear = True

        rospy.loginfo(
            "ShapeAssembly: initialized with %d robots, leaders=%s init_center_mode=%s strategy=%s target=%s shape=%s center=(%.2f, %.2f) switch_ref_radius=%.2f",
            n,
            self.inform_index,
            init_mode,
            self.control_strategy,
            self.shape_target_mode,
            self._shape_desc,
            sum(self.shape_state.pos_x) / n,
            sum(self.shape_state.pos_y) / n,
            self.switch_reference_radius,
        )

    def _get_robot_state(self) -> Optional[RobotState]:
        if any(msg is None for msg in self.odom_msgs):
            return None
        n = len(self.odom_msgs)
        state = RobotState(n)
        for i, msg in enumerate(self.odom_msgs):
            state.pos_x[i] = msg.pose.pose.position.x
            state.pos_y[i] = msg.pose.pose.position.y
            state.vel_x[i] = msg.twist.twist.linear.x
            state.vel_y[i] = msg.twist.twist.linear.y
            ori = msg.pose.pose.orientation
            state.yaw[i] = _yaw_from_quat(ori.x, ori.y, ori.z, ori.w)
            if self.odom_twist_in_base:
                vx_body = state.vel_x[i]
                vy_body = state.vel_y[i]
                yaw = state.yaw[i]
                state.vel_x[i] = math.cos(yaw) * vx_body - math.sin(yaw) * vy_body
                state.vel_y[i] = math.sin(yaw) * vx_body + math.cos(yaw) * vy_body
        return state

    def _odom_cb(self, msg: Odometry, idx: int) -> None:
        if idx < len(self.odom_msgs):
            self.odom_msgs[idx] = msg

    def _local_costmap_cb(self, msg: OccupancyGrid, idx: int) -> None:
        if idx < len(self.local_costmap_msgs):
            self.local_costmap_msgs[idx] = msg

    def _auto_shape_heading_map_cb(self, msg: OccupancyGrid) -> None:
        self.auto_shape_heading_map_msg = msg

    def _is_local_costmap_obstacle(self, value: int) -> bool:
        if value < 0:
            return self.local_costmap_unknown_is_obstacle
        return value >= self.local_costmap_obstacle_threshold

    def _lookup_transform_2d(
        self,
        target_frame: str,
        source_frame: str,
        stamp: rospy.Time,
    ) -> Optional[Tuple[float, float, float]]:
        if not target_frame or not source_frame or target_frame == source_frame:
            return (0.0, 0.0, 0.0)
        if self.tf_buffer is None:
            return None
        query_stamp = rospy.Time(0)
        if stamp is not None and stamp.to_sec() > 0.0:
            query_stamp = stamp
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                query_stamp,
                rospy.Duration(0.03),
            )
        except Exception:
            return None
        t = tf_msg.transform.translation
        r = tf_msg.transform.rotation
        yaw = _yaw_from_quat(r.x, r.y, r.z, r.w)
        return (float(t.x), float(t.y), float(yaw))

    def _cal_local_obstacle_cmd(self, robot_state: RobotState) -> Tuple[List[float], List[float], List[float]]:
        n = self.sim_param.swarm_size
        cmd_x = [0.0] * n
        cmd_y = [0.0] * n
        nearest = [float("inf")] * n
        if not self.use_local_costmap_avoid:
            return cmd_x, cmd_y, nearest
        if len(self.local_costmap_msgs) != n:
            return cmd_x, cmd_y, nearest

        avoid_radius = max(1e-3, self.local_costmap_avoid_radius)
        hard_radius = max(1e-3, self.local_costmap_hard_radius)
        stride = max(1, self.local_costmap_stride)
        max_samples = max(1, self.local_costmap_max_samples)
        now_sec = rospy.Time.now().to_sec()

        for i in range(n):
            if self.distributed_mode and self.self_agent_index >= 0 and i != self.self_agent_index:
                continue
            if i < len(self.local_costmap_ready):
                self.local_costmap_ready[i] = False
            if i < len(self.local_costmap_obs_cells):
                self.local_costmap_obs_cells[i] = 0
            if i < len(self.local_costmap_msg_age):
                self.local_costmap_msg_age[i] = float("inf")
            if i < len(self.local_costmap_frame):
                self.local_costmap_frame[i] = ""
            msg = self.local_costmap_msgs[i]
            if msg is None:
                ns = self.ns_list[i] if i < len(self.ns_list) else f"robot{i+1}"
                topic = self.local_costmap_topics[i] if i < len(self.local_costmap_topics) else self.local_costmap_topic_suffix
                rospy.logwarn_throttle(3.0, "ShapeAssembly[%s]: waiting local costmap topic %s", ns, topic)
                continue
            odom_msg = self.odom_msgs[i] if i < len(self.odom_msgs) else None
            odom_frame = ""
            if odom_msg is not None:
                odom_frame = str(odom_msg.header.frame_id)
            costmap_frame = str(msg.header.frame_id)
            if i < len(self.local_costmap_frame):
                self.local_costmap_frame[i] = costmap_frame
            msg_age = 0.0
            if now_sec > 0.0 and msg.header.stamp.to_sec() > 0.0:
                msg_age = max(0.0, now_sec - msg.header.stamp.to_sec())
            if i < len(self.local_costmap_msg_age):
                self.local_costmap_msg_age[i] = msg_age
            if self.local_costmap_timeout > 0.0 and msg_age > self.local_costmap_timeout:
                ns = self.ns_list[i] if i < len(self.ns_list) else f"robot{i+1}"
                rospy.logwarn_throttle(
                    3.0,
                    "ShapeAssembly[%s]: stale local costmap age=%.2fs (> %.2fs), skip obstacle avoid.",
                    ns,
                    msg_age,
                    self.local_costmap_timeout,
                )
                continue
            tf_2d: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)
            if odom_msg is not None:
                if odom_frame and costmap_frame and odom_frame != costmap_frame:
                    ns = self.ns_list[i] if i < len(self.ns_list) else f"robot{i+1}"
                    tf_2d = self._lookup_transform_2d(odom_frame, costmap_frame, msg.header.stamp)
                    if tf_2d is None:
                        rospy.logwarn_throttle(
                            5.0,
                            "ShapeAssembly[%s]: frame mismatch odom=%s costmap=%s and tf unavailable.",
                            ns,
                            odom_frame,
                            costmap_frame,
                        )
                        if self.local_costmap_skip_on_frame_mismatch:
                            continue
                    else:
                        rospy.loginfo_throttle(
                            5.0,
                            "ShapeAssembly[%s]: local costmap transformed %s -> %s via tf2.",
                            ns,
                            costmap_frame,
                            odom_frame,
                        )
            width = int(msg.info.width)
            height = int(msg.info.height)
            resolution = float(msg.info.resolution)
            if width <= 0 or height <= 0 or resolution <= 0:
                continue
            total = width * height
            if len(msg.data) < total:
                continue

            rx = robot_state.pos_x[i]
            ry = robot_state.pos_y[i]
            origin_x = float(msg.info.origin.position.x)
            origin_y = float(msg.info.origin.position.y)
            q = msg.info.origin.orientation
            origin_yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
            cos_origin = math.cos(origin_yaw)
            sin_origin = math.sin(origin_yaw)

            sum_x = 0.0
            sum_y = 0.0
            hard_x = 0.0
            hard_y = 0.0
            sampled = 0
            obstacle_cells = 0
            tf_cos = 1.0
            tf_sin = 0.0
            tf_tx = 0.0
            tf_ty = 0.0
            if tf_2d is not None:
                tf_tx, tf_ty, tf_yaw = tf_2d
                tf_cos = math.cos(tf_yaw)
                tf_sin = math.sin(tf_yaw)

            for row in range(0, height, stride):
                base = row * width
                for col in range(0, width, stride):
                    idx = base + col
                    if idx >= total:
                        continue
                    value = int(msg.data[idx])
                    if not self._is_local_costmap_obstacle(value):
                        continue
                    obstacle_cells += 1
                    cell_x = (col + 0.5) * resolution
                    cell_y = (row + 0.5) * resolution
                    obs_x = origin_x + cos_origin * cell_x - sin_origin * cell_y
                    obs_y = origin_y + sin_origin * cell_x + cos_origin * cell_y
                    if tf_2d is not None:
                        src_x = obs_x
                        src_y = obs_y
                        obs_x = tf_tx + tf_cos * src_x - tf_sin * src_y
                        obs_y = tf_ty + tf_sin * src_x + tf_cos * src_y
                    dx = rx - obs_x
                    dy = ry - obs_y
                    dist = math.hypot(dx, dy)
                    if dist < nearest[i]:
                        nearest[i] = dist
                    if dist <= max(1e-6, self.local_costmap_self_ignore_radius) or dist > avoid_radius:
                        continue

                    unit_x = dx / dist
                    unit_y = dy / dist
                    closeness = (avoid_radius - dist) / avoid_radius
                    sum_x += closeness * unit_x
                    sum_y += closeness * unit_y
                    if dist < hard_radius:
                        hard = (hard_radius - dist) / max(dist, 1e-3)
                        hard_x += hard * unit_x
                        hard_y += hard * unit_y
                    sampled += 1
                    if sampled >= max_samples:
                        break
                if sampled >= max_samples:
                    break

            cmd_x[i] = self.local_costmap_avoid_gain * sum_x + self.local_costmap_hard_gain * hard_x
            cmd_y[i] = self.local_costmap_avoid_gain * sum_y + self.local_costmap_hard_gain * hard_y
            if i < len(self.local_costmap_obs_cells):
                self.local_costmap_obs_cells[i] = obstacle_cells
            if i < len(self.local_costmap_ready):
                self.local_costmap_ready[i] = True

        return cmd_x, cmd_y, nearest

    def _build_shape_overlap_samples(self) -> None:
        self._shape_overlap_samples = []
        if self.shape_info is None or self.shape_mtr is None:
            return
        stride = max(1, self.auto_shape_heading_shape_stride)
        for r in range(0, self.shape_info.rn, stride):
            for c in range(0, self.shape_info.cn, stride):
                gray = float(self.shape_mtr.value[r][c])
                if gray >= 1.0:
                    continue
                weight = max(0.05, 1.0 - gray)
                if gray <= self.shape_black_threshold:
                    weight += 1.0
                self._shape_overlap_samples.append(
                    (float(self.shape_mtr.base_x[r][c]), float(self.shape_mtr.base_y[r][c]), weight)
                )

    def _is_auto_shape_heading_obstacle(self, value: int) -> bool:
        if value < 0:
            return self.auto_shape_heading_unknown_is_obstacle
        return value >= self.auto_shape_heading_obstacle_threshold

    def _world_to_occ_index(self, occ_msg: OccupancyGrid, wx: float, wy: float) -> Optional[int]:
        width = int(occ_msg.info.width)
        height = int(occ_msg.info.height)
        resolution = float(occ_msg.info.resolution)
        if width <= 0 or height <= 0 or resolution <= 0:
            return None
        origin_x = float(occ_msg.info.origin.position.x)
        origin_y = float(occ_msg.info.origin.position.y)
        q = occ_msg.info.origin.orientation
        origin_yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
        dx = wx - origin_x
        dy = wy - origin_y
        # Inverse transform world -> map grid coordinates.
        local_x = math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy
        local_y = -math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy
        col = int(math.floor(local_x / resolution))
        row = int(math.floor(local_y / resolution))
        if col < 0 or row < 0 or col >= width or row >= height:
            return None
        idx = row * width + col
        if idx < 0 or idx >= len(occ_msg.data):
            return None
        return idx

    def _score_heading_overlap(self, head: float, occ_msg: OccupancyGrid) -> float:
        if self.refer_state is None:
            return float("inf")
        if not self._shape_overlap_samples:
            return float("inf")
        cx = float(self.refer_state.pos_x)
        cy = float(self.refer_state.pos_y)
        cos_h = math.cos(head)
        sin_h = math.sin(head)
        sum_w = 0.0
        hit_w = 0.0
        for base_x, base_y, weight in self._shape_overlap_samples:
            wx = base_x * cos_h - base_y * sin_h + cx
            wy = base_x * sin_h + base_y * cos_h + cy
            idx = self._world_to_occ_index(occ_msg, wx, wy)
            sum_w += weight
            if idx is None:
                if self.auto_shape_heading_oob_is_obstacle:
                    hit_w += weight
                continue
            if self._is_auto_shape_heading_obstacle(int(occ_msg.data[idx])):
                hit_w += weight
        if sum_w <= 1e-9:
            return float("inf")
        return hit_w / sum_w

    def _optimize_reference_heading_by_obstacle(self) -> None:
        if not self.auto_shape_heading:
            return
        if self.shape_target_mode != ShapeTargetMode.REFERENCE:
            return
        if self.refer_state is None:
            return
        occ_msg = self.auto_shape_heading_map_msg
        if occ_msg is None:
            return
        if not self._shape_overlap_samples:
            self._build_shape_overlap_samples()
            if not self._shape_overlap_samples:
                return
        now = time.time()
        if now - self._last_auto_heading_time < self.auto_shape_heading_update_interval:
            return
        self._last_auto_heading_time = now

        step_rad = math.radians(self.auto_shape_heading_angle_step_deg)
        if step_rad <= 1e-6:
            return
        current_head = _limit_angle(float(self.refer_state.head))
        current_overlap = self._score_heading_overlap(current_head, occ_msg)
        if not math.isfinite(current_overlap):
            return

        best_head = current_head
        best_overlap = current_overlap
        samples = max(8, int(math.ceil((2.0 * math.pi) / step_rad)))
        for k in range(samples):
            cand_head = -math.pi + (2.0 * math.pi * float(k) / float(samples))
            cand_overlap = self._score_heading_overlap(cand_head, occ_msg)
            if not math.isfinite(cand_overlap):
                continue
            cand_bias = self.auto_shape_heading_yaw_bias * (abs(_limit_angle(cand_head - current_head)) / math.pi)
            best_bias = self.auto_shape_heading_yaw_bias * (abs(_limit_angle(best_head - current_head)) / math.pi)
            if (cand_overlap + cand_bias) < (best_overlap + best_bias):
                best_head = cand_head
                best_overlap = cand_overlap

        if (current_overlap - best_overlap) < self.auto_shape_heading_min_improve:
            return
        self.refer_state.head = best_head
        self.reference_heading = best_head
        rospy.loginfo(
            "ShapeAssembly: auto heading %.1f -> %.1f deg overlap %.3f -> %.3f",
            math.degrees(current_head),
            math.degrees(best_head),
            current_overlap,
            best_overlap,
        )

    def _reference_center_cb(self, msg: PoseStamped) -> None:
        self.reference_center_received = True
        self.reference_center = [float(msg.pose.position.x), float(msg.pose.position.y)]
        if self.reference_center_use_topic_heading:
            q = msg.pose.orientation
            self.reference_heading = _yaw_from_quat(q.x, q.y, q.z, q.w)
        if self.refer_state is not None:
            self.refer_state.pos_x = float(msg.pose.position.x)
            self.refer_state.pos_y = float(msg.pose.position.y)
            if self.reference_center_use_topic_heading:
                self.refer_state.head = self.reference_heading
        self._converged_since = -1.0
        self._converged_reported = False
        self.marker_needs_clear = True

    def _build_reference_target_state(self, base_state: ShapeState) -> ShapeState:
        n = self.sim_param.swarm_size
        target = ShapeState(n)
        target.pos_x = list(base_state.pos_x)
        target.pos_y = list(base_state.pos_y)
        target.vel_x = list(base_state.vel_x)
        target.vel_y = list(base_state.vel_y)
        target.head = list(base_state.head)
        target.hvel = list(base_state.hvel)
        if self.refer_state is None:
            return target
        target.pos_x = [self.refer_state.pos_x] * n
        target.pos_y = [self.refer_state.pos_y] * n
        target.head = [self.refer_state.head] * n
        target.vel_x = [self.refer_state.vel_x] * n
        target.vel_y = [self.refer_state.vel_y] * n
        target.hvel = [self.refer_state.hvel] * n
        return target

    def _gray_value_in_shape(
        self,
        pos_x: float,
        pos_y: float,
        center_x: float,
        center_y: float,
        head: float,
    ) -> float:
        if self.shape_info is None or self.shape_mtr is None:
            return 1.0
        # Convert world point into shape-grid coordinate and read local gray value.
        # Convention in this implementation: lower gray means deeper inside target area.
        location = trans_goal_to_local(
            pos_x,
            pos_y,
            center_x,
            center_y,
            head,
            self.shape_info.grid,
            self.shape_info.cen_x,
            self.shape_info.cen_y,
            self.shape_info.rn,
            self.shape_info.cn,
        )
        return get_gray_value(location, self.shape_mtr.value)

    def _compute_switch_gray_values(self, robot_state: RobotState) -> List[float]:
        n = self.sim_param.swarm_size
        values = [1.0] * n
        if self.shape_state is None:
            return values

        use_reference = self.switch_use_reference_shape and self.refer_state is not None
        # Switching can use either:
        # 1) global reference shape (stable switch region),
        # 2) negotiated local shape of each robot (adaptive region).
        for i in range(n):
            if use_reference and self.refer_state is not None:
                center_x = self.refer_state.pos_x
                center_y = self.refer_state.pos_y
                head = self.refer_state.head
            else:
                center_x = self.shape_state.pos_x[i]
                center_y = self.shape_state.pos_y[i]
                head = self.shape_state.head[i]
            values[i] = self._gray_value_in_shape(
                robot_state.pos_x[i],
                robot_state.pos_y[i],
                center_x,
                center_y,
                head,
            )
        return values

    def _compute_reference_switch_radius(self) -> float:
        radius = self.switch_reference_radius_min
        if not self.switch_reference_radius_enable:
            return radius
        if self.shape_info is None or self.shape_mtr is None:
            return radius

        half_grid = 0.5 * max(self.shape_info.grid, 0.0)
        max_dist = 0.0
        for r in range(self.shape_info.rn):
            for c in range(self.shape_info.cn):
                if self.shape_mtr.value[r][c] >= 1.0:
                    continue
                cell_dist = math.hypot(self.shape_mtr.base_x[r][c], self.shape_mtr.base_y[r][c]) + half_grid
                if cell_dist > max_dist:
                    max_dist = cell_dist

        if max_dist <= 0.0:
            return radius
        return max(radius, max_dist + self.switch_reference_radius_margin)

    def _update_control_switch(self, robot_state: RobotState) -> List[bool]:
        n = self.sim_param.swarm_size
        if self.control_strategy != ControlStrategy.MOVE_BASE_THEN_SHAPE:
            self.shape_ctrl_active = [True] * n
            self.shape_ctrl_hold_until = [0.0] * n
            self.switch_gray_values = [0.0] * n
            return self.shape_ctrl_active

        if len(self.shape_ctrl_active) != n:
            self.shape_ctrl_active = [False] * n
        if len(self.shape_ctrl_hold_until) != n:
            self.shape_ctrl_hold_until = [0.0] * n

        self.switch_gray_values = self._compute_switch_gray_values(robot_state)
        threshold = max(0.0, min(1.0, self.switch_gray_threshold))
        reference_radius = max(0.0, self.switch_reference_radius)
        now = time.time()

        switched_indices: List[int] = []
        released_indices: List[int] = []
        switched_reasons: List[str] = []
        released_reasons: List[str] = []
        for i in range(n):
            gray_trigger = self.switch_gray_values[i] < threshold
            radius_trigger = False
            if (
                not gray_trigger
                and self.switch_reference_radius_enable
                and self.refer_state is not None
                and reference_radius > 1e-6
            ):
                dx = robot_state.pos_x[i] - self.refer_state.pos_x
                dy = robot_state.pos_y[i] - self.refer_state.pos_y
                radius_trigger = math.hypot(dx, dy) <= reference_radius

            next_active = gray_trigger or radius_trigger
            if next_active and not self.shape_ctrl_active[i]:
                self.shape_ctrl_active[i] = True
                if self.cancel_grace_period > 0.0:
                    self.shape_ctrl_hold_until[i] = now + self.cancel_grace_period
                switched_indices.append(i)
                if gray_trigger:
                    switched_reasons.append("gray")
                else:
                    switched_reasons.append("radius")
            elif (not next_active) and self.shape_ctrl_active[i]:
                self.shape_ctrl_active[i] = False
                self.shape_ctrl_hold_until[i] = 0.0
                released_indices.append(i)
                released_reasons.append("outside")

        if switched_indices:
            if self.cancel_move_base_on_switch:
                cancel_msg = GoalID()
                cancel_msg.stamp = rospy.Time.now()
                for idx in switched_indices:
                    if idx < len(self.move_base_cancel_pubs):
                        pub = self.move_base_cancel_pubs[idx]
                        if pub is not None:
                            pub.publish(cancel_msg)
            switched_names = [self.ns_list[idx] if idx < len(self.ns_list) else f"robot{idx+1}" for idx in switched_indices]
            rospy.loginfo(
                "ShapeAssembly: switched to shape control for %s (gray<th=%.3f ref_r=%.2f active=%d/%d reasons=%s).",
                ", ".join(switched_names),
                threshold,
                reference_radius,
                sum(1 for v in self.shape_ctrl_active if v),
                n,
                ",".join(switched_reasons),
            )

        if released_indices:
            released_names = [self.ns_list[idx] if idx < len(self.ns_list) else f"robot{idx+1}" for idx in released_indices]
            rospy.loginfo(
                "ShapeAssembly: released to move_base for %s (active=%d/%d reasons=%s).",
                ", ".join(released_names),
                sum(1 for v in self.shape_ctrl_active if v),
                n,
                ",".join(released_reasons),
            )

        return self.shape_ctrl_active

    def _cancel_move_base_for_active(self, control_active: List[bool]) -> None:
        if self.control_strategy != ControlStrategy.MOVE_BASE_THEN_SHAPE:
            return
        if not self.cancel_move_base_on_switch:
            return
        cancel_msg = GoalID()
        cancel_msg.stamp = rospy.Time.now()
        for idx, active in enumerate(control_active):
            if not active:
                continue
            if idx >= len(self.move_base_cancel_pubs):
                continue
            pub = self.move_base_cancel_pubs[idx]
            if pub is not None:
                pub.publish(cancel_msg)

    def _publish_zero(self) -> None:
        for pub in self.cmd_pubs:
            if pub is not None:
                pub.publish(Twist())

    def _parse_debug_indices(self) -> List[int]:
        if not self.debug_log_robot_indices:
            return []
        if isinstance(self.debug_log_robot_indices, list):
            result = []
            for item in self.debug_log_robot_indices:
                try:
                    result.append(int(item))
                except Exception:
                    continue
            if result and all(v >= 1 for v in result):
                if max(result) <= self.sim_param.swarm_size:
                    return [v - 1 for v in result]
            return result
        try:
            raw = str(self.debug_log_robot_indices)
            result = []
            for token in raw.replace(",", " ").split():
                result.append(int(token))
            if result and all(v >= 1 for v in result):
                if max(result) <= self.sim_param.swarm_size:
                    return [v - 1 for v in result]
            return result
        except Exception:
            return []

    def _monitor_source(self) -> str:
        if self.robot_namespaces:
            return "robot_namespaces"
        if self.auto_detect:
            return "auto_detect"
        if self.agent_number > 0:
            return "agent_number"
        return "none"

    def _extract_robot_id_label(self, ns: str, index: int) -> str:
        name = ns.strip("/")
        if name and self.namespace_prefix and name.startswith(self.namespace_prefix):
            suffix = name[len(self.namespace_prefix):]
            if suffix.isdigit():
                return suffix
        match = re.search(r"(\d+)$", name)
        if match:
            return match.group(1)
        if name:
            return name
        return str(index + 1)

    def _extract_robot_id_value(self, ns: str, index: int) -> Optional[int]:
        label = self._extract_robot_id_label(ns, index)
        try:
            value = int(label)
        except Exception:
            return None
        if value <= 0:
            return None
        return value

    def _active_shape_robot_ids(self, control_active: Optional[List[bool]] = None) -> List[int]:
        active = control_active if control_active is not None else self.shape_ctrl_active
        result: List[int] = []
        for idx, enabled in enumerate(active):
            if not enabled:
                continue
            robot_id = self._extract_robot_id_value(
                self.ns_list[idx] if idx < len(self.ns_list) else "",
                idx,
            )
            if robot_id is not None:
                result.append(robot_id)
        return result

    def _update_takeover_state_param(self, control_active: Optional[List[bool]] = None) -> None:
        if self.distributed_mode and self.ns_list and not self.marker_owner_active:
            return

        stop_path_planning = False
        active_ids: List[int] = []
        if self.control_strategy == ControlStrategy.MOVE_BASE_THEN_SHAPE:
            active_ids = self._active_shape_robot_ids(control_active)
        if (
            self.stop_path_planning_on_shape_takeover
            and self.control_strategy == ControlStrategy.MOVE_BASE_THEN_SHAPE
        ):
            stop_path_planning = bool(self.ns_list) and len(active_ids) == len(self.ns_list)

        if (
            self._last_stop_path_planning_state == stop_path_planning
            and self._last_stop_path_planning_ids == active_ids
        ):
            return

        rospy.set_param("/shape_assembly/active_robot_ids", active_ids)
        rospy.set_param("/shape_assembly/stop_path_planning", stop_path_planning)

        if self._last_stop_path_planning_state is not None:
            if stop_path_planning:
                rospy.loginfo(
                    "ShapeAssembly: stop path planning after shape takeover, active_ids=%s",
                    active_ids,
                )
            else:
                rospy.loginfo("ShapeAssembly: cleared stop-path-planning takeover state.")

        self._last_stop_path_planning_state = stop_path_planning
        self._last_stop_path_planning_ids = list(active_ids)

    def _publish_local_status(self, robot_state: RobotState, control_active: List[bool]) -> None:
        if RobotFormationStatus is None or self.robot_status_pub is None or not self.ns_list:
            return

        status_index = 0
        if self.distributed_mode and self.self_agent_index >= 0:
            status_index = self.self_agent_index
        if status_index < 0 or status_index >= len(self.ns_list):
            return

        ns = self.ns_list[status_index] if status_index < len(self.ns_list) else ""
        robot_id = self._extract_robot_id_value(ns, status_index)
        shape_active = status_index < len(control_active) and bool(control_active[status_index])
        gray_value = self.switch_gray_values[status_index] if status_index < len(self.switch_gray_values) else 1.0
        inside_shape = gray_value < max(0.0, min(1.0, self.switch_gray_threshold))
        yaw = robot_state.yaw[status_index]

        msg = RobotFormationStatus()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.marker_frame
        msg.task_id = max(0, int(self.active_task_id))
        msg.robot_id = robot_id if robot_id is not None else status_index + 1
        msg.robot_namespace = ns
        if self._converged_reported:
            msg.phase = RobotFormationStatus.PHASE_CONVERGED
        elif shape_active:
            msg.phase = RobotFormationStatus.PHASE_SHAPE_ACTIVE
        elif self.control_strategy == ControlStrategy.MOVE_BASE_THEN_SHAPE:
            msg.phase = RobotFormationStatus.PHASE_NAVIGATE
        else:
            msg.phase = RobotFormationStatus.PHASE_STAGING
        msg.move_base_state = 255
        msg.inside_shape = inside_shape
        msg.shape_active = shape_active
        msg.converged = bool(self._converged_reported)
        msg.pose.position.x = robot_state.pos_x[status_index]
        msg.pose.position.y = robot_state.pos_y[status_index]
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(0.5 * yaw)
        msg.pose.orientation.w = math.cos(0.5 * yaw)
        if self.refer_state is not None:
            dx = robot_state.pos_x[status_index] - self.refer_state.pos_x
            dy = robot_state.pos_y[status_index] - self.refer_state.pos_y
            msg.distance_to_center = math.hypot(dx, dy)
        else:
            msg.distance_to_center = 0.0
        msg.note = self.shape_type
        self.robot_status_pub.publish(msg)

    def _format_monitor_robot_ids(self, indices: List[int]) -> str:
        if not indices:
            return "[]"
        labels = [
            self._extract_robot_id_label(self.ns_list[idx] if idx < len(self.ns_list) else "", idx)
            for idx in indices
        ]
        return "[" + ", ".join(labels) + "]"

    def _should_report_monitor_status(self) -> bool:
        if not self.monitor_report_enabled:
            return False
        if self.monitor_report_interval <= 0.0:
            return False
        if self.distributed_mode and self.ns_list and not self.marker_owner_active:
            return False
        now = time.time()
        if now - self._last_monitor_report_time < self.monitor_report_interval:
            return False
        self._last_monitor_report_time = now
        return True

    def _maybe_report_monitored_robots(self) -> None:
        if not self._should_report_monitor_status():
            return

        source = self._monitor_source()
        if not self.ns_list:
            if source == "auto_detect":
                rospy.loginfo(
                    "ShapeAssembly monitor: source=%s monitored=0 waiting for /%s*%s topics.",
                    source,
                    self.namespace_prefix,
                    self.robot_detect_topic_suffix,
                )
            else:
                rospy.loginfo("ShapeAssembly monitor: source=%s monitored=0.", source)
            return

        ready_indices = [idx for idx, msg in enumerate(self.odom_msgs) if msg is not None]
        missing_indices = [idx for idx, msg in enumerate(self.odom_msgs) if msg is None]
        namespaces = "[" + ", ".join([ns or "(root)" for ns in self.ns_list]) + "]"
        rospy.loginfo(
            "ShapeAssembly monitor: source=%s monitored=%d namespaces=%s ids=%s odom_ready=%s odom_missing=%s",
            source,
            len(self.ns_list),
            namespaces,
            self._format_monitor_robot_ids(list(range(len(self.ns_list)))),
            self._format_monitor_robot_ids(ready_indices),
            self._format_monitor_robot_ids(missing_indices),
        )

    def _update_converged_status(self, metric: SwarmMetric) -> None:
        now = time.time()
        ready = (
            metric.inside_rate >= self.converge_inside_rate
            and metric.enter_rate >= self.converge_enter_rate
            and metric.cover_rate >= self.converge_cover_rate
        )
        if not ready:
            self._converged_since = -1.0
            self._converged_reported = False
            return
        if self._converged_since < 0.0:
            self._converged_since = now
            return
        if self._converged_reported:
            return
        if now - self._converged_since >= self.converge_hold_time:
            self._converged_reported = True
            rospy.loginfo(
                "ShapeAssembly: converged after %.2fs (inside=%.3f enter=%.3f cover=%.3f).",
                now - self._converged_since,
                metric.inside_rate,
                metric.enter_rate,
                metric.cover_rate,
            )

    def _maybe_log_debug(
        self,
        robot_state: RobotState,
        neigh: Dict[str, List[List[float]]],
        metric: Optional[SwarmMetric],
        cmd_x: List[float],
        cmd_y: List[float],
        pub_cmd_x: List[float],
        pub_cmd_y: List[float],
        cmd_enter_x: Optional[List[float]] = None,
        cmd_enter_y: Optional[List[float]] = None,
        cmd_explore_x: Optional[List[float]] = None,
        cmd_explore_y: Optional[List[float]] = None,
        cmd_interact_x: Optional[List[float]] = None,
        cmd_interact_y: Optional[List[float]] = None,
        cmd_obs_x: Optional[List[float]] = None,
        cmd_obs_y: Optional[List[float]] = None,
        enter_goals: Optional[List[Tuple[float, float, float]]] = None,
        control_active: Optional[List[bool]] = None,
        switch_gray_values: Optional[List[float]] = None,
        local_obs_nearest: Optional[List[float]] = None,
    ) -> None:
        if not self.debug_log:
            return
        now = time.time()
        if self.debug_log_hz <= 0:
            return
        if now - self._last_debug_time < 1.0 / self.debug_log_hz:
            return
        self._last_debug_time = now

        indices = self._parse_debug_indices()
        if indices:
            indices = [i for i in indices if 0 <= i < self.sim_param.swarm_size]
        else:
            indices = list(range(self.sim_param.swarm_size))

        if self.refer_state is not None:
            active_num = self.sim_param.swarm_size
            if control_active and len(control_active) == self.sim_param.swarm_size:
                active_num = int(sum(1 for v in control_active if v))
            rospy.loginfo(
                "shape_assembly dbg: ref_center=(%.2f, %.2f) ref_head=%.2f cmd_in_map=%s odom_in_base=%s strategy=%s target=%s active=%d/%d shape=%s",
                self.refer_state.pos_x,
                self.refer_state.pos_y,
                self.refer_state.head,
                str(self.cmd_in_map_frame),
                str(self.odom_twist_in_base),
                self.control_strategy,
                self.shape_target_mode,
                active_num,
                self.sim_param.swarm_size,
                self._shape_desc,
            )
        if metric is not None and self.debug_log_metric:
            rospy.loginfo("shape_assembly metric: %s", format_metric(metric))

        a_mtr = neigh.get("a_mtr", [])
        for i in indices:
            neigh_count = 0
            if a_mtr:
                neigh_count = int(sum(a_mtr[i]))
            ns = self.ns_list[i] if i < len(self.ns_list) else f"robot{i+1}"
            ctrl_mode = "shape"
            if control_active and i < len(control_active) and not control_active[i]:
                ctrl_mode = "move_base"
            sw_gray = -1.0
            if switch_gray_values and i < len(switch_gray_values):
                sw_gray = switch_gray_values[i]
            lc_ok = False
            lc_obs = 0
            lc_age = float("inf")
            if i < len(self.local_costmap_ready):
                lc_ok = self.local_costmap_ready[i]
            if i < len(self.local_costmap_obs_cells):
                lc_obs = self.local_costmap_obs_cells[i]
            if i < len(self.local_costmap_msg_age):
                lc_age = self.local_costmap_msg_age[i]
            lc_age_print = lc_age if math.isfinite(lc_age) else -1.0
            speed = math.hypot(robot_state.vel_x[i], robot_state.vel_y[i])
            cmd_speed = math.hypot(cmd_x[i], cmd_y[i])
            pub_speed = math.hypot(pub_cmd_x[i], pub_cmd_y[i])
            rospy.loginfo(
                "dbg %s mode=%s sw_gray=%.2f lc_ok=%s lc_obs=%d lc_age=%.2f pos(%.2f,%.2f) yaw=%.2f neigh=%d v_odom(%.2f,%.2f)|%.2f cmd_map(%.2f,%.2f)|%.2f cmd_pub(%.2f,%.2f)|%.2f",
                ns,
                ctrl_mode,
                sw_gray,
                str(lc_ok),
                lc_obs,
                lc_age_print,
                robot_state.pos_x[i],
                robot_state.pos_y[i],
                robot_state.yaw[i],
                neigh_count,
                robot_state.vel_x[i],
                robot_state.vel_y[i],
                speed,
                cmd_x[i],
                cmd_y[i],
                cmd_speed,
                pub_cmd_x[i],
                pub_cmd_y[i],
                pub_speed,
            )
            if self.debug_log_components and cmd_enter_x and cmd_explore_x and cmd_interact_x and cmd_obs_x and cmd_obs_y:
                ce = math.hypot(cmd_enter_x[i], cmd_enter_y[i])
                cx = math.hypot(cmd_explore_x[i], cmd_explore_y[i])
                ci = math.hypot(cmd_interact_x[i], cmd_interact_y[i])
                co = math.hypot(cmd_obs_x[i], cmd_obs_y[i])
                obs_near = float("inf")
                if local_obs_nearest and i < len(local_obs_nearest):
                    obs_near = local_obs_nearest[i]
                rospy.loginfo(
                    "dbg %s comp enter(%.2f,%.2f)|%.2f explore(%.2f,%.2f)|%.2f interact(%.2f,%.2f)|%.2f obst(%.2f,%.2f)|%.2f obs_near=%.2f",
                    ns,
                    cmd_enter_x[i],
                    cmd_enter_y[i],
                    ce,
                    cmd_explore_x[i],
                    cmd_explore_y[i],
                    cx,
                    cmd_interact_x[i],
                    cmd_interact_y[i],
                    ci,
                    cmd_obs_x[i],
                    cmd_obs_y[i],
                    co,
                    obs_near,
                )
                if enter_goals and i < len(enter_goals):
                    goal_x, goal_y, goal_gray = enter_goals[i]
                    rospy.loginfo(
                        "dbg %s goal(%.2f,%.2f) gray=%.2f",
                        ns,
                        goal_x,
                        goal_y,
                        goal_gray,
                    )

    def _avg_head(self, heads: List[float]) -> float:
        if not heads:
            return 0.0
        sx = sum(math.cos(h) for h in heads)
        sy = sum(math.sin(h) for h in heads)
        if abs(sx) < 1e-9 and abs(sy) < 1e-9:
            return 0.0
        return math.atan2(sy, sx)

    def _publish_markers(
        self,
        robot_state: RobotState,
        cmd_x: List[float],
        cmd_y: List[float],
        neigh: Optional[Dict[str, List[List[float]]]] = None,
        shape_dyn: Optional[ShapeDyn] = None,
        enter_goals: Optional[List[Tuple[float, float, float]]] = None,
        metric: Optional[SwarmMetric] = None,
    ) -> None:
        if not self.publish_markers or self.marker_pub is None:
            return
        if self.distributed_mode and not self.marker_owner_active:
            return
        if self.shape_state is None or self.shape_mtr is None or self.shape_info is None:
            return

        now = rospy.Time.now()
        markers = MarkerArray()
        if self.marker_needs_clear:
            clear_marker = Marker()
            clear_marker.action = Marker.DELETEALL
            markers.markers.append(clear_marker)
            self.marker_needs_clear = False

        # Target shape marker (points)
        def _mk_color(rgb: List[float], alpha: float) -> ColorRGBA:
            if not isinstance(rgb, list) or len(rgb) < 3:
                rgb = [1.0, 1.0, 1.0]
            return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=alpha)

        def _blend_color(start_rgb: List[float], end_rgb: List[float], ratio: float, alpha: float) -> ColorRGBA:
            ratio = min(max(float(ratio), 0.0), 1.0)
            if not isinstance(start_rgb, list) or len(start_rgb) < 3:
                start_rgb = [1.0, 1.0, 1.0]
            if not isinstance(end_rgb, list) or len(end_rgb) < 3:
                end_rgb = [1.0, 1.0, 1.0]
            return ColorRGBA(
                r=float(start_rgb[0]) + (float(end_rgb[0]) - float(start_rgb[0])) * ratio,
                g=float(start_rgb[1]) + (float(end_rgb[1]) - float(start_rgb[1])) * ratio,
                b=float(start_rgb[2]) + (float(end_rgb[2]) - float(start_rgb[2])) * ratio,
                a=alpha,
            )

        def _mk_delete(ns: str, marker_id: int) -> Marker:
            m = Marker()
            m.header.frame_id = self.marker_frame
            m.header.stamp = now
            m.ns = ns
            m.id = marker_id
            m.action = Marker.DELETE
            return m

        black_marker = Marker()
        black_marker.header.frame_id = self.marker_frame
        black_marker.header.stamp = now
        black_marker.ns = "shape"
        black_marker.id = 0
        black_marker.type = Marker.POINTS
        black_marker.action = Marker.ADD
        black_marker.scale.x = self.shape_point_size
        black_marker.scale.y = self.shape_point_size
        black_marker.color = _mk_color(self.shape_black_color, self.shape_black_alpha)

        gray_marker = Marker()
        gray_marker.header.frame_id = self.marker_frame
        gray_marker.header.stamp = now
        gray_marker.ns = "shape"
        gray_marker.id = 1
        gray_marker.type = Marker.POINTS
        gray_marker.action = Marker.ADD
        gray_marker.scale.x = self.shape_point_size
        gray_marker.scale.y = self.shape_point_size
        gray_marker.color = _mk_color(self.shape_gray_color, self.shape_gray_alpha)

        gradient_marker = Marker()
        gradient_marker.header.frame_id = self.marker_frame
        gradient_marker.header.stamp = now
        gradient_marker.ns = "shape"
        gradient_marker.id = 4
        gradient_marker.type = Marker.POINTS
        gradient_marker.action = Marker.ADD
        gradient_marker.scale.x = self.shape_point_size
        gradient_marker.scale.y = self.shape_point_size
        gradient_marker.color = _mk_color([1.0, 1.0, 1.0], self.shape_gradient_alpha)

        color_mode = str(self.shape_color_mode).strip().lower()
        use_gradient = color_mode in ("gradient", "gray", "grayscale")

        def _append_shape_point(row: int, col: int, px: float, py: float) -> None:
            gray_value = min(max(float(self.shape_mtr.value[row][col]), 0.0), 1.0)
            if use_gradient:
                gradient_marker.points.append(Point(x=px, y=py, z=0.0))
                gradient_marker.colors.append(
                    _blend_color(
                        self.shape_black_color,
                        self.shape_gray_color,
                        gray_value,
                        self.shape_gradient_alpha,
                    )
                )
            elif gray_value <= self.shape_black_threshold:
                black_marker.points.append(Point(x=px, y=py, z=0.0))
            else:
                gray_marker.points.append(Point(x=px, y=py, z=0.0))

        stride = self.shape_point_stride
        n = max(1, self.sim_param.swarm_size)
        marker_mode = str(self.shape_marker_mode).strip().lower()
        if marker_mode == "reference" and self.refer_state is not None:
            center_x = self.refer_state.pos_x
            center_y = self.refer_state.pos_y
            head = self.refer_state.head
            cos_h = math.cos(head)
            sin_h = math.sin(head)
            for r in range(0, self.shape_info.rn, stride):
                for c in range(0, self.shape_info.cn, stride):
                    if self.shape_mtr.value[r][c] >= 1.0:
                        continue
                    base_x = self.shape_mtr.base_x[r][c]
                    base_y = self.shape_mtr.base_y[r][c]
                    rx = base_x * cos_h - base_y * sin_h + center_x
                    ry = base_x * sin_h + base_y * cos_h + center_y
                    _append_shape_point(r, c, rx, ry)
        elif marker_mode == "negotiated" and shape_dyn is not None:
            for r in range(0, self.shape_info.rn, stride):
                for c in range(0, self.shape_info.cn, stride):
                    if self.shape_mtr.value[r][c] >= 1.0:
                        continue
                    sum_x = 0.0
                    sum_y = 0.0
                    for i in range(n):
                        sum_x += shape_dyn.shape_x[i][r][c]
                        sum_y += shape_dyn.shape_y[i][r][c]
                    rx = sum_x / n
                    ry = sum_y / n
                    _append_shape_point(r, c, rx, ry)
        else:
            center_x = sum(self.shape_state.pos_x) / n
            center_y = sum(self.shape_state.pos_y) / n
            head = self._avg_head(self.shape_state.head)
            cos_h = math.cos(head)
            sin_h = math.sin(head)
            for r in range(0, self.shape_info.rn, stride):
                for c in range(0, self.shape_info.cn, stride):
                    if self.shape_mtr.value[r][c] >= 1.0:
                        continue
                    base_x = self.shape_mtr.base_x[r][c]
                    base_y = self.shape_mtr.base_y[r][c]
                    rx = base_x * cos_h - base_y * sin_h + center_x
                    ry = base_x * sin_h + base_y * cos_h + center_y
                    _append_shape_point(r, c, rx, ry)

        if use_gradient:
            markers.markers.append(gradient_marker)
            markers.markers.append(_mk_delete("shape", 0))
            markers.markers.append(_mk_delete("shape", 1))
        else:
            markers.markers.append(black_marker)
            markers.markers.append(gray_marker)
            markers.markers.append(_mk_delete("shape", 4))

        if self.show_enter_targets and enter_goals and len(enter_goals) >= self.sim_param.swarm_size:
            goal_point_marker = Marker()
            goal_point_marker.header.frame_id = self.marker_frame
            goal_point_marker.header.stamp = now
            goal_point_marker.ns = "shape"
            goal_point_marker.id = 2
            goal_point_marker.type = Marker.POINTS
            goal_point_marker.action = Marker.ADD
            goal_point_marker.scale.x = self.enter_target_size
            goal_point_marker.scale.y = self.enter_target_size
            goal_point_marker.color = _mk_color([1.0, 1.0, 1.0], self.enter_target_alpha)

            goal_line_marker = Marker()
            goal_line_marker.header.frame_id = self.marker_frame
            goal_line_marker.header.stamp = now
            goal_line_marker.ns = "shape"
            goal_line_marker.id = 3
            goal_line_marker.type = Marker.LINE_LIST
            goal_line_marker.action = Marker.ADD
            goal_line_marker.scale.x = self.enter_target_line_width
            goal_line_marker.color = _mk_color([1.0, 1.0, 0.1], self.enter_target_alpha)

            for i in range(self.sim_param.swarm_size):
                goal_x, goal_y, goal_gray = enter_goals[i]
                goal_point_marker.points.append(Point(x=goal_x, y=goal_y, z=0.08))

                if goal_gray >= 0.999:
                    point_color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=self.enter_target_alpha)
                elif goal_gray <= self.shape_black_threshold:
                    point_color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=self.enter_target_alpha)
                else:
                    point_color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=self.enter_target_alpha)
                goal_point_marker.colors.append(point_color)

                goal_line_marker.points.append(Point(x=robot_state.pos_x[i], y=robot_state.pos_y[i], z=0.06))
                goal_line_marker.points.append(Point(x=goal_x, y=goal_y, z=0.06))

            if goal_line_marker.points:
                markers.markers.append(goal_line_marker)
            if goal_point_marker.points:
                markers.markers.append(goal_point_marker)
        else:
            markers.markers.append(_mk_delete("shape", 2))
            markers.markers.append(_mk_delete("shape", 3))

        if self.publish_links and neigh is not None:
            link_marker = Marker()
            link_marker.header.frame_id = self.marker_frame
            link_marker.header.stamp = now
            link_marker.ns = "links"
            link_marker.id = 0
            link_marker.type = Marker.LINE_LIST
            link_marker.action = Marker.ADD
            link_marker.scale.x = self.link_width
            link_marker.color = _mk_color(self.link_color, self.link_alpha)

            a_mtr = neigh.get("a_mtr", [])
            n = self.sim_param.swarm_size
            pair_count = 0
            for i in range(n):
                for j in range(i + 1, n):
                    if not a_mtr or a_mtr[i][j] == 0:
                        continue
                    link_marker.points.append(Point(x=robot_state.pos_x[i], y=robot_state.pos_y[i], z=0.05))
                    link_marker.points.append(Point(x=robot_state.pos_x[j], y=robot_state.pos_y[j], z=0.05))
                    pair_count += 1
                    if self.link_max_pairs > 0 and pair_count >= self.link_max_pairs:
                        break
                if self.link_max_pairs > 0 and pair_count >= self.link_max_pairs:
                    break

            if link_marker.points:
                markers.markers.append(link_marker)
        else:
            markers.markers.append(_mk_delete("links", 0))

        comm_radius = self.comm_range_radius if self.comm_range_radius > 0.0 else self.sim_param.r_sense
        if self.show_comm_range and comm_radius > 1e-6:
            comm_marker = Marker()
            comm_marker.header.frame_id = self.marker_frame
            comm_marker.header.stamp = now
            comm_marker.ns = "comm_range"
            comm_marker.id = 0
            comm_marker.type = Marker.LINE_LIST
            comm_marker.action = Marker.ADD
            comm_marker.scale.x = max(1e-4, self.comm_range_line_width)
            comm_marker.color = _mk_color(self.comm_range_color, self.comm_range_alpha)

            seg_n = max(12, int(self.comm_range_segments))
            for i in range(self.sim_param.swarm_size):
                cx = robot_state.pos_x[i]
                cy = robot_state.pos_y[i]
                for seg in range(seg_n):
                    th0 = 2.0 * math.pi * float(seg) / float(seg_n)
                    th1 = 2.0 * math.pi * float(seg + 1) / float(seg_n)
                    comm_marker.points.append(
                        Point(x=cx + comm_radius * math.cos(th0), y=cy + comm_radius * math.sin(th0), z=0.03)
                    )
                    comm_marker.points.append(
                        Point(x=cx + comm_radius * math.cos(th1), y=cy + comm_radius * math.sin(th1), z=0.03)
                    )
            markers.markers.append(comm_marker)
        else:
            markers.markers.append(_mk_delete("comm_range", 0))

        occupancy_radius = self.occupancy_radius if self.occupancy_radius > 0.0 else self.sim_param.r_body
        occupancy_height = max(0.001, self.occupancy_height)
        if self.show_occupancy and occupancy_radius > 1e-6:
            for i in range(self.sim_param.swarm_size):
                occ_marker = Marker()
                occ_marker.header.frame_id = self.marker_frame
                occ_marker.header.stamp = now
                occ_marker.ns = "occupancy"
                occ_marker.id = i
                occ_marker.type = Marker.CYLINDER
                occ_marker.action = Marker.ADD
                occ_marker.pose.position.x = robot_state.pos_x[i]
                occ_marker.pose.position.y = robot_state.pos_y[i]
                occ_marker.pose.position.z = occupancy_height * 0.5
                occ_marker.pose.orientation.w = 1.0
                occ_marker.scale.x = 2.0 * occupancy_radius
                occ_marker.scale.y = 2.0 * occupancy_radius
                occ_marker.scale.z = occupancy_height
                occ_marker.color = _mk_color(self.occupancy_color, self.occupancy_alpha)
                markers.markers.append(occ_marker)
        else:
            for i in range(self.sim_param.swarm_size):
                markers.markers.append(_mk_delete("occupancy", i))

        # Velocity arrows and speed text
        vel_max = max(self.sim_param.vel_max, 1e-6)
        for i in range(self.sim_param.swarm_size):
            if self.velocity_source == "cmd":
                vx = cmd_x[i]
                vy = cmd_y[i]
            else:
                vx = robot_state.vel_x[i]
                vy = robot_state.vel_y[i]
            speed = math.hypot(vx, vy)
            dx = vx * self.velocity_scale
            dy = vy * self.velocity_scale
            if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                dx = 0.001
                dy = 0.0

            arrow = Marker()
            arrow.header.frame_id = self.marker_frame
            arrow.header.stamp = now
            arrow.ns = "velocity"
            arrow.id = i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = self.arrow_shaft_diameter
            arrow.scale.y = self.arrow_head_diameter
            arrow.scale.z = self.arrow_head_length
            ratio = min(speed / vel_max, 1.0)
            arrow.color.r = ratio
            arrow.color.g = 0.2
            arrow.color.b = 1.0 - ratio
            arrow.color.a = 0.9

            start = Point(x=robot_state.pos_x[i], y=robot_state.pos_y[i], z=0.05)
            end = Point(x=robot_state.pos_x[i] + dx, y=robot_state.pos_y[i] + dy, z=0.05)
            arrow.points = [start, end]
            markers.markers.append(arrow)

            if self.show_speed_text:
                text_marker = Marker()
                text_marker.header.frame_id = self.marker_frame
                text_marker.header.stamp = now
                text_marker.ns = "speed"
                text_marker.id = i
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.scale.z = self.speed_text_size
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 0.85
                text_marker.pose.position.x = robot_state.pos_x[i]
                text_marker.pose.position.y = robot_state.pos_y[i]
                text_marker.pose.position.z = self.speed_text_z
                text_marker.text = f"{speed:.2f}"
                markers.markers.append(text_marker)
            else:
                markers.markers.append(_mk_delete("speed", i))

        if self.show_metric_text and metric is not None:
            metric_marker = Marker()
            metric_marker.header.frame_id = self.marker_frame
            metric_marker.header.stamp = now
            metric_marker.ns = "metric"
            metric_marker.id = 0
            metric_marker.type = Marker.TEXT_VIEW_FACING
            metric_marker.action = Marker.ADD
            metric_marker.scale.z = self.metric_text_size
            metric_marker.color = _mk_color(self.metric_text_color, self.metric_text_alpha)

            if self.refer_state is not None:
                anchor_x = self.refer_state.pos_x
                anchor_y = self.refer_state.pos_y
            else:
                n = max(1, self.sim_param.swarm_size)
                anchor_x = sum(robot_state.pos_x) / n
                anchor_y = sum(robot_state.pos_y) / n
            metric_marker.pose.position.x = anchor_x
            metric_marker.pose.position.y = anchor_y
            metric_marker.pose.position.z = self.metric_text_z
            metric_marker.pose.orientation.w = 1.0
            metric_marker.text = (
                "cover {:.2f} inside {:.2f} enter {:.2f} dist_var {:.2f} "
                "align {:.2f} neigh {:.2f} sat {:.2f} c_err {:.2f} "
                "min_pair {:.2f} coll {}".format(
                    metric.cover_rate,
                    metric.inside_rate,
                    metric.enter_rate,
                    metric.dist_var,
                    metric.vel_align,
                    metric.neigh_mean,
                    metric.cmd_sat_rate,
                    metric.shape_center_err,
                    metric.min_pair_dist,
                    metric.collision_pairs,
                )
            )
            markers.markers.append(metric_marker)
        else:
            markers.markers.append(_mk_delete("metric", 0))

        self.marker_pub.publish(markers)

    def _on_timer(self, _event) -> None:
        if not self.enabled:
            self._publish_zero()
            return

        if not self.ns_list:
            if self.robot_namespaces:
                ns_list = list(self.robot_namespaces)
            elif self.auto_detect:
                ns_list = self._detect_namespaces()
            elif self.agent_number > 0:
                ns_list = [f"{self.namespace_prefix}{i}" for i in range(1, self.agent_number + 1)]
            else:
                ns_list = []
            if ns_list:
                self._setup_io(ns_list)
            self._maybe_report_monitored_robots()
            return

        self._maybe_report_monitored_robots()

        self._maybe_init()
        if self.pending_shape_reload and self.sim_param.swarm_size > 0:
            self._reload_shape_model()
        if self.shape_state is None or self.shape_mtr is None or self.shape_info is None or self.refer_state is None:
            return

        robot_state = self._get_robot_state()
        if robot_state is None:
            return

        neigh = get_neighbor_set(self.sim_param, robot_state)
        self.shape_state = negotiate_position(self.sim_param, neigh, self.shape_state, self.refer_state, self.inform_index)
        self.shape_state = negotiate_orientation(self.sim_param, neigh, self.shape_state, self.refer_state, self.inform_index)
        self._optimize_reference_heading_by_obstacle()
        target_shape_state = self.shape_state
        if self.shape_target_mode == ShapeTargetMode.REFERENCE:
            target_shape_state = self._build_reference_target_state(self.shape_state)
        shape_dyn = get_dyn_formation(self.sim_param, self.shape_mtr, self.shape_info, target_shape_state)

        cmd_enter_x, cmd_enter_y, grid_set, enter_goals = cal_entering_cmd(
            self.sim_param,
            shape_dyn,
            self.shape_info,
            robot_state,
            target_shape_state,
        )
        cmd_explore_x, cmd_explore_y = cal_exploration_cmd(
            self.sim_param,
            shape_dyn,
            self.shape_info,
            grid_set,
            neigh,
            robot_state,
            target_shape_state,
        )
        cmd_interact_x, cmd_interact_y = cal_interaction_cmd(self.sim_param, neigh, robot_state)
        cmd_obs_x, cmd_obs_y, local_obs_nearest = self._cal_local_obstacle_cmd(robot_state)

        cmd_x = [
            cmd_enter_x[i] + cmd_explore_x[i] + cmd_interact_x[i] + cmd_obs_x[i]
            for i in range(self.sim_param.swarm_size)
        ]
        cmd_y = [
            cmd_enter_y[i] + cmd_explore_y[i] + cmd_interact_y[i] + cmd_obs_y[i]
            for i in range(self.sim_param.swarm_size)
        ]

        cmd_x, cmd_y = enforce_safety_barrier(self.sim_param, neigh, cmd_x, cmd_y)
        cmd_x, cmd_y = limit_speed(cmd_x, cmd_y, self.sim_param.vel_max)

        if self.sim_param.cmd_smooth_ratio > 0.0:
            ratio = self.sim_param.cmd_smooth_ratio
            if self.cmd_smooth_use_odom:
                cmd_x = [ratio * cmd_x[i] + (1.0 - ratio) * robot_state.vel_x[i] for i in range(self.sim_param.swarm_size)]
                cmd_y = [ratio * cmd_y[i] + (1.0 - ratio) * robot_state.vel_y[i] for i in range(self.sim_param.swarm_size)]
            else:
                cmd_x = [ratio * cmd_x[i] + (1.0 - ratio) * self.prev_cmd_x[i] for i in range(self.sim_param.swarm_size)]
                cmd_y = [ratio * cmd_y[i] + (1.0 - ratio) * self.prev_cmd_y[i] for i in range(self.sim_param.swarm_size)]

        if self.sim_param.cmd_scale != 1.0:
            cmd_x = [self.sim_param.cmd_scale * v for v in cmd_x]
            cmd_y = [self.sim_param.cmd_scale * v for v in cmd_y]

        cmd_x, cmd_y = enforce_safety_barrier(self.sim_param, neigh, cmd_x, cmd_y)
        cmd_x, cmd_y = limit_speed(cmd_x, cmd_y, self.sim_param.vel_max)

        self.prev_cmd_x = list(cmd_x)
        self.prev_cmd_y = list(cmd_y)
        control_active = self._update_control_switch(robot_state)
        self._cancel_move_base_for_active(control_active)
        self._update_takeover_state_param(control_active)

        pub_cmd_x = [0.0] * self.sim_param.swarm_size
        pub_cmd_y = [0.0] * self.sim_param.swarm_size
        for i in range(self.sim_param.swarm_size):
            if self.cmd_in_map_frame:
                yaw = robot_state.yaw[i]
                pub_cmd_x[i] = math.cos(yaw) * cmd_x[i] + math.sin(yaw) * cmd_y[i]
                pub_cmd_y[i] = -math.sin(yaw) * cmd_x[i] + math.cos(yaw) * cmd_y[i]
            else:
                pub_cmd_x[i] = cmd_x[i]
                pub_cmd_y[i] = cmd_y[i]

        for i, pub in enumerate(self.cmd_pubs):
            if pub is None:
                continue
            if i < len(control_active) and not control_active[i]:
                # Keep move_base in charge before switch condition is reached.
                continue
            twist = Twist()
            hold_active = i < len(self.shape_ctrl_hold_until) and time.time() < self.shape_ctrl_hold_until[i]
            if not hold_active:
                twist.linear.x = pub_cmd_x[i]
                twist.linear.y = pub_cmd_y[i]
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            pub.publish(twist)

        metric = compute_swarm_metric(
            self.sim_param,
            self.shape_mtr,
            self.shape_info,
            robot_state,
            neigh,
            self.shape_state,
            self.refer_state,
            cmd_x,
            cmd_y,
        )
        self._update_converged_status(metric)
        self._publish_local_status(robot_state, control_active)

        self._maybe_log_debug(
            robot_state,
            neigh,
            metric,
            cmd_x,
            cmd_y,
            pub_cmd_x,
            pub_cmd_y,
            cmd_enter_x,
            cmd_enter_y,
            cmd_explore_x,
            cmd_explore_y,
            cmd_interact_x,
            cmd_interact_y,
            cmd_obs_x,
            cmd_obs_y,
            enter_goals,
            control_active,
            self.switch_gray_values,
            local_obs_nearest,
        )
        self._publish_markers(robot_state, cmd_x, cmd_y, neigh, shape_dyn, enter_goals, metric)


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        parser = build_self_test_arg_parser()
        args = parser.parse_args()
        raise SystemExit(run_offline_self_test(args))

    rospy.init_node("shape_assembly_swarm")
    ShapeAssemblySwarm()
    rospy.spin()
