#!/usr/bin/env python3
import math
import os
import re
from dataclasses import dataclass
from typing import List, Optional, Tuple

import actionlib
import rospy
from formation_msgs.msg import ShapeTask
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


def _yaw_to_quat(yaw: float) -> Quaternion:
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    quat = Quaternion()
    quat.x = qx
    quat.y = qy
    quat.z = qz
    quat.w = qw
    return quat


def _extract_robot_id(namespace: str) -> int:
    match = re.search(r"(\d+)$", namespace or "")
    if not match:
        return 1
    return max(1, int(match.group(1)))


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
            "../../ros_motion_planning/src/sim_env/shape_images",
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
        "sphere": "Image_sphere.mat",
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
    except Exception as exc:
        return None, f"scipy.io unavailable: {exc}"
    try:
        data = scipy.io.loadmat(shape_mat_path)
    except Exception as exc:
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


def _generate_ring_shape(resolution: int, inner_ratio: float, outer_ratio: float, gray_width: int) -> List[List[float]]:
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


def _load_shape_matrix(
    shape_source: str,
    shape_type: str,
    shape_resolution: int,
    ring_inner_ratio: float,
    ring_outer_ratio: float,
    gray_width: int,
    shape_mat_path: str,
    shape_library_root: str,
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
        return _generate_ring_shape(shape_resolution, ring_inner_ratio, ring_outer_ratio, gray_width), f"fallback_ring({desc})"
    return _generate_ring_shape(shape_resolution, ring_inner_ratio, ring_outer_ratio, gray_width), "analytic:ring"


@dataclass
class _ShapeInfo:
    rn: int = 0
    cn: int = 0
    cen_x: int = 0
    cen_y: int = 0
    black_num: int = 1
    grid: float = 1.0


@dataclass
class _ShapeMatrix:
    value: List[List[float]]
    base_x: List[List[float]]
    base_y: List[List[float]]


def _init_form_shape(agent_count: int, r_avoid: float, image_mtr: List[List[float]]) -> Tuple[_ShapeMatrix, _ShapeInfo]:
    info = _ShapeInfo()
    info.rn = len(image_mtr)
    info.cn = len(image_mtr[0]) if info.rn > 0 else 0
    info.cen_x = int(math.ceil(info.cn / 2.0))
    info.cen_y = int(math.ceil(info.rn / 2.0))

    black_num = 0
    for r in range(info.rn):
        for c in range(info.cn):
            if image_mtr[r][c] == 0:
                black_num += 1
    info.black_num = max(1, black_num)
    info.grid = math.sqrt((math.pi / 4.0) * (max(1, agent_count) / info.black_num)) * r_avoid

    shape = _ShapeMatrix(
        value=image_mtr,
        base_x=[[0.0 for _ in range(info.cn)] for _ in range(info.rn)],
        base_y=[[0.0 for _ in range(info.cn)] for _ in range(info.rn)],
    )
    for r in range(info.rn):
        y = (info.rn - r - info.cen_y) * info.grid
        for c in range(info.cn):
            x = (c + 1 - info.cen_x) * info.grid
            shape.base_x[r][c] = x
            shape.base_y[r][c] = y
    return shape, info


class ShapeTaskBridge:
    def __init__(self) -> None:
        self.robot_namespace = str(rospy.get_param("~robot_namespace", rospy.get_namespace().strip("/"))).strip("/")
        self.robot_id = int(rospy.get_param("~robot_id", _extract_robot_id(self.robot_namespace)))
        self.agent_count = int(rospy.get_param("~agent_count", 0))
        self.task_topic = rospy.get_param("~task_topic", "/shape_assembly/task")
        self.goal_topic = rospy.get_param("~goal_topic", "shape_assembly/staging_goal")
        self.move_base_action = rospy.get_param("~move_base_action", f"/{self.robot_namespace}/move_base")
        self.goal_frame_id = rospy.get_param("~goal_frame_id", "map")
        self.use_center_as_goal = bool(rospy.get_param("~use_center_as_goal", False))
        self.angle_offset = math.radians(float(rospy.get_param("~staging_angle_offset_deg", 0.0)))
        self.goal_min_delta = max(0.0, float(rospy.get_param("~goal_min_delta", 0.05)))
        self.wait_for_server_timeout = max(0.0, float(rospy.get_param("~wait_for_server_timeout", 3.0)))
        self.face_center = bool(rospy.get_param("~face_center", True))
        self.min_staging_radius = max(0.0, float(rospy.get_param("~min_staging_radius", 0.8)))
        self.staging_margin = max(0.0, float(rospy.get_param("~staging_margin", 0.45)))
        self.radius_gain = max(0.0, float(rospy.get_param("~staging_radius_gain", 1.0)))
        self.shape_source = rospy.get_param("~shape_source", "mat")
        self.shape_resolution = int(rospy.get_param("~shape_resolution", 80))
        self.ring_inner_ratio = float(rospy.get_param("~ring_inner_ratio", 0.25))
        self.ring_outer_ratio = float(rospy.get_param("~ring_outer_ratio", 0.4))
        self.gray_width = int(rospy.get_param("~gray_width", 4))
        self.shape_mat_path = rospy.get_param("~shape_mat_path", "")
        self.shape_library_root = os.path.expanduser(str(rospy.get_param("~shape_library_root", "")).strip() or _get_default_shape_dir())
        self.r_avoid = float(rospy.get_param("~r_avoid", 0.35))
        self.shape_sample_stride = max(1, int(rospy.get_param("~shape_sample_stride", 2)))
        self.debug_log = bool(rospy.get_param("~debug_log", False))

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=2, latch=True)
        self.task_sub = rospy.Subscriber(self.task_topic, ShapeTask, self._task_cb, queue_size=2)
        self.client = actionlib.SimpleActionClient(self.move_base_action, MoveBaseAction)

        self._server_ready = False
        self._last_goal_xy: Optional[Tuple[float, float]] = None
        self._last_task_id = 0
        self._last_goal: Optional[MoveBaseGoal] = None
        self._last_goal_sent = False
        self._shape_cache_key: Optional[Tuple[object, ...]] = None
        self._shape_samples: List[Tuple[float, float]] = []
        self.retry_timer = rospy.Timer(rospy.Duration(1.0), self._retry_timer_cb)

        if str(self.shape_source).strip().lower() == "mat" and not os.path.isdir(self.shape_library_root):
            rospy.logwarn("ShapeTaskBridge[%s]: local shape_library_root=%s missing. Each robot must store local shape_images or override ~shape_library_root.", self.robot_namespace or "(no-ns)", self.shape_library_root)

        rospy.loginfo(
            "ShapeTaskBridge[%s]: task_topic=%s move_base_action=%s robot_id=%d use_center_as_goal=%s",
            self.robot_namespace or "(no-ns)",
            self.task_topic,
            self.move_base_action,
            self.robot_id,
            str(self.use_center_as_goal),
        )

    def _ensure_server(self) -> bool:
        if self._server_ready:
            return True
        if self.client.wait_for_server(rospy.Duration(self.wait_for_server_timeout)):
            self._server_ready = True
            return True
        rospy.logwarn_throttle(
            5.0,
            "ShapeTaskBridge[%s]: waiting move_base action server %s",
            self.robot_namespace or "(no-ns)",
            self.move_base_action,
        )
        return False

    def _ensure_shape_samples(self, shape_type: str) -> bool:
        cache_key = (
            self.shape_source,
            _normalize_shape_type(shape_type),
            self.shape_resolution,
            self.ring_inner_ratio,
            self.ring_outer_ratio,
            self.gray_width,
            self.shape_mat_path,
            self.shape_library_root,
            self.agent_count,
            self.r_avoid,
            self.shape_sample_stride,
        )
        if cache_key == self._shape_cache_key and self._shape_samples:
            return True
        try:
            image_mtr, _ = _load_shape_matrix(
                shape_source=self.shape_source,
                shape_type=shape_type,
                shape_resolution=self.shape_resolution,
                ring_inner_ratio=self.ring_inner_ratio,
                ring_outer_ratio=self.ring_outer_ratio,
                gray_width=self.gray_width,
                shape_mat_path=self.shape_mat_path,
                shape_library_root=self.shape_library_root,
            )
            shape_mtr, shape_info = _init_form_shape(self.agent_count, self.r_avoid, image_mtr)
            samples: List[Tuple[float, float]] = []
            stride = max(1, self.shape_sample_stride)
            for r in range(0, shape_info.rn, stride):
                for c in range(0, shape_info.cn, stride):
                    if float(shape_mtr.value[r][c]) >= 1.0:
                        continue
                    samples.append((float(shape_mtr.base_x[r][c]), float(shape_mtr.base_y[r][c])))
            if not samples:
                return False
            self._shape_cache_key = cache_key
            self._shape_samples = samples
            return True
        except Exception as exc:
            rospy.logwarn("ShapeTaskBridge[%s]: failed to prepare shape samples (%s)", self.robot_namespace or "(no-ns)", str(exc))
            return False

    def _directional_shape_extent(self, shape_type: str, shape_heading: float, shape_scale: float, angle: float) -> float:
        if not self._ensure_shape_samples(shape_type):
            return 0.0
        scale = max(1.0e-3, float(shape_scale))
        dir_x = math.cos(angle)
        dir_y = math.sin(angle)
        cos_h = math.cos(shape_heading)
        sin_h = math.sin(shape_heading)
        max_proj = 0.0
        for base_x, base_y in self._shape_samples:
            wx = scale * (base_x * cos_h - base_y * sin_h)
            wy = scale * (base_x * sin_h + base_y * cos_h)
            proj = wx * dir_x + wy * dir_y
            if proj > max_proj:
                max_proj = proj
        return max_proj

    def _compute_goal(self, task: ShapeTask) -> MoveBaseGoal:
        if self.use_center_as_goal:
            goal = MoveBaseGoal()
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.header.frame_id = task.header.frame_id or self.goal_frame_id
            goal.target_pose.pose.position.x = float(task.center.position.x)
            goal.target_pose.pose.position.y = float(task.center.position.y)
            goal.target_pose.pose.position.z = 0.0
            goal.target_pose.pose.orientation = _yaw_to_quat(float(task.shape_heading))
            return goal

        slot_count = int(task.agent_count) if int(task.agent_count) > 0 else max(1, self.agent_count)
        if slot_count <= 0:
            slot_count = max(1, self.robot_id)
        slot_index = (self.robot_id - 1) % slot_count
        angle = float(task.shape_heading) + self.angle_offset + (2.0 * math.pi * float(slot_index) / float(slot_count))
        extent = self._directional_shape_extent(task.shape_type, float(task.shape_heading), float(task.shape_scale), angle)
        radius = max(
            self.min_staging_radius,
            extent + self.staging_margin,
            float(task.staging_radius) * self.radius_gain,
        )
        goal_x = float(task.center.position.x) + radius * math.cos(angle)
        goal_y = float(task.center.position.y) + radius * math.sin(angle)
        goal_yaw = angle + math.pi if self.face_center else float(task.shape_heading)

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = task.header.frame_id or self.goal_frame_id
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation = _yaw_to_quat(goal_yaw)
        return goal

    def _task_cb(self, task: ShapeTask) -> None:
        goal = self._compute_goal(task)
        goal_xy = (goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
        if (
            self._last_goal_xy is not None
            and self._last_task_id == int(task.task_id)
            and math.hypot(goal_xy[0] - self._last_goal_xy[0], goal_xy[1] - self._last_goal_xy[1]) < self.goal_min_delta
        ):
            return

        self._last_task_id = int(task.task_id)
        self._last_goal_xy = goal_xy
        self._last_goal = goal
        self._last_goal_sent = False

        preview = PoseStamped()
        preview.header = goal.target_pose.header
        preview.pose = goal.target_pose.pose
        self.goal_pub.publish(preview)

        if not self._ensure_server():
            return
        self.client.send_goal(goal)
        self._last_goal_sent = True
        if self.debug_log:
            rospy.loginfo(
                "ShapeTaskBridge[%s]: task=%d shape=%s goal=(%.2f, %.2f) heading=%.1fdeg",
                self.robot_namespace or "(no-ns)",
                task.task_id,
                task.shape_type,
                goal_xy[0],
                goal_xy[1],
                math.degrees(task.shape_heading),
            )

    def _retry_timer_cb(self, _event) -> None:
        if self._last_goal is None or self._last_goal_sent:
            return
        if not self._ensure_server():
            return
        self.client.send_goal(self._last_goal)
        self._last_goal_sent = True
        if self.debug_log:
            rospy.loginfo(
                "ShapeTaskBridge[%s]: resent pending goal for task=%d",
                self.robot_namespace or "(no-ns)",
                self._last_task_id,
            )


if __name__ == "__main__":
    rospy.init_node("shape_task_bridge")
    ShapeTaskBridge()
    rospy.spin()
