#!/usr/bin/env python3
import math
import os
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rospy
from formation_msgs.msg import ShapeTask
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid


def _angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _limit_angle(angle: float) -> float:
    if angle > math.pi:
        angle -= 2.0 * math.pi
    if angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


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
    gray_num: int = 0
    gray_scale: float = 1.0
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
    gray_num = 0
    for r in range(info.rn):
        for c in range(info.cn):
            v = image_mtr[r][c]
            if v == 0:
                black_num += 1
            if v < 1.0:
                gray_num += 1
    info.black_num = max(1, black_num)
    info.gray_num = max(0, gray_num - black_num)

    temp_row = image_mtr[max(0, info.cen_y - 1)] if info.rn > 0 else []
    temp = [2.0 if v == 0 else v for v in temp_row]
    info.gray_scale = 1.0 / min(temp) if temp else 1.0
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


class ShapeTaskSupervisor:
    def __init__(self) -> None:
        self.task_topic = rospy.get_param("~task_topic", "/shape_assembly/task")
        self.gather_center_topic = rospy.get_param("~gather_center_topic", "/gather_center")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.agent_count = int(rospy.get_param("~agent_count", 0))
        self.source = rospy.get_param("~source", "fm2_gather")
        self.center_min_delta = max(0.0, float(rospy.get_param("~center_min_delta", 0.05)))
        self.heading_min_delta = max(0.0, float(rospy.get_param("~heading_min_delta", math.radians(5.0))))
        self.config_poll_hz = max(0.1, float(rospy.get_param("~config_poll_hz", 1.0)))

        self.auto_shape_heading = bool(rospy.get_param("~auto_shape_heading", True))
        self.auto_shape_heading_map_topic = str(rospy.get_param("~auto_shape_heading_map_topic", "/map"))
        self.auto_shape_heading_angle_step_deg = max(1.0, float(rospy.get_param("~auto_shape_heading_angle_step_deg", 15.0)))
        self.auto_shape_heading_obstacle_threshold = int(rospy.get_param("~auto_shape_heading_obstacle_threshold", 80))
        self.auto_shape_heading_unknown_is_obstacle = bool(rospy.get_param("~auto_shape_heading_unknown_is_obstacle", True))
        self.auto_shape_heading_shape_stride = max(1, int(rospy.get_param("~auto_shape_heading_shape_stride", 2)))
        self.auto_shape_heading_min_improve = max(0.0, float(rospy.get_param("~auto_shape_heading_min_improve", 0.01)))
        self.auto_shape_heading_yaw_bias = max(0.0, float(rospy.get_param("~auto_shape_heading_yaw_bias", 0.02)))
        self.auto_shape_heading_oob_is_obstacle = bool(rospy.get_param("~auto_shape_heading_oob_is_obstacle", True))

        self.shape_source = rospy.get_param("~shape_source", "mat")
        self.shape_resolution = int(rospy.get_param("~shape_resolution", 80))
        self.ring_inner_ratio = float(rospy.get_param("~ring_inner_ratio", 0.25))
        self.ring_outer_ratio = float(rospy.get_param("~ring_outer_ratio", 0.4))
        self.gray_width = int(rospy.get_param("~gray_width", 4))
        self.shape_mat_path = rospy.get_param("~shape_mat_path", "")
        self.shape_library_root = os.path.expanduser(str(rospy.get_param("~shape_library_root", "")).strip() or _get_default_shape_dir())
        self.r_avoid = float(rospy.get_param("~r_avoid", 0.35))
        self.shape_black_threshold = float(rospy.get_param("~shape_black_threshold", 1e-6))
        if str(self.shape_source).strip().lower() == "mat" and not os.path.isdir(self.shape_library_root):
            rospy.logwarn("ShapeTaskSupervisor: local shape_library_root=%s missing. Host must store local shape_images or override ~shape_library_root.", self.shape_library_root)

        self._task_id = int(rospy.get_param("~initial_task_id", 0))
        self._last_center: Optional[Tuple[float, float, str]] = None
        self._last_shape_type: Optional[str] = None
        self._last_shape_heading: Optional[float] = None
        self._last_shape_scale: Optional[float] = None
        self._last_staging_radius: Optional[float] = None
        self._last_task: Optional[ShapeTask] = None
        self._map_msg: Optional[OccupancyGrid] = None
        self._shape_cache_key: Optional[Tuple[object, ...]] = None
        self._shape_info: Optional[_ShapeInfo] = None
        self._shape_mtr: Optional[_ShapeMatrix] = None
        self._shape_overlap_samples: List[Tuple[float, float, float]] = []

        self.task_pub = rospy.Publisher(self.task_topic, ShapeTask, queue_size=2, latch=True)
        self.center_sub = rospy.Subscriber(self.gather_center_topic, PoseStamped, self._center_cb, queue_size=2)
        self.map_sub = None
        if self.auto_shape_heading and self.auto_shape_heading_map_topic:
            self.map_sub = rospy.Subscriber(self.auto_shape_heading_map_topic, OccupancyGrid, self._map_cb, queue_size=1)
        self.config_timer = rospy.Timer(rospy.Duration(1.0 / self.config_poll_hz), self._config_timer_cb)

        rospy.loginfo(
            "ShapeTaskSupervisor: task_topic=%s gather_center_topic=%s auto_heading=%s map_topic=%s agent_count=%d",
            self.task_topic,
            self.gather_center_topic,
            str(self.auto_shape_heading),
            self.auto_shape_heading_map_topic,
            self.agent_count,
        )

    def _read_shape_config(self) -> Tuple[str, float, float]:
        shape_type = str(rospy.get_param("~shape_type", "rectangle")).strip() or "rectangle"
        shape_heading = float(rospy.get_param("~shape_heading", 0.0))
        shape_scale = max(0.0, float(rospy.get_param("~shape_scale", 1.0)))
        return shape_type, shape_heading, shape_scale

    def _resolve_staging_radius(self) -> float:
        explicit_radius = float(rospy.get_param("~staging_radius", 0.0))
        if explicit_radius > 0.0:
            return explicit_radius
        return 0.0

    def _has_meaningful_center_change(self, center_key: Tuple[float, float, str]) -> bool:
        if self._last_center is None:
            return True
        dx = center_key[0] - self._last_center[0]
        dy = center_key[1] - self._last_center[1]
        frame_changed = center_key[2] != self._last_center[2]
        return frame_changed or math.hypot(dx, dy) >= self.center_min_delta

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map_msg = msg

    def _ensure_shape_model(self, shape_type: str) -> bool:
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
        )
        if cache_key == self._shape_cache_key and self._shape_info is not None and self._shape_mtr is not None:
            return True
        try:
            image_mtr, shape_desc = _load_shape_matrix(
                shape_source=self.shape_source,
                shape_type=shape_type,
                shape_resolution=self.shape_resolution,
                ring_inner_ratio=self.ring_inner_ratio,
                ring_outer_ratio=self.ring_outer_ratio,
                gray_width=self.gray_width,
                shape_mat_path=self.shape_mat_path,
                shape_library_root=self.shape_library_root,
            )
            self._shape_mtr, self._shape_info = _init_form_shape(self.agent_count, self.r_avoid, image_mtr)
            self._shape_cache_key = cache_key
            self._shape_overlap_samples = []
            stride = max(1, self.auto_shape_heading_shape_stride)
            for r in range(0, self._shape_info.rn, stride):
                for c in range(0, self._shape_info.cn, stride):
                    gray = float(self._shape_mtr.value[r][c])
                    if gray >= 1.0:
                        continue
                    weight = max(0.05, 1.0 - gray)
                    if gray <= self.shape_black_threshold:
                        weight += 1.0
                    self._shape_overlap_samples.append(
                        (
                            float(self._shape_mtr.base_x[r][c]),
                            float(self._shape_mtr.base_y[r][c]),
                            weight,
                        )
                    )
            rospy.loginfo(
                "ShapeTaskSupervisor: loaded shape model %s samples=%d",
                shape_desc,
                len(self._shape_overlap_samples),
            )
            return bool(self._shape_overlap_samples)
        except Exception as exc:
            rospy.logwarn("ShapeTaskSupervisor: failed to prepare shape model for auto heading (%s)", str(exc))
            return False

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

    def _score_heading_overlap(self, center_x: float, center_y: float, head: float, occ_msg: OccupancyGrid) -> float:
        if not self._shape_overlap_samples:
            return float("inf")
        cos_h = math.cos(head)
        sin_h = math.sin(head)
        sum_w = 0.0
        hit_w = 0.0
        for base_x, base_y, weight in self._shape_overlap_samples:
            wx = base_x * cos_h - base_y * sin_h + center_x
            wy = base_x * sin_h + base_y * cos_h + center_y
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

    def _compute_auto_shape_heading(
        self,
        center_msg: PoseStamped,
        shape_type: str,
        current_heading: float,
    ) -> float:
        if not self.auto_shape_heading:
            return current_heading
        occ_msg = self._map_msg
        if occ_msg is None:
            return current_heading
        center_frame = center_msg.header.frame_id or self.frame_id
        occ_frame = occ_msg.header.frame_id or self.frame_id
        if center_frame and occ_frame and center_frame != occ_frame:
            rospy.logwarn_throttle(
                5.0,
                "ShapeTaskSupervisor: center frame %s != map frame %s, skip auto heading.",
                center_frame,
                occ_frame,
            )
            return current_heading
        if not self._ensure_shape_model(shape_type):
            return current_heading

        center_x = float(center_msg.pose.position.x)
        center_y = float(center_msg.pose.position.y)
        current_head = _limit_angle(float(current_heading))
        current_overlap = self._score_heading_overlap(center_x, center_y, current_head, occ_msg)
        if not math.isfinite(current_overlap):
            return current_heading

        step_rad = math.radians(self.auto_shape_heading_angle_step_deg)
        if step_rad <= 1e-6:
            return current_heading
        best_head = current_head
        best_overlap = current_overlap
        samples = max(8, int(math.ceil((2.0 * math.pi) / step_rad)))
        for k in range(samples):
            cand_head = -math.pi + (2.0 * math.pi * float(k) / float(samples))
            cand_overlap = self._score_heading_overlap(center_x, center_y, cand_head, occ_msg)
            if not math.isfinite(cand_overlap):
                continue
            cand_bias = self.auto_shape_heading_yaw_bias * (abs(_limit_angle(cand_head - current_head)) / math.pi)
            best_bias = self.auto_shape_heading_yaw_bias * (abs(_limit_angle(best_head - current_head)) / math.pi)
            if (cand_overlap + cand_bias) < (best_overlap + best_bias):
                best_head = cand_head
                best_overlap = cand_overlap

        if (current_overlap - best_overlap) < self.auto_shape_heading_min_improve:
            return current_heading
        rospy.loginfo(
            "ShapeTaskSupervisor: auto heading %.1f -> %.1f deg overlap %.3f -> %.3f",
            math.degrees(current_head),
            math.degrees(best_head),
            current_overlap,
            best_overlap,
        )
        return best_head

    def _publish_task(self, center_msg: PoseStamped, replan: bool) -> None:
        shape_type, manual_heading, shape_scale = self._read_shape_config()
        shape_heading_seed = manual_heading
        if self._last_shape_type == shape_type and self._last_shape_heading is not None:
            shape_heading_seed = self._last_shape_heading
        shape_heading = self._compute_auto_shape_heading(center_msg, shape_type, shape_heading_seed)
        staging_radius = self._resolve_staging_radius()
        frame_id = center_msg.header.frame_id or self.frame_id
        center_key = (
            float(center_msg.pose.position.x),
            float(center_msg.pose.position.y),
            frame_id,
        )
        changed = (
            self._has_meaningful_center_change(center_key)
            or self._last_shape_type != shape_type
            or self._last_shape_scale is None
            or abs(self._last_shape_scale - shape_scale) > 1e-6
            or self._last_staging_radius is None
            or abs(self._last_staging_radius - staging_radius) > 1e-6
            or self._last_shape_heading is None
            or abs(_angle_diff(self._last_shape_heading, shape_heading)) >= self.heading_min_delta
        )
        if not changed and self._last_task is not None:
            return

        self._task_id += 1
        task = ShapeTask()
        task.header.stamp = rospy.Time.now()
        task.header.frame_id = frame_id
        task.task_id = self._task_id
        task.agent_count = max(0, self.agent_count)
        task.center = center_msg.pose
        task.shape_type = shape_type
        task.shape_heading = shape_heading
        task.shape_scale = shape_scale
        task.staging_radius = staging_radius
        task.replan = replan or (self._last_task is not None)
        task.source = self.source
        self.task_pub.publish(task)

        self._last_task = task
        self._last_center = center_key
        self._last_shape_type = shape_type
        self._last_shape_heading = shape_heading
        self._last_shape_scale = shape_scale
        self._last_staging_radius = staging_radius
        rospy.loginfo(
            "ShapeTaskSupervisor: publish task=%d center=(%.2f, %.2f) shape=%s heading=%.1fdeg staging=%.2f replan=%s",
            task.task_id,
            center_key[0],
            center_key[1],
            shape_type,
            math.degrees(shape_heading),
            staging_radius,
            str(task.replan),
        )

    def _center_cb(self, msg: PoseStamped) -> None:
        self._publish_task(msg, replan=self._last_task is not None)

    def _config_timer_cb(self, _event) -> None:
        if self._last_task is None:
            return
        center_msg = PoseStamped()
        center_msg.header = self._last_task.header
        center_msg.pose = self._last_task.center
        self._publish_task(center_msg, replan=True)


if __name__ == "__main__":
    rospy.init_node("shape_task_supervisor")
    ShapeTaskSupervisor()
    rospy.spin()
