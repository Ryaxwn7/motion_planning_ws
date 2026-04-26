#!/usr/bin/env python3

import argparse
import os
import shlex
import sys
import tempfile
from pathlib import Path
from typing import Any, Dict, Iterable

import yaml


HOST_LAUNCH_KEYS = {
    "agent_number",
    "robot_ids",
    "gather_config",
    "enable_map_combine",
    "shape_type",
    "shape_heading",
    "shape_scale",
    "shape_source",
    "shape_library_root",
    "shape_mat_path",
    "shape_resolution",
    "ring_inner_ratio",
    "ring_outer_ratio",
    "gray_width",
    "r_avoid",
    "shape_black_threshold",
    "publish_target_markers",
    "target_marker_topic",
    "staging_radius",
    "auto_shape_heading",
    "auto_shape_heading_map_topic",
    "auto_shape_heading_angle_step_deg",
    "auto_shape_heading_obstacle_threshold",
    "auto_shape_heading_unknown_is_obstacle",
    "auto_shape_heading_shape_stride",
    "auto_shape_heading_min_improve",
    "auto_shape_heading_yaw_bias",
    "auto_shape_heading_oob_is_obstacle",
    "publish_center_goal_only",
    "robot_namespace_prefix",
    "robot_detect_topic_suffix",
    "robot_odom_topic_suffix",
    "external_center_goal_topic",
    "external_center_sync_wait",
    "replan_mode",
    "periodic_interval",
    "cooldown",
    "per_robot_cooldown",
    "max_len_growth_ratio",
    "long_path_replan_min_length",
    "format_radius",
    "enable_stable_len_replan",
    "use_goal_occupied_peer_fallback",
    "goal_occupied_radius",
    "loop_hz",
}

ROBOT_LAUNCH_KEYS = {
    "map_started",
    "agent_number",
    "agent_id",
    "start_ns",
    "robot_namespace",
    "invert_y_cmd",
    "invert_y_odom",
    "robot_ids",
    "global_planner",
    "fm2_distance_field_backend",
    "local_planner",
    "robot",
    "map_file",
    "enable_shape_assembly",
    "launch_shape_assembly_host",
    "shape_source",
    "shape_type",
    "shape_heading",
    "shape_scale",
    "shape_library_root",
    "shape_resolution",
    "ring_inner_ratio",
    "ring_outer_ratio",
    "gray_width",
    "staging_radius",
    "auto_shape_heading",
    "use_center_as_goal",
    "gather_config",
    "enable_map_combine",
    "publish_target_markers",
    "target_marker_topic",
    "distributed_marker_owner",
    "publish_markers",
    "robot_odom_topic_suffix",
    "shape_assembly_config",
    "my_local_planner_config",
    "global_costmap_config",
    "local_costmap_config",
    "graph_planner_config",
}


def _deep_expand(value: Any, env: Dict[str, str]) -> Any:
    if isinstance(value, dict):
        return {k: _deep_expand(v, env) for k, v in value.items()}
    if isinstance(value, list):
        return [_deep_expand(v, env) for v in value]
    if isinstance(value, str):
        expanded = value
        for key, raw in env.items():
            expanded = expanded.replace(f"${{{key}}}", raw).replace(f"${key}", raw)
        return os.path.expandvars(expanded)
    return value


def _load_yaml(path: Path, ws_root: Path) -> Dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    env = dict(os.environ)
    env["WS_ROOT"] = str(ws_root)
    env["PWD"] = str(ws_root)
    return _deep_expand(data, env)


def _ros_scalar(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return ""
    return str(value)


def _ros_arg_value(value: Any) -> str:
    if isinstance(value, list):
        return "[{}]".format(",".join(_ros_scalar(v) for v in value))
    return _ros_scalar(value)


def _emit_assignment(name: str, value: Any) -> str:
    return f"{name}={shlex.quote(_ros_scalar(value))}"


def _emit_array(name: str, values: Iterable[str]) -> str:
    lines = [f"declare -a {name}=("]
    for value in values:
        lines.append(f"  {shlex.quote(value)}")
    lines.append(")")
    return "\n".join(lines)


def _write_yaml(path: Path, data: Dict[str, Any]) -> None:
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def _materialize_host(config: Dict[str, Any], ws_root: Path) -> str:
    temp_dir = Path(tempfile.mkdtemp(prefix="host_param_", dir="/tmp"))

    script = config.get("script", {})
    ros = config.get("ros", {})
    map_server = config.get("map_server", {})
    shape_real = config.get("shape_assembly_real_robot", {})
    shape_host = dict(config.get("shape_assembly_host", {}))
    formation_host = dict(config.get("formation_host", {}))
    fm2_gather = dict(config.get("fm2_gather", {}))

    robot_ids = shape_host.get("robot_ids")
    agent_number = shape_host.get("agent_number")
    if robot_ids and "robot_ids" not in fm2_gather:
        fm2_gather["robot_ids"] = robot_ids
    if agent_number is not None and "num_robots" not in fm2_gather:
        fm2_gather["num_robots"] = agent_number

    gather_yaml = temp_dir / "fm2_gather.yaml"
    _write_yaml(gather_yaml, fm2_gather)

    launch_args_dict: Dict[str, Any] = {}
    launch_args_dict.update(formation_host)
    launch_args_dict.update(shape_host)
    launch_args_dict["gather_config"] = str(gather_yaml)
    host_args = [
        f"{key}:={_ros_arg_value(value)}"
        for key, value in launch_args_dict.items()
        if key in HOST_LAUNCH_KEYS and value not in ("", None)
    ]

    map_file = map_server.get("map_file", shape_real.get("map_file", ""))

    lines = [
        _emit_assignment("CONFIG_TEMP_DIR", str(temp_dir)),
        _emit_assignment("START_ROSCORE", script.get("start_roscore", False)),
        _emit_assignment("LAUNCH_MAP_SERVER", shape_real.get("launch_map_server", True)),
        _emit_assignment("AUTO_START_GATHER", shape_real.get("auto_start_gather", False)),
        _emit_assignment("ROSCORE_WAIT", script.get("roscore_wait", 8.0)),
        _emit_assignment("ROS_MASTER_URI_VALUE", ros.get("master_uri", "")),
        _emit_assignment("ROS_IP_VALUE", ros.get("ip", "")),
        _emit_assignment("MAP_FILE_VALUE", map_file),
        _emit_array("HOST_LAUNCH_ARGS", host_args),
    ]
    if "start_gather_delay" in shape_real:
        lines.append(_emit_assignment("START_GATHER_DELAY", shape_real["start_gather_delay"]))
    if "start_gather_wait_started" in shape_real:
        lines.append(_emit_assignment("START_GATHER_WAIT_STARTED", shape_real["start_gather_wait_started"]))
    if "start_gather_wait_connections" in shape_real:
        lines.append(_emit_assignment("START_GATHER_WAIT_CONNECTIONS", shape_real["start_gather_wait_connections"]))
    if "start_gather_repeat" in shape_real:
        lines.append(_emit_assignment("START_GATHER_REPEAT", shape_real["start_gather_repeat"]))
    if "start_gather_rate" in shape_real:
        lines.append(_emit_assignment("START_GATHER_RATE", shape_real["start_gather_rate"]))
    return "\n".join(lines) + "\n"


def _materialize_robot(config: Dict[str, Any], ws_root: Path) -> str:
    temp_dir = Path(tempfile.mkdtemp(prefix="robot_param_", dir="/tmp"))

    ros = config.get("ros", {})
    time_sync = config.get("time_sync", {})
    motion = dict(config.get("motion_navigate_multi4", {}))
    graph_planner = dict(config.get("graph_planner", {}))
    my_local_planner = dict(config.get("my_local_planner", {}))
    global_costmap = dict(config.get("global_costmap", {}))
    local_costmap = dict(config.get("local_costmap", {}))
    shape_assembly = dict(config.get("shape_assembly", {}))

    graph_planner_yaml = temp_dir / "graph_planner.yaml"
    my_local_planner_yaml = temp_dir / "my_local_planner.yaml"
    global_costmap_yaml = temp_dir / "global_costmap.yaml"
    local_costmap_yaml = temp_dir / "local_costmap.yaml"
    shape_assembly_yaml = temp_dir / "shape_assembly.yaml"

    _write_yaml(graph_planner_yaml, {"GraphPlanner": graph_planner})
    _write_yaml(my_local_planner_yaml, {"MyPlanner": my_local_planner})
    _write_yaml(global_costmap_yaml, {"global_costmap": global_costmap})
    _write_yaml(local_costmap_yaml, {"local_costmap": local_costmap})
    _write_yaml(shape_assembly_yaml, shape_assembly)

    motion["graph_planner_config"] = str(graph_planner_yaml)
    motion["my_local_planner_config"] = str(my_local_planner_yaml)
    motion["global_costmap_config"] = str(global_costmap_yaml)
    motion["local_costmap_config"] = str(local_costmap_yaml)
    motion["shape_assembly_config"] = str(shape_assembly_yaml)

    robot_args = [
        f"{key}:={_ros_arg_value(value)}"
        for key, value in motion.items()
        if key in ROBOT_LAUNCH_KEYS and value not in ("", None)
    ]

    lines = [
        _emit_assignment("CONFIG_TEMP_DIR", str(temp_dir)),
        _emit_assignment("ROS_MASTER_URI_VALUE", ros.get("master_uri", "")),
        _emit_assignment("ROS_IP_VALUE", ros.get("ip", "")),
        _emit_assignment("TIME_SYNC_ENABLED", time_sync.get("enabled", True)),
        _emit_assignment("TIME_SYNC_SERVER", time_sync.get("server", "")),
        _emit_array("ROBOT_LAUNCH_ARGS", robot_args),
    ]
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("host", "robot"), required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--ws-root", required=True)
    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    ws_root = Path(args.ws_root).resolve()
    if not config_path.exists():
        raise FileNotFoundError(f"config file not found: {config_path}")

    config = _load_yaml(config_path, ws_root)
    if args.mode == "host":
        sys.stdout.write(_materialize_host(config, ws_root))
    else:
        sys.stdout.write(_materialize_robot(config, ws_root))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
