#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import sys
import xml.etree.ElementTree as ET

import yaml


def repo_root():
    return os.path.split(os.path.realpath(__file__))[0] + "/../../"


ROOT_PATH = repo_root()


def yaml_parser(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.load(f, Loader=yaml.FullLoader)


def indent(elem, level=0):
    pad = "\n" + level * "\t"
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = pad + "\t"
        if not elem.tail or not elem.tail.strip():
            elem.tail = pad
        for child in elem:
            indent(child, level + 1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = pad
    elif level and (not elem.tail or not elem.tail.strip()):
        elem.tail = pad


def create_element(name, text=None, props=None):
    elem = ET.Element(name, attrib=props or {})
    if text is not None:
        elem.text = text
    return elem


def launch_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def name_to_type(name):
    if name == "static_map":
        return "costmap_2d::StaticLayer"
    if name == "voxel_layer":
        return "costmap_2d::VoxelLayer"
    if name == "obstacle_layer":
        return "costmap_2d::ObstacleLayer"
    if name == "voronoi_layer":
        return "costmap_2d::VoronoiLayer"
    if name == "inflation_layer":
        return "costmap_2d::InflationLayer"
    raise NotImplementedError("Unsupported map layer: %s" % name)


def write_simtest_costmap_plugins(user_cfg):
    plugins_cfg = user_cfg.get("plugins", {})
    map_layers = plugins_cfg.get("map_layers")
    if not map_layers:
        return

    maps_cfg = yaml_parser(os.path.join(ROOT_PATH, "user_config", map_layers))

    global_path = os.path.join(ROOT_PATH, "sim_env/config/costmap/global_costmap_plugins_simtest.yaml")
    local_path = os.path.join(ROOT_PATH, "sim_env/config/costmap/local_costmap_plugins_simtest.yaml")

    if "global_costmap" in maps_cfg:
        global_data = {
            "global_costmap": {
                "plugins": [{"name": name, "type": name_to_type(name)} for name in maps_cfg["global_costmap"]]
            }
        }
        with open(global_path, "w", encoding="utf-8") as f:
            yaml.dump(global_data, f, default_flow_style=None, sort_keys=False)

    if "local_costmap" in maps_cfg:
        local_data = {
            "local_costmap": {
                "plugins": [{"name": name, "type": name_to_type(name)} for name in maps_cfg["local_costmap"]]
            }
        }
        with open(local_path, "w", encoding="utf-8") as f:
            yaml.dump(local_data, f, default_flow_style=None, sort_keys=False)


def write_start_robots_simtest(user_cfg, path):
    def robot_value(field, index=-1):
        if index == -1:
            return "$(eval arg('robot' + str(arg('agent_id')) + '_%s'))" % field
        key = "robot%d_%s" % (index + 1, field)
        return user_cfg["robots_config"][index][key]

    launch = create_element("launch")
    robots_num = len(user_cfg.get("robots_config", []))
    if robots_num <= 0:
        raise ValueError("There is no robot!")

    launch.append(create_element("arg", props={"name": "agent_number", "default": str(robots_num)}))
    launch.append(create_element("arg", props={"name": "agent_id", "default": str(robots_num)}))
    launch.append(create_element("arg", props={"name": "spawn_wait_for", "default": ""}))
    map_to_odom = user_cfg.get("map_to_odom", "auto")
    if map_to_odom not in ("amcl", "gazebo", "auto"):
        map_to_odom = "auto"
    launch.append(create_element("arg", props={"name": "map_to_odom", "default": map_to_odom}))
    launch.append(create_element("arg", props={"name": "use_move_base", "default": launch_value(user_cfg.get("use_move_base", True))}))

    for i in range(robots_num):
        for field in ("type", "global_planner", "local_planner", "x_pos", "y_pos", "z_pos", "yaw"):
            launch.append(create_element("arg", props={"name": "robot%d_%s" % (i + 1, field), "value": robot_value(field, i)}))

    include = create_element("include", props={"file": "$(find sim_env)/launch/app/environment_single_simtest.launch.xml"})
    include.append(create_element("arg", props={"name": "agent_number", "value": "$(arg agent_number)"}))
    include.append(create_element("arg", props={"name": "agent_id", "value": "$(arg agent_id)"}))
    include.append(create_element("arg", props={"name": "robot", "value": "$(eval arg('robot' + str(arg('agent_id')) + '_type'))"}))
    include.append(create_element("arg", props={"name": "global_planner", "value": robot_value("global_planner")}))
    include.append(create_element("arg", props={"name": "local_planner", "value": robot_value("local_planner")}))
    include.append(create_element("arg", props={"name": "map_to_odom", "value": "$(arg map_to_odom)"}))
    include.append(create_element("arg", props={"name": "use_move_base", "value": "$(arg use_move_base)"}))
    include.append(create_element("arg", props={"name": "robot_namespace", "value": "robot$(arg agent_id)"}))
    include.append(create_element("arg", props={"name": "start_ns", "value": "true" if robots_num > 1 else "false"}))
    include.append(create_element("arg", props={"name": "robot_x", "value": "$(eval arg('robot' + str(arg('agent_id')) + '_x_pos'))"}))
    include.append(create_element("arg", props={"name": "robot_y", "value": "$(eval arg('robot' + str(arg('agent_id')) + '_y_pos'))"}))
    include.append(create_element("arg", props={"name": "robot_z", "value": "$(eval arg('robot' + str(arg('agent_id')) + '_z_pos'))"}))
    include.append(create_element("arg", props={"name": "robot_yaw", "value": "$(eval arg('robot' + str(arg('agent_id')) + '_yaw'))"}))
    include.append(create_element("arg", props={"name": "spawn_wait_for", "value": "$(arg spawn_wait_for)"}))

    cycle = create_element(
        "include",
        props={"file": "$(find sim_env)/launch/include/robots/start_robots_simtest.launch.xml", "if": "$(eval arg('agent_id') > 1)"},
    )
    cycle.append(create_element("arg", props={"name": "agent_id", "value": "$(eval arg('agent_id') - 1)"}))
    cycle.append(create_element("arg", props={"name": "spawn_wait_for", "value": "robot$(arg agent_id)"}))

    launch.append(include)
    launch.append(cycle)
    indent(launch)

    with open(path, "wb+") as f:
        ET.ElementTree(launch).write(f, encoding="utf-8", xml_declaration=True)


def write_main_simtest(user_cfg, path):
    launch = create_element("launch")
    launch.append(create_element("arg", props={"name": "world_parameter", "value": user_cfg["world"]}))

    include = create_element("include", props={"file": "$(find sim_env)/launch/config_simtest.launch"})
    include.append(create_element("arg", props={"name": "world", "value": "$(arg world_parameter)"}))
    include.append(create_element("arg", props={"name": "map", "value": user_cfg["map"]}))
    include.append(create_element("arg", props={"name": "robot_number", "value": str(len(user_cfg["robots_config"]))}))
    include.append(create_element("arg", props={"name": "rviz_file", "value": user_cfg["rviz_file"]}))
    include.append(create_element("arg", props={"name": "debug", "value": launch_value(user_cfg.get("debug", False))}))
    include.append(create_element("arg", props={"name": "gui", "value": launch_value(user_cfg.get("gui", True))}))
    include.append(create_element("arg", props={"name": "headless", "value": launch_value(user_cfg.get("headless", False))}))
    include.append(create_element("arg", props={"name": "use_rviz", "value": launch_value(user_cfg.get("use_rviz", True))}))
    launch.append(include)
    indent(launch)

    with open(path, "wb+") as f:
        ET.ElementTree(launch).write(f, encoding="utf-8", xml_declaration=True)


def main():
    if len(sys.argv) != 2:
        raise SystemExit("Usage: main_generate_simtest.py <user_config.yaml>")

    user_cfg = yaml_parser(os.path.join(ROOT_PATH, "user_config", sys.argv[1]))
    write_simtest_costmap_plugins(user_cfg)
    write_start_robots_simtest(
        user_cfg,
        os.path.join(ROOT_PATH, "sim_env/launch/include/robots/start_robots_simtest.launch.xml"),
    )
    write_main_simtest(
        user_cfg,
        os.path.join(ROOT_PATH, "sim_env/launch/main_simtest.launch"),
    )


if __name__ == "__main__":
    main()
