#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# ==============================================================================
# [1] 차량별 경로(Node Sequence) 데이터베이스 (CAV용)
# ==============================================================================
CAV_PATH_SETTINGS = [
    [21, 51, 46, 40, 43, 9, 56, 59, 18, 21],  # CAV 1
    [60, 52, 24, 37, 39, 49, 55, 12, 15, 60], # CAV 2
    [19, 22, 25, 36, 38, 48, 58, 19],         # CAV 3
    [16, 61, 47, 41, 42, 8, 11, 16],          # CAV 4
]

def load_yaml_file(file_path):
    try:
        with open(file_path, "r") as f:
            return yaml.safe_load(f)
    except Exception as e:
        return {}


def build_priority_gate_param_files(pkg_dir, cav_id):
    id_str = f"{int(cav_id):02d}"
    common_file = os.path.join(pkg_dir, "config", "priority_gate_common.yaml")
    cav_file = os.path.join(pkg_dir, "config", f"priority_gate_cav{id_str}.yaml")
    files = [common_file]
    if os.path.exists(cav_file):
        files.append(cav_file)
    return files


def resolve_mpc_node_params(full_config, cav_id, slot_index):
    id_str = f"{int(cav_id):02d}"
    slot_str = f"{int(slot_index) + 1:02d}"
    yaml_section_name = f"mpc_tracker_cav{id_str}"
    yaml_section_fallback = f"mpc_tracker_cav{slot_str}"
    if yaml_section_name in full_config:
        return full_config[yaml_section_name].get("ros__parameters", {})
    if yaml_section_fallback in full_config:
        return full_config[yaml_section_fallback].get("ros__parameters", {})
    return {}

def generate_launch_description():
    pkg_dir = get_package_share_directory("bisa")
    
    # 설정 파일 경로
    config_file = os.path.join(pkg_dir, "config", "cav_config.yaml")
    # RViz 설정 파일 경로 (bisa.rviz 파일이 해당 위치에 있어야 함)
    rviz_config = os.path.join(pkg_dir, "rviz", "bisa.rviz")

    # 1. YAML 파일 로드
    full_config = load_yaml_file(config_file)

    # 파라미터 추출 로직
    try:
        ros_params_dict = full_config["/**"]["ros__parameters"]
    except KeyError:
        ros_params_dict = {"cav_ids": [1, 2, 3, 4]}

    hv_settings = full_config.get("hv_settings", [])
    
    nodes = []

    # ---------------------------------------------------------
    # [추가됨] RViz2 실행 노드
    # ---------------------------------------------------------
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],  # 설정된 .rviz 파일 로드
            output='screen',
            additional_env={"ROS_DOMAIN_ID": "100"},
        )
    )

    # ---------------------------------------------------------
    # 2. 공통 노드 (HDMap Visualizer)
    # ---------------------------------------------------------
    nodes.append(
        Node(
            package="bisa",
            executable="hdmap_visualizer.py",
            name="hdmap_visualizer",
            output="screen",
            additional_env={"ROS_DOMAIN_ID": "100"},
        )
    )

    # ---------------------------------------------------------
    # 3. HV 차량 자동 생성
    # ---------------------------------------------------------
    for hv in hv_settings:
        hv_id = hv["id"]
        hv_path = hv["node_sequence"]

        nodes.append(
            Node(
                package="bisa",
                executable="global_path_pub_multi.py",
                name=f"global_path_pub_hv{hv_id}",
                output="screen",
                parameters=[
                    {"cav_id": hv_id, "node_sequence": hv_path, "rviz_slot": -1}
                ],
                remappings=[("/user_global_path", f"/hv{hv_id}/global_path")],
                additional_env={"ROS_DOMAIN_ID": str(hv_id)},
            )
        )

    # ---------------------------------------------------------
    # 4. 동적 CAV 차량 노드 생성
    # ---------------------------------------------------------
    active_ids = ros_params_dict.get("cav_ids", [1, 2, 3, 4])

    # ---------------------------------------------------------
    # 4-1. Runtime monitor GUIs
    # ---------------------------------------------------------
    nodes.append(
        Node(
            package="bisa",
            executable="cav_runtime_log_gui.py",
            name="cav_runtime_log_gui",
            output="screen",
            parameters=[{"cav_ids": active_ids}],
            additional_env={"ROS_DOMAIN_ID": "100"},
        )
    )
    nodes.append(
        Node(
            package="bisa",
            executable="cav_path_error_gui.py",
            name="cav_path_error_gui",
            output="screen",
            parameters=[
                {
                    "cav_ids": active_ids,
                    "focus_cav_id": 1,
                    "gui_frame_ms": 30,
                    "update_period_sec": 0.20,
                    "error_history_sec": 45.0,
                    "overshoot_error_m_threshold": 0.08,
                }
            ],
            additional_env={"ROS_DOMAIN_ID": "100"},
        )
    )

    # CAV 경로 설정 데이터 수에 맞춰 반복 (최대 4대)
    loop_count = min(len(active_ids), len(CAV_PATH_SETTINGS))
    cav_specs = []
    for index in range(loop_count):
        cav_id = active_ids[index]
        node_seq = CAV_PATH_SETTINGS[index]
        node_params = resolve_mpc_node_params(full_config, cav_id, index)
        cav_specs.append(
            {
                "index": index,
                "cav_id": cav_id,
                "node_seq": node_seq,
                "node_params": node_params,
            }
        )

    if cav_specs:
        scheduler_cav_ids = [spec["cav_id"] for spec in cav_specs]
        nominal_speed_entries = []
        for spec in cav_specs:
            cav_id = spec["cav_id"]
            node_params = spec["node_params"]
            nominal_speed = float(node_params.get("max_velocity", 0.55))
            nominal_speed_entries.append(f"{int(cav_id)}:{nominal_speed:.3f}")
        nodes.append(
            Node(
                package="bisa",
                executable="offline_speed_scheduler",
                name="offline_speed_scheduler",
                output="screen",
                parameters=[
                    {
                        "cav_ids": scheduler_cav_ids,
                        "default_speed_mps": 0.55,
                        "nominal_speed_entries": nominal_speed_entries,
                        "conflict_distance": 0.60,
                        "heading_conflict_min_deg": 12.0,
                        "zone_merge_distance": 0.75,
                        "zone_half_window_m": 0.65,
                        "safety_margin_sec": 1.80,
                        "max_start_delay_sec": 35.0,
                        "publish_hz": 10.0,
                        "plan_timeout_sec": 10.0,
                    }
                ],
                additional_env={"ROS_DOMAIN_ID": "100"},
            )
        )

    for spec in cav_specs:
        index = spec["index"]
        cav_id = spec["cav_id"]
        node_seq = spec["node_seq"]
        node_params = spec["node_params"]
        id_str = f"{cav_id:02d}"    # 예: "01"
        rviz_slot = index           # RViz 시각화 슬롯 (0~3)
        cav_prefix = f"/cav{id_str}"

        # (A) Global Path Publisher
        nodes.append(
            Node(
                package="bisa",
                executable="global_path_pub_multi.py",
                name=f"global_path_pub_cav{id_str}",
                output="screen",
                parameters=[
                    {
                        "cav_id": cav_id,
                        "node_sequence": node_seq,
                        "rviz_slot": rviz_slot,
                    }
                ],
                remappings=[
                    ("/user_global_path", f"{cav_prefix}/global_path"),
                ],
                additional_env={"ROS_DOMAIN_ID": "100"},
            )
        )

        # (B) Local Path Publisher
        nodes.append(
            Node(
                package="bisa",
                executable="local_path_pub_cpp",
                name=f"local_path_pub_cav{id_str}",
                output="screen",
                parameters=[
                    ros_params_dict, # 공통 파라미터
                    {
                        "target_cav_id": cav_id,
                        "rviz_slot": rviz_slot,
                        "local_path_size": 220,
                    },
                ],
                remappings=[
                    (f"/user_global_path_cav{id_str}", f"{cav_prefix}/global_path"),
                    (f"/local_path_cav{id_str}", f"{cav_prefix}/local_path"),
                    (f"/car_marker_{id_str}", f"{cav_prefix}/car_marker"),
                    (f"/lap_info_cav{id_str}", f"{cav_prefix}/lap_info"),
                ],
                additional_env={"ROS_DOMAIN_ID": "100"},
            )
        )

        # (C) MPC Path Tracker
        nodes.append(
            Node(
                package="bisa",
                executable="mpc_path_tracker_cpp",
                name=f"mpc_tracker_cav{id_str}",
                output="screen",
                parameters=[node_params, {"target_cav_id": cav_id, "publish_accel_cmd": False}],
                remappings=[
                    ("/local_path", f"{cav_prefix}/local_path"),
                    ("/Ego_pose", f"/CAV_{id_str}"),
                    ("/Accel", f"{cav_prefix}/accel_cmd"),
                    ("/Accel_raw", f"{cav_prefix}/accel_raw"),
                    ("/mpc_predicted_path", f"{cav_prefix}/mpc_predicted_path"),
                    ("/mpc_performance", f"{cav_prefix}/mpc_performance"),
                ],
                additional_env={"ROS_DOMAIN_ID": "100"},
            )
        )

        # (D) Priority Collision Gate (path-overlap based)
        priority_gate_param_files = build_priority_gate_param_files(pkg_dir, cav_id)
        priority_gate_param_files.append(
            {
                "cav_id": cav_id,
                "active_cav_ids": active_ids,
                # Offline-schedule first, gate logic as runtime fallback.
                "ttc_mode_enabled": False,
                "ttc_threshold_sec": 3.0,
                "ttc_tie_margin_sec": 0.15,
                "ttc_overlap_distance": 0.35,
                "ttc_conflict_distance": 0.35,
                "ttc_max_hit_index": 24,
                "ttc_physical_guard_distance": 0.80,
                "ttc_emergency_stop_distance": 0.45,
                "ttc_absolute_emergency_stop_distance": 0.40,
                "ttc_startup_hold_enabled": True,
                "ttc_startup_hold_sec": 1.0,
                "ttc_min_speed": 0.05,
                "ttc_default_other_speed": 0.35,
                "yield_speed_ratio": 0.55,
                "pass_speed_boost": 1.15,
                "max_velocity_cap": float(node_params.get("max_velocity", 0.72)),
                "overlap_distance": 0.62,
                "release_overlap_distance": 0.72,
                "stop_lookahead_index": 10,
                "hard_stop_lookahead_index": 14,
                "geometric_stop_pose_distance": 1.10,
                "same_priority_ids": [1, 2, 3, 4],
                "eta_min_speed": 0.15,
                "near_pose_stationary_speed_mps": 0.02,
                "near_pose_stationary_ignore_sec": 1.2,
                "same_priority_eta_margin_sec": 0.25,
                "preentry_stop_min_hit_index": 4,
                "same_priority_latch_hit_index": 20,
                "same_priority_latch_release_hit_index": 32,
                "same_priority_latch_timeout_sec": 2.0,
                "offline_speed_cap_enabled": True,
                "offline_speed_cap_timeout_sec": 2.0,
                "rear_same_direction_ignore_distance": 0.25,
                "rear_same_direction_heading_deg": 45.0,
                "rear_follow_slow_distance": 0.95,
                "rear_follow_slow_release_distance": 1.15,
                "rear_follow_stop_distance": 0.58,
                "rear_follow_stop_release_distance": 0.74,
                "rear_follow_lateral_distance": 0.28,
            }
        )
        nodes.append(
            Node(
                package="bisa",
                executable="priority_collision_gate",
                name=f"priority_collision_gate_cav{id_str}",
                output="screen",
                parameters=priority_gate_param_files,
                remappings=[
                    ("/Accel_raw", f"{cav_prefix}/accel_raw"),
                    ("/planned_path", f"{cav_prefix}/local_path"),
                    ("/Ego_pose", f"/CAV_{id_str}"),
                    ("/Accel", f"/sim/cav{id_str}/accel"),
                    ("/offline_speed_cap", f"{cav_prefix}/offline_speed_cap"),
                ],
                additional_env={"ROS_DOMAIN_ID": "100"},
            )
        )

    return LaunchDescription(nodes)
