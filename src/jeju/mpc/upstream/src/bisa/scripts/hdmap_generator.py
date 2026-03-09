#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import json
import math
from collections import defaultdict
from ament_index_python.packages import get_package_share_directory


class HDMapGenerator:
    def __init__(self):
        # ROS2 패키지 경로 사용
        try:
            package_share = get_package_share_directory("bisa")
            self.waypoint_dir = os.path.join(package_share, "waypoint")
            self.output_dir = os.path.join(package_share, "hdmap_data")
        except:
            # 개발 환경 (소스에서 실행)
            current_dir = os.path.dirname(os.path.abspath(__file__))
            package_path = os.path.dirname(current_dir)
            self.waypoint_dir = os.path.join(package_path, "waypoint")
            self.output_dir = os.path.join(package_path, "hdmap_data")

        # print(f"Waypoint dir: {self.waypoint_dir}")
        # print(f"Output dir: {self.output_dir}")

        self.nodes = {}
        self.links = {}

    def scan_waypoint_files(self):
        """waypoint CSV 파일들을 스캔"""
        if not os.path.exists(self.waypoint_dir):
            raise FileNotFoundError(
                f"Waypoint directory not found: {self.waypoint_dir}"
            )

        csv_files = sorted(
            [f for f in os.listdir(self.waypoint_dir) if f.endswith(".csv")]
        )
        # print(f"\n{'='*60}")
        # print(f"Found {len(csv_files)} waypoint CSV files")
        # print(f"{'='*60}\n")

        for csv_file in csv_files:
            base_name = csv_file.replace(".csv", "")
            parts = base_name.split("_")

            if len(parts) != 2:
                # print(f"[SKIP] Invalid filename: {csv_file}")
                continue

            from_node = int(parts[0])
            to_node = int(parts[1])

            csv_path = os.path.join(self.waypoint_dir, csv_file)
            points = self._read_csv(csv_path)

            if len(points) < 2:
                # print(f"[SKIP] Not enough points in {csv_file}")
                continue

            link_id = f"LINK_{from_node}_{to_node}"
            cost = self._calculate_distance(points)

            self.links[link_id] = {
                "idx": link_id,
                "from_node_idx": f"NODE_{from_node}",
                "to_node_idx": f"NODE_{to_node}",
                "points": points,
                "cost": cost,
            }

    def _read_csv(self, csv_path):
        """CSV 파일 읽기"""
        points = []

        with open(csv_path, "r") as f:
            reader = csv.reader(f)
            rows = list(reader)

            if not rows:
                return points

            # 헤더 감지
            start_idx = 0
            try:
                float(rows[0][0])
                float(rows[0][1])
            except:
                start_idx = 1

            # 좌표 읽기 (중복 제거)
            prev_x, prev_y = None, None
            for row in rows[start_idx:]:
                try:
                    if len(row) >= 2:
                        x = float(row[0])
                        y = float(row[1])

                        # 유효성 체크
                        if (
                            math.isnan(x)
                            or math.isnan(y)
                            or math.isinf(x)
                            or math.isinf(y)
                        ):
                            continue

                        # 중복 체크 (0.1mm 단위)
                        if (
                            prev_x is None
                            or abs(x - prev_x) > 0.0001
                            or abs(y - prev_y) > 0.0001
                        ):
                            points.append([x, y, 0.0])
                            prev_x, prev_y = x, y

                except (ValueError, IndexError):
                    continue

        return points

    def _calculate_distance(self, points):
        """이 거리 계산"""
        if len(points) < 2:
            return 0.01

        total_distance = 0.0

        for i in range(len(points) - 1):
            x1, y1 = points[i][0], points[i][1]
            x2, y2 = points[i + 1][0], points[i + 1][1]

            dx = x2 - x1
            dy = y2 - y1

            dist = math.sqrt(dx * dx + dy * dy)

            # 유효성 체크
            if not (math.isnan(dist) or math.isinf(dist)):
                total_distance += dist

        # 최소값 보장
        if total_distance < 0.001:
            total_distance = 0.01

        return total_distance

    def generate_nodes(self):
        """노드 생성"""
        node_positions = defaultdict(list)

        for link in self.links.values():
            from_id = int(link["from_node_idx"].replace("NODE_", ""))
            to_id = int(link["to_node_idx"].replace("NODE_", ""))

            points = link["points"]
            if points:
                node_positions[from_id].append(points[0])
                node_positions[to_id].append(points[-1])

        for node_id, positions in node_positions.items():
            avg_x = sum(p[0] for p in positions) / len(positions)
            avg_y = sum(p[1] for p in positions) / len(positions)

            self.nodes[f"NODE_{node_id}"] = {
                "idx": f"NODE_{node_id}",
                "node_id": node_id,
                "point": [avg_x, avg_y, 0.0],
                "junction": [],
            }

        # print(f"\n{'='*60}")
        # print(f"Generated {len(self.nodes)} nodes")
        # print(f"{'='*60}")

    def save_json(self):
        """JSON 파일 저장"""
        os.makedirs(self.output_dir, exist_ok=True)

        with open(os.path.join(self.output_dir, "node_set.json"), "w") as f:
            json.dump(list(self.nodes.values()), f, indent=2)
        # print(f"\n✓ Saved node_set.json ({len(self.nodes)} nodes)")

        with open(os.path.join(self.output_dir, "link_set.json"), "w") as f:
            json.dump(list(self.links.values()), f, indent=2)
        # print(f"✓ Saved link_set.json ({len(self.links)} links)")

        global_info = {
            "maj_ver": 1,
            "min_ver": 0,
            "global_coordinate_system": "Local",
            "local_origin_in_global": [0.0, 0.0, 0.0],
            "lane_change_link_included": False,
        }
        with open(os.path.join(self.output_dir, "global_info.json"), "w") as f:
            json.dump(global_info, f, indent=2)
        # print(f"✓ Saved global_info.json")

        total_dist = sum(l["cost"] for l in self.links.values())
        # print(f"\n{'='*60}")
        # print(f"HD Map Generation Complete!")
        # print(f"  Total Distance: {total_dist:.2f}m")
        # print(f"  Output: {self.output_dir}")
        # print(f"{'='*60}\n")

    def generate(self):
        """전체 프로세스 실행"""
        # print("\n" + "=" * 60)
        # print("BISA HD Map Generator (ROS2)")
        # print("=" * 60)

        self.scan_waypoint_files()
        self.generate_nodes()
        self.save_json()


def main():
    generator = HDMapGenerator()
    generator.generate()


if __name__ == "__main__":
    main()
