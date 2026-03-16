#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import heapq
import math
import numpy as np


class DijkstraPlanner:
    def __init__(self, nodes, links, enable_lane_change=True, lane_change_distance=0.2):
        self.nodes = nodes
        self.links = links
        self.enable_lane_change = enable_lane_change
        self.lane_change_distance = lane_change_distance

        self.graph = self._build_graph()

    def _build_graph(self):
        graph = {}
        for node_idx in self.nodes.keys():
            graph[node_idx] = []
        for link_idx, link in self.links.items():
            from_node = link.from_node_idx
            to_node = link.to_node_idx
            cost = link.cost
            if from_node in graph:
                graph[from_node].append((to_node, link, cost))
        return graph

    def get_euclidean_distance(self, node1_idx, node2_idx):
        n1 = self.nodes[node1_idx]
        n2 = self.nodes[node2_idx]
        return math.sqrt(
            (n1.point[0] - n2.point[0]) ** 2 + (n1.point[1] - n2.point[1]) ** 2
        )

    def _to_list(self, data):
        """NumPy 배열이면 리스트로 변환, 아니면 그대로 반환 (에러 방지용)"""
        if hasattr(data, "tolist"):
            return data.tolist()
        return data

    def _trim_path_back(self, points, trim_dist):
        """경로 끝부분을 30cm 잘라내는 함수"""
        if not points or len(points) < 2:
            return points

        accumulated_dist = 0

        # 뒤에서부터 거리를 잼
        for i in range(len(points) - 1, 0, -1):
            curr_pt = points[i]
            prev_pt = points[i - 1]

            seg_dist = math.sqrt(
                (curr_pt[0] - prev_pt[0]) ** 2 + (curr_pt[1] - prev_pt[1]) ** 2
            )

            if accumulated_dist + seg_dist >= trim_dist:
                remain = trim_dist - accumulated_dist
                ratio = 1.0 - (remain / seg_dist)

                new_x = prev_pt[0] + (curr_pt[0] - prev_pt[0]) * ratio
                new_y = prev_pt[1] + (curr_pt[1] - prev_pt[1]) * ratio
                new_z = prev_pt[2] + (curr_pt[2] - prev_pt[2]) * ratio

                del points[i:]
                points.append([new_x, new_y, new_z])
                return points

            accumulated_dist += seg_dist

        del points[1:]
        return points

    def generate_path(self, node_sequence):
        if len(node_sequence) < 2:
            raise ValueError("Node sequence must have at least 2 nodes")

        total_node_path = [node_sequence[0]]
        total_point_path = []
        total_distance = 0.0

        # 시작점 처리 (안전 변환)
        start_node_obj = self.nodes[node_sequence[0]]
        total_point_path.append(self._to_list(start_node_obj.point))

        for i in range(len(node_sequence) - 1):
            curr_id = node_sequence[i]
            next_id = node_sequence[i + 1]

            found_direct_link = None

            # 1. 물리적 링크 확인
            if curr_id in self.graph:
                for target, link, cost in self.graph[curr_id]:
                    if target == next_id:
                        found_direct_link = link
                        break

            # 2. 링크 연결
            if found_direct_link:
                total_node_path.append(next_id)
                link_points = self._to_list(found_direct_link.points)  # 안전 변환
                total_point_path.extend(link_points)
                total_distance += found_direct_link.cost

            else:
                # [차선 변경 구간] -> 30cm Cut Logic
                dist = self.get_euclidean_distance(curr_id, next_id)

                if dist <= 2.0 or self.enable_lane_change:
                    # 기존 경로 끝 30cm 삭제
                    if len(total_point_path) > 1:
                        self._trim_path_back(total_point_path, 0.3)

                    path_end_pt = total_point_path[-1]
                    target_node_pt = self._to_list(self.nodes[next_id].point)  # 안전 변환

                    # 직선 보간 (5개 점)
                    steps = 5
                    for s in range(1, steps + 1):
                        t = s / steps
                        x = path_end_pt[0] + (target_node_pt[0] - path_end_pt[0]) * t
                        y = path_end_pt[1] + (target_node_pt[1] - path_end_pt[1]) * t
                        z = path_end_pt[2] + (target_node_pt[2] - path_end_pt[2]) * t
                        total_point_path.append([x, y, z])

                    total_node_path.append(next_id)
                    total_distance += dist
                # else:
                #     print(f"[ERROR] Too far: {curr_id}->{next_id}")

        return {
            "node_path": total_node_path,
            "point_path": total_point_path,
            "total_distance": total_distance,
        }
