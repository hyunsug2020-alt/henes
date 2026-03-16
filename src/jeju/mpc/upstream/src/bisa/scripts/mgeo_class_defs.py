#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os


class Node:
    def __init__(self, idx, point, junction=None):
        self.idx = idx
        self.point = point
        self.junction = junction if junction else []


class Link:
    def __init__(self, idx, points, from_node_idx=None, to_node_idx=None, cost=None):
        self.idx = idx
        self.points = points
        self.from_node_idx = from_node_idx  # 추가
        self.to_node_idx = to_node_idx  # 추가
        self.cost = cost


class NodeSet:
    def __init__(self):
        self.nodes = {}


class LinkSet:
    def __init__(self):
        self.lines = {}


class MGeoPlannerMap:
    def __init__(self):
        self.node_set = NodeSet()
        self.link_set = LinkSet()
        self.global_info = {}

    @staticmethod
    def create_instance_from_json(load_path):
        """JSON에서 MGeo 맵 로드"""
        mgeo = MGeoPlannerMap()

        # print(f"\nLoading HD Map from: {load_path}")

        # node_set.json
        with open(os.path.join(load_path, "node_set.json"), "r") as f:
            node_list = json.load(f)

        for node_data in node_list:
            node = Node(
                idx=node_data["idx"],
                point=node_data["point"],
                junction=node_data.get("junction", []),
            )
            mgeo.node_set.nodes[node.idx] = node

        # print(f"✓ Loaded {len(node_list)} nodes")

        # link_set.json
        with open(os.path.join(load_path, "link_set.json"), "r") as f:
            link_list = json.load(f)

        for link_data in link_list:
            link = Link(
                idx=link_data["idx"],
                points=link_data["points"],
                from_node_idx=link_data.get("from_node_idx"),  # 추가
                to_node_idx=link_data.get("to_node_idx"),  # 추가
                cost=link_data.get("cost"),
            )
            mgeo.link_set.lines[link.idx] = link

        # print(f"✓ Loaded {len(link_list)} links\n")

        return mgeo
