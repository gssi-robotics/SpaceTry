#!/usr/bin/env python3
import yaml

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory


class MissionServer(Node):
    def __init__(self):
        super().__init__("spacetry_mission_server")

        pkg_share = get_package_share_directory("spacetry_mission")
        self._waypoints_path = f"{pkg_share}/config/waypoints.yaml"
        self._objects_path = f"{pkg_share}/config/objects.yaml"

        self._srv_wp = self.create_service(
            Trigger, "/marti/mission/get_waypoints", self._handle_waypoints
        )
        self._srv_obj = self.create_service(
            Trigger, "/marti/mission/get_objects", self._handle_objects
        )

        self.get_logger().info(f"Waypoints: {self._waypoints_path}")
        self.get_logger().info(f"Objects:   {self._objects_path}")
        self.get_logger().info("Services ready: /marti/mission/get_waypoints, /marti/mission/get_objects")

    def _load_yaml(self, path: str):
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    def _handle_waypoints(self, request, response):
        try:
            data = self._load_yaml(self._waypoints_path)
            response.success = True
            response.message = yaml.safe_dump(data, sort_keys=False)
        except Exception as e:
            response.success = False
            response.message = f"Failed to load waypoints: {e}"
        return response

    def _handle_objects(self, request, response):
        try:
            data = self._load_yaml(self._objects_path)
            response.success = True
            response.message = yaml.safe_dump(data, sort_keys=False)
        except Exception as e:
            response.success = False
            response.message = f"Failed to load objects: {e}"
        return response


def main():
    rclpy.init()
    node = MissionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
