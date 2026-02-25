#!/usr/bin/env python3
import rclpy
import py_trees
from py_nodes.base_bt_nodes import (
    NavigateToWaypoint,
    ObstacleFront,
    AvoidObstacle,
    AlignWithGoal,
    StopAndObserve,
    LogMessage
)

class NavigateToWaypointAction(py_trees.behaviour.Behaviour):
    def __init__(self, waypoint):
        super().__init__("NavigateToWaypoint")
        self.node = NavigateToWaypoint(waypoint)
        self.done = False

    def update(self):
        if not self.done:
            self.node.run()
            self.done = True
        return py_trees.common.Status.SUCCESS

class ObstacleFrontAction(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("ObstacleFront")
        self.node = ObstacleFront()

    def update(self):
        return py_trees.common.Status.SUCCESS if self.node.run() else py_trees.common.Status.FAILURE

class AvoidObstacleAction(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("AvoidObstacle")
        self.node = AvoidObstacle()
        self.done = False

    def update(self):
        if not self.done:
            self.node.run()
            self.done = True
        return py_trees.common.Status.SUCCESS

class AlignWithGoalAction(py_trees.behaviour.Behaviour):
    def __init__(self, waypoint):
        super().__init__("AlignWithGoal")
        self.node = AlignWithGoal(waypoint)
        self.done = False

    def update(self):
        if not self.done:
            self.node.run()
            self.done = True
        return py_trees.common.Status.SUCCESS

class StopAndObserveAction(py_trees.behaviour.Behaviour):
    def __init__(self, duration_s=5.0):
        super().__init__("StopAndObserve")
        self.node = StopAndObserve(duration_s)
        self.done = False

    def update(self):
        if not self.done:
            self.node.run()
            self.done = True
        return py_trees.common.Status.SUCCESS

class LogMessageAction(py_trees.behaviour.Behaviour):
    def __init__(self, message):
        super().__init__("LogMessage")
        self.node = LogMessage(message)
        self.done = False

    def update(self):
        if not self.done:
            self.node.run()
            self.done = True
        return py_trees.common.Status.SUCCESS



def main():
    rclpy.init()
    node = rclpy.create_node('bt_runner')
    # Declare parameters with default values
    node.declare_parameter('tree_file', '/ws/install/spacetry_bt/share/spacetry_bt/trees/base_bt.xml')
    node.declare_parameter('tick_hz', 10.0)
    node.declare_parameter('max_runtime_s', 30.0)

    tree_file = node.get_parameter('tree_file').get_parameter_value().string_value
    tick_hz = node.get_parameter('tick_hz').get_parameter_value().double_value
    max_runtime_s = node.get_parameter('max_runtime_s').get_parameter_value().double_value

    # Example: science_rock coordinates could be loaded from the tree or parameter
    science_rock = (5.0, 3.0)
    root = py_trees.composites.Sequence("MissionSequence", memory=True)
    root.add_children([
        NavigateToWaypointAction(science_rock),
        py_trees.composites.Selector("ObstacleAvoidance", [
            py_trees.composites.Sequence("AvoidIfObstacle", [
                ObstacleFrontAction(),
                AvoidObstacleAction()
            ]),
            py_trees.behaviours.Success("Pass")
        ]),
        AlignWithGoalAction(science_rock),
        StopAndObserveAction(5.0),
        LogMessageAction("Science rock reached")
    ])
    tree = py_trees.trees.BehaviourTree(root)
    # Setup feedback and visualisation (like C++ runner)
    print(root)
    visitor = py_trees.visitors.DisplaySnapshotVisitor()
    tree.add_visitor(visitor)
    period_ms = int(1000.0 / tick_hz)
    iterations = int(max_runtime_s * tick_hz)
    for i in range(iterations):
        print(f"\n[Tick {i+1}/{iterations}]")
        tree.tick()
        print(root)
        py_trees.display.print_snapshot_visitor(visitor)
        time.sleep(period_ms / 1000.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
