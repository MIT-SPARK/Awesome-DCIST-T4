"""Execute explicit Phoenix path tasks, one accepted goal at a time.

No command-velocity publisher. Existing SRP owns motion and stopping. This node
is opt-in and must replace (not run alongside) the legacy path-to-goal bridge.
"""
import copy
import json
import math
import signal
import time
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from phx_nav_msgs.action import MRPGoal, SRPGoal
from phx_nav_msgs.msg import Trajectory, TrajectoryArray, Waypoint
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from robot_executor_msgs.msg import ActionSequenceMsg
from std_msgs.msg import String
from std_srvs.srv import Trigger


def validate_tasks(message, robot):
    """Validate the whole mission before any action goal can be sent."""
    if not message.plan_id or message.robot_name != robot:
        raise ValueError("A plan ID and matching robot_name are required")
    if not message.actions:
        raise ValueError("Mission has no tasks")
    tasks = []
    for action in message.actions:
        if action.action_type != "FOLLOW":
            raise ValueError(f"Unsupported task: {action.action_type}")
        path = copy.deepcopy(action.path)
        if not path.header.frame_id or len(path.poses) < 2:
            raise ValueError("Each task needs a frame and at least two poses")
        for pose in path.poses:
            if pose.header.frame_id and pose.header.frame_id != path.header.frame_id:
                raise ValueError("Mixed coordinate frames within a task")
            pose.header.frame_id = path.header.frame_id
            p = pose.pose.position
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                raise ValueError("Non-finite task position")
            q = pose.pose.orientation
            if not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)):
                raise ValueError("Non-finite task orientation")
            norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            if norm < 1e-6:
                raise ValueError("Invalid task orientation")
            q.x, q.y, q.z, q.w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        tasks.append(path)
    return tasks


class TaskExecutor(Node):
    def __init__(self):
        super().__init__("phoenix_task_executor")
        self.robot = self.declare_parameter("robot", "gauss").value
        self.goal_radius = self.declare_parameter("goal_radius", 0.75).value
        if not math.isfinite(self.goal_radius) or self.goal_radius <= 0:
            raise ValueError("goal_radius must be positive and finite")
        self.plan_id = ""
        self.tasks = []
        self.index = 0
        self.state = "idle"
        self.reason = ""
        self.seen = deque(maxlen=256)
        self.handles = {}
        self.pending = set()
        self.results = {}
        self.deadline = 0.0
        self.ready_at = 0.0
        self.action_clients = {
            "mrp": ActionClient(self, MRPGoal, "mid_range_planner_action"),
            "srp": ActionClient(self, SRPGoal, "short_range_planner_action"),
        }
        self.path_pub = self.create_publisher(
            TrajectoryArray, "planner/PASSTHROUGH/mrp/passthrough_topic", 1)
        self.status_pub = self.create_publisher(String, "task_executor/status", 10)
        self.subscription = self.create_subscription(
            ActionSequenceMsg, "omniplanner_node/compiled_tasks_out", self.receive, 1)
        self.cancel_service = self.create_service(Trigger, "task_executor/cancel", self.cancel)
        self.timer = self.create_timer(0.1, self.tick)

    def status(self, state, reason="", plan_id=None):
        self.status_pub.publish(String(data=json.dumps({
            "plan_id": self.plan_id if plan_id is None else plan_id,
            "state": state, "task_index": self.index,
            "task_count": len(self.tasks), "reason": reason,
        })))

    def receive(self, message):
        if message.plan_id in self.seen:
            self.status("ignored", "Duplicate plan ID", message.plan_id)
            return
        if self.state != "idle":
            self.status("rejected", "Another mission is active; cancel it first", message.plan_id)
            return
        try:
            tasks = validate_tasks(message, self.robot)
        except ValueError as error:
            self.status("rejected", str(error), message.plan_id)
            return
        self.plan_id, self.tasks, self.index = message.plan_id, tasks, 0
        self.seen.append(message.plan_id)
        self.reason = ""
        self.begin_leg()

    def begin_leg(self):
        self.handles, self.pending, self.results = {}, set(), {}
        self.state = "waiting_servers"
        self.deadline = time.monotonic() + 5.0
        self.status("starting")

    def waypoint(self):
        waypoint = Waypoint()
        waypoint.name = f"{self.plan_id}/task/{self.index}"
        waypoint.id = self.index
        waypoint.region_center = copy.deepcopy(self.tasks[self.index].poses[-1])
        waypoint.region_center.header.stamp = self.get_clock().now().to_msg()
        waypoint.radius = self.goal_radius
        waypoint.angle_threshold = 2 * math.pi
        return waypoint

    def publish_path(self, clear=False):
        message = TrajectoryArray()
        if not clear:
            trajectory = Trajectory()
            stamp = self.get_clock().now().to_msg()
            for pose in self.tasks[self.index].poses:
                odom = Odometry()
                odom.header = copy.deepcopy(pose.header)
                # Equal timestamps request MPPI's normal distance-based timing.
                odom.header.stamp = stamp
                odom.pose.pose = copy.deepcopy(pose.pose)
                trajectory.trajectory.append(odom)
            message.trajectories = [trajectory]
        self.path_pub.publish(message)

    def send(self, role):
        goal = MRPGoal.Goal() if role == "mrp" else SRPGoal.Goal()
        goal.waypoint = self.waypoint()
        token = (self.plan_id, self.index)
        self.pending.add(role)
        self.deadline = time.monotonic() + 5.0
        future = self.action_clients[role].send_goal_async(goal)
        future.add_done_callback(lambda f: self.accepted(role, token, f))

    def accepted(self, role, token, future):
        if token != (self.plan_id, self.index):
            handle = future.result()
            if handle and handle.accepted:
                handle.cancel_goal_async()
            return
        self.pending.discard(role)
        try:
            handle = future.result()
        except Exception as error:
            self.results[role] = GoalStatus.STATUS_ABORTED
            self.stop("aborted", f"{role} goal request failed: {error}")
            return
        if not handle.accepted:
            self.results[role] = GoalStatus.STATUS_ABORTED
            self.stop("aborted", f"{role} rejected the task")
            return
        self.handles[role] = handle
        handle.get_result_async().add_done_callback(lambda f: self.completed(role, token, f))
        if self.state in ("canceling", "aborting", "finishing_leg"):
            handle.cancel_goal_async()
        elif role == "mrp":
            self.state = "starting_srp"
            self.ready_at = time.monotonic() + 0.2
        else:
            self.state = "running"
            self.status("running")

    def completed(self, role, token, future):
        if token != (self.plan_id, self.index):
            return
        try:
            status = future.result().status
        except Exception as error:
            status = GoalStatus.STATUS_ABORTED
            self.reason = str(error)
        self.results[role] = status
        if self.state in ("canceling", "aborting", "finishing_leg"):
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.stop("aborted", f"{role} finished with status {status}")
        elif role == "srp":
            self.state = "finishing_leg"
            self.publish_path(clear=True)
            self.cancel_handles()
            self.ready_at = time.monotonic() + 0.25

    def cancel_handles(self):
        for role, handle in self.handles.items():
            if role not in self.results:
                handle.cancel_goal_async()

    def stop(self, outcome, reason):
        self.state = "canceling" if outcome == "canceled" else "aborting"
        self.reason = reason
        self.publish_path(clear=True)
        self.cancel_handles()
        self.status(self.state, reason)

    def cancel(self, request, response):
        if self.state != "idle":
            self.stop("canceled", "Operator requested cancellation")
        response.success = True
        response.message = "Cancellation requested; wait for canceled status before another mission"
        return response

    def all_terminal(self):
        return not self.pending and all(role in self.results for role in self.handles)

    def tick(self):
        now = time.monotonic()
        if self.state == "idle":
            return
        if self.pending and self.state not in ("canceling", "aborting") and now > self.deadline:
            self.stop("aborted", "Planner goal acceptance timed out; awaiting cancellation")
        if self.state == "waiting_servers":
            if all(client.server_is_ready() for client in self.action_clients.values()):
                self.publish_path()
                self.state = "starting_mrp"
                self.send("mrp")
            elif now > self.deadline:
                self.stop("aborted", "Planner action servers unavailable")
        elif self.state in ("starting_mrp", "starting_srp", "running"):
            self.publish_path()
            if self.state == "starting_srp" and now >= self.ready_at and "srp" not in self.pending:
                self.state = "awaiting_srp"
                self.send("srp")
        elif self.state == "awaiting_srp":
            self.publish_path()
        elif self.state == "finishing_leg" and self.all_terminal() and now >= self.ready_at:
            self.status("task_succeeded")
            self.index += 1
            if self.index == len(self.tasks):
                self.status("succeeded")
                self.state = "idle"
            else:
                self.begin_leg()
        elif self.state in ("canceling", "aborting") and self.all_terminal():
            self.status("canceled" if self.state == "canceling" else "aborted", self.reason)
            self.state = "idle"


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    node = TaskExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.state != "idle":
            node.stop("canceled", "Task executor shutting down")
            deadline = time.monotonic() + 5.0
            while node.state != "idle" and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if node.state != "idle":
                node.get_logger().error("Planner cancellation unconfirmed at shutdown")
    finally:
        node.destroy_node()
        rclpy.shutdown()
