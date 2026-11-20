"""Real ROS actions against fake planners; run only in a network-isolated container."""
import copy
import json
import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from phx_nav_msgs.action import MRPGoal, SRPGoal
from phx_nav_msgs.msg import TrajectoryArray
from phoenix_task_executor.node import TaskExecutor, validate_tasks
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from robot_executor_msgs.msg import ActionMsg, ActionSequenceMsg
from std_msgs.msg import String
from std_srvs.srv import Trigger


def mission(plan_id="out-and-back"):
    message = ActionSequenceMsg(plan_id=plan_id, robot_name="gauss")
    # The final goal is back at the start, which must not complete the outbound leg.
    for points in ([(0., 0.), (5., 1.), (10., 0.)], [(10., 0.), (5., -1.), (0., 0.)]):
        action = ActionMsg(action_type="FOLLOW")
        action.path.header.frame_id = "map"
        for x, y in points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.
            action.path.poses.append(pose)
        message.actions.append(action)
    return message


class FakePlanners(Node):
    def __init__(self):
        super().__init__("fake_planners")
        self.goals = {"mrp": [], "srp": []}
        self.futures = {"mrp": [], "srp": []}
        self.handles = {}
        self.reject = set()
        self.paths, self.statuses = [], []
        self.servers = []
        for role, kind, topic in [("mrp", MRPGoal, "mid_range_planner_action"),
                                  ("srp", SRPGoal, "short_range_planner_action")]:
            async def execute(handle, r=role, k=kind):
                return await self.execute(r, k, handle)
            self.servers.append(ActionServer(
                self, kind, topic,
                execute_callback=execute,
                goal_callback=lambda g, r=role: GoalResponse.REJECT if r in self.reject else GoalResponse.ACCEPT,
                cancel_callback=lambda h, r=role: self.canceled(r, h),
                callback_group=ReentrantCallbackGroup()))
        self.path_sub = self.create_subscription(TrajectoryArray,
            "planner/PASSTHROUGH/mrp/passthrough_topic", self.paths.append, 10)
        self.status_sub = self.create_subscription(String, "task_executor/status",
            lambda m: self.statuses.append(json.loads(m.data)), 10)
        self.pub = self.create_publisher(ActionSequenceMsg, "omniplanner_node/compiled_tasks_out", 1)

    async def execute(self, role, kind, handle):
        self.goals[role].append(copy.deepcopy(handle.request.waypoint))
        future = Future()
        self.futures[role].append(future)
        self.handles[bytes(handle.goal_id.uuid)] = future
        status = await future
        if status == GoalStatus.STATUS_SUCCEEDED:
            handle.succeed()
        elif status == GoalStatus.STATUS_CANCELED:
            handle.canceled()
        else:
            handle.abort()
        return kind.Result()

    def canceled(self, role, handle):
        future = self.handles.get(bytes(handle.goal_id.uuid))
        if future and not future.done():
            # Complete after the cancel service has accepted the request.
            self.cancel_timer = self.create_timer(.02, lambda: self.finish_cancel(future))
        return CancelResponse.ACCEPT

    def finish_cancel(self, future):
        if not future.done():
            future.set_result(GoalStatus.STATUS_CANCELED)


@pytest.fixture
def harness():
    rclpy.init()
    executor = SingleThreadedExecutor()
    task, fake = TaskExecutor(), FakePlanners()
    executor.add_node(task)
    executor.add_node(fake)
    def wait(predicate, timeout=5):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.02)
            if predicate():
                return
        raise AssertionError(f"Timed out: state={task.state}, goals={fake.goals}, statuses={fake.statuses}")
    wait(lambda: fake.pub.get_subscription_count() > 0)
    yield task, fake, wait
    executor.shutdown()
    task.destroy_node()
    fake.destroy_node()
    rclpy.shutdown()


def test_outbound_then_return_only_after_success(harness):
    task, fake, wait = harness
    fake.pub.publish(mission())
    wait(lambda: len(fake.goals['srp']) == 1)
    assert fake.goals['srp'][0].region_center.pose.position.x == 10.
    assert fake.goals['mrp'][0].region_center.pose.position.x == 10.
    wait(lambda: any(p.trajectories for p in fake.paths))
    assert all(p.trajectories[0].trajectory[-1].pose.pose.position.x == 10.
               for p in fake.paths if p.trajectories)
    # MRP success is not task completion. Only SRP success permits advancement.
    fake.futures['mrp'][0].set_result(GoalStatus.STATUS_SUCCEEDED)
    wait(lambda: 'mrp' in task.results)
    assert len(fake.goals['srp']) == 1
    fake.futures['srp'][0].set_result(GoalStatus.STATUS_SUCCEEDED)
    wait(lambda: len(fake.goals['srp']) == 2)
    assert fake.goals['srp'][1].region_center.pose.position.x == 0.
    assert fake.goals['mrp'][1].region_center.pose.position.x == 0.
    wait(lambda: any(p.trajectories and p.trajectories[0].trajectory[-1].pose.pose.position.x == 0.
                     for p in fake.paths))
    fake.futures['srp'][1].set_result(GoalStatus.STATUS_SUCCEEDED)
    wait(lambda: any(s['state'] == 'succeeded' for s in fake.statuses))
    assert task.state == 'idle'
    assert fake.futures['mrp'][1].done()  # old MRP canceled before mission completion
    fake.pub.publish(mission())
    wait(lambda: any(s['state'] == 'ignored' for s in fake.statuses))
    assert len(fake.goals['srp']) == 2


@pytest.mark.parametrize('role', ['mrp', 'srp'])
def test_rejected_goal_never_advances(harness, role):
    task, fake, wait = harness
    fake.reject.add(role)
    fake.pub.publish(mission())
    wait(lambda: any(s['state'] == 'aborted' for s in fake.statuses))
    assert not fake.goals['srp']
    assert task.index == 0


@pytest.mark.parametrize('role', ['mrp', 'srp'])
def test_failed_goal_never_advances(harness, role):
    task, fake, wait = harness
    fake.pub.publish(mission())
    wait(lambda: len(fake.goals['srp']) == 1)
    fake.futures[role][0].set_result(GoalStatus.STATUS_ABORTED)
    wait(lambda: any(s['state'] == 'aborted' for s in fake.statuses))
    assert len(fake.goals['srp']) == 1
    assert all(f.done() for futures in fake.futures.values() for f in futures)
    assert task.index == 0


def test_cancel_and_busy_rejection(harness):
    task, fake, wait = harness
    fake.pub.publish(mission())
    wait(lambda: len(fake.goals['srp']) == 1)
    fake.pub.publish(mission('replacement'))
    wait(lambda: any(s['state'] == 'rejected' for s in fake.statuses))
    assert task.plan_id == 'out-and-back'
    client = fake.create_client(Trigger, 'task_executor/cancel')
    wait(lambda: client.service_is_ready())
    future = client.call_async(Trigger.Request())
    wait(lambda: future.done())
    wait(lambda: any(s['state'] == 'canceled' for s in fake.statuses))
    assert len(fake.goals['srp']) == 1
    assert all(f.done() for futures in fake.futures.values() for f in futures)


@pytest.mark.parametrize('bad', ['robot', 'empty', 'type', 'frame', 'nan', 'quaternion'])
def test_validate_entire_mission_before_first_goal(bad):
    message = mission()
    if bad == 'robot': message.robot_name = 'ada'
    if bad == 'empty': message.actions[1].path.poses = []
    if bad == 'type': message.actions[1].action_type = 'GAZE'
    if bad == 'frame': message.actions[1].path.poses[0].header.frame_id = 'odom'
    if bad == 'nan': message.actions[1].path.poses[0].pose.position.x = float('nan')
    if bad == 'quaternion': message.actions[1].path.poses[0].pose.orientation.w = 0.
    with pytest.raises(ValueError): validate_tasks(message, 'gauss')


def test_cancel_while_goal_acceptance_is_in_flight(harness):
    from types import SimpleNamespace
    task, fake, wait = harness
    # Model a request already sent, whose acceptance reply has not arrived.
    task.plan_id = 'pending'
    task.tasks = validate_tasks(mission(), 'gauss')
    task.state = 'awaiting_srp'
    task.pending.add('srp')
    task.stop('canceled', 'test')
    assert not task.all_terminal()
    class Handle:
        accepted = True
        def __init__(self): self.cancels = 0; self.result = Future()
        def get_result_async(self): return self.result
        def cancel_goal_async(self): self.cancels += 1
    handle = Handle()
    future = Future(); future.set_result(handle)
    task.accepted('srp', ('pending', 0), future)
    assert handle.cancels == 1
    handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    wait(lambda: task.state == 'idle')
    assert not fake.goals['srp']


def test_invalid_later_task_cannot_start_first_task(harness):
    task, fake, wait = harness
    message = mission()
    message.actions[1].path.poses[0].pose.position.x = float('nan')
    fake.pub.publish(message)
    wait(lambda: any(s['state'] == 'rejected' for s in fake.statuses))
    assert not fake.goals['mrp'] and not fake.goals['srp']
