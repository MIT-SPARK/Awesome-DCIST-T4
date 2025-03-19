import time
from functools import singledispatch

import numpy as np
import rclpy.time
from geometry_msgs.msg import Point
from robot_executor_msgs.msg import ActionMsg, ActionSequenceMsg
from spot_executor.utils import path_to_waypoints, waypoints_to_path
from visualization_msgs.msg import Marker, MarkerArray

from robot_executor_interface.action_descriptions import (
    ActionSequence,
    Follow,
    Gaze,
    Pick,
    Place,
)


# get time message
def gtm():
    return rclpy.time.Time(nanoseconds=time.time() * 1e9).to_msg()


@singledispatch
def to_msg(action):
    raise Exception(f"Unknown action type {type(action)}")


@singledispatch
def to_viz_msg(action, marker_ns):
    raise Exception(f"Unknown action type {type(action)}")


@to_msg.register
def _(action: ActionSequence):
    msg = ActionSequenceMsg()
    msg.plan_id = action.plan_id
    msg.robot_name = action.robot_name
    msg.header.stamp = gtm()

    print([to_msg(a) for a in action.actions])
    msg.actions = [to_msg(a) for a in action.actions]
    return msg


@to_viz_msg.register
def _(action: ActionSequence, marker_ns):
    ma = MarkerArray()
    for ix, a in enumerate(action.actions):
        ma.markers += to_viz_msg(a, marker_ns + f"/{ix}")
    return ma


def from_msg(msg):
    actions = []
    for a in msg.actions:
        match a.action_type:
            case a.FOLLOW:
                actions.append(follow_from_msg(a))
            case a.PICK:
                actions.append(pick_from_msg(a))
            case a.PLACE:
                actions.append(place_from_msg(a))
            case a.GAZE:
                actions.append(gaze_from_msg(a))
            case _:
                raise Exception(f"Received invalid action type {a.action_type}")
    return ActionSequence(
        plan_id=msg.plan_id, robot_name=msg.robot_name, actions=actions
    )


def follow_from_msg(msg):
    return Follow(frame=msg.path.header.frame_id, path2d=path_to_waypoints(msg.path))


@to_msg.register
def _(action: Follow):
    msg = ActionMsg()
    msg.action_type = msg.FOLLOW
    msg.path = waypoints_to_path(action.frame, action.path2d)
    return msg


@to_viz_msg.register
def _(action: Follow, marker_ns):
    points = []
    for p in action.path2d:
        pt = Point()
        pt.x = p[0]
        pt.y = p[1]
        pt.z = 0.0
        points.append(pt)
    m = Marker()
    m.header.frame_id = action.frame
    m.header.stamp = gtm()
    m.ns = marker_ns
    m.id = 0
    m.type = m.LINE_STRIP
    m.action = m.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.1
    m.color.a = 1.0
    m.color.r = 0.0
    m.color.g = 1.0
    m.color.b = 0.0
    m.points = points
    return [m]


def gaze_from_msg(msg):
    robot_point = np.array([msg.robot_point.x, msg.robot_point.y, msg.robot_point.z])
    gaze_point = np.array([msg.gaze_point.x, msg.gaze_point.y, msg.gaze_point.z])
    stow_after = msg.place_frame == "STOW"

    return Gaze(
        frame=msg.gaze_frame,
        robot_point=robot_point,
        gaze_point=gaze_point,
        stow_after=stow_after,
    )


@to_msg.register
def _(action: Gaze):
    msg = ActionMsg()
    msg.action_type = msg.GAZE

    print(action.robot_point[0])
    print(type(action.robot_point[0]))
    msg.robot_point.x = action.robot_point[0]
    msg.robot_point.y = action.robot_point[1]
    msg.robot_point.z = action.robot_point[2]

    msg.gaze_point.x = action.gaze_point[0]
    msg.gaze_point.y = action.gaze_point[1]
    msg.gaze_point.z = action.gaze_point[2]

    msg.place_frame = "STOW" if action.stow_after else "NO_STOW"

    return msg


@to_viz_msg.register
def _(action: Gaze, marker_ns):
    pt1 = Point()
    pt1.x = action.robot_point[0]
    pt1.y = action.robot_point[1]
    pt1.z = action.robot_point[2]
    pt2 = Point()
    pt2.x = action.gaze_point[0]
    pt2.y = action.gaze_point[1]
    pt2.z = action.gaze_point[2]

    m = Marker()
    m.header.frame_id = action.frame
    m.header.stamp = gtm()
    m.ns = marker_ns
    m.id = 0
    m.type = m.ARROW
    m.action = m.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.1  # shaft diameter
    m.scale.y = 0.2  # head diameter
    m.color.a = 1.0
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 0.0
    m.points = [pt1, pt2]

    return [m]


def pick_from_msg(msg):
    robot_point = np.array([msg.robot_point.x, msg.robot_point.y, msg.robot_point.z])
    object_point = np.array(
        [msg.object_point.x, msg.object_point.y, msg.object_point.z]
    )

    return Pick(
        frame=msg.pick_frame,
        object_class=msg.object_class,
        robot_point=robot_point,
        object_point=object_point,
    )


@to_msg.register
def _(action: Pick):
    msg = ActionMsg()
    msg.action_type = msg.PICK

    msg.robot_point.x = action.robot_point[0]
    msg.robot_point.y = action.robot_point[1]
    msg.robot_point.z = action.robot_point[2]

    msg.object_point.x = action.object_point[0]
    msg.object_point.y = action.object_point[1]
    msg.object_point.z = action.object_point[2]

    msg.object_class = action.object_class

    return msg


@to_viz_msg.register
def _(action: Pick, marker_ns):
    pt1 = Point()
    pt1.x = action.robot_point[0]
    pt1.y = action.robot_point[1]
    pt1.z = action.robot_point[2]
    pt2 = Point()
    pt2.x = action.object_point[0]
    pt2.y = action.object_point[1]
    pt2.z = action.object_point[2]

    m = Marker()
    m.header.frame_id = action.frame
    m.header.stamp = gtm()
    m.ns = marker_ns
    m.id = 0
    m.type = m.ARROW
    m.action = m.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.1  # shaft diameter
    m.scale.y = 0.2  # head diameter
    m.color.a = 1.0
    m.color.r = 0.0
    m.color.g = 1.0
    m.color.b = 0.0
    m.points = [pt1, pt2]

    return [m]


def place_from_msg(msg):
    robot_point = np.array([msg.robot_point.x, msg.robot_point.y, msg.robot_point.z])
    object_point = np.array(
        [msg.object_point.x, msg.object_point.y, msg.object_point.z]
    )

    return Place(
        frame=msg.place_frame,
        object_class=msg.object_class,
        robot_point=robot_point,
        object_point=object_point,
    )


@to_msg.register
def _(action: Place):
    msg = ActionMsg()
    msg.action_type = msg.PLACE

    msg.robot_point.x = action.robot_point[0]
    msg.robot_point.y = action.robot_point[1]
    msg.robot_point.z = action.robot_point[2]

    msg.object_point.x = action.object_point[0]
    msg.object_point.y = action.object_point[1]
    msg.object_point.z = action.object_point[2]

    return msg


@to_viz_msg.register
def _(action: Place, marker_ns):
    pt1 = Point()
    pt1.x = action.robot_point[0]
    pt1.y = action.robot_point[1]
    pt1.z = action.robot_point[2]
    pt2 = Point()
    pt2.x = action.object_point[0]
    pt2.y = action.object_point[1]
    pt2.z = action.object_point[2]

    m = Marker()
    m.header.frame_id = action.frame
    m.header.stamp = gtm()
    m.ns = marker_ns
    m.id = 0
    m.type = m.ARROW
    m.action = m.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.1  # shaft diameter
    m.scale.y = 0.2  # head diameter
    m.color.a = 1.0
    m.color.r = 0.0
    m.color.g = 0.0
    m.color.b = 1.0
    m.points = [pt1, pt2]

    return [m]
