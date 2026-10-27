"""ROS action adapter. All waits have wall-clock deadlines, including paused simulation."""
import json
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String
from sort_place_interfaces.action import SortPlace
from sort_place_interfaces.msg import PartArray
from sort_place_interfaces.srv import Suction
from .engine import Engine, Part, CellError, Cancelled
from .kinematics import JOINTS, Pose, trajectory


class TaskServer(Node):
    def __init__(self):
        super().__init__('sort_place_task')
        path = self.declare_parameter('config_file', '').value
        with open(path, encoding='utf-8') as stream:
            self.config = json.load(stream)
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.busy = False
        self.faulted = False
        self.latest_parts, self.latest_joints = None, None
        self.parts_received, self.joints_received = 0.0, 0.0
        self.active_goal = None
        self.status = self.create_publisher(String, '/sort_place/status', 10)
        self.create_subscription(PartArray, '/sort_place/parts', self.on_parts, 10,
                                 callback_group=self.group)
        self.create_subscription(JointState, '/joint_states', self.on_joints,
                                 qos_profile_sensor_data, callback_group=self.group)
        self.arm = ActionClient(self, FollowJointTrajectory,
                               '/arm_controller/follow_joint_trajectory', callback_group=self.group)
        self.gripper = self.create_client(Suction, '/sort_place/suction', callback_group=self.group)
        self.engine = Engine(self.config, self, self.feedback)
        self.server = ActionServer(self, SortPlace, '/sort_place/run',
                                   execute_callback=self.execute, goal_callback=self.accept,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=self.group)

    def on_parts(self, msg):
        if msg.header.frame_id != 'world':
            self.get_logger().error('Rejected parts: expected world frame')
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if age > 1.0 or age < -0.25:
            return
        values = []
        for p in msg.parts:
            q = p.pose.orientation
            if not all(math.isfinite(v) for v in
                       (p.pose.position.x, p.pose.position.y, p.pose.position.z,
                        q.x, q.y, q.z, q.w, p.confidence)):
                return
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            values.append(Part(p.object_id, p.class_id,
                               Pose(p.pose.position.x, p.pose.position.y, p.pose.position.z, yaw),
                               p.confidence, p.attached))
        with self.lock:
            self.latest_parts, self.parts_received = values, time.monotonic()

    def on_joints(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions and math.isfinite(positions[name]) for name in JOINTS):
            with self.lock:
                self.latest_joints = tuple(positions[name] for name in JOINTS)
                self.joints_received = time.monotonic()

    def parts(self):
        with self.lock:
            if self.latest_parts is None or time.monotonic() - self.parts_received > 1.0:
                raise CellError('Part observations unavailable or stale (>1 s)')
            return list(self.latest_parts)

    def joints(self):
        with self.lock:
            if self.latest_joints is None or time.monotonic() - self.joints_received > 1.0:
                raise CellError('Joint states unavailable or stale (>1 s)')
            return self.latest_joints

    def accept(self, _request):
        with self.lock:
            if self.busy or self.faulted:
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def feedback(self, stage, object_id, completed):
        text = json.dumps(dict(stage=stage, object_id=object_id, completed=completed))
        self.status.publish(String(data=text))
        if self.active_goal:
            self.active_goal.publish_feedback(SortPlace.Feedback(
                stage=stage, object_id=object_id, completed=completed))
        self.get_logger().info(text)

    def wait(self, future, timeout, cancelled=lambda: False):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        deadline = time.monotonic() + timeout
        while not done.wait(0.02):
            if cancelled():
                raise Cancelled('Cancellation requested')
            if not rclpy.ok() or time.monotonic() >= deadline:
                raise CellError('ROS operation timed out (wall clock)')
        if future.exception():
            raise CellError(str(future.exception()))
        return future.result()

    def move(self, target, cancelled):
        if not self.arm.wait_for_server(timeout_sec=3.0):
            raise CellError('Arm trajectory action is not ready')
        points = trajectory(self.joints(), target)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for t, q, v, a in points:
            p = JointTrajectoryPoint(positions=list(q), velocities=list(v), accelerations=list(a))
            ns = int(t * 1e9)
            p.time_from_start.sec, p.time_from_start.nanosec = divmod(ns, 1_000_000_000)
            goal.trajectory.points.append(p)
        # Do not abandon an unacknowledged goal on cancellation: get its handle first.
        send = self.arm.send_goal_async(goal)
        try:
            handle = self.wait(send, 5.0)
        except Exception:
            # A delayed acceptance must also be cancelled.
            def cancel_late(future):
                if not future.exception() and future.result().accepted:
                    future.result().cancel_goal_async()
            send.add_done_callback(cancel_late)
            raise
        if not handle.accepted:
            raise CellError('Controller rejected trajectory')
        result_future = handle.get_result_async()
        try:
            result = self.wait(result_future, max(10.0, points[-1][0]*4 + 5), cancelled)
        except Exception:
            try:
                self.wait(handle.cancel_goal_async(), 2.0)
                self.wait(result_future, 2.0)
            except Exception as stop_error:
                self.get_logger().error(f'Controller stop unconfirmed: {stop_error}; inspect simulation')
            raise
        if cancelled():
            raise Cancelled('Cancellation requested at trajectory completion')
        if result.status != 4 or result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise CellError(f'Trajectory failed: {result.result.error_string}')
        measured = self.joints()
        if any(abs(a-b) > tol for a, b, tol in zip(measured, target, (0.02, 0.02, 0.003, 0.02))):
            raise CellError('Controller result disagrees with measured joints')

    def suction(self, object_id, enable):
        if not self.gripper.wait_for_service(timeout_sec=3.0):
            raise CellError('Suction simulation service unavailable')
        result = self.wait(self.gripper.call_async(Suction.Request(
            object_id=object_id, enable=enable)), 3.0)
        if not result.success:
            raise CellError(f'Suction failed: {result.message}')

    def dwell(self, seconds, cancelled):
        end_sim_ns = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        deadline = time.monotonic() + max(3.0, seconds * 6)
        while self.get_clock().now().nanoseconds < end_sim_ns:
            if cancelled():
                raise Cancelled('Cancelled during dwell')
            if time.monotonic() >= deadline:
                raise CellError('Simulation did not advance during dwell')
            time.sleep(0.02)

    def execute(self, handle):
        self.active_goal = handle
        before = len(self.engine.completed)
        result = SortPlace.Result()
        try:
            if not self.get_clock().now().nanoseconds:
                raise CellError('Simulation clock has not started')
            result.completed = self.engine.run(handle.request.max_parts,
                                               lambda: handle.is_cancel_requested)
            if handle.is_cancel_requested:
                raise Cancelled('Cancellation requested after final placement')
            result.success = True
            result.message = 'All selected source parts placed and verified'
            handle.succeed()
        except Cancelled as exc:
            with self.lock:
                self.faulted = True
            result.message = str(exc)
            result.completed = len(self.engine.completed) - before
            handle.canceled()
        except Exception as exc:
            with self.lock:
                self.faulted = True
            result.message = str(exc)
            result.completed = len(self.engine.completed) - before
            self.feedback('FAULT', self.engine.held or '', result.completed)
            handle.abort()
        finally:
            self.active_goal = None
            with self.lock:
                self.busy = False
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TaskServer()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
