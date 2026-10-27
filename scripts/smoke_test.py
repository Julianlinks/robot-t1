#!/usr/bin/env python3
"""Ubuntu/ROS-only end-to-end check. Owns a fresh headless launch and stops it afterward."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sort_place_interfaces.action import SortPlace
from sort_place_interfaces.msg import PartArray

ROOT = Path(__file__).resolve().parents[1]


def until(node, predicate, timeout, description):
    end = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= end:
            raise RuntimeError(f'Timeout: {description}')
        rclpy.spin_once(node, timeout_sec=0.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=240)
    args = parser.parse_args()
    (ROOT / 'work').mkdir(exist_ok=True)
    log_path = ROOT / 'work/gazebo_smoke.log'
    with log_path.open('w', encoding='utf-8') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', 'sort_place_sim', 'cell.launch.py',
             'rviz:=false', 'gz_args:=-s -r -v 3'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Node('sort_place_smoke_test')
        observations = []
        def on_parts(msg):
            observations[:] = [msg]
        subscription = node.create_subscription(PartArray, '/sort_place/parts', on_parts, 10)
        client = ActionClient(node, SortPlace, '/sort_place/run')
        try:
            until(node, lambda: client.server_is_ready() or process.poll() is not None,
                  100, 'action server startup')
            if process.poll() is not None:
                raise RuntimeError(f'Launch exited early; inspect {log_path}')
            until(node, lambda: bool(observations), 10, 'part observations')
            if len(observations[-1].parts) != 6:
                raise RuntimeError('Expected the unmodified six-part scene')
            sent = client.send_goal_async(SortPlace.Goal(max_parts=0))
            until(node, sent.done, 5, 'goal response')
            handle = sent.result()
            if not handle.accepted:
                raise RuntimeError('Batch rejected')
            result_future = handle.get_result_async()
            until(node, result_future.done, args.timeout, 'batch result')
            response = result_future.result()
            if response.status != 4 or not response.result.success or response.result.completed != 6:
                raise RuntimeError(f'Batch failed: {response.result.message}')
            until(node, lambda: bool(observations) and not any(p.attached for p in observations[-1].parts),
                  5, 'all parts detached')
            config = json.loads((ROOT / 'src/sort_place_sim/config/cell.json').read_text())
            remaining = list(config['slots'])
            errors = {}
            for p in observations[-1].parts:
                xyz = (p.pose.position.x, p.pose.position.y, p.pose.position.z)
                candidates = [s for s in remaining if s['class_id'] == p.class_id]
                if not candidates:
                    raise RuntimeError(f'No matching slot for {p.object_id}')
                slot = min(candidates, key=lambda s: math.dist(xyz, s['pose'][:3]))
                error = math.dist(xyz, slot['pose'][:3])
                if error > config['position_tolerance']:
                    raise RuntimeError(f'{p.object_id}: final error {error} m')
                q = p.pose.orientation
                yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                yaw_error = abs(math.atan2(math.sin(yaw-slot['pose'][3]), math.cos(yaw-slot['pose'][3])))
                if yaw_error > config['yaw_tolerance']:
                    raise RuntimeError(f'{p.object_id}: final yaw error {yaw_error} rad')
                remaining.remove(slot)
                errors[p.object_id] = {'slot': slot['id'], 'position_error_m': error,
                                       'yaw_error_rad': yaw_error}
            if remaining:
                raise RuntimeError('Some slots remain unfilled')
            report = {'status': 'PASS', 'completed': 6, 'physical_pose_checks': errors}
            # Second action must not process already-placed objects again.
            sent_again = client.send_goal_async(SortPlace.Goal(max_parts=0))
            until(node, sent_again.done, 5, 'repeat goal response')
            repeated = sent_again.result()
            if not repeated.accepted:
                raise RuntimeError('Repeat request rejected')
            finished = repeated.get_result_async()
            until(node, finished.done, 10, 'repeat batch')
            if not finished.result().result.success or finished.result().result.completed != 0:
                raise RuntimeError('Placed objects were processed again')
            report_path = ROOT / 'work/gazebo_smoke_result.json'
            report_path.write_text(json.dumps(report, indent=2), encoding='utf-8')
            print(json.dumps(report, indent=2))
            print('PASS: physical placement and repeat-run checks')
        finally:
            node.destroy_node()
            rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10)


if __name__ == '__main__':
    main()
