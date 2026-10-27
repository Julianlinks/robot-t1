import copy
import json
import math
from pathlib import Path
import random
import sys
import unittest
from dataclasses import replace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/sort_place_core'))
from sort_place_core.engine import Engine, CellError, Cancelled, Part
from sort_place_core.kinematics import (Pose, ik, fk, trajectory, angle_error,
                                       VELOCITIES, ACCELERATIONS)
from sort_place_core.offline_demo import MemoryBackend


class KinematicsTests(unittest.TestCase):
    def test_round_trip_random_valid_joints(self):
        rng = random.Random(42)
        for _ in range(400):
            q = (rng.uniform(-1.5, 1.5), rng.uniform(.2, 2.8),
                 rng.uniform(.01, .25), rng.uniform(-2, 2))
            p = fk(q)
            actual = fk(ik(p))
            self.assertAlmostEqual(p.x, actual.x, places=9)
            self.assertAlmostEqual(p.y, actual.y, places=9)
            self.assertAlmostEqual(p.z, actual.z, places=9)
            self.assertAlmostEqual(angle_error(p.yaw, actual.yaw), 0, places=9)

    def test_unreachable_and_nan_rejected(self):
        for pose in (Pose(2, 0, .9), Pose(.4, 0, .5), Pose(float('nan'), 0, .9),
                     Pose(.6, 0, .9)):
            with self.assertRaises(ValueError):
                ik(pose)

    def test_trajectory_limits_and_endpoint(self):
        start, end = ik(Pose(.32, -.21, .90, .15)), ik(Pose(.46, .22, .76, 0))
        values = trajectory(start, end)
        self.assertTrue(all(a[0] < b[0] for a, b in zip(values, values[1:])))
        for _, q, velocity, acceleration in values:
            for v, limit in zip(velocity, VELOCITIES):
                self.assertLessEqual(abs(v), limit + 1e-9)
            for a, limit in zip(acceleration, ACCELERATIONS):
                self.assertLessEqual(abs(a), limit + 1e-9)
        for a, b in zip(values[-1][1], end):
            self.assertAlmostEqual(a, b)
        self.assertTrue(all(abs(v) < 1e-8 for v in values[-1][2]))


class EngineTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((ROOT / 'src/sort_place_sim/config/cell.json').read_text())
        self.backend = MemoryBackend(self.config)
        self.engine = Engine(self.config, self.backend)

    def test_six_parts_and_no_duplicate_processing(self):
        self.assertEqual(self.engine.run(), 6)
        self.assertEqual(len(self.engine.occupied), 6)
        self.assertEqual(self.engine.run(), 0)
        self.assertIsNone(self.backend.held)

    def test_partial_batch_and_resume(self):
        self.assertEqual(self.engine.run(2), 2)
        self.assertEqual(self.engine.run(), 4)

    def test_cancel_during_transport_keeps_part_and_reserves_slot(self):
        self.engine.feedback = lambda stage, obj, count: setattr(self, 'stage', stage)
        self.stage = ''
        def cancelled():
            return self.stage == 'LIFT'
        with self.assertRaises(Cancelled):
            self.engine.run(cancelled=cancelled)
        self.assertIsNotNone(self.engine.held)
        self.assertIsNotNone(self.backend.held)
        self.assertEqual(len(self.engine.quarantined), 1)
        with self.assertRaises(CellError):
            self.engine.run()

    def test_bad_release_does_not_commit(self):
        original = self.backend.suction
        def suction(object_id, enable):
            original(object_id, enable)
            if not enable:
                p = self.backend.objects[object_id]
                self.backend.objects[object_id] = replace(p, pose=replace(p.pose, x=p.pose.x+.02))
        self.backend.suction = suction
        with self.assertRaisesRegex(CellError, 'verification failed'):
            self.engine.run()
        self.assertFalse(self.engine.completed)
        self.assertEqual(len(self.engine.quarantined), 1)

    def test_failed_physical_attachment_is_detected(self):
        original = self.backend.move
        def move(q, cancel):
            held = self.backend.held
            self.backend.held = None
            original(q, cancel)
            self.backend.held = held
        self.backend.move = move
        with self.assertRaisesRegex(CellError, 'did not follow'):
            self.engine.run()
        self.assertFalse(self.engine.completed)

    def test_no_slot_stops_before_any_motion(self):
        self.engine.occupied = {s.slot_id for s in self.engine.slots}
        with self.assertRaisesRegex(CellError, 'No free slot'):
            self.engine.run()
        self.assertEqual(self.backend.moves, 0)

    def test_low_confidence_skipped(self):
        for key, p in list(self.backend.objects.items()):
            self.backend.objects[key] = replace(p, confidence=.2)
        self.assertEqual(self.engine.run(), 0)

    def test_restart_reconstructs_occupancy(self):
        self.assertEqual(self.engine.run(2), 2)
        restarted = Engine(self.config, self.backend)
        self.assertEqual(restarted.run(), 4)
        self.assertEqual(len(restarted.occupied), 6)

    def test_unreachable_pick_fails_before_motion(self):
        p = next(iter(self.backend.objects.values()))
        self.backend.objects = {p.object_id: replace(p, pose=Pose(.58, -.2, .754))}
        self.config['source_x'] = [.2, .7]
        with self.assertRaises(ValueError):
            self.engine.run()
        self.assertEqual(self.backend.moves, 0)

    def test_lost_suction_reply_reserves_destination(self):
        original = self.backend.suction
        def suction(object_id, enable):
            original(object_id, enable)
            raise CellError('Response lost')
        self.backend.suction = suction
        with self.assertRaises(CellError):
            self.engine.run()
        self.assertEqual(len(self.engine.quarantined), 1)
        self.assertIsNotNone(self.backend.held)
        with self.assertRaisesRegex(CellError, 'already attached'):
            self.engine.run()


if __name__ == '__main__':
    unittest.main()
