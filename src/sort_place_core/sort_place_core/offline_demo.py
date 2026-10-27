"""Logic-only backend; deliberately not a Gazebo/physics substitute."""
import argparse
import json
from dataclasses import replace
from .engine import Engine, Part
from .kinematics import Pose, ik, fk


class MemoryBackend:
    def __init__(self, config):
        self.config = config
        self.q = ik(Pose(0.44, 0.0, config['safe_z']))
        self.objects = {p['id']: Part(p['id'], p['class_id'], Pose(*p['pose']))
                        for p in config['parts']}
        self.held = None
        self.offset = None
        self.moves = 0

    def parts(self):
        return list(self.objects.values())

    def joints(self):
        return self.q

    def move(self, target, cancelled):
        self.q = target
        self.moves += 1
        if self.held:
            tcp = fk(target)
            self.objects[self.held] = replace(self.objects[self.held],
                pose=Pose(tcp.x, tcp.y, tcp.z - self.offset, tcp.yaw))

    def suction(self, object_id, enable):
        p = self.objects[object_id]
        if enable:
            self.held = object_id
            self.offset = fk(self.q).z - p.pose.z
            self.objects[object_id] = replace(p, attached=True)
        else:
            self.objects[object_id] = replace(p, attached=False)
            self.held = None

    def dwell(self, seconds, cancelled):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    with open(args.config, encoding='utf-8') as stream:
        config = json.load(stream)
    backend = MemoryBackend(config)
    engine = Engine(config, backend,
                    lambda stage, obj, n: print(f'{stage:16} {obj:16} completed={n}'))
    count = engine.run()
    print(json.dumps(dict(completed=count, occupied=sorted(engine.occupied),
                          motion_segments=backend.moves), indent=2))


if __name__ == '__main__':
    main()
