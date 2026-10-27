"""Regenerate the SDF world from config/cell.json. Run after changing parts/slots."""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / 'src/sort_place_sim'
COLORS = {'red': '0.9 0.12 0.10 1', 'green': '0.1 0.75 0.25 1', 'blue': '0.1 0.35 0.95 1'}


def box(name, pose, size, color, static=True, mass=1.0):
    inertia = [mass*(size[1]**2 + size[2]**2)/12,
               mass*(size[0]**2 + size[2]**2)/12,
               mass*(size[0]**2 + size[1]**2)/12]
    geometry = f'<geometry><box><size>{" ".join(map(str, size))}</size></box></geometry>'
    return f'''<model name="{name}"><static>{str(static).lower()}</static>
      <pose>{pose[0]} {pose[1]} {pose[2]} 0 0 {pose[3]}</pose>
      <link name="link"><inertial><mass>{mass}</mass><inertia>
        <ixx>{inertia[0]}</ixx><iyy>{inertia[1]}</iyy><izz>{inertia[2]}</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="collision">{geometry}<surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision>
        <visual name="visual">{geometry}<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
      </link></model>'''


def generate():
    c = json.loads((SIM / 'config/cell.json').read_text(encoding='utf-8'))
    objects = [box('table', (0.39, 0, 0.375, 0), (0.62, 0.80, 0.75), '0.32 0.34 0.38 1')]
    for p in c['parts']:
        objects.append(box(p['id'], p['pose'], (0.026, 0.018, 0.008),
                           COLORS[p['class_id']], static=False, mass=0.005))
    # Thin non-colliding target outlines lie directly on the table surface.
    for s in c['slots']:
        x, y, _, yaw = s['pose']
        bars = []
        for dx, dy, sx, sy in ((0, -.012, .034, .002), (0, .012, .034, .002),
                                (-.017, 0, .002, .024), (.017, 0, .002, .024)):
            bars.append(f'<visual name="v{len(bars)}"><pose>{dx} {dy} 0 0 0 0</pose>'
                        f'<geometry><box><size>{sx} {sy} 0.0003</size></box></geometry>'
                        f'<material><ambient>{COLORS[s["class_id"]]}</ambient>'
                        f'<diffuse>{COLORS[s["class_id"]]}</diffuse></material></visual>')
        objects.append(f'<model name="slot_{s["id"]}"><static>true</static>'
                       f'<pose>{x} {y} 0.7502 0 0 {yaw}</pose><link name="link">'
                       + ''.join(bars) + '</link></model>')
    world = '''<?xml version="1.0"?>
<sdf version="1.9"><world name="sort_cell">
  <physics name="physics" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
  <gravity>0 0 -9.81</gravity>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="libcell_vacuum.so" name="sort_place::CellVacuum"/>
  <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.12 0.15 0.20 1</background><shadows>true</shadows></scene>
  <light name="sun" type="directional"><pose>0 0 3 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.4 0.2 -1</direction><cast_shadows>true</cast_shadows></light>
'''+ '\n'.join(objects) + '\n</world></sdf>\n'
    ET.fromstring(world)
    (SIM / 'worlds').mkdir(exist_ok=True)
    (SIM / 'worlds/cell.sdf').write_text(world, encoding='utf-8')


if __name__ == '__main__':
    generate()
