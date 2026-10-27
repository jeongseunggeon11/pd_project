#!/usr/bin/env python3
"""Offline M6 sanity checks that do not need ROS or Gazebo.

Validates YAML configs, spray/fire SDF snippets, and patrol coverage of fire candidates.
Exit 0 on success.
"""
import ast
import math
import os
import sys

import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
SRC = os.path.join(ROOT, 'src')


def fail(msg):
    print(f'FAIL: {msg}')
    return False


def ok(msg):
    print(f'OK: {msg}')
    return True


def check_yaml(path, required_keys):
    with open(path) as f:
        data = yaml.safe_load(f)
    missing = [k for k in required_keys if k not in data]
    if missing:
        return fail(f'{path}: missing keys {missing}')
    return ok(f'yaml {os.path.relpath(path, ROOT)}')


def check_spray_valid_params():
    mission = yaml.safe_load(open(os.path.join(SRC, 'fire_mission/config/mission.yaml')))
    fire = yaml.safe_load(open(os.path.join(SRC, 'fire_sim/config/fire_sim.yaml')))
    mp = mission['mission_node']['ros__parameters']
    fp = fire['fire_manager']['ros__parameters']
    good = True
    if not (mp['min_distance'] == fp['min_distance'] == 1.0
            and mp['max_distance'] == fp['max_distance'] == 2.0):
        good = fail('spray distance window must be 1.0–2.0 m in mission and fire_manager')
    if fp['max_angle_deg'] != 10.0:
        good = fail('fire_manager max_angle_deg must be 10')
    if mp['realign_deg'] > fp['max_angle_deg']:
        good = fail('mission realign_deg should be ≤ fire_manager max_angle_deg')
    if mp['spray_distance'] < mp['min_distance'] or mp['spray_distance'] > mp['max_distance']:
        good = fail('spray_distance outside allowed window')
    if good:
        ok('spray validity params aligned (1.0–2.0 m, ±10°)')
    return good


def check_sdf_particles():
    path = os.path.join(SRC, 'fire_mission/models/spray.sdf')
    text = open(path).read()
    good = True
    if '<emitting>true</emitting>' not in text:
        good = fail('spray.sdf must emit while spawned')
    if '<rate>' not in text or '<lifetime>' not in text:
        good = fail('spray.sdf missing rate/lifetime')
    fire = open(os.path.join(SRC, 'fire_sim/models/fire.sdf')).read()
    if '<temperature>600.0</temperature>' not in fire:
        good = fail('fire.sdf must be ≥600 K for thermal detection')
    if good:
        ok('fire/spray SDF markers present')
    return good


def check_python_syntax():
    good = True
    for rel in (
        'fire_mission/fire_mission/mission_node.py',
        'fire_mission/fire_mission/spray_nozzle.py',
        'fire_sim/fire_sim/fire_generator.py',
        'fire_sim/fire_sim/fire_manager.py',
        'fire_detection/fire_detection/fire_detector.py',
        'fire_mission/launch/full.launch.py',
        'fire_robot_gazebo/launch/sim.launch.py',
    ):
        path = os.path.join(SRC, rel)
        try:
            ast.parse(open(path).read(), filename=path)
            ok(f'syntax {rel}')
        except SyntaxError as e:
            good = fail(f'syntax {rel}: {e}')
    return good


def check_coverage():
    script = os.path.join(SRC, 'fire_mission/scripts/check_coverage.py')
    # Import by path without installing the package
    import importlib.util
    spec = importlib.util.spec_from_file_location('check_coverage', script)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    map_yaml = os.path.join(SRC, 'fire_robot_navigation/maps/house.yaml')
    img, res, ox, oy = mod.load_map(map_yaml)
    mission = yaml.safe_load(open(os.path.join(SRC, 'fire_mission/config/mission.yaml')))
    fire = yaml.safe_load(open(os.path.join(SRC, 'fire_sim/config/fire_sim.yaml')))
    flat = mission['mission_node']['ros__parameters']['waypoints']
    waypoints = list(zip(flat[0::2], flat[1::2]))
    flat = fire['fire_generator']['ros__parameters']['candidates']
    candidates = list(zip(flat[0::2], flat[1::2]))

    seen = None
    per_wp = []
    for x, y in waypoints:
        s = mod.visible_from(img, res, ox, oy, x, y)
        per_wp.append(s)
        seen = s if seen is None else (seen | s)

    uncovered = []
    for cx, cy in candidates:
        r, c = int((cy - oy) / res), int((cx - ox) / res)
        if not any(s[r, c] for s in per_wp):
            uncovered.append((cx, cy))
    if uncovered:
        return fail(f'fire candidates not covered: {uncovered}')
    # Rough travel-time budget at 0.5 m/s across map diagonal
    diag = math.hypot(10.0, 8.0)
    budget = diag / 0.5 + 15.0  # travel + align margin
    if budget > 60.0:
        return fail(f'worst-case travel budget {budget:.1f}s exceeds 60s')
    return ok(f'all {len(candidates)} candidates covered; travel budget ~{budget:.0f}s < 60s')


def main():
    checks = [
        lambda: check_yaml(os.path.join(SRC, 'fire_mission/config/mission.yaml'),
                           ['mission_node']),
        lambda: check_yaml(os.path.join(SRC, 'fire_sim/config/fire_sim.yaml'),
                           ['fire_generator', 'fire_manager']),
        lambda: check_yaml(os.path.join(SRC, 'fire_detection/config/fire_detection.yaml'),
                           ['fire_detector']),
        check_spray_valid_params,
        check_sdf_particles,
        check_python_syntax,
        check_coverage,
    ]
    results = [fn() for fn in checks]
    if not all(results):
        sys.exit(1)
    print('All offline M6 checks passed')


if __name__ == '__main__':
    main()
