#!/usr/bin/env python3
"""
generate_mission.py -- generate a world YAML + mission YAML pair from a spec file.

Writes (single run):
    config/<name>.yaml          drone initial positions
    missions/<name>.yaml        mission spec consumed by send_mission.py / mission_executor.py
    <name>_experiments.yaml     ready-to-use input for run_experiments.py

Sweep mode (spec contains a `sweep:` block):
    One set of files per parameter combination, plus:
    <name>_sweep_experiments.yaml   ready-to-use input for run_experiments.py

Usage
-----
    python3 generate_mission.py my_spec.yaml
    python3 generate_mission.py my_spec.yaml --dry-run
    python3 generate_mission.py my_spec.yaml --run          # generate + run immediately
    python3 generate_mission.py my_spec.yaml --run --stack-wait 60

Spec format  (see generate_mission.yaml.example)
-----------
name: my_run

arena:
  half: 10.0           # spans [-half, half] on both axes

drones:
  count: 5
  start:
    strategy: left_edge   # 'left_edge' | 'custom'
    x: -10.0

areas:
  layout: grid_areas    # 'grid_areas' | 'strip_areas' | 'custom'
  rows: 2
  cols: 3
  prefix: area

mission:
  takeoff_height: 1.0

output:
  world_dir:   config/
  mission_dir: missions/

world (optional):
  output_dir:     assets/worlds/
  street_spacing: 1.0    # metres between coverage streets
  wp_space:       1.0    # metres between waypoints along each street
  height:         5.0    # coverage flight height
  speed:          2.0    # coverage flight speed (m/s)

experiment (optional):
  mode:    centralized   # 'centralized' | 'decentralized'
  times:   1             # repetitions; comparison table shows mean±std
  failure:               # failure injection shorthand
    drone: drone0
    at_waypoint: 3       # centralized
    # delay: 20.0        # decentralized
"""

__authors__ = 'Guillermo GP-Lenza'
__license__ = 'BSD-3-Clause'

import argparse
import copy
import subprocess
import sys
from itertools import product
from pathlib import Path
from string import ascii_uppercase
from typing import List, Optional, Tuple

import jinja2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).parent))
from swarm_pylib.back_and_force import BackAndForce

SCRIPT_DIR = Path(__file__).parent
TEMPLATE_DIR = SCRIPT_DIR / 'assets' / 'templates'


def _slug(name: str) -> str:
    """Convert a display name to a filesystem-safe slug (spaces → underscores)."""
    import re
    return re.sub(r'[^\w\-.]', '_', name)

GPS_ORIGIN = {'latitude': 40.4405287, 'longitude': -3.6898277, 'altitude': 100.0}


# ---------------------------------------------------------------------------
# World SDF generation
# ---------------------------------------------------------------------------

def waypoints_for_areas(areas: List[dict], height: float,
                        street_spacing: float, wp_space: float) -> List[List[float]]:
    """Run the back-and-force coverage planner on every area and return all waypoints."""
    all_waypoints: List[List[float]] = []
    for area in areas:
        verts = [[v['x'], v['y'], height] for v in area['vertices']]
        centroid_x = sum(v[0] for v in verts) / len(verts)
        centroid_y = sum(v[1] for v in verts) / len(verts)
        start = [[centroid_x, centroid_y, height]]
        wps = BackAndForce.generate_path(
            initial_position_list=start,
            last_position_list=start,
            area_values=verts,
            street_spacing=street_spacing,
            waypoints_spacing=wp_space,
        )
        all_waypoints.extend(wps)
    return all_waypoints


def render_world_sdf(world_name: str, waypoints: List[List[float]],
                     gps_origin: dict) -> str:
    """Render the world.sdf.jinja template with a solar panel at each waypoint."""
    models = {}
    for i, wp in enumerate(waypoints):
        models[f'panel_{i:04d}'] = [round(wp[0], 3), round(wp[1], 3), 0.0, 0.0, 0.0, 0.0]

    env = jinja2.Environment(loader=jinja2.FileSystemLoader(str(TEMPLATE_DIR)))
    tmpl = env.get_template('world.sdf.jinja')
    return tmpl.render(world_name=world_name, origin=gps_origin, models=models)


# ---------------------------------------------------------------------------
# Area generators
# Each returns a list of area dicts:
#   {'name': str, 'vertices': [{'x': float, 'y': float}, ...]}
# ---------------------------------------------------------------------------

def generate_grid_areas(rows: int, cols: int,
                        arena_half: float, prefix: str) -> List[dict]:
    """Divide the arena into a rows×cols grid of equal rectangles."""
    w = (2 * arena_half) / cols
    h = (2 * arena_half) / rows
    areas = []
    for r in range(rows):
        row_letter = ascii_uppercase[r % 26]
        for c in range(cols):
            x0 = -arena_half + c * w
            y0 =  arena_half - (r + 1) * h
            x1 = x0 + w
            y1 = y0 + h
            areas.append({
                'name': f'{prefix}_{row_letter}{c + 1}',
                'vertices': [
                    {'x': round(x0, 3), 'y': round(y0, 3)},
                    {'x': round(x1, 3), 'y': round(y0, 3)},
                    {'x': round(x1, 3), 'y': round(y1, 3)},
                    {'x': round(x0, 3), 'y': round(y1, 3)},
                ],
            })
    return areas


def generate_strip_areas(count: int, axis: str,
                         arena_half: float, prefix: str) -> List[dict]:
    """Divide the arena into `count` equal parallel strips.

    axis: 'x' → vertical strips (constant x-width)
          'y' → horizontal strips (constant y-height)
    """
    areas = []
    step = (2 * arena_half) / count
    for i in range(count):
        name = f'{prefix}_{i + 1:02d}'
        if axis == 'x':
            x0 = round(-arena_half + i * step, 3)
            x1 = round(x0 + step, 3)
            areas.append({'name': name, 'vertices': [
                {'x': x0, 'y': -arena_half},
                {'x': x1, 'y': -arena_half},
                {'x': x1, 'y':  arena_half},
                {'x': x0, 'y':  arena_half},
            ]})
        else:
            y1 = round( arena_half - i * step, 3)
            y0 = round(y1 - step, 3)
            areas.append({'name': name, 'vertices': [
                {'x': -arena_half, 'y': y0},
                {'x':  arena_half, 'y': y0},
                {'x':  arena_half, 'y': y1},
                {'x': -arena_half, 'y': y1},
            ]})
    return areas


def generate_custom_areas(polygons: list) -> List[dict]:
    """Pass-through for user-defined polygon list from the spec."""
    areas = []
    for p in polygons:
        areas.append({
            'name': p['name'],
            'vertices': [{'x': float(v['x']), 'y': float(v['y'])}
                         for v in p['vertices']],
        })
    return areas


# ---------------------------------------------------------------------------
# Drone position generators
# ---------------------------------------------------------------------------

def place_left_edge(n: int, arena_half: float, x: float) -> List[Tuple[float, float]]:
    if n == 1:
        return [(x, 0.0)]
    step = (2 * arena_half) / (n - 1)
    return [(x, round(arena_half - i * step, 3)) for i in range(n)]


# ---------------------------------------------------------------------------
# File writers
# ---------------------------------------------------------------------------

def build_world_yaml(drone_names: List[str],
                     positions: List[Tuple[float, float]],
                     gps_origin: dict) -> str:
    lines = ['/**:',
             '  platform:',
             '    ros__parameters:',
             '      gps_origin:',
             f"        latitude: {gps_origin['latitude']}",
             f"        longitude: {gps_origin['longitude']}",
             f"        altitude: {gps_origin['altitude']}",
             '']
    for name, (x, y) in zip(drone_names, positions):
        lines += [
            f'{name}:',
            '  platform:',
            '    ros__parameters:',
            '      vehicle_initial_pose:',
            f'        x: {x}',
            f'        y: {y}',
            '        z: 0.0',
            '',
        ]
    return '\n'.join(lines)


def build_mission_yaml(name: str,
                       drone_names: List[str],
                       areas: List[dict],
                       takeoff_height: float,
                       coverage_height: float,
                       coverage_speed: float,
                       street_spacing: float,
                       wp_space: float) -> dict:
    """Mission YAML consumed by send_mission.py / mission_executor.py.

    One Area layer per inspection polygon, one LandPoint layer per drone.
    """
    layers = []
    for area in areas:
        layers.append({
            'name': 'Area',
            'uavList': ['auto'],
            'height': coverage_height,
            'speed': coverage_speed,
            'algorithm': 'back_and_force',
            'streetSpacing': street_spacing,
            'wpSpace': wp_space,
            'orientation': 0.0,
            'values': [[v['x'], v['y']] for v in area['vertices']],
        })
    for drone in drone_names:
        layers.append({
            'name': 'LandPoint',
            'uavList': [drone],
            'height': 0.0,
            'speed': 1.0,
            'values': [0.0, 0.0],
        })
    return {
        'status': 'request',
        'id': name,
        'use_cartesian_coordinates': True,
        'takeoff_height': takeoff_height,
        'max_wp_distance': 10.0,
        'uavList': drone_names,
        'layers': layers,
    }


# ---------------------------------------------------------------------------
# Sweep helper
# ---------------------------------------------------------------------------

def _set_nested(d: dict, path: str, value) -> dict:
    d = copy.deepcopy(d)
    keys = path.split('.')
    node = d
    for k in keys[:-1]:
        node = node[k]
    node[keys[-1]] = value
    return d


# ---------------------------------------------------------------------------
# Core generator
# ---------------------------------------------------------------------------

def _generate_from_spec(spec: dict, dry_run: bool = False) -> Optional[dict]:
    name        = spec['name']
    arena_half  = float(spec.get('arena', {}).get('half', 10.0))
    n_drones    = int(spec['drones']['count'])
    drone_names = [f'drone{i}' for i in range(n_drones)]

    # -- drone positions -------------------------------------------------------
    start_cfg = spec['drones'].get('start', {})
    strategy  = start_cfg.get('strategy', 'left_edge')
    if strategy == 'left_edge':
        positions = place_left_edge(n_drones, arena_half,
                                    float(start_cfg.get('x', -(arena_half + 1))))
    elif strategy == 'custom':
        raw = start_cfg['positions']
        positions = [(float(p['x']), float(p['y'])) for p in raw]
        if len(positions) != n_drones:
            print(f'[gen] ERROR: {len(positions)} custom positions for {n_drones} drones')
            return None
    else:
        print(f'[gen] unknown drone start strategy: {strategy}')
        return None

    # -- inspection areas ------------------------------------------------------
    area_cfg = spec['areas']
    layout   = area_cfg.get('layout', 'grid_areas')
    prefix   = area_cfg.get('prefix', 'area')

    if layout == 'grid_areas':
        areas = generate_grid_areas(
            rows=int(area_cfg['rows']), cols=int(area_cfg['cols']),
            arena_half=arena_half, prefix=prefix,
        )
    elif layout == 'strip_areas':
        areas = generate_strip_areas(
            count=int(area_cfg['count']),
            axis=area_cfg.get('axis', 'y'),
            arena_half=arena_half, prefix=prefix,
        )
    elif layout == 'custom':
        areas = generate_custom_areas(area_cfg['polygons'])
    else:
        print(f'[gen] unknown area layout: {layout}')
        return None

    # -- mission config --------------------------------------------------------
    m_cfg          = spec.get('mission', {})
    takeoff_height = float(m_cfg.get('takeoff_height', 1.0))

    # -- coverage params (from world section; used in mission YAML and optionally SDF) --
    world_cfg      = spec.get('world', {})
    street_spacing = float(world_cfg.get('street_spacing', 1.0))
    wp_space       = float(world_cfg.get('wp_space',       1.0))
    cov_height     = float(world_cfg.get('height',         5.0))
    cov_speed      = float(world_cfg.get('speed',          2.0))

    # -- paths -----------------------------------------------------------------
    slug         = _slug(name)
    out_cfg      = spec.get('output', {})
    world_dir    = SCRIPT_DIR / out_cfg.get('world_dir',   'config')
    missions_root = SCRIPT_DIR / out_cfg.get('mission_dir', 'missions')
    mission_subdir = missions_root / _slug(spec.get('_subdir', name))
    world_path    = world_dir    / f'{slug}.yaml'
    mission_path  = mission_subdir / f'{slug}.yaml'

    world_text   = build_world_yaml(drone_names, positions, GPS_ORIGIN)
    mission_dict = build_mission_yaml(name, drone_names, areas,
                                      takeoff_height, cov_height, cov_speed,
                                      street_spacing, wp_space)

    # -- world SDF with solar panels at coverage waypoints ----------------------
    sdf_text = None
    sdf_path = None
    if world_cfg.get('output_dir'):
        sdf_dir  = SCRIPT_DIR / world_cfg['output_dir']
        sdf_path = sdf_dir / f'{slug}.sdf'

        waypoints = waypoints_for_areas(areas, cov_height, street_spacing, wp_space)
        sdf_text  = render_world_sdf(name, waypoints, GPS_ORIGIN)

    if dry_run:
        print(f'=== {world_path} ===\n{world_text}')
        print(f'=== {mission_path} ===')
        print(yaml.dump(mission_dict, default_flow_style=False, sort_keys=False))
        if sdf_text:
            print(f'=== {sdf_path} ===\n{sdf_text[:500]}...')
        return None

    world_path.write_text(world_text)
    mission_subdir.mkdir(parents=True, exist_ok=True)
    mission_path.write_text(yaml.dump(mission_dict, default_flow_style=False, sort_keys=False))
    if sdf_text and sdf_path:
        sdf_path.parent.mkdir(parents=True, exist_ok=True)
        sdf_path.write_text(sdf_text)
        print(f'[gen] {name}: {n_drones} drones, {len(areas)} areas, '
              f'{len(waypoints)} panel waypoints → {sdf_path}')
    else:
        print(f'[gen] {name}: {n_drones} drones, {len(areas)} inspection areas ({layout})')

    return {'name': name, 'world_path': world_path, 'mission_path': mission_path,
            'mission_subdir': mission_subdir, 'sdf_path': sdf_path}


def _build_run_entry(meta: dict, exp_cfg: dict, name_override: Optional[str] = None) -> dict:
    """Build one experiments.yaml run entry from generation metadata + experiment config."""
    entry = {
        'name':        name_override or meta['name'],
        'world_file':  str(Path(meta['world_path']).relative_to(SCRIPT_DIR)),
        'mission_dir': str(Path(meta['mission_subdir']).relative_to(SCRIPT_DIR)),
    }
    modes = exp_cfg.get('modes') or ([exp_cfg['mode']] if exp_cfg.get('mode') else [])
    if len(modes) > 1:
        entry['modes'] = modes
    elif modes:
        entry['mode'] = modes[0]
    if exp_cfg.get('times', 1) > 1:
        entry['times'] = exp_cfg['times']
    if exp_cfg.get('failure'):
        entry['failure'] = exp_cfg['failure']
    if exp_cfg.get('reinspect'):
        entry['reinspect'] = exp_cfg['reinspect']
    return entry


def generate(spec_path: Path, dry_run: bool = False) -> Optional[Path]:
    """Generate files for a single spec. Returns path to the written experiments YAML."""
    spec = yaml.safe_load(spec_path.read_text())
    if 'sweep' in spec:
        print('[gen] sweep key detected — use generate_sweep or remove the sweep block')
        sys.exit(1)
    meta = _generate_from_spec(spec, dry_run)
    if dry_run or not meta:
        return None

    exp_cfg  = spec.get('experiment', {})
    exp_path = SCRIPT_DIR / f'{_slug(spec["name"])}_experiments.yaml'
    entry    = _build_run_entry(meta, exp_cfg)
    exp_path.write_text(yaml.dump({'runs': [entry]}, default_flow_style=False, sort_keys=False))
    print(f'[gen] wrote experiments -> {exp_path}')
    return exp_path


def generate_sweep(spec_path: Path, dry_run: bool = False) -> Optional[Path]:
    """Generate files for all sweep variants. Returns path to the written experiments YAML."""
    spec       = yaml.safe_load(spec_path.read_text())
    sweep_axes = spec.pop('sweep')
    base_name  = spec['name']
    exp_cfg    = spec.get('experiment', {})

    axes         = [(s['param'], s['values']) for s in sweep_axes]
    combinations = list(product(*[vals for _, vals in axes]))
    print(f'[gen] sweep: {len(combinations)} variant(s) across {[p for p, _ in axes]}')

    generated = []
    for combo in combinations:
        variant = copy.deepcopy(spec)
        suffix_parts = []
        for (param, _), value in zip(axes, combo):
            variant      = _set_nested(variant, param, value)
            suffix_parts.append(f'{param.split(".")[-1]}{value}')
        variant['name']    = f'{base_name}_{"_".join(suffix_parts)}'
        variant['_subdir'] = base_name  # all variants share one directory
        meta = _generate_from_spec(variant, dry_run)
        if meta:
            generated.append(meta)

    if dry_run or not generated:
        return None

    # One consolidated run entry pointing at the shared mission directory.
    # world_file is taken from the first variant (limitation: varies if sweep
    # includes drones.count or other world-affecting parameters).
    exp_path = SCRIPT_DIR / f'{_slug(base_name)}_sweep_experiments.yaml'
    entry    = _build_run_entry(generated[0], exp_cfg, name_override=base_name)
    exp_path.write_text(yaml.dump({'runs': [entry]}, default_flow_style=False, sort_keys=False))
    print(f'[gen] wrote sweep experiments ({len(generated)} variants in {entry["mission_dir"]}) -> {exp_path}')
    return exp_path


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description='Generate world YAML + mission YAML from a spec')
    parser.add_argument('spec', type=Path, help='Path to generator spec YAML')
    parser.add_argument('--dry-run', action='store_true',
                        help='Print output to stdout instead of writing files')
    parser.add_argument('--run', action='store_true',
                        help='After generation, immediately execute via run_experiments.py')
    parser.add_argument('--stack-wait', type=float, default=40.0,
                        help='Seconds to wait for the AS2 stack (passed to run_experiments.py)')
    args = parser.parse_args()

    if not args.spec.exists():
        print(f'[gen] spec file not found: {args.spec}')
        sys.exit(1)

    raw = yaml.safe_load(args.spec.read_text())
    if 'sweep' in raw:
        exp_path = generate_sweep(args.spec, dry_run=args.dry_run)
    else:
        exp_path = generate(args.spec, dry_run=args.dry_run)

    if args.run and exp_path:
        print(f'\n[gen] handing off to run_experiments.py (assume-running) ...')
        result = subprocess.run(
            [sys.executable, str(SCRIPT_DIR / 'run_experiments.py'),
             '--config', str(exp_path),
             '--stack-wait', str(args.stack_wait),
             '--assume-running'],
            cwd=SCRIPT_DIR,
        )
        sys.exit(result.returncode)


if __name__ == '__main__':
    main()
