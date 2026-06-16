#!/usr/bin/env python3
"""
run_experiments.py -- batch simulation runner with automatic metrics and plots.

For each SimConfig it:
  1. Launches the AS2 stack        (./launch_as2.bash)
  2. Waits for nodes to be ready   (polls for pose topics)
  3. Records a rosbag              (ros2 bag record)
  4. Runs the mission script       (waits for completion)
  5. Stops the bag recorder
  6. Stops the stack               (./stop.bash)
  7. Computes metrics              (evaluate_metrics)
  8. Generates a trajectory plot   (plot_simulation_goto2)

Results land in results/<run_name>/:
    bag/          rosbag directory
    metrics.txt   text report
    trajectory.png

A comparison table across all runs is printed at the end.

Usage
-----
    python3 run_experiments.py
    python3 run_experiments.py --results-dir /tmp/exp
    python3 run_experiments.py --stack-wait 25
"""

__authors__ = 'Guillermo GP-Lenza'
__license__ = 'BSD-3-Clause'

import argparse
import importlib.util
import signal
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass, field, replace
from io import StringIO
from pathlib import Path
from typing import Dict, List, Optional

import yaml

SCRIPT_DIR = Path(__file__).parent


# ---------------------------------------------------------------------------
# Experiment configuration
# ---------------------------------------------------------------------------

@dataclass
class SimConfig:
    name: str          # unique run identifier
    world_file: Path   # passed to launch_as2.bash -w
    mission_file: Path # YAML mission file
    # 'centralized'  → mission_executor.py  (tracks waypoints, replans on failure internally)
    # 'decentralized' → send_mission.py     (auction-based; failure via KB topic publish)
    mode: str = 'centralized'
    # top-level key overrides injected into the mission YAML before sending
    params: dict = field(default_factory=dict)
    # per-layer field overrides keyed by layer name (e.g. {'Area': {'streetSpacing': 2.0}})
    layer_params: dict = field(default_factory=dict)
    # failure injection shorthand — overrides any failure_injection in the base mission YAML
    # centralized:   {drone: 'drone0', at_waypoint: 3}  → injected into YAML failure_injection
    # decentralized: {drone: 'drone0', delay: 20.0}     → KB topic publish after delay seconds
    failure: Optional[dict] = None
    # number of times to repeat this run; metrics are averaged with std across repetitions
    times: int = 1
    # reinspect injection: {waypoint: 'wp_3', delay: 30.0}
    # publishes wp_3 panelStatus anomaly to all drones' KBs after delay seconds
    reinspect: Optional[dict] = None


DEFAULT_CONFIG_FILE = SCRIPT_DIR / 'experiments.yaml'


def _expand_entry(entry: dict) -> List[SimConfig]:
    """Expand a single experiments.yaml entry into one or more SimConfigs.

    mission_dir expands to all *.yaml files in that directory.
    modes (list) expands to one run per mode.
    Both together produce the full cross-product.
    Run names are auto-suffixed to stay unique.
    """
    base_name = entry['name']

    if 'mission_dir' in entry:
        mission_files = sorted(Path(entry['mission_dir']).glob('*.yaml'))
        if not mission_files:
            print(f'[run_experiments] warning: no .yaml files found in {entry["mission_dir"]}')
        multi_files = True
    else:
        mission_files = [Path(entry['mission_file'])]
        multi_files = False

    modes = entry.get('modes') or [entry.get('mode', 'centralized')]
    multi_modes = len(modes) > 1

    configs = []
    for mf in mission_files:
        for mode in modes:
            if multi_files and multi_modes:
                name = f'{base_name}_{mf.stem}_{mode}'
            elif multi_files:
                name = f'{base_name}_{mf.stem}'
            elif multi_modes:
                name = f'{base_name}_{mode}'
            else:
                name = base_name

            configs.append(SimConfig(
                name=name,
                world_file=Path(entry['world_file']),
                mission_file=mf,
                mode=mode,
                params=entry.get('params') or {},
                layer_params=entry.get('layer_params') or {},
                failure=entry.get('failure'),
                reinspect=entry.get('reinspect'),
                times=int(entry.get('times', 1)),
            ))
    return configs


def load_configs(path: Path) -> List[SimConfig]:
    data = yaml.safe_load(path.read_text())
    configs = []
    for entry in data.get('runs', []):
        configs.extend(_expand_entry(entry))
    return configs


def _parse_mission_yaml(mission_file: Path) -> dict:
    return yaml.safe_load(mission_file.read_text())


def _apply_params(payload: dict, params: dict, layer_params: dict,
                  failure: Optional[dict], mode: str) -> dict:
    """Return a new payload dict with all overrides merged in.

    failure_injection is only written into the YAML for centralized mode;
    decentralized failures are injected at runtime via the KB topic.
    """
    import copy
    result = copy.deepcopy(payload)
    result.update(params)
    if layer_params:
        for layer in result.get('layers', []):
            overrides = layer_params.get(layer.get('name'))
            if overrides:
                layer.update(overrides)
    if failure and mode == 'centralized':
        drone = failure['drone']
        result['failure_injection'] = {
            drone: {k: v for k, v in failure.items() if k != 'drone'}
        }
    return result


def _effective_payload(config: SimConfig) -> dict:
    payload = _parse_mission_yaml(SCRIPT_DIR / config.mission_file)
    return _apply_params(payload, config.params, config.layer_params, config.failure, config.mode)


def _drones_from_payload(payload: dict) -> List[str]:
    return [str(u) for u in payload.get('uavList', [])]


def _failed_drone_from_payload(payload: dict) -> Optional[str]:
    fi = payload.get('failure_injection')
    return next(iter(fi)) if fi else None


# ---------------------------------------------------------------------------
# Decentralized failure injection
# ---------------------------------------------------------------------------

def _inject_reinspect_kb(waypoint_id: str, drone_names: List[str], delay: float) -> None:
    """Publish panelStatus anomaly for waypoint_id to all drones' KBs after delay seconds."""
    time.sleep(delay)
    fact = f'{waypoint_id} panelStatus anomaly'
    for drone_ns in drone_names:
        topic = f'/{drone_ns}/kb/add_fact'
        subprocess.run(
            ['ros2', 'topic', 'pub', '--once', topic, 'std_msgs/msg/String',
             f'{{data: "{fact}"}}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    print(f'  [reinspect] published "{fact}" to {len(drone_names)} drone KB(s)', flush=True)


def _inject_failure_kb(drone_ns: str, delay: float) -> None:
    """Publish droneStatus failed to the KB after `delay` seconds (daemon thread)."""
    time.sleep(delay)
    topic = f'/{drone_ns}/kb/add_fact'
    fact  = f'{drone_ns} droneStatus failed'
    subprocess.run(
        ['ros2', 'topic', 'pub', '--once', topic, 'std_msgs/msg/String',
         f'{{data: "{fact}"}}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    print(f'  [failure] published "{fact}" to {topic}', flush=True)


# ---------------------------------------------------------------------------
# Stack lifecycle helpers
# ---------------------------------------------------------------------------

def launch_stack(config: SimConfig) -> None:
    import re, shutil
    payload = _effective_payload(config)
    drones  = _drones_from_payload(payload)
    world_file = config.world_file
    if ' ' in str(world_file):
        safe_name = re.sub(r'[^\w\-.]', '_', world_file.name)
        safe_file = world_file.parent / safe_name
        shutil.copy2(world_file, safe_file)
        world_file = safe_file
    cmd = ['bash', 'launch_as2.bash',
           '-w', str(world_file),
           '-n', ','.join(drones)]
    with open(SCRIPT_DIR / 'launch.log', 'w') as log:
        subprocess.run(cmd, cwd=SCRIPT_DIR,
                       stdin=subprocess.DEVNULL,
                       stdout=log, stderr=log)


def stop_stack() -> None:
    subprocess.run(['bash', 'stop.bash'], cwd=SCRIPT_DIR,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)


def wait_for_stack(drones: List[str], timeout: float = 40.0) -> bool:
    """Poll ros2 topic list until all drone pose topics appear or timeout expires."""
    expected = {f'/{d}/self_localization/pose' for d in drones}
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'list', '--spin-time', '2'],
                capture_output=True, text=True, timeout=10,
            )
            active = set(result.stdout.splitlines())
            if expected.issubset(active):
                return True
        except subprocess.TimeoutExpired:
            pass
        time.sleep(1.0)
    return False


# ---------------------------------------------------------------------------
# Bag recording helpers
# ---------------------------------------------------------------------------

def start_bag(bag_dir: Path) -> subprocess.Popen:
    bag_dir.parent.mkdir(parents=True, exist_ok=True)  # parent must exist; rosbag2 creates bag_dir
    proc = subprocess.Popen(
        ['ros2', 'bag', 'record', '--all', '--include-hidden-topics',
         '-o', str(bag_dir)],
        cwd=SCRIPT_DIR,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    time.sleep(1.5)  # give the recorder time to open the bag
    return proc


def stop_bag(proc: subprocess.Popen) -> None:
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        proc.kill()


# ---------------------------------------------------------------------------
# Assume-running helpers (no stack lifecycle, no bag)
# ---------------------------------------------------------------------------

def run_mission_only(config: SimConfig, results_root: Path) -> int:
    """Send one mission to an already-running stack. No bag, no metrics, no stack lifecycle."""
    run_dir = results_root / config.name
    run_dir.mkdir(parents=True, exist_ok=True)

    payload = _effective_payload(config)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False, dir=run_dir) as f:
        yaml.dump(payload, f, default_flow_style=False, allow_unicode=True)
        tmp_path = f.name

    drones = _drones_from_payload(payload)
    if not drones:
        print(f'  [!] could not read uavList from {config.mission_file}', flush=True)
        return 1

    failed_drone = config.failure['drone'] if config.failure else _failed_drone_from_payload(payload)
    if failed_drone:
        print(f'  [failure] {config.mode} injection for {failed_drone}', flush=True)

    if config.mode == 'decentralized' and config.failure:
        delay = float(config.failure.get('delay', 15.0))
        print(f'  scheduling KB failure in {delay:.1f}s ...', flush=True)
        threading.Thread(target=_inject_failure_kb, args=(failed_drone, delay), daemon=True).start()

    if config.reinspect:
        wp    = config.reinspect['waypoint']
        delay = float(config.reinspect.get('delay', 30.0))
        print(f'  scheduling reinspect injection for {wp} in {delay:.1f}s ...', flush=True)
        threading.Thread(target=_inject_reinspect_kb, args=(wp, drones, delay), daemon=True).start()

    script = 'mission_executor.py' if config.mode == 'centralized' else 'send_mission.py'
    print(f'  [{config.mode}] {config.mission_file.name} via {script} ...', flush=True)
    return subprocess.run(
        ['python3', str(SCRIPT_DIR / script), tmp_path, '--use-sim-time'],
        cwd=SCRIPT_DIR,
    ).returncode


def reset_simulation(config: SimConfig, stack_wait: float) -> bool:
    """Stop the stack, relaunch with the run's world file, and wait for readiness."""
    payload = _effective_payload(config)
    drones  = _drones_from_payload(payload)
    stop_stack()
    launch_stack(config)
    print(f'  waiting for nodes (up to {stack_wait:.0f}s) ...', flush=True)
    return wait_for_stack(drones, timeout=stack_wait)


# ---------------------------------------------------------------------------
# Single run (full lifecycle: launch → bag → mission → stop → metrics)
# ---------------------------------------------------------------------------

def run_one(config: SimConfig, results_root: Path, stack_wait: float) -> Optional[dict]:
    import re
    safe_name = re.sub(r'[^\w\-.]', '_', config.name)
    run_dir = results_root / safe_name
    run_dir.mkdir(parents=True, exist_ok=True)
    bag_dir = run_dir / 'bag'
    if bag_dir.exists():
        import shutil
        shutil.rmtree(bag_dir)  # rosbag2 won't write into a pre-existing directory

    payload = _effective_payload(config)

    drones = _drones_from_payload(payload)
    if not drones:
        print(f'  [!] could not read uavList from {config.mission_file}', flush=True)
        return None

    # failed_drone: prefer explicit shorthand over base YAML's failure_injection
    failed_drone = config.failure['drone'] if config.failure else _failed_drone_from_payload(payload)
    if failed_drone:
        print(f'  [failure] {config.mode} failure injection for {failed_drone}', flush=True)

    if config.params or config.layer_params:
        print(f'  [params] overrides: {config.params or ""} layer_params: {config.layer_params or ""}', flush=True)

    print(f'  [{config.mode}] launching stack  ({drones}) ...', flush=True)
    launch_stack(config)

    print(f'  waiting for nodes (up to {stack_wait:.0f}s) ...', flush=True)
    ready = wait_for_stack(drones, timeout=stack_wait)
    if not ready:
        print(f'  [!] stack did not become ready after {stack_wait:.0f}s — aborting run', flush=True)
        print(f'  [!] check launch.log for errors (launch_as2.bash may have failed)', flush=True)
        return None

    print('  recording bag ...', flush=True)
    bag_proc = start_bag(bag_dir)

    tmp_mission = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, dir=run_dir
    )
    try:
        yaml.dump(payload, tmp_mission, default_flow_style=False, allow_unicode=True)
        tmp_mission.flush()
        tmp_mission_path = tmp_mission.name
    finally:
        tmp_mission.close()

    if config.mode == 'decentralized' and config.failure:
        delay = float(config.failure.get('delay', 15.0))
        print(f'  scheduling KB failure injection for {failed_drone} in {delay:.1f}s ...', flush=True)
        threading.Thread(
            target=_inject_failure_kb, args=(failed_drone, delay), daemon=True
        ).start()

    if config.reinspect:
        wp = config.reinspect['waypoint']
        delay = float(config.reinspect.get('delay', 30.0))
        print(f'  scheduling reinspect injection for {wp} in {delay:.1f}s ...', flush=True)
        threading.Thread(
            target=_inject_reinspect_kb, args=(wp, drones, delay), daemon=True
        ).start()

    script = 'mission_executor.py' if config.mode == 'centralized' else 'send_mission.py'
    print(f'  running mission: {config.mission_file.name} via {script} ...', flush=True)
    mission_result = subprocess.run(
        ['python3', str(SCRIPT_DIR / script), tmp_mission_path, '--use-sim-time'],
        cwd=SCRIPT_DIR,
    )
    if mission_result.returncode != 0:
        print('  [!] mission script exited with error', flush=True)

    time.sleep(3)  # capture landing in the bag
    stop_bag(bag_proc)
    print('  stopping stack ...', flush=True)
    stop_stack()
    time.sleep(5)  # let rosbag2 flush and close the sqlite file

    # -- metrics ---------------------------------------------------------------
    print('  computing metrics ...', flush=True)
    metrics = _compute_metrics(bag_dir, drones, run_dir, failed_drone=failed_drone)

    # -- plot ------------------------------------------------------------------
    print('  generating plot ...', flush=True)
    _generate_plot(bag_dir, drones, run_dir / 'trajectory.png')

    return metrics


# ---------------------------------------------------------------------------
# Metrics + plot (delegates to existing modules)
# ---------------------------------------------------------------------------

def _load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _compute_metrics(bag_dir: Path, drones: List[str], out_dir: Path,
                     failed_drone: Optional[str] = None) -> dict:
    em = _load_module(SCRIPT_DIR / 'evaluate_metrics.py', 'evaluate_metrics')
    records = em.read_bag(bag_dir, drones)

    # capture printed report to file
    buf = StringIO()
    _stdout, sys.stdout = sys.stdout, buf
    em.print_report(records, failed_drone=failed_drone)
    sys.stdout = _stdout
    report = buf.getvalue()
    (out_dir / 'metrics.txt').write_text(report)
    print(report, end='')

    # extract scalar summary for comparison table
    lengths = {ns: em.path_length(r.poses) for ns, r in records.items()}
    ttr = em.time_to_reassignment(records, failed_drone) if failed_drone else None
    metrics = {
        'makespan':       em.makespan(records),
        'utilization':    em.capacity_utilization(records),
        'fairness':       em.jains_fairness(list(lengths.values())),
        'convergence':    em.auction_convergence(records),
        'path_total':     sum(lengths.values()),
        'n_drones':       len(drones),
        'reassignment':   ttr,
    }
    (out_dir / 'metrics.yaml').write_text(
        yaml.dump(metrics, default_flow_style=False, sort_keys=False, allow_unicode=True)
    )
    return metrics


def _generate_plot(bag_dir: Path, drones: List[str], out_path: Path) -> None:
    try:
        pg = _load_module(SCRIPT_DIR / 'plot_simulation_goto2.py', 'plot_goto2')
        poses, planned_paths, landed = pg.read_bag(bag_dir, drones)
        drone_starts = {
            d: poses[d][0] if poses.get(d) else (0.0, 0.0)
            for d in drones
        }
        pg.plot_results(
            drones=drones,
            poses=poses,
            planned_paths=planned_paths,
            drone_starts=drone_starts,
            arena_half=pg.ARENA_HALF,
            title=out_path.parent.name,
            output=out_path,
            landed=landed,
        )
    except Exception as e:
        print(f'  [!] plot failed: {e}', flush=True)


# ---------------------------------------------------------------------------
# Metrics aggregation (for repeated runs)
# ---------------------------------------------------------------------------

def _aggregate_metrics(metrics_list: List[Optional[dict]]) -> Optional[dict]:
    import numpy as np
    valid = [m for m in metrics_list if m is not None]
    if not valid:
        return None
    agg = {'_aggregated': True, '_n': len(valid), '_n_total': len(metrics_list)}
    for key in valid[0]:
        vals = [m[key] for m in valid if m.get(key) is not None]
        if vals and all(isinstance(v, (int, float)) for v in vals):
            mean = float(np.mean(vals))
            std  = float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0
            agg[key] = (mean, std)
        else:
            agg[key] = (None, None)
    return agg


def _scalar(val):
    """Return the mean from a (mean, std) tuple, or the value itself."""
    return val[0] if isinstance(val, tuple) else val


def _fmt(val, fmt_str: str) -> str:
    """Format a scalar or (mean, std) tuple as 'value' or 'mean±std'."""
    if val is None:
        return 'N/A'
    if isinstance(val, tuple):
        mean, std = val
        if mean is None:
            return 'N/A'
        return fmt_str.format(mean) + '±' + fmt_str.format(std)
    return fmt_str.format(val)


# ---------------------------------------------------------------------------
# Comparison table
# ---------------------------------------------------------------------------

def print_comparison(results: Dict[str, Optional[dict]]) -> None:
    import numpy as np

    runs = list(results.keys())
    if not runs:
        return

    has_ttr = any(
        m is not None and _scalar(m.get('reassignment')) is not None
        for m in results.values()
    )
    has_agg = any(
        isinstance(m, dict) and m.get('_aggregated')
        for m in results.values()
        if m is not None
    )
    # widen numeric columns when mean±std strings are present
    cw = 16 if has_agg else 10
    uw = 12 if has_agg else 8

    print('\n' + '=' * (25 + 1 + cw + 1 + uw + 1 + cw + 1 + cw + 1 + cw + (1 + cw if has_ttr else 0)))
    print('  Experiment Comparison')
    print('=' * (25 + 1 + cw + 1 + uw + 1 + cw + 1 + cw + 1 + cw + (1 + cw if has_ttr else 0)))
    header = (f"{'Run':<25} {'Makespan':>{cw}} {'Util':>{uw}} {'Fairness':>{cw}}"
              f" {'Path(m)':>{cw}} {'AuctConv':>{cw}}")
    if has_ttr:
        header += f" {'TTReassign':>{cw}}"
    print(header)
    print('-' * len(header))

    for name, m in results.items():
        if m is None:
            print(f'{name:<25}  (all repetitions failed)')
            continue
        label = name
        if m.get('_aggregated'):
            label = f"{name} (n={m['_n']}/{m['_n_total']})"
        ms = _fmt(m.get('makespan'),    '{:.1f}s')
        ut = _fmt(m.get('utilization'), '{:.0%}')
        fa = _fmt(m.get('fairness'),    '{:.3f}')
        pt = _fmt(m.get('path_total'),  '{:.1f}')
        ac = _fmt(m.get('convergence'), '{:.2f}s')
        row = f'{label:<25} {ms:>{cw}} {ut:>{uw}} {fa:>{cw}} {pt:>{cw}} {ac:>{cw}}'
        if has_ttr:
            row += f" {_fmt(m.get('reassignment'), '{:.2f}s'):>{cw}}"
        print(row)

    # Degradation slope: linear fit of utilization vs drone count across runs
    slope_pts = [
        (_scalar(m['n_drones']), _scalar(m['utilization']))
        for m in results.values()
        if m is not None
        and _scalar(m.get('n_drones')) is not None
        and _scalar(m.get('utilization')) is not None
    ]
    unique_counts = len({pt[0] for pt in slope_pts})
    if unique_counts >= 2:
        xs = np.array([pt[0] for pt in slope_pts], dtype=float)
        ys = np.array([pt[1] for pt in slope_pts], dtype=float)
        slope, _ = np.polyfit(xs, ys, 1)
        print(f'\n  Degradation slope    : {slope:+.4f} utilization/drone'
              f'  (shallower → more resilient)')
    print()


# ---------------------------------------------------------------------------
# Assume-running batch loop
# ---------------------------------------------------------------------------

def _run_assume_running(configs: List[SimConfig], total_runs: int,
                        results_root: Path, stack_wait: float) -> None:
    """Batch loop for an already-running stack: send missions, reset between runs."""
    print('[run_experiments] assume-running mode: simulation must already be up', flush=True)
    run_idx   = 0
    flat_runs: List[SimConfig] = []
    for config in configs:
        for r in range(config.times):
            label = f'{config.name}_r{r + 1}' if config.times > 1 else config.name
            flat_runs.append(replace(config, name=label))

    for i, config in enumerate(flat_runs):
        run_idx += 1
        print(f'\n[{run_idx}/{total_runs}] {config.name} — waiting for stack ...', flush=True)

        payload = _effective_payload(config)
        drones  = _drones_from_payload(payload)
        if not drones:
            print(f'  [!] could not read uavList from {config.mission_file}', flush=True)
            continue

        if not wait_for_stack(drones, timeout=stack_wait):
            print('  [!] stack not ready — skipping this run', flush=True)
            continue

        rc = run_mission_only(config, results_root)
        if rc != 0:
            print(f'  [!] mission exited with code {rc}', flush=True)

        is_last = (i == len(flat_runs) - 1)
        if not is_last:
            print('  resetting simulation ...', flush=True)
            if not reset_simulation(config, stack_wait):
                print('  [!] stack did not come back after reset — stopping batch', flush=True)
                return

    print('\n[run_experiments] all runs complete — simulation still running', flush=True)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description='Batch simulation runner')
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG_FILE,
                        help=f'YAML file with simulation configs (default: {DEFAULT_CONFIG_FILE.name})')
    parser.add_argument('--results-dir', type=Path, default=SCRIPT_DIR / 'results',
                        help='Root directory for all run outputs (default: results/)')
    parser.add_argument('--stack-wait', type=float, default=40.0,
                        help='Seconds to wait for the AS2 stack to become ready (default: 40)')
    parser.add_argument('--assume-running', action='store_true',
                        help='Skip initial stack launch; send missions to the already-running '
                             'stack and reset (stop+relaunch) between runs')
    args = parser.parse_args()

    if not args.config.exists():
        print(f'[run_experiments] config file not found: {args.config}')
        print(f'[run_experiments] create it following the structure in experiments.yaml.example')
        sys.exit(1)

    configs = load_configs(args.config)
    if not configs:
        print(f'[run_experiments] no runs defined in {args.config}')
        sys.exit(1)

    total_runs = sum(c.times for c in configs)
    print(f'[run_experiments] loaded {len(configs)} experiment(s), {total_runs} total run(s) from {args.config.name}')

    if args.assume_running:
        _run_assume_running(configs, total_runs, args.results_dir, args.stack_wait)
        return

    results = {}
    run_idx = 0
    for i, config in enumerate(configs):
        if config.times == 1:
            run_idx += 1
            print(f'\n[{run_idx}/{total_runs}] experiment: {config.name}')
            stop_stack()
            results[config.name] = run_one(config, args.results_dir, args.stack_wait)
        else:
            rep_metrics = []
            for r in range(config.times):
                run_idx += 1
                rep_name = f'{config.name}_r{r + 1}'
                print(f'\n[{run_idx}/{total_runs}] experiment: {config.name}  (rep {r + 1}/{config.times} → {rep_name})')
                stop_stack()
                rep_config = replace(config, name=rep_name)
                rep_metrics.append(run_one(rep_config, args.results_dir, args.stack_wait))
            results[config.name] = _aggregate_metrics(rep_metrics)

    print_comparison(results)

    results_path = args.results_dir / 'results.yaml'
    args.results_dir.mkdir(parents=True, exist_ok=True)

    def _serialisable(v):
        if isinstance(v, tuple):
            return list(v)
        if isinstance(v, dict):
            return {k: _serialisable(w) for k, w in v.items()}
        return v

    yaml_results = {name: _serialisable(m) for name, m in results.items()}
    results_path.write_text(
        yaml.dump(yaml_results, default_flow_style=False, sort_keys=False, allow_unicode=True)
    )
    print(f'\n[run_experiments] results saved to {results_path}', flush=True)


if __name__ == '__main__':
    main()
