# Experiment Toolchain Guide

Three scripts form a pipeline:

```
generate_mission.py  →  run_experiments.py  →  evaluate_metrics.py
   (generate)               (execute)              (analyse)
```

---

## Quick start — end to end in one command

```bash
cp generate_mission.yaml.example my_spec.yaml
# edit my_spec.yaml (arena, drones, areas, experiment params)
python3 generate_mission.py my_spec.yaml --run
```

`--run` generates the world config, mission YAML, and `<name>_experiments.yaml`, then hands off to `run_experiments.py` automatically. Results land in `results/<name>/`.

For a sweep:

```bash
# spec includes a sweep: block
python3 generate_mission.py sweep_spec.yaml --run
# → generates all variants, writes <name>_sweep_experiments.yaml, then runs all of them
```

To control the stack startup timeout:

```bash
python3 generate_mission.py my_spec.yaml --run --stack-wait 60
```

---

## 1. Generating missions — `generate_mission.py`

Takes a YAML spec and writes two files per run:

| Output | Location |
|---|---|
| World config | `config/<name>.yaml` |
| Mission YAML | `missions/<name>.yaml` |

The mission YAML is ready to use directly with `send_mission.py` and `mission_executor.py`. It contains the full coverage spec: polygon vertices, `streetSpacing`, `wpSpace`, `height`, and `speed` — so `run_experiments.py` can run it without any further editing.

### Single run

```bash
cp generate_mission.yaml.example my_spec.yaml
# edit my_spec.yaml
python3 generate_mission.py my_spec.yaml           # generate only
python3 generate_mission.py my_spec.yaml --run     # generate + run immediately
python3 generate_mission.py my_spec.yaml --dry-run # preview without writing files
```

This writes three files: `config/<name>.yaml`, `missions/<name>.yaml`, and `<name>_experiments.yaml`. The experiments file is ready to pass to `run_experiments.py` if you prefer to run manually:

```bash
python3 run_experiments.py --config my_experiment_experiments.yaml
```

### Configuring mode and repetitions in the spec

Add an `experiment:` block to the spec to control how the mission runs. This is the main place to set these options when using `generate_mission.py --run`.

```yaml
name: my_coverage_run

arena:
  half: 10.0

drones:
  count: 3
  start:
    strategy: left_edge
    x: -10.0

areas:
  layout: grid_areas
  rows: 2
  cols: 2
  prefix: area

mission:
  takeoff_height: 1.0

world:
  street_spacing: 1.0
  wp_space:       1.0
  height:         5.0
  speed:          2.0

output:
  world_dir:   config/
  mission_dir: missions/

experiment:
  mode: centralized   # which execution approach to use (see below)
  times: 5            # run the same mission 5 times and average the metrics
```

```bash
python3 generate_mission.py my_spec.yaml --run
# → runs 5 repetitions, prints a comparison table with mean±std for each metric
```

#### `mode` — centralized vs decentralized

| Value | What runs | How waypoints are assigned | How failure is handled |
|---|---|---|---|
| `centralized` | `mission_executor.py` | Pre-planned before takeoff; replanned on failure | Executor tracks which waypoint each drone is at; detects failure and redistributes remaining waypoints via `binpat` |
| `decentralized` | `send_mission.py` | Auction at runtime — drones bid on areas | Launcher publishes a failure fact to the drone's knowledge base; surviving drones re-auction the failed drone's tasks |

If you omit `mode`, it defaults to `centralized`.

#### `times` — repeated runs

`times: N` runs the same mission N times back-to-back (each with a fresh stack launch). The comparison table shows `mean±std` instead of a single value:

```
Run                        Makespan           Util         Fairness
------------------------------------------------------------------------
my_coverage_run (n=5/5)    47.3s±2.1s    94%±3%    0.991±0.004
```

`n=5/5` means all 5 runs produced valid metrics. If a run fails (stack didn't start, mission script error), it is excluded from the average and the count drops, e.g. `n=4/5`.

Individual repetition results are saved to `results/my_coverage_run_r1/`, `results/my_coverage_run_r2/`, etc.

#### Failure injection

To simulate a drone failing mid-mission, add a `failure:` sub-block. The fields differ by mode:

```yaml
experiment:
  mode: centralized
  failure:
    drone: drone0
    at_waypoint: 3    # drone0 fails after visiting its 3rd assigned waypoint
                      # mission_executor.py detects this and replans for the survivors
```

```yaml
experiment:
  mode: decentralized
  failure:
    drone: drone0
    delay: 20.0       # drone0 is declared failed 20 seconds after mission start
                      # the launcher publishes this to /drone0/kb/add_fact
```

When a `failure:` block is present, a `TTReassign` column appears in the comparison table showing how long it took the surviving drones to receive new tasks.

#### Re-inspection anomaly

`reinspect` simulates a panel anomaly being detected mid-mission. It is decentralized-only (the KB monitor system handles it).

```yaml
experiment:
  mode: decentralized
  reinspect:
    waypoint: wp_3   # panel waypoint ID to flag as anomalous
    delay: 45.0      # seconds after mission start before the anomaly is injected
```

After `delay` seconds, the launcher publishes `wp_3 panelStatus anomaly` to every drone's `/droneX/kb/add_fact` topic. Each drone's KB monitor checks whether `wp_3` was assigned to it — only the owner reacts: it flies back to re-inspect, then reconstructs its remaining waypoint sequence and continues.

The waypoint ID must match the ID assigned by the coverage planner. Check the KB or mission logs after a dry run to confirm the exact strings (e.g. `wp_0`, `wp_1`, ...).

You can combine `reinspect` and `failure` in the same experiment — they fire on independent timers.

You can also trigger a re-inspection manually during a live run:

```bash
for d in drone0 drone1 drone2; do
  ros2 topic pub --once /${d}/kb/add_fact std_msgs/msg/String '{data: "wp_3 panelStatus anomaly"}'
done
```

This gets embedded into `<name>_experiments.yaml` so `--run` picks it up automatically. You can also edit the experiments file directly if you prefer to separate generation from execution.

### Parametric sweep

Add a `sweep:` block to the spec to generate one variant per parameter combination:

```yaml
name: solar_sweep

arena:
  half: 10.0

drones:
  count: 5
  start:
    strategy: left_edge
    x: -10.0

areas:
  layout: grid_areas
  rows: 3
  cols: 3
  prefix: area

mission:
  takeoff_height: 1.0

world:
  street_spacing: 1.0
  wp_space:       1.0
  height:         5.0
  speed:          2.0

sweep:
  - param: drones.count
    values: [3, 5, 7]
  - param: areas.rows
    values: [2, 3, 4]
```

```bash
python3 generate_mission.py my_sweep_spec.yaml        # generate only
python3 generate_mission.py my_sweep_spec.yaml --run  # generate + run all variants
```

This generates 9 variants (`solar_sweep_count3_rows2`, …) and writes `solar_sweep_sweep_experiments.yaml`. Any `experiment:` block in the spec is applied to every variant in the sweep.

### Area layouts

| Layout | Key params | What it produces |
|---|---|---|
| `grid_areas` | `rows`, `cols` | rows×cols equal rectangles covering the arena |
| `strip_areas` | `count`, `axis` (`x`/`y`) | N equal parallel strips (vertical or horizontal) |
| `custom` | `polygons: [{name, vertices: [{x,y},...]}]` | Arbitrary user-defined polygons |

### Drone start strategies

| Strategy | Params | Behaviour |
|---|---|---|
| `left_edge` | `x` | All drones at fixed x, evenly spaced along y |
| `custom` | `positions: [{x, y}, ...]` | Explicit position per drone |

---

## 2. Running experiments — `run_experiments.py`

Runs a list of configurations end-to-end: launches the stack, records a bag, runs the mission, stops everything, then computes metrics and generates a trajectory plot.

### Setup

```bash
cp experiments.yaml.example experiments.yaml
# edit experiments.yaml
```

### Experiment config fields

| Field | Required | Description |
|---|---|---|
| `name` | yes | Unique run identifier; used as output directory name |
| `world_file` | yes | Path to world YAML (relative to project root) |
| `mission_file` | yes | Path to YAML mission file in `missions/` |
| `mode` | no | `centralized` (default) or `decentralized` — controls which script is used and how failures work |
| `params` | no | Top-level mission YAML key overrides (`takeoff_height`, `max_wp_distance`, etc.) |
| `layer_params` | no | Per-layer field overrides keyed by layer name (`Area.streetSpacing`, `Area.orientation`, etc.) |
| `mission_file` | no* | Path to a single YAML mission file in `missions/` |
| `mission_dir` | no* | Path to a directory — expands to one run per `*.yaml` file found |
| `modes` | no | List of modes, e.g. `[centralized, decentralized]` — expands to one run per mode |
| `failure` | no | Drone failure injection shorthand — fields differ by `mode` (see below) |
| `reinspect` | no | Panel anomaly injection — `{waypoint, delay}` — decentralized only (see below) |
| `times` | no | Repeat this run N times; comparison table shows mean±std across repetitions |

*Exactly one of `mission_file` or `mission_dir` is required.

### Mode

`mode` selects the mission execution paradigm:

| mode | Script | Task allocation | Failure mechanism |
|---|---|---|---|
| `centralized` | `mission_executor.py` | Pre-planned, replans on failure | Waypoint-triggered: executor sends `MissionUpdate.STOP` to failed drone and `MissionUpdate.PAUSE` to survivors, then replans |
| `decentralized` | `send_mission.py` | Auction-based at runtime | Time-triggered: launcher publishes `droneX droneStatus failed` to `/<droneX>/kb/add_fact` after a delay |

### Failure injection

The `failure` shorthand overrides any `failure_injection` declared in the base mission YAML. Fields differ by mode:

```yaml
# centralized: trigger at a specific waypoint
# mission_executor.py detects _next_wp >= at_waypoint and replans internally
failure:
  drone: drone0
  at_waypoint: 3

# decentralized: trigger after a wall-clock delay (seconds from mission start)
# launcher publishes 'drone0 droneStatus failed' to /drone0/kb/add_fact
failure:
  drone: drone0
  delay: 20.0
```

### Re-inspection anomaly injection

`reinspect` simulates a panel anomaly being detected mid-mission (decentralized mode only):

```yaml
reinspect:
  waypoint: wp_3   # panel waypoint ID to flag — must match the ID assigned by the planner
  delay: 45.0      # seconds after mission start before the anomaly is injected
```

After `delay` seconds, the launcher publishes `wp_3 panelStatus anomaly` to every drone's `/droneX/kb/add_fact`. Each drone checks if `wp_3` was assigned to it — only the owner reacts: it pauses, flies back to re-inspect, then reconstructs its remaining waypoint sequence from the KB and continues.

`reinspect` and `failure` can be set in the same run entry — they use independent timers.

### Mode comparison and directory expansion

Use `modes` to run the same mission under multiple modes without repeating the entry:

```yaml
runs:
  - name: coverage
    world_file: config/world_swarm.yaml
    mission_file: missions/two_drones_area_coverage.yaml
    modes: [centralized, decentralized]   # expands into two runs
```

This produces runs named `coverage_centralized` and `coverage_decentralized`, which appear as separate rows in the comparison table.

Use `mission_dir` to run every mission in a directory:

```yaml
runs:
  - name: sweep
    world_file: config/world_swarm.yaml
    mission_dir: missions/my_sweep/       # one run per *.yaml found
```

Combine both to get the full cross-product — one run per mission × per mode:

```yaml
runs:
  - name: sweep
    world_file: config/world_swarm.yaml
    mission_dir: missions/my_sweep/
    modes: [centralized, decentralized]
    times: 3
```

Auto-generated run names follow this pattern:

| Expansion | Name format |
|---|---|
| single file, single mode | `<name>` |
| single file, multiple modes | `<name>_<mode>` |
| directory, single mode | `<name>_<mission_stem>` |
| directory, multiple modes | `<name>_<mission_stem>_<mode>` |

All other fields (`params`, `layer_params`, `failure`, `reinspect`, `times`) apply uniformly to every expanded run.

### Example `experiments.yaml`

```yaml
runs:
  # centralized: mission_executor.py handles planning and replanning
  - name: two_drones_centralized
    world_file: config/world_swarm.yaml
    mission_file: missions/two_drones_area_coverage.yaml
    mode: centralized

  # decentralized: send_mission.py + auction; no failure
  - name: two_drones_decentralized
    world_file: config/world_swarm.yaml
    mission_file: missions/two_drones_area_coverage.yaml
    mode: decentralized

  # centralized with failure + planner param overrides
  - name: two_drones_tight_failure
    world_file: config/world_swarm.yaml
    mission_file: missions/two_drones_area_coverage.yaml
    mode: centralized
    params:
      takeoff_height: 4.0
      max_wp_distance: 8.0
    layer_params:
      Area:
        streetSpacing: 0.5
        orientation: 45.0

  # decentralized with panel anomaly re-inspection
  - name: two_drones_reinspect
    world_file: config/world_swarm.yaml
    mission_file: missions/two_drones_area_coverage.yaml
    mode: decentralized
    reinspect:
      waypoint: wp_3   # panel ID assigned by the coverage planner
      delay: 45.0      # inject anomaly 45s after mission start

  # decentralized with both drone failure and panel anomaly
  - name: two_drones_failure_and_reinspect
    world_file: config/world_swarm.yaml
    mission_file: missions/two_drones_area_coverage.yaml
    mode: decentralized
    failure:
      drone: drone0
      delay: 20.0
    reinspect:
      waypoint: wp_3
      delay: 45.0
    failure:
      drone: drone0
      at_waypoint: 3
```

### Running

```bash
python3 run_experiments.py                              # uses experiments.yaml
python3 run_experiments.py --config my_experiments.yaml
python3 run_experiments.py --stack-wait 60              # longer stack startup timeout
python3 run_experiments.py --results-dir /tmp/results
```

### Output structure

```
results/
  <run_name>/
    bag/              rosbag directory
    metrics.txt       full metrics report
    trajectory.png    trajectory plot
    *.yaml            effective mission YAML written for this run (with all overrides applied)
```

A comparison table is printed at the end of all runs:

```
Run                       Makespan     Util   Fairness    Path(m)   AuctConv
--------------------------------------------------------------------------------
two_drones_centralized     47.2s      94%      0.991      312.4      1.832s
two_drones_decentralized   43.8s      97%      0.988      298.1      1.901s

  Degradation slope    : +0.0150 utilization/drone  (shallower → more resilient)
```

When any run includes a `failure` block, a `TTReassign` column is appended:

```
Run                          Makespan     Util   Fairness    Path(m)   AuctConv   TTReassign
--------------------------------------------------------------------------------------------
two_drones_tight_failure      51.4s      81%      0.974      287.3      1.910s        3.42s
```

---

## 3. Evaluating a single bag — `evaluate_metrics.py`

Use this to analyse any bag independently of `run_experiments.py`.

```bash
python3 evaluate_metrics.py                            # latest bag in rosbag/rosbags/
python3 evaluate_metrics.py --bag rosbag/rosbags/my_bag
python3 evaluate_metrics.py --bag rosbag/rosbags/my_bag --world config/my_world.yaml
python3 evaluate_metrics.py --bag rosbag/rosbags/my_bag --failed-drone drone2   # enables time-to-reassignment
```

### Metrics reported

**Per-drone:**
- `Path length` — total distance flown (m)
- `Tracking error` — mean deviation from motion reference (m); requires `motion_reference/pose` topic in bag
- `Waypoints` — visited / assigned count and percentage
- `Replan times` — inter-`mission_update` intervals (s); mean, min, max, count

**Swarm-level:**
- `Makespan` — time from first movement to last pose across all drones (s)
- `Capacity utilization` — `visited_waypoints / assigned_waypoints`
- `Load balance` — Jain's fairness index on path lengths (1.0 = perfectly equal)
- `Auction convergence` — mean time from first to last auction feedback message (s)
- `Time-to-reassignment` — gap from drone failure (last pose timestamp) to first `mission_update` on surviving drones (s); only reported when `--failed-drone` is provided or a `failure` block is set in the run config

---

## 4. Resilience metrics

### Degradation slope
Run the same mission with sweeping drone counts (`drones.count: [3, 5, 7]`). `run_experiments.py` fits a linear regression on `(n_drones, utilization)` pairs across all runs and prints the slope at the end of the comparison table.

A **shallower slope** (closer to 0) means performance degrades less as drones are removed — i.e. the approach is more resilient to losses.

```bash
# spec with sweep: [{param: drones.count, values: [3, 5, 7]}]
python3 generate_mission.py sweep_spec.yaml
python3 run_experiments.py --config solar_sweep_sweep_experiments.yaml
```

### Time-to-reassignment

The mechanism differs by mode:

**Centralized:** `mission_executor.py` detects `_next_wp[drone] >= at_waypoint`, sends `MissionUpdate.STOP` to the failed drone and `MissionUpdate.PAUSE` to survivors, then redistributes remaining waypoints with `binpat`. Aerostack2 keeps running for all drones.

**Decentralized:** The launcher publishes `<drone> droneStatus failed` to `/<drone>/kb/add_fact` (a `std_msgs/String` triple) after `delay` seconds. The KB monitor picks up this fact and triggers the resilience handler, which re-auctions the failed drone's tasks to the survivors.

`time-to-reassignment` = timestamp of first `mission_update` on any surviving drone after the failure − timestamp of the failed drone's last pose in the bag.

You can also measure it from an existing bag:

```bash
python3 evaluate_metrics.py \
    --bag results/two_drones_tight_failure/bag \
    --world config/world_swarm.yaml \
    --failed-drone drone0
```

---

## 5. Typical workflows

### Compare centralized vs decentralized on the same mission

```yaml
runs:
  - name: coverage
    world_file: config/world_swarm.yaml
    mission_file: missions/two_drones_area_coverage.yaml
    modes: [centralized, decentralized]
```

```bash
python3 run_experiments.py
# → runs coverage_centralized and coverage_decentralized, side by side in the table
```

### Sweep planner parameters without editing mission files

```yaml
runs:
  - name: spacing_0.5
    world_file: config/world_swarm.yaml
    mission_file: missions/two_drones_area_coverage.yaml
    mode: centralized
    layer_params:
      Area:
        streetSpacing: 0.5

  - name: spacing_1.0
    world_file: config/world_swarm.yaml
    mission_file: missions/two_drones_area_coverage.yaml
    mode: centralized
    layer_params:
      Area:
        streetSpacing: 1.0
```

### Compare resilience under drone failure across modes

```yaml
runs:
  - name: centralized_failure
    world_file: config/world_swarm.yaml
    mission_file: missions/two_drones_area_coverage.yaml
    mode: centralized
    failure:
      drone: drone0
      at_waypoint: 3

  - name: decentralized_failure
    world_file: config/world_swarm.yaml
    mission_file: missions/two_drones_area_coverage.yaml
    mode: decentralized
    failure:
      drone: drone0
      delay: 20.0
```

```bash
python3 run_experiments.py
# TTReassign column appears in the comparison table for both runs
```

### Analyse an existing bag

```bash
python3 evaluate_metrics.py \
    --bag results/coverage_centralized/bag \
    --world config/world_swarm.yaml
```
