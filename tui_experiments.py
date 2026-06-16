#!/usr/bin/env python3
"""
tui_experiments.py -- TUI for configuring Aerostack2 spec and experiment files.

Usage
-----
    python3 tui_experiments.py
    python3 tui_experiments.py --spec my_spec.yaml
    python3 tui_experiments.py --experiments experiments.yaml
"""

from __future__ import annotations

import re
import subprocess
import sys
from copy import deepcopy
from pathlib import Path
from typing import Optional

import yaml
from textual import on, work
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.screen import ModalScreen, Screen
from textual.widgets import (
    Button,
    Checkbox,
    Collapsible,
    DataTable,
    Footer,
    Header,
    Input,
    Label,
    ProgressBar,
    RadioButton,
    RadioSet,
    RichLog,
    Rule,
    Select,
    Static,
    TextArea,
)

SCRIPT_DIR = Path(__file__).parent


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def _world_files() -> list[tuple[str, str]]:
    return [(f.name, str(f.relative_to(SCRIPT_DIR)))
            for f in sorted(SCRIPT_DIR.glob('config/*.yaml'))]


def _mission_files() -> list[tuple[str, str]]:
    root = SCRIPT_DIR / 'missions'
    return [(f.name, str(f.relative_to(SCRIPT_DIR)))
            for f in sorted(root.rglob('*.yaml'))]


def _mission_dirs() -> list[str]:
    root = SCRIPT_DIR / 'missions'
    if not root.exists():
        return []
    return [str(d.relative_to(SCRIPT_DIR))
            for d in sorted(d for d in root.iterdir() if d.is_dir())]


def _select_or_input(opts: list[tuple[str, str]], value: str, select_id: str,
                     input_id: str, placeholder: str) -> ComposeResult:
    """Yield a Select if options exist, otherwise a plain Input."""
    if opts:
        yield Select(opts, value=value or opts[0][1], id=select_id)
    else:
        yield Input(value, placeholder=placeholder, id=input_id)


def _get_widget_value(screen, *ids: str, default: str = '') -> str:
    """Try each widget id in order, return the first non-blank value found."""
    for wid in ids:
        try:
            w = screen.query_one(wid)
            v = w.value
            if v is not Select.BLANK and str(v).strip():
                return str(v).strip()
        except Exception:
            pass
    return default


# ---------------------------------------------------------------------------
# Shared: command display modal
# ---------------------------------------------------------------------------

class CommandModal(ModalScreen):
    BINDINGS = [Binding('escape', 'dismiss_modal')]

    def __init__(self, cmd: list[str]) -> None:
        super().__init__()
        # show relative paths so the command is clean to copy-paste
        self._cmd_str = ' '.join(
            str(Path(p).relative_to(SCRIPT_DIR)) if Path(p).is_absolute()
            and Path(p).is_relative_to(SCRIPT_DIR) else p
            for p in cmd
        )

    def compose(self) -> ComposeResult:
        with Vertical(id='cmd-dialog'):
            yield Static('Run command', id='cmd-title')
            yield Static('From the project directory:', id='cmd-hint')
            yield Static(self._cmd_str, id='cmd-text')
            with Horizontal(id='cmd-buttons'):
                yield Button('Copy', variant='success', id='btn-cmd-copy')
                yield Button('Close', variant='default', id='btn-cmd-close')

    @on(Button.Pressed, '#btn-cmd-copy')
    def _copy(self) -> None:
        try:
            subprocess.run(['xclip', '-selection', 'clipboard'],
                           input=self._cmd_str, text=True, check=True)
            self.notify('Copied to clipboard')
        except FileNotFoundError:
            self.notify('xclip not found — copy manually', severity='warning')
        except subprocess.CalledProcessError as e:
            self.notify(f'Copy failed: {e}', severity='error')

    @on(Button.Pressed, '#btn-cmd-close')
    def action_dismiss_modal(self) -> None:
        self.dismiss()


# ---------------------------------------------------------------------------
# Shared: delete confirmation modal
# ---------------------------------------------------------------------------

class ConfirmModal(ModalScreen[bool]):
    BINDINGS = [Binding('escape', 'cancel')]

    def __init__(self, message: str) -> None:
        super().__init__()
        self._message = message

    def compose(self) -> ComposeResult:
        with Vertical(id='confirm-dialog'):
            yield Static(self._message, id='confirm-msg')
            with Horizontal(id='confirm-buttons'):
                yield Button('Cancel', variant='default', id='btn-cancel')
                yield Button('Delete', variant='error',   id='btn-confirm')

    @on(Button.Pressed, '#btn-confirm')
    def _yes(self) -> None:
        self.dismiss(True)

    @on(Button.Pressed, '#btn-cancel')
    def action_cancel(self) -> None:
        self.dismiss(False)


# ---------------------------------------------------------------------------
# Shared: run output screen
# ---------------------------------------------------------------------------

class RunScreen(Screen):
    """Streams a subprocess that does NOT need a TTY (e.g. generate-only).

    For anything that launches the AS2 stack (tmux / tmuxinator), use
    run_suspended() on the parent screen instead — that hands back the real
    terminal via App.suspend().
    """

    BINDINGS = [Binding('escape', 'stop', 'Stop & back')]

    def __init__(self, cmd: list[str], title: str = '') -> None:
        super().__init__()
        self._cmd   = cmd
        self._title = title
        self._proc: Optional[subprocess.Popen] = None

    def compose(self) -> ComposeResult:
        yield Header()
        yield Static(self._title or ' '.join(self._cmd[-2:]), id='run-status')
        yield ProgressBar(total=100, show_eta=False, id='run-progress')
        yield RichLog(highlight=True, markup=True, id='run-log', wrap=True)
        with Horizontal(id='run-footer'):
            yield Button('Stop & Back', variant='warning', id='btn-stop')
        yield Footer()

    def on_mount(self) -> None:
        self._stream()

    @work(thread=True)
    def _stream(self) -> None:
        log      = self.query_one('#run-log',      RichLog)
        status   = self.query_one('#run-status',   Static)
        progress = self.query_one('#run-progress', ProgressBar)

        self.app.call_from_thread(log.write,
            f'[bold]$ {" ".join(self._cmd)}[/bold]\n')
        try:
            self._proc = subprocess.Popen(
                self._cmd, cwd=SCRIPT_DIR,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1,
            )
        except FileNotFoundError as e:
            self.app.call_from_thread(log.write, f'[red]{e}[/red]')
            return

        for line in self._proc.stdout:
            line = line.rstrip('\n')
            self.app.call_from_thread(log.write, line)
            m = re.search(r'\[(\d+)/(\d+)\]', line)
            if m:
                cur, tot = int(m.group(1)), int(m.group(2))
                pct = int(cur / tot * 100) if tot else 0
                self.app.call_from_thread(status.update,
                    f'Run {cur}/{tot} — {line.split("]", 1)[-1].strip()}')
                self.app.call_from_thread(progress.update, progress=pct)

        self._proc.wait()
        rc = self._proc.returncode
        self.app.call_from_thread(progress.update, progress=100)
        if rc == 0:
            self.app.call_from_thread(status.update, '[green]✓ Complete[/green]')
            self.app.call_from_thread(log.write,
                '\n[bold green]Finished successfully.[/bold green]')
        else:
            self.app.call_from_thread(status.update,
                f'[red]Finished with errors (exit {rc})[/red]')
            self.app.call_from_thread(log.write,
                f'\n[bold red]Exit code {rc}[/bold red]')

    def action_stop(self) -> None:
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        self.app.pop_screen()

    @on(Button.Pressed, '#btn-stop')
    def _on_stop(self) -> None:
        self.action_stop()


def run_suspended(app: App, cmd: list[str]) -> int:
    """Suspend the TUI, run cmd in the restored terminal, resume when done."""
    with app.suspend():
        result = subprocess.run(cmd, cwd=SCRIPT_DIR)
    return result.returncode


# ===========================================================================
# SPEC GENERATOR SCREENS
# ===========================================================================

def _spec_summary(s: dict) -> tuple[str, str, str, str, str]:
    name   = s.get('name', '')
    drones = str(s.get('drones', {}).get('count', '?'))
    areas  = s.get('areas', {})
    area_s = f"{areas.get('layout','?')} {areas.get('rows','')}{areas.get('cols','')}".strip()
    exp    = s.get('experiment', {})
    modes  = ', '.join(exp['modes']) if 'modes' in exp else exp.get('mode', 'centralized')
    sweep  = '✓' if 'sweep' in s else '—'
    return name, drones, area_s, modes, sweep


class EditSpecScreen(Screen):
    """Form for creating / editing a generate_mission.py spec."""

    BINDINGS = [Binding('escape', 'cancel')]

    def __init__(self, spec: Optional[dict] = None) -> None:
        super().__init__()
        self._spec = deepcopy(spec) if spec else {}

    def compose(self) -> ComposeResult:
        s = self._spec
        arena   = s.get('arena',   {})
        drones  = s.get('drones',  {})
        start   = drones.get('start', {})
        areas   = s.get('areas',   {})
        mission = s.get('mission', {})
        world   = s.get('world',   {})
        output  = s.get('output',  {})
        exp     = s.get('experiment', {})
        failure  = exp.get('failure', {})
        reinspect = exp.get('reinspect', {})
        sweep_text = (yaml.dump(s['sweep'], default_flow_style=False)
                      if 'sweep' in s else '')

        yield Header()
        with VerticalScroll(id='edit-scroll'):

            # ── basics ────────────────────────────────────────────────────
            yield Label('Run name', classes='field-label')
            yield Input(s.get('name', ''), placeholder='my_coverage_run', id='inp-name')

            yield Label('Arena half-size (m)', classes='field-label')
            yield Input(str(arena.get('half', 10.0)), placeholder='10.0', id='inp-arena-half')

            yield Rule()

            # ── drones ───────────────────────────────────────────────────
            yield Label('Drones — count', classes='field-label')
            yield Input(str(drones.get('count', 3)), placeholder='3', id='inp-drone-count')

            yield Label('Start strategy', classes='field-label')
            strategy = start.get('strategy', 'left_edge')
            with RadioSet(id='rs-strategy'):
                yield RadioButton('left_edge', value=strategy == 'left_edge')
                yield RadioButton('custom',    value=strategy == 'custom')

            with Vertical(id='strategy-left-edge'):
                yield Label('Start X position', classes='field-label')
                yield Input(str(start.get('x', -10.0)), placeholder='-10.0', id='inp-start-x')

            with Vertical(id='strategy-custom'):
                yield Label('Custom positions (YAML list of {x, y})', classes='field-label')
                pos_text = (yaml.dump(start['positions'], default_flow_style=False)
                            if 'positions' in start else '')
                yield TextArea(pos_text, id='ta-positions', language='yaml')

            yield Rule()

            # ── areas ────────────────────────────────────────────────────
            yield Label('Area layout', classes='field-label')
            layout = areas.get('layout', 'grid_areas')
            yield Select(
                [('grid_areas — rows × cols rectangles', 'grid_areas'),
                 ('strip_areas — parallel strips',       'strip_areas'),
                 ('custom — explicit polygons',          'custom')],
                value=layout, id='sel-layout',
            )
            yield Label('Area name prefix', classes='field-label')
            yield Input(areas.get('prefix', 'area'), placeholder='area', id='inp-prefix')

            with Vertical(id='layout-grid'):
                with Horizontal():
                    with Vertical():
                        yield Label('Rows', classes='field-label')
                        yield Input(str(areas.get('rows', 2)), placeholder='2', id='inp-rows')
                    with Vertical():
                        yield Label('Cols', classes='field-label')
                        yield Input(str(areas.get('cols', 3)), placeholder='3', id='inp-cols')

            with Vertical(id='layout-strip'):
                with Horizontal():
                    with Vertical():
                        yield Label('Strip count', classes='field-label')
                        yield Input(str(areas.get('count', 3)), placeholder='3', id='inp-strip-count')
                    with Vertical():
                        yield Label('Axis', classes='field-label')
                        yield Select([('y — horizontal strips', 'y'),
                                      ('x — vertical strips',   'x')],
                                     value=areas.get('axis', 'y'), id='sel-axis')

            with Vertical(id='layout-custom'):
                yield Label('Custom polygons (YAML list of {name, vertices: [{x,y}]})',
                            classes='field-label')
                poly_text = (yaml.dump(areas['polygons'], default_flow_style=False)
                             if 'polygons' in areas else '')
                yield TextArea(poly_text, id='ta-polygons', language='yaml')

            yield Rule()

            # ── mission ──────────────────────────────────────────────────
            yield Label('Takeoff height (m)', classes='field-label')
            yield Input(str(mission.get('takeoff_height', 1.0)),
                        placeholder='1.0', id='inp-takeoff')

            yield Rule()

            # ── coverage params ──────────────────────────────────────────
            with Collapsible(title='Coverage params (world)', collapsed=False):
                with Horizontal():
                    with Vertical():
                        yield Label('Street spacing (m)', classes='field-label')
                        yield Input(str(world.get('street_spacing', 1.0)),
                                    placeholder='1.0', id='inp-street-spacing')
                    with Vertical():
                        yield Label('WP space (m)', classes='field-label')
                        yield Input(str(world.get('wp_space', 1.0)),
                                    placeholder='1.0', id='inp-wp-space')
                with Horizontal():
                    with Vertical():
                        yield Label('Coverage height (m)', classes='field-label')
                        yield Input(str(world.get('height', 5.0)),
                                    placeholder='5.0', id='inp-cov-height')
                    with Vertical():
                        yield Label('Coverage speed (m/s)', classes='field-label')
                        yield Input(str(world.get('speed', 2.0)),
                                    placeholder='2.0', id='inp-cov-speed')
                yield Label('SDF output dir (optional — generates Gazebo world)',
                            classes='field-label')
                yield Input(world.get('output_dir', ''),
                            placeholder='assets/worlds/', id='inp-sdf-dir')

            # ── output paths ─────────────────────────────────────────────
            with Collapsible(title='Output paths', collapsed=True):
                yield Label('World dir', classes='field-label')
                yield Input(output.get('world_dir', 'config/'),
                            placeholder='config/', id='inp-out-world')
                yield Label('Mission dir', classes='field-label')
                yield Input(output.get('mission_dir', 'missions/'),
                            placeholder='missions/', id='inp-out-mission')

            yield Rule()

            # ── experiment block ─────────────────────────────────────────
            with Collapsible(title='Experiment run config', collapsed=False):
                yield Label('Modes', classes='field-label')
                active_modes = set(exp.get('modes', [exp.get('mode', 'centralized')]))
                with Horizontal(id='spec-modes-row'):
                    yield Checkbox('centralized',   'centralized'   in active_modes,
                                   id='chk-central')
                    yield Checkbox('decentralized', 'decentralized' in active_modes,
                                   id='chk-decentral')

                yield Label('Repetitions (times)', classes='field-label')
                yield Input(str(exp.get('times', 1)), placeholder='1', id='inp-times')

                with Collapsible(title='Failure injection', collapsed=not failure):
                    yield Label('Drone namespace', classes='field-label')
                    yield Input(failure.get('drone', ''), placeholder='drone0',
                                id='inp-fail-drone')
                    with Horizontal():
                        with Vertical():
                            yield Label('at_waypoint (centralized)', classes='field-label')
                            yield Input(str(failure.get('at_waypoint', '')),
                                        placeholder='3', id='inp-fail-wp')
                        with Vertical():
                            yield Label('delay s (decentralized)', classes='field-label')
                            yield Input(str(failure.get('delay', '')),
                                        placeholder='20.0', id='inp-fail-delay')

                with Collapsible(title='Re-inspection anomaly', collapsed=not reinspect):
                    with Horizontal():
                        with Vertical():
                            yield Label('Waypoint ID', classes='field-label')
                            yield Input(reinspect.get('waypoint', ''),
                                        placeholder='wp_3', id='inp-ri-wp')
                        with Vertical():
                            yield Label('Delay (s)', classes='field-label')
                            yield Input(str(reinspect.get('delay', '')),
                                        placeholder='45.0', id='inp-ri-delay')

            # ── sweep ─────────────────────────────────────────────────────
            with Collapsible(title='Sweep parameters (YAML)', collapsed=not sweep_text):
                yield Label('List of {param, values} — one per axis. e.g.:\n'
                            '- param: drones.count\n  values: [3, 5, 7]',
                            classes='field-label')
                yield TextArea(sweep_text, id='ta-sweep', language='yaml')

        with Horizontal(id='edit-footer'):
            yield Button('Cancel',       variant='default', id='btn-cancel')
            yield Button('Save',         variant='primary',  id='btn-save')
            yield Button('Save & Run',   variant='success',  id='btn-save-run')
        yield Footer()

    # -- visibility -----------------------------------------------------------

    def on_mount(self) -> None:
        self._update_strategy_visibility()
        self._update_layout_visibility()

    def _update_strategy_visibility(self) -> None:
        try:
            rs: RadioSet = self.query_one('#rs-strategy')
            is_custom = rs.pressed_index == 1
            self.query_one('#strategy-left-edge').display = not is_custom
            self.query_one('#strategy-custom').display    =     is_custom
        except Exception:
            pass

    def _update_layout_visibility(self) -> None:
        try:
            layout = str(self.query_one('#sel-layout', Select).value)
            self.query_one('#layout-grid').display   = layout == 'grid_areas'
            self.query_one('#layout-strip').display  = layout == 'strip_areas'
            self.query_one('#layout-custom').display = layout == 'custom'
        except Exception:
            pass

    @on(RadioSet.Changed, '#rs-strategy')
    def _strategy_changed(self) -> None:
        self._update_strategy_visibility()

    @on(Select.Changed, '#sel-layout')
    def _layout_changed(self) -> None:
        self._update_layout_visibility()

    # -- actions --------------------------------------------------------------

    def action_cancel(self) -> None:
        self.dismiss((None, False))

    @on(Button.Pressed, '#btn-cancel')
    def _cancel(self) -> None:
        self.dismiss((None, False))

    @on(Button.Pressed, '#btn-save')
    def _save(self) -> None:
        result = self._collect()
        if result:
            self.dismiss((result, False))

    @on(Button.Pressed, '#btn-save-run')
    def _save_run(self) -> None:
        result = self._collect()
        if result:
            self.dismiss((result, True))

    def _f(self, widget_id: str, default: str = '') -> str:
        return _get_widget_value(self, widget_id, default=default)

    def _float(self, widget_id: str, default: float) -> float:
        try:
            return float(self._f(widget_id) or default)
        except ValueError:
            return default

    def _int(self, widget_id: str, default: int) -> int:
        try:
            return int(self._f(widget_id) or default)
        except ValueError:
            return default

    def _collect(self) -> Optional[dict]:
        name = self._f('#inp-name')
        if not name:
            self.notify('Name is required.', severity='error')
            return None

        spec: dict = {
            'name':  name,
            'arena': {'half': self._float('#inp-arena-half', 10.0)},
        }

        # drones
        rs: RadioSet = self.query_one('#rs-strategy')
        strategy = 'left_edge' if rs.pressed_index == 0 else 'custom'
        drones_cfg: dict = {
            'count': self._int('#inp-drone-count', 3),
            'start': {'strategy': strategy},
        }
        if strategy == 'left_edge':
            drones_cfg['start']['x'] = self._float('#inp-start-x', -10.0)
        else:
            pos_text = self.query_one('#ta-positions', TextArea).text.strip()
            if pos_text:
                try:
                    drones_cfg['start']['positions'] = yaml.safe_load(pos_text)
                except yaml.YAMLError as e:
                    self.notify(f'Positions YAML error: {e}', severity='error')
                    return None
        spec['drones'] = drones_cfg

        # areas
        layout = self._f('#sel-layout', 'grid_areas')
        areas_cfg: dict = {'layout': layout, 'prefix': self._f('#inp-prefix') or 'area'}
        if layout == 'grid_areas':
            areas_cfg['rows'] = self._int('#inp-rows', 2)
            areas_cfg['cols'] = self._int('#inp-cols', 3)
        elif layout == 'strip_areas':
            areas_cfg['count'] = self._int('#inp-strip-count', 3)
            areas_cfg['axis']  = self._f('#sel-axis', 'y')
        elif layout == 'custom':
            poly_text = self.query_one('#ta-polygons', TextArea).text.strip()
            if poly_text:
                try:
                    areas_cfg['polygons'] = yaml.safe_load(poly_text)
                except yaml.YAMLError as e:
                    self.notify(f'Polygons YAML error: {e}', severity='error')
                    return None
        spec['areas'] = areas_cfg

        # mission
        spec['mission'] = {'takeoff_height': self._float('#inp-takeoff', 1.0)}

        # world / coverage
        world_cfg: dict = {
            'street_spacing': self._float('#inp-street-spacing', 1.0),
            'wp_space':       self._float('#inp-wp-space', 1.0),
            'height':         self._float('#inp-cov-height', 5.0),
            'speed':          self._float('#inp-cov-speed', 2.0),
        }
        sdf_dir = self._f('#inp-sdf-dir')
        if sdf_dir:
            world_cfg['output_dir'] = sdf_dir
        spec['world'] = world_cfg

        # output paths
        out_world   = self._f('#inp-out-world')   or 'config/'
        out_mission = self._f('#inp-out-mission') or 'missions/'
        spec['output'] = {'world_dir': out_world, 'mission_dir': out_mission}

        # experiment block
        central   = self.query_one('#chk-central',   Checkbox).value
        decentral = self.query_one('#chk-decentral', Checkbox).value
        modes = [m for m, on_ in [('centralized', central), ('decentralized', decentral)] if on_]
        if not modes:
            modes = ['centralized']
        exp_cfg: dict = {}
        if len(modes) == 1:
            exp_cfg['mode'] = modes[0]
        else:
            exp_cfg['modes'] = modes
        times = self._int('#inp-times', 1)
        if times > 1:
            exp_cfg['times'] = times

        fail_drone = self._f('#inp-fail-drone')
        if fail_drone:
            failure: dict = {'drone': fail_drone}
            wp_s = self._f('#inp-fail-wp')
            dl_s = self._f('#inp-fail-delay')
            if wp_s:
                try:
                    failure['at_waypoint'] = int(wp_s)
                except ValueError:
                    pass
            if dl_s:
                try:
                    failure['delay'] = float(dl_s)
                except ValueError:
                    pass
            exp_cfg['failure'] = failure

        ri_wp = self._f('#inp-ri-wp')
        if ri_wp:
            ri: dict = {'waypoint': ri_wp}
            ri_d = self._f('#inp-ri-delay')
            if ri_d:
                try:
                    ri['delay'] = float(ri_d)
                except ValueError:
                    pass
            exp_cfg['reinspect'] = ri

        if exp_cfg:
            spec['experiment'] = exp_cfg

        # sweep
        sweep_text = self.query_one('#ta-sweep', TextArea).text.strip()
        if sweep_text:
            try:
                spec['sweep'] = yaml.safe_load(sweep_text)
            except yaml.YAMLError as e:
                self.notify(f'Sweep YAML error: {e}', severity='error')
                return None

        return spec


class SpecMainScreen(Screen):
    BINDINGS = [
        Binding('a',      'add_spec',    'Add spec'),
        Binding('e',      'edit_spec',   'Edit'),
        Binding('d',      'delete_spec', 'Delete'),
        Binding('ctrl+s', 'save',        'Save YAML'),
        Binding('ctrl+o', 'load',        'Load YAML'),
        Binding('r',      'run_spec',    'Generate & Run'),
        Binding('g',      'generate',    'Generate only'),
        Binding('tab',    'switch_mode', 'Switch to Experiments'),
    ]

    def __init__(self, initial_path: Optional[Path] = None) -> None:
        super().__init__()
        self._specs: list[dict] = []
        self._config_path = initial_path or SCRIPT_DIR / 'specs.yaml'
        if initial_path and initial_path.exists():
            self._load_from_file(initial_path)

    def compose(self) -> ComposeResult:
        yield Header()
        with Vertical(id='main-body'):
            with Horizontal(id='mode-bar'):
                yield Button('▶ Spec Generator',  variant='primary',  id='btn-mode-spec')
                yield Button('  Experiment Runner', variant='default', id='btn-mode-exp')
            yield Static(f'Config: {self._config_path}', id='config-path-label')
            yield DataTable(id='specs-table', cursor_type='row')
            with Horizontal(id='main-toolbar'):
                yield Button('Add [a]',           variant='success', id='btn-add')
                yield Button('Edit [e]',           variant='primary',  id='btn-edit')
                yield Button('Delete [d]',         variant='error',    id='btn-delete')
                yield Button('Load [^o]',          variant='default',  id='btn-load')
                yield Button('Save [^s]',          variant='default',  id='btn-save')
                yield Button('Generate [g]',       variant='default',  id='btn-gen')
                yield Button('Generate & Run [r]', variant='warning',  id='btn-run')
        yield Footer()

    def on_mount(self) -> None:
        table: DataTable = self.query_one('#specs-table')
        table.add_columns('Name', 'Drones', 'Areas', 'Mode(s)', 'Sweep')
        self._refresh_table()

    def _refresh_table(self) -> None:
        table: DataTable = self.query_one('#specs-table')
        table.clear()
        for s in self._specs:
            table.add_row(*_spec_summary(s))

    def _load_from_file(self, path: Path) -> None:
        try:
            data = yaml.safe_load(path.read_text()) or {}
            # support a list of specs under 'specs:' or a bare list or a single spec
            if 'specs' in data:
                self._specs = data['specs']
            elif isinstance(data, list):
                self._specs = data
            elif 'name' in data:
                self._specs = [data]   # single spec file
            else:
                self._specs = []
            self._config_path = path
        except Exception as e:
            self.notify(f'Load failed: {e}', severity='error')

    def _save_to_file(self, path: Optional[Path] = None) -> None:
        target = path or self._config_path
        try:
            target.write_text(
                yaml.dump({'specs': self._specs}, default_flow_style=False, sort_keys=False)
            )
            self.notify(f'Saved → {target}')
            self._config_path = target
            self.query_one('#config-path-label', Static).update(f'Config: {target}')
        except Exception as e:
            self.notify(f'Save failed: {e}', severity='error')

    def _current_spec(self) -> Optional[dict]:
        table: DataTable = self.query_one('#specs-table')
        if not self._specs or table.cursor_row < 0:
            return None
        return self._specs[table.cursor_row]

    def _run_spec(self, spec: dict, generate_only: bool = False) -> None:
        tmp = SCRIPT_DIR / f'.tui_{spec["name"]}_spec.yaml'
        tmp.write_text(yaml.dump(spec, default_flow_style=False, sort_keys=False))
        cmd = [sys.executable, str(SCRIPT_DIR / 'generate_mission.py'), str(tmp)]
        if generate_only:
            self.app.push_screen(RunScreen(cmd, f'Generating {spec["name"]}'))
        else:
            cmd.append('--run')
            rc = run_suspended(self.app, cmd)
            self.notify('Done.' if rc == 0 else f'Exited with code {rc}.',
                        severity='information' if rc == 0 else 'error')

    # -- button / key actions -------------------------------------------------

    @on(Button.Pressed, '#btn-mode-exp')
    def action_switch_mode(self) -> None:
        self.app.switch_screen('experiments')

    @on(Button.Pressed, '#btn-mode-spec')
    def _stay(self) -> None:
        pass

    @on(Button.Pressed, '#btn-add')
    def action_add_spec(self) -> None:
        def _done(result) -> None:
            spec, run_after = result
            if spec is not None:
                self._specs.append(spec)
                self._refresh_table()
                if run_after:
                    self._run_spec(spec)
        self.app.push_screen(EditSpecScreen(), _done)

    @on(Button.Pressed, '#btn-edit')
    def action_edit_spec(self) -> None:
        spec = self._current_spec()
        if spec is None:
            return
        idx = self.query_one('#specs-table', DataTable).cursor_row
        def _done(result) -> None:
            s, run_after = result
            if s is not None:
                self._specs[idx] = s
                self._refresh_table()
                if run_after:
                    self._run_spec(s)
        self.app.push_screen(EditSpecScreen(spec), _done)

    @on(Button.Pressed, '#btn-delete')
    def action_delete_spec(self) -> None:
        spec = self._current_spec()
        if spec is None:
            return
        idx = self.query_one('#specs-table', DataTable).cursor_row
        def _done(yes: bool) -> None:
            if yes:
                del self._specs[idx]
                self._refresh_table()
        self.app.push_screen(ConfirmModal(f'Delete "{spec.get("name", "")}"?'), _done)

    @on(Button.Pressed, '#btn-save')
    def action_save(self) -> None:
        self._save_to_file()

    @on(Button.Pressed, '#btn-load')
    def action_load(self) -> None:
        candidates = sorted(SCRIPT_DIR.glob('*.yaml'))
        for c in candidates:
            if c != self._config_path:
                try:
                    data = yaml.safe_load(c.read_text()) or {}
                    if 'specs' in data or 'name' in data:
                        self._load_from_file(c)
                        self._refresh_table()
                        self.notify(f'Loaded {c.name}')
                        return
                except Exception:
                    pass
        self.notify('No other spec YAML files found.', severity='warning')

    @on(Button.Pressed, '#btn-gen')
    def action_generate(self) -> None:
        spec = self._current_spec()
        if spec is None:
            self.notify('Select a spec first.', severity='warning')
            return
        self._run_spec(spec, generate_only=True)

    @on(Button.Pressed, '#btn-run')
    def action_run_spec(self) -> None:
        spec = self._current_spec()
        if spec is None:
            self.notify('Select a spec first.', severity='warning')
            return
        self._run_spec(spec, generate_only=False)


# ===========================================================================
# EXPERIMENT RUNNER SCREENS
# ===========================================================================

def _run_summary(run: dict) -> tuple[str, str, str, str, str]:
    name    = run.get('name', '')
    modes   = ', '.join(run['modes']) if 'modes' in run else run.get('mode', 'centralized')
    times   = str(run.get('times', 1))
    mission = run.get('mission_file') or run.get('mission_dir', '')
    mission = Path(mission).name if mission else ''
    extras  = [k for k in ('failure', 'reinspect') if run.get(k)]
    return name, mission, modes, times, ', '.join(extras) or '—'


class EditRunScreen(Screen):
    BINDINGS = [Binding('escape', 'cancel')]

    def __init__(self, run: Optional[dict] = None) -> None:
        super().__init__()
        self._run = deepcopy(run) if run else {}

    def compose(self) -> ComposeResult:
        r = self._run
        yield Header()
        with VerticalScroll(id='edit-scroll'):
            yield Label('Run name', classes='field-label')
            yield Input(r.get('name', ''), placeholder='my_coverage_run', id='inp-name')

            yield Rule()
            yield Label('World file', classes='field-label')
            world_opts = _world_files()
            world_val  = r.get('world_file', '')
            yield from _select_or_input(world_opts, world_val,
                                        'sel-world', 'inp-world', 'config/world_swarm.yaml')

            yield Rule()
            yield Label('Mission source', classes='field-label')
            use_dir = 'mission_dir' in r
            with RadioSet(id='rs-mission-source'):
                yield RadioButton('Single file', value=not use_dir)
                yield RadioButton('Directory',   value=use_dir)

            with Vertical(id='mission-file-section'):
                mission_opts = _mission_files()
                yield from _select_or_input(mission_opts, r.get('mission_file', ''),
                                            'sel-mission-file', 'inp-mission-file',
                                            'missions/coverage.yaml')

            with Vertical(id='mission-dir-section'):
                with Horizontal(id='dir-row'):
                    yield Input(r.get('mission_dir', ''),
                                placeholder='missions/my_sweep/', id='inp-mission-dir')
                    yield Button('↺', id='btn-refresh-dirs', variant='default')

            yield Rule()
            yield Label('Modes', classes='field-label')
            active = set(r.get('modes', [r.get('mode', 'centralized')]))
            with Horizontal(id='modes-row'):
                yield Checkbox('centralized',   'centralized'   in active, id='chk-central')
                yield Checkbox('decentralized', 'decentralized' in active, id='chk-decentral')

            yield Rule()
            yield Label('Repetitions (times)', classes='field-label')
            yield Input(str(r.get('times', 1)), placeholder='1', id='inp-times')

            yield Rule()
            failure = r.get('failure') or {}
            with Collapsible(title='Failure injection', collapsed=not failure):
                yield Label('Drone namespace', classes='field-label')
                yield Input(failure.get('drone', ''), placeholder='drone0', id='inp-fail-drone')
                with Horizontal():
                    with Vertical():
                        yield Label('at_waypoint (centralized)', classes='field-label')
                        yield Input(str(failure.get('at_waypoint', '')),
                                    placeholder='3', id='inp-fail-wp')
                    with Vertical():
                        yield Label('delay s (decentralized)', classes='field-label')
                        yield Input(str(failure.get('delay', '')),
                                    placeholder='20.0', id='inp-fail-delay')

            reinspect = r.get('reinspect') or {}
            with Collapsible(title='Re-inspection anomaly', collapsed=not reinspect):
                with Horizontal():
                    with Vertical():
                        yield Label('Waypoint ID', classes='field-label')
                        yield Input(reinspect.get('waypoint', ''),
                                    placeholder='wp_3', id='inp-ri-wp')
                    with Vertical():
                        yield Label('Delay (s)', classes='field-label')
                        yield Input(str(reinspect.get('delay', '')),
                                    placeholder='45.0', id='inp-ri-delay')

            params_text = yaml.dump(r['params'], default_flow_style=False) if r.get('params') else ''
            with Collapsible(title='params overrides (YAML)', collapsed=not params_text):
                yield TextArea(params_text, id='ta-params', language='yaml')

            layer_text = (yaml.dump(r['layer_params'], default_flow_style=False)
                          if r.get('layer_params') else '')
            with Collapsible(title='layer_params overrides (YAML)', collapsed=not layer_text):
                yield TextArea(layer_text, id='ta-layer-params', language='yaml')

        with Horizontal(id='edit-footer'):
            yield Button('Cancel', variant='default', id='btn-cancel')
            yield Button('Save',   variant='primary',  id='btn-save')
        yield Footer()

    def on_mount(self) -> None:
        self._update_mission_visibility()

    def _update_mission_visibility(self) -> None:
        try:
            rs: RadioSet = self.query_one('#rs-mission-source')
            use_dir = rs.pressed_index == 1
            self.query_one('#mission-file-section').display = not use_dir
            self.query_one('#mission-dir-section').display  =     use_dir
        except Exception:
            pass

    @on(RadioSet.Changed, '#rs-mission-source')
    def _source_changed(self) -> None:
        self._update_mission_visibility()
        if self.query_one('#rs-mission-source', RadioSet).pressed_index == 1:
            self._refresh_dirs()

    @on(Button.Pressed, '#btn-refresh-dirs')
    def _refresh_dirs(self) -> None:
        dirs = _mission_dirs()
        inp  = self.query_one('#inp-mission-dir', Input)
        if dirs and not inp.value:
            inp.value = dirs[0]
        self.notify(f'{len(dirs)} director{"y" if len(dirs) == 1 else "ies"} found'
                    + (': ' + ', '.join(Path(d).name for d in dirs) if dirs else ''),
                    severity='information' if dirs else 'warning')

    def action_cancel(self) -> None:
        self.dismiss(None)

    @on(Button.Pressed, '#btn-cancel')
    def _cancel(self) -> None:
        self.dismiss(None)

    @on(Button.Pressed, '#btn-save')
    def _save(self) -> None:
        result = self._collect()
        if result is not None:
            self.dismiss(result)

    def _f(self, *ids: str, default: str = '') -> str:
        return _get_widget_value(self, *ids, default=default)

    def _collect(self) -> Optional[dict]:
        name = self._f('#inp-name')
        if not name:
            self.notify('Run name is required.', severity='error')
            return None

        run: dict = {'name': name}
        world = self._f('#sel-world', '#inp-world')
        if not world:
            self.notify('World file is required.', severity='error')
            return None
        run['world_file'] = world

        rs: RadioSet = self.query_one('#rs-mission-source')
        if rs.pressed_index == 0:
            mf = self._f('#sel-mission-file', '#inp-mission-file')
            if not mf:
                self.notify('Mission file is required.', severity='error')
                return None
            run['mission_file'] = mf
        else:
            md = self._f('#inp-mission-dir')
            if not md:
                self.notify('Mission directory is required.', severity='error')
                return None
            run['mission_dir'] = md

        central   = self.query_one('#chk-central',   Checkbox).value
        decentral = self.query_one('#chk-decentral', Checkbox).value
        modes = [m for m, on_ in [('centralized', central), ('decentralized', decentral)] if on_]
        if not modes:
            modes = ['centralized']
        if len(modes) == 1:
            run['mode'] = modes[0]
        else:
            run['modes'] = modes

        try:
            times = int(self._f('#inp-times') or '1')
        except ValueError:
            times = 1
        if times > 1:
            run['times'] = times

        fail_drone = self._f('#inp-fail-drone')
        if fail_drone:
            failure: dict = {'drone': fail_drone}
            wp_s = self._f('#inp-fail-wp')
            dl_s = self._f('#inp-fail-delay')
            if wp_s:
                try:
                    failure['at_waypoint'] = int(wp_s)
                except ValueError:
                    pass
            if dl_s:
                try:
                    failure['delay'] = float(dl_s)
                except ValueError:
                    pass
            run['failure'] = failure

        ri_wp = self._f('#inp-ri-wp')
        if ri_wp:
            ri: dict = {'waypoint': ri_wp}
            ri_d = self._f('#inp-ri-delay')
            if ri_d:
                try:
                    ri['delay'] = float(ri_d)
                except ValueError:
                    pass
            run['reinspect'] = ri

        params_text = self.query_one('#ta-params', TextArea).text.strip()
        if params_text:
            try:
                run['params'] = yaml.safe_load(params_text) or {}
            except yaml.YAMLError as e:
                self.notify(f'params YAML error: {e}', severity='error')
                return None

        layer_text = self.query_one('#ta-layer-params', TextArea).text.strip()
        if layer_text:
            try:
                run['layer_params'] = yaml.safe_load(layer_text) or {}
            except yaml.YAMLError as e:
                self.notify(f'layer_params YAML error: {e}', severity='error')
                return None

        return run


class MainScreen(Screen):
    BINDINGS = [
        Binding('a',      'add_run',    'Add run'),
        Binding('e',      'edit_run',   'Edit'),
        Binding('d',      'delete_run', 'Delete'),
        Binding('ctrl+s', 'save',       'Save YAML'),
        Binding('ctrl+o', 'load',       'Load YAML'),
        Binding('r',      'run_all',    'Run all'),
        Binding('tab',    'switch_mode','Switch to Spec Generator'),
    ]

    def __init__(self, initial_path: Optional[Path] = None) -> None:
        super().__init__()
        self._runs: list[dict] = []
        self._config_path = initial_path or SCRIPT_DIR / 'experiments.yaml'
        if initial_path and initial_path.exists():
            self._load_from_file(initial_path)

    def compose(self) -> ComposeResult:
        yield Header()
        with Vertical(id='main-body'):
            with Horizontal(id='mode-bar'):
                yield Button('  Spec Generator',   variant='default',  id='btn-mode-spec')
                yield Button('▶ Experiment Runner', variant='primary',  id='btn-mode-exp')
            yield Static(f'Config: {self._config_path}', id='config-path-label')
            yield DataTable(id='runs-table', cursor_type='row')
            with Horizontal(id='main-toolbar'):
                yield Button('Add [a]',    variant='success', id='btn-add')
                yield Button('Edit [e]',   variant='primary',  id='btn-edit')
                yield Button('Delete [d]', variant='error',    id='btn-delete')
                yield Button('Load [^o]',  variant='default',  id='btn-load')
                yield Button('Save [^s]',  variant='default',  id='btn-save')
                yield Button('Run All [r]',variant='warning',  id='btn-run')
        yield Footer()

    def on_mount(self) -> None:
        table: DataTable = self.query_one('#runs-table')
        table.add_columns('Name', 'Mission', 'Mode(s)', 'Times', 'Extras')
        self._refresh_table()

    def _refresh_table(self) -> None:
        table: DataTable = self.query_one('#runs-table')
        table.clear()
        for run in self._runs:
            table.add_row(*_run_summary(run))

    def _load_from_file(self, path: Path) -> None:
        try:
            data = yaml.safe_load(path.read_text()) or {}
            self._runs = data.get('runs', [])
            self._config_path = path
        except Exception as e:
            self.notify(f'Load failed: {e}', severity='error')

    def _save_to_file(self, path: Optional[Path] = None) -> None:
        target = path or self._config_path
        try:
            target.write_text(
                yaml.dump({'runs': self._runs}, default_flow_style=False, sort_keys=False)
            )
            self.notify(f'Saved → {target}')
            self._config_path = target
            self.query_one('#config-path-label', Static).update(f'Config: {target}')
        except Exception as e:
            self.notify(f'Save failed: {e}', severity='error')

    @on(Button.Pressed, '#btn-mode-spec')
    def action_switch_mode(self) -> None:
        self.app.switch_screen('specs')

    @on(Button.Pressed, '#btn-mode-exp')
    def _stay(self) -> None:
        pass

    @on(Button.Pressed, '#btn-add')
    def action_add_run(self) -> None:
        def _done(run: Optional[dict]) -> None:
            if run is not None:
                self._runs.append(run)
                self._refresh_table()
        self.app.push_screen(EditRunScreen(), _done)

    @on(Button.Pressed, '#btn-edit')
    def action_edit_run(self) -> None:
        table: DataTable = self.query_one('#runs-table')
        if not self._runs or table.cursor_row < 0:
            return
        idx = table.cursor_row
        def _done(run: Optional[dict]) -> None:
            if run is not None:
                self._runs[idx] = run
                self._refresh_table()
        self.app.push_screen(EditRunScreen(self._runs[idx]), _done)

    @on(Button.Pressed, '#btn-delete')
    def action_delete_run(self) -> None:
        table: DataTable = self.query_one('#runs-table')
        if not self._runs or table.cursor_row < 0:
            return
        idx  = table.cursor_row
        name = self._runs[idx].get('name', f'run {idx}')
        def _done(yes: bool) -> None:
            if yes:
                del self._runs[idx]
                self._refresh_table()
        self.app.push_screen(ConfirmModal(f'Delete "{name}"?'), _done)

    @on(Button.Pressed, '#btn-save')
    def action_save(self) -> None:
        self._save_to_file()

    @on(Button.Pressed, '#btn-load')
    def action_load(self) -> None:
        candidates = sorted(
            list(SCRIPT_DIR.glob('*experiments*.yaml')) +
            list((SCRIPT_DIR / 'experiments').glob('*.yaml')
                 if (SCRIPT_DIR / 'experiments').exists() else [])
        )
        for c in candidates:
            if c != self._config_path:
                self._load_from_file(c)
                self._refresh_table()
                self.notify(f'Loaded {c.name}')
                return
        self.notify('No other experiment YAML files found.', severity='warning')

    @on(Button.Pressed, '#btn-run')
    def action_run_all(self) -> None:
        if not self._runs:
            self.notify('No runs configured.', severity='warning')
            return
        exp_dir = SCRIPT_DIR / 'experiments'
        exp_dir.mkdir(exist_ok=True)
        tmp = exp_dir / self._config_path.name
        self._save_to_file(tmp)
        cmd = [sys.executable, str(SCRIPT_DIR / 'run_experiments.py'),
               '--config', str(tmp), '--assume-running']
        self.app.push_screen(CommandModal(cmd))


# ===========================================================================
# App
# ===========================================================================

STYLESHEET = """
/* ── shared ──────────────────────────────────────────────── */
#main-body {
    height: 1fr;
}
#mode-bar {
    height: auto;
    padding: 0 1;
}
#mode-bar Button {
    margin: 0 1;
}
#main-toolbar {
    height: auto;
    align: center middle;
    padding: 0 1;
}
#main-toolbar Button {
    margin: 0 1;
}
#config-path-label {
    color: $text-muted;
    padding: 0 1;
}
#runs-table, #specs-table {
    height: 1fr;
    border: solid $primary;
    margin: 1 0;
}

/* ── edit screens ─────────────────────────────────────────── */
#edit-scroll {
    height: 1fr;
    padding: 1 2;
}
.field-label {
    color: $text-muted;
    margin-top: 1;
}
#edit-footer {
    height: auto;
    align: right middle;
    padding: 1 2;
    border-top: solid $primary;
}
#edit-footer Button {
    margin: 0 1;
}
#modes-row, #spec-modes-row {
    height: auto;
}
#modes-row Checkbox, #spec-modes-row Checkbox {
    margin-right: 3;
}
#mission-dir-section {
    display: none;
}
#dir-row {
    height: auto;
}
#dir-row Input {
    width: 1fr;
}
#btn-refresh-dirs {
    width: auto;
    min-width: 5;
}
#strategy-custom {
    display: none;
}
#layout-strip, #layout-custom {
    display: none;
}
TextArea {
    height: 6;
}

/* ── run screen ───────────────────────────────────────────── */
#run-status {
    height: 1;
    padding: 0 1;
    background: $boost;
    text-align: center;
}
#run-progress {
    margin: 1 2;
}
#run-log {
    height: 1fr;
    border: solid $primary;
    margin: 0 1;
}
#run-footer {
    height: auto;
    align: right middle;
    padding: 1 2;
}

/* ── confirm modal ────────────────────────────────────────── */
ConfirmModal {
    align: center middle;
}
#confirm-dialog {
    width: 50;
    height: auto;
    border: solid $warning;
    padding: 2 4;
    background: $surface;
}
#confirm-msg {
    text-align: center;
    margin-bottom: 2;
}
#confirm-buttons {
    align: center middle;
    height: auto;
}
#confirm-buttons Button {
    margin: 0 2;
}

/* ── command modal ────────────────────────────────────────── */
CommandModal {
    align: center middle;
}
#cmd-dialog {
    width: 80;
    height: auto;
    border: solid $success;
    padding: 2 4;
    background: $surface;
}
#cmd-title {
    text-style: bold;
    margin-bottom: 1;
}
#cmd-hint {
    color: $text-muted;
    margin-bottom: 1;
}
#cmd-text {
    background: $boost;
    padding: 1 2;
    margin-bottom: 2;
}
#cmd-buttons {
    align: right middle;
    height: auto;
}
#cmd-buttons Button {
    margin-left: 2;
}
"""


class ExperimentsApp(App):
    CSS   = STYLESHEET
    TITLE = 'Aerostack2 Experiment Runner'
    BINDINGS = [Binding('q', 'quit', 'Quit')]

    def __init__(self, spec_path: Optional[Path] = None,
                 exp_path: Optional[Path] = None) -> None:
        super().__init__()
        self._spec_path = spec_path
        self._exp_path  = exp_path

    def on_mount(self) -> None:
        self.install_screen(SpecMainScreen(self._spec_path), name='specs')
        self.install_screen(MainScreen(self._exp_path),      name='experiments')
        # open spec generator first if a spec file was given, else experiments
        if self._spec_path:
            self.push_screen('specs')
        else:
            self.push_screen('experiments')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    import argparse
    parser = argparse.ArgumentParser(description='TUI experiment configurator')
    parser.add_argument('--spec',        type=Path, help='Spec YAML to load')
    parser.add_argument('--experiments', type=Path, help='Experiments YAML to load')
    # positional shorthand: if a single arg is given, auto-detect type
    parser.add_argument('file', nargs='?', type=Path, help='Spec or experiments YAML')
    args = parser.parse_args()

    spec_path = args.spec
    exp_path  = args.experiments
    if args.file and not spec_path and not exp_path:
        try:
            data = yaml.safe_load(args.file.read_text()) or {}
            if 'runs' in data:
                exp_path = args.file
            else:
                spec_path = args.file
        except Exception:
            spec_path = args.file

    ExperimentsApp(spec_path=spec_path, exp_path=exp_path).run()


if __name__ == '__main__':
    main()
