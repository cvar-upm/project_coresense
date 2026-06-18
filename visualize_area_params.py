"""
Interactive visualization of how Area layer parameters affect coverage path generation.
Run from the project root: python3 visualize_area_params.py

NOTE on wpSpace vs max_wp_distance
-----------------------------------
wpSpace inserts intermediate waypoints along each street pass.  However,
Swarm.filter_waypoints (filter_path_waypoints=True) immediately removes every
collinear point — which is exactly what those intermediates are.  As a result
wpSpace has *no visible effect* on final waypoint geometry when filtering is on.

In practice (mission_manager.py) waypoint density is controlled by
max_wp_distance / _densify_path, which re-inserts points *after* filtering.

This visualizer lets you toggle filtering on/off and compare wpSpace vs
max_wp_distance so the difference is obvious.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, RadioButtons, CheckButtons
from swarm_pylib.swarm_pylib import Swarm

# ── Preset areas ──────────────────────────────────────────────────────────────

PRESETS = {
    'Square 6×6':     [[-3,-3],[3,-3],[3,3],[-3,3]],
    'Rectangle 12×4': [[-6,-2],[6,-2],[6,2],[-6,2]],
    'L-shape':        [[0,0],[6,0],[6,3],[3,3],[3,6],[0,6]],
    'Triangle':       [[-5,-4],[5,-4],[0,5]],
}
PRESET_NAMES = list(PRESETS.keys())
DRONE_COLORS = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12']
HEIGHT = 5.0


def initial_positions_for(area_2d, n):
    xs = [p[0] for p in area_2d]
    ys = [p[1] for p in area_2d]
    x_range = np.linspace(min(xs), max(xs), n + 2)[1:-1]
    return [[float(x), float(min(ys)), HEIGHT] for x in x_range]


def densify(path, max_dist):
    """Re-insert intermediate points so no gap exceeds max_dist (mirrors mission_manager)."""
    result = []
    for i, p in enumerate(path):
        result.append(p)
        if i < len(path) - 1:
            q = path[i + 1]
            dist = math.sqrt(sum((b - a) ** 2 for a, b in zip(p, q)))
            n_seg = math.ceil(dist / max_dist)
            for j in range(1, n_seg):
                t = j / n_seg
                result.append([a + t * (b - a) for a, b in zip(p, q)])
    return result


def compute_paths(area_2d, n_drones, street_sp, wp_sp, orient, auto_orient,
                  do_filter, max_wp_dist):
    area = [[p[0], p[1], HEIGHT] for p in area_2d]
    init_pos = initial_positions_for(area_2d, n_drones)
    uavs_state = {
        f'drone{i}': {'initial_position': init_pos[i], 'last_position': init_pos[i]}
        for i in range(n_drones)
    }
    theta = None if auto_orient else float(orient)

    paths = Swarm.swarm_planning(
        uavs_state, area,
        'back_and_force', 'binpat',
        float(street_sp), float(wp_sp),
        theta,
        filter_path_waypoints=do_filter,
    )

    if do_filter and max_wp_dist > 0:
        paths = [densify(p, max_wp_dist) for p in paths]

    return paths, init_pos


# ── Figure layout ─────────────────────────────────────────────────────────────

fig = plt.figure(figsize=(15, 8))
fig.patch.set_facecolor('#1e1e2e')

ax = fig.add_axes([0.30, 0.13, 0.68, 0.80])
ax.set_facecolor('#2a2a3e')
ax.set_aspect('equal')

S = dict(color='#3a3a5e', track_color='#555577',
         handle_style={'facecolor': '#7c7cff', 'edgecolor': '#9999ff', 'size': 10})

ax_street   = fig.add_axes([0.04, 0.80, 0.20, 0.03])
ax_wpspace  = fig.add_axes([0.04, 0.71, 0.20, 0.03])
ax_maxwp    = fig.add_axes([0.04, 0.62, 0.20, 0.03])
ax_orient   = fig.add_axes([0.04, 0.53, 0.20, 0.03])

sl_street  = Slider(ax_street,  'streetSpacing',  0.3, 5.0,  valinit=1.0, valstep=0.05, **S)
sl_wpspace = Slider(ax_wpspace, 'wpSpace',         0.3, 10.0, valinit=2.0, valstep=0.05, **S)
sl_maxwp   = Slider(ax_maxwp,   'max_wp_dist',     0.3, 10.0, valinit=2.0, valstep=0.05, **S)
sl_orient  = Slider(ax_orient,  'orientation °',   0,   359,  valinit=0,   valstep=1,    **S)

for sl in (sl_street, sl_wpspace, sl_maxwp, sl_orient):
    sl.label.set_color('#ccccee')
    sl.valtext.set_color('#aaaaff')

ax_checks = fig.add_axes([0.04, 0.43, 0.20, 0.08])
ax_checks.set_facecolor('#2a2a3e')
cb_checks = CheckButtons(ax_checks,
                         ['Auto orientation', 'Filter collinear (prod)'],
                         actives=[False, True])
for lbl in cb_checks.labels:
    lbl.set_color('#ccccee')
    lbl.set_fontsize(8)

ax_drones = fig.add_axes([0.04, 0.27, 0.20, 0.13])
ax_drones.set_facecolor('#2a2a3e')
rb_drones = RadioButtons(ax_drones, ['1 drone', '2 drones', '3 drones'], active=0)
ax_drones.set_title('Drones', color='#aaaacc', fontsize=9, pad=2)
for lbl in rb_drones.labels:
    lbl.set_color('#ccccee')

ax_preset = fig.add_axes([0.04, 0.04, 0.20, 0.18])
ax_preset.set_facecolor('#2a2a3e')
rb_preset = RadioButtons(ax_preset, PRESET_NAMES, active=0)
ax_preset.set_title('Area preset', color='#aaaacc', fontsize=9, pad=2)
for lbl in rb_preset.labels:
    lbl.set_color('#ccccee')

stats_text = fig.text(
    0.04, 0.97, '', color='#aaffaa', fontsize=7.5, va='top', family='monospace',
    bbox=dict(boxstyle='round', facecolor='#2a2a3e', edgecolor='#555577', alpha=0.9))


# ── Draw ──────────────────────────────────────────────────────────────────────

def redraw(_=None):
    ax.cla()
    ax.set_facecolor('#2a2a3e')
    ax.set_aspect('equal')
    for spine in ax.spines.values():
        spine.set_color('#555577')
    ax.tick_params(colors='#aaaacc')

    preset_name = rb_preset.value_selected
    n_drones    = int(rb_drones.value_selected.split()[0])
    street_sp   = sl_street.val
    wp_sp       = sl_wpspace.val
    max_wp_dist = sl_maxwp.val
    orient      = sl_orient.val
    statuses    = cb_checks.get_status()
    auto_orient = statuses[0]
    do_filter   = statuses[1]

    area_2d = PRESETS[preset_name]

    poly = plt.Polygon(area_2d, closed=True,
                       facecolor='#3a3a5e', edgecolor='#7777bb', linewidth=1.5, zorder=1)
    ax.add_patch(poly)

    try:
        paths, init_pos = compute_paths(
            area_2d, n_drones, street_sp, wp_sp, orient, auto_orient,
            do_filter, max_wp_dist if do_filter else 0)
    except Exception as e:
        ax.text(0, 0, f'Error:\n{e}', color='#ff6666', ha='center', va='center', fontsize=9)
        stats_text.set_text(f'Error: {e}')
        fig.canvas.draw_idle()
        return

    total_wps  = 0
    total_dist = 0.0
    for i, wps in enumerate(paths):
        color = DRONE_COLORS[i % len(DRONE_COLORS)]
        xs = [w[0] for w in wps]
        ys = [w[1] for w in wps]
        ax.plot(xs, ys, '-', color=color, linewidth=1.2, alpha=0.7, zorder=2)
        ax.scatter(xs, ys, color=color, s=14, zorder=3, label=f'drone{i}')
        ax.plot(init_pos[i][0], init_pos[i][1], '*', color=color,
                markersize=12, zorder=4, markeredgecolor='white', markeredgewidth=0.5)
        if len(wps) <= 50:
            for j, (x, y) in enumerate(zip(xs, ys)):
                ax.text(x, y, str(j), color=color, fontsize=5,
                        ha='center', va='bottom', zorder=5)
        total_wps  += len(wps)
        total_dist += sum(np.hypot(xs[k+1]-xs[k], ys[k+1]-ys[k])
                          for k in range(len(xs)-1))

    ax.legend(loc='upper right', fontsize=8,
              facecolor='#2a2a3e', edgecolor='#555577', labelcolor='#ccccee')

    all_x = [p[0] for p in area_2d]
    all_y = [p[1] for p in area_2d]
    pad = max(max(all_x)-min(all_x), max(all_y)-min(all_y)) * 0.15
    ax.set_xlim(min(all_x)-pad, max(all_x)+pad)
    ax.set_ylim(min(all_y)-pad, max(all_y)+pad)
    ax.grid(True, color='#444466', linewidth=0.4, alpha=0.6)

    orient_str  = 'auto' if auto_orient else f'{int(orient)}°'
    filter_note = ('filtered + densified' if do_filter
                   else 'RAW (wpSpace visible)')
    per_drone   = total_wps // n_drones if n_drones else total_wps

    # Highlight which density param is active
    wp_note    = '← no effect (filtered)' if do_filter else '← controls density'
    maxwp_note = f'← controls density' if do_filter else '← ignored (filter off)'

    ax.set_title(f'Mode: {filter_note}', color='#ffdd88', pad=6, fontsize=9)

    stats_text.set_text(
        f'streetSpacing  : {street_sp:.2f} m\n'
        f'wpSpace        : {wp_sp:.2f} m  {wp_note}\n'
        f'max_wp_dist    : {max_wp_dist:.2f} m  {maxwp_note}\n'
        f'orientation    : {orient_str}\n'
        f'drones         : {n_drones}\n'
        f'total wps      : {total_wps}  (~{per_drone}/drone)\n'
        f'total path     : {total_dist:.1f} m'
    )

    fig.canvas.draw_idle()


sl_street.on_changed(redraw)
sl_wpspace.on_changed(redraw)
sl_maxwp.on_changed(redraw)
sl_orient.on_changed(redraw)
cb_checks.on_clicked(redraw)
rb_drones.on_clicked(redraw)
rb_preset.on_clicked(redraw)

redraw()
plt.show()
