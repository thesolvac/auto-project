"""
A* search visualization tool.

Records every iteration of the A* algorithm via an `on_step` callback,
then renders the result as either:
  - A series of static snapshots (for the project book), or
  - An animated GIF / MP4 (for the defense slides)

PREREQUISITE
============
Add an `on_step` parameter to the `plan` function in
`autoproject/algorithms/astar.py`. After `heapq.heappop` and before the
goal check, insert:

    if on_step is not None:
        open_cells = {(item[2], item[3]) for item in open_heap}
        on_step({
            'current': (x, y),
            'open': list(open_cells),
            'closed': list(set(g_score.keys()) - open_cells - {(x, y)}),
            'came_from': dict(came_from),
            'g_score': dict(g_score),
            'goal': (gx, gy),
        })

The cost is zero when `on_step=None` (the default), so production calls
to `plan()` are unaffected.

USAGE
=====
From the repo root:

    python tools/visualize_astar.py

Produces three files in docs/figures/:
  - astar_progress.png      4-up snapshots for the book (chapter 15.3.1)
  - astar_demo.gif          animation for the defense slides
  - astar_demo.mp4          same animation, smaller file, plays better in PowerPoint
"""
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyArrowPatch, Rectangle

# Make `autoproject` importable when running from the repo root
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autoproject.algorithms.astar import plan
from autoproject.algorithms.occupancy_grid import OccupancyGrid


# ---------------------------------------------------------------------------
# Colors — chosen so they (a) stay distinguishable in print/grayscale and
# (b) match the project book's overall palette. Tweak freely.
# ---------------------------------------------------------------------------
COLORS = {
    'free':      '#FFFFFF',
    'occupied':  '#2C2C2A',
    'closed':    '#CECBF6',   # purple-100, "explored"
    'open':      '#9FE1CB',   # teal-100,   "frontier"
    'current':   '#FAC775',   # amber-100,  "expanding now"
    'path':      '#185FA5',   # blue-600,   "the answer"
    'start':     '#3B6D11',
    'goal':      '#A32D2D',
    'arrow':     '#666666',
    'arrow_hi':  '#222222',   # the arrow leading to `current`
}


# ---------------------------------------------------------------------------
# render: one A* iteration -> a matplotlib axes
# ---------------------------------------------------------------------------
def render(ax, grid, state, start_cell, goal_cell,
           final_path=None, title='', show_costs=False):
    """Draw one A* iteration onto a matplotlib axes.

    state is the dict produced by the `on_step` callback (see PREREQUISITE).
    Pass final_path=None for in-progress frames; pass the reconstructed
    path on the last frame only.

    show_costs=True overlays g (top-left), h (top-right) and f (bottom-
    center, bold) inside every explored cell. Useful for the algorithm
    deep-dive figure in the book; switch off for the high-level animation.
    """
    ax.clear()
    W, H = grid.width, grid.height

    # Layer 1: base grid (free cells and walls)
    for y in range(H):
        for x in range(W):
            color = COLORS['occupied'] if grid.cell_occupied(x, y) else COLORS['free']
            ax.add_patch(Rectangle((x, y), 1, 1, facecolor=color,
                                    edgecolor='#DDDDDD', linewidth=0.3))

    # Layer 2: closed set (popped from heap, fully explored)
    for (x, y) in state['closed']:
        ax.add_patch(Rectangle((x, y), 1, 1, facecolor=COLORS['closed'],
                                edgecolor='#DDDDDD', linewidth=0.3, alpha=0.9))

    # Layer 3: open set (still in heap, frontier)
    for (x, y) in state['open']:
        ax.add_patch(Rectangle((x, y), 1, 1, facecolor=COLORS['open'],
                                edgecolor='#DDDDDD', linewidth=0.3, alpha=0.95))

    # Layer 4: came_from arrows (the search tree)
    current = state['current']
    for child, parent in state['came_from'].items():
        cx, cy = child[0] + 0.5, child[1] + 0.5
        px, py = parent[0] + 0.5, parent[1] + 0.5
        is_to_current = (child == current)
        ax.add_patch(FancyArrowPatch(
            (px, py), (cx, cy),
            arrowstyle='->', mutation_scale=8 if is_to_current else 7,
            color=COLORS['arrow_hi'] if is_to_current else COLORS['arrow'],
            alpha=0.9 if is_to_current else 0.45,
            linewidth=1.4 if is_to_current else 0.7,
        ))

    # Layer 4.5 (optional): f, g, h text inside explored cells
    if show_costs and state.get('g_score') and state.get('goal'):
        gx, gy = state['goal']
        res = grid.resolution
        for (cx, cy), g in state['g_score'].items():
            if grid.cell_occupied(cx, cy):
                continue
            h = math.hypot(gx - cx, gy - cy) * res
            f = g + h
            ax.text(cx + 0.06, cy + 0.22, f'{g:.1f}',
                    fontsize=5.5, color='#444', ha='left', va='center')
            ax.text(cx + 0.94, cy + 0.22, f'{h:.1f}',
                    fontsize=5.5, color='#444', ha='right', va='center')
            ax.text(cx + 0.50, cy + 0.72, f'{f:.1f}',
                    fontsize=7.5, color='#000', ha='center', va='center',
                    fontweight='bold')

    # Layer 5: current cell — thick border on top
    ccx, ccy = current
    ax.add_patch(Rectangle((ccx, ccy), 1, 1, facecolor=COLORS['current'],
                            edgecolor='black', linewidth=1.8, zorder=5))
    if show_costs and state.get('g_score') and state.get('goal'):
        gx, gy = state['goal']
        res = grid.resolution
        if (ccx, ccy) in state['g_score']:
            g = state['g_score'][(ccx, ccy)]
            h = math.hypot(gx - ccx, gy - ccy) * res
            f = g + h
            ax.text(ccx + 0.06, ccy + 0.22, f'{g:.1f}',
                    fontsize=5.5, color='#444', ha='left', va='center', zorder=6)
            ax.text(ccx + 0.94, ccy + 0.22, f'{h:.1f}',
                    fontsize=5.5, color='#444', ha='right', va='center', zorder=6)
            ax.text(ccx + 0.50, ccy + 0.72, f'{f:.1f}',
                    fontsize=7.5, color='#000', ha='center', va='center',
                    fontweight='bold', zorder=6)

    # Layer 6: final reconstructed path (only on the last frame)
    if final_path:
        xs = [p[0] + 0.5 for p in final_path]
        ys = [p[1] + 0.5 for p in final_path]
        ax.plot(xs, ys, color=COLORS['path'], linewidth=3, zorder=10)

    # Layer 7: start (green circle) + goal (red star)
    sx, sy = start_cell
    gx_cell, gy_cell = goal_cell
    ax.plot(sx + 0.5, sy + 0.5, 'o', color=COLORS['start'],
            markersize=14, markeredgecolor='white', markeredgewidth=1.5,
            zorder=11)
    ax.plot(gx_cell + 0.5, gy_cell + 0.5, '*', color=COLORS['goal'],
            markersize=20, markeredgecolor='white', markeredgewidth=1.5,
            zorder=11)

    # Cosmetics
    ax.set_xlim(0, W)
    ax.set_ylim(H, 0)         # row 0 at top, matching the occupancy grid
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    if title:
        ax.set_title(title, fontsize=11)


# ---------------------------------------------------------------------------
# visualize: run A* once with recording, then save figures
# ---------------------------------------------------------------------------
def visualize(grid, start_xy, goal_xy, mode='animation',
              out_path=None, snapshot_indices=None,
              show_costs=False, fps=12):
    """Run A* once with a recording callback, then write figures.

    mode='snapshots' writes a 4-up PNG suitable for the book.
    mode='animation' writes a GIF and (if ffmpeg is available) an MP4.
    """
    out_path = Path(out_path) if out_path else None

    frames = []
    final_path = plan(grid, start_xy, goal_xy,
                      on_step=lambda s: frames.append({
                          'current': s['current'],
                          'open': list(s['open']),
                          'closed': list(s['closed']),
                          'came_from': dict(s['came_from']),
                          'g_score': dict(s.get('g_score', {})),
                          'goal': s.get('goal'),
                      }))

    if not frames:
        raise RuntimeError("A* returned no frames — is on_step wired up?")
    if final_path is None:
        print("Warning: A* did not find a path. Animating the search anyway.")

    sxy = grid.world_to_cell(*start_xy)
    gxy = grid.world_to_cell(*goal_xy)

    if mode == 'snapshots':
        idxs = snapshot_indices or [
            max(1, len(frames) // 8),
            len(frames) // 3,
            2 * len(frames) // 3,
            len(frames) - 1,
        ]
        fig, axes = plt.subplots(1, len(idxs), figsize=(4 * len(idxs), 4))
        fig.patch.set_facecolor('white')
        if len(idxs) == 1:
            axes = [axes]
        for ax, i in zip(axes, idxs):
            i = min(i, len(frames) - 1)
            show_path = final_path if i == len(frames) - 1 else None
            render(ax, grid, frames[i], sxy, gxy,
                   final_path=show_path,
                   title=f'iteration {i}',
                   show_costs=show_costs)
        plt.tight_layout()
        target = out_path or Path('astar_snapshots.png')
        target.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(target, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close(fig)
        print(f"Wrote {target}")
        return

    # mode == 'animation'
    fig, ax = plt.subplots(figsize=(6, 6))
    fig.patch.set_facecolor('white')

    def update(i):
        show_path = final_path if i == len(frames) - 1 else None
        render(ax, grid, frames[i], sxy, gxy,
               final_path=show_path,
               title=f'A* — iteration {i + 1}/{len(frames)}',
               show_costs=show_costs)

    ani = FuncAnimation(fig, update, frames=len(frames),
                        interval=int(1000 / fps), repeat_delay=2000)

    gif_path = out_path or Path('astar_demo.gif')
    gif_path.parent.mkdir(parents=True, exist_ok=True)
    ani.save(gif_path, writer='pillow', fps=fps)
    print(f"Wrote {gif_path}")

    # Also write MP4 — smaller, plays better in PowerPoint. Needs ffmpeg.
    mp4_path = gif_path.with_suffix('.mp4')
    try:
        ani.save(mp4_path, writer='ffmpeg', fps=fps, dpi=120,
                 extra_args=['-vcodec', 'libx264', '-pix_fmt', 'yuv420p'])
        print(f"Wrote {mp4_path}")
    except (RuntimeError, FileNotFoundError) as e:
        print(f"Skipped MP4 (install ffmpeg to enable): {e}")

    plt.close(fig)


# ---------------------------------------------------------------------------
# CLI entry point — edit the constants below to match your map / start / goal
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    MAP_PATH = Path('config/maps/room.yaml')
    OUT_DIR = Path('docs/figures')
    START_XY = (0.5, 0.5)
    GOAL_XY = (3.0, 2.5)

    if not MAP_PATH.exists():
        print(f"Map not found: {MAP_PATH}")
        print("Edit MAP_PATH / START_XY / GOAL_XY at the bottom of this file.")
        sys.exit(1)

    grid = OccupancyGrid.from_yaml(str(MAP_PATH))

    # 1. Static snapshots for the project book (chapter 15.3.1)
    visualize(grid, START_XY, GOAL_XY,
              mode='snapshots',
              out_path=OUT_DIR / 'astar_progress.png')

    # 2. Animation for the defense slides (overview)
    visualize(grid, START_XY, GOAL_XY,
              mode='animation',
              out_path=OUT_DIR / 'astar_demo.gif')

    # 3. Animation with g/h/f values shown inside each cell (algorithm deep-dive)
    visualize(grid, START_XY, GOAL_XY,
              mode='animation',
              out_path=OUT_DIR / 'astar_demo_fgh.gif',
              show_costs=True, fps=8)
