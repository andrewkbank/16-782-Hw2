import os
import sys
import subprocess
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

MAP_SPECS = [
    {
        "map_name": "map1.txt",
        "dofs": 5,
        "pairs_file": "../output/benchmark_pairs_map1.csv",
        "trial_id": 0,
        "label": "Map 1 (5 DOFs - Open with Boundary Obstacles)"
    },
    {
        "map_name": "map2.txt",
        "dofs": 3,
        "pairs_file": "../output/benchmark_pairs_map2.csv",
        "trial_id": 0,
        "label": "Map 2 (3 DOFs - Narrow Slot & Pocket)"
    },
    {
        "map_name": "map3.txt",
        "dofs": 4,
        "pairs_file": "../output/benchmark_pairs_map3.csv",
        "trial_id": 0,
        "label": "Map 3 (4 DOFs - Staggered S-Corridor)"
    },
    {
        "map_name": "map4.txt",
        "dofs": 5,
        "pairs_file": "../output/benchmark_pairs_map4.csv",
        "trial_id": 1,
        "label": "Map 4 (5 DOFs - Cluttered Forest Pillars)"
    },
    {
        "map_name": "map5.txt",
        "dofs": 4,
        "pairs_file": "../output/benchmark_pairs_map5.csv",
        "trial_id": 0,
        "label": "Map 5 (4 DOFs - Central Floating Block)"
    },
    {
        "map_name": "map_challenge.txt",
        "dofs": 5,
        "pairs_file": "../output/pairs_challenge.csv",
        "trial_id": 14,
        "label": "Map Challenge (5 DOFs - Double-Slit & Central Baffle)"
    }
]

PLANNERS = [
    (0, "rrt", "Vanilla RRT"),
    (1, "rrt_connect", "RRT-Connect"),
    (2, "rrt_star", "RRT*"),
    (3, "prm", "PRM")
]

def read_map(map_path):
    with open(map_path) as f:
        h = int(f.readline().split()[1])
        w = int(f.readline().split()[1])
        data = np.array([line.strip().split() for line in f if line.strip()]).astype(int)
    return data.reshape((w, h))

def render_gif(map_data, solution, gif_path, title_text, fps=12):
    # Subsample solution if too long to keep GIF snappy (< 60 frames)
    n_pts = len(solution)
    if n_pts > 60:
        step = int(np.ceil(n_pts / 60))
        indices = list(range(0, n_pts, step))
        if indices[-1] != n_pts - 1:
            indices.append(n_pts - 1)
        sub_sol = solution[indices]
    else:
        sub_sol = solution

    fig, ax = plt.subplots(figsize=(5, 5), dpi=80)
    ax.imshow(1 - map_data.T, cmap=plt.cm.gray)
    ax.set_title(title_text, fontsize=9)
    ax.set_xticks([])
    ax.set_yticks([])

    x_base = map_data.shape[0] // 2
    y_base = 0
    link_len = 10

    line_arm, = ax.plot([], [], color='#00E5FF', linewidth=3, solid_capstyle='round')
    pt_base, = ax.plot([], [], 'o', color='#76FF03', markersize=8)
    pt_ee, = ax.plot([], [], 'o', color='#FF1744', markersize=6)
    trail_x, trail_y = [], []
    line_trail, = ax.plot([], [], '--', color='#FF80AB', alpha=0.5, linewidth=1.5)

    def init():
        line_arm.set_data([], [])
        pt_base.set_data([], [])
        pt_ee.set_data([], [])
        line_trail.set_data([], [])
        return line_arm, pt_base, pt_ee, line_trail

    def update(frame_idx):
        pose = sub_sol[frame_idx]
        xs = [x_base]
        ys = [y_base]
        for k in range(len(pose)):
            xs.append(xs[k] + link_len * np.cos(pose[k]))
            ys.append(ys[k] + link_len * np.sin(pose[k]))
        line_arm.set_data(xs, ys)
        pt_base.set_data([xs[0]], [ys[0]])
        pt_ee.set_data([xs[-1]], [ys[-1]])
        trail_x.append(xs[-1])
        trail_y.append(ys[-1])
        line_trail.set_data(trail_x, trail_y)
        return line_arm, pt_base, pt_ee, line_trail

    ani = FuncAnimation(fig, update, frames=len(sub_sol), init_func=init, blit=True, repeat=True)
    ani.save(gif_path, writer=PillowWriter(fps=fps))
    plt.close(fig)

def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    out_dir = "../output/gifs"
    os.makedirs(out_dir, exist_ok=True)

    print("=====================================================================")
    print("GENERATING ANIMATED SOLUTION GIFS FOR ALL 4 PLANNERS ACROSS 6 MAPS")
    print(f"Output directory: {os.path.abspath(out_dir)}")
    print("=====================================================================\n")

    summary_records = []

    for spec in MAP_SPECS:
        map_name = spec["map_name"]
        dofs = spec["dofs"]
        trial_id = spec["trial_id"]
        label = spec["label"]
        map_file = f"../maps/{map_name}"
        map_data = read_map(map_file)

        df = pd.read_csv(spec["pairs_file"])
        row = df[df["trial"] == trial_id].iloc[0]
        start_str = str(row["start"]).replace(":", ",")
        goal_str = str(row["goal"]).replace(":", ",")

        print(f"\nProcessing {label} (Trial {trial_id}):")

        for pid, pcode, pname in PLANNERS:
            sol_filename = f"sol_{map_name.replace('.txt','')}_{pcode}.txt"
            sol_path = f"../output/{sol_filename}"
            gif_filename = f"{map_name.replace('.txt','')}_{pcode}.gif"
            gif_path = f"{out_dir}/{gif_filename}"

            cmd = [
                "../build/planner",
                map_name,
                str(dofs),
                start_str,
                goal_str,
                str(pid),
                sol_filename
            ]
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=10.0)

            # Check verification
            cmd_ver = [
                "../build/verifier",
                map_name,
                str(dofs),
                start_str,
                goal_str,
                sol_path
            ]
            pv = subprocess.run(cmd_ver, capture_output=True, text=True)
            valid = (proc.returncode == 0 and pv.returncode == 0)

            if valid and os.path.exists(sol_path):
                sol_pts = []
                with open(sol_path) as f:
                    _ = f.readline()
                    for line in f:
                        parts = [p for p in line.strip().split(",") if p]
                        if parts:
                            sol_pts.append([float(x) for x in parts])
                sol_arr = np.array(sol_pts)

                title = f"{map_name} ({dofs}D) - {pname} | Steps: {len(sol_arr)}"
                render_gif(map_data, sol_arr, gif_path, title)
                gif_size_kb = os.path.getsize(gif_path) / 1024.0
                print(f"  [OK] {pname:12s} -> {gif_filename} ({gif_size_kb:.1f} KB, {len(sol_arr)} steps)")
                summary_records.append({
                    "Map": map_name,
                    "DOFs": dofs,
                    "Planner": pname,
                    "Status": "VALID",
                    "Steps": len(sol_arr),
                    "GIF": gif_filename,
                    "Path": gif_path
                })
            else:
                print(f"  [FAILED] {pname:12s} could not find valid solution")
                summary_records.append({
                    "Map": map_name,
                    "DOFs": dofs,
                    "Planner": pname,
                    "Status": "FAILED",
                    "Steps": 0,
                    "GIF": "N/A",
                    "Path": "N/A"
                })

    print("\nAll GIF generation complete!")

if __name__ == "__main__":
    main()
