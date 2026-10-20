import os
import subprocess
import re
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from timeit import default_timer as timer

HEURISTIC_NAMES = {
    2: "Plain RRT* (PL)",
    4: "Node Rejection (NR)",
    5: "Local Biasing (LB)",
    6: "Combined (CO)"
}

def calculate_cost(solution_file):
    try:
        with open(solution_file) as f:
            _ = f.readline()
            points = []
            for line in f:
                parts = [p for p in line.strip().split(",") if p]
                if parts:
                    points.append([float(x) for x in parts])
            if not points:
                return -1.0
            arr = np.array(points)
            if arr.shape[0] <= 1:
                return 0.0
            diffs = np.abs(arr[1:] - arr[:-1])
            wrapped_diffs = np.minimum(diffs, np.abs(2 * np.pi - diffs))
            return float(wrapped_diffs.sum())
    except Exception:
        return -1.0

def run_heuristics_benchmark():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    map_name = "map2.txt"
    dofs = 3
    pairs_file = "../output/benchmark_pairs_map2.csv"
    df_pairs = pd.read_csv(pairs_file)

    print("=====================================================================")
    print("EXTRA CREDIT BENCHMARK: RRT* SAMPLING HEURISTICS (PL, NR, LB, CO)")
    print(f"Map: {map_name} ({dofs} DOFs) | Pairs: {len(df_pairs)} | Timeout: 5.0s")
    print("=====================================================================\n")

    all_records = []

    for pid in [2, 4, 5, 6]:
        pname = HEURISTIC_NAMES[pid]
        print(f"\n--- Testing {pname} (ID: {pid}) ---")
        succ_count = 0

        for idx, row in df_pairs.iterrows():
            trial_id = int(row["trial"])
            start_pos = str(row["start"]).replace(":", ",")
            goal_pos = str(row["goal"]).replace(":", ",")
            sol_file = f"../output/sol_heur_{pid}_t{trial_id}.txt"

            cmd_plan = [
                "../build/planner",
                map_name,
                str(dofs),
                start_pos,
                goal_pos,
                str(pid),
                os.path.basename(sol_file)
            ]
            cmd_ver = [
                "../build/verifier",
                map_name,
                str(dofs),
                start_pos,
                goal_pos,
                sol_file
            ]

            t0 = timer()
            success = False
            time_spent = 0.0
            vertices = -1

            try:
                proc = subprocess.run(cmd_plan, capture_output=True, text=True, timeout=5.0)
                time_spent = timer() - t0
                m = re.search(r"\[STATS\] vertices:\s+(\d+)", proc.stdout)
                if m:
                    vertices = int(m.group(1))

                pv = subprocess.run(cmd_ver, capture_output=True, text=True)
                if proc.returncode == 0 and pv.returncode == 0 and time_spent <= 5.0:
                    success = True
                    succ_count += 1
            except subprocess.TimeoutExpired:
                time_spent = 5.0
                success = False

            cost = calculate_cost(sol_file) if success else np.nan

            all_records.append({
                "planner_id": pid,
                "heuristic": pname,
                "trial": trial_id,
                "success": success,
                "time": time_spent,
                "vertices": vertices,
                "cost": cost
            })

            status = "SUCCESS" if success else "FAIL"
            print(f"  Trial {trial_id:2d}: {status:7s} | Time: {time_spent:6.3f}s | Vertices: {vertices:5d} | Cost: {cost:6.2f} rad")

        print(f">> {pname} Finished: {succ_count}/{len(df_pairs)} succeeded.")

    df = pd.DataFrame(all_records)
    csv_out = "../output/heuristics_benchmark_results.csv"
    df.to_csv(csv_out, index=False)
    print(f"\nSaved results to {csv_out}")

    # Summary Table (Table 2 in Akgun & Stilman)
    summary = []
    for pid in [2, 4, 5, 6]:
        sub = df[df["planner_id"] == pid]
        s_rate = sub["success"].mean() * 100.0
        t_mean = sub["time"].mean()
        t_med = sub["time"].median()
        v_mean = sub[sub["vertices"] >= 0]["vertices"].mean()
        c_mean = sub[sub["success"]]["cost"].mean()
        c_std = sub[sub["success"]]["cost"].std()
        c_min = sub[sub["success"]]["cost"].min()

        summary.append({
            "Method": HEURISTIC_NAMES[pid],
            "Success Rate": f"{s_rate:5.1f}%",
            "Mean Time (s)": f"{t_mean:.3f}",
            "Median Time (s)": f"{t_med:.3f}",
            "Mean Vertices": f"{v_mean:.1f}",
            "Mean Cost (rad)": f"{c_mean:.2f} (±{c_std:.2f})",
            "Min Cost (rad)": f"{c_min:.2f}"
        })

    df_sum = pd.DataFrame(summary)
    print("\n================== TABLE 2: RRT* SAMPLING HEURISTICS COMPARISON ==================")
    print(df_sum.to_string(index=False))
    print("==================================================================================\n")

    # Generate Figure 8: Bar chart of Mean Cost across Heuristics
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4.5), dpi=100)
    
    methods = [HEURISTIC_NAMES[pid] for pid in [2, 4, 5, 6]]
    short_names = ["PL", "NR", "LB", "CO"]
    mean_costs = [df[(df["planner_id"] == pid) & df["success"]]["cost"].mean() for pid in [2, 4, 5, 6]]
    std_costs = [df[(df["planner_id"] == pid) & df["success"]]["cost"].std() for pid in [2, 4, 5, 6]]
    mean_times = [df[df["planner_id"] == pid]["time"].mean() for pid in [2, 4, 5, 6]]

    colors = ['#78909C', '#42A5F5', '#FFA726', '#66BB6A']

    # Cost comparison
    bars1 = ax1.bar(short_names, mean_costs, yerr=std_costs, capsize=5, color=colors, edgecolor='black', alpha=0.85)
    ax1.set_ylabel("Solution Path Cost (radians)", fontsize=11)
    ax1.set_title("Path Cost by Sampling Heuristic\n(Lower is Better)", fontsize=11, fontweight='bold')
    ax1.grid(axis='y', linestyle='--', alpha=0.4)
    for bar in bars1:
        yval = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2.0, yval / 2.0, f"{yval:.2f}", ha='center', va='center', color='white', fontweight='bold')

    # Runtime comparison
    bars2 = ax2.bar(short_names, mean_times, color=colors, edgecolor='black', alpha=0.85)
    ax2.set_ylabel("Mean Planning Time (seconds)", fontsize=11)
    ax2.set_title("Planning Time by Sampling Heuristic", fontsize=11, fontweight='bold')
    ax2.grid(axis='y', linestyle='--', alpha=0.4)
    for bar in bars2:
        yval = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2.0, yval + 0.005, f"{yval:.3f}s", ha='center', va='bottom', fontsize=9)

    plt.tight_layout()
    plot_path = "../output/heuristics_cost_comparison.png"
    plt.savefig(plot_path)
    plt.close(fig)
    print(f"Generated comparison plot: {plot_path}")

if __name__ == "__main__":
    run_heuristics_benchmark()
