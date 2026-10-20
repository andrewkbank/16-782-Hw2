import os
import subprocess
import re
import numpy as np
import pandas as pd
from timeit import default_timer as timer

PLANNER_NAMES = {
    0: "Vanilla RRT",
    1: "RRT-Connect",
    2: "RRT*",
    3: "PRM"
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

def run_challenge_benchmark():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    map_file = "map_challenge.txt"
    pairs_file = "../output/pairs_challenge.csv"
    dofs = 5
    
    df_pairs = pd.read_csv(pairs_file)
    results = []

    print("=====================================================================")
    print("BENCHMARKING ALL 4 PLANNERS ON EXTREME CHALLENGE MAP (5 DOFs)")
    print("Map: map_challenge.txt (Double-Slit Barriers + Central Baffle)")
    print(f"Total Test Cases: {len(df_pairs)} Pairs | Timeout: 5.0s")
    print("=====================================================================\n")

    for pid in [0, 1, 2, 3]:
        pname = PLANNER_NAMES[pid]
        print(f"\n--- Testing Planner {pid}: {pname} ---")
        succ_count = 0
        p_times = []
        p_costs = []
        p_verts = []

        for idx, row in df_pairs.iterrows():
            trial_id = int(row["trial"])
            start_pos = str(row["start"]).replace(":", ",")
            goal_pos = str(row["goal"]).replace(":", ",")
            sol_file = f"../output/sol_p{pid}_t{trial_id}.txt"

            cmd_plan = [
                "../build/planner",
                map_file,
                str(dofs),
                start_pos,
                goal_pos,
                str(pid),
                os.path.basename(sol_file)
            ]
            cmd_ver = [
                "../build/verifier",
                map_file,
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

            p_times.append(time_spent)
            if success:
                p_costs.append(cost)
            if vertices >= 0:
                p_verts.append(vertices)

            results.append({
                "planner_id": pid,
                "planner_name": pname,
                "trial": trial_id,
                "success": success,
                "time": time_spent,
                "vertices": vertices,
                "cost": cost
            })

            status_str = "SUCCESS" if success else "TIMEOUT/FAIL"
            print(f"  Trial {trial_id:2d}: {status_str:12s} | Time: {time_spent:6.3f}s | Vertices: {vertices:5d} | Cost: {cost:6.2f} rad")

        mean_t = np.mean(p_times)
        med_t = np.median(p_times)
        succ_rate = succ_count / len(df_pairs) * 100.0
        mean_c = np.mean(p_costs) if p_costs else 0.0
        print(f">> {pname} Summary: Success {succ_rate:.1f}% | Mean Time: {mean_t:.3f}s | Median Time: {med_t:.3f}s | Mean Cost: {mean_c:.2f} rad")

    df_res = pd.DataFrame(results)
    df_res.to_csv("../output/challenge_benchmark_all.csv", index=False)
    print("\nSaved full results to ../output/challenge_benchmark_all.csv")

    summary = []
    for pid in [0, 1, 2, 3]:
        sub = df_res[df_res["planner_id"] == pid]
        s_rate = sub["success"].mean() * 100.0
        t_mean = sub["time"].mean()
        t_med = sub["time"].median()
        t_max = sub["time"].max()
        c_mean = sub[sub["success"]]["cost"].mean()
        c_std = sub[sub["success"]]["cost"].std()

        summary.append({
            "Planner": PLANNER_NAMES[pid],
            "Success Rate": f"{s_rate:5.1f}%",
            "Mean Time (s)": f"{t_mean:.3f}",
            "Median Time (s)": f"{t_med:.3f}",
            "Max Time (s)": f"{t_max:.3f}",
            "Mean Path Cost (rad)": f"{c_mean:.2f} (±{c_std:.2f})"
        })

    df_sum = pd.DataFrame(summary)
    print("\n================== CHALLENGE MAP COMPARISON TABLE ==================")
    print(df_sum.to_string(index=False))
    print("====================================================================\n")

if __name__ == "__main__":
    run_challenge_benchmark()
