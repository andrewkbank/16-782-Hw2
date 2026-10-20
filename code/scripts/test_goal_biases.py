import os
import sys
import subprocess
import re
import numpy as np
import pandas as pd
from timeit import default_timer as timer

MAP_CONFIGS = [
    ("map1.txt", 5, "../output/benchmark_pairs_map1.csv"),
    ("map2.txt", 3, "../output/benchmark_pairs_map2.csv"),
    ("map3.txt", 4, "../output/benchmark_pairs_map3.csv"),
    ("map4.txt", 5, "../output/benchmark_pairs_map4.csv"),
    ("map5.txt", 4, "../output/benchmark_pairs_map5.csv")
]

BIAS_LEVELS = [0.0, 0.02, 0.05, 0.10, 0.20, 0.35, 0.50]

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
                return -1, 0
            arr = np.array(points)
            if arr.shape[0] <= 1:
                return 0.0, arr.shape[0]
            diffs = np.abs(arr[1:] - arr[:-1])
            wrapped_diffs = np.minimum(diffs, np.abs(2 * np.pi - diffs))
            return float(wrapped_diffs.sum()), arr.shape[0]
    except Exception:
        return -1, 0

def run_goal_bias_experiment():
    executable_path = "./../build/planner"
    verifier_path = "./../build/verifier"
    tmp_sol = "../output/bias_tmp.txt"

    all_results = []

    print("=====================================================================")
    print("TESTING MULTIPLE GOAL BIASES ON VANILLA RRT")
    print(f"Biases: {BIAS_LEVELS}")
    print("Across Maps 1 to 5 (100 total test cases per bias level)")
    print("=====================================================================\n")

    for bias in BIAS_LEVELS:
        print(f"\n>>> TESTING GOAL BIAS = {bias * 100:.0f}% ({bias}) <<<")
        custom_env = os.environ.copy()
        custom_env["RRT_GOAL_BIAS"] = str(bias)

        bias_times = []
        bias_success = []
        bias_vertices = []
        bias_costs = []

        for map_name, num_dofs, pairs_file in MAP_CONFIGS:
            df_pairs = pd.read_csv(pairs_file)
            map_successes = 0

            for idx, row in df_pairs.iterrows():
                trial_id = int(row["trial"])
                start_pos = str(row["start"]).replace(":", ",")
                goal_pos = str(row["goal"]).replace(":", ",")

                cmd_plan = [
                    executable_path,
                    map_name,
                    str(num_dofs),
                    start_pos,
                    goal_pos,
                    "0", # RRT
                    "bias_tmp.txt"
                ]

                cmd_verify = [
                    verifier_path,
                    map_name,
                    str(num_dofs),
                    start_pos,
                    goal_pos,
                    tmp_sol
                ]

                t0 = timer()
                success = False
                time_spent = 0.0
                vertices = -1

                try:
                    proc = subprocess.run(
                        cmd_plan, capture_output=True, text=True, timeout=5.0, env=custom_env
                    )
                    time_spent = timer() - t0

                    m = re.search(r"\[STATS\] vertices:\s+(\d+)", proc.stdout)
                    if m:
                        vertices = int(m.group(1))

                    pv = subprocess.run(cmd_verify, capture_output=True, text=True)
                    if proc.returncode == 0 and pv.returncode == 0 and time_spent <= 5.0:
                        success = True
                        map_successes += 1

                    cost, steps = calculate_cost(tmp_sol)
                except subprocess.TimeoutExpired:
                    time_spent = 5.0
                    success = False
                    cost, steps = -1, 0
                except Exception:
                    time_spent = timer() - t0
                    success = False
                    cost, steps = -1, 0

                all_results.append({
                    "bias": bias,
                    "map": map_name,
                    "dofs": num_dofs,
                    "trial": trial_id,
                    "time": time_spent,
                    "success": success,
                    "vertices": vertices,
                    "cost": cost if success else np.nan,
                    "steps": steps if success else np.nan
                })

                bias_times.append(time_spent)
                bias_success.append(success)
                if vertices >= 0:
                    bias_vertices.append(vertices)
                if success:
                    bias_costs.append(cost)

            print(f"  {map_name} ({num_dofs} DOFs): {map_successes}/{len(df_pairs)} succeeded")

        avg_time = np.mean(bias_times)
        succ_rate = np.mean(bias_success) * 100.0
        avg_vert = np.mean(bias_vertices) if bias_vertices else 0
        avg_cost = np.mean(bias_costs) if bias_costs else 0
        print(f"  --> OVERALL {bias*100:.0f}% Bias: Success: {succ_rate:.1f}% | Avg Time: {avg_time:.4f}s | Avg Vertices: {avg_vert:.1f} | Avg Cost: {avg_cost:.2f}")

    df_all = pd.DataFrame(all_results)
    out_csv = "../output/goal_bias_experiment_results.csv"
    df_all.to_csv(out_csv, index=False)
    print(f"\nSaved raw data to {out_csv}")

    # Summary table by bias
    summary = []
    for bias in BIAS_LEVELS:
        sub = df_all[df_all["bias"] == bias]
        succ = sub["success"].mean() * 100.0
        t_mean = sub["time"].mean()
        t_median = sub["time"].median()
        v_mean = sub[sub["vertices"] >= 0]["vertices"].mean()
        c_mean = sub[sub["success"]]["cost"].mean()
        c_std = sub[sub["success"]]["cost"].std()

        summary.append({
            "Goal Bias": f"{bias*100:4.1f}%",
            "Success Rate": f"{succ:5.1f}%",
            "Mean Time (s)": f"{t_mean:.4f}",
            "Median Time (s)": f"{t_median:.4f}",
            "Mean Vertices": f"{v_mean:.1f}",
            "Mean Cost": f"{c_mean:.2f} (±{c_std:.2f})"
        })

    df_sum = pd.DataFrame(summary)
    print("\n================== GOAL BIAS COMPARISON TABLE ==================")
    print(df_sum.to_string(index=False))
    print("================================================================\n")
    return df_sum

if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    run_goal_bias_experiment()
