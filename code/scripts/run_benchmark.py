import os
import sys
import subprocess
import re
import numpy as np
import pandas as pd
from timeit import default_timer as timer

PLANNER_NAMES = {
    0: "RRT",
    1: "RRT-Connect",
    2: "RRT*",
    3: "PRM"
}

def calculate_cost(solution_file):
    try:
        with open(solution_file) as f:
            _ = f.readline() # map path
            points = []
            for line in f:
                parts = [p for p in line.strip().split(",") if p]
                if parts:
                    points.append([float(x) for x in parts])
            if not points:
                return -1, 0
            arr = np.array(points)
            num_steps = arr.shape[0]
            if num_steps <= 1:
                return 0.0, num_steps
            diffs = np.abs(arr[1:] - arr[:-1])
            wrapped_diffs = np.minimum(diffs, np.abs(2 * np.pi - diffs))
            cost = float(wrapped_diffs.sum())
            return cost, num_steps
    except Exception as e:
        print(f"Error calculating cost: {e}")
        return -1, 0

def run_benchmark_on_map(map_name, pairs_file, output_csv):
    print(f"\n========================================================")
    print(f"BENCHMARKING ON {map_name} (20 Trials per Planner)")
    print(f"========================================================\n")

    df_pairs = pd.read_csv(pairs_file)
    results = []

    executable_path = "./../build/planner"
    verifier_path = "./../build/verifier"
    tmp_solution_file = "../output/benchmark_tmp.txt"

    for planner_id in [0, 1, 2, 3]:
        planner_name = PLANNER_NAMES[planner_id]
        print(f"--> Running Planner {planner_id}: {planner_name}...")

        for idx, row in df_pairs.iterrows():
            trial_id = int(row["trial"])
            num_dofs = int(row["numDOFs"])
            start_pos = str(row["start"]).replace(":", ",")
            goal_pos = str(row["goal"]).replace(":", ",")

            command_plan = [
                executable_path,
                map_name,
                str(num_dofs),
                start_pos,
                goal_pos,
                str(planner_id),
                "benchmark_tmp.txt" # planner.cpp prepends OUTPUT_DIR
            ]

            command_verify = [
                verifier_path,
                map_name,
                str(num_dofs),
                start_pos,
                goal_pos,
                tmp_solution_file
            ]

            t_start = timer()
            success = False
            time_spent = 0.0
            vertices = -1

            try:
                # 5.0 second timeout budget
                proc = subprocess.run(command_plan, capture_output=True, text=True, timeout=5.5)
                time_spent = timer() - t_start

                # Parse vertices from stdout
                stdout_str = proc.stdout
                match = re.search(r"\[STATS\] vertices:\s+(\d+)", stdout_str)
                if match:
                    vertices = int(match.group(1))

                # Verify solution
                proc_verify = subprocess.run(command_verify, capture_output=True, text=True)
                if proc.returncode == 0 and proc_verify.returncode == 0 and time_spent <= 5.0:
                    success = True

                cost, steps = calculate_cost(tmp_solution_file)
            except subprocess.TimeoutExpired:
                time_spent = 5.0
                success = False
                cost, steps = -1, 0
            except Exception as e:
                time_spent = timer() - t_start
                success = False
                cost, steps = -1, 0

            results.append({
                "planner_id": planner_id,
                "planner_name": planner_name,
                "map": map_name,
                "trial": trial_id,
                "time": time_spent,
                "success": success,
                "vertices": vertices,
                "cost": cost if success else np.nan,
                "steps": steps if success else np.nan
            })

            status_str = "SUCCESS" if success else "FAILED"
            print(f"  Trial {trial_id:2d}: {status_str} | Time: {time_spent:.3f}s | Vertices: {vertices} | Cost: {cost:.2f}")

    df_res = pd.DataFrame(results)
    df_res.to_csv(output_csv, index=False)
    print(f"\nDetailed results saved to {output_csv}\n")

    # Compile Summary Table
    summary = []
    for planner_id in [0, 1, 2, 3]:
        p_name = PLANNER_NAMES[planner_id]
        p_df = df_res[df_res["planner_id"] == planner_id]
        total_trials = len(p_df)
        success_trials = p_df["success"].sum()
        success_rate = (success_trials / total_trials) * 100.0

        avg_time = p_df["time"].mean()
        avg_vertices = p_df[p_df["vertices"] >= 0]["vertices"].mean()
        avg_cost = p_df[p_df["success"]]["cost"].mean()
        std_cost = p_df[p_df["success"]]["cost"].std()

        summary.append({
            "Planner": p_name,
            "Success Rate (%)": f"{success_rate:.1f}%",
            "Avg Planning Time (s)": f"{avg_time:.4f}",
            "Avg Vertices": f"{avg_vertices:.1f}",
            "Avg Path Quality (Cost)": f"{avg_cost:.2f} (±{std_cost:.2f})"
        })

    summary_df = pd.DataFrame(summary)
    print(f"=== SUMMARY FOR {map_name} ===")
    print(summary_df.to_string(index=False))
    print("\n")
    return summary_df

if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    summary_map2 = run_benchmark_on_map("map2.txt", "../output/benchmark_pairs_map2.csv", "../output/benchmark_map2.csv")
    summary_map1 = run_benchmark_on_map("map1.txt", "../output/benchmark_pairs_map1.csv", "../output/benchmark_map1.csv")
