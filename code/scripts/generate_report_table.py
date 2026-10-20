import os
import subprocess
import re
import numpy as np
import pandas as pd
from timeit import default_timer as timer

MAP_CONFIGS = [
    {"name": "map1.txt", "dofs": 5, "pairs": "../output/benchmark_pairs_map1.csv", "label": "Map 1 (5 DOFs)"},
    {"name": "map2.txt", "dofs": 3, "pairs": "../output/benchmark_pairs_map2.csv", "label": "Map 2 (3 DOFs)"},
    {"name": "map3.txt", "dofs": 4, "pairs": "../output/benchmark_pairs_map3.csv", "label": "Map 3 (4 DOFs)"},
    {"name": "map4.txt", "dofs": 5, "pairs": "../output/benchmark_pairs_map4.csv", "label": "Map 4 (5 DOFs)"},
    {"name": "map5.txt", "dofs": 4, "pairs": "../output/benchmark_pairs_map5.csv", "label": "Map 5 (4 DOFs)"},
    {"name": "map_challenge.txt", "dofs": 5, "pairs": "../output/pairs_challenge.csv", "label": "Map Challenge (5 DOFs)"}
]

PLANNERS = [
    (0, "RRT", "Vanilla RRT"),
    (1, "RRT-Connect", "RRT-Connect"),
    (2, "RRT*", "RRT*"),
    (3, "PRM", "PRM"),
    (6, "RRT* (CO)", "RRT* Combined Heuristic")
]

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

def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    executable_path = "../build/planner"
    verifier_path = "../build/verifier"
    tmp_sol = "../output/report_tmp.txt"

    all_results = []

    print("=====================================================================")
    print("COLLECTING FULL DATA FOR OVERLEAF REPORT TABLE")
    print("Maps: 6 Maps | Planners: RRT, RRT-Connect, RRT*, PRM, RRT* (CO)")
    print("=====================================================================\n")

    for m_cfg in MAP_CONFIGS:
        map_name = m_cfg["name"]
        num_dofs = m_cfg["dofs"]
        label = m_cfg["label"]
        df_pairs = pd.read_csv(m_cfg["pairs"])

        print(f"\n>>> Benchmarking {label} ({len(df_pairs)} trials per planner) <<<")

        for pid, short_name, long_name in PLANNERS:
            p_times = []
            p_success = []
            p_costs = []

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
                    str(pid),
                    "report_tmp.txt"
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

                try:
                    proc = subprocess.run(cmd_plan, capture_output=True, text=True, timeout=5.0)
                    time_spent = timer() - t0
                    pv = subprocess.run(cmd_verify, capture_output=True, text=True)

                    if proc.returncode == 0 and pv.returncode == 0 and time_spent <= 5.0:
                        success = True
                    cost = calculate_cost(tmp_sol) if success else np.nan
                except subprocess.TimeoutExpired:
                    time_spent = 5.0
                    success = False
                    cost = np.nan
                except Exception:
                    time_spent = timer() - t0
                    success = False
                    cost = np.nan

                p_times.append(time_spent)
                p_success.append(success)
                if success:
                    p_costs.append(cost)

                all_results.append({
                    "map": map_name,
                    "map_label": label,
                    "dofs": num_dofs,
                    "planner_id": pid,
                    "planner_name": short_name,
                    "trial": trial_id,
                    "success": success,
                    "time": time_spent,
                    "cost": cost
                })

            succ_rate = np.mean(p_success) * 100.0
            mean_t = np.mean(p_times)
            mean_c = np.mean(p_costs) if p_costs else 0.0
            print(f"  {short_name:12s}: Success: {succ_rate:5.1f}% | Time: {mean_t:.4f}s | Cost: {mean_c:.2f} rad")

    df_all = pd.DataFrame(all_results)
    df_all.to_csv("../output/comprehensive_report_data.csv", index=False)
    print("\nSaved full dataset to ../output/comprehensive_report_data.csv")

    # Build Summary Table
    summary_rows = []
    for m_cfg in MAP_CONFIGS:
        map_name = m_cfg["name"]
        label = m_cfg["label"]
        dofs = m_cfg["dofs"]

        for pid, short_name, _ in PLANNERS:
            sub = df_all[(df_all["map"] == map_name) & (df_all["planner_id"] == pid)]
            succ_rate = sub["success"].mean() * 100.0
            t_mean = sub["time"].mean()
            t_std = sub["time"].std()
            t_med = sub["time"].median()
            t_max = sub["time"].max()

            succ_sub = sub[sub["success"]]
            if not succ_sub.empty:
                c_mean = succ_sub["cost"].mean()
                c_std = succ_sub["cost"].std()
            else:
                c_mean = np.nan
                c_std = np.nan

            summary_rows.append({
                "Map": label,
                "DOFs": dofs,
                "Algorithm": short_name,
                "Success Rate (%)": f"{succ_rate:.1f}%",
                "Mean Time (s)": f"{t_mean:.3f}",
                "Time (Mean ± Std)": f"{t_mean:.3f} ± {t_std:.3f}",
                "Median Time (s)": f"{t_med:.3f}",
                "Max Time (s)": f"{t_max:.3f}",
                "Cost (Mean ± Std)": f"{c_mean:.2f} ± {c_std:.2f}" if not np.isnan(c_mean) else "N/A"
            })

    df_sum = pd.DataFrame(summary_rows)

    # Output Markdown
    print("\n" + "="*80)
    print("MARKDOWN TABLE")
    print("="*80)
    print(df_sum.to_markdown(index=False))

    # Generate LaTeX code
    latex_code = generate_latex(summary_rows)
    with open("../output/report_table.tex", "w") as f:
        f.write(latex_code)
    print("\n" + "="*80)
    print("SAVED LATEX CODE TO ../output/report_table.tex")
    print("="*80)
    print(latex_code)

def generate_latex(summary_rows):
    lines = []
    lines.append("% -----------------------------------------------------------------------------")
    lines.append("% LaTeX Table for CMU 16-782 Homework 2 Report")
    lines.append("% Requires \\usepackage{booktabs} and \\usepackage{multirow} in your preamble.")
    lines.append("% -----------------------------------------------------------------------------")
    lines.append("\\begin{table*}[t]")
    lines.append("\\centering")
    lines.append("\\small")
    lines.append("\\caption{Comprehensive performance comparison across all benchmark environments. All planners evaluated over 20 random collision-free queries per map under a 5.0\\,s timeout.}")
    lines.append("\\label{tab:planner_benchmark}")
    lines.append("\\begin{tabular}{llrcccc}")
    lines.append("\\toprule")
    lines.append("\\textbf{Environment} & \\textbf{Algorithm} & \\textbf{DOFs} & \\textbf{Success Rate} & \\textbf{Mean Time (s)} & \\textbf{Median Time (s)} & \\textbf{Path Cost (rad)} \\\\")
    lines.append("\\midrule")

    cur_map = ""
    for i, r in enumerate(summary_rows):
        map_label = r["Map"]
        alg = r["Algorithm"]
        dofs = r["DOFs"]
        succ = r["Success Rate (%)"]
        t_mean = r["Mean Time (s)"]
        t_med = r["Median Time (s)"]
        cost = r["Cost (Mean ± Std)"].replace("±", "$\\pm$")

        # Formatting algorithm names
        if alg == "RRT":
            alg_tex = "Vanilla RRT"
        elif alg == "RRT-Connect":
            alg_tex = "\\textbf{RRT-Connect}"
        elif alg == "RRT*":
            alg_tex = "RRT*"
        elif alg == "PRM":
            alg_tex = "PRM"
        elif alg == "RRT* (CO)":
            alg_tex = "RRT* (Combined Heuristic)"
        else:
            alg_tex = alg

        if map_label != cur_map:
            if cur_map != "":
                lines.append("\\midrule")
            cur_map = map_label
            # First row of a map section
            clean_map = map_label.split(" (")[0]
            lines.append(f"\\multirow{{5}}{{*}}{{{clean_map}}} & {alg_tex} & {dofs} & {succ} & {t_mean} & {t_med} & {cost} \\\\")
        else:
            lines.append(f" & {alg_tex} & {dofs} & {succ} & {t_mean} & {t_med} & {cost} \\\\")

    lines.append("\\bottomrule")
    lines.append("\\end{tabular}")
    lines.append("\\end{table*}")
    return "\n".join(lines)

if __name__ == "__main__":
    main()
