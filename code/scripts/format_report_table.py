import os
import numpy as np
import pandas as pd

MAP_CONFIGS = [
    {"name": "map1.txt", "dofs": 5, "label": "Map 1 (5 DOFs)"},
    {"name": "map2.txt", "dofs": 3, "label": "Map 2 (3 DOFs)"},
    {"name": "map3.txt", "dofs": 4, "label": "Map 3 (4 DOFs)"},
    {"name": "map4.txt", "dofs": 5, "label": "Map 4 (5 DOFs)"},
    {"name": "map5.txt", "dofs": 4, "label": "Map 5 (4 DOFs)"},
    {"name": "map_challenge.txt", "dofs": 5, "label": "Map Challenge (5 DOFs)"}
]

PLANNERS = [
    (0, "RRT", "Vanilla RRT"),
    (1, "RRT-Connect", "RRT-Connect"),
    (2, "RRT*", "RRT*"),
    (3, "PRM", "PRM"),
    (6, "RRT* (CO)", "RRT* (Combined Heuristic)")
]

def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    df_all = pd.read_csv("../output/comprehensive_report_data.csv")

    summary_rows = []
    for m_cfg in MAP_CONFIGS:
        map_name = m_cfg["name"]
        label = m_cfg["label"]
        dofs = m_cfg["dofs"]

        for pid, short_name, long_name in PLANNERS:
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
                "LongName": long_name,
                "Success Rate": f"{succ_rate:5.1f}%",
                "Mean Time (s)": f"{t_mean:.3f}",
                "Time (Mean ± Std)": f"{t_mean:.3f} ± {t_std:.3f}",
                "Median Time (s)": f"{t_med:.3f}",
                "Max Time (s)": f"{t_max:.3f}",
                "Cost (Mean ± Std)": f"{c_mean:5.2f} ± {c_std:4.2f}" if not np.isnan(c_mean) else "N/A"
            })

    df_sum = pd.DataFrame(summary_rows)

    # 1. Print formatted plaintext/markdown
    print("\n" + "="*95)
    print("ALL MAPS COMPREHENSIVE BENCHMARK TABLE")
    print("="*95)
    header = f"{'Map':<24} | {'Algorithm':<14} | {'DOFs':<4} | {'Success':<8} | {'Mean Time':<10} | {'Median':<8} | {'Max Time':<9} | {'Cost (Mean ± Std)':<18}"
    print(header)
    print("-" * len(header))
    for r in summary_rows:
        print(f"{r['Map']:<24} | {r['Algorithm']:<14} | {r['DOFs']:<4} | {r['Success Rate']:<8} | {r['Mean Time (s)'] + 's':<10} | {r['Median Time (s)'] + 's':<8} | {r['Max Time (s)'] + 's':<9} | {r['Cost (Mean ± Std)'] + ' rad':<18}")

    # 2. LaTeX Table for Overleaf
    latex_code = generate_latex(summary_rows)
    with open("../output/report_table.tex", "w") as f:
        f.write(latex_code)
    print("\n" + "="*95)
    print("SAVED OVERLEAF LATEX TABLE TO ../output/report_table.tex")
    print("="*95)
    print(latex_code)

def generate_latex(summary_rows):
    lines = []
    lines.append("% ---------------------------------------------------------------------------------")
    lines.append("% CMU 16-782 Homework 2: Comprehensive Planner Benchmark Table")
    lines.append("% Instructions: Paste directly into your Overleaf report.")
    lines.append("% Ensure \\usepackage{booktabs} and \\usepackage{multirow} are in your preamble.")
    lines.append("% ---------------------------------------------------------------------------------")
    lines.append("\\begin{table*}[t]")
    lines.append("\\centering")
    lines.append("\\small")
    lines.append("\\caption{Comprehensive performance comparison of all five motion planning implementations across all six benchmark environments (20 random collision-free queries per map, 5.0\\,s cutoff).}")
    lines.append("\\label{tab:comprehensive_planner_results}")
    lines.append("\\begin{tabular}{llcccccc}")
    lines.append("\\toprule")
    lines.append("\\textbf{Environment} & \\textbf{Algorithm} & \\textbf{DOFs} & \\textbf{Success Rate} & \\textbf{Mean Time (s)} & \\textbf{Median Time (s)} & \\textbf{Max Time (s)} & \\textbf{Path Cost (rad)} \\\\")
    lines.append("\\midrule")

    cur_map = ""
    for r in summary_rows:
        map_label = r["Map"]
        alg = r["Algorithm"]
        dofs = r["DOFs"]
        succ = r["Success Rate"].replace("%", "\\%")
        t_mean = r["Mean Time (s)"]
        t_med = r["Median Time (s)"]
        t_max = r["Max Time (s)"]
        cost = r["Cost (Mean ± Std)"].replace("±", "$\\pm$")

        if alg == "RRT":
            alg_tex = "Vanilla RRT"
        elif alg == "RRT-Connect":
            alg_tex = "RRT-Connect"
        elif alg == "RRT*":
            alg_tex = "Vanilla RRT*"
        elif alg == "PRM":
            alg_tex = "PRM"
        elif alg == "RRT* (CO)":
            alg_tex = "\\textbf{RRT* (Combined Heuristic)}"
        else:
            alg_tex = alg

        if map_label != cur_map:
            if cur_map != "":
                lines.append("\\midrule")
            cur_map = map_label
            clean_map = map_label.split(" (")[0]
            lines.append(f"\\multirow{{5}}{{*}}{{{clean_map}}} & {alg_tex} & {dofs} & {succ} & {t_mean} & {t_med} & {t_max} & {cost} \\\\")
        else:
            lines.append(f" & {alg_tex} & {dofs} & {succ} & {t_mean} & {t_med} & {t_max} & {cost} \\\\")

    lines.append("\\bottomrule")
    lines.append("\\end{tabular}")
    lines.append("\\end{table*}")
    return "\n".join(lines)

if __name__ == "__main__":
    main()
