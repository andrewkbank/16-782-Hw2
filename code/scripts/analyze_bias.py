import pandas as pd
import numpy as np

df = pd.read_csv('../output/goal_bias_experiment_results.csv')

print("=== OVERALL METRICS BY GOAL BIAS ===")
for b in sorted(df['bias'].unique()):
    sub = df[df['bias'] == b]
    succ = sub['success'].mean() * 100
    t_mean = sub['time'].mean()
    t_med = sub['time'].median()
    v_mean = sub[sub['vertices'] >= 0]['vertices'].mean()
    c_mean = sub[sub['success']]['cost'].mean()
    print(f"Bias {b*100:4.1f}% | Success: {succ:5.1f}% | Mean Time: {t_mean:.4f}s | Median Time: {t_med:.4f}s | Vertices: {v_mean:5.1f} | Cost: {c_mean:.2f}")

print("\n=== PER-MAP METRICS BREAKDOWN ===")
for m in sorted(df['map'].unique()):
    sub_m = df[df['map'] == m]
    dofs = sub_m.iloc[0]['dofs']
    print(f"\n--- {m} ({dofs} DOFs) ---")
    for b in [0.0, 0.05, 0.10, 0.20, 0.35, 0.50]:
        bsub = sub_m[sub_m['bias'] == b]
        succ = bsub['success'].mean() * 100
        t_mean = bsub['time'].mean()
        t_med = bsub['time'].median()
        c_mean = bsub[bsub['success']]['cost'].mean() if not bsub[bsub['success']].empty else 0
        print(f"  Bias {b*100:4.1f}%: Succ: {succ:5.1f}% | Mean Time: {t_mean:.4f}s | Median Time: {t_med:.4f}s | Cost: {c_mean:.2f}")
