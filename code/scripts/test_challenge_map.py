import os
import subprocess
import re
import numpy as np
from timeit import default_timer as timer

def run_planner(map_file, dofs, start_str, goal_str, planner_id, timeout=5.0, goal_bias=0.08):
    cmd = [
        "../build/planner",
        map_file,
        str(dofs),
        start_str,
        goal_str,
        str(planner_id),
        "test_out.txt"
    ]
    env = os.environ.copy()
    env["RRT_GOAL_BIAS"] = str(goal_bias)
    t0 = timer()
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, env=env)
        dt = timer() - t0
        success = (proc.returncode == 0)
        vertices = -1
        m = re.search(r"\[STATS\] vertices:\s+(\d+)", proc.stdout)
        if m:
            vertices = int(m.group(1))
        # Verify
        cmd_ver = [
            "../build/verifier",
            map_file,
            str(dofs),
            start_str,
            goal_str,
            "../output/test_out.txt"
        ]
        pv = subprocess.run(cmd_ver, capture_output=True, text=True)
        if proc.returncode != 0:
            print("PROC ERROR, stderr:", proc.stderr, "stdout:", proc.stdout[:200])
        if pv.returncode != 0:
            print("VERIFIER ERROR, stderr:", pv.stderr, "stdout:", pv.stdout[:200])
        return success, dt, vertices, proc.stdout
    except subprocess.TimeoutExpired:
        return False, timeout, -1, "TIMEOUT"

def find_valid_pairs(map_file, dofs, n_pairs=10):
    cmd = [
        "../build/generate_pairs",
        f"../maps/{map_file}",
        str(dofs),
        str(n_pairs),
        "../output/pairs_challenge.csv"
    ]
    subprocess.run(cmd, capture_output=True, text=True)
    
if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    print("Generating valid collision-free pairs for map_challenge.txt (5 DOFs)...")
    find_valid_pairs("map_challenge.txt", 5, 20)
    
    with open("../output/pairs_challenge.csv") as f:
        lines = f.readlines()[1:]
        
    print(f"Testing Vanilla RRT (Planner 0) on {len(lines)} benchmark pairs...")
    successes = 0
    times = []
    
    for idx, line in enumerate(lines):
        parts = line.strip().split(",")
        trial = parts[0]
        start_str = parts[2].replace(":", ",")
        goal_str = parts[3].replace(":", ",")
        
        ok, t, v, out = run_planner("map_challenge.txt", 5, start_str, goal_str, 0, timeout=5.0)
        times.append(t)
        if ok:
            successes += 1
        print(f"Trial {trial:2s} | {'SUCCESS' if ok else 'TIMEOUT'} | Time: {t:6.3f}s | Vertices: {v:5d}")
        
    print("\n---------------- SUMMARY ----------------")
    print(f"Success Rate: {successes}/{len(lines)} ({successes/len(lines)*100:.1f}%)")
    print(f"Mean Time:    {np.mean(times):.3f}s")
    print(f"Median Time:  {np.median(times):.3f}s")
    print(f"Max Time:     {np.max(times):.3f}s")
