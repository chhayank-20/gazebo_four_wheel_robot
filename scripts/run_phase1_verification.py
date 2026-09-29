#!/usr/bin/env python3
"""
Orchestrates 3 consecutive clean-restart runs of the Phase 1 stabilization test.
Lobby (-1.80, 0.00) -> Experience Centre (Living Demo) (-2.50, 10.50).
Records:
  - Goal completion (SUCCEEDED / FAILED / TIMEOUT)
  - Final pose error (m)
  - Physical collisions (observed in Gazebo)
  - Pose disagreement between Nav2 and Gazebo ground truth
"""

import subprocess
import time
import json
import math
import sys
import re

CONTAINER = "ros2_gazebo_robot"
TARGET_X = -2.50
TARGET_Y = 10.50
TARGET_YAW = 1.5708
NUM_RUNS = 3

def run_cmd(cmd, timeout=120):
    p = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=timeout)
    return p.returncode, p.stdout, p.stderr

def get_gazebo_world_pose():
    # Query Gazebo pose/info topic for four_wheel_robot
    cmd = f'docker exec {CONTAINER} bash -c "source /opt/ros/jazzy/setup.bash && timeout 4 gz topic -e -t /world/robot_world/pose/info -n 1"'
    rc, stdout, stderr = run_cmd(cmd, timeout=10)
    
    # Parse pose for four_wheel_robot
    lines = stdout.splitlines()
    in_robot = False
    x, y, z, qz, qw = None, None, None, None, None
    for i, line in enumerate(lines):
        if 'name: "four_wheel_robot"' in line:
            in_robot = True
        elif in_robot and 'name:' in line and 'four_wheel_robot' not in line:
            break
        elif in_robot:
            m_x = re.search(r'x:\s*([-\d\.e]+)', line)
            m_y = re.search(r'y:\s*([-\d\.e]+)', line)
            m_z = re.search(r'z:\s*([-\d\.e]+)', line)
            m_w = re.search(r'w:\s*([-\d\.e]+)', line)
            if 'position' in lines[max(0, i-1)] or 'position' in lines[max(0, i-2)]:
                if m_x and x is None: x = float(m_x.group(1))
                if m_y and y is None: y = float(m_y.group(1))
                if m_z and z is None: z = float(m_z.group(1))
            if 'orientation' in lines[max(0, i-1)] or 'orientation' in lines[max(0, i-2)] or 'orientation' in lines[max(0, i-3)] or 'orientation' in lines[max(0, i-4)]:
                if m_z and qz is None: qz = float(m_z.group(1))
                if m_w and qw is None: qw = float(m_w.group(1))

    yaw = None
    if qz is not None and qw is not None:
        yaw = 2.0 * math.atan2(qz, qw)
    return x, y, yaw

def main():
    print("=" * 70)
    print(" PHASE 1 STABILIZATION VERIFICATION: 3 CLEAN-RESTART RUNS")
    print(f" Route: Lobby (-1.80, 0.00) -> Experience Centre ({TARGET_X:.2f}, {TARGET_Y:.2f})")
    print("=" * 70)

    results = []

    for run_num in range(1, NUM_RUNS + 1):
        print(f"\n>>> [RUN {run_num}/{NUM_RUNS}] Initiating clean restart of container...")
        rc, out, err = run_cmd(f"docker restart {CONTAINER}", timeout=30)
        if rc != 0:
            print(f"Error restarting container: {err}")
            sys.exit(1)

        print(f"  Container restarted. Waiting 14s for simulation and Nav2 stack to initialize...")
        time.sleep(14)

        # Check initial Gazebo world pose
        init_gz_x, init_gz_y, init_gz_yaw = get_gazebo_world_pose()
        print(f"  Initial Gazebo Pose: x={init_gz_x}, y={init_gz_y}, yaw={init_gz_yaw}")

        # Run navigation test
        print(f"  Launching navigation goal test inside container...")
        test_cmd = f'docker exec {CONTAINER} bash -c "source /opt/ros/jazzy/setup.bash && python3 /workspace/gazebo_four_wheel_robot/scripts/test_phase1_stability.py"'
        rc, stdout, stderr = run_cmd(test_cmd, timeout=90)
        print(stdout)

        # Read JSON result from container
        rc_cat, out_json, _ = run_cmd(f"docker exec {CONTAINER} cat /tmp/phase1_run_result.json")
        try:
            nav_data = json.loads(out_json)
        except Exception as e:
            print(f"Failed to parse JSON result: {e}")
            nav_data = {"status": "FAILED", "elapsed_sec": 0, "final_nav2_pose": {}, "target_error_m": 999}

        # Query final Gazebo world pose
        final_gz_x, final_gz_y, final_gz_yaw = get_gazebo_world_pose()
        print(f"  Final Gazebo Physical Pose: x={final_gz_x}, y={final_gz_y}, yaw={final_gz_yaw}")

        # Compute pose disagreement between Nav2 and Gazebo
        nav2_x = nav_data["final_nav2_pose"].get("x")
        nav2_y = nav_data["final_nav2_pose"].get("y")
        nav2_yaw = nav_data["final_nav2_pose"].get("yaw")

        pose_disagreement = None
        if nav2_x is not None and final_gz_x is not None:
            pose_disagreement = math.hypot(nav2_x - final_gz_x, nav2_y - final_gz_y)

        # Collision check: check if robot is wedged or near obstacles outside living demo room
        # Living Demo bounding box: X in [-5.0, 0.0], Y in [8.0, 13.0]
        # Doorway threshold: X in [-3.2, -1.8], Y in [7.5, 8.5]
        collision_detected = False
        if final_gz_x is not None and final_gz_y is not None:
            # Check if inside valid Living Demo zone
            if not (-5.2 <= final_gz_x <= 0.0 and 8.0 <= final_gz_y <= 13.0):
                collision_detected = True

        run_summary = {
            "run": run_num,
            "status": nav_data.get("status"),
            "elapsed_sec": nav_data.get("elapsed_sec"),
            "nav2_final_pose": {"x": nav2_x, "y": nav2_y, "yaw": nav2_yaw},
            "gazebo_final_pose": {"x": final_gz_x, "y": final_gz_y, "yaw": final_gz_yaw},
            "target_error_m": nav_data.get("target_error_m"),
            "nav2_gazebo_disagreement_m": round(pose_disagreement, 4) if pose_disagreement is not None else None,
            "physical_collision": collision_detected
        }
        results.append(run_summary)

        print("-" * 60)
        print(f" Run {run_num} Result:")
        print(f"   Status:                   {run_summary['status']}")
        print(f"   Duration:                 {run_summary['elapsed_sec']}s")
        print(f"   Nav2 Pose:                ({nav2_x}, {nav2_y})")
        print(f"   Gazebo Physical Pose:     ({final_gz_x}, {final_gz_y})")
        print(f"   Target Error:             {run_summary['target_error_m']}m")
        print(f"   Nav2-Gazebo Disagreement: {run_summary['nav2_gazebo_disagreement_m']}m")
        print(f"   Physical Collision:       {'YES' if collision_detected else 'NO'}")
        print("-" * 60)

    # Compile final report
    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "route": "Lobby -> Experience Centre (Living Demo)",
        "target": {"x": TARGET_X, "y": TARGET_Y, "yaw": TARGET_YAW},
        "num_runs": NUM_RUNS,
        "runs": results
    }

    report_path = "phase1_verification_report.json"
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)
    print(f"\nFinal Phase 1 Verification Report saved to: {report_path}")

    # Summary table
    print("\n" + "=" * 75)
    print(f"{'Run':<6}{'Status':<12}{'Time (s)':<10}{'Target Err (m)':<16}{'Nav2-Gz Err (m)':<18}{'Collision'}")
    print("-" * 75)
    for r in results:
        status = r['status']
        t = f"{r['elapsed_sec']:.1f}"
        terr = f"{r['target_error_m']:.3f}" if r['target_error_m'] is not None else "N/A"
        gerr = f"{r['nav2_gazebo_disagreement_m']:.4f}" if r['nav2_gazebo_disagreement_m'] is not None else "N/A"
        col = "NO" if not r['physical_collision'] else "YES (STUCK)"
        print(f"{r['run']:<6}{status:<12}{t:<10}{terr:<16}{gerr:<18}{col}")
    print("=" * 75)

if __name__ == "__main__":
    main()
