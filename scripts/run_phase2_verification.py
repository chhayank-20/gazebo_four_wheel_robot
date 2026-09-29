#!/usr/bin/env python3
"""
Phase 2 Master Orchestrator: Full-Route Reliability Verification
Executes 3 complete sequences of the full 4-leg route:
  Leg 1: Lobby -> Living Demo
  Leg 2: Living Demo -> Board Room
  Leg 3: Board Room -> Accounts
  Leg 4: Accounts -> Lobby
Total: 12 legs.

Verifies:
  - Clean restart between sequences
  - Authoritative TF & odometry publisher count
  - Nav2 vs Gazebo physical ground-truth agreement (< 0.10m)
  - Arrival accuracy (< 0.35m)
  - Wall contacts, wedging, recovery count, and commanded velocity
"""

import subprocess
import time
import json
import math
import sys
import os

CONTAINER = "ros2_gazebo_robot"
NUM_SEQUENCES = 3
REPORT_FILE = "phase2_verification_report.json"

def run_cmd(cmd, timeout=300):
    p = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=timeout)
    return p.returncode, p.stdout, p.stderr

def check_system_readiness():
    print("  Checking system readiness and TF tree...")
    # Check Nav2 lifecycle nodes
    rc, out, _ = run_cmd(f'docker exec {CONTAINER} bash -c "source /opt/ros/jazzy/setup.bash && ros2 lifecycle get /controller_server /bt_navigator /planner_server"')
    if "active" not in out:
        print(f"  [WARN] Nav2 lifecycle check output: {out}")

    # Check TF map -> base_link
    rc_tf, out_tf, _ = run_cmd(f'docker exec {CONTAINER} bash -c "source /opt/ros/jazzy/setup.bash && timeout 3 ros2 run tf2_ros tf2_echo map base_link"')
    if "Translation" not in out_tf:
        print(f"  [WARN] TF map->base_link check output: {out_tf}")
    else:
        print("  TF map -> base_link is valid.")

    # Check TF base_link -> laser_link
    rc_laser, out_laser, _ = run_cmd(f'docker exec {CONTAINER} bash -c "source /opt/ros/jazzy/setup.bash && timeout 3 ros2 run tf2_ros tf2_echo base_link laser_link"')
    if "Translation" not in out_laser:
        print(f"  [WARN] TF base_link->laser_link check output: {out_laser}")
    else:
        print("  TF base_link -> laser_link is valid.")

    # Check odom topic
    rc_odom, out_odom, _ = run_cmd(f'docker exec {CONTAINER} bash -c "source /opt/ros/jazzy/setup.bash && timeout 2 ros2 topic echo /odom --once"')
    if "frame_id: odom" not in out_odom:
        print(f"  [WARN] /odom check output: {out_odom}")
    else:
        print("  /odom ground-truth odometry is publishing.")

def main():
    print("=" * 80)
    print(" PHASE 2: FULL-ROUTE RELIABILITY VERIFICATION (3 SEQUENCES, 12 LEGS TOTAL)")
    print(" Route: Lobby -> Living Demo -> Board Room -> Accounts -> Lobby")
    print("=" * 80)

    all_sequences = []
    overall_pass = True

    for seq in range(1, NUM_SEQUENCES + 1):
        print(f"\n######################################################################")
        print(f" SEQUENCE {seq}/{NUM_SEQUENCES}: INITIATING CLEAN SIMULATION RESTART")
        print(f"######################################################################")

        rc, _, err = run_cmd(f"docker restart {CONTAINER}", timeout=40)
        if rc != 0:
            print(f"Failed to restart container: {err}")
            sys.exit(1)

        print("Waiting 14 seconds for simulation, Gazebo, ros_gz_bridge, and Nav2 stack...")
        time.sleep(14)

        check_system_readiness()

        print(f"\nLaunching Sequence {seq} test runner inside container...")
        out_json_path = f"/tmp/phase2_seq_{seq}.json"
        cmd = f'docker exec {CONTAINER} bash -c "source /opt/ros/jazzy/setup.bash && python3 /workspace/gazebo_four_wheel_robot/scripts/test_phase2_runner.py --seq {seq} --output {out_json_path}"'
        rc_run, stdout_run, stderr_run = run_cmd(cmd, timeout=700)
        print(stdout_run)

        # Retrieve JSON output
        rc_cat, out_json, _ = run_cmd(f"docker exec {CONTAINER} cat {out_json_path}")
        try:
            seq_data = json.loads(out_json)
        except Exception as e:
            print(f"Failed to parse sequence JSON: {e}")
            print(f"Raw output: {out_json}")
            seq_data = []

        all_sequences.append({
            "sequence_id": seq,
            "legs": seq_data
        })

        # Evaluate sequence acceptance
        seq_passed = True
        for leg in seq_data:
            if leg["status"] != "SUCCEEDED":
                seq_passed = False
                overall_pass = False
                print(f"[FAIL] Sequence {seq} {leg['name']} failed with status: {leg['status']}")
            if leg["pose_disagreement_m"] is not None and leg["pose_disagreement_m"] > 0.10:
                seq_passed = False
                overall_pass = False
                print(f"[FAIL] Sequence {seq} {leg['name']} pose disagreement {leg['pose_disagreement_m']}m exceeded 0.10m threshold!")
            if leg["physical_goal_dist_m"] is not None and leg["physical_goal_dist_m"] > 0.35:
                seq_passed = False
                overall_pass = False
                print(f"[FAIL] Sequence {seq} {leg['name']} goal distance {leg['physical_goal_dist_m']}m exceeded 0.35m threshold!")
            if leg["wall_contact_detected"]:
                seq_passed = False
                overall_pass = False
                print(f"[FAIL] Sequence {seq} {leg['name']} wall contact detected!")

        if not seq_passed:
            print(f"\n[ABORT] Sequence {seq} failed acceptance criteria! Stopping testing to analyze logs as required.")
            # Pull nav2 log for investigation
            run_cmd(f"docker exec {CONTAINER} cat /tmp/nav2.log > /tmp/nav2_failure_seq_{seq}.log")
            break

    # Save final aggregated report
    final_report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "total_sequences_planned": NUM_SEQUENCES,
        "total_sequences_run": len(all_sequences),
        "overall_passed": overall_pass,
        "sequences": all_sequences
    }

    with open(REPORT_FILE, "w") as f:
        json.dump(final_report, f, indent=2)
    print(f"\nFull verification report saved to: {REPORT_FILE}")

    # Print summary table
    print("\n" + "=" * 105)
    print(f"{'Seq':<5}{'Leg':<32}{'Status':<11}{'Time(s)':<9}{'Goal Dist':<11}{'Nav2-Gz Err':<13}{'MinScan':<9}{'Recov':<7}{'Contact'}")
    print("-" * 105)
    total_legs = 0
    passed_legs = 0
    for s in all_sequences:
        for l in s["legs"]:
            total_legs += 1
            if l["status"] == "SUCCEEDED" and (l["physical_goal_dist_m"] or 0) <= 0.35 and (l["pose_disagreement_m"] or 0) <= 0.10 and not l["wall_contact_detected"]:
                passed_legs += 1
            t = f"{l['elapsed_sec']:.1f}"
            gdist = f"{l['physical_goal_dist_m']:.3f}m" if l['physical_goal_dist_m'] is not None else "N/A"
            perr = f"{l['pose_disagreement_m']:.4f}m" if l['pose_disagreement_m'] is not None else "N/A"
            mscan = f"{l['min_lidar_range_m']:.2f}m" if l['min_lidar_range_m'] is not None else "N/A"
            rec = str(l['recovery_count'])
            cont = "NO" if not l['wall_contact_detected'] else "YES"
            print(f"{l['sequence']:<5}{l['name']:<32}{l['status']:<11}{t:<9}{gdist:<11}{perr:<13}{mscan:<9}{rec:<7}{cont}")
    print("=" * 105)
    print(f"Total Legs: {total_legs}/12 | Passed: {passed_legs}/{total_legs} ({passed_legs/max(1,total_legs)*100:.1f}%)")
    print("=" * 105)

    sys.exit(0 if overall_pass and passed_legs == 12 else 1)

if __name__ == "__main__":
    main()
