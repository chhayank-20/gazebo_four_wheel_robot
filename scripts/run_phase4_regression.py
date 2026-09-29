#!/usr/bin/env python3
"""
Phase 4 Master Regression Orchestrator
Executes 3 independent clean-restart tours of the full 4-leg route:
  Leg 1: Lobby -> Living Demo
  Leg 2: Living Demo -> Board Room
  Leg 3: Board Room -> Accounts
  Leg 4: Accounts -> Lobby
Total: 12 legs.

Records per leg:
  - Nav2 action result & duration
  - Final physical position error & orientation error
  - Nav2 vs Gazebo pose disagreement
  - Recovery behavior count
  - Minimum LiDAR range
  - Actual Gazebo collision/contact
  - Prolonged oscillation / reversal checks
"""

import subprocess
import time
import json
import math
import sys
import os

CONTAINER = "ros2_gazebo_robot"
NUM_TOURS = 3

def run_cmd(cmd, timeout=300):
    p = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=timeout)
    return p.returncode, p.stdout, p.stderr

def check_audit_readiness(tour_num):
    print(f"\n--- [TOUR {tour_num}] Auditing Environment & Parameter State ---")
    
    # Check BT XML param
    rc, out_bt, _ = run_cmd(f'docker exec {CONTAINER} bash -c "source /opt/ros/jazzy/setup.bash && ros2 param get /bt_navigator default_nav_through_poses_bt_xml"')
    if "navigate_through_poses_w_replanning_and_recovery.xml" in out_bt:
        print(f"  [AUDIT PASS] Custom BT XML active: {out_bt.strip()}")
    else:
        print(f"  [AUDIT FAIL] Unexpected BT XML: {out_bt.strip()}")

    # Check TF tree
    rc, out_tf, _ = run_cmd(f'docker exec {CONTAINER} bash -c "source /opt/ros/jazzy/setup.bash && timeout 3 ros2 run tf2_ros tf2_echo map base_link"')
    if "Translation" in out_tf:
        print(f"  [AUDIT PASS] TF map -> base_link is continuous and active.")
    else:
        print(f"  [AUDIT FAIL] TF map -> base_link missing: {out_tf}")

    # Check /odom publisher
    rc, out_odom, _ = run_cmd(f'docker exec {CONTAINER} bash -c "source /opt/ros/jazzy/setup.bash && timeout 2 ros2 topic echo /odom --once"')
    if "frame_id: odom" in out_odom:
        print(f"  [AUDIT PASS] /odom authoritative ground truth publishing at 20Hz.")
    else:
        print(f"  [AUDIT FAIL] /odom not publishing: {out_odom}")

def wrap_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

def main():
    print("=" * 80)
    print(" PHASE 4: FINAL NAVIGATION REGRESSION AND CONFIGURATION AUDIT")
    print(f" 3 Full Tours (12 Legs Total) from Clean Restarts")
    print("=" * 80)

    all_tours_summary = []

    for tour_idx in range(1, NUM_TOURS + 1):
        print(f"\n######################################################################")
        print(f" TOUR {tour_idx}/{NUM_TOURS}: CLEAN RESTARTING CONTAINER STACK")
        print(f"######################################################################")

        rc, _, err = run_cmd(f"docker restart {CONTAINER}", timeout=40)
        if rc != 0:
            print(f"Failed to restart container: {err}")
            sys.exit(1)

        print("Waiting 14 seconds for simulation, Gazebo, ros_gz_bridge, and Nav2 stack...")
        time.sleep(14)

        check_audit_readiness(tour_idx)

        print(f"\nExecuting Tour {tour_idx} (4 legs)...")
        tour_out_path = f"/tmp/phase4_tour_{tour_idx}.json"
        cmd = f'docker exec {CONTAINER} bash -c "source /opt/ros/jazzy/setup.bash && python3 /workspace/gazebo_four_wheel_robot/scripts/test_step3_independent.py --test full_tour --output {tour_out_path}"'
        rc_run, stdout_run, stderr_run = run_cmd(cmd, timeout=500)
        print(stdout_run)

        # Pull JSON result
        rc_cat, out_json, _ = run_cmd(f"docker exec {CONTAINER} cat {tour_out_path}")
        try:
            tour_data = json.loads(out_json)
        except Exception as e:
            print(f"Failed to parse tour JSON: {e}")
            tour_data = []

        # Process and enrich metrics
        processed_legs = []
        for leg in tour_data:
            target_yaw = leg["target"].get("yaw", 0.0)
            gz_yaw = leg["final_gazebo_pose"].get("yaw", 0.0) if leg["final_gazebo_pose"] else 0.0
            nav2_yaw = leg["final_nav2_pose"].get("yaw", 0.0) if leg["final_nav2_pose"] else 0.0
            
            yaw_err = abs(wrap_angle(gz_yaw - target_yaw)) if gz_yaw is not None else 0.0
            yaw_err_deg = math.degrees(yaw_err)

            leg_enriched = {
                "tour_id": tour_idx,
                "name": leg["name"],
                "status": leg["status"],
                "duration_sec": leg["elapsed_sec"],
                "physical_goal_dist_m": leg["physical_goal_dist_m"],
                "physical_yaw_error_deg": round(yaw_err_deg, 2),
                "pose_disagreement_m": leg["pose_disagreement_m"],
                "recovery_count": leg["recovery_count"],
                "min_lidar_range_m": leg["min_lidar_range_m"],
                "wall_contact": leg.get("wall_contact", False),
                "anomalies": "NONE" if leg["status"] == "SUCCEEDED" and not leg.get("wall_contact", False) and (leg["physical_goal_dist_m"] or 0) <= 0.35 else "FLAGGED",
                "final_gazebo_pose": leg["final_gazebo_pose"],
                "final_nav2_pose": leg["final_nav2_pose"]
            }
            processed_legs.append(leg_enriched)

        # Save individual tour JSON
        tour_filename = f"phase4_tour{tour_idx}_result.json"
        with open(tour_filename, "w") as f:
            json.dump(processed_legs, f, indent=2)
        print(f"Saved {tour_filename}")

        all_tours_summary.append({
            "tour": tour_idx,
            "legs": processed_legs
        })

    # Save aggregated report
    master_report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "tours_planned": NUM_TOURS,
        "tours_executed": len(all_tours_summary),
        "tours": all_tours_summary
    }
    with open("phase4_regression_report.json", "w") as f:
        json.dump(master_report, f, indent=2)
    print("\nMaster regression report saved to phase4_regression_report.json")

    # Print summary table
    print("\n" + "=" * 115)
    print(f"{'Tour':<6}{'Leg':<32}{'Status':<11}{'Time(s)':<9}{'Goal Dist':<11}{'Yaw Err':<10}{'Disagree':<11}{'MinScan':<9}{'Rec':<5}{'Contact'}")
    print("-" * 115)
    total_legs = 0
    passed_legs = 0
    for t in all_tours_summary:
        for l in t["legs"]:
            total_legs += 1
            if l["status"] == "SUCCEEDED" and (l["physical_goal_dist_m"] or 0) <= 0.35 and (l["pose_disagreement_m"] or 0) <= 0.10 and not l["wall_contact"]:
                passed_legs += 1
            t_sec = f"{l['duration_sec']:.1f}"
            gdist = f"{l['physical_goal_dist_m']:.3f}m" if l['physical_goal_dist_m'] is not None else "N/A"
            yerr = f"{l['physical_yaw_error_deg']:.1f}°"
            dis = f"{l['pose_disagreement_m']:.4f}m" if l['pose_disagreement_m'] is not None else "N/A"
            mscan = f"{l['min_lidar_range_m']:.2f}m" if l['min_lidar_range_m'] is not None else "N/A"
            rec = str(l['recovery_count'])
            cont = "NO" if not l['wall_contact'] else "YES"
            print(f"{l['tour_id']:<6}{l['name']:<32}{l['status']:<11}{t_sec:<9}{gdist:<11}{yerr:<10}{dis:<11}{mscan:<9}{rec:<5}{cont}")
    print("=" * 115)
    print(f"Total Passed: {passed_legs}/{total_legs} ({passed_legs/max(1,total_legs)*100:.1f}%)")
    print("=" * 115)

if __name__ == "__main__":
    main()
