#!/usr/bin/env python3
"""
Performance Diagnostic and Benchmarking Script
Measures:
1. Gazebo Real-Time Factor (RTF) and physics step stats
2. Container and Process CPU & Memory usage breakdown
3. Robot Command-to-Motion latency
4. Motion vs Stationary resource consumption
"""

import sys
import os
import time
import json
import re
import subprocess
import urllib.request
import urllib.error

API_BASE = "http://localhost:8000"

def get_container_stats():
    """Gets container CPU% and memory via docker stats."""
    res = subprocess.run(
        ["docker", "stats", "ros2_gazebo_robot", "--no-stream", "--format", "{{.CPUPerc}}\t{{.MemUsage}}"],
        capture_output=True, text=True
    )
    if res.returncode == 0 and res.stdout.strip():
        parts = res.stdout.strip().split("\t")
        return {
            "container_cpu_percent": parts[0].strip(),
            "container_memory": parts[1].strip() if len(parts) > 1 else "N/A"
        }
    return {"container_cpu_percent": "N/A", "container_memory": "N/A"}

def get_top_processes(n=5):
    """Gets top N CPU consuming processes inside the container."""
    cmd = ["docker", "exec", "ros2_gazebo_robot", "ps", "aux", "--sort=-%cpu"]
    res = subprocess.run(cmd, capture_output=True, text=True)
    procs = []
    if res.returncode == 0:
        lines = res.stdout.strip().split("\n")[1:n+1]
        for l in lines:
            parts = l.split(None, 10)
            if len(parts) >= 11:
                procs.append({
                    "pid": parts[1],
                    "cpu_percent": parts[2],
                    "mem_percent": parts[3],
                    "command": parts[10][:50]
                })
    return procs

def get_gazebo_stats(duration_sec=3.0):
    """Calculates Gazebo RTF over a duration."""
    def sample():
        cmd = ["docker", "exec", "ros2_gazebo_robot", "bash", "-c",
               "source /opt/ros/jazzy/setup.bash && gz topic -e -t /world/robot_world/stats -n 1"]
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=6)
        out = res.stdout
        sim_sec = int(re.search(r"sim_time\s*\{\s*sec:\s*(\d+)", out).group(1))
        sim_nsec = int(re.search(r"sim_time\s*\{[^}]*nsec:\s*(\d+)", out).group(1))
        real_sec = int(re.search(r"real_time\s*\{\s*sec:\s*(\d+)", out).group(1))
        real_nsec = int(re.search(r"real_time\s*\{[^}]*nsec:\s*(\d+)", out).group(1))
        rtf_m = re.search(r"real_time_factor:\s*([0-9.]+)", out)
        rtf = float(rtf_m.group(1)) if rtf_m else 0.0
        return (sim_sec + sim_nsec * 1e-9), (real_sec + real_nsec * 1e-9), rtf

    try:
        s1, r1, rtf_inst1 = sample()
        time.sleep(duration_sec)
        s2, r2, rtf_inst2 = sample()
        avg_rtf = (s2 - s1) / (r2 - r1) if (r2 - r1) > 0 else 0.0
        return {
            "instantaneous_rtf": round(rtf_inst2, 3),
            "measured_avg_rtf": round(avg_rtf, 3),
            "sim_time_delta_s": round(s2 - s1, 3),
            "real_time_delta_s": round(r2 - r1, 3)
        }
    except Exception as e:
        return {"error": str(e), "measured_avg_rtf": 0.0}

def measure_command_latency():
    """Measures time from POST /navigate to first observed robot velocity/motion."""
    payload = json.dumps({"location_id": "living_demo"}).encode("utf-8")
    req = urllib.request.Request(
        f"{API_BASE}/navigate",
        data=payload,
        headers={"Content-Type": "application/json"}
    )
    t0 = time.time()
    cmd_id = None
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            data = json.loads(resp.read().decode())
            cmd_id = data.get("command_id")
    except Exception as e:
        return {"error": f"Failed to send command: {e}"}

    t_motion = None
    start_poll = time.time()
    while time.time() - start_poll < 6.0:
        try:
            with urllib.request.urlopen(f"{API_BASE}/status", timeout=2) as r:
                st = json.loads(r.read().decode())
                vel = st.get("velocity") or {}
                vx = abs(vel.get("linear_x", 0.0))
                wz = abs(vel.get("angular_z", 0.0))
                if vx > 0.03 or wz > 0.03:
                    t_motion = time.time()
                    break
        except Exception:
            pass
        time.sleep(0.04)

    latency_ms = round((t_motion - t0) * 1000, 1) if t_motion else None

    if cmd_id:
        try:
            cancel_req = urllib.request.Request(
                f"{API_BASE}/commands/{cmd_id}/cancel",
                data=b"{}",
                headers={"Content-Type": "application/json"}
            )
            urllib.request.urlopen(cancel_req, timeout=5)
        except Exception:
            pass

    return {
        "command_id": cmd_id,
        "motion_detected": t_motion is not None,
        "command_to_motion_latency_ms": latency_ms
    }

def main():
    print("=" * 60)
    print("STEP 2: BASELINE PERFORMANCE MEASUREMENTS")
    print("=" * 60)

    print("\n[1/3] Measuring Stationary Performance (Robot at Rest)...")
    c_stats_stat = get_container_stats()
    top_stat = get_top_processes(6)
    gz_stat = get_gazebo_stats(duration_sec=3.0)

    print(f" -> Container CPU: {c_stats_stat['container_cpu_percent']}, Memory: {c_stats_stat['container_memory']}")
    print(f" -> Gazebo RTF (Average over 3s): {gz_stat.get('measured_avg_rtf', 'N/A')}")
    print(" -> Top CPU Consumers:")
    for p in top_stat:
        print(f"    - PID {p['pid']}: {p['cpu_percent']}% CPU ({p['command']})")

    print("\n[2/3] Measuring Command-to-Motion Latency...")
    lat_result = measure_command_latency()
    print(f" -> Motion Latency: {lat_result.get('command_to_motion_latency_ms')} ms (Detected: {lat_result.get('motion_detected')})")

    print("\n[3/3] Measuring Moving Performance...")
    payload = json.dumps({"location_id": "hotel_exp"}).encode("utf-8")
    req = urllib.request.Request(f"{API_BASE}/navigate", data=payload, headers={"Content-Type": "application/json"})
    cmd_id = None
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            cmd_id = json.loads(resp.read().decode()).get("command_id")
    except Exception as e:
        print(f"  Failed moving dispatch: {e}")

    time.sleep(1.0)
    c_stats_mov = get_container_stats()
    top_mov = get_top_processes(6)
    gz_mov = get_gazebo_stats(duration_sec=3.0)

    if cmd_id:
        try:
            cancel_req = urllib.request.Request(f"{API_BASE}/commands/{cmd_id}/cancel", data=b"{}", headers={"Content-Type": "application/json"})
            urllib.request.urlopen(cancel_req, timeout=5)
        except Exception:
            pass

    print(f" -> Moving Container CPU: {c_stats_mov['container_cpu_percent']}, Memory: {c_stats_mov['container_memory']}")
    print(f" -> Moving Gazebo RTF: {gz_mov.get('measured_avg_rtf', 'N/A')}")
    print(" -> Moving Top CPU Consumers:")
    for p in top_mov:
        print(f"    - PID {p['pid']}: {p['cpu_percent']}% CPU ({p['command']})")

    baseline = {
        "stationary": {
            "container": c_stats_stat,
            "gazebo_rtf": gz_stat,
            "top_processes": top_stat
        },
        "command_latency": lat_result,
        "moving": {
            "container": c_stats_mov,
            "gazebo_rtf": gz_mov,
            "top_processes": top_mov
        }
    }

    with open("/tmp/baseline_perf.json", "w") as f:
        json.dump(baseline, f, indent=2)
    print("\nSaved baseline measurements to /tmp/baseline_perf.json")

if __name__ == "__main__":
    main()
