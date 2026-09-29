#!/usr/bin/env python3
"""
Measurement script for UI responsiveness, API latencies, Gazebo RTF, and container resources.
"""
import time
import json
import urllib.request
import subprocess
import re

API_BASE = "http://localhost:8000"

def measure_api_latencies(n=20):
    health_times = []
    status_times = []

    for _ in range(n):
        t0 = time.perf_counter()
        req = urllib.request.Request(f"{API_BASE}/health")
        with urllib.request.urlopen(req, timeout=5) as resp:
            resp.read()
        health_times.append((time.perf_counter() - t0) * 1000)

        t0 = time.perf_counter()
        req = urllib.request.Request(f"{API_BASE}/status")
        with urllib.request.urlopen(req, timeout=5) as resp:
            resp.read()
        status_times.append((time.perf_counter() - t0) * 1000)

        time.sleep(0.05)

    return {
        "health_avg_ms": round(sum(health_times) / len(health_times), 2),
        "health_min_ms": round(min(health_times), 2),
        "health_max_ms": round(max(health_times), 2),
        "status_avg_ms": round(sum(status_times) / len(status_times), 2),
        "status_min_ms": round(min(status_times), 2),
        "status_max_ms": round(max(status_times), 2),
    }

def measure_telemetry_freshness():
    req = urllib.request.Request(f"{API_BASE}/status")
    with urllib.request.urlopen(req, timeout=5) as resp:
        data = json.loads(resp.read().decode())
    
    wall_now = time.time()
    api_ts = data.get("timestamp", wall_now)
    api_age_ms = round((wall_now - api_ts) * 1000, 2)
    
    # Check internal sensor timestamps
    pose = data.get("pose") or {}
    vel = data.get("velocity") or {}
    lidar = data.get("lidar") or {}
    
    return {
        "api_response_age_ms": api_age_ms,
        "pose_timestamp": pose.get("timestamp"),
        "velocity_timestamp": vel.get("timestamp"),
        "lidar_timestamp": lidar.get("timestamp"),
        "all_feeds_present": all(x is not None for x in [pose.get("x"), vel.get("linear_x"), lidar.get("min_lidar_m")])
    }

def measure_container_stats():
    res = subprocess.run(
        ["docker", "stats", "ros2_gazebo_robot", "--no-stream", "--format", "{{.CPUPerc}}\t{{.MemUsage}}\t{{.MemPerc}}"],
        capture_output=True, text=True
    )
    if res.returncode == 0:
        parts = res.stdout.strip().split("\t")
        return {
            "cpu_percent": parts[0].strip(),
            "mem_usage": parts[1].strip(),
            "mem_percent": parts[2].strip() if len(parts) > 2 else "N/A"
        }
    return {}

def measure_top_processes():
    res = subprocess.run(
        ["docker", "exec", "ros2_gazebo_robot", "ps", "aux", "--sort=-%cpu"],
        capture_output=True, text=True
    )
    lines = res.stdout.strip().split("\n")[1:8]
    procs = []
    for l in lines:
        parts = l.split(None, 10)
        if len(parts) >= 11:
            procs.append({
                "pid": parts[1],
                "cpu": parts[2] + "%",
                "mem": parts[3] + "%",
                "command": parts[10][:60]
            })
    return procs

def measure_gazebo_rtf():
    cmd = ["docker", "exec", "ros2_gazebo_robot", "bash", "-c",
           "source /opt/ros/jazzy/setup.bash && gz topic -e -t /world/robot_world/stats -n 1"]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
    out = res.stdout
    rtf_m = re.search(r"real_time_factor:\s*([0-9.]+)", out)
    rtf = float(rtf_m.group(1)) if rtf_m else None
    return {"rtf": rtf}

if __name__ == "__main__":
    print("Measuring API latencies...")
    latencies = measure_api_latencies()
    print("Latencies:", latencies)
    
    print("Measuring telemetry freshness...")
    freshness = measure_telemetry_freshness()
    print("Freshness:", freshness)
    
    print("Measuring container stats...")
    c_stats = measure_container_stats()
    print("Container Stats:", c_stats)
    
    print("Measuring Gazebo RTF...")
    try:
        gz_stats = measure_gazebo_rtf()
    except Exception as e:
        gz_stats = {"error": str(e)}
    print("Gazebo RTF:", gz_stats)
    
    print("Top processes:")
    procs = measure_top_processes()
    for p in procs:
        print(f"  {p['cpu']:>6} {p['mem']:>6} {p['command']}")
    
    out = {
        "latencies": latencies,
        "freshness": freshness,
        "container_stats": c_stats,
        "gazebo_stats": gz_stats,
        "top_processes": procs
    }
    with open("ui_and_system_metrics.json", "w") as f:
        json.dump(out, f, indent=2)
    print("\nSaved to ui_and_system_metrics.json")
