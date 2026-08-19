#!/usr/bin/env python3
import psutil
import os

print(f"{'PID':<8} | {'CPU %':<7} | {'RAM (MB)':<10} | {'PROCESS NAME':<25} | {'COMMAND'}")
print("-" * 90)

proc_list = []
# Take CPU sample over 0.5 sec
for p in psutil.process_iter(['pid', 'name', 'cmdline']):
    try:
        p.cpu_percent(None)
    except:
        pass

import time
time.sleep(0.5)

for p in psutil.process_iter(['pid', 'name', 'cmdline', 'memory_info']):
    try:
        cpu = p.cpu_percent(None)
        mem_mb = p.info['memory_info'].rss / (1024 * 1024) if p.info['memory_info'] else 0
        cmd = ' '.join(p.info['cmdline']) if p.info['cmdline'] else p.info['name']
        
        keywords = ['ros', 'gz', 'ign', 'rviz', 'python', 'yolo', 'nav2', 'controller', 'planner', 'slam', 'rtabmap', 'twist_mux', 'smoother', 'behavior']
        if any(k in cmd.lower() for k in keywords):
            proc_list.append({
                'pid': p.info['pid'],
                'cpu': cpu,
                'mem': mem_mb,
                'name': p.info['name'],
                'cmd': cmd[:75]
            })
    except:
        pass

# Sort by CPU usage
proc_list.sort(key=lambda x: x['cpu'], reverse=True)

print("=== TOP CPU CONSUMING PROCESSES ===")
for p in proc_list[:15]:
    print(f"{p['pid']:<8} | {p['cpu']:<7.1f} | {p['mem']:<10.1f} | {p['name'][:24]:<25} | {p['cmd']}")

print("\n=== TOP RAM (MEMORY) CONSUMING PROCESSES ===")
proc_list.sort(key=lambda x: x['mem'], reverse=True)
for p in proc_list[:15]:
    print(f"{p['pid']:<8} | {p['cpu']:<7.1f} | {p['mem']:<10.1f} | {p['name'][:24]:<25} | {p['cmd']}")
