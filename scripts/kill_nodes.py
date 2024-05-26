#!/usr/bin/env python3

import psutil

def kill_ros_nodes():
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            if '__node' in proc.info['cmdline']:
                # print(proc.)
                proc.kill()
        except (psutil.AccessDenied, psutil.NoSuchProcess):
            pass

if __name__ == "__main__":
    kill_ros_nodes()
    exit()