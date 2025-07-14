#!/usr/bin/env python3

import re
import argparse
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument("-f", "--file", default="node_temp.log", help="Path to log file")
parser.add_argument("-n", "--node", default="camera_b", help="Target thermal node name")
parser.add_argument("-t", "--time", type=int, default=30, help="Duration in seconds")
args = parser.parse_args()

temperatures = []

with open(args.file, "r") as f:
    found = False
    for line in f:
        if f'name: {args.node}' in line:
            found = True
        elif found and "temperature:" in line:
            match = re.search(r"temperature:\s+([\d\.]+)", line)
            if match:
                temperatures.append(float(match.group(1)))
                found = False

N = len(temperatures)
if N < 2:
    raise ValueError("Not enough data points found for interpolation.")

time_interval = args.time / (N - 1)
timestamps = [i * time_interval for i in range(N)]

# plot
plt.plot(timestamps, temperatures, marker='.')
plt.xlabel("Time [s]")
plt.ylabel("Temperature [K]")
plt.title(f"Thermal Node '{args.node}' Temperature Over Time")
plt.grid(True)
plt.tight_layout()
plt.savefig(f"{args.node}_temperature_plot.png")
plt.show()

