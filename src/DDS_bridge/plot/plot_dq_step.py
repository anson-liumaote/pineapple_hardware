import time
import os
import pandas as pd
import matplotlib.pyplot as plt

csv_file = "dq_sine3_log.csv"

# Wait for the log file to appear (optional)
wait_time = 0
while not os.path.exists(csv_file) and wait_time < 30:
    print(f"Waiting for {csv_file} to be created...")
    time.sleep(1)
    wait_time += 1

if not os.path.exists(csv_file):
    print(f"File {csv_file} not found. Exiting.")
    exit(1)

# Read the CSV log
df = pd.read_csv(csv_file)

# Plot
plt.figure(figsize=(10, 6))
plt.plot(df['time'], df['cmd_dq'], label='Command dq', linewidth=2)
plt.plot(df['time'], df['state_dq'], label='Output dq', linewidth=2)
plt.xlabel('Time (s)')
plt.ylabel('dq')
plt.title('Step Response of dq (motor 2)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("dq_sine3_plot.png")
plt.show()
